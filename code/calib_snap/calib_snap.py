#!/usr/bin/env python3
"""
Capture paired image frames and lidar point clouds for calibration.

The node supports three modes:
1. Pull frames from an RTSP stream with OpenCV, republish them as ROS image
   messages, and pair them with a PointCloud2 topic.
2. Pull frames from a local camera with OpenCV, republish them as ROS image
   messages, and pair them with a PointCloud2 topic.
3. Subscribe to both an image topic and a PointCloud2 topic, then save the
   latest pair when the user presses a key.
"""

import argparse
import select
import sys
import threading
import time
import termios
import tty
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField

try:
    from cv_bridge import CvBridge  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    CvBridge = None


PCD_DATATYPE_MAP = {
    PointField.INT8: (1, "I"),
    PointField.UINT8: (1, "U"),
    PointField.INT16: (2, "I"),
    PointField.UINT16: (2, "U"),
    PointField.INT32: (4, "I"),
    PointField.UINT32: (4, "U"),
    PointField.FLOAT32: (4, "F"),
    PointField.FLOAT64: (8, "F"),
}


def parse_args() -> argparse.Namespace:
    examples = """Examples:
  RTSP + ROS point cloud:
    python calib_snap.py --input-mode rtsp --rtsp-uri rtsp://127.0.0.1:8554/test --pointcloud-topic /livox/lidar

  Camera + ROS point cloud:
    python calib_snap.py --input-mode camera --camera-source 0 --pointcloud-topic /livox/lidar

  Dual ROS topics:
    python calib_snap.py --input-mode dual_ros --source-image-topic /g1/camera/0/color/image_raw --pointcloud-topic /livox/lidar
"""
    parser = argparse.ArgumentParser(
        description=(
            "Capture paired image frames and PointCloud2 samples from either "
            "RTSP/camera + ROS topics or dual ROS topics."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=examples,
    )
    parser.add_argument(
        "--input-mode",
        choices=["rtsp", "camera", "dual_ros"],
        default="rtsp",
        help=(
            "Input mode. 'rtsp' reads images from RTSP and point clouds from a "
            "ROS topic. 'camera' reads images from an OpenCV camera source and "
            "point clouds from a ROS topic. 'dual_ros' reads both images and "
            "point clouds from ROS topics."
        ),
    )
    parser.add_argument(
        "--rtsp-uri",
        help="RTSP input URI used in rtsp mode, for example rtsp://127.0.0.1:8554/test",
    )
    parser.add_argument(
        "--camera-source",
        default="0",
        help=(
            "OpenCV camera source used in camera mode. Numeric strings are "
            "treated as camera indices, for example 0 or 1."
        ),
    )
    parser.add_argument(
        "--source-image-topic",
        help="ROS image topic used in dual_ros mode.",
    )
    parser.add_argument(
        "--pointcloud-topic",
        required=True,
        help="PointCloud2 topic to capture.",
    )
    parser.add_argument(
        "--image-topic",
        "--publish-image-topic",
        default="/calib/image_raw",
        help="Published raw image topic in stream modes (rtsp/camera).",
    )
    parser.add_argument(
        "--compressed-topic",
        "--publish-compressed-topic",
        default="/calib/image_raw/compressed",
        help="Published compressed image topic in stream modes (rtsp/camera).",
    )
    parser.add_argument(
        "--frame-id",
        default="camera",
        help="Frame ID used for published images in stream modes.",
    )
    parser.add_argument(
        "--output-dir",
        default="data",
        help="Output root directory. Defaults to ./data.",
    )
    parser.add_argument(
        "--image-ext",
        default="png",
        choices=["png", "jpg", "jpeg"],
        help="Image file extension. Defaults to png.",
    )
    parser.add_argument(
        "--pcd-fields",
        default="x,y,z,intensity",
        help=(
            "Comma-separated PointCloud2 fields saved into PCD. Fields that do "
            "not exist in the incoming cloud are ignored."
        ),
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=15.0,
        help="Target read/publish FPS in stream modes.",
    )
    parser.add_argument(
        "--max-pointcloud-age",
        type=float,
        default=0.5,
        help=(
            "Reject captures when the latest point cloud is older than this "
            "many seconds. Set to a negative value to disable."
        ),
    )
    parser.add_argument(
        "--save-key",
        default="s",
        help="Keyboard key used to save a sample.",
    )
    parser.add_argument(
        "--quit-key",
        default="q",
        help="Keyboard key used to quit.",
    )
    args = parser.parse_args()
    if args.input_mode == "rtsp" and not args.rtsp_uri:
        parser.error("--rtsp-uri is required when --input-mode=rtsp.")
    if args.input_mode == "camera" and not str(args.camera_source).strip():
        parser.error("--camera-source must not be empty when --input-mode=camera.")
    if args.input_mode == "dual_ros" and not args.source_image_topic:
        parser.error("--source-image-topic is required when --input-mode=dual_ros.")
    return args


def normalize_extension(ext: str) -> str:
    ext = ext.lower().lstrip(".")
    if ext == "jpeg":
        return "jpg"
    return ext


def sanitize_fields(raw_fields: str) -> List[str]:
    fields = [item.strip() for item in raw_fields.split(",") if item.strip()]
    if not fields:
        raise ValueError("At least one point cloud field must be provided.")
    return fields


def resolve_opencv_source(raw_source: str):
    stripped = str(raw_source).strip()
    if stripped.isdigit():
        return int(stripped)
    return stripped


def pointfield_map(msg: PointCloud2) -> dict:
    return {field.name: field for field in msg.fields}


def resolve_capture_fields(msg: PointCloud2, requested: Sequence[str]) -> List[PointField]:
    field_lookup = pointfield_map(msg)
    resolved: List[PointField] = []
    for name in requested:
        field = field_lookup.get(name)
        if field is None:
            rospy.logwarn_throttle(
                5.0, "Point cloud field '%s' not present; skipping it.", name
            )
            continue
        if field.datatype not in PCD_DATATYPE_MAP:
            rospy.logwarn_throttle(
                5.0,
                "Point cloud field '%s' uses unsupported datatype %s; skipping it.",
                name,
                field.datatype,
            )
            continue
        resolved.append(field)
    return resolved


def format_pcd_value(value) -> str:
    if isinstance(value, bytes):
        return str(int.from_bytes(value, byteorder="little", signed=False))
    return repr(value)


def image_msg_to_bgr(msg: Image):
    encoding = msg.encoding.lower()
    height = int(msg.height)
    width = int(msg.width)
    if height <= 0 or width <= 0:
        raise ValueError("Image message has invalid shape.")

    channels_by_encoding = {
        "mono8": 1,
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
    }
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    expected_step = width * channels
    if msg.step < expected_step:
        raise ValueError(
            f"Image step {msg.step} is smaller than expected {expected_step}."
        )

    expected_size = height * msg.step
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if data.size < expected_size:
        raise ValueError(
            f"Image data size {data.size} is smaller than expected {expected_size}."
        )

    frame = data[:expected_size].reshape((height, msg.step))[:, :expected_step]
    if channels == 1:
        mono = frame.reshape((height, width))
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    frame = frame.reshape((height, width, channels))
    if encoding == "bgr8":
        return frame.copy()
    if encoding == "rgb8":
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def save_pointcloud_as_pcd(
    msg: PointCloud2, output_path: Path, requested_fields: Sequence[str]
) -> Tuple[int, List[str]]:
    resolved_fields = resolve_capture_fields(msg, requested_fields)
    names = [field.name for field in resolved_fields]
    if not {"x", "y", "z"}.issubset(set(names)):
        raise ValueError(
            "Point cloud must include x, y, z fields for calibration capture."
        )

    sizes = []
    types = []
    counts = []
    for field in resolved_fields:
        size, pcd_type = PCD_DATATYPE_MAP[field.datatype]
        sizes.append(str(size))
        types.append(pcd_type)
        counts.append("1")

    points = list(
        point_cloud2.read_points(
            msg,
            field_names=names,
            skip_nans=False,
        )
    )
    width = msg.width if msg.width > 0 else len(points)
    height = msg.height if msg.height > 0 else 1

    header = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        f"FIELDS {' '.join(names)}",
        f"SIZE {' '.join(sizes)}",
        f"TYPE {' '.join(types)}",
        f"COUNT {' '.join(counts)}",
        f"WIDTH {width}",
        f"HEIGHT {height}",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {len(points)}",
        "DATA ascii",
    ]

    with output_path.open("w", encoding="ascii") as stream:
        stream.write("\n".join(header))
        stream.write("\n")
        for point in points:
            stream.write(" ".join(format_pcd_value(v) for v in point))
            stream.write("\n")

    return len(points), names


class TerminalKeyReader:
    def __init__(self) -> None:
        self._fd = None
        self._old_settings = None

    def __enter__(self):
        if not sys.stdin.isatty():
            rospy.logwarn("STDIN is not a TTY; terminal hotkeys are disabled.")
            return self
        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._fd is not None and self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def poll(self) -> str:
        if self._fd is None:
            return ""
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return ""
        return sys.stdin.read(1)


class CalibrationCaptureNode:
    def __init__(
        self,
        input_mode: str,
        pointcloud_topic: str,
        source_image_topic: Optional[str],
        image_topic: str,
        compressed_topic: str,
        frame_id: str,
        output_dir: Path,
        image_ext: str,
        requested_pcd_fields: Sequence[str],
        max_pointcloud_age: float,
    ) -> None:
        self._input_mode = input_mode
        self._frame_id = frame_id
        self._source_image_topic = source_image_topic or ""
        self._image_ext = image_ext
        self._requested_pcd_fields = requested_pcd_fields
        self._max_pointcloud_age = max_pointcloud_age
        self._bridge = CvBridge() if CvBridge is not None else None
        self._lock = threading.Lock()
        self._latest_pointcloud = None
        self._latest_frame = None
        self._latest_image_stamp = rospy.Time()
        self._capture_index = 0

        self._image_dir = output_dir / "images"
        self._pointcloud_dir = output_dir / "pointclouds"
        self._image_dir.mkdir(parents=True, exist_ok=True)
        self._pointcloud_dir.mkdir(parents=True, exist_ok=True)

        self._pointcloud_sub = rospy.Subscriber(
            pointcloud_topic, PointCloud2, self._on_pointcloud, queue_size=5
        )
        self._image_sub = None
        self._image_pub = None
        self._compressed_pub = None

        if self._input_mode == "dual_ros":
            self._image_sub = rospy.Subscriber(
                self._source_image_topic,
                Image,
                self._on_image,
                queue_size=2,
                buff_size=1 << 24,
            )
        else:
            self._image_pub = rospy.Publisher(image_topic, Image, queue_size=10)
            self._compressed_pub = rospy.Publisher(
                compressed_topic, CompressedImage, queue_size=10
            )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        with self._lock:
            self._latest_pointcloud = msg

    def _on_image(self, msg: Image) -> None:
        try:
            if self._bridge is not None:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                frame = image_msg_to_bgr(msg)
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0,
                "Failed to decode image from topic %s: %s",
                self._source_image_topic,
                exc,
            )
            return

        with self._lock:
            self._latest_frame = frame
            self._latest_image_stamp = msg.header.stamp

    def publish_image(self, frame) -> None:
        stamp = rospy.Time.now()
        with self._lock:
            self._latest_frame = frame
            self._latest_image_stamp = stamp

        if self._image_pub is None:
            return

        if self._bridge is not None:
            msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
        else:
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = "bgr8"
            msg.step = msg.width * 3
            msg.data = frame.tobytes()
        self._image_pub.publish(msg)

        ok, encoded = cv2.imencode(".jpg", frame)
        if not ok:
            rospy.logwarn("Failed to encode frame for compressed topic.")
            return
        compressed = CompressedImage()
        compressed.header.stamp = msg.header.stamp
        compressed.header.frame_id = msg.header.frame_id
        compressed.format = "jpeg"
        compressed.data = encoded.tobytes()
        if self._compressed_pub is not None:
            self._compressed_pub.publish(compressed)

    def image_status(self) -> str:
        with self._lock:
            frame = self._latest_frame
            stamp = self._latest_image_stamp

        if frame is None:
            return "image: waiting"
        if stamp == rospy.Time():
            return "image: ready (no stamp)"
        age = (rospy.Time.now() - stamp).to_sec()
        return f"image age: {age:.3f}s"

    def pointcloud_status(self) -> str:
        with self._lock:
            msg = self._latest_pointcloud
        if msg is None:
            return "pointcloud: waiting"
        stamp = msg.header.stamp
        if stamp == rospy.Time():
            return "pointcloud: ready (no stamp)"
        age = (rospy.Time.now() - stamp).to_sec()
        return f"pointcloud age: {age:.3f}s"

    def capture_status(self) -> str:
        return f"{self.image_status()}, {self.pointcloud_status()}"

    def save_capture(self) -> None:
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            pointcloud_msg = self._latest_pointcloud

        if frame is None:
            rospy.logwarn("No image frame received yet; capture skipped.")
            return
        if pointcloud_msg is None:
            rospy.logwarn("No point cloud received yet; capture skipped.")
            return

        stamp = pointcloud_msg.header.stamp
        if self._max_pointcloud_age >= 0.0 and stamp != rospy.Time():
            age = (rospy.Time.now() - stamp).to_sec()
            if age > self._max_pointcloud_age:
                rospy.logwarn(
                    "Latest point cloud is too old (%.3fs > %.3fs); capture skipped.",
                    age,
                    self._max_pointcloud_age,
                )
                return

        self._capture_index += 1
        wall_time = time.strftime("%Y%m%d_%H%M%S")
        millis = int((time.time() % 1.0) * 1000.0)
        stem = f"capture_{self._capture_index:06d}_{wall_time}_{millis:03d}"

        image_path = self._image_dir / f"{stem}.{self._image_ext}"
        pointcloud_path = self._pointcloud_dir / f"{stem}.pcd"

        if not cv2.imwrite(str(image_path), frame):
            rospy.logerr("Failed to save image to %s", image_path)
            return

        try:
            point_count, field_names = save_pointcloud_as_pcd(
                pointcloud_msg, pointcloud_path, self._requested_pcd_fields
            )
        except Exception as exc:
            image_path.unlink(missing_ok=True)
            rospy.logerr("Failed to save point cloud: %s", exc)
            return

        rospy.loginfo(
            "Saved capture %06d: image=%s, pointcloud=%s, points=%d, fields=%s",
            self._capture_index,
            image_path,
            pointcloud_path,
            point_count,
            ",".join(field_names),
        )


def run_stream_capture(
    node: CalibrationCaptureNode,
    cap: cv2.VideoCapture,
    source_label: str,
    rate: rospy.Rate,
    save_key: str,
    quit_key: str,
) -> None:
    try:
        with TerminalKeyReader() as key_reader:
            last_status_log = 0.0
            while not rospy.is_shutdown():
                ret, frame = cap.read()
                if not ret:
                    rospy.logwarn_throttle(
                        5.0, "Failed to read frame from %s.", source_label
                    )
                    time.sleep(0.1)
                    continue

                node.publish_image(frame)

                now = time.time()
                if now - last_status_log >= 2.0:
                    rospy.loginfo_throttle(2.0, "Streaming. %s", node.capture_status())
                    last_status_log = now

                key = key_reader.poll()
                if key in (save_key, save_key.upper()):
                    node.save_capture()
                elif key in (quit_key, quit_key.upper(), "\x1b"):
                    break

                rate.sleep()
    finally:
        cap.release()


def main() -> None:
    args = parse_args()
    image_ext = normalize_extension(args.image_ext)
    requested_fields = sanitize_fields(args.pcd_fields)

    rospy.init_node("calib_snap")
    output_dir = Path(args.output_dir).expanduser().resolve()
    node = CalibrationCaptureNode(
        input_mode=args.input_mode,
        pointcloud_topic=args.pointcloud_topic,
        source_image_topic=args.source_image_topic,
        image_topic=args.image_topic,
        compressed_topic=args.compressed_topic,
        frame_id=args.frame_id,
        output_dir=output_dir,
        image_ext=image_ext,
        requested_pcd_fields=requested_fields,
        max_pointcloud_age=args.max_pointcloud_age,
    )

    rate = rospy.Rate(max(args.fps, 1.0))
    save_key = args.save_key[:1]
    quit_key = args.quit_key[:1]
    rospy.loginfo(
        "Terminal hotkeys enabled: [%s] save current image + latest pointcloud, [%s] quit",
        save_key,
        quit_key,
    )

    if args.input_mode in {"rtsp", "camera"}:
        source_label = ""
        capture_source = None
        open_failure_target = ""
        if args.input_mode == "rtsp":
            source_label = f"RTSP stream {args.rtsp_uri}"
            capture_source = args.rtsp_uri
            open_failure_target = args.rtsp_uri
            rospy.loginfo(
                "Capture mode=rtsp, rtsp_uri=%s, pointcloud_topic=%s, image_topic=%s, compressed_topic=%s",
                args.rtsp_uri,
                args.pointcloud_topic,
                args.image_topic,
                args.compressed_topic,
            )
        else:
            capture_source = resolve_opencv_source(args.camera_source)
            source_label = f"camera source {args.camera_source}"
            open_failure_target = str(args.camera_source)
            rospy.loginfo(
                "Capture mode=camera, camera_source=%s, pointcloud_topic=%s, image_topic=%s, compressed_topic=%s",
                args.camera_source,
                args.pointcloud_topic,
                args.image_topic,
                args.compressed_topic,
            )

        cap = cv2.VideoCapture(capture_source)
        if not cap.isOpened():
            rospy.logerr(
                "Failed to open %s: %s",
                "RTSP stream" if args.input_mode == "rtsp" else "camera source",
                open_failure_target,
            )
            return

        run_stream_capture(
            node=node,
            cap=cap,
            source_label=source_label,
            rate=rate,
            save_key=save_key,
            quit_key=quit_key,
        )
        return

    rospy.loginfo(
        "Capture mode=dual_ros, source_image_topic=%s, pointcloud_topic=%s",
        args.source_image_topic,
        args.pointcloud_topic,
    )
    with TerminalKeyReader() as key_reader:
        last_status_log = 0.0
        while not rospy.is_shutdown():
            now = time.time()
            if now - last_status_log >= 2.0:
                rospy.loginfo_throttle(2.0, "Listening. %s", node.capture_status())
                last_status_log = now

            key = key_reader.poll()
            if key in (save_key, save_key.upper()):
                node.save_capture()
            elif key in (quit_key, quit_key.upper(), "\x1b"):
                break

            rate.sleep()


if __name__ == "__main__":
    main()
