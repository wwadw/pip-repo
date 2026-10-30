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

Dense capture freezes the trigger image and accumulates subsequent point clouds
until the configured frame count or duration is reached.
"""

import argparse
import math
import os
import select
import sys
import tempfile
import threading
import time
import termios
import tty
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence, Tuple
from uuid import uuid4

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

  Dense capture, 50 point cloud frames per key press:
    python calib_snap.py --input-mode camera --camera-source 0 --pointcloud-topic /livox/lidar --capture-mode dense --accumulate-frames 50
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
        "--capture-mode",
        choices=["auto", "paired", "image", "dense"],
        default="auto",
        help=(
            "Capture behavior. 'auto' captures image+pointcloud when "
            "--pointcloud-topic is set, otherwise image-only. 'paired' requires "
            "--pointcloud-topic. 'image' saves images only. 'dense' accumulates "
            "point clouds after the save key and saves one paired sample."
        ),
    )
    target = parser.add_mutually_exclusive_group()
    target.add_argument(
        "--accumulate-frames", type=int,
        help="Dense mode: finish after this many nonempty point cloud messages.",
    )
    target.add_argument(
        "--accumulate-seconds", type=float,
        help="Dense mode: collect for this many seconds after the save key.",
    )
    parser.add_argument(
        "--accumulate-timeout", type=float,
        help="Dense mode: maximum silence between accepted clouds (default: 5 seconds).",
    )
    parser.add_argument(
        "--accumulate-max-mb", type=int,
        help="Dense mode: raw point cloud cache limit in MiB (default: 256).",
    )
    parser.add_argument(
        "--voxel-size", type=float,
        help=(
            "Dense mode: voxel edge length in meters for downsampling the merged "
            "cloud before saving; 0 disables it (default: 0)."
        ),
    )
    parser.add_argument(
        "--cancel-key",
        help="Dense mode: cancel a capture before writing starts (default: c).",
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
        "--camera-width",
        type=int,
        help="Requested camera capture width in pixels. Used only in camera mode.",
    )
    parser.add_argument(
        "--camera-height",
        type=int,
        help="Requested camera capture height in pixels. Used only in camera mode.",
    )
    parser.add_argument(
        "--camera-mjpg",
        action="store_true",
        help="Request MJPG camera capture format. Used only in camera mode.",
    )
    parser.add_argument(
        "--camera-fps",
        type=float,
        help=(
            "Requested camera hardware capture FPS. Used only in camera mode. "
            "This is separate from --fps, which controls the read/publish loop."
        ),
    )
    parser.add_argument(
        "--source-image-topic",
        help="ROS image topic used in dual_ros mode.",
    )
    parser.add_argument(
        "--pointcloud-topic",
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
        "--save-dir",
        default="./images",
        help="Image-only output directory. Used only when capture mode is image.",
    )
    parser.add_argument(
        "--image-ext",
        default="png",
        choices=["png", "jpg", "jpeg"],
        help="Image file extension. Defaults to png.",
    )
    parser.add_argument(
        "--image-save-mode",
        choices=["interval", "manual", "both"],
        default="interval",
        help=(
            "Image-only save trigger. 'interval' saves every --image-interval "
            "frames, 'manual' saves only on --save-key, and 'both' enables both."
        ),
    )
    parser.add_argument(
        "--image-interval",
        type=int,
        default=10,
        help="Image-only automatic save interval in frames. Defaults to 10.",
    )
    parser.add_argument(
        "--prefix",
        default="img",
        help="Image-only filename prefix. Defaults to img.",
    )
    parser.add_argument(
        "--image-quality",
        type=int,
        default=95,
        help="Image-only save quality from 1 to 100. Defaults to 95.",
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
            "many seconds; dense mode checks each cloud when received. "
            "Set to a negative value to disable."
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
    has_camera_width = args.camera_width is not None
    has_camera_height = args.camera_height is not None
    has_camera_fps = args.camera_fps is not None
    if has_camera_width != has_camera_height:
        parser.error("--camera-width and --camera-height must be provided together.")
    if args.input_mode != "camera" and (
        has_camera_width or has_camera_height or args.camera_mjpg or has_camera_fps
    ):
        parser.error("camera capture options are only supported when --input-mode=camera.")
    if has_camera_width and (args.camera_width <= 0 or args.camera_height <= 0):
        parser.error("--camera-width and --camera-height must be positive integers.")
    if has_camera_fps and args.camera_fps <= 0:
        parser.error("--camera-fps must be positive.")
    args.resolved_capture_mode = resolve_capture_mode(args, parser)
    dense_options = (
        args.accumulate_frames, args.accumulate_seconds, args.accumulate_timeout,
        args.accumulate_max_mb, args.cancel_key, args.voxel_size,
    )
    if args.resolved_capture_mode != "dense" and any(v is not None for v in dense_options):
        parser.error(
            "Accumulation options, --voxel-size and --cancel-key require --capture-mode=dense."
        )
    args.accumulate_timeout = 5.0 if args.accumulate_timeout is None else args.accumulate_timeout
    args.accumulate_max_mb = 256 if args.accumulate_max_mb is None else args.accumulate_max_mb
    args.cancel_key = "c" if args.cancel_key is None else args.cancel_key
    args.voxel_size = 0.0 if args.voxel_size is None else args.voxel_size
    if args.resolved_capture_mode == "dense":
        try:
            validate_dense_limits(
                args.accumulate_frames, args.accumulate_seconds,
                args.accumulate_timeout, args.accumulate_max_mb * 1024 * 1024,
                args.max_pointcloud_age,
            )
            validate_voxel_size(args.voxel_size)
        except ValueError as exc:
            parser.error(str(exc))
        keys = [value[:1].lower() for value in (args.save_key, args.cancel_key, args.quit_key)]
        if not all(keys) or len(set(keys)) != 3 or "\x1b" in keys[:2]:
            parser.error("Dense save, cancel and quit keys must be nonempty and distinct.")
        args.save_key, args.cancel_key, args.quit_key = keys
    return args


def resolve_capture_mode(args: argparse.Namespace, parser=None) -> str:
    mode = args.capture_mode
    pointcloud_topic = getattr(args, "pointcloud_topic", None)
    if mode == "auto":
        return "paired" if pointcloud_topic else "image"
    if mode in {"paired", "dense"} and not pointcloud_topic:
        message = f"--pointcloud-topic is required when --capture-mode={mode}."
        if parser is not None:
            parser.error(message)
        raise ValueError(message)
    if mode == "image":
        return "image"
    if mode == "dense":
        return "dense"
    return "paired"


def normalize_extension(ext: str) -> str:
    ext = ext.lower().lstrip(".")
    if ext == "jpeg":
        return "jpg"
    return ext


class ImageFileWriter:
    def __init__(
        self,
        save_dir: Path,
        image_ext: str,
        prefix: str,
        quality: int,
    ) -> None:
        self.save_dir = Path(save_dir).expanduser().resolve()
        self.image_ext = normalize_extension(image_ext)
        self.prefix = prefix
        self.quality = min(100, max(1, quality))
        self.saved_count = 0
        self.save_dir.mkdir(parents=True, exist_ok=True)

    def _write_params(self) -> List[int]:
        if self.image_ext in {"jpg", "jpeg"}:
            return [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        if self.image_ext == "png":
            compression = 9 - int(self.quality / 11)
            compression = min(9, max(0, compression))
            return [cv2.IMWRITE_PNG_COMPRESSION, compression]
        return []

    def save(self, frame) -> Path:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = self.save_dir / f"{self.prefix}_{timestamp}.{self.image_ext}"
        if not cv2.imwrite(str(path), frame, self._write_params()):
            raise RuntimeError(f"Failed to save image to {path}")
        self.saved_count += 1
        return path


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


def configure_camera_capture_size(cap: cv2.VideoCapture, args: argparse.Namespace) -> None:
    if getattr(args, "input_mode", None) != "camera":
        return
    width = getattr(args, "camera_width", None)
    height = getattr(args, "camera_height", None)
    camera_mjpg = bool(getattr(args, "camera_mjpg", False))
    camera_fps = getattr(args, "camera_fps", None)
    if width is None and height is None and not camera_mjpg and camera_fps is None:
        return

    failed_options = []
    if camera_mjpg:
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        if not cap.set(cv2.CAP_PROP_FOURCC, fourcc):
            failed_options.append("mjpg")

    if width is not None and height is not None:
        if not cap.set(cv2.CAP_PROP_FRAME_WIDTH, width):
            failed_options.append("width")
        if not cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height):
            failed_options.append("height")

    if camera_fps is not None and not cap.set(cv2.CAP_PROP_FPS, camera_fps):
        failed_options.append("fps")

    if failed_options:
        rospy.logwarn(
            "Camera capture option request may not be supported: %s",
            ",".join(failed_options),
        )
        return

    rospy.loginfo(
        "Requested camera capture options: mjpg=%s width=%s height=%s camera_fps=%s",
        camera_mjpg,
        width,
        height,
        camera_fps,
    )


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


def validate_dense_limits(frame_limit, duration, timeout, max_bytes, max_age):
    if (frame_limit is None) == (duration is None):
        raise ValueError("Choose exactly one of --accumulate-frames or --accumulate-seconds.")
    if frame_limit is not None and (
        not isinstance(frame_limit, int) or isinstance(frame_limit, bool) or frame_limit <= 0
    ):
        raise ValueError("--accumulate-frames must be a positive integer.")
    if duration is not None and (not math.isfinite(duration) or duration <= 0):
        raise ValueError("--accumulate-seconds must be finite and positive.")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("--accumulate-timeout must be finite and positive.")
    if not isinstance(max_bytes, int) or max_bytes <= 0:
        raise ValueError("--accumulate-max-mb must be a positive integer.")
    if not math.isfinite(max_age):
        raise ValueError("--max-pointcloud-age must be finite in dense mode.")


def validate_voxel_size(voxel_size):
    if not math.isfinite(voxel_size) or voxel_size < 0:
        raise ValueError("--voxel-size must be finite and nonnegative; 0 disables it.")


def dense_capture_fields(msg, requested):
    lookup = pointfield_map(msg)
    if len(lookup) != len(msg.fields):
        raise ValueError("Point cloud contains duplicate field names.")
    fields = [
        lookup[name] for name in dict.fromkeys(requested)
        if name in lookup and lookup[name].datatype in PCD_DATATYPE_MAP
    ]
    selected = {item.name: item for item in fields}
    if any(name not in selected or selected[name].count != 1 for name in ("x", "y", "z")):
        raise ValueError("Dense point clouds require scalar x, y, z output fields.")
    return fields


def validate_pointcloud_layout(msg, fields):
    if msg.width <= 0 or msg.height <= 0 or msg.point_step <= 0:
        raise ValueError("Point cloud has an invalid shape or point stride.")
    if msg.row_step < msg.width * msg.point_step:
        raise ValueError("Point cloud row stride is too small.")
    if len(msg.data) < msg.row_step * msg.height:
        raise ValueError("Point cloud data is shorter than its declared layout.")
    previous_end = 0
    for item in sorted(fields, key=lambda item: item.offset):
        size = PCD_DATATYPE_MAP[item.datatype][0]
        if item.count <= 0 or item.offset < 0 or item.offset + size * item.count > msg.point_step:
            raise ValueError(f"Point cloud field '{item.name}' has an invalid layout.")
        if item.offset < previous_end:
            raise ValueError(f"Selected point cloud field '{item.name}' overlaps another field.")
        previous_end = item.offset + size * item.count


def dense_field_schema(fields):
    return tuple((item.name, item.datatype, item.count) for item in fields)


@dataclass(frozen=True)
class DenseCaptureSnapshot:
    session_id: int
    image: object
    image_stamp: object
    clouds: tuple
    fields: tuple
    started_at: float
    finished_at: float
    raw_point_count: int
    byte_count: int
    reason: str
    frame_id: str


class DenseCaptureSession:
    """One capture; callers serialize access and supply both clocks explicitly."""

    def __init__(
        self, requested_fields, frame_limit=None, duration=None,
        timeout=5.0, max_bytes=256 * 1024 * 1024, max_age=0.5,
    ):
        validate_dense_limits(frame_limit, duration, timeout, max_bytes, max_age)
        self.requested_fields = tuple(requested_fields)
        self.frame_limit = frame_limit
        self.duration = duration
        self.timeout = timeout
        self.max_bytes = max_bytes
        self.max_age = max_age
        self.session_id = 0
        self.error = None
        self.state = "IDLE"
        self._clear()

    def _clear(self):
        self.image = None
        self.image_stamp = None
        self.clouds = []
        self.fields = ()
        self.frame_id = ""
        self.frame_count = 0
        self.raw_point_count = 0
        self.byte_count = 0
        self.skipped_count = 0
        self.started_at = None
        self.finished_at = None
        self._last_receipt = None
        self._last_stamp = None
        self._reason = ""

    def start(self, image, image_stamp, now):
        if self.state != "IDLE" or image is None:
            return False
        self._clear()
        self.session_id += 1
        self.error = None
        self.image = image
        self.image_stamp = image_stamp
        self.started_at = now
        self._last_receipt = now
        self.state = "COLLECTING"
        return True

    def _fail(self, reason):
        self._clear()
        self.error = reason
        self.state = "IDLE"

    def _check_deadlines(self, now):
        if self.state != "COLLECTING":
            return
        silence = self._last_receipt + self.timeout
        deadline = None if self.duration is None else self.started_at + self.duration
        if deadline is not None and deadline <= silence and now >= deadline:
            if not self.frame_count:
                self._fail("No point clouds received within the capture window.")
            else:
                self.state = "READY"
                self.finished_at = deadline
                self._reason = "duration"
        elif now >= silence:
            self._fail(f"Point cloud timeout: no accepted cloud for {self.timeout:g}s.")

    def add_cloud(self, msg, received_at, ros_now):
        if self.state != "COLLECTING" or received_at < self.started_at:
            return False
        self._check_deadlines(received_at)
        if self.state != "COLLECTING":
            return False
        try:
            stamp = msg.header.stamp.to_sec()
            if not math.isfinite(stamp):
                raise ValueError("Point cloud timestamp must be finite.")
            if msg.width == 0 or msg.height == 0:
                self.skipped_count += 1
                return False
            if stamp and self.max_age >= 0 and ros_now - stamp > self.max_age:
                self.skipped_count += 1
                return False
            if stamp and self._last_stamp is not None and stamp <= self._last_stamp:
                self.skipped_count += 1
                return False
            names = [item.name for item in self.fields] if self.fields else self.requested_fields
            fields = dense_capture_fields(msg, names)
            validate_pointcloud_layout(msg, fields)
            if self.frame_count and (
                msg.header.frame_id != self.frame_id
                or dense_field_schema(fields) != dense_field_schema(self.fields)
            ):
                raise ValueError("Point cloud frame_id or selected field schema changed.")
            if self.byte_count + len(msg.data) > self.max_bytes:
                raise ValueError("Raw point cloud cache limit exceeded.")
        except (ValueError, TypeError, AttributeError, KeyError) as exc:
            self._fail(str(exc))
            return False
        if not self.frame_count:
            self.fields = tuple(fields)
            self.frame_id = msg.header.frame_id
        self.clouds.append(msg)
        self.frame_count += 1
        self.raw_point_count += msg.width * msg.height
        self.byte_count += len(msg.data)
        self._last_receipt = received_at
        if stamp:
            self._last_stamp = stamp
        if self.frame_limit is not None and self.frame_count >= self.frame_limit:
            self.state = "READY"
            self.finished_at = received_at
            self._reason = "frames"
        return True

    def take_ready(self, now):
        self._check_deadlines(now)
        if self.state != "READY":
            return None
        snapshot = DenseCaptureSnapshot(
            self.session_id, self.image, self.image_stamp, tuple(self.clouds),
            self.fields, self.started_at, self.finished_at, self.raw_point_count,
            self.byte_count, self._reason, self.frame_id,
        )
        self.state = "SAVING"
        self.image = None
        self.image_stamp = None
        self.clouds = []
        return snapshot

    def cancel(self):
        if self.state not in {"COLLECTING", "READY"}:
            return False
        self._clear()
        self.state = "IDLE"
        self.error = None
        return True

    def finish_save(self, session_id):
        if self.state == "SAVING" and session_id == self.session_id:
            self._clear()
            self.state = "IDLE"

    def status(self, now):
        if self.state == "IDLE":
            return "dense: IDLE"
        target = str(self.frame_limit) if self.frame_limit is not None else "-"
        elapsed = (self.finished_at if self.finished_at is not None else now) - self.started_at
        return (
            f"dense: {self.state}, frames={self.frame_count}/{target}, "
            f"raw points={self.raw_point_count}, elapsed={elapsed:.2f}s"
            f"{('/' + str(self.duration) + 's') if self.duration is not None else ''}, "
            f"cache={self.byte_count / (1024 * 1024):.2f}/"
            f"{self.max_bytes / (1024 * 1024):g}MiB, skipped={self.skipped_count}"
        )


def iter_dense_points(clouds, fields):
    names = [item.name for item in fields]
    schema = dense_field_schema(fields)
    xyz_indices = []
    column = 0
    for item in fields:
        if item.name in {"x", "y", "z"}:
            xyz_indices.append(column)
        column += item.count
    frame_id = clouds[0].header.frame_id
    for msg in clouds:
        selected = dense_capture_fields(msg, names)
        validate_pointcloud_layout(msg, selected)
        if dense_field_schema(selected) != schema or msg.header.frame_id != frame_id:
            raise ValueError("Point cloud frame_id or selected field schema changed.")
        # ROS read_points yields selected fields by byte offset, not requested order.
        source_columns = {}
        column = 0
        for item in sorted(selected, key=lambda item: item.offset):
            source_columns[item.name] = tuple(range(column, column + item.count))
            column += item.count
        output_columns = [index for name in names for index in source_columns[name]]
        for values in point_cloud2.read_points(msg, field_names=names, skip_nans=False):
            ordered = tuple(values[index] for index in output_columns)
            if all(math.isfinite(ordered[index]) for index in xyz_indices):
                yield ordered


def voxel_downsample_points(points, fields, voxel_size):
    """Use XYZ centroids and retain the first observation's other fields."""
    xyz_columns = []
    column = 0
    for item in fields:
        if item.name in {"x", "y", "z"}:
            if item.datatype not in {PointField.FLOAT32, PointField.FLOAT64}:
                raise ValueError("Voxel downsampling requires floating-point x, y, z fields.")
            xyz_columns.append(column)
        column += item.count
    voxels = {}
    for point in points:
        scaled = tuple(point[index] / voxel_size for index in xyz_columns)
        if not all(math.isfinite(value) for value in scaled):
            raise ValueError("Point coordinates are too large relative to --voxel-size.")
        key = tuple(math.floor(value) for value in scaled)
        state = voxels.get(key)
        if state is None:
            voxels[key] = [1, list(point)]
        else:
            state[0] += 1
            # An incremental mean avoids overflowing sums for large coordinates.
            for index in xyz_columns:
                state[1][index] += (point[index] - state[1][index]) / state[0]
    return [point for count, point in voxels.values()]


def save_pointclouds_as_pcd(clouds, output_path, requested_fields, voxel_size=0.0):
    validate_voxel_size(voxel_size)
    if not clouds:
        raise ValueError("No point clouds to save.")
    fields = dense_capture_fields(clouds[0], requested_fields)
    if voxel_size > 0:
        points = voxel_downsample_points(iter_dense_points(clouds, fields), fields, voxel_size)
        point_count = len(points)
    else:
        point_count = sum(1 for _ in iter_dense_points(clouds, fields))
        points = iter_dense_points(clouds, fields)
    if not point_count:
        raise ValueError("No finite XYZ points to save.")
    names = [item.name for item in fields]
    header = [
        "# .PCD v0.7 - Point Cloud Data file format", "VERSION 0.7",
        f"FIELDS {' '.join(names)}",
        "SIZE " + " ".join(str(PCD_DATATYPE_MAP[item.datatype][0]) for item in fields),
        "TYPE " + " ".join(PCD_DATATYPE_MAP[item.datatype][1] for item in fields),
        "COUNT " + " ".join(str(item.count) for item in fields),
        f"WIDTH {point_count}", "HEIGHT 1", "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {point_count}", "DATA ascii",
    ]
    with Path(output_path).open("w", encoding="ascii") as stream:
        stream.write("\n".join(header) + "\n")
        for point in points:
            stream.write(" ".join(format_pcd_value(value) for value in point) + "\n")
    return point_count, names


def save_dense_capture_pair(
    snapshot, image_dir, pointcloud_dir, image_ext, capture_index, voxel_size=0.0,
):
    temporary = []
    published = []
    try:
        for directory, suffix in ((image_dir, "." + image_ext), (pointcloud_dir, ".pcd")):
            fd, path = tempfile.mkstemp(prefix=".capture-", suffix=suffix, dir=str(directory))
            os.close(fd)
            temporary.append(Path(path))
        if not cv2.imwrite(str(temporary[0]), snapshot.image):
            raise RuntimeError("Failed to write capture image.")
        point_count, names = save_pointclouds_as_pcd(
            snapshot.clouds, temporary[1], [item.name for item in snapshot.fields],
            voxel_size=voxel_size,
        )
        for _ in range(10):
            stem = (
                f"capture_{capture_index:06d}_{datetime.now():%Y%m%d_%H%M%S_%f}"
                f"_{uuid4().hex[:8]}"
            )
            paths = (image_dir / f"{stem}.{image_ext}", pointcloud_dir / f"{stem}.pcd")
            try:
                # Hard links publish complete files without replacing an existing sample.
                for source, destination in zip(temporary, paths):
                    os.link(str(source), str(destination))
                    published.append(destination)
            except FileExistsError:
                for path in published:
                    path.unlink()
                published.clear()
                continue
            break
        else:
            raise RuntimeError("Unable to allocate a unique capture filename.")
        return paths[0], paths[1], point_count, names
    except Exception:
        for path in published:
            path.unlink(missing_ok=True)
        raise
    finally:
        for path in temporary:
            path.unlink(missing_ok=True)


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
        capture_mode: str,
        pointcloud_topic: Optional[str],
        source_image_topic: Optional[str],
        image_topic: str,
        compressed_topic: str,
        frame_id: str,
        output_dir: Path,
        image_ext: str,
        requested_pcd_fields: Sequence[str],
        max_pointcloud_age: float,
        image_writer: Optional[ImageFileWriter] = None,
        image_save_mode: str = "interval",
        image_interval: int = 10,
        accumulate_frames: Optional[int] = None,
        accumulate_seconds: Optional[float] = None,
        accumulate_timeout: float = 5.0,
        accumulate_max_mb: int = 256,
        voxel_size: float = 0.0,
    ) -> None:
        self._input_mode = input_mode
        self._capture_mode = capture_mode
        self._frame_id = frame_id
        self._source_image_topic = source_image_topic or ""
        self._image_ext = image_ext
        self._requested_pcd_fields = requested_pcd_fields
        self._max_pointcloud_age = max_pointcloud_age
        validate_voxel_size(voxel_size)
        self._voxel_size = voxel_size
        self._image_writer = image_writer
        self._image_save_mode = image_save_mode
        self._image_interval = max(1, image_interval)
        self._bridge = CvBridge() if CvBridge is not None else None
        self._lock = threading.Lock()
        self._latest_pointcloud = None
        self._latest_frame = None
        self._latest_image_stamp = rospy.Time()
        self._capture_index = 0
        self._source_frame_count = 0
        self._closed = False
        self._monotonic = time.monotonic
        self._dense_session = None
        self._dense_thread = None
        self._dense_event = threading.Event()
        self._dense_reported_error = 0
        if self._capture_mode == "dense":
            self._dense_session = DenseCaptureSession(
                requested_fields=requested_pcd_fields, frame_limit=accumulate_frames,
                duration=accumulate_seconds, timeout=accumulate_timeout,
                max_bytes=accumulate_max_mb * 1024 * 1024, max_age=max_pointcloud_age,
            )

        self._image_dir = output_dir / "images"
        self._pointcloud_dir = output_dir / "pointclouds"
        if self._capture_mode in {"paired", "dense"}:
            self._image_dir.mkdir(parents=True, exist_ok=True)
            self._pointcloud_dir.mkdir(parents=True, exist_ok=True)
        elif self._image_writer is None:
            self._image_writer = ImageFileWriter(
                save_dir=output_dir,
                image_ext=image_ext,
                prefix="img",
                quality=95,
            )

        self._pointcloud_sub = None
        if self._capture_mode in {"paired", "dense"}:
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

        if self._dense_session is not None:
            rospy.on_shutdown(self.close)
            self._dense_thread = threading.Thread(
                target=self._run_dense_worker, name="calib-snap-dense", daemon=True
            )
            self._dense_thread.start()

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        received_at = self._monotonic()
        ros_now = rospy.Time.now().to_sec() if self._dense_session is not None else None
        error = None
        with self._lock:
            self._latest_pointcloud = msg
            if self._dense_session is not None and not self._closed:
                self._dense_session.add_cloud(msg, received_at, ros_now)
                error = self._take_dense_error()
        if error:
            rospy.logerr("Dense capture failed: %s", error)
        if self._dense_session is not None:
            self._dense_event.set()

    def _take_dense_error(self):
        session = self._dense_session
        if session.error and session.session_id != self._dense_reported_error:
            self._dense_reported_error = session.session_id
            return session.error
        return None

    def _run_dense_worker(self):
        while True:
            self._dense_event.wait(0.05)
            self._dense_event.clear()
            with self._lock:
                if self._closed:
                    return
            self._process_dense_capture()

    def _process_dense_capture(self):
        with self._lock:
            if self._closed or self._dense_session is None:
                return
            snapshot = self._dense_session.take_ready(self._monotonic())
            error = self._take_dense_error()
            capture_index = self._capture_index + 1
        if error:
            rospy.logerr("Dense capture failed: %s", error)
        if snapshot is None:
            return
        try:
            image_path, pointcloud_path, point_count, names = save_dense_capture_pair(
                snapshot, self._image_dir, self._pointcloud_dir, self._image_ext,
                capture_index, self._voxel_size,
            )
            with self._lock:
                self._capture_index = capture_index
            stamps = [
                msg.header.stamp.to_sec() for msg in snapshot.clouds
                if msg.header.stamp.to_sec() != 0
            ]
            rospy.loginfo(
                "Saved dense capture %06d (session=%d): image=%s, pointcloud=%s, "
                "reason=%s, frames=%d, raw_points=%d, saved_points=%d, voxel_size=%gm, "
                "elapsed=%.3fs, "
                "cloud_stamps=%s, image_stamp=%s, frame_id=%s, fields=%s",
                capture_index, snapshot.session_id, image_path, pointcloud_path,
                snapshot.reason, len(snapshot.clouds), snapshot.raw_point_count,
                point_count, self._voxel_size, snapshot.finished_at - snapshot.started_at,
                (min(stamps), max(stamps)) if stamps else "unstamped",
                snapshot.image_stamp, snapshot.frame_id, ",".join(names),
            )
        except Exception as exc:
            rospy.logerr("Failed to save dense capture %d: %s", snapshot.session_id, exc)
        finally:
            with self._lock:
                self._dense_session.finish_save(snapshot.session_id)

    def close(self):
        with self._lock:
            self._closed = True
            if self._dense_session is not None:
                self._dense_session.cancel()
            worker = self._dense_thread
        self._dense_event.set()
        if worker is not None and worker is not threading.current_thread() and worker.is_alive():
            worker.join()

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

        self.process_image_frame(frame, msg.header.stamp)

    def publish_image(self, frame) -> None:
        stamp = rospy.Time.now()
        self.process_image_frame(frame, stamp)

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
        if self._capture_mode == "image":
            return self.image_status()
        if self._dense_session is not None:
            with self._lock:
                status = self._dense_session.status(self._monotonic())
            return f"{self.image_status()}, {self.pointcloud_status()}, {status}"
        return f"{self.image_status()}, {self.pointcloud_status()}"

    def process_image_frame(self, frame, stamp=None) -> None:
        if stamp is None:
            stamp = rospy.Time.now()
        with self._lock:
            self._latest_frame = frame
            self._latest_image_stamp = stamp

        if self._capture_mode != "image":
            return

        self._source_frame_count += 1
        if self._image_save_mode not in {"interval", "both"}:
            return
        if self._source_frame_count % self._image_interval != 0:
            return
        self._save_image_frame(frame)

    def _save_image_frame(self, frame) -> bool:
        if self._image_writer is None:
            rospy.logerr("Image-only writer is not configured; image skipped.")
            return False
        try:
            image_path = self._image_writer.save(frame)
        except Exception as exc:
            rospy.logerr("Failed to save image: %s", exc)
            return False
        rospy.loginfo(
            "Saved image %06d: image=%s",
            self._image_writer.saved_count,
            image_path,
        )
        return True

    def save_image_capture(self) -> None:
        with self._lock:
            frame = None if self._latest_frame is None else self._copy_frame(self._latest_frame)

        if frame is None:
            rospy.logwarn("No image frame received yet; image save skipped.")
            return
        self._save_image_frame(frame)

    def handle_save_key(self) -> None:
        if self._capture_mode == "dense":
            with self._lock:
                session = self._dense_session
                if self._closed:
                    return
                if session.state != "IDLE":
                    message = "Dense capture already active. " + session.status(self._monotonic())
                elif self._latest_frame is None:
                    message = "No image frame received yet; dense capture skipped."
                else:
                    session.start(
                        self._copy_frame(self._latest_frame), self._latest_image_stamp,
                        self._monotonic(),
                    )
                    message = (
                        "Started dense capture; keep camera, lidar and calibration target still. "
                        + session.status(self._monotonic())
                    )
            rospy.loginfo("%s", message)
            self._dense_event.set()
            return
        if self._capture_mode == "paired":
            self.save_capture()
            return
        if self._image_save_mode in {"manual", "both"}:
            self.save_image_capture()
            return
        rospy.logwarn(
            "Save key ignored because --image-save-mode=%s only saves by interval.",
            self._image_save_mode,
        )

    def handle_cancel_key(self):
        if self._dense_session is None:
            return
        with self._lock:
            cancelled = self._dense_session.cancel()
            status = self._dense_session.status(self._monotonic())
        rospy.loginfo("Dense capture %s. %s", "cancelled" if cancelled else "unchanged", status)
        self._dense_event.set()

    def _copy_frame(self, frame):
        return frame.copy() if hasattr(frame, "copy") else frame

    def save_capture(self) -> None:
        with self._lock:
            frame = None if self._latest_frame is None else self._copy_frame(self._latest_frame)
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
    cancel_key: str = "c",
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
                else:
                    node.publish_image(frame)

                now = time.time()
                if now - last_status_log >= 2.0:
                    rospy.loginfo_throttle(2.0, "Streaming. %s", node.capture_status())
                    last_status_log = now

                key = key_reader.poll()
                if key in (save_key, save_key.upper()):
                    node.handle_save_key()
                elif key in (quit_key, quit_key.upper(), "\x1b"):
                    break
                elif key in (cancel_key, cancel_key.upper()) and node._capture_mode == "dense":
                    node.handle_cancel_key()

                rate.sleep()
    finally:
        cap.release()
        node.close()


def main() -> None:
    args = parse_args()
    image_ext = normalize_extension(args.image_ext)
    capture_mode = args.resolved_capture_mode
    requested_fields = (
        sanitize_fields(args.pcd_fields) if capture_mode in {"paired", "dense"} else []
    )

    rospy.init_node("calib_snap")
    output_dir = Path(args.output_dir).expanduser().resolve()
    image_writer = None
    if capture_mode == "image":
        image_writer = ImageFileWriter(
            save_dir=Path(args.save_dir),
            image_ext=image_ext,
            prefix=args.prefix,
            quality=args.image_quality,
        )
        if args.image_save_mode == "manual":
            rospy.loginfo(
                "Image-only manual mode enabled; --image-interval is ignored."
            )
    node = CalibrationCaptureNode(
        input_mode=args.input_mode,
        capture_mode=capture_mode,
        pointcloud_topic=args.pointcloud_topic,
        source_image_topic=args.source_image_topic,
        image_topic=args.image_topic,
        compressed_topic=args.compressed_topic,
        frame_id=args.frame_id,
        output_dir=output_dir,
        image_ext=image_ext,
        requested_pcd_fields=requested_fields,
        max_pointcloud_age=args.max_pointcloud_age,
        image_writer=image_writer,
        image_save_mode=args.image_save_mode,
        image_interval=args.image_interval,
        accumulate_frames=args.accumulate_frames,
        accumulate_seconds=args.accumulate_seconds,
        accumulate_timeout=args.accumulate_timeout,
        accumulate_max_mb=args.accumulate_max_mb,
        voxel_size=args.voxel_size,
    )
    try:
        run_capture_inputs(args, node, image_writer)
    finally:
        node.close()


def run_capture_inputs(args, node, image_writer=None):
    capture_mode = args.resolved_capture_mode
    rate = rospy.Rate(max(args.fps, 1.0))
    save_key = args.save_key[:1]
    quit_key = args.quit_key[:1]
    if capture_mode == "dense":
        rospy.loginfo(
            "Dense capture: frames=%s, seconds=%s, silence_timeout=%.3fs, cache=%dMiB, "
            "voxel_size=%gm. "
            "Keep camera, lidar and calibration target still while collecting.",
            args.accumulate_frames, args.accumulate_seconds,
            args.accumulate_timeout, args.accumulate_max_mb, args.voxel_size,
        )
        rospy.loginfo(
            "Terminal hotkeys enabled: [%s] start accumulation, [%s] cancel, [%s] quit",
            save_key, args.cancel_key[:1], quit_key,
        )
    elif capture_mode == "paired":
        rospy.loginfo(
            "Terminal hotkeys enabled: [%s] save current image + latest pointcloud, [%s] quit",
            save_key,
            quit_key,
        )
    else:
        rospy.loginfo(
            "Image-only mode=%s, save_dir=%s, prefix=%s, interval=%d",
            args.image_save_mode,
            image_writer.save_dir if image_writer is not None else "",
            args.prefix,
            max(1, args.image_interval),
        )
        rospy.loginfo(
            "Terminal hotkeys enabled: [%s] save current image when manual/both mode allows it, [%s] quit",
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
                "Capture mode=%s, input=rtsp, rtsp_uri=%s, pointcloud_topic=%s, image_topic=%s, compressed_topic=%s",
                capture_mode,
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
                "Capture mode=%s, input=camera, camera_source=%s, pointcloud_topic=%s, image_topic=%s, compressed_topic=%s",
                capture_mode,
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
        configure_camera_capture_size(cap, args)

        run_stream_capture(
            node=node,
            cap=cap,
            source_label=source_label,
            rate=rate,
            save_key=save_key,
            quit_key=quit_key,
            cancel_key=args.cancel_key[:1],
        )
        return

    rospy.loginfo(
        "Capture mode=%s, input=dual_ros, source_image_topic=%s, pointcloud_topic=%s",
        capture_mode,
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
                node.handle_save_key()
            elif key in (quit_key, quit_key.upper(), "\x1b"):
                break
            elif key in (args.cancel_key[:1], args.cancel_key[:1].upper()) and capture_mode == "dense":
                node.handle_cancel_key()

            rate.sleep()


if __name__ == "__main__":
    main()
