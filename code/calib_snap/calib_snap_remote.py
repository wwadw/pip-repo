"""Calibration capture over a discovered network connection or standard streams."""

import argparse
import base64
import fcntl
import json
import os
from pathlib import Path
import re
import select
import shutil
import struct
import sys
import threading
import time
from urllib.parse import urlsplit
from uuid import uuid4

from calib_snap import (
    CvBridge, DenseCaptureSession, Image, MECHANICAL_MODE, PCD_DATATYPE_MAP, PointCloud2, cv2,
    dense_capture_fields, image_msg_to_bgr, mechanical_capture_fields, np,
    point_cloud2, rospy, save_dense_capture_pair, save_pointcloud_as_pcd,
    stamp_metadata, validate_pointcloud_layout,
)
from calib_snap_ros import RosInput, TOPIC_TYPES, master_uri, published_topics, publisher_ip
from calib_snap_network import serve_network


PROTOCOL_VERSION = 1
MAX_COMMAND = 16 * 1024
MAX_SAMPLE = 2 * 1024**3
MAX_SOURCE_AGE = 3
TOPIC_NAME = re.compile(r"/[A-Za-z][A-Za-z0-9_]*(?:/[A-Za-z0-9_]+)*")
CAPTURE_ID = re.compile(r"[0-9a-f]{32}")
SINGLE_FRAME_MODES = {"paired", "image-only", "pointcloud-only"}
MODES = SINGLE_FRAME_MODES | {"dense", MECHANICAL_MODE}
SAMPLE_FILES = {"image": "image.png", "pointcloud": "cloud.pcd", "metadata": "metadata.json"}


def exit_on_timeout():
    sys.stderr.write("calib-snap-remote: ROS or input setup timed out.\n")
    sys.stderr.flush()
    os._exit(1)


def topic_name(value):
    if not isinstance(value, str) or not TOPIC_NAME.fullmatch(value):
        raise ValueError("Enter a full ROS topic name, starting with /.")
    return value


def source_config(request):
    mode = request.get("image_source")
    if mode == "rtsp":
        value = request.get("rtsp_uri")
        if not isinstance(value, str) or len(value) > 2048 or any(ch.isspace() for ch in value):
            raise ValueError("Enter a valid RTSP address.")
        try:
            parsed = urlsplit(value)
            valid = parsed.scheme in {"rtsp", "rtsps"} and parsed.hostname
        except ValueError:
            valid = False
        if not valid:
            raise ValueError("Enter an rtsp:// or rtsps:// address.")
        image = value
    elif mode == "ros":
        image = topic_name(request.get("image_topic"))
    elif mode == "none":
        image = None
    else:
        raise ValueError("Select RTSP, a ROS image topic or point cloud only.")
    pointcloud = request.get("pointcloud_topic")
    if pointcloud is not None:
        pointcloud = topic_name(pointcloud)
    elif image is None:
        raise ValueError("Select an image or point cloud source.")
    return mode, image, pointcloud


def capture_config(request):
    mode = request.get("mode")
    if not isinstance(mode, str) or mode not in MODES:
        raise ValueError("Unknown capture mode.")
    frames, seconds = request.get("frames"), request.get("seconds")
    if mode in SINGLE_FRAME_MODES:
        return mode, None, None
    if (frames is None) == (seconds is None):
        raise ValueError("Specify either a frame count or a duration.")
    if frames is not None and (type(frames) is not int or not 1 <= frames <= 500):
        raise ValueError("Frame count must be between 1 and 500.")
    if seconds is not None and (type(seconds) not in {int, float} or not 0.1 <= seconds <= 30):
        raise ValueError("Duration must be between 0.1 and 30 seconds.")
    return mode, frames, seconds


def capture_field_names(cloud, mode):
    names = [
        field.name for field in sorted(cloud.fields, key=lambda field: field.offset)
        if field.datatype in PCD_DATATYPE_MAP
        and re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", field.name)
        and (mode not in SINGLE_FRAME_MODES or field.count == 1)
    ]
    if mode == MECHANICAL_MODE:
        mechanical_capture_fields(cloud, names)
    else:
        validate_pointcloud_layout(cloud, dense_capture_fields(cloud, names))
    return names


class Sources:
    def __init__(self, mode, image, pointcloud, ros_sources):
        self.mode, self.image, self.pointcloud = mode, image, pointcloud
        self.ros_sources = ros_sources
        self.revision = uuid4().hex
        self.condition = threading.Condition()
        self.frame = self.cloud = self.stamp = self.accumulation = None
        self.image_received = self.cloud_received = float("-inf")
        self.error = None
        self.stop = threading.Event()
        self.cloud_sub = self.image_sub = self.reader = None
        try:
            if pointcloud is not None:
                self.cloud_sub = RosInput(
                    "pointcloud", pointcloud, *ros_sources["pointcloud"], PointCloud2, self.on_cloud,
                )
            if mode == "ros":
                self.bridge = CvBridge() if CvBridge else None
                self.image_sub = RosInput("image", image, *ros_sources["image"], Image, self.on_image)
            elif mode == "rtsp":
                self.reader = threading.Thread(target=self.read_rtsp, daemon=True)
                self.reader.start()
        except Exception:
            self.close()
            raise

    def close(self):
        self.stop.set()
        for subscriber in (self.cloud_sub, self.image_sub):
            if subscriber is not None:
                subscriber.unregister()
        if self.reader is not None and self.reader.is_alive():
            self.reader.join()

    def accept_image(self, frame, stamp):
        with self.condition:
            self.frame, self.stamp = frame, stamp
            self.image_received = time.monotonic()
            self.error = None

    def on_image(self, message, _ros_now):
        try:
            frame = (self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                     if self.bridge else image_msg_to_bgr(message))
        except Exception:
            with self.condition:
                self.error = "ROS image could not be decoded."
            return
        self.accept_image(frame, message.header.stamp)

    def on_cloud(self, message, ros_now):
        if not message.width or not message.height:
            return
        now = time.monotonic()
        with self.condition:
            self.cloud = message
            self.cloud_received = now
            self.cloud_ros_now = ros_now
            if self.accumulation is not None:
                self.accumulation.add_cloud(message, now, ros_now)
                self.condition.notify_all()

    def read_rtsp(self):
        cap = None
        try:
            cap = cv2.VideoCapture(self.image, cv2.CAP_FFMPEG, [
                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 8000, cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000,
            ])
            if not cap.isOpened():
                raise RuntimeError()
            while not self.stop.is_set() and not rospy.is_shutdown():
                success, frame = cap.read()
                if success:
                    self.accept_image(frame, rospy.Time.from_sec(time.time()))
                else:
                    with self.condition:
                        self.error = "RTSP frame read timed out."
                    self.stop.wait(0.2)
        except Exception:
            with self.condition:
                self.error = "Cannot open RTSP stream with the OpenCV FFmpeg backend."
        finally:
            if cap is not None:
                cap.release()

    def readiness(self):
        now = time.monotonic()
        image_ready = now - self.image_received < MAX_SOURCE_AGE and not self.error
        cloud_ready = now - self.cloud_received < MAX_SOURCE_AGE
        return (image_ready and not (self.image_sub and self.image_sub.error),
                cloud_ready and not (self.cloud_sub and self.cloud_sub.error))

    def status(self, check_mechanical=False):
        with self.condition:
            image_ready, cloud_ready = self.readiness()
            error = self.error or next(
                (sub.error for sub in (self.image_sub, self.cloud_sub) if sub and sub.error), None,
            )
            result = dict(revision=self.revision, image_ready=bool(image_ready),
                          cloud_ready=cloud_ready, error=error)
            cloud = self.cloud if cloud_ready and check_mechanical else None
        if check_mechanical:
            result["mechanical_format"] = None
            if cloud is not None:
                try:
                    names = capture_field_names(cloud, MECHANICAL_MODE)
                    result["mechanical_format"] = dict(valid=True, scan_id="scan_id" in names)
                except ValueError as exc:
                    result["mechanical_format"] = dict(valid=False, error=str(exc))
        return result

    def preview(self):
        with self.condition:
            image_ready, cloud_ready = self.readiness()
            frame = self.frame if image_ready else None
            cloud = self.cloud if cloud_ready else None
        jpeg, points = None, []
        if frame is not None:
            scale = min(1, 640 / max(frame.shape[:2]))
            if scale < 1:
                frame = cv2.resize(frame, tuple(max(1, int(size * scale))
                                                for size in frame.shape[1::-1]))
            success, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
            if success:
                jpeg = base64.b64encode(encoded).decode("ascii")
        if cloud is not None:
            try:
                count = cloud.width * cloud.height
                pixels = [(index % cloud.width, index // cloud.width)
                          for index in range(0, count, (count + 1999) // 2000)]
                points = [point for point in point_cloud2.read_points(
                    cloud, field_names=("x", "y", "z"), skip_nans=True, uvs=pixels,
                ) if all(np.isfinite(point))]
            except (ValueError, KeyError, AssertionError, TypeError, struct.error):
                pass
        return dict(image=jpeg, points=points, revision=self.revision)

    def freeze(self, mode, frames, seconds):
        needs_image = mode != "pointcloud-only"
        needs_cloud = mode != "image-only"
        with self.condition:
            image_ready, cloud_ready = self.readiness()
            if needs_image and needs_cloud and not (image_ready and cloud_ready):
                raise ValueError("Wait for a current image and point cloud.")
            if needs_image and not image_ready:
                raise ValueError("Wait for a current image.")
            if needs_cloud and not cloud_ready:
                raise ValueError("Wait for a current point cloud.")
            frame, stamp = (self.frame.copy(), self.stamp) if needs_image else (None, None)
            cloud, names = None, []
            if needs_cloud:
                cloud = self.cloud
                cloud_stamp = cloud.header.stamp.to_sec()
                cloud_ros_now = self.cloud_ros_now + time.monotonic() - self.cloud_received
                if not np.isfinite(cloud_stamp) or (
                    cloud_stamp and cloud_ros_now - cloud_stamp > 0.5
                ):
                    raise ValueError("The current point cloud has a stale or invalid ROS timestamp.")
                names = capture_field_names(cloud, mode)
            if mode not in SINGLE_FRAME_MODES:
                self.accumulation = DenseCaptureSession(
                    requested_fields=names, frame_limit=frames, duration=seconds,
                    timeout=5, max_bytes=256 * 1024**2, max_age=0.5,
                    mechanical=mode == MECHANICAL_MODE,
                )
                self.accumulation.start(frame, stamp, time.monotonic())
            return frame, stamp, cloud, names

    def await_accumulation(self):
        with self.condition:
            session = self.accumulation
            while session.state != "IDLE":
                snapshot = session.take_ready(time.monotonic())
                if snapshot is not None:
                    return snapshot
                self.condition.wait(0.1)
            raise ValueError(session.error or "Capture cancelled.")

    def cancel(self):
        with self.condition:
            cancelled = self.accumulation is not None and self.accumulation.cancel()
            self.condition.notify_all()
            return cancelled


class RemoteCapture:
    def __init__(self, spool):
        self.spool = spool
        self.sources = self.worker = None
        self.last_capture = None

    def busy(self):
        return self.worker is not None and self.worker.is_alive()

    def close(self):
        if self.sources is not None:
            self.sources.cancel()
            if self.worker is not None:
                self.worker.join()
            self.sources.close()

    def topics(self, request):
        result = {"image": [], "pointcloud": [], "errors": {}}
        published = {}
        for name, type_name in TOPIC_TYPES.items():
            if request.get(f"include_{name}") is False:
                continue
            try:
                uri = master_uri(request.get(f"{name}_master"))
                if uri not in published:
                    try:
                        published[uri] = published_topics(uri)
                    except ValueError as exc:
                        published[uri] = exc
                topics = published[uri]
                if isinstance(topics, ValueError):
                    raise topics
                result[name] = sorted(topic for topic, kind in topics if kind == type_name)
            except ValueError as exc:
                result["errors"][name] = str(exc)
        return result

    def configure(self, request):
        if self.busy():
            raise ValueError("A capture is in progress.")
        mode, image, cloud = source_config(request)
        ros_sources = {
            name: (master_uri(request.get(f"{name}_master")), publisher_ip(request.get(f"{name}_publisher_ip")))
            for name, enabled in (("image", mode == "ros"), ("pointcloud", cloud is not None))
            if enabled
        }
        if self.sources is not None:
            self.sources.close()
            self.sources = None
        self.sources = Sources(mode, image, cloud, ros_sources)
        return self.status(request)

    def status(self, request):
        check_mechanical = request.get("mode") == MECHANICAL_MODE
        return dict(source=self.sources.status(check_mechanical) if self.sources else None,
                    busy=self.busy(), capture=self.last_capture)

    def preview(self, _request):
        return self.sources.preview() if self.sources else dict(image=None, points=[], revision=None)

    def capture(self, request):
        if self.busy():
            raise ValueError("A capture is in progress.")
        if not self.sources or self.sources.revision != request.get("revision"):
            raise ValueError("The source configuration has changed. Refresh the preview.")
        capture_id = request.get("id")
        target = self.sample_path(capture_id)
        if target.exists():
            return dict(id=capture_id, state="saved")
        mode, frames, seconds = capture_config(request)
        frozen = self.sources.freeze(mode, frames, seconds)
        self.last_capture = dict(id=capture_id, state="capturing")
        self.worker = threading.Thread(
            target=self.save_sample, args=(target, mode, frozen), daemon=True,
        )
        self.worker.start()
        return self.last_capture

    def save_sample(self, target, mode, frozen):
        staging = self.spool / f".{target.name}"
        try:
            staging.mkdir(mode=0o700)
            frame, stamp, cloud, requested_fields = frozen
            context = dict(
                capture_id=target.name,
                input_mode=self.sources.mode,
                image_stamp_source=("receiver" if self.sources.mode == "rtsp" else "ros_header")
                if frame is not None else None,
                pointcloud_topic=self.sources.pointcloud if cloud is not None else None,
                source_image_topic=self.sources.image if frame is not None and self.sources.mode == "ros" else None,
                source_revision=self.sources.revision,
            )
            for name, saved in (("image", frame is not None), ("pointcloud", cloud is not None)):
                if saved and name in self.sources.ros_sources:
                    context[f"{name}_master"] = self.sources.ros_sources[name][0]
            files = {"metadata": SAMPLE_FILES["metadata"]}
            if mode in SINGLE_FRAME_MODES:
                metadata = dict(context, schema_version=1, capture_mode=mode)
                if frame is not None:
                    files["image"] = SAMPLE_FILES["image"]
                    if not cv2.imwrite(str(staging / files["image"]), frame):
                        raise ValueError("Failed to write the image.")
                    metadata["image_stamp"] = stamp_metadata(stamp)
                if cloud is not None:
                    files["pointcloud"] = SAMPLE_FILES["pointcloud"]
                    point_count, fields = save_pointcloud_as_pcd(
                        cloud, staging / files["pointcloud"], requested_fields,
                    )
                    metadata.update(cloud_stamp=stamp_metadata(cloud.header.stamp),
                                    frame_id=cloud.header.frame_id, saved_point_count=point_count,
                                    output_fields=fields)
            else:
                snapshot = self.sources.await_accumulation()
                image_path, cloud_path, point_count, fields = save_dense_capture_pair(
                    snapshot, staging, staging, "png", 1,
                    metadata_dir=staging, metadata_context=context,
                )
                original_metadata = cloud_path.with_suffix(".json")
                if original_metadata.exists():
                    metadata = json.loads(original_metadata.read_text(encoding="utf-8"))
                    original_metadata.unlink()
                else:
                    metadata = dict(context, schema_version=1, capture_mode=mode,
                                    image_stamp=stamp_metadata(stamp), accepted_frames=len(snapshot.clouds),
                                    cloud_stamps=[stamp_metadata(msg.header.stamp) for msg in snapshot.clouds],
                                    frame_id=snapshot.frame_id, saved_point_count=point_count,
                                    output_fields=fields)
                files = SAMPLE_FILES.copy()
                image_path.rename(staging / files["image"])
                cloud_path.rename(staging / files["pointcloud"])
            metadata["files"] = files
            (staging / files["metadata"]).write_text(
                json.dumps(metadata, allow_nan=False) + "\n", encoding="utf-8",
            )
            if sum((staging / name).stat().st_size for name in files.values()) > MAX_SAMPLE:
                raise ValueError("The captured sample exceeds 2 GiB.")
            staging.rename(target)
            self.last_capture = dict(id=target.name, state="saved")
        except Exception as exc:
            self.last_capture = dict(id=target.name, state="failed", error=str(exc))
        finally:
            shutil.rmtree(staging, ignore_errors=True)
            with self.sources.condition:
                self.sources.accumulation = None

    def cancel(self, _request):
        return dict(cancelled=bool(self.sources and self.sources.cancel()))

    def sample_path(self, capture_id):
        if not isinstance(capture_id, str) or not CAPTURE_ID.fullmatch(capture_id):
            raise ValueError("Invalid capture ID.")
        return self.spool / capture_id

    def pending(self, _request):
        return dict(ids=sorted(
            folder.name for folder in self.spool.iterdir()
            if CAPTURE_ID.fullmatch(folder.name) and folder.is_dir()
        ))

    def fetch(self, request):
        name = request.get("name")
        if name not in SAMPLE_FILES.values():
            raise ValueError("Unknown capture file.")
        path = self.sample_path(request.get("id")) / name
        if not path.is_file():
            raise ValueError("The requested capture file does not exist.")
        return path.open("rb")

    def ack(self, request):
        target = self.sample_path(request.get("id"))
        if target.exists():
            retired = self.spool / f".{target.name}"
            target.rename(retired)
            shutil.rmtree(retired)
        return {}

    def serve(self, reader, writer, idle_timeout, robot_id=None):
        commands = {
            "hello": lambda _: dict(version=PROTOCOL_VERSION, dual_master=True, id=robot_id),
            "topics": self.topics, "configure": self.configure, "status": self.status,
            "preview": self.preview, "capture": self.capture, "cancel": self.cancel,
            "pending": self.pending, "fetch": self.fetch, "ack": self.ack,
        }
        while not rospy.is_shutdown():
            if idle_timeout is not None and not select.select([reader], [], [], idle_timeout)[0]:
                return
            line = reader.readline(MAX_COMMAND + 1)
            if not line or len(line) > MAX_COMMAND or not line.endswith(b"\n"):
                return
            try:
                request = json.loads(line)
                if not isinstance(request, dict):
                    raise ValueError("Expected a JSON object.")
                command = request.get("command")
                if command == "close":
                    return
                if not isinstance(command, str) or command not in commands:
                    raise ValueError("Unknown command.")
                if command in {"topics", "configure"}:
                    watchdog = threading.Timer(12, exit_on_timeout)
                    watchdog.start()
                    try:
                        result = commands[command](request)
                    finally:
                        watchdog.cancel()
                else:
                    result = commands[command](request)
            except Exception as exc:
                result = dict(error=str(exc))
            if isinstance(result, dict):
                writer.write(json.dumps(result, allow_nan=False).encode() + b"\n")
            else:
                with result:
                    length = os.fstat(result.fileno()).st_size
                    writer.write(json.dumps(dict(length=length)).encode() + b"\n")
                    shutil.copyfileobj(result, writer, length=256 * 1024)
            writer.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spool", default=os.environ.get("CALIB_SNAP_SPOOL", "~/.local/state/calib-snap"))
    parser.add_argument("--idle-timeout", type=float, default=60)
    transport = parser.add_mutually_exclusive_group()
    transport.add_argument("--network-config", help="Override LAN discovery and data source settings from a JSON file.")
    transport.add_argument("--stdio", action="store_true", help="Serve requests over standard input and output.")
    args = parser.parse_args()
    if not 10 <= args.idle_timeout <= 600:
        parser.error("--idle-timeout must be between 10 and 600 seconds.")
    spool = Path(args.spool).expanduser().resolve()
    spool.mkdir(mode=0o700, parents=True, exist_ok=True)
    with (spool / ".session.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("A remote capture session is already active.", file=sys.stderr)
            return 1
        for folder in spool.iterdir():
            if folder.is_dir() and CAPTURE_ID.fullmatch(folder.name[1:]) and folder.name.startswith("."):
                shutil.rmtree(folder)
        if not args.stdio:
            def serve(reader, writer, robot_id):
                remote = RemoteCapture(spool)
                try:
                    remote.serve(reader, writer, None, robot_id)
                finally:
                    remote.close()

            serve_network(args.network_config, spool, serve, args.idle_timeout, rospy.is_shutdown)
            return 0
        with os.fdopen(os.dup(sys.stdout.fileno()), "wb") as output:
            sys.stdout.flush()
            os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
            remote = RemoteCapture(spool)
            try:
                with os.fdopen(sys.stdin.fileno(), "rb", buffering=0, closefd=False) as reader:
                    remote.serve(reader, output, args.idle_timeout)
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                remote.close()
    return 0
