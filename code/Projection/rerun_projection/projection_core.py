import bisect
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class ProjectionResult:
    pixel_coordinates: np.ndarray
    front_mask: np.ndarray
    in_image_mask: np.ndarray
    camera_points: np.ndarray
    valid_point_indices: np.ndarray


@dataclass(frozen=True)
class BagMessage:
    stamp: float
    msg: object


def _message_stamp(msg, fallback_stamp: float) -> float:
    if hasattr(msg, "_has_header") and msg._has_header:
        return float(msg.header.stamp.to_sec())
    return float(fallback_stamp)


def load_bag_messages(bag_path: str, image_topic: str, pointcloud_topic: str) -> Tuple[List[BagMessage], List[BagMessage]]:
    import rosbag

    images: List[BagMessage] = []
    clouds: List[BagMessage] = []
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=[image_topic, pointcloud_topic]):
            entry = BagMessage(stamp=_message_stamp(msg, stamp.to_sec()), msg=msg)
            if topic == image_topic:
                images.append(entry)
            elif topic == pointcloud_topic:
                clouds.append(entry)
    return images, clouds


def load_topic_messages(bag_path: str, topic_name: str) -> List[BagMessage]:
    import rosbag

    entries: List[BagMessage] = []
    if not topic_name:
        return entries
    with rosbag.Bag(bag_path) as bag:
        for _, msg, stamp in bag.read_messages(topics=[topic_name]):
            entries.append(BagMessage(stamp=_message_stamp(msg, stamp.to_sec()), msg=msg))
    return entries


def load_first_topic_message(bag_path: str, topic_name: str) -> BagMessage | None:
    import rosbag

    if not topic_name:
        return None
    with rosbag.Bag(bag_path) as bag:
        for _, msg, stamp in bag.read_messages(topics=[topic_name]):
            return BagMessage(stamp=_message_stamp(msg, stamp.to_sec()), msg=msg)
    return None


def load_topic_stamps(bag_path: str, topic_name: str) -> List[float]:
    import rosbag

    stamps: List[float] = []
    if not topic_name:
        return stamps
    with rosbag.Bag(bag_path) as bag:
        for _, msg, stamp in bag.read_messages(topics=[topic_name]):
            stamps.append(_message_stamp(msg, stamp.to_sec()))
    return stamps


def load_topic_message_by_index(bag_path: str, topic_name: str, index: int) -> BagMessage | None:
    import rosbag

    if not topic_name or index < 0:
        return None

    with rosbag.Bag(bag_path) as bag:
        for current_index, (_, msg, stamp) in enumerate(bag.read_messages(topics=[topic_name])):
            if current_index == index:
                return BagMessage(stamp=_message_stamp(msg, stamp.to_sec()), msg=msg)
    return None


def load_nearest_topic_message(bag_path: str, topic_name: str, target_stamp: float) -> BagMessage | None:
    import rosbag

    if not topic_name:
        return None

    previous: BagMessage | None = None
    with rosbag.Bag(bag_path) as bag:
        for _, msg, stamp in bag.read_messages(topics=[topic_name]):
            entry = BagMessage(stamp=_message_stamp(msg, stamp.to_sec()), msg=msg)
            if entry.stamp >= target_stamp:
                if previous is None:
                    return entry
                if abs(previous.stamp - target_stamp) <= abs(entry.stamp - target_stamp):
                    return previous
                return entry
            previous = entry
    return previous


def load_initial_aligned_messages(
    bag_path: str,
    *,
    image_topic: str,
    overlay_image_topic: str,
    pointcloud_topic: str,
) -> Tuple[BagMessage, BagMessage | None, BagMessage]:
    image_entry = load_first_topic_message(bag_path, image_topic)
    if image_entry is None:
        raise RuntimeError(f"Bag did not contain image topic: {image_topic}")

    overlay_entry = load_nearest_topic_message(bag_path, overlay_image_topic, image_entry.stamp)
    cloud_entry = load_nearest_topic_message(bag_path, pointcloud_topic, image_entry.stamp)
    if cloud_entry is None:
        raise RuntimeError(f"Bag did not contain pointcloud topic near first image: {pointcloud_topic}")

    return image_entry, overlay_entry, cloud_entry


def find_nearest_message(entries: Sequence[BagMessage], stamps: Sequence[float], target_stamp: float) -> BagMessage:
    index = bisect.bisect_left(stamps, target_stamp)
    candidates: List[BagMessage] = []
    if index < len(entries):
        candidates.append(entries[index])
    if index > 0:
        candidates.append(entries[index - 1])
    if not candidates:
        raise ValueError("No candidate messages available for nearest-neighbor lookup.")
    return min(candidates, key=lambda item: abs(item.stamp - target_stamp))


def pointcloud2_to_xyz(msg) -> np.ndarray:
    import sensor_msgs.point_cloud2 as pc2

    points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not points:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(points, dtype=np.float64).reshape(-1, 3)


def ros_image_to_array(msg) -> np.ndarray:
    image = np.frombuffer(msg.data, dtype=np.uint8)
    encoding = str(getattr(msg, "encoding", "mono8")).lower()
    if encoding in ("rgb8", "bgr8"):
        array = image.reshape(msg.height, msg.width, 3)
        return array[..., ::-1] if encoding == "bgr8" else array
    if encoding in ("rgba8", "bgra8"):
        array = image.reshape(msg.height, msg.width, 4)
        return array[..., [2, 1, 0, 3]] if encoding == "bgra8" else array
    return image.reshape(msg.height, msg.width)


def semantic_image_to_array(msg) -> np.ndarray:
    return ros_image_to_array(msg)


def project_lidar_points(
    *,
    points_xyz: np.ndarray,
    image_width: int,
    image_height: int,
    camera_matrix: np.ndarray,
    distortion_coeffs: np.ndarray,
    lidar_to_camera: np.ndarray,
    min_depth: float,
) -> ProjectionResult:
    points_xyz = np.asarray(points_xyz, dtype=np.float64)
    if points_xyz.size == 0:
        return ProjectionResult(
            pixel_coordinates=np.empty((0, 2), dtype=np.float64),
            front_mask=np.empty((0,), dtype=bool),
            in_image_mask=np.empty((0,), dtype=bool),
            camera_points=np.empty((0, 3), dtype=np.float64),
            valid_point_indices=np.empty((0,), dtype=np.int64),
        )
    if points_xyz.ndim != 2 or points_xyz.shape[1] != 3:
        raise ValueError(f"Expected point array with shape (N, 3), got {points_xyz.shape}")

    points_h = np.hstack([points_xyz, np.ones((points_xyz.shape[0], 1), dtype=np.float64)])
    camera_points = (lidar_to_camera @ points_h.T).T[:, :3]
    front_mask = camera_points[:, 2] > min_depth
    front_points = camera_points[front_mask]
    if front_points.size == 0:
        return ProjectionResult(
            pixel_coordinates=np.empty((0, 2), dtype=np.float64),
            front_mask=front_mask,
            in_image_mask=np.empty((0,), dtype=bool),
            camera_points=camera_points,
            valid_point_indices=np.empty((0,), dtype=np.int64),
        )

    image_points, _ = cv2.projectPoints(
        front_points,
        np.zeros(3, dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        camera_matrix,
        distortion_coeffs,
    )
    pixel_coordinates = image_points.reshape(-1, 2)
    in_image_mask = (
        (pixel_coordinates[:, 0] >= 0.0)
        & (pixel_coordinates[:, 0] < image_width)
        & (pixel_coordinates[:, 1] >= 0.0)
        & (pixel_coordinates[:, 1] < image_height)
    )
    front_indices = np.flatnonzero(front_mask)
    return ProjectionResult(
        pixel_coordinates=pixel_coordinates,
        front_mask=front_mask,
        in_image_mask=in_image_mask,
        camera_points=camera_points,
        valid_point_indices=front_indices[in_image_mask],
    )
