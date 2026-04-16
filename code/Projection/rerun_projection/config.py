from dataclasses import dataclass
from typing import Any, Dict, Optional

import numpy as np
import yaml


DEFAULT_CONFIG = {
    "bag_file": "/home/ww/bags/b2biaopin/louti/2026-04-14-20-18-10_semantic.bag",
    "yaml_path": "/home/ww/elevation_ws/src/elevation_mapping/elevation_mapping_demos/config/robots/minimal_semantic_robot.yaml",
    "image_topic": "/usb_cam/image_semantic_id",
    "overlay_image_topic": "/usb_cam/image_raw",
    "pointcloud_topic": "/mfla/frame_cloud",
    "image_width": 1280,
    "image_height": 720,
    "camera_matrix": np.eye(3, dtype=np.float64),
    "distortion_coeffs": np.zeros(5, dtype=np.float64),
    "lidar_to_camera": np.eye(4, dtype=np.float64),
    "min_depth": 0.05,
}


@dataclass
class RuntimeConfig:
    bag_file: str
    yaml_path: str
    image_topic: str
    overlay_image_topic: str
    pointcloud_topic: str
    image_width: int
    image_height: int
    camera_matrix: np.ndarray
    distortion_coeffs: np.ndarray
    lidar_to_camera: np.ndarray
    min_depth: float

    def __post_init__(self) -> None:
        self.image_width = int(self.image_width)
        self.image_height = int(self.image_height)
        self.camera_matrix = np.asarray(self.camera_matrix, dtype=np.float64).reshape(3, 3)
        self.distortion_coeffs = np.asarray(self.distortion_coeffs, dtype=np.float64).reshape(-1)
        self.lidar_to_camera = np.asarray(self.lidar_to_camera, dtype=np.float64).reshape(4, 4)
        self.min_depth = float(self.min_depth)


def load_yaml_data(yaml_path: str) -> Dict[str, Any]:
    if not yaml_path:
        return {}
    with open(yaml_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return data or {}


def _extract_pointcloud_topic(data: Dict[str, Any], fallback: str) -> str:
    for key in ("pointcloud_topic", "raw_pointcloud_topic", "compensated_pointcloud_topic"):
        value = data.get(key)
        if value:
            return str(value)
    input_sources = data.get("input_sources", {})
    semantic_input = input_sources.get("semantic_pointcloud", {})
    if semantic_input.get("topic"):
        return str(semantic_input["topic"])
    return fallback


def resolve_runtime_config(*, yaml_data: Optional[Dict[str, Any]] = None, cli_overrides: Optional[Dict[str, Any]] = None) -> RuntimeConfig:
    data = dict(yaml_data or {})
    overrides = dict(cli_overrides or {})
    camera = data.get("semantic_camera", {})
    intrinsics = camera.get("camera_matrix", {})

    config_values = {
        "bag_file": overrides.get("bag_file", DEFAULT_CONFIG["bag_file"]),
        "yaml_path": overrides.get("yaml_path", DEFAULT_CONFIG["yaml_path"]),
        "image_topic": data.get("semantic_image_topic", DEFAULT_CONFIG["image_topic"]),
        "overlay_image_topic": data.get("overlay_image_topic", DEFAULT_CONFIG["overlay_image_topic"]),
        "pointcloud_topic": _extract_pointcloud_topic(data, DEFAULT_CONFIG["pointcloud_topic"]),
        "image_width": camera.get("image_width", data.get("image_width", DEFAULT_CONFIG["image_width"])),
        "image_height": camera.get("image_height", data.get("image_height", DEFAULT_CONFIG["image_height"])),
        "camera_matrix": np.array(
            [
                [float(intrinsics.get("fx", DEFAULT_CONFIG["camera_matrix"][0, 0])), 0.0, float(intrinsics.get("cx", DEFAULT_CONFIG["camera_matrix"][0, 2]))],
                [0.0, float(intrinsics.get("fy", DEFAULT_CONFIG["camera_matrix"][1, 1])), float(intrinsics.get("cy", DEFAULT_CONFIG["camera_matrix"][1, 2]))],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        "distortion_coeffs": camera.get("distortion_coeffs", DEFAULT_CONFIG["distortion_coeffs"]),
        "lidar_to_camera": camera.get("lidar_to_camera_transform", DEFAULT_CONFIG["lidar_to_camera"]),
        "min_depth": overrides.get("min_depth", DEFAULT_CONFIG["min_depth"]),
    }

    for key, value in overrides.items():
        if value is None:
            continue
        config_values[key] = value

    return RuntimeConfig(**config_values)
