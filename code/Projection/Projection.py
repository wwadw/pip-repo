#!/usr/bin/env python3
"""
LiDAR-camera projection test tool.

This tool defaults to an interactive Open3D + OpenCV calibration viewer and can
also run in a headless batch mode with `--batch`.

Configuration sources:
1. Edit DEFAULT_CONFIG below for convenient file-local defaults.
2. Provide `--yaml` to read camera intrinsics/extrinsics and topic defaults.
3. Use CLI flags to override the final configuration.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from rerun_projection.projection_core import (
    BagMessage,
    ProjectionResult,
    find_nearest_message as _core_find_nearest_message,
    load_bag_messages as _core_load_bag_messages,
    pointcloud2_to_xyz as _core_pointcloud2_to_xyz,
    project_lidar_points as _core_project_lidar_points,
    semantic_image_to_array as _core_semantic_image_to_array,
)


DEFAULT_COLOR_MAP_BGR: Dict[int, Tuple[int, int, int]] = {
    0: (0, 0, 0),
    1: (0, 255, 0),
    2: (0, 0, 255),
    3: (0, 255, 255),
    4: (255, 0, 0),
    5: (255, 0, 255),
    255: (128, 128, 128),
}


DEFAULT_CONFIG = {
    "bag_file": "",
    "yaml_path": "",
    "image_topic": "/usb_cam/image_semantic_id",
    "pointcloud_topic": "/mfla/frame_cloud",
    "output_dir": "/home/ww/elevation_ws/logs/projection_tool_output",
    "image_width": 1280,
    "image_height": 720,
    "camera_matrix": np.eye(3, dtype=np.float64),
    "distortion_coeffs": np.zeros(5, dtype=np.float64),
    "lidar_to_camera": np.eye(4, dtype=np.float64),
    "color_map_bgr": dict(DEFAULT_COLOR_MAP_BGR),
    "min_depth": 0.05,
    "rotation_step_deg": 0.5,
    "translation_step": 0.01,
    "overlay_max_points": 6000,
    "batch_sample_count": 6,
    "playback_sleep_sec": 0.03,
}


@dataclass
class ToolConfig:
    bag_file: str
    yaml_path: str
    image_topic: str
    pointcloud_topic: str
    camera_matrix: np.ndarray
    distortion_coeffs: np.ndarray
    lidar_to_camera: np.ndarray
    image_width: int
    image_height: int
    color_map_bgr: Dict[int, Tuple[int, int, int]] = field(default_factory=lambda: dict(DEFAULT_COLOR_MAP_BGR))
    output_dir: str = DEFAULT_CONFIG["output_dir"]
    min_depth: float = float(DEFAULT_CONFIG["min_depth"])
    rotation_step_deg: float = float(DEFAULT_CONFIG["rotation_step_deg"])
    translation_step: float = float(DEFAULT_CONFIG["translation_step"])
    overlay_max_points: int = int(DEFAULT_CONFIG["overlay_max_points"])
    batch_sample_count: int = int(DEFAULT_CONFIG["batch_sample_count"])
    playback_sleep_sec: float = float(DEFAULT_CONFIG["playback_sleep_sec"])

    def __post_init__(self) -> None:
        self.camera_matrix = np.asarray(self.camera_matrix, dtype=np.float64).reshape(3, 3)
        self.distortion_coeffs = np.asarray(self.distortion_coeffs, dtype=np.float64).reshape(-1)
        self.lidar_to_camera = np.asarray(self.lidar_to_camera, dtype=np.float64).reshape(4, 4)
        self.image_width = int(self.image_width)
        self.image_height = int(self.image_height)
        self.min_depth = float(self.min_depth)
        self.rotation_step_deg = float(self.rotation_step_deg)
        self.translation_step = float(self.translation_step)
        self.overlay_max_points = int(self.overlay_max_points)
        self.batch_sample_count = int(self.batch_sample_count)
        self.playback_sleep_sec = float(self.playback_sleep_sec)


def _rgb_list_to_bgr_tuple(value: Sequence[int]) -> Tuple[int, int, int]:
    if len(value) != 3:
        raise ValueError(f"Expected 3 color components, got {value}")
    return int(value[2]), int(value[1]), int(value[0])


def _read_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return data or {}


def _extract_pointcloud_topic(data: dict, fallback: str) -> str:
    for key in ("pointcloud_topic", "raw_pointcloud_topic", "compensated_pointcloud_topic"):
        value = data.get(key)
        if value:
            return str(value)
    input_sources = data.get("input_sources", {})
    semantic_input = input_sources.get("semantic_pointcloud", {})
    if semantic_input.get("topic"):
        return str(semantic_input["topic"])
    return fallback


def _extract_color_map(data: dict, fallback: Dict[int, Tuple[int, int, int]]) -> Dict[int, Tuple[int, int, int]]:
    semantic_colors = data.get("semantic_colors")
    if not semantic_colors:
        return dict(fallback)
    parsed = dict(fallback)
    for label, rgb in semantic_colors.items():
        parsed[int(label)] = _rgb_list_to_bgr_tuple(rgb)
    return parsed


def load_tool_config(yaml_path: Optional[str] = None, **overrides) -> ToolConfig:
    config_values = {
        "bag_file": DEFAULT_CONFIG["bag_file"],
        "yaml_path": yaml_path or "",
        "image_topic": DEFAULT_CONFIG["image_topic"],
        "pointcloud_topic": DEFAULT_CONFIG["pointcloud_topic"],
        "camera_matrix": np.array(DEFAULT_CONFIG["camera_matrix"], copy=True),
        "distortion_coeffs": np.array(DEFAULT_CONFIG["distortion_coeffs"], copy=True),
        "lidar_to_camera": np.array(DEFAULT_CONFIG["lidar_to_camera"], copy=True),
        "image_width": DEFAULT_CONFIG["image_width"],
        "image_height": DEFAULT_CONFIG["image_height"],
        "color_map_bgr": dict(DEFAULT_CONFIG["color_map_bgr"]),
        "output_dir": DEFAULT_CONFIG["output_dir"],
        "min_depth": DEFAULT_CONFIG["min_depth"],
        "rotation_step_deg": DEFAULT_CONFIG["rotation_step_deg"],
        "translation_step": DEFAULT_CONFIG["translation_step"],
        "overlay_max_points": DEFAULT_CONFIG["overlay_max_points"],
        "batch_sample_count": DEFAULT_CONFIG["batch_sample_count"],
        "playback_sleep_sec": DEFAULT_CONFIG["playback_sleep_sec"],
    }

    if yaml_path:
        data = _read_yaml(yaml_path)
        camera = data.get("semantic_camera", {})
        intrinsics = camera.get("camera_matrix", {})
        if intrinsics:
            config_values["camera_matrix"] = np.array(
                [
                    [float(intrinsics.get("fx", 0.0)), 0.0, float(intrinsics.get("cx", 0.0))],
                    [0.0, float(intrinsics.get("fy", 0.0)), float(intrinsics.get("cy", 0.0))],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
        if camera.get("distortion_coeffs") is not None:
            config_values["distortion_coeffs"] = np.asarray(camera["distortion_coeffs"], dtype=np.float64)
        if camera.get("lidar_to_camera_transform") is not None:
            config_values["lidar_to_camera"] = np.asarray(camera["lidar_to_camera_transform"], dtype=np.float64)
        if camera.get("image_width") is not None:
            config_values["image_width"] = int(camera["image_width"])
        if camera.get("image_height") is not None:
            config_values["image_height"] = int(camera["image_height"])
        if data.get("semantic_image_topic"):
            config_values["image_topic"] = str(data["semantic_image_topic"])
        config_values["pointcloud_topic"] = _extract_pointcloud_topic(data, config_values["pointcloud_topic"])
        config_values["color_map_bgr"] = _extract_color_map(data, config_values["color_map_bgr"])

    for key, value in overrides.items():
        if value is None:
            continue
        if key in {"camera_matrix", "distortion_coeffs", "lidar_to_camera"}:
            config_values[key] = np.asarray(value, dtype=np.float64)
        elif key == "color_map_bgr":
            config_values[key] = dict(value)
        else:
            config_values[key] = value

    return ToolConfig(**config_values)


def load_bag_messages(bag_path: str, image_topic: str, pointcloud_topic: str) -> Tuple[List[BagMessage], List[BagMessage]]:
    return _core_load_bag_messages(bag_path, image_topic, pointcloud_topic)


def find_nearest_message(entries: Sequence[BagMessage], stamps: Sequence[float], target_stamp: float) -> BagMessage:
    return _core_find_nearest_message(entries, stamps, target_stamp)


def pointcloud2_to_xyz(msg) -> np.ndarray:
    return _core_pointcloud2_to_xyz(msg)


def semantic_image_to_array(msg) -> np.ndarray:
    return _core_semantic_image_to_array(msg)


def project_lidar_points(
    points_xyz: np.ndarray,
    config: ToolConfig,
    transform: Optional[np.ndarray] = None,
    min_depth: Optional[float] = None,
) -> ProjectionResult:
    effective_transform = config.lidar_to_camera if transform is None else np.asarray(transform, dtype=np.float64).reshape(4, 4)
    effective_min_depth = config.min_depth if min_depth is None else float(min_depth)
    return _core_project_lidar_points(
        points_xyz=points_xyz,
        image_width=config.image_width,
        image_height=config.image_height,
        camera_matrix=config.camera_matrix,
        distortion_coeffs=config.distortion_coeffs,
        lidar_to_camera=effective_transform,
        min_depth=effective_min_depth,
    )


def colorize_semantic_image(semantic_image: np.ndarray, color_map_bgr: Dict[int, Tuple[int, int, int]]) -> np.ndarray:
    color = np.zeros((semantic_image.shape[0], semantic_image.shape[1], 3), dtype=np.uint8)
    for label, bgr in color_map_bgr.items():
        color[semantic_image == label] = bgr
    return color


def sample_projected_labels(semantic_image: np.ndarray, projection: ProjectionResult) -> np.ndarray:
    if projection.pixel_coordinates.size == 0 or not np.any(projection.in_image_mask):
        return np.empty((0,), dtype=np.uint8)
    valid_pixels = projection.pixel_coordinates[projection.in_image_mask]
    u = valid_pixels[:, 0].astype(np.int32)
    v = valid_pixels[:, 1].astype(np.int32)
    return semantic_image[v, u]


def build_overlay_image(
    semantic_image: np.ndarray,
    projection: ProjectionResult,
    color_map_bgr: Dict[int, Tuple[int, int, int]],
    info_lines: Sequence[str],
    overlay_max_points: int,
) -> np.ndarray:
    overlay = colorize_semantic_image(semantic_image, color_map_bgr)
    if projection.pixel_coordinates.size > 0 and np.any(projection.in_image_mask):
        valid_pixels = projection.pixel_coordinates[projection.in_image_mask]
        step = max(1, len(valid_pixels) // max(1, overlay_max_points))
        for x, y in valid_pixels[::step]:
            cv2.circle(overlay, (int(x), int(y)), 1, (255, 255, 255), -1)

    for line_index, text in enumerate(info_lines):
        cv2.putText(
            overlay,
            text,
            (10, 30 + line_index * 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return overlay


def _current_transform(base_transform: np.ndarray, translation: np.ndarray, rpy_deg: np.ndarray) -> np.ndarray:
    adjustment = np.eye(4, dtype=np.float64)
    adjustment[:3, :3] = R.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    adjustment[:3, 3] = translation
    return adjustment @ base_transform


def _label_histogram(labels: Iterable[int]) -> Dict[int, int]:
    return {int(label): int(count) for label, count in sorted(Counter(labels).items())}


class InteractiveProjectionTool:
    def __init__(self, config: ToolConfig) -> None:
        try:
            import open3d as o3d
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Interactive mode requires `open3d`. Install it or run the tool with `--batch`."
            ) from exc

        self.o3d = o3d
        self.config = config
        self.base_transform = np.array(config.lidar_to_camera, copy=True)
        self.translation_adjustment = np.zeros(3, dtype=np.float64)
        self.rpy_adjustment_deg = np.zeros(3, dtype=np.float64)
        self.current_transform = np.array(self.base_transform, copy=True)

        self.images, self.clouds = load_bag_messages(config.bag_file, config.image_topic, config.pointcloud_topic)
        if not self.images or not self.clouds:
            raise RuntimeError(
                f"Bag did not contain both topics: image={config.image_topic}, pointcloud={config.pointcloud_topic}"
            )
        self.cloud_stamps = [entry.stamp for entry in self.clouds]
        self.current_index = 0
        self.is_playing = False
        self.is_running = True

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="Projection Tool 3D View", width=960, height=720)
        render_option = self.vis.get_render_option()
        render_option.background_color = np.asarray([0.0, 0.0, 0.0])
        render_option.point_size = 3.0

        self.axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0.0, 0.0, 0.0])
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.axis)
        self.vis.add_geometry(self.pcd)
        self._register_key_callbacks()
        self._print_controls()
        self.update_view()

        self.image_thread = threading.Thread(target=self._image_thread_loop, daemon=True)
        self.image_thread.start()

    def _print_controls(self) -> None:
        print("Interactive projection controls:")
        print("  Space: play/pause")
        print("  A / D: previous / next frame")
        print("  W / S: pitch + / -")
        print("  R / F: yaw + / -")
        print("  T / G: roll + / -")
        print("  Arrow keys: translate X / Y")
        print("  Z / X: translate Z + / -")
        print("  P: print current lidar_to_camera transform")
        print("  C: reset adjustments")
        print("  Q or Esc: quit")

    def _register_key_callbacks(self) -> None:
        self.vis.register_key_callback(32, self._toggle_play)
        self.vis.register_key_callback(ord("A"), self._prev_frame)
        self.vis.register_key_callback(ord("D"), self._next_frame)
        self.vis.register_key_callback(ord("W"), lambda vis: self._adjust_rpy(1, self.config.rotation_step_deg))
        self.vis.register_key_callback(ord("S"), lambda vis: self._adjust_rpy(1, -self.config.rotation_step_deg))
        self.vis.register_key_callback(ord("R"), lambda vis: self._adjust_rpy(2, self.config.rotation_step_deg))
        self.vis.register_key_callback(ord("F"), lambda vis: self._adjust_rpy(2, -self.config.rotation_step_deg))
        self.vis.register_key_callback(ord("T"), lambda vis: self._adjust_rpy(0, self.config.rotation_step_deg))
        self.vis.register_key_callback(ord("G"), lambda vis: self._adjust_rpy(0, -self.config.rotation_step_deg))
        self.vis.register_key_callback(262, lambda vis: self._adjust_translation(0, self.config.translation_step))
        self.vis.register_key_callback(263, lambda vis: self._adjust_translation(0, -self.config.translation_step))
        self.vis.register_key_callback(265, lambda vis: self._adjust_translation(1, self.config.translation_step))
        self.vis.register_key_callback(264, lambda vis: self._adjust_translation(1, -self.config.translation_step))
        self.vis.register_key_callback(ord("Z"), lambda vis: self._adjust_translation(2, self.config.translation_step))
        self.vis.register_key_callback(ord("X"), lambda vis: self._adjust_translation(2, -self.config.translation_step))
        self.vis.register_key_callback(ord("P"), self._print_transform)
        self.vis.register_key_callback(ord("C"), self._reset_adjustments)
        self.vis.register_key_callback(ord("Q"), self._close)

    def _toggle_play(self, vis) -> bool:
        self.is_playing = not self.is_playing
        return False

    def _close(self, vis) -> bool:
        self.is_running = False
        self.vis.close()
        return False

    def _next_frame(self, vis) -> bool:
        self.current_index = (self.current_index + 1) % len(self.images)
        self.update_view()
        return False

    def _prev_frame(self, vis) -> bool:
        self.current_index = (self.current_index - 1) % len(self.images)
        self.update_view()
        return False

    def _adjust_rpy(self, axis: int, amount: float) -> bool:
        self.rpy_adjustment_deg[axis] += amount
        print(f"Rotation adjustment [roll, pitch, yaw] deg = {self.rpy_adjustment_deg}")
        self._refresh_transform()
        return False

    def _adjust_translation(self, axis: int, amount: float) -> bool:
        self.translation_adjustment[axis] += amount
        print(f"Translation adjustment [x, y, z] m = {self.translation_adjustment}")
        self._refresh_transform()
        return False

    def _reset_adjustments(self, vis) -> bool:
        self.translation_adjustment[:] = 0.0
        self.rpy_adjustment_deg[:] = 0.0
        print("Adjustments reset to zero.")
        self._refresh_transform()
        return False

    def _print_transform(self, vis) -> bool:
        print("\nCurrent lidar_to_camera transform:")
        print(np.array2string(self.current_transform, precision=6, suppress_small=False))
        print(f"Translation adjustment: {self.translation_adjustment}")
        print(f"Rotation adjustment deg: {self.rpy_adjustment_deg}\n")
        return False

    def _refresh_transform(self) -> None:
        self.current_transform = _current_transform(
            self.base_transform,
            self.translation_adjustment,
            self.rpy_adjustment_deg,
        )
        self.update_view()

    def _current_entries(self) -> Tuple[BagMessage, BagMessage]:
        image_entry = self.images[self.current_index]
        cloud_entry = find_nearest_message(self.clouds, self.cloud_stamps, image_entry.stamp)
        return image_entry, cloud_entry

    def _build_visualization_data(self) -> Tuple[np.ndarray, np.ndarray, ProjectionResult, BagMessage, BagMessage]:
        image_entry, cloud_entry = self._current_entries()
        semantic_image = semantic_image_to_array(image_entry.msg)
        points = pointcloud2_to_xyz(cloud_entry.msg)
        projection = project_lidar_points(points, self.config, transform=self.current_transform)
        return semantic_image, points, projection, image_entry, cloud_entry

    def _update_open3d_cloud(self) -> None:
        semantic_image, points, projection, image_entry, cloud_entry = self._build_visualization_data()
        colors = np.full((points.shape[0], 3), 0.5, dtype=np.float64)
        labels = sample_projected_labels(semantic_image, projection)
        for label, bgr in self.config.color_map_bgr.items():
            if labels.size == 0:
                continue
            label_mask = labels == label
            if not np.any(label_mask):
                continue
            rgb = np.array([bgr[2], bgr[1], bgr[0]], dtype=np.float64) / 255.0
            colors[projection.valid_point_indices[label_mask]] = rgb

        self.pcd.points = self.o3d.utility.Vector3dVector(points)
        self.pcd.colors = self.o3d.utility.Vector3dVector(colors)

    def _current_overlay(self) -> np.ndarray:
        semantic_image, points, projection, image_entry, cloud_entry = self._build_visualization_data()
        labels = sample_projected_labels(semantic_image, projection)
        info_lines = [
            f"Frame: {self.current_index + 1}/{len(self.images)}",
            f"Image stamp: {image_entry.stamp:.3f}",
            f"Cloud stamp: {cloud_entry.stamp:.3f}",
            f"dt(ms): {abs(image_entry.stamp - cloud_entry.stamp) * 1000.0:.2f}",
            f"Points: {len(points)}  front: {int(projection.front_mask.sum())}",
            f"In image: {int(projection.in_image_mask.sum())}",
            f"Labels: {_label_histogram(labels)}",
            f"T xyz: {np.round(self.translation_adjustment, 4).tolist()}",
            f"R rpy(deg): {np.round(self.rpy_adjustment_deg, 3).tolist()}",
        ]
        return build_overlay_image(
            semantic_image,
            projection,
            self.config.color_map_bgr,
            info_lines,
            self.config.overlay_max_points,
        )

    def _image_thread_loop(self) -> None:
        cv2.namedWindow("Projection Tool 2D View", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Projection Tool 2D View", 960, 540)
        while self.is_running:
            try:
                overlay = self._current_overlay()
                cv2.imshow("Projection Tool 2D View", overlay)
            except Exception as exc:  # pragma: no cover - defensive UI loop logging
                print(f"Overlay update failed: {exc}")
            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                self._close(self.vis)
                break
        cv2.destroyAllWindows()

    def update_view(self) -> None:
        self._update_open3d_cloud()
        self.vis.update_geometry(self.pcd)

    def run(self) -> None:
        while self.is_running:
            self.vis.poll_events()
            if self.is_playing:
                self.current_index = (self.current_index + 1) % len(self.images)
                self.update_view()
            self.vis.update_renderer()
            time.sleep(self.config.playback_sleep_sec)
        self.vis.destroy_window()
        self.is_running = False


def run_batch_mode(config: ToolConfig) -> Path:
    output_dir = Path(config.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    images, clouds = load_bag_messages(config.bag_file, config.image_topic, config.pointcloud_topic)
    if not images or not clouds:
        raise RuntimeError(
            f"Bag did not contain both topics: image={config.image_topic}, pointcloud={config.pointcloud_topic}"
        )

    image_stamps = [entry.stamp for entry in images]
    stats: List[Tuple[float, int, int, int]] = []
    label_counter: Counter = Counter()

    sample_count = max(1, min(config.batch_sample_count, len(clouds)))
    sample_indices = set(np.linspace(0, len(clouds) - 1, sample_count, dtype=int).tolist())

    for cloud_index, cloud_entry in enumerate(clouds):
        image_entry = find_nearest_message(images, image_stamps, cloud_entry.stamp)
        semantic_image = semantic_image_to_array(image_entry.msg)
        points = pointcloud2_to_xyz(cloud_entry.msg)
        projection = project_lidar_points(points, config)
        labels = sample_projected_labels(semantic_image, projection)
        label_counter.update(int(label) for label in labels.tolist())
        stats.append(
            (
                abs(cloud_entry.stamp - image_entry.stamp),
                len(points),
                int(projection.front_mask.sum()),
                int(projection.in_image_mask.sum()),
            )
        )

        if cloud_index in sample_indices:
            info_lines = [
                f"Cloud frame: {cloud_index + 1}/{len(clouds)}",
                f"Image stamp: {image_entry.stamp:.3f}",
                f"Cloud stamp: {cloud_entry.stamp:.3f}",
                f"dt(ms): {abs(image_entry.stamp - cloud_entry.stamp) * 1000.0:.2f}",
                f"Points: {len(points)}  front: {int(projection.front_mask.sum())}",
                f"In image: {int(projection.in_image_mask.sum())}",
                f"Labels: {_label_histogram(labels)}",
            ]
            overlay = build_overlay_image(
                semantic_image,
                projection,
                config.color_map_bgr,
                info_lines,
                config.overlay_max_points,
            )
            overlay_path = output_dir / f"overlay_cloud_{cloud_index:04d}.png"
            cv2.imwrite(str(overlay_path), overlay)

    stats_array = np.asarray(stats, dtype=np.float64)
    dt = stats_array[:, 0]
    total_points = stats_array[:, 1]
    front_points = stats_array[:, 2]
    in_image_points = stats_array[:, 3]
    summary_lines = [
        f"bag_file: {config.bag_file}",
        f"yaml_path: {config.yaml_path}",
        f"image_topic: {config.image_topic}",
        f"pointcloud_topic: {config.pointcloud_topic}",
        f"cloud_frames: {len(clouds)}",
        f"image_frames: {len(images)}",
        f"dt_ms_mean: {dt.mean() * 1000.0:.6f}",
        f"dt_ms_p95: {np.percentile(dt, 95.0) * 1000.0:.6f}",
        f"dt_ms_max: {dt.max() * 1000.0:.6f}",
        f"points_mean: {total_points.mean():.6f}",
        f"front_ratio_mean: {(front_points / np.maximum(total_points, 1.0)).mean():.6f}",
        f"in_image_ratio_vs_front_mean: {(in_image_points / np.maximum(front_points, 1.0)).mean():.6f}",
        f"in_image_ratio_vs_total_mean: {(in_image_points / np.maximum(total_points, 1.0)).mean():.6f}",
        f"projected_label_hist_total: {dict(sorted(label_counter.items()))}",
    ]
    summary_path = output_dir / "batch_summary.txt"
    summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    print("\n".join(summary_lines))
    print(f"Batch summary written to: {summary_path}")
    return summary_path


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="LiDAR-camera projection test tool")
    parser.add_argument("--bag", dest="bag_file", help="Input rosbag path")
    parser.add_argument("--yaml", dest="yaml_path", help="YAML file with semantic_camera parameters")
    parser.add_argument("--image-topic", help="Semantic image topic")
    parser.add_argument("--cloud-topic", dest="pointcloud_topic", help="Point cloud topic")
    parser.add_argument("--output-dir", help="Output directory for batch results")
    parser.add_argument("--min-depth", type=float, help="Minimum camera depth to keep projected points")
    parser.add_argument("--batch", action="store_true", help="Run headless batch mode instead of interactive viewer")
    parser.add_argument("--batch-sample-count", type=int, help="Number of overlay images to save in batch mode")
    parser.add_argument("--rotation-step-deg", type=float, help="Interactive rotation adjustment step in degrees")
    parser.add_argument("--translation-step", type=float, help="Interactive translation adjustment step in meters")
    return parser


def normalize_cli_args(argv: Optional[Sequence[str]]) -> List[str]:
    raw_args = list(sys.argv[1:] if argv is None else argv)
    return [arg for arg in raw_args if str(arg).strip()]


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(normalize_cli_args(argv))

    yaml_path = args.yaml_path or DEFAULT_CONFIG["yaml_path"] or None
    config = load_tool_config(
        yaml_path=yaml_path,
        bag_file=args.bag_file or DEFAULT_CONFIG["bag_file"],
        image_topic=args.image_topic,
        pointcloud_topic=args.pointcloud_topic,
        output_dir=args.output_dir,
        min_depth=args.min_depth,
        batch_sample_count=args.batch_sample_count,
        rotation_step_deg=args.rotation_step_deg,
        translation_step=args.translation_step,
    )

    if not config.bag_file:
        parser.error("A bag file is required. Set DEFAULT_CONFIG['bag_file'] or pass --bag.")

    if args.batch:
        run_batch_mode(config)
        return 0

    try:
        tool = InteractiveProjectionTool(config)
        tool.run()
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
