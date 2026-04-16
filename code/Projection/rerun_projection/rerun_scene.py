import colorsys
from typing import Dict, Iterable, List, Optional

import cv2
import numpy as np


SEMANTIC_CAMERA_PATH = "world/ego_vehicle/semantic_camera"
OVERLAY_CAMERA_PATH = "world/ego_vehicle/overlay_camera"


def build_locked_pair_payloads(locked_pairs: Iterable[object]) -> Dict[str, List[object]]:
    pairs = list(locked_pairs)
    return {
        "keypoint_ids": [pair.pair_id for pair in pairs],
        "positions2d": [pair.projected_pixel.tolist() for pair in pairs],
        "positions3d": [pair.point_xyz.tolist() for pair in pairs],
        "colors": [list(pair.color_rgb) for pair in pairs],
        "labels": [f"P{pair.pair_id}" for pair in pairs],
    }


class RerunSceneLogger:
    def __init__(self, recording: Optional[object] = None, view_kind: str = "both") -> None:
        self.recording = recording
        self.view_kind = view_kind

    def log_current_state(self, frame, config, frame_index: int | None = None) -> None:
        import rerun as rr

        self._set_frame_time(rr, frame_index)
        point_colors = _pointcloud_colors(frame.points_xyz)
        if self.view_kind in ("3d", "both"):
            rr.log(
                "world/ego_vehicle/lidar",
                rr.Points3D(
                    frame.points_xyz,
                    colors=point_colors,
                    radii=[0.045] * len(frame.points_xyz),
                    keypoint_ids=list(range(len(frame.points_xyz))),
                ),
                recording=self.recording,
            )
        self._log_camera_view(rr, SEMANTIC_CAMERA_PATH, frame.semantic_image, config, [96, 208, 255], lossless=True)
        self._log_camera_view(rr, OVERLAY_CAMERA_PATH, frame.overlay_image, config, [255, 178, 92], lossless=False)

        valid_pixels = frame.projection.pixel_coordinates[frame.projection.in_image_mask]
        valid_indices = frame.projection.valid_point_indices.tolist()
        valid_colors = [point_colors[index] for index in valid_indices] if valid_indices else []

        if self.view_kind in ("2d", "both"):
            for camera_path in (SEMANTIC_CAMERA_PATH, OVERLAY_CAMERA_PATH):
                rr.log(
                    f"{camera_path}/projected_points",
                    rr.Points2D(
                        valid_pixels.tolist() if len(valid_pixels) else [],
                        colors=valid_colors if valid_colors else None,
                        radii=[1.6] * len(valid_pixels),
                        keypoint_ids=valid_indices if valid_indices else [],
                    ),
                    recording=self.recording,
                )

        if frame_index is not None:
            self.clear_interactions(frame_index)

    def clear_interactions(self, frame_index: int) -> None:
        import rerun as rr

        self._set_frame_time(rr, frame_index)
        for path in (
            "selection/lidar",
            "selection/semantic_camera",
            "selection/overlay_camera",
            "pairs/lidar",
            "pairs/semantic_camera",
            "pairs/overlay_camera",
        ):
            rr.log(path, rr.Clear(recursive=True), recording=self.recording)

    def log_selection(self, selection, frame_index: int | None = None) -> None:
        if selection is None:
            return
        import rerun as rr

        self._set_frame_time(rr, frame_index if frame_index is not None else selection.frame_index)
        if self.view_kind in ("3d", "both"):
            rr.log(
                "selection/lidar/current_match",
                rr.Points3D([selection.point_xyz.tolist()], colors=[[255, 255, 255]], radii=[0.09]),
                recording=self.recording,
            )
        if self.view_kind in ("2d", "both"):
            for camera_name in ("semantic_camera", "overlay_camera"):
                rr.log(
                    f"selection/{camera_name}/current_match",
                    rr.Points2D([selection.matched_pixel.tolist()], colors=[[255, 255, 255]], radii=[4.0]),
                    recording=self.recording,
                )

        if self.view_kind in ("2d", "both") and selection.clicked_pixel is not None:
            for camera_name in ("semantic_camera", "overlay_camera"):
                rr.log(
                    f"selection/{camera_name}/current_click",
                    rr.Points2D([selection.clicked_pixel.tolist()], colors=[[255, 122, 107]], radii=[3.0]),
                    recording=self.recording,
                )

    def log_locked_pairs(self, locked_pairs: Iterable[object], frame_index: int | None = None) -> None:
        import rerun as rr

        pairs = list(locked_pairs)
        if not pairs:
            return
        payload = build_locked_pair_payloads(pairs)
        resolved_frame = frame_index if frame_index is not None else pairs[0].frame_index
        self._set_frame_time(rr, resolved_frame)
        if self.view_kind in ("2d", "both"):
            for camera_name in ("semantic_camera", "overlay_camera"):
                rr.log(
                    f"pairs/{camera_name}/locked",
                    rr.Points2D(
                        payload["positions2d"],
                        colors=payload["colors"],
                        labels=payload["labels"],
                        radii=[3.2] * len(payload["positions2d"]),
                        keypoint_ids=payload["keypoint_ids"],
                    ),
                    recording=self.recording,
                )
        if self.view_kind in ("3d", "both"):
            rr.log(
                "pairs/lidar/locked",
                rr.Points3D(
                    payload["positions3d"],
                    colors=payload["colors"],
                    labels=payload["labels"],
                    radii=[0.08] * len(payload["positions3d"]),
                    keypoint_ids=payload["keypoint_ids"],
                ),
                recording=self.recording,
            )

    def _log_camera_view(self, rr, path: str, image: np.ndarray, config, color: list[int], *, lossless: bool) -> None:
        rr.log(
            path,
            rr.Transform3D(
                translation=config.lidar_to_camera[:3, 3],
                mat3x3=config.lidar_to_camera[:3, :3],
                relation=rr.TransformRelation.ChildFromParent,
            ),
            recording=self.recording,
        )
        rr.log(path, rr.TransformAxes3D(0.28), recording=self.recording)
        rr.log(
            path,
            rr.Pinhole.from_fields(
                image_from_camera=config.camera_matrix,
                resolution=[config.image_width, config.image_height],
                image_plane_distance=5.0,
                line_width=1.2,
                color=color,
            ),
            recording=self.recording,
        )
        rr.log(path, _encoded_image(rr, image, lossless=lossless), recording=self.recording)

    def _set_frame_time(self, rr, frame_index: int | None) -> None:
        if frame_index is None:
            return
        rr.set_time("frame", sequence=int(frame_index), recording=self.recording)


def _encoded_image(rr, image: np.ndarray, *, lossless: bool):
    if image.ndim == 2 or lossless:
        ok, encoded = cv2.imencode(".png", image)
        if not ok:
            raise RuntimeError("Failed to encode semantic image to PNG.")
        return rr.EncodedImage(contents=encoded.tobytes(), media_type="image/png")

    rgb = image[..., :3]
    bgr = rgb[..., ::-1]
    ok, encoded = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if not ok:
        raise RuntimeError("Failed to encode overlay image to JPEG.")
    return rr.EncodedImage(contents=encoded.tobytes(), media_type="image/jpeg")

def _pointcloud_colors(points_xyz: np.ndarray) -> list[list[int]]:
    if len(points_xyz) == 0:
        return []
    ranges = np.linalg.norm(points_xyz[:, :2], axis=1)
    span = float(np.ptp(ranges))
    normalized = np.zeros_like(ranges) if span <= 1e-6 else (ranges - ranges.min()) / span
    colors: list[list[int]] = []
    for value in normalized.tolist():
        red, green, blue = colorsys.hsv_to_rgb(0.02 + 0.63 * value, 0.9, 1.0)
        colors.append([int(red * 255), int(green * 255), int(blue * 255)])
    return colors
