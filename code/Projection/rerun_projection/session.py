from dataclasses import dataclass, field
from typing import List, Literal, Optional

import numpy as np

from rerun_projection.models import CurrentSelection, LockedPair


PAIR_COLORS = [
    (255, 80, 80),
    (80, 200, 255),
    (120, 255, 120),
    (255, 220, 80),
]


@dataclass
class ProjectionSession:
    projected_pixels: np.ndarray
    valid_point_indices: np.ndarray
    points_xyz: np.ndarray
    camera_points: np.ndarray
    current_selection: Optional[CurrentSelection] = None
    locked_pairs: List[LockedPair] = field(default_factory=list)
    _next_pair_id: int = 1

    @classmethod
    def for_test(
        cls,
        *,
        projected_pixels: np.ndarray,
        valid_point_indices: np.ndarray,
        points_xyz: np.ndarray,
        camera_points: np.ndarray,
    ) -> "ProjectionSession":
        return cls(
            projected_pixels=projected_pixels,
            valid_point_indices=valid_point_indices,
            points_xyz=points_xyz,
            camera_points=camera_points,
        )

    def select_2d(self, *, frame_index: int, pixel: np.ndarray) -> CurrentSelection:
        if len(self.projected_pixels) == 0:
            raise RuntimeError("No projected points are visible in the current frame.")
        deltas = self.projected_pixels - pixel.reshape(1, 2)
        distances = np.einsum("ij,ij->i", deltas, deltas)
        nearest_idx = int(np.argmin(distances))
        point_index = int(self.valid_point_indices[nearest_idx])
        return self._select_visible_point(frame_index=frame_index, point_index=point_index, source_view="2d", clicked_pixel=pixel)

    def select_3d(self, *, frame_index: int, instance_id: Optional[int], position: Optional[np.ndarray]) -> CurrentSelection:
        if instance_id is not None:
            point_index = int(instance_id)
        else:
            if position is None:
                raise RuntimeError("3D selection requires an instance id or position.")
            deltas = self.points_xyz - position.reshape(1, 3)
            point_index = int(np.argmin(np.einsum("ij,ij->i", deltas, deltas)))
        return self._select_visible_point(frame_index=frame_index, point_index=point_index, source_view="3d", clicked_pixel=None)

    def select_projected_point(
        self,
        *,
        frame_index: int,
        instance_id: Optional[int],
        position: Optional[np.ndarray],
    ) -> CurrentSelection:
        if len(self.projected_pixels) == 0:
            raise RuntimeError("No projected points are visible in the current frame.")
        if instance_id is not None:
            point_index = int(instance_id)
            match_indices = np.where(self.valid_point_indices == point_index)[0]
            if match_indices.size == 0:
                raise RuntimeError(f"Point index {point_index} is not visible in the current projection.")
            clicked_pixel = self.projected_pixels[int(match_indices[0])]
        else:
            if position is None:
                raise RuntimeError("2D projected-point selection requires an instance id or pixel position.")
            clicked_pixel = np.asarray(position[:2], dtype=np.float64)
            deltas = self.projected_pixels - clicked_pixel.reshape(1, 2)
            nearest_idx = int(np.argmin(np.einsum("ij,ij->i", deltas, deltas)))
            point_index = int(self.valid_point_indices[nearest_idx])
        return self._select_visible_point(frame_index=frame_index, point_index=point_index, source_view="2d", clicked_pixel=clicked_pixel)

    def lock_current_pair(self) -> LockedPair:
        if self.current_selection is None:
            raise RuntimeError("No current selection to lock.")
        selection = self.current_selection
        pair = LockedPair(
            pair_id=self._next_pair_id,
            frame_index=selection.frame_index,
            source_view=selection.source_view,
            point_index=selection.point_index,
            point_xyz=selection.point_xyz.copy(),
            clicked_pixel=None if selection.clicked_pixel is None else selection.clicked_pixel.copy(),
            projected_pixel=selection.matched_pixel.copy(),
            depth=selection.depth,
            color_rgb=PAIR_COLORS[(self._next_pair_id - 1) % len(PAIR_COLORS)],
        )
        self.locked_pairs.append(pair)
        self._next_pair_id += 1
        return pair

    def replace_projection_for_test(
        self,
        *,
        projected_pixels: np.ndarray,
        valid_point_indices: np.ndarray,
        camera_points: np.ndarray,
    ) -> None:
        self.projected_pixels = projected_pixels
        self.valid_point_indices = valid_point_indices
        self.camera_points = camera_points

    def reproject_locked_pairs(self) -> None:
        pixel_map = {
            int(point_index): self.projected_pixels[idx]
            for idx, point_index in enumerate(self.valid_point_indices.tolist())
        }
        if self.current_selection is not None:
            if self.current_selection.point_index in pixel_map:
                self.current_selection.matched_pixel = pixel_map[self.current_selection.point_index].copy()
                self.current_selection.depth = float(self.camera_points[self.current_selection.point_index, 2])
                if self.current_selection.clicked_pixel is not None:
                    self.current_selection.pixel_error = float(
                        np.linalg.norm(self.current_selection.clicked_pixel - self.current_selection.matched_pixel)
                    )
            else:
                self.current_selection = None
        for pair in self.locked_pairs:
            if pair.point_index in pixel_map:
                pair.projected_pixel = pixel_map[pair.point_index].copy()

    def delete_last_pair(self) -> None:
        if self.locked_pairs:
            self.locked_pairs.pop()

    def clear_pairs(self) -> None:
        self.locked_pairs.clear()

    def clear_for_new_frame(self) -> None:
        self.current_selection = None
        self.locked_pairs.clear()
        self._next_pair_id = 1

    def clear_for_new_source(self) -> None:
        self.clear_for_new_frame()

    def _select_visible_point(
        self,
        *,
        frame_index: int,
        point_index: int,
        source_view: Literal["2d", "3d"],
        clicked_pixel: Optional[np.ndarray],
    ) -> CurrentSelection:
        match_indices = np.where(self.valid_point_indices == point_index)[0]
        if match_indices.size == 0:
            raise RuntimeError(f"Point index {point_index} is not visible in the current projection.")
        matched_pixel = self.projected_pixels[int(match_indices[0])].copy()
        point_xyz = self.points_xyz[point_index].copy()
        depth = float(self.camera_points[point_index, 2])
        resolved_click = None if clicked_pixel is None else np.asarray(clicked_pixel, dtype=np.float64).copy()
        pixel_error = 0.0 if resolved_click is None else float(np.linalg.norm(resolved_click - matched_pixel))
        selection = CurrentSelection(
            frame_index=frame_index,
            source_view=source_view,
            clicked_pixel=resolved_click,
            matched_pixel=matched_pixel,
            point_index=point_index,
            point_xyz=point_xyz,
            depth=depth,
            pixel_error=pixel_error,
        )
        self.current_selection = selection
        return selection
