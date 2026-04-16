from dataclasses import dataclass
from typing import Literal, Optional, Tuple

import numpy as np

from rerun_projection.projection_core import ProjectionResult


@dataclass
class FrameData:
    frame_index: int
    image_stamp: float
    cloud_stamp: float
    overlay_stamp: float | None
    semantic_image: np.ndarray
    overlay_image: np.ndarray
    points_xyz: np.ndarray
    projection: ProjectionResult


@dataclass
class CurrentSelection:
    frame_index: int
    source_view: Literal["2d", "3d"]
    clicked_pixel: Optional[np.ndarray]
    matched_pixel: np.ndarray
    point_index: int
    point_xyz: np.ndarray
    depth: float
    pixel_error: float


@dataclass
class LockedPair:
    pair_id: int
    frame_index: int
    source_view: Literal["2d", "3d"]
    point_index: int
    point_xyz: np.ndarray
    clicked_pixel: Optional[np.ndarray]
    projected_pixel: np.ndarray
    depth: float
    color_rgb: Tuple[int, int, int]
