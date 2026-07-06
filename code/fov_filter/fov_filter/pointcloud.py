from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from fov_filter.types import FovRegion


_FIELD_DTYPES = {
    PointField.INT8: "i1",
    PointField.UINT8: "u1",
    PointField.INT16: "i2",
    PointField.UINT16: "u2",
    PointField.INT32: "i4",
    PointField.UINT32: "u4",
    PointField.FLOAT32: "f4",
    PointField.FLOAT64: "f8",
}


def build_numpy_dtype(fields: Sequence[PointField], point_step: int, is_bigendian: bool) -> np.dtype:
    endian = ">" if is_bigendian else "<"
    sorted_fields = sorted(fields, key=lambda field: field.offset)
    dtype_fields = []
    cursor = 0
    pad_index = 0

    for field in sorted_fields:
        if field.datatype not in _FIELD_DTYPES:
            raise ValueError(f"不支持的 PointField datatype: {field.datatype}")

        base_dtype = np.dtype(endian + _FIELD_DTYPES[field.datatype])
        if cursor < field.offset:
            dtype_fields.append((f"__pad_{pad_index}", f"V{field.offset - cursor}"))
            pad_index += 1
            cursor = field.offset

        shape = (field.count,) if field.count > 1 else ()
        dtype_fields.append((field.name, base_dtype if not shape else (base_dtype, shape)))
        cursor = field.offset + base_dtype.itemsize * max(1, field.count)

    if cursor < point_step:
        dtype_fields.append((f"__pad_{pad_index}", f"V{point_step - cursor}"))

    return np.dtype(dtype_fields)


def pointcloud2_to_array(msg: PointCloud2) -> np.ndarray:
    dtype = build_numpy_dtype(msg.fields, msg.point_step, msg.is_bigendian)
    count = msg.width * msg.height
    return np.frombuffer(msg.data, dtype=dtype, count=count).copy()


def extract_xyz(array: np.ndarray) -> np.ndarray:
    required = {"x", "y", "z"}
    missing = sorted(required.difference(array.dtype.names or ()))
    if missing:
        raise ValueError(f"点云缺少字段: {', '.join(missing)}")

    xyz = np.empty((array.shape[0], 3), dtype=np.float32)
    xyz[:, 0] = np.asarray(array["x"], dtype=np.float32).reshape(-1)
    xyz[:, 1] = np.asarray(array["y"], dtype=np.float32).reshape(-1)
    xyz[:, 2] = np.asarray(array["z"], dtype=np.float32).reshape(-1)
    return xyz


def angle_mask_deg(angles_deg: np.ndarray, min_deg: float, max_deg: float) -> np.ndarray:
    normalized_angles = np.mod(angles_deg, 360.0)
    normalized_min = float(min_deg) % 360.0
    normalized_max = float(max_deg) % 360.0
    if normalized_min <= normalized_max:
        return (normalized_angles >= normalized_min) & (normalized_angles <= normalized_max)
    return (normalized_angles >= normalized_min) | (normalized_angles <= normalized_max)


def build_region_mask(xyz: np.ndarray, regions: Iterable[FovRegion]) -> np.ndarray:
    if xyz.size == 0:
        return np.zeros((0,), dtype=bool)

    finite_mask = np.isfinite(xyz).all(axis=1)
    enabled_regions = [region for region in regions if region.enabled]
    if not enabled_regions:
        return np.zeros(xyz.shape[0], dtype=bool)

    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]
    horizontal_deg = np.degrees(np.arctan2(y, x))
    vertical_deg = np.degrees(np.arctan2(z, np.hypot(x, y)))
    distance_m = np.linalg.norm(xyz, axis=1)

    keep_mask = np.zeros(xyz.shape[0], dtype=bool)
    for region in enabled_regions:
        h_mask = angle_mask_deg(
            horizontal_deg,
            region.horizontal_min_deg,
            region.horizontal_max_deg,
        )
        v_min = min(region.vertical_min_deg, region.vertical_max_deg)
        v_max = max(region.vertical_min_deg, region.vertical_max_deg)
        v_mask = (vertical_deg >= v_min) & (vertical_deg <= v_max)
        d_min = min(region.min_distance_m, region.max_distance_m)
        d_max = max(region.min_distance_m, region.max_distance_m)
        d_mask = (distance_m >= d_min) & (distance_m <= d_max)
        keep_mask |= h_mask & v_mask & d_mask

    return finite_mask & keep_mask


def subset_pointcloud(msg: PointCloud2, array: np.ndarray) -> PointCloud2:
    filtered = PointCloud2()
    filtered.header = msg.header
    filtered.height = 1
    filtered.width = int(array.shape[0])
    filtered.fields = list(msg.fields)
    filtered.is_bigendian = msg.is_bigendian
    filtered.point_step = msg.point_step
    filtered.row_step = filtered.point_step * filtered.width
    filtered.is_dense = msg.is_dense
    filtered.data = array.tobytes()
    return filtered


def _rgb_u32(r: int, g: int, b: int) -> np.uint32:
    return np.uint32((r << 16) | (g << 8) | b)


def make_visual_cloud(
    header,
    accepted_xyz: np.ndarray,
    rejected_xyz: np.ndarray,
    accepted_rgb: np.uint32 = _rgb_u32(230, 230, 230),
    rejected_rgb: np.uint32 = _rgb_u32(255, 0, 0),
) -> PointCloud2:
    total = int(accepted_xyz.shape[0] + rejected_xyz.shape[0])
    array = np.zeros(
        total,
        dtype=np.dtype(
            [("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.uint32)]
        ),
    )

    accepted_count = int(accepted_xyz.shape[0])
    if accepted_count:
        array["x"][:accepted_count] = accepted_xyz[:, 0]
        array["y"][:accepted_count] = accepted_xyz[:, 1]
        array["z"][:accepted_count] = accepted_xyz[:, 2]
        array["rgb"][:accepted_count] = accepted_rgb

    if rejected_xyz.shape[0]:
        array["x"][accepted_count:] = rejected_xyz[:, 0]
        array["y"][accepted_count:] = rejected_xyz[:, 1]
        array["z"][accepted_count:] = rejected_xyz[:, 2]
        array["rgb"][accepted_count:] = rejected_rgb

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = total
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * total
    msg.is_dense = True
    msg.data = array.tobytes()
    return msg
