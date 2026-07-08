from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional

DEFAULT_MIN_DISTANCE_M = 0.0
DEFAULT_MAX_DISTANCE_M = 2.0
LIDAR_MODEL_POINTCLOUD = "pointcloud"
LIDAR_MODEL_RSHELIOS = "rshelios"
SUPPORTED_LIDAR_MODELS = {
    LIDAR_MODEL_POINTCLOUD,
    LIDAR_MODEL_RSHELIOS,
}


def parse_bool(value: Any, default: bool = True) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "on", "enable", "enabled"}:
        return True
    if text in {"0", "false", "no", "n", "off", "disable", "disabled"}:
        return False
    raise ValueError(f"无法解析布尔值: {value}")


def normalize_lidar_model(value: Any) -> str:
    model = str(value or LIDAR_MODEL_POINTCLOUD).strip().lower().replace("_", "-")
    aliases = {
        "default": LIDAR_MODEL_POINTCLOUD,
        "published": LIDAR_MODEL_POINTCLOUD,
        "published-pointcloud": LIDAR_MODEL_POINTCLOUD,
        "point-cloud": LIDAR_MODEL_POINTCLOUD,
        "rslidar-rshelios": LIDAR_MODEL_RSHELIOS,
        "rs-helios": LIDAR_MODEL_RSHELIOS,
        "rshelios": LIDAR_MODEL_RSHELIOS,
    }
    model = aliases.get(model, model)
    if model not in SUPPORTED_LIDAR_MODELS:
        supported = ", ".join(sorted(SUPPORTED_LIDAR_MODELS))
        raise ValueError(f"不支持的 lidar_model: {value}，可选: {supported}")
    return model


def pointcloud_horizontal_to_driver_deg(angle_deg: float, lidar_model: str) -> float:
    model = normalize_lidar_model(lidar_model)
    normalized = float(angle_deg) % 360.0
    if model == LIDAR_MODEL_RSHELIOS:
        return (360.0 - normalized) % 360.0
    return normalized


def pointcloud_horizontal_to_driver_max_deg(angle_deg: float, lidar_model: str) -> float:
    converted = pointcloud_horizontal_to_driver_deg(angle_deg, lidar_model)
    if normalize_lidar_model(lidar_model) == LIDAR_MODEL_RSHELIOS and converted == 0.0:
        return 360.0
    return converted


@dataclass
class FovRegion:
    name: str
    horizontal_min_deg: float
    horizontal_max_deg: float
    vertical_min_deg: float
    vertical_max_deg: float
    min_distance_m: float = DEFAULT_MIN_DISTANCE_M
    max_distance_m: float = DEFAULT_MAX_DISTANCE_M
    enabled: bool = True

    @classmethod
    def from_cli_spec(cls, spec: str) -> "FovRegion":
        parts = [part.strip() for part in spec.split(":")]
        if len(parts) not in {5, 6, 7, 8}:
            raise ValueError(
                "区域格式应为 name:hmin:hmax:vmin:vmax[:enabled][:dmin:dmax]"
            )
        enabled = True
        min_distance_m = DEFAULT_MIN_DISTANCE_M
        max_distance_m = DEFAULT_MAX_DISTANCE_M

        if len(parts) == 6:
            enabled = parse_bool(parts[5], default=True)
        elif len(parts) == 7:
            min_distance_m = float(parts[5])
            max_distance_m = float(parts[6])
        elif len(parts) == 8:
            enabled = parse_bool(parts[5], default=True)
            min_distance_m = float(parts[6])
            max_distance_m = float(parts[7])

        return cls(
            name=parts[0],
            horizontal_min_deg=float(parts[1]),
            horizontal_max_deg=float(parts[2]),
            vertical_min_deg=float(parts[3]),
            vertical_max_deg=float(parts[4]),
            min_distance_m=min_distance_m,
            max_distance_m=max_distance_m,
            enabled=enabled,
        )

    @classmethod
    def from_mapping(
        cls,
        data: Mapping[str, Any],
        default_name: Optional[str] = None,
    ) -> "FovRegion":
        name = str(data.get("name") or default_name or "").strip()
        if not name:
            raise ValueError("区域必须提供 name")

        horizontal = data.get("horizontal") or data.get("h")
        vertical = data.get("vertical") or data.get("v")
        distance = data.get("distance") or data.get("dist")

        if horizontal is not None:
            if len(horizontal) != 2:
                raise ValueError("horizontal 必须包含 2 个值")
            horizontal_min_deg, horizontal_max_deg = horizontal
        else:
            horizontal_min_deg = data.get(
                "horizontal_min_deg",
                data.get("h_min", data.get("min_horiz_deg")),
            )
            horizontal_max_deg = data.get(
                "horizontal_max_deg",
                data.get("h_max", data.get("max_horiz_deg")),
            )

        if vertical is not None:
            if len(vertical) != 2:
                raise ValueError("vertical 必须包含 2 个值")
            vertical_min_deg, vertical_max_deg = vertical
        else:
            vertical_min_deg = data.get(
                "vertical_min_deg",
                data.get("v_min", data.get("min_vert_deg")),
            )
            vertical_max_deg = data.get(
                "vertical_max_deg",
                data.get("v_max", data.get("max_vert_deg")),
            )

        if distance is not None:
            if len(distance) != 2:
                raise ValueError("distance 必须包含 2 个值")
            min_distance_m, max_distance_m = distance
        else:
            min_distance_m = data.get(
                "min_distance_m",
                data.get("d_min", data.get("min_dist_m", DEFAULT_MIN_DISTANCE_M)),
            )
            max_distance_m = data.get(
                "max_distance_m",
                data.get("d_max", data.get("max_dist_m", DEFAULT_MAX_DISTANCE_M)),
            )

        if horizontal_min_deg is None or horizontal_max_deg is None:
            raise ValueError(f"区域 {name} 缺少水平角范围")
        if vertical_min_deg is None or vertical_max_deg is None:
            raise ValueError(f"区域 {name} 缺少垂直角范围")

        enabled = parse_bool(data.get("enabled", True), default=True)
        return cls(
            name=name,
            horizontal_min_deg=float(horizontal_min_deg),
            horizontal_max_deg=float(horizontal_max_deg),
            vertical_min_deg=float(vertical_min_deg),
            vertical_max_deg=float(vertical_max_deg),
            min_distance_m=float(min_distance_m),
            max_distance_m=float(max_distance_m),
            enabled=enabled,
        )

    def update_from_mapping(self, data: Mapping[str, Any]) -> None:
        if "name" in data and data["name"]:
            self.name = str(data["name"])

        horizontal = data.get("horizontal") or data.get("h")
        if horizontal is not None:
            if len(horizontal) != 2:
                raise ValueError("horizontal 必须包含 2 个值")
            self.horizontal_min_deg = float(horizontal[0])
            self.horizontal_max_deg = float(horizontal[1])

        vertical = data.get("vertical") or data.get("v")
        if vertical is not None:
            if len(vertical) != 2:
                raise ValueError("vertical 必须包含 2 个值")
            self.vertical_min_deg = float(vertical[0])
            self.vertical_max_deg = float(vertical[1])

        distance = data.get("distance") or data.get("dist")
        if distance is not None:
            if len(distance) != 2:
                raise ValueError("distance 必须包含 2 个值")
            self.min_distance_m = float(distance[0])
            self.max_distance_m = float(distance[1])

        scalar_updates = {
            "horizontal_min_deg": "horizontal_min_deg",
            "h_min": "horizontal_min_deg",
            "min_horiz_deg": "horizontal_min_deg",
            "horizontal_max_deg": "horizontal_max_deg",
            "h_max": "horizontal_max_deg",
            "max_horiz_deg": "horizontal_max_deg",
            "vertical_min_deg": "vertical_min_deg",
            "v_min": "vertical_min_deg",
            "min_vert_deg": "vertical_min_deg",
            "vertical_max_deg": "vertical_max_deg",
            "v_max": "vertical_max_deg",
            "max_vert_deg": "vertical_max_deg",
            "min_distance_m": "min_distance_m",
            "d_min": "min_distance_m",
            "min_dist_m": "min_distance_m",
            "max_distance_m": "max_distance_m",
            "d_max": "max_distance_m",
            "max_dist_m": "max_distance_m",
        }
        for key, attr in scalar_updates.items():
            if key in data and data[key] is not None:
                setattr(self, attr, float(data[key]))

        if "enabled" in data:
            self.enabled = parse_bool(data.get("enabled"), default=self.enabled)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "horizontal": [self.horizontal_min_deg, self.horizontal_max_deg],
            "vertical": [self.vertical_min_deg, self.vertical_max_deg],
            "distance": [self.min_distance_m, self.max_distance_m],
            "enabled": self.enabled,
        }

    def to_filter_region_dict(self, lidar_model: str = LIDAR_MODEL_POINTCLOUD) -> Dict[str, float]:
        model = normalize_lidar_model(lidar_model)
        if model == LIDAR_MODEL_RSHELIOS:
            driver_min = pointcloud_horizontal_to_driver_deg(
                self.horizontal_max_deg,
                model,
            )
            driver_max = pointcloud_horizontal_to_driver_max_deg(
                self.horizontal_min_deg,
                model,
            )
        else:
            driver_min = pointcloud_horizontal_to_driver_deg(
                self.horizontal_min_deg,
                model,
            )
            driver_max = pointcloud_horizontal_to_driver_deg(
                self.horizontal_max_deg,
                model,
            )
        return {
            "min_horiz_deg": float(driver_min),
            "max_horiz_deg": float(driver_max),
            "min_vert_deg": float(self.vertical_min_deg),
            "max_vert_deg": float(self.vertical_max_deg),
            "min_dist_m": float(self.min_distance_m),
            "max_dist_m": float(self.max_distance_m),
        }


def parse_regions_config(items: Optional[Iterable[Mapping[str, Any]]]) -> List[FovRegion]:
    if not items:
        return []
    return [
        FovRegion.from_mapping(item, default_name=f"region_{index + 1}")
        for index, item in enumerate(items)
    ]
