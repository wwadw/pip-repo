from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, List

import yaml

try:
    import tomli
except ImportError:
    try:
        import tomllib as tomli
    except ImportError as exc:
        raise ImportError("请安装 tomli 或使用 Python 3.11+") from exc

from fov_filter.types import FovRegion, normalize_lidar_model, parse_regions_config


_FILTER_REGION_KEYS = (
    "min_horiz_deg",
    "max_horiz_deg",
    "min_vert_deg",
    "max_vert_deg",
    "min_dist_m",
    "max_dist_m",
)


def load_config(path: str) -> Dict[str, Any]:
    filepath = Path(path)
    suffix = filepath.suffix.lower()

    if suffix in {".yaml", ".yml"}:
        with filepath.open("r", encoding="utf-8") as stream:
            loaded = yaml.safe_load(stream) or {}
    else:
        with filepath.open("rb") as stream:
            loaded = tomli.load(stream) or {}

    if not isinstance(loaded, dict):
        raise ValueError(f"配置文件顶层必须是字典: {path}")
    return loaded


def regions_from_config(config: Dict[str, Any]) -> List[FovRegion]:
    region_items = config.get("regions")
    if region_items is None:
        region_items = config.get("filter_regions")
    return parse_regions_config(region_items)


def _format_filter_regions_yaml(region_list: List[Dict[str, float]]) -> str:
    lines = ["filter_regions:"]
    for region in region_list:
        for index, key in enumerate(_FILTER_REGION_KEYS):
            prefix = "        - " if index == 0 else "          "
            lines.append(f"{prefix}{key}: {float(region[key])}")
    return "\n".join(lines) + "\n"


def dump_filter_regions_yaml(
    regions: Iterable[FovRegion],
    enabled_only: bool = True,
    lidar_model: str = "pointcloud",
) -> str:
    region_list = []
    normalized_lidar_model = normalize_lidar_model(lidar_model)
    for region in regions:
        if enabled_only and not region.enabled:
            continue
        region_list.append(region.to_filter_region_dict(lidar_model=normalized_lidar_model))

    return _format_filter_regions_yaml(region_list)


def write_filter_regions_yaml(
    path: str,
    regions: Iterable[FovRegion],
    enabled_only: bool = True,
    lidar_model: str = "pointcloud",
) -> int:
    region_list = list(regions)
    content = dump_filter_regions_yaml(
        regions=region_list,
        enabled_only=enabled_only,
        lidar_model=lidar_model,
    )
    filepath = Path(path)
    filepath.parent.mkdir(parents=True, exist_ok=True)
    filepath.write_text(content, encoding="utf-8")
    return len([region for region in region_list if region.enabled or not enabled_only])
