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

from fov_filter.types import FovRegion, parse_regions_config


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


def dump_filter_regions_yaml(
    regions: Iterable[FovRegion],
    enabled_only: bool = True,
) -> str:
    region_list = []
    for region in regions:
        if enabled_only and not region.enabled:
            continue
        region_list.append(region.to_filter_region_dict())

    return yaml.safe_dump(
        {"filter_regions": region_list},
        allow_unicode=True,
        sort_keys=False,
        default_flow_style=False,
    )


def write_filter_regions_yaml(
    path: str,
    regions: Iterable[FovRegion],
    enabled_only: bool = True,
) -> int:
    region_list = list(regions)
    content = dump_filter_regions_yaml(regions=region_list, enabled_only=enabled_only)
    filepath = Path(path)
    filepath.parent.mkdir(parents=True, exist_ok=True)
    filepath.write_text(content, encoding="utf-8")
    return len([region for region in region_list if region.enabled or not enabled_only])
