from __future__ import annotations

import argparse
import json
import sys
from typing import Any, Dict, List, Optional

from fov_filter.client import FovFilterRosClient
from fov_filter.config_io import write_filter_regions_yaml
from fov_filter.types import SUPPORTED_LIDAR_MODELS, normalize_lidar_model, parse_regions_config


def topic_join(prefix: str, leaf: str) -> str:
    normalized = (prefix or "/fov_filter").strip()
    if not normalized.startswith("/"):
        normalized = "/" + normalized
    return normalized.rstrip("/") + "/" + leaf.lstrip("/")


def parse_bool_text(text: str) -> bool:
    value = text.strip().lower()
    if value in {"1", "true", "yes", "y", "on"}:
        return True
    if value in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"无法解析布尔值: {text}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="fov-filter 动态控制工具")
    parser.add_argument("--topic-prefix", default="/fov_filter", help="控制/状态话题前缀")
    parser.add_argument("--command-topic", help="命令话题")
    parser.add_argument("--state-topic", help="状态话题")
    parser.add_argument("--timeout", type=float, default=2.0, help="等待状态超时时间")

    subparsers = parser.add_subparsers(dest="subcommand", required=True)

    subparsers.add_parser("play")
    subparsers.add_parser("pause")
    subparsers.add_parser("toggle")
    subparsers.add_parser("status")
    subparsers.add_parser("republish")
    subparsers.add_parser("clear")

    next_parser = subparsers.add_parser("next")
    next_parser.add_argument("--count", type=int, default=1)

    prev_parser = subparsers.add_parser("prev")
    prev_parser.add_argument("--count", type=int, default=1)

    seek_parser = subparsers.add_parser("seek")
    seek_parser.add_argument("index", type=int)

    add_parser = subparsers.add_parser("add")
    add_parser.add_argument("--name", required=True)
    add_parser.add_argument("--h-min", type=float, required=True)
    add_parser.add_argument("--h-max", type=float, required=True)
    add_parser.add_argument("--v-min", type=float, required=True)
    add_parser.add_argument("--v-max", type=float, required=True)
    add_parser.add_argument("--d-min", type=float, default=0.0)
    add_parser.add_argument("--d-max", type=float, default=2.0)
    add_parser.add_argument("--enabled", type=parse_bool_text, default=True)

    update_parser = subparsers.add_parser("update")
    update_parser.add_argument("--name", required=True)
    update_parser.add_argument("--new-name")
    update_parser.add_argument("--h-min", type=float)
    update_parser.add_argument("--h-max", type=float)
    update_parser.add_argument("--v-min", type=float)
    update_parser.add_argument("--v-max", type=float)
    update_parser.add_argument("--d-min", type=float)
    update_parser.add_argument("--d-max", type=float)
    update_parser.add_argument("--enabled", type=parse_bool_text)

    remove_parser = subparsers.add_parser("remove")
    remove_parser.add_argument("--name", required=True)

    load_parser = subparsers.add_parser("load-config")
    load_parser.add_argument("path")

    export_parser = subparsers.add_parser("export-config")
    export_parser.add_argument("path")
    export_parser.add_argument(
        "--lidar-model",
        default="pointcloud",
        choices=sorted(SUPPORTED_LIDAR_MODELS),
        help="导出给驱动时使用的角度模型；rshelios 会把发布点云水平角转换为驱动内部水平角",
    )

    option_parser = subparsers.add_parser("set-option")
    option_parser.add_argument("--rate", type=float)
    option_parser.add_argument("--loop", type=parse_bool_text)
    option_parser.add_argument("--paint-rejected", type=parse_bool_text)
    option_parser.add_argument("--publish-rejected", type=parse_bool_text)
    option_parser.add_argument("--playing", type=parse_bool_text)

    return parser


def payload_from_args(args: argparse.Namespace) -> Optional[Dict[str, Any]]:
    subcommand = args.subcommand
    if subcommand == "status":
        return None
    if subcommand == "play":
        return {"op": "play"}
    if subcommand == "pause":
        return {"op": "pause"}
    if subcommand == "toggle":
        return {"op": "toggle"}
    if subcommand == "republish":
        return {"op": "republish"}
    if subcommand == "clear":
        return {"op": "clear_regions"}
    if subcommand == "next":
        return {"op": "next", "count": args.count}
    if subcommand == "prev":
        return {"op": "prev", "count": args.count}
    if subcommand == "seek":
        return {"op": "seek", "index": args.index}
    if subcommand == "add":
        return {
            "op": "add_region",
            "region": {
                "name": args.name,
                "horizontal": [args.h_min, args.h_max],
                "vertical": [args.v_min, args.v_max],
                "distance": [args.d_min, args.d_max],
                "enabled": args.enabled,
            },
        }
    if subcommand == "update":
        region: Dict[str, Any] = {}
        if args.new_name is not None:
            region["name"] = args.new_name
        if args.h_min is not None:
            region["h_min"] = args.h_min
        if args.h_max is not None:
            region["h_max"] = args.h_max
        if args.v_min is not None:
            region["v_min"] = args.v_min
        if args.v_max is not None:
            region["v_max"] = args.v_max
        if args.d_min is not None:
            region["d_min"] = args.d_min
        if args.d_max is not None:
            region["d_max"] = args.d_max
        if args.enabled is not None:
            region["enabled"] = args.enabled
        return {"op": "update_region", "name": args.name, "region": region}
    if subcommand == "remove":
        return {"op": "remove_region", "name": args.name}
    if subcommand == "load-config":
        return {"op": "load_config", "config_path": args.path}
    if subcommand == "export-config":
        return None
    if subcommand == "set-option":
        return {
            "op": "set_option",
            "rate": args.rate,
            "loop": args.loop,
            "paint_rejected": args.paint_rejected,
            "publish_rejected": args.publish_rejected,
            "playing": args.playing,
        }
    raise ValueError(f"未知子命令: {subcommand}")


def pretty_print_state(raw: str) -> None:
    try:
        parsed = json.loads(raw)
    except Exception:
        print(raw)
        return
    print(json.dumps(parsed, ensure_ascii=False, indent=2, sort_keys=True))


def main(argv: List[str] = None) -> None:
    parser = build_parser()
    raw_argv = argv or sys.argv

    try:
        import rospy

        parse_argv = rospy.myargv(argv=raw_argv)[1:]
    except Exception:
        rospy = None
        parse_argv = raw_argv[1:]

    args = parser.parse_args(parse_argv)
    if rospy is None:
        raise SystemExit("rospy 未安装或 ROS 环境未加载，请先 source ROS1 环境")

    try:
        client = FovFilterRosClient(
            command_topic=args.command_topic or topic_join(args.topic_prefix, "command"),
            state_topic=args.state_topic or topic_join(args.topic_prefix, "state"),
            node_name="fov_filter_control",
            anonymous=True,
            init_node=True,
        )
    except Exception as exc:
        raise SystemExit(str(exc)) from exc

    payload = payload_from_args(args)
    if args.subcommand == "export-config":
        state = client.request_status(timeout=args.timeout)
        regions = parse_regions_config(state.get("regions"))
        lidar_model = normalize_lidar_model(args.lidar_model)
        exported_count = write_filter_regions_yaml(
            args.path,
            regions,
            enabled_only=True,
            lidar_model=lidar_model,
        )
        print(
            json.dumps(
                {
                    "exported_regions": exported_count,
                    "path": args.path,
                    "enabled_only": True,
                    "lidar_model": lidar_model,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return

    if payload is None:
        pretty_print_state(json.dumps(client.request_status(timeout=args.timeout), ensure_ascii=False))
        return

    updated_state = client.send_command(payload=payload, timeout=args.timeout, wait=True) or {}
    pretty_print_state(json.dumps(updated_state, ensure_ascii=False))


if __name__ == "__main__":
    main()
