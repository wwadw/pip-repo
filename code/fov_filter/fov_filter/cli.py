from __future__ import annotations

import argparse
import sys
from typing import Any, Dict, List

from fov_filter.config_io import load_config, regions_from_config
from fov_filter.types import FovRegion


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="bag 点云 FOV 过滤与回放节点",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  fov-filter --bag /home/ww/bags/ren/go7.bag --topic /mfla/frame_cloud --paint-rejected
  fov-filter --config /home/ww/test/scripts/fov_filter/config.example.toml
        """,
    )
    parser.add_argument("-c", "--config", help="TOML/YAML 配置文件路径")
    parser.add_argument("--bag", help="bag 文件路径")
    parser.add_argument("--topic", help="PointCloud2 话题")
    parser.add_argument("--topic-prefix", help="输出/控制话题前缀，默认 /fov_filter")
    parser.add_argument("--out-topic", help="过滤后保留点云发布话题")
    parser.add_argument("--rejected-topic", help="被过滤移除点云发布话题")
    parser.add_argument("--visual-topic", help="保留/移除双色点云发布话题")
    parser.add_argument("--marker-topic", help="RViz FOV 区域 MarkerArray 发布话题")
    parser.add_argument("--command-topic", help="控制命令输入话题")
    parser.add_argument("--state-topic", help="状态输出话题")
    parser.add_argument("--node-name", help="ROS 节点名")
    parser.add_argument("--rate", type=float, help="回放倍率")
    parser.add_argument(
        "--region",
        action="append",
        default=[],
        help="区域: name:hmin:hmax:vmin:vmax[:enabled][:dmin:dmax]",
    )
    parser.add_argument("--loop", action="store_true", help="播到末尾后循环")
    parser.add_argument("--start-paused", action="store_true", help="启动后先暂停")
    parser.add_argument("--paint-rejected", action="store_true", help="被过滤点云标红并发布可视化点云")
    parser.add_argument("--publish-clock", action="store_true", help="发布 /clock")
    parser.add_argument(
        "--no-publish-rejected",
        action="store_true",
        help="不发布被过滤掉的原始点云",
    )
    return parser


def resolve_value(cli_value: Any, config_value: Any, default: Any) -> Any:
    if cli_value is not None:
        return cli_value
    if config_value is not None:
        return config_value
    return default


def main(argv: List[str] = None) -> None:
    parser = build_arg_parser()
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

    from fov_filter.player import FovFilterPlayer, _topic_join

    config: Dict[str, Any] = {}
    if args.config:
        config = load_config(args.config)
    player_config = config.get("player", {})

    bag = resolve_value(args.bag, player_config.get("bag"), None)
    topic = resolve_value(args.topic, player_config.get("topic"), None)
    if not bag or not topic:
        parser.error("必须提供 --bag 和 --topic，或在 --config 中提供")

    regions = regions_from_config(config)
    regions.extend(FovRegion.from_cli_spec(spec) for spec in args.region)

    topic_prefix = resolve_value(args.topic_prefix, player_config.get("topic_prefix"), "/fov_filter")
    out_topic = resolve_value(
        args.out_topic,
        player_config.get("out_topic"),
        _topic_join(topic_prefix, "points_kept"),
    )
    rejected_topic = resolve_value(
        args.rejected_topic,
        player_config.get("rejected_topic"),
        _topic_join(topic_prefix, "points_removed"),
    )
    visual_topic = resolve_value(
        args.visual_topic,
        player_config.get("visual_topic"),
        _topic_join(topic_prefix, "points_colored"),
    )
    marker_topic = resolve_value(
        args.marker_topic,
        player_config.get("marker_topic"),
        _topic_join(topic_prefix, "fov_regions"),
    )
    command_topic = resolve_value(
        args.command_topic,
        player_config.get("command_topic"),
        _topic_join(topic_prefix, "command"),
    )
    state_topic = resolve_value(
        args.state_topic,
        player_config.get("state_topic"),
        _topic_join(topic_prefix, "state"),
    )
    node_name = resolve_value(args.node_name, player_config.get("node_name"), "fov_filter_player")
    rate = float(resolve_value(args.rate, player_config.get("rate"), 1.0))
    loop = bool(args.loop or player_config.get("loop", False))
    start_paused = bool(args.start_paused or player_config.get("start_paused", False))
    paint_rejected = bool(args.paint_rejected or player_config.get("paint_rejected", False))
    publish_clock = bool(args.publish_clock or player_config.get("publish_clock", False))
    publish_rejected = not args.no_publish_rejected
    if not args.no_publish_rejected:
        publish_rejected = bool(player_config.get("publish_rejected", True))

    rospy.init_node(node_name, anonymous=False)

    player = FovFilterPlayer(
        bag_path=bag,
        topic=topic,
        out_topic=out_topic,
        rejected_topic=rejected_topic,
        visual_topic=visual_topic,
        marker_topic=marker_topic,
        command_topic=command_topic,
        state_topic=state_topic,
        rate=rate,
        loop=loop,
        start_paused=start_paused,
        paint_rejected=paint_rejected,
        publish_rejected=publish_rejected,
        publish_clock=publish_clock,
        regions=regions,
    )
    player.spin()


if __name__ == "__main__":
    main()
