import argparse

import uvicorn

from rerun_projection.config import DEFAULT_CONFIG
from rerun_projection.runtime import build_runtime
from rerun_projection.server import create_app


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Rerun LiDAR-camera projection workbench")
    parser.add_argument("--bag", dest="bag_file", default=DEFAULT_CONFIG["bag_file"], help="Input rosbag path")
    parser.add_argument("--yaml", dest="yaml_path", default=DEFAULT_CONFIG["yaml_path"], help="YAML file with camera parameters")
    parser.add_argument("--image-topic", dest="image_topic", default=DEFAULT_CONFIG["image_topic"], help="Semantic image topic")
    parser.add_argument(
        "--overlay-image-topic",
        dest="overlay_image_topic",
        default=DEFAULT_CONFIG["overlay_image_topic"],
        help="Detection overlay image topic",
    )
    parser.add_argument("--cloud-topic", dest="pointcloud_topic", default=DEFAULT_CONFIG["pointcloud_topic"], help="Point cloud topic")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP host")
    parser.add_argument("--port", type=int, default=8765, help="HTTP port")
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    cli_overrides = {
        "bag_file": args.bag_file,
        "yaml_path": args.yaml_path,
        "image_topic": args.image_topic,
        "overlay_image_topic": args.overlay_image_topic,
        "pointcloud_topic": args.pointcloud_topic,
    }
    runtime = build_runtime(test_mode=False, cli_overrides=cli_overrides)
    app = create_app(runtime=runtime)
    uvicorn.run(app, host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
