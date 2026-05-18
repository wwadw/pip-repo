#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS图像数据采集工具

订阅ROS图像话题，每隔指定帧数采集一次图片保存到指定目录
"""

import os
import sys
import argparse
from datetime import datetime
from pathlib import Path
from typing import Optional

try:
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except ImportError:
    rospy = None
    Image = None
    CvBridge = None

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

try:
    import tomli
except ImportError:
    try:
        import tomllib as tomli
    except ImportError:
        tomli = None


class ImageCollector:
    """ROS图像数据采集器"""

    def __init__(
        self,
        topic: str,
        interval: int = 10,
        save_dir: str = "./images",
        image_format: str = "jpg",
        prefix: str = "img",
        quality: int = 95,
    ):
        """
        初始化图像采集器

        Args:
            topic: ROS图像话题名称
            interval: 采集间隔帧数
            save_dir: 图片保存目录
            image_format: 图片格式 (jpg, png)
            prefix: 图片文件名前缀
            quality: 图片质量 (1-100, 仅jpg有效)
        """
        if rospy is None:
            raise ImportError("请安装rospy: pip install rospy 或确保ROS环境已配置")
        if cv2 is None:
            raise ImportError("请安装opencv: pip install opencv-python")

        self.topic = topic
        self.interval = max(1, interval)
        self.save_dir = Path(save_dir)
        self.image_format = image_format.lower()
        self.prefix = prefix
        self.quality = min(100, max(1, quality))

        self.bridge = CvBridge()
        self.frame_count = 0
        self.saved_count = 0

        # 创建保存目录
        self.save_dir.mkdir(parents=True, exist_ok=True)

        self.subscriber: Optional[rospy.Subscriber] = None

    def _image_callback(self, msg: "Image"):
        """图像回调函数"""
        self.frame_count += 1

        # 检查是否需要保存
        if self.frame_count % self.interval != 0:
            return

        try:
            # 转换ROS图像消息为OpenCV格式
            if msg.encoding == "rgb8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            elif msg.encoding == "bgr8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            elif msg.encoding == "mono8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"{self.prefix}_{timestamp}.{self.image_format}"
            filepath = self.save_dir / filename

            # 保存图片
            if self.image_format == "jpg" or self.image_format == "jpeg":
                params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]
            elif self.image_format == "png":
                params = [cv2.IMWRITE_PNG_COMPRESSION, 9 - int(self.quality / 11)]
            else:
                params = []

            cv2.imwrite(str(filepath), cv_image, params)
            self.saved_count += 1

            rospy.loginfo(
                f"[{self.saved_count}] 保存图片: {filename} (帧数: {self.frame_count})"
            )

        except Exception as e:
            rospy.logerr(f"保存图片失败: {e}")

    def start(self):
        """启动采集"""
        rospy.init_node("image_collector", anonymous=True)

        rospy.loginfo(f"开始采集图像...")
        rospy.loginfo(f"  话题: {self.topic}")
        rospy.loginfo(f"  间隔: 每 {self.interval} 帧")
        rospy.loginfo(f"  保存目录: {self.save_dir.absolute()}")
        rospy.loginfo(f"  图片格式: {self.image_format}")

        self.subscriber = rospy.Subscriber(self.topic, Image, self._image_callback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("用户中断")
        finally:
            self.stop()

    def stop(self):
        """停止采集"""
        if self.subscriber:
            self.subscriber.unregister()
        rospy.loginfo(f"采集结束，共保存 {self.saved_count} 张图片")


def load_config(config_path: str) -> dict:
    """加载TOML配置文件"""
    if tomli is None:
        raise ImportError("请安装tomli: pip install tomli (Python < 3.11)")

    with open(config_path, "rb") as f:
        return tomli.load(f)


def main():
    """主入口函数"""
    parser = argparse.ArgumentParser(
        description="ROS图像数据采集工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  dataset /camera/image_raw 10 ./images
  dataset --config config.toml
        """,
    )

    parser.add_argument("topic", nargs="?", help="ROS图像话题名称")
    parser.add_argument("interval", nargs="?", type=int, help="采集间隔帧数")
    parser.add_argument("save_dir", nargs="?", help="图片保存目录")
    parser.add_argument("-c", "--config", help="TOML配置文件路径")
    parser.add_argument("-f", "--format", default="jpg", help="图片格式 (jpg/png)")
    parser.add_argument("-p", "--prefix", default="img", help="图片文件名前缀")
    parser.add_argument("-q", "--quality", type=int, default=95, help="图片质量 (1-100)")

    args = parser.parse_args()

    # 从配置文件加载或使用命令行参数
    if args.config:
        config = load_config(args.config)
        collector_config = config.get("collector", {})
        topic = collector_config.get("topic", args.topic)
        interval = collector_config.get("interval", args.interval or 10)
        save_dir = collector_config.get("save_dir", args.save_dir or "./images")
        image_format = collector_config.get("format", args.format)
        prefix = collector_config.get("prefix", args.prefix)
        quality = collector_config.get("quality", args.quality)
    else:
        if not args.topic:
            parser.error("请提供话题名称或使用 --config 指定配置文件")
        topic = args.topic
        interval = args.interval or 10
        save_dir = args.save_dir or "./images"
        image_format = args.format
        prefix = args.prefix
        quality = args.quality

    # 创建采集器并启动
    collector = ImageCollector(
        topic=topic,
        interval=interval,
        save_dir=save_dir,
        image_format=image_format,
        prefix=prefix,
        quality=quality,
    )
    collector.start()


if __name__ == "__main__":
    main()
