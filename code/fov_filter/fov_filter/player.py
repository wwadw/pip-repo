from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any, Dict, Iterable, List, Optional, Tuple

import rosbag
import rospy
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from fov_filter.config_io import load_config, regions_from_config
from fov_filter.pointcloud import (
    build_region_mask,
    extract_xyz,
    make_visual_cloud,
    pointcloud2_to_array,
    subset_pointcloud,
)
from fov_filter.types import FovRegion, parse_bool, parse_regions_config


def _topic_join(prefix: str, leaf: str) -> str:
    normalized = (prefix or "/fov_filter").strip()
    if not normalized.startswith("/"):
        normalized = "/" + normalized
    return normalized.rstrip("/") + "/" + leaf.lstrip("/")


class FovFilterPlayer:
    def __init__(
        self,
        bag_path: str,
        topic: str,
        out_topic: str = "/fov_filter/points_kept",
        rejected_topic: str = "/fov_filter/points_removed",
        visual_topic: str = "/fov_filter/points_colored",
        marker_topic: str = "/fov_filter/fov_regions",
        command_topic: str = "/fov_filter/command",
        state_topic: str = "/fov_filter/state",
        rate: float = 1.0,
        loop: bool = False,
        start_paused: bool = False,
        paint_rejected: bool = False,
        publish_rejected: bool = True,
        publish_clock: bool = False,
        regions: Optional[Iterable[FovRegion]] = None,
    ) -> None:
        self.bag_path = os.path.abspath(bag_path)
        self.topic = topic
        self.out_topic = out_topic
        self.rejected_topic = rejected_topic
        self.visual_topic = visual_topic
        self.marker_topic = marker_topic
        self.command_topic = command_topic
        self.state_topic = state_topic
        self.rate = max(1e-3, float(rate))
        self.loop = bool(loop)
        self.playing = not start_paused
        self.paint_rejected = bool(paint_rejected)
        self.publish_rejected = bool(publish_rejected)
        self.publish_clock = bool(publish_clock)
        self._lock = threading.RLock()

        self.filtered_pub = rospy.Publisher(self.out_topic, PointCloud2, queue_size=1)
        self.rejected_pub = rospy.Publisher(self.rejected_topic, PointCloud2, queue_size=1)
        self.visual_pub = rospy.Publisher(self.visual_topic, PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=1, latch=True)
        self.command_sub = None
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1) if self.publish_clock else None

        self.regions: Dict[str, FovRegion] = {}
        for region in regions or []:
            self.regions[region.name] = region

        self.frames: List[PointCloud2] = []
        self.frame_times: List[float] = []
        self.playback_times: List[float] = []
        self.current_index = 0
        self._next_due_at: Optional[float] = None
        self.last_stats: Dict[str, Any] = {}
        self.last_command_id: Optional[str] = None
        self.load_warnings: List[str] = []
        self.loading_frames = False
        self.bag_fully_loaded = False
        self._load_generation = 0

        self._load_frames(lazy=True)
        if not self.frames:
            raise RuntimeError(f"bag 中未找到话题 {self.topic} 的 PointCloud2 消息")

        self.publish_current_frame(reason="startup")
        self._start_command_subscriber()
        self._schedule_next_frame()

    def _start_command_subscriber(self) -> None:
        if self.command_sub is not None:
            return
        self.command_sub = rospy.Subscriber(
            self.command_topic,
            String,
            self._command_callback,
            queue_size=10,
        )

    def _open_bag(self):
        try:
            return rosbag.Bag(self.bag_path, "r", allow_unindexed=True)
        except TypeError:
            return rosbag.Bag(self.bag_path, "r")

    def _clear_frame_cache(self) -> None:
        self.frames = []
        self.frame_times = []
        self.playback_times = []
        self.current_index = 0
        self.loading_frames = False
        self.bag_fully_loaded = False

    def _available_pointcloud_topics(self, bag) -> List[str]:
        try:
            _types, topic_infos = bag.get_type_and_topic_info()
        except Exception as exc:
            warning = f"无法读取 bag topic 索引信息: {exc}"
            self.load_warnings.append(warning)
            rospy.logwarn(warning)
            return []

        topics = []
        for name, info in topic_infos.items():
            if getattr(info, "msg_type", "") == "sensor_msgs/PointCloud2":
                topics.append(name)
        return sorted(topics)

    def _append_frame(self, msg: PointCloud2, bag_time) -> None:
        bag_stamp = float(bag_time.to_sec())
        header_stamp = float(msg.header.stamp.to_sec())
        display_stamp = header_stamp if header_stamp > 0 else bag_stamp
        playback_stamp = bag_stamp if bag_stamp > 0 else display_stamp

        if self.playback_times and playback_stamp <= self.playback_times[-1]:
            playback_stamp = self.playback_times[-1] + 0.1
            if len(self.load_warnings) < 5:
                self.load_warnings.append("检测到非递增 bag 时间戳，已为播放调度自动修正")

        self.frames.append(msg)
        self.frame_times.append(display_stamp)
        self.playback_times.append(playback_stamp)

    def _iter_pointcloud_messages(self, bag):
        for _, msg, bag_time in bag.read_messages(topics=[self.topic]):
            if getattr(msg, "_type", "") != "sensor_msgs/PointCloud2":
                continue
            yield msg, bag_time

    def _load_frames(self, lazy: bool = True) -> None:
        if not os.path.exists(self.bag_path):
            raise FileNotFoundError(f"bag 文件不存在: {self.bag_path}")

        self._load_generation += 1
        generation = self._load_generation
        self._clear_frame_cache()
        rospy.loginfo("加载 bag: %s", self.bag_path)

        bag = self._open_bag()
        try:
            available_pointcloud_topics = self._available_pointcloud_topics(bag)
            if available_pointcloud_topics and self.topic not in available_pointcloud_topics:
                raise RuntimeError(
                    "bag 中没有指定的 PointCloud2 话题 "
                    f"{self.topic}。可用 PointCloud2 话题: "
                    f"{', '.join(available_pointcloud_topics)}"
                )

            iterator = self._iter_pointcloud_messages(bag)
            count = 0
            try:
                for msg, bag_time in iterator:
                    self._append_frame(msg, bag_time)
                    count += 1
                    if lazy:
                        break
                    if count % 100 == 0:
                        rospy.loginfo("已缓存 %d 帧点云", count)
            except Exception as exc:
                if self.frames:
                    warning = (
                        f"读取 bag 时遇到异常，已保留前 {len(self.frames)} 帧继续播放: {exc}"
                    )
                    self.load_warnings.append(warning)
                    rospy.logwarn(warning)
                else:
                    topics_hint = (
                        f" 可用 PointCloud2 话题: {', '.join(available_pointcloud_topics)}"
                        if available_pointcloud_topics
                        else ""
                    )
                    raise RuntimeError(f"bag 读取失败，未加载到可播放点云: {exc}.{topics_hint}") from exc

            if lazy and self.frames:
                self.loading_frames = True
                self.bag_fully_loaded = False
                self._start_background_loader(bag=bag, iterator=iterator, generation=generation)
                bag = None
                rospy.loginfo("首帧加载完成，后台继续缓存剩余点云")
            else:
                self.loading_frames = False
                self.bag_fully_loaded = True
                rospy.loginfo("加载完成，共 %d 帧", len(self.frames))
        finally:
            if bag is not None:
                bag.close()

        for warning in self.load_warnings:
            rospy.logwarn("bag 加载提示: %s", warning)

    def _start_background_loader(self, bag, iterator, generation: int) -> None:
        thread = threading.Thread(
            target=self._background_load_frames,
            args=(bag, iterator, generation),
            daemon=True,
        )
        thread.start()

    def _background_load_frames(self, bag, iterator, generation: int) -> None:
        loaded_in_background = 0
        try:
            for msg, bag_time in iterator:
                if generation != self._load_generation or rospy.is_shutdown():
                    return
                with self._lock:
                    self._append_frame(msg, bag_time)
                    loaded_in_background += 1
                    total = len(self.frames)
                    if total <= 5 or total % 100 == 0:
                        rospy.loginfo(
                            "后台已缓存 %d 帧点云 当前帧=%d playing=%s",
                            total,
                            self.current_index,
                            self.playing,
                        )
                    should_publish_state = total <= 5 or total % 100 == 0
                if should_publish_state:
                    with self._lock:
                        self._publish_state()
        except Exception as exc:
            warning = f"后台读取 bag 时遇到异常，已保留前 {len(self.frames)} 帧: {exc}"
            with self._lock:
                if generation == self._load_generation:
                    self.load_warnings.append(warning)
                    self.loading_frames = False
                    self.bag_fully_loaded = True
                    rospy.logwarn(warning)
                    self._publish_state()
            return
        finally:
            try:
                bag.close()
            except Exception:
                pass

        with self._lock:
            if generation == self._load_generation:
                self.loading_frames = False
                self.bag_fully_loaded = True
                rospy.loginfo("后台加载完成，共 %d 帧点云", len(self.frames))
                self._publish_state()

    def _reload_bag(self, bag_path: str, topic: str) -> None:
        old_state = {
            "bag_path": self.bag_path,
            "topic": self.topic,
            "frames": self.frames,
            "frame_times": self.frame_times,
            "playback_times": self.playback_times,
            "current_index": self.current_index,
            "next_due_at": self._next_due_at,
            "playing": self.playing,
            "loading_frames": self.loading_frames,
            "bag_fully_loaded": self.bag_fully_loaded,
            "load_warnings": self.load_warnings,
            "last_stats": self.last_stats,
        }

        try:
            self.bag_path = os.path.abspath(bag_path)
            self.topic = topic
            self.load_warnings = []
            self.playing = False
            self._next_due_at = None
            self._load_frames(lazy=True)
            if not self.frames:
                raise RuntimeError(f"bag 中未找到话题 {self.topic} 的 PointCloud2 消息")
        except Exception:
            self.bag_path = old_state["bag_path"]
            self.topic = old_state["topic"]
            self.frames = old_state["frames"]
            self.frame_times = old_state["frame_times"]
            self.playback_times = old_state["playback_times"]
            self.current_index = old_state["current_index"]
            self._next_due_at = old_state["next_due_at"]
            self.playing = old_state["playing"]
            self.loading_frames = old_state["loading_frames"]
            self.bag_fully_loaded = old_state["bag_fully_loaded"]
            self.load_warnings = old_state["load_warnings"]
            self.last_stats = old_state["last_stats"]
            raise

    def _current_msg(self) -> PointCloud2:
        return self.frames[self.current_index]

    def _current_stamp(self) -> float:
        return self.frame_times[self.current_index]

    def _schedule_next_frame(self) -> None:
        if not self.playing:
            self._next_due_at = None
            return

        next_index = self.current_index + 1
        if next_index >= len(self.frames):
            if self.loading_frames and not self.bag_fully_loaded:
                self._next_due_at = time.monotonic() + 0.05
                return
            if not self.loop:
                self._next_due_at = None
                return
            next_index = 0

        delta = self.playback_times[next_index] - self.playback_times[self.current_index]
        delta = max(1e-3, min(10.0, delta) / self.rate)
        self._next_due_at = time.monotonic() + delta

    def _advance_index(self, step: int) -> bool:
        if not self.frames:
            return False

        total = len(self.frames)
        new_index = self.current_index + int(step)
        if self.loop:
            new_index %= total
        else:
            new_index = max(0, min(total - 1, new_index))
            if new_index == self.current_index and step > 0 and self.current_index == total - 1:
                if self.loading_frames and not self.bag_fully_loaded:
                    return False
                self.playing = False
                self._next_due_at = None
                return False
            if new_index == self.current_index and step < 0 and self.current_index == 0:
                self._next_due_at = None
                return False

        self.current_index = new_index
        return True

    def _apply_filter(self, msg: PointCloud2) -> Dict[str, Any]:
        array = pointcloud2_to_array(msg)
        xyz = extract_xyz(array)
        remove_mask = build_region_mask(xyz, self.regions.values())

        kept_array = array[~remove_mask]
        removed_array = array[remove_mask]
        kept_xyz = xyz[~remove_mask]
        removed_xyz = xyz[remove_mask]

        result = {
            "filtered_msg": subset_pointcloud(msg, kept_array),
            "rejected_msg": subset_pointcloud(msg, removed_array),
            "visual_msg": make_visual_cloud(msg.header, kept_xyz, removed_xyz)
            if self.paint_rejected
            else None,
            "stats": {
                "total_points": int(array.shape[0]),
                "kept_points": int(kept_array.shape[0]),
                "rejected_points": int(removed_array.shape[0]),
                "enabled_regions": sum(1 for region in self.regions.values() if region.enabled),
            },
        }
        return result

    def publish_current_frame(self, reason: str = "manual") -> None:
        with self._lock:
            msg = self._current_msg()
            filtered = self._apply_filter(msg)
            self.filtered_pub.publish(filtered["filtered_msg"])
            if self.publish_rejected:
                self.rejected_pub.publish(filtered["rejected_msg"])
            if self.paint_rejected and filtered["visual_msg"] is not None:
                self.visual_pub.publish(filtered["visual_msg"])
            if self.clock_pub is not None:
                stamp = msg.header.stamp
                if stamp.to_sec() <= 0:
                    stamp = rospy.Time.from_sec(self._current_stamp())
                self.clock_pub.publish(Clock(clock=stamp))
            self.marker_pub.publish(self._build_marker_array(msg.header))

            self.last_stats = {
                "reason": reason,
                "index": self.current_index,
                "stamp": self._current_stamp(),
                **filtered["stats"],
            }
            self._publish_state()

    def _publish_state(self) -> None:
        msg = self._current_msg() if self.frames else None
        stamp = self._current_stamp() if self.frames else 0.0
        frame_id = msg.header.frame_id if msg is not None else ""
        state = {
            "bag": self.bag_path,
            "topic": self.topic,
            "last_command_id": self.last_command_id,
            "playing": self.playing,
            "rate": self.rate,
            "loop": self.loop,
            "paint_rejected": self.paint_rejected,
            "publish_rejected": self.publish_rejected,
            "current_index": self.current_index,
            "total_frames": len(self.frames),
            "loaded_frames": len(self.frames),
            "loading_frames": self.loading_frames,
            "bag_fully_loaded": self.bag_fully_loaded,
            "stamp": stamp,
            "frame_id": frame_id,
            "load_warnings": self.load_warnings,
            "regions": [region.to_dict() for region in self.regions.values()],
            "last_stats": self.last_stats,
            "topics": {
                "source": self.topic,
                "filtered": self.out_topic,
                "rejected": self.rejected_topic,
                "colored": self.visual_topic,
                "visualized": self.visual_topic,
                "fov_regions": self.marker_topic,
                "command": self.command_topic,
                "state": self.state_topic,
            },
        }
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False, sort_keys=True)))

    def _marker_color(self, index: int) -> Tuple[float, float, float]:
        palette = [
            (1.0, 0.22, 0.08),
            (0.1, 0.55, 1.0),
            (0.1, 0.8, 0.35),
            (1.0, 0.72, 0.05),
            (0.75, 0.35, 1.0),
            (0.0, 0.85, 0.85),
        ]
        return palette[index % len(palette)]

    def _sample_horizontal_degrees(self, min_deg: float, max_deg: float, samples: int = 24) -> List[float]:
        h_min = float(min_deg) % 360.0
        h_max = float(max_deg) % 360.0
        span = h_max - h_min if h_min <= h_max else (360.0 - h_min) + h_max
        if abs(span) < 1e-6:
            span = 360.0
        return [(h_min + span * i / max(1, samples - 1)) % 360.0 for i in range(samples)]

    def _fov_point(self, distance_m: float, horizontal_deg: float, vertical_deg: float) -> Point:
        h = math.radians(horizontal_deg)
        v = math.radians(vertical_deg)
        horizontal_radius = distance_m * math.cos(v)
        return Point(
            x=horizontal_radius * math.cos(h),
            y=horizontal_radius * math.sin(h),
            z=distance_m * math.sin(v),
        )

    def _append_segment(self, points: List[Point], start: Point, end: Point) -> None:
        points.append(start)
        points.append(end)

    def _region_marker_points(self, region: FovRegion) -> List[Point]:
        points: List[Point] = []
        d_min = max(0.0, min(region.min_distance_m, region.max_distance_m))
        d_max = max(region.min_distance_m, region.max_distance_m)
        if d_max <= 0.0:
            d_max = 0.05
        v_min = min(region.vertical_min_deg, region.vertical_max_deg)
        v_max = max(region.vertical_min_deg, region.vertical_max_deg)
        h_samples = self._sample_horizontal_degrees(
            region.horizontal_min_deg,
            region.horizontal_max_deg,
        )
        h_edges = [h_samples[0], h_samples[-1]]

        for distance in (d_min, d_max):
            for vertical in (v_min, v_max):
                arc = [self._fov_point(distance, h, vertical) for h in h_samples]
                for left, right in zip(arc, arc[1:]):
                    self._append_segment(points, left, right)

            for horizontal in h_edges:
                self._append_segment(
                    points,
                    self._fov_point(distance, horizontal, v_min),
                    self._fov_point(distance, horizontal, v_max),
                )

        for horizontal in h_edges:
            for vertical in (v_min, v_max):
                self._append_segment(
                    points,
                    self._fov_point(d_min, horizontal, vertical),
                    self._fov_point(d_max, horizontal, vertical),
                )

        return points

    def _build_marker_array(self, header) -> MarkerArray:
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header = header
        clear_marker.ns = "fov_filter_regions"
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for index, region in enumerate(self.regions.values(), start=1):
            if not region.enabled:
                continue

            marker = Marker()
            marker.header = header
            marker.ns = "fov_filter_regions"
            marker.id = index
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.025
            red, green, blue = self._marker_color(index - 1)
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = 0.95
            marker.points = self._region_marker_points(region)
            marker.text = region.name
            marker_array.markers.append(marker)

        return marker_array

    def _load_regions_from_config(self, path: str) -> List[FovRegion]:
        return regions_from_config(load_config(path))

    def _command_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn("无法解析命令 JSON: %s", exc)
            return

        op = str(payload.get("op", "")).strip().lower()
        if not op:
            rospy.logwarn("命令缺少 op 字段")
            return

        try:
            self.last_command_id = payload.get("command_id")
            with self._lock:
                rospy.loginfo(
                    "收到命令 op=%s id=%s index=%d loaded=%d playing=%s loading=%s",
                    op,
                    self.last_command_id,
                    self.current_index,
                    len(self.frames),
                    self.playing,
                    self.loading_frames,
                )
                should_publish = self._handle_command(op, payload)
                if should_publish:
                    self.publish_current_frame(reason=op)
                else:
                    self._publish_state()
        except Exception as exc:
            rospy.logwarn("执行命令 %s 失败: %s", op, exc)

    def _handle_command(self, op: str, payload: Dict[str, Any]) -> bool:
        if op == "play":
            self.playing = True
            self._schedule_next_frame()
            rospy.loginfo(
                "开始播放 index=%d loaded=%d next_due=%s loading=%s",
                self.current_index,
                len(self.frames),
                self._next_due_at,
                self.loading_frames,
            )
            return False

        if op == "pause":
            self.playing = False
            self._next_due_at = None
            return False

        if op == "toggle":
            self.playing = not self.playing
            self._schedule_next_frame()
            return False

        if op in {"next", "forward"}:
            self.playing = False
            self._next_due_at = None
            count = max(1, int(payload.get("count", 1)))
            advanced = self._advance_index(count)
            rospy.loginfo(
                "单步前进 count=%d advanced=%s index=%d loaded=%d loading=%s",
                count,
                advanced,
                self.current_index,
                len(self.frames),
                self.loading_frames,
            )
            return True

        if op in {"prev", "back", "backward"}:
            self.playing = False
            self._next_due_at = None
            count = max(1, int(payload.get("count", 1)))
            self._advance_index(-count)
            return True

        if op == "seek":
            self.playing = False
            self._next_due_at = None
            index = int(payload["index"])
            self.current_index = max(0, min(len(self.frames) - 1, index))
            return True

        if op == "republish":
            return True

        if op == "status":
            return False

        if op == "add_region":
            region = FovRegion.from_mapping(payload["region"])
            self.regions[region.name] = region
            return True

        if op == "update_region":
            name = str(payload["name"])
            if name not in self.regions:
                raise KeyError(f"区域不存在: {name}")
            self.regions[name].update_from_mapping(payload.get("region", {}))
            renamed = self.regions[name].name
            if renamed != name:
                self.regions[renamed] = self.regions.pop(name)
            return True

        if op == "remove_region":
            name = str(payload["name"])
            self.regions.pop(name, None)
            return True

        if op == "clear_regions":
            self.regions.clear()
            return True

        if op == "set_regions":
            new_regions = parse_regions_config(payload.get("regions"))
            self.regions = {region.name: region for region in new_regions}
            return True

        if op == "load_config":
            config_path = str(payload["config_path"])
            new_regions = self._load_regions_from_config(config_path)
            self.regions = {region.name: region for region in new_regions}
            return True

        if op == "load_bag":
            bag_path = str(payload.get("bag") or payload.get("bag_path") or "").strip()
            topic = str(payload.get("topic") or "").strip()
            if not bag_path:
                raise ValueError("load_bag 缺少 bag/bag_path")
            if not topic:
                raise ValueError("load_bag 缺少 topic")
            self._reload_bag(bag_path, topic)
            return True

        if op == "set_option":
            if "rate" in payload and payload["rate"] is not None:
                self.rate = max(1e-3, float(payload["rate"]))
            if "loop" in payload and payload["loop"] is not None:
                self.loop = parse_bool(payload["loop"], default=self.loop)
            if "paint_rejected" in payload and payload["paint_rejected"] is not None:
                self.paint_rejected = parse_bool(
                    payload["paint_rejected"], default=self.paint_rejected
                )
            if "publish_rejected" in payload and payload["publish_rejected"] is not None:
                self.publish_rejected = parse_bool(
                    payload["publish_rejected"], default=self.publish_rejected
                )
            if "playing" in payload and payload["playing"] is not None:
                self.playing = parse_bool(payload["playing"], default=self.playing)
            self._schedule_next_frame()
            return True

        raise ValueError(f"未知命令: {op}")

    def spin(self) -> None:
        while not rospy.is_shutdown():
            should_publish = False
            with self._lock:
                if (
                    self.playing
                    and self._next_due_at is not None
                    and time.monotonic() >= self._next_due_at
                ):
                    should_publish = self._advance_index(1)
                    self._schedule_next_frame()
                    if should_publish:
                        rospy.loginfo(
                            "播放推进到 index=%d loaded=%d next_due=%s loading=%s",
                            self.current_index,
                            len(self.frames),
                            self._next_due_at,
                            self.loading_frames,
                        )
            if should_publish:
                self.publish_current_frame(reason="play")
            # Do not use rospy.Rate here: when /use_sim_time is enabled and
            # /clock is not advancing, rospy.Rate.sleep() can block forever.
            # Playback scheduling already uses wall-clock time.monotonic().
            time.sleep(0.005)
