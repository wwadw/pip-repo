from bisect import bisect_left
from dataclasses import asdict, dataclass, field
import socket
import threading
from typing import Any, Dict, List

import numpy as np

from rerun_projection.config import RuntimeConfig, load_yaml_data, resolve_runtime_config
from rerun_projection.models import FrameData
from rerun_projection.projection_core import (
    BagMessage,
    load_topic_message_by_index,
    load_topic_stamps,
    pointcloud2_to_xyz,
    project_lidar_points,
    ros_image_to_array,
    semantic_image_to_array,
)
from rerun_projection.rerun_scene import (
    OVERLAY_CAMERA_PATH,
    SEMANTIC_CAMERA_PATH,
    RerunSceneLogger,
)
from rerun_projection.session import ProjectionSession


def _array_to_json(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def config_to_payload(config: RuntimeConfig) -> Dict[str, Any]:
    return {key: _array_to_json(value) for key, value in asdict(config).items()}


def _workbench_blueprint():
    from rerun import blueprint as rrb

    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Spatial3DView(
                origin="world",
                contents=[
                    "world/ego_vehicle/lidar",
                    SEMANTIC_CAMERA_PATH,
                    OVERLAY_CAMERA_PATH,
                    "selection/lidar/**",
                    "pairs/lidar/**",
                ],
                name="3D",
            ),
            rrb.Grid(
                rrb.Spatial2DView(
                    origin=SEMANTIC_CAMERA_PATH,
                    contents=["$origin/**", "selection/semantic_camera/**", "pairs/semantic_camera/**"],
                    name="Semantic Camera",
                ),
                rrb.Spatial2DView(
                    origin=OVERLAY_CAMERA_PATH,
                    contents=["$origin/**", "selection/overlay_camera/**", "pairs/overlay_camera/**"],
                    name="Overlay Camera",
                ),
                grid_columns=2,
            ),
            row_shares=[0.7, 0.3],
        ),
        collapse_panels=True,
    )


def _find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _nearest_index(stamps: List[float], target_stamp: float) -> int:
    index = bisect_left(stamps, target_stamp)
    if index <= 0:
        return 0
    if index >= len(stamps):
        return len(stamps) - 1
    previous = stamps[index - 1]
    current = stamps[index]
    return index - 1 if abs(previous - target_stamp) <= abs(current - target_stamp) else index


class _SequentialTopicLoader:
    def __init__(self, bag_path: str, topic_name: str) -> None:
        import rosbag

        self.topic_name = topic_name
        self._bag = rosbag.Bag(bag_path)
        self._iter = iter(self._bag.read_messages(topics=[topic_name]))
        self._cache: Dict[int, BagMessage] = {}
        self._current_index = -1

    def get(self, index: int) -> BagMessage:
        if index < 0:
            raise IndexError(f"Negative index requested for topic {self.topic_name}: {index}")
        while self._current_index < index:
            _, msg, stamp = next(self._iter)
            self._current_index += 1
            self._cache[self._current_index] = BagMessage(stamp=float(msg.header.stamp.to_sec()) if getattr(msg, "_has_header", False) else float(stamp.to_sec()), msg=msg)
            if len(self._cache) > 32:
                self._cache.pop(next(iter(self._cache)))
        if index not in self._cache:
            raise RuntimeError(f"Failed to cache topic {self.topic_name} at index {index}.")
        return self._cache[index]

    def close(self) -> None:
        self._bag.close()


@dataclass
class ProjectionRuntime:
    config: RuntimeConfig
    session: ProjectionSession
    rerun_grpc_url: str = ""
    test_mode: bool = False
    startup_state: str = "idle"
    startup_error: str | None = None
    recording: object | None = None
    scene_loggers: List[RerunSceneLogger] = field(default_factory=list)
    frame_stamps: List[float] = field(default_factory=list)
    image_stamps: List[float] = field(default_factory=list)
    overlay_stamps: List[float] = field(default_factory=list)
    cloud_stamps: List[float] = field(default_factory=list)
    current_index: int = 0
    current_frame: FrameData | None = None
    _image_cache: Dict[int, BagMessage] = field(default_factory=dict)
    _overlay_cache: Dict[int, BagMessage] = field(default_factory=dict)
    _cloud_cache: Dict[int, BagMessage] = field(default_factory=dict)
    _startup_thread: threading.Thread | None = field(default=None, repr=False)
    _startup_generation: int = 0

    @classmethod
    def for_test(cls) -> "ProjectionRuntime":
        config = resolve_runtime_config(
            yaml_data={},
            cli_overrides={
                "bag_file": "/tmp/test.bag",
                "camera_matrix": np.eye(3, dtype=np.float64),
                "lidar_to_camera": np.eye(4, dtype=np.float64),
            },
        )
        session = ProjectionSession.for_test(
            projected_pixels=np.array([[10.0, 10.0], [30.0, 30.0]], dtype=np.float64),
            valid_point_indices=np.array([0, 1], dtype=np.int64),
            points_xyz=np.array([[1.0, 0.0, 1.0], [2.0, 0.0, 2.0]], dtype=np.float64),
            camera_points=np.array([[1.0, 0.0, 1.0], [2.0, 0.0, 2.0]], dtype=np.float64),
        )
        return cls(
            config=config,
            session=session,
            rerun_grpc_url="rerun+http://127.0.0.1:9876/proxy",
            test_mode=True,
            startup_state="ready",
        )

    def bootstrap_payload(self) -> Dict[str, Any]:
        return {
            "config": config_to_payload(self.config),
            "rerun_grpc_url": self.rerun_grpc_url,
            "startup": {
                "state": self.startup_state,
                "error": self.startup_error,
            },
            "current_frame": self._frame_payload(),
            "current_selection": self._selection_payload(),
            "locked_pairs": self._locked_pair_payloads(),
        }

    def reload_source(self) -> None:
        if self.test_mode:
            self.session.clear_for_new_source()
            self.startup_state = "ready"
            self.startup_error = None
            return
        self.image_stamps = load_topic_stamps(self.config.bag_file, self.config.image_topic)
        self.overlay_stamps = load_topic_stamps(self.config.bag_file, self.config.overlay_image_topic)
        self.cloud_stamps = load_topic_stamps(self.config.bag_file, self.config.pointcloud_topic)
        if not self.image_stamps:
            raise RuntimeError(f"Bag did not contain image topic: {self.config.image_topic}")
        if not self.cloud_stamps:
            raise RuntimeError(f"Bag did not contain pointcloud topic: {self.config.pointcloud_topic}")
        self.frame_stamps = list(self.cloud_stamps)
        self._clear_caches()
        self.current_index = 0
        self.current_frame = None
        self.session.clear_for_new_source()
        self.startup_state = "building"
        self.startup_error = None
        self._startup_generation += 1
        if self.recording is not None:
            self.recording.disconnect()
        self.recording = None
        self.rerun_grpc_url = ""
        self.scene_loggers = []
        self._start_recording_build()

    def apply_source(self, payload: Dict[str, Any]) -> None:
        values = config_to_payload(self.config)
        values.update(payload)
        yaml_path = str(values.get("yaml_path", ""))
        yaml_data = load_yaml_data(yaml_path) if yaml_path else {}
        self.config = resolve_runtime_config(yaml_data=yaml_data, cli_overrides=values)
        self.reload_source()

    def apply_projection(self, payload: Dict[str, Any]) -> None:
        values = config_to_payload(self.config)
        values.update(payload)
        self.config = RuntimeConfig(**values)
        if self.test_mode:
            if self.current_frame is None:
                self.session.reproject_locked_pairs()
                return
            self._apply_frame(self._frame_from_arrays(
                frame_index=self.current_frame.frame_index,
                image_stamp=self.current_frame.image_stamp,
                cloud_stamp=self.current_frame.cloud_stamp,
                overlay_stamp=self.current_frame.overlay_stamp,
                semantic_image=self.current_frame.semantic_image,
                overlay_image=self.current_frame.overlay_image,
                points_xyz=self.current_frame.points_xyz,
            ))
            return
        target_index = min(self.current_index, max(self._frame_count() - 1, 0))
        recording, rerun_grpc_url, scene_loggers = self._rebuild_recording()
        if self.recording is not None:
            self.recording.disconnect()
        self.recording = recording
        self.rerun_grpc_url = rerun_grpc_url
        self.scene_loggers = scene_loggers
        self._load_frame_index(target_index)
        self.startup_state = "ready"
        self.startup_error = None

    def next_frame(self) -> None:
        frame_count = self._frame_count()
        if frame_count == 0:
            return
        self.set_frame((self.current_index + 1) % frame_count)

    def prev_frame(self) -> None:
        frame_count = self._frame_count()
        if frame_count == 0:
            return
        self.set_frame((self.current_index - 1) % frame_count)

    def set_frame(self, index: int) -> None:
        frame_count = self._frame_count()
        if frame_count == 0:
            return
        self.session.clear_for_new_frame()
        self._load_frame_index(max(0, min(index, frame_count - 1)))

    def select_2d(self, payload: Dict[str, Any]) -> None:
        frame_index = int(payload.get("frame_index", self.current_index))
        if frame_index != self.current_index:
            self._load_frame_index(frame_index)
        self.session.select_2d(frame_index=frame_index, pixel=np.asarray(payload["pixel"], dtype=np.float64))
        self._sync_scene()

    def select_3d(self, payload: Dict[str, Any]) -> None:
        frame_index = int(payload.get("frame_index", self.current_index))
        if frame_index != self.current_index:
            self._load_frame_index(frame_index)
        position = payload.get("position")
        position_array = None if position is None else np.asarray(position, dtype=np.float64)
        entity_path = str(payload.get("entity_path", ""))
        if entity_path in {f"{SEMANTIC_CAMERA_PATH}/projected_points", f"{OVERLAY_CAMERA_PATH}/projected_points"}:
            self.session.select_projected_point(
                frame_index=frame_index,
                instance_id=payload.get("instance_id"),
                position=position_array,
            )
        else:
            self.session.select_3d(
                frame_index=frame_index,
                instance_id=payload.get("instance_id"),
                position=position_array,
            )
        self._sync_scene()

    def lock_current_pair(self) -> None:
        self.session.lock_current_pair()
        self._sync_scene()

    def delete_last_pair(self) -> None:
        self.session.delete_last_pair()
        self._sync_scene()

    def clear_pairs(self) -> None:
        self.session.clear_pairs()
        self._sync_scene()

    def _frame_count(self) -> int:
        return len(self.frame_stamps) or len(self.image_stamps)

    def _clear_caches(self) -> None:
        self._image_cache.clear()
        self._overlay_cache.clear()
        self._cloud_cache.clear()

    def _start_recording_build(self) -> None:
        generation = self._startup_generation
        self._startup_thread = threading.Thread(
            target=self._finish_background_startup,
            args=(generation,),
            daemon=True,
            name="rerun-recording-build",
        )
        self._startup_thread.start()

    def _finish_background_startup(self, generation: int) -> None:
        recording = None
        try:
            recording, rerun_grpc_url, scene_loggers = self._rebuild_recording()
            frame = self._compose_frame(0)
        except Exception as exc:
            if recording is not None:
                try:
                    recording.disconnect()
                except Exception:
                    pass
            if generation != self._startup_generation:
                return
            self.startup_state = "error"
            self.startup_error = f"{type(exc).__name__}: {exc}"
            self.rerun_grpc_url = ""
            self.recording = None
            self.scene_loggers = []
            self.current_frame = None
            return

        if generation != self._startup_generation:
            recording.disconnect()
            return

        self.recording = recording
        self.rerun_grpc_url = rerun_grpc_url
        self.scene_loggers = scene_loggers
        self.current_index = 0
        self._apply_frame(frame)
        self.startup_state = "ready"
        self.startup_error = None

    def _rebuild_recording(self) -> tuple[object, str, List[RerunSceneLogger]]:
        import rerun as rr

        recording = rr.RecordingStream("rerun_projection_workbench")
        port = _find_free_port()
        rerun_grpc_url = recording.serve_grpc(grpc_port=port, default_blueprint=_workbench_blueprint())
        scene_loggers = [RerunSceneLogger(recording, view_kind="both")]

        cloud_loader = _SequentialTopicLoader(self.config.bag_file, self.config.pointcloud_topic)
        image_loader = _SequentialTopicLoader(self.config.bag_file, self.config.image_topic)
        overlay_loader = _SequentialTopicLoader(self.config.bag_file, self.config.overlay_image_topic) if self.overlay_stamps else None
        try:
            for frame_index in range(self._frame_count()):
                cloud_entry = cloud_loader.get(frame_index)
                image_entry = image_loader.get(_nearest_index(self.image_stamps, cloud_entry.stamp))
                overlay_entry = overlay_loader.get(_nearest_index(self.overlay_stamps, cloud_entry.stamp)) if overlay_loader else None
                frame = self._frame_from_entries(frame_index, image_entry, overlay_entry, cloud_entry)
                for logger in scene_loggers:
                    logger.log_current_state(frame, self.config, frame_index=frame_index)
        finally:
            cloud_loader.close()
            image_loader.close()
            if overlay_loader is not None:
                overlay_loader.close()
        rr.reset_time(recording=recording)
        return recording, rerun_grpc_url, scene_loggers

    def _load_frame_index(self, index: int) -> None:
        self.current_index = index
        self.session.clear_for_new_frame()
        self._apply_frame(self._compose_frame(index))

    def _compose_frame(self, index: int) -> FrameData:
        cloud_entry = self._load_cached_entry(self.config.pointcloud_topic, index, self._cloud_cache)
        image_entry = self._load_nearest_entry(
            self.config.image_topic,
            self.image_stamps,
            cloud_entry.stamp,
            self._image_cache,
        )
        if image_entry is None:
            raise RuntimeError(f"Bag did not contain image topic near frame stamp: {self.config.image_topic}")
        overlay_entry = self._load_nearest_entry(
            self.config.overlay_image_topic,
            self.overlay_stamps,
            cloud_entry.stamp,
            self._overlay_cache,
        )
        return self._frame_from_entries(index, image_entry, overlay_entry, cloud_entry)

    def _frame_from_entries(
        self,
        frame_index: int,
        image_entry: BagMessage,
        overlay_entry: BagMessage | None,
        cloud_entry: BagMessage,
    ) -> FrameData:
        semantic_image = semantic_image_to_array(image_entry.msg)
        overlay_image = ros_image_to_array(overlay_entry.msg) if overlay_entry is not None else semantic_image
        points_xyz = pointcloud2_to_xyz(cloud_entry.msg)
        return self._frame_from_arrays(
            frame_index=frame_index,
            image_stamp=image_entry.stamp,
            cloud_stamp=cloud_entry.stamp,
            overlay_stamp=None if overlay_entry is None else overlay_entry.stamp,
            semantic_image=semantic_image,
            overlay_image=overlay_image,
            points_xyz=points_xyz,
        )

    def _frame_from_arrays(
        self,
        *,
        frame_index: int,
        image_stamp: float,
        cloud_stamp: float,
        overlay_stamp: float | None,
        semantic_image: np.ndarray,
        overlay_image: np.ndarray,
        points_xyz: np.ndarray,
    ) -> FrameData:
        projection = project_lidar_points(
            points_xyz=points_xyz,
            image_width=self.config.image_width,
            image_height=self.config.image_height,
            camera_matrix=self.config.camera_matrix,
            distortion_coeffs=self.config.distortion_coeffs,
            lidar_to_camera=self.config.lidar_to_camera,
            min_depth=self.config.min_depth,
        )
        return FrameData(
            frame_index=frame_index,
            image_stamp=image_stamp,
            cloud_stamp=cloud_stamp,
            overlay_stamp=overlay_stamp,
            semantic_image=semantic_image,
            overlay_image=overlay_image,
            points_xyz=points_xyz,
            projection=projection,
        )

    def _apply_frame(self, frame: FrameData) -> None:
        projected_pixels = frame.projection.pixel_coordinates[frame.projection.in_image_mask]
        self.session.projected_pixels = projected_pixels
        self.session.valid_point_indices = frame.projection.valid_point_indices
        self.session.points_xyz = frame.points_xyz
        self.session.camera_points = frame.projection.camera_points
        self.session.reproject_locked_pairs()
        self.current_frame = frame

    def _load_cached_entry(self, topic: str, index: int, cache: Dict[int, BagMessage]) -> BagMessage:
        if index not in cache:
            entry = load_topic_message_by_index(self.config.bag_file, topic, index)
            if entry is None:
                raise RuntimeError(f"Bag did not contain {topic} at index {index}.")
            cache[index] = entry
            while len(cache) > 16:
                cache.pop(next(iter(cache)))
        return cache[index]

    def _load_nearest_entry(
        self,
        topic: str,
        stamps: List[float],
        target_stamp: float,
        cache: Dict[int, BagMessage],
    ) -> BagMessage | None:
        if not topic or not stamps:
            return None
        return self._load_cached_entry(topic, _nearest_index(stamps, target_stamp), cache)

    def _sync_scene(self) -> None:
        if self.test_mode or self.current_frame is None:
            return
        for logger in self.scene_loggers:
            logger.clear_interactions(self.current_index)
            logger.log_selection(self.session.current_selection, frame_index=self.current_index)
            logger.log_locked_pairs(self.session.locked_pairs, frame_index=self.current_index)

    def _frame_payload(self) -> Dict[str, Any] | None:
        if self.current_frame is None:
            return None
        return {
            "frame_index": self.current_frame.frame_index,
            "image_stamp": self.current_frame.image_stamp,
            "cloud_stamp": self.current_frame.cloud_stamp,
            "overlay_stamp": self.current_frame.overlay_stamp,
            "image_count": self._frame_count(),
            "cloud_count": len(self.cloud_stamps),
            "visible_points": int(self.current_frame.projection.in_image_mask.sum()),
            "total_points": int(len(self.current_frame.points_xyz)),
        }

    def _selection_payload(self) -> Dict[str, Any] | None:
        selection = self.session.current_selection
        if selection is None:
            return None
        return {
            "frame_index": selection.frame_index,
            "source_view": selection.source_view,
            "clicked_pixel": None if selection.clicked_pixel is None else selection.clicked_pixel.tolist(),
            "matched_pixel": selection.matched_pixel.tolist(),
            "point_index": selection.point_index,
            "point_xyz": selection.point_xyz.tolist(),
            "depth": selection.depth,
            "pixel_error": selection.pixel_error,
        }

    def _locked_pair_payloads(self) -> list[Dict[str, Any]]:
        return [
            {
                "pair_id": pair.pair_id,
                "frame_index": pair.frame_index,
                "source_view": pair.source_view,
                "point_index": pair.point_index,
                "point_xyz": pair.point_xyz.tolist(),
                "clicked_pixel": None if pair.clicked_pixel is None else pair.clicked_pixel.tolist(),
                "projected_pixel": pair.projected_pixel.tolist(),
                "depth": pair.depth,
                "color_rgb": list(pair.color_rgb),
            }
            for pair in self.session.locked_pairs
        ]


def build_runtime(*, test_mode: bool = False, cli_overrides: Dict[str, Any] | None = None) -> ProjectionRuntime:
    if test_mode:
        return ProjectionRuntime.for_test()

    overrides = dict(cli_overrides or {})
    yaml_path = str(overrides.get("yaml_path") or "")
    if not yaml_path:
        yaml_path = resolve_runtime_config().yaml_path
        overrides["yaml_path"] = yaml_path
    yaml_data = load_yaml_data(yaml_path) if yaml_path else {}
    config = resolve_runtime_config(yaml_data=yaml_data, cli_overrides=overrides)

    runtime = ProjectionRuntime(
        config=config,
        session=ProjectionSession.for_test(
            projected_pixels=np.empty((0, 2), dtype=np.float64),
            valid_point_indices=np.empty((0,), dtype=np.int64),
            points_xyz=np.empty((0, 3), dtype=np.float64),
            camera_points=np.empty((0, 3), dtype=np.float64),
        ),
    )
    runtime.reload_source()
    return runtime
