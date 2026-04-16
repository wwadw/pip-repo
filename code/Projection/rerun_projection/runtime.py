from dataclasses import asdict, dataclass, field
import socket
from typing import Any, Dict, List, Sequence

import numpy as np

from rerun_projection.config import RuntimeConfig, load_yaml_data, resolve_runtime_config
from rerun_projection.models import FrameData
from rerun_projection.projection_core import (
    BagMessage,
    find_nearest_message,
    load_bag_messages,
    pointcloud2_to_xyz,
    project_lidar_points,
    semantic_image_to_array,
)
from rerun_projection.rerun_scene import RerunSceneLogger
from rerun_projection.session import ProjectionSession


def _array_to_json(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def config_to_payload(config: RuntimeConfig) -> Dict[str, Any]:
    return {key: _array_to_json(value) for key, value in asdict(config).items()}


def _blueprint_3d():
    from rerun import blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(origin="world", contents=["world/**", "selection/**", "pairs/**"], name="3D Point Cloud"),
        collapse_panels=True,
    )


def _blueprint_2d():
    from rerun import blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial2DView(origin="world/camera", contents=["world/camera/**", "selection/**", "pairs/**"], name="2D Projection"),
        collapse_panels=True,
    )


def _find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


@dataclass
class ProjectionRuntime:
    config: RuntimeConfig
    session: ProjectionSession
    rerun_grpc_url: str = ""
    rerun_grpc_url_3d: str = ""
    rerun_grpc_url_2d: str = ""
    test_mode: bool = False
    scene_loggers: List[RerunSceneLogger] = field(default_factory=list)
    images: List[BagMessage] = field(default_factory=list)
    clouds: List[BagMessage] = field(default_factory=list)
    cloud_stamps: List[float] = field(default_factory=list)
    current_index: int = 0
    current_frame: FrameData | None = None

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
            rerun_grpc_url_3d="rerun+http://127.0.0.1:9876/proxy",
            rerun_grpc_url_2d="rerun+http://127.0.0.1:9876/proxy",
            test_mode=True,
        )

    def bootstrap_payload(self) -> Dict[str, Any]:
        return {
            "config": config_to_payload(self.config),
            "rerun_grpc_url": self.rerun_grpc_url,
            "rerun_grpc_url_3d": self.rerun_grpc_url_3d or self.rerun_grpc_url,
            "rerun_grpc_url_2d": self.rerun_grpc_url_2d or self.rerun_grpc_url,
            "current_frame": self._frame_payload(),
            "current_selection": self._selection_payload(),
            "locked_pairs": self._locked_pair_payloads(),
        }

    def reload_source(self) -> None:
        if self.test_mode:
            self.session.clear_for_new_source()
            return
        self.images, self.clouds = load_bag_messages(
            self.config.bag_file,
            self.config.image_topic,
            self.config.pointcloud_topic,
        )
        if not self.images or not self.clouds:
            raise RuntimeError(
                f"Bag did not contain both topics: image={self.config.image_topic}, pointcloud={self.config.pointcloud_topic}"
            )
        self.cloud_stamps = [entry.stamp for entry in self.clouds]
        self.current_index = 0
        self.session.clear_for_new_source()
        self._load_current_frame()

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
        if self.current_frame is None:
            self.session.reproject_locked_pairs()
            self._sync_scene()
            return
        self._set_frame_from_arrays(
            frame_index=self.current_frame.frame_index,
            image_stamp=self.current_frame.image_stamp,
            cloud_stamp=self.current_frame.cloud_stamp,
            semantic_image=self.current_frame.semantic_image,
            points_xyz=self.current_frame.points_xyz,
        )

    def next_frame(self) -> None:
        if not self.images:
            return
        self.current_index = (self.current_index + 1) % len(self.images)
        self._load_current_frame()

    def prev_frame(self) -> None:
        if not self.images:
            return
        self.current_index = (self.current_index - 1) % len(self.images)
        self._load_current_frame()

    def set_frame(self, index: int) -> None:
        if not self.images:
            return
        self.current_index = max(0, min(index, len(self.images) - 1))
        self._load_current_frame()

    def select_2d(self, payload: Dict[str, Any]) -> None:
        self.session.select_2d(
            frame_index=int(payload.get("frame_index", self.current_index)),
            pixel=np.asarray(payload["pixel"], dtype=np.float64),
        )
        self._sync_scene()

    def select_3d(self, payload: Dict[str, Any]) -> None:
        position = payload.get("position")
        self.session.select_3d(
            frame_index=int(payload.get("frame_index", self.current_index)),
            instance_id=payload.get("instance_id"),
            position=None if position is None else np.asarray(position, dtype=np.float64),
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

    def _load_current_frame(self) -> None:
        image_entry = self.images[self.current_index]
        cloud_entry = find_nearest_message(self.clouds, self.cloud_stamps, image_entry.stamp)
        semantic_image = semantic_image_to_array(image_entry.msg)
        points_xyz = pointcloud2_to_xyz(cloud_entry.msg)
        self._set_frame_from_arrays(
            frame_index=self.current_index,
            image_stamp=image_entry.stamp,
            cloud_stamp=cloud_entry.stamp,
            semantic_image=semantic_image,
            points_xyz=points_xyz,
        )

    def _set_frame_from_arrays(
        self,
        *,
        frame_index: int,
        image_stamp: float,
        cloud_stamp: float,
        semantic_image: np.ndarray,
        points_xyz: np.ndarray,
    ) -> None:
        projection = project_lidar_points(
            points_xyz=points_xyz,
            image_width=self.config.image_width,
            image_height=self.config.image_height,
            camera_matrix=self.config.camera_matrix,
            distortion_coeffs=self.config.distortion_coeffs,
            lidar_to_camera=self.config.lidar_to_camera,
            min_depth=self.config.min_depth,
        )
        projected_pixels = projection.pixel_coordinates[projection.in_image_mask]
        self.session.projected_pixels = projected_pixels
        self.session.valid_point_indices = projection.valid_point_indices
        self.session.points_xyz = points_xyz
        self.session.camera_points = projection.camera_points
        self.session.reproject_locked_pairs()
        self.current_frame = FrameData(
            frame_index=frame_index,
            image_stamp=image_stamp,
            cloud_stamp=cloud_stamp,
            semantic_image=semantic_image,
            points_xyz=points_xyz,
            projection=projection,
        )
        self._sync_scene()

    def _sync_scene(self) -> None:
        if self.test_mode or self.current_frame is None:
            return
        for logger in self.scene_loggers:
            logger.log_current_state(self.current_frame, self.config)
            logger.log_selection(self.session.current_selection)
            logger.log_locked_pairs(self.session.locked_pairs)

    def _frame_payload(self) -> Dict[str, Any] | None:
        if self.current_frame is None:
            return None
        return {
            "frame_index": self.current_frame.frame_index,
            "image_stamp": self.current_frame.image_stamp,
            "cloud_stamp": self.current_frame.cloud_stamp,
            "image_count": len(self.images),
            "cloud_count": len(self.clouds),
            "visible_points": int(self.current_frame.projection.in_image_mask.sum()),
            "total_points": int(len(self.current_frame.points_xyz)),
        }

    def _selection_payload(self) -> Dict[str, Any] | None:
        selection = self.session.current_selection
        if selection is None:
            return None
        return {
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
    yaml_data = load_yaml_data(yaml_path) if yaml_path else {}
    config = resolve_runtime_config(yaml_data=yaml_data, cli_overrides=overrides)

    import rerun as rr

    recording_3d = rr.RecordingStream("rerun_projection_workbench_3d")
    recording_2d = rr.RecordingStream("rerun_projection_workbench_2d")
    port_3d = _find_free_port()
    port_2d = _find_free_port()
    rerun_url_3d = recording_3d.serve_grpc(grpc_port=port_3d, default_blueprint=_blueprint_3d())
    rerun_url_2d = recording_2d.serve_grpc(grpc_port=port_2d, default_blueprint=_blueprint_2d())
    runtime = ProjectionRuntime(
        config=config,
        session=ProjectionSession.for_test(
            projected_pixels=np.empty((0, 2), dtype=np.float64),
            valid_point_indices=np.empty((0,), dtype=np.int64),
            points_xyz=np.empty((0, 3), dtype=np.float64),
            camera_points=np.empty((0, 3), dtype=np.float64),
        ),
        rerun_grpc_url=rerun_url_3d,
        rerun_grpc_url_3d=rerun_url_3d,
        rerun_grpc_url_2d=rerun_url_2d,
        scene_loggers=[RerunSceneLogger(recording_3d), RerunSceneLogger(recording_2d)],
    )
    runtime.reload_source()
    return runtime
