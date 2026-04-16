from types import SimpleNamespace

import numpy as np

from rerun_projection.rerun_scene import RerunSceneLogger


class FakeRerun:
    def __init__(self):
        self.records = []
        self.times = []

    def log(self, path, value, recording=None):
        del recording
        self.records.append((path, value))

    def set_time(self, timeline, *, sequence=None, recording=None):
        del recording
        self.times.append((timeline, sequence))

    class Points2D:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    class Points3D:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    class Image:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    class Pinhole:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

        @classmethod
        def from_fields(cls, **kwargs):
            return cls(**kwargs)

    class Transform3D:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    class TransformAxes3D:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    class LineStrips3D:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    class EncodedImage:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    class Clear:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    class TransformRelation:
        ChildFromParent = "ChildFromParent"
        ParentFromChild = "ParentFromChild"


def _frame():
    return SimpleNamespace(
        points_xyz=np.array([[1.0, 2.0, 3.0]], dtype=np.float64),
        semantic_image=np.zeros((2, 2), dtype=np.uint8),
        overlay_image=np.zeros((2, 2, 3), dtype=np.uint8),
        projection=SimpleNamespace(
            pixel_coordinates=np.array([[1.0, 1.0]], dtype=np.float64),
            in_image_mask=np.array([True]),
            valid_point_indices=np.array([0], dtype=np.int64),
        ),
    )


def _config():
    return SimpleNamespace(
        lidar_to_camera=np.eye(4, dtype=np.float64),
        camera_matrix=np.eye(3, dtype=np.float64),
        image_width=640,
        image_height=480,
    )


def test_2d_logger_skips_3d_pointcloud(monkeypatch):
    fake_rr = FakeRerun()
    monkeypatch.setitem(__import__("sys").modules, "rerun", fake_rr)

    RerunSceneLogger(view_kind="2d").log_current_state(_frame(), _config())

    assert "world/ego_vehicle/semantic_camera" in [path for path, _ in fake_rr.records]
    assert "world/ego_vehicle/lidar" not in [path for path, _ in fake_rr.records]


def test_nuscenes_style_logger_uses_vehicle_sensor_paths(monkeypatch):
    fake_rr = FakeRerun()
    monkeypatch.setitem(__import__("sys").modules, "rerun", fake_rr)

    RerunSceneLogger(view_kind="both").log_current_state(_frame(), _config())

    paths = [path for path, _ in fake_rr.records]

    assert "world/ego_vehicle/lidar" in paths
    assert "world/ego_vehicle/semantic_camera" in paths
    assert "world/ego_vehicle/overlay_camera" in paths
    assert "world/ego_vehicle/semantic_camera_rig" not in paths
    assert "world/ego_vehicle/overlay_camera_rig" not in paths


def test_camera_entities_hold_image_plane_and_projected_points(monkeypatch):
    fake_rr = FakeRerun()
    monkeypatch.setitem(__import__("sys").modules, "rerun", fake_rr)

    RerunSceneLogger(view_kind="both").log_current_state(_frame(), _config())

    paths = [path for path, _ in fake_rr.records]
    assert "world/ego_vehicle/semantic_camera/projected_points" in paths
    assert "world/ego_vehicle/overlay_camera/projected_points" in paths

    semantic_records = [value for path, value in fake_rr.records if path == "world/ego_vehicle/semantic_camera"]
    overlay_records = [value for path, value in fake_rr.records if path == "world/ego_vehicle/overlay_camera"]

    assert any(isinstance(value, fake_rr.Pinhole) for value in semantic_records)
    assert any(isinstance(value, fake_rr.EncodedImage) for value in semantic_records)
    assert any(isinstance(value, fake_rr.Pinhole) for value in overlay_records)
    assert any(isinstance(value, fake_rr.EncodedImage) for value in overlay_records)


def test_camera_transform_uses_child_from_parent_relation(monkeypatch):
    fake_rr = FakeRerun()
    monkeypatch.setitem(__import__("sys").modules, "rerun", fake_rr)

    config = _config()
    config.lidar_to_camera = np.array(
        [
            [1.0, 0.0, 0.0, 1.5],
            [0.0, 1.0, 0.0, -0.2],
            [0.0, 0.0, 1.0, 0.8],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    RerunSceneLogger(view_kind="both").log_current_state(_frame(), config)

    transform = next(
        value for path, value in fake_rr.records if path == "world/ego_vehicle/semantic_camera" and isinstance(value, fake_rr.Transform3D)
    )

    assert transform.kwargs["relation"] == fake_rr.TransformRelation.ChildFromParent
    assert np.allclose(transform.kwargs["translation"], [1.5, -0.2, 0.8])
