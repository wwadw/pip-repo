from types import MethodType, SimpleNamespace

import numpy as np

import rerun_projection.runtime as runtime_module
from rerun_projection.runtime import ProjectionRuntime


def test_apply_projection_updates_runtime_state():
    runtime = ProjectionRuntime.for_test()

    runtime.apply_projection(
        {
            "image_width": 800,
            "image_height": 600,
            "camera_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "distortion_coeffs": [0, 0, 0, 0, 0],
            "lidar_to_camera": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            "min_depth": 0.05,
        }
    )

    assert runtime.bootstrap_payload()["config"]["image_width"] == 800


def test_lock_pair_returns_locked_pair_summary():
    runtime = ProjectionRuntime.for_test()

    runtime.select_2d({"frame_index": 0, "pixel": [10.0, 10.0]})
    runtime.lock_current_pair()

    assert len(runtime.bootstrap_payload()["locked_pairs"]) == 1


def test_apply_source_resets_selection_and_locked_pairs():
    runtime = ProjectionRuntime.for_test()

    runtime.select_2d({"frame_index": 0, "pixel": [10.0, 10.0]})
    runtime.lock_current_pair()
    runtime.apply_source(
        {
            "bag_file": "/bags/other.bag",
            "yaml_path": "",
            "image_topic": "/new/image",
            "overlay_image_topic": "/new/raw",
            "pointcloud_topic": "/new/cloud",
        }
    )

    payload = runtime.bootstrap_payload()
    assert payload["config"]["bag_file"] == "/bags/other.bag"
    assert payload["config"]["overlay_image_topic"] == "/new/raw"
    assert payload["current_selection"] is None
    assert payload["locked_pairs"] == []


def test_next_frame_uses_image_timeline_instead_of_static_image_cache():
    runtime = ProjectionRuntime.for_test()
    runtime.image_stamps = [10.0, 20.0, 30.0]
    seen = []

    def _fake_load_frame(self, index: int) -> None:
        seen.append(index)

    runtime._load_frame_index = MethodType(_fake_load_frame, runtime)

    runtime.next_frame()

    assert seen == [1]


def test_set_frame_clears_selection_and_locked_pairs_before_loading_new_frame():
    runtime = ProjectionRuntime.for_test()
    runtime.image_stamps = [10.0, 20.0]
    runtime.current_frame = SimpleNamespace(
        frame_index=0,
        image_stamp=10.0,
        cloud_stamp=10.0,
        overlay_stamp=10.0,
        semantic_image=np.zeros((2, 2), dtype=np.uint8),
        overlay_image=np.zeros((2, 2, 3), dtype=np.uint8),
        points_xyz=np.zeros((1, 3), dtype=np.float64),
    )
    runtime.select_2d({"frame_index": 0, "pixel": [10.0, 10.0]})
    runtime.lock_current_pair()

    def _fake_load_frame(self, index: int) -> None:
        self.current_index = index

    runtime._load_frame_index = MethodType(_fake_load_frame, runtime)

    runtime.set_frame(1)

    assert runtime.session.current_selection is None
    assert runtime.session.locked_pairs == []
    assert runtime.current_index == 1


def test_bootstrap_payload_exposes_startup_status():
    runtime = ProjectionRuntime.for_test()
    runtime.startup_state = "building"
    runtime.startup_error = None

    payload = runtime.bootstrap_payload()

    assert payload["startup"]["state"] == "building"
    assert payload["startup"]["error"] is None


def test_reload_source_defers_recording_build(monkeypatch):
    runtime = ProjectionRuntime.for_test()
    runtime.test_mode = False
    started = []

    monkeypatch.setattr(runtime_module, "load_topic_stamps", lambda *_args, **_kwargs: [1.0, 2.0])

    def _unexpected_rebuild(self) -> None:
        raise AssertionError("reload_source should not rebuild the recording synchronously")

    def _fake_start_build(self) -> None:
        started.append("background")

    runtime._rebuild_recording = MethodType(_unexpected_rebuild, runtime)
    runtime._start_recording_build = MethodType(_fake_start_build, runtime)

    runtime.reload_source()

    assert started == ["background"]
