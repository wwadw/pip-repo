import numpy as np

from rerun_projection.config import resolve_runtime_config
from rerun_projection.models import LockedPair
from rerun_projection.rerun_scene import build_locked_pair_payloads
from rerun_projection.session import ProjectionSession


def test_cli_values_override_yaml_values():
    yaml_dict = {
        "semantic_image_topic": "/yaml/image",
        "pointcloud_topic": "/yaml/cloud",
    }

    config = resolve_runtime_config(
        yaml_data=yaml_dict,
        cli_overrides={"bag_file": "/bags/demo.bag", "image_topic": "/cli/image"},
    )

    assert config.bag_file == "/bags/demo.bag"
    assert config.image_topic == "/cli/image"
    assert config.pointcloud_topic == "/yaml/cloud"


def test_default_config_matches_legacy_projection_command():
    config = resolve_runtime_config()

    assert config.yaml_path == "/home/ww/elevation_ws/src/elevation_mapping/elevation_mapping_demos/config/robots/minimal_semantic_robot.yaml"
    assert config.bag_file == "/home/ww/bags/b2biaopin/louti/2026-04-14-20-18-10_semantic.bag"
    assert config.pointcloud_topic == "/mfla/frame_cloud"
    assert config.image_topic == "/usb_cam/image_semantic_id"
    assert config.overlay_image_topic == "/usb_cam/image_raw"


def test_select_2d_matches_nearest_projected_point():
    session = ProjectionSession.for_test(
        projected_pixels=np.array([[10.0, 10.0], [30.0, 30.0]], dtype=np.float64),
        valid_point_indices=np.array([3, 7], dtype=np.int64),
        points_xyz=np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 2.0, 3.0],
                [4.0, 5.0, 6.0],
                [7.0, 8.0, 9.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [3.0, 3.0, 3.0],
            ],
            dtype=np.float64,
        ),
        camera_points=np.array(
            [
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.1, 0.2, 1.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.3, 0.3, 2.0],
            ],
            dtype=np.float64,
        ),
    )

    selection = session.select_2d(frame_index=0, pixel=np.array([11.0, 9.0], dtype=np.float64))

    assert selection.point_index == 3
    assert selection.matched_pixel.tolist() == [10.0, 10.0]
    assert round(selection.pixel_error, 4) == round(2 ** 0.5, 4)


def test_apply_projection_keeps_locked_pairs_and_updates_pixels():
    session = ProjectionSession.for_test(
        projected_pixels=np.array([[10.0, 10.0]], dtype=np.float64),
        valid_point_indices=np.array([1], dtype=np.int64),
        points_xyz=np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]], dtype=np.float64),
        camera_points=np.array([[0.0, 0.0, 0.0], [0.1, 0.2, 1.0]], dtype=np.float64),
    )
    session.current_selection = session.select_2d(frame_index=0, pixel=np.array([10.0, 10.0], dtype=np.float64))
    session.lock_current_pair()

    session.replace_projection_for_test(
        projected_pixels=np.array([[12.0, 14.0]], dtype=np.float64),
        valid_point_indices=np.array([1], dtype=np.int64),
        camera_points=np.array([[0.0, 0.0, 0.0], [0.1, 0.2, 1.0]], dtype=np.float64),
    )
    session.reproject_locked_pairs()

    assert session.locked_pairs[0].projected_pixel.tolist() == [12.0, 14.0]


def test_build_locked_pair_payloads_keeps_keypoint_ids():
    pair = LockedPair(
        pair_id=2,
        frame_index=0,
        source_view="2d",
        point_index=7,
        point_xyz=np.array([1.0, 2.0, 3.0], dtype=np.float64),
        clicked_pixel=np.array([10.0, 10.0], dtype=np.float64),
        projected_pixel=np.array([12.0, 14.0], dtype=np.float64),
        depth=3.0,
        color_rgb=(255, 0, 0),
    )

    payload = build_locked_pair_payloads([pair])

    assert payload["keypoint_ids"] == [2]
    assert payload["positions2d"] == [[12.0, 14.0]]
    assert payload["positions3d"] == [[1.0, 2.0, 3.0]]
