import numpy as np

from rerun_projection.projection_core import (
    load_initial_aligned_messages,
    load_topic_message_by_index,
    project_lidar_points,
)


class _Stamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


class _Msg:
    _has_header = False

    def __init__(self, name):
        self.name = name


class _FakeBag:
    def __init__(self, messages):
        self.messages = messages

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return None

    def read_messages(self, topics, **kwargs):
        del kwargs
        topic_set = set(topics)
        for topic, stamp, name in self.messages:
            if topic in topic_set:
                yield topic, _Msg(name), _Stamp(stamp)


def test_project_lidar_points_returns_visible_index_mapping():
    camera_matrix = np.array(
        [[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    points = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.1, 0.0, 1.0],
            [0.0, 0.0, -1.0],
        ],
        dtype=np.float64,
    )

    result = project_lidar_points(
        points_xyz=points,
        image_width=100,
        image_height=100,
        camera_matrix=camera_matrix,
        distortion_coeffs=np.zeros(5, dtype=np.float64),
        lidar_to_camera=np.eye(4, dtype=np.float64),
        min_depth=0.05,
    )

    assert result.valid_point_indices.tolist() == [0, 1]
    assert result.pixel_coordinates.shape == (2, 2)
    assert result.in_image_mask.tolist() == [True, True]


def test_load_initial_aligned_messages_keeps_only_nearest_start_frame(monkeypatch):
    messages = [
        ("/cloud", 9.0, "cloud-before"),
        ("/overlay", 8.0, "overlay-before"),
        ("/image", 10.0, "image"),
        ("/overlay", 11.0, "overlay-after"),
        ("/cloud", 12.0, "cloud-after"),
        ("/image", 20.0, "later-image"),
    ]
    fake_rosbag = type("FakeRosbagModule", (), {"Bag": lambda self, path: _FakeBag(messages)})()
    monkeypatch.setitem(__import__("sys").modules, "rosbag", fake_rosbag)

    image, overlay, cloud = load_initial_aligned_messages(
        "/tmp/test.bag",
        image_topic="/image",
        overlay_image_topic="/overlay",
        pointcloud_topic="/cloud",
    )

    assert image.msg.name == "image"
    assert overlay.msg.name == "overlay-after"
    assert cloud.msg.name == "cloud-before"


def test_load_topic_message_by_index_returns_requested_frame(monkeypatch):
    messages = [
        ("/image", 10.0, "image-0"),
        ("/image", 20.0, "image-1"),
        ("/image", 30.0, "image-2"),
    ]
    fake_rosbag = type("FakeRosbagModule", (), {"Bag": lambda self, path: _FakeBag(messages)})()
    monkeypatch.setitem(__import__("sys").modules, "rosbag", fake_rosbag)

    entry = load_topic_message_by_index("/tmp/test.bag", "/image", 1)

    assert entry is not None
    assert entry.msg.name == "image-1"
