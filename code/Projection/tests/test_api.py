from fastapi.testclient import TestClient

from rerun_projection.server import create_app


def test_apply_projection_route_updates_runtime_state():
    app = create_app(test_mode=True)
    client = TestClient(app)

    response = client.post(
        "/api/config/projection",
        json={
            "image_width": 800,
            "image_height": 600,
            "camera_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "distortion_coeffs": [0, 0, 0, 0, 0],
            "lidar_to_camera": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            "min_depth": 0.05,
        },
    )

    assert response.status_code == 200
    assert response.json()["config"]["image_width"] == 800


def test_lock_pair_route_returns_locked_pair_summary():
    app = create_app(test_mode=True)
    client = TestClient(app)

    client.post("/api/select/2d", json={"frame_index": 0, "pixel": [10.0, 10.0]})
    response = client.post("/api/pairs/lock")

    assert response.status_code == 200
    assert len(response.json()["locked_pairs"]) == 1


def test_apply_source_route_resets_selection_and_locked_pairs():
    app = create_app(test_mode=True)
    client = TestClient(app)

    client.post("/api/select/2d", json={"frame_index": 0, "pixel": [10.0, 10.0]})
    client.post("/api/pairs/lock")
    response = client.post(
        "/api/config/source",
        json={
            "bag_file": "/bags/other.bag",
            "yaml_path": "",
            "image_topic": "/new/image",
            "pointcloud_topic": "/new/cloud",
        },
    )

    assert response.status_code == 200
    assert response.json()["config"]["bag_file"] == "/bags/other.bag"
    assert response.json()["current_selection"] is None
    assert response.json()["locked_pairs"] == []
