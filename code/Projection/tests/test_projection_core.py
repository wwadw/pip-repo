import numpy as np

from rerun_projection.projection_core import project_lidar_points


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
