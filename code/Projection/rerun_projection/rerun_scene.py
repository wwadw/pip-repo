from typing import Dict, Iterable, List, Optional


def build_locked_pair_payloads(locked_pairs: Iterable[object]) -> Dict[str, List[object]]:
    pairs = list(locked_pairs)
    return {
        "keypoint_ids": [pair.pair_id for pair in pairs],
        "positions2d": [pair.projected_pixel.tolist() for pair in pairs],
        "positions3d": [pair.point_xyz.tolist() for pair in pairs],
        "colors": [list(pair.color_rgb) for pair in pairs],
        "labels": [f"P{pair.pair_id}" for pair in pairs],
    }


class RerunSceneLogger:
    def __init__(self, recording: Optional[object] = None) -> None:
        self.recording = recording

    def log_current_state(self, frame, config) -> None:
        import rerun as rr

        rr.log("world/points", rr.Points3D(frame.points_xyz), recording=self.recording)
        rr.log(
            "world/camera",
            rr.Transform3D(
                translation=config.lidar_to_camera[:3, 3],
                mat3x3=config.lidar_to_camera[:3, :3],
                from_parent=False,
            ),
            recording=self.recording,
        )
        rr.log(
            "world/camera",
            rr.Pinhole(
                image_from_camera=config.camera_matrix,
                resolution=[config.image_width, config.image_height],
            ),
            recording=self.recording,
        )
        rr.log("world/camera/image", rr.Image(frame.semantic_image), recording=self.recording)
        valid_pixels = frame.projection.pixel_coordinates[frame.projection.in_image_mask]
        rr.log(
            "world/camera/projected_points",
            rr.Points2D(valid_pixels.tolist() if len(valid_pixels) else []),
            recording=self.recording,
        )

    def log_selection(self, selection) -> None:
        if selection is None:
            return
        import rerun as rr

        rr.log(
            "selection/current_3d_match",
            rr.Points3D([selection.point_xyz.tolist()]),
            recording=self.recording,
        )
        rr.log(
            "selection/current_2d_match",
            rr.Points2D([selection.matched_pixel.tolist()]),
            recording=self.recording,
        )

        if selection.clicked_pixel is not None:
            rr.log(
                "selection/current_2d_click",
                rr.Points2D([selection.clicked_pixel.tolist()]),
                recording=self.recording,
            )

    def log_locked_pairs(self, locked_pairs: Iterable[object]) -> None:
        import rerun as rr

        payload = build_locked_pair_payloads(locked_pairs)
        rr.log(
            "pairs/locked/2d",
            rr.Points2D(
                payload["positions2d"],
                colors=payload["colors"],
                labels=payload["labels"],
                keypoint_ids=payload["keypoint_ids"],
            ),
            recording=self.recording,
        )
        rr.log(
            "pairs/locked/3d",
            rr.Points3D(
                payload["positions3d"],
                colors=payload["colors"],
                labels=payload["labels"],
                keypoint_ids=payload["keypoint_ids"],
            ),
            recording=self.recording,
        )
