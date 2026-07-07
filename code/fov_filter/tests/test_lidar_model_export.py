import unittest

import yaml

from fov_filter.control import build_parser
from fov_filter.config_io import dump_filter_regions_yaml
from fov_filter.types import FovRegion


class LidarModelExportTest(unittest.TestCase):
    def test_rshelios_export_converts_pointcloud_horizontal_angles_to_driver_angles(self):
        regions = [
            FovRegion(
                name="front_right",
                horizontal_min_deg=330.0,
                horizontal_max_deg=340.0,
                vertical_min_deg=-10.0,
                vertical_max_deg=10.0,
                min_distance_m=0.0,
                max_distance_m=2.0,
            )
        ]

        content = dump_filter_regions_yaml(regions, lidar_model="rshelios")
        exported = yaml.safe_load(content)

        self.assertEqual(
            exported["filter_regions"],
            [
                {
                    "min_horiz_deg": 20.0,
                    "max_horiz_deg": 30.0,
                    "min_vert_deg": -10.0,
                    "max_vert_deg": 10.0,
                    "min_dist_m": 0.0,
                    "max_dist_m": 2.0,
                }
            ],
        )
        self.assertEqual(
            content,
            "filter_regions:\n"
            "        - min_horiz_deg: 20.0\n"
            "          max_horiz_deg: 30.0\n"
            "          min_vert_deg: -10.0\n"
            "          max_vert_deg: 10.0\n"
            "          min_dist_m: 0.0\n"
            "          max_dist_m: 2.0\n",
        )

    def test_default_export_keeps_pointcloud_horizontal_angles_unchanged(self):
        regions = [
            FovRegion(
                name="front_left",
                horizontal_min_deg=20.0,
                horizontal_max_deg=30.0,
                vertical_min_deg=-8.0,
                vertical_max_deg=12.0,
                min_distance_m=0.5,
                max_distance_m=1.5,
            )
        ]

        exported = yaml.safe_load(dump_filter_regions_yaml(regions))

        self.assertEqual(exported["filter_regions"][0]["min_horiz_deg"], 20.0)
        self.assertEqual(exported["filter_regions"][0]["max_horiz_deg"], 30.0)

    def test_control_export_config_accepts_lidar_model_option(self):
        args = build_parser().parse_args(
            ["export-config", "/tmp/filter_regions.yaml", "--lidar-model", "rshelios"]
        )

        self.assertEqual(args.subcommand, "export-config")
        self.assertEqual(args.lidar_model, "rshelios")


if __name__ == "__main__":
    unittest.main()
