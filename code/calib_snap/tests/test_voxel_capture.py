import io
import sys
import threading
import unittest
from unittest import mock

import test_dense_capture as fixtures
from test_dense_capture import DEFAULT_FIELDS, DenseTestCase, cloud


class VoxelArgumentTests(DenseTestCase):
    def test_dense_accepts_voxel_size_and_defaults_to_disabled(self):
        base = ["--capture-mode", "dense", "--pointcloud-topic", "/lidar",
                "--accumulate-frames", "2"]
        for extra, expected in [([], 0.0), (["--voxel-size", "0"], 0.0),
                                (["--voxel-size", "0.005"], 0.005)]:
            with self.subTest(extra=extra), \
                    mock.patch("sys.stderr", new_callable=io.StringIO):
                try:
                    args = self.parse(*base, *extra)
                except SystemExit:
                    self.fail("Dense mode must accept --voxel-size")
                self.assertEqual(getattr(args, "voxel_size", None), expected)

    def test_invalid_voxel_sizes_are_rejected_with_the_parameter_name(self):
        for value in ["-0.1", "nan", "inf", "-inf"]:
            with self.subTest(value=value), \
                    mock.patch("sys.stderr", new_callable=io.StringIO) as error:
                with self.assertRaises(SystemExit):
                    self.parse("--capture-mode", "dense", "--pointcloud-topic", "/lidar",
                               "--accumulate-frames", "2", "--voxel-size", value)
                self.assertIn("--voxel-size", error.getvalue())

    def test_voxel_size_requires_dense_mode_even_when_zero(self):
        for mode in ["auto", "paired", "image"]:
            with self.subTest(mode=mode), \
                    mock.patch("sys.stderr", new_callable=io.StringIO) as error:
                with self.assertRaises(SystemExit):
                    self.parse("--capture-mode", mode, "--pointcloud-topic", "/lidar",
                               "--voxel-size", "0")
                self.assertIn("--capture-mode", error.getvalue())


class VoxelPcdTests(DenseTestCase):
    def save(self, messages, fields=None, voxel_size=1.0):
        output = self.root / "voxel.pcd"
        try:
            count, names = self.writer()(
                messages, output, fields or ["x", "y", "z", "intensity"],
                voxel_size=voxel_size,
            )
        except TypeError as exc:
            if "voxel_size" in str(exc):
                self.fail("The dense writer must support voxel downsampling")
            raise
        text = output.read_text()
        rows = [line.split() for line in text.split("DATA ascii\n")[1].splitlines()]
        self.assertEqual(len(rows), count)
        self.assertIn("WIDTH {}\nHEIGHT 1\n".format(count), text)
        self.assertIn("POINTS {}\n".format(count), text)
        return text, names, rows

    def test_voxels_merge_across_frames_and_use_centroids(self):
        _, names, rows = self.save([
            cloud(points=((0.125, 0.125, 0.125, 10), (1.125, 0.125, 0.125, 30))),
            cloud(points=((0.375, 0.375, 0.375, 20), (0.625, 0.625, 0.625, 40))),
        ])
        self.assertEqual(names, ["x", "y", "z", "intensity"])
        self.assertEqual(rows, [
            ["0.375", "0.375", "0.375", "10.0"],
            ["1.125", "0.125", "0.125", "30.0"],
        ])

    def test_voxel_indices_use_floor_and_include_exact_lower_boundaries(self):
        _, _, rows = self.save([cloud(points=(
            (-0.125, 0, 0, 1), (-0.375, 0, 0, 2), (-1.0, 0, 0, 3),
            (0.125, 0, 0, 4), (0.375, 0, 0, 5), (1.0, 0, 0, 6),
        ))])
        self.assertEqual([float(row[0]) for row in rows], [-0.5, 0.25, 1.0])

    def test_distinct_y_and_z_voxels_remain_separate(self):
        _, _, rows = self.save([cloud(points=(
            (0.125, 0.125, 0.125, 1), (0.125, 1.125, 0.125, 2),
            (0.125, 0.125, 1.125, 3),
        ))])
        self.assertEqual(len(rows), 3)
        self.assertEqual([float(row[3]) for row in rows], [1, 2, 3])

    def test_large_float64_coordinates_keep_a_finite_centroid(self):
        fields = (("x", 0, 8, 1), ("y", 8, 8, 1), ("z", 16, 8, 1),
                  ("intensity", 24, 7, 1))
        _, _, rows = self.save([cloud(points=(
            (1e308, 1e308, 1e308, 1), (1.2e308, 1.2e308, 1.2e308, 2),
        ), fields=fields)], voxel_size=1e308)
        self.assertEqual(len(rows), 1)
        for value in rows[0][:3]:
            self.assertAlmostEqual(float(value) / 1e308, 1.1)

    def test_requested_order_vectors_and_integer_attributes_are_preserved(self):
        fields = DEFAULT_FIELDS + (("ring", 16, 4, 1), ("descriptor", 18, 7, 2))
        text, names, rows = self.save([
            cloud(points=((0.125, 0.125, 0.125, 7, 3, 10, 20),
                          (0.375, 0.375, 0.375, 9, 8, 30, 40)), fields=fields),
        ], fields=["descriptor", "z", "ring", "intensity", "x", "y"])
        self.assertEqual(names, ["descriptor", "z", "ring", "intensity", "x", "y"])
        self.assertIn("COUNT 2 1 1 1 1 1\n", text)
        self.assertIn("TYPE F F U F F F\n", text)
        self.assertEqual(rows, [["10.0", "20.0", "0.25", "3", "7.0", "0.25", "0.25"]])

    def test_nan_and_infinite_xyz_are_filtered_before_voxelization(self):
        _, _, rows = self.save([cloud(points=(
            (float("nan"), 0, 0, 90), (0.125, 0.125, 0.125, float("nan")),
            (0, float("inf"), 0, 91), (0.375, 0.375, 0.375, 8),
        ))])
        self.assertEqual(rows, [["0.25", "0.25", "0.25", "nan"]])

    def test_zero_voxel_size_preserves_the_existing_pcd_exactly(self):
        messages = [cloud(points=((0.125, 0, 0, 10), (0.375, 0, 0, 20)))]
        original = self.root / "original.pcd"
        self.writer()(messages, original, ["x", "y", "z", "intensity"])
        text, _, _ = self.save(messages, voxel_size=0)
        self.assertEqual(text, original.read_text())

    def test_voxelization_rejects_integer_xyz_without_publishing_a_file(self):
        message = cloud(points=((1, 2, 3, 4),), fields=(
            ("x", 0, 5, 1), ("y", 4, 5, 1), ("z", 8, 5, 1),
            ("intensity", 12, 7, 1),
        ))
        with self.assertRaisesRegex(ValueError, "float"):
            self.save([message])
        self.assertFalse((self.root / "voxel.pcd").exists())

    def test_no_finite_points_fails_before_creating_a_file(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            self.save([cloud(points=((float("nan"), 0, 0, 1),))])
        self.assertFalse((self.root / "voxel.pcd").exists())

    def test_unrepresentable_voxel_coordinates_report_a_clear_error(self):
        with self.assertRaisesRegex(ValueError, "voxel"):
            self.save([cloud()], voxel_size=1e-320)
        self.assertFalse((self.root / "voxel.pcd").exists())

    def test_larger_voxels_reduce_output_while_smaller_voxels_keep_details(self):
        messages = [cloud(points=((0.125, 0, 0, 1), (0.375, 0, 0, 2),
                                  (0.625, 0, 0, 3), (0.875, 0, 0, 4)))]
        _, _, fine = self.save(messages, voxel_size=0.25)
        _, _, coarse = self.save(messages, voxel_size=1)
        self.assertEqual(len(fine), 4)
        self.assertEqual(len(coarse), 1)
        self.assertEqual(float(coarse[0][0]), 0.5)


class VoxelNodeTests(DenseTestCase):
    def test_cli_voxel_size_reaches_the_paired_files_written_by_main(self):
        def collect(args, node, image_writer):
            node.process_image_frame(self.image)
            node.handle_save_key()
            node._on_pointcloud(cloud(points=((0.125, 0, 0, 7), (0.375, 0, 0, 9))))
            node._process_dense_capture()

        with mock.patch.object(sys, "argv", [
            "calib-snap", "--input-mode", "camera", "--capture-mode", "dense",
            "--pointcloud-topic", "/lidar", "--accumulate-frames", "1",
            "--voxel-size", "1", "--output-dir", str(self.root),
        ]), mock.patch.object(threading.Thread, "start"), \
                mock.patch.object(self.calib, "run_capture_inputs", side_effect=collect), \
                mock.patch("sys.stderr", new_callable=io.StringIO):
            try:
                self.calib.main()
            except SystemExit:
                self.fail("The CLI must pass --voxel-size through the dense capture node")
        images = list((self.root / "images").glob("*.png"))
        clouds = list((self.root / "pointclouds").glob("*.pcd"))
        self.assertEqual(len(images), 1)
        self.assertEqual(len(clouds), 1)
        self.assertEqual(images[0].stem, clouds[0].stem)
        self.assertIn("POINTS 1\n", clouds[0].read_text())
        self.assertEqual(clouds[0].read_text().split("DATA ascii\n")[1], "0.25 0.0 0.0 7.0\n")

    def test_voxel_failure_cleans_pair_and_allows_a_new_round(self):
        try:
            node = self.node(accumulate_frames=1, voxel_size=1e-320)
        except TypeError:
            self.fail("Dense capture nodes must accept voxel_size")
        node.process_image_frame(self.image)
        node.handle_save_key()
        node._on_pointcloud(cloud())
        node._process_dense_capture()
        self.assertEqual(node._capture_index, 0)
        self.assertEqual(list((self.root / "images").iterdir()), [])
        self.assertEqual(list((self.root / "pointclouds").iterdir()), [])
        self.assertIn("IDLE", node.capture_status())
        node.handle_save_key()
        self.assertIn("COLLECTING", node.capture_status())


@unittest.skipIf(fixtures.RosPointCloud2 is None, "ROS sensor_msgs Python packages are unavailable")
class RosVoxelIntegrationTests(DenseTestCase):
    def test_real_ros_reader_merges_padded_big_endian_frames(self):
        fixture = cloud(points=((0.125, 0.125, 0.125, 7),
                                (0.375, 0.375, 0.375, 9)),
                        bigendian=True, width=1, padding=12)
        message = fixtures.RosPointCloud2()
        message.header.frame_id = "lidar"
        message.fields = [
            fixtures.RosPointField(item.name, item.offset, item.datatype, item.count)
            for item in fixture.fields
        ]
        for name in ("height", "width", "point_step", "row_step", "data",
                     "is_bigendian", "is_dense"):
            setattr(message, name, getattr(fixture, name))
        output = self.root / "ros-voxel.pcd"
        with mock.patch.object(self.calib, "point_cloud2", fixtures.ros_point_cloud2):
            try:
                count, _ = self.writer()(
                    [message], output, ["x", "y", "z", "intensity"], voxel_size=1
                )
            except TypeError:
                self.fail("The dense writer must support voxel downsampling")
        self.assertEqual(count, 1)
        self.assertEqual(output.read_text().split("DATA ascii\n")[1], "0.25 0.25 0.25 7.0\n")
