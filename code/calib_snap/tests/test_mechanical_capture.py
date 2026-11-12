import io
import json
import sys
import threading
import unittest
from unittest import mock

import test_dense_capture as fixtures
from test_dense_capture import ClockTime, DEFAULT_FIELDS, DenseTestCase, cloud


MODE = "fastcalib2-mechanical"
FIELDS = DEFAULT_FIELDS + (("ring", 16, 4, 1),)


def mechanical_cloud(points=((1, 2, 3, 4, 7),), **options):
    return cloud(points=points, fields=options.pop("fields", FIELDS), **options)


class MechanicalArguments(DenseTestCase):
    def test_frame_and_duration_targets_use_existing_accumulation_options(self):
        for target in [("--accumulate-frames", "50"), ("--accumulate-seconds", "3")]:
            with self.subTest(target=target):
                args = self.parse("--capture-mode", MODE, "--pointcloud-topic", "/lidar",
                                  *target)
                self.assertEqual(args.resolved_capture_mode, MODE)
                self.assertEqual(args.voxel_size, 0)
                self.assertEqual(args.accumulate_timeout, 5)

    def test_requires_topic_and_one_target(self):
        for options, message in [
            (["--accumulate-frames", "1"], "--pointcloud-topic"),
            (["--pointcloud-topic", "/lidar"], "--accumulate-frames"),
        ]:
            with self.subTest(options=options), \
                    mock.patch("sys.stderr", new_callable=io.StringIO) as error:
                with self.assertRaises(SystemExit):
                    self.parse("--capture-mode", MODE, *options)
                self.assertIn(message, error.getvalue())

    def test_rejects_voxel_averaging(self):
        with mock.patch("sys.stderr", new_callable=io.StringIO) as error:
            with self.assertRaises(SystemExit):
                self.parse("--capture-mode", MODE, "--pointcloud-topic", "/lidar",
                           "--accumulate-frames", "1", "--voxel-size", "0.005")
            self.assertIn("--voxel-size", error.getvalue())


class MechanicalTestCase(DenseTestCase):
    def mechanical_node(self, **options):
        options.setdefault("capture_mode", MODE)
        options.setdefault("accumulate_frames", 1)
        return self.node(**options)

    def collect(self, messages, **options):
        options.setdefault("accumulate_frames", len(messages))
        node = self.mechanical_node(**options)
        node._monotonic = lambda: 10.0
        node.process_image_frame(self.image)
        node.handle_save_key()
        for index, message in enumerate(messages):
            node._monotonic = lambda index=index: 10.1 + index * 0.1
            ClockTime.current = max(100, message.header.stamp.to_sec())
            node._on_pointcloud(message)
        node._process_dense_capture()
        return node

    def sample(self):
        pcds = list((self.root / "pointclouds").glob("*.pcd"))
        self.assertEqual(len(pcds), 1, "A mechanical capture must save a PCD")
        text = pcds[0].read_text()
        metadata_path = self.root / "metadata" / (pcds[0].stem + ".json")
        self.assertTrue(metadata_path.exists(), "Capture metadata must accompany the PCD")
        images = list((self.root / "images").glob("*.png"))
        self.assertEqual([path.stem for path in images], [pcds[0].stem])
        metadata = json.loads(metadata_path.read_text())
        rows = [row.split() for row in text.split("DATA ascii\n")[1].splitlines()]
        return text, rows, metadata

    def assert_no_sample(self, node):
        self.assertEqual(node._capture_index, 0)
        for directory in ["images", "pointclouds", "metadata"]:
            self.assertEqual(list((self.root / directory).iterdir()), [])


class MechanicalOutputTests(MechanicalTestCase):
    def test_required_header_and_message_ids_preserve_points_without_mutating_input(self):
        messages = [
            mechanical_cloud(((1, 2, 3, 10, 0), (1, 2, 3, 20, 65535)), stamp=100),
            mechanical_cloud(((1, 2, 3, 30, 12),), stamp=100.1),
        ]
        original_data = [msg.data for msg in messages]
        node = self.collect(messages, requested_pcd_fields=["x"])
        self.assertEqual(node._capture_index, 1)
        text, rows, metadata = self.sample()
        self.assertIn("FIELDS x y z intensity ring scan_id\n", text)
        self.assertIn("SIZE 4 4 4 4 2 4\nTYPE F F F F U U\nCOUNT 1 1 1 1 1 1\n", text)
        self.assertEqual(rows, [
            ["1.0", "2.0", "3.0", "10.0", "0", "0"],
            ["1.0", "2.0", "3.0", "20.0", "65535", "0"],
            ["1.0", "2.0", "3.0", "30.0", "12", "1"],
        ])
        self.assertEqual([msg.data for msg in messages], original_data)
        self.assertFalse(any(item.name == "scan_id" for item in messages[0].fields))
        self.assertEqual(metadata["capture_mode"], MODE)
        self.assertEqual(metadata["accepted_frames"], 2)
        self.assertEqual(metadata["saved_point_count"], 3)
        self.assertEqual(metadata["frame_id"], "lidar")

    def test_filters_whole_records_for_nonfinite_xyz_or_intensity_and_keeps_extras(self):
        fields = FIELDS + (("descriptor", 18, 5, 2),)
        msg = mechanical_cloud((
            (1, 2, 3, 4, 1, 11, 12),
            (float("nan"), 2, 3, 5, 2, 21, 22),
            (1, 2, 3, float("inf"), 3, 31, 32),
            (5, 6, 7, 8, 4, 41, 42),
        ), fields=fields)
        self.collect([msg], requested_pcd_fields=["descriptor", "ring", "x", "descriptor"])
        text, rows, metadata = self.sample()
        self.assertIn("FIELDS x y z intensity ring scan_id descriptor\n", text)
        self.assertIn("COUNT 1 1 1 1 1 1 2\n", text)
        self.assertEqual(rows, [
            ["1.0", "2.0", "3.0", "4.0", "1", "0", "11", "12"],
            ["5.0", "6.0", "7.0", "8.0", "4", "0", "41", "42"],
        ])
        self.assertEqual(metadata["filtered_point_count"], 2)
        self.assertEqual(metadata["frames"][0]["raw_point_count"], 4)

    def test_checked_conversions_emit_float32_and_uint16(self):
        fields = (("x", 0, 8, 1), ("y", 8, 5, 1), ("z", 12, 3, 1),
                  ("intensity", 14, 6, 1), ("ring", 18, 8, 1))
        self.collect([mechanical_cloud(((1.00000006, -2, 3, 4294967295, 65535.0),),
                                       fields=fields)])
        _, rows, _ = self.sample()
        self.assertEqual(rows, [
            ["1.0000001192092896", "-2.0", "3.0", "4294967296.0", "65535", "0"]
        ])

    def test_padded_big_endian_and_reordered_fields_keep_their_attributes(self):
        first = mechanical_cloud(((1, 2, 3, 4, 5), (6, 7, 8, 9, 10)),
                                 width=1, padding=12, bigendian=True)
        second = mechanical_cloud(((20, 19, 18, 17, 16),), stamp=100.1, fields=(
            ("ring", 0, 4, 1), ("intensity", 2, 7, 1), ("z", 6, 7, 1),
            ("y", 10, 7, 1), ("x", 14, 7, 1),
        ))
        self.collect([first, second])
        _, rows, _ = self.sample()
        self.assertEqual(rows, [
            ["1.0", "2.0", "3.0", "4.0", "5", "0"],
            ["6.0", "7.0", "8.0", "9.0", "10", "0"],
            ["16.0", "17.0", "18.0", "19.0", "20", "1"],
        ])

    def test_internal_scan_ids_are_remapped_per_message_in_first_observation_order(self):
        fields = FIELDS + (("scan_id", 18, 6, 1),)
        messages = [
            mechanical_cloud(((1, 2, 3, 4, 5, 42), (2, 2, 3, 4, 6, 7),
                              (3, 2, 3, 4, 7, 42)), fields=fields),
            mechanical_cloud(((4, 2, 3, 4, 8, 42), (5, 2, 3, 4, 9, 7)),
                             fields=fields, stamp=100.1),
        ]
        self.collect(messages)
        _, rows, metadata = self.sample()
        self.assertEqual([row[5] for row in rows], ["0", "1", "0", "2", "3"])
        self.assertEqual(metadata["frames"][0]["scan_id_mapping"], [
            {"source_scan_id": 42, "scan_id": 0},
            {"source_scan_id": 7, "scan_id": 1},
        ])
        self.assertEqual(metadata["frames"][1]["scan_id_mapping"], [
            {"source_scan_id": 42, "scan_id": 2},
            {"source_scan_id": 7, "scan_id": 3},
        ])

    def test_new_sample_restarts_scan_ids_at_zero(self):
        node = self.collect([mechanical_cloud(stamp=0)])
        node.handle_save_key()
        node._on_pointcloud(mechanical_cloud(stamp=0))
        node._process_dense_capture()
        self.assertEqual(node._capture_index, 2)
        for path in (self.root / "pointclouds").glob("*.pcd"):
            self.assertEqual(path.read_text().split("DATA ascii\n")[1].split()[-1], "0")

    def test_main_forwards_mode_and_requested_extra_fields(self):
        def capture(args, node, image_writer):
            node.process_image_frame(self.image)
            node.handle_save_key()
            node._on_pointcloud(mechanical_cloud(((1, 2, 3, 4, 5, 99),),
                fields=FIELDS + (("time", 18, 8, 1),)))
            node._process_dense_capture()

        with mock.patch.object(sys, "argv", [
            "calib-snap", "--input-mode", "camera", "--capture-mode", MODE,
            "--pointcloud-topic", "/lidar", "--accumulate-frames", "1",
            "--pcd-fields", "time", "--output-dir", str(self.root),
        ]), mock.patch.object(threading.Thread, "start"), \
                mock.patch.object(self.calib, "run_capture_inputs", side_effect=capture):
            self.calib.main()
        text, rows, metadata = self.sample()
        self.assertIn("FIELDS x y z intensity ring scan_id time\n", text)
        self.assertEqual(rows[0][-1], "99.0")
        self.assertEqual(metadata["pointcloud_topic"], "/lidar")


class MechanicalValidationTests(MechanicalTestCase):
    def test_missing_required_field_fails_during_receipt_and_can_restart(self):
        node = self.mechanical_node()
        node.process_image_frame(self.image)
        for fields, values, missing in [
            (DEFAULT_FIELDS, (1, 2, 3, 4), "ring"),
            (DEFAULT_FIELDS[:3] + (("ring", 12, 4, 1),), (1, 2, 3, 4), "intensity"),
        ]:
            node.handle_save_key()
            node._on_pointcloud(mechanical_cloud((values,), fields=fields))
            self.assertIsNotNone(node._dense_session)
            self.assertEqual(node._dense_session.state, "IDLE")
            self.assertIn(missing, node._dense_session.error)
            self.assert_no_sample(node)
        node.handle_save_key()
        node._on_pointcloud(mechanical_cloud())
        node._process_dense_capture()
        self.assertEqual(node._capture_index, 1)

    def test_ring_invalid_values_abort_before_caching(self):
        for ring in [-1, 65536, 1.5, float("nan"), float("inf")]:
            with self.subTest(ring=ring):
                fields = DEFAULT_FIELDS + (("ring", 16, 8, 1),)
                node = self.collect([mechanical_cloud(((1, 2, 3, 4, ring),), fields=fields)])
                self.assertIsNotNone(node._dense_session)
                self.assertIn("ring", node._dense_session.error)
                self.assertEqual(node._dense_session.byte_count, 0)
                self.assert_no_sample(node)

    def test_float32_conversion_overflow_is_a_capture_error(self):
        for name in ["x", "intensity"]:
            with self.subTest(name=name):
                fields = (("x", 0, 8, 1), ("y", 8, 8, 1), ("z", 16, 8, 1),
                          ("intensity", 24, 8, 1), ("ring", 32, 4, 1))
                values = [1, 2, 3, 4, 5]
                values[["x", "y", "z", "intensity", "ring"].index(name)] = 1e100
                node = self.collect([mechanical_cloud((tuple(values),), fields=fields)])
                self.assertIsNotNone(node._dense_session)
                self.assertIn("float32", node._dense_session.error)
                self.assertIn(name, node._dense_session.error)
                self.assert_no_sample(node)

    def test_non_scalar_required_field_is_rejected(self):
        fields = DEFAULT_FIELDS + (("ring", 16, 4, 2),)
        node = self.collect([mechanical_cloud(((1, 2, 3, 4, 5, 6),), fields=fields)])
        self.assertIsNotNone(node._dense_session)
        self.assertIn("ring", node._dense_session.error)
        self.assert_no_sample(node)

    def test_invalid_internal_scan_ids_abort_instead_of_truncating(self):
        fields = FIELDS + (("scan_id", 18, 8, 1),)
        for scan_id in [-1, 1.5, float("nan"), 4294967296]:
            with self.subTest(scan_id=scan_id):
                node = self.collect([mechanical_cloud(((1, 2, 3, 4, 5, scan_id),),
                                                      fields=fields)])
                self.assertIsNotNone(node._dense_session)
                self.assertIn("scan_id", node._dense_session.error)
                self.assert_no_sample(node)

    def test_float32_scan_id_boundary_is_checked_without_rounding_the_limit(self):
        fields = FIELDS + (("scan_id", 18, 7, 1),)
        node = self.collect([mechanical_cloud(((1, 2, 3, 4, 5, 4294967296),),
                                              fields=fields)])
        self.assertIsNotNone(node._dense_session)
        self.assertIsNotNone(node._dense_session.error)
        self.assertIn("scan_id", node._dense_session.error)
        self.assert_no_sample(node)
        node.handle_save_key()
        node._on_pointcloud(mechanical_cloud(((1, 2, 3, 4, 5, 4294967040),),
                                             fields=fields))
        node._process_dense_capture()
        _, rows, metadata = self.sample()
        self.assertEqual(rows[0][-1], "0")
        self.assertEqual(metadata["frames"][0]["scan_id_mapping"], [
            {"source_scan_id": 4294967040, "scan_id": 0},
        ])

    def test_all_invalid_messages_do_not_count_or_consume_a_scan_id(self):
        node = self.collect([
            mechanical_cloud(((1, 2, 3, float("nan"), 1),), stamp=100),
            mechanical_cloud(stamp=100.1),
        ], accumulate_frames=1)
        _, rows, metadata = self.sample()
        self.assertEqual(rows[0][-1], "0")
        self.assertEqual(metadata["accepted_frames"], 1)
        self.assertEqual(metadata["filtered_point_count"], 1)
        self.assertEqual(metadata["invalid_cloud_count"], 1)
        self.assertEqual(metadata["raw_point_count"], 2)
        self.assertEqual(metadata["accepted_raw_point_count"], 1)

    def test_all_invalid_capture_times_out_without_any_files(self):
        node = self.collect([mechanical_cloud(((float("inf"), 2, 3, 4, 5),))])
        self.assertIsNotNone(node._dense_session)
        node._monotonic = lambda: 16
        node._process_dense_capture()
        self.assertEqual(node._dense_session.state, "IDLE")
        self.assert_no_sample(node)

    def test_skipped_invalid_cloud_does_not_consume_cache_capacity(self):
        session = self.session(frame_limit=2, max_bytes=36, mechanical=True)
        session.start(self.image, ClockTime(100), 10)
        self.assertTrue(session.add_cloud(mechanical_cloud(), 10.1, 100))
        invalid = mechanical_cloud(((float("nan"), 2, 3, 4, 5),) * 2, stamp=100.1)
        self.assertFalse(session.add_cloud(invalid, 10.2, 100.1))
        self.assertEqual(session.state, "COLLECTING")
        self.assertEqual(session.byte_count, 18)
        self.assertTrue(session.add_cloud(mechanical_cloud(stamp=100.2), 10.3, 100.2))
        snapshot = session.take_ready(10.3)
        self.assertEqual(snapshot.raw_point_count, 2)
        self.assertEqual(snapshot.byte_count, 36)
        self.assertEqual(snapshot.mechanical_metadata["filtered_point_count"], 2)

    def test_stale_duplicate_and_backward_clouds_do_not_consume_scan_ids(self):
        self.collect([
            mechanical_cloud(stamp=98),
            mechanical_cloud(stamp=100),
            mechanical_cloud(stamp=100),
            mechanical_cloud(stamp=99.9),
            mechanical_cloud(stamp=100.1),
        ], accumulate_frames=2)
        _, rows, metadata = self.sample()
        self.assertEqual([row[-1] for row in rows], ["0", "1"])
        self.assertEqual(metadata["skipped_cloud_count"], 3)

    def test_metadata_publication_failure_rolls_back_the_whole_sample(self):
        link = self.calib.os.link

        def fail_metadata(source, destination):
            if str(destination).endswith(".json"):
                raise OSError("metadata disk failure")
            return link(source, destination)

        with mock.patch.object(self.calib.os, "link", side_effect=fail_metadata):
            node = self.collect([mechanical_cloud()])
        self.assert_no_sample(node)
        self.assertIn("IDLE", node.capture_status())

    def test_zero_voxel_is_enforced_for_programmatic_callers(self):
        with self.assertRaisesRegex(ValueError, "voxel"):
            self.mechanical_node(voxel_size=0.005)


@unittest.skipIf(fixtures.RosPointCloud2 is None, "ROS sensor_msgs Python packages are unavailable")
class MechanicalRosIntegrationTests(MechanicalTestCase):
    def test_real_ros_message_keeps_ring_and_generates_scan_ids(self):
        fixture = mechanical_cloud(((1, 2, 3, 4, 5), (6, 7, 8, 9, 10)),
                                   width=1, padding=6, bigendian=True)
        message = fixtures.RosPointCloud2()
        message.header.frame_id = "lidar"
        message.header.stamp = fixtures.RosTime.from_sec(100)
        message.fields = [
            fixtures.RosPointField(item.name, item.offset, item.datatype, item.count)
            for item in fixture.fields
        ]
        for name in ("height", "width", "point_step", "row_step", "data",
                     "is_bigendian", "is_dense"):
            setattr(message, name, getattr(fixture, name))
        self.collect([message])
        _, rows, _ = self.sample()
        self.assertEqual([row[4:] for row in rows], [["5", "0"], ["10", "0"]])
