import io
import math
import struct
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import cv2
import numpy as np

from test_image_only import build_stub_modules, load_calib_snap

try:
    from genpy import Time as RosTime
    from sensor_msgs import point_cloud2 as ros_point_cloud2
    from sensor_msgs.msg import PointCloud2 as RosPointCloud2, PointField as RosPointField
except ImportError:
    RosPointCloud2 = None


class ClockTime:
    current = 100.0

    def __init__(self, seconds=0.0):
        self.seconds = seconds

    @classmethod
    def now(cls):
        return cls(cls.current)

    def to_sec(self):
        return self.seconds

    def __sub__(self, other):
        return ClockTime(self.seconds - other.seconds)

    def __eq__(self, other):
        return isinstance(other, ClockTime) and self.seconds == other.seconds


def field(name, offset, datatype=7, count=1):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)


DEFAULT_FIELDS = (
    ("x", 0, 7, 1),
    ("y", 4, 7, 1),
    ("z", 8, 7, 1),
    ("intensity", 12, 7, 1),
)
STRUCT_TYPES = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d"}


def cloud(points=((1.0, 2.0, 3.0, 4.0),), stamp=100.0, frame_id="lidar",
          fields=DEFAULT_FIELDS, bigendian=False, width=None, padding=0):
    fields = [field(*item) for item in fields]
    point_step = max(
        (item.offset + struct.calcsize(STRUCT_TYPES[item.datatype]) * item.count
         for item in fields), default=0
    )
    width = len(points) if width is None else width
    height = len(points) // width if width else 1
    row_step = width * point_step + padding
    data = bytearray(row_step * height)
    endian = ">" if bigendian else "<"
    for index, values in enumerate(points):
        start = (index // width) * row_step + (index % width) * point_step
        cursor = 0
        for item in fields:
            struct.pack_into(
                endian + STRUCT_TYPES[item.datatype] * item.count,
                data, start + item.offset, *values[cursor:cursor + item.count]
            )
            cursor += item.count
    return SimpleNamespace(
        header=SimpleNamespace(stamp=ClockTime(stamp), frame_id=frame_id, seq=0),
        fields=fields, width=width, height=height, point_step=point_step,
        row_step=row_step, data=bytes(data), is_bigendian=bigendian, is_dense=False,
    )


def read_binary_points(msg, field_names, skip_nans=False):
    selected = sorted(
        (item for item in msg.fields if item.name in field_names),
        key=lambda item: item.offset,
    )
    for row in range(msg.height):
        for column in range(msg.width):
            start = row * msg.row_step + column * msg.point_step
            values = []
            for item in selected:
                values.extend(struct.unpack_from(
                    (">" if msg.is_bigendian else "<")
                    + STRUCT_TYPES[item.datatype] * item.count,
                    msg.data, start + item.offset
                ))
            if skip_nans and any(isinstance(v, float) and math.isnan(v) for v in values):
                continue
            yield tuple(values)


class DenseTestCase(unittest.TestCase):
    def setUp(self):
        modules = build_stub_modules()
        del modules["cv2"]
        del modules["numpy"]
        modules["rospy"].Time = ClockTime
        modules["rospy"].on_shutdown = lambda callback: None
        modules["sensor_msgs.point_cloud2"].read_points = read_binary_points
        self.patcher = mock.patch.dict(sys.modules, modules)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.calib = load_calib_snap()
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.image = np.full((2, 3, 3), 37, dtype=np.uint8)
        ClockTime.current = 100.0

    def session(self, **options):
        factory = getattr(self.calib, "DenseCaptureSession", None)
        self.assertIsNotNone(factory, "Dense capture sessions are not implemented")
        kwargs = dict(
            requested_fields=["x", "y", "z", "intensity"], frame_limit=3,
            duration=None, timeout=5.0, max_bytes=1024, max_age=0.5,
        )
        kwargs.update(options)
        return factory(**kwargs)

    def parse(self, *options):
        with mock.patch.object(sys, "argv", [
            "calib-snap", "--input-mode", "camera", *options
        ]):
            return self.calib.parse_args()

    def writer(self):
        writer = getattr(self.calib, "save_pointclouds_as_pcd", None)
        self.assertIsNotNone(writer, "Multi-frame PCD output is not implemented")
        return writer

    def snapshot(self):
        session = self.session(frame_limit=1)
        session.start(self.image.copy(), ClockTime(100), now=10.0)
        session.add_cloud(cloud(), received_at=10.1, ros_now=100.0)
        return session.take_ready(now=10.1)

    def node(self, **options):
        self.assertIsNotNone(getattr(self.calib, "DenseCaptureSession", None),
                             "Dense capture is not implemented")
        kwargs = dict(
            input_mode="camera", capture_mode="dense", pointcloud_topic="/lidar",
            source_image_topic=None, image_topic="/image",
            compressed_topic="/compressed", frame_id="camera", output_dir=self.root,
            image_ext="png", requested_pcd_fields=["x", "y", "z", "intensity"],
            max_pointcloud_age=0.5, accumulate_frames=2,
        )
        kwargs.update(options)
        with mock.patch.object(threading.Thread, "start"):
            node = self.calib.CalibrationCaptureNode(**kwargs)
        self.addCleanup(node.close)
        return node


class DenseArgumentTests(DenseTestCase):
    def test_dense_frame_target_is_resolved_and_defaults_are_applied(self):
        try:
            args = self.parse("--capture-mode", "dense", "--pointcloud-topic", "/lidar",
                              "--accumulate-frames", "50")
        except SystemExit:
            self.fail("Dense frame parameters are not supported")
        self.assertEqual(args.resolved_capture_mode, "dense")
        self.assertEqual(args.accumulate_frames, 50)
        self.assertEqual(args.accumulate_timeout, 5.0)
        self.assertEqual(args.accumulate_max_mb, 256)

    def test_dense_time_target_is_supported(self):
        try:
            args = self.parse("--capture-mode", "dense", "--pointcloud-topic", "/lidar",
                              "--accumulate-seconds", "3.5")
        except SystemExit:
            self.fail("Dense duration parameters are not supported")
        self.assertEqual(args.resolved_capture_mode, "dense")
        self.assertEqual(args.accumulate_seconds, 3.5)

    def test_dense_custom_hotkeys_are_normalized_for_case_insensitive_dispatch(self):
        args = self.parse(
            "--capture-mode", "dense", "--pointcloud-topic", "/lidar",
            "--accumulate-frames", "2", "--save-key", "S",
            "--cancel-key", "C", "--quit-key", "Q",
        )
        self.assertEqual((args.save_key, args.cancel_key, args.quit_key), ("s", "c", "q"))

    def test_dense_requires_topic_and_exactly_one_target(self):
        cases = [
            (["--capture-mode", "dense", "--accumulate-frames", "2"], "--pointcloud-topic"),
            (["--capture-mode", "dense", "--pointcloud-topic", "/lidar"],
             "--accumulate-frames"),
            (["--capture-mode", "dense", "--pointcloud-topic", "/lidar",
              "--accumulate-frames", "2", "--accumulate-seconds", "1"],
             "--accumulate-seconds"),
        ]
        for argv, message in cases:
            with self.subTest(argv=argv), mock.patch("sys.stderr", new_callable=io.StringIO) as err:
                with self.assertRaises(SystemExit):
                    self.parse(*argv)
                self.assertIn(message, err.getvalue())

    def test_dense_rejects_invalid_limits_and_colliding_keys(self):
        for argv in [
            ["--accumulate-frames", "0"], ["--accumulate-frames", "-1"],
            ["--accumulate-seconds", "0"], ["--accumulate-seconds", "nan"],
            ["--accumulate-seconds", "inf"],
            ["--accumulate-frames", "1", "--accumulate-timeout", "nan"],
            ["--accumulate-frames", "1", "--accumulate-timeout", "-1"],
            ["--accumulate-frames", "1", "--accumulate-max-mb", "0"],
            ["--accumulate-frames", "1", "--cancel-key", "S"],
            ["--accumulate-frames", "1", "--save-key", ""],
        ]:
            with self.subTest(argv=argv), mock.patch("sys.stderr", new_callable=io.StringIO):
                with self.assertRaises(SystemExit):
                    self.parse("--capture-mode", "dense", "--pointcloud-topic", "/lidar", *argv)

    def test_non_dense_modes_reject_explicit_dense_parameters(self):
        for argv in [
            ["--accumulate-frames", "2"], ["--accumulate-seconds", "1"],
            ["--accumulate-timeout", "5"], ["--accumulate-max-mb", "256"],
            ["--cancel-key", "c"],
        ]:
            with self.subTest(argv=argv), mock.patch("sys.stderr", new_callable=io.StringIO):
                with self.assertRaises(SystemExit):
                    self.parse(*argv)


class DenseSessionTests(DenseTestCase):
    def test_frame_target_freezes_exactly_three_received_clouds(self):
        session = self.session()
        self.assertTrue(session.start(self.image, ClockTime(100), now=10.0))
        self.assertFalse(session.start(self.image, ClockTime(100), now=10.0))
        for index in range(3):
            self.assertTrue(session.add_cloud(
                cloud(stamp=100 + index / 10), received_at=10.1 + index / 10,
                ros_now=100 + index / 10,
            ))
        self.assertFalse(session.add_cloud(cloud(stamp=101), 10.4, 101))
        snapshot = session.take_ready(now=10.5)
        self.assertEqual(len(snapshot.clouds), 3)
        self.assertEqual(snapshot.raw_point_count, 3)
        self.assertEqual(snapshot.reason, "frames")
        self.assertEqual(session.state, "SAVING")
        self.assertIsNone(session.take_ready(now=10.6))
        session.finish_save(snapshot.session_id)
        self.assertEqual(session.state, "IDLE")
        self.assertEqual(session.byte_count, 0)

    def test_duration_completes_without_another_cloud(self):
        session = self.session(frame_limit=None, duration=3)
        session.start(self.image, ClockTime(), now=10)
        session.add_cloud(cloud(), 10.2, 100)
        self.assertIsNone(session.take_ready(12.99))
        snapshot = session.take_ready(13)
        self.assertEqual(len(snapshot.clouds), 1)
        self.assertEqual(snapshot.finished_at, 13)
        self.assertEqual(snapshot.reason, "duration")

    def test_duration_boundary_excludes_late_cloud_even_before_worker_poll(self):
        session = self.session(frame_limit=None, duration=3)
        session.start(self.image, ClockTime(), now=10)
        session.add_cloud(cloud(), 10.1, 100)
        self.assertFalse(session.add_cloud(cloud(stamp=100.1), 13, 100.1))
        self.assertEqual(len(session.take_ready(15).clouds), 1)

    def test_timeout_is_not_revived_by_late_cloud(self):
        for receipt in [15, 16]:
            with self.subTest(receipt=receipt):
                session = self.session()
                session.start(self.image, ClockTime(), 10)
                self.assertFalse(session.add_cloud(cloud(), receipt, 100))
                self.assertEqual(session.state, "IDLE")
                self.assertIn("timeout", session.error.lower())
                self.assertEqual(session.byte_count, 0)

    def test_silence_deadline_before_duration_cancels_round(self):
        session = self.session(frame_limit=None, duration=10, timeout=2)
        session.start(self.image, ClockTime(), 10)
        session.add_cloud(cloud(), 10.1, 100)
        self.assertIsNone(session.take_ready(20))
        self.assertEqual(session.state, "IDLE")
        self.assertIn("timeout", session.error.lower())

    def test_equal_duration_and_silence_deadlines_complete_received_data(self):
        session = self.session(frame_limit=None, duration=3, timeout=3)
        session.start(self.image, ClockTime(), 10)
        session.add_cloud(cloud(), 10, 100)
        self.assertEqual(len(session.take_ready(13).clouds), 1)

    def test_empty_duration_capture_fails(self):
        session = self.session(frame_limit=None, duration=1)
        session.start(self.image, ClockTime(), 10)
        self.assertIsNone(session.take_ready(11))
        self.assertEqual(session.state, "IDLE")
        self.assertTrue(session.error)

    def test_empty_stale_duplicate_and_out_of_order_clouds_do_not_count(self):
        session = self.session()
        session.start(self.image, ClockTime(), 10)
        for msg in [cloud(points=()), cloud(stamp=98), cloud(stamp=99.9)]:
            session.add_cloud(msg, 10.1, 100)
        self.assertEqual(session.frame_count, 1)
        session.add_cloud(cloud(stamp=99.9), 11, 100)
        session.add_cloud(cloud(stamp=99.8), 12, 100)
        self.assertEqual(session.frame_count, 1)
        self.assertIsNone(session.take_ready(15.1))
        self.assertEqual(session.state, "IDLE")

    def test_zero_stamp_uses_receipt_events(self):
        session = self.session(frame_limit=2)
        session.start(self.image, ClockTime(), 10)
        session.add_cloud(cloud(stamp=0), 10.1, 100)
        session.add_cloud(cloud(stamp=0), 10.2, 100)
        self.assertEqual(len(session.take_ready(10.2).clouds), 2)

    def test_receipt_before_start_is_excluded(self):
        session = self.session()
        session.start(self.image, ClockTime(), 10)
        self.assertFalse(session.add_cloud(cloud(), 9.9, 100))
        self.assertEqual(session.frame_count, 0)

    def test_cache_overflow_fails_and_releases_round(self):
        session = self.session(max_bytes=16)
        session.start(self.image, ClockTime(), 10)
        session.add_cloud(cloud(), 10.1, 100)
        self.assertEqual(session.byte_count, 16)
        session.add_cloud(cloud(stamp=100.1), 10.2, 100.1)
        self.assertEqual(session.state, "IDLE")
        self.assertEqual(session.byte_count, 0)
        self.assertIn("cache", session.error.lower())

    def test_frame_id_and_schema_changes_fail(self):
        for changed in [
            cloud(stamp=100.1, frame_id="other"),
            cloud(points=((1, 2, 3),), stamp=100.1, fields=DEFAULT_FIELDS[:3]),
            cloud(points=((1, 2, 3, 4),), stamp=100.1,
                  fields=DEFAULT_FIELDS[:3] + (("intensity", 12, 6, 1),)),
        ]:
            with self.subTest(changed=changed.fields):
                session = self.session()
                session.start(self.image, ClockTime(), 10)
                session.add_cloud(cloud(), 10.1, 100)
                session.add_cloud(changed, 10.2, 100.1)
                self.assertEqual(session.state, "IDLE")
                self.assertTrue(session.error)
                self.assertEqual(session.byte_count, 0)

    def test_missing_xyz_or_invalid_layout_fails(self):
        bad_data = cloud()
        bad_data.data = b"x"
        bad_stride = cloud()
        bad_stride.row_step = 1
        bad_field = cloud()
        bad_field.fields[0].offset = 16
        for msg in [
            cloud(points=((1, 2),), fields=DEFAULT_FIELDS[:2]),
            bad_data, bad_stride, bad_field,
        ]:
            with self.subTest(msg=msg):
                session = self.session()
                session.start(self.image, ClockTime(), 10)
                session.add_cloud(msg, 10.1, 100)
                self.assertEqual(session.state, "IDLE")
                self.assertTrue(session.error)

    def test_overlapping_selected_fields_fail_before_caching(self):
        msg = cloud()
        msg.fields[-1].offset = 8
        session = self.session()
        session.start(self.image, ClockTime(), 10)
        self.assertFalse(session.add_cloud(msg, 10.1, 100))
        self.assertEqual(session.state, "IDLE")
        self.assertIn("overlap", session.error.lower())
        self.assertEqual(session.byte_count, 0)

    def test_unselected_alias_field_does_not_invalidate_layout(self):
        msg = cloud()
        msg.fields.append(field("alias", 8))
        session = self.session(frame_limit=1)
        session.start(self.image, ClockTime(), 10)
        self.assertTrue(session.add_cloud(msg, 10.1, 100))
        self.assertEqual(len(session.take_ready(10.1).clouds), 1)

    def test_cancel_and_restart_reset_image_clouds_and_schema(self):
        session = self.session(frame_limit=1)
        session.start(self.image, ClockTime(), 10)
        session.add_cloud(cloud(), 10.1, 100)
        self.assertTrue(session.cancel())
        self.assertIsNone(session.take_ready(10.2))
        other = np.zeros_like(self.image)
        session.start(other, ClockTime(), 20)
        session.add_cloud(cloud(points=((5, 6, 7, 8),), stamp=101), 20.1, 101)
        snapshot = session.take_ready(20.1)
        self.assertIs(snapshot.image, other)
        self.assertEqual(len(snapshot.clouds), 1)
        self.assertFalse(session.cancel())
        session.finish_save(snapshot.session_id - 1)
        self.assertEqual(session.state, "SAVING")

    def test_old_frames_are_not_rechecked_for_age_at_save_time(self):
        session = self.session(frame_limit=None, duration=3, max_age=0.5)
        session.start(self.image, ClockTime(), 10)
        session.add_cloud(cloud(), 10.1, 100)
        self.assertEqual(len(session.take_ready(13).clouds), 1)


class DensePcdTests(DenseTestCase):
    def test_three_two_point_clouds_write_six_points(self):
        clouds = [cloud(points=((1, 2, 3, 4), (5, 6, 7, 8))) for _ in range(3)]
        path = self.root / "merged.pcd"
        count, names = self.writer()(clouds, path, ["x", "y", "z", "intensity"])
        text = path.read_text()
        self.assertEqual(count, 6)
        self.assertEqual(names, ["x", "y", "z", "intensity"])
        self.assertIn("WIDTH 6\nHEIGHT 1\n", text)
        self.assertIn("POINTS 6\nDATA ascii\n", text)
        self.assertEqual(text.split("DATA ascii\n")[1].splitlines(), [
            "1.0 2.0 3.0 4.0", "5.0 6.0 7.0 8.0",
        ] * 3)

    def test_requested_order_and_vector_counts_match_data_columns(self):
        msg = cloud(
            points=((1, 2, 3, 4, 9, 10),),
            fields=DEFAULT_FIELDS + (("descriptor", 16, 7, 2),),
        )
        path = self.root / "vector.pcd"
        self.writer()([msg], path, ["intensity", "z", "x", "y", "descriptor"])
        text = path.read_text()
        self.assertIn("FIELDS intensity z x y descriptor\n", text)
        self.assertIn("COUNT 1 1 1 1 2\n", text)
        self.assertEqual(text.split("DATA ascii\n")[1], "4.0 3.0 1.0 2.0 9.0 10.0\n")

    def test_padding_endian_and_changed_offsets_are_decoded_per_message(self):
        msg = cloud(points=((1, 2, 3, 4), (5, 6, 7, 8)), width=1,
                    bigendian=True, padding=12)
        reordered = cloud(
            points=((12, 11, 10, 13),),
            fields=(("z", 0, 7, 1), ("y", 4, 7, 1),
                    ("x", 8, 7, 1), ("intensity", 12, 7, 1)),
        )
        path = self.root / "layout.pcd"
        count, _ = self.writer()([msg, reordered], path, ["x", "y", "z", "intensity"])
        self.assertEqual(count, 3)
        self.assertEqual(path.read_text().split("DATA ascii\n")[1].splitlines(), [
            "1.0 2.0 3.0 4.0", "5.0 6.0 7.0 8.0", "10.0 11.0 12.0 13.0",
        ])

    def test_only_invalid_xyz_is_filtered(self):
        msg = cloud(points=((1, 2, 3, float("nan")),
                            (float("nan"), 2, 3, 4), (1, float("inf"), 3, 4)))
        path = self.root / "finite.pcd"
        count, _ = self.writer()([msg], path, ["x", "y", "z", "intensity"])
        self.assertEqual(count, 1)
        self.assertEqual(path.read_text().split("DATA ascii\n")[1], "1.0 2.0 3.0 nan\n")

    def test_no_finite_points_is_an_error(self):
        msg = cloud(points=((float("nan"), 2, 3, 4),))
        with self.assertRaisesRegex(ValueError, "valid|finite"):
            self.writer()([msg], self.root / "empty.pcd", ["x", "y", "z"])


class DenseNodeTests(DenseTestCase):
    def test_failed_camera_reads_still_dispatch_cancel_and_quit(self):
        node = mock.Mock(_capture_mode="dense")
        cap = mock.Mock()
        cap.read.return_value = (False, None)
        reader = mock.MagicMock()
        reader.__enter__.return_value.poll.side_effect = ["c", "q"]
        with mock.patch.object(self.calib, "TerminalKeyReader", return_value=reader), \
                mock.patch.object(self.calib.rospy, "is_shutdown",
                                  side_effect=[False, False, True]) as shutdown, \
                mock.patch.object(self.calib.time, "sleep"):
            self.calib.run_stream_capture(
                node, cap, "camera", mock.Mock(), save_key="s", quit_key="q"
            )
        self.assertEqual(reader.__enter__.return_value.poll.call_count, 2)
        self.assertEqual(shutdown.call_count, 2)
        node.handle_cancel_key.assert_called_once_with()
        node.publish_image.assert_not_called()
        cap.release.assert_called_once_with()
        node.close.assert_called_once_with()

    def test_worker_finishes_duration_without_more_clouds_or_camera_reads(self):
        clock = [10.0]
        with mock.patch.object(self.calib.time, "monotonic", side_effect=lambda: clock[0]):
            node = self.node(accumulate_frames=None, accumulate_seconds=3)
        saved = threading.Event()
        original = self.calib.save_dense_capture_pair

        def save_and_notify(*args):
            result = original(*args)
            saved.set()
            return result

        with mock.patch.object(self.calib, "save_dense_capture_pair", side_effect=save_and_notify):
            node.process_image_frame(self.image)
            node.handle_save_key()
            node._on_pointcloud(cloud())
            node._dense_thread.start()
            clock[0] = 13.0
            self.assertTrue(saved.wait(2), "Deadline worker did not save independently")
            node.close()
        self.assertEqual(node._capture_index, 1)
        self.assertFalse(node._dense_thread.is_alive())
        paths = list((self.root / "pointclouds").glob("*.pcd"))
        self.assertEqual(len(paths), 1)
        self.assertIn("POINTS 1\n", paths[0].read_text())

    def test_close_waits_for_active_save_and_repeated_keys_do_not_queue(self):
        node = self.node(accumulate_frames=1)
        writing = threading.Event()
        release = threading.Event()
        closed = threading.Event()
        self.addCleanup(release.set)
        original = self.calib.save_dense_capture_pair

        def gated_save(*args):
            writing.set()
            if not release.wait(2):
                raise RuntimeError("Test writer was not released")
            return original(*args)

        def close_node():
            node.close()
            closed.set()

        with mock.patch.object(self.calib, "save_dense_capture_pair", side_effect=gated_save):
            node.process_image_frame(self.image)
            node.handle_save_key()
            node._on_pointcloud(cloud())
            node._dense_thread.start()
            self.assertTrue(writing.wait(2))
            node.handle_save_key()
            node.handle_cancel_key()
            self.assertIn("SAVING", node.capture_status())
            closer = threading.Thread(target=close_node)
            closer.start()
            try:
                self.assertFalse(closed.wait(0.01))
            finally:
                release.set()
                closer.join(2)
            self.assertTrue(closed.is_set())
        self.assertEqual(node._capture_index, 1)
        self.assertFalse(node._dense_thread.is_alive())
        self.assertEqual(len(list((self.root / "pointclouds").glob("*.pcd"))), 1)

    def test_trigger_image_is_frozen_and_pretrigger_cloud_is_excluded(self):
        clock = [10.0]
        with mock.patch.object(self.calib.time, "monotonic", side_effect=lambda: clock[0]):
            node = self.node()
        node.process_image_frame(self.image)
        node._on_pointcloud(cloud(points=((90, 91, 92, 93),)))
        node.handle_save_key()
        self.image[:] = 99
        node.process_image_frame(np.zeros_like(self.image))
        for stamp in [100, 100.1]:
            clock[0] += 0.1
            ClockTime.current = stamp
            node._on_pointcloud(cloud(stamp=stamp))
        node._process_dense_capture()
        images = list((self.root / "images").glob("*.png"))
        clouds = list((self.root / "pointclouds").glob("*.pcd"))
        self.assertEqual(len(images), 1)
        self.assertEqual(len(clouds), 1)
        self.assertTrue(np.all(cv2.imread(str(images[0])) == 37))
        self.assertIn("POINTS 2\n", clouds[0].read_text())
        self.assertNotIn("90.0", clouds[0].read_text())
        self.assertEqual(node._capture_index, 1)

    def test_repeated_save_key_does_not_restart_and_cancel_discards_data(self):
        node = self.node()
        node.process_image_frame(self.image)
        node.handle_save_key()
        node._on_pointcloud(cloud())
        node.handle_save_key()
        self.assertIn("1/2", node.capture_status())
        node.handle_cancel_key()
        node._process_dense_capture()
        self.assertEqual(list((self.root / "images").iterdir()), [])
        self.assertEqual(list((self.root / "pointclouds").iterdir()), [])
        self.assertIn("IDLE", node.capture_status())

    def test_missing_image_does_not_start_collection(self):
        node = self.node()
        node.handle_save_key()
        self.assertIn("IDLE", node.capture_status())

    def test_image_write_failure_cleans_files_and_does_not_count_success(self):
        node = self.node(accumulate_frames=1)
        node.process_image_frame(self.image)
        node.handle_save_key()
        node._on_pointcloud(cloud())
        with mock.patch.object(cv2, "imwrite", return_value=False):
            node._process_dense_capture()
        self.assertEqual(node._capture_index, 0)
        self.assertEqual(list((self.root / "images").iterdir()), [])
        self.assertEqual(list((self.root / "pointclouds").iterdir()), [])
        self.assertIn("IDLE", node.capture_status())

    def test_partial_commit_failure_cleans_new_pair_and_preserves_old_sample(self):
        node = self.node(accumulate_frames=1)
        old = self.root / "images" / "old.png"
        old.write_bytes(b"existing sample")
        node.process_image_frame(self.image)
        node.handle_save_key()
        node._on_pointcloud(cloud())
        link = getattr(self.calib, "os", None)
        self.assertIsNotNone(link, "Non-overwriting pair publication is not implemented")
        original = link.link
        calls = []

        def fail_second(source, destination):
            calls.append(destination)
            if len(calls) == 2:
                raise OSError("disk failure")
            return original(source, destination)

        with mock.patch.object(link, "link", side_effect=fail_second):
            node._process_dense_capture()
        self.assertEqual(old.read_bytes(), b"existing sample")
        self.assertEqual(list((self.root / "images").iterdir()), [old])
        self.assertEqual(list((self.root / "pointclouds").iterdir()), [])
        self.assertEqual(node._capture_index, 0)

    def test_close_discards_unfinished_capture_and_is_idempotent(self):
        node = self.node()
        node.process_image_frame(self.image)
        node.handle_save_key()
        node._on_pointcloud(cloud())
        node.close()
        node.close()
        node._process_dense_capture()
        self.assertEqual(list((self.root / "images").iterdir()), [])
        self.assertIn("IDLE", node.capture_status())

    def test_pcd_write_failure_cleans_files_and_allows_another_round(self):
        node = self.node(accumulate_frames=1)
        node.process_image_frame(self.image)
        node.handle_save_key()
        node._on_pointcloud(cloud())
        with mock.patch.object(self.calib, "save_pointclouds_as_pcd",
                               side_effect=OSError("PCD disk failure")):
            node._process_dense_capture()
        self.assertEqual(node._capture_index, 0)
        self.assertEqual(list((self.root / "images").iterdir()), [])
        self.assertEqual(list((self.root / "pointclouds").iterdir()), [])
        node.handle_save_key()
        node._on_pointcloud(cloud())
        node._process_dense_capture()
        self.assertEqual(node._capture_index, 1)


class PairedCompatibilityTests(DenseTestCase):
    def test_legacy_paired_mode_still_saves_one_latest_cloud(self):
        node = self.node(capture_mode="paired", accumulate_frames=None)
        node.process_image_frame(self.image)
        node._on_pointcloud(cloud(points=((1, 2, 3, 4), (5, 6, 7, 8))))
        node.handle_save_key()
        images = list((self.root / "images").glob("*.png"))
        clouds = list((self.root / "pointclouds").glob("*.pcd"))
        self.assertEqual(len(images), 1)
        self.assertEqual(len(clouds), 1)
        self.assertEqual(images[0].stem, clouds[0].stem)
        self.assertIn("POINTS 2\n", clouds[0].read_text())

    def test_legacy_paired_mode_still_rejects_stale_clouds(self):
        node = self.node(capture_mode="paired", accumulate_frames=None)
        node.process_image_frame(self.image)
        node._on_pointcloud(cloud(stamp=98))
        node.handle_save_key()
        self.assertEqual(list((self.root / "images").iterdir()), [])
        self.assertEqual(list((self.root / "pointclouds").iterdir()), [])


@unittest.skipIf(RosPointCloud2 is None, "ROS sensor_msgs Python packages are unavailable")
class RosReaderIntegrationTests(DenseTestCase):
    def test_real_ros_messages_decode_padded_endian_vector_fields_in_requested_order(self):
        messages = [
            cloud(points=((1, 2, 3, 4, 9, 10), (5, 6, 7, 8, 15, 16)),
                  fields=DEFAULT_FIELDS + (("descriptor", 16, 7, 2),),
                  width=1, padding=4, bigendian=True),
            cloud(points=((19, 20, 14, 11, 12, 13),), fields=(
                ("descriptor", 0, 7, 2), ("intensity", 8, 7, 1),
                ("x", 12, 7, 1), ("y", 16, 7, 1), ("z", 20, 7, 1),
            )),
        ]
        real_messages = []
        for fixture in messages:
            msg = RosPointCloud2()
            msg.header.stamp = RosTime.from_sec(100)
            msg.header.frame_id = "lidar"
            msg.fields = [
                RosPointField(item.name, item.offset, item.datatype, item.count)
                for item in fixture.fields
            ]
            for name in ("height", "width", "point_step", "row_step", "data",
                         "is_bigendian", "is_dense"):
                setattr(msg, name, getattr(fixture, name))
            real_messages.append(msg)
        output = self.root / "ros-reader.pcd"
        with mock.patch.object(self.calib, "point_cloud2", ros_point_cloud2):
            count, _ = self.writer()(
                real_messages, output, ["intensity", "z", "x", "y", "descriptor"]
            )
        self.assertEqual(count, 3)
        self.assertEqual(output.read_text().split("DATA ascii\n")[1].splitlines(), [
            "4.0 3.0 1.0 2.0 9.0 10.0",
            "8.0 7.0 5.0 6.0 15.0 16.0",
            "14.0 13.0 11.0 12.0 19.0 20.0",
        ])


if __name__ == "__main__":
    unittest.main()
