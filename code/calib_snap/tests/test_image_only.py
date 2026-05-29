import importlib.util
import io
import sys
import tempfile
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "calib_snap.py"


def build_stub_modules():
    rospy = types.ModuleType("rospy")

    class FakeTime:
        @staticmethod
        def now():
            return FakeTime()

        def __sub__(self, other):
            return SimpleNamespace(to_sec=lambda: 0.0)

        def __eq__(self, other):
            return isinstance(other, FakeTime)

    class FakeSubscriber:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

        def unregister(self):
            pass

    class FakePublisher:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

        def publish(self, msg):
            pass

    rospy.Time = FakeTime
    rospy.Subscriber = FakeSubscriber
    rospy.Publisher = FakePublisher
    rospy.Rate = lambda hz: SimpleNamespace(sleep=lambda: None)
    rospy.init_node = lambda *args, **kwargs: None
    rospy.is_shutdown = lambda: True
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.logwarn = lambda *args, **kwargs: None
    rospy.logerr = lambda *args, **kwargs: None
    rospy.loginfo_throttle = lambda *args, **kwargs: None
    rospy.logwarn_throttle = lambda *args, **kwargs: None

    sensor_msgs = types.ModuleType("sensor_msgs")
    point_cloud2 = types.ModuleType("sensor_msgs.point_cloud2")
    point_cloud2.read_points = lambda *args, **kwargs: []

    msg = types.ModuleType("sensor_msgs.msg")

    class FakePointField:
        INT8 = 1
        UINT8 = 2
        INT16 = 3
        UINT16 = 4
        INT32 = 5
        UINT32 = 6
        FLOAT32 = 7
        FLOAT64 = 8

    class FakeImage:
        pass

    class FakeCompressedImage:
        pass

    class FakePointCloud2:
        pass

    msg.PointField = FakePointField
    msg.Image = FakeImage
    msg.CompressedImage = FakeCompressedImage
    msg.PointCloud2 = FakePointCloud2
    sensor_msgs.point_cloud2 = point_cloud2

    cv2 = types.ModuleType("cv2")
    cv2.IMWRITE_JPEG_QUALITY = 1
    cv2.IMWRITE_PNG_COMPRESSION = 16
    cv2.CAP_PROP_FRAME_WIDTH = 3
    cv2.CAP_PROP_FRAME_HEIGHT = 4
    cv2.COLOR_GRAY2BGR = 100
    cv2.COLOR_RGB2BGR = 101
    cv2.COLOR_BGRA2BGR = 102
    cv2.COLOR_RGBA2BGR = 103
    cv2.cvtColor = lambda frame, code: frame
    cv2.imencode = lambda ext, frame: (True, SimpleNamespace(tobytes=lambda: b"jpg"))
    cv2.VideoCapture = lambda source: SimpleNamespace(isOpened=lambda: False)

    numpy = types.ModuleType("numpy")
    numpy.uint8 = "uint8"
    numpy.frombuffer = lambda data, dtype=None: data

    cv_bridge = types.ModuleType("cv_bridge")

    class FakeCvBridge:
        def imgmsg_to_cv2(self, msg, desired_encoding="bgr8"):
            return getattr(msg, "frame", None)

        def cv2_to_imgmsg(self, frame, encoding="bgr8"):
            return SimpleNamespace(
                header=SimpleNamespace(stamp=None, frame_id=""),
                height=1,
                width=1,
                encoding=encoding,
                step=3,
                data=b"",
            )

    cv_bridge.CvBridge = FakeCvBridge

    return {
        "rospy": rospy,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.point_cloud2": point_cloud2,
        "sensor_msgs.msg": msg,
        "cv2": cv2,
        "numpy": numpy,
        "cv_bridge": cv_bridge,
    }


def load_calib_snap():
    spec = importlib.util.spec_from_file_location("calib_snap_under_test", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class ImageOnlyCaptureTests(unittest.TestCase):
    def setUp(self):
        self.module_patcher = mock.patch.dict(sys.modules, build_stub_modules())
        self.module_patcher.start()
        self.addCleanup(self.module_patcher.stop)
        self.calib_snap = load_calib_snap()

    def test_auto_without_pointcloud_defaults_to_image_mode(self):
        args = SimpleNamespace(capture_mode="auto", pointcloud_topic=None)

        self.assertEqual(self.calib_snap.resolve_capture_mode(args), "image")

    def test_auto_with_pointcloud_defaults_to_paired_mode(self):
        args = SimpleNamespace(capture_mode="auto", pointcloud_topic="/livox/lidar")

        self.assertEqual(self.calib_snap.resolve_capture_mode(args), "paired")

    def test_paired_mode_requires_pointcloud_topic(self):
        args = SimpleNamespace(capture_mode="paired", pointcloud_topic=None)

        with self.assertRaisesRegex(ValueError, "--pointcloud-topic"):
            self.calib_snap.resolve_capture_mode(args)

    def test_image_mode_ignores_pointcloud_topic(self):
        args = SimpleNamespace(capture_mode="image", pointcloud_topic="/livox/lidar")

        self.assertEqual(self.calib_snap.resolve_capture_mode(args), "image")

    def test_camera_size_options_parse_as_width_and_height(self):
        argv = [
            "calib_snap",
            "--input-mode",
            "camera",
            "--camera-source",
            "0",
            "--camera-width",
            "1280",
            "--camera-height",
            "720",
        ]

        with mock.patch.object(sys, "argv", argv):
            args = self.calib_snap.parse_args()

        self.assertEqual(args.camera_width, 1280)
        self.assertEqual(args.camera_height, 720)

    def test_camera_size_options_must_be_provided_together(self):
        argv = [
            "calib_snap",
            "--input-mode",
            "camera",
            "--camera-source",
            "0",
            "--camera-width",
            "1280",
        ]

        with mock.patch.object(sys, "argv", argv), mock.patch(
            "sys.stderr", new_callable=io.StringIO
        ) as stderr:
            with self.assertRaises(SystemExit):
                self.calib_snap.parse_args()

        self.assertIn(
            "--camera-width and --camera-height must be provided together",
            stderr.getvalue(),
        )

    def test_camera_size_options_are_only_supported_in_camera_mode(self):
        argv = [
            "calib_snap",
            "--input-mode",
            "rtsp",
            "--rtsp-uri",
            "rtsp://127.0.0.1:8554/test",
            "--camera-width",
            "1280",
            "--camera-height",
            "720",
        ]

        with mock.patch.object(sys, "argv", argv), mock.patch(
            "sys.stderr", new_callable=io.StringIO
        ) as stderr:
            with self.assertRaises(SystemExit):
                self.calib_snap.parse_args()

        self.assertIn(
            "--camera-width/--camera-height are only supported when --input-mode=camera",
            stderr.getvalue(),
        )

    def test_camera_capture_size_is_applied_to_video_capture(self):
        calls = []

        class FakeCapture:
            def set(self, prop, value):
                calls.append((prop, value))
                return True

        args = SimpleNamespace(
            input_mode="camera",
            camera_width=1280,
            camera_height=720,
        )

        self.calib_snap.configure_camera_capture_size(FakeCapture(), args)

        self.assertEqual(
            calls,
            [
                (self.calib_snap.cv2.CAP_PROP_FRAME_WIDTH, 1280),
                (self.calib_snap.cv2.CAP_PROP_FRAME_HEIGHT, 720),
            ],
        )

    def test_image_file_writer_uses_save_dir_prefix_extension_and_quality(self):
        calls = []

        def fake_imwrite(path, frame, params):
            calls.append((Path(path), params))
            return True

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            self.calib_snap.cv2.imwrite = fake_imwrite
            writer = self.calib_snap.ImageFileWriter(
                save_dir=tmp_path / "images",
                image_ext="jpeg",
                prefix="manual",
                quality=87,
            )

            saved_path = writer.save(object())

            self.assertEqual(saved_path.parent, tmp_path / "images")
            self.assertTrue(saved_path.name.startswith("manual_"))
            self.assertEqual(saved_path.suffix, ".jpg")
            self.assertTrue((tmp_path / "images").is_dir())
            self.assertEqual(
                calls, [(saved_path, [self.calib_snap.cv2.IMWRITE_JPEG_QUALITY, 87])]
            )
            self.assertEqual(writer.saved_count, 1)

    def build_image_node(self, writer, image_save_mode="interval", image_interval=2):
        return self.calib_snap.CalibrationCaptureNode(
            input_mode="camera",
            capture_mode="image",
            pointcloud_topic=None,
            source_image_topic=None,
            image_topic="/calib/image_raw",
            compressed_topic="/calib/image_raw/compressed",
            frame_id="camera",
            output_dir=Path(tempfile.gettempdir()) / "unused-calib-output",
            image_ext="jpg",
            requested_pcd_fields=[],
            max_pointcloud_age=0.5,
            image_writer=writer,
            image_save_mode=image_save_mode,
            image_interval=image_interval,
        )

    def test_image_mode_does_not_subscribe_to_pointcloud_and_saves_by_interval(self):
        saved_frames = []

        class FakeWriter:
            saved_count = 0

            def save(self, frame):
                saved_frames.append(frame)
                self.saved_count += 1
                return Path("x.jpg")

        writer = FakeWriter()

        node = self.build_image_node(writer, image_save_mode="interval", image_interval=2)
        node.process_image_frame("frame-1")
        node.process_image_frame("frame-2")
        node.process_image_frame("frame-3")
        node.process_image_frame("frame-4")

        self.assertIsNone(node._pointcloud_sub)
        self.assertEqual(saved_frames, ["frame-2", "frame-4"])

    def test_manual_image_mode_saves_only_when_requested(self):
        saved_frames = []

        class FakeWriter:
            saved_count = 0

            def save(self, frame):
                saved_frames.append(frame)
                self.saved_count += 1
                return Path("x.jpg")

        writer = FakeWriter()

        node = self.build_image_node(writer, image_save_mode="manual", image_interval=1)
        node.process_image_frame("frame-1")
        node.process_image_frame("frame-2")
        node.save_image_capture()

        self.assertEqual(saved_frames, ["frame-2"])


if __name__ == "__main__":
    unittest.main()
