import importlib.util
import json
import sys
from pathlib import Path
from unittest import mock

from test_dense_capture import ClockTime, DenseTestCase, cloud
from test_mechanical_capture import mechanical_cloud


class RemoteCaptureTests(DenseTestCase):
    def setUp(self):
        super().setUp()
        path = Path(__file__).resolve().parents[1] / "calib_snap_remote.py"
        spec = importlib.util.spec_from_file_location("calib_snap_remote_under_test", path)
        self.module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, calib_snap=self.calib):
            spec.loader.exec_module(self.module)
        subscriber = mock.Mock(side_effect=lambda *args: mock.Mock(error=None))
        patcher = mock.patch.object(self.module, "RosInput", subscriber)
        self.addCleanup(patcher.stop)
        self.subscribe = patcher.start()
        self.remote = self.module.RemoteCapture(self.root)
        self.addCleanup(self.remote.close)

    def configure(self, cloud_topic):
        return self.remote.configure({
            "image_source": "ros", "image_topic": "/camera/raw", "pointcloud_topic": cloud_topic,
        })["source"]

    def test_discovers_and_applies_independent_masters(self):
        settings = dict(image_source="ros", image_topic="/camera/raw", image_master="192.168.1.7",
                        pointcloud_topic="/lidar/front", pointcloud_master="192.168.2.7:11312",
                        pointcloud_publisher_ip="192.168.2.8")
        image_master = "http://192.168.1.7:11311"
        cloud_master = "http://192.168.2.7:11312"
        published = {
            image_master: [("/camera/raw", "sensor_msgs/Image"), ("/other/cloud", "sensor_msgs/PointCloud2")],
            cloud_master: [("/lidar/front", "sensor_msgs/PointCloud2"), ("/other/image", "sensor_msgs/Image")],
        }
        with mock.patch.object(self.module, "published_topics", side_effect=published.__getitem__):
            self.assertEqual(self.remote.topics(settings), {
                "image": ["/camera/raw"], "pointcloud": ["/lidar/front"], "errors": {},
            })
        self.remote.configure(settings)
        receiver_args = [call.args[:4] for call in self.subscribe.call_args_list]
        self.assertEqual(receiver_args, [
            ("pointcloud", "/lidar/front", cloud_master, "192.168.2.8"),
            ("image", "/camera/raw", image_master, ""),
        ])
        sources = self.remote.sources
        with self.assertRaisesRegex(ValueError, "IPv4"):
            self.remote.configure(dict(settings, image_master="invalid-host"))
        self.assertIs(self.remote.sources, sources)

    def test_switch_discards_previous_frames_and_rejects_stale_revision(self):
        first = self.configure("/lidar/front")
        old_sources = self.remote.sources
        old_sources.accept_image(self.image, ClockTime.now())
        old_sources.on_cloud(cloud(), 100)
        self.assertTrue(self.remote.status({})["source"]["cloud_ready"])
        second = self.configure("/lidar/rear")
        old_sources.cloud_sub.unregister.assert_called_once()
        old_sources.image_sub.unregister.assert_called_once()
        old_sources.on_cloud(cloud(), 100)
        self.assertFalse(self.remote.status({})["source"]["cloud_ready"])
        capture = {"mode": "paired", "id": "a" * 32}
        with self.assertRaisesRegex(ValueError, "configuration has changed"):
            self.remote.capture(dict(capture, revision=first["revision"]))
        with self.assertRaisesRegex(ValueError, "current image and point cloud"):
            self.remote.capture(dict(capture, revision=second["revision"]))

    def test_mechanical_format_checks_current_raw_cloud_only_when_requested(self):
        self.configure("/lidar/front")
        request = {"mode": self.module.MECHANICAL_MODE}
        self.assertIsNone(self.remote.status(request)["source"]["mechanical_format"])
        self.remote.sources.on_cloud(cloud(), 100)
        self.assertNotIn("mechanical_format", self.remote.status({"mode": "dense"})["source"])
        result = self.remote.status(request)["source"]["mechanical_format"]
        self.assertFalse(result["valid"])
        self.assertIn("'ring'", result["error"])

        malformed = mechanical_cloud()
        malformed.data = malformed.data[:-1]
        self.remote.sources.on_cloud(malformed, 100)
        result = self.remote.status(request)["source"]["mechanical_format"]
        self.assertFalse(result["valid"])
        self.assertIn("declared layout", result["error"])

        self.remote.sources.on_cloud(mechanical_cloud(), 100)
        self.assertEqual(self.remote.status(request)["source"]["mechanical_format"],
                         {"valid": True, "scan_id": False})
        first = self.remote.sources
        second = self.configure("/lidar/rear")
        first.on_cloud(mechanical_cloud(), 100)
        result = self.remote.status(request)["source"]
        self.assertEqual(result["revision"], second["revision"])
        self.assertIsNone(result["mechanical_format"])

    def test_mechanical_sample_is_available_only_after_save_and_retained_until_ack(self):
        source = self.configure("/lidar/front")
        self.remote.sources.accept_image(self.image, ClockTime.now())
        self.remote.sources.on_cloud(mechanical_cloud(), 100)
        capture_id = "e" * 32
        self.remote.capture({"id": capture_id, "mode": self.module.MECHANICAL_MODE,
                             "frames": 2, "revision": source["revision"]})
        with self.assertRaisesRegex(ValueError, "in progress"):
            self.configure("/lidar/rear")
        self.assertEqual(self.remote.pending({})["ids"], [])
        self.remote.sources.on_cloud(mechanical_cloud(stamp=100.1), 100.1)
        self.remote.sources.on_cloud(mechanical_cloud(stamp=100.2), 100.2)
        self.remote.worker.join(timeout=3)
        self.assertFalse(self.remote.busy())
        self.assertEqual(self.remote.pending({})["ids"], [capture_id])
        with self.remote.fetch({"id": capture_id, "name": "cloud.pcd"}) as file:
            self.assertIn(b"FIELDS x y z intensity ring scan_id\n", file.read())
        with self.remote.fetch({"id": capture_id, "name": "metadata.json"}) as file:
            data = json.load(file)
        self.assertEqual(data["accepted_frames"], 2)
        self.assertEqual(data["pointcloud_topic"], "/lidar/front")
        self.assertEqual(data["image_stamp_source"], "ros_header")
        self.remote.ack({"id": capture_id})
        self.assertEqual(self.remote.pending({})["ids"], [])

    def test_paired_and_dense_save_xyz_and_optional_fields(self):
        source = self.configure("/lidar/front")
        self.remote.sources.accept_image(self.image, ClockTime.now())
        self.remote.sources.on_cloud(cloud(), 100)
        for mode, capture_id in (("paired", "a" * 32), ("dense", "b" * 32)):
            with self.subTest(mode=mode):
                self.remote.capture({"id": capture_id, "mode": mode, "frames": 1,
                                     "revision": source["revision"]})
                if mode == "dense":
                    self.remote.sources.on_cloud(cloud(stamp=100.1), 100.1)
                self.remote.worker.join(timeout=3)
                self.assertEqual(self.remote.status({})["capture"]["state"], "saved")
                with self.remote.fetch({"id": capture_id, "name": "cloud.pcd"}) as file:
                    self.assertIn(b"FIELDS x y z intensity\n", file.read())

    def test_single_input_capture_subscribes_and_saves_only_the_selected_input(self):
        for mode, input_mode, topic, name, kind, unused in (
            ("image-only", "ros", None, "image.png", "image", "cloud.pcd"),
            ("pointcloud-only", "none", "/lidar/raw", "cloud.pcd", "pointcloud", "image.png"),
        ):
            with self.subTest(mode=mode):
                self.subscribe.reset_mock()
                source = self.remote.configure({
                    "image_source": input_mode, "image_topic": "/camera/raw", "pointcloud_topic": topic,
                })["source"]
                self.subscribe.assert_called_once()
                self.assertEqual(self.subscribe.call_args.args[1], topic or "/camera/raw")
                if topic:
                    self.remote.sources.on_cloud(cloud(), 100)
                else:
                    self.remote.sources.accept_image(self.image, ClockTime.now())
                capture_id = ("a" if topic else "b") * 32
                self.remote.capture({"id": capture_id, "mode": mode, "revision": source["revision"]})
                self.remote.worker.join(timeout=3)
                self.assertEqual(self.remote.status({})["capture"]["state"], "saved")
                with self.remote.fetch({"id": capture_id, "name": "metadata.json"}) as file:
                    metadata = json.load(file)
                self.assertEqual(metadata["files"], {"metadata": "metadata.json", kind: name})
                with self.assertRaisesRegex(ValueError, "does not exist"):
                    self.remote.fetch({"id": capture_id, "name": unused})
                self.remote.ack({"id": capture_id})
