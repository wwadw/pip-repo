"""ROS 1 discovery and isolated subscriptions for remote capture."""

from io import BytesIO
from ipaddress import IPv4Address
import os
import socket
import struct
import subprocess
import sys
import threading
from urllib.parse import urlsplit
from xmlrpc.client import ServerProxy, Transport


DEFAULT_MASTER = "http://127.0.0.1:11311"
TOPIC_TYPES = {"image": "sensor_msgs/Image", "pointcloud": "sensor_msgs/PointCloud2"}
FRAME_HEADER = struct.Struct("<Id")
MAX_FRAME = 256 * 1024**2


def master_uri(value):
    if value in (None, ""):
        return os.environ.get("ROS_MASTER_URI") or DEFAULT_MASTER
    try:
        if not isinstance(value, str) or any(char.isspace() for char in value):
            raise ValueError()
        parsed = urlsplit(value if "://" in value else "http://" + value)
        if (parsed.scheme != "http" or parsed.username is not None or parsed.query
                or parsed.fragment or parsed.path not in ("", "/") or parsed.port == 0):
            raise ValueError()
        return f"http://{IPv4Address(parsed.hostname)}:{parsed.port or 11311}"
    except ValueError as exc:
        raise ValueError("ROS Master must be an IPv4 address with an optional port.") from exc


def publisher_ip(value):
    if value in (None, ""):
        return ""
    try:
        if not isinstance(value, str):
            raise ValueError()
        return str(IPv4Address(value))
    except ValueError as exc:
        raise ValueError("The ROS publisher IP must be an IPv4 address.") from exc


class DiscoveryTransport(Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 4
        return connection


def published_topics(uri):
    try:
        with ServerProxy(uri, transport=DiscoveryTransport()) as master:
            code, message, topics = master.getPublishedTopics("/calib_snap_remote", "/")
        if code != 1:
            raise ValueError(message)
    except Exception as exc:
        raise ValueError(f"Could not list ROS topics from {uri}. Check the Master IP and port.") from exc
    return topics


class RosInput:
    def __init__(self, kind, topic, uri, ip, message_type, callback):
        self.error = None
        self.stop = threading.Event()
        self.connection, child = socket.socketpair()
        try:
            with child:
                self.process = subprocess.Popen(
                    [sys.executable, "-m", "calib_snap_ros", str(child.fileno()), kind, topic, uri, ip],
                    pass_fds=(child.fileno(),), stdin=subprocess.DEVNULL, stdout=sys.stderr,
                )
        except OSError:
            self.connection.close()
            raise
        self.reader = threading.Thread(
            target=self.read, args=(kind, uri, message_type, callback), daemon=True,
        )
        self.reader.start()

    def read(self, kind, uri, message_type, callback):
        try:
            with self.connection.makefile("rb") as stream:
                while not self.stop.is_set():
                    header = stream.read(FRAME_HEADER.size)
                    if len(header) != FRAME_HEADER.size:
                        return
                    length, ros_now = FRAME_HEADER.unpack(header)
                    if not 0 < length <= MAX_FRAME:
                        return
                    data = stream.read(length)
                    if len(data) != length:
                        return
                    callback(message_type().deserialize(data), ros_now)
        except Exception as exc:
            print(f"calib-snap-remote: {kind} receiver: {exc}", file=sys.stderr)
        finally:
            if not self.stop.is_set():
                self.error = f"ROS {kind} receiver stopped for {uri}. Check the source settings and apply again."
                self.connection.shutdown(socket.SHUT_RDWR)

    def unregister(self):
        self.stop.set()
        self.connection.shutdown(socket.SHUT_RDWR)
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.reader.join()
        self.connection.close()


def use_publisher_ip(ip):
    resolve = socket.getaddrinfo
    connect = socket.socket.connect

    def resolve_with_fallback(host, *args, **kwargs):
        try:
            return resolve(host, *args, **kwargs)
        except socket.gaierror:
            return resolve(ip, *args, **kwargs)

    def connect_with_fallback(connection, address):
        try:
            return connect(connection, address)
        except socket.gaierror:
            if connection.family != socket.AF_INET:
                raise
            return connect(connection, (ip, address[1]))

    socket.getaddrinfo = resolve_with_fallback
    socket.socket.connect = connect_with_fallback


def receive(fd, kind, topic, uri, ip):
    target = urlsplit(uri)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route:
        route.connect((target.hostname, target.port or 11311))
        os.environ["ROS_IP"] = route.getsockname()[0]
    os.environ.pop("ROS_HOSTNAME", None)
    os.environ["ROS_MASTER_URI"] = uri
    if ip:
        use_publisher_ip(ip)

    import rospy
    from sensor_msgs.msg import Image, PointCloud2

    with socket.socket(fileno=int(fd)) as connection:
        write_lock = threading.Lock()

        def send(message):
            ros_now = rospy.Time.now().to_sec()
            data = BytesIO()
            message.serialize(data)
            payload = data.getbuffer()
            if len(payload) <= MAX_FRAME:
                with write_lock:
                    try:
                        connection.sendall(FRAME_HEADER.pack(len(payload), ros_now))
                        connection.sendall(payload)
                    except OSError:
                        return

        timeout = threading.Timer(8, os._exit, args=(1,))
        timeout.start()
        try:
            rospy.init_node("calib_snap_" + kind, anonymous=True, disable_rosout=True,
                            argv=["calib-snap-remote"])
            subscriber = rospy.Subscriber(
                topic, Image if kind == "image" else PointCloud2, send,
                queue_size=1 if kind == "image" else 2, buff_size=1 << 24,
            )
        finally:
            timeout.cancel()
        try:
            connection.recv(1)
        finally:
            subscriber.unregister()
            rospy.signal_shutdown("Capture source disconnected")


if __name__ == "__main__":
    receive(*sys.argv[1:])
