"""Advertise a capture node and accept sessions on the local network."""

import fcntl
from ipaddress import IPv4Address
import json
from pathlib import Path
import re
import socket
import struct
import sys
import threading
from uuid import uuid4


DISCOVERY_PORT = 45817
CAPTURE_PORT = 45818
BROADCAST_INTERVAL = 2
MAX_ANNOUNCEMENT = 4096
SIOCGIFFLAGS = 0x8913
SIOCGIFBRDADDR = 0x8919
IFF_UP_AND_BROADCAST = 0x3
SOURCE_DEFAULT_FIELDS = {
    "image_source", "image_topic", "pointcloud_topic", "image_master", "pointcloud_master",
    "image_publisher_ip", "pointcloud_publisher_ip",
}


def interface_broadcasts(sender):
    addresses = set()
    for _, name in socket.if_nameindex():
        request = struct.pack("256s", name.encode())
        try:
            flags, = struct.unpack_from("H", fcntl.ioctl(sender, SIOCGIFFLAGS, request), 16)
            if flags & IFF_UP_AND_BROADCAST != IFF_UP_AND_BROADCAST:
                continue
            address = socket.inet_ntoa(fcntl.ioctl(sender, SIOCGIFBRDADDR, request)[20:24])
            if address != "0.0.0.0":
                addresses.add(address)
        except OSError:
            continue
    return addresses or {"255.255.255.255"}


def announce(sender, target, port, packet, stopped):
    while not stopped.is_set():
        addresses = (target,) if target else interface_broadcasts(sender)
        for address in addresses:
            try:
                sender.sendto(packet, (address, port))
            except OSError as error:
                print(f"calib-snap-remote: discovery broadcast to {address} failed: {error}", file=sys.stderr)
        stopped.wait(BROADCAST_INTERVAL)


def persistent_id(spool):
    identity = spool / "node-id"
    if not identity.exists():
        temporary = spool / "node-id.tmp"
        temporary.write_text(uuid4().hex + "\n", encoding="ascii")
        temporary.replace(identity)
    return identity.read_text(encoding="ascii").strip()


def serve_network(path, spool, serve, idle_timeout, is_shutdown):
    settings_path = Path(path).expanduser() if path else Path("/etc/calib-snap/node.json")
    settings = json.loads(settings_path.read_text(encoding="utf-8")) if path or settings_path.is_file() else {}
    robot_id = settings.get("id") or persistent_id(spool)
    name = settings.get("name", socket.gethostname()[:96])
    defaults = settings.get("source_defaults", {"image_source": "ros"})
    if not isinstance(robot_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,48}", robot_id):
        raise ValueError("The node ID must contain 1–48 letters, digits, underscores or hyphens.")
    if not isinstance(name, str) or not name.strip() or len(name) > 96:
        raise ValueError("The node name must contain 1–96 characters.")
    if not isinstance(defaults, dict) or any(
        key not in SOURCE_DEFAULT_FIELDS or not isinstance(value, str) for key, value in defaults.items()
    ):
        raise ValueError("Invalid capture source defaults.")

    target = settings.get("broadcast")
    discovery_port = settings.get("discovery_port", DISCOVERY_PORT)

    with socket.socket() as listener, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((settings.get("bind", "0.0.0.0"), settings.get("port", CAPTURE_PORT)))
        listener.listen()
        listener.settimeout(1)
        sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        identity = dict(discovery="calib-snap-lan-v1", id=robot_id, name=name,
                        port=listener.getsockname()[1], source_defaults=defaults)
        packet = json.dumps(identity, ensure_ascii=False).encode()
        if len(packet) > MAX_ANNOUNCEMENT:
            raise ValueError("The node announcement exceeds 4096 bytes.")
        stopped = threading.Event()
        broadcaster = threading.Thread(target=announce, args=(sender, target, discovery_port, packet, stopped),
                                       daemon=True)
        broadcaster.start()
        try:
            while not is_shutdown():
                try:
                    connection, (host, _) = listener.accept()
                except socket.timeout:
                    continue
                with connection:
                    if not IPv4Address(host).is_private:
                        continue
                    try:
                        connection.settimeout(idle_timeout)
                        with connection.makefile("rb") as reader, connection.makefile("wb") as writer:
                            serve(reader, writer, robot_id)
                    except OSError:
                        continue
        finally:
            stopped.set()
            broadcaster.join()
