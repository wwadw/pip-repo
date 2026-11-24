"""Advertise a capture node and accept mutually authenticated TLS sessions."""

import hmac
import json
from pathlib import Path
import re
import socket
import ssl
import sys
import threading
import time


DISCOVERY_PORT = 45817
CAPTURE_PORT = 45818
BROADCAST_INTERVAL = 2
MAX_ANNOUNCEMENT = 4096
SOURCE_DEFAULT_FIELDS = {
    "image_source", "image_topic", "pointcloud_topic", "image_master", "pointcloud_master",
    "image_publisher_ip", "pointcloud_publisher_ip",
}


def announce(sender, target, identity, secret, stopped):
    while not stopped.is_set():
        body = json.dumps(dict(identity, sent=int(time.time())), ensure_ascii=False).encode()
        packet = hmac.digest(secret, body, "sha256").hex().encode() + b"\n" + body
        try:
            sender.sendto(packet, target)
        except OSError as error:
            print(f"calib-snap-remote: discovery broadcast failed: {error}", file=sys.stderr)
        stopped.wait(BROADCAST_INTERVAL)


def serve_network(path, serve, idle_timeout, is_shutdown):
    path = Path(path).expanduser().resolve()
    settings = json.loads(path.read_text(encoding="utf-8"))
    secret = (path.parent / settings["discovery_key"]).read_bytes().strip()
    if len(secret) < 32:
        raise ValueError("The discovery key must contain at least 32 bytes.")
    robot_id, name = settings["id"], settings["name"]
    defaults = settings.get("source_defaults", {})
    if not isinstance(robot_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,48}", robot_id):
        raise ValueError("The node ID must contain 1–48 letters, digits, underscores or hyphens.")
    if not isinstance(name, str) or not name.strip() or len(name) > 96:
        raise ValueError("The node name must contain 1–96 characters.")
    if not isinstance(defaults, dict) or any(
        key not in SOURCE_DEFAULT_FIELDS or not isinstance(value, str) for key, value in defaults.items()
    ):
        raise ValueError("Invalid capture source defaults.")

    tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    tls.minimum_version = ssl.TLSVersion.TLSv1_2
    tls.load_cert_chain(path.parent / settings["certificate"], path.parent / settings["private_key"])
    tls.load_verify_locations(cafile=str(path.parent / settings["ca"]))
    tls.verify_mode = ssl.CERT_REQUIRED
    target = (settings.get("broadcast", "255.255.255.255"), settings.get("discovery_port", DISCOVERY_PORT))

    with socket.socket() as listener, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((settings.get("bind", "0.0.0.0"), settings.get("port", CAPTURE_PORT)))
        listener.listen()
        listener.settimeout(1)
        sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        identity = dict(discovery="calib-snap-v1", id=robot_id, name=name,
                        port=listener.getsockname()[1], source_defaults=defaults)
        if len(json.dumps(dict(identity, sent=int(time.time())), ensure_ascii=False).encode()) + 65 > MAX_ANNOUNCEMENT:
            raise ValueError("The node announcement exceeds 4096 bytes.")
        stopped = threading.Event()
        broadcaster = threading.Thread(target=announce, args=(sender, target, identity, secret, stopped), daemon=True)
        broadcaster.start()
        try:
            while not is_shutdown():
                try:
                    connection, _ = listener.accept()
                except socket.timeout:
                    continue
                with connection:
                    try:
                        connection.settimeout(5)
                        with tls.wrap_socket(connection, server_side=True) as channel:
                            channel.settimeout(idle_timeout)
                            with channel.makefile("rb") as reader, channel.makefile("wb") as writer:
                                serve(reader, writer)
                    except OSError:
                        continue
        finally:
            stopped.set()
            broadcaster.join()
