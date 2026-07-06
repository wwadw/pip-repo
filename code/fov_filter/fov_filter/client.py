from __future__ import annotations

import copy
import json
import threading
import time
import uuid
from typing import Any, Dict, Optional


class FovFilterRosClient:
    def __init__(
        self,
        command_topic: str = "/fov_filter/command",
        state_topic: str = "/fov_filter/state",
        node_name: str = "fov_filter_client",
        anonymous: bool = True,
        init_node: bool = True,
    ) -> None:
        try:
            import rospy
            from std_msgs.msg import String
        except Exception as exc:
            raise RuntimeError("rospy/std_msgs 不可用，请先 source ROS1 环境") from exc

        self.rospy = rospy
        self.String = String
        self.command_topic = command_topic
        self.state_topic = state_topic
        self._lock = threading.RLock()
        self._state_event = threading.Event()
        self._latest_state_raw: Optional[str] = None
        self._latest_state: Optional[Dict[str, Any]] = None
        self._latest_state_wall_time = 0.0

        if init_node and not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=anonymous, disable_signals=True)

        self.publisher = rospy.Publisher(self.command_topic, String, queue_size=1)
        self.subscriber = rospy.Subscriber(
            self.state_topic,
            String,
            self._state_callback,
            queue_size=10,
        )

    def _state_callback(self, message) -> None:
        try:
            parsed = json.loads(message.data)
        except Exception:
            parsed = None

        with self._lock:
            self._latest_state_raw = message.data
            self._latest_state = parsed
            self._latest_state_wall_time = time.time()
            self._state_event.set()

    def latest_state_raw(self) -> Optional[str]:
        with self._lock:
            return self._latest_state_raw

    def latest_state(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            return copy.deepcopy(self._latest_state)

    def wait_for_state(self, timeout: float = 2.0) -> Dict[str, Any]:
        latest = self.latest_state()
        if latest is not None:
            return latest

        if not self._state_event.wait(timeout):
            raise TimeoutError(f"等待状态超时: {self.state_topic}")

        latest = self.latest_state()
        if latest is None:
            raise TimeoutError(f"状态话题未返回有效 JSON: {self.state_topic}")
        return latest

    def _wait_for_connection(self, timeout: float) -> None:
        start = time.time()
        while self.publisher.get_num_connections() == 0:
            if time.time() - start >= timeout:
                raise TimeoutError(f"命令话题无订阅者: {self.command_topic}")
            self.rospy.sleep(0.05)

    def wait_for_command(self, command_id: str, timeout: float = 2.0) -> Dict[str, Any]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            latest = self.latest_state()
            if latest is not None and latest.get("last_command_id") == command_id:
                return latest

            remaining = max(0.0, deadline - time.time())
            if not self._state_event.wait(min(0.2, remaining)):
                continue
            self._state_event.clear()

        raise TimeoutError(f"等待命令回执超时: {command_id}")

    def send_command(
        self,
        payload: Dict[str, Any],
        timeout: float = 2.0,
        wait: bool = True,
    ) -> Optional[Dict[str, Any]]:
        command = dict(payload)
        command_id = str(command.get("command_id") or uuid.uuid4().hex)
        command["command_id"] = command_id

        self._wait_for_connection(timeout)
        self.publisher.publish(self.String(data=json.dumps(command, ensure_ascii=False)))
        if not wait:
            return None
        return self.wait_for_command(command_id, timeout=timeout)

    def request_status(self, timeout: float = 2.0) -> Dict[str, Any]:
        return self.send_command({"op": "status"}, timeout=timeout, wait=True) or {}
