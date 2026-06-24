from __future__ import annotations

import json
import math
import socket
import time
from typing import Any, Dict

import rclpy
from denso_rc7m_interfaces.msg import ControllerState
from rclpy.node import Node
from sensor_msgs.msg import JointState


PROTOCOL = "denso_rc7m.telemetry.v1"


class TelemetryBridge(Node):
    """Receive validated, read-only RC7M telemetry and publish ROS state."""

    def __init__(self) -> None:
        super().__init__("telemetry_bridge")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("udp_port", 15000)
        self.declare_parameter("maximum_packet_bytes", 8192)
        self.declare_parameter("stale_timeout_sec", 0.25)
        self.declare_parameter("joint_state_topic", "/denso/joint_states")
        self.declare_parameter("controller_state_topic", "/denso/controller_state")
        self.declare_parameter("frame_id", "denso_base")

        self.joint_publisher = self.create_publisher(
            JointState, str(self.get_parameter("joint_state_topic").value), 10
        )
        self.state_publisher = self.create_publisher(
            ControllerState,
            str(self.get_parameter("controller_state_topic").value),
            10,
        )
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind(
            (
                str(self.get_parameter("bind_address").value),
                int(self.get_parameter("udp_port").value),
            )
        )
        self.last_sequence = None
        self.dropped_packets = 0
        self.create_timer(0.002, self._poll)
        self.get_logger().info(
            f"Telemetry-only RC7M bridge listening on UDP "
            f"{self.get_parameter('bind_address').value}:"
            f"{self.get_parameter('udp_port').value}"
        )

    @staticmethod
    def _required(packet: Dict[str, Any], key: str, expected_type):
        value = packet.get(key)
        if not isinstance(value, expected_type):
            raise ValueError(f"{key} has invalid type")
        return value

    def _validate(self, packet: Dict[str, Any]) -> Dict[str, Any]:
        if packet.get("protocol") != PROTOCOL:
            raise ValueError("unsupported protocol")
        sequence = self._required(packet, "sequence", int)
        source_time = self._required(packet, "source_time_unix_ns", int)
        names = self._required(packet, "joint_names", list)
        positions = self._required(packet, "joint_position_deg", list)
        if len(names) != 6 or len(positions) != 6:
            raise ValueError("six joint names and positions are required")
        if len(set(str(name) for name in names)) != 6:
            raise ValueError("joint names must be unique")
        numeric_positions = [float(value) for value in positions]
        if not all(math.isfinite(value) and abs(value) <= 720.0 for value in numeric_positions):
            raise ValueError("joint position outside telemetry bounds")
        velocities = packet.get("joint_velocity_deg_s", [0.0] * 6)
        if not isinstance(velocities, list) or len(velocities) != 6:
            raise ValueError("joint velocity must contain six values")
        numeric_velocities = [float(value) for value in velocities]
        if not all(math.isfinite(value) and abs(value) <= 2000.0 for value in numeric_velocities):
            raise ValueError("joint velocity outside telemetry bounds")
        packet["sequence"] = sequence
        packet["source_time_unix_ns"] = source_time
        packet["joint_names"] = [str(value)[:32] for value in names]
        packet["joint_position_deg"] = numeric_positions
        packet["joint_velocity_deg_s"] = numeric_velocities
        return packet

    def _poll(self) -> None:
        maximum = int(self.get_parameter("maximum_packet_bytes").value)
        for _ in range(20):
            try:
                payload, _address = self.socket.recvfrom(maximum + 1)
            except BlockingIOError:
                return
            if len(payload) > maximum:
                self.get_logger().warning("Rejected oversized telemetry packet.")
                continue
            try:
                packet = self._validate(json.loads(payload.decode("utf-8")))
                self._publish(packet)
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError, TypeError) as exc:
                self.get_logger().warning(f"Rejected telemetry packet: {exc}")

    def _publish(self, packet: Dict[str, Any]) -> None:
        sequence = packet["sequence"]
        if self.last_sequence is not None:
            if sequence <= self.last_sequence:
                return
            if sequence > self.last_sequence + 1:
                self.dropped_packets += sequence - self.last_sequence - 1
        self.last_sequence = sequence

        now = self.get_clock().now()
        packet_age = max(
            0.0,
            time.time() - packet["source_time_unix_ns"] / 1_000_000_000.0,
        )
        stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        if packet_age > stale_timeout:
            self.get_logger().warning(
                f"Rejected stale packet: age={packet_age:.3f}s"
            )
            return

        joint = JointState()
        joint.header.stamp = now.to_msg()
        joint.header.frame_id = str(self.get_parameter("frame_id").value)
        joint.name = packet["joint_names"]
        joint.position = [math.radians(value) for value in packet["joint_position_deg"]]
        joint.velocity = [
            math.radians(value) for value in packet["joint_velocity_deg_s"]
        ]
        self.joint_publisher.publish(joint)

        state = ControllerState()
        state.header = joint.header
        state.controller_model = str(packet.get("controller", "RC7M"))[:32]
        state.robot_model = str(packet.get("robot", "VS-6556E"))[:32]
        state.sequence = sequence
        state.source_time_unix_ns = packet["source_time_unix_ns"]
        state.mode = str(packet.get("mode", "UNKNOWN"))[:16]
        state.servo_on = bool(packet.get("servo_on", False))
        state.emergency_stop = bool(packet.get("emergency_stop", False))
        state.protective_stop = bool(packet.get("protective_stop", False))
        state.alarm_code = int(packet.get("alarm_code", 0))
        state.alarm_text = str(packet.get("alarm_text", ""))[:256]
        state.native_task = str(packet.get("native_task", ""))[:64]
        state.cycle_count = max(0, int(packet.get("cycle_count", 0)))
        state.packet_age_sec = packet_age
        state.dropped_packets = self.dropped_packets
        self.state_publisher.publish(state)

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
