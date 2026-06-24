from __future__ import annotations

from typing import Dict, Optional

import numpy as np
import rclpy
from harvest_interfaces.msg import TwinResidual
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TwinMonitor(Node):
    """Compare synchronized real and simulated joint states."""

    def __init__(self) -> None:
        super().__init__("twin_monitor")
        self.declare_parameter("asset_id", "fanuc_robot")
        self.declare_parameter("real_joint_state_topic", "/joint_states")
        self.declare_parameter("twin_joint_state_topic", "/harvest/twin/joint_states")
        self.declare_parameter("residual_topic", "/harvest/twin/residual")
        self.declare_parameter("maximum_sync_age_sec", 0.10)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.real_state: Optional[JointState] = None
        self.twin_state: Optional[JointState] = None
        self.create_subscription(
            JointState,
            str(self.get_parameter("real_joint_state_topic").value),
            self._on_real_state,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("twin_joint_state_topic").value),
            self._on_twin_state,
            10,
        )
        self.publisher = self.create_publisher(
            TwinResidual,
            str(self.get_parameter("residual_topic").value),
            10,
        )
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish_residual)

    def _on_real_state(self, msg: JointState) -> None:
        self.real_state = msg

    def _on_twin_state(self, msg: JointState) -> None:
        self.twin_state = msg

    @staticmethod
    def _stamp_seconds(msg: JointState) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    @staticmethod
    def _values_by_name(names, values) -> Dict[str, float]:
        return {name: float(values[index]) for index, name in enumerate(names) if index < len(values)}

    def _publish_residual(self) -> None:
        if self.real_state is None or self.twin_state is None:
            return

        real_position = self._values_by_name(self.real_state.name, self.real_state.position)
        twin_position = self._values_by_name(self.twin_state.name, self.twin_state.position)
        real_velocity = self._values_by_name(self.real_state.name, self.real_state.velocity)
        twin_velocity = self._values_by_name(self.twin_state.name, self.twin_state.velocity)
        names = sorted(set(real_position) & set(twin_position))
        if not names:
            return

        position_error = np.asarray(
            [real_position[name] - twin_position[name] for name in names],
            dtype=np.float64,
        )
        velocity_error = np.asarray(
            [real_velocity.get(name, 0.0) - twin_velocity.get(name, 0.0) for name in names],
            dtype=np.float64,
        )
        age = abs(self._stamp_seconds(self.real_state) - self._stamp_seconds(self.twin_state))
        max_age = float(self.get_parameter("maximum_sync_age_sec").value)

        msg = TwinResidual()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.asset_id = str(self.get_parameter("asset_id").value)
        msg.joint_names = names
        msg.position_error = position_error.tolist()
        msg.velocity_error = velocity_error.tolist()
        msg.position_rmse = float(np.sqrt(np.mean(position_error**2)))
        msg.velocity_rmse = float(np.sqrt(np.mean(velocity_error**2)))
        msg.maximum_absolute_position_error = float(np.max(np.abs(position_error)))
        msg.synchronization_age_sec = age
        msg.synchronized = bool(age <= max_age)
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
