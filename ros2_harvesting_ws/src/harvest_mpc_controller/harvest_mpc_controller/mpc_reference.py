from __future__ import annotations

import time
from typing import Dict, Optional

import numpy as np
import rclpy
from harvest_interfaces.msg import MpcStatus
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MpcReferenceNode(Node):
    """Finite-horizon linear MPC prototype.

    It publishes a shadow-mode velocity reference. It is not a ros2_control
    hardware controller and must not be connected directly to physical motion.
    """

    def __init__(self) -> None:
        super().__init__("mpc_reference")
        defaults = {
            "state_topic": "/joint_states",
            "target_topic": "/harvest/mpc/target",
            "reference_topic": "/harvest/mpc/reference",
            "status_topic": "/harvest/mpc/status",
            "update_rate_hz": 20.0,
            "state_timeout_sec": 0.10,
            "horizon_steps": 15,
            "model_dt_sec": 0.05,
            "position_weight": 20.0,
            "velocity_weight": 2.0,
            "acceleration_weight": 0.20,
            "maximum_velocity_rad_s": 0.25,
            "maximum_acceleration_rad_s2": 0.50,
            "shadow_mode": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.state: Optional[JointState] = None
        self.target: Optional[JointState] = None
        self.state_received_monotonic = 0.0
        self.create_subscription(
            JointState, str(self.get_parameter("state_topic").value), self._on_state, 10
        )
        self.create_subscription(
            JointState, str(self.get_parameter("target_topic").value), self._on_target, 10
        )
        self.reference_publisher = self.create_publisher(
            JointState, str(self.get_parameter("reference_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            MpcStatus, str(self.get_parameter("status_topic").value), 10
        )
        rate = max(1.0, float(self.get_parameter("update_rate_hz").value))
        self.create_timer(1.0 / rate, self._update)

    def _on_state(self, msg: JointState) -> None:
        self.state = msg
        self.state_received_monotonic = time.monotonic()

    def _on_target(self, msg: JointState) -> None:
        self.target = msg

    @staticmethod
    def _map(names, values) -> Dict[str, float]:
        return {name: float(values[i]) for i, name in enumerate(names) if i < len(values)}

    def _publish_invalid(self, reason: str, solve_ms: float = 0.0) -> None:
        status = MpcStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.valid = False
        status.solver_status = "INVALID"
        status.solve_time_ms = solve_ms
        status.fallback_reason = reason
        self.status_publisher.publish(status)

    def _first_control(self, q: np.ndarray, qd: np.ndarray, q_ref: np.ndarray) -> np.ndarray:
        """Solve an unconstrained finite-horizon quadratic problem, then project u."""
        n = len(q)
        horizon = max(2, int(self.get_parameter("horizon_steps").value))
        dt = float(self.get_parameter("model_dt_sec").value)
        q_weight = float(self.get_parameter("position_weight").value)
        v_weight = float(self.get_parameter("velocity_weight").value)
        u_weight = float(self.get_parameter("acceleration_weight").value)
        a = np.block([
            [np.eye(n), dt * np.eye(n)],
            [np.zeros((n, n)), np.eye(n)],
        ])
        b = np.vstack([0.5 * dt * dt * np.eye(n), dt * np.eye(n)])
        state_size = 2 * n
        sx = np.zeros((horizon * state_size, state_size))
        su = np.zeros((horizon * state_size, horizon * n))
        for row in range(horizon):
            sx[row * state_size:(row + 1) * state_size] = np.linalg.matrix_power(a, row + 1)
            for col in range(row + 1):
                su[
                    row * state_size:(row + 1) * state_size,
                    col * n:(col + 1) * n,
                ] = np.linalg.matrix_power(a, row - col) @ b
        stage_q = np.diag(np.r_[np.full(n, q_weight), np.full(n, v_weight)])
        q_bar = np.kron(np.eye(horizon), stage_q)
        r_bar = np.eye(horizon * n) * u_weight
        x0 = np.r_[q, qd]
        x_ref = np.tile(np.r_[q_ref, np.zeros(n)], horizon)
        hessian = su.T @ q_bar @ su + r_bar + np.eye(horizon * n) * 1e-9
        gradient = su.T @ q_bar @ (sx @ x0 - x_ref)
        controls = -np.linalg.solve(hessian, gradient)
        acceleration_limit = abs(float(self.get_parameter("maximum_acceleration_rad_s2").value))
        return np.clip(controls[:n], -acceleration_limit, acceleration_limit)

    def _update(self) -> None:
        if self.state is None or self.target is None:
            self._publish_invalid("state_or_target_missing")
            return
        timeout = float(self.get_parameter("state_timeout_sec").value)
        if time.monotonic() - self.state_received_monotonic > timeout:
            self._publish_invalid("state_stale")
            return

        state_position = self._map(self.state.name, self.state.position)
        state_velocity = self._map(self.state.name, self.state.velocity)
        target_position = self._map(self.target.name, self.target.position)
        names = [name for name in self.state.name if name in state_position and name in target_position]
        if not names:
            self._publish_invalid("no_common_joint_names")
            return

        started = time.perf_counter()
        try:
            q = np.asarray([state_position[name] for name in names])
            qd = np.asarray([state_velocity.get(name, 0.0) for name in names])
            q_ref = np.asarray([target_position[name] for name in names])
            acceleration = self._first_control(q, qd, q_ref)
            dt = float(self.get_parameter("model_dt_sec").value)
            velocity_limit = abs(float(self.get_parameter("maximum_velocity_rad_s").value))
            velocity = np.clip(qd + dt * acceleration, -velocity_limit, velocity_limit)
            solve_ms = (time.perf_counter() - started) * 1000.0

            reference = JointState()
            reference.header.stamp = self.get_clock().now().to_msg()
            reference.name = names
            reference.position = (q + dt * velocity).tolist()
            reference.velocity = velocity.tolist()
            reference.effort = []
            self.reference_publisher.publish(reference)

            status = MpcStatus()
            status.header.stamp = reference.header.stamp
            status.valid = True
            status.solver_status = "SHADOW_LINEAR_MPC"
            status.solve_time_ms = solve_ms
            status.prediction_error = float(np.sqrt(np.mean((q_ref - q) ** 2)))
            status.saturated = bool(
                np.any(np.abs(velocity) >= velocity_limit - 1e-9)
            )
            status.fallback_reason = (
                "shadow_mode_output_not_authorized_for_hardware"
                if bool(self.get_parameter("shadow_mode").value)
                else ""
            )
            self.status_publisher.publish(status)
        except (ValueError, np.linalg.LinAlgError) as exc:
            solve_ms = (time.perf_counter() - started) * 1000.0
            self._publish_invalid(f"solver_failure:{exc}", solve_ms)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MpcReferenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
