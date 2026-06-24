from __future__ import annotations

import math
import threading
import time
from typing import Dict, List

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .serial_protocol import (
    DensoProtocolError,
    DensoRc7mSerialClient,
    DensoSerialError,
    DensoSerialTimeout,
)


class SerialGateway(Node):
    """ROS adapter for the custom RC7M serial PACScript protocol.

    This is not a real-time ros2_control SystemInterface. The controller accepts
    blocking point commands, so cancellation is possible only between points.
    """

    def __init__(self) -> None:
        super().__init__("serial_gateway")
        defaults = {
            "port": "/dev/ttyUSB0",
            "baud_rate": 19200,
            "message_timeout_sec": 0.5,
            "position_timeout_sec": 1500.0,
            "poll_rate_hz": 5.0,
            "joint_state_topic": "/joint_states",
            "action_name": "/denso_joint_trajectory_controller/follow_joint_trajectory",
            "joint_names": [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            ],
            "execution_enabled": False,
            "execution_speed_percent": 5,
            "maximum_trajectory_points": 25,
            "maximum_joint_step_rad": 0.20,
            "goal_tolerance_rad": 0.05,
            "figure": 5,
            "gripper_closed": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.joint_names = [
            str(value) for value in self.get_parameter("joint_names").value
        ]
        if len(self.joint_names) != 6 or len(set(self.joint_names)) != 6:
            raise ValueError("joint_names must contain six unique names.")

        self.client = DensoRc7mSerialClient(
            port=str(self.get_parameter("port").value),
            baud_rate=int(self.get_parameter("baud_rate").value),
            message_timeout_sec=float(
                self.get_parameter("message_timeout_sec").value
            ),
            position_timeout_sec=float(
                self.get_parameter("position_timeout_sec").value
            ),
        )
        self.client.connect()
        self.latest_joint_radians: List[float] | None = None
        self.latest_state_time = 0.0
        self.motion_active = threading.Event()
        self.callback_group = ReentrantCallbackGroup()

        self.joint_publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            10,
        )
        rate = max(0.2, float(self.get_parameter("poll_rate_hz").value))
        self.create_timer(
            1.0 / rate, self._poll_joint_state, callback_group=self.callback_group
        )
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().warning(
            "RC7M serial gateway started. execution_enabled="
            f"{self.get_parameter('execution_enabled').value}. "
            "This is a blocking point-command adapter, not a real-time trajectory controller."
        )

    def _poll_joint_state(self) -> None:
        if self.motion_active.is_set():
            return
        try:
            degrees = self.client.get_joint_degrees()
        except DensoSerialError as exc:
            self.get_logger().error(f"Joint read failed: {exc}")
            return
        self._publish_joint_state([math.radians(value) for value in degrees])

    def _publish_joint_state(self, radians: List[float]) -> None:
        self.latest_joint_radians = list(radians)
        self.latest_state_time = time.monotonic()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(radians)
        self.joint_publisher.publish(msg)

    def _ordered_positions(self, trajectory_names, positions) -> List[float]:
        mapping: Dict[str, float] = dict(zip(trajectory_names, positions))
        return [float(mapping[name]) for name in self.joint_names]

    def _validate_goal(self, goal_request) -> str | None:
        if not bool(self.get_parameter("execution_enabled").value):
            return "Hardware trajectory execution is disabled by parameter."
        trajectory = goal_request.trajectory
        if set(trajectory.joint_names) != set(self.joint_names):
            return "Trajectory joint names do not match configured robot joints."
        if not trajectory.points:
            return "Trajectory contains no points."
        maximum_points = int(
            self.get_parameter("maximum_trajectory_points").value
        )
        if len(trajectory.points) > maximum_points:
            return (
                f"Trajectory has {len(trajectory.points)} points; "
                f"maximum is {maximum_points}."
            )

        maximum_step = float(self.get_parameter("maximum_joint_step_rad").value)
        previous = self.latest_joint_radians
        for index, point in enumerate(trajectory.points):
            if len(point.positions) != len(trajectory.joint_names):
                return f"Trajectory point {index} has invalid position count."
            ordered = self._ordered_positions(
                trajectory.joint_names, point.positions
            )
            if not all(math.isfinite(value) for value in ordered):
                return f"Trajectory point {index} contains non-finite positions."
            if previous is not None:
                largest = max(
                    abs(current - prior)
                    for current, prior in zip(ordered, previous)
                )
                if largest > maximum_step:
                    return (
                        f"Trajectory point {index} step {largest:.3f} rad exceeds "
                        f"{maximum_step:.3f} rad."
                    )
            previous = ordered
        return None

    def _goal_callback(self, goal_request) -> GoalResponse:
        reason = self._validate_goal(goal_request)
        if reason:
            self.get_logger().error(f"Rejected MoveIt trajectory: {reason}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().warning(
            "Cancel requested. The custom serial protocol has no implemented "
            "mid-command stop; cancellation is honored after the current point finishes."
        )
        return CancelResponse.ACCEPT

    def _result(self, code: int, text: str):
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = text
        return result

    def _execute(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        speed = int(self.get_parameter("execution_speed_percent").value)
        figure = int(self.get_parameter("figure").value)
        gripper = bool(self.get_parameter("gripper_closed").value)
        self.motion_active.set()
        try:
            for index, point in enumerate(trajectory.points):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        "Canceled between serial point commands.",
                    )
                ordered_radians = self._ordered_positions(
                    trajectory.joint_names, point.positions
                )
                degrees = [math.degrees(value) for value in ordered_radians]
                self.client.move_joint_degrees(
                    degrees,
                    speed=speed,
                    gripper_closed=gripper,
                    figure=figure,
                )
                self._publish_joint_state(ordered_radians)
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = self.joint_names
                feedback.actual.positions = ordered_radians
                feedback.desired.positions = ordered_radians
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(
                    f"Completed serial trajectory point {index + 1}/"
                    f"{len(trajectory.points)}."
                )

            measured = [
                math.radians(value) for value in self.client.get_joint_degrees()
            ]
            self._publish_joint_state(measured)
            final_target = self._ordered_positions(
                trajectory.joint_names, trajectory.points[-1].positions
            )
            error = max(
                abs(actual - target)
                for actual, target in zip(measured, final_target)
            )
            tolerance = float(self.get_parameter("goal_tolerance_rad").value)
            if error > tolerance:
                goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                    f"Final joint error {error:.4f} rad exceeds {tolerance:.4f} rad.",
                )
            goal_handle.succeed()
            return self._result(
                FollowJointTrajectory.Result.SUCCESSFUL,
                "Serial point sequence completed.",
            )
        except (DensoSerialError, DensoProtocolError, DensoSerialTimeout, ValueError) as exc:
            self.get_logger().error(f"Serial trajectory failed: {exc}")
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                str(exc),
            )
        finally:
            self.motion_active.clear()

    def destroy_node(self):
        self.action_server.destroy()
        self.client.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialGateway()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
