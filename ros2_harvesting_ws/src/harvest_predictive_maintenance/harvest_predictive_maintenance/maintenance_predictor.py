from __future__ import annotations

from collections import deque

import rclpy
from harvest_interfaces.msg import AssetHealth, MaintenancePrediction
from rclpy.node import Node


class MaintenancePredictor(Node):
    """Risk baseline that refuses to invent RUL without sufficient history."""

    def __init__(self) -> None:
        super().__init__("maintenance_predictor")
        self.declare_parameter("health_topic", "/harvest/health/state")
        self.declare_parameter("prediction_topic", "/harvest/maintenance/prediction")
        self.declare_parameter("minimum_samples", 1000)
        self.declare_parameter("publish_every_samples", 20)
        self.declare_parameter("model_version", "transparent_trend_baseline_v1")
        self.history = deque(maxlen=5000)
        self.publisher = self.create_publisher(
            MaintenancePrediction,
            str(self.get_parameter("prediction_topic").value),
            10,
        )
        self.create_subscription(
            AssetHealth,
            str(self.get_parameter("health_topic").value),
            self._on_health,
            10,
        )

    def _on_health(self, health: AssetHealth) -> None:
        self.history.append(float(health.anomaly_score))
        every = max(1, int(self.get_parameter("publish_every_samples").value))
        if len(self.history) % every:
            return

        minimum = max(2, int(self.get_parameter("minimum_samples").value))
        enough = len(self.history) >= minimum
        recent = list(self.history)[-min(100, len(self.history)) :]
        risk = max(0.0, min(1.0, sum(recent) / max(1, len(recent))))

        msg = MaintenancePrediction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.asset_id = health.asset_id
        msg.failure_risk = risk
        msg.estimated_rul_hours = -1.0
        msg.confidence = min(0.49, len(self.history) / float(minimum) * 0.49)
        msg.sufficient_history = enough
        msg.model_version = str(self.get_parameter("model_version").value)
        msg.recommended_action = (
            "Collect labeled maintenance and failure history; RUL is unavailable."
            if not enough
            else "Review anomaly trend and schedule condition-based inspection."
        )
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MaintenancePredictor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
