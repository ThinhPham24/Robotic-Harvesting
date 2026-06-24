from __future__ import annotations

import rclpy
from harvest_interfaces.msg import AssetHealth, TwinResidual
from rclpy.node import Node


class ConditionMonitor(Node):
    """EWMA residual monitor with explicit, auditable thresholds."""

    def __init__(self) -> None:
        super().__init__("condition_monitor")
        self.declare_parameter("residual_topic", "/harvest/twin/residual")
        self.declare_parameter("health_topic", "/harvest/health/state")
        self.declare_parameter("ewma_alpha", 0.10)
        self.declare_parameter("position_rmse_warning_rad", 0.02)
        self.declare_parameter("position_rmse_critical_rad", 0.05)
        self.declare_parameter("synchronization_warning_sec", 0.10)
        self.ewma_rmse = 0.0
        self.initialized = False
        self.publisher = self.create_publisher(
            AssetHealth, str(self.get_parameter("health_topic").value), 10
        )
        self.create_subscription(
            TwinResidual,
            str(self.get_parameter("residual_topic").value),
            self._on_residual,
            10,
        )

    def _on_residual(self, residual: TwinResidual) -> None:
        alpha = min(1.0, max(0.001, float(self.get_parameter("ewma_alpha").value)))
        self.ewma_rmse = (
            residual.position_rmse
            if not self.initialized
            else alpha * residual.position_rmse + (1.0 - alpha) * self.ewma_rmse
        )
        self.initialized = True
        warning = float(self.get_parameter("position_rmse_warning_rad").value)
        critical = float(self.get_parameter("position_rmse_critical_rad").value)
        sync_warning = float(self.get_parameter("synchronization_warning_sec").value)
        indicators = []
        state = AssetHealth.HEALTHY

        if not residual.synchronized or residual.synchronization_age_sec > sync_warning:
            state = max(state, AssetHealth.WARNING)
            indicators.append("digital_twin_not_synchronized")
        if self.ewma_rmse >= critical:
            state = AssetHealth.CRITICAL
            indicators.append("joint_tracking_residual_critical")
        elif self.ewma_rmse >= warning:
            state = max(state, AssetHealth.WARNING)
            indicators.append("joint_tracking_residual_high")

        normalized = min(1.0, self.ewma_rmse / max(critical, 1e-9))
        msg = AssetHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.asset_id = residual.asset_id
        msg.state = state
        msg.health_score = max(0.0, 1.0 - normalized)
        msg.anomaly_score = normalized
        msg.active_indicators = indicators
        msg.recommendation = (
            "Stop and inspect before automatic operation."
            if state == AssetHealth.CRITICAL
            else "Inspect synchronization and tracking trend."
            if state == AssetHealth.WARNING
            else "Continue monitoring."
        )
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConditionMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
