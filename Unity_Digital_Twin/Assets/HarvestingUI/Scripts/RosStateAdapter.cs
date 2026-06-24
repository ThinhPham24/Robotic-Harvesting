namespace Harvesting.DigitalTwin.UI
{
    /// <summary>
    /// ROS integration boundary.
    ///
    /// Add Unity ROS-TCP-Connector to the project, generate the ROS message
    /// classes, and implement subscriptions here. Keep generated ROS classes
    /// and transport details out of DashboardController.
    /// </summary>
    public sealed class RosStateAdapter : IRobotStateSource
    {
        private readonly DashboardState state = new();

        public DashboardState Current => state;

        public RosStateAdapter()
        {
            state.RosConnected = false;
            state.SafetyOk = false;
            state.EmergencyStopReady = false;
            state.RobotMode = "ROS OFFLINE";
            state.MissionState = "DISABLED";
            state.MaintenanceRecommendation = "Waiting for ROS telemetry.";
        }

        public void Tick(float deltaTime)
        {
            // Process normalized state buffered by ROS callbacks.
        }

        public void RequestMissionStart()
        {
            // Do not directly publish a robot command here. Send a reviewed,
            // safety-gated mission request through the ROS task manager.
        }

        public void RequestHold()
        {
            // Publish only through the reviewed hold/cancel workflow.
        }

        public void Dispose()
        {
            // Unregister ROS subscriptions.
        }
    }
}
