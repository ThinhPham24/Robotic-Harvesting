using UnityEngine;

namespace Harvesting.DigitalTwin.UI
{
    /// <summary>
    /// Deterministic-looking demo telemetry for UI development without ROS.
    /// </summary>
    public sealed class DemoRobotStateSource : IRobotStateSource
    {
        private readonly DashboardState state = new();
        private float elapsed;
        private float sampleAccumulator;

        public DashboardState Current => state;

        public DemoRobotStateSource()
        {
            state.RosConnected = true;
            state.SafetyOk = true;
            state.EmergencyStopReady = true;
            for (var i = 0; i < 90; i++)
                AddSample(i * 0.05f);
        }

        public void Tick(float deltaTime)
        {
            elapsed += deltaTime;
            sampleAccumulator += deltaTime;
            state.NetworkLatencyMs = 13f + Mathf.Sin(elapsed * 0.65f) * 2.1f;
            state.MpcSolveTimeMs = 4.0f + Mathf.Sin(elapsed * 1.7f) * 0.35f;
            state.TargetConfidence = 0.91f + Mathf.Sin(elapsed * 0.3f) * 0.015f;

            if (sampleAccumulator >= 0.10f)
            {
                sampleAccumulator = 0f;
                AddSample(elapsed);
            }
        }

        private void AddSample(float t)
        {
            Push(state.TrackingHistory, 0.20f + Mathf.Sin(t * 2.2f) * 0.08f + Random.Range(-0.025f, 0.025f));
            Push(state.LoadHistory, 42f + Mathf.Sin(t * 0.8f) * 9f + Random.Range(-2f, 2f));
            Push(state.LatencyHistory, 13f + Mathf.Sin(t * 0.65f) * 2f + Random.Range(-0.7f, 0.7f));
        }

        private static void Push(System.Collections.Generic.List<float> values, float value)
        {
            values.Add(value);
            if (values.Count > 100)
                values.RemoveAt(0);
        }

        public void RequestMissionStart()
        {
            if (state.SafetyOk && state.EmergencyStopReady)
                state.MissionState = "APPROACH";
        }

        public void RequestHold()
        {
            state.MissionState = "HOLD";
        }

        public void Dispose()
        {
        }
    }
}
