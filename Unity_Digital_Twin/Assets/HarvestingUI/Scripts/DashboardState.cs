using System;
using System.Collections.Generic;
using UnityEngine;

namespace Harvesting.DigitalTwin.UI
{
    [Serializable]
    public sealed class DashboardState
    {
        public bool RosConnected;
        public bool SafetyOk;
        public bool EmergencyStopReady;
        public string RobotMode = "AUTO SUPERVISED";
        public string MissionState = "APPROACH";
        public string TargetId = "C-042";
        public float TargetConfidence = 0.91f;
        public float TargetUncertaintyMm = 12f;
        public float TargetDistanceM = 0.48f;
        public float HealthScore = 0.92f;
        public float NetworkLatencyMs = 13f;
        public float MpcSolveTimeMs = 4.2f;
        public float MpcPredictionError = 0.008f;
        public string MaintenanceRecommendation = "Inspect J2 vibration trend within 7 days.";
        public readonly List<float> TrackingHistory = new();
        public readonly List<float> LoadHistory = new();
        public readonly List<float> LatencyHistory = new();
    }
}
