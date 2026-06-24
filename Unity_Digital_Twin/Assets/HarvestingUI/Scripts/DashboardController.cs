using System;
using UnityEngine;
using UnityEngine.UIElements;

namespace Harvesting.DigitalTwin.UI
{
    public sealed class DashboardController : MonoBehaviour
    {
        private UIDocument document;
        private IRobotStateSource stateSource;
        private Label rosStatus;
        private Label safetyStatus;
        private Label estopStatus;
        private Label modeStatus;
        private Label clock;
        private Label targetTitle;
        private Label confidenceValue;
        private Label uncertaintyValue;
        private Label distanceValue;
        private Label missionState;
        private Label healthValue;
        private Label maintenanceText;
        private Label mpcValue;
        private Label latencyValue;
        private VisualElement healthRing;
        private SparklineElement trackingChart;
        private SparklineElement loadChart;
        private SparklineElement latencyChart;
        private bool initialized;

        public void Initialize(UIDocument uiDocument)
        {
            document = uiDocument;
        }

        private void Start()
        {
            if (document == null)
                document = GetComponent<UIDocument>();
            if (document == null)
                return;

            stateSource = new DemoRobotStateSource();
            Bind(document.rootVisualElement);
            initialized = true;
        }

        private void Bind(VisualElement root)
        {
            rosStatus = root.Q<Label>("ros-status");
            safetyStatus = root.Q<Label>("safety-status");
            estopStatus = root.Q<Label>("estop-status");
            modeStatus = root.Q<Label>("mode-status");
            clock = root.Q<Label>("clock");
            targetTitle = root.Q<Label>("target-title");
            confidenceValue = root.Q<Label>("confidence-value");
            uncertaintyValue = root.Q<Label>("uncertainty-value");
            distanceValue = root.Q<Label>("distance-value");
            missionState = root.Q<Label>("mission-state");
            healthValue = root.Q<Label>("health-value");
            maintenanceText = root.Q<Label>("maintenance-text");
            mpcValue = root.Q<Label>("mpc-value");
            latencyValue = root.Q<Label>("latency-value");
            healthRing = root.Q("health-ring");

            trackingChart = InstallChart(root.Q("tracking-chart"));
            loadChart = InstallChart(root.Q("load-chart"));
            latencyChart = InstallChart(root.Q("latency-chart"));

            root.Q<Button>("start-mission").clicked += stateSource.RequestMissionStart;
            root.Q<Button>("hold-mission").clicked += stateSource.RequestHold;
            root.Q<Button>("estop-button").clicked += () =>
                Debug.LogWarning("UI E-stop request selected. Connect only through the reviewed safety workflow.");

            foreach (var button in root.Query<Button>(className: "nav-button").ToList())
                button.clicked += () => SelectNavigation(root, button);
        }

        private static SparklineElement InstallChart(VisualElement host)
        {
            var chart = new SparklineElement();
            chart.AddToClassList("sparkline");
            host.Add(chart);
            return chart;
        }

        private static void SelectNavigation(VisualElement root, Button selected)
        {
            foreach (var button in root.Query<Button>(className: "nav-button").ToList())
                button.EnableInClassList("nav-button--active", button == selected);
        }

        private void Update()
        {
            if (!initialized)
                return;
            stateSource.Tick(Time.unscaledDeltaTime);
            Render(stateSource.Current);
        }

        private void Render(DashboardState state)
        {
            clock.text = DateTime.UtcNow.ToString("yyyy-MM-dd  HH:mm:ss 'UTC'");
            rosStatus.text = state.RosConnected ? "●  ROS 2 CONNECTED" : "●  ROS OFFLINE";
            rosStatus.EnableInClassList("status--ok", state.RosConnected);
            rosStatus.EnableInClassList("status--critical", !state.RosConnected);
            safetyStatus.text = state.SafetyOk ? "◆  SAFETY OK" : "◆  SAFETY BLOCKED";
            safetyStatus.EnableInClassList("status--ok", state.SafetyOk);
            safetyStatus.EnableInClassList("status--critical", !state.SafetyOk);
            estopStatus.text = state.EmergencyStopReady ? "E-STOP\nREADY" : "E-STOP\nACTIVE";
            estopStatus.EnableInClassList("estop-card--ready", state.EmergencyStopReady);
            estopStatus.EnableInClassList("estop-card--active", !state.EmergencyStopReady);
            modeStatus.text = state.RobotMode;
            targetTitle.text = $"TARGET CUCUMBER {state.TargetId}";
            confidenceValue.text = state.TargetConfidence.ToString("P0");
            uncertaintyValue.text = $"± {state.TargetUncertaintyMm:0} mm";
            distanceValue.text = $"{state.TargetDistanceM:0.00} m";
            missionState.text = state.MissionState;
            healthValue.text = state.HealthScore.ToString("P0");
            healthRing.style.unityBackgroundImageTintColor =
                state.HealthScore >= 0.85f ? new Color(0.13f, 0.77f, 0.37f) : new Color(0.96f, 0.62f, 0.04f);
            maintenanceText.text = state.MaintenanceRecommendation;
            mpcValue.text = $"{state.MpcSolveTimeMs:0.0} ms";
            latencyValue.text = $"{state.NetworkLatencyMs:0.0} ms";

            trackingChart.SetData(state.TrackingHistory, 0f, 0.6f, new Color(0.22f, 0.74f, 0.97f));
            loadChart.SetData(state.LoadHistory, 0f, 100f, new Color(0.20f, 0.83f, 0.60f));
            latencyChart.SetData(state.LatencyHistory, 0f, 35f, new Color(0.96f, 0.62f, 0.04f));
        }

        private void OnDestroy()
        {
            stateSource?.Dispose();
        }
    }
}
