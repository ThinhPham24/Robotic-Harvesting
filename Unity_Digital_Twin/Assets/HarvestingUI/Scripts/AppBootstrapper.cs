using UnityEngine;
using UnityEngine.UIElements;

namespace Harvesting.DigitalTwin.UI
{
    /// <summary>
    /// Runtime entry point. This keeps the dashboard usable in any Unity scene.
    /// </summary>
    public sealed class AppBootstrapper : MonoBehaviour
    {
        private const string RootObjectName = "Harvesting Digital Twin UI";

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void EnsureDashboardExists()
        {
            if (FindObjectOfType<AppBootstrapper>() != null)
                return;

            var root = new GameObject(RootObjectName);
            DontDestroyOnLoad(root);
            root.AddComponent<AppBootstrapper>();
        }

        private void Awake()
        {
            var document = gameObject.GetComponent<UIDocument>();
            if (document == null)
                document = gameObject.AddComponent<UIDocument>();

            var panelSettings = ScriptableObject.CreateInstance<PanelSettings>();
            panelSettings.name = "Runtime Harvesting Panel";
            panelSettings.scaleMode = PanelScaleMode.ScaleWithScreenSize;
            panelSettings.referenceResolution = new Vector2Int(1920, 1080);
            panelSettings.screenMatchMode = PanelScreenMatchMode.MatchWidthOrHeight;
            panelSettings.match = 0.5f;
            panelSettings.sortingOrder = 100;

            var layout = Resources.Load<VisualTreeAsset>("UI/HarvestDashboard");
            if (layout == null)
            {
                Debug.LogError("HarvestDashboard.uxml was not found in Resources/UI.");
                enabled = false;
                return;
            }

            document.panelSettings = panelSettings;
            document.visualTreeAsset = layout;

            var controller = gameObject.GetComponent<DashboardController>();
            if (controller == null)
                controller = gameObject.AddComponent<DashboardController>();
            controller.Initialize(document);
        }
    }
}
