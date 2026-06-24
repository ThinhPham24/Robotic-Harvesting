using System;

namespace Harvesting.DigitalTwin.UI
{
    public interface IRobotStateSource : IDisposable
    {
        DashboardState Current { get; }
        void Tick(float deltaTime);
        void RequestMissionStart();
        void RequestHold();
    }
}
