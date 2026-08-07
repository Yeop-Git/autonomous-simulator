using UnityEngine;
using V2X.Communication;
using V2X.Vehicle;

// Minimal on-screen sync/telemetry HUD (plan §16.2 DebugUI). Shows the loop's
// health so timestep drift — the project's #1 risk — is visible at a glance:
// connection state, current vs last-applied tick, and command lag. Also lists
// each vehicle's behaviour, speed, and lateral error.

namespace V2X.UI
{
    public class DebugDashboard : MonoBehaviour
    {
        public V2XClient client;
        public VehicleController[] vehicles;
        public bool show = true;

        private void Start()
        {
            if (client == null) client = FindFirstObjectByType<V2XClient>();
            if (vehicles == null || vehicles.Length == 0)
                vehicles = FindObjectsByType<VehicleController>(FindObjectsSortMode.None);
        }

        private void OnGUI()
        {
            AppleGui.BeginFrame();
            if (!show || client == null) return;
            GUILayout.BeginArea(new Rect(18, 18, 460, 400), AppleGui.Panel);
            string conn = client.IsConnected
                ? $"<color={AppleGui.GoodHex}>연결됨</color>"
                : $"<color={AppleGui.DangerHex}>연결 안 됨</color>";
            GUILayout.Label("V2X 상태", AppleGui.Title);
            GUILayout.Label($"중앙 서버 {conn}", AppleGui.State);
            GUILayout.Label($"tick {client.CurrentTick}  applied {client.LastAppliedTick}  " +
                            $"lag {client.LastLagTicks}", AppleGui.MutedBody);
            GUILayout.Space(12);
            foreach (var v in vehicles)
            {
                if (v == null) continue;
                GUILayout.Label(
                    $"{v.Id}: <b>{v.Behavior}</b>  v={v.CurrentSpeed:0.0} m/s  " +
                    $"lane={v.CurrentLaneId}  latErr={v.LateralError:0.00}", AppleGui.Body);
            }
            GUILayout.EndArea();
        }
    }
}
