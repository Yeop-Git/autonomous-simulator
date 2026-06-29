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

        private GUIStyle _style;

        private void Start()
        {
            if (client == null) client = FindFirstObjectByType<V2XClient>();
            if (vehicles == null || vehicles.Length == 0)
                vehicles = FindObjectsByType<VehicleController>(FindObjectsSortMode.None);
        }

        private void OnGUI()
        {
            if (!show || client == null) return;
            _style ??= new GUIStyle(GUI.skin.label) { fontSize = 13, richText = true };

            GUILayout.BeginArea(new Rect(10, 10, 460, 400), GUI.skin.box);
            string conn = client.IsConnected ? "<color=#6f6>connected</color>"
                                             : "<color=#f66>disconnected</color>";
            GUILayout.Label($"V2X: {conn}", _style);
            GUILayout.Label($"tick {client.CurrentTick}  applied {client.LastAppliedTick}  " +
                            $"lag {client.LastLagTicks}", _style);
            GUILayout.Space(6);
            foreach (var v in vehicles)
            {
                if (v == null) continue;
                GUILayout.Label(
                    $"{v.Id}: <b>{v.Behavior}</b>  v={v.CurrentSpeed:0.0} m/s  " +
                    $"lane={v.CurrentLaneId}  latErr={v.LateralError:0.00}", _style);
            }
            GUILayout.EndArea();
        }
    }
}
