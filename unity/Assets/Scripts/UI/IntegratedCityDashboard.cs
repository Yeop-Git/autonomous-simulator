using UnityEngine;
using V2X.Communication;
using V2X.Sim;
using V2X.Vehicle;

namespace V2X.UI
{
    /// <summary>Compact operator HUD for the ten-minute integrated showcase.</summary>
    public class IntegratedCityDashboard : MonoBehaviour
    {
        public VehicleController ego;
        public SimulationManager simulation;
        public IntegratedCityScenarioDirector director;
        public V2XClient client;
        private GUIStyle _title;
        private GUIStyle _body;
        private GUIStyle _state;

        private void Start()
        {
            ego ??= GameObject.Find("urban_ego")?.GetComponent<VehicleController>();
            simulation ??= FindFirstObjectByType<SimulationManager>();
            director ??= FindFirstObjectByType<IntegratedCityScenarioDirector>();
            client ??= FindFirstObjectByType<V2XClient>();
        }

        private void OnGUI()
        {
            _title ??= new GUIStyle(GUI.skin.label)
            { fontSize = 21, fontStyle = FontStyle.Bold, normal = { textColor = Color.white } };
            _body ??= new GUIStyle(GUI.skin.label)
            { fontSize = 14, richText = true, normal = { textColor = new Color(.88f, .94f, 1f) } };
            _state ??= new GUIStyle(_body)
            { fontSize = 17, fontStyle = FontStyle.Bold };

            GUILayout.BeginArea(new Rect(18, 18, 470, 325), GUI.skin.window);
            GUILayout.Label("V2X INTEGRATED CITY - 10 MIN SHOWCASE", _title);
            float remaining = director?.RemainingTime ?? 600f;
            GUILayout.Label($"RUN {Mathf.FloorToInt(remaining / 60f):00}:{Mathf.FloorToInt(remaining % 60f):00} remaining   LAP {director?.Lap ?? 0}", _state);
            GUILayout.Label($"PHASE  <color=#69f7ff>{director?.Phase ?? "Initializing"}</color>", _body);
            GUILayout.Space(6);
            if (ego != null)
            {
                GUILayout.Label($"STATE  <b>{ego.Behavior}</b>   lane {ego.CurrentLaneId}", _state);
                GUILayout.Label($"Planner <b>{ego.Planner.ToUpperInvariant()}</b>   plan {ego.PlanningTimeMs:0.0} ms   clearance {ego.MinimumClearance:0.00} m", _body);
                GUILayout.Label($"Speed {ego.CurrentSpeed * 3.6f:0.0} km/h / target {ego.TargetSpeed * 3.6f:0.0} km/h   signal {ego.TurnSignal.ToUpperInvariant()}", _body);
            }
            string link = client != null && client.IsConnected ? "<color=#66ff99>ONLINE</color>" : "<color=#ff6655>OFFLINE</color>";
            GUILayout.Label($"Central V2X {link}   tick {client?.LastAppliedTick ?? -1}", _body);
            GUILayout.Space(7);
            GUILayout.Label("ACTIVE FEATURES", _state);
            GUILayout.Label("Signals + pedestrians + ACC + collision prediction\nA* routing + RRT/RRT* obstacle escape + emergency pull-over", _body);
            GUILayout.EndArea();

            GUILayout.BeginArea(new Rect(Screen.width - 235, 18, 215, 105), GUI.skin.box);
            GUILayout.Label("PATH TELEMETRY", _state);
            GUILayout.Label("<color=#33ff66>A* GLOBAL</color>  <color=#20f5ff>RRT LOCAL</color>\n<color=#ff33dd>RRT* OPTIMAL</color>  <color=#ffbb22>YIELD</color>", _body);
            GUILayout.EndArea();
        }
    }
}
