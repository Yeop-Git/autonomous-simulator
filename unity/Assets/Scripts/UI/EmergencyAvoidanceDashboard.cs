using UnityEngine;
using V2X.Communication;
using V2X.Sim;
using V2X.Vehicle;

namespace V2X.UI
{
    /// <summary>Presentation HUD for the RRT/RRT* emergency-avoidance lab.</summary>
    public class EmergencyAvoidanceDashboard : MonoBehaviour
    {
        public VehicleController ego;
        public SimulationManager simulation;
        public EmergencyAvoidanceScenarioController scenario;
        public V2XClient client;
        private GUIStyle _title;
        private GUIStyle _label;
        private GUIStyle _state;

        private void Start()
        {
            ego ??= FindFirstObjectByType<VehicleController>();
            simulation ??= FindFirstObjectByType<SimulationManager>();
            scenario ??= FindFirstObjectByType<EmergencyAvoidanceScenarioController>();
            client ??= FindFirstObjectByType<V2XClient>();
        }

        private void OnGUI()
        {
            _title ??= new GUIStyle(GUI.skin.label)
            { fontSize = 22, fontStyle = FontStyle.Bold, normal = { textColor = Color.white } };
            _label ??= new GUIStyle(GUI.skin.label)
            { fontSize = 14, richText = true, normal = { textColor = new Color(.88f, .93f, 1f) } };
            _state ??= new GUIStyle(_label)
            { fontSize = 17, fontStyle = FontStyle.Bold };

            GUILayout.BeginArea(new Rect(18, 18, 430, 310), GUI.skin.window);
            GUILayout.Label("V2X EMERGENCY AVOIDANCE LAB", _title);
            GUILayout.Label("A* GLOBAL  ·  RRT LOCAL  ·  ACTIVE PULL-OVER", _label);
            GUILayout.Space(8);
            if (ego != null)
            {
                string stateColor = ego.Behavior == "EmergencyBraking" ||
                                    ego.Behavior == "ControlledStopping" ? "#ff5544" : "#69f7ff";
                GUILayout.Label($"STATE  <color={stateColor}>{ego.Behavior}</color>", _state);
                GUILayout.Label($"Planner <b>{ego.Planner.ToUpperInvariant()}</b>   " +
                                $"plan {ego.PlanningTimeMs:0.0} ms   " +
                                $"clearance {ego.MinimumClearance:0.00} m", _label);
                GUILayout.Label($"Speed {ego.CurrentSpeed * 3.6f:0.0} km/h   " +
                                $"target {ego.TargetSpeed * 3.6f:0.0} km/h   lane {ego.CurrentLaneId}", _label);
                GUILayout.Label($"Lateral error {ego.LateralError:0.00} m   " +
                                $"signal {ego.TurnSignal.ToUpperInvariant()}", _label);
            }
            string connection = client != null && client.IsConnected ? "<color=#66ff99>ONLINE</color>" : "<color=#ff6655>OFFLINE</color>";
            GUILayout.Label($"V2X {connection}   tick {client?.LastAppliedTick ?? -1}", _label);
            GUILayout.Space(6);
            GUILayout.Label($"EVENT  {scenario?.LastEvent ?? "대기 중"}", _label);
            GUILayout.Label("[4] 낙하물   [5] 긴급차   [6] RRT↔RRT*   [0] 재시작", _label);
            GUILayout.EndArea();

            GUILayout.BeginArea(new Rect(Screen.width - 220, 18, 200, 92), GUI.skin.box);
            GUILayout.Label("PATH LEGEND", _state);
            GUILayout.Label("<color=#33ff66>━ A* GLOBAL</color>   <color=#20f5ff>━ RRT</color>", _label);
            GUILayout.Label("<color=#ff33dd>━ RRT*</color>   <color=#ffbb22>━ YIELD</color>", _label);
            GUILayout.EndArea();
        }
    }
}
