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
        private void Start()
        {
            ego ??= FindFirstObjectByType<VehicleController>();
            simulation ??= FindFirstObjectByType<SimulationManager>();
            scenario ??= FindFirstObjectByType<EmergencyAvoidanceScenarioController>();
            client ??= FindFirstObjectByType<V2XClient>();
        }

        private void OnGUI()
        {
            float width = AppleGui.BeginFrame();
            GUILayout.BeginArea(new Rect(18, 18, 430, 310), AppleGui.Panel);
            GUILayout.Label("긴급 회피 실험", AppleGui.Title);
            GUILayout.Label("A* 전역 경로 · RRT 국부 회피 · 긴급차 양보", AppleGui.MutedBody);
            GUILayout.Space(12);
            if (ego != null)
            {
                string stateColor = ego.Behavior == "EmergencyBraking" ||
                                    ego.Behavior == "ControlledStopping"
                    ? AppleGui.DangerHex : AppleGui.BlueHex;
                GUILayout.Label($"<color={stateColor}>{ego.Behavior}</color>", AppleGui.State);
                GUILayout.Label($"{ego.Planner.ToUpperInvariant()}  ·  계획 {ego.PlanningTimeMs:0.0} ms  ·  여유 {ego.MinimumClearance:0.00} m", AppleGui.Body);
                GUILayout.Label($"{ego.CurrentSpeed * 3.6f:0.0} / {ego.TargetSpeed * 3.6f:0.0} km/h  ·  {ego.CurrentLaneId}", AppleGui.Body);
                GUILayout.Label($"횡오차 {ego.LateralError:0.00} m  ·  {ego.TurnSignal.ToUpperInvariant()}", AppleGui.Body);
            }
            string connection = client != null && client.IsConnected
                ? $"<color={AppleGui.GoodHex}>온라인</color>"
                : $"<color={AppleGui.DangerHex}>오프라인</color>";
            GUILayout.Label($"V2X {connection}  ·  tick {client?.LastAppliedTick ?? -1}", AppleGui.MutedBody);
            GUILayout.Space(8);
            GUILayout.Label($"이벤트 · {scenario?.LastEvent ?? "대기 중"}", AppleGui.State);
            GUILayout.Label("4 낙하물   5 긴급차   6 플래너   0 재시작", AppleGui.MutedBody);
            GUILayout.EndArea();

            GUILayout.BeginArea(new Rect(width - 220, 82, 200, 104), AppleGui.SubtlePanel);
            GUILayout.Label("경로 표시", AppleGui.State);
            GUILayout.Label("<color=#33aa55>━ A* 전역</color>  <color=#008fb8>━ RRT</color>", AppleGui.Body);
            GUILayout.Label("<color=#b030b0>━ RRT*</color>  <color=#a05a00>━ 양보</color>", AppleGui.Body);
            GUILayout.EndArea();
        }
    }
}
