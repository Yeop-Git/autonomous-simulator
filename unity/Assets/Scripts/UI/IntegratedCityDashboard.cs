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
        private void Start()
        {
            ego ??= GameObject.Find("urban_ego")?.GetComponent<VehicleController>();
            simulation ??= FindFirstObjectByType<SimulationManager>();
            director ??= FindFirstObjectByType<IntegratedCityScenarioDirector>();
            client ??= FindFirstObjectByType<V2XClient>();
        }

        private void OnGUI()
        {
            float width = AppleGui.BeginFrame();
            GUILayout.BeginArea(new Rect(18, 18, 470, 325), AppleGui.Panel);
            GUILayout.Label("통합 주행", AppleGui.Title);
            float remaining = director?.RemainingTime ?? 600f;
            GUILayout.Label($"남은 시간 {Mathf.FloorToInt(remaining / 60f):00}:{Mathf.FloorToInt(remaining % 60f):00}  ·  LAP {director?.Lap ?? 0}", AppleGui.State);
            GUILayout.Label($"단계  <color={AppleGui.BlueHex}>{director?.Phase ?? "초기화"}</color>", AppleGui.Body);
            GUILayout.Space(12);
            if (ego != null)
            {
                GUILayout.Label($"{ego.Behavior}  ·  {ego.CurrentLaneId}", AppleGui.State);
                GUILayout.Label($"{ego.Planner.ToUpperInvariant()}  ·  계획 {ego.PlanningTimeMs:0.0} ms  ·  여유 {ego.MinimumClearance:0.00} m", AppleGui.Body);
                GUILayout.Label($"{ego.CurrentSpeed * 3.6f:0.0} / {ego.TargetSpeed * 3.6f:0.0} km/h  ·  {ego.TurnSignal.ToUpperInvariant()}", AppleGui.Body);
            }
            string link = client != null && client.IsConnected
                ? $"<color={AppleGui.GoodHex}>온라인</color>"
                : $"<color={AppleGui.DangerHex}>오프라인</color>";
            GUILayout.Label($"중앙 V2X {link}  ·  tick {client?.LastAppliedTick ?? -1}", AppleGui.MutedBody);
            GUILayout.Space(10);
            GUILayout.Label("신호 · 보행자 · ACC · 충돌 예측\nA* 전역 경로 · RRT 국부 회피 · 긴급차 양보", AppleGui.MutedBody);
            GUILayout.EndArea();

            GUILayout.BeginArea(new Rect(width - 235, 82, 215, 112), AppleGui.SubtlePanel);
            GUILayout.Label("경로 표시", AppleGui.State);
            GUILayout.Label("<color=#33aa55>A* 전역</color>  <color=#008fb8>RRT 국부</color>\n<color=#b030b0>RRT* 비교</color>  <color=#a05a00>양보</color>", AppleGui.Body);
            GUILayout.EndArea();
        }
    }
}
