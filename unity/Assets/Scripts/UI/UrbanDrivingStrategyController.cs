using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using V2X.Road;
using V2X.Vehicle;

namespace V2X.UI
{
    /// <summary>
    /// Publishes the selected manoeuvre through the ego vehicle's V2X state.
    /// The server remains responsible for route, signal, gap, and commands.
    /// </summary>
    public class UrbanDrivingStrategyController : MonoBehaviour
    {
        public VehicleController ego;
        public Toggle straightToggle;
        public Toggle leftToggle;
        public Toggle rightToggle;
        public Text status;
        public RoadNetworkManager road;

        public Vector3 straightGoal = new(5.4f, 0f, 66f);
        public Vector3 leftGoal = new(-66f, 0f, 5.4f);
        public Vector3 rightGoal = new(66f, 0f, -5.4f);

        private bool _binding;
        private string _targetLane;

        private void Awake()
        {
            if (ego == null) return;
            if (road == null) road = FindFirstObjectByType<RoadNetworkManager>();
            // No selection means ordinary straight driving in the current
            // lane. Buttons may override this intent at any later tick.
            ego.SetManeuverSelectionPending(false);
            if (status != null) status.text = "주행 방향을 선택하세요";
        }

        private void Start()
        {
            if (ego == null) return;
            StartCoroutine(ArmSelectionAfterToggleGroupSettles());
        }

        private IEnumerator ArmSelectionAfterToggleGroupSettles()
        {
            yield return null;
            _binding = true;
            if (straightToggle != null) straightToggle.SetIsOnWithoutNotify(false);
            if (leftToggle != null) leftToggle.SetIsOnWithoutNotify(false);
            if (rightToggle != null) rightToggle.SetIsOnWithoutNotify(false);
            _binding = false;
            straightToggle?.onValueChanged.AddListener(on => { if (on) SelectStraight(); });
            leftToggle?.onValueChanged.AddListener(on => { if (on) SelectLeft(); });
            rightToggle?.onValueChanged.AddListener(on => { if (on) SelectRight(); });
            BeginDefaultCruise();
        }

        public void SelectStraight() => Apply("straight", straightGoal, "urban_nb_0_in", "직진");
        public void SelectLeft() => Apply("left", leftGoal, "urban_nb_1_in", "보호 좌회전");
        public void SelectRight() => Apply("right", rightGoal, "urban_nb_0_in", "우회전");

        private void Update()
        {
            if (ego != null && ego.LeftTurnPhase == "AbortedStraight" &&
                ego.maneuver == "left")
            {
                SelectStraight();
                if (straightToggle != null) straightToggle.isOn = true;
                if (status != null)
                    status.text = "좌회전 진입 공간 부족 · 안전하게 직진 전환";
                return;
            }

            if (ego == null || string.IsNullOrEmpty(_targetLane) ||
                string.IsNullOrEmpty(ego.CurrentLaneId)) return;

            if (ego.CurrentLaneId == _targetLane)
            {
                ego.ClearTargetLaneRequest();
                return;
            }

            UpdateTargetLaneRequest();
        }

        private void BeginDefaultCruise()
        {
            if (ego == null) return;
            ego.maneuver = "straight";
            ego.SetManeuverSelectionPending(false);
            ego.ConfigureThroughRoute(straightGoal);
            _targetLane = ResolveTargetLane("straight", ego.CurrentLaneId);
            ego.ClearTargetLaneRequest();
            Debug.Log($"[DrivingStrategy] default=straight ego={ego.Id} " +
                      $"lane={ego.CurrentLaneId}");
        }

        private string ResolveTargetLane(string maneuver, string fallback)
        {
            if (road == null) road = FindFirstObjectByType<RoadNetworkManager>();
            var current = road?.GetLane(ego.CurrentLaneId) ??
                          road?.NearestLane(ego.transform.position);
            if (current == null) return fallback;
            if (maneuver == "left" && current.leftLane != null)
                return current.leftLane.Id;
            if (maneuver == "right" && current.rightLane != null)
                return current.rightLane.Id;
            return current.Id;
        }

        private void UpdateTargetLaneRequest()
        {
            if (ego == null || string.IsNullOrEmpty(_targetLane)) return;
            if (ego.CurrentLaneId == _targetLane)
            {
                ego.ClearTargetLaneRequest();
                return;
            }

            var current = road?.GetLane(ego.CurrentLaneId);
            bool adjacent = current != null &&
                ((current.leftLane != null && current.leftLane.Id == _targetLane) ||
                 (current.rightLane != null && current.rightLane.Id == _targetLane));
            if (adjacent) ego.RequestTargetLane(_targetLane);
            else ego.ClearTargetLaneRequest();
        }

        private void Apply(string maneuver, Vector3 goal, string targetLane, string label)
        {
            if (_binding || ego == null) return;
            ego.maneuver = maneuver;
            ego.SetManeuverSelectionPending(false);
            ego.ConfigureThroughRoute(goal);
            _targetLane = ResolveTargetLane(maneuver, targetLane);
            UpdateTargetLaneRequest();
            Debug.Log($"[LeftTurn] selection={maneuver} ego={ego.Id} " +
                      $"pos={ego.transform.position} lane={ego.CurrentLaneId} " +
                      $"request={ego.RequestedTargetLane}");
            if (status != null)
                status.text = $"운전 전략: {label} · V2X 신호/교통 확인";
        }
    }
}
