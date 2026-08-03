using UnityEngine;
using UnityEngine.UI;
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

        public Vector3 straightGoal = new(5.4f, 0f, 66f);
        public Vector3 leftGoal = new(-66f, 0f, 5.4f);
        public Vector3 rightGoal = new(66f, 0f, -5.4f);

        private bool _binding;

        private void Awake()
        {
            if (ego == null) return;
            _binding = true;
            if (straightToggle != null) straightToggle.isOn = true;
            if (leftToggle != null) leftToggle.isOn = false;
            if (rightToggle != null) rightToggle.isOn = false;
            _binding = false;

            straightToggle?.onValueChanged.AddListener(on => { if (on) SelectStraight(); });
            leftToggle?.onValueChanged.AddListener(on => { if (on) SelectLeft(); });
            rightToggle?.onValueChanged.AddListener(on => { if (on) SelectRight(); });
            SelectStraight();
        }

        public void SelectStraight() => Apply("straight", straightGoal, "urban_nb_0_in", "직진");
        public void SelectLeft() => Apply("left", leftGoal, "urban_nb_1_in", "보호 좌회전");
        public void SelectRight() => Apply("right", rightGoal, "urban_nb_0_in", "우회전");

        private void Apply(string maneuver, Vector3 goal, string targetLane, string label)
        {
            if (_binding || ego == null) return;
            ego.maneuver = maneuver;
            if (ego.goal != null) ego.goal.position = goal;
            if (!string.IsNullOrEmpty(ego.CurrentLaneId) && ego.CurrentLaneId != targetLane)
                ego.RequestTargetLane(targetLane);
            else
                ego.ClearTargetLaneRequest();
            if (status != null)
                status.text = $"운전 전략: {label} · V2X 신호/교통 확인";
        }
    }
}
