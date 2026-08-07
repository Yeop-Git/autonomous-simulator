using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using V2X.Road;
using V2X.Vehicle;

namespace V2X.UI
{
    /// <summary>
    /// Sends a desired adjacent lane through the vehicle state.  The Python
    /// central controller performs V2X gap acceptance before commanding it.
    /// </summary>
    public class HighwayLaneChangeController : MonoBehaviour
    {
        public VehicleController ego;
        public RoadNetworkManager road;
        public Button leftButton;
        public Button rightButton;

        public string LastRequest { get; private set; } = "대기";

        private InputActionMap _actions;

        private void Awake()
        {
            if (road == null) road = FindFirstObjectByType<RoadNetworkManager>();
            leftButton?.onClick.AddListener(RequestLeft);
            rightButton?.onClick.AddListener(RequestRight);
            _actions = new InputActionMap("Lane Change");
            var left = _actions.AddAction("Lane Left", InputActionType.Button, "<Keyboard>/q");
            var right = _actions.AddAction("Lane Right", InputActionType.Button, "<Keyboard>/e");
            left.performed += _ => RequestLeft();
            right.performed += _ => RequestRight();
        }

        private void OnEnable() => _actions?.Enable();
        private void OnDisable() => _actions?.Disable();
        private void OnDestroy() => _actions?.Dispose();

        public void RequestLeft() => Request(true);

        public void RequestRight() => Request(false);

        private bool Request(bool left)
        {
            if (ego == null || road == null) return false;
            var lane = road.GetLane(ego.CurrentLaneId) ?? road.NearestLane(ego.transform.position);
            var target = left ? lane?.leftLane : lane?.rightLane;
            if (target == null)
            {
                LastRequest = left ? "왼쪽 차선 없음" : "오른쪽 차선 없음";
                return false;
            }
            ego.RequestTargetLane(target.Id);
            LastRequest = $"요청 → {target.Id}";
            return true;
        }

        private void OnGUI()
        {
            var style = new GUIStyle(GUI.skin.box) { fontSize = 15, alignment = TextAnchor.MiddleCenter };
            GUI.Box(new Rect(Screen.width - 350, 10, 340, 40),
                $"V2X 차선변경: {LastRequest}", style);
        }
    }
}
