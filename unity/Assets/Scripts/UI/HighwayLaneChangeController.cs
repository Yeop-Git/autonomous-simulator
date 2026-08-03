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
        public Button automaticButton;
        public float automaticInterval = 12f;

        public float SecondsUntilAutomatic => Mathf.Max(0f, _nextAutomatic - Time.time);
        public string LastRequest { get; private set; } = "대기";
        public bool AutomaticEnabled { get; private set; } = true;

        private InputActionMap _actions;
        private float _nextAutomatic;
        private bool _autoPrefersLeft = true;

        private void Awake()
        {
            if (road == null) road = FindFirstObjectByType<RoadNetworkManager>();
            leftButton?.onClick.AddListener(RequestLeft);
            rightButton?.onClick.AddListener(RequestRight);
            automaticButton?.onClick.AddListener(ToggleAutomatic);
            _actions = new InputActionMap("Lane Change");
            var left = _actions.AddAction("Lane Left", InputActionType.Button, "<Keyboard>/q");
            var right = _actions.AddAction("Lane Right", InputActionType.Button, "<Keyboard>/e");
            var automatic = _actions.AddAction("Toggle Automatic", InputActionType.Button, "<Keyboard>/r");
            left.performed += _ => RequestLeft();
            right.performed += _ => RequestRight();
            automatic.performed += _ => ToggleAutomatic();
            _nextAutomatic = Time.time + automaticInterval;
            UpdateAutomaticButton();
        }

        private void OnEnable() => _actions?.Enable();
        private void OnDisable() => _actions?.Disable();
        private void OnDestroy() => _actions?.Dispose();

        private void Update()
        {
            if (!AutomaticEnabled || Time.time < _nextAutomatic) return;
            bool sent = _autoPrefersLeft ? Request(true, true) : Request(false, true);
            if (!sent) sent = _autoPrefersLeft ? Request(false, true) : Request(true, true);
            _autoPrefersLeft = !_autoPrefersLeft;
            _nextAutomatic = Time.time + automaticInterval;
        }

        public void RequestLeft()
        {
            DisableAutomaticForManual();
            Request(true, false);
        }

        public void RequestRight()
        {
            DisableAutomaticForManual();
            Request(false, false);
        }

        public void ToggleAutomatic()
        {
            AutomaticEnabled = !AutomaticEnabled;
            if (AutomaticEnabled)
            {
                _nextAutomatic = Time.time + automaticInterval;
                LastRequest = "자동 차선변경 켜짐";
            }
            else LastRequest = "자동 차선변경 꺼짐";
            UpdateAutomaticButton();
        }

        private void DisableAutomaticForManual()
        {
            AutomaticEnabled = false;
            UpdateAutomaticButton();
        }

        private void UpdateAutomaticButton()
        {
            if (automaticButton == null) return;
            var label = automaticButton.GetComponentInChildren<Text>();
            if (label != null) label.text = AutomaticEnabled ? "R 자동 ON" : "R 자동 OFF";
        }

        private bool Request(bool left, bool automatic)
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
            LastRequest = $"{(automatic ? "자동" : "수동")} → {target.Id}";
            return true;
        }

        private void OnGUI()
        {
            var style = new GUIStyle(GUI.skin.box) { fontSize = 15, alignment = TextAnchor.MiddleCenter };
            string automatic = AutomaticEnabled
                ? $"자동 변경까지 {SecondsUntilAutomatic:0.0}s"
                : "자동 변경 OFF (R/UI로 재개)";
            GUI.Box(new Rect(Screen.width - 350, 10, 340, 58),
                $"V2X 차선변경: {LastRequest}\n{automatic}", style);
        }
    }
}
