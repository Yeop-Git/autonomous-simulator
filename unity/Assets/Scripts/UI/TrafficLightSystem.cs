using System;
using UnityEngine;
using V2X.Vehicle;

namespace V2X.UI
{
    public enum SignalState { Green, Yellow, Red }

    [Serializable]
    public class TrafficSignalHead
    {
        public string label;
        public float offset;
        public Renderer red;
        public Renderer yellow;
        public Renderer[] left;
        public Renderer green;
    }

    /// <summary>Fixed-cycle signal visualization synchronized with simulation time.</summary>
    public class TrafficLightSystem : MonoBehaviour
    {
        private const float CycleTime = 60f;

        public TrafficSignalHead[] heads;
        public VehicleController focusVehicle;

        // The three phase windows below are the *same plan the Python server
        // enforces* (CentralController's TrafficLight table). Retiming one side
        // alone shows a driver a green the server is holding at red;
        // test_scene_networks.py::test_unity_signal_plan_matches_the_servers
        // fails if they drift apart.
        // The perpendicular east-west approaches receive the initial green.
        public SignalState EastWestState => WindowState(Time.fixedTime, 0f, 10f, 3f);
        public SignalState NorthSouthState => WindowState(Time.fixedTime, 21f, 10f, 3f);
        public SignalState ProtectedLeftState => WindowState(Time.fixedTime, 36f, 6f, 2f);
        public bool PedestriansMayCross
        {
            get
            {
                float t = Mathf.Repeat(Time.fixedTime, CycleTime);
                return (t >= 13f && t < 21f) || (t >= 47f && t < 55f);
            }
        }

        private void Update()
        {
            if (heads == null) return;
            foreach (var head in heads)
            {
                if (head == null) continue;
                SignalState state = head.label.StartsWith("LEFT")
                    ? ProtectedLeftState
                    : head.label.StartsWith("EW") ? EastWestState : NorthSouthState;
                SetBulb(head.red, Color.red, state == SignalState.Red);
                SetBulb(head.yellow, Color.yellow, state == SignalState.Yellow);
                bool leftActive = head.label.StartsWith("NS")
                    && ProtectedLeftState == SignalState.Green;
                SetBulbs(head.left, new Color(.15f, 1f, .45f), leftActive);
                SetBulb(head.green, Color.green, state == SignalState.Green);
            }
        }

        private static void SetBulbs(Renderer[] renderers, Color color, bool active)
        {
            if (renderers == null) return;
            foreach (var renderer in renderers) SetBulb(renderer, color, active);
        }

        private static SignalState WindowState(float time, float start, float green, float yellow)
        {
            float phase = Mathf.Repeat(time - start, CycleTime);
            if (phase < green) return SignalState.Green;
            if (phase < green + yellow) return SignalState.Yellow;
            return SignalState.Red;
        }

        private static void SetBulb(Renderer renderer, Color color, bool active)
        {
            if (renderer == null) return;
            renderer.material.color = active ? color : color * 0.12f;
            if (!renderer.material.HasProperty("_EmissionColor")) return;
            renderer.material.EnableKeyword("_EMISSION");
            renderer.material.SetColor("_EmissionColor", active ? color * 2f : Color.black);
        }

        private void OnGUI()
        {
            float width = AppleGui.BeginFrame();
            string ns = ColorText("남북", NorthSouthState);
            string ew = ColorText("동서", EastWestState);
            string left = ColorText("보호좌회전", ProtectedLeftState);
            var view = FindFirstObjectByType<CameraViewController>();
            if (view != null && view.ActiveView == 2)
            {
                string walk = PedestriansMayCross
                    ? $"<color={AppleGui.GoodHex}>보행 GREEN</color>"
                    : $"<color={AppleGui.DangerHex}>보행 RED</color>";
                GUI.Box(new Rect(width * .5f - 270f, 18f, 540f, 72f),
                    $"교차로 · {ns}   {left}   {ew}\n{walk}", AppleGui.StatusBox);
                return;
            }
            GUI.Box(new Rect(width * .5f - 125f, 18f, 250f, 52f),
                "전방 신호 · " + ColorText("", FacingState()), AppleGui.StatusBox);
        }

        private SignalState FacingState()
        {
            string lane = focusVehicle != null ? focusVehicle.CurrentLaneId ?? "" : "";
            if (focusVehicle != null && focusVehicle.maneuver == "left")
                return ProtectedLeftState;
            if (lane.Contains("_eb_") || lane.Contains("_wb_"))
                return EastWestState;
            return NorthSouthState;
        }

        private static string ColorText(string label, SignalState state)
        {
            string color = state == SignalState.Green ? AppleGui.GoodHex :
                state == SignalState.Yellow ? "#a05a00" : AppleGui.DangerHex;
            return $"<color={color}>{label} {state}</color>";
        }
    }
}
