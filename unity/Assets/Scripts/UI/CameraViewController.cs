using Unity.Cinemachine;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

namespace V2X.UI
{
    /// <summary>Switches three Cinemachine shots from UI buttons or Input System keys 1/2/3.</summary>
    public class CameraViewController : MonoBehaviour
    {
        public CinemachineCamera[] cameras;
        public Button[] buttons;
        public int ActiveView { get; private set; }

        private InputActionMap _actions;

        private void Awake()
        {
            _actions = new InputActionMap("Camera Views");
            Bind("View 1", "<Keyboard>/digit1", 0);
            Bind("View 2", "<Keyboard>/digit2", 1);
            Bind("View 3", "<Keyboard>/digit3", 2);
            if (buttons != null)
                for (int i = 0; i < buttons.Length; i++)
                {
                    int view = i;
                    if (buttons[i] != null) buttons[i].onClick.AddListener(() => SelectView(view));
                }
            SelectView(0);
        }

        private void OnEnable() => _actions?.Enable();
        private void OnDisable() => _actions?.Disable();
        private void OnDestroy() => _actions?.Dispose();

        public void SelectView(int index)
        {
            if (cameras == null || index < 0 || index >= cameras.Length) return;
            ActiveView = index;
            for (int i = 0; i < cameras.Length; i++)
                if (cameras[i] != null) cameras[i].Priority = i == index ? 20 : 0;
            if (buttons != null)
                for (int i = 0; i < buttons.Length; i++)
                    if (buttons[i] != null) buttons[i].interactable = i != index;
            RefreshButtonVisuals(index);
        }

        private void RefreshButtonVisuals(int selected)
        {
            if (buttons == null) return;
            for (int i = 0; i < buttons.Length; i++)
            {
                var button = buttons[i];
                if (button == null) continue;
                bool active = i == selected;
                var colors = button.colors;
                colors.normalColor = Color.white;
                colors.highlightedColor = new Color(.961f, .961f, .969f, 1f);
                colors.pressedColor = new Color(.88f, .88f, .9f, 1f);
                colors.disabledColor = active
                    ? new Color(0f, .4f, .8f, 1f)
                    : new Color(.82f, .82f, .84f, .72f);
                button.colors = colors;
                var label = button.GetComponentInChildren<Text>();
                if (label != null)
                    label.color = active
                        ? Color.white
                        : new Color(.114f, .114f, .122f, 1f);
            }
        }

        private void Bind(string name, string binding, int view)
        {
            var action = _actions.AddAction(name, InputActionType.Button, binding);
            action.performed += _ => SelectView(view);
        }
    }
}
