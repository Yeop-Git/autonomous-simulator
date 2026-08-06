using System;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

// The Main scene is a hub, not an experiment: it lists every demo scene, says
// what each one is for and which parts of the stack it exercises, and loads the
// selected one. Selecting and running are deliberately two steps so the
// description has somewhere to live and a stray click cannot leave the menu.

namespace V2X.UI
{
    [Serializable]
    public class HubSceneEntry
    {
        [Tooltip("Scene name as registered in Build Settings.")]
        public string sceneName;
        public string title;
        [TextArea(2, 4)] public string summary;
        [Tooltip("One technique per line — what this scene actually exercises.")]
        [TextArea(3, 8)] public string techniques;
        [Tooltip("Shown under the description; keys/buttons the scene responds to.")]
        [TextArea(2, 4)] public string controls;
    }

    public class SceneHubController : MonoBehaviour
    {
        public HubSceneEntry[] entries = Array.Empty<HubSceneEntry>();

        [Header("Wiring (parallel to entries)")]
        public Button[] sceneButtons = Array.Empty<Button>();
        public Button runButton;

        [Header("Detail panel")]
        public Text titleText;
        public Text summaryText;
        public Text techniquesText;
        public Text controlsText;
        public Text statusText;

        private int _selected = -1;

        private void Start()
        {
            for (int i = 0; i < sceneButtons.Length; i++)
            {
                if (sceneButtons[i] == null) continue;
                int index = i;                       // capture per iteration
                sceneButtons[i].onClick.AddListener(() => Select(index));
            }
            runButton?.onClick.AddListener(RunSelected);
            Select(entries.Length > 0 ? 0 : -1);
        }

        private void Update()
        {
            var keyboard = Keyboard.current;
            if (keyboard == null) return;

            // Number keys pick a scene, Enter runs it. Digit1..Digit9 are
            // consecutive in Key, so index straight off the device — building a
            // key array every frame would be pure garbage for a menu screen.
            for (int i = 0; i < entries.Length && i < 9; i++)
                if (keyboard[Key.Digit1 + i].wasPressedThisFrame) Select(i);

            if (keyboard.enterKey.wasPressedThisFrame ||
                keyboard.numpadEnterKey.wasPressedThisFrame)
                RunSelected();
        }

        public void Select(int index)
        {
            _selected = index;
            HubSceneEntry entry = Valid(index) ? entries[index] : null;

            for (int i = 0; i < sceneButtons.Length; i++)
            {
                if (sceneButtons[i] == null) continue;
                var image = sceneButtons[i].targetGraphic as Image;
                if (image != null)
                    image.color = i == index
                        ? new Color(.16f, .38f, .58f, .96f)
                        : new Color(.08f, .12f, .18f, .92f);
            }

            if (titleText != null)
                titleText.text = entry?.title ?? "";
            if (summaryText != null)
                summaryText.text = entry?.summary ?? "";
            if (techniquesText != null)
                techniquesText.text = entry?.techniques ?? "";
            if (controlsText != null)
                controlsText.text = entry?.controls ?? "";
            if (runButton != null)
                runButton.interactable = entry != null;

            SetStatus(entry);
        }

        public void RunSelected()
        {
            if (!Valid(_selected)) return;
            string sceneName = entries[_selected].sceneName;
            if (!IsLoadable(sceneName))
            {
                // Loading an unregistered scene throws at runtime and looks like
                // a frozen menu, so say what is actually wrong instead.
                if (statusText != null)
                    statusText.text =
                        $"'{sceneName}' 씬이 Build Settings에 없습니다 · " +
                        "File > Build Profiles에서 추가하거나 " +
                        "V2X > Build All Demo Scenes 실행";
                Debug.LogError($"[SceneHub] scene '{sceneName}' is not in Build Settings");
                return;
            }
            SceneManager.LoadScene(sceneName);
        }

        private void SetStatus(HubSceneEntry entry)
        {
            if (statusText == null) return;
            if (entry == null)
            {
                statusText.text = "실행할 씬을 선택하세요";
                return;
            }
            statusText.text = IsLoadable(entry.sceneName)
                ? "숫자키로 선택 · Enter 또는 실행 버튼으로 진입 " +
                  "(Python 서버가 켜져 있어야 주행합니다)"
                : $"'{entry.sceneName}' 씬이 Build Settings에 없습니다";
        }

        private bool Valid(int index) =>
            entries != null && index >= 0 && index < entries.Length &&
            entries[index] != null && !string.IsNullOrEmpty(entries[index].sceneName);

        private static bool IsLoadable(string sceneName) =>
            Application.CanStreamedLevelBeLoaded(sceneName);
    }
}
