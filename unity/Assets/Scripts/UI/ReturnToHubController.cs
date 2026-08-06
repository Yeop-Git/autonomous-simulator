using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

// Every demo scene is entered from the Main hub, so every demo scene needs a way
// back. Esc or the button returns; nothing else in the scene owns that key.

namespace V2X.UI
{
    public class ReturnToHubController : MonoBehaviour
    {
        [Tooltip("Hub scene name as registered in Build Settings.")]
        public string hubSceneName = "Main";
        public Button returnButton;

        private void Start() => returnButton?.onClick.AddListener(ReturnToHub);

        private void Update()
        {
            var keyboard = Keyboard.current;
            if (keyboard != null && keyboard.escapeKey.wasPressedThisFrame)
                ReturnToHub();
        }

        public void ReturnToHub()
        {
            if (!Application.CanStreamedLevelBeLoaded(hubSceneName))
            {
                Debug.LogError(
                    $"[ReturnToHub] scene '{hubSceneName}' is not in Build Settings");
                return;
            }
            SceneManager.LoadScene(hubSceneName);
        }
    }
}
