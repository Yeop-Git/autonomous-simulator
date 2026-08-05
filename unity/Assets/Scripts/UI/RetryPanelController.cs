using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace V2X.UI
{
    public class RetryPanelController : MonoBehaviour
    {
        public GameObject panel;
        public Button retryButton;

        private void Awake() => retryButton?.onClick.AddListener(Retry);

        public void Show()
        {
            if (panel != null) panel.SetActive(true);
        }

        public void Retry()
        {
            Time.timeScale = 1f;
            SceneManager.LoadScene(SceneManager.GetActiveScene().name);
        }
    }
}
