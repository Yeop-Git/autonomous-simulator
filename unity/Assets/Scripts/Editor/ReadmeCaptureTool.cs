using System;
using System.IO;
using UnityEditor;
using UnityEngine;
using V2X.Communication;
using V2X.Sim;
using V2X.UI;
using V2X.Vehicle;

namespace V2X.EditorTools
{
    /// <summary>
    /// Captures deterministic Game View sequences for the repository README.
    /// A caller stores a JSON CaptureConfig in EditorPrefs before entering Play
    /// Mode; this editor-only hook consumes it and records frames plus runtime
    /// metadata without modifying a scene.
    /// </summary>
    [InitializeOnLoad]
    public static class ReadmeCaptureTool
    {
        public const string ConfigKey = "V2X.ReadmeCapture.Config";

        [Serializable]
        public sealed class CaptureConfig
        {
            public string outputDirectory = "../docs/videos/capture-frames";
            public string action = "none";
            public int cameraView;
            public int frameCount = 12;
            public float intervalSeconds = 0.65f;
            public float timeScale = 1f;
        }

        private static CaptureConfig _config;
        private static string _outputDirectory;
        private static string _metadataPath;
        private static int _frame;
        private static double _nextCaptureAt;

        static ReadmeCaptureTool()
        {
            EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
            EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
        }

        private static void OnPlayModeStateChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.EnteredPlayMode &&
                EditorPrefs.HasKey(ConfigKey))
            {
                EditorApplication.delayCall += BeginCapture;
            }
            else if (state == PlayModeStateChange.ExitingPlayMode)
            {
                StopCapture();
            }
        }

        private static void BeginCapture()
        {
            string json = EditorPrefs.GetString(ConfigKey, string.Empty);
            EditorPrefs.DeleteKey(ConfigKey);
            _config = JsonUtility.FromJson<CaptureConfig>(json);
            if (_config == null)
            {
                Debug.LogError("[ReadmeCapture] Invalid capture configuration.");
                return;
            }

            string projectRoot = Directory.GetParent(Application.dataPath)?.FullName ??
                                 Application.dataPath;
            _outputDirectory = Path.GetFullPath(
                Path.Combine(projectRoot, _config.outputDirectory));
            string videosRoot = Path.GetFullPath(
                Path.Combine(projectRoot, "../docs/videos"));
            if (!_outputDirectory.StartsWith(
                    videosRoot + Path.DirectorySeparatorChar,
                    StringComparison.OrdinalIgnoreCase))
            {
                Debug.LogError($"[ReadmeCapture] Output must be below {videosRoot}");
                _config = null;
                return;
            }
            Directory.CreateDirectory(_outputDirectory);
            foreach (string file in Directory.GetFiles(_outputDirectory, "frame-*.png"))
                File.Delete(file);

            _metadataPath = Path.Combine(_outputDirectory, "capture.csv");
            File.WriteAllText(_metadataPath,
                "frame,tick,behavior,lane,target_lane,maneuver,speed_kmh\n");
            _frame = 0;
            _nextCaptureAt = EditorApplication.timeSinceStartup;
            Time.timeScale = Mathf.Max(0.05f, _config.timeScale);

            var camera = UnityEngine.Object.FindFirstObjectByType<CameraViewController>();
            camera?.SelectView(_config.cameraView);
            PrepareAction();

            EditorApplication.update -= CaptureUpdate;
            EditorApplication.update += CaptureUpdate;
            Debug.Log($"[ReadmeCapture] Started action={_config.action} " +
                      $"frames={_config.frameCount} output={_outputDirectory}");
        }

        private static void CaptureUpdate()
        {
            if (!EditorApplication.isPlaying)
            {
                StopCapture();
                return;
            }
            if (EditorApplication.timeSinceStartup < _nextCaptureAt) return;

            TriggerAction(_frame);
            ScreenCapture.CaptureScreenshot(
                Path.Combine(_outputDirectory, $"frame-{_frame:00}.png"), 1);

            var client = UnityEngine.Object.FindFirstObjectByType<V2XClient>();
            var ego = UnityEngine.Object.FindFirstObjectByType<VehicleController>();
            File.AppendAllText(_metadataPath,
                $"{_frame},{client?.LastAppliedTick ?? -1}," +
                $"{ego?.Behavior ?? "none"},{ego?.CurrentLaneId ?? "none"}," +
                $"{ego?.RequestedTargetLane ?? "none"}," +
                $"{ego?.maneuver ?? "none"},{(ego != null ? ego.CurrentSpeed * 3.6f : 0f):0.0}\n");

            _frame++;
            _nextCaptureAt = EditorApplication.timeSinceStartup +
                             Math.Max(0.1f, _config.intervalSeconds);
            if (_frame >= _config.frameCount)
            {
                Debug.Log($"[ReadmeCapture] Completed {_frame} frames at {_outputDirectory}");
                StopCapture();
            }
        }

        private static void PrepareAction()
        {
            if (_config.action == "highway_merge_lane_change")
            {
                // Use a repeatable, safe lead gap so the requested move toward
                // the ramp lane is accepted during the short README capture.
                var rampCar = GameObject.Find("ramp_car");
                if (rampCar != null)
                    rampCar.transform.position += rampCar.transform.forward * 24f;
            }

            if (_config.action == "emergency_demo")
            {
                var scenario = UnityEngine.Object.FindFirstObjectByType<
                    EmergencyAvoidanceScenarioController>();
                if (scenario != null) scenario.automaticDemo = false;
            }
        }

        private static void TriggerAction(int frame)
        {
            if (frame >= 2 && frame <= 8 &&
                _config.action == "highway_merge_lane_change")
            {
                UnityEngine.Object.FindFirstObjectByType<HighwayLaneChangeController>()
                    ?.RequestRight();
                GameObject.Find("car_0")?.GetComponent<VehicleController>()
                    ?.RequestTargetLane("hw_l2");
            }

            if (frame == 1 && _config.action.StartsWith("urban_",
                    StringComparison.Ordinal))
            {
                var strategy = UnityEngine.Object.FindFirstObjectByType<
                    UrbanDrivingStrategyController>();
                switch (_config.action)
                {
                    case "urban_straight": strategy?.SelectStraight(); break;
                    case "urban_left": strategy?.SelectLeft(); break;
                    case "urban_right": strategy?.SelectRight(); break;
                }
            }

            if (_config.action == "emergency_demo")
            {
                var scenario = UnityEngine.Object.FindFirstObjectByType<
                    EmergencyAvoidanceScenarioController>();
                if (frame == 1) scenario?.SpawnObstacle();
                if (frame == 9) scenario?.DispatchEmergencyVehicle();
            }
        }

        private static void StopCapture()
        {
            EditorApplication.update -= CaptureUpdate;
            if (EditorApplication.isPlaying) Time.timeScale = 1f;
            _config = null;
        }
    }
}
