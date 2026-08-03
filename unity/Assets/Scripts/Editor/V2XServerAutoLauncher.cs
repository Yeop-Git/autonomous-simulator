using System;
using System.Diagnostics;
using System.IO;
using System.Net.Sockets;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using Debug = UnityEngine.Debug;

namespace V2X.EditorTools
{
    /// <summary>
    /// Starts the matching Python V2X server when Editor Play begins and
    /// stops only the process that this launcher created when Play ends.
    /// </summary>
    [InitializeOnLoad]
    public static class V2XServerAutoLauncher
    {
        private const string EnabledKey = "V2X.AutoStartPythonServer";
        private const string ProcessIdKey = "V2X.AutoServerPid";
        private const int Port = 8765;
        private static Process _process;

        static V2XServerAutoLauncher()
        {
            EditorApplication.playModeStateChanged -= OnPlayModeChanged;
            EditorApplication.playModeStateChanged += OnPlayModeChanged;
            ReattachTrackedProcess();
        }

        [MenuItem("V2X/Auto Start Python Server")]
        private static void ToggleEnabled()
        {
            bool enabled = !EditorPrefs.GetBool(EnabledKey, true);
            EditorPrefs.SetBool(EnabledKey, enabled);
            Menu.SetChecked("V2X/Auto Start Python Server", enabled);
            Debug.Log($"[V2X Server] automatic launch {(enabled ? "enabled" : "disabled")}. ");
        }

        [MenuItem("V2X/Auto Start Python Server", true)]
        private static bool ValidateToggle()
        {
            Menu.SetChecked("V2X/Auto Start Python Server",
                EditorPrefs.GetBool(EnabledKey, true));
            return true;
        }

        [MenuItem("V2X/Start Python Server Now")]
        private static void StartNow() => StartForActiveScene();

        [MenuItem("V2X/Stop Automatic Python Server")]
        private static void StopNow() => StopTrackedProcess();

        private static void OnPlayModeChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.ExitingEditMode
                && EditorPrefs.GetBool(EnabledKey, true))
                StartForActiveScene();
            // Let runtime V2XClient.OnDestroy send its WebSocket close frame
            // before terminating Python after the Editor is back in Edit mode.
            else if (state == PlayModeStateChange.EnteredEditMode)
                StopTrackedProcess();
        }

        private static void StartForActiveScene()
        {
            ReattachTrackedProcess();
            if (_process != null && !_process.HasExited) return;

            string scene = SceneManager.GetActiveScene().name;
            if (string.IsNullOrWhiteSpace(scene))
            {
                Debug.LogWarning("[V2X Server] save the scene before entering Play mode.");
                return;
            }

            string unityDir = Directory.GetParent(Application.dataPath)?.FullName;
            string projectRoot = unityDir != null ? Directory.GetParent(unityDir)?.FullName : null;
            if (projectRoot == null)
            {
                Debug.LogError("[V2X Server] could not resolve repository root.");
                return;
            }

            string serverDir = Path.Combine(projectRoot, "server");
            string network = Path.Combine(serverDir, "scenarios", $"{scene}_lanes.json");
            if (!File.Exists(network))
            {
                Debug.LogError($"[V2X Server] network export not found: {network}\n" +
                               "Run V2X > Build All Demo Scenes once.");
                return;
            }

            if (IsPortOpen())
            {
                Debug.LogWarning("[V2X Server] ws://localhost:8765 is already in use; " +
                                 "the existing server will be reused. Confirm it uses " +
                                 $"{Path.GetFileName(network)}.");
                return;
            }

            string venvPython = Path.Combine(serverDir, ".venv", "Scripts", "python.exe");
            string python = ResolvePython(serverDir);
            if (python == null)
            {
                Debug.LogError("[V2X Server] python.exe was not found. Install Python or " +
                               "create server/.venv before entering Play mode.");
                return;
            }
            if (!File.Exists(venvPython))
                Debug.Log("[V2X Server] server/.venv was not found; using system Python. " +
                          "A project virtual environment is recommended.");

            try
            {
                var info = new ProcessStartInfo
                {
                    FileName = python,
                    // Repository paths are space-free; avoiding embedded quotes
                    // also keeps Unity/Mono's Windows argument parser reliable.
                    Arguments = $"-u {Path.Combine(serverDir, "auto_server.py")} --network {network}",
                    WorkingDirectory = serverDir,
                    UseShellExecute = false,
                    CreateNoWindow = true,
                    // Do not redirect process streams: Unity reloads its AppDomain
                    // while entering Play mode, which would close those pipes and
                    // terminate Python with a broken stdout/stderr stream.
                    RedirectStandardOutput = false,
                    RedirectStandardError = false,
                };
                string logPath = Path.Combine(unityDir, "Temp", "V2XServerAuto.log");
                Directory.CreateDirectory(Path.GetDirectoryName(logPath));
                File.WriteAllText(logPath,
                    $"python={python}{Environment.NewLine}network={network}{Environment.NewLine}");
                _process = new Process { StartInfo = info, EnableRaisingEvents = true };
                _process.Start();
                SessionState.SetInt(ProcessIdKey, _process.Id);
                Debug.Log($"[V2X Server] starting {Path.GetFileName(network)} " +
                          $"(PID {_process.Id}, ws://localhost:{Port}).");
            }
            catch (Exception e)
            {
                _process?.Dispose();
                _process = null;
                SessionState.EraseInt(ProcessIdKey);
                Debug.LogError("[V2X Server] automatic start failed: " + e.Message +
                               "\nCreate server/.venv and install requirements.txt.");
            }
        }

        private static bool IsPortOpen()
        {
            try
            {
                using var client = new TcpClient();
                var result = client.BeginConnect("127.0.0.1", Port, null, null);
                bool connected = result.AsyncWaitHandle.WaitOne(120);
                if (connected) client.EndConnect(result);
                return connected;
            }
            catch { return false; }
        }

        private static string ResolvePython(string serverDir)
        {
            string venv = Path.Combine(serverDir, ".venv", "Scripts", "python.exe");
            if (File.Exists(venv)) return venv;

            string path = Environment.GetEnvironmentVariable("PATH") ?? "";
            foreach (string entry in path.Split(Path.PathSeparator))
            {
                if (string.IsNullOrWhiteSpace(entry)) continue;
                string candidate = Path.Combine(entry.Trim().Trim('"'), "python.exe");
                if (File.Exists(candidate)) return candidate;
            }

            string driveRoot = Path.GetPathRoot(serverDir);
            if (!string.IsNullOrEmpty(driveRoot))
            {
                try
                {
                    foreach (string directory in Directory.GetDirectories(driveRoot, "Python*"))
                    {
                        string candidate = Path.Combine(directory, "python.exe");
                        if (File.Exists(candidate)) return candidate;
                    }
                }
                catch { /* some drive roots may not allow enumeration */ }
            }

            string local = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData);
            string programs = Path.Combine(local, "Programs", "Python");
            if (Directory.Exists(programs))
            {
                foreach (string candidate in Directory.GetFiles(programs, "python.exe",
                             SearchOption.AllDirectories))
                    return candidate;
            }
            return null;
        }

        private static void ReattachTrackedProcess()
        {
            if (_process != null && !_process.HasExited) return;
            int pid = SessionState.GetInt(ProcessIdKey, 0);
            if (pid <= 0) return;
            try
            {
                _process = Process.GetProcessById(pid);
                if (_process.HasExited) _process = null;
            }
            catch { _process = null; }
            if (_process == null) SessionState.EraseInt(ProcessIdKey);
        }

        private static void StopTrackedProcess()
        {
            ReattachTrackedProcess();
            if (_process == null) return;
            try
            {
                if (!_process.HasExited)
                {
                    int pid = _process.Id;
                    _process.Kill();
                    _process.WaitForExit(1500);
                    Debug.Log($"[V2X Server] stopped automatic server (PID {pid}).");
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning("[V2X Server] could not stop automatic server: " + e.Message);
            }
            finally
            {
                _process.Dispose();
                _process = null;
                SessionState.EraseInt(ProcessIdKey);
            }
        }
    }
}
