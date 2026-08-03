using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using V2X.Road;

// Editor tool: export every Lane in the open scene to the lane-network JSON
// the Python server loads (shared/protocol/lane_network.schema.json). Run via
// the menu "V2X / Export Lane Network...". This is how a human-authored scene
// becomes the planner's search graph — no manual JSON editing.

namespace V2X.EditorTools
{
    public static class LaneNetworkExporter
    {
        // Plain DTOs so Newtonsoft emits exactly the schema's snake_case keys.
        private class LaneDto
        {
            public string id;
            public List<float[]> centerline = new();
            public float width;
            public float speed_limit;
            public string left_lane_id;
            public string right_lane_id;
            public List<string> next_lane_ids = new();
        }

        private class NetworkDto
        {
            public string name;
            public string scenario;
            public List<LaneDto> lanes = new();
        }

        [MenuItem("V2X/Export Lane Network...")]
        public static void Export()
        {
            string scene = UnityEngine.SceneManagement.SceneManager.GetActiveScene().name;
            string path = EditorUtility.SaveFilePanel(
                "Export Lane Network", DefaultDir(), $"{scene}_lanes.json", "json");
            if (string.IsNullOrEmpty(path)) return;

            try
            {
                ExportTo(path);
                EditorUtility.RevealInFinder(path);
            }
            catch (System.InvalidOperationException e)
            {
                EditorUtility.DisplayDialog("Export Lane Network", e.Message, "OK");
            }
        }

        /// <summary>Non-interactive export entry point for MCP/editor automation.</summary>
        public static string ExportTo(string path)
        {
            var lanes = Object.FindObjectsByType<Lane>(FindObjectsSortMode.None);
            if (lanes == null || lanes.Length == 0)
                throw new System.InvalidOperationException("No Lane components found in the open scene.");

            string scene = UnityEngine.SceneManagement.SceneManager.GetActiveScene().name;
            string scenario = GuessScenario(scene);

            var net = new NetworkDto { name = scene, scenario = scenario };
            foreach (var lane in lanes)
            {
                var dto = new LaneDto
                {
                    id = lane.Id,
                    width = lane.width,
                    speed_limit = lane.speedLimit,
                    left_lane_id = lane.leftLane != null ? lane.leftLane.Id : null,
                    right_lane_id = lane.rightLane != null ? lane.rightLane.Id : null,
                };
                foreach (var p in lane.Centerline())
                    dto.centerline.Add(new[] { p.x, p.y, p.z });
                foreach (var nxt in lane.nextLanes)
                    if (nxt != null) dto.next_lane_ids.Add(nxt.Id);
                net.lanes.Add(dto);
            }

            string directory = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(path, JsonConvert.SerializeObject(net, Formatting.Indented));
            Debug.Log($"[LaneNetworkExporter] wrote {net.lanes.Count} lanes to {path}");
            return path;
        }

        public static string ExportToDefaultLocation()
        {
            string scene = UnityEngine.SceneManagement.SceneManager.GetActiveScene().name;
            return ExportTo(Path.Combine(DefaultDir(), $"{scene}_lanes.json"));
        }

        private static string GuessScenario(string scene)
        {
            string s = scene.ToLowerInvariant();
            if (s.Contains("urban") || s.Contains("city")) return "urban";
            if (s.Contains("lka") || s.Contains("test")) return "lka_test";
            return "highway";
        }

        private static string DefaultDir()
        {
            // repo/server/scenarios if reachable, else project root
            string proj = Directory.GetParent(Application.dataPath)?.Parent?.FullName;
            string dir = proj != null ? Path.Combine(proj, "server", "scenarios") : Application.dataPath;
            return Directory.Exists(dir) ? dir : Application.dataPath;
        }
    }
}
