using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using V2X.Road;
using V2X.Vehicle;

// The server-confirmed route is a solid line. Alternative lanes that remain
// available from the current lane are rendered as translucent dashed lines.
namespace V2X.Visualization
{
    [RequireComponent(typeof(LineRenderer))]
    public class PathVisualizer : MonoBehaviour
    {
        public VehicleController vehicle;
        public float yOffset = 0.2f;
        public Color color = new(0.2f, 1f, 0.4f);

        private LineRenderer _confirmedLine;
        private RoadNetworkManager _road;
        private readonly List<LineRenderer> _possibleLines = new();
        private string _possibleCacheKey;

        private void Awake()
        {
            _confirmedLine = GetComponent<LineRenderer>();
            ConfigureLine(_confirmedLine, .3f, color, false);
            if (vehicle == null) vehicle = GetComponent<VehicleController>();
            _road = FindFirstObjectByType<RoadNetworkManager>();
        }

        private void LateUpdate()
        {
            DrawConfirmedRoute();
            DrawPossibleRoutes();
        }

        private void DrawConfirmedRoute()
        {
            if (vehicle == null || vehicle.Path == null || vehicle.Path.Count < 2)
            {
                _confirmedLine.positionCount = 0;
                return;
            }

            var path = vehicle.Path;
            _confirmedLine.positionCount = path.Count;
            for (int i = 0; i < path.Count; i++)
                _confirmedLine.SetPosition(i, path[i] + Vector3.up * yOffset);
        }

        private void DrawPossibleRoutes()
        {
            if (vehicle == null || _road == null || string.IsNullOrEmpty(vehicle.CurrentLaneId))
                return;

            string key = $"{vehicle.CurrentLaneId}|{vehicle.maneuver}|{vehicle.RequestedTargetLane}";
            if (key == _possibleCacheKey) return;
            _possibleCacheKey = key;
            ClearPossibleLines();

            var current = _road.GetLane(vehicle.CurrentLaneId);
            if (current == null) return;

            AddAdjacentCandidate(current.leftLane);
            AddAdjacentCandidate(current.rightLane);
            foreach (var next in current.nextLanes)
            {
                if (next == null || IsConfirmedNext(next, current.nextLanes.Count)) continue;
                var points = new List<Vector3>(next.Centerline());
                if (next.nextLanes.Count > 0 && next.nextLanes[0] != null)
                {
                    var continuation = next.nextLanes[0].Centerline();
                    for (int i = 1; i < continuation.Count; i++) points.Add(continuation[i]);
                }
                AddPossibleLine(points, next.Id);
            }
        }

        private void AddAdjacentCandidate(Lane lane)
        {
            if (lane == null || lane.Id == vehicle.RequestedTargetLane) return;
            AddPossibleLine(lane.Centerline(), lane.Id);
        }

        private bool IsConfirmedNext(Lane lane, int optionCount)
        {
            if (optionCount == 1) return true;
            string id = lane.Id.ToLowerInvariant();
            return (!string.IsNullOrEmpty(vehicle.maneuver) && id.Contains(vehicle.maneuver)) ||
                   lane.Id == vehicle.RequestedTargetLane;
        }

        private void AddPossibleLine(IReadOnlyList<Vector3> points, string laneId)
        {
            if (points == null || points.Count < 2) return;
            var go = new GameObject($"Possible Route {laneId}");
            go.transform.SetParent(transform, false);
            var line = go.AddComponent<LineRenderer>();
            Color possibleColor = new(color.r, color.g, color.b, .48f);
            ConfigureLine(line, .2f, possibleColor, true);
            line.positionCount = points.Count;
            float length = 0f;
            for (int i = 0; i < points.Count; i++)
            {
                line.SetPosition(i, points[i] + Vector3.up * (yOffset + .03f));
                if (i > 0) length += Vector3.Distance(points[i - 1], points[i]);
            }
            line.material.mainTextureScale = new Vector2(Mathf.Max(1f, length / 3f), 1f);
            _possibleLines.Add(line);
        }

        private static void ConfigureLine(LineRenderer line, float width, Color lineColor, bool dashed)
        {
            line.useWorldSpace = true;
            line.widthMultiplier = width;
            line.textureMode = dashed ? LineTextureMode.Tile : LineTextureMode.Stretch;
            line.shadowCastingMode = ShadowCastingMode.Off;
            line.receiveShadows = false;
            line.material = dashed ? CreateDashedMaterial() :
                new Material(Shader.Find("Sprites/Default"));
            line.startColor = line.endColor = lineColor;
        }

        private void ClearPossibleLines()
        {
            foreach (var line in _possibleLines)
                if (line != null) Destroy(line.gameObject);
            _possibleLines.Clear();
        }

        private static Material CreateDashedMaterial()
        {
            var texture = new Texture2D(8, 1, TextureFormat.RGBA32, false)
            {
                name = "V2X Route Dashes",
                wrapMode = TextureWrapMode.Repeat,
                filterMode = FilterMode.Point,
            };
            for (int x = 0; x < texture.width; x++)
                texture.SetPixel(x, 0, x < 5 ? Color.white : Color.clear);
            texture.Apply();

            var material = new Material(Shader.Find("Sprites/Default"));
            material.mainTexture = texture;
            return material;
        }
    }
}
