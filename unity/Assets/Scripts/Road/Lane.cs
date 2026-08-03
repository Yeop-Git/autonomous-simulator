using System.Collections.Generic;
using UnityEngine;

// One lane = an ordered centerline plus metadata (plan §10.1). Authored in the
// editor: drop empty child GameObjects as waypoints (in order from lane start
// to end), or assign them explicitly to `waypoints`. Width / speed limit /
// neighbour links are set in the inspector and exported to the Python server
// via LaneNetworkExporter (matches shared/protocol/lane_network.schema.json).

namespace V2X.Road
{
    public class Lane : MonoBehaviour
    {
        [Tooltip("Stable id used on the wire. Defaults to the GameObject name.")]
        public string laneId;
        public float width = 3.5f;
        [Tooltip("Speed limit in m/s. 13.9 ~= 50 km/h, 27.8 ~= 100 km/h.")]
        public float speedLimit = 13.9f;

        [Header("Centerline (order = start -> end)")]
        [Tooltip("Leave empty to auto-use child transforms in hierarchy order.")]
        public List<Transform> waypoints = new();

        [Header("Graph links")]
        public Lane leftLane;
        public Lane rightLane;
        public List<Lane> nextLanes = new();

        public string Id => string.IsNullOrEmpty(laneId) ? name : laneId;

        private readonly List<Vector3> _cache = new();

        public IReadOnlyList<Vector3> Centerline()
        {
            _cache.Clear();
            if (waypoints != null && waypoints.Count >= 2)
            {
                foreach (var t in waypoints)
                    if (t != null) _cache.Add(t.position);
            }
            else
            {
                foreach (Transform child in transform)
                    _cache.Add(child.position);
            }
            return _cache;
        }

        // Closest point on the polyline (xz) + lateral distance + arc length.
        public Vector3 ClosestPoint(Vector3 p, out float lateral, out float arc)
        {
            var cl = Centerline();
            Vector3 best = cl.Count > 0 ? cl[0] : transform.position;
            lateral = float.MaxValue;
            arc = 0f;
            float acc = 0f;
            for (int i = 0; i < cl.Count - 1; i++)
            {
                Vector3 a = cl[i], b = cl[i + 1];
                Vector3 ab = b - a; ab.y = 0f;
                float segSq = ab.sqrMagnitude;
                float segLen = Mathf.Sqrt(segSq);
                Vector3 ap = p - a; ap.y = 0f;
                float t = segSq > 0f ? Mathf.Clamp01(Vector3.Dot(ap, ab) / segSq) : 0f;
                Vector3 proj = a + t * ab;
                float lat = new Vector2(p.x - proj.x, p.z - proj.z).magnitude;
                if (lat < lateral)
                {
                    lateral = lat;
                    best = proj;
                    arc = acc + t * segLen;
                }
                acc += segLen;
            }
            return best;
        }

        public float Length()
        {
            var cl = Centerline();
            float len = 0f;
            for (int i = 0; i < cl.Count - 1; i++)
            {
                Vector3 d = cl[i + 1] - cl[i]; d.y = 0f;
                len += d.magnitude;
            }
            return len;
        }

        public float HeadingAtArc(float targetArc)
        {
            var cl = Centerline();
            float acc = 0f;
            for (int i = 0; i < cl.Count - 1; i++)
            {
                Vector3 d = cl[i + 1] - cl[i];
                d.y = 0f;
                float len = d.magnitude;
                if (len > 1e-4f && targetArc <= acc + len)
                    return Mathf.Atan2(d.x, d.z) * Mathf.Rad2Deg;
                acc += len;
            }
            if (cl.Count >= 2)
            {
                Vector3 d = cl[^1] - cl[^2];
                return Mathf.Atan2(d.x, d.z) * Mathf.Rad2Deg;
            }
            return 0f;
        }

        private void OnDrawGizmos()
        {
            var cl = Centerline();
            Gizmos.color = Color.cyan;
            for (int i = 0; i < cl.Count - 1; i++)
                Gizmos.DrawLine(cl[i], cl[i + 1]);
            Gizmos.color = Color.yellow;
            foreach (var p in cl) Gizmos.DrawSphere(p, 0.3f);
        }
    }
}
