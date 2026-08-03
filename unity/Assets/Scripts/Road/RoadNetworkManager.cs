using System.Collections.Generic;
using UnityEngine;

// Scene-level registry of all Lanes. Collects them at startup, offers id
// lookup and nearest-lane queries (used to tag a vehicle's current_lane in
// the StateMessage). Geometry is authored by a human; this just indexes it.

namespace V2X.Road
{
    public class RoadNetworkManager : MonoBehaviour
    {
        public List<Lane> lanes = new();
        private readonly Dictionary<string, Lane> _byId = new();

        private void Awake() => Rebuild();

        // Re-scan the scene for lanes. Call after spawning/editing lanes.
        public void Rebuild()
        {
            if (lanes == null || lanes.Count == 0)
                lanes = new List<Lane>(FindObjectsByType<Lane>(FindObjectsSortMode.None));
            _byId.Clear();
            foreach (var lane in lanes)
                if (lane != null) _byId[lane.Id] = lane;
        }

        public Lane GetLane(string id) =>
            id != null && _byId.TryGetValue(id, out var lane) ? lane : null;

        public Lane NearestLane(Vector3 position)
        {
            Lane best = null;
            float bestLat = float.MaxValue;
            foreach (var lane in lanes)
            {
                if (lane == null) continue;
                lane.ClosestPoint(position, out float lat, out _);
                if (lat < bestLat) { bestLat = lat; best = lane; }
            }
            return best;
        }

        public string NearestLaneId(Vector3 position)
        {
            var lane = NearestLane(position);
            return lane != null ? lane.Id : null;
        }

        public string NearestLaneId(Vector3 position, float headingDeg)
        {
            Lane best = null;
            float bestScore = float.MaxValue;
            foreach (var lane in lanes)
            {
                if (lane == null) continue;
                lane.ClosestPoint(position, out float lateral, out float arc);
                float delta = Mathf.Abs(Mathf.DeltaAngle(headingDeg, lane.HeadingAtArc(arc)));
                // A wrong-way overlapping connector is much worse than a
                // slightly farther lane aligned with the vehicle.
                float score = lateral + delta / 30f;
                if (score < bestScore) { bestScore = score; best = lane; }
            }
            return best != null ? best.Id : null;
        }
    }
}
