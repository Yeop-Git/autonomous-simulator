using UnityEngine;
using V2X.Vehicle;

// Draws the server-returned route for a vehicle with a LineRenderer at runtime
// (plan §16.2 PathVisualizer). Attach to the same GameObject as a
// VehicleController, or assign one. Auto-adds a LineRenderer if missing.

namespace V2X.Visualization
{
    [RequireComponent(typeof(LineRenderer))]
    public class PathVisualizer : MonoBehaviour
    {
        public VehicleController vehicle;
        public float yOffset = 0.2f;
        public Color color = new(0.2f, 1f, 0.4f);

        private LineRenderer _lr;

        private void Awake()
        {
            _lr = GetComponent<LineRenderer>();
            _lr.widthMultiplier = 0.3f;
            _lr.material = new Material(Shader.Find("Sprites/Default"));
            _lr.startColor = _lr.endColor = color;
            if (vehicle == null) vehicle = GetComponent<VehicleController>();
        }

        private void LateUpdate()
        {
            if (vehicle == null || vehicle.Path == null || vehicle.Path.Count < 2)
            {
                _lr.positionCount = 0;
                return;
            }
            var path = vehicle.Path;
            _lr.positionCount = path.Count;
            for (int i = 0; i < path.Count; i++)
                _lr.SetPosition(i, path[i] + Vector3.up * yOffset);
        }
    }
}
