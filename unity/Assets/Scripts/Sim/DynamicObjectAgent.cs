using UnityEngine;

// Non-vehicle dynamic objects the server must know about: pedestrians,
// bicycles, obstacles, emergency vehicles (plan §14.1). Reports id/type/
// position/velocity each tick. Motion is whatever drives this transform
// (animation, NavMeshAgent, a hand-authored path) — this only reports it.

namespace V2X.Sim
{
    public class DynamicObjectAgent : MonoBehaviour
    {
        [Tooltip("pedestrian | bicycle | emergency_vehicle | static_obstacle | unexpected_obstacle")]
        public string objectType = "pedestrian";
        public string objectId;
        public float radius = 0.4f;

        private Vector3 _lastPos;
        private Vector3 _velocity;

        public string Id => string.IsNullOrEmpty(objectId) ? name : objectId;
        public Vector3 Velocity => _velocity;

        private void Start() => _lastPos = transform.position;

        private void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;
            if (dt > 0f) _velocity = (transform.position - _lastPos) / dt;
            _lastPos = transform.position;
        }
    }
}
