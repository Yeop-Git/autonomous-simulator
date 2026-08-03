using UnityEngine;

namespace V2X.Sim
{
    /// <summary>One-shot pedestrian: cross once, unregister, then destroy.</summary>
    [RequireComponent(typeof(DynamicObjectAgent))]
    public class PedestrianCrossingAgent : MonoBehaviour
    {
        public Vector3 destination;
        public float speed = 1.9f;

        private DynamicObjectAgent _agent;
        private SimulationManager _simulation;

        private void Start()
        {
            _agent = GetComponent<DynamicObjectAgent>();
            _simulation = FindFirstObjectByType<SimulationManager>();
            _simulation?.RegisterObject(_agent);
        }

        private void Update()
        {
            transform.position = Vector3.MoveTowards(
                transform.position, destination, speed * Time.deltaTime);
            if ((transform.position - destination).sqrMagnitude > .01f) return;
            _simulation?.UnregisterObject(_agent);
            Destroy(gameObject);
        }
    }
}
