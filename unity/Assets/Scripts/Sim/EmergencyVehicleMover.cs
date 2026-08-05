using UnityEngine;

namespace V2X.Sim
{
    /// <summary>Deterministic emergency-vehicle motion and siren visualization.</summary>
    [RequireComponent(typeof(DynamicObjectAgent))]
    public class EmergencyVehicleMover : MonoBehaviour
    {
        public float speed = 24f;
        public float despawnZ = 340f;
        public Renderer leftBeacon;
        public Renderer rightBeacon;
        public float flashHz = 5f;

        private Material _leftMaterial;
        private Material _rightMaterial;

        private void Start()
        {
            if (leftBeacon != null) _leftMaterial = leftBeacon.material;
            if (rightBeacon != null) _rightMaterial = rightBeacon.material;
        }

        private void FixedUpdate()
        {
            transform.position += transform.forward * (speed * Time.fixedDeltaTime);
            if (Mathf.Abs(transform.position.z) > despawnZ) Destroy(gameObject);
        }

        private void Update()
        {
            bool leftOn = Mathf.FloorToInt(Time.time * flashHz) % 2 == 0;
            SetBeacon(_leftMaterial, leftOn ? Color.red : new Color(.15f, 0f, 0f));
            SetBeacon(_rightMaterial, leftOn ? new Color(0f, 0f, .15f) : Color.blue);
        }

        private static void SetBeacon(Material material, Color value)
        {
            if (material == null) return;
            material.color = value;
            if (material.HasProperty("_EmissionColor"))
            {
                material.EnableKeyword("_EMISSION");
                material.SetColor("_EmissionColor", value * 2.5f);
            }
        }
    }
}
