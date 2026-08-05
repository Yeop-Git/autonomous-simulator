using System;
using UnityEngine;
using V2X.UI;

namespace V2X.Sim
{
    [Serializable]
    public class PedestrianRoute
    {
        public Vector3 spawn;
        public Vector3 destination;
    }

    [Serializable]
    public class PedestrianSignalVisual
    {
        public Renderer[] redIcon;
        public Renderer[] greenIcon;
    }

    /// <summary>Pre-spawns pedestrians to wait for their signal, then replenishes each wave.</summary>
    public class PedestrianSpawner : MonoBehaviour
    {
        public TrafficLightSystem signals;
        public PedestrianRoute[] routes;
        public PedestrianSignalVisual[] pedestrianSignals;
        public GameObject pedestrianPrefab;
        public Material pedestrianBodyMaterial;
        public Material pedestrianJointMaterial;
        public Color[] pedestrianColors;
        public float walkingSpeed = 1.9f;
        public float pedestrianScale = 1.25f;
        public float postCrossingDistance = 3f;
        public float postCrossingWait = 2f;

        private int _sequence;
        private int _activePedestrians;

        private void Start() => SpawnWave();

        private void Update()
        {
            bool walk = signals != null && signals.PedestriansMayCross;
            if (pedestrianSignals != null)
                foreach (var signal in pedestrianSignals)
                {
                    if (signal == null) continue;
                    SetIcon(signal.greenIcon, new Color(.15f, 1f, .4f), walk);
                    SetIcon(signal.redIcon, Color.red, !walk);
                }
            if (_activePedestrians == 0 && !walk) SpawnWave();
        }

        private static void SetIcon(Renderer[] parts, Color color, bool active)
        {
            if (parts == null) return;
            foreach (var part in parts)
            {
                if (part == null) continue;
                Color shown = active ? color : color * .08f;
                part.material.color = shown;
                if (!part.material.HasProperty("_EmissionColor")) continue;
                part.material.EnableKeyword("_EMISSION");
                part.material.SetColor("_EmissionColor", active ? color * 2f : Color.black);
            }
        }

        private void SpawnWave()
        {
            if (routes == null) return;
            foreach (var route in routes)
            {
                if (route == null) continue;
                var go = new GameObject($"pedestrian_{_sequence++:000}");
                go.transform.position = route.spawn;
                Vector3 travel = route.destination - route.spawn;
                travel.y = 0f;
                if (travel.sqrMagnitude > .001f)
                    go.transform.rotation = Quaternion.LookRotation(travel.normalized, Vector3.up);

                if (pedestrianPrefab != null)
                {
                    var visual = Instantiate(pedestrianPrefab, go.transform);
                    visual.name = $"Visual_{pedestrianPrefab.name}";
                    visual.transform.localPosition = Vector3.down;
                    visual.transform.localRotation = Quaternion.identity;
                    visual.transform.localScale = Vector3.one * pedestrianScale;
                    int colorIndex = pedestrianColors != null && pedestrianColors.Length > 0
                        ? (_sequence - 1) % pedestrianColors.Length
                        : 0;
                    Color bodyColor = pedestrianColors != null && pedestrianColors.Length > 0
                        ? pedestrianColors[colorIndex]
                        : Color.yellow;
                    ApplyPedestrianMaterials(visual, bodyColor);
                    foreach (var animator in visual.GetComponentsInChildren<Animator>(true))
                    {
                        animator.applyRootMotion = false;
                        animator.speed = 1f;
                        animator.SetBool("walk", false);
                    }
                }
                else
                {
                    var visual = GameObject.CreatePrimitive(PrimitiveType.Capsule);
                    visual.name = "Fallback Pedestrian Visual";
                    visual.transform.SetParent(go.transform, false);
                    visual.transform.localScale = new Vector3(.7f, 1f, .7f);
                    visual.GetComponent<Renderer>().material.color =
                        new Color(1f, .25f + UnityEngine.Random.value * .5f, .75f);
                }

                var agent = go.AddComponent<DynamicObjectAgent>();
                agent.objectId = go.name;
                agent.objectType = "pedestrian";
                agent.radius = .4f * pedestrianScale;
                var crossing = go.AddComponent<PedestrianCrossingAgent>();
                Vector3 exitDirection = travel.sqrMagnitude > .001f
                    ? travel.normalized
                    : Vector3.zero;
                crossing.destination = route.destination + exitDirection * postCrossingDistance;
                crossing.speed = walkingSpeed;
                crossing.signals = signals;
                crossing.owner = this;
                crossing.postCrossingWait = postCrossingWait;
                _activePedestrians++;
            }
        }

        internal void NotifyPedestrianRemoved()
        {
            _activePedestrians = Mathf.Max(0, _activePedestrians - 1);
        }

        private void ApplyPedestrianMaterials(GameObject visual, Color bodyColor)
        {
            foreach (var renderer in visual.GetComponentsInChildren<Renderer>(true))
            {
                var materials = renderer.sharedMaterials;
                for (int i = 0; i < materials.Length; i++)
                {
                    bool isJoint = materials[i] != null &&
                                   materials[i].name.IndexOf("joint", StringComparison.OrdinalIgnoreCase) >= 0;
                    materials[i] = isJoint ? pedestrianJointMaterial : pedestrianBodyMaterial;
                }
                renderer.sharedMaterials = materials;
                for (int i = 0; i < materials.Length; i++)
                {
                    var block = new MaterialPropertyBlock();
                    Color slotColor = materials[i] == pedestrianJointMaterial
                        ? new Color(.05f, .05f, .05f)
                        : bodyColor;
                    block.SetColor("_Color", slotColor);
                    block.SetColor("_BaseColor", slotColor);
                    renderer.SetPropertyBlock(block, i);
                }
            }
        }
    }
}
