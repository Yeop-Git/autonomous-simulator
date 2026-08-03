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

    /// <summary>Spawns one-shot pedestrians once at the start of each pedestrian phase.</summary>
    public class PedestrianSpawner : MonoBehaviour
    {
        public TrafficLightSystem signals;
        public PedestrianRoute[] routes;
        public PedestrianSignalVisual[] pedestrianSignals;
        public float walkingSpeed = 1.9f;

        private int _sequence;
        private bool _wasWalk;

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
            if (walk && !_wasWalk) SpawnWave();
            _wasWalk = walk;
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
                var go = GameObject.CreatePrimitive(PrimitiveType.Capsule);
                go.name = $"pedestrian_{_sequence++:000}";
                go.transform.position = route.spawn;
                go.transform.localScale = new Vector3(.7f, 1f, .7f);
                go.GetComponent<Renderer>().material.color =
                    new Color(1f, .25f + UnityEngine.Random.value * .5f, .75f);
                var agent = go.AddComponent<DynamicObjectAgent>();
                agent.objectId = go.name;
                agent.objectType = "pedestrian";
                agent.radius = .4f;
                var crossing = go.AddComponent<PedestrianCrossingAgent>();
                crossing.destination = route.destination;
                crossing.speed = walkingSpeed;
            }
        }
    }
}
