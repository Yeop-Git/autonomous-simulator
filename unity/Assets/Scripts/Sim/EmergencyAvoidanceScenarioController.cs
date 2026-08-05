using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;
using V2X.Vehicle;

namespace V2X.Sim
{
    /// <summary>
    /// Repeatable obstacle/emergency triggers for the dedicated avoidance scene.
    /// Runtime-created objects are registered in the central V2X snapshot.
    /// </summary>
    public class EmergencyAvoidanceScenarioController : MonoBehaviour
    {
        public SimulationManager simulation;
        public VehicleController ego;
        public GameObject policePrefab;
        public string scenarioName = "emergency_avoidance";
        public bool automaticDemo = true;
        public float obstacleTriggerTime = 7f;
        public float emergencyTriggerTime = 22f;
        public float obstacleDistance = 48f;
        public float emergencySpawnDistance = 35f;

        public string LastEvent { get; private set; } = "대기 중";
        public float Runtime => Time.timeSinceLevelLoad;

        private DynamicObjectAgent _obstacle;
        private DynamicObjectAgent _emergency;
        private bool _obstacleTriggered;
        private bool _emergencyTriggered;

        private void Start()
        {
            simulation ??= FindFirstObjectByType<SimulationManager>();
            ego ??= FindFirstObjectByType<VehicleController>();
            if (simulation != null)
            {
                simulation.scenario = scenarioName;
                if (string.IsNullOrEmpty(simulation.plannerMode))
                    simulation.plannerMode = "rrt";
            }
        }

        private void Update()
        {
            var keyboard = Keyboard.current;
            if (keyboard != null)
            {
                if (keyboard.digit4Key.wasPressedThisFrame) SpawnObstacle();
                if (keyboard.digit5Key.wasPressedThisFrame) DispatchEmergencyVehicle();
                if (keyboard.digit6Key.wasPressedThisFrame) TogglePlanner();
                if (keyboard.digit0Key.wasPressedThisFrame) ResetScenario();
            }

            if (automaticDemo && !_obstacleTriggered && Runtime >= obstacleTriggerTime)
                SpawnObstacle();
            if (automaticDemo && !_emergencyTriggered && Runtime >= emergencyTriggerTime)
                DispatchEmergencyVehicle();

            if (_obstacle != null && ego != null &&
                Vector3.Dot(_obstacle.transform.position - ego.transform.position,
                            ego.transform.forward) < -22f)
            {
                simulation?.UnregisterObject(_obstacle);
                Destroy(_obstacle.gameObject);
                _obstacle = null;
                LastEvent = "낙하물 통과 · 원차선 복귀 판단";
            }
        }

        public void SpawnObstacle()
        {
            if (ego == null || _obstacle != null) return;
            _obstacleTriggered = true;
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = "Unexpected Falling Cargo";
            go.transform.position = ego.transform.position + ego.transform.forward * obstacleDistance;
            go.transform.position += Vector3.up * .65f;
            go.transform.rotation = Quaternion.Euler(18f, 27f, 12f);
            go.transform.localScale = new Vector3(2.1f, 1.3f, 1.6f);
            var renderer = go.GetComponent<Renderer>();
            renderer.material.color = new Color(1f, .22f, .04f);
            var agent = go.AddComponent<DynamicObjectAgent>();
            agent.objectId = "falling_cargo";
            agent.objectType = "unexpected_obstacle";
            agent.radius = 1.25f;
            _obstacle = agent;
            simulation?.RegisterObject(agent);
            CreateWarningBeacon(go.transform);
            LastEvent = "돌발 낙하물 발생 · 국부 경로 재계획";
        }

        public void DispatchEmergencyVehicle()
        {
            if (ego == null || _emergency != null) return;
            _emergencyTriggered = true;
            GameObject go = policePrefab != null
                ? Instantiate(policePrefab)
                : GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = "V2X Emergency Vehicle";
            go.transform.SetPositionAndRotation(
                ego.transform.position - ego.transform.forward * emergencySpawnDistance,
                ego.transform.rotation);
            if (policePrefab == null)
            {
                go.transform.localScale = new Vector3(1.9f, 1.2f, 4.4f);
                go.GetComponent<Renderer>().material.color = Color.white;
            }
            var agent = go.GetComponent<DynamicObjectAgent>() ??
                        go.AddComponent<DynamicObjectAgent>();
            agent.objectId = "ambulance_demo";
            agent.objectType = "emergency_vehicle";
            agent.radius = 1.2f;
            var mover = go.GetComponent<EmergencyVehicleMover>() ??
                        go.AddComponent<EmergencyVehicleMover>();
            mover.speed = 31f;
            CreateSiren(go.transform, mover);
            _emergency = agent;
            simulation?.RegisterObject(agent);
            LastEvent = "후방 긴급차 접근 · 우측 갓길 대피";
        }

        public void TogglePlanner()
        {
            if (simulation == null) return;
            simulation.plannerMode = simulation.plannerMode == "rrt" ? "rrt_star" : "rrt";
            LastEvent = $"플래너 전환: {simulation.plannerMode.ToUpperInvariant()}";
        }

        public void ResetScenario() =>
            SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);

        private static void CreateWarningBeacon(Transform parent)
        {
            var lightGo = new GameObject("Cargo Warning Light");
            lightGo.transform.SetParent(parent, false);
            lightGo.transform.localPosition = new Vector3(0f, .8f, 0f);
            var light = lightGo.AddComponent<Light>();
            light.type = LightType.Point;
            light.color = new Color(1f, .2f, .02f);
            light.range = 8f;
            light.intensity = 5f;
        }

        private static void CreateSiren(Transform parent, EmergencyVehicleMover mover)
        {
            var left = GameObject.CreatePrimitive(PrimitiveType.Cube);
            left.name = "Red Siren";
            left.transform.SetParent(parent, false);
            left.transform.localPosition = new Vector3(-.42f, .85f, 0f);
            left.transform.localScale = new Vector3(.5f, .18f, .35f);
            var right = GameObject.CreatePrimitive(PrimitiveType.Cube);
            right.name = "Blue Siren";
            right.transform.SetParent(parent, false);
            right.transform.localPosition = new Vector3(.42f, .85f, 0f);
            right.transform.localScale = new Vector3(.5f, .18f, .35f);
            mover.leftBeacon = left.GetComponent<Renderer>();
            mover.rightBeacon = right.GetComponent<Renderer>();
        }
    }
}
