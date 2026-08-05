using System.Collections.Generic;
using UnityEngine;
using V2X.Vehicle;

namespace V2X.Sim
{
    /// <summary>
    /// Drives the integrated city showcase for ten minutes. The ego repeatedly
    /// traverses the signalized urban core and the outer V2X boulevard while
    /// deterministic obstacle and emergency-vehicle events exercise both RRT
    /// variants without requiring operator input.
    /// </summary>
    public class IntegratedCityScenarioDirector : MonoBehaviour
    {
        public SimulationManager simulation;
        public VehicleController ego;
        public EmergencyAvoidanceScenarioController events;
        public Transform northCheckpoint;
        public Transform southCheckpoint;
        public float showcaseDuration = 600f;
        public float checkpointRadius = 16f;
        [Range(1f, 8f)] public float playbackSpeed = 1f;

        public int Lap { get; private set; }
        public string Phase { get; private set; } = "Urban core departure";
        public float Runtime => Time.timeSinceLevelLoad;
        public float RemainingTime => Mathf.Max(0f, showcaseDuration - Runtime);
        public bool Completed => Runtime >= showcaseDuration;

        private bool _headingSouth;
        private readonly HashSet<int> _triggeredLaps = new();

        private void Start()
        {
            simulation ??= FindFirstObjectByType<SimulationManager>();
            ego ??= GameObject.Find("urban_ego")?.GetComponent<VehicleController>();
            events ??= FindFirstObjectByType<EmergencyAvoidanceScenarioController>();
            if (simulation != null)
            {
                simulation.scenario = "integrated_city";
                simulation.plannerMode = "rrt";
            }
            SetGoal(northCheckpoint, "Urban core to V2X boulevard");
        }

        private void Update()
        {
            Time.timeScale = playbackSpeed;
            if (ego == null) return;

            if (_headingSouth && southCheckpoint != null &&
                Vector3.Distance(ego.transform.position, southCheckpoint.position) <= checkpointRadius)
            {
                _headingSouth = false;
                SetGoal(northCheckpoint, "Return through signalized urban core");
            }
            else if (!_headingSouth && northCheckpoint != null &&
                     Vector3.Distance(ego.transform.position, northCheckpoint.position) <= checkpointRadius)
            {
                _headingSouth = true;
                SetGoal(southCheckpoint, "Outer boulevard southbound leg");
            }

            if (ego.CurrentLaneId == "city_boulevard_main")
            {
                Phase = "V2X boulevard dynamic-event zone";
                int eventLap = Lap;
                if (ego.transform.position.z > 155f && !_triggeredLaps.Contains(eventLap))
                {
                    _triggeredLaps.Add(eventLap);
                    TriggerEvent(eventLap);
                }
            }

            if (Completed)
                Phase = "10-minute showcase complete - safe circulation continues";
        }

        private void OnDisable() => Time.timeScale = 1f;

        private void SetGoal(Transform checkpoint, string phase)
        {
            if (ego == null || checkpoint == null) return;
            ego.goal.position = checkpoint.position;
            Phase = phase;
            if (!_headingSouth && checkpoint == northCheckpoint && Runtime > 2f)
                Lap++;
        }

        private void TriggerEvent(int lap)
        {
            if (events == null || simulation == null) return;
            switch (lap % 4)
            {
                case 0:
                    simulation.plannerMode = "rrt";
                    events.SpawnObstacle();
                    Phase = "Unexpected cargo - RRT lateral avoidance";
                    break;
                case 1:
                    events.DispatchEmergencyVehicle();
                    Phase = "Emergency vehicle - active right-side pull-over";
                    break;
                case 2:
                    simulation.plannerMode = "rrt_star";
                    events.SpawnObstacle();
                    Phase = "Unexpected cargo - RRT* optimized avoidance";
                    break;
                default:
                    simulation.plannerMode = "rrt_star";
                    events.DispatchEmergencyVehicle();
                    Phase = "Emergency vehicle - RRT* pull-over and rejoin";
                    break;
            }
        }
    }
}
