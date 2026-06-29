using System.Collections.Generic;
using UnityEngine;
using V2X.Communication;
using V2X.Protocol;
using V2X.Road;
using V2X.Vehicle;

// Bridges the scene and the V2XClient. As IWorldStateProvider it gathers every
// vehicle + dynamic object into a StateMessage each tick (tagging each
// vehicle's current_lane via the RoadNetworkManager). As ICommandSink it
// dispatches the server's per-vehicle commands back to the controllers.
//
// Assign this same component to BOTH the V2XClient's stateProviderSource and
// commandSinkSource fields.

namespace V2X.Sim
{
    public class SimulationManager : MonoBehaviour, IWorldStateProvider, ICommandSink
    {
        [Tooltip("highway | urban | lka_test")]
        public string scenario = "highway";
        public RoadNetworkManager road;

        [Tooltip("Leave empty to auto-collect from the scene at Start.")]
        public List<VehicleController> vehicles = new();
        public List<DynamicObjectAgent> objects = new();

        private readonly Dictionary<string, VehicleController> _byId = new();

        private void Start()
        {
            if (road == null) road = FindFirstObjectByType<RoadNetworkManager>();
            if (vehicles == null || vehicles.Count == 0)
                vehicles = new List<VehicleController>(
                    FindObjectsByType<VehicleController>(FindObjectsSortMode.None));
            if (objects == null || objects.Count == 0)
                objects = new List<DynamicObjectAgent>(
                    FindObjectsByType<DynamicObjectAgent>(FindObjectsSortMode.None));
            _byId.Clear();
            foreach (var v in vehicles)
                if (v != null) _byId[v.Id] = v;
        }

        // ---- IWorldStateProvider ---------------------------------------- //
        public StateMessage CollectState(int tick, float time)
        {
            var msg = new StateMessage { time = time, tick = tick, scenario = scenario };

            foreach (var v in vehicles)
            {
                if (v == null) continue;
                if (road != null)
                {
                    string lane = road.NearestLaneId(v.transform.position);
                    if (lane != null) v.CurrentLaneId = lane;
                }
                Vector3 pos = v.transform.position;
                Vector3 vel = v.Velocity;
                Vector3 g = v.GoalPosition;
                msg.vehicles.Add(new VehicleState
                {
                    id = v.Id,
                    type = v.type,
                    position = new[] { pos.x, pos.y, pos.z },
                    velocity = new[] { vel.x, vel.y, vel.z },
                    acceleration = new[] { 0f, 0f, 0f },
                    heading = v.HeadingDeg,
                    current_lane = v.CurrentLaneId,
                    target_lane = null,
                    has_goal = v.HasGoal,
                    goal = new[] { g.x, g.y, g.z },
                    behavior_state = v.Behavior,
                });
            }

            foreach (var o in objects)
            {
                if (o == null) continue;
                Vector3 pos = o.transform.position;
                Vector3 vel = o.Velocity;
                msg.objects.Add(new MovingObject
                {
                    id = o.Id,
                    type = o.objectType,
                    position = new[] { pos.x, pos.y, pos.z },
                    velocity = new[] { vel.x, vel.y, vel.z },
                    radius = o.radius,
                });
            }
            return msg;
        }

        // ---- ICommandSink ----------------------------------------------- //
        public void Apply(CommandMessage command)
        {
            if (command?.commands == null) return;
            foreach (var cmd in command.commands)
                if (cmd != null && _byId.TryGetValue(cmd.vehicle_id, out var v))
                    v.ApplyCommand(cmd);
        }
    }
}
