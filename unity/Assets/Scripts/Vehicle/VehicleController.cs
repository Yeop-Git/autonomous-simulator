using System.Collections.Generic;
using UnityEngine;
using V2X.Protocol;
using V2X.Sim;

// Kinematic bicycle-model vehicle. Holds identity + goal, applies the latest
// CommandMessage from the server (target speed, path, behaviour, LKA flag),
// and integrates motion each FixedUpdate. Longitudinal: first-order tracking
// of target_speed under accel/decel limits. Lateral: LKAController steers
// toward the commanded path. Visual-only physics (plan §16.3) — Unity owns
// motion, the server owns decisions.

namespace V2X.Vehicle
{
    [DisallowMultipleComponent]
    public class VehicleController : MonoBehaviour
    {
        [Header("Identity")]
        public string vehicleId;
        [Tooltip("Optional destination. If set, has_goal=true in the state message.")]
        public Transform goal;
        public string type = "car";
        [Tooltip("straight | left | right")]
        public string maneuver = "straight";

        [Header("Kinematics")]
        public float maxSpeed = 30f;        // m/s ceiling
        public float maxAccel = 3f;         // m/s^2
        public float maxDecel = 6f;         // m/s^2
        public float wheelBase = 2.7f;      // m

        [Header("Lateral control")]
        public LateralLaw lateralLaw = LateralLaw.PurePursuit;
        public float lookaheadBase = 4f;
        public float lookaheadK = 0.4f;
        public bool destroyOnArrival = true;
        public float arrivalDestroyDelay = 1f;

        // runtime state
        public float CurrentSpeed { get; private set; }
        public string CurrentLaneId { get; set; }
        public string Behavior { get; private set; } = "LaneKeeping";
        public bool LkaEnabled { get; private set; } = true;
        public float LateralError => _lka.LastLateralError;
        public float HeadingError => _lka.LastHeadingError;
        public string RequestedTargetLane { get; private set; }

        private readonly List<Vector3> _path = new();
        private float _targetSpeed;
        private bool _hasCommand;
        private readonly LKAController _lka = new();
        private float _arrivedTime;

        public string Id => string.IsNullOrEmpty(vehicleId) ? name : vehicleId;
        public bool HasGoal => goal != null;
        public Vector3 GoalPosition => goal != null ? goal.position : transform.position;
        public Vector3 Velocity => transform.forward * CurrentSpeed;
        public float HeadingDeg => transform.eulerAngles.y;
        public IReadOnlyList<Vector3> Path => _path;

        public void RequestTargetLane(string laneId)
        {
            if (!string.IsNullOrEmpty(laneId) && laneId != CurrentLaneId)
                RequestedTargetLane = laneId;
        }

        public void ClearTargetLaneRequest() => RequestedTargetLane = null;

        // Apply one server command (called by SimulationManager / ICommandSink).
        public void ApplyCommand(VehicleCommand cmd)
        {
            _hasCommand = true;
            _targetSpeed = Mathf.Min(cmd.target_speed, maxSpeed);
            Behavior = string.IsNullOrEmpty(cmd.behavior) ? Behavior : cmd.behavior;
            LkaEnabled = cmd.lka_enabled;
            if (!string.IsNullOrEmpty(RequestedTargetLane)
                && RequestedTargetLane == CurrentLaneId)
                RequestedTargetLane = null;
            if (cmd.path != null && cmd.path.Count > 0)
            {
                _path.Clear();
                foreach (var p in cmd.path)
                    if (p != null && p.Length >= 3) _path.Add(new Vector3(p[0], p[1], p[2]));
            }
            _lka.law = lateralLaw;
            _lka.lookaheadBase = lookaheadBase;
            _lka.lookaheadK = lookaheadK;
        }

        private void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;
            if (!_hasCommand) return;

            if (destroyOnArrival && Behavior == "Arrived" && CurrentSpeed < .05f)
            {
                _arrivedTime += Time.fixedDeltaTime;
                if (_arrivedTime >= arrivalDestroyDelay)
                {
                    FindFirstObjectByType<SimulationManager>()?.UnregisterVehicle(this);
                    if (goal != null) Destroy(goal.gameObject);
                    Destroy(gameObject);
                    return;
                }
            }
            else _arrivedTime = 0f;

            // --- longitudinal: track target speed under accel/decel limits ---
            float dv = _targetSpeed - CurrentSpeed;
            float maxDv = (dv >= 0 ? maxAccel : maxDecel) * dt;
            CurrentSpeed = Mathf.Clamp(CurrentSpeed + Mathf.Clamp(dv, -maxDv, maxDv),
                                       0f, maxSpeed);

            if (CurrentSpeed < 1e-3f && _targetSpeed <= 0f) return;

            // --- lateral: steer toward the commanded path -------------------
            float steer = 0f;
            if (LkaEnabled && _path.Count >= 2)
                steer = _lka.Steer(transform.position, HeadingDeg, CurrentSpeed,
                                   _path, wheelBase);

            // --- integrate kinematic bicycle model --------------------------
            if (Mathf.Abs(steer) > 1e-5f && CurrentSpeed > 1e-3f)
            {
                float yawRate = CurrentSpeed / wheelBase * Mathf.Tan(steer); // rad/s
                transform.Rotate(0f, yawRate * Mathf.Rad2Deg * dt, 0f);
            }
            transform.position += transform.forward * (CurrentSpeed * dt);
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.green;
            for (int i = 0; i < _path.Count - 1; i++)
                Gizmos.DrawLine(_path[i], _path[i + 1]);
            if (goal != null)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawWireSphere(goal.position, 1.5f);
            }
        }
    }
}
