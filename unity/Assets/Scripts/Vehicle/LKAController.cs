using System.Collections.Generic;
using UnityEngine;

// Lateral controllers behind one interface so Pure Pursuit / Stanley / PID are
// interchangeable in experiments (plan §10.2). Each returns a front-wheel
// steering angle (radians). Pure C# (no MonoBehaviour) so it is unit-testable
// and driven by VehicleController. Gain tuning is a human task (plan §4 risk).

namespace V2X.Vehicle
{
    public enum LateralLaw { PurePursuit, Stanley }

    public class LKAController
    {
        public LateralLaw law = LateralLaw.PurePursuit;

        [Header("Pure Pursuit")]
        public float lookaheadBase = 4.0f;    // m at standstill
        public float lookaheadK = 0.4f;       // + this * speed (m per m/s)

        [Header("Stanley")]
        public float stanleyGain = 1.5f;
        public float stanleySoftening = 1.0f; // avoids blow-up near v=0

        public float maxSteer = 0.6f;         // rad (~34 deg)

        // Latest diagnostics, for logging (plan §21.1).
        public float LastLateralError { get; private set; }
        public float LastHeadingError { get; private set; }

        // Compute steering (rad) toward the path. `heading` is yaw in degrees
        // (Unity convention, 0=+Z, CW). `wheelBase` and `speed` in SI.
        public float Steer(Vector3 pos, float headingDeg, float speed,
                           IReadOnlyList<Vector3> path, float wheelBase)
        {
            if (path == null || path.Count < 2) return 0f;

            // nearest point + path heading there, for the error diagnostics
            int nearIdx = NearestIndex(pos, path);
            Vector3 segDir = SegmentDir(path, nearIdx);
            float pathHeading = Mathf.Atan2(segDir.x, segDir.z) * Mathf.Rad2Deg;
            LastHeadingError = Mathf.DeltaAngle(headingDeg, pathHeading);
            LastLateralError = SignedLateral(pos, path[nearIdx], segDir);

            return law == LateralLaw.Stanley
                ? StanleySteer(speed, wheelBase)
                : PurePursuitSteer(pos, headingDeg, speed, path, wheelBase);
        }

        private float PurePursuitSteer(Vector3 pos, float headingDeg, float speed,
                                       IReadOnlyList<Vector3> path, float wheelBase)
        {
            float ld = lookaheadBase + lookaheadK * speed;
            Vector3 target = LookaheadPoint(pos, path, ld);
            Vector3 to = target - pos; to.y = 0f;
            float headingRad = headingDeg * Mathf.Deg2Rad;
            // angle of target relative to vehicle heading
            float targetAngle = Mathf.Atan2(to.x, to.z);
            float alpha = Mathf.DeltaAngle(headingDeg, targetAngle * Mathf.Rad2Deg) * Mathf.Deg2Rad;
            float delta = Mathf.Atan2(2f * wheelBase * Mathf.Sin(alpha), Mathf.Max(ld, 0.1f));
            return Mathf.Clamp(delta, -maxSteer, maxSteer);
        }

        private float StanleySteer(float speed, float wheelBase)
        {
            float headingErrRad = LastHeadingError * Mathf.Deg2Rad;
            float crossTrack = Mathf.Atan2(stanleyGain * (-LastLateralError),
                                           stanleySoftening + speed);
            return Mathf.Clamp(headingErrRad + crossTrack, -maxSteer, maxSteer);
        }

        // ---- helpers ---------------------------------------------------- //
        private static int NearestIndex(Vector3 pos, IReadOnlyList<Vector3> path)
        {
            int best = 0; float bestD = float.MaxValue;
            for (int i = 0; i < path.Count; i++)
            {
                Vector3 d = path[i] - pos; d.y = 0f;
                float sq = d.sqrMagnitude;
                if (sq < bestD) { bestD = sq; best = i; }
            }
            return best;
        }

        private static Vector3 SegmentDir(IReadOnlyList<Vector3> path, int idx)
        {
            int a = Mathf.Min(idx, path.Count - 2);
            Vector3 d = path[a + 1] - path[a]; d.y = 0f;
            return d.sqrMagnitude > 1e-6f ? d.normalized : Vector3.forward;
        }

        private static float SignedLateral(Vector3 pos, Vector3 onPath, Vector3 dir)
        {
            Vector3 off = pos - onPath; off.y = 0f;
            // left/right sign via cross product y component
            return Vector3.Cross(dir, off).y;
        }

        private static Vector3 LookaheadPoint(Vector3 pos, IReadOnlyList<Vector3> path, float ld)
        {
            int near = NearestIndex(pos, path);
            for (int i = near; i < path.Count; i++)
            {
                Vector3 d = path[i] - pos; d.y = 0f;
                if (d.magnitude >= ld) return path[i];
            }
            return path[path.Count - 1];
        }
    }
}
