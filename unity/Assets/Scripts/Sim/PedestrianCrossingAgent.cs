using UnityEngine;
using V2X.UI;
using V2X.Visualization;

namespace V2X.Sim
{
    /// <summary>One-shot pedestrian: cross once, unregister, then destroy.</summary>
    [RequireComponent(typeof(DynamicObjectAgent))]
    public class PedestrianCrossingAgent : MonoBehaviour
    {
        private static readonly int WalkParameter = Animator.StringToHash("walk");

        public Vector3 destination;
        public float speed = 1.9f;
        public float postCrossingWait = 2f;
        public float despawnFadeDuration = .6f;
        public TrafficLightSystem signals;
        public PedestrianSpawner owner;

        public bool HasStartedCrossing => _startedCrossing;
        public bool HasFinishedCrossing => _finishedCrossing;

        private DynamicObjectAgent _agent;
        private SimulationManager _simulation;
        private Animator[] _animators;
        private bool _startedCrossing;
        private bool _finishedCrossing;
        private bool _ownerNotified;
        private bool _removing;
        private float _finishedTime;

        private void Start()
        {
            _agent = GetComponent<DynamicObjectAgent>();
            _simulation = FindFirstObjectByType<SimulationManager>();
            _simulation?.RegisterObject(_agent);
            _animators = GetComponentsInChildren<Animator>(true);
            SetWalkingAnimation(false);
        }

        private void Update()
        {
            if (!_startedCrossing)
            {
                if (signals != null && !signals.PedestriansMayCross) return;
                _startedCrossing = true;
                SetWalkingAnimation(true);
            }

            if (_finishedCrossing)
            {
                _finishedTime += Time.deltaTime;
                if (_finishedTime >= postCrossingWait) Remove();
                return;
            }

            transform.position = Vector3.MoveTowards(
                transform.position, destination, speed * Time.deltaTime);
            if ((transform.position - destination).sqrMagnitude > .01f) return;
            _finishedCrossing = true;
            SetWalkingAnimation(false);
        }

        private void SetWalkingAnimation(bool walking)
        {
            if (_animators == null) return;
            foreach (var animator in _animators)
            {
                if (animator == null || !HasWalkParameter(animator)) continue;
                animator.speed = 1f;
                animator.SetBool(WalkParameter, walking);
            }
        }

        private static bool HasWalkParameter(Animator animator)
        {
            foreach (var parameter in animator.parameters)
                if (parameter.type == AnimatorControllerParameterType.Bool &&
                    parameter.nameHash == WalkParameter)
                    return true;
            return false;
        }

        private void Remove()
        {
            if (_removing) return;
            _removing = true;
            _simulation?.UnregisterObject(_agent);
            DespawnFader.FadeAndDestroy(gameObject, despawnFadeDuration);
        }

        private void OnDestroy()
        {
            if (_ownerNotified || owner == null) return;
            _ownerNotified = true;
            owner.NotifyPedestrianRemoved();
        }
    }
}
