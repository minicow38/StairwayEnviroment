using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Sirenix.OdinInspector;
/// <summary>
/// 3500行版から物理核だけを抜き出した小型版。
/// Ground/Slope観測、SlopeFrame、Target Progress、独立Jerk、
/// Critical Stick、Entry Bridge、Natural Release、Rigidbody所有権を保持します。
/// Baseline JSON、DOTween、詳細統計、Ray履歴、旧Fallback群は含みません。
/// </summary>
[Searchable]
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody), typeof(SphereCollider))]
public sealed class SlopeStick3DCompact1 : MonoBehaviour
{
    const string ImplementationVersion = "CompactInvariantController-v2.1-MathUnified-2026-07-29";
    const float Epsilon = 0.000001f;
    static readonly Dictionary<int, SlopeStick3DCompact1> Owners = new Dictionary<int, SlopeStick3DCompact1>();
    enum GroundKind
    {
        Air, Flat, Slope, SlopeGrace
    }
    enum ObservationSource
    {
        None, CollisionContact, SphereCast, SlopeEntryBridge, SurfaceObserverGrace
    }
    enum TargetPhase
    {
        Inactive, Observing, Controlling, Completed
    }
    enum ReleasePhase
    {
        Controlled, Releasing, Released
    }
    enum EntryPhase
    {
        Inactive, FlatApproach, SlopeEntrySettling, NormalSlope
    }
    [Serializable]
    struct GroundObservation
    {
        public bool valid;
        public Collider collider;
        public Vector3 point;
        public Vector3 normal;
        public ObservationSource source;
    }
    [Serializable]
    struct SlopeFrame
    {
        public bool valid;
        public Collider collider;
        public Vector3 normal;
        public Vector3 axis;
        public Vector3 entryPoint;
        public Vector3 exitPoint;
        public float length;
        public float entryCurvature;
        public float exitCurvature;
        public float representativeCurvature;
    }
    struct SurfaceBasis
    {
        public bool valid;
        public Vector3 tangent;
        public Vector3 side;
        public Vector3 normal;
        public static SurfaceBasis Create( Vector3 normalInput, Vector3 preferredTangent, Vector3 fallbackTangent) {
            SurfaceBasis basis = default;
            Vector3 normal = normalInput.sqrMagnitude > Epsilon ? normalInput.normalized : Vector3.up;
            Vector3 tangent = Vector3.ProjectOnPlane( preferredTangent, normal);
            if (tangent.sqrMagnitude <= Epsilon)
                tangent = Vector3.ProjectOnPlane(fallbackTangent, normal);
            if (tangent.sqrMagnitude <= Epsilon)
                tangent = Vector3.Cross(normal, Vector3.right);
            if (tangent.sqrMagnitude <= Epsilon)
                tangent = Vector3.Cross(normal, Vector3.forward);
            if (tangent.sqrMagnitude <= Epsilon)
                return basis;
            tangent.Normalize();
            Vector3 side = Vector3.Cross(normal, tangent);
            if (side.sqrMagnitude <= Epsilon)
                return basis;
            side.Normalize();
            tangent = Vector3.Cross(side, normal).normalized;
            basis.valid = true;
            basis.tangent = tangent;
            basis.side = side;
            basis.normal = normal;
            return basis;
        }
    }
    struct SurfaceMotion
    {
        public float tangentSpeed, lateralSpeed, outwardSpeed, gravityAlong, gravitySupport;
        public static SurfaceMotion Evaluate(SurfaceBasis basis, Vector3 velocity) {
            return new SurfaceMotion
            {
                tangentSpeed = Vector3.Dot(velocity, basis.tangent), lateralSpeed = Vector3.Dot(velocity, basis.side),
                outwardSpeed = Mathf.Max(0f, Vector3.Dot(velocity, basis.normal)),
                gravityAlong = Vector3.Dot(Physics.gravity, basis.tangent),
                gravitySupport = Mathf.Max(0f, Vector3.Dot(Physics.gravity, -basis.normal))
            };
        }
    }
    // ---------------------------------------------------------------------
    // 基本移動
    // ---------------------------------------------------------------------
    [Header("Motion")]
    [SerializeField] bool autoProgress = true;
    [SerializeField] bool readLegacyKeyboardInput = true;
    [SerializeField] Vector3 initialHeading = Vector3.forward;
    [Range(0f, 1f)] [SerializeField] float steeringStrength = 0.75f;
    [Min(0f)] [SerializeField] float maxGroundSpeed = 16f;
    [Min(0f)] [SerializeField] float groundAcceleration = 35f;
    [Min(0.01f)] [SerializeField] float groundResponseSeconds = 0.12f;
    [Min(0f)] [SerializeField] float airAcceleration = 6f;
    [Min(0.1f)] [SerializeField] float maximumTangentialDeceleration = 80f;
    [Min(0.1f)] [SerializeField] float tangentialJerkLimit = 600f;
    [Min(0.01f)] [SerializeField] float lateralResponseSeconds = 0.12f;

    [Header("Hybrid Surface Observer")]
    [SerializeField] bool useDistanceNormalizedSurfaceObserver = true;
    [Min(0.05f)] [SerializeField] float surfaceSwitchConfirmDistance = 0.40f;
    [Min(0.05f)] [SerializeField] float supportGraceDistance = 0.95f;
    [Min(0.10f)] [SerializeField] float maximumSurfaceObserverScore = 2.25f;
    // ---------------------------------------------------------------------
    // 支持面観測
    // ---------------------------------------------------------------------
    public Vector3 restart;
    [Header("Ground Detection")]
    [Min(0.05f)] [SerializeField] float sphereRadius = 0.5f;
    [Min(0.01f)] [SerializeField] float groundProbeDistance = 0.45f;
    [SerializeField] LayerMask groundMask = ~0;
    [Range(0f, 20f)] [SerializeField] float minimumSlopeAngle = 2f;
    [Range(1f, 89f)] [SerializeField] float maximumSlopeAngle = 75f;
    [SerializeField] bool useCollisionContacts = false;
    [Range(0, 2)] [SerializeField] int collisionContactMemorySteps = 1;
    [Header("Slope Entry Bridge")]
    [SerializeField] bool useSlopeEntryBridge = true;
    [Range(1, 3)] [SerializeField] int bridgeMaximumMissFrames = 1;
    [Range(0.05f, 0.40f)] [SerializeField] float bridgeEndProgress = 0.25f;
    [Min(0.01f)] [SerializeField] float bridgeMaximumGap = 0.18f;
    [Min(0f)] [SerializeField] float bridgeMaximumOutwardSpeed = 1.25f;
    [Min(0f)] [SerializeField] float bridgeRecoveryAcceleration = 180f;
    // ---------------------------------------------------------------------
    // 前方斜面と SlopeFrame
    // ---------------------------------------------------------------------
    [Header("Forward Slope Detection")]
    [Min(0.5f)] [SerializeField] float forwardSlopeProbeDistance = 8f;
    [Range(4, 32)] [SerializeField] int forwardSlopeProbeSegments = 32;
    [Min(0.1f)] [SerializeField] float forwardProbeHeight = 2.5f;
    [Min(0.1f)] [SerializeField] float forwardProbeDownDistance = 7f;
    [Header("Entry Velocity Transport")]
    [SerializeField] bool useEntryVelocityTransport = true;
    [Range(0f, 1f)] [SerializeField] float entryVelocityTransportWeight = 1f;
    [Header("Slope Geometry")]
    [Min(0.05f)] [SerializeField] float connectedSurfaceProbeOffset = 0.5f;
    [Min(0.05f)] [SerializeField] float connectedSurfaceMaximumGap = 2f;
    [Min(0.05f)] [SerializeField] float boundaryCurvatureDistance = 0.75f;
    [Min(0.001f)] [SerializeField] float maximumRepresentativeCurvature = 1f;
    [Min(0.000001f)] [SerializeField] float minimumCurvature = 0.0005f;
    // ---------------------------------------------------------------------
    // Target Progress
    // ---------------------------------------------------------------------
    [Header("Target Progress")]
    [SerializeField] bool useTargetProgressControl = true;
    [Range(0f, 100f)] [SerializeField] float targetSlopeProgressPercent = 60f;
    [Range(1, 8)] [SerializeField] int requiredStableSlopeFrames = 2;
    [Min(0.01f)] [SerializeField] float targetMinimumDistance = 0.27f;
    [Min(0f)] [SerializeField] float targetMaximumArtificialAcceleration = 120f;
    [Min(0.1f)] [SerializeField] float targetMaximumArtificialDeceleration = 120f;
    [Min(0.1f)] [SerializeField] float targetJerkLimit = 600f;
    [SerializeField] bool compensateSlopeGravity = true;

    [Header("Post Target Gravity Feedforward")]
    [SerializeField] bool usePostTargetGravityCompensation = true;
    [Range(0f, 1f)] [SerializeField] float postTargetGravityCompensationRatio = 0.50f;
    [Range(0.1f, 20f)] [SerializeField] float postTargetGravityBlendWidthPercent = 5f;
    // ---------------------------------------------------------------------
    // Critical Stick
    // ---------------------------------------------------------------------
    [Header("Critical Adhesion")]
    [SerializeField] bool useAdaptiveCriticalStick = true;
    [Range(0.80f, 0.999f)] [SerializeField] float targetCriticalRatio = 0.98f;
    [Min(0f)] [SerializeField] float flatStickAcceleration = 24.6f;
    [Min(1f)] [SerializeField] float maximumAdaptiveStickAcceleration = 1000f;
    [Range(0f, 0.25f)] [SerializeField] float stickPredictionSeconds = 0.06f;
    [Min(1f)] [SerializeField] float stickRiseJerkLimit = 5000f;
    [Min(1f)] [SerializeField] float stickFallJerkLimit = 1200f;
    [Min(0.01f)] [SerializeField] float outwardNormalResponseSeconds = 0.08f;
    [Range(1f, 1.5f)] [SerializeField] float stickSafetyMargin = 1.08f;
    // ---------------------------------------------------------------------
    // Natural Release
    // ---------------------------------------------------------------------
    [Header("Natural Release")]
    [SerializeField] bool useNaturalRelease = true;
    [Range(0f, 20f)] [SerializeField] float releaseHoldAfterTargetPercent = 2f;
    [Range(1f, 100f)] [SerializeField] float naturalReleaseProgressPercent = 90f;
    [Range(0.1f, 30f)] [SerializeField] float minimumReleaseWidthPercent = 8f;
    // ---------------------------------------------------------------------
    // 排他的所有権とログ
    // ---------------------------------------------------------------------
    [Header("Ownership / Logging")]
    [SerializeField] bool disableLegacyControllersOnSameBody = true;
    [SerializeField] bool logEvents = true;
    [SerializeField] bool logEntryFrameDebug = true;
    [SerializeField] bool logPeriodicState = false;
    [Range(1, 120)] [SerializeField] int periodicLogEveryFixedFrames = 20;
    // ---------------------------------------------------------------------
    // Runtime 表示
    // ---------------------------------------------------------------------
    [Header("Runtime")]
    [SerializeField] GroundKind groundKind;
    [SerializeField] ObservationSource observationSource;
    [SerializeField] string supportColliderName;
    [SerializeField] string activeSlopeName;
    [SerializeField] float slopeProgressPercent;
    [SerializeField] float tangentSpeed;
    [SerializeField] float targetTangentSpeed;
    [SerializeField] float currentTangentialAcceleration;
    [SerializeField] float currentStickAcceleration;
    [SerializeField] float currentCriticalRatio;
    [SerializeField] float currentCurvature;
    [SerializeField] float currentReleaseWeight = 1f;
    [SerializeField] TargetPhase targetPhase;
    [SerializeField] ReleasePhase releasePhase;
    [SerializeField] int consecutiveGroundMissFrames;
    [SerializeField] int stableSlopeFrames;
    [SerializeField] float stableSlopeDistance;
    [SerializeField] bool bridgeActive;
    [SerializeField] bool surfaceGraceActive;
    [SerializeField] float supportMissDistance;
    [SerializeField] string surfaceCandidateName;
    [SerializeField] float surfaceCandidateDistance;
    [SerializeField] float surfaceCandidateScore;
    [SerializeField] int surfaceCandidateSamples;
    [SerializeField] float forwardFrameMissDistance;
    [SerializeField] float targetCrossingEstimatedTime;
    [SerializeField] float targetCrossingSampleOvershootPercent;
    [SerializeField] float postTargetGravityCompensationWeight;
    [SerializeField] float postTargetGravityCompensationAcceleration;
    [SerializeField] EntryPhase entryPhase;
    [SerializeField] float entryDistance;
    [SerializeField] float entryWeight;
    [SerializeField] float entryPreloadAcceleration;
    [SerializeField] bool ownsRigidbody;
    Rigidbody rb;
    SphereCollider sphereCollider;
    Vector3 heading;
    Vector2 moveInput;
    GroundObservation currentObservation;
    GroundObservation recentCollisionObservation;
    GroundObservation lastMeasuredSlopeObservation;
    float recentCollisionFixedTime = float.NegativeInfinity;
    float previousProgress01;
    float previousTargetAcceleration;
    float previousGroundAcceleration;
    float previousStickAcceleration;
    float nextPeriodicLogFrame;
    bool hadProgressObservation;
    bool targetPlanValid;
    bool stickSaturationLogged;
    bool naturalMotionReleased;
    SlopeFrame activeFrame;
    SlopeFrame forwardFrame;
    int forwardFrameMissFrames;
    SlopeFrame pendingSurfaceFrame;
    GroundObservation pendingSurfaceObservation;
    float capturedTargetProgress01;
    float capturedTargetSpeed;
    float releaseStartProgress01;
    float releaseEndProgress01;
    // ---------------------------------------------------------------------
    // Unity lifecycle
    // ---------------------------------------------------------------------
    void Reset() {
        sphereCollider = GetComponent<SphereCollider>();
        if (sphereCollider != null)
            sphereRadius = sphereCollider.radius;
    }
    void Awake() {
       
    }
    void OnEnable() {
        if (rb != null && !ownsRigidbody)
            AcquireOwnership();
    }
    void OnDisable() {
        ReleaseOwnership();
    }
    void OnDestroy() {
        ReleaseOwnership();
    }
    void Update() {
#if ENABLE_LEGACY_INPUT_MANAGER
        if (readLegacyKeyboardInput) {
            float horizontal = Input.GetAxisRaw("Horizontal");
            float vertical = Input.GetAxisRaw("Vertical");
            moveInput = Vector2.ClampMagnitude( new Vector2(horizontal, vertical), 1f);
        }
#endif
    }
    void FixedUpdate() {
        if (!ownsRigidbody || rb == null)
            return;

        float dt = Time.fixedDeltaTime;
        Vector3 desiredMove = GetDesiredMoveWorld();
        EntryPhase phaseBeforeObservation = entryPhase;
        bridgeActive = false;

        bool measuredGrounded = ObserveGround(out GroundObservation observation);
        bool measuredFlat = measuredGrounded && Vector3.Angle(observation.normal, Vector3.up) < minimumSlopeAngle;
        bool mayBeEntryClassificationMiss = measuredFlat && phaseBeforeObservation == EntryPhase.SlopeEntrySettling &&
            activeFrame.valid;

        if ((!measuredGrounded || mayBeEntryClassificationMiss) &&
            TryBuildSlopeEntryBridge(dt, phaseBeforeObservation, out GroundObservation bridgedObservation)) {
            observation = bridgedObservation;
            measuredGrounded = true;
        }

        if (!measuredGrounded) {
            supportMissDistance += CalculateObserverDistanceIncrement(dt);

            if (TryHoldSlopeStateAfterSupportLoss(dt, phaseBeforeObservation)) {
                LogPeriodic();
                return;
            }

            EndSurfaceGrace("support_limit");
            currentObservation = default;
            observationSource = ObservationSource.None;
            supportColliderName = string.Empty;
            entryPhase = EntryPhase.Inactive;
            SolveAir(desiredMove);
            ClearGroundState();
            LogPeriodic();
            return;
        }

        if (surfaceGraceActive)
            EndSurfaceGrace("support_recovered");

        supportMissDistance = 0f;
        consecutiveGroundMissFrames = 0;
        currentObservation = observation;
        observationSource = observation.source;
        supportColliderName = observation.collider != null ? observation.collider.name : string.Empty;

        float angle = Vector3.Angle(observation.normal, Vector3.up);
        bool isSlope = angle >= minimumSlopeAngle && angle <= maximumSlopeAngle;

        if (isSlope) {
            groundKind = GroundKind.Slope;
            UpdateActiveSlopeFrame(observation, desiredMove, dt);
            UpdateEntryPhase();
            SolveSlope(desiredMove, dt);
        }
        else {
            groundKind = GroundKind.Flat;
            bool keepEntryFrame = ShouldKeepActiveEntryFrame(phaseBeforeObservation);

            if (!keepEntryFrame) {
                activeSlopeName = string.Empty;
                stableSlopeFrames = 0;
                stableSlopeDistance = 0f;
                ClearSurfaceCandidate();
                ResetSlopeSession(false);
                if (activeFrame.valid && activeFrame.collider != observation.collider)
                    activeFrame = default;
            }

            UpdateForwardSlope(desiredMove, observation.normal);
            UpdateEntryPhase(keepEntryFrame);
            SolveFlat(desiredMove, observation.normal, dt);
        }

        LogPeriodic();
    }
    void Start() {
        rb = GetComponent<Rigidbody>();
        sphereCollider = GetComponent<SphereCollider>();
        heading = NormalizeFlat(initialHeading, transform.forward);
        AcquireOwnership();
        if (!ownsRigidbody) {
            enabled = false;
            return;
        }
        if (disableLegacyControllersOnSameBody)
            DisableLegacyControllers();
        if (logEvents) {
            Debug.Log( $"[COMPACT SLOPE STICK] version={ImplementationVersion} " + $"maxSpeed={maxGroundSpeed:F3} " +
                       $"target={targetSlopeProgressPercent:F2}% " + $"release={naturalReleaseProgressPercent:F2}% " +
                       $"surfaceConfirmDistance={surfaceSwitchConfirmDistance:F3} " +
                       $"supportGraceDistance={supportGraceDistance:F3} " +
                       $"surfaceScoreLimit={maximumSurfaceObserverScore:F3}", this);
        }
        StartCoroutine(DelayStart());
    }
    IEnumerator DelayStart() {
        yield return new WaitForSeconds(.8f);
        GameObject startSlab = GameObject.Find("ArcSlab4");
        restart = startSlab.transform.position;
        rb.position = new Vector3(restart.x, restart.y + 2f, restart.z);
    }
    // ---------------------------------------------------------------------
    // 外部入力 API
    // ---------------------------------------------------------------------
    public void SetMoveInput(Vector2 input) {
        moveInput = Vector2.ClampMagnitude(input, 1f);
    }
    public void SetHeading(Vector3 worldHeading) {
        heading = NormalizeFlat(worldHeading, heading);
    }
    public void AddHeadingRotation(float degrees) {
        heading = Quaternion.AngleAxis(degrees, Vector3.up) * heading;
        heading = NormalizeFlat(heading, transform.forward);
    }
    Vector3 GetDesiredMoveWorld() {
        Vector3 forward = NormalizeFlat(heading, transform.forward);
        Vector3 right = Vector3.Cross(Vector3.up, forward).normalized;
        float forwardInput = autoProgress ? Mathf.Max(1f, moveInput.y) : moveInput.y;
        Vector3 desired = forward * forwardInput + right * moveInput.x * steeringStrength;
        if (desired.sqrMagnitude <= Epsilon)
            return Vector3.zero;
        return desired.normalized;
    }
    // ---------------------------------------------------------------------
    // Rigidbody ownership
    // ---------------------------------------------------------------------
    void AcquireOwnership() {
        ownsRigidbody = false;
        if (rb == null)
            return;
        int key = rb.GetInstanceID();
        if (Owners.TryGetValue(key, out SlopeStick3DCompact1 owner) && owner != null && owner != this) {
            Debug.LogError( $"[COMPACT CONTROLLER DUPLICATE] Rigidbody={rb.name} " + $"owner={owner.name} blocked={name}", this);
            return;
        }
        Owners[key] = this;
        ownsRigidbody = true;
        if (logEvents) {
            Debug.Log( $"[COMPACT CONTROLLER OWNERSHIP] Rigidbody={rb.name} " + $"controller={name}", this);
        }
    }
    void ReleaseOwnership() {
        if (!ownsRigidbody || rb == null)
            return;
        int key = rb.GetInstanceID();
        if (Owners.TryGetValue(key, out SlopeStick3DCompact1 owner) && owner == this) {
            Owners.Remove(key);
        }
        ownsRigidbody = false;
    }
    void DisableLegacyControllers() {
        MonoBehaviour[] behaviours = rb.GetComponentsInParent<MonoBehaviour>(true);
        for (int i = 0; i < behaviours.Length; i++) {
            MonoBehaviour behaviour = behaviours[i];
            if (behaviour == null || behaviour == this)
                continue;
            string typeName = behaviour.GetType().Name;
            bool legacy = typeName == "SlopeStick3D" || typeName == "SlopeStick3D16" || typeName == "SlopeStickBall3D" || typeName == "SlopeStickBall3D2" ||
                typeName == "SlopeStickBall3D4" || typeName == "SlopeStickBall_StateGrouped";
            if (!legacy)
                continue;
            Rigidbody candidateBody = behaviour.GetComponentInParent<Rigidbody>();
            if (candidateBody != rb)
                continue;
            behaviour.enabled = false;
            if (logEvents) {
                Debug.LogWarning( $"[LEGACY CONTROLLER DISABLED] " + $"type={typeName} object={behaviour.name}", behaviour);
            }
        }
    }
    // ---------------------------------------------------------------------
    // Ground observation
    // ---------------------------------------------------------------------
    bool ObserveGround(out GroundObservation observation) {
        if (useCollisionContacts && TryGetRecentCollisionObservation(out observation)) {
            return true;
        }
        Vector3 origin = rb.worldCenterOfMass + Vector3.up * 0.05f;
        float radius = Mathf.Max(0.01f, sphereRadius * 0.95f);
        float distance = groundProbeDistance + 0.05f;
        if (Physics.SphereCast( origin, radius, Vector3.down, out RaycastHit hit, distance, groundMask, QueryTriggerInteraction.Ignore)) {
            float angle = Vector3.Angle(hit.normal, Vector3.up);
            if (angle <= maximumSlopeAngle) {
                observation = new GroundObservation
                {
                    valid = true, collider = hit.collider, point = hit.point, normal = hit.normal.normalized, source = ObservationSource.SphereCast
                };
                return true;
            }
        }
        consecutiveGroundMissFrames++;
        observation = default;
        return false;
    }
    void OnCollisionEnter(Collision collision) {
        CaptureCollisionObservation(collision);
    }
    void OnCollisionStay(Collision collision) {
        CaptureCollisionObservation(collision);
    }
    void CaptureCollisionObservation(Collision collision) {
        if (!useCollisionContacts || collision == null)
            return;
        float bestScore = float.NegativeInfinity;
        GroundObservation best = default;
        for (int i = 0; i < collision.contactCount; i++) {
            ContactPoint contact = collision.GetContact(i);
            float angle = Vector3.Angle(contact.normal, Vector3.up);
            if (angle > maximumSlopeAngle)
                continue;
            float score = Vector3.Dot(contact.normal, Vector3.up);
            if (score <= bestScore)
                continue;
            bestScore = score;
            best = new GroundObservation
            {
                valid = true, collider = contact.otherCollider, point = contact.point, normal = contact.normal.normalized, source = ObservationSource.CollisionContact
            };
        }
        if (!best.valid)
            return;
        recentCollisionObservation = best;
        recentCollisionFixedTime = Time.fixedTime;
    }
    bool TryGetRecentCollisionObservation( out GroundObservation observation) {
        float maximumAge = Time.fixedDeltaTime * (collisionContactMemorySteps + 1.25f);
        if (recentCollisionObservation.valid && Time.fixedTime - recentCollisionFixedTime <= maximumAge) {
            observation = recentCollisionObservation;
            return true;
        }
        observation = default;
        return false;
    }
    void RememberMeasuredSlope(GroundObservation observation) {
        lastMeasuredSlopeObservation = observation;
    }
    bool TryBuildSlopeEntryBridge(float dt, EntryPhase phaseBeforeObservation, out GroundObservation observation) {
        observation = default;
        bridgeActive = false;
        if (!useSlopeEntryBridge || phaseBeforeObservation != EntryPhase.SlopeEntrySettling ||
            !lastMeasuredSlopeObservation.valid || !activeFrame.valid ||
            consecutiveGroundMissFrames > bridgeMaximumMissFrames) {
            return false;
        }
        float progress = CalculateProgress(activeFrame, rb.position);
        if (progress > bridgeEndProgress)
            return false;
        Vector3 normal = lastMeasuredSlopeObservation.normal;
        float centerPlaneDistance = Vector3.Dot( rb.worldCenterOfMass - lastMeasuredSlopeObservation.point, normal);
        float gap = Mathf.Abs(centerPlaneDistance - sphereRadius);
        float outwardSpeed = Mathf.Max( 0f, Vector3.Dot(rb.velocity, normal));
        if (gap > bridgeMaximumGap || outwardSpeed > bridgeMaximumOutwardSpeed) {
            return false;
        }
        observation = lastMeasuredSlopeObservation;
        observation.source = ObservationSource.SlopeEntryBridge;
        observation.point = rb.worldCenterOfMass - normal * sphereRadius;
        bridgeActive = true;
        if (logEvents) {
            Debug.Log( $"[COMPACT SUPPORT BRIDGE] time={Time.fixedTime:F4} " + $"slope={activeFrame.collider.name} " + $"progress={progress * 100f:F3}% gap={gap:F5} " +
                $"outward={outwardSpeed:F5} dt={dt:F4}", this);
        }
        return true;
    }
    bool TryHoldSlopeStateAfterSupportLoss(float dt, EntryPhase phaseBeforeObservation) {
        if (!useDistanceNormalizedSurfaceObserver || !activeFrame.valid || activeFrame.collider == null ||
            !lastMeasuredSlopeObservation.valid || lastMeasuredSlopeObservation.collider != activeFrame.collider ||
            naturalMotionReleased) {
            return false;
        }

        if (phaseBeforeObservation != EntryPhase.SlopeEntrySettling &&
            phaseBeforeObservation != EntryPhase.NormalSlope) {
            return false;
        }

        if (supportMissDistance > supportGraceDistance)
            return false;

        Vector3 normal = lastMeasuredSlopeObservation.normal.normalized;
        Vector3 tangentVelocity = Vector3.ProjectOnPlane(rb.velocity, normal);
        float forwardSpeed = Vector3.Dot(tangentVelocity, activeFrame.axis);
        float outwardSpeed = Mathf.Max(0f, Vector3.Dot(rb.velocity, normal));

        if (forwardSpeed < -0.25f || outwardSpeed > Mathf.Max(bridgeMaximumOutwardSpeed, 1.25f)) {
            return false;
        }

        bool starting = !surfaceGraceActive;
        surfaceGraceActive = true;
        entryPhase = phaseBeforeObservation;
        groundKind = GroundKind.SlopeGrace;
        observationSource = ObservationSource.SurfaceObserverGrace;
        supportColliderName = activeFrame.collider.name;
        activeSlopeName = activeFrame.collider.name;

        GroundObservation retained = lastMeasuredSlopeObservation;
        retained.source = ObservationSource.SurfaceObserverGrace;
        currentObservation = retained;

        if (starting && logEvents) {
            Debug.Log( $"[SURFACE OBSERVER GRACE START] slope={activeFrame.collider.name} " +
                $"missDistance={supportMissDistance:F4}/{supportGraceDistance:F4} " +
                $"progress={CalculateProgress(activeFrame, rb.position) * 100f:F3}% " + $"outward={outwardSpeed:F4}",
                this);
        }

        SolveSurfaceObserverGrace(dt);
        return true;
    }

    void SolveSurfaceObserverGrace(float dt) {
        if (!activeFrame.valid || !lastMeasuredSlopeObservation.valid)
            return;

        Vector3 normal = lastMeasuredSlopeObservation.normal.normalized;
        SurfaceBasis basis = SurfaceBasis.Create(normal, activeFrame.axis, heading);
        if (!basis.valid)
            return;

        if (Vector3.Dot(basis.tangent, activeFrame.axis) < 0f)
            basis.tangent = -basis.tangent;

        float progress01 = CalculateProgress(activeFrame, rb.position);
        slopeProgressPercent = progress01 * 100f;

        SurfaceMotion motion = SurfaceMotion.Evaluate(basis, rb.velocity);
        tangentSpeed = Mathf.Max(0f, motion.tangentSpeed);

        UpdateTargetCrossing(progress01);
        EnsureTargetPlan(progress01);

        float outwardDamping = motion.outwardSpeed / Mathf.Max(outwardNormalResponseSeconds, dt);

        float retainedStickTarget = Mathf.Max( flatStickAcceleration, previousStickAcceleration + outwardDamping);
        float recoveryLimit = Mathf.Min( maximumAdaptiveStickAcceleration,
            Mathf.Max(flatStickAcceleration, bridgeRecoveryAcceleration));
        retainedStickTarget = Mathf.Min(retainedStickTarget, recoveryLimit);

        previousStickAcceleration = StepAsymmetricJerkLimited( previousStickAcceleration, retainedStickTarget,
            stickRiseJerkLimit, stickFallJerkLimit, dt);

        ApplySurfaceAcceleration( basis, 0f, previousStickAcceleration, motion.lateralSpeed);

        currentCurvature = EvaluateCurvature(activeFrame, progress01);
        UpdateCriticalReadout( tangentSpeed, currentStickAcceleration, motion.gravitySupport);

        previousProgress01 = progress01;
        hadProgressObservation = true;
    }

    void EndSurfaceGrace(string reason) {
        if (!surfaceGraceActive)
            return;

        if (logEvents) {
            string slopeName = activeFrame.valid && activeFrame.collider != null ? activeFrame.collider.name
                : activeSlopeName;
            Debug.Log( $"[SURFACE OBSERVER GRACE END] slope={slopeName} " +
                $"missDistance={supportMissDistance:F4} reason={reason}", this);
        }

        surfaceGraceActive = false;
    }

    // ---------------------------------------------------------------------
    // SlopeFrame construction
    // ---------------------------------------------------------------------
    void UpdateActiveSlopeFrame( GroundObservation observation, Vector3 desiredMove, float dt) {
        if (logEntryFrameDebug) {
            string forwardName = forwardFrame.valid && forwardFrame.collider != null ? forwardFrame.collider.name
                : "null";
            string contactName = observation.collider != null ? observation.collider.name : "null";
            string activeName = activeFrame.valid && activeFrame.collider != null ? activeFrame.collider.name
                : "null";

            float forwardSignedDistance = forwardFrame.valid
                ? Vector3.Dot(forwardFrame.entryPoint - rb.position, forwardFrame.axis) : 0f;

            Debug.Log( $"[ENTRY FRAME DEBUG] time={Time.fixedTime:F4} " +
                $"forwardValid={forwardFrame.valid} forwardCollider={forwardName} " +
                $"contactCollider={contactName} forwardMissFrames={forwardFrameMissFrames} " +
                $"forwardMissDistance={forwardFrameMissDistance:F4} " +
                $"entryPhase={entryPhase} activeValid={activeFrame.valid} activeCollider={activeName} " +
                $"candidate={surfaceCandidateName} candidateDistance={surfaceCandidateDistance:F4} " +
                $"candidateScore={surfaceCandidateScore:F4} " + $"forwardSignedDistance={forwardSignedDistance:F5}",
                this);
        }

        bool changed = !activeFrame.valid || activeFrame.collider != observation.collider;

        if (!changed) {
            if (pendingSurfaceFrame.valid && logEvents) {
                Debug.Log( $"[SURFACE TRANSIENT IGNORED] active={activeFrame.collider.name} " +
                    $"ignored={surfaceCandidateName} distance={surfaceCandidateDistance:F4} " +
                    $"samples={surfaceCandidateSamples}", this);
            }

            ClearSurfaceCandidate();
            activeFrame.normal = BlendUnitVectors( activeFrame.normal, observation.normal, 0.20f);
            stableSlopeFrames++;
            stableSlopeDistance += CalculateSurfaceTravelDistance( activeFrame, dt);
            RememberMeasuredSlope(observation);
            return;
        }

        bool promoted = forwardFrame.valid && forwardFrame.collider == observation.collider;

        SlopeFrame nextFrame;
        if (promoted) {
            nextFrame = forwardFrame;
            if (Vector3.Dot(nextFrame.normal, observation.normal) < 0f)
                nextFrame.normal = -nextFrame.normal;
        }
        else if (!BuildSlopeFrame( observation.collider, observation.point, observation.normal, desiredMove,
            out nextFrame)) {
            RetainActiveObservation();
            return;
        }

        if (activeFrame.valid && useDistanceNormalizedSurfaceObserver && !ConfirmSurfaceSwitch( observation,
                nextFrame, dt)) {
            RetainActiveObservation();
            return;
        }

        AdoptActiveSlopeFrame( observation, nextFrame, promoted, dt);
    }

    bool ConfirmSurfaceSwitch( GroundObservation observation, SlopeFrame candidateFrame, float dt) {
        float score = CalculateSurfaceObserverScore( observation, candidateFrame);

        if (IsBackwardEntryCandidate(candidateFrame)) {
            if (logEvents) {
                Debug.Log( $"[SURFACE CANDIDATE REJECTED] active={activeFrame.collider.name} " +
                    $"candidate={candidateFrame.collider.name} reason=backward_entry " +
                    $"progress={CalculateProgress(activeFrame, rb.position) * 100f:F3}%", this);
            }

            ClearSurfaceCandidate();
            return false;
        }

        if (!pendingSurfaceFrame.valid || pendingSurfaceFrame.collider != candidateFrame.collider) {
            pendingSurfaceFrame = candidateFrame;
            pendingSurfaceObservation = observation;
            surfaceCandidateDistance = 0f;
            surfaceCandidateSamples = 0;
        }
        else {
            pendingSurfaceFrame = candidateFrame;
            pendingSurfaceObservation = observation;
        }

        surfaceCandidateDistance += CalculateObserverDistanceIncrement(dt);
        surfaceCandidateSamples++;
        surfaceCandidateScore = score;
        surfaceCandidateName = candidateFrame.collider != null ? candidateFrame.collider.name : string.Empty;

        if (score > maximumSurfaceObserverScore) {
            if (logEvents) {
                Debug.Log( $"[SURFACE CANDIDATE REJECTED] active={activeFrame.collider.name} " +
                    $"candidate={surfaceCandidateName} score={score:F4}/" +
                    $"{maximumSurfaceObserverScore:F4} reason=geometry", this);
            }

            ClearSurfaceCandidate();
            return false;
        }

        if (surfaceCandidateSamples < 2 || surfaceCandidateDistance < surfaceSwitchConfirmDistance) {
            if (logEvents) {
                Debug.Log( $"[SURFACE CANDIDATE] active={activeFrame.collider.name} " +
                    $"candidate={surfaceCandidateName} samples={surfaceCandidateSamples} " +
                    $"distance={surfaceCandidateDistance:F4}/" +
                    $"{surfaceSwitchConfirmDistance:F4} score={score:F4}", this);
            }

            return false;
        }

        if (logEvents) {
            Debug.Log( $"[SURFACE SWITCH CONFIRMED] old={activeFrame.collider.name} " +
                $"new={surfaceCandidateName} samples={surfaceCandidateSamples} " +
                $"distance={surfaceCandidateDistance:F4} score={score:F4}", this);
        }

        ClearSurfaceCandidate();
        return true;
    }

    float CalculateSurfaceObserverScore( GroundObservation observation, SlopeFrame candidateFrame) {
        float normalDot = Mathf.Clamp01(Vector3.Dot( activeFrame.normal.normalized,
            candidateFrame.normal.normalized));
        float normalCost = (1f - normalDot) * 2f;

        float axisDot = Mathf.Clamp01(Vector3.Dot( activeFrame.axis.normalized, candidateFrame.axis.normalized));
        float axisCost = (1f - axisDot) * 1.5f;

        float supportGap = Mathf.Abs( Vector3.Dot( rb.worldCenterOfMass - observation.point,
                observation.normal.normalized) - sphereRadius);
        float gapCost = Mathf.Min( 2f, supportGap / Mathf.Max(0.05f, sphereRadius)) * 0.5f;

        float forwardConnectionGap = Vector3.Distance( activeFrame.exitPoint, candidateFrame.entryPoint);
        float backwardConnectionGap = Vector3.Distance( activeFrame.entryPoint, candidateFrame.exitPoint);
        float topologyScale = Mathf.Max( 0.10f, connectedSurfaceMaximumGap);
        float topologyCost = Mathf.Min( 2f, forwardConnectionGap / topologyScale) * 0.5f;

        float backwardPenalty = backwardConnectionGap + sphereRadius * 0.5f < forwardConnectionGap ? 2f : 0f;

        Vector3 candidateVelocity = Vector3.ProjectOnPlane( rb.velocity, candidateFrame.normal);
        float candidateForwardSpeed = Vector3.Dot( candidateVelocity, candidateFrame.axis);
        float reversePenalty = candidateForwardSpeed < -0.25f ? 3f : 0f;

        return normalCost + axisCost + gapCost + topologyCost + backwardPenalty + reversePenalty;
    }

    bool IsBackwardEntryCandidate(SlopeFrame candidateFrame) {
        if (!activeFrame.valid || entryPhase != EntryPhase.SlopeEntrySettling) {
            return false;
        }

        float activeProgress = CalculateProgress( activeFrame, rb.position);
        if (activeProgress >= bridgeEndProgress)
            return false;

        float candidateProgress = CalculateProgress( candidateFrame, rb.position);
        float forwardConnectionGap = Vector3.Distance( activeFrame.exitPoint, candidateFrame.entryPoint);
        float backwardConnectionGap = Vector3.Distance( activeFrame.entryPoint, candidateFrame.exitPoint);

        return candidateProgress > 0.55f && backwardConnectionGap + sphereRadius * 0.5f < forwardConnectionGap;
    }

    void AdoptActiveSlopeFrame( GroundObservation observation, SlopeFrame nextFrame, bool promoted, float dt) {
        activeFrame = nextFrame;
        activeSlopeName = nextFrame.collider.name;
        stableSlopeFrames = 1;
        ResetSlopeSession(false);
        ClearSurfaceCandidate();

        if (promoted) {
            ApplyEntryVelocityTransport(activeFrame);
            forwardFrame = default;
            forwardFrameMissFrames = 0;
            forwardFrameMissDistance = 0f;
        }

        stableSlopeDistance = CalculateSurfaceTravelDistance( activeFrame, dt);

        GroundObservation acceptedObservation = observation;
        acceptedObservation.normal = activeFrame.normal;
        currentObservation = acceptedObservation;
        observationSource = acceptedObservation.source;
        supportColliderName = activeFrame.collider.name;
        RememberMeasuredSlope(acceptedObservation);

        if (logEvents) {
            string eventName = promoted ? "[COMPACT SLOPE FRAME PROMOTED]" : "[COMPACT SLOPE START]";
            Debug.Log( $"{eventName} slope={activeSlopeName} " + $"length={activeFrame.length:F4} " +
                $"curvature={activeFrame.representativeCurvature:F6}", this);
        }
    }

    void RetainActiveObservation() {
        if (!activeFrame.valid)
            return;

        GroundObservation retained;
        if (lastMeasuredSlopeObservation.valid && lastMeasuredSlopeObservation.collider == activeFrame.collider) {
            retained = lastMeasuredSlopeObservation;
        }
        else {
            retained = new GroundObservation
            {
                valid = true, collider = activeFrame.collider, point = rb.worldCenterOfMass -
                    activeFrame.normal.normalized * sphereRadius, normal = activeFrame.normal.normalized,
                source = ObservationSource.SurfaceObserverGrace
            };
        }

        retained.source = ObservationSource.SurfaceObserverGrace;
        currentObservation = retained;
        observationSource = ObservationSource.SurfaceObserverGrace;
        supportColliderName = activeFrame.collider.name;
    }

    void ClearSurfaceCandidate() {
        pendingSurfaceFrame = default;
        pendingSurfaceObservation = default;
        surfaceCandidateName = string.Empty;
        surfaceCandidateDistance = 0f;
        surfaceCandidateScore = 0f;
        surfaceCandidateSamples = 0;
    }

    void UpdateForwardSlope(Vector3 desiredMove, Vector3 flatNormal) {
        if (desiredMove.sqrMagnitude <= Epsilon) {
            RegisterForwardFrameMiss(Time.fixedDeltaTime);
            return;
        }

        Vector3 travel = Vector3.ProjectOnPlane(desiredMove, flatNormal).normalized;
        if (travel.sqrMagnitude <= Epsilon) {
            RegisterForwardFrameMiss(Time.fixedDeltaTime);
            return;
        }

        float step = forwardSlopeProbeDistance / Mathf.Max(1, forwardSlopeProbeSegments);
        for (int i = 1; i <= forwardSlopeProbeSegments; i++) {
            Vector3 sample = rb.position + travel * (step * i);
            Vector3 origin = sample + Vector3.up * forwardProbeHeight;
            if (!Physics.Raycast(origin, Vector3.down, out RaycastHit hit,
                forwardProbeDownDistance, groundMask, QueryTriggerInteraction.Ignore)) {
                continue;
            }

            float angle = Vector3.Angle(hit.normal, Vector3.up);
            if (angle < minimumSlopeAngle || angle > maximumSlopeAngle)
                continue;

            if (BuildSlopeFrame(hit.collider, hit.point, hit.normal, travel, out SlopeFrame detected)) {
                forwardFrame = detected;
                forwardFrameMissFrames = 0;
                forwardFrameMissDistance = 0f;

                if (logEntryFrameDebug) {
                    float signedDistance = Vector3.Dot( detected.entryPoint - rb.position, detected.axis);

                    Debug.Log( $"[FORWARD FRAME CAPTURED] time={Time.fixedTime:F4} " +
                        $"collider={detected.collider.name} sampleIndex={i} " +
                        $"entry={detected.entryPoint:F4} exit={detected.exitPoint:F4} " +
                        $"axis={detected.axis:F4} length={detected.length:F4} " +
                        $"signedDistance={signedDistance:F5} angle={angle:F3}", this);
                }

                return;
            }
        }

        RegisterForwardFrameMiss(Time.fixedDeltaTime);
    }

    void RegisterForwardFrameMiss(float dt) {
        forwardFrameMissFrames++;
        forwardFrameMissDistance += CalculateObserverDistanceIncrement(dt);

        float retentionDistance = Mathf.Max( sphereRadius, surfaceSwitchConfirmDistance * 1.75f);

        if (forwardFrameMissDistance > retentionDistance) {
            if (logEntryFrameDebug && forwardFrame.valid) {
                string colliderName = forwardFrame.collider != null ? forwardFrame.collider.name : "null";

                Debug.Log( $"[FORWARD FRAME CLEARED] time={Time.fixedTime:F4} " + $"collider={colliderName} " +
                    $"missDistance={forwardFrameMissDistance:F4}/" + $"{retentionDistance:F4} reason=distance_limit",
                    this);
            }

            forwardFrame = default;
            forwardFrameMissFrames = 0;
            forwardFrameMissDistance = 0f;
        }
    }
    bool BuildSlopeFrame( Collider slopeCollider, Vector3 referencePoint, Vector3 referenceNormal, Vector3 desiredMove, out SlopeFrame frame) {
        frame = default;
        if (slopeCollider == null)
            return false;
        Vector3 axis = Vector3.ProjectOnPlane( desiredMove, referenceNormal);
        if (axis.sqrMagnitude <= Epsilon && activeFrame.valid)
            axis = Vector3.ProjectOnPlane( activeFrame.axis, referenceNormal);
        if (axis.sqrMagnitude <= Epsilon)
            axis = Vector3.ProjectOnPlane( transform.forward, referenceNormal);
        if (axis.sqrMagnitude <= Epsilon)
            return false;
        axis.Normalize();
        if (desiredMove.sqrMagnitude > Epsilon && Vector3.Dot(axis, desiredMove) < 0f) {
            axis = -axis;
        }
        Vector3[] points = GetColliderWorldCorners(slopeCollider);
        if (points == null || points.Length == 0)
            return false;
        float minimum = float.PositiveInfinity;
        float maximum = float.NegativeInfinity;
        for (int i = 0; i < points.Length; i++) {
            float coordinate = Vector3.Dot(points[i], axis);
            minimum = Mathf.Min(minimum, coordinate);
            maximum = Mathf.Max(maximum, coordinate);
        }
        float referenceCoordinate = Vector3.Dot(referencePoint, axis);
        Vector3 entry = referencePoint + axis * (minimum - referenceCoordinate);
        Vector3 exit = referencePoint + axis * (maximum - referenceCoordinate);
        float length = Mathf.Max(0.01f, maximum - minimum);
        frame.valid = true;
        frame.collider = slopeCollider;
        frame.normal = referenceNormal.normalized;
        frame.axis = axis;
        frame.entryPoint = entry;
        frame.exitPoint = exit;
        frame.length = length;
        PopulateBoundaryCurvature(ref frame);
        return true;
    }
    Vector3[] GetColliderWorldCorners(Collider collider) {
        Vector3[] points = new Vector3[8];
        if (collider is BoxCollider box) {
            Vector3 half = box.size * 0.5f;
            int index = 0;
            for (int x = -1; x <= 1; x += 2)
            for (int y = -1; y <= 1; y += 2)
            for (int z = -1; z <= 1; z += 2) {
                Vector3 local = box.center + Vector3.Scale( half, new Vector3(x, y, z));
                points[index++] = box.transform.TransformPoint(local);
            }
            return points;
        }
        Bounds bounds = collider.bounds;
        Vector3 min = bounds.min;
        Vector3 max = bounds.max;
        points[0] = new Vector3(min.x, min.y, min.z);
        points[1] = new Vector3(min.x, min.y, max.z);
        points[2] = new Vector3(min.x, max.y, min.z);
        points[3] = new Vector3(min.x, max.y, max.z);
        points[4] = new Vector3(max.x, min.y, min.z);
        points[5] = new Vector3(max.x, min.y, max.z);
        points[6] = new Vector3(max.x, max.y, min.z);
        points[7] = new Vector3(max.x, max.y, max.z);
        return points;
    }
    void PopulateBoundaryCurvature(ref SlopeFrame frame) {
        frame.entryCurvature = 0f;
        frame.exitCurvature = 0f;
        frame.representativeCurvature = minimumCurvature;
        if (TrySampleConnectedNormal( frame.entryPoint, -frame.axis, frame.collider, out Vector3 entryNormal)) {
            frame.entryCurvature = CalculateBoundaryCurvature( entryNormal, frame.normal);
        }
        if (TrySampleConnectedNormal( frame.exitPoint, frame.axis, frame.collider, out Vector3 exitNormal)) {
            frame.exitCurvature = CalculateBoundaryCurvature( frame.normal, exitNormal);
        }
        frame.representativeCurvature = Mathf.Clamp( Mathf.Max( minimumCurvature, Mathf.Max(frame.entryCurvature, frame.exitCurvature)), minimumCurvature,
            maximumRepresentativeCurvature);
    }
    bool TrySampleConnectedNormal( Vector3 boundaryPoint, Vector3 outwardAxis, Collider excluded, out Vector3 normal) {
        normal = Vector3.up;
        Vector3 sample = boundaryPoint + outwardAxis.normalized * connectedSurfaceProbeOffset;
        Vector3 origin = sample + Vector3.up * forwardProbeHeight;
        RaycastHit[] hits = Physics.RaycastAll( origin, Vector3.down, forwardProbeDownDistance, groundMask, QueryTriggerInteraction.Ignore);
        float bestDistance = float.PositiveInfinity;
        bool found = false;
        for (int i = 0; i < hits.Length; i++) {
            RaycastHit hit = hits[i];
            if (hit.collider == null || hit.collider == excluded)
                continue;
            if (Vector3.Distance(hit.point, boundaryPoint) > connectedSurfaceMaximumGap) {
                continue;
            }
            float angle = Vector3.Angle(hit.normal, Vector3.up);
            if (angle > maximumSlopeAngle)
                continue;
            if (hit.distance >= bestDistance)
                continue;
            bestDistance = hit.distance;
            normal = hit.normal.normalized;
            found = true;
        }
        return found;
    }
    float CalculateBoundaryCurvature( Vector3 beforeNormal, Vector3 afterNormal) {
        float angleRadians = Vector3.Angle( beforeNormal, afterNormal) * Mathf.Deg2Rad;
        float regularizedDistance = Mathf.Max( boundaryCurvatureDistance, sphereRadius * 1.5f);
        return Mathf.Clamp( angleRadians / regularizedDistance, 0f, maximumRepresentativeCurvature);
    }
    float CalculateProgress(SlopeFrame frame, Vector3 position) {
        if (!frame.valid || frame.length <= Epsilon)
            return 0f;
        float coordinate = Vector3.Dot( position - frame.entryPoint, frame.axis);
        return Mathf.Clamp01(coordinate / frame.length);
    }
    void UpdateEntryPhase(bool preserveSettlingOnFlat = false) {
        entryDistance = 0f;
        entryWeight = 0f;

        if (groundKind == GroundKind.Slope && activeFrame.valid) {
            float progress = CalculateProgress(activeFrame, rb.position);
            entryPhase = progress < bridgeEndProgress ? EntryPhase.SlopeEntrySettling : EntryPhase.NormalSlope;
            return;
        }

        if (preserveSettlingOnFlat && activeFrame.valid) {
            entryPhase = EntryPhase.SlopeEntrySettling;
            return;
        }

        if (groundKind == GroundKind.Flat && forwardFrame.valid) {
            entryPhase = EntryPhase.FlatApproach;
            return;
        }

        entryPhase = EntryPhase.Inactive;
    }

    bool ShouldKeepActiveEntryFrame(EntryPhase phaseBeforeObservation) {
        if (!activeFrame.valid || phaseBeforeObservation != EntryPhase.SlopeEntrySettling)
            return false;

        return CalculateProgress(activeFrame, rb.position) < bridgeEndProgress;
    }

    float CalculateEntryPreloadAcceleration(SlopeFrame frame, float signedDistanceToEntry, float dt, out float weight) {
        weight = 0f;
        if (!frame.valid || signedDistanceToEntry <= 0f)
            return 0f;

        float speed = Vector3.ProjectOnPlane(rb.velocity, Vector3.up).magnitude;
        float window = Mathf.Max(2f * sphereRadius, speed * Mathf.Max(0.10f, 5f * dt));
        weight = SmootherStep01(1f - signedDistanceToEntry / Mathf.Max(0.01f, window));

        const float preloadRatio = 0.25f;
        return Mathf.Max(0f, flatStickAcceleration * preloadRatio * weight);
    }

    void ApplyEntryVelocityTransport(SlopeFrame frame) {
        if (!useEntryVelocityTransport || !frame.valid || rb == null)
            return;

        Vector3 normal = frame.normal.sqrMagnitude > Epsilon ? frame.normal.normalized : Vector3.up;
        Vector3 velocityBefore = rb.velocity;
        float outwardSpeedBefore = Vector3.Dot(velocityBefore, normal);
        if (outwardSpeedBefore <= 0.01f || velocityBefore.sqrMagnitude <= Epsilon)
            return;

        Vector3 slopePlaneVelocity = Vector3.ProjectOnPlane(velocityBefore, normal);
        if (slopePlaneVelocity.sqrMagnitude <= Epsilon)
            return;

        Vector3 transportedVelocity = slopePlaneVelocity.normalized * velocityBefore.magnitude;
        Vector3 velocityChange = (transportedVelocity - velocityBefore) * Mathf.Clamp01(entryVelocityTransportWeight);

        rb.AddForce(velocityChange, ForceMode.VelocityChange);

        if (logEvents) {
            Vector3 expectedVelocity = velocityBefore + velocityChange;
            float outwardSpeedAfter = Vector3.Dot(expectedVelocity, normal);
            Debug.Log( $"[ENTRY VELOCITY TRANSPORT] slope={frame.collider.name} " +
                $"weight={entryVelocityTransportWeight:F3} " + $"speedBefore={velocityBefore.magnitude:F4} " +
                $"outwardBefore={outwardSpeedBefore:F4} " + $"deltaV={velocityChange.magnitude:F4} " +
                $"speedAfter={expectedVelocity.magnitude:F4} " + $"outwardAfter={outwardSpeedAfter:F4}", this);
        }
    }

    float CalculateBridgeRecoveryAcceleration(float dt) {
        if (!lastMeasuredSlopeObservation.valid)
            return 0f;

        Vector3 normal = lastMeasuredSlopeObservation.normal.normalized;
        float centerPlaneDistance = Vector3.Dot( rb.worldCenterOfMass - lastMeasuredSlopeObservation.point, normal);
        float gap = Mathf.Max(0f, Mathf.Abs(centerPlaneDistance) - sphereRadius);
        float outwardSpeed = Mathf.Max(0f, Vector3.Dot(rb.velocity, normal));
        float horizon = Mathf.Max(2f * dt, 0.04f);
        float positionRecovery = 2f * (gap + outwardSpeed * horizon) / Mathf.Max(horizon * horizon, Epsilon);
        float velocityRecovery = outwardSpeed / horizon;
        float required = Mathf.Max(positionRecovery, velocityRecovery);
        float limit = Mathf.Min(bridgeRecoveryAcceleration, maximumAdaptiveStickAcceleration);
        return Mathf.Clamp(required, 0f, limit);
    }
    static Vector3 BlendUnitVectors(Vector3 from, Vector3 to, float weight) {
        if (from.sqrMagnitude <= Epsilon)
            return to.sqrMagnitude > Epsilon ? to.normalized : Vector3.up;
        if (to.sqrMagnitude <= Epsilon)
            return from.normalized;
        if (Vector3.Dot(from, to) < 0f)
            to = -to;
        return Vector3.Slerp(from.normalized, to.normalized, Mathf.Clamp01(weight)).normalized;
    }
    // ---------------------------------------------------------------------
    // Flat control
    // ---------------------------------------------------------------------
    void SolveFlat(Vector3 desiredMove, Vector3 flatNormal, float dt) {
        SurfaceBasis basis = SurfaceBasis.Create(flatNormal, desiredMove, heading);
        if (!basis.valid)
            return;
        SurfaceMotion motion = SurfaceMotion.Evaluate(basis, rb.velocity);
        float desiredSpeed = desiredMove.sqrMagnitude > Epsilon ? maxGroundSpeed : 0f;
        float targetAcceleration = (desiredSpeed - motion.tangentSpeed) / Mathf.Max(0.01f, groundResponseSeconds);
        targetAcceleration = Mathf.Clamp(targetAcceleration, -maximumTangentialDeceleration, groundAcceleration);
        entryPreloadAcceleration = 0f;
        if (entryPhase == EntryPhase.FlatApproach && forwardFrame.valid) {
            float signedEntryDistance = Vector3.Dot(forwardFrame.entryPoint - rb.position, basis.tangent);
            if (signedEntryDistance > 0f) {
                entryDistance = signedEntryDistance;
                float entrySpeed = CalculateSafeSpeed(forwardFrame);
                float permitted = ReachableSpeedBeforeConstraint(entrySpeed, maximumTangentialDeceleration, entryDistance);
                if (motion.tangentSpeed > permitted) {
                    float brake = RequiredAccelerationForSpeedChange(Mathf.Max(0f, motion.tangentSpeed), permitted,
                        Mathf.Max(targetMinimumDistance, entryDistance));
                    targetAcceleration = Mathf.Min(targetAcceleration, brake);
                }
                entryPreloadAcceleration = CalculateEntryPreloadAcceleration(forwardFrame, signedEntryDistance, dt, out entryWeight);
            }
            else {
                entryDistance = 0f;
                entryWeight = 0f;
                entryPreloadAcceleration = 0f;
            }
        }
        previousGroundAcceleration = StepJerkLimited(previousGroundAcceleration, targetAcceleration, tangentialJerkLimit, dt);
        ApplySurfaceAcceleration(basis, previousGroundAcceleration, flatStickAcceleration, motion.lateralSpeed);
        if (entryPreloadAcceleration > 0f && forwardFrame.valid)
            rb.AddForce(-forwardFrame.normal.normalized * entryPreloadAcceleration, ForceMode.Acceleration);
        currentStickAcceleration = flatStickAcceleration + entryPreloadAcceleration;
        currentCurvature = 0f;
        currentCriticalRatio = 0f;
        currentReleaseWeight = 1f;
        releasePhase = ReleasePhase.Controlled;
    }
    // ---------------------------------------------------------------------
    // Slope control
    // ---------------------------------------------------------------------
    void SolveSlope(Vector3 desiredMove, float dt) {
        if (!activeFrame.valid) {
            SolveAir(desiredMove);
            return;
        }
        Vector3 controlNormal = entryPhase == EntryPhase.SlopeEntrySettling ? activeFrame.normal : currentObservation.normal;
        SurfaceBasis basis = SurfaceBasis.Create(controlNormal, activeFrame.axis, desiredMove);
        if (!basis.valid)
            return;
        if (Vector3.Dot(basis.tangent, activeFrame.axis) < 0f)
            basis.tangent = -basis.tangent;
        float progress01 = CalculateProgress(activeFrame, rb.position);
        slopeProgressPercent = progress01 * 100f;
        SurfaceMotion motion = SurfaceMotion.Evaluate(basis, rb.velocity);
        tangentSpeed = Mathf.Max(0f, motion.tangentSpeed);
        UpdateTargetCrossing(progress01);
        EnsureTargetPlan(progress01);
        UpdateReleasePlan(progress01);
        float baseAcceleration = CalculateGroundDriveAcceleration(motion.tangentSpeed, desiredMove, dt);
        float targetAcceleration = CalculateTargetProgressAcceleration(progress01, tangentSpeed, motion.gravityAlong, dt);
        float selectedAcceleration = targetPhase == TargetPhase.Controlling ? targetAcceleration : baseAcceleration;
        selectedAcceleration += CalculatePostTargetGravityCompensation(progress01, motion.gravityAlong);
        if (progress01 >= capturedTargetProgress01)
            selectedAcceleration = Mathf.Min(0f, selectedAcceleration);
        selectedAcceleration = Mathf.Max(-maximumTangentialDeceleration, selectedAcceleration);
        if (bridgeActive && entryPhase == EntryPhase.SlopeEntrySettling)
            selectedAcceleration = Mathf.Min(0f, selectedAcceleration);
        float releaseWeight = EvaluateReleaseWeight(progress01);
        selectedAcceleration *= releaseWeight;
        currentCurvature = EvaluateCurvature(activeFrame, progress01);
        float stick = CalculateAdaptiveStick(tangentSpeed, selectedAcceleration, currentCurvature,
            motion.gravitySupport, motion.outwardSpeed, dt) * releaseWeight;
        if (bridgeActive)
            stick = Mathf.Max(stick, CalculateBridgeRecoveryAcceleration(dt));
        ApplySurfaceAcceleration(basis, selectedAcceleration, stick, motion.lateralSpeed);
        UpdateCriticalReadout(tangentSpeed, stick, motion.gravitySupport);
        previousProgress01 = progress01;
        hadProgressObservation = true;
    }
    float CalculateGroundDriveAcceleration(float signedSpeed, Vector3 desiredMove, float dt) {
        float desiredSpeed = desiredMove.sqrMagnitude > Epsilon ? maxGroundSpeed : 0f;
        float raw = (desiredSpeed - signedSpeed) / Mathf.Max(0.01f, groundResponseSeconds);
        raw = Mathf.Clamp(raw, -maximumTangentialDeceleration, groundAcceleration);
        previousGroundAcceleration = StepJerkLimited(previousGroundAcceleration, raw, tangentialJerkLimit, dt);
        return previousGroundAcceleration;
    }
    float CalculatePostTargetGravityCompensation(float progress01, float gravityAlong) {
        postTargetGravityCompensationWeight = 0f;
        postTargetGravityCompensationAcceleration = 0f;
        if (!usePostTargetGravityCompensation || !targetPlanValid || progress01 < capturedTargetProgress01)
            return 0f;
        float blendEnd01 = capturedTargetProgress01 + Mathf.Max(0.001f, postTargetGravityBlendWidthPercent * 0.01f);
        postTargetGravityCompensationWeight = SmoothRange01(progress01, capturedTargetProgress01, blendEnd01);
        float downhillGravity = Mathf.Max(0f, gravityAlong);
        postTargetGravityCompensationAcceleration = -downhillGravity * Mathf.Clamp01(postTargetGravityCompensationRatio) *
            postTargetGravityCompensationWeight;
        return postTargetGravityCompensationAcceleration;
    }
    void EnsureTargetPlan(float progress01) {
        if (!useTargetProgressControl) {
            targetPhase = TargetPhase.Inactive;
            targetPlanValid = false;
            capturedTargetSpeed = maxGroundSpeed;
            capturedTargetProgress01 = Mathf.Clamp01(targetSlopeProgressPercent * 0.01f);
            return;
        }
        if (targetPlanValid)
            return;
        targetPhase = TargetPhase.Observing;
        if (stableSlopeFrames < requiredStableSlopeFrames)
            return;
        if (useDistanceNormalizedSurfaceObserver && stableSlopeDistance < surfaceSwitchConfirmDistance)
            return;
        capturedTargetProgress01 = Mathf.Clamp01(targetSlopeProgressPercent * 0.01f);
        capturedTargetSpeed = Mathf.Min(maxGroundSpeed, CalculateSafeSpeed(activeFrame));
        targetTangentSpeed = capturedTargetSpeed;
        targetPlanValid = true;
        targetPhase = progress01 < capturedTargetProgress01 ? TargetPhase.Controlling : TargetPhase.Completed;
        if (logEvents) {
            Debug.Log($"[COMPACT TARGET PLAN] slope={activeFrame.collider.name} " +
                $"progress={progress01 * 100f:F3}% target={capturedTargetProgress01 * 100f:F3}% " +
                $"targetSpeed={capturedTargetSpeed:F4} curvature={activeFrame.representativeCurvature:F6}", this);
        }
    }
    float CalculateTargetProgressAcceleration(float progress01, float speed, float gravityAlong, float dt) {
        if (!targetPlanValid || targetPhase != TargetPhase.Controlling) {
            previousTargetAcceleration = StepJerkLimited(previousTargetAcceleration, 0f, targetJerkLimit, dt);
            return previousTargetAcceleration;
        }
        float remaining = Mathf.Max(targetMinimumDistance, (capturedTargetProgress01 - progress01) * activeFrame.length);
        float requiredNet = RequiredAccelerationForSpeedChange(speed, capturedTargetSpeed, remaining);
        float artificial = compensateSlopeGravity ? requiredNet - gravityAlong : requiredNet;
        artificial = Mathf.Clamp(artificial, -targetMaximumArtificialDeceleration, targetMaximumArtificialAcceleration);
        previousTargetAcceleration = StepJerkLimited(previousTargetAcceleration, artificial, targetJerkLimit, dt);
        return previousTargetAcceleration;
    }
    void UpdateTargetCrossing(float progress01) {
        if (!hadProgressObservation)
            return;
        float target01 = targetPlanValid ? capturedTargetProgress01 : Mathf.Clamp01(targetSlopeProgressPercent * 0.01f);
        if (previousProgress01 >= target01 || progress01 < target01)
            return;
        targetPhase = TargetPhase.Completed;
        float denominator = Mathf.Max(Epsilon, progress01 - previousProgress01);
        float fraction = Mathf.Clamp01((target01 - previousProgress01) / denominator);
        targetCrossingEstimatedTime = Time.fixedTime - Time.fixedDeltaTime * (1f - fraction);
        targetCrossingSampleOvershootPercent = (progress01 - target01) * 100f;
        if (logEvents) {
            Debug.Log($"[COMPACT TARGET CROSSED] slope={activeFrame.collider.name} " +
                $"target={target01 * 100f:F3}% previous={previousProgress01 * 100f:F3}% " +
                $"actual={progress01 * 100f:F3}% sampleOvershoot={targetCrossingSampleOvershootPercent:F3}% " +
                $"interpolated={target01 * 100f:F3}% estimatedTime={targetCrossingEstimatedTime:F5}", this);
        }
    }
    // ---------------------------------------------------------------------
    // Critical speed and stick
    // ---------------------------------------------------------------------
    float CalculateSafeSpeed(SlopeFrame frame) {
        float curvature = Mathf.Max(minimumCurvature, frame.representativeCurvature);
        float availableSupport = CalculateGravitySupport(frame.normal) + maximumAdaptiveStickAcceleration;
        return Mathf.Sqrt(Mathf.Max(0f, targetCriticalRatio * availableSupport / curvature));
    }
    float EvaluateCurvature(SlopeFrame frame, float progress01) {
        float entryWeight = 1f - SmoothRange01(progress01, 0f, bridgeEndProgress);
        float exitWeight = SmoothRange01(progress01, Mathf.Clamp01(1f - bridgeEndProgress), 1f);
        float curvature = frame.representativeCurvature;
        if (frame.entryCurvature > minimumCurvature)
            curvature = Mathf.Lerp(curvature, frame.entryCurvature, entryWeight);
        if (frame.exitCurvature > minimumCurvature)
            curvature = Mathf.Lerp(curvature, frame.exitCurvature, exitWeight);
        return Mathf.Clamp(curvature, minimumCurvature, maximumRepresentativeCurvature);
    }
    float CalculateAdaptiveStick(float currentSpeed, float tangentialAcceleration, float curvature,
        float gravitySupport, float outwardSpeed, float dt) {
        if (!useAdaptiveCriticalStick || curvature <= minimumCurvature) {
            previousStickAcceleration = StepAsymmetricJerkLimited(previousStickAcceleration, flatStickAcceleration,
                stickRiseJerkLimit, stickFallJerkLimit, dt);
            return previousStickAcceleration;
        }
        float forecastSpeed = Mathf.Max(0f, currentSpeed + Mathf.Max(0f, tangentialAcceleration) * stickPredictionSeconds);
        float requiredStick = CriticalDemand(forecastSpeed, curvature) / Mathf.Max(0.01f, targetCriticalRatio) - gravitySupport;
        requiredStick = Mathf.Max(0f, requiredStick) * stickSafetyMargin;
        float outwardDamping = outwardSpeed / Mathf.Max(dt, outwardNormalResponseSeconds);
        float unsaturatedTarget = requiredStick + outwardDamping;
        float target = Mathf.Min(maximumAdaptiveStickAcceleration, unsaturatedTarget);
        if (unsaturatedTarget > maximumAdaptiveStickAcceleration + 0.01f && !stickSaturationLogged && logEvents) {
            stickSaturationLogged = true;
            Debug.LogWarning($"[COMPACT STICK SATURATED] slope={activeFrame.collider.name} " +
                $"progress={slopeProgressPercent:F3}% speed={currentSpeed:F4} required={unsaturatedTarget:F3} " +
                $"limit={maximumAdaptiveStickAcceleration:F3}", this);
        }
        previousStickAcceleration = StepAsymmetricJerkLimited(previousStickAcceleration, target,
            stickRiseJerkLimit, stickFallJerkLimit, dt);
        return previousStickAcceleration;
    }
    float CalculateGravitySupport(Vector3 normal) {
        return Mathf.Max(0f, Vector3.Dot(Physics.gravity, -normal.normalized));
    }
    static float CriticalDemand(float speed, float curvature) {
        float nonnegativeSpeed = Mathf.Max(0f, speed);
        return nonnegativeSpeed * nonnegativeSpeed * Mathf.Max(0f, curvature);
    }
    static float CriticalRatio(float demand, float availableSupport) {
        return availableSupport > Epsilon ? demand / availableSupport : 0f;
    }
    void UpdateCriticalReadout(float speed, float stick, float gravitySupport) {
        currentCriticalRatio = CriticalRatio(CriticalDemand(speed, currentCurvature), gravitySupport + stick);
    }
    // ---------------------------------------------------------------------
    // Natural release
    // ---------------------------------------------------------------------
    void UpdateReleasePlan(float progress01) {
        float target01 = targetPlanValid ? capturedTargetProgress01 : Mathf.Clamp01(targetSlopeProgressPercent * 0.01f);
        releaseStartProgress01 = Mathf.Clamp01(target01 + releaseHoldAfterTargetPercent * 0.01f);
        float configuredEnd01 = Mathf.Max(naturalReleaseProgressPercent * 0.01f,
            releaseStartProgress01 + minimumReleaseWidthPercent * 0.01f);
        releaseEndProgress01 = Mathf.Clamp01(configuredEnd01);
        if (!useNaturalRelease || progress01 < releaseStartProgress01)
            releasePhase = ReleasePhase.Controlled;
        else if (progress01 < releaseEndProgress01)
            releasePhase = ReleasePhase.Releasing;
        else
            releasePhase = ReleasePhase.Released;
        naturalMotionReleased = releasePhase == ReleasePhase.Released;
    }
    float EvaluateReleaseWeight(float progress01) {
        if (!useNaturalRelease) {
            currentReleaseWeight = 1f;
            return 1f;
        }
        currentReleaseWeight = 1f - SmoothRange01(progress01, releaseStartProgress01, releaseEndProgress01);
        return currentReleaseWeight;
    }
    // ---------------------------------------------------------------------
    // Command application
    // ---------------------------------------------------------------------
    void ApplySurfaceAcceleration(SurfaceBasis basis, float tangentialAcceleration,
        float inwardAcceleration, float lateralSpeed) {
        if (!basis.valid || rb == null)
            return;
        float lateralAcceleration = -lateralSpeed / Mathf.Max(0.01f, lateralResponseSeconds);
        Vector3 acceleration = basis.tangent * tangentialAcceleration + basis.side * lateralAcceleration -
            basis.normal * inwardAcceleration;
        rb.AddForce(acceleration, ForceMode.Acceleration);
        currentTangentialAcceleration = tangentialAcceleration;
        currentStickAcceleration = inwardAcceleration;
    }
    void SolveAir(Vector3 desiredMove) {
        groundKind = GroundKind.Air;
        bridgeActive = false;
        if (!naturalMotionReleased && desiredMove.sqrMagnitude > Epsilon) {
            rb.AddForce( desiredMove * airAcceleration, ForceMode.Acceleration);
        }
    }
    void ClearGroundState() {
        activeSlopeName = string.Empty;
        stableSlopeFrames = 0;
        stableSlopeDistance = 0f;
        supportMissDistance = 0f;
        surfaceGraceActive = false;
        ClearSurfaceCandidate();
        lastMeasuredSlopeObservation = default;
        currentStickAcceleration = 0f;
        currentTangentialAcceleration = 0f;
        currentCriticalRatio = 0f;
        currentCurvature = 0f;
        bool released = naturalMotionReleased; ResetSlopeSession(false); naturalMotionReleased = released;
    }
    void ResetSlopeSession(bool clearFrame) {
        targetPhase = TargetPhase.Inactive;
        releasePhase = ReleasePhase.Controlled;
        targetPlanValid = false;
        hadProgressObservation = false;
        slopeProgressPercent = 0f;
        previousProgress01 = 0f;
        previousTargetAcceleration = 0f;
        previousStickAcceleration = 0f;
        currentReleaseWeight = 1f;
        targetCrossingEstimatedTime = 0f;
        targetCrossingSampleOvershootPercent = 0f;
        postTargetGravityCompensationWeight = 0f;
        postTargetGravityCompensationAcceleration = 0f;
        capturedTargetProgress01 = Mathf.Clamp01(targetSlopeProgressPercent * 0.01f);
        capturedTargetSpeed = maxGroundSpeed;
        stickSaturationLogged = false;
        naturalMotionReleased = false;
        bridgeActive = false;
        entryDistance = 0f;
        entryWeight = 0f;
        entryPreloadAcceleration = 0f;
        if (clearFrame)
            activeFrame = default;
    }
    // ---------------------------------------------------------------------
    // Scalar helpers
    // ---------------------------------------------------------------------
    float CalculateObserverDistanceIncrement(float dt) {
        float referenceSpeed = Mathf.Max( 0.25f, maxGroundSpeed * 0.10f);
        float speed = rb != null ? rb.velocity.magnitude : 0f;
        return Mathf.Max(speed, referenceSpeed) * Mathf.Max(0f, dt);
    }

    float CalculateSurfaceTravelDistance( SlopeFrame frame, float dt) {
        if (!frame.valid || rb == null)
            return 0f;

        float forwardSpeed = Mathf.Abs(Vector3.Dot( rb.velocity, frame.axis));
        return forwardSpeed * Mathf.Max(0f, dt);
    }

    static float StepJerkLimited( float current, float target, float jerkLimit, float dt) {
        float maximumDelta = Mathf.Max(0f, jerkLimit) * Mathf.Max(0f, dt);
        return Mathf.MoveTowards(current, target, maximumDelta);
    }
    static float StepAsymmetricJerkLimited( float current, float target, float riseJerk, float fallJerk, float dt) {
        float jerk = target >= current ? riseJerk : fallJerk;
        return StepJerkLimited(current, target, jerk, dt);
    }
    static float RequiredAccelerationForSpeedChange( float currentSpeed, float targetSpeed, float distance) {
        float safeDistance = Mathf.Max(0.0001f, distance);
        return (targetSpeed * targetSpeed - currentSpeed * currentSpeed) / (2f * safeDistance);
    }
    static float ReachableSpeedBeforeConstraint( float terminalSpeed, float deceleration, float distance) {
        float squared = terminalSpeed * terminalSpeed + 2f * Mathf.Max(0f, deceleration) * Mathf.Max(0f, distance);
        return Mathf.Sqrt(Mathf.Max(0f, squared));
    }
    static float SmootherStep01(float value) {
        float t = Mathf.Clamp01(value);
        return t * t * t * (t * (t * 6f - 15f) + 10f);
    }

    static float SmoothRange01(float value, float start, float end) {
        if (end <= start + Epsilon)
            return value >= end ? 1f : 0f;
        return SmootherStep01(Mathf.InverseLerp(start, end, value));
    }
    static Vector3 NormalizeFlat( Vector3 value, Vector3 fallback) {
        Vector3 flat = Vector3.ProjectOnPlane(value, Vector3.up);
        if (flat.sqrMagnitude <= Epsilon)
            flat = Vector3.ProjectOnPlane(fallback, Vector3.up);
        if (flat.sqrMagnitude <= Epsilon)
            flat = Vector3.forward;
        return flat.normalized;
    }
    // ---------------------------------------------------------------------
    // Diagnostics
    // ---------------------------------------------------------------------
    void LogPeriodic() {
        if (!logPeriodicState)
            return;
        if (Time.frameCount < nextPeriodicLogFrame)
            return;
        nextPeriodicLogFrame = Time.frameCount + periodicLogEveryFixedFrames;
        Debug.Log( $"[COMPACT TRACE] ground={groundKind} " + $"source={observationSource} " + $"slope={activeSlopeName} " + $"progress={slopeProgressPercent:F3}% " +
            $"speed={tangentSpeed:F4} " + $"targetSpeed={targetTangentSpeed:F4} " + $"accel={currentTangentialAcceleration:F4} " +
            $"stick={currentStickAcceleration:F4} " + $"ratio={currentCriticalRatio:F5} " +
            $"release={currentReleaseWeight:F5} " + $"stableDistance={stableSlopeDistance:F4} " +
            $"candidate={surfaceCandidateName} " + $"candidateDistance={surfaceCandidateDistance:F4} " +
            $"candidateScore={surfaceCandidateScore:F4} " + $"supportMissDistance={supportMissDistance:F4} " +
            $"gravityFFWeight={postTargetGravityCompensationWeight:F4} " +
            $"gravityFFAccel={postTargetGravityCompensationAcceleration:F4} " + $"phase={targetPhase}/{releasePhase}",
            this);
    }
    void OnDrawGizmosSelected() {
        if (activeFrame.valid) {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(activeFrame.entryPoint, 0.12f);
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(activeFrame.exitPoint, 0.12f);
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine( activeFrame.entryPoint, activeFrame.exitPoint);
            float target01 = Mathf.Clamp01( targetSlopeProgressPercent * 0.01f);
            Vector3 targetPoint = Vector3.Lerp( activeFrame.entryPoint, activeFrame.exitPoint, target01);
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(targetPoint, 0.16f);
        }
        if (forwardFrame.valid) {
            Gizmos.color = Color.magenta;
            Gizmos.DrawLine( forwardFrame.entryPoint, forwardFrame.exitPoint);
        }
    }
}
