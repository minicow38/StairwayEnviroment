using UnityEngine;

public sealed class BallVisualEqualizerSync : MonoBehaviour
{
    // Experiment sampling constants.  Keep them internal so precision improves
    // without expanding the Inspector parameter surface.
    private const float ExperimentMinimumContactAlignment = 0.95f;
    private const float ExperimentMinimumTransportRetention = 0.95f;
    private const float StableNormalZeroCrossVelocityEpsilon = 0.01f;
    private const float GeometryPeriodMinimumSpeed = 0.05f;

    // ================================================================
    // Hybrid state
    // ================================================================

    public enum EqualizerPhase
    {
        Synchronized,
        ReleaseArmed,
        FreeFlight,
        LowerContact,
        HopperFlight,
        UpperContact,
        Reacquiring
    }
    


    [System.Serializable]
    private struct OscillationFrame
    {
        public bool valid;
        public bool fromEnvelope;

        public Vector3 origin;
        public Vector3 tangent;
        public Vector3 normal;
        public Vector3 lateral;
    }


    [System.Serializable]
    private struct ReleaseFrame
    {
        public Vector3 position;

        // RegainBall用:
        // Equalizer Release時点におけるSubject位置。
        public Vector3 subjectPosition;

        public Vector3 velocity;
        public Vector3 angularVelocity;

        public float speed;
        public float kineticEnergy;

        public float sourceEnergy;
        public float envelopeEntryHeight;
        public Vector3 sourceEnergyAxis;
        public float canonicalNormalAcceleration;
    }


    [System.Serializable]
    private struct ReleaseFeasibility
    {
        public bool valid;
        public bool limitAvailable;

        public Vector3 limitCenterVisual;

        public float initialOscillationEnergy;
        public float initialOscillationRatio;

        public float remainingTransportDistance;
        public float availableTime;

        public float estimatedCycleTime;
        public float availableCycles;
        public float predictedResidualOscillationEnergy;
        public float dampingFeasibility01;

        public float initialSubjectGap;
        public float subjectGap;
        public float minimumSubjectClosingTime;
        public float subjectConvergenceFeasibility;

        public float observedSameBoundaryPeriod;
    }


    [System.Serializable]
    private struct ContactFrame
    {
        public bool valid;
        public bool envelopeContact;
        public bool stairLikeContact;

        public Vector3 point;

        // Raw Unity contact normal and the impact-oriented normal.
        //
        // impact normal is always oriented so that
        //
        //     dot(v_incident, n) <= 0
        //
        // and therefore inwardNormalSpeed is always the magnitude of the
        // incoming normal component, regardless of MeshCollider winding.
        public Vector3 rawNormal;
        public Vector3 normal;
        public bool normalWasFlipped;

        public Vector3 incidentVelocity;
        public Vector3 tangentVelocity;
        public Vector3 inwardNormalVelocity;

        public float speed;
        public float tangentSpeed;
        public float inwardNormalSpeed;
        public float outwardNormalSpeedAfterSolver;

        public float incidenceAngleDeg;

        public float totalKineticEnergy;
        public float tangentEnergy;
        public float normalEnergy;
        public float normalEnergyRatio;

        public float estimatedRestitution;
        public float normalImpulse;

        // Impact Map result:
        //
        //     v+ = v_t + e_eff * v_in * n
        //
        // where e_eff is additionally capped by the single canonical
        // Stable-N energy reservoir.
        public bool impactMapApplied;
        public Vector3 mappedOutgoingVelocity;
        public float mappedOutgoingNormalSpeed;
        public float mappedOutgoingNormalEnergy;
        public float mappedNormalEnergyRatio;
        public float mappedEffectiveRestitution;
        public float previousCanonicalNormalEnergy;
        public float canonicalEnergyCeiling;
        public float reservoirEnergyRatio;

        // Stable oscillation-frame diagnostics.
        public bool oscillationFrameValid;
        public Vector3 oscillationNormal;
        public Vector3 oscillationOutgoingAxis;
        public float oscillationIncomingSpeed;
        public float oscillationIncomingEnergy;
        public float oscillationIncomingEnergyRatio;
        public float contactOscillationAlignment;
        public float contactConstraintCorrectionSpeed;

        // Clean-impact diagnostics.
        public float preTransportSpeed;
        public float postTransportSpeed;
        public float transportRetention;
        public float requestedOutgoingOscillationSpeed;
        public float finalOutgoingOscillationSpeed;
        public float finalOutgoingOscillationEnergy;
        public float contactWallness;
        public float finalSeparationSpeed;
        public bool forwardGuardApplied;
        public bool oscillationReducedByConstraint;
        public bool severeTransportLoss;

        // Contact-selection / quality diagnostics.
        public float contactAuthority;
        public float contactApproach01;
        public float contactForwardCompatibility01;
        public float contactPredictedSeparationSpeed;
        public bool canonicalContact;
        public bool emergencyContactGuard;
        public bool physicsClean;
        public bool dampingSuccess;
        public bool transportSuccess;
        public bool subjectConverging;
        public bool gameImpactSuccess;

    }


    private struct ContactSelectionResult
    {
        public ContactPoint contact;
        public bool canonicalContact;
        public float authority01;
        public float approach01;
        public float oscillationAlignment01;
        public float wallness01;
        public float forwardCompatibility01;
        public float predictedSeparationSpeed;
        public float estimatedRequestedOscillationSpeed;
    }


    // ================================================================
    // References
    // ================================================================

    [Header("References")]
    [SerializeField]
    private Rigidbody ballVisual;

    [Tooltip(
        "Spline版InSubject。EqualizerからはREAD ONLYで参照します。")]
    [SerializeField]
    private SlopeStickCore slopeCore;

    [Tooltip(
        "PhysicsRoot上のInSubjectをVisualPlayerRoot側へ写す正式なSubject座標変換です。")]
    [SerializeField]
    private CorrespondSubject correspondSubject;

    [Tooltip(
        "互換用のSubject Transform。Spline版ではCorrespondSubject.MappedPositionを優先します。")]
    [SerializeField]
    private Transform subjectTransform;

    [Tooltip(
        "互換用のSubject Rigidbody。Spline版ではCorrespondSubject.MappedPhysicalVelocityを優先します。")]
    [SerializeField]
    private Rigidbody subjectBody;

    [SerializeField]
    private Rigidbody ballVisualEqualizer;

    [SerializeField]
    private SphereCollider ballVisualEqualizerCollider;

    [SerializeField]
    private BallVisualNegativeEnvelopeCollider negativeEnvelope;

    // Dedicated Equalizer channel invariant.
    // UpperはNegativeEnvelope、LowerはScene上の実Stairway Collider。
    // InSubject / BallVisual / 非Stairway Stageとの衝突は除外する。
    // LowerGuide Colliderは生成・使用しない。

    // ================================================================
    // Impact Map + Energy Contraction
    // ================================================================
    //
    // The Equalizer is a hybrid system:
    //
    //   free flight -> impact -> free flight -> impact ...
    //
    // Releaseごとに固定したOscillation Normal n_oで減衰を測る。
    // 実Collider Normal n_cは非貫通制約だけを担当する。
    //
    // Canonical impact map:
    //
    //   v^- = v_planar^- - u^- d
    //   v*  = lambda_t v_planar^- + u^+ d
    //
    //   d = +n_o (Lower), -n_o (Upper)
    //   u^+ = e_eff u^-
    //
    // 最後にStable T/L/N基底の中でClean Impact制約を解く。
    //   - Tは逆転させない
    //   - OscillationはImpact Map要求値より増やさない
    //   - 実Collider Normal n_cに対して非貫通を満たす
    // 矛盾時はL -> Oscillationの順に削り、Tを守れない面はEmergencyへ送る。
    //
    //   0 <= e_eff < 1
    //
    // Therefore:
    //
    //   E_n^+ = e_eff^2 E_n^-
    //
    // A single canonical Stable-N reservoir prevents acceleration / geometry
    // from re-inflating the rebound between impacts:
    //
    //   E_(k+1)^+ <= e^2 * E_k^+
    //
    // There is no second same-boundary contraction path.
    // ================================================================

    [Header("Impact Map + Energy Contraction")]

    [Tooltip(
        "各Oscillation impactで基準法線速度へ掛ける反発率 e。"
        + "局所Collider角度ではなくRelease固定のOscillation Frameで収縮させます。")]
    [Range(0f, 0.999f)]
    [SerializeField]
    private float impactNormalRestitution =
        0.92f;

    // ================================================================
    // Subject convergence - N-orthogonal transport plane
    // ================================================================
    //
    //   v = v_plane + v_N N
    //   dot(v_plane, N) = 0
    //
    // N is owned exclusively by damping.
    // The complete T/L plane follows Subject position/velocity, so a Turn can
    // redirect transport without leaking correction into the oscillation axis.
    // No position teleport and no oscillation-normal catch-up is used.
    // ================================================================

    [Header("Subject Convergence")]

    [Tooltip(
        "SubjectとのN直交平面(T/L)距離誤差を速度差へ変換する時定数[s]。"
        + "N方向の減衰速度には一切作用しません。")]
    [Min(0.05f)]
    [SerializeField]
    private float subjectConvergenceTime =
        0.45f;

    [Tooltip(
        "Subject速度へ上乗せできるN直交平面の最大Closing Speed[m/s]。"
        + "T/Lだけに作用し、N方向の減衰は変更しません。")]
    [Min(0f)]
    [SerializeField]
    private float maximumTransportSpeedBoost =
        12f;

    [Tooltip(
        "EqualizerのN直交平面(T/L)速度をSubjectへ寄せる最大加速度[m/s^2]。")]
    [Min(0.01f)]
    [SerializeField]
    private float maximumTransportAcceleration =
        45f;

    [Tooltip(
        "このN直交平面距離内では位置誤差による速度上乗せを止め、"
        + "SubjectのT/L速度への一致だけを行います[m]。")]
    [Min(0f)]
    [SerializeField]
    private float subjectTransportDeadZone =
        0.15f;


    // Numerical safety only. These are not expression parameters.
    private const float MinimumImpactNormalSpeed =
        0.025f;

    private const float ImpactEnergyEpsilon =
        0.000001f;

    // Physics safety and game-quality thresholds.  They are diagnostics /
    // invariants for the single model, not selectable response modes.
    private const float MinimumCleanTransportRetention =
        0.35f;

    private const float MinimumGameTransportRetention =
        0.70f;

    private const float ContactAuthorityEpsilon =
        0.0001f;

    private const float CleanConstraintEpsilon =
        0.0005f;

    // Feasibility values are planning/diagnostic quantities for the same
    // single physical model.  They do not select another response mode.
    private const float MinimumFeasibilityTime =
        0.02f;

    private const float MaximumFeasibilityTime =
        8f;

    private const float MinimumObservedCyclePeriod =
        0.04f;

    // Period T is owned by BallVisualNegativeEnvelopeCollider Inspector.
    // Equalizer only enforces the resulting T/2 boundary-contact schedule.
    // 2 FixedUpdate / half-cycle is the minimum resolvable phase interval.
    private const int MinimumForcedBoundaryHalfCycleFixedSteps =
        2;


    // ================================================================
    // Runtime - Read Only
    // ================================================================

    [Header("Runtime - Read Only")]
    [SerializeField]
    private bool synchronized = true;

    private EqualizerPhase phase =
        EqualizerPhase.Synchronized;

    [Tooltip("Runtime表示のみ。物理状態を選択する設定ではありません。")]
    [SerializeField]
    private string phaseRuntime =
        "Synchronized";

    [SerializeField]
    private float phaseElapsed;

    [SerializeField]
    private ReleaseFrame releaseFrame;

    [Header("Canonical Energy Runtime - Read Only")]
    [SerializeField]
    private float canonicalReferenceHeight;

    [SerializeField]
    private Vector3 canonicalSourceAxis = Vector3.up;

    [SerializeField]
    private float canonicalNormalAcceleration;

    [SerializeField, Range(0f, 1f)]
    private float currentCanonicalEnergyRatio = 1f;

    [Header("Release Feasibility Runtime - Read Only")]
    [SerializeField]
    private ReleaseFeasibility releaseFeasibility;

    [SerializeField]
    private float lastLowerImpactTime = -1f;

    [SerializeField]
    private float lastUpperImpactTime = -1f;

    [SerializeField]
    private float smoothedSameBoundaryPeriod;

    [SerializeField]
    private float currentCanonicalOscillationEnergy;

    [Header("Period Reference Runtime - Read Only")]
    [Tooltip("常にfalse。T/2 deadline steeringは無効で、Tは観測基準だけに使います。")]
    [SerializeField]
    private bool forcedBoundaryDriveActive;

    [SerializeField]
    private float forcedBoundaryReleaseTime;

    [SerializeField]
    private float forcedBoundaryTimeCost;

    [SerializeField]
    private float forcedBoundaryPeriod;

    [SerializeField]
    private float forcedBoundaryHalfPeriod;

    [SerializeField]
    private float forcedBoundaryPhaseErrorSeconds;

    [SerializeField]
    private float forcedBoundaryAppliedPhaseAcceleration;

    [SerializeField]
    private float forcedBoundaryGamma;

    [SerializeField]
    private float forcedBoundaryRadiusClearanceScale;

    [SerializeField]
    private float forcedBoundaryPlannedReleaseSpan;

    [SerializeField]
    private float forcedBoundaryPlannedReleaseTargetNormalSpeed;

    [SerializeField]
    private float forcedBoundaryPlannedReleasePhaseAcceleration;

    [Header("Period / Phase Observation Runtime - Read Only")]
    [Tooltip("Tを1.0とした現在位相。Upper=0.00, Zero=0.25, Lower=0.50, Zero=0.75。")]
    [SerializeField, Range(0f, 1f)]
    private float oscillationPhase01 = 0.5f;

    [Tooltip("16->8実験の現在サンプル番号n。Releaseをまたいで0..8を保持します。")]
    [SerializeField]
    private int oscillationCycleIndex;

    [Tooltip("現在ReleaseでCanonical Upper + 実Stairwayの振幅サンプルを確定済みか。")]
    [SerializeField]
    private bool experimentSampleCompletedThisRelease;

    [Tooltip("次Release開始時にnを1つ進める予約。Envelope途中でTだけを変更しないため遅延適用します。")]
    [SerializeField]
    private bool experimentCycleAdvancePending;

    [Tooltip("8Tサンプルまで取得済み。以後は位相観測だけ継続します。")]
    [SerializeField]
    private bool maxGroundSpeedExperimentCompleted;

    [SerializeField]
    private float oscillationPhaseOriginTime;

    [SerializeField]
    private float oscillationPhaseOrigin01 = 0.5f;

    [SerializeField]
    private bool oscillationPhaseAnchorValid;

    [Tooltip("実境界接触の位相誤差 / T。符号付き。")]
    [SerializeField]
    private float normalizedPhaseError01;

    [SerializeField]
    private float observedUpperPeriod;

    [SerializeField]
    private float observedLowerPeriod;

    [SerializeField]
    private float upperPeriodError01;

    [SerializeField]
    private float lowerPeriodError01;

    [Header("Stable-N Amplitude Runtime - Read Only")]
    [SerializeField]
    private bool hasReferenceUpperExtremum;

    [SerializeField]
    private bool lowerSeenAfterReferenceUpper;

    [SerializeField]
    private float upperExtremumStableN;

    [SerializeField]
    private float lowerExtremumStableN;

    [Tooltip("A_n = |xMax - xMin| / 2。Stable-N座標で測定。")]
    [SerializeField]
    private float measuredAmplitude;

    [SerializeField]
    private float previousMeasuredAmplitude;

    [Tooltip("r_n = A_n / A_(n-1)。最初の周期は1。")]
    [SerializeField]
    private float amplitudeDecayRatio = 1f;

    [SerializeField]
    private int amplitudeCycleIndex = -1;

    [Header("Stable-N Zero Crossing Runtime - Read Only")]
    [Tooltip("FixedUpdate間のStable-N相対速度の符号反転で極値を確定します。衝突座標だけを極値扱いしません。")]
    [SerializeField]
    private bool stableNormalSampleValid;

    [SerializeField]
    private float previousStableNormalCoordinate;

    [SerializeField]
    private float previousStableNormalVelocity;

    [SerializeField]
    private float previousStableNormalSampleTime;

    [SerializeField]
    private bool pendingUpperExtremumValidation;

    [SerializeField]
    private bool pendingLowerExtremumValidation;

    [SerializeField]
    private float pendingExtremumImpactTime;

    [SerializeField]
    private float lastZeroCrossingStableN;

    [SerializeField]
    private float lastZeroCrossingTime;

    [SerializeField]
    private string lastZeroCrossingKind = "None";

    [Header("Stair Geometry Period Runtime - Read Only")]
    [SerializeField]
    private bool hasPreviousAcceptedStairContact;

    [SerializeField]
    private Vector3 previousAcceptedStairPoint;

    [SerializeField]
    private float previousAcceptedStairTangentSpeed;

    [SerializeField]
    private float observedStairSpatialInterval;

    [SerializeField]
    private float observedGeometryPeriod;

    [SerializeField]
    private int observedSpatialPeriodMultiplicity = 1;

    [Tooltip("実験データへ採用したCanonical Stair接触数。物理衝突総数とは別です。")]
    [SerializeField]
    private int acceptedExperimentStairContactCount;

    [Header("maxGroundSpeed / Period Experiment Runtime - Read Only")]
    [Tooltip("SlopeStickCore.maxGroundSpeedをEnvelope経由でREAD ONLY取得したV0。")]
    [SerializeField]
    private float sourceMaxGroundSpeedReadOnly;

    [Tooltip("V(n)=V0*(1/2)^(n/8)。Coreには書き戻しません。")]
    [SerializeField]
    private float plannedMaxGroundSpeedForCycle;

    [SerializeField]
    private float plannedMaxGroundSpeedRatio = 1f;

    [SerializeField]
    private bool usingBallVisualAsSubjectProxy;

    [SerializeField]
    private ContactFrame lastContactFrame;

    [SerializeField]
    private int lowerContactCount;

    [SerializeField]
    private int upperContactCount;

    [SerializeField]
    private float positionErrorToBallVisual;

    [SerializeField]
    private float velocityErrorToBallVisual;

    [SerializeField]
    private float currentKineticEnergy;

    [Header("Oscillation Frame Runtime - Read Only")]

    [SerializeField]
    private OscillationFrame oscillationFrame;

    [SerializeField]
    private float releaseOscillationEnergy;

    [SerializeField]
    private float releaseOscillationEnergyRatio;

    [SerializeField]
    private float lastContactConstraintCorrectionSpeed;

    [Header("Impact Map Runtime - Read Only")]

    [SerializeField]
    private int impactCount;

    [SerializeField]
    private int lowerImpactCount;

    [SerializeField]
    private int upperImpactCount;

    [SerializeField]
    private float lastMappedNormalEnergy;

    [SerializeField]
    private float lastImpactNormalEnergyRatio;

    [SerializeField]
    private float lastReservoirEnergyRatio;

    [SerializeField]
    private float lastEffectiveRestitution;

    [SerializeField]
    private Vector3 lastMappedOutgoingVelocity;

    [Header("Subject Convergence Runtime - Read Only")]

    [SerializeField]
    private bool subjectConvergenceActive;

    [SerializeField]
    private string subjectVelocitySource =
        "Unavailable";

    [SerializeField]
    private float subjectDistance;

    [SerializeField]
    private float subjectTransportGap;

    [SerializeField]
    private float subjectTransportSpeed;

    [SerializeField]
    private float equalizerTransportSpeed;

    [SerializeField]
    private float desiredTransportSpeed;

    [SerializeField]
    private float appliedTransportDeltaSpeed;

    [SerializeField]
    private Vector3 estimatedSubjectVelocity;

    [Header("Impact Quality Runtime - Read Only")]

    [SerializeField]
    private int physicsCleanCount;

    [SerializeField]
    private int canonicalImpactCount;

    [SerializeField]
    private int emergencyImpactCount;

    [SerializeField]
    private int gameImpactSuccessCount;

    [SerializeField]
    private int forwardGuardCount;

    [SerializeField]
    private int oscillationConstraintReductionCount;

    [SerializeField]
    private int severeTransportLossCount;

    [SerializeField]
    private float physicsCleanRate;

    [SerializeField]
    private float gameImpactSuccessRate;

    [SerializeField]
    private float averageGameImpactQuality;

    [SerializeField]
    private float lastGameImpactQuality;

    private float accumulatedGameImpactQuality;

    [SerializeField]
    private float lastPreTransportSpeed;

    [SerializeField]
    private float lastPostTransportSpeed;

    [SerializeField]
    private float lastTransportRetention;

    [SerializeField]
    private float lastContactWallness;

    [SerializeField]
    private float lastFinalSeparationSpeed;

    [SerializeField]
    private float lastFinalOscillationEnergy;

    [Header("Release Success Runtime - Read Only")]
    [SerializeField]
    private bool releaseEvaluationActive;

    [SerializeField]
    private bool directSubjectObservedThisRelease;

    [SerializeField]
    private int releaseDampingViolationCount;

    [SerializeField]
    private float minimumReleaseTransportRetention = 1f;

    [SerializeField]
    private bool releaseDampingSuccess;

    [SerializeField]
    private bool releaseTransportSuccess;

    [SerializeField]
    private bool releaseSubjectSuccess;

    [SerializeField]
    private bool releaseOverallSuccess;

    [SerializeField]
    private int completedReleaseCount;

    [SerializeField]
    private int successfulReleaseCount;

    [SerializeField]
    private float releaseOverallSuccessRate;

    [SerializeField]
    private float finalReleaseSubjectGap;

    private Vector3 previousSubjectPosition;
    private bool hasSubjectPositionSample;


    // ================================================================
    // Public read only
    // ================================================================
    public Rigidbody Body =>
        ballVisualEqualizer;


    public bool TryGetRegainOscillationFrame(
        out Vector3 equalizerReleasePositionVisual,
        out Vector3 subjectReleasePositionVisual,
        out Vector3 oscillationNormalVisual)
    {
        equalizerReleasePositionVisual =
            releaseFrame.position;

        subjectReleasePositionVisual =
            releaseFrame.subjectPosition;

        oscillationNormalVisual =
            Vector3.up;

        if (synchronized ||
            !ballVisualEqualizer ||
            !oscillationFrame.valid)
        {
            return false;
        }

        Vector3 normal =
            oscillationFrame.normal;

        if (normal.sqrMagnitude <=
            ImpactEnergyEpsilon)
        {
            return false;
        }

        oscillationNormalVisual =
            normal.normalized;

        return true;
    }
    public bool IsSynchronized =>
        synchronized;

    public EqualizerPhase Phase =>
        phase;

    public float EqualizerMass =>
        ballVisualEqualizer
            ? Mathf.Max(
                0.0001f,
                ballVisualEqualizer.mass)
            : 1f;

    public float PositionErrorToBallVisual =>
        positionErrorToBallVisual;

    public float VelocityErrorToBallVisual =>
        velocityErrorToBallVisual;

    public float CurrentKineticEnergy =>
        currentKineticEnergy;

    public float SubjectTransportGap =>
        subjectTransportGap;

    public float SubjectDistance =>
        subjectDistance;

    // Compatibility alias for existing Inspector / external diagnostics.
    public float CleanImpactRate =>
        physicsCleanRate;

    public float PhysicsCleanImpactRate =>
        physicsCleanRate;

    public float GameImpactSuccessRate =>
        gameImpactSuccessRate;

    public float ReleaseOverallSuccessRate =>
        releaseOverallSuccessRate;

    public float DampingFeasibility01 =>
        releaseFeasibility.dampingFeasibility01;

    public float SubjectConvergenceFeasibility =>
        releaseFeasibility.subjectConvergenceFeasibility;

    public float AvailableTimeToLimit =>
        releaseFeasibility.availableTime;

    public float AverageGameImpactQuality =>
        averageGameImpactQuality;

    public float OscillationPhase01 =>
        oscillationPhase01;

    public int OscillationCycleIndex =>
        oscillationCycleIndex;

    public float MeasuredAmplitude =>
        measuredAmplitude;

    public float AmplitudeDecayRatio =>
        amplitudeDecayRatio;

    public float PlannedMaxGroundSpeedForCycle =>
        plannedMaxGroundSpeedForCycle;


    // ================================================================
    // Unity
    // ================================================================

    private void Start()
    {
        Debug.Log("######## BallVisualEqualizerSync START ########");
        ResolveReferences();
        ResetMaxGroundSpeedPeriodExperimentState();
        RefreshVisualCollisionOwnership();
        InitializeSubjectMotionEstimate();

        if (usingBallVisualAsSubjectProxy)
        {
            Debug.LogWarning(
                "[EQUALIZER SUBJECT] Subject was not found. " +
                "BallVisual is used as an explicit transport proxy. " +
                "Assign Subject Transform for direct Subject-time-course convergence.",
                this);
        }
        else if (!subjectTransform)
        {
            Debug.LogWarning(
                "[EQUALIZER SUBJECT] No convergence target was found. " +
                "Damping remains active, but T-direction convergence is unavailable.",
                this);
        }

        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            Debug.LogError(
                "[EQUALIZER] References are missing.",
                this);

            return;
        }

        ConfigureCollisionIgnore();

        // ------------------------------------------------------------
        // Initial ownership:
        // BallVisual owns the Equalizer completely.
        // ------------------------------------------------------------

        synchronized = true;

        ballVisualEqualizer.useGravity =
            false;

        ballVisualEqualizer.detectCollisions =
            false;

        ballVisualEqualizer.isKinematic =
            true;

        ResetOscillationFrame();
        ResetImpactMapState();
        ResetReleaseFeasibility();
        CopyBallVisualPose();

        TransitionTo(
            EqualizerPhase.Synchronized,
            "Start");

        LogPhysicMaterialState(
            "START");
    }


    private void FixedUpdate()
    {
        phaseElapsed +=
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f);

        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        if (usingBallVisualAsSubjectProxy)
        {
            // Subject may be spawned after Start / Release.  Retry only while
            // the explicit BallVisual proxy is active.
            ResolveReferences();
        }

        if (releaseEvaluationActive &&
            !usingBallVisualAsSubjectProxy &&
            subjectTransform)
        {
            directSubjectObservedThisRelease =
                true;
        }

        UpdateSubjectMotionEstimate();

        // ------------------------------------------------------------
        // Synchronized:
        // Equalizer is not an independent plant yet.
        // BallVisual owns its pose.
        // ------------------------------------------------------------

        if (synchronized)
        {
            CopyBallVisualPose();
            UpdateObserver();
            return;
        }

        // ------------------------------------------------------------
        // Dynamic Hopper:
        // Unity Physics owns position.  Only the plane perpendicular to Stable-N
        // may follow Subject; the N scalar remains owned exclusively by damping.
        // ------------------------------------------------------------

        // Lower is the real Stairway; no synthetic LowerGuide departure arm exists.

        // Sample the post-previous-physics Stable-N state before applying this
        // frame's forces.  A boundary impulse appears here as a true sign reversal
        // between the previous and current relative Stable-N velocity samples.
        UpdateStableNormalZeroCrossingObservation();

        // E0 / H0 defines one Stable-N acceleration for this Release.
        // Gravity is compensated only on N so the total N acceleration is -aN;
        // tangent/lateral gravity remains untouched.
        ApplyCanonicalNormalAcceleration();

        ApplySubjectTransportConvergence();
        UpdateObserver();
        UpdateDynamicPhaseFromContacts();
        UpdateOscillationPhaseRuntime();
    }


    // ================================================================
    // Reference resolution
    // ================================================================

    private void ResolveReferences()
    {
        if (!ballVisualEqualizer)
        {
            ballVisualEqualizer =
                GetComponent<Rigidbody>();

            if (!ballVisualEqualizer)
            {
                GameObject equalizerObject =
                    GameObject.Find(
                        "/VisualPlayerRoot/BallVisualEqualizer");

                if (!equalizerObject)
                {
                    equalizerObject =
                        GameObject.Find(
                            "BallVisualEqualizer");
                }

                if (equalizerObject)
                {
                    ballVisualEqualizer =
                        equalizerObject.GetComponent<
                            Rigidbody>();
                }
            }
        }

        if (!ballVisualEqualizerCollider &&
            ballVisualEqualizer)
        {
            ballVisualEqualizerCollider =
                ballVisualEqualizer.GetComponent<
                    SphereCollider>();
        }

        if (!ballVisual)
        {
            GameObject ballVisualObject =
                GameObject.Find(
                    "BallVisual");

            if (ballVisualObject)
            {
                ballVisual =
                    ballVisualObject.GetComponent<
                        Rigidbody>();
            }
        }

        if (!slopeCore)
        {
            slopeCore =
                FindFirstObjectByType<
                    SlopeStickCore>();
        }

        if (!correspondSubject)
        {
            correspondSubject =
                FindFirstObjectByType<
                    CorrespondSubject>();
        }

        // Spline版ではCorrespondSubjectがSubjectの唯一の座標写像担当。
        // Transform/Rigidbody参照はInspector互換と非常時Fallbackとしてだけ保持する。
        if (correspondSubject)
        {
            if (correspondSubject.SubjectBody)
            {
                subjectBody =
                    correspondSubject.SubjectBody;

                subjectTransform =
                    subjectBody.transform;
            }
            else if (!subjectTransform)
            {
                subjectTransform =
                    correspondSubject.transform;
            }
        }

        if (!subjectTransform)
        {
            GameObject subjectObject =
                GameObject.Find(
                    "subject");

            if (!subjectObject)
            {
                subjectObject =
                    GameObject.Find(
                        "Subject");
            }

            if (subjectObject &&
                (!ballVisual ||
                 subjectObject.transform !=
                    ballVisual.transform))
            {
                subjectTransform =
                    subjectObject.transform;

                subjectBody =
                    subjectObject.GetComponent<
                        Rigidbody>();
            }
        }

        if (subjectTransform &&
            (!ballVisual ||
             subjectTransform !=
                ballVisual.transform) &&
            !subjectBody)
        {
            subjectBody =
                subjectTransform.GetComponent<
                    Rigidbody>();
        }

        usingBallVisualAsSubjectProxy =
            !correspondSubject &&
            ballVisual &&
            subjectTransform ==
                ballVisual.transform;

        if (!correspondSubject &&
            !subjectTransform &&
            ballVisual)
        {
            // Explicit legacy fallback only.
            subjectTransform =
                ballVisual.transform;

            subjectBody =
                ballVisual;

            usingBallVisualAsSubjectProxy =
                true;
        }

        if (!negativeEnvelope)
        {
            negativeEnvelope =
                FindFirstObjectByType<
                    BallVisualNegativeEnvelopeCollider>();
        }
    }


    private bool HasMappedSubject =>
        correspondSubject &&
        correspondSubject.InSubjectBody;


    private Vector3 ReadSubjectPositionVisual()
    {
        if (HasMappedSubject)
            return correspondSubject.MappedPosition;

        if (subjectTransform)
            return subjectTransform.position;

        return ballVisual
            ? ballVisual.position
            : transform.position;
    }


    private Vector3 ReadSubjectVelocityVisual()
    {
        if (HasMappedSubject)
        {
            // BallVisualSlopeDriveと同じ「InSubject物理速度のVisual写像」。
            // Visual frameの公転微分はTransport Energyへ混ぜない。
            return correspondSubject.MappedPhysicalVelocity;
        }

        if (subjectBody &&
            (!ballVisual ||
             subjectBody != ballVisual))
        {
            return subjectBody.velocity;
        }

        return ballVisual
            ? ballVisual.velocity
            : Vector3.zero;
    }


    private void ConfigureCollisionIgnore()
    {
        if (!ballVisual ||
            !ballVisualEqualizerCollider)
        {
            return;
        }

        SphereCollider ballVisualCollider =
            ballVisual.GetComponent<
                SphereCollider>();

        if (!ballVisualCollider)
            return;

        Physics.IgnoreCollision(
            ballVisualCollider,
            ballVisualEqualizerCollider,
            true);
    }


    // ================================================================
    // Normal synchronization
    // ================================================================

    public void Equalize()
    {
        if (!synchronized)
            return;

        CopyBallVisualPose();
    }


    private void CopyBallVisualPose()
    {
        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        // ------------------------------------------------------------
        // Kinematic synchronized state:
        //
        // Do not write velocity / angularVelocity here.
        // Unity rejects velocity assignment to a kinematic Rigidbody.
        //
        // Synchronization means:
        //
        //     x_E = x_B
        //     q_E = q_B
        //
        // Velocity is captured immediately before Release and is then
        // assigned after the Equalizer becomes Dynamic.
        // ------------------------------------------------------------

        ballVisualEqualizer.transform.SetPositionAndRotation(
            ballVisual.position,
            ballVisual.rotation);
    }


    // ================================================================
    // BallVisual -> Equalizer Handoff
    // ================================================================

    // Legacy 3-argument entry point kept for existing callers.
    public bool ReleaseToEnvelopeSimulation(
        Vector3 equalizerLaunchVelocity,
        float sourceEnergyJoule,
        float envelopeEntryHeight)
    {
        Vector3 subjectVelocity =
            ReadSubjectVelocityVisual();

        Vector3 inferredAxis =
            equalizerLaunchVelocity -
            subjectVelocity;

        if (inferredAxis.sqrMagnitude <=
            ImpactEnergyEpsilon)
        {
            inferredAxis =
                Vector3.up;
        }

        return ReleaseToEnvelopeSimulation(
            equalizerLaunchVelocity,
            sourceEnergyJoule,
            envelopeEntryHeight,
            inferredAxis);
    }


    // Canonical BallVisualSlopeDrive handoff:
    //
    //   E0 : BallVisual source energy
    //   H0 : Stable-N reference height
    //   v  : Equalizer world/visual release velocity
    //   n0 : source energy axis in Visual coordinates
    //
    // Equalizer:
    //   aN = E0 / (m H0)
    //
    // Envelope:
    //   epsilon = E / E0
    //   A = H0 * epsilon
    public bool ReleaseToEnvelopeSimulation(
        Vector3 equalizerLaunchVelocity,
        float sourceEnergyJoule,
        float envelopeEntryHeight,
        Vector3 sourceEnergyAxisVisual)
    {
        ResolveReferences();

        // A new BallVisual Incident is the reset map for the previous hopper.
        if (!synchronized)
            ReacquireForNextIncident();

        if (!synchronized)
        {
            Debug.LogWarning(
                "[EQUALIZER] Reacquire for next incident failed.",
                this);

            return false;
        }

        if (!ballVisual ||
            !ballVisualEqualizer ||
            !negativeEnvelope)
        {
            Debug.LogError(
                "[EQUALIZER] Release references are missing.",
                this);

            return false;
        }

        // Stage generation may have produced new Physics-only colliders since
        // Start.  Exclude them before this Rigidbody becomes Dynamic.
        RefreshVisualCollisionOwnership();

        float safeEnergy =
            Mathf.Max(
                0f,
                sourceEnergyJoule);

        float safeReferenceHeight =
            Mathf.Max(
                0f,
                envelopeEntryHeight);

        if (safeEnergy <=
            ImpactEnergyEpsilon)
        {
            Debug.LogWarning(
                "[EQUALIZER] Source Energy is zero.",
                this);

            return false;
        }

        if (safeReferenceHeight <=
            ImpactEnergyEpsilon)
        {
            Debug.LogWarning(
                "[EQUALIZER] Stable-N reference height is zero.",
                this);

            return false;
        }

        if (equalizerLaunchVelocity.sqrMagnitude <=
            ImpactEnergyEpsilon)
        {
            Debug.LogWarning(
                "[EQUALIZER] Launch velocity is zero.",
                this);

            return false;
        }

        Vector3 sourceAxis =
            sourceEnergyAxisVisual;

        if (sourceAxis.sqrMagnitude <=
            ImpactEnergyEpsilon)
        {
            sourceAxis =
                equalizerLaunchVelocity -
                ReadSubjectVelocityVisual();
        }

        if (sourceAxis.sqrMagnitude <=
            ImpactEnergyEpsilon)
        {
            sourceAxis =
                Vector3.up;
        }

        sourceAxis.Normalize();

        if (Vector3.Dot(
                sourceAxis,
                Vector3.up) < 0f)
        {
            sourceAxis =
                -sourceAxis;
        }

        // ------------------------------------------------------------
        // High-N canonical release.
        //
        // E0 is the authoritative Stable-N energy at handoff:
        //
        //     vN0 = sqrt(2 E0 / m)
        //     aN  = E0 / (m H0)
        //
        // Therefore the ballistic Stable-N turning height is invariant:
        //
        //     vN0^2 / (2 aN) = H0
        //
        // Increasing N speed therefore increases the oscillation rate/force,
        // not the geometric excursion.  The first actual Collider impact is
        // the first restitution event; no synthetic pre-damping is applied.
        // T/L transport is preserved exactly and only the supplied N component
        // is replaced by the energy-consistent Stable-N component.
        // ------------------------------------------------------------

        float initialReleaseEnergyRatio =
            1f;

        float initialReleaseOscillationEnergy =
            safeEnergy;

        Vector3 transportLateralVelocity =
            Vector3.ProjectOnPlane(
                equalizerLaunchVelocity,
                sourceAxis);

        float canonicalLaunchNormalSpeed =
            Mathf.Sqrt(
                Mathf.Max(
                    0f,
                    2f *
                    initialReleaseOscillationEnergy /
                    EqualizerMass));

        Vector3 canonicalLaunchVelocity =
            transportLateralVelocity +
            sourceAxis *
            canonicalLaunchNormalSpeed;

        // Exact synchronization immediately before ownership transfer.
        CopyBallVisualPose();

        Vector3 releasePosition =
            ballVisual.position;

        Quaternion releaseRotation =
            ballVisual.rotation;

        Vector3 releaseAngularVelocity =
            ballVisual.angularVelocity;

        float releaseSpeed =
            canonicalLaunchVelocity.magnitude;

        canonicalReferenceHeight =
            safeReferenceHeight;

        canonicalSourceAxis =
            sourceAxis;

        canonicalNormalAcceleration =
            safeEnergy /
            Mathf.Max(
                ImpactEnergyEpsilon,
                EqualizerMass *
                safeReferenceHeight);

        currentCanonicalEnergyRatio =
            initialReleaseEnergyRatio;

        releaseFrame =
            new ReleaseFrame
            {
                position =
                    releasePosition,

                subjectPosition =
                    ReadSubjectPositionVisual(),

                velocity =
                    canonicalLaunchVelocity,

                angularVelocity =
                    releaseAngularVelocity,

                speed =
                    releaseSpeed,

                kineticEnergy =
                    0.5f *
                    EqualizerMass *
                    releaseSpeed *
                    releaseSpeed,

                sourceEnergy =
                    safeEnergy,

                envelopeEntryHeight =
                    safeReferenceHeight,

                sourceEnergyAxis =
                    sourceAxis,

                canonicalNormalAcceleration =
                    canonicalNormalAcceleration
            };

        TransitionTo(
            EqualizerPhase.ReleaseArmed,
            "ReleaseRequested");

        // A completed Upper+Stair sample advances the experiment only at the
        // NEXT Release.  This guarantees that one generated Envelope uses one
        // immutable T(n) from build to destruction.
        ApplyPendingMaxGroundSpeedExperimentAdvance();

        if (negativeEnvelope)
        {
            negativeEnvelope.SetMaxGroundSpeedExperimentCycle(
                oscillationCycleIndex);
        }

        UpdateMaxGroundSpeedPeriodExperimentRuntime();

        // Envelope must exist before the Equalizer becomes Dynamic.
        bool envelopeReady =
            negativeEnvelope.ArmFromBallVisualEnergy(
                safeEnergy,
                safeReferenceHeight,
                canonicalLaunchVelocity,
                sourceAxis,
                initialReleaseEnergyRatio);

        if (!envelopeReady)
        {
            TransitionTo(
                EqualizerPhase.Synchronized,
                "EnvelopeBuildFailed");

            Debug.LogWarning(
                "[EQUALIZER] Envelope could not be armed. Synchronization is kept.",
                this);

            return false;
        }

        // UpperEnvelope + real Stairway are now the Equalizer's exclusive
        // physical channel. Refresh pairwise ownership before Dynamic release.
        RefreshVisualCollisionOwnership();

        ResetImpactMapState();

        CaptureOscillationFrame(
            releasePosition,
            canonicalLaunchVelocity);

        if (!oscillationFrame.valid)
        {
            TransitionTo(
                EqualizerPhase.Synchronized,
                "OscillationFrameInvalid");

            Debug.LogWarning(
                "[EQUALIZER] Stable oscillation frame could not be captured.",
                this);

            return false;
        }

        synchronized =
            false;

        lowerContactCount =
            0;

        upperContactCount =
            0;

        // Lower is the real Stairway. The first genuine stair impact is valid.

        ballVisualEqualizer.detectCollisions =
            true;

        ballVisualEqualizer.isKinematic =
            false;

        ballVisualEqualizer.useGravity =
            true;

        ballVisualEqualizer.position =
            releasePosition;

        ballVisualEqualizer.rotation =
            releaseRotation;

        ballVisualEqualizer.velocity =
            canonicalLaunchVelocity;

        ballVisualEqualizer.angularVelocity =
            releaseAngularVelocity;

        ballVisualEqualizer.collisionDetectionMode =
            CollisionDetectionMode.ContinuousDynamic;

        // Numerical hardening for high Stable-N speed.  These are solver
        // quality floors, not gameplay tuning parameters.
        ballVisualEqualizer.solverIterations =
            Mathf.Max(
                ballVisualEqualizer.solverIterations,
                12);

        ballVisualEqualizer.solverVelocityIterations =
            Mathf.Max(
                ballVisualEqualizer.solverVelocityIterations,
                4);

        ballVisualEqualizer.WakeUp();

        BuildReleaseFeasibility(
            releasePosition,
            canonicalLaunchVelocity);

        // T remains the common geometry/observation clock. Lower timing is supplied
        // by real Stairway collisions; there is no synthetic LowerGuide or T/2 steering.
        InitializePeriodReference();

        UpdateCanonicalEnergyCoupling();

        TransitionTo(
            EqualizerPhase.FreeFlight,
            "ReleasedToPhysics");

        Debug.Log(
            $"[EQUALIZER CANONICAL RELEASE] " +
            $"E0={safeEnergy:F4}J " +
            $"H0={safeReferenceHeight:F4}m " +
            $"aN={canonicalNormalAcceleration:F4}m/s^2 " +
            $"n0={sourceAxis:F4} " +
            $"position={releasePosition:F4} " +
            $"releaseE={initialReleaseOscillationEnergy:F4}J " +
            $"releaseRatio={initialReleaseEnergyRatio:F4} " +
            $"canonicalVN0={canonicalLaunchNormalSpeed:F4}m/s " +
            $"predictedH={(canonicalNormalAcceleration > ImpactEnergyEpsilon ? canonicalLaunchNormalSpeed * canonicalLaunchNormalSpeed / (2f * canonicalNormalAcceleration) : 0f):F4}m " +
            $"transportLateral={transportLateralVelocity:F4} " +
            $"velocity={canonicalLaunchVelocity:F4} " +
            $"periodicDrive={forcedBoundaryDriveActive} " +
            $"T={forcedBoundaryPeriod:F4}s " +
            $"halfT={forcedBoundaryHalfPeriod:F4}s",
            this);

        return true;
    }


    // ================================================================
    // Stable oscillation frame
    // ================================================================

    private void ResetOscillationFrame()
    {
        ResetForcedBoundaryDrive();

        oscillationFrame =
            default;

        releaseOscillationEnergy =
            0f;

        releaseOscillationEnergyRatio =
            0f;

        lastContactConstraintCorrectionSpeed =
            0f;
    }


    private void CaptureOscillationFrame(
        Vector3 releasePosition,
        Vector3 releaseVelocity)
    {
        ResetOscillationFrame();

        Vector3 tangent =
            Vector3.zero;

        Vector3 normal =
            Vector3.zero;

        bool fromEnvelope =
            negativeEnvelope &&
            negativeEnvelope.TryGetLatestOscillationFrameVisual(
                out tangent,
                out normal);

        if (!fromEnvelope)
        {
            // Fallback is intentionally simple and deterministic.
            // It is only used when an Envelope frame cannot be obtained.
            tangent =
                Vector3.ProjectOnPlane(
                    releaseVelocity,
                    Vector3.up);

            if (tangent.sqrMagnitude <= ImpactEnergyEpsilon)
                tangent = Vector3.forward;

            tangent.Normalize();

            normal =
                Vector3.ProjectOnPlane(
                    Vector3.up,
                    tangent);
        }

        if (tangent.sqrMagnitude <= ImpactEnergyEpsilon ||
            normal.sqrMagnitude <= ImpactEnergyEpsilon)
        {
            return;
        }

        tangent.Normalize();

        normal =
            Vector3.ProjectOnPlane(
                normal,
                tangent);

        if (normal.sqrMagnitude <= ImpactEnergyEpsilon)
            return;

        normal.Normalize();

        Vector3 orientationAxis =
            canonicalSourceAxis.sqrMagnitude >
            ImpactEnergyEpsilon
                ? canonicalSourceAxis
                : Vector3.up;

        if (Vector3.Dot(
                normal,
                orientationAxis) < 0f)
        {
            normal =
                -normal;
        }

        Vector3 lateral =
            Vector3.Cross(
                normal,
                tangent);

        if (lateral.sqrMagnitude <= ImpactEnergyEpsilon)
            return;

        lateral.Normalize();

        // Re-orthogonalize T so numerical mapping error cannot accumulate.
        tangent =
            Vector3.Cross(
                lateral,
                normal);

        tangent.Normalize();

        oscillationFrame =
            new OscillationFrame
            {
                valid = true,
                fromEnvelope = fromEnvelope,
                origin = releasePosition,
                tangent = tangent,
                normal = normal,
                lateral = lateral
            };

        // Release starts from the full E0 reservoir.  Actual Collider impacts
        // are the only events allowed to contract this Stable-N reservoir.
        releaseOscillationEnergyRatio =
            releaseFrame.sourceEnergy >
            ImpactEnergyEpsilon
                ? Mathf.Clamp01(
                    currentCanonicalEnergyRatio)
                : 0f;

        releaseOscillationEnergy =
            Mathf.Max(
                0f,
                releaseFrame.sourceEnergy *
                releaseOscillationEnergyRatio);
    }


    // ================================================================
    // Release feasibility - one continuous model, no mode selection
    // ================================================================

    private void ResetForcedBoundaryDrive()
    {
        forcedBoundaryDriveActive = false;
        forcedBoundaryReleaseTime = 0f;
        forcedBoundaryTimeCost = 0f;
        forcedBoundaryPeriod = 0f;
        forcedBoundaryHalfPeriod = 0f;
        forcedBoundaryPhaseErrorSeconds = 0f;
        forcedBoundaryAppliedPhaseAcceleration = 0f;
        forcedBoundaryGamma = 0f;
        forcedBoundaryRadiusClearanceScale = 0f;
        forcedBoundaryPlannedReleaseSpan = 0f;
        forcedBoundaryPlannedReleaseTargetNormalSpeed = 0f;
        forcedBoundaryPlannedReleasePhaseAcceleration = 0f;

        ResetPeriodPhaseObservation();
    }


    private void InitializePeriodReference()
    {
        ResetForcedBoundaryDrive();

        if (!negativeEnvelope ||
            !oscillationFrame.valid)
        {
            return;
        }

        if (!negativeEnvelope.TryGetPeriodicContactPlan(
                out float period,
                out float halfPeriod,
                out float timeCost,
                out float gammaPerSecond,
                out float radiusClearanceScale,
                out float releaseSpan,
                out float releaseTargetNormalSpeed,
                out float releasePhaseAcceleration))
        {
            return;
        }

        // IMPORTANT: Lower is a real Stairway collision.
        // Do not solve a deadline acceleration toward Lower/Upper. T is only a
        // reference clock for envelope geometry, phase error and cycle diagnostics.
        forcedBoundaryDriveActive =
            false;

        forcedBoundaryReleaseTime =
            Time.fixedTime;

        forcedBoundaryTimeCost =
            timeCost;

        forcedBoundaryPeriod =
            period;

        forcedBoundaryHalfPeriod =
            halfPeriod;

        forcedBoundaryGamma =
            gammaPerSecond;

        forcedBoundaryRadiusClearanceScale =
            radiusClearanceScale;

        forcedBoundaryPlannedReleaseSpan =
            releaseSpan;

        forcedBoundaryPlannedReleaseTargetNormalSpeed =
            releaseTargetNormalSpeed;

        forcedBoundaryPlannedReleasePhaseAcceleration =
            releasePhaseAcceleration;

        forcedBoundaryAppliedPhaseAcceleration =
            0f;

        forcedBoundaryPhaseErrorSeconds =
            0f;

        oscillationPhaseAnchorValid =
            false;

        UpdateMaxGroundSpeedPeriodExperimentRuntime();

        Debug.Log(
            $"[EQUALIZER PERIOD REFERENCE] " +
            $"T={forcedBoundaryPeriod:F4}s " +
            $"halfT={forcedBoundaryHalfPeriod:F4}s " +
            $"lower=RealStairway " +
            $"phaseDrive=false " +
            $"Tcost={forcedBoundaryTimeCost:F4}s " +
            $"gamma={forcedBoundaryGamma:F4}/s",
            this);
    }



    // T is an observation/geometry reference only. The old T/2 deadline
    // steering methods were removed so real Stairway geometry owns lower impact timing.
    private void ResetReleaseFeasibility()
    {
        releaseFeasibility =
            default;

        lastLowerImpactTime =
            -1f;

        lastUpperImpactTime =
            -1f;

        smoothedSameBoundaryPeriod =
            0f;

        currentCanonicalOscillationEnergy =
            0f;
    }


    private void BeginReleaseEvaluation()
    {
        releaseEvaluationActive =
            true;

        directSubjectObservedThisRelease =
            !usingBallVisualAsSubjectProxy &&
            subjectTransform;

        releaseDampingViolationCount =
            0;

        minimumReleaseTransportRetention =
            1f;

        releaseDampingSuccess =
            false;

        releaseTransportSuccess =
            false;

        releaseSubjectSuccess =
            false;

        releaseOverallSuccess =
            false;

        finalReleaseSubjectGap =
            0f;
    }
    private void BuildReleaseFeasibility(
        Vector3 releasePosition,
        Vector3 releaseVelocity)
    {
        // Minimal effective specification:
        // Equalizer does not own Exact-Limit arrival timing or cycle scheduling.
        // BallVisualSlopeDrive owns the global trajectory.  Keep only release
        // evaluation state required by existing diagnostics.
        ResetReleaseFeasibility();
        BeginReleaseEvaluation();

        currentCanonicalOscillationEnergy =
            Mathf.Max(0f, releaseOscillationEnergy);

        UpdateCanonicalEnergyCoupling();

        releaseFeasibility.valid = true;
        releaseFeasibility.limitAvailable = false;
        releaseFeasibility.initialOscillationEnergy = releaseOscillationEnergy;
        releaseFeasibility.initialOscillationRatio = releaseOscillationEnergyRatio;
        releaseFeasibility.initialSubjectGap =
            subjectTransform && oscillationFrame.valid
                ? Vector3.Dot(
                    ReadSubjectPositionVisual() - releasePosition,
                    oscillationFrame.tangent.normalized)
                : 0f;
    }
    private void UpdateObservedCyclePeriod(
        bool envelopeContact)
    {
        float now =
            Time.fixedTime;

        float previous =
            envelopeContact
                ? lastUpperImpactTime
                : lastLowerImpactTime;

        if (previous >= 0f)
        {
            float observed =
                now - previous;

            if (observed >=
                MinimumObservedCyclePeriod)
            {
                smoothedSameBoundaryPeriod =
                    smoothedSameBoundaryPeriod > 0f
                        ? Mathf.Lerp(
                            smoothedSameBoundaryPeriod,
                            observed,
                            0.35f)
                        : observed;

                float period =
                    Mathf.Max(
                        0.0001f,
                        forcedBoundaryPeriod);

                float error01 =
                    (observed - period) /
                    period;

                if (envelopeContact)
                {
                    observedUpperPeriod = observed;
                    upperPeriodError01 = error01;
                }
                else
                {
                    observedLowerPeriod = observed;
                    lowerPeriodError01 = error01;
                }
            }
        }

        if (envelopeContact)
            lastUpperImpactTime = now;
        else
            lastLowerImpactTime = now;
    }


    private void ResetPeriodPhaseObservation()
    {
        // Release-local phase/extremum state only.
        // The 0T..8T experiment ledger intentionally survives Rejoin,
        // ResetForcedBoundaryDrive() and the next Envelope generation.
        oscillationPhase01 = 0.5f;
        oscillationPhaseOriginTime = 0f;
        oscillationPhaseOrigin01 = 0.5f;
        oscillationPhaseAnchorValid = false;
        normalizedPhaseError01 = 0f;

        observedUpperPeriod = 0f;
        observedLowerPeriod = 0f;
        upperPeriodError01 = 0f;
        lowerPeriodError01 = 0f;

        hasReferenceUpperExtremum = false;
        lowerSeenAfterReferenceUpper = false;
        upperExtremumStableN = 0f;
        lowerExtremumStableN = 0f;

        stableNormalSampleValid = false;
        previousStableNormalCoordinate = 0f;
        previousStableNormalVelocity = 0f;
        previousStableNormalSampleTime = 0f;
        pendingUpperExtremumValidation = false;
        pendingLowerExtremumValidation = false;
        pendingExtremumImpactTime = 0f;

        experimentSampleCompletedThisRelease = false;
    }


    private void ResetMaxGroundSpeedPeriodExperimentState()
    {
        oscillationCycleIndex = 0;
        experimentSampleCompletedThisRelease = false;
        experimentCycleAdvancePending = false;
        maxGroundSpeedExperimentCompleted = false;

        measuredAmplitude = 0f;
        previousMeasuredAmplitude = 0f;
        amplitudeDecayRatio = 1f;
        amplitudeCycleIndex = -1;

        sourceMaxGroundSpeedReadOnly = 0f;
        plannedMaxGroundSpeedForCycle = 0f;
        plannedMaxGroundSpeedRatio = 1f;

        hasPreviousAcceptedStairContact = false;
        previousAcceptedStairPoint = Vector3.zero;
        previousAcceptedStairTangentSpeed = 0f;
        observedStairSpatialInterval = 0f;
        observedGeometryPeriod = 0f;
        observedSpatialPeriodMultiplicity = 1;
        acceptedExperimentStairContactCount = 0;
        lastZeroCrossingStableN = 0f;
        lastZeroCrossingTime = 0f;
        lastZeroCrossingKind = "None";

        if (negativeEnvelope)
        {
            negativeEnvelope.ResetMaxGroundSpeedExperiment();
        }

        UpdateMaxGroundSpeedPeriodExperimentRuntime();
    }


    private void ApplyPendingMaxGroundSpeedExperimentAdvance()
    {
        if (!experimentCycleAdvancePending ||
            maxGroundSpeedExperimentCompleted)
        {
            return;
        }

        int maxCycle =
            negativeEnvelope
                ? negativeEnvelope.MaxGroundSpeedDecayCycleCount
                : 8;

        oscillationCycleIndex =
            Mathf.Clamp(
                oscillationCycleIndex + 1,
                0,
                maxCycle);

        experimentCycleAdvancePending = false;

        if (negativeEnvelope)
        {
            negativeEnvelope.SetMaxGroundSpeedExperimentCycle(
                oscillationCycleIndex);
        }

        UpdateMaxGroundSpeedPeriodExperimentRuntime();

        Debug.Log(
            $"[EQUALIZER EXPERIMENT ADVANCE] " +
            $"cycle={oscillationCycleIndex} " +
            $"maxGround0={sourceMaxGroundSpeedReadOnly:F3}m/s " +
            $"planned={plannedMaxGroundSpeedForCycle:F3}m/s",
            this);
    }


    private void UpdateOscillationPhaseRuntime()
    {
        if (!oscillationPhaseAnchorValid ||
            forcedBoundaryPeriod <= 0.0001f)
        {
            return;
        }

        float elapsed =
            Time.fixedTime -
            oscillationPhaseOriginTime;

        oscillationPhase01 =
            Mathf.Repeat(
                oscillationPhaseOrigin01 +
                elapsed /
                forcedBoundaryPeriod,
                1f);

        normalizedPhaseError01 =
            forcedBoundaryPhaseErrorSeconds /
            forcedBoundaryPeriod;
    }


    private bool TryMeasureStableNormalCoordinate(
        out float coordinate)
    {
        coordinate = 0f;

        if (!ballVisualEqualizer ||
            !oscillationFrame.valid)
        {
            return false;
        }

        Vector3 normal =
            oscillationFrame.normal;

        if (normal.sqrMagnitude <=
            ImpactEnergyEpsilon)
        {
            return false;
        }

        normal.Normalize();

        // Moving carrier C(t):
        // release Equalizer position + Subject travel since release.
        // This removes T/L transport and leaves only the Stable-N oscillation.
        Vector3 subjectTravel =
            ReadSubjectPositionVisual() -
            releaseFrame.subjectPosition;

        Vector3 carrierPosition =
            releaseFrame.position +
            subjectTravel;

        coordinate =
            Vector3.Dot(
                ballVisualEqualizer.position -
                carrierPosition,
                normal);

        return true;
    }


    private void UpdateMaxGroundSpeedPeriodExperimentRuntime()
    {
        if (!negativeEnvelope)
            return;

        if (!negativeEnvelope.TryEvaluateMaxGroundSpeedDecayExperiment(
                oscillationCycleIndex,
                out float sourceSpeed,
                out float plannedSpeed,
                out _))
        {
            sourceMaxGroundSpeedReadOnly = 0f;
            plannedMaxGroundSpeedForCycle = 0f;
            plannedMaxGroundSpeedRatio = 0f;
            return;
        }

        sourceMaxGroundSpeedReadOnly =
            sourceSpeed;

        plannedMaxGroundSpeedForCycle =
            plannedSpeed;

        plannedMaxGroundSpeedRatio =
            sourceSpeed > 0.000001f
                ? plannedSpeed / sourceSpeed
                : 0f;
    }


    private void ObservePeriodPhaseExtremum(
        bool upperBoundary,
        bool acceptedForExperiment)
    {
        if (forcedBoundaryPeriod <= 0.0001f)
        {
            return;
        }

        float now =
            Time.fixedTime;

        float targetPhase01 =
            upperBoundary
                ? 0f
                : 0.5f;

        // Phase is observation-only.  Every canonical impact may re-anchor the
        // phase clock, while the experiment ledger below only arms on clean,
        // high-alignment contacts.
        if (oscillationPhaseAnchorValid)
        {
            float elapsed =
                now - oscillationPhaseOriginTime;

            float predictedPhase01 =
                Mathf.Repeat(
                    oscillationPhaseOrigin01 +
                    elapsed / forcedBoundaryPeriod,
                    1f);

            normalizedPhaseError01 =
                Mathf.DeltaAngle(
                    targetPhase01 * 360f,
                    predictedPhase01 * 360f) /
                360f;

            forcedBoundaryPhaseErrorSeconds =
                normalizedPhaseError01 *
                forcedBoundaryPeriod;
        }
        else
        {
            normalizedPhaseError01 = 0f;
            forcedBoundaryPhaseErrorSeconds = 0f;
        }

        oscillationPhaseOriginTime =
            now;

        oscillationPhaseOrigin01 =
            targetPhase01;

        oscillationPhase01 =
            targetPhase01;

        oscillationPhaseAnchorValid =
            true;

        if (!TryMeasureStableNormalCoordinate(
                out float stableN))
        {
            return;
        }

        // A clean canonical impact is accepted as the primary extremum by
        // TryAcceptCanonicalImpactExtremum().  These pending flags only keep a
        // short zero-crossing fallback/diagnostic window; they are not required
        // for the canonical impact path to commit an experiment sample.
        if (acceptedForExperiment &&
            !maxGroundSpeedExperimentCompleted &&
            !experimentSampleCompletedThisRelease)
        {
            if (upperBoundary)
            {
                pendingUpperExtremumValidation = true;
                pendingLowerExtremumValidation = false;
                pendingExtremumImpactTime = now;
            }
            else if (hasReferenceUpperExtremum)
            {
                pendingLowerExtremumValidation = true;
                pendingUpperExtremumValidation = false;
                pendingExtremumImpactTime = now;
            }
        }

        Debug.Log(
            $"[EQUALIZER PHASE SAMPLE] " +
            $"boundary={(upperBoundary ? "Upper" : "Lower")} " +
            $"accepted={acceptedForExperiment} " +
            $"cycle={oscillationCycleIndex} " +
            $"phase={oscillationPhase01:F4} " +
            $"phaseError01={normalizedPhaseError01:F5} " +
            $"T={forcedBoundaryPeriod:F5}s " +
            $"xN={stableN:F5}m " +
            $"A={measuredAmplitude:F5}m " +
            $"rA={amplitudeDecayRatio:F5} " +
            $"maxGround0={sourceMaxGroundSpeedReadOnly:F3}m/s " +
            $"planned={plannedMaxGroundSpeedForCycle:F3}m/s " +
            $"sampleDone={experimentSampleCompletedThisRelease} " +
            $"nextPending={experimentCycleAdvancePending}",
            this);
    }


    private bool TryMeasureStableNormalRelativeVelocity(
        out float velocityN)
    {
        velocityN = 0f;

        if (!ballVisualEqualizer ||
            !oscillationFrame.valid)
        {
            return false;
        }

        Vector3 normal =
            oscillationFrame.normal;

        if (normal.sqrMagnitude <=
            ImpactEnergyEpsilon)
        {
            return false;
        }

        normal.Normalize();

        Vector3 relativeVelocity =
            ballVisualEqualizer.velocity -
            ReadSubjectVelocityVisual();

        velocityN =
            Vector3.Dot(
                relativeVelocity,
                normal);

        return !float.IsNaN(velocityN) &&
               !float.IsInfinity(velocityN);
    }


    private void TryAcceptCanonicalImpactExtremum(
        in ContactFrame frame,
        Vector3 mappedOutgoingVelocity,
        bool upperBoundary,
        bool acceptedForExperiment)
    {
        if (!acceptedForExperiment ||
            maxGroundSpeedExperimentCompleted ||
            experimentSampleCompletedThisRelease ||
            !oscillationFrame.valid)
        {
            return;
        }

        Vector3 normal =
            oscillationFrame.normal;

        if (normal.sqrMagnitude <=
            ImpactEnergyEpsilon)
        {
            return;
        }

        normal.Normalize();

        Vector3 subjectVelocity =
            ReadSubjectVelocityVisual();

        float incomingVN =
            Vector3.Dot(
                frame.incidentVelocity -
                subjectVelocity,
                normal);

        float outgoingVN =
            Vector3.Dot(
                mappedOutgoingVelocity -
                subjectVelocity,
                normal);

        bool upperReversal =
            incomingVN >
                StableNormalZeroCrossVelocityEpsilon &&
            outgoingVN <
                -StableNormalZeroCrossVelocityEpsilon;

        bool lowerReversal =
            incomingVN <
                -StableNormalZeroCrossVelocityEpsilon &&
            outgoingVN >
                StableNormalZeroCrossVelocityEpsilon;

        bool reversalConfirmed =
            upperBoundary
                ? upperReversal
                : lowerReversal;

        if (!TryMeasureStableNormalCoordinate(
                out float stableN))
        {
            return;
        }

        float now =
            Time.fixedTime;

        // The canonical, clean, high-alignment impact is the primary extremum
        // observation.  Contact quality has already been validated by the
        // caller, so velocity sign reversal is diagnostic rather than a hard
        // gate.  This avoids dropping valid Upper extrema when the solver/map
        // exposes the post-impact velocity in a basis that does not cross the
        // numerical epsilon inside this callback.
        lastZeroCrossingStableN =
            stableN;

        lastZeroCrossingTime =
            now;

        lastZeroCrossingKind =
            upperBoundary
                ? "UpperCanonicalImpact"
                : "LowerCanonicalImpact";

        Debug.Log(
            $"[EQUALIZER CANONICAL IMPACT EXTREMUM] " +
            $"kind={(upperBoundary ? "Upper" : "Lower")} " +
            $"cycle={oscillationCycleIndex} " +
            $"vNin={incomingVN:F5}m/s " +
            $"vNout={outgoingVN:F5}m/s " +
            $"reversalConfirmed={reversalConfirmed} " +
            $"xN={stableN:F5}m",
            this);

        if (reversalConfirmed)
        {
            Debug.Log(
                $"[EQUALIZER STABLE-N REVERSAL] " +
                $"kind={(upperBoundary ? "Upper" : "Lower")} " +
                $"cycle={oscillationCycleIndex} " +
                $"vNin={incomingVN:F5}m/s " +
                $"vNout={outgoingVN:F5}m/s " +
                $"xN={stableN:F5}m",
                this);
        }

        AcceptStableNormalExtremum(
            upperBoundary,
            stableN,
            now,
            reversalConfirmed
                ? "CanonicalImpact+VelocityReversal"
                : "CanonicalImpact");
    }


    private void UpdateStableNormalZeroCrossingObservation()
    {
        if (!TryMeasureStableNormalCoordinate(
                out float coordinate) ||
            !TryMeasureStableNormalRelativeVelocity(
                out float velocityN))
        {
            stableNormalSampleValid = false;
            return;
        }

        float now =
            Time.fixedTime;

        if (stableNormalSampleValid)
        {
            bool upperCross =
                previousStableNormalVelocity >
                    StableNormalZeroCrossVelocityEpsilon &&
                velocityN <
                    -StableNormalZeroCrossVelocityEpsilon;

            bool lowerCross =
                previousStableNormalVelocity <
                    -StableNormalZeroCrossVelocityEpsilon &&
                velocityN >
                    StableNormalZeroCrossVelocityEpsilon;

            if (upperCross || lowerCross)
            {
                float denominator =
                    previousStableNormalVelocity -
                    velocityN;

                float interpolation01 =
                    Mathf.Abs(denominator) > 0.000001f
                        ? Mathf.Clamp01(
                            previousStableNormalVelocity /
                            denominator)
                        : 0.5f;

                float zeroCrossingCoordinate =
                    Mathf.Lerp(
                        previousStableNormalCoordinate,
                        coordinate,
                        interpolation01);

                float zeroCrossingTime =
                    Mathf.Lerp(
                        previousStableNormalSampleTime,
                        now,
                        interpolation01);

                lastZeroCrossingStableN =
                    zeroCrossingCoordinate;

                lastZeroCrossingTime =
                    zeroCrossingTime;

                lastZeroCrossingKind =
                    upperCross
                        ? "Upper"
                        : "Lower";

                Debug.Log(
                    $"[EQUALIZER ZERO CROSS OBSERVATION] " +
                    $"kind={(upperCross ? "Upper" : "Lower")} " +
                    $"cycle={oscillationCycleIndex} " +
                    $"time={zeroCrossingTime:F5}s " +
                    $"xN={zeroCrossingCoordinate:F5}m",
                    this);

                float validationWindow =
                    Mathf.Max(
                        0.0001f,
                        Time.fixedDeltaTime * 2.5f);

                bool impactStillRecent =
                    now - pendingExtremumImpactTime <=
                    validationWindow;

                if (upperCross &&
                    pendingUpperExtremumValidation &&
                    impactStillRecent)
                {
                    AcceptStableNormalExtremum(
                        true,
                        zeroCrossingCoordinate,
                        zeroCrossingTime,
                        "StableNZeroCross");
                }
                else if (lowerCross &&
                         pendingLowerExtremumValidation &&
                         impactStillRecent)
                {
                    AcceptStableNormalExtremum(
                        false,
                        zeroCrossingCoordinate,
                        zeroCrossingTime,
                        "StableNZeroCross");
                }
            }
        }

        // Stale pending impact must never validate a later unrelated extremum.
        float pendingAge =
            now - pendingExtremumImpactTime;

        float pendingLifetime =
            Mathf.Max(
                0.0001f,
                Time.fixedDeltaTime * 3f);

        if (pendingUpperExtremumValidation &&
            pendingAge > pendingLifetime)
        {
            pendingUpperExtremumValidation = false;
        }

        if (pendingLowerExtremumValidation &&
            pendingAge > pendingLifetime)
        {
            pendingLowerExtremumValidation = false;
        }

        previousStableNormalCoordinate =
            coordinate;

        previousStableNormalVelocity =
            velocityN;

        previousStableNormalSampleTime =
            now;

        stableNormalSampleValid =
            true;
    }


    private void AcceptStableNormalExtremum(
        bool upperExtremum,
        float stableN,
        float extremumTime,
        string source)
    {
        if (maxGroundSpeedExperimentCompleted ||
            experimentSampleCompletedThisRelease)
        {
            pendingUpperExtremumValidation = false;
            pendingLowerExtremumValidation = false;
            return;
        }

        if (upperExtremum)
        {
            upperExtremumStableN =
                stableN;

            hasReferenceUpperExtremum =
                true;

            lowerSeenAfterReferenceUpper =
                false;

            pendingUpperExtremumValidation =
                false;

            Debug.Log(
                $"[EQUALIZER EXTREMUM ACCEPTED] " +
                $"source={source} " +
                $"kind=Upper cycle={oscillationCycleIndex} " +
                $"time={extremumTime:F5}s " +
                $"xN={stableN:F5}m",
                this);

            return;
        }

        if (!hasReferenceUpperExtremum)
        {
            pendingLowerExtremumValidation =
                false;
            return;
        }

        lowerExtremumStableN =
            stableN;

        lowerSeenAfterReferenceUpper =
            true;

        pendingLowerExtremumValidation =
            false;

        float amplitude =
            0.5f *
            Mathf.Abs(
                upperExtremumStableN -
                lowerExtremumStableN);

        if (amplitude <=
            ImpactEnergyEpsilon)
        {
            return;
        }

        amplitudeDecayRatio =
            previousMeasuredAmplitude >
            ImpactEnergyEpsilon
                ? amplitude /
                  previousMeasuredAmplitude
                : 1f;

        measuredAmplitude =
            amplitude;

        previousMeasuredAmplitude =
            amplitude;

        amplitudeCycleIndex =
            oscillationCycleIndex;

        experimentSampleCompletedThisRelease =
            true;

        int maxCycle =
            negativeEnvelope
                ? negativeEnvelope.MaxGroundSpeedDecayCycleCount
                : 8;

        if (oscillationCycleIndex >= maxCycle)
        {
            maxGroundSpeedExperimentCompleted =
                true;

            experimentCycleAdvancePending =
                false;
        }
        else
        {
            experimentCycleAdvancePending =
                true;
        }

        Debug.Log(
            $"[EQUALIZER EXPERIMENT SAMPLE] " +
            $"source={source} " +
            $"cycle={oscillationCycleIndex} " +
            $"T={forcedBoundaryPeriod:F5}s " +
            $"maxGround0={sourceMaxGroundSpeedReadOnly:F3}m/s " +
            $"planned={plannedMaxGroundSpeedForCycle:F3}m/s " +
            $"upperN={upperExtremumStableN:F5}m " +
            $"lowerN={lowerExtremumStableN:F5}m " +
            $"A={measuredAmplitude:F5}m " +
            $"rA={amplitudeDecayRatio:F5} " +
            $"nextPending={experimentCycleAdvancePending} " +
            $"complete={maxGroundSpeedExperimentCompleted}",
            this);
    }


    private void ObserveAcceptedStairGeometryPeriod(
        Vector3 contactPoint,
        float actualTangentSpeed)
    {
        if (!negativeEnvelope ||
            !oscillationFrame.valid ||
            actualTangentSpeed < GeometryPeriodMinimumSpeed)
        {
            return;
        }

        Vector3 tangent =
            oscillationFrame.tangent;

        if (tangent.sqrMagnitude <=
            ImpactEnergyEpsilon)
        {
            return;
        }

        tangent.Normalize();

        if (hasPreviousAcceptedStairContact)
        {
            float currentSpeed =
                Mathf.Max(
                    GeometryPeriodMinimumSpeed,
                    actualTangentSpeed);

            float previousSpeed =
                Mathf.Max(
                    GeometryPeriodMinimumSpeed,
                    previousAcceptedStairTangentSpeed);

            float meanSpeed =
                0.5f *
                (currentSpeed + previousSpeed);

            float projectedDistance =
                Mathf.Abs(
                    Vector3.Dot(
                        contactPoint -
                        previousAcceptedStairPoint,
                        tangent));

            float nominalT =
                Mathf.Max(
                    0.0001f,
                    negativeEnvelope.NominalExperimentPeriodSeconds > 0.0001f
                        ? negativeEnvelope.NominalExperimentPeriodSeconds
                        : forcedBoundaryPeriod);

            float nominalSpatialInterval =
                Mathf.Max(
                    0.0001f,
                    nominalT * meanSpeed);

            // If one or more clean Stair contacts were skipped, reduce the
            // measured distance to its most likely fundamental interval.
            int multiplicity =
                Mathf.Max(
                    1,
                    Mathf.RoundToInt(
                        projectedDistance /
                        nominalSpatialInterval));

            float fundamentalDistance =
                projectedDistance /
                multiplicity;

            float spacingRatio =
                fundamentalDistance /
                nominalSpatialInterval;

            if (projectedDistance > 0.0001f &&
                spacingRatio >= 0.5f &&
                spacingRatio <= 1.5f)
            {
                observedStairSpatialInterval =
                    fundamentalDistance;

                observedSpatialPeriodMultiplicity =
                    multiplicity;

                observedGeometryPeriod =
                    fundamentalDistance /
                    meanSpeed;

                negativeEnvelope.SubmitObservedGeometryPeriod(
                    observedGeometryPeriod);

                Debug.Log(
                    $"[EQUALIZER STAIR PERIOD OBSERVATION] " +
                    $"cycle={oscillationCycleIndex} " +
                    $"deltaSraw={projectedDistance:F5}m " +
                    $"multiple={multiplicity} " +
                    $"deltaS={fundamentalDistance:F5}m " +
                    $"actualVT={meanSpeed:F5}m/s " +
                    $"Tgeom={observedGeometryPeriod:F5}s " +
                    $"Tnom={nominalT:F5}s " +
                    $"correction={negativeEnvelope.ObservedGeometryPeriodCorrectionRatio:F5} " +
                    $"apply=NextRelease",
                    this);
            }
        }

        previousAcceptedStairPoint =
            contactPoint;

        previousAcceptedStairTangentSpeed =
            Mathf.Max(
                GeometryPeriodMinimumSpeed,
                actualTangentSpeed);

        hasPreviousAcceptedStairContact =
            true;

        acceptedExperimentStairContactCount++;
    }


    private void FinalizeReleaseEvaluation(
        string reason)
    {
        if (!releaseEvaluationActive)
            return;

        UpdateSubjectConvergenceDiagnostics();

        finalReleaseSubjectGap =
            subjectTransportGap;

        float negligibleOscillationEnergy =
            0.5f *
            EqualizerMass *
            MinimumImpactNormalSpeed *
            MinimumImpactNormalSpeed;

        releaseDampingSuccess =
            releaseDampingViolationCount == 0 &&
            (canonicalImpactCount > 0 ||
             releaseOscillationEnergy <=
                negligibleOscillationEnergy);

        releaseTransportSuccess =
            minimumReleaseTransportRetention >=
            MinimumGameTransportRetention;

        float initialSubjectGapAbs =
            Mathf.Abs(
                releaseFeasibility.initialSubjectGap);

        float finalSubjectGapAbs =
            Mathf.Abs(
                finalReleaseSubjectGap);

        float subjectTolerance =
            Mathf.Max(
                subjectTransportDeadZone,
                0.05f);

        releaseSubjectSuccess =
            directSubjectObservedThisRelease &&
            !usingBallVisualAsSubjectProxy &&
            subjectTransform &&
            (finalSubjectGapAbs <=
                subjectTolerance ||
             finalSubjectGapAbs <=
                initialSubjectGapAbs +
                0.05f);

        releaseOverallSuccess =
            releaseDampingSuccess &&
            releaseTransportSuccess &&
            releaseSubjectSuccess;

        completedReleaseCount++;

        if (releaseOverallSuccess)
            successfulReleaseCount++;

        releaseOverallSuccessRate =
            completedReleaseCount > 0
                ? (float)successfulReleaseCount /
                  completedReleaseCount
                : 0f;

        releaseEvaluationActive =
            false;

        Debug.Log(
            $"[EQUALIZER RELEASE RESULT] " +
            $"reason={reason} " +
            $"dampingSuccess={releaseDampingSuccess} " +
            $"transportSuccess={releaseTransportSuccess} " +
            $"subjectSuccess={releaseSubjectSuccess} " +
            $"overallSuccess={releaseOverallSuccess} " +
            $"overallRate={releaseOverallSuccessRate:P1} " +
            $"canonicalImpacts={canonicalImpactCount} " +
            $"emergencyImpacts={emergencyImpactCount} " +
            $"dampingViolations={releaseDampingViolationCount} " +
            $"minTransportRetention={minimumReleaseTransportRetention:F3} " +
            $"initialSubjectGap={releaseFeasibility.initialSubjectGap:F4}m " +
            $"finalSubjectGap={finalReleaseSubjectGap:F4}m " +
            $"directSubjectObserved={directSubjectObservedThisRelease} " +
            $"canonicalOscE={currentCanonicalOscillationEnergy:F4}J",
            this);
    }


    // ================================================================
    // Subject transport convergence
    // ================================================================

    private void InitializeSubjectMotionEstimate()
    {
        ResolveReferences();

        if (!HasMappedSubject &&
            !subjectTransform &&
            !ballVisual)
        {
            return;
        }

        previousSubjectPosition =
            ReadSubjectPositionVisual();

        hasSubjectPositionSample =
            true;

        estimatedSubjectVelocity =
            ReadSubjectVelocityVisual();

        subjectVelocitySource =
            HasMappedSubject
                ? "CorrespondSubject.MappedPhysicalVelocity"
                : subjectBody
                    ? "SubjectRigidbodyFallback"
                    : "BallVisualFallback";
    }


    private void UpdateSubjectMotionEstimate()
    {
        if (!HasMappedSubject &&
            !subjectTransform &&
            !ballVisual)
        {
            estimatedSubjectVelocity =
                Vector3.zero;

            subjectVelocitySource =
                "Unavailable";

            hasSubjectPositionSample =
                false;

            return;
        }

        Vector3 currentPosition =
            ReadSubjectPositionVisual();

        float dt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f);

        if (HasMappedSubject)
        {
            estimatedSubjectVelocity =
                correspondSubject.MappedPhysicalVelocity;

            subjectVelocitySource =
                "CorrespondSubject.MappedPhysicalVelocity";
        }
        else if (subjectBody &&
                 (!ballVisual ||
                  subjectBody != ballVisual))
        {
            estimatedSubjectVelocity =
                subjectBody.velocity;

            subjectVelocitySource =
                "SubjectRigidbodyFallback";
        }
        else if (ballVisual)
        {
            estimatedSubjectVelocity =
                ballVisual.velocity;

            subjectVelocitySource =
                "BallVisualFallback";
        }
        else if (hasSubjectPositionSample)
        {
            estimatedSubjectVelocity =
                (currentPosition -
                 previousSubjectPosition) /
                dt;

            subjectVelocitySource =
                "SubjectTransformDeltaFallback";
        }
        else
        {
            estimatedSubjectVelocity =
                Vector3.zero;

            subjectVelocitySource =
                "SubjectTransformWaiting";
        }

        previousSubjectPosition =
            currentPosition;

        hasSubjectPositionSample =
            true;
    }



    private void ApplyCanonicalNormalAcceleration()
    {
        bool realLowerImpactContact =
            lowerContactCount > 0;

        if (synchronized ||
            !ballVisualEqualizer ||
            !oscillationFrame.valid ||
            realLowerImpactContact ||
            upperContactCount > 0 ||
            canonicalNormalAcceleration <=
                ImpactEnergyEpsilon)
        {
            return;
        }

        Vector3 normal =
            oscillationFrame.normal;

        if (normal.sqrMagnitude <=
            ImpactEnergyEpsilon)
        {
            return;
        }

        normal.Normalize();

        // Desired total acceleration on Stable N:
        //
        //     dot(a_total, N) = -aN
        //
        // Rigidbody gravity already contributes dot(g,N), therefore add only
        // the scalar correction on N.  T/L gravity is preserved.
        float gravityAlongNormal =
            ballVisualEqualizer.useGravity
                ? Vector3.Dot(
                    Physics.gravity,
                    normal)
                : 0f;

        float targetTotalNormalAcceleration =
            -canonicalNormalAcceleration;

        // No T/2 deadline steering: Stairway impact timing comes from real geometry.
        // Canonical Stable-N acceleration remains the only continuous N drive.

        float correctionAlongNormal =
            targetTotalNormalAcceleration -
            gravityAlongNormal;

        ballVisualEqualizer.AddForce(
            normal *
            correctionAlongNormal,
            ForceMode.Acceleration);
    }


    private void UpdateCanonicalEnergyCoupling()
    {
        float sourceEnergy =
            Mathf.Max(
                0f,
                releaseFrame.sourceEnergy);

        currentCanonicalEnergyRatio =
            sourceEnergy >
            ImpactEnergyEpsilon
                ? Mathf.Clamp01(
                    currentCanonicalOscillationEnergy /
                    sourceEnergy)
                : 0f;

        if (negativeEnvelope)
        {
            // Scalar ledger only; active Upper geometry is not rebuilt mid-Release.
            negativeEnvelope.SetCanonicalEnergyRatio(
                currentCanonicalEnergyRatio);
        }
    }


    private void ApplySubjectTransportConvergence()
    {
        subjectConvergenceActive =
            false;

        appliedTransportDeltaSpeed =
            0f;

        if (synchronized ||
            !subjectTransform ||
            !ballVisualEqualizer ||
            !oscillationFrame.valid)
        {
            return;
        }

        // Contact owns the instantaneous non-penetration solve.  Convergence
        // resumes only in free flight so it can never weaken or reinforce a
        // Collider reflection.
        if (lowerContactCount > 0 ||
            upperContactCount > 0)
        {
            return;
        }

        Vector3 normal =
            oscillationFrame.normal;

        if (normal.sqrMagnitude <=
            ImpactEnergyEpsilon)
        {
            return;
        }

        normal.Normalize();

        Vector3 velocity =
            ballVisualEqualizer.velocity;

        // Hard ownership split:
        //   N      -> damping only
        //   N^perp -> Subject transport convergence
        // Preserve the N scalar exactly, even for very large launch speeds.
        float normalSpeed =
            Vector3.Dot(
                velocity,
                normal);

        Vector3 currentPlaneVelocity =
            velocity -
            normal *
            normalSpeed;

        Vector3 subjectPlaneVelocity =
            Vector3.ProjectOnPlane(
                estimatedSubjectVelocity,
                normal);

        Vector3 gapPlane =
            Vector3.ProjectOnPlane(
                ReadSubjectPositionVisual() -
                ballVisualEqualizer.position,
                normal);

        float deadZone =
            Mathf.Max(
                0f,
                subjectTransportDeadZone);

        if (gapPlane.magnitude <= deadZone)
        {
            gapPlane =
                Vector3.zero;
        }
        else if (gapPlane.sqrMagnitude >
                 ImpactEnergyEpsilon)
        {
            gapPlane =
                gapPlane.normalized *
                (gapPlane.magnitude - deadZone);
        }

        float convergenceTime =
            Mathf.Max(
                0.05f,
                subjectConvergenceTime);

        Vector3 closingVelocity =
            gapPlane /
            convergenceTime;

        closingVelocity =
            Vector3.ClampMagnitude(
                closingVelocity,
                Mathf.Max(
                    0f,
                    maximumTransportSpeedBoost));

        Vector3 targetPlaneVelocity =
            subjectPlaneVelocity +
            closingVelocity;

        float maxAcceleration =
            Mathf.Max(
                0.01f,
                maximumTransportAcceleration);

        Vector3 newPlaneVelocity =
            Vector3.MoveTowards(
                currentPlaneVelocity,
                targetPlaneVelocity,
                maxAcceleration *
                Mathf.Max(
                    Time.fixedDeltaTime,
                    0.000001f));

        Vector3 planeDelta =
            newPlaneVelocity -
            currentPlaneVelocity;

        ballVisualEqualizer.velocity =
            newPlaneVelocity +
            normal *
            normalSpeed;

        subjectConvergenceActive =
            planeDelta.sqrMagnitude >
            0.00000001f;

        appliedTransportDeltaSpeed =
            planeDelta.magnitude;

        Vector3 tangent =
            oscillationFrame.tangent.sqrMagnitude >
                ImpactEnergyEpsilon
                ? oscillationFrame.tangent.normalized
                : Vector3.forward;

        desiredTransportSpeed =
            Vector3.Dot(
                targetPlaneVelocity,
                tangent);
    }


    private void UpdateSubjectConvergenceDiagnostics()
    {
        if (!subjectTransform ||
            !ballVisualEqualizer ||
            !oscillationFrame.valid)
        {
            subjectDistance =
                subjectTransform &&
                ballVisualEqualizer
                    ? Vector3.Distance(
                        ReadSubjectPositionVisual(),
                        ballVisualEqualizer.position)
                    : 0f;

            subjectTransportGap =
                0f;

            subjectTransportSpeed =
                0f;

            equalizerTransportSpeed =
                0f;

            if (!subjectConvergenceActive)
            {
                desiredTransportSpeed =
                    0f;
            }

            return;
        }

        Vector3 tangent =
            oscillationFrame.tangent.normalized;

        subjectDistance =
            Vector3.Distance(
                ReadSubjectPositionVisual(),
                ballVisualEqualizer.position);

        subjectTransportGap =
            Vector3.Dot(
                ReadSubjectPositionVisual() -
                ballVisualEqualizer.position,
                tangent);

        subjectTransportSpeed =
            Vector3.Dot(
                estimatedSubjectVelocity,
                tangent);

        equalizerTransportSpeed =
            Vector3.Dot(
                ballVisualEqualizer.velocity,
                tangent);

        if (!subjectConvergenceActive)
        {
            desiredTransportSpeed =
                Mathf.Max(
                    0f,
                    subjectTransportSpeed);
        }
    }


    // ================================================================
    // Clean impact solver
    // ================================================================
    //
    // The impact response is solved only in the stable orthogonal basis:
    //
    //     v+ = v_T T + v_L L + u D
    //
    // D = +N for Lower, -N for Upper.
    //
    // Hard invariants:
    //     v_T >= 0                 (no reverse transport)
    //     0 <= u <= u_mapped       (no oscillation energy re-injection)
    //     dot(v+, n_c) >= 0        (non-penetration)
    //
    // If these compete, harmful L is reduced first, then harmful
    // oscillation.  T is not sacrificed by the canonical map.  A face that
    // still cannot separate is demoted to Emergency / Unity-solver handling.
    // ================================================================

    private struct CleanImpactSolveResult
    {
        public Vector3 velocity;
        public float preTransportSpeed;
        public float postTransportSpeed;
        public float transportRetention;
        public float requestedOscillationSpeed;
        public float finalOscillationSpeed;
        public float finalOscillationEnergy;
        public float wallness;
        public float finalSeparationSpeed;
        public float correctionSpeed;
        public bool forwardGuardApplied;
        public bool oscillationReduced;
        public bool severeTransportLoss;
        public bool physicsClean;
        public bool requiresEmergency;
        public float gameQuality01;
    }


    private CleanImpactSolveResult SolveCleanImpactVelocity(
        ContactFrame frame,
        float requestedOscillationSpeed)
    {
        Vector3 tangent =
            oscillationFrame.tangent.normalized;

        Vector3 lateral =
            oscillationFrame.lateral.normalized;

        Vector3 outgoingAxis =
            frame.oscillationOutgoingAxis.normalized;

        Vector3 contactNormal =
            frame.normal.normalized;

        float rawTransportSpeed =
            Vector3.Dot(
                frame.incidentVelocity,
                tangent);

        float transportSpeed =
            Mathf.Max(
                0f,
                rawTransportSpeed);

        bool forwardGuardApplied =
            rawTransportSpeed < 0f;

        float lateralSpeed =
            Vector3.Dot(
                frame.incidentVelocity,
                lateral);

        float oscillationSpeed =
            Mathf.Max(
                0f,
                requestedOscillationSpeed);

        Vector3 desiredVelocity =
            tangent *
            transportSpeed +
            lateral *
            lateralSpeed +
            outgoingAxis *
            oscillationSpeed;

        float nT =
            Vector3.Dot(
                contactNormal,
                tangent);

        float nL =
            Vector3.Dot(
                contactNormal,
                lateral);

        float nD =
            Vector3.Dot(
                contactNormal,
                outgoingAxis);

        float wallness =
            Mathf.Clamp01(
                Mathf.Max(
                    0f,
                    -nT));

        float separationSpeed =
            nT * transportSpeed +
            nL * lateralSpeed +
            nD * oscillationSpeed;

        bool oscillationReduced =
            false;

        // Canonical solve never sacrifices T.  Contact Authority is expected
        // to choose a face that can be satisfied while preserving transport.
        // If that assumption fails, return requiresEmergency and let Unity's
        // contact solver be the last resort instead of collapsing vT to zero.
        if (separationSpeed < 0f)
        {
            float deficit =
                -separationSpeed;

            // 1) L is not part of damping or Subject convergence.
            float lateralContribution =
                nL *
                lateralSpeed;

            if (lateralContribution < 0f &&
                deficit > 0f)
            {
                float improvementAvailable =
                    -lateralContribution;

                float fraction =
                    Mathf.Clamp01(
                        deficit /
                        Mathf.Max(
                            improvementAvailable,
                            ImpactEnergyEpsilon));

                lateralSpeed *=
                    1f -
                    fraction;

                deficit -=
                    improvementAvailable *
                    fraction;
            }

            // 2) If the requested oscillation itself points into the face,
            // reduce it.  It is never increased above the damping map.
            float oscillationContribution =
                nD *
                oscillationSpeed;

            if (oscillationContribution < 0f &&
                deficit > 0f &&
                oscillationSpeed > 0f)
            {
                float reduction =
                    Mathf.Min(
                        oscillationSpeed,
                        deficit /
                        Mathf.Max(
                            -nD,
                            ImpactEnergyEpsilon));

                oscillationSpeed -=
                    reduction;

                deficit -=
                    reduction *
                    (-nD);

                oscillationReduced =
                    reduction >
                    0.00001f;
            }
        }

        Vector3 finalVelocity =
            tangent *
            transportSpeed +
            lateral *
            lateralSpeed +
            outgoingAxis *
            oscillationSpeed;

        float finalSeparationSpeed =
            Vector3.Dot(
                finalVelocity,
                contactNormal);

        if (finalSeparationSpeed < 0f &&
            finalSeparationSpeed >
            -CleanConstraintEpsilon)
        {
            finalSeparationSpeed =
                0f;
        }

        bool requiresEmergency =
            finalSeparationSpeed <
            -CleanConstraintEpsilon;

        float preTransportSpeed =
            Mathf.Max(
                0f,
                rawTransportSpeed);

        float transportRetention =
            preTransportSpeed >
            ImpactEnergyEpsilon
                ? transportSpeed /
                  preTransportSpeed
                : 1f;

        bool severeTransportLoss =
            preTransportSpeed >
            MinimumImpactNormalSpeed &&
            transportRetention <
            MinimumCleanTransportRetention;

        float finalOscillationEnergy =
            0.5f *
            EqualizerMass *
            oscillationSpeed *
            oscillationSpeed;

        bool physicsClean =
            !requiresEmergency &&
            transportSpeed >=
                -CleanConstraintEpsilon &&
            oscillationSpeed <=
                requestedOscillationSpeed +
                CleanConstraintEpsilon &&
            finalSeparationSpeed >=
                -CleanConstraintEpsilon;

        float gameQuality01 =
            physicsClean
                ? Mathf.Clamp01(
                    transportRetention)
                : 0f;

        return new CleanImpactSolveResult
        {
            velocity = finalVelocity,
            preTransportSpeed = preTransportSpeed,
            postTransportSpeed = transportSpeed,
            transportRetention = transportRetention,
            requestedOscillationSpeed = requestedOscillationSpeed,
            finalOscillationSpeed = oscillationSpeed,
            finalOscillationEnergy = finalOscillationEnergy,
            wallness = wallness,
            finalSeparationSpeed = finalSeparationSpeed,
            correctionSpeed =
                Vector3.Distance(
                    desiredVelocity,
                    finalVelocity),
            forwardGuardApplied = forwardGuardApplied,
            oscillationReduced = oscillationReduced,
            severeTransportLoss = severeTransportLoss,
            physicsClean = physicsClean,
            requiresEmergency = requiresEmergency,
            gameQuality01 = gameQuality01
        };
    }


    private CleanImpactSolveResult BuildEmergencyContactResult(
        ContactFrame frame,
        float requestedOscillationSpeed,
        float maximumOscillationEnergy)
    {
        Vector3 tangent =
            oscillationFrame.tangent.normalized;

        Vector3 lateral =
            oscillationFrame.lateral.normalized;

        Vector3 outgoingAxis =
            frame.oscillationOutgoingAxis.normalized;

        Vector3 contactNormal =
            frame.normal.normalized;

        // ------------------------------------------------------------
        // Emergency is a contact-constraint solve, NOT a new energy source.
        //
        // The Unity solver may create a large Stable-N component when a
        // non-canonical boundary face is encountered.  That component must never
        // exceed the strict per-impact energy budget supplied by ApplyImpactMap:
        //
        //     E_emergency,out
        //         <= min(E_canonical,current, E0, E_requested)
        //
        // If the contact cannot be satisfied with that N-energy budget, remove
        // harmful L first, then spend the remaining canonical N budget, and
        // finally reduce T.  We prefer transport loss over manufacturing
        // oscillation energy.
        // ------------------------------------------------------------

        Vector3 solverVelocity =
            ballVisualEqualizer.velocity;

        float rawTransportSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    frame.incidentVelocity,
                    tangent));

        float solverTransportSpeed =
            Vector3.Dot(
                solverVelocity,
                tangent);

        bool forwardGuardApplied =
            solverTransportSpeed < 0f;

        float transportSpeed =
            Mathf.Max(
                0f,
                solverTransportSpeed);

        float lateralSpeed =
            Vector3.Dot(
                solverVelocity,
                lateral);

        float solverOscillationSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    solverVelocity,
                    outgoingAxis));

        float safeMaximumOscillationEnergy =
            Mathf.Max(
                0f,
                maximumOscillationEnergy);

        float maximumOscillationSpeed =
            Mathf.Sqrt(
                Mathf.Max(
                    0f,
                    2f *
                    safeMaximumOscillationEnergy /
                    EqualizerMass));

        float oscillationSpeed =
            Mathf.Min(
                solverOscillationSpeed,
                maximumOscillationSpeed);

        bool oscillationReduced =
            oscillationSpeed <
            solverOscillationSpeed -
            CleanConstraintEpsilon;

        float nT =
            Vector3.Dot(
                contactNormal,
                tangent);

        float nL =
            Vector3.Dot(
                contactNormal,
                lateral);

        float nD =
            Vector3.Dot(
                contactNormal,
                outgoingAxis);

        float separationSpeed =
            nT * transportSpeed +
            nL * lateralSpeed +
            nD * oscillationSpeed;

        if (separationSpeed <
            -CleanConstraintEpsilon)
        {
            float deficit =
                -separationSpeed;

            // 1) L is not part of canonical damping or Subject transport.
            // Remove only the part that pushes into the contacted face.
            float lateralContribution =
                nL *
                lateralSpeed;

            if (lateralContribution < 0f &&
                deficit > 0f)
            {
                float improvementAvailable =
                    -lateralContribution;

                float fraction =
                    Mathf.Clamp01(
                        deficit /
                        Mathf.Max(
                            improvementAvailable,
                            ImpactEnergyEpsilon));

                lateralSpeed *=
                    1f -
                    fraction;

                deficit -=
                    improvementAvailable *
                    fraction;
            }

            separationSpeed =
                nT * transportSpeed +
                nL * lateralSpeed +
                nD * oscillationSpeed;

            // 2) If Stable-N points out of the face, it may use the remaining
            // strict per-impact budget, never more than the already-contracted
            // E_requested supplied by ApplyImpactMap.
            if (separationSpeed <
                    -CleanConstraintEpsilon &&
                nD >
                    ImpactEnergyEpsilon &&
                oscillationSpeed <
                    maximumOscillationSpeed)
            {
                float speedNeeded =
                    -separationSpeed /
                    nD;

                float availableIncrease =
                    maximumOscillationSpeed -
                    oscillationSpeed;

                float addedSpeed =
                    Mathf.Min(
                        availableIncrease,
                        speedNeeded);

                oscillationSpeed +=
                    Mathf.Max(
                        0f,
                        addedSpeed);

                separationSpeed =
                    nT * transportSpeed +
                    nL * lateralSpeed +
                    nD * oscillationSpeed;
            }

            // If Stable-N itself points into the face, reducing N improves the
            // constraint and also removes energy.
            if (separationSpeed <
                    -CleanConstraintEpsilon &&
                nD <
                    -ImpactEnergyEpsilon &&
                oscillationSpeed > 0f)
            {
                float reduction =
                    Mathf.Min(
                        oscillationSpeed,
                        -separationSpeed /
                        (-nD));

                oscillationSpeed -=
                    Mathf.Max(
                        0f,
                        reduction);

                oscillationReduced =
                    true;

                separationSpeed =
                    nT * transportSpeed +
                    nL * lateralSpeed +
                    nD * oscillationSpeed;
            }

            // 3) Last physical degree of freedom is T.
            // A non-canonical wall may force forward transport to be reduced.
            // Never replace that loss by creating Stable-N energy.
            if (separationSpeed <
                    -CleanConstraintEpsilon &&
                nT <
                    -ImpactEnergyEpsilon &&
                transportSpeed > 0f)
            {
                float reduction =
                    Mathf.Min(
                        transportSpeed,
                        -separationSpeed /
                        (-nT));

                transportSpeed -=
                    Mathf.Max(
                        0f,
                        reduction);

                separationSpeed =
                    nT * transportSpeed +
                    nL * lateralSpeed +
                    nD * oscillationSpeed;
            }

            // Numerical / degenerate-face fallback:
            // zero only components that still point into the face.
            // The zero vector is always non-penetrating, so this cannot create
            // energy and guarantees a deterministic emergency result.
            if (separationSpeed <
                -CleanConstraintEpsilon)
            {
                if (nL * lateralSpeed < 0f)
                    lateralSpeed = 0f;

                if (nD * oscillationSpeed < 0f)
                {
                    oscillationSpeed = 0f;
                    oscillationReduced = true;
                }

                if (nT * transportSpeed < 0f)
                    transportSpeed = 0f;

                separationSpeed =
                    nT * transportSpeed +
                    nL * lateralSpeed +
                    nD * oscillationSpeed;
            }
        }

        Vector3 velocity =
            tangent *
            transportSpeed +
            lateral *
            lateralSpeed +
            outgoingAxis *
            oscillationSpeed;

        float finalSeparationSpeed =
            Vector3.Dot(
                velocity,
                contactNormal);

        if (finalSeparationSpeed < 0f &&
            finalSeparationSpeed >
            -CleanConstraintEpsilon)
        {
            finalSeparationSpeed =
                0f;
        }

        float finalOscillationEnergy =
            0.5f *
            EqualizerMass *
            oscillationSpeed *
            oscillationSpeed;

        // Hard invariant against numeric drift.
        finalOscillationEnergy =
            Mathf.Min(
                safeMaximumOscillationEnergy,
                Mathf.Max(
                    0f,
                    finalOscillationEnergy));

        float transportRetention =
            rawTransportSpeed >
            ImpactEnergyEpsilon
                ? transportSpeed /
                  rawTransportSpeed
                : 1f;

        bool severeTransportLoss =
            rawTransportSpeed >
            MinimumImpactNormalSpeed &&
            transportRetention <
            MinimumCleanTransportRetention;

        bool physicsClean =
            transportSpeed >=
                -CleanConstraintEpsilon &&
            finalOscillationEnergy <=
                safeMaximumOscillationEnergy +
                ImpactEnergyEpsilon &&
            finalSeparationSpeed >=
                -CleanConstraintEpsilon;

        return new CleanImpactSolveResult
        {
            velocity = velocity,
            preTransportSpeed = rawTransportSpeed,
            postTransportSpeed = transportSpeed,
            transportRetention = transportRetention,
            requestedOscillationSpeed = requestedOscillationSpeed,
            finalOscillationSpeed = oscillationSpeed,
            finalOscillationEnergy = finalOscillationEnergy,
            wallness =
                Mathf.Clamp01(
                    Mathf.Max(
                        0f,
                        -Vector3.Dot(
                            contactNormal,
                            tangent))),
            finalSeparationSpeed = finalSeparationSpeed,
            correctionSpeed =
                Vector3.Distance(
                    solverVelocity,
                    velocity),
            forwardGuardApplied = forwardGuardApplied,
            oscillationReduced = oscillationReduced,
            severeTransportLoss = severeTransportLoss,
            physicsClean = physicsClean,
            requiresEmergency = true,
            gameQuality01 =
                Mathf.Clamp01(
                    transportRetention)
        };
    }

    // ================================================================
    // Observer
    // ================================================================

    private void UpdateObserver()
    {
        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        positionErrorToBallVisual =
            Vector3.Distance(
                ballVisualEqualizer.position,
                ballVisual.position);

        velocityErrorToBallVisual =
            Vector3.Distance(
                ballVisualEqualizer.velocity,
                ballVisual.velocity);

        float speed =
            ballVisualEqualizer.velocity.magnitude;

        currentKineticEnergy =
            0.5f *
            EqualizerMass *
            speed *
            speed;

        UpdateSubjectConvergenceDiagnostics();
    }


    // ================================================================
    // Impact geometry / orthogonal decomposition
    // ================================================================

    private float EstimateRequestedOscillationSpeedForSelection(
        bool envelopeContact,
        Vector3 incidentVelocity)
    {
        if (!oscillationFrame.valid)
            return 0f;

        Vector3 outgoingAxis =
            envelopeContact
                ? -oscillationFrame.normal.normalized
                : oscillationFrame.normal.normalized;

        float incomingSpeed =
            Mathf.Max(
                0f,
                -Vector3.Dot(
                    incidentVelocity,
                    outgoingAxis));

        if (incomingSpeed <
            MinimumImpactNormalSpeed)
        {
            return 0f;
        }

        float incomingEnergy =
            0.5f *
            EqualizerMass *
            incomingSpeed *
            incomingSpeed;

        float restitution =
            Mathf.Clamp(
                impactNormalRestitution,
                0f,
                0.985f);

        float impactContraction =
            restitution *
            restitution;

        float requestedEnergy =
            incomingEnergy *
            impactContraction;

        // Selection uses the same single-reservoir budget as ApplyImpactMap.
        // This prevents a high-N Unity solver/contact sample from advertising
        // more outgoing N energy than the canonical ledger permits.
        if (currentCanonicalOscillationEnergy >
            ImpactEnergyEpsilon)
        {
            requestedEnergy =
                Mathf.Min(
                    requestedEnergy,
                    currentCanonicalOscillationEnergy *
                    impactContraction);
        }

        return Mathf.Sqrt(
            Mathf.Max(
                0f,
                2f *
                requestedEnergy /
                EqualizerMass));
    }


    private ContactSelectionResult SelectDominantImpactContact(
        Collision collision,
        bool envelopeContact)
    {
        ContactPoint first =
            collision.GetContact(0);

        Vector3 incidentVelocity =
            -collision.relativeVelocity;

        if (!oscillationFrame.valid)
        {
            // Stable-frame authority is unavailable.  Preserve the old
            // dominant-approach selection as a deterministic fallback.
            ContactPoint best =
                first;

            float bestApproach =
                -1f;

            for (int i = 0;
                 i < collision.contactCount;
                 i++)
            {
                ContactPoint candidate =
                    collision.GetContact(i);

                Vector3 normal =
                    candidate.normal.sqrMagnitude >
                    ImpactEnergyEpsilon
                        ? candidate.normal.normalized
                        : Vector3.up;

                float approach =
                    Mathf.Abs(
                        Vector3.Dot(
                            incidentVelocity,
                            normal));

                if (approach >
                    bestApproach)
                {
                    bestApproach =
                        approach;

                    best =
                        candidate;
                }
            }

            return new ContactSelectionResult
            {
                contact = best,
                canonicalContact = false,
                authority01 = 0f,
                approach01 = 0f,
                oscillationAlignment01 = 0f,
                wallness01 = 0f,
                forwardCompatibility01 = 0f,
                predictedSeparationSpeed = 0f,
                estimatedRequestedOscillationSpeed = 0f
            };
        }

        Vector3 tangent =
            oscillationFrame.tangent.normalized;

        Vector3 lateral =
            oscillationFrame.lateral.normalized;

        Vector3 outgoingAxis =
            envelopeContact
                ? -oscillationFrame.normal.normalized
                : oscillationFrame.normal.normalized;

        float requestedOscillationSpeed =
            EstimateRequestedOscillationSpeedForSelection(
                envelopeContact,
                incidentVelocity);

        float incidentSpeed =
            Mathf.Max(
                incidentVelocity.magnitude,
                ImpactEnergyEpsilon);

        float transportSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    incidentVelocity,
                    tangent));

        float lateralSpeed =
            Vector3.Dot(
                incidentVelocity,
                lateral);

        ContactSelectionResult bestCanonical =
            default;

        ContactSelectionResult bestFallback =
            default;

        float bestCanonicalScore =
            -1f;

        float bestFallbackScore =
            -1f;

        for (int i = 0;
             i < collision.contactCount;
             i++)
        {
            ContactPoint candidate =
                collision.GetContact(i);

            Vector3 rawNormal =
                candidate.normal.sqrMagnitude >
                ImpactEnergyEpsilon
                    ? candidate.normal.normalized
                    : Vector3.up;

            Vector3 normal =
                Vector3.Dot(
                    incidentVelocity,
                    rawNormal) > 0f
                    ? -rawNormal
                    : rawNormal;

            float inwardSpeed =
                Mathf.Max(
                    0f,
                    -Vector3.Dot(
                        incidentVelocity,
                        normal));

            float approach01 =
                Mathf.Clamp01(
                    inwardSpeed /
                    incidentSpeed);

            float nT =
                Vector3.Dot(
                    normal,
                    tangent);

            float nL =
                Vector3.Dot(
                    normal,
                    lateral);

            float nD =
                Vector3.Dot(
                    normal,
                    outgoingAxis);

            float alignment01 =
                Mathf.Clamp01(
                    nD);

            float wallness01 =
                Mathf.Clamp01(
                    Mathf.Max(
                        0f,
                        -nT));

            // For authority selection, harmful L may be dropped and harmful
            // oscillation may be reduced to zero.  T is never sacrificed here.
            float helpfulLateralSeparation =
                Mathf.Max(
                    0f,
                    nL *
                    lateralSpeed);

            float helpfulOscillationSeparation =
                Mathf.Max(
                    0f,
                    nD) *
                requestedOscillationSpeed;

            float predictedSeparationSpeed =
                nT *
                transportSpeed +
                helpfulLateralSeparation +
                helpfulOscillationSeparation;

            bool preservesTransport =
                predictedSeparationSpeed >=
                -CleanConstraintEpsilon;

            float forwardCompatibility01 =
                preservesTransport
                    ? 1f
                    : Mathf.Clamp01(
                        1f -
                        wallness01);

            float authority01 =
                approach01 *
                alignment01 *
                Mathf.Clamp01(
                    1f -
                    wallness01);

            ContactSelectionResult result =
                new ContactSelectionResult
                {
                    contact = candidate,
                    canonicalContact =
                        preservesTransport &&
                        authority01 >
                        ContactAuthorityEpsilon,
                    authority01 = authority01,
                    approach01 = approach01,
                    oscillationAlignment01 =
                        alignment01,
                    wallness01 = wallness01,
                    forwardCompatibility01 =
                        forwardCompatibility01,
                    predictedSeparationSpeed =
                        predictedSeparationSpeed,
                    estimatedRequestedOscillationSpeed =
                        requestedOscillationSpeed
                };

            if (result.canonicalContact &&
                authority01 >
                bestCanonicalScore)
            {
                bestCanonicalScore =
                    authority01;

                bestCanonical =
                    result;
            }

            // If no canonical face exists, keep the least-bad observation for
            // emergency diagnostics.  It is NOT promoted into the Poincare map.
            float fallbackScore =
                approach01 *
                Mathf.Max(
                    0.05f,
                    alignment01) *
                Mathf.Max(
                    0.05f,
                    1f -
                    wallness01);

            if (fallbackScore >
                bestFallbackScore)
            {
                bestFallbackScore =
                    fallbackScore;

                bestFallback =
                    result;

                bestFallback.canonicalContact =
                    false;
            }
        }

        return bestCanonicalScore >= 0f
            ? bestCanonical
            : bestFallback;
    }


    private ContactFrame BuildContactFrame(
        Collision collision,
        ContactPoint contact,
        bool envelopeContact)
    {
        Vector3 rawNormal =
            contact.normal.sqrMagnitude >
            ImpactEnergyEpsilon
                ? contact.normal.normalized
                : Vector3.up;

        Vector3 incidentVelocity =
            -collision.relativeVelocity;

        float rawSignedNormalSpeed =
            Vector3.Dot(
                incidentVelocity,
                rawNormal);

        bool normalWasFlipped =
            rawSignedNormalSpeed > 0f;

        Vector3 normal =
            normalWasFlipped
                ? -rawNormal
                : rawNormal;

        float signedNormalSpeed =
            Vector3.Dot(
                incidentVelocity,
                normal);

        float inwardNormalSpeed =
            Mathf.Max(
                0f,
                -signedNormalSpeed);

        Vector3 tangentVelocity =
            incidentVelocity -
            normal *
            signedNormalSpeed;

        float tangentSpeed =
            tangentVelocity.magnitude;

        Vector3 inwardNormalVelocity =
            -normal *
            inwardNormalSpeed;

        float speed =
            incidentVelocity.magnitude;

        float incidenceAngleDeg =
            Mathf.Atan2(
                inwardNormalSpeed,
                Mathf.Max(
                    tangentSpeed,
                    ImpactEnergyEpsilon)) *
            Mathf.Rad2Deg;

        float mass =
            EqualizerMass;

        float totalEnergy =
            0.5f *
            mass *
            speed *
            speed;

        float tangentEnergy =
            0.5f *
            mass *
            tangentSpeed *
            tangentSpeed;

        float normalEnergy =
            0.5f *
            mass *
            inwardNormalSpeed *
            inwardNormalSpeed;

        float normalEnergyRatio =
            totalEnergy >
            ImpactEnergyEpsilon
                ? normalEnergy /
                  totalEnergy
                : 0f;

        bool oscillationFrameValid =
            oscillationFrame.valid;

        Vector3 oscillationNormal =
            oscillationFrameValid
                ? oscillationFrame.normal
                : normal;

        Vector3 oscillationOutgoingAxis =
            oscillationFrameValid
                ? (envelopeContact
                    ? -oscillationNormal
                    : oscillationNormal)
                : normal;

        float oscillationIncomingSpeed =
            Mathf.Max(
                0f,
                -Vector3.Dot(
                    incidentVelocity,
                    oscillationOutgoingAxis));

        float oscillationIncomingEnergy =
            0.5f *
            mass *
            oscillationIncomingSpeed *
            oscillationIncomingSpeed;

        float oscillationIncomingEnergyRatio =
            totalEnergy > ImpactEnergyEpsilon
                ? oscillationIncomingEnergy /
                  totalEnergy
                : 0f;

        float contactOscillationAlignment =
            Mathf.Clamp01(
                Vector3.Dot(
                    normal,
                    oscillationOutgoingAxis));

        float outwardNormalSpeedAfterSolver =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    ballVisualEqualizer.velocity,
                    normal));

        float estimatedRestitution =
            inwardNormalSpeed > 0.0001f
                ? outwardNormalSpeedAfterSolver /
                  inwardNormalSpeed
                : 0f;

        float normalImpulse =
            Mathf.Abs(
                Vector3.Dot(
                    collision.impulse,
                    normal));

        return new ContactFrame
        {
            valid = true,
            envelopeContact = envelopeContact,
            stairLikeContact = IsStairLikeCollision(collision),
            point = contact.point,
            rawNormal = rawNormal,
            normal = normal,
            normalWasFlipped = normalWasFlipped,
            incidentVelocity = incidentVelocity,
            tangentVelocity = tangentVelocity,
            inwardNormalVelocity = inwardNormalVelocity,
            speed = speed,
            tangentSpeed = tangentSpeed,
            inwardNormalSpeed = inwardNormalSpeed,
            outwardNormalSpeedAfterSolver = outwardNormalSpeedAfterSolver,
            incidenceAngleDeg = incidenceAngleDeg,
            totalKineticEnergy = totalEnergy,
            tangentEnergy = tangentEnergy,
            normalEnergy = normalEnergy,
            normalEnergyRatio = normalEnergyRatio,
            estimatedRestitution = estimatedRestitution,
            normalImpulse = normalImpulse,
            impactMapApplied = false,
            mappedOutgoingVelocity = ballVisualEqualizer.velocity,
            mappedOutgoingNormalSpeed = 0f,
            mappedOutgoingNormalEnergy = 0f,
            mappedNormalEnergyRatio = 0f,
            mappedEffectiveRestitution = 0f,
            previousCanonicalNormalEnergy = -1f,
            canonicalEnergyCeiling = -1f,
            reservoirEnergyRatio = -1f,
            oscillationFrameValid = oscillationFrameValid,
            oscillationNormal = oscillationNormal,
            oscillationOutgoingAxis = oscillationOutgoingAxis,
            oscillationIncomingSpeed = oscillationIncomingSpeed,
            oscillationIncomingEnergy = oscillationIncomingEnergy,
            oscillationIncomingEnergyRatio = oscillationIncomingEnergyRatio,
            contactOscillationAlignment = contactOscillationAlignment,
            contactConstraintCorrectionSpeed = 0f,
            preTransportSpeed = 0f,
            postTransportSpeed = 0f,
            transportRetention = 1f,
            requestedOutgoingOscillationSpeed = 0f,
            finalOutgoingOscillationSpeed = 0f,
            finalOutgoingOscillationEnergy = 0f,
            contactWallness = 0f,
            finalSeparationSpeed = 0f,
            forwardGuardApplied = false,
            oscillationReducedByConstraint = false,
            severeTransportLoss = false,
            contactAuthority = 0f,
            contactApproach01 = 0f,
            contactForwardCompatibility01 = 0f,
            contactPredictedSeparationSpeed = 0f,
            canonicalContact = false,
            emergencyContactGuard = false,
            physicsClean = false,
            dampingSuccess = false,
            transportSuccess = false,
            subjectConverging = false,
            gameImpactSuccess = false
        };
    }


    // ================================================================
    // Impact Map + Energy Contraction
    // ================================================================

    private void ResetImpactMapState()
    {
        impactCount = 0;
        lowerImpactCount = 0;
        upperImpactCount = 0;

        lastMappedNormalEnergy = 0f;
        lastImpactNormalEnergyRatio = 0f;
        lastReservoirEnergyRatio = 0f;
        lastEffectiveRestitution = 0f;
        lastMappedOutgoingVelocity = Vector3.zero;

        lastContactConstraintCorrectionSpeed = 0f;

        physicsCleanCount = 0;
        canonicalImpactCount = 0;
        emergencyImpactCount = 0;
        gameImpactSuccessCount = 0;
        forwardGuardCount = 0;
        oscillationConstraintReductionCount = 0;
        severeTransportLossCount = 0;
        physicsCleanRate = 0f;
        gameImpactSuccessRate = 0f;
        averageGameImpactQuality = 0f;
        lastGameImpactQuality = 0f;
        accumulatedGameImpactQuality = 0f;
        lastPreTransportSpeed = 0f;
        lastPostTransportSpeed = 0f;
        lastTransportRetention = 1f;
        lastContactWallness = 0f;
        lastFinalSeparationSpeed = 0f;
        lastFinalOscillationEnergy = 0f;
    }


    private bool IsStairLikeCollision(
        Collision collision)
    {
        return
            collision != null &&
            IsStairwayCollider(
                collision.collider);
    }


    private void ApplyImpactMap(
        ref ContactFrame frame)
    {
        if (!frame.valid ||
            synchronized ||
            !ballVisualEqualizer)
        {
            return;
        }

        bool useOscillationMap =
            frame.oscillationFrameValid;

        float incomingSpeed =
            useOscillationMap
                ? frame.oscillationIncomingSpeed
                : frame.inwardNormalSpeed;

        float incomingEnergy =
            useOscillationMap
                ? frame.oscillationIncomingEnergy
                : frame.normalEnergy;

        // Glancing / incidental contact does not manufacture a bounce and does
        // not advance the canonical damping ledger.
        if (incomingSpeed <
            MinimumImpactNormalSpeed)
        {
            return;
        }

        float restitution =
            Mathf.Clamp(
                impactNormalRestitution,
                0f,
                0.985f);

        float impactContraction =
            restitution *
            restitution;

        float localMappedNormalEnergy =
            incomingEnergy *
            impactContraction;

        float previousCanonicalNormalEnergy =
            Mathf.Max(
                0f,
                currentCanonicalOscillationEnergy);

        // Single monotone Stable-N reservoir:
        //
        //     E_next <= e^2 * E_current
        //     E_next <= e^2 * E_in
        //
        // The first inequality makes damping obvious even when constant
        // Stable-N acceleration restores kinetic energy between impacts.
        // The second is the ordinary restitution limit of the current contact.
        float canonicalEnergyCeiling =
            previousCanonicalNormalEnergy >
                ImpactEnergyEpsilon
                ? previousCanonicalNormalEnergy *
                  impactContraction
                : localMappedNormalEnergy;

        float requestedMappedNormalEnergy =
            Mathf.Min(
                localMappedNormalEnergy,
                canonicalEnergyCeiling);

        requestedMappedNormalEnergy =
            Mathf.Max(
                0f,
                requestedMappedNormalEnergy);

        float mass =
            EqualizerMass;

        // Emergency contacts are constraint events, not energy sources.
        // The current canonical reservoir is a historical upper bound, but the
        // present impact has already requested a contracted output:
        //
        //     E_requested = min(e^2 * E_in, e^2 * E_canonical,current)
        //
        // Emergency geometry may change the direction needed for nonpenetration,
        // but it must never relax that damping request:
        //
        //     E_emergency,out
        //         <= min(E_canonical,current, E0, E_requested)
        //
        // Therefore even a non-canonical boundary face cannot increase Stable-N
        // energy above the amount allowed by this impact.
        float emergencyOscillationEnergyCeiling =
            Mathf.Max(
                0f,
                currentCanonicalOscillationEnergy);

        float sourceEnergyCeiling =
            Mathf.Max(
                0f,
                releaseFrame.sourceEnergy);

        if (sourceEnergyCeiling >
            ImpactEnergyEpsilon)
        {
            emergencyOscillationEnergyCeiling =
                Mathf.Min(
                    emergencyOscillationEnergyCeiling,
                    sourceEnergyCeiling);
        }

        // Strict contraction invariant for Emergency contacts.
        // If the normal impact map asks for zero energy, Emergency also receives
        // zero Stable-N budget and must satisfy nonpenetration by removing L/T
        // components instead of manufacturing N energy.
        emergencyOscillationEnergyCeiling =
            Mathf.Min(
                emergencyOscillationEnergyCeiling,
                requestedMappedNormalEnergy);

        float requestedOutgoingNormalSpeed =
            Mathf.Sqrt(
                Mathf.Max(
                    0f,
                    2f *
                    requestedMappedNormalEnergy /
                    mass));

        if (requestedOutgoingNormalSpeed <
            MinimumImpactNormalSpeed)
        {
            requestedOutgoingNormalSpeed =
                0f;

            requestedMappedNormalEnergy =
                0f;
        }

        Vector3 mappedOutgoingVelocity =
            ballVisualEqualizer.velocity;

        float finalOutgoingNormalSpeed =
            0f;

        float finalMappedNormalEnergy =
            0f;

        float constraintCorrectionSpeed =
            0f;

        float preTransportSpeed =
            0f;

        float postTransportSpeed =
            0f;

        float transportRetention =
            1f;

        float contactWallness =
            frame.contactWallness;

        float finalSeparationSpeed =
            0f;

        bool forwardGuardApplied =
            false;

        bool oscillationReducedByConstraint =
            false;

        bool severeTransportLoss =
            false;

        bool physicsClean =
            false;

        bool canonicalApplied =
            false;

        bool emergencyUsed =
            false;

        float gameImpactQuality =
            0f;

        if (useOscillationMap)
        {
            CleanImpactSolveResult solved;

            if (frame.canonicalContact)
            {
                solved =
                    SolveCleanImpactVelocity(
                        frame,
                        requestedOutgoingNormalSpeed);

                if (solved.requiresEmergency ||
                    !solved.physicsClean)
                {
                    solved =
                        BuildEmergencyContactResult(
                            frame,
                            requestedOutgoingNormalSpeed,
                            emergencyOscillationEnergyCeiling);

                    emergencyUsed =
                        true;
                }
                else
                {
                    canonicalApplied =
                        true;
                }
            }
            else
            {
                solved =
                    BuildEmergencyContactResult(
                        frame,
                        requestedOutgoingNormalSpeed,
                        emergencyOscillationEnergyCeiling);

                emergencyUsed =
                    true;
            }

            mappedOutgoingVelocity =
                solved.velocity;

            finalOutgoingNormalSpeed =
                solved.finalOscillationSpeed;

            finalMappedNormalEnergy =
                solved.finalOscillationEnergy;

            constraintCorrectionSpeed =
                solved.correctionSpeed;

            preTransportSpeed =
                solved.preTransportSpeed;

            postTransportSpeed =
                solved.postTransportSpeed;

            transportRetention =
                solved.transportRetention;

            contactWallness =
                solved.wallness;

            finalSeparationSpeed =
                solved.finalSeparationSpeed;

            forwardGuardApplied =
                solved.forwardGuardApplied;

            oscillationReducedByConstraint =
                solved.oscillationReduced;

            severeTransportLoss =
                solved.severeTransportLoss;

            physicsClean =
                solved.physicsClean;

            gameImpactQuality =
                solved.gameQuality01;
        }
        else
        {
            // Stable frame unavailable: keep the deterministic normal fallback,
            // but do not promote it into the canonical Poincare ledger.
            mappedOutgoingVelocity =
                frame.tangentVelocity +
                frame.normal *
                requestedOutgoingNormalSpeed;

            finalOutgoingNormalSpeed =
                requestedOutgoingNormalSpeed;

            finalMappedNormalEnergy =
                requestedMappedNormalEnergy;

            finalSeparationSpeed =
                Vector3.Dot(
                    mappedOutgoingVelocity,
                    frame.normal);

            physicsClean =
                finalSeparationSpeed >=
                -CleanConstraintEpsilon;

            gameImpactQuality =
                physicsClean
                    ? 1f
                    : 0f;
        }

        if (canonicalApplied)
        {
            // Canonical geometry may only contract the requested damping map.
            finalMappedNormalEnergy =
                Mathf.Min(
                    requestedMappedNormalEnergy,
                    Mathf.Max(
                        0f,
                        finalMappedNormalEnergy));
        }
        else
        {
            // Emergency / fallback values are observations, not ledger writes.
            finalMappedNormalEnergy =
                Mathf.Max(
                    0f,
                    finalMappedNormalEnergy);
        }

        ballVisualEqualizer.velocity =
            mappedOutgoingVelocity;

        ballVisualEqualizer.WakeUp();

        UpdateSubjectConvergenceDiagnostics();

        impactCount++;

        if (frame.envelopeContact)
            upperImpactCount++;
        else
            lowerImpactCount++;

        // T is observation-only. Real Upper/Stairway impacts anchor phase below;
        // no T/2 deadline is consumed and no boundary steering is applied.

        if (canonicalApplied)
        {
            canonicalImpactCount++;

            UpdateObservedCyclePeriod(
                frame.envelopeContact);

            bool acceptedForExperiment =
                frame.canonicalContact &&
                physicsClean &&
                !severeTransportLoss &&
                transportRetention >=
                    ExperimentMinimumTransportRetention &&
                frame.contactOscillationAlignment >=
                    ExperimentMinimumContactAlignment;

            ObservePeriodPhaseExtremum(
                frame.envelopeContact,
                acceptedForExperiment);

            TryAcceptCanonicalImpactExtremum(
                frame,
                mappedOutgoingVelocity,
                frame.envelopeContact,
                acceptedForExperiment);

            if (!frame.envelopeContact &&
                acceptedForExperiment)
            {
                ObserveAcceptedStairGeometryPeriod(
                    frame.point,
                    preTransportSpeed);
            }

            // Feasibility follows the actually applied canonical energy only.
            currentCanonicalOscillationEnergy =
                finalMappedNormalEnergy;

            UpdateCanonicalEnergyCoupling();

            if (finalMappedNormalEnergy >
                incomingEnergy +
                ImpactEnergyEpsilon)
            {
                releaseDampingViolationCount++;
            }

            if (previousCanonicalNormalEnergy >
                    ImpactEnergyEpsilon &&
                finalMappedNormalEnergy >
                    canonicalEnergyCeiling +
                    ImpactEnergyEpsilon)
            {
                releaseDampingViolationCount++;
            }
        }
        else if (emergencyUsed)
        {
            emergencyImpactCount++;

            // Strict Emergency is now provably contractive:
            // Eout <= E_requested <= e^2 * Ein.
            // Therefore the scalar canonical reservoir must follow the energy that
            // was actually applied, otherwise Envelope epsilon would keep stale
            // pre-emergency energy.  No boundary-specific history exists; the
            // single canonical reservoir alone remains authoritative.
            bool emergencyEnergySafe =
                useOscillationMap &&
                physicsClean &&
                finalMappedNormalEnergy <=
                    requestedMappedNormalEnergy +
                    ImpactEnergyEpsilon;

            if (emergencyEnergySafe)
            {
                currentCanonicalOscillationEnergy =
                    currentCanonicalOscillationEnergy >
                    ImpactEnergyEpsilon
                        ? Mathf.Min(
                            currentCanonicalOscillationEnergy,
                            finalMappedNormalEnergy)
                        : finalMappedNormalEnergy;

                UpdateCanonicalEnergyCoupling();

                if (finalMappedNormalEnergy >
                    incomingEnergy +
                    ImpactEnergyEpsilon)
                {
                    releaseDampingViolationCount++;
                }
            }
        }
        bool dampingSuccess =
            canonicalApplied &&
            finalMappedNormalEnergy <=
                localMappedNormalEnergy +
                ImpactEnergyEpsilon &&
            finalMappedNormalEnergy <=
                canonicalEnergyCeiling +
                ImpactEnergyEpsilon;

        bool transportSuccess =
            preTransportSpeed <=
                MinimumImpactNormalSpeed ||
            transportRetention >=
                MinimumGameTransportRetention;

        float subjectGapRate =
            subjectTransportSpeed -
            equalizerTransportSpeed;

        bool subjectConverging =
            !usingBallVisualAsSubjectProxy &&
            subjectTransform &&
            (Mathf.Abs(
                 subjectTransportGap) <=
                 Mathf.Max(
                     subjectTransportDeadZone,
                     CleanConstraintEpsilon) ||
             subjectTransportGap *
             subjectGapRate <= 0f);

        bool gameImpactSuccess =
            canonicalApplied &&
            physicsClean &&
            dampingSuccess &&
            transportSuccess;

        minimumReleaseTransportRetention =
            Mathf.Min(
                minimumReleaseTransportRetention,
                Mathf.Clamp01(
                    transportRetention));

        if (physicsClean)
            physicsCleanCount++;

        if (gameImpactSuccess)
            gameImpactSuccessCount++;

        if (forwardGuardApplied)
            forwardGuardCount++;

        if (oscillationReducedByConstraint)
            oscillationConstraintReductionCount++;

        if (severeTransportLoss)
            severeTransportLossCount++;

        physicsCleanRate =
            impactCount > 0
                ? (float)physicsCleanCount /
                  impactCount
                : 0f;

        gameImpactSuccessRate =
            impactCount > 0
                ? (float)gameImpactSuccessCount /
                  impactCount
                : 0f;

        lastGameImpactQuality =
            gameImpactQuality;

        accumulatedGameImpactQuality +=
            gameImpactQuality;

        averageGameImpactQuality =
            impactCount > 0
                ? accumulatedGameImpactQuality /
                  impactCount
                : 0f;

        lastContactConstraintCorrectionSpeed =
            constraintCorrectionSpeed;

        lastPreTransportSpeed =
            preTransportSpeed;

        lastPostTransportSpeed =
            postTransportSpeed;

        lastTransportRetention =
            transportRetention;

        lastContactWallness =
            contactWallness;

        lastFinalSeparationSpeed =
            finalSeparationSpeed;

        lastFinalOscillationEnergy =
            finalMappedNormalEnergy;

        float mappedNormalEnergyRatio =
            incomingEnergy >
            ImpactEnergyEpsilon
                ? finalMappedNormalEnergy /
                  incomingEnergy
                : 0f;

        float effectiveRestitution =
            incomingSpeed >
            ImpactEnergyEpsilon
                ? finalOutgoingNormalSpeed /
                  incomingSpeed
                : 0f;

        float reservoirEnergyRatio =
            canonicalApplied &&
            previousCanonicalNormalEnergy >
            ImpactEnergyEpsilon
                ? finalMappedNormalEnergy /
                  previousCanonicalNormalEnergy
                : -1f;

        lastMappedNormalEnergy =
            canonicalApplied
                ? finalMappedNormalEnergy
                : lastMappedNormalEnergy;

        lastImpactNormalEnergyRatio =
            mappedNormalEnergyRatio;

        lastReservoirEnergyRatio =
            reservoirEnergyRatio;

        lastEffectiveRestitution =
            effectiveRestitution;

        lastMappedOutgoingVelocity =
            mappedOutgoingVelocity;

        frame.impactMapApplied =
            canonicalApplied ||
            !useOscillationMap;

        frame.mappedOutgoingVelocity =
            mappedOutgoingVelocity;

        frame.requestedOutgoingOscillationSpeed =
            requestedOutgoingNormalSpeed;

        frame.mappedOutgoingNormalSpeed =
            finalOutgoingNormalSpeed;

        frame.mappedOutgoingNormalEnergy =
            finalMappedNormalEnergy;

        frame.mappedNormalEnergyRatio =
            mappedNormalEnergyRatio;

        frame.mappedEffectiveRestitution =
            effectiveRestitution;

        frame.previousCanonicalNormalEnergy =
            previousCanonicalNormalEnergy;

        frame.canonicalEnergyCeiling =
            canonicalEnergyCeiling;

        frame.reservoirEnergyRatio =
            reservoirEnergyRatio;

        frame.contactConstraintCorrectionSpeed =
            constraintCorrectionSpeed;

        frame.preTransportSpeed =
            preTransportSpeed;

        frame.postTransportSpeed =
            postTransportSpeed;

        frame.transportRetention =
            transportRetention;

        frame.finalOutgoingOscillationSpeed =
            finalOutgoingNormalSpeed;

        frame.finalOutgoingOscillationEnergy =
            finalMappedNormalEnergy;

        frame.contactWallness =
            contactWallness;

        frame.finalSeparationSpeed =
            finalSeparationSpeed;

        frame.forwardGuardApplied =
            forwardGuardApplied;

        frame.oscillationReducedByConstraint =
            oscillationReducedByConstraint;

        frame.severeTransportLoss =
            severeTransportLoss;

        frame.emergencyContactGuard =
            emergencyUsed;

        frame.physicsClean =
            physicsClean;

        frame.dampingSuccess =
            dampingSuccess;

        frame.transportSuccess =
            transportSuccess;

        frame.subjectConverging =
            subjectConverging;

        frame.gameImpactSuccess =
            gameImpactSuccess;

        Debug.Log(
            $"[EQUALIZER IMPACT MAP] " +
            $"index={impactCount} " +
            $"kind={(frame.envelopeContact ? "UpperEnvelope" : "Stairway")} " +
            $"basis={(useOscillationMap ? "StableOscillation" : "ContactNormalFallback")} " +
            $"stairLike={frame.stairLikeContact} " +
            $"canonical={canonicalApplied} " +
            $"emergency={emergencyUsed} " +
            $"authority={frame.contactAuthority:F4} " +
            $"authorityApproach={frame.contactApproach01:F4} " +
            $"authorityForward={frame.contactForwardCompatibility01:F4} " +
            $"authorityPredSep={frame.contactPredictedSeparationSpeed:F4}m/s " +
            $"EinContactN={frame.normalEnergy:F6}J " +
            $"EinOsc={incomingEnergy:F6}J " +
            $"localEoutOsc={localMappedNormalEnergy:F6}J " +
            $"requestedEoutOsc={requestedMappedNormalEnergy:F6}J " +
            $"prevCanonicalOscE={previousCanonicalNormalEnergy:F6}J " +
            $"canonicalCeilingOsc={canonicalEnergyCeiling:F6}J " +
            $"EoutOsc={finalMappedNormalEnergy:F6}J " +
            $"canonicalOscE={currentCanonicalOscillationEnergy:F6}J " +
            $"emergencyCeilingOsc={emergencyOscillationEnergyCeiling:F6}J " +
            $"impactRatio={mappedNormalEnergyRatio:F6} " +
            $"reservoirRatio={reservoirEnergyRatio:F6} " +
            $"e={restitution:F4} " +
            $"alignment={frame.contactOscillationAlignment:F4} " +
            $"wallness={contactWallness:F4} " +
            $"preVT={preTransportSpeed:F4}m/s " +
            $"postVT={postTransportSpeed:F4}m/s " +
            $"transportRetention={transportRetention:F4} " +
            $"vOscRequested={requestedOutgoingNormalSpeed:F4}m/s " +
            $"vOscFinal={finalOutgoingNormalSpeed:F4}m/s " +
            $"separation={finalSeparationSpeed:F4}m/s " +
            $"constraintChange={constraintCorrectionSpeed:F4}m/s " +
            $"forwardGuard={forwardGuardApplied} " +
            $"oscReduced={oscillationReducedByConstraint} " +
            $"severeTransportLoss={severeTransportLoss} " +
            $"physicsClean={physicsClean} " +
            $"physicsCleanRate={physicsCleanRate:P1} " +
            $"dampingSuccess={dampingSuccess} " +
            $"transportSuccess={transportSuccess} " +
            $"subjectConverging={subjectConverging} " +
            $"gameImpactSuccess={gameImpactSuccess} " +
            $"gameImpactSuccessRate={gameImpactSuccessRate:P1} " +
            $"gameQuality={gameImpactQuality:F3} " +
            $"gameQualityAvg={averageGameImpactQuality:F3} " +
            $"remainT={releaseFeasibility.remainingTransportDistance:F3}m " +
            $"timeToLimit={releaseFeasibility.availableTime:F3}s " +
            $"cyclesAvailable={releaseFeasibility.availableCycles:F2} " +
            $"predResidualE={releaseFeasibility.predictedResidualOscillationEnergy:F3}J " +
            $"dampingFeas={releaseFeasibility.dampingFeasibility01:F3} " +
            $"subjectFeas={releaseFeasibility.subjectConvergenceFeasibility:F3} " +
            $"subjectDistance={subjectDistance:F4}m " +
            $"subjectGapT={subjectTransportGap:F4}m " +
            $"subjectVT={subjectTransportSpeed:F4}m/s " +
            $"equalizerVT={equalizerTransportSpeed:F4}m/s " +
            $"subjectVelocitySource={subjectVelocitySource} " +
            $"velocityOut={mappedOutgoingVelocity:F4}",
            this);
    }


    // ================================================================
    // Hybrid state machine
    // ================================================================

    private void UpdateDynamicPhaseFromContacts()
    {
        if (synchronized)
            return;

        if (upperContactCount > 0)
        {
            if (phase !=
                EqualizerPhase.UpperContact)
            {
                TransitionTo(
                    EqualizerPhase.UpperContact,
                    "EnvelopeContactActive");
            }

            return;
        }

        if (lowerContactCount > 0)
        {
            if (phase !=
                EqualizerPhase.LowerContact)
            {
                TransitionTo(
                    EqualizerPhase.LowerContact,
                    "LowerContactActive");
            }

            return;
        }

        if (phase ==
                EqualizerPhase.LowerContact ||
            phase ==
                EqualizerPhase.UpperContact)
        {
            TransitionTo(
                EqualizerPhase.HopperFlight,
                "BetweenContacts");

            return;
        }

        if (phase ==
            EqualizerPhase.FreeFlight)
        {
            return;
        }

        if (phase ==
            EqualizerPhase.HopperFlight)
        {
            return;
        }
    }


    private void TransitionTo(
        EqualizerPhase next,
        string reason)
    {
        if (phase == next)
            return;

        EqualizerPhase previous =
            phase;

        phase =
            next;

        phaseRuntime =
            next.ToString();

        phaseElapsed =
            0f;

        Debug.Log(
            $"[EQUALIZER PHASE] " +
            $"{previous} -> {next} " +
            $"reason={reason}",
            this);
    }


    // ================================================================
    // Visual-frame turn mapping (energy-preserving)
    // ================================================================
    // The turn is treated as a change of visual coordinates, not as an impact.
    // Scalars that define damping are deliberately untouched:
    //   - releaseFrame.sourceEnergy (E0)
    //   - canonicalReferenceHeight (H0)
    //   - currentCanonicalOscillationEnergy
    //   - currentCanonicalEnergyRatio (epsilon)
    //   - impact counters / Poincare history
    //   - phase / phaseElapsed
    // Only world-space points and vectors are transported by the incremental
    // rigid map.
    public void PrepareForVisualFrameTurnMapping()
    {
        // Compatibility API only. Stage turns are expected after TerminalRejoin
        // has returned the Equalizer to Synchronized state.
        ResolveReferences();
    }
    public void ApplyVisualFrameTurnDelta(
        Vector3 pivot,
        Quaternion deltaTurn)
    {
        // Compatibility API only. Do not transport a live oscillator through a
        // turn. BallVisualSlopeDrive completes TerminalRejoin first, after which
        // the kinematic Equalizer simply follows BallVisual.
        ResolveReferences();

        if (synchronized)
            CopyBallVisualPose();
    }


    public void RefreshVisualCollisionOwnership()
    {
        ResolveReferences();

        if (!ballVisualEqualizerCollider)
            return;

        Collider[] colliders =
            FindObjectsByType<Collider>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);

        foreach (Collider other in colliders)
        {
            if (!other ||
                other == ballVisualEqualizerCollider ||
                other.attachedRigidbody == ballVisualEqualizer)
            {
                continue;
            }

            bool dampingBoundary =
                IsEqualizerDampingBoundaryCollider(other);

            // Dedicated channel invariant:
            //   Upper = generated NegativeEnvelope
            //   Lower = real Stairway Collider
            // Everything else remains isolated from the Equalizer.
            Physics.IgnoreCollision(
                ballVisualEqualizerCollider,
                other,
                !dampingBoundary);
        }

        if (negativeEnvelope)
        {
            negativeEnvelope.RefreshEqualizerBoundaryCollisionOwnership();
        }
    }


    private bool IsEqualizerDampingBoundaryCollider(
        Collider other)
    {
        return
            IsUpperEnvelopeCollider(other) ||
            IsStairwayCollider(other);
    }


    private bool IsUpperEnvelopeCollider(
        Collider other)
    {
        return
            other &&
            negativeEnvelope &&
            negativeEnvelope.IsUpperEnvelopeCollider(other);
    }


    private bool IsStairwayCollider(
        Collider other)
    {
        if (!other)
            return false;

        Transform current =
            other.transform;

        // Keep the same naming contract already used by IsStairLikeCollision.
        // Generated physics StairWay objects and their parents contain "Stair".
        for (int depth = 0;
             current && depth < 8;
             depth++)
        {
            if (current.name.Contains("Stair"))
                return true;

            current =
                current.parent;
        }

        return false;
    }
    // ================================================================
    // Re-synchronization
    // ================================================================

    private void ReacquireForNextIncident()
    {
        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        float distanceBeforeReset =
            Vector3.Distance(
                ballVisualEqualizer.position,
                ballVisual.position);

        float velocityErrorBeforeReset =
            Vector3.Distance(
                ballVisualEqualizer.velocity,
                ballVisual.velocity);

        FinalizeReleaseEvaluation(
            "NextIncidentRequested");

        if (negativeEnvelope)
            negativeEnvelope.ClearEnvelope();

        TransitionTo(
            EqualizerPhase.Reacquiring,
            "NextIncidentRequested");

        // 現在のHopper Physicsを停止する。
        lowerContactCount =
            0;

        upperContactCount =
            0;

        ResetImpactMapState();
        ResetOscillationFrame();
        ResetReleaseFeasibility();

        ballVisualEqualizer.useGravity =
            false;

        ballVisualEqualizer.detectCollisions =
            false;

        // Dynamicのうちに残留速度を消す。
        if (!ballVisualEqualizer.isKinematic)
        {
            ballVisualEqualizer.velocity =
                Vector3.zero;

            ballVisualEqualizer.angularVelocity =
                Vector3.zero;
        }

        ballVisualEqualizer.isKinematic =
            true;

        // Reset Map:
        //
        //     x_E^+ = x_B
        //     q_E^+ = q_B
        //
        // 次のRelease初速は、この直後に新しいIncident解から
        // 1回だけDynamic Rigidbodyへ書き込まれる。
        CopyBallVisualPose();

        Physics.SyncTransforms();

        synchronized =
            true;

        UpdateObserver();

        TransitionTo(
            EqualizerPhase.Synchronized,
            "NextIncidentReacquired");

        Debug.Log(
            $"[EQUALIZER NEXT INCIDENT REACQUIRE] " +
            $"distanceBefore={distanceBeforeReset:F4}m " +
            $"velocityErrorBefore={velocityErrorBeforeReset:F4}m/s " +
            $"positionAfter={ballVisualEqualizer.position:F4} " +
            $"ballVisualPosition={ballVisual.position:F4} " +
            $"positionErrorAfter={PositionErrorToBallVisual:F6}m",
            this);
    }


    public void ResumeSynchronization()
    {
        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        FinalizeReleaseEvaluation(
            "ResumeRequested");

        if (negativeEnvelope)
            negativeEnvelope.ClearEnvelope();

        TransitionTo(
            EqualizerPhase.Reacquiring,
            "ResumeRequested");


        // ------------------------------------------------------------
        // Reset map:
        //
        // Dynamic Hopper -> Kinematic synchronized body
        // ------------------------------------------------------------

        lowerContactCount =
            0;

        upperContactCount =
            0;

        ResetImpactMapState();
        ResetOscillationFrame();
        ResetReleaseFeasibility();

        ballVisualEqualizer.useGravity =
            false;

        ballVisualEqualizer.detectCollisions =
            false;

        ballVisualEqualizer.velocity =
            Vector3.zero;

        ballVisualEqualizer.angularVelocity =
            Vector3.zero;

        ballVisualEqualizer.isKinematic =
            true;

        CopyBallVisualPose();

        synchronized =
            true;

        TransitionTo(
            EqualizerPhase.Synchronized,
            "ReacquiredBallVisual");
    }


    // Existing external calls remain valid.
    public void ResumeSynchronization1()
    {
        ResumeSynchronization();
    }


    public void ResumeSynchronization2()
    {
        ResumeSynchronization();
    }


    // ================================================================
    // Collision classification
    // ================================================================

    private bool IsUpperEnvelopeCollision(
        Collision collision)
    {
        return
            collision != null &&
            IsUpperEnvelopeCollider(
                collision.collider);
    }


    private bool IsStairwayCollision(
        Collision collision)
    {
        return
            collision != null &&
            IsStairwayCollider(
                collision.collider);
    }


    // ================================================================
    // Collision observer
    // ================================================================

    private void OnCollisionEnter(
        Collision collision)
    {
        if (synchronized ||
            !ballVisualEqualizer ||
            collision == null ||
            collision.contactCount <= 0)
        {
            return;
        }

        bool envelopeContact =
            IsUpperEnvelopeCollision(
                collision);

        bool stairwayContact =
            IsStairwayCollision(
                collision);

        // Only Upper Envelope or a real Stairway contact belongs to this channel.
        if (!envelopeContact &&
            !stairwayContact)
        {
            return;
        }

        if (envelopeContact)
        {
            upperContactCount++;
        }
        else
        {
            lowerContactCount++;
        }


        ContactSelectionResult selection =
            SelectDominantImpactContact(
                collision,
                envelopeContact);

        lastContactFrame =
            BuildContactFrame(
                collision,
                selection.contact,
                envelopeContact);

        lastContactFrame.contactAuthority =
            selection.authority01;

        lastContactFrame.contactApproach01 =
            selection.approach01;

        lastContactFrame.contactForwardCompatibility01 =
            selection.forwardCompatibility01;

        lastContactFrame.contactPredictedSeparationSpeed =
            selection.predictedSeparationSpeed;

        // Lower is an actual Stairway collision, not a synthetic guide overlap.
        lastContactFrame.canonicalContact =
            selection.canonicalContact;

        ApplyImpactMap(
            ref lastContactFrame);


        TransitionTo(
            envelopeContact
                ? EqualizerPhase.UpperContact
                : EqualizerPhase.LowerContact,
            envelopeContact
                ? "EnvelopeCollisionEnter"
                : "StairwayCollisionEnter");


        Debug.Log(
            $"[EQUALIZER CONTACT FRAME] " +
            $"kind={(envelopeContact ? "UpperEnvelope" : "Stairway")} " +
            $"other={collision.collider.name} " +
            $"stairLike={lastContactFrame.stairLikeContact} " +
            $"position={ballVisualEqualizer.position:F4} " +
            $"point={lastContactFrame.point:F4} " +
            $"rawNormal={lastContactFrame.rawNormal:F4} " +
            $"normal={lastContactFrame.normal:F4} " +
            $"normalFlipped={lastContactFrame.normalWasFlipped} " +
            $"incidentVelocity={lastContactFrame.incidentVelocity:F4} " +
            $"v0={lastContactFrame.speed:F4}m/s " +
            $"v0cos={lastContactFrame.tangentSpeed:F4}m/s " +
            $"v0sin={lastContactFrame.inwardNormalSpeed:F4}m/s " +
            $"angle={lastContactFrame.incidenceAngleDeg:F3}deg " +
            $"Et={lastContactFrame.tangentEnergy:F4}J " +
            $"En={lastContactFrame.normalEnergy:F4}J " +
            $"EnRatio={lastContactFrame.normalEnergyRatio:F4} " +
            $"oscFrame={lastContactFrame.oscillationFrameValid} " +
            $"oscNormal={lastContactFrame.oscillationNormal:F4} " +
            $"oscAxisOut={lastContactFrame.oscillationOutgoingAxis:F4} " +
            $"oscIn={lastContactFrame.oscillationIncomingSpeed:F4}m/s " +
            $"oscEin={lastContactFrame.oscillationIncomingEnergy:F4}J " +
            $"oscEnRatio={lastContactFrame.oscillationIncomingEnergyRatio:F4} " +
            $"contactOscAlignment={lastContactFrame.contactOscillationAlignment:F4} " +
            $"authority={lastContactFrame.contactAuthority:F4} " +
            $"authorityApproach={lastContactFrame.contactApproach01:F4} " +
            $"authorityForward={lastContactFrame.contactForwardCompatibility01:F4} " +
            $"authorityPredSep={lastContactFrame.contactPredictedSeparationSpeed:F4}m/s " +
            $"canonicalContact={lastContactFrame.canonicalContact} " +
            $"emergency={lastContactFrame.emergencyContactGuard} " +
            $"constraintCorrection={lastContactFrame.contactConstraintCorrectionSpeed:F4}m/s " +
            $"wallness={lastContactFrame.contactWallness:F4} " +
            $"preVT={lastContactFrame.preTransportSpeed:F4}m/s " +
            $"postVT={lastContactFrame.postTransportSpeed:F4}m/s " +
            $"transportRetention={lastContactFrame.transportRetention:F4} " +
            $"vOscRequested={lastContactFrame.requestedOutgoingOscillationSpeed:F4}m/s " +
            $"vOscFinal={lastContactFrame.finalOutgoingOscillationSpeed:F4}m/s " +
            $"separation={lastContactFrame.finalSeparationSpeed:F4}m/s " +
            $"forwardGuard={lastContactFrame.forwardGuardApplied} " +
            $"oscReduced={lastContactFrame.oscillationReducedByConstraint} " +
            $"severeTransportLoss={lastContactFrame.severeTransportLoss} " +
            $"physicsClean={lastContactFrame.physicsClean} " +
            $"dampingSuccess={lastContactFrame.dampingSuccess} " +
            $"transportSuccess={lastContactFrame.transportSuccess} " +
            $"subjectConverging={lastContactFrame.subjectConverging} " +
            $"gameImpactSuccess={lastContactFrame.gameImpactSuccess} " +
            $"solverOutNormal={lastContactFrame.outwardNormalSpeedAfterSolver:F4}m/s " +
            $"solverEEstimate={lastContactFrame.estimatedRestitution:F4} " +
            $"mapApplied={lastContactFrame.impactMapApplied} " +
            $"mapVnOut={lastContactFrame.mappedOutgoingNormalSpeed:F4}m/s " +
            $"mapEnOut={lastContactFrame.mappedOutgoingNormalEnergy:F4}J " +
            $"mapImpactRatio={lastContactFrame.mappedNormalEnergyRatio:F4} " +
            $"mapReservoirRatio={lastContactFrame.reservoirEnergyRatio:F4} " +
            $"mapEEffective={lastContactFrame.mappedEffectiveRestitution:F4} " +
            $"normalImpulse={lastContactFrame.normalImpulse:F4} " +
            $"lowerBoundary=RealStairway " +
            $"phaseDrive={forcedBoundaryDriveActive} " +
            $"T={forcedBoundaryPeriod:F4}s " +
            $"halfT={forcedBoundaryHalfPeriod:F4}s " +
            $"heightScale={forcedBoundaryRadiusClearanceScale:F4}R " +
            $"phaseAN={forcedBoundaryAppliedPhaseAcceleration:F4}m/s2 " +
            $"phaseARef={forcedBoundaryPlannedReleasePhaseAcceleration:F4}m/s2",
            this);
    }


    private void OnCollisionStay(
        Collision collision)
    {
        if (synchronized ||
            !ballVisualEqualizer ||
            collision == null ||
            !IsEqualizerDampingBoundaryCollider(
                collision.collider))
        {
            return;
        }

        Debug.Log(
            $"[EQUALIZER COLLISION STAY] " +
            $"other={collision.collider.name} " +
            $"phase={phase} " +
            $"position={ballVisualEqualizer.position:F4} " +
            $"velocity={ballVisualEqualizer.velocity:F4}",
            this);
    }


    private void OnCollisionExit(
        Collision collision)
    {
        if (synchronized ||
            collision == null ||
            !IsEqualizerDampingBoundaryCollider(
                collision.collider))
        {
            return;
        }

        bool envelopeContact =
            IsUpperEnvelopeCollision(
                collision);

        if (envelopeContact)
        {
            upperContactCount =
                Mathf.Max(
                    0,
                    upperContactCount - 1);
        }
        else
        {
            lowerContactCount =
                Mathf.Max(
                    0,
                    lowerContactCount - 1);
        }

        if (upperContactCount == 0 &&
            lowerContactCount == 0)
        {
            // Upper Envelope is immutable for this Release.
            // CollisionExit only advances state; no Mesh recook is requested.
            TransitionTo(
                EqualizerPhase.HopperFlight,
                "CollisionExit");
        }
    }


    // ================================================================
    // Physics material diagnostics
    // ================================================================

    private void LogPhysicMaterialState(
        string phaseName)
    {
        Debug.Log(
            $"[EQUALIZER PHYSICS MATERIAL] " +
            $"phase={phaseName} " +
            $"{GetPhysicMaterialState(ballVisualEqualizerCollider)}",
            this);
    }


    private string GetPhysicMaterialState(
        Collider collider)
    {
        if (!collider)
            return "Collider=NULL";
        
        PhysicMaterial material =
            collider.sharedMaterial;

        if (!material)
        {
            return
                $"Collider={collider.name} " +
                $"Material=None";
        }

        return
            $"Collider={collider.name} " +
            $"Material={material.name} " +
            $"StaticFriction={material.staticFriction:F3} " +
            $"DynamicFriction={material.dynamicFriction:F3} " +
            $"Bounciness={material.bounciness:F3} " +
            $"FrictionCombine={material.frictionCombine} " +
            $"BounceCombine={material.bounceCombine}";
    }
}

