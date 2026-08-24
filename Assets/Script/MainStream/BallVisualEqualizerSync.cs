using UnityEngine;

public sealed class BallVisualEqualizerSync : MonoBehaviour
{
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
    // Equalizer solves contacts only against the generated LowerGuide and
    // NegativeEnvelope.  Real stage / InSubject / BallVisual contacts are excluded.
    // No Inspector tuning parameter is introduced.

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

    [Header("Periodic Boundary Drive Runtime - Read Only")]
    [SerializeField]
    private bool forcedBoundaryDriveActive;

    [SerializeField]
    private float forcedBoundaryReleaseTime;

    [SerializeField]
    private float forcedBoundaryTimeCost;

    [SerializeField]
    private int forcedBoundaryCycleCount;

    [SerializeField]
    private float forcedBoundaryPeriod;

    [SerializeField]
    private float forcedBoundaryHalfPeriod;

    [SerializeField]
    private int forcedBoundaryHalfCycleIndex;

    [SerializeField]
    private bool forcedBoundaryNextUpper = true;

    [SerializeField]
    private float forcedBoundaryNextTime;

    [SerializeField]
    private float forcedBoundaryPhaseErrorSeconds;

    [SerializeField]
    private float forcedBoundaryTimeToBoundary;

    [SerializeField]
    private float forcedBoundaryDistance;

    [SerializeField]
    private float forcedBoundaryRequiredTotalNormalAcceleration;

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

    [SerializeField]
    private float forcedBoundaryAutomaticAccelerationLimit;

    [SerializeField]
    private bool lowerBoundaryReflectionArmed;

    [SerializeField]
    private int forcedBoundaryRephaseCount;

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


    // ================================================================
    // Unity
    // ================================================================

    private void Start()
    {
        Debug.Log("######## BallVisualEqualizerSync START ########");
        ResolveReferences();
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

        // Once the Release has survived one physics step without a Lower
        // overlap, arm Lower as a genuine future damping boundary.  If an
        // initial overlap existed, OnCollisionExit performs the same arm.
        UpdateLowerBoundaryReflectionArm();

        // E0 / H0 defines one Stable-N acceleration for this Release.
        // Gravity is compensated only on N so the total N acceleration is -aN;
        // tangent/lateral gravity remains untouched.
        ApplyCanonicalNormalAcceleration();

        ApplySubjectTransportConvergence();
        UpdateObserver();
        UpdateDynamicPhaseFromContacts();
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

        // LowerGuide / UpperEnvelope are now the Equalizer's exclusive physical
        // channel.  The guide was created by ArmFromBallVisualEnergy(), so refresh
        // pairwise collision ownership again before switching the body Dynamic.
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

        // Release starts on/very near the smooth LowerGuide.  That first
        // overlap is a departure contact, not a Poincare impact.  Arm the
        // Lower reflection only after the body has actually exited the guide.
        lowerBoundaryReflectionArmed =
            false;

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

        // Period T is the master. Geometry resolves S(T)*R + A(t), then
        // Equalizer measures the ACTUAL Upper/Lower distance with SphereCast
        // and solves only the Stable-N acceleration needed to hit the next
        // boundary exactly T/2 after the previous real impact.
        InitializeForcedBoundaryDrive();

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
            //$"halfT={forcedBoundaryHalfPeriod:F4}s",
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
        forcedBoundaryCycleCount = 0;
        forcedBoundaryPeriod = 0f;
        forcedBoundaryHalfPeriod = 0f;
        forcedBoundaryHalfCycleIndex = 0;
        forcedBoundaryNextUpper = true;
        forcedBoundaryNextTime = 0f;
        forcedBoundaryPhaseErrorSeconds = 0f;
        forcedBoundaryTimeToBoundary = 0f;
        forcedBoundaryDistance = 0f;
        forcedBoundaryRequiredTotalNormalAcceleration = 0f;
        forcedBoundaryAppliedPhaseAcceleration = 0f;
        forcedBoundaryGamma = 0f;
        forcedBoundaryRadiusClearanceScale = 0f;
        forcedBoundaryPlannedReleaseSpan = 0f;
        forcedBoundaryPlannedReleaseTargetNormalSpeed = 0f;
        forcedBoundaryPlannedReleasePhaseAcceleration = 0f;
        forcedBoundaryAutomaticAccelerationLimit = 0f;
        forcedBoundaryRephaseCount = 0;
    }


    private void InitializeForcedBoundaryDrive()
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

        float fixedDt =
            Mathf.Max(
                0.0001f,
                Time.fixedDeltaTime);

        float minimumHalfPeriod =
            fixedDt *
            MinimumForcedBoundaryHalfCycleFixedSteps;

        forcedBoundaryDriveActive =
            halfPeriod >= minimumHalfPeriod;

        if (!forcedBoundaryDriveActive)
            return;

        forcedBoundaryReleaseTime =
            Time.fixedTime;

        // Tcost/gamma remain diagnostics for the exponential envelope only.
        // They no longer terminate or reschedule the periodic boundary drive.
        forcedBoundaryTimeCost =
            timeCost;

        forcedBoundaryCycleCount =
            0;

        forcedBoundaryPeriod =
            period;

        forcedBoundaryHalfPeriod =
            halfPeriod;

        forcedBoundaryHalfCycleIndex =
            0;

        forcedBoundaryNextUpper =
            true;

        forcedBoundaryNextTime =
            forcedBoundaryReleaseTime +
            forcedBoundaryHalfPeriod;

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

        forcedBoundaryAutomaticAccelerationLimit =
            2f *
            Mathf.Max(
                Mathf.Abs(releasePhaseAcceleration),
                Mathf.Max(
                    Mathf.Abs(releaseTargetNormalSpeed) /
                    Mathf.Max(0.0001f, halfPeriod),
                    canonicalNormalAcceleration));

        Debug.Log(
            $"[EQUALIZER PERIODIC DRIVE] " +
            //$"T={forcedBoundaryPeriod:F4}s " +
            //$"halfT={forcedBoundaryHalfPeriod:F4}s " +
            $"clearanceScale={forcedBoundaryRadiusClearanceScale:F4}R " +
            $"releaseSpan={forcedBoundaryPlannedReleaseSpan:F4}m " +
            $"targetVN={forcedBoundaryPlannedReleaseTargetNormalSpeed:F4}m/s " +
            $"phaseA0={forcedBoundaryPlannedReleasePhaseAcceleration:F4}m/s2 " +
            $"phaseALimit={forcedBoundaryAutomaticAccelerationLimit:F4}m/s2 " +
            $"Tcost={forcedBoundaryTimeCost:F4}s " +
            $"gamma={forcedBoundaryGamma:F4}/s " +
            $"next=Upper",
            this);
    }


    private void RefreshPeriodicBoundaryPlanIfChanged()
    {
        if (!forcedBoundaryDriveActive ||
            !negativeEnvelope)
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

        bool timingChanged =
            Mathf.Abs(period - forcedBoundaryPeriod) > 0.00001f ||
            Mathf.Abs(halfPeriod - forcedBoundaryHalfPeriod) > 0.00001f;

        forcedBoundaryTimeCost =
            timeCost;

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

        forcedBoundaryAutomaticAccelerationLimit =
            2f *
            Mathf.Max(
                Mathf.Abs(releasePhaseAcceleration),
                Mathf.Max(
                    Mathf.Abs(releaseTargetNormalSpeed) /
                    Mathf.Max(0.0001f, halfPeriod),
                    canonicalNormalAcceleration));

        if (!timingChanged)
            return;

        forcedBoundaryPeriod =
            period;

        forcedBoundaryHalfPeriod =
            halfPeriod;

        forcedBoundaryNextTime =
            Time.fixedTime +
            forcedBoundaryHalfPeriod;

        forcedBoundaryRephaseCount++;

        Debug.Log(
            $"[EQUALIZER PERIODIC PLAN UPDATED] " +
            //$"T={forcedBoundaryPeriod:F4}s " +
            //$"halfT={forcedBoundaryHalfPeriod:F4}s " +
            $"heightScale={forcedBoundaryRadiusClearanceScale:F4}R " +
            $"phaseALimit={forcedBoundaryAutomaticAccelerationLimit:F4}m/s2",
            this);
    }


    private void ObserveForcedBoundaryImpact(
        bool upperBoundary)
    {
        if (!forcedBoundaryDriveActive)
            return;

        float now =
            Time.fixedTime;

        forcedBoundaryPhaseErrorSeconds =
            now -
            forcedBoundaryNextTime;

        bool expectedBoundary =
            upperBoundary ==
            forcedBoundaryNextUpper;

        if (expectedBoundary)
        {
            forcedBoundaryHalfCycleIndex++;

            forcedBoundaryCycleCount =
                forcedBoundaryHalfCycleIndex / 2;

            forcedBoundaryNextUpper =
                !forcedBoundaryNextUpper;
        }
        else
        {
            // Unexpected real boundary: use the real impact as the new phase
            // origin. The next leg still receives exactly T/2.
            forcedBoundaryNextUpper =
                !upperBoundary;
        }

        // Every leg starts from the ACTUAL boundary impact.  There is no
        // absolute Release clock, so a late leg can never compress the next one.
        forcedBoundaryNextTime =
            now +
            forcedBoundaryHalfPeriod;

        Debug.Log(
            $"[EQUALIZER PERIODIC IMPACT] " +
            $"kind={(upperBoundary ? "Upper" : "Lower")} " +
            $"expected={expectedBoundary} " +
            $"phaseError={forcedBoundaryPhaseErrorSeconds:F5}s " +
            $"halfIndex={forcedBoundaryHalfCycleIndex} " +
            $"cycles={forcedBoundaryCycleCount} " +
           // $"T={forcedBoundaryPeriod:F4}s " +
            $"next={(forcedBoundaryNextUpper ? "Upper" : "Lower")}",
            this);
    }


    private float ResolveEqualizerWorldRadiusForBoundaryDrive()
    {
        if (!ballVisualEqualizerCollider)
            return 0f;

        Vector3 scale =
            ballVisualEqualizerCollider.transform.lossyScale;

        float maximumScale =
            Mathf.Max(
                Mathf.Abs(scale.x),
                Mathf.Max(
                    Mathf.Abs(scale.y),
                    Mathf.Abs(scale.z)));

        return
            Mathf.Max(
                0f,
                ballVisualEqualizerCollider.radius *
                maximumScale);
    }


    private bool TryMeasureForcedBoundaryDistance(
        bool upperBoundary,
        Vector3 direction,
        out float distance)
    {
        distance = 0f;

        if (!ballVisualEqualizer ||
            !ballVisualEqualizerCollider ||
            direction.sqrMagnitude <= ImpactEnergyEpsilon)
        {
            return false;
        }

        direction.Normalize();

        float radius =
            ResolveEqualizerWorldRadiusForBoundaryDrive();

        Vector3 origin =
            ballVisualEqualizerCollider.transform.TransformPoint(
                ballVisualEqualizerCollider.center);

        float maximumDistance =
            Mathf.Max(
                2f,
                canonicalReferenceHeight * 4f +
                radius * 12f +
                1f);

        float castRadius =
            Mathf.Max(
                0.0001f,
                radius * 0.985f);

        RaycastHit[] sphereHits =
            Physics.SphereCastAll(
                origin,
                castRadius,
                direction,
                maximumDistance,
                Physics.AllLayers,
                QueryTriggerInteraction.Ignore);

        float bestDistance =
            float.PositiveInfinity;

        for (int i = 0;
             i < sphereHits.Length;
             i++)
        {
            Collider collider =
                sphereHits[i].collider;

            if (!collider ||
                collider == ballVisualEqualizerCollider)
            {
                continue;
            }

            bool wanted =
                upperBoundary
                    ? IsUpperEnvelopeCollider(collider)
                    : IsLowerGuideCollider(collider);

            if (!wanted)
                continue;

            float candidate =
                Mathf.Max(
                    0f,
                    sphereHits[i].distance);

            if (candidate < bestDistance)
                bestDistance = candidate;
        }

        if (!float.IsInfinity(bestDistance))
        {
            distance = bestDistance;
            return true;
        }

        // 開始時オーバーラップでSphereCastが空になる場合のfallback。
        RaycastHit[] rayHits =
            Physics.RaycastAll(
                origin,
                direction,
                maximumDistance + radius,
                Physics.AllLayers,
                QueryTriggerInteraction.Ignore);

        for (int i = 0;
             i < rayHits.Length;
             i++)
        {
            Collider collider =
                rayHits[i].collider;

            if (!collider ||
                collider == ballVisualEqualizerCollider)
            {
                continue;
            }

            bool wanted =
                upperBoundary
                    ? IsUpperEnvelopeCollider(collider)
                    : IsLowerGuideCollider(collider);

            if (!wanted)
                continue;

            float candidate =
                Mathf.Max(
                    0f,
                    rayHits[i].distance - radius);

            if (candidate < bestDistance)
                bestDistance = candidate;
        }

        if (float.IsInfinity(bestDistance))
            return false;

        distance = bestDistance;
        return true;
    }


    private bool TryResolveForcedBoundaryTotalNormalAcceleration(
        Vector3 normal,
        out float targetTotalNormalAcceleration)
    {
        targetTotalNormalAcceleration =
            -canonicalNormalAcceleration;

        forcedBoundaryDistance = 0f;
        forcedBoundaryRequiredTotalNormalAcceleration =
            targetTotalNormalAcceleration;
        forcedBoundaryAppliedPhaseAcceleration = 0f;

        if (!forcedBoundaryDriveActive ||
            !ballVisualEqualizer ||
            forcedBoundaryHalfPeriod <= 0f)
        {
            return false;
        }

        RefreshPeriodicBoundaryPlanIfChanged();

        float fixedDt =
            Mathf.Max(
                0.0001f,
                Time.fixedDeltaTime);

        // There is no absolute Release deadline.  The drive remains active
        // for the current Equalizer release and every real impact becomes the
        // phase origin of the next T/2 leg.

        float directionSign =
            forcedBoundaryNextUpper
                ? 1f
                : -1f;

        Vector3 direction =
            normal *
            directionSign;

        if (!TryMeasureForcedBoundaryDistance(
                forcedBoundaryNextUpper,
                direction,
                out float distance))
        {
            return false;
        }

        forcedBoundaryDistance =
            distance;

        float now =
            Time.fixedTime;

        float remainingTime =
            forcedBoundaryNextTime -
            now;

        // Only a truly missed deadline is rephased.  Positive Tgo is kept so
        // the requested period T is not silently stretched every FixedUpdate.
        // No distance/fixedDt velocity replacement exists.
        if (remainingTime <= 0f)
        {
            forcedBoundaryPhaseErrorSeconds =
                now -
                forcedBoundaryNextTime;

            forcedBoundaryNextTime =
                now +
                forcedBoundaryHalfPeriod;

            remainingTime =
                forcedBoundaryHalfPeriod;

            forcedBoundaryRephaseCount++;

            Debug.Log(
                $"[EQUALIZER PERIODIC REPHASE] " +
                $"next={(forcedBoundaryNextUpper ? "Upper" : "Lower")} " +
                $"phaseError={forcedBoundaryPhaseErrorSeconds:F5}s " +
                $"count={forcedBoundaryRephaseCount}",
                this);
        }

        forcedBoundaryTimeToBoundary =
            remainingTime;

        float currentNormalVelocity =
            Vector3.Dot(
                ballVisualEqualizer.velocity,
                normal);

        float currentTowardSpeed =
            currentNormalVelocity *
            directionSign;

        float safeRemainingTime =
            Mathf.Max(
                fixedDt,
                remainingTime);

        // 実ColliderまでのStable-N距離 s を指定half-period tau に合わせる。
        //
        //     s = v*tau + 1/2*a*tau^2
        //     a = 2(s - v*tau) / tau^2
        //
        // Geometry側も同じTからS(T)を決めているため、通常は大きな
        // 修正を必要としない。衝突遅延やSolver外乱だけをここで吸収する。
        float requiredTowardAcceleration =
            2f *
            (distance -
             currentTowardSpeed *
             safeRemainingTime) /
            (safeRemainingTime *
             safeRemainingTime);

        targetTotalNormalAcceleration =
            requiredTowardAcceleration *
            directionSign;

        float phaseAcceleration =
            targetTotalNormalAcceleration -
            (-canonicalNormalAcceleration);

        // Period planから自動算出した加速度尺度を上限にする。
        // 以前の2500m/s^2級のdeadline recoveryはここで発生しない。
        float automaticLimit =
            Mathf.Max(
                canonicalNormalAcceleration,
                forcedBoundaryAutomaticAccelerationLimit);

        if (automaticLimit > 0f &&
            Mathf.Abs(phaseAcceleration) > automaticLimit)
        {
            phaseAcceleration =
                Mathf.Clamp(
                    phaseAcceleration,
                    -automaticLimit,
                    automaticLimit);

            targetTotalNormalAcceleration =
                -canonicalNormalAcceleration +
                phaseAcceleration;
        }

        forcedBoundaryRequiredTotalNormalAcceleration =
            targetTotalNormalAcceleration;

        forcedBoundaryAppliedPhaseAcceleration =
            phaseAcceleration;

        return true;
    }


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
            }
        }

        if (envelopeContact)
            lastUpperImpactTime = now;
        else
            lastLowerImpactTime = now;
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


    private void UpdateLowerBoundaryReflectionArm()
    {
        if (synchronized ||
            lowerBoundaryReflectionArmed ||
            lowerContactCount > 0)
        {
            return;
        }

        float fixedDt =
            Mathf.Max(
                0.0001f,
                Time.fixedDeltaTime);

        // Release occurs during FixedUpdate and contact callbacks are delivered
        // by the following physics solve.  Waiting one FixedUpdate therefore
        // distinguishes "no initial Lower overlap" from a departure overlap.
        if (phaseElapsed <
            fixedDt * 0.75f)
        {
            return;
        }

        lowerBoundaryReflectionArmed =
            true;

        Debug.Log(
            "[EQUALIZER LOWER ARMED] No departure overlap remained after Release; future Lower contacts are real damping impacts.",
            this);
    }


    private void ApplyCanonicalNormalAcceleration()
    {
        bool realLowerImpactContact =
            lowerBoundaryReflectionArmed &&
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

        TryResolveForcedBoundaryTotalNormalAcceleration(
            normal,
            out targetTotalNormalAcceleration);

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
        if (!collision.collider)
            return false;

        Transform current =
            collision.collider.transform;

        // Development diagnostic only.
        // It never selects another Equalizer physics model.
        for (int depth = 0;
             current && depth < 5;
             depth++)
        {
            if (current.name.Contains("Stair"))
                return true;

            current =
                current.parent;
        }

        return false;
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

        // 周期位相を進めるのはStable-N正面のCanonical impactだけ。
        // Edge/裏面/Release直後Lower overlapでは次のT/2を消費しない。
        if (canonicalApplied)
        {
            ObserveForcedBoundaryImpact(
                frame.envelopeContact);
        }

        if (canonicalApplied)
        {
            canonicalImpactCount++;

            UpdateObservedCyclePeriod(
                frame.envelopeContact);

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
            $"kind={(frame.envelopeContact ? "UpperEnvelope" : "LowerBoundary")} " +
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
            // Equalizer collides only with LowerGuide + UpperEnvelope.
            // BallVisual / real StairWay / Render / Physics stages never solve
            // impulses on the Equalizer.
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
            other &&
            negativeEnvelope &&
            negativeEnvelope.IsEqualizerBoundaryCollider(other);
    }


    private bool IsUpperEnvelopeCollider(
        Collider other)
    {
        return
            other &&
            negativeEnvelope &&
            negativeEnvelope.IsUpperEnvelopeCollider(other);
    }


    private bool IsLowerGuideCollider(
        Collider other)
    {
        return
            other &&
            negativeEnvelope &&
            negativeEnvelope.IsLowerGuideCollider(other);
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

        lowerBoundaryReflectionArmed =
            false;

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

        lowerBoundaryReflectionArmed =
            false;

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


    private bool IsLowerGuideCollision(
        Collision collision)
    {
        return
            collision != null &&
            IsLowerGuideCollider(
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

        bool lowerGuideContact =
            IsLowerGuideCollision(
                collision);

        // Ignore callbacks from every non-channel collider. Pairwise ignores are
        // configured before Dynamic release, so those colliders should not solve an
        // impulse either; this is the defensive classification layer.
        if (!envelopeContact &&
            !lowerGuideContact)
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

        bool departureLowerContact =
            lowerGuideContact &&
            !lowerBoundaryReflectionArmed;

        lastContactFrame.canonicalContact =
            departureLowerContact
                ? false
                : selection.canonicalContact;

        if (!departureLowerContact)
        {
            ApplyImpactMap(
                ref lastContactFrame);
        }


        TransitionTo(
            envelopeContact
                ? EqualizerPhase.UpperContact
                : EqualizerPhase.LowerContact,
            envelopeContact
                ? "EnvelopeCollisionEnter"
                : "LowerCollisionEnter");


        Debug.Log(
            $"[EQUALIZER CONTACT FRAME] " +
            $"kind={(envelopeContact ? "UpperEnvelope" : "LowerBoundary")} " +
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
            $"departureLowerIgnored={departureLowerContact} " +
            $"lowerArmed={lowerBoundaryReflectionArmed} " +
            $"periodicDrive={forcedBoundaryDriveActive} " +
           // $"T={forcedBoundaryPeriod:F4}s " +
           // $"halfT={forcedBoundaryHalfPeriod:F4}s " +
            $"heightScale={forcedBoundaryRadiusClearanceScale:F4}R " +
            $"phaseAN={forcedBoundaryAppliedPhaseAcceleration:F4}m/s2 " +
            $"phaseALimit={forcedBoundaryAutomaticAccelerationLimit:F4}m/s2 " +
            //$"phaseTgo={forcedBoundaryTimeToBoundary:F5}s " +
            $"boundaryDistance={forcedBoundaryDistance:F4}m",
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

            if (!lowerBoundaryReflectionArmed &&
                lowerContactCount == 0)
            {
                lowerBoundaryReflectionArmed =
                    true;

                Debug.Log(
                    "[EQUALIZER LOWER ARMED] Release departure cleared; subsequent Lower contacts are real damping impacts.",
                    this);
            }
        }

        if (upperContactCount == 0 &&
            lowerContactCount == 0)
        {
            if (negativeEnvelope)
                negativeEnvelope.CommitDeferredCanonicalGeometryUpdate();

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
