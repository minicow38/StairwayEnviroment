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
        public Vector3 velocity;
        public Vector3 angularVelocity;

        public float speed;
        public float kineticEnergy;
        public float sourceEnergy;
        public float envelopeEntryHeight;
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
        // where e_eff is additionally capped by the same-boundary
        // Poincare energy contraction.
        public bool impactMapApplied;
        public Vector3 mappedOutgoingVelocity;
        public float mappedOutgoingNormalSpeed;
        public float mappedOutgoingNormalEnergy;
        public float mappedNormalEnergyRatio;
        public float mappedEffectiveRestitution;
        public float previousSameBoundaryNormalEnergy;
        public float sameBoundaryEnergyCeiling;
        public float sameBoundaryEnergyRatio;

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
        "BallVisualEqualizerが減衰運動中に進行方向Tで収束する対象。"
        + "状態選択ではなく、単一モデルのTransport基準です。")]
    [SerializeField]
    private Transform subjectTransform;

    [Tooltip(
        "SubjectにRigidbodyがある場合の速度基準。"
        + "未設定でもSubject TransformとBallVisual速度から追従を継続します。")]
    [SerializeField]
    private Rigidbody subjectBody;

    [SerializeField]
    private Rigidbody ballVisualEqualizer;

    [SerializeField]
    private SphereCollider ballVisualEqualizerCollider;

    [SerializeField]
    private BallVisualNegativeEnvelopeCollider negativeEnvelope;


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
    // In addition, the return to the SAME boundary is constrained by
    // a Poincare-style energy contraction:
    //
    //   E_(k+1, same boundary)^+
    //       <= rho * E_(k, same boundary)^+
    //
    // This prevents gravity / geometry from making the bounce envelope
    // grow again from cycle to cycle.
    // ================================================================

    [Header("Impact Map + Energy Contraction")]

    [Tooltip(
        "各Oscillation impactで基準法線速度へ掛ける反発率 e。"
        + "局所Collider角度ではなくRelease固定のOscillation Frameで収縮させます。")]
    [Range(0f, 0.999f)]
    [SerializeField]
    private float impactNormalRestitution =
        0.92f;

    [Tooltip(
        "同じ境界へ戻った時のOscillation Energy上限 rho。"
        + "Lower→Lower / Upper→Upper を同じ基準軸で比較し収縮させます。")]
    [Range(0f, 0.999f)]
    [SerializeField]
    private float sameBoundaryEnergyContraction =
        0.82f;


    // ================================================================
    // Subject convergence - single transport model
    // ================================================================
    //
    //   v = v_T T + v_L L + v_N N
    //
    // N is owned by damping.
    // T is owned by forward transport / Subject convergence.
    // No position teleport and no oscillation-normal catch-up is used.
    // ================================================================

    [Header("Subject Convergence")]

    [Tooltip(
        "SubjectとのT方向距離誤差を速度差へ変換する時定数[s]。"
        + "小さいほど早く詰めますが、最大加速度で必ず制限されます。")]
    [Min(0.05f)]
    [SerializeField]
    private float subjectConvergenceTime =
        0.45f;

    [Tooltip(
        "Subject速度へ上乗せできる最大Closing Speed[m/s]。"
        + "T方向だけに作用し、N方向の減衰は変更しません。")]
    [Min(0f)]
    [SerializeField]
    private float maximumTransportSpeedBoost =
        12f;

    [Tooltip(
        "EqualizerのT速度をSubjectへ寄せる最大加速度[m/s^2]。")]
    [Min(0.01f)]
    [SerializeField]
    private float maximumTransportAcceleration =
        45f;

    [Tooltip(
        "このT方向距離内では位置誤差による速度上乗せを止め、"
        + "SubjectのT速度への一致だけを行います[m]。")]
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
    private float lastLowerMappedNormalEnergy =
        -1f;

    [SerializeField]
    private float lastUpperMappedNormalEnergy =
        -1f;

    [SerializeField]
    private float lastMappedNormalEnergy;

    [SerializeField]
    private float lastImpactNormalEnergyRatio;

    [SerializeField]
    private float lastSameBoundaryEnergyRatio;

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
        // Unity Physics owns position.  Only the stable T component may be
        // adjusted to close Subject distance; N remains owned by damping.
        // ------------------------------------------------------------

        RefreshReleaseFeasibility();
        ApplySubjectTransportConvergence();
        RefreshReleaseFeasibility();
        UpdateObserver();
        UpdateDynamicPhaseFromContacts();
    }


    // ================================================================
    // Reference resolution
    // ================================================================

    private void ResolveReferences()
    {
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

        // If a real Subject was assigned in the Inspector, always prefer it.
        if (subjectBody &&
            (!subjectTransform ||
             subjectTransform != subjectBody.transform))
        {
            subjectTransform =
                subjectBody.transform;
        }

        // Subject may be spawned after Start.  Retry the real target whenever
        // no target exists or the current target is only the BallVisual proxy.
        bool targetIsProxy =
            ballVisual &&
            subjectTransform ==
                ballVisual.transform;

        if (!subjectTransform ||
            usingBallVisualAsSubjectProxy ||
            targetIsProxy)
        {
            GameObject subjectObject =
                GameObject.Find(
                    "Subject");

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

                previousSubjectPosition =
                    subjectTransform.position;

                hasSubjectPositionSample =
                    true;

                estimatedSubjectVelocity =
                    subjectBody
                        ? subjectBody.velocity
                        : (ballVisual
                            ? ballVisual.velocity
                            : Vector3.zero);
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
            ballVisual &&
            subjectTransform ==
                ballVisual.transform;

        if (!subjectTransform &&
            ballVisual)
        {
            // Explicit fallback, not another motion mode.  BallVisual is the
            // visual course already synchronized to Subject in this project,
            // so it is safer than disabling T-direction convergence entirely.
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

    public bool ReleaseToEnvelopeSimulation(
        Vector3 equalizerLaunchVelocity,
        float sourceEnergyJoule,
        float envelopeEntryHeight)
    {
        Debug.Log("######## ReleaseToEnvelopeSimulation ENTER ########");

        ResolveReferences();

        // ------------------------------------------------------------
        // Previous Hopper -> next Incident reset map
        //
        // 以前は synchronized == false のまま次のSlopeへ入ると、
        //
        //     if (!synchronized) return false;
        //
        // で2回目以降のReleaseを全て拒否していた。
        //
        // その結果、
        //   - Equalizerは前SlopeのPhysics軌道を走り続ける
        //   - BallVisualとの距離が拡大する
        //   - 次のEnvelopeが生成されない
        //
        // 次のIncidentが要求された時点をHybrid SystemのReset Mapとし、
        // まずBallVisualへ完全再同期してから新しいEnvelopeをArmする。
        // ------------------------------------------------------------

        if (!synchronized)
        {
            ReacquireForNextIncident();
        }

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

        if (equalizerLaunchVelocity.sqrMagnitude <=
            0.000001f)
        {
            Debug.LogWarning(
                "[EQUALIZER] Launch velocity is zero.",
                this);

            return false;
        }


        // ------------------------------------------------------------
        // ① Reset map: exact synchronization immediately before release.
        // ------------------------------------------------------------

        CopyBallVisualPose();

        Vector3 releasePosition =
            ballVisual.position;

        Quaternion releaseRotation =
            ballVisual.rotation;

        Vector3 releaseAngularVelocity =
            ballVisual.angularVelocity;


        // ------------------------------------------------------------
        // ② Capture initial condition.
        //
        // This is a state snapshot, not a continuously updated target.
        // ------------------------------------------------------------

        float releaseSpeed =
            equalizerLaunchVelocity.magnitude;

        releaseFrame =
            new ReleaseFrame
            {
                position =
                    releasePosition,

                velocity =
                    equalizerLaunchVelocity,

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
                    Mathf.Max(
                        0f,
                        sourceEnergyJoule),

                envelopeEntryHeight =
                    Mathf.Max(
                        0f,
                        envelopeEntryHeight)
            };


        TransitionTo(
            EqualizerPhase.ReleaseArmed,
            "ReleaseRequested");


        // ------------------------------------------------------------
        // ③ Build / arm the envelope before ownership transfer.
        // ------------------------------------------------------------

        bool envelopeReady =
            negativeEnvelope.ArmFromBallVisualEnergy(
                sourceEnergyJoule,
                envelopeEntryHeight,
                equalizerLaunchVelocity);

        if (!envelopeReady)
        {
            TransitionTo(
                EqualizerPhase.Synchronized,
                "EnvelopeBuildFailed");

            Debug.LogWarning(
                "[EQUALIZER] Envelope could not be armed. " +
                "Synchronization is kept.",
                this);

            return false;
        }


        // ------------------------------------------------------------
        // ④ Capture one stable oscillation frame for this Release.
        // The Envelope has already cached the Slope-wide orthogonal basis.
        // ------------------------------------------------------------

        ResetImpactMapState();

        CaptureOscillationFrame(
            releasePosition,
            equalizerLaunchVelocity);


        // ------------------------------------------------------------
        // ⑤ Ownership transfer:
        //
        // BallVisual -> Unity Physics
        // ------------------------------------------------------------

        synchronized =
            false;

        lowerContactCount =
            0;

        upperContactCount =
            0;

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
            equalizerLaunchVelocity;

        ballVisualEqualizer.angularVelocity =
            releaseAngularVelocity;

        ballVisualEqualizer.collisionDetectionMode =
            CollisionDetectionMode.ContinuousDynamic;

        ballVisualEqualizer.WakeUp();

        // Feasibility must be built after the Dynamic launch velocity is
        // assigned, otherwise the initial time-to-limit would be estimated
        // from the kinematic synchronized body's zero transport speed.
        BuildReleaseFeasibility(
            releasePosition,
            equalizerLaunchVelocity);


        TransitionTo(
            EqualizerPhase.FreeFlight,
            "ReleasedToPhysics");


        Debug.Log(
            $"[EQUALIZER RELEASE FRAME] " +
            $"position={releaseFrame.position:F4} " +
            $"velocity={releaseFrame.velocity:F4} " +
            $"speed={releaseFrame.speed:F4}m/s " +
            $"kineticEnergy={releaseFrame.kineticEnergy:F4}J " +
            $"sourceEnergy={releaseFrame.sourceEnergy:F4}J " +
            $"entryHeight={releaseFrame.envelopeEntryHeight:F4}m",
            this);

        Debug.Log(
            $"[EQUALIZER OSCILLATION FRAME] " +
            $"valid={oscillationFrame.valid} " +
            $"source={(oscillationFrame.fromEnvelope ? "Envelope" : "Fallback")} " +
            $"tangent={oscillationFrame.tangent:F4} " +
            $"normal={oscillationFrame.normal:F4} " +
            $"releaseOscEnergy={releaseOscillationEnergy:F4}J " +
            $"releaseOscRatio={releaseOscillationEnergyRatio:F4}",
            this);

        Debug.Log(
            $"[EQUALIZER FEASIBILITY] " +
            $"valid={releaseFeasibility.valid} " +
            $"limitAvailable={releaseFeasibility.limitAvailable} " +
            $"remainingT={releaseFeasibility.remainingTransportDistance:F4}m " +
            $"availableTime={releaseFeasibility.availableTime:F4}s " +
            $"cycle={releaseFeasibility.estimatedCycleTime:F4}s " +
            $"availableCycles={releaseFeasibility.availableCycles:F3} " +
            $"predictedResidualOscE={releaseFeasibility.predictedResidualOscillationEnergy:F4}J " +
            $"dampingFeasibility={releaseFeasibility.dampingFeasibility01:F3} " +
            $"subjectGapT={releaseFeasibility.subjectGap:F4}m " +
            $"subjectCloseMin={releaseFeasibility.minimumSubjectClosingTime:F4}s " +
            $"subjectFeasibility={releaseFeasibility.subjectConvergenceFeasibility:F3} " +
            $"subjectProxy={usingBallVisualAsSubjectProxy}",
            this);

        Debug.Log(
            $"[EQUALIZER PHYSICS STATE] " +
            $"kinematic={ballVisualEqualizer.isKinematic} " +
            $"gravity={ballVisualEqualizer.useGravity} " +
            $"detectCollisions={ballVisualEqualizer.detectCollisions} " +
            $"collisionMode={ballVisualEqualizer.collisionDetectionMode}",
            this);
        Debug.Log("######## BEFORE ARM ########");

        bool envelopeReady1 =
            negativeEnvelope.ArmFromBallVisualEnergy(
                sourceEnergyJoule,
                envelopeEntryHeight,
                equalizerLaunchVelocity);

        Debug.Log(
            "######## AFTER ARM envelopeReady=" +
            envelopeReady1 +
            " ########"
        );
        return true;
    }


    // ================================================================
    // Stable oscillation frame
    // ================================================================

    private void ResetOscillationFrame()
    {
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

        if (!fromEnvelope &&
            Vector3.Dot(
                normal,
                Vector3.up) < 0f)
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

        float normalSpeed =
            Vector3.Dot(
                releaseVelocity,
                normal);

        releaseOscillationEnergy =
            0.5f *
            EqualizerMass *
            normalSpeed *
            normalSpeed;

        float totalEnergy =
            0.5f *
            EqualizerMass *
            releaseVelocity.sqrMagnitude;

        releaseOscillationEnergyRatio =
            totalEnergy > ImpactEnergyEpsilon
                ? releaseOscillationEnergy /
                  totalEnergy
                : 0f;
    }


    // ================================================================
    // Release feasibility - one continuous model, no mode selection
    // ================================================================

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
        ResetReleaseFeasibility();
        BeginReleaseEvaluation();

        currentCanonicalOscillationEnergy =
            Mathf.Max(
                0f,
                releaseOscillationEnergy);

        releaseFeasibility.initialOscillationEnergy =
            releaseOscillationEnergy;

        releaseFeasibility.initialOscillationRatio =
            releaseOscillationEnergyRatio;

        if (!oscillationFrame.valid)
            return;

        Vector3 releaseTangent =
            oscillationFrame.tangent.normalized;

        releaseFeasibility.initialSubjectGap =
            subjectTransform
                ? Vector3.Dot(
                    subjectTransform.position -
                    releasePosition,
                    releaseTangent)
                : 0f;

        Vector3 entryCenterVisual =
            Vector3.zero;

        Vector3 limitCenterVisual =
            Vector3.zero;

        float firstContactTime =
            -1f;

        bool geometryAvailable =
            negativeEnvelope &&
            negativeEnvelope.TryGetLatestFeasibilityGeometryVisual(
                out entryCenterVisual,
                out limitCenterVisual,
                out firstContactTime);

        releaseFeasibility.valid =
            true;

        releaseFeasibility.limitAvailable =
            geometryAvailable;

        releaseFeasibility.limitCenterVisual =
            geometryAvailable
                ? limitCenterVisual
                : releasePosition;

        // The first release->Upper time is a much better initial cadence seed
        // than the full ballistic period because the Envelope intercepts the
        // trajectory before the unconstrained apex.  Once a same-boundary
        // period is observed, runtime observation replaces this seed.
        float cycleSeed =
            firstContactTime > 0f
                ? firstContactTime * 2f
                : EstimateBallisticCycleTime(
                    releaseVelocity);

        releaseFeasibility.estimatedCycleTime =
            Mathf.Max(
                MinimumObservedCyclePeriod,
                cycleSeed);

        RefreshReleaseFeasibility();
    }


    private float EstimateBallisticCycleTime(
        Vector3 velocity)
    {
        if (!oscillationFrame.valid)
            return MinimumObservedCyclePeriod;

        float oscillationSpeed =
            Mathf.Abs(
                Vector3.Dot(
                    velocity,
                    oscillationFrame.normal));

        float gravityAlongNormal =
            Mathf.Abs(
                Vector3.Dot(
                    Physics.gravity,
                    oscillationFrame.normal));

        if (gravityAlongNormal <=
            ImpactEnergyEpsilon)
        {
            return Mathf.Max(
                MinimumObservedCyclePeriod,
                Time.fixedDeltaTime * 4f);
        }

        return Mathf.Max(
            MinimumObservedCyclePeriod,
            2f * oscillationSpeed /
            gravityAlongNormal);
    }


    private float EstimateTravelTime(
        float distance,
        float initialSpeed,
        float acceleration)
    {
        distance =
            Mathf.Max(
                0f,
                distance);

        if (distance <=
            ImpactEnergyEpsilon)
        {
            return 0f;
        }

        initialSpeed =
            Mathf.Max(
                0f,
                initialSpeed);

        if (Mathf.Abs(acceleration) <=
            ImpactEnergyEpsilon)
        {
            return initialSpeed >
                ImpactEnergyEpsilon
                    ? distance /
                      initialSpeed
                    : MaximumFeasibilityTime;
        }

        // Solve 0.5*a*t^2 + v0*t - s = 0.
        float discriminant =
            initialSpeed * initialSpeed +
            2f * acceleration * distance;

        if (discriminant >= 0f)
        {
            float root =
                Mathf.Sqrt(
                    discriminant);

            float t =
                (-initialSpeed + root) /
                acceleration;

            if (t > 0f &&
                !float.IsNaN(t) &&
                !float.IsInfinity(t))
            {
                return t;
            }
        }

        // Deceleration may stop the body before the limit.  In that case the
        // limit is not reachable under the current constant-acceleration model;
        // report the maximum planning horizon instead of inventing extra speed.
        return MaximumFeasibilityTime;
    }


    private float EstimateMinimumSubjectClosingTime(
        float gap,
        float equalizerSpeed,
        float subjectSpeed)
    {
        float distance =
            Mathf.Abs(
                gap);

        if (distance <=
            Mathf.Max(
                subjectTransportDeadZone,
                ImpactEnergyEpsilon))
        {
            return 0f;
        }

        // Positive means the distance is already shrinking.
        float closingSpeed =
            Mathf.Sign(gap) *
            (equalizerSpeed -
             subjectSpeed);

        float acceleration =
            Mathf.Max(
                0.01f,
                maximumTransportAcceleration);

        float discriminant =
            closingSpeed * closingSpeed +
            2f * acceleration * distance;

        float time =
            (-closingSpeed +
             Mathf.Sqrt(
                 Mathf.Max(
                     0f,
                     discriminant))) /
            acceleration;

        return Mathf.Max(
            0f,
            time);
    }


    private void RefreshReleaseFeasibility()
    {
        if (!releaseFeasibility.valid ||
            !ballVisualEqualizer ||
            !oscillationFrame.valid)
        {
            return;
        }

        Vector3 tangent =
            oscillationFrame.tangent.normalized;

        float currentTransportSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    ballVisualEqualizer.velocity,
                    tangent));

        float remainingDistance =
            releaseFeasibility.limitAvailable
                ? Mathf.Max(
                    0f,
                    Vector3.Dot(
                        releaseFeasibility.limitCenterVisual -
                        ballVisualEqualizer.position,
                        tangent))
                : 0f;

        float gravityAlongTangent =
            Vector3.Dot(
                Physics.gravity,
                tangent);

        float availableTime =
            releaseFeasibility.limitAvailable
                ? EstimateTravelTime(
                    remainingDistance,
                    currentTransportSpeed,
                    gravityAlongTangent)
                : MaximumFeasibilityTime;

        availableTime =
            Mathf.Clamp(
                availableTime,
                0f,
                MaximumFeasibilityTime);

        float cycleTime =
            smoothedSameBoundaryPeriod >
            MinimumObservedCyclePeriod
                ? smoothedSameBoundaryPeriod
                : Mathf.Max(
                    MinimumObservedCyclePeriod,
                    releaseFeasibility.estimatedCycleTime);

        float availableCycles =
            availableTime /
            Mathf.Max(
                cycleTime,
                MinimumObservedCyclePeriod);

        // Feasibility follows only the canonical damping ledger.
        // Incidental / emergency contacts must not erase the predicted
        // residual energy simply because a local constraint collapsed to zero.
        float currentLedgerEnergy =
            Mathf.Max(
                0f,
                currentCanonicalOscillationEnergy);

        float contraction =
            Mathf.Clamp(
                sameBoundaryEnergyContraction,
                0f,
                0.985f);

        float predictedResidualEnergy =
            currentLedgerEnergy *
            Mathf.Pow(
                contraction,
                Mathf.Max(
                    0f,
                    availableCycles));

        float dampingFeasibility =
            currentLedgerEnergy >
            ImpactEnergyEpsilon
                ? Mathf.Clamp01(
                    1f -
                    predictedResidualEnergy /
                    currentLedgerEnergy)
                : 1f;

        float gap =
            0f;

        float subjectSpeed =
            0f;

        float minimumClosingTime =
            0f;

        float subjectFeasibility =
            1f;

        if (subjectTransform)
        {
            gap =
                Vector3.Dot(
                    subjectTransform.position -
                    ballVisualEqualizer.position,
                    tangent);

            subjectSpeed =
                Mathf.Max(
                    0f,
                    Vector3.Dot(
                        estimatedSubjectVelocity,
                        tangent));

            minimumClosingTime =
                EstimateMinimumSubjectClosingTime(
                    gap,
                    currentTransportSpeed,
                    subjectSpeed);

            subjectFeasibility =
                minimumClosingTime <=
                ImpactEnergyEpsilon
                    ? 1f
                    : availableTime /
                      minimumClosingTime;
        }

        releaseFeasibility.remainingTransportDistance =
            remainingDistance;

        releaseFeasibility.availableTime =
            availableTime;

        releaseFeasibility.estimatedCycleTime =
            cycleTime;

        releaseFeasibility.availableCycles =
            availableCycles;

        releaseFeasibility.predictedResidualOscillationEnergy =
            predictedResidualEnergy;

        releaseFeasibility.dampingFeasibility01 =
            dampingFeasibility;

        releaseFeasibility.subjectGap =
            gap;

        releaseFeasibility.minimumSubjectClosingTime =
            minimumClosingTime;

        releaseFeasibility.subjectConvergenceFeasibility =
            subjectFeasibility;

        releaseFeasibility.observedSameBoundaryPeriod =
            smoothedSameBoundaryPeriod;
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

        RefreshReleaseFeasibility();
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
        if (!subjectTransform)
            return;

        previousSubjectPosition =
            subjectTransform.position;

        hasSubjectPositionSample =
            true;

        estimatedSubjectVelocity =
            subjectBody
                ? subjectBody.velocity
                : (ballVisual
                    ? ballVisual.velocity
                    : Vector3.zero);
    }


    private void UpdateSubjectMotionEstimate()
    {
        if (!subjectTransform)
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
            subjectTransform.position;

        float dt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f);

        if (subjectBody)
        {
            estimatedSubjectVelocity =
                subjectBody.velocity;

            subjectVelocitySource =
                "SubjectRigidbody";
        }
        else if (ballVisual)
        {
            // BallVisual is already driven from the visual Subject frame in
            // this project, so its Rigidbody velocity is a more stable
            // fallback than differentiating a Transform during stage rotation.
            estimatedSubjectVelocity =
                ballVisual.velocity;

            subjectVelocitySource =
                "BallVisualVelocity";
        }
        else if (hasSubjectPositionSample)
        {
            estimatedSubjectVelocity =
                (currentPosition -
                 previousSubjectPosition) /
                dt;

            subjectVelocitySource =
                "SubjectTransformDelta";
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

        // Do not fight an active contact constraint.  CleanImpactSolve()
        // preserves non-negative T at impact; convergence resumes as soon as
        // the body returns to free flight.
        if (lowerContactCount > 0 ||
            upperContactCount > 0)
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

        Vector3 velocity =
            ballVisualEqualizer.velocity;

        float currentTransportSpeed =
            Vector3.Dot(
                velocity,
                tangent);

        // A clean damping run must not continue backwards between impacts.
        // Remove only the negative T component; N and L are untouched.
        if (currentTransportSpeed < 0f)
        {
            velocity -=
                tangent *
                currentTransportSpeed;

            currentTransportSpeed =
                0f;
        }

        float gap =
            Vector3.Dot(
                subjectTransform.position -
                ballVisualEqualizer.position,
                tangent);

        float subjectSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    estimatedSubjectVelocity,
                    tangent));

        float deadZone =
            Mathf.Max(
                0f,
                subjectTransportDeadZone);

        float gapCommand;

        if (Mathf.Abs(gap) <=
            deadZone)
        {
            gapCommand =
                0f;
        }
        else
        {
            gapCommand =
                Mathf.Sign(gap) *
                (Mathf.Abs(gap) -
                 deadZone);
        }

        float convergenceTime =
            Mathf.Max(
                0.05f,
                subjectConvergenceTime);

        float convergenceCommand =
            gapCommand /
            convergenceTime;

        float remainingTimeCommand =
            0f;

        if (releaseFeasibility.valid &&
            releaseFeasibility.limitAvailable &&
            releaseFeasibility.availableTime >
                MinimumFeasibilityTime)
        {
            remainingTimeCommand =
                gapCommand /
                releaseFeasibility.availableTime;
        }

        // Near the limit, use the stronger command that is still consistent
        // with the sign of the actual Subject gap.  The existing speed and
        // acceleration caps remain hard limits, so impossible convergence is
        // reported as feasibility < 1 instead of being forced by teleport.
        float closingCommand =
            Mathf.Abs(remainingTimeCommand) >
            Mathf.Abs(convergenceCommand)
                ? remainingTimeCommand
                : convergenceCommand;

        float closingSpeed =
            Mathf.Clamp(
                closingCommand,
                -Mathf.Max(
                    0f,
                    maximumTransportSpeedBoost),
                Mathf.Max(
                    0f,
                    maximumTransportSpeedBoost));

        float targetTransportSpeed =
            Mathf.Max(
                0f,
                subjectSpeed +
                closingSpeed);

        float maxAcceleration =
            Mathf.Max(
                0.01f,
                maximumTransportAcceleration);

        float newTransportSpeed =
            Mathf.MoveTowards(
                currentTransportSpeed,
                targetTransportSpeed,
                maxAcceleration *
                Mathf.Max(
                    Time.fixedDeltaTime,
                    0.000001f));

        float deltaTransportSpeed =
            newTransportSpeed -
            currentTransportSpeed;

        ballVisualEqualizer.velocity =
            velocity +
            tangent *
            deltaTransportSpeed;

        subjectConvergenceActive =
            Mathf.Abs(
                deltaTransportSpeed) >
            0.00001f;

        appliedTransportDeltaSpeed =
            deltaTransportSpeed;

        desiredTransportSpeed =
            targetTransportSpeed;
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
                        subjectTransform.position,
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
                subjectTransform.position,
                ballVisualEqualizer.position);

        subjectTransportGap =
            Vector3.Dot(
                subjectTransform.position -
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
        float requestedOscillationSpeed)
    {
        Vector3 tangent =
            oscillationFrame.tangent.normalized;

        Vector3 outgoingAxis =
            frame.oscillationOutgoingAxis.normalized;

        Vector3 contactNormal =
            frame.normal.normalized;

        // At OnCollisionEnter, Rigidbody.velocity reflects Unity's
        // contact-solver result.  Emergency handling starts there and does not
        // project the velocity back onto a wall face with a custom T collapse.
        Vector3 velocity =
            ballVisualEqualizer.velocity;

        float rawTransportSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    frame.incidentVelocity,
                    tangent));

        float solverTransportSpeed =
            Vector3.Dot(
                velocity,
                tangent);

        bool forwardGuardApplied =
            solverTransportSpeed < 0f;

        if (solverTransportSpeed < 0f)
        {
            // Only remove reverse transport.  Do not inject positive T here;
            // emergency contact is not a new authored bounce.
            velocity -=
                tangent *
                solverTransportSpeed;

            solverTransportSpeed =
                0f;
        }

        float finalOscillationSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    velocity,
                    outgoingAxis));

        float finalOscillationEnergy =
            0.5f *
            EqualizerMass *
            finalOscillationSpeed *
            finalOscillationSpeed;

        float finalSeparationSpeed =
            Vector3.Dot(
                velocity,
                contactNormal);

        float transportRetention =
            rawTransportSpeed >
            ImpactEnergyEpsilon
                ? solverTransportSpeed /
                  rawTransportSpeed
                : 1f;

        bool severeTransportLoss =
            rawTransportSpeed >
            MinimumImpactNormalSpeed &&
            transportRetention <
            MinimumCleanTransportRetention;

        bool physicsClean =
            solverTransportSpeed >=
                -CleanConstraintEpsilon &&
            finalOscillationSpeed <=
                requestedOscillationSpeed +
                CleanConstraintEpsilon &&
            finalSeparationSpeed >=
                -CleanConstraintEpsilon;

        return new CleanImpactSolveResult
        {
            velocity = velocity,
            preTransportSpeed = rawTransportSpeed,
            postTransportSpeed = solverTransportSpeed,
            transportRetention = transportRetention,
            requestedOscillationSpeed = requestedOscillationSpeed,
            finalOscillationSpeed = finalOscillationSpeed,
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
                    ballVisualEqualizer.velocity,
                    velocity),
            forwardGuardApplied = forwardGuardApplied,
            oscillationReduced = false,
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

        float requestedEnergy =
            incomingEnergy *
            restitution *
            restitution;

        float previousSameBoundaryEnergy =
            envelopeContact
                ? lastUpperMappedNormalEnergy
                : lastLowerMappedNormalEnergy;

        if (previousSameBoundaryEnergy >= 0f)
        {
            requestedEnergy =
                Mathf.Min(
                    requestedEnergy,
                    previousSameBoundaryEnergy *
                    Mathf.Clamp(
                        sameBoundaryEnergyContraction,
                        0f,
                        0.985f));
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
            previousSameBoundaryNormalEnergy = -1f,
            sameBoundaryEnergyCeiling = -1f,
            sameBoundaryEnergyRatio = -1f,
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

        lastLowerMappedNormalEnergy = -1f;
        lastUpperMappedNormalEnergy = -1f;

        lastMappedNormalEnergy = 0f;
        lastImpactNormalEnergyRatio = 0f;
        lastSameBoundaryEnergyRatio = 0f;
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

        float contraction =
            Mathf.Clamp(
                sameBoundaryEnergyContraction,
                0f,
                0.985f);

        float localMappedNormalEnergy =
            incomingEnergy *
            restitution *
            restitution;

        float previousSameBoundaryNormalEnergy =
            frame.envelopeContact
                ? lastUpperMappedNormalEnergy
                : lastLowerMappedNormalEnergy;

        bool hasPreviousSameBoundaryEnergy =
            previousSameBoundaryNormalEnergy >= 0f;

        float sameBoundaryEnergyCeiling =
            hasPreviousSameBoundaryEnergy
                ? previousSameBoundaryNormalEnergy *
                  contraction
                : float.PositiveInfinity;

        float requestedMappedNormalEnergy =
            hasPreviousSameBoundaryEnergy
                ? Mathf.Min(
                    localMappedNormalEnergy,
                    sameBoundaryEnergyCeiling)
                : localMappedNormalEnergy;

        requestedMappedNormalEnergy =
            Mathf.Max(
                0f,
                requestedMappedNormalEnergy);

        float mass =
            EqualizerMass;

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
                            requestedOutgoingNormalSpeed);

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
                        requestedOutgoingNormalSpeed);

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

        if (canonicalApplied)
        {
            canonicalImpactCount++;

            UpdateObservedCyclePeriod(
                frame.envelopeContact);

            if (frame.envelopeContact)
            {
                lastUpperMappedNormalEnergy =
                    finalMappedNormalEnergy;
            }
            else
            {
                lastLowerMappedNormalEnergy =
                    finalMappedNormalEnergy;
            }

            // Feasibility follows the actually applied canonical energy only.
            currentCanonicalOscillationEnergy =
                finalMappedNormalEnergy;

            if (finalMappedNormalEnergy >
                incomingEnergy +
                ImpactEnergyEpsilon)
            {
                releaseDampingViolationCount++;
            }

            if (hasPreviousSameBoundaryEnergy &&
                finalMappedNormalEnergy >
                previousSameBoundaryNormalEnergy +
                ImpactEnergyEpsilon)
            {
                releaseDampingViolationCount++;
            }
        }
        else if (emergencyUsed)
        {
            emergencyImpactCount++;
        }

        RefreshReleaseFeasibility();

        bool dampingSuccess =
            canonicalApplied &&
            finalMappedNormalEnergy <=
                incomingEnergy +
                ImpactEnergyEpsilon &&
            (!hasPreviousSameBoundaryEnergy ||
             finalMappedNormalEnergy <=
                previousSameBoundaryNormalEnergy +
                ImpactEnergyEpsilon);

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
            transportSuccess &&
            subjectConverging;

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

        float sameBoundaryEnergyRatio =
            canonicalApplied &&
            hasPreviousSameBoundaryEnergy &&
            previousSameBoundaryNormalEnergy >
            ImpactEnergyEpsilon
                ? finalMappedNormalEnergy /
                  previousSameBoundaryNormalEnergy
                : -1f;

        lastMappedNormalEnergy =
            canonicalApplied
                ? finalMappedNormalEnergy
                : lastMappedNormalEnergy;

        lastImpactNormalEnergyRatio =
            mappedNormalEnergyRatio;

        lastSameBoundaryEnergyRatio =
            sameBoundaryEnergyRatio;

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

        frame.previousSameBoundaryNormalEnergy =
            hasPreviousSameBoundaryEnergy
                ? previousSameBoundaryNormalEnergy
                : -1f;

        frame.sameBoundaryEnergyCeiling =
            hasPreviousSameBoundaryEnergy
                ? sameBoundaryEnergyCeiling
                : -1f;

        frame.sameBoundaryEnergyRatio =
            sameBoundaryEnergyRatio;

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
            $"prevSameEoutOsc={(hasPreviousSameBoundaryEnergy ? previousSameBoundaryNormalEnergy : -1f):F6}J " +
            $"ceilingOsc={(hasPreviousSameBoundaryEnergy ? sameBoundaryEnergyCeiling : -1f):F6}J " +
            $"EoutOsc={finalMappedNormalEnergy:F6}J " +
            $"canonicalOscE={currentCanonicalOscillationEnergy:F6}J " +
            $"impactRatio={mappedNormalEnergyRatio:F6} " +
            $"sameBoundaryRatio={sameBoundaryEnergyRatio:F6} " +
            $"e={restitution:F4} " +
            $"rho={contraction:F4} " +
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

    private bool IsEnvelopeCollision(
        Collision collision)
    {
        if (!negativeEnvelope ||
            !collision.collider)
        {
            return false;
        }

        Transform colliderTransform =
            collision.collider.transform;

        return
            colliderTransform ==
                negativeEnvelope.transform ||
            colliderTransform.IsChildOf(
                negativeEnvelope.transform);
    }


    // ================================================================
    // Collision observer
    // ================================================================

    private void OnCollisionEnter(
        Collision collision)
    {
        if (synchronized ||
            !ballVisualEqualizer ||
            collision.contactCount <= 0)
        {
            return;
        }

        bool envelopeContact =
            IsEnvelopeCollision(
                collision);

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
            $"mapSameBoundaryRatio={lastContactFrame.sameBoundaryEnergyRatio:F4} " +
            $"mapEEffective={lastContactFrame.mappedEffectiveRestitution:F4} " +
            $"normalImpulse={lastContactFrame.normalImpulse:F4}",
            this);
    }


    private void OnCollisionStay(
        Collision collision)
    {
        if (synchronized ||
            !ballVisualEqualizer)
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
        if (synchronized)
            return;

        bool envelopeContact =
            IsEnvelopeCollision(
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
