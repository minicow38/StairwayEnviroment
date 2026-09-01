using UnityEngine;

/// <summary>
/// GuiltyStairway - Floating Rigidbody Equalizer.
///
/// Web/Toyful-style responsibility split:
///   Envelope:
///     - Virtual Lower Spline frame
///     - physical Upper boundary
///     - 4R-Hn amplitude / decay
///     - observed Upper->Upper natural period
///
///   Sync:
///     - Rigidbody Spring/Damper around the middle of 4R-Hn
///     - real PhysX Upper collision
///     - velocity-deficit + position-lag catch-up in the transport plane
///
/// Two selectable timing modes are supported:
///   1) NaturalObserved:
///      T is an observed result of the Rigidbody oscillator.
///   2) ThreeWavesPerStair:
///      the active stair Spline domain (release -> terminal target) owns exactly
///      N spatial waves. Temporal T then follows actual progress/speed.
///
/// There is NO Transform animation while released.
/// SlopeStickCore.maxGroundSpeed is READ ONLY.
/// </summary>
[DisallowMultipleComponent]
public sealed class BallVisualEqualizerSync : MonoBehaviour
{
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

    public enum WaveTimingMode
    {
        NaturalObserved,
        ThreeWavesPerStair
    }

    [System.Serializable]
    private struct OscillationFrame
    {
        public bool valid;
        public Vector3 tangent;
        public Vector3 normal;
        public Vector3 lateral;
    }

    [System.Serializable]
    private struct ReleaseFrame
    {
        public Vector3 position;
        public Vector3 subjectPosition;
        public Vector3 velocity;
        public Vector3 angularVelocity;
        public float sourceEnergy;
        public float referenceHeight;
        public Vector3 sourceAxis;
    }

    // ================================================================
    // References
    // ================================================================

    [Header("References")]
    [SerializeField] private Rigidbody ballVisual;
    [SerializeField] private SlopeStickCore slopeCore;
    [SerializeField] private CorrespondSubject correspondSubject;
    [SerializeField] private Transform subjectTransform;
    [SerializeField] private Rigidbody subjectBody;

    [SerializeField] private Rigidbody ballVisualEqualizer;
    [SerializeField] private SphereCollider ballVisualEqualizerCollider;
    [SerializeField] private BallVisualNegativeEnvelopeCollider negativeEnvelope;

    // ================================================================
    // Wave timing mode switch
    // ================================================================

    [Header("Wave Timing Mode")]

    [Tooltip(
        "NaturalObserved: 現在のFloating Rigidbody自然振動。Tは観測結果。\n" +
        "ThreeWavesPerStair: Release->TerminalのSpline区間へ空間的に3波を割り当てる。\n" +
        "速度が変化すると時間周期Tは自動的に変化します。")]
    [SerializeField]
    private WaveTimingMode waveTimingMode = WaveTimingMode.NaturalObserved;

    [Tooltip("ThreeWavesPerStair時の波数。通常は3。")]
    [Range(1, 8)]
    [SerializeField]
    private int spatialWavesPerStair = 3;

    [Tooltip(
        "4R-Hn Envelopeの何割を実Carrier波の希望Lower->Upper高さに使うか。\n" +
        "実際は加速度Budgetにより自動的に小さくなることがあります。")]
    [Range(0.05f, 0.8f)]
    [SerializeField]
    private float spatialCarrierHeightFractionOfEnvelope = 0.22f;

    [Tooltip(
        "3波モードのCarrier波が使ってよいStable-N最大加速度[m/s^2]。\n" +
        "波数3を高速で見せるためNaturalモードより大きくできます。")]
    [Min(50f)]
    [SerializeField]
    private float spatialWaveAccelerationBudget = 1800f;

    [Tooltip("3波モードのStable-N加速度Jerk上限[m/s^3]。")]
    [Min(100f)]
    [SerializeField]
    private float spatialWaveJerkBudget = 12000f;

    [Tooltip(
        "3波モードの参照波加速度feed-forward倍率。1で理論参照加速度を全て使います。")]
    [Range(0f, 1.5f)]
    [SerializeField]
    private float spatialWaveFeedForward = 1f;

    [Tooltip(
        "3波モードでSubjectのTerminal到着時刻へ間に合わせるために許すmaxGroundSpeed超過率。\n" +
        "0.75なら最大1.75倍。SlopeStickCoreへは書き込みません。")]
    [Range(0f, 2f)]
    [SerializeField]
    private float spatialCatchUpSpeedHeadroom01 = 0.75f;

    [Tooltip("3波モードのCatch-up最大加速度[m/s^2]。")]
    [Min(1f)]
    [SerializeField]
    private float spatialMaximumCatchUpAcceleration = 180f;

    [Tooltip("3波モードでGoal速度自体を追従させる最大加速度[m/s^2]。")]
    [Min(1f)]
    [SerializeField]
    private float spatialGoalVelocityAcceleration = 160f;

    // ================================================================
    // Floating Ride Spring / Damper
    // ================================================================

    [Header("Floating Ride - Rigidbody Spring/Damper")]

    [Tooltip(
        "WebのrideSpringStrength相当。Stable-N位置誤差[m]をAccelerationへ変換します。\n" +
        "ForceMode.Accelerationなので mass 非依存の s^-2 相当です。")]
    [Min(1f)]
    [SerializeField] private float rideSpringStrength = 420f;

    [Tooltip(
        "WebのrideSpringDamper相当。Stable-N相対速度[m/s]へ掛ける減衰係数[s^-1]。")]
    [Min(0f)]
    [SerializeField] private float rideSpringDamper = 18f;

    [Tooltip(
        "Stable-Nに掛かるUnity重力を何割相殺するか。\n" +
        "1でFloating Controllerと同様に重力を相殺してSpringが平衡点を所有します。")]
    [Range(0f, 1.5f)]
    [SerializeField] private float gravityCompensation = 1f;

    [Tooltip("Stable-N Spring/Damperの最大加速度[m/s^2]。")]
    [Min(1f)]
    [SerializeField] private float maximumRideAcceleration = 450f;

    [Tooltip("Stable-N加速度の最大変化率[m/s^3]。")]
    [Min(1f)]
    [SerializeField] private float maximumRideJerk = 3000f;

    [Tooltip(
        "4R-Hn内のSpring平衡点。0=Lower、0.5=中央、1=Upper。\n" +
        "通常は0.5。上下へ自然に振幅させるため中央を使います。")]
    [Range(0.05f, 0.95f)]
    [SerializeField] private float rideEquilibrium01 = 0.5f;

    // ================================================================
    // Goal velocity / catch-up
    // ================================================================

    [Header("Transport - Goal Velocity / Catch-up")]

    [Tooltip(
        "WebのgoalVelへMoveTowardsする加速度。Goal自体を急変させないための値[m/s^2]。")]
    [Min(1f)]
    [SerializeField] private float goalVelocityAcceleration = 80f;

    [Tooltip(
        "Subjectとの進行方向位置遅れ[m]を追加Goal速度へ戻す時間[s]。\n" +
        "階段/Upper衝突でタイムロスすると lag/time がGoal速度へ加算されます。")]
    [Min(0.03f)]
    [SerializeField] private float catchUpPositionTime = 0.25f;

    [Tooltip(
        "neededAccel=(goalVel-rb.velocity)/FixedDeltaTime の最大値[m/s^2]。")]
    [Min(1f)]
    [SerializeField] private float maximumCatchUpAcceleration = 90f;

    [Tooltip("Transport加速度の最大変化率[m/s^3]。")]
    [Min(1f)]
    [SerializeField] private float maximumTransportJerk = 1400f;

    [Tooltip(
        "maxGroundSpeedを超えてCatch-upする時の追加許容量。\n" +
        "例0.35なら一時的に1.35倍まで許可。SlopeStickCoreへは書き込みません。")]
    [Range(0f, 1f)]
    [SerializeField] private float catchUpSpeedHeadroom01 = 0.35f;

    [Tooltip(
        "Tangent以外のTransport面ズレを戻す弱いSpring[s^-2]。Stable-Nには作用しません。")]
    [Min(0f)]
    [SerializeField] private float lateralSpringStrength = 12f;

    [Tooltip("横方向相対速度のDamper[s^-1]。")]
    [Min(0f)]
    [SerializeField] private float lateralDamper = 5f;

    // ================================================================
    // Upper impact observation
    // ================================================================

    [Header("Upper Impact Observation")]

    [Tooltip(
        "Upper->Upper実測周期として採用する最小時間[s]。\n" +
        "0ならFixedDeltaTime*2を使用。")]
    [Min(0f)]
    [SerializeField] private float minimumObservedCycleSeconds = 0f;

    [Tooltip(
        "Upper衝突前後のStable-N速度から求めるEnergy retentionをEnvelopeへ反映するか。")]
    [SerializeField] private bool applyMeasuredImpactEnergyLoss = true;

    [Tooltip(
        "衝突後速度がSolver/接触ノイズで極端な場合のEnergy retention下限。")]
    [Range(0f, 1f)]
    [SerializeField] private float minimumImpactEnergyRetention01 = 0.05f;

    [Tooltip(
        "Upperを一度受理した後、4R-Hnのこの高さ率より下へ戻るまで次Upperを受理しません。\n" +
        "Mesh swap / 接触継続による1 FixedUpdate毎の誤カウントを防ぎます。")]
    [Range(0.05f, 0.75f)]
    [SerializeField] private float upperPeakRearmHeight01 = 0.35f;

    [Tooltip(
        "Upper impact energyは接触直後ではなく、この高さ率より下へ離れて下降した時に測定します。")]
    [Range(0.4f, 0.95f)]
    [SerializeField] private float impactMeasurementReleaseHeight01 = 0.85f;

    // ================================================================
    // Runtime diagnostics
    // ================================================================

    [Header("Runtime - Read Only")]
    [SerializeField] private bool synchronized = true;
    [SerializeField] private EqualizerPhase phase = EqualizerPhase.Synchronized;

    [SerializeField] private int waveCycleIndex;

    [SerializeField] private float current4RHnMeters;
    [SerializeField] private float current4RHnR;
    [SerializeField] private float observedNaturalPeriodSeconds;

    [SerializeField] private float rideTargetHeight;
    [SerializeField] private float rideActualHeight;
    [SerializeField] private float ridePositionError;
    [SerializeField] private float rideRelativeNormalVelocity;
    [SerializeField] private float springAcceleration;
    [SerializeField] private float damperAcceleration;
    [SerializeField] private float gravityCompensationAcceleration;
    [SerializeField] private float rideAccelerationCommand;

    [SerializeField] private float transportLagMeters;
    [SerializeField] private float subjectTangentSpeed;
    [SerializeField] private float equalizerTangentSpeed;
    [SerializeField] private float requiredCatchUpSpeed;
    [SerializeField] private float catchUpAccelerationCommand;

    [Header("Spatial 3-Wave Runtime - Read Only")]
    [SerializeField] private float spatialDomainProgress01;
    [SerializeField] private float spatialWavePhase01;
    [SerializeField] private float spatialReferencePeriodSeconds;
    [SerializeField] private float spatialCarrierHeightMeters;
    [SerializeField] private float spatialCarrierFeasibility01 = 1f;
    [SerializeField] private float spatialSubjectTimeToGo;
    [SerializeField] private float spatialRequiredArrivalSpeed;
    [SerializeField] private float spatialArrivalFeasibility01 = 1f;
    [SerializeField] private bool upperPeakArmed = true;

    [SerializeField] private float positionErrorToBallVisual;
    [SerializeField] private float velocityErrorToBallVisual;
    [SerializeField] private float currentKineticEnergy;
    [SerializeField] private float subjectTransportGap;
    [SerializeField] private float subjectDistance;

    [SerializeField] private int physicsCollisionCount;
    [SerializeField] private int upperCollisionCount;

    [SerializeField] private float lastUpperIncomingNormalSpeed;
    [SerializeField] private float lastUpperOutgoingNormalSpeed;
    [SerializeField] private float lastImpactEnergyRetention01 = 1f;

    // Compatibility diagnostics.
    [SerializeField] private float physicsCleanRate = 1f;
    [SerializeField] private float gameImpactSuccessRate = 1f;
    [SerializeField] private float releaseOverallSuccessRate = 1f;
    [SerializeField] private float dampingFeasibility01 = 1f;
    [SerializeField] private float subjectConvergenceFeasibility = 1f;
    [SerializeField] private float availableTimeToLimit;
    [SerializeField] private float averageGameImpactQuality = 1f;

    // ================================================================
    // Runtime state
    // ================================================================

    private const float Epsilon = 0.000001f;

    private OscillationFrame oscillationFrame;
    private ReleaseFrame releaseFrame;

    private Vector3 rideAccelerationState;
    private Vector3 transportAccelerationState;
    private Vector3 goalPlanarVelocityState;

    private float lastUpperContactFixedTime = -1f;

    private bool pendingUpperImpactEnergyMeasurement;
    private float pendingUpperIncomingNormalSpeed;

    // ================================================================
    // Public compatibility
    // ================================================================

    public Rigidbody Body => ballVisualEqualizer;

    public int OscillationCycleIndex => waveCycleIndex;

    public float PlannedMaxGroundSpeedForCycle =>
        negativeEnvelope
            ? negativeEnvelope.PlannedMaxGroundSpeedForCycle
            : 0f;

    public bool IsSynchronized => synchronized;

    public EqualizerPhase Phase => phase;

    public float EqualizerMass =>
        ballVisualEqualizer
            ? Mathf.Max(0.0001f, ballVisualEqualizer.mass)
            : 1f;

    public float PositionErrorToBallVisual => positionErrorToBallVisual;
    public float VelocityErrorToBallVisual => velocityErrorToBallVisual;
    public float CurrentKineticEnergy => currentKineticEnergy;
    public float SubjectTransportGap => subjectTransportGap;
    public float SubjectDistance => subjectDistance;

    public float CleanImpactRate => physicsCleanRate;
    public float PhysicsCleanImpactRate => physicsCleanRate;
    public float GameImpactSuccessRate => gameImpactSuccessRate;
    public float ReleaseOverallSuccessRate => releaseOverallSuccessRate;
    public float DampingFeasibility01 => dampingFeasibility01;
    public float SubjectConvergenceFeasibility => subjectConvergenceFeasibility;
    public float AvailableTimeToLimit => availableTimeToLimit;
    public float AverageGameImpactQuality => averageGameImpactQuality;

    public float ObservedNaturalPeriodSeconds =>
        observedNaturalPeriodSeconds;


    public WaveTimingMode TimingMode => waveTimingMode;
    public bool ThreeWavesPerStairEnabled =>
        waveTimingMode == WaveTimingMode.ThreeWavesPerStair;
    public float SpatialReferencePeriodSeconds => spatialReferencePeriodSeconds;
    public float SpatialCarrierHeightMeters => spatialCarrierHeightMeters;
    public float SpatialCarrierFeasibility01 => spatialCarrierFeasibility01;
    public float SpatialSubjectTimeToGo => spatialSubjectTimeToGo;
    public float SpatialRequiredArrivalSpeed => spatialRequiredArrivalSpeed;
    public float SpatialArrivalFeasibility01 => spatialArrivalFeasibility01;

    // ================================================================
    // Unity
    // ================================================================

    private void Start()
    {
        Debug.Log(
            "[EQUALIZER BUILD] FloatingRigidbody4RHn-Switchable3Wave-20260831-C",
            this);

        ResolveReferences();
        RefreshVisualCollisionOwnership();

        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            Debug.LogError(
                "[EQUALIZER] BallVisual / BallVisualEqualizer reference is missing.",
                this);
            return;
        }

        EnterSynchronizedState(
            "Start");
    }


    private void FixedUpdate()
    {
        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        if (synchronized)
        {
            CopyBallVisualPose();
            UpdateObserver();
            return;
        }

        UpdateWaveTimingAuthority();

        // Measure impact only after the Rigidbody has actually left the Upper
        // and is descending. This avoids reading the contact-constrained
        // velocity on the immediately following FixedUpdate.
        ResolvePendingUpperImpactEnergyLoss();

        UpdateFloatingRideSpring();
        ApplyGoalVelocityCatchUp();
        UpdateObserver();
    }

    // ================================================================
    // References / Subject mapping
    // ================================================================

    private void ResolveReferences()
    {
        if (!ballVisualEqualizer)
        {
            ballVisualEqualizer =
                GetComponent<Rigidbody>();

            if (!ballVisualEqualizer)
            {
                GameObject obj =
                    GameObject.Find(
                        "BallVisualEqualizer");

                if (obj)
                {
                    ballVisualEqualizer =
                        obj.GetComponent<Rigidbody>();
                }
            }
        }

        if (!ballVisualEqualizerCollider &&
            ballVisualEqualizer)
        {
            ballVisualEqualizerCollider =
                ballVisualEqualizer.GetComponent<SphereCollider>();
        }

        if (!ballVisual)
        {
            GameObject obj =
                GameObject.Find(
                    "BallVisual");

            if (obj)
            {
                ballVisual =
                    obj.GetComponent<Rigidbody>();
            }
        }

        if (!slopeCore)
        {
            slopeCore =
                FindFirstObjectByType<SlopeStickCore>();
        }

        if (!correspondSubject)
        {
            correspondSubject =
                FindFirstObjectByType<CorrespondSubject>();
        }

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
            GameObject obj =
                GameObject.Find("Subject");

            if (!obj)
                obj = GameObject.Find("subject");

            if (obj)
            {
                subjectTransform =
                    obj.transform;

                subjectBody =
                    obj.GetComponent<Rigidbody>();
            }
        }

        if (!negativeEnvelope)
        {
            negativeEnvelope =
                FindFirstObjectByType<BallVisualNegativeEnvelopeCollider>();
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
            return correspondSubject.MappedPhysicalVelocity;

        if (subjectBody &&
            subjectBody != ballVisual)
        {
            return subjectBody.velocity;
        }

        return ballVisual
            ? ballVisual.velocity
            : Vector3.zero;
    }

    // ================================================================
    // Synchronization
    // ================================================================

    public void Equalize()
    {
        if (synchronized)
            CopyBallVisualPose();
    }


    private void CopyBallVisualPose()
    {
        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        ballVisualEqualizer.transform.SetPositionAndRotation(
            ballVisual.position,
            ballVisual.rotation);
    }


    private void EnterSynchronizedState(
        string reason)
    {
        if (!ballVisualEqualizer ||
            !ballVisual)
        {
            return;
        }

        synchronized = true;
        phase = EqualizerPhase.Synchronized;

        waveCycleIndex = 0;
        observedNaturalPeriodSeconds = 0f;
        lastUpperContactFixedTime = -1f;
        upperPeakArmed = true;

        spatialDomainProgress01 = 0f;
        spatialWavePhase01 = 0f;
        spatialReferencePeriodSeconds = 0f;
        spatialCarrierHeightMeters = 0f;
        spatialCarrierFeasibility01 = 1f;
        spatialSubjectTimeToGo = 0f;
        spatialRequiredArrivalSpeed = 0f;
        spatialArrivalFeasibility01 = 1f;

        pendingUpperImpactEnergyMeasurement = false;
        pendingUpperIncomingNormalSpeed = 0f;

        rideAccelerationState = Vector3.zero;
        transportAccelerationState = Vector3.zero;
        goalPlanarVelocityState = Vector3.zero;

        oscillationFrame = default;

        if (negativeEnvelope)
        {
            negativeEnvelope.ConfigureSpatialWaveAuthority(
                false,
                Mathf.Max(
                    1,
                    spatialWavesPerStair));
        }

        ballVisualEqualizer.useGravity = false;
        ballVisualEqualizer.detectCollisions = false;

        if (!ballVisualEqualizer.isKinematic)
        {
            ballVisualEqualizer.velocity =
                Vector3.zero;

            ballVisualEqualizer.angularVelocity =
                Vector3.zero;
        }

        ballVisualEqualizer.isKinematic =
            true;

        CopyBallVisualPose();

        Debug.Log(
            $"[EQUALIZER SYNC] reason={reason}",
            this);
    }

    // ================================================================
    // Release
    // ================================================================

    public bool ReleaseToEnvelopeSimulation(
        Vector3 equalizerLaunchVelocity,
        float sourceEnergyJoule,
        float envelopeEntryHeight)
    {
        Vector3 inferredAxis =
            equalizerLaunchVelocity -
            ReadSubjectVelocityVisual();

        if (inferredAxis.sqrMagnitude <= Epsilon)
            inferredAxis = Vector3.up;

        return ReleaseToEnvelopeSimulation(
            equalizerLaunchVelocity,
            sourceEnergyJoule,
            envelopeEntryHeight,
            inferredAxis);
    }


    public bool ReleaseToEnvelopeSimulation(
        Vector3 equalizerLaunchVelocity,
        float sourceEnergyJoule,
        float envelopeEntryHeight,
        Vector3 sourceEnergyAxisVisual)
    {
        ResolveReferences();

        if (!ballVisual ||
            !ballVisualEqualizer ||
            !negativeEnvelope)
        {
            Debug.LogError(
                "[EQUALIZER] Release references are missing.",
                this);
            return false;
        }

        if (!synchronized)
            ReacquireForNextIncident();

        float safeEnergy =
            Mathf.Max(
                0f,
                sourceEnergyJoule);

        float safeHeight =
            Mathf.Max(
                0f,
                envelopeEntryHeight);

        if (safeEnergy <= Epsilon ||
            safeHeight <= Epsilon)
        {
            Debug.LogWarning(
                "[EQUALIZER] Release energy / height is invalid.",
                this);
            return false;
        }

        Vector3 tangent = Vector3.zero;
        Vector3 normal = sourceEnergyAxisVisual;

        if (negativeEnvelope.TryGetReleaseSurfaceFrameVisual(
                out Vector3 releaseTangent,
                out Vector3 releaseNormal))
        {
            tangent = releaseTangent;
            normal = releaseNormal;
        }

        if (normal.sqrMagnitude <= Epsilon)
            normal = Vector3.up;

        normal.Normalize();

        if (tangent.sqrMagnitude <= Epsilon)
        {
            tangent =
                Vector3.ProjectOnPlane(
                    ReadSubjectVelocityVisual(),
                    normal);
        }

        if (tangent.sqrMagnitude <= Epsilon)
            tangent = Vector3.forward;

        tangent =
            Vector3.ProjectOnPlane(
                tangent,
                normal).normalized;

        Vector3 lateral =
            Vector3.Cross(
                normal,
                tangent).normalized;

        Vector3 subjectVelocity =
            ReadSubjectVelocityVisual();

        Vector3 planarTransportVelocity =
            Vector3.ProjectOnPlane(
                subjectVelocity,
                normal);

        float canonicalNormalSpeed =
            Mathf.Sqrt(
                Mathf.Max(
                    0f,
                    2f *
                    safeEnergy /
                    EqualizerMass));

        Vector3 canonicalLaunchVelocity =
            planarTransportVelocity +
            normal *
            canonicalNormalSpeed;

        CopyBallVisualPose();

        Vector3 releasePosition =
            ballVisual.position;

        Quaternion releaseRotation =
            ballVisual.rotation;

        Vector3 releaseAngularVelocity =
            ballVisual.angularVelocity;

        bool envelopeReady =
            negativeEnvelope.ArmFromBallVisualEnergy(
                safeEnergy,
                safeHeight,
                canonicalLaunchVelocity,
                normal,
                1f);

        if (!envelopeReady)
        {
            Debug.LogWarning(
                "[EQUALIZER] Envelope arm failed.",
                this);

            EnterSynchronizedState(
                "EnvelopeArmFailed");

            return false;
        }

        if (negativeEnvelope.TryGetLatestOscillationFrameVisual(
                out Vector3 envelopeTangent,
                out Vector3 envelopeNormal))
        {
            if (envelopeNormal.sqrMagnitude > Epsilon)
                normal = envelopeNormal.normalized;

            if (envelopeTangent.sqrMagnitude > Epsilon)
            {
                tangent =
                    Vector3.ProjectOnPlane(
                        envelopeTangent,
                        normal).normalized;
            }

            lateral =
                Vector3.Cross(
                    normal,
                    tangent).normalized;
        }

        oscillationFrame =
            new OscillationFrame
            {
                valid = true,
                tangent = tangent,
                normal = normal,
                lateral = lateral
            };

        releaseFrame =
            new ReleaseFrame
            {
                position = releasePosition,
                subjectPosition = ReadSubjectPositionVisual(),
                velocity = canonicalLaunchVelocity,
                angularVelocity = releaseAngularVelocity,
                sourceEnergy = safeEnergy,
                referenceHeight = safeHeight,
                sourceAxis = normal
            };

        synchronized = false;
        phase = EqualizerPhase.ReleaseArmed;

        ballVisualEqualizer.isKinematic = false;
        ballVisualEqualizer.detectCollisions = true;
        ballVisualEqualizer.useGravity = true;

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

        ballVisualEqualizer.solverIterations =
            Mathf.Max(
                ballVisualEqualizer.solverIterations,
                12);

        ballVisualEqualizer.solverVelocityIterations =
            Mathf.Max(
                ballVisualEqualizer.solverVelocityIterations,
                4);

        ballVisualEqualizer.WakeUp();

        rideAccelerationState = Vector3.zero;
        transportAccelerationState = Vector3.zero;
        goalPlanarVelocityState =
            planarTransportVelocity;

        waveCycleIndex = 0;
        observedNaturalPeriodSeconds = 0f;
        lastUpperContactFixedTime = -1f;

        pendingUpperImpactEnergyMeasurement = false;
        pendingUpperIncomingNormalSpeed = 0f;

        UpdateWaveTimingAuthority();

        negativeEnvelope.SetUpperEnvelopeSolidEnabled(
            true,
            "FloatingRigidbodyRelease");

        negativeEnvelope.RefreshEqualizerBoundaryCollisionOwnership();

        phase = EqualizerPhase.FreeFlight;

        if (negativeEnvelope.TryGetCurrentPresentationCenterTravel(
                out current4RHnMeters,
                out current4RHnR))
        {
            // diagnostics updated
        }

        Debug.Log(
            $"[EQUALIZER FLOATING RELEASE] " +
            $"E0={safeEnergy:F4}J " +
            $"H0={safeHeight:F4}m " +
            $"vN0={canonicalNormalSpeed:F4}m/s " +
            $"span={current4RHnMeters:F4}m " +
            $"spanR={current4RHnR:F3}R " +
            $"mode={waveTimingMode} " +
            $"waves={(waveTimingMode == WaveTimingMode.ThreeWavesPerStair ? spatialWavesPerStair : 0)} " +
            $"masterT=None",
            this);

        return true;
    }

    // ================================================================
    // Floating Ride Spring
    // ================================================================

    private void UpdateFloatingRideSpring()
    {
        if (!negativeEnvelope ||
            !ballVisualEqualizer)
        {
            return;
        }

        if (!negativeEnvelope.TryGetFloatingRideFrame(
                out Vector3 lowerCenter,
                out _,
                out Vector3 upperCenter,
                out Vector3 tangent,
                out Vector3 normal,
                out float spanMeters,
                out float spanR,
                out float observedPeriod))
        {
            return;
        }

        if (normal.sqrMagnitude <= Epsilon)
            return;

        normal.Normalize();

        tangent =
            Vector3.ProjectOnPlane(
                tangent,
                normal);

        if (tangent.sqrMagnitude <= Epsilon)
            return;

        tangent.Normalize();

        Vector3 lateral =
            Vector3.Cross(
                normal,
                tangent).normalized;

        oscillationFrame =
            new OscillationFrame
            {
                valid = true,
                tangent = tangent,
                normal = normal,
                lateral = lateral
            };

        current4RHnMeters = spanMeters;
        current4RHnR = spanR;

        if (observedPeriod > 0f)
            observedNaturalPeriodSeconds = observedPeriod;

        Vector3 subjectVelocity =
            ReadSubjectVelocityVisual();

        float supportNormalVelocity =
            Vector3.Dot(
                subjectVelocity,
                normal);

        float equalizerNormalVelocity =
            Vector3.Dot(
                ballVisualEqualizer.velocity,
                normal);

        rideRelativeNormalVelocity =
            equalizerNormalVelocity -
            supportNormalVelocity;

        rideActualHeight =
            Vector3.Dot(
                ballVisualEqualizer.position -
                lowerCenter,
                normal);

        // A previously accepted Upper cannot be accepted again until the
        // Rigidbody has actually returned near the virtual Lower.
        if (!upperPeakArmed &&
            spanMeters > Epsilon &&
            rideActualHeight <=
            spanMeters *
            Mathf.Clamp01(
                upperPeakRearmHeight01))
        {
            upperPeakArmed = true;
        }

        float targetNormalVelocity = 0f;
        float targetNormalAcceleration = 0f;
        Vector3 targetCenter;

        bool spatialMode =
            waveTimingMode ==
            WaveTimingMode.ThreeWavesPerStair;

        if (spatialMode &&
            TryResolveSpatialWaveReference(
                lowerCenter,
                spanMeters,
                tangent,
                normal,
                out targetCenter,
                out targetNormalVelocity,
                out targetNormalAcceleration))
        {
            // target / diagnostics are resolved by spatial progress.
        }
        else
        {
            float equilibrium01 =
                Mathf.Clamp01(
                    rideEquilibrium01);

            targetCenter =
                Vector3.Lerp(
                    lowerCenter,
                    upperCenter,
                    equilibrium01);

            rideTargetHeight =
                spanMeters *
                equilibrium01;

            spatialDomainProgress01 = 0f;
            spatialWavePhase01 = 0f;
            spatialReferencePeriodSeconds = 0f;
            spatialCarrierHeightMeters = 0f;
            spatialCarrierFeasibility01 = 1f;
        }

        ridePositionError =
            Vector3.Dot(
                targetCenter -
                ballVisualEqualizer.position,
                normal);

        springAcceleration =
            ridePositionError *
            Mathf.Max(
                0f,
                rideSpringStrength);

        float normalVelocityError =
            targetNormalVelocity -
            rideRelativeNormalVelocity;

        damperAcceleration =
            normalVelocityError *
            Mathf.Max(
                0f,
                rideSpringDamper);

        float gravityAlongNormal =
            ballVisualEqualizer.useGravity
                ? Vector3.Dot(
                    Physics.gravity,
                    normal)
                : 0f;

        gravityCompensationAcceleration =
            -gravityAlongNormal *
            Mathf.Clamp(
                gravityCompensation,
                0f,
                1.5f);

        float desiredScalarAcceleration =
            springAcceleration +
            damperAcceleration +
            gravityCompensationAcceleration;

        if (spatialMode)
        {
            desiredScalarAcceleration +=
                targetNormalAcceleration *
                Mathf.Clamp(
                    spatialWaveFeedForward,
                    0f,
                    1.5f);
        }

        float accelerationLimit =
            spatialMode
                ? Mathf.Max(
                    50f,
                    spatialWaveAccelerationBudget)
                : Mathf.Max(
                    1f,
                    maximumRideAcceleration);

        desiredScalarAcceleration =
            Mathf.Clamp(
                desiredScalarAcceleration,
                -accelerationLimit,
                accelerationLimit);

        Vector3 desiredAcceleration =
            normal *
            desiredScalarAcceleration;

        float dt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f);

        float jerkLimit =
            spatialMode
                ? Mathf.Max(
                    100f,
                    spatialWaveJerkBudget)
                : Mathf.Max(
                    1f,
                    maximumRideJerk);

        rideAccelerationState =
            Vector3.MoveTowards(
                rideAccelerationState,
                desiredAcceleration,
                jerkLimit *
                dt);

        rideAccelerationState =
            Vector3.Project(
                rideAccelerationState,
                normal);

        ballVisualEqualizer.AddForce(
            rideAccelerationState,
            ForceMode.Acceleration);

        rideAccelerationCommand =
            Vector3.Dot(
                rideAccelerationState,
                normal);

        phase =
            rideRelativeNormalVelocity >= 0f
                ? EqualizerPhase.HopperFlight
                : EqualizerPhase.FreeFlight;
    }


    private bool TryResolveSpatialWaveReference(
        Vector3 lowerCenter,
        float envelopeSpanMeters,
        Vector3 tangent,
        Vector3 normal,
        out Vector3 targetCenter,
        out float targetNormalVelocity,
        out float targetNormalAcceleration)
    {
        targetCenter = lowerCenter;
        targetNormalVelocity = 0f;
        targetNormalAcceleration = 0f;

        if (!negativeEnvelope ||
            envelopeSpanMeters <= Epsilon)
        {
            return false;
        }

        if (!negativeEnvelope.TryGetActiveSplineWaveDomain(
                out float releaseProgress01,
                out float targetProgress01,
                out float equalizerProgress01,
                out float activeArcLengthMeters))
        {
            return false;
        }

        float progressRange =
            targetProgress01 -
            releaseProgress01;

        if (progressRange <= Epsilon ||
            activeArcLengthMeters <= Epsilon)
        {
            return false;
        }

        spatialDomainProgress01 =
            Mathf.Clamp01(
                (equalizerProgress01 -
                 releaseProgress01) /
                progressRange);

        int waveCount =
            Mathf.Max(
                1,
                spatialWavesPerStair);

        float totalWavePhase =
            spatialDomainProgress01 *
            waveCount;

        int completedWaveCount =
            Mathf.Clamp(
                Mathf.FloorToInt(
                    totalWavePhase),
                0,
                waveCount - 1);

        spatialWavePhase01 =
            totalWavePhase -
            Mathf.Floor(
                totalWavePhase);

        if (spatialDomainProgress01 >= 0.999999f)
            spatialWavePhase01 = 0f;

        waveCycleIndex =
            completedWaveCount;

        negativeEnvelope.SetSpatialPresentationProgress(
            spatialDomainProgress01);

        float equalizerForwardSpeed =
            Vector3.Dot(
                Vector3.ProjectOnPlane(
                    ballVisualEqualizer.velocity,
                    normal),
                tangent);

        float subjectForwardSpeed =
            Vector3.Dot(
                Vector3.ProjectOnPlane(
                    ReadSubjectVelocityVisual(),
                    normal),
                tangent);

        float speedForFeasibility =
            Mathf.Max(
                0.25f,
                Mathf.Max(
                    Mathf.Abs(
                        equalizerForwardSpeed),
                    Mathf.Abs(
                        subjectForwardSpeed)));

        float spatialFrequency =
            waveCount *
            speedForFeasibility /
            activeArcLengthMeters;

        spatialReferencePeriodSeconds =
            1f /
            Mathf.Max(
                0.0001f,
                spatialFrequency);

        float desiredCarrierHeight =
            envelopeSpanMeters *
            Mathf.Clamp(
                spatialCarrierHeightFractionOfEnvelope,
                0.05f,
                0.8f);

        // h = H/2(1-cos wt)
        // |a|max = H/2 * w^2 = 2*pi^2*H*f^2
        float feasibleCarrierHeight =
            Mathf.Max(
                0f,
                spatialWaveAccelerationBudget) /
            Mathf.Max(
                0.0001f,
                2f *
                Mathf.PI *
                Mathf.PI *
                spatialFrequency *
                spatialFrequency);

        spatialCarrierHeightMeters =
            Mathf.Min(
                envelopeSpanMeters * 0.95f,
                Mathf.Min(
                    desiredCarrierHeight,
                    feasibleCarrierHeight));

        spatialCarrierFeasibility01 =
            desiredCarrierHeight > Epsilon
                ? Mathf.Clamp01(
                    spatialCarrierHeightMeters /
                    desiredCarrierHeight)
                : 1f;

        float theta =
            2f *
            Mathf.PI *
            spatialWavePhase01;

        float wave01 =
            0.5f *
            (1f -
             Mathf.Cos(
                 theta));

        rideTargetHeight =
            spatialCarrierHeightMeters *
            wave01;

        targetCenter =
            lowerCenter +
            normal *
            rideTargetHeight;

        float signedPhaseRateCyclesPerSecond =
            waveCount *
            equalizerForwardSpeed /
            activeArcLengthMeters;

        float angularRate =
            2f *
            Mathf.PI *
            signedPhaseRateCyclesPerSecond;

        targetNormalVelocity =
            spatialCarrierHeightMeters *
            0.5f *
            Mathf.Sin(
                theta) *
            angularRate;

        targetNormalAcceleration =
            spatialCarrierHeightMeters *
            0.5f *
            Mathf.Cos(
                theta) *
            angularRate *
            angularRate;

        return true;
    }

    // ================================================================
    // Goal velocity catch-up
    // ================================================================

    private void ApplyGoalVelocityCatchUp()
    {
        if (!oscillationFrame.valid ||
            !ballVisualEqualizer)
        {
            return;
        }

        Vector3 normal =
            oscillationFrame.normal.normalized;

        Vector3 tangent =
            Vector3.ProjectOnPlane(
                oscillationFrame.tangent,
                normal);

        if (tangent.sqrMagnitude <= Epsilon)
            return;

        tangent.Normalize();

        Vector3 lateral =
            oscillationFrame.lateral;

        if (lateral.sqrMagnitude <= Epsilon)
        {
            lateral =
                Vector3.Cross(
                    normal,
                    tangent);
        }

        if (lateral.sqrMagnitude > Epsilon)
            lateral.Normalize();

        float dt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f);

        Vector3 subjectPosition =
            ReadSubjectPositionVisual();

        Vector3 subjectVelocity =
            ReadSubjectVelocityVisual();

        Vector3 planeGap =
            Vector3.ProjectOnPlane(
                subjectPosition -
                ballVisualEqualizer.position,
                normal);

        transportLagMeters =
            Vector3.Dot(
                planeGap,
                tangent);

        Vector3 subjectPlanarVelocity =
            Vector3.ProjectOnPlane(
                subjectVelocity,
                normal);

        subjectTangentSpeed =
            Vector3.Dot(
                subjectPlanarVelocity,
                tangent);

        Vector3 equalizerPlanarVelocity =
            Vector3.ProjectOnPlane(
                ballVisualEqualizer.velocity,
                normal);

        equalizerTangentSpeed =
            Vector3.Dot(
                equalizerPlanarVelocity,
                tangent);

        Vector3 rawGoalVelocity =
            subjectPlanarVelocity +
            planeGap /
            Mathf.Max(
                0.03f,
                catchUpPositionTime);

        bool spatialMode =
            waveTimingMode ==
            WaveTimingMode.ThreeWavesPerStair;

        // In 3-wave mode, position lag alone is not enough.
        // Recompute the speed required for Equalizer to reach the same Spline
        // terminal at the Subject's current estimated arrival time.
        if (spatialMode)
        {
            ApplySpatialArrivalDeadlineToGoal(
                ref rawGoalVelocity,
                subjectPosition,
                subjectPlanarVelocity,
                tangent,
                lateral);
        }
        else
        {
            spatialSubjectTimeToGo = 0f;
            spatialRequiredArrivalSpeed = 0f;
            spatialArrivalFeasibility01 = 1f;
        }

        requiredCatchUpSpeed =
            Vector3.Dot(
                rawGoalVelocity,
                tangent);

        if (negativeEnvelope.TryGetSourceMaxGroundSpeedReadOnly(
                out float maxGroundSpeed) &&
            maxGroundSpeed > Epsilon)
        {
            float headroom =
                spatialMode
                    ? Mathf.Max(
                        0f,
                        spatialCatchUpSpeedHeadroom01)
                    : Mathf.Clamp01(
                        catchUpSpeedHeadroom01);

            float maximumGoalSpeed =
                maxGroundSpeed *
                (1f + headroom);

            float tangentGoal =
                Mathf.Clamp(
                    Vector3.Dot(
                        rawGoalVelocity,
                        tangent),
                    -maximumGoalSpeed,
                    maximumGoalSpeed);

            Vector3 lateralGoal =
                lateral.sqrMagnitude > Epsilon
                    ? lateral *
                      Vector3.Dot(
                          rawGoalVelocity,
                          lateral)
                    : Vector3.zero;

            rawGoalVelocity =
                tangent *
                tangentGoal +
                lateralGoal;

            requiredCatchUpSpeed =
                tangentGoal;

            if (spatialMode &&
                spatialRequiredArrivalSpeed > Epsilon)
            {
                spatialArrivalFeasibility01 =
                    Mathf.Clamp01(
                        maximumGoalSpeed /
                        spatialRequiredArrivalSpeed);
            }
        }

        float goalAccel =
            spatialMode
                ? Mathf.Max(
                    1f,
                    spatialGoalVelocityAcceleration)
                : Mathf.Max(
                    1f,
                    goalVelocityAcceleration);

        goalPlanarVelocityState =
            Vector3.MoveTowards(
                goalPlanarVelocityState,
                rawGoalVelocity,
                goalAccel *
                dt);

        Vector3 neededAcceleration =
            (goalPlanarVelocityState -
             equalizerPlanarVelocity) /
            dt;

        if (lateral.sqrMagnitude > Epsilon)
        {
            float lateralGap =
                Vector3.Dot(
                    planeGap,
                    lateral);

            float lateralRelativeVelocity =
                Vector3.Dot(
                    ballVisualEqualizer.velocity -
                    subjectVelocity,
                    lateral);

            neededAcceleration +=
                lateral *
                (lateralGap *
                     Mathf.Max(
                         0f,
                         lateralSpringStrength) -
                 lateralRelativeVelocity *
                     Mathf.Max(
                         0f,
                         lateralDamper));
        }

        neededAcceleration =
            Vector3.ProjectOnPlane(
                neededAcceleration,
                normal);

        float catchUpAccelLimit =
            spatialMode
                ? Mathf.Max(
                    1f,
                    spatialMaximumCatchUpAcceleration)
                : Mathf.Max(
                    1f,
                    maximumCatchUpAcceleration);

        neededAcceleration =
            Vector3.ClampMagnitude(
                neededAcceleration,
                catchUpAccelLimit);

        transportAccelerationState =
            Vector3.MoveTowards(
                transportAccelerationState,
                neededAcceleration,
                Mathf.Max(
                    1f,
                    maximumTransportJerk) *
                dt);

        transportAccelerationState =
            Vector3.ProjectOnPlane(
                transportAccelerationState,
                normal);

        ballVisualEqualizer.AddForceAtPosition(
            transportAccelerationState *
            EqualizerMass,
            ballVisualEqualizer.worldCenterOfMass,
            ForceMode.Force);

        catchUpAccelerationCommand =
            Vector3.Dot(
                transportAccelerationState,
                tangent);
    }


    private void ApplySpatialArrivalDeadlineToGoal(
        ref Vector3 rawGoalVelocity,
        Vector3 subjectPosition,
        Vector3 subjectPlanarVelocity,
        Vector3 tangent,
        Vector3 lateral)
    {
        if (!negativeEnvelope)
            return;

        if (!negativeEnvelope.TryGetActiveSplineWaveDomain(
                out _,
                out float targetProgress01,
                out float equalizerProgress01,
                out _))
        {
            return;
        }

        if (!negativeEnvelope.TryProjectVisualPointToSplineFrameVisual(
                subjectPosition,
                out _,
                out _,
                out _,
                out _,
                out float subjectProgress01))
        {
            return;
        }

        float subjectRemaining =
            subjectProgress01 >=
            targetProgress01 - 0.000001f
                ? 0f
                : negativeEnvelope.EstimateSplineArcDistanceBetweenProgress(
                    subjectProgress01,
                    targetProgress01);

        float equalizerRemaining =
            equalizerProgress01 >=
            targetProgress01 - 0.000001f
                ? 0f
                : negativeEnvelope.EstimateSplineArcDistanceBetweenProgress(
                    equalizerProgress01,
                    targetProgress01);

        subjectRemaining =
            Mathf.Max(
                0f,
                subjectRemaining);

        equalizerRemaining =
            Mathf.Max(
                0f,
                equalizerRemaining);

        float forwardSubjectSpeed =
            Mathf.Abs(
                Vector3.Dot(
                    subjectPlanarVelocity,
                    tangent));

        spatialSubjectTimeToGo =
            subjectRemaining /
            Mathf.Max(
                0.5f,
                forwardSubjectSpeed);

        spatialSubjectTimeToGo =
            Mathf.Max(
                Time.fixedDeltaTime * 2f,
                spatialSubjectTimeToGo);

        spatialRequiredArrivalSpeed =
            equalizerRemaining /
            spatialSubjectTimeToGo;

        float directionSign =
            Vector3.Dot(
                subjectPlanarVelocity,
                tangent) >= 0f
                ? 1f
                : -1f;

        float signedRequiredSpeed =
            spatialRequiredArrivalSpeed *
            directionSign;

        float currentGoalTangent =
            Vector3.Dot(
                rawGoalVelocity,
                tangent);

        float deadlineGoalTangent =
            directionSign >= 0f
                ? Mathf.Max(
                    currentGoalTangent,
                    signedRequiredSpeed)
                : Mathf.Min(
                    currentGoalTangent,
                    signedRequiredSpeed);

        Vector3 lateralGoal =
            lateral.sqrMagnitude > Epsilon
                ? lateral *
                  Vector3.Dot(
                      rawGoalVelocity,
                      lateral)
                : Vector3.zero;

        rawGoalVelocity =
            tangent *
            deadlineGoalTangent +
            lateralGoal;
    }

    // ================================================================
    // Upper collision -> measured T / measured energy loss
    // ================================================================

    private void OnCollisionEnter(
        Collision collision)
    {
        if (synchronized ||
            collision == null ||
            collision.contactCount <= 0)
        {
            return;
        }

        physicsCollisionCount++;

        if (!negativeEnvelope ||
            !negativeEnvelope.IsUpperEnvelopeCollider(
                collision.collider))
        {
            return;
        }

        // A collider rebuild or continuous contact can generate another
        // OnCollisionEnter while the body is still near the same Upper.
        // Accept only one Upper until the body has returned near Virtual Lower.
        if (!upperPeakArmed)
            return;

        upperPeakArmed = false;
        upperCollisionCount++;
        phase = EqualizerPhase.UpperContact;

        float now =
            Time.fixedTime;

        float minimumPeriod =
            minimumObservedCycleSeconds > 0f
                ? minimumObservedCycleSeconds
                : Time.fixedDeltaTime * 2f;

        if (lastUpperContactFixedTime >= 0f)
        {
            float measuredPeriod =
                now -
                lastUpperContactFixedTime;

            if (measuredPeriod >= minimumPeriod)
            {
                observedNaturalPeriodSeconds =
                    measuredPeriod;

                negativeEnvelope.SubmitObservedGeometryPeriod(
                    measuredPeriod);
            }
        }

        lastUpperContactFixedTime = now;

        if (waveTimingMode ==
            WaveTimingMode.NaturalObserved)
        {
            waveCycleIndex++;
            negativeEnvelope.NotifyCanonicalUpperPeak();
        }

        if (oscillationFrame.valid)
        {
            Vector3 normal =
                oscillationFrame.normal.normalized;

            float incoming =
                Mathf.Abs(
                    Vector3.Dot(
                        collision.relativeVelocity,
                        normal));

            if (incoming > Epsilon)
            {
                pendingUpperImpactEnergyMeasurement = true;
                pendingUpperIncomingNormalSpeed = incoming;
                lastUpperIncomingNormalSpeed = incoming;
            }
        }

        // PhysX owns collision response. Transport catch-up reacts later.
    }


    private void ResolvePendingUpperImpactEnergyLoss()
    {
        if (!pendingUpperImpactEnergyMeasurement ||
            !ballVisualEqualizer ||
            !negativeEnvelope ||
            !oscillationFrame.valid)
        {
            return;
        }

        Vector3 normal =
            oscillationFrame.normal.normalized;

        float supportNormalVelocity =
            Vector3.Dot(
                ReadSubjectVelocityVisual(),
                normal);

        float relativeNormalVelocity =
            Vector3.Dot(
                ballVisualEqualizer.velocity,
                normal) -
            supportNormalVelocity;

        // Do not sample while PhysX is still constraining the body at Upper.
        // Wait until the ball has clearly left Upper and is descending.
        float releaseHeight =
            current4RHnMeters *
            Mathf.Clamp01(
                impactMeasurementReleaseHeight01);

        if (current4RHnMeters > Epsilon &&
            (rideActualHeight > releaseHeight ||
             relativeNormalVelocity >= 0f))
        {
            return;
        }

        pendingUpperImpactEnergyMeasurement = false;

        float outgoing =
            Mathf.Abs(
                relativeNormalVelocity);

        lastUpperOutgoingNormalSpeed = outgoing;

        float incoming =
            Mathf.Max(
                Epsilon,
                pendingUpperIncomingNormalSpeed);

        float retention =
            Mathf.Clamp01(
                (outgoing * outgoing) /
                (incoming * incoming));

        retention =
            Mathf.Max(
                Mathf.Clamp01(
                    minimumImpactEnergyRetention01),
                retention);

        lastImpactEnergyRetention01 = retention;

        if (applyMeasuredImpactEnergyLoss)
        {
            float currentRatio =
                negativeEnvelope.CanonicalDampingEnergyRatio;

            negativeEnvelope.SetCanonicalEnergyRatio(
                currentRatio *
                retention);
        }

        pendingUpperIncomingNormalSpeed = 0f;
    }


    private void OnCollisionExit(
        Collision collision)
    {
        if (!synchronized)
            phase = EqualizerPhase.FreeFlight;
    }

    private void UpdateWaveTimingAuthority()
    {
        if (!negativeEnvelope)
            return;

        negativeEnvelope.ConfigureSpatialWaveAuthority(
            waveTimingMode ==
            WaveTimingMode.ThreeWavesPerStair,
            Mathf.Max(
                1,
                spatialWavesPerStair));
    }


    // ================================================================
    // Observer
    // ================================================================

    private void UpdateObserver()
    {
        if (!ballVisualEqualizer)
            return;

        if (ballVisual)
        {
            positionErrorToBallVisual =
                Vector3.Distance(
                    ballVisualEqualizer.position,
                    ballVisual.position);

            velocityErrorToBallVisual =
                Vector3.Distance(
                    ballVisualEqualizer.velocity,
                    ballVisual.velocity);
        }

        currentKineticEnergy =
            0.5f *
            EqualizerMass *
            ballVisualEqualizer.velocity.sqrMagnitude;

        Vector3 subjectPosition =
            ReadSubjectPositionVisual();

        subjectDistance =
            Vector3.Distance(
                ballVisualEqualizer.position,
                subjectPosition);

        if (oscillationFrame.valid)
        {
            subjectTransportGap =
                Vector3.ProjectOnPlane(
                    subjectPosition -
                    ballVisualEqualizer.position,
                    oscillationFrame.normal).magnitude;
        }
        else
        {
            subjectTransportGap =
                subjectDistance;
        }
    }

    // ================================================================
    // Regain compatibility
    // ================================================================

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
            oscillationFrame.valid
                ? oscillationFrame.normal
                : Vector3.up;

        return
            !synchronized &&
            ballVisualEqualizer &&
            oscillationFrame.valid;
    }

    // ================================================================
    // Reacquire / external compatibility
    // ================================================================

    private void ReacquireForNextIncident()
    {
        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        if (negativeEnvelope)
        {
            negativeEnvelope.SetUpperEnvelopeSolidEnabled(
                false,
                "Reacquire");

            negativeEnvelope.ClearEnvelope();
        }

        phase = EqualizerPhase.Reacquiring;

        EnterSynchronizedState(
            "NextIncidentReacquired");
    }


    public void ResumeSynchronization()
    {
        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        if (negativeEnvelope)
        {
            negativeEnvelope.SetUpperEnvelopeSolidEnabled(
                false,
                "ResumeSynchronization");

            negativeEnvelope.ClearEnvelope();
        }

        phase = EqualizerPhase.Reacquiring;

        EnterSynchronizedState(
            "ResumeRequested");
    }


    public void ResumeSynchronization1()
    {
        ResumeSynchronization();
    }


    public void ResumeSynchronization2()
    {
        ResumeSynchronization();
    }


    public void PrepareForVisualFrameTurnMapping()
    {
        ResolveReferences();
    }


    public void ApplyVisualFrameTurnDelta(
        Vector3 pivot,
        Quaternion deltaTurn)
    {
        ResolveReferences();

        if (synchronized)
            CopyBallVisualPose();
    }

    // ================================================================
    // Collision ownership
    // ================================================================

    public void RefreshVisualCollisionOwnership()
    {
        ResolveReferences();

        if (!ballVisual ||
            !ballVisualEqualizerCollider)
        {
            return;
        }

        SphereCollider ballVisualCollider =
            ballVisual.GetComponent<SphereCollider>();

        if (ballVisualCollider)
        {
            Physics.IgnoreCollision(
                ballVisualCollider,
                ballVisualEqualizerCollider,
                true);
        }

        if (negativeEnvelope)
        {
            negativeEnvelope.RefreshEqualizerBoundaryCollisionOwnership();
        }
    }
}