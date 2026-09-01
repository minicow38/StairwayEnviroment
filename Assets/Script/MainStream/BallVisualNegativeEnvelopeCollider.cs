using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using UnityEngine.Serialization;


/// <summary>
/// BallVisualEqualizer専用のUpper Envelopeと4R-Hn/Spline基準を提供します。
///
/// 有効な物理仕様は次の1系統だけです。
///   Lower : SlopeStickCoreの連続Spline surface
///   Hybrid decay signal:
///       q_time(t) = exp(-gamma * t)
///       q_energy(t) = min(epsilon, lerp(1, q_time, waveTimeDecayInfluence))
///   Human presentation ceiling:
///       C_n = B(n) * R
///   Applied wave loss:
///       q_old(t) = epsilon * q_time(t)
///       q(t) = lerp(1, q_old(t), waveTimeDecayInfluence)
///       D_n  = C_n * q(t)
///       H_n  = C_n - D_n
///
/// Presentation Ceilingは「そこまで必ず上げる目標」ではなく上限です。
/// Q版のように旧D_legacyの絶対値をそのまま採用せず、旧Envelopeが持っていた
/// 減衰率 epsilon * exp(-gamma*t) だけを抽出してCeilingへ穏やかに反映します。
/// これにより旧Canonical/指数減衰の形を残しつつ、初期波が0.1R～0.9Rへ潰れるのを防ぎます。
///
/// 周期TはこのComponentの制御目標ではありません。
/// BallVisualEqualizerSyncのRigidbody Spring/Damperと実Upper衝突から自然に生じた
/// Upper->Upper実測周期を診断値として保持します。preferredContactPeriodSecondsは
/// 実測前のfallback値だけに使用します。
///
/// LowerはVirtual Turnpointです。物理Lower Collider/Trigger/Pressは生成しません。
/// Equalizer自身の直下Spline射影をLower中心とし、現在Presentation center travelを
/// Upper中心までのHybrid Wave振幅としてSyncへ公開します。
///
/// S(T) は以前成功した300-400m/s^2帯の中心値を基準に [2R,4R] で保持します。
/// gammaはReleaseからExact LimitまでのSpline移動時間から一度だけ求めます。
/// First Contact方式選択、Curved Offset、World-Y補正は使用しません。
/// SlopeStickCore / CorrespondSubjectはREAD ONLYです。
/// </summary>
[DisallowMultipleComponent]
public sealed class BallVisualNegativeEnvelopeCollider : MonoBehaviour
{
    // ================================================================
    // FLOATING RIGIDBODY ENVELOPE 2026-08-31
    // Upper is the only physical impact boundary. Lower is the live under-Spline
    // virtual reference. This component owns geometry + 4R-Hn + decay only.
    // Sync owns Rigidbody Spring/Damper and velocity-deficit catch-up.
    // No master phase clock is imposed on the Rigidbody.
    // ================================================================

    // ================================================================
    // References
    // ================================================================

    [Header("References")]
    [Tooltip("Spline版InSubject。READ ONLYで参照し、このComponentからSlopeStickCoreを書き換えません。")]
    [SerializeField]
    private SlopeStickCore slopeCore;

    [Tooltip("PhysicsRoot座標 -> VisualPlayerRoot座標への正式な変換に使用します。")]
    [SerializeField]
    private CorrespondSubject correspondSubject;

    [Tooltip("包絡線内を実際に跳ねるBallVisualEqualizerのRigidbodyです。")] [SerializeField]
    private Rigidbody ballVisualEqualizer;

    [Tooltip("BallVisualEqualizerのSphereColliderです。")] [SerializeField]
    private SphereCollider ballVisualEqualizerCollider;

    [SerializeField] private Collider inSubjectCollider;
    // ================================================================
    // Geometry only
    // 物理挙動を調整する値ではなく、Collider近似精度です。
    // ================================================================

    [Header("Envelope Geometry")] [Tooltip("指数Envelope Meshを進行方向に何分割してサンプリングするか。")] [Range(8, 64)] [SerializeField]
    private int segmentCount = 32;

    [Tooltip("進行方向と直交するEnvelopeの全幅[m]。中心から左右へ envelopeWidth / 2 ずつ広がります。")] [Min(0.1f)] [SerializeField]
    private float envelopeWidth = 10.0f;

    // ================================================================
    // Human presentation controls
    // ================================================================

    [Header("Wave Presentation - Human Controls")]
    [Tooltip(
        "Upper -> Lower -> Upper の好ましい1周期 T[s]。絶対deadlineではありません。\n" +
        "通常LegはこのT/2を目安にし、Turnpointで物理時間が不足する場合は実時間を優先します。")]
    [Min(0.04f)]
    [FormerlySerializedAs("targetContactPeriodSeconds")]
    [SerializeField, HideInInspector]
    private float preferredContactPeriodSeconds = 0.10f;

    [Tooltip(
        "X=波番号(0が第1波)、Y=その波で許可するLower->Upper中心移動量[R]。\n" +
        "これは目標値ではなくCeilingです。旧減衰モデルがこれより低ければ旧減衰を優先します。")]
    [SerializeField, HideInInspector]
    private AnimationCurve presentationCeilingR =
        new AnimationCurve(
            new Keyframe(0f, 4.0f),
            new Keyframe(1f, 3.8f),
            new Keyframe(2f, 3.3f),
            new Keyframe(3f, 2.5f),
            new Keyframe(6f, 0.8f));

    [Tooltip("Ceiling Curveが極端に小さくなった場合の数値安全下限[R]。演出値ではありません。")]
    [Min(0.01f)]
    [SerializeField, HideInInspector]
    private float minimumPresentationCeilingR = 0.05f;

    [Tooltip(
        "時間減衰 exp(-gamma*t) とCanonical Energy減衰をWave Ceilingへ何割反映するか。\n" +
        "0 = Ceilingだけ、1 = 減衰率を100%反映。0.35なら損失の35%を連続Wave収縮へ反映します。\n" +
        "旧D_legacyの絶対高さは使わないため、H0が小さいReleaseでも波全体が即座に潰れません。")]
    [Range(0f, 1f)]
    [FormerlySerializedAs("legacyDecayLossInfluence")]
    [SerializeField, HideInInspector]
    private float waveTimeDecayInfluence = 0.35f;

    [Header("Hybrid Wave Event Capture")]
    [Tooltip("ApproachUpper中の実Upper接触を正規Wave eventとして認識するN方向Capture幅[R]。物理Collider形状は変更しません。")]
    [SerializeField, HideInInspector, Range(0.10f, 2f)]
    private float hybridUpperCaptureDistanceR = 0.75f;

    [Tooltip("Virtual Lowerの厳密Turn判定より手前でCandidate監視を始める距離[R]。Colliderは生成しません。")]
    [SerializeField, HideInInspector, Range(0.10f, 1f)]
    private float hybridLowerCandidateDistanceR = 0.30f;

    [Tooltip("Upper event認識に必要なStable-N進入速度を平均Half-Wave速度の何割にするか。")]
    [SerializeField, HideInInspector, Range(0.005f, 0.20f)]
    private float hybridUpperApproachSpeedRatio = 0.03f;

    [Header("Wave Presentation - Physical Timing Resolver")]
    [Tooltip(
        "A_n = B_n R - H_n をPreferred T/2へ押し込むために必要なPhase加速度の上限[m/s^2]。\n" +
        "超える場合は振幅を潰さず、T/2をFixedUpdate整数ステップで延長します。")]
    [Min(1f)]
    [SerializeField, HideInInspector]
    private float maximumPresentationPhaseAcceleration = 350f;

    [Tooltip("大振幅時にPresentation T/2を延長してよい最大FixedUpdate数。")]
    [Range(2, 64)]
    [SerializeField]
    private int maximumPresentationHalfCycleFixedSteps = 24;

    // 球がUpper/Lowerの両面に同時接触しないため、surface-to-surface基準の
    // clearance scaleは最低2R必要。以前安定した4Rを上限としてAuto化する。
    private const float MinimumPeriodicRadiusClearanceScale = 2f;
    private const float MaximumPeriodicRadiusClearanceScale = 4f;
    private const int MinimumPeriodicHalfCycleFixedSteps = 2;

    // 以前実際に減衰振幅が成立した300-400m/s^2帯の中心値。
    // Inspectorパラメータにはせず、周期Tから高さS(T)を一意に決めるための
    // 内部基準だけに使う。実際のPhase Drive加速度はSphereCast距離から毎F解く。
    private const float PeriodicHeightReferencePhaseAcceleration = 350f;


    // ================================================================
    // maxGroundSpeed -> period-cycle experiment (READ ONLY)
    // ================================================================
    private const int MaxGroundSpeedDecayExperimentCycles = 8;
    private const float MaxGroundSpeedDecayExperimentEndRatio = 0.5f;

    private const float MinimumObservedPeriodCorrectionRatio = 0.95f;
    private const float MaximumObservedPeriodCorrectionRatio = 1.05f;

    [Tooltip(
        "実測周期をMaster Tへ弱く補正する割合。K版では補正幅を±5%以内に制限します。")]
    [Range(0f, 0.25f)]
    [SerializeField]
    private float periodObservationBlend = 0.15f;

    private static readonly FieldInfo MaxGroundSpeedField =
        typeof(SlopeStickCore).GetField(
            "maxGroundSpeed",
            BindingFlags.Instance |
            BindingFlags.NonPublic);


    // ================================================================
    // Runtime
    // ================================================================

    [Header("Runtime - Read Only")] [SerializeField]
    private bool armed;

    [SerializeField] private bool envelopeBuilt;

    [SerializeField] private float sourceEnergyJoule;

    [Tooltip("BallVisualSlopeDriveから渡されたStable-N基準高さ H0[m]。")]
    [SerializeField] private float canonicalReferenceHeight;

    [Tooltip("BallVisualSlopeDriveから渡されたEnergy source axis n0（Visual座標）。")]
    [SerializeField] private Vector3 sourceEnergyAxisVisual = Vector3.up;

    [Tooltip("現在のCanonical Oscillation Energy比 epsilon = E/E0。")]
    [SerializeField, Range(0f, 1f)] private float canonicalEnergyRatio = 1f;

    [SerializeField] private float canonicalNormalAcceleration;

    [SerializeField] private float equalizerLaunchSpeed;

    [SerializeField] private float equalizerVerticalLaunchSpeed;

    [SerializeField] private float entryApexHeight;

    [Header("Full Spline / Real-Time Decay Runtime - Read Only")]
    [SerializeField] private float capturedReleaseProgress01;
    [SerializeField] private float decayRatePerSecond;

    [Tooltip("Release -> Exact Limitの実時間コスト[s]。指数Envelopeのgamma診断に使用します。周期Tのスケジューラではありません。")]
    [SerializeField] private float decayTimeCostSeconds;

    [SerializeField] private float latestEnvelopeTravelTimeSeconds;
    [SerializeField] private float latestSlopeBaseLength;
    [SerializeField] private float latestColliderCurveLength;
    [SerializeField] private float latestCurveLengthRatio;

    [SerializeField] private float gamma;

    [SerializeField] private float minimumFreeAmplitude;

    [SerializeField] private Vector3 capturedEntryPhysics;

    [SerializeField] private Vector3 capturedLimitPhysics;

    [SerializeField] private float capturedTargetProgress01;

    [SerializeField] private Vector3 capturedEqualizerLaunchVelocityVisual;

    [Header("Equalizer Under-Spline Projection - Read Only")]
    [Tooltip("BallVisualEqualizer中心からStable-N負方向へ最も直下になるSpline進歩率。Knot番号は使いません。")]
    [SerializeField] private float equalizerUnderProgress01;

    [SerializeField] private Vector3 equalizerUnderSurfaceVisual;
    [SerializeField] private Vector3 equalizerUnderCenterVisual;
    [SerializeField] private Vector3 equalizerUnderTangentVisual = Vector3.forward;
    [SerializeField] private Vector3 equalizerUnderNormalVisual = Vector3.up;
    [SerializeField] private float equalizerUnderClearanceMeters;
    [SerializeField] private bool equalizerUnderProjectionValid;

    [Header("Arrival Terminal Gate - Read Only")]
    [SerializeField] private bool arrivalTerminalActive;
    [SerializeField] private float arrivalTerminalTimeToGo;
    [SerializeField] private float arrivalTerminalBlend01;


    private Transform generatedRoot;
    private Mesh generatedMesh;

    // 最新Envelopeだけを保持する。次の生成前に旧Rootは破棄する。
    private readonly List<Transform> generatedEnvelopeRoots =
        new List<Transform>();

    // 現在「最後に生成された」Envelopeだけをリアルタイム更新するための参照。
    private Transform generatedMeshTransform;
    private MeshFilter generatedMeshFilter;
    private MeshCollider generatedMeshCollider;

    // Rhythm-gated Upper:
    // Sensor is always active; Solid is enabled only inside the scheduler window.
    private MeshCollider generatedMeshSensorCollider;
    private bool upperEnvelopeSolidRequested;

    // Double-buffered geometry. Build a standby Mesh first, then swap only while
    // the active Upper Solid is disabled.
    private Mesh pendingUpperEnvelopeMesh;
    private bool pendingUpperEnvelopeMeshReady;
    private float nextPendingUpperEnvelopeBuildRetryTime;


    [Header("Periodic Contact Runtime - Read Only")]
    [Tooltip("FixedUpdate解像度を考慮した実際の1周期T[s]。")]
    [SerializeField]
    private float resolvedContactPeriodSeconds = 0.10f;

    [Tooltip("Upper/Lower反対側境界までの目標時間 T/2[s]。")]
    [SerializeField]
    private float resolvedHalfPeriodSeconds = 0.05f;

    [Tooltip("自動決定されたUpper基準R倍率 S(T)。")]
    [SerializeField]
    private float resolvedEnvelopeRadiusClearanceScale =
        MinimumPeriodicRadiusClearanceScale;

    [Tooltip("Release時点のLower中心 -> Upper中心の自動目標距離[m]。")]
    [SerializeField]
    private float resolvedReleaseCenterTravelDistance;

    [Tooltip("Release時点のLower surface -> Upper surfaceの自動高さ[m]。")]
    [SerializeField]
    private float resolvedReleaseSurfaceClearance;

    [Tooltip("ReleaseからT/2でUpperへ届くための理論Stable-N初速[m/s]。")]
    [SerializeField]
    private float resolvedReleaseTargetNormalSpeed;

    [Tooltip("Canonical初速からPreferred Tへ合わせるための初期Phase加速度[m/s^2]。")]
    [SerializeField]
    private float resolvedReleasePhaseAcceleration;

    [Header("Presentation Physical Timing Runtime - Read Only")]
    [SerializeField] private bool presentationTimingExpanded;
    [SerializeField] private int presentationResolvedHalfCycleFixedSteps;
    [SerializeField] private float presentationRequiredPhaseAcceleration;
    [SerializeField] private float presentationPhaseAccelerationBudget;

    [Header("Presentation Ceiling Runtime - Read Only")]
    [SerializeField] private int presentationWaveIndex;

    [Header("Spatial Wave Authority - Runtime Read Only")]
    [SerializeField] private bool spatialWaveAuthorityActive;
    [SerializeField] private int spatialWaveAuthorityCount = 3;
    [SerializeField, Range(0f, 1f)] private float spatialWaveAuthorityProgress01;
    [SerializeField] private float presentationReleaseFixedTime;
    [SerializeField] private float presentationCurrentCeilingR;
    [SerializeField] private float presentationCurrentLegacyTravelR;
    [SerializeField] private float presentationCurrentResolvedTravelR;
    [SerializeField] private float presentationCurrentLossHnR;
    [SerializeField, Range(0f, 1f)] private float presentationCurrentLegacyEnergyRetention01 = 1f;
    [SerializeField, Range(0f, 1f)] private float presentationCurrentLegacyTimeRetention01 = 1f;
    [SerializeField, Range(0f, 1f)] private float presentationCurrentRawLegacyRetention01 = 1f;
    [SerializeField, Range(0f, 1f)] private float presentationCurrentAppliedRetention01 = 1f;


    [Header("maxGroundSpeed Adaptive Runtime - Read Only")]
    [SerializeField] private int maxGroundSpeedExperimentCycleIndex;
    [SerializeField] private bool maxGroundSpeedReadAvailable;
    [SerializeField] private float sourceMaxGroundSpeedReadOnly;
    [SerializeField] private float plannedMaxGroundSpeedForCycle;
    [SerializeField] private float plannedMaxGroundSpeedRatio = 1f;
    [SerializeField] private float baseExperimentPeriodSeconds;
    [SerializeField] private float nominalExperimentPeriodSeconds;
    [SerializeField] private float observedGeometryPeriodSeconds;
    [SerializeField] private float observedGeometryPeriodCorrectionRatio = 1f;
    [SerializeField] private bool observedGeometryPeriodValid;
    [Header("Upper Rhythm Gate Runtime - Read Only")]
    [SerializeField] private bool upperEnvelopeSolidEnabled;
    [SerializeField] private bool upperEnvelopeSensorEnabled;
    [SerializeField] private bool upperEnvelopeSensorOccupied;
    [SerializeField] private int upperEnvelopeMeshSwapCount;

    // D: resolved by BallVisualEqualizerSync. Hidden because this is a derived
    // safety profile, not a user-facing tuning surface.
    [SerializeField, HideInInspector, Range(0f, 1f)]
    private float hybridWaveFeasibilityAmplitudeScale01 = 1f;
    [SerializeField, HideInInspector]
    private float hybridWaveFeasibilityHalfPeriodSeconds;
    [SerializeField, HideInInspector]
    private bool hybridWaveFeasibilityAmplitudeLimited;

    // Canonical energy may change inside a collision callback. MeshCollider recook
    // is deferred and double-buffered; the active Solid is never recooked while ON.
    private bool pendingCanonicalGeometryRebuild;
    // 最新Envelopeを作った時点の固定幾何。
    // Inspector調整ではSlopeStickCoreから取り直さず、この区間だけを再生成する。
    private bool latestEnvelopeGeometryCached;
    private Vector3 cachedAxisPhysics;
    private Vector3 cachedSlopeNormalPhysics;
    private float cachedA0;
    private float cachedGamma;
    private float cachedEqualizerRadius;

    // Inspector変更検出用。物理式ではなくMesh近似だけを監視する。
    private bool liveSettingsSnapshotValid;
    private int lastSegmentCount;
    private float lastEnvelopeWidth;
    private float lastPreferredContactPeriodSeconds;
    private float lastMinimumPresentationCeilingR;
    private float lastWaveTimeDecayInfluence;
    private int lastPresentationCeilingCurveHash;


    // ================================================================
    // Unity
    // ================================================================

    private void Awake()
    {
        Debug.Log("[ENVELOPE BUILD] HybridWave-20260830-StairLower-A", this);
        ResolveReferences();
        CaptureLiveSettingsSnapshot();
    }
    private void FixedUpdate()
    {
        TryBuildEnvelopeIfReady();

        // Geometry changes are prepared/committed outside contact callbacks.
        if (pendingCanonicalGeometryRebuild &&
            envelopeBuilt &&
            latestEnvelopeGeometryCached)
        {
            PreparePendingUpperEnvelopeMesh();
        }

        TryCommitPendingUpperEnvelopeMesh();
    }


    private void Update()
    {
        // ------------------------------------------------------------
        // Play中のInspector変更を毎描画フレーム監視する。
        //
        // 重要:
        // BuildNegativeEnvelope()をやり直すのではなく、
        // 「最後に生成されたEnvelope」の固定幾何を使って
        // Meshアセットだけを差し替える。
        // ------------------------------------------------------------

        if (!Application.isPlaying)
            return;

        if (!envelopeBuilt ||
            !latestEnvelopeGeometryCached ||
            !generatedMeshTransform ||
            !generatedMeshFilter ||
            !generatedMeshCollider)
        {
            CaptureLiveSettingsSnapshot();
            return;
        }

        if (!LiveSettingsChanged())
            return;

        // Defer recook to FixedUpdate. The active Solid is never modified here.
        pendingCanonicalGeometryRebuild = true;
    }



    // ================================================================
    // Arm - compatibility overloads
    // ================================================================

    /// <summary>
    /// 旧2引数API。新規コードでは4引数のCanonical APIを使用してください。
    /// </summary>
    public void ArmFromBallVisualEnergy(
        float energyJoule,
        Vector3 equalizerLaunchDirectionVisual)
    {
        ResolveReferences();

        if (!ballVisualEqualizer)
            return;

        Vector3 direction =
            equalizerLaunchDirectionVisual;

        if (direction.sqrMagnitude <=
            0.000001f)
        {
            return;
        }

        direction.Normalize();

        float mass =
            Mathf.Max(
                0.0001f,
                ballVisualEqualizer.mass);

        float safeEnergy =
            Mathf.Max(
                0f,
                energyJoule);

        if (safeEnergy <=
            0.000001f)
        {
            return;
        }

        float speed =
            Mathf.Sqrt(
                2f *
                safeEnergy /
                mass);

        Vector3 launchVelocity =
            direction *
            speed;

        float gravity =
            Mathf.Max(
                0.0001f,
                Mathf.Abs(
                    Physics.gravity.y));

        float verticalSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    launchVelocity,
                    Vector3.up));

        float legacyReferenceHeight =
            Mathf.Max(
                0.01f,
                verticalSpeed *
                verticalSpeed /
                (2f * gravity));

        ArmFromBallVisualEnergy(
            safeEnergy,
            legacyReferenceHeight,
            launchVelocity,
            direction);
    }


    /// <summary>
    /// 旧Equalizer初速API。新規コードでは4引数Canonical APIを使用してください。
    /// </summary>
    public void ArmFromEqualizerLaunchVelocity(
        Vector3 launchVelocityVisual)
    {
        ResolveReferences();

        float mass =
            ballVisualEqualizer
                ? Mathf.Max(
                    0.0001f,
                    ballVisualEqualizer.mass)
                : 1f;

        float energy =
            0.5f *
            mass *
            launchVelocityVisual.sqrMagnitude;

        Vector3 axis =
            launchVelocityVisual.sqrMagnitude >
            0.000001f
                ? launchVelocityVisual.normalized
                : Vector3.up;

        float gravity =
            Mathf.Max(
                0.0001f,
                Mathf.Abs(
                    Physics.gravity.y));

        float verticalSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    launchVelocityVisual,
                    Vector3.up));

        float legacyReferenceHeight =
            Mathf.Max(
                0.01f,
                verticalSpeed *
                verticalSpeed /
                (2f * gravity));

        ArmFromBallVisualEnergy(
            energy,
            legacyReferenceHeight,
            launchVelocityVisual,
            axis);
    }


    // ================================================================
    // Build
    // ================================================================

    private void BuildNegativeEnvelope()
    {
        if (!ReferencesValid())
            return;

        if (!slopeCore.TryGetBallVisualTargetProgressCenterPhysics(
                out Vector3 limitCenterPhysics))
        {
            return;
        }

        capturedReleaseProgress01 =
            Mathf.Clamp01(
                slopeCore.BallVisualSlopeProgress01);

        capturedTargetProgress01 =
            Mathf.Clamp01(
                slopeCore.BallVisualSlopeProgress01 -
                slopeCore.slopeProgressErrorPercent *
                0.01f);

        if (!slopeCore.TryEvaluateBallVisualSectionFramePhysics(
                0f,
                out Vector3 sectionEntryCenterPhysics,
                out _,
                out _) ||
            !slopeCore.TryEvaluateBallVisualSectionFramePhysics(
                1f,
                out Vector3 sectionEndCenterPhysics,
                out _,
                out _) ||
            !slopeCore.TryEvaluateBallVisualSectionFramePhysics(
                capturedReleaseProgress01,
                out _,
                out Vector3 releaseTangentPhysics,
                out Vector3 releaseNormalPhysics) ||
            !slopeCore.TryEvaluateBallVisualSectionFramePhysics(
                capturedTargetProgress01,
                out _,
                out _,
                out _))
        {
            Debug.LogWarning(
                "[ENVELOPE SPLINE] Full section sampling is not ready.",
                this);
            return;
        }

        Vector3 sourceNormalPhysics =
            correspondSubject.InverseMapDirection(
                sourceEnergyAxisVisual);

        if (sourceNormalPhysics.sqrMagnitude <= 0.000001f)
            sourceNormalPhysics = releaseNormalPhysics;

        sourceNormalPhysics.Normalize();

        releaseNormalPhysics =
            Vector3.ProjectOnPlane(
                releaseNormalPhysics,
                releaseTangentPhysics);

        if (releaseNormalPhysics.sqrMagnitude <= 0.000001f)
            releaseNormalPhysics = sourceNormalPhysics;

        releaseNormalPhysics.Normalize();

        if (Vector3.Dot(
                releaseNormalPhysics,
                sourceNormalPhysics) < 0f)
        {
            releaseNormalPhysics = -releaseNormalPhysics;
        }

        releaseTangentPhysics =
            Vector3.ProjectOnPlane(
                releaseTangentPhysics,
                releaseNormalPhysics);

        if (releaseTangentPhysics.sqrMagnitude <= 0.000001f)
            return;

        releaseTangentPhysics.Normalize();

        capturedEntryPhysics =
            sectionEntryCenterPhysics;

        capturedLimitPhysics =
            limitCenterPhysics;

        float equalizerRadius =
            ResolveEqualizerWorldRadius();

        float subjectRadius =
            ResolveSlopeCoreWorldRadius();

        if (!TryEvaluateSplineSurfacePhysics(
                capturedReleaseProgress01,
                subjectRadius,
                out _,
                out Vector3 releaseSurfacePhysics,
                out _,
                out Vector3 stableReleaseNormalPhysics) ||
            !TryEvaluateSplineSurfacePhysics(
                capturedTargetProgress01,
                subjectRadius,
                out _,
                out Vector3 limitSurfacePhysics,
                out _,
                out _))
        {
            return;
        }

        float contactOffset =
            ballVisualEqualizerCollider
                ? Mathf.Max(
                    ballVisualEqualizerCollider.contactOffset,
                    Physics.defaultContactOffset)
                : Physics.defaultContactOffset;

        minimumFreeAmplitude =
            Mathf.Max(
                0.001f,
                contactOffset * 2f);

        // Canonical contract:
        // H0 is already the Stable-N free amplitude at Release.
        float A0 =
            Mathf.Max(
                minimumFreeAmplitude,
                canonicalReferenceHeight);
        float releaseToLimitDistance =
            EstimateSplineArcDistancePhysics(
                capturedReleaseProgress01,
                capturedTargetProgress01,
                32);

        if (releaseToLimitDistance <= 0.0001f)
        {
            releaseToLimitDistance =
                Vector3.Distance(
                    releaseSurfacePhysics,
                    limitSurfacePhysics);
        }

        float timeToLimit =
            ResolveTravelTimeForArcDistance(
                releaseToLimitDistance,
                releaseTangentPhysics);

        decayTimeCostSeconds =
            Mathf.Max(
                0f,
                timeToLimit);

        float AL =
            Mathf.Min(
                A0,
                minimumFreeAmplitude);

        decayRatePerSecond =
            A0 > AL + 0.000001f &&
            timeToLimit > 0.0001f
                ? Mathf.Log(A0 / AL) /
                  timeToLimit
                : 0f;

        // Backward diagnostic alias.  In this integrated version gamma is 1/s.
        gamma = decayRatePerSecond;

        // 周期Tを唯一の時間スケールとして、Release時の高さ・速度を
        // 同じ幾何式から解決する。R倍率は [2R, 4R] の範囲で自動決定。
        ResolvePeriodicContactPlan(
            A0,
            equalizerRadius);

        // Existing cached fields remain the compatibility cache.
        // cachedEntry = section entry, cachedLimit = section end.
        if (!TryEvaluateSplineSurfacePhysics(
                0f,
                subjectRadius,
                out _,
                out Vector3 entrySurfacePhysics,
                out _,
                out _) ||
            !TryEvaluateSplineSurfacePhysics(
                1f,
                subjectRadius,
                out _,
                out Vector3 slopeEndSurfacePhysics,
                out _,
                out _))
        {
            return;
        }

        CacheLatestEnvelopeGeometry(
            entrySurfacePhysics,
            slopeEndSurfacePhysics,
            releaseTangentPhysics,
            stableReleaseNormalPhysics,
            A0,
            decayRatePerSecond,
            equalizerRadius);

        CreateRoot();

        envelopeBuilt =
            CreateFullSplineEnvelopeMesh(
                A0,
                decayRatePerSecond,
                equalizerRadius);

        if (envelopeBuilt)
        {
            Debug.Log(
                $"[ENVELOPE FULL SPLINE DOMAIN] " +
                $"releaseP={capturedReleaseProgress01 * 100f:F2}% " +
                $"limitP={capturedTargetProgress01 * 100f:F2}% " +
                $"baseLength={latestSlopeBaseLength:F4}m " +
                $"curveLength={latestColliderCurveLength:F4}m " +
                $"curveRatio={latestCurveLengthRatio:F6} " +
                $"timeToLimit={timeToLimit:F4}s " +
                $"k={decayRatePerSecond:F4}/s " +
                $"preferredT={preferredContactPeriodSeconds:F4}s " +
                $"resolvedT={resolvedContactPeriodSeconds:F4}s " +
                $"S={resolvedEnvelopeRadiusClearanceScale:F4}R " +
                $"wave={presentationWaveIndex + 1} " +
                $"ceiling={presentationCurrentCeilingR:F3}R " +
                $"legacy={presentationCurrentLegacyTravelR:F3}R " +
                $"resolved={presentationCurrentResolvedTravelR:F3}R " +
                $"Hn={presentationCurrentLossHnR:F3}R " +
                $"rawRet={presentationCurrentRawLegacyRetention01:F4} " +
                $"appliedRet={presentationCurrentAppliedRetention01:F4} " +
                $"lossInfluence={waveTimeDecayInfluence:F3} " +
                $"releaseSpan={resolvedReleaseCenterTravelDistance:F4}m " +
                $"targetVN={resolvedReleaseTargetNormalSpeed:F4}m/s " +
                $"phaseA0={resolvedReleasePhaseAcceleration:F4}m/s2 " +
                $"timingExpanded={presentationTimingExpanded} " +
                $"halfSteps={presentationResolvedHalfCycleFixedSteps} " +
                $"requiredPhaseA={presentationRequiredPhaseAcceleration:F4}m/s2 " +
                $"phaseBudget={presentationPhaseAccelerationBudget:F4}m/s2 " +
                $"E0={sourceEnergyJoule:F4}J " +
                $"H0={canonicalReferenceHeight:F4}m " +
                $"epsilon={canonicalEnergyRatio:F4} " +
                $"adaptiveMaxGround={sourceMaxGroundSpeedReadOnly:F3}m/s " +
                $"periodScaleFromMaxGround=1.0000 " +
                $"expCycle={maxGroundSpeedExperimentCycleIndex} " +
                $"maxGround0={sourceMaxGroundSpeedReadOnly:F3}m/s " +
                $"plannedMaxGround={plannedMaxGroundSpeedForCycle:F3}m/s " +
                $"adaptiveMode=Common",
                this);
        }
    }


    // ================================================================
    // First contact prediction / selection
    // ================================================================
    // ================================================================
    // Envelope mesh
    // ================================================================
    private void CacheLatestEnvelopeGeometry(
        Vector3 entrySurfacePhysics,
        Vector3 limitSurfacePhysics,
        Vector3 axisPhysics,
        Vector3 slopeNormalPhysics,
        float A0,
        float gammaValue,
        float equalizerRadius)
    {
        cachedAxisPhysics =
            axisPhysics;

        cachedSlopeNormalPhysics =
            slopeNormalPhysics;

        cachedA0 =
            A0;

        cachedGamma =
            gammaValue;

        cachedEqualizerRadius =
            equalizerRadius;

        latestEnvelopeGeometryCached =
            true;
    }
    private int ComputePresentationCeilingCurveHash()
    {
        unchecked
        {
            int hash = 17;

            if (presentationCeilingR == null)
                return hash;

            Keyframe[] keys = presentationCeilingR.keys;
            hash = hash * 31 + keys.Length;

            for (int i = 0; i < keys.Length; i++)
            {
                hash = hash * 31 + keys[i].time.GetHashCode();
                hash = hash * 31 + keys[i].value.GetHashCode();
                hash = hash * 31 + keys[i].inTangent.GetHashCode();
                hash = hash * 31 + keys[i].outTangent.GetHashCode();
            }

            return hash;
        }
    }

    private bool LiveSettingsChanged()
    {
        if (!liveSettingsSnapshotValid)
            return true;

        return
            segmentCount != lastSegmentCount ||
            Mathf.Abs(envelopeWidth - lastEnvelopeWidth) > 0.00001f ||
            Mathf.Abs(
                preferredContactPeriodSeconds -
                lastPreferredContactPeriodSeconds) > 0.00001f ||
            Mathf.Abs(
                minimumPresentationCeilingR -
                lastMinimumPresentationCeilingR) > 0.00001f ||
            Mathf.Abs(
                waveTimeDecayInfluence -
                lastWaveTimeDecayInfluence) > 0.00001f ||
            ComputePresentationCeilingCurveHash() !=
                lastPresentationCeilingCurveHash;
    }
    private void CaptureLiveSettingsSnapshot()
    {
        lastSegmentCount = segmentCount;
        lastEnvelopeWidth = envelopeWidth;
        lastPreferredContactPeriodSeconds =
            preferredContactPeriodSeconds;
        lastMinimumPresentationCeilingR =
            minimumPresentationCeilingR;
        lastWaveTimeDecayInfluence =
            waveTimeDecayInfluence;
        lastPresentationCeilingCurveHash =
            ComputePresentationCeilingCurveHash();
        liveSettingsSnapshotValid = true;
    }
    private void RebuildLatestGeneratedMesh()
    {
        // Compatibility entry point. Actual MeshCollider recook is deferred.
        pendingCanonicalGeometryRebuild = true;
    }


    private void PreparePendingUpperEnvelopeMesh()
    {
        if (!pendingCanonicalGeometryRebuild ||
            pendingUpperEnvelopeMeshReady ||
            Time.fixedTime < nextPendingUpperEnvelopeBuildRetryTime ||
            !latestEnvelopeGeometryCached ||
            !generatedMeshTransform ||
            !generatedMeshFilter ||
            !generatedMeshCollider)
        {
            return;
        }

        ResolvePeriodicContactPlan(
            cachedA0,
            cachedEqualizerRadius);

        Mesh newMesh =
            BuildFullSplineEnvelopeMeshAsset(
                generatedMeshTransform,
                cachedA0,
                cachedGamma,
                cachedEqualizerRadius);

        if (!newMesh)
        {
            nextPendingUpperEnvelopeBuildRetryTime =
                Time.fixedTime + 0.10f;

            Debug.LogWarning(
                "[ENVELOPE STANDBY BUILD] Full Spline mesh build failed; active mesh preserved. Retry in 0.10s.",
                this);
            return;
        }

        if (pendingUpperEnvelopeMesh &&
            pendingUpperEnvelopeMesh != newMesh)
        {
            Destroy(pendingUpperEnvelopeMesh);
        }

        pendingUpperEnvelopeMesh = newMesh;
        pendingUpperEnvelopeMeshReady = true;
        pendingCanonicalGeometryRebuild = false;
        nextPendingUpperEnvelopeBuildRetryTime = 0f;
    }


    private void TryCommitPendingUpperEnvelopeMesh()
    {
        if (!pendingUpperEnvelopeMeshReady ||
            !pendingUpperEnvelopeMesh ||
            !generatedMeshFilter ||
            !generatedMeshCollider)
        {
            return;
        }

        // Mesh recook is deferred to FixedUpdate. Temporarily disable the Upper,
        // swap the cooked Mesh, then restore the requested solid state.
        bool restoreSolid =
            upperEnvelopeSolidRequested;

        Mesh oldMesh =
            generatedMesh;

        generatedMeshCollider.enabled =
            false;

        generatedMeshFilter.sharedMesh =
            pendingUpperEnvelopeMesh;

        generatedMeshCollider.sharedMesh =
            null;

        generatedMeshCollider.sharedMesh =
            pendingUpperEnvelopeMesh;

        generatedMesh =
            pendingUpperEnvelopeMesh;

        pendingUpperEnvelopeMesh =
            null;

        pendingUpperEnvelopeMeshReady =
            false;

        generatedMeshCollider.enabled =
            restoreSolid;

        upperEnvelopeSolidEnabled =
            generatedMeshCollider.enabled;

        if (oldMesh &&
            oldMesh != generatedMesh)
        {
            Destroy(oldMesh);
        }

        Physics.SyncTransforms();
        CaptureLiveSettingsSnapshot();

        UpdatePresentationRuntimeDiagnostics(
            Mathf.Max(
                0f,
                Time.fixedTime -
                presentationReleaseFixedTime));

        upperEnvelopeMeshSwapCount++;

        Debug.Log(
            $"[ENVELOPE FLOATING SWAP] " +
            $"swapCount={upperEnvelopeMeshSwapCount} " +
            $"wave={presentationWaveIndex + 1} " +
            $"ceiling={presentationCurrentCeilingR:F3}R " +
            $"resolved={presentationCurrentResolvedTravelR:F3}R " +
            $"Hn={presentationCurrentLossHnR:F3}R " +
            $"solidEnabled={generatedMeshCollider.enabled}",
            this);
    }



private bool TryEvaluateSplineSurfacePhysics(
        float progress01,
        float subjectRadius,
        out Vector3 centerPhysics,
        out Vector3 surfacePhysics,
        out Vector3 tangentPhysics,
        out Vector3 normalPhysics)
    {
        centerPhysics = Vector3.zero;
        surfacePhysics = Vector3.zero;
        tangentPhysics = Vector3.forward;
        normalPhysics = Vector3.up;

        if (!slopeCore ||
            !slopeCore.TryEvaluateBallVisualSectionFramePhysics(
                Mathf.Clamp01(progress01),
                out centerPhysics,
                out tangentPhysics,
                out normalPhysics))
        {
            return false;
        }

        Vector3 sourceAxisPhysics =
            correspondSubject
                ? correspondSubject.InverseMapDirection(
                    sourceEnergyAxisVisual)
                : Vector3.up;

        tangentPhysics.Normalize();

        normalPhysics =
            Vector3.ProjectOnPlane(
                normalPhysics,
                tangentPhysics);

        if (normalPhysics.sqrMagnitude <= 0.000001f)
            return false;

        normalPhysics.Normalize();

        if (sourceAxisPhysics.sqrMagnitude > 0.000001f &&
            Vector3.Dot(normalPhysics, sourceAxisPhysics) < 0f)
        {
            normalPhysics = -normalPhysics;
        }

        tangentPhysics =
            Vector3.ProjectOnPlane(
                tangentPhysics,
                normalPhysics).normalized;

        surfacePhysics =
            centerPhysics -
            normalPhysics *
            Mathf.Max(0f, subjectRadius);

        return true;
    }


    private float EstimateSplineArcDistancePhysics(
        float progressA,
        float progressB,
        int samples)
    {
        float a = Mathf.Clamp01(progressA);
        float b = Mathf.Clamp01(progressB);

        if (Mathf.Abs(b - a) <= 0.000001f)
            return 0f;

        int count = Mathf.Max(2, samples);
        float distance = 0f;
        bool hasPrevious = false;
        Vector3 previous = Vector3.zero;

        for (int i = 0; i <= count; i++)
        {
            float p =
                Mathf.Lerp(
                    a,
                    b,
                    i / (float)count);

            if (!slopeCore.TryEvaluateBallVisualSectionFramePhysics(
                    p,
                    out Vector3 center,
                    out _,
                    out _))
            {
                continue;
            }

            if (hasPrevious)
                distance += Vector3.Distance(previous, center);

            previous = center;
            hasPrevious = true;
        }

        return distance;
    }


    private float ResolveTravelTimeForArcDistance(
        float distance,
        Vector3 tangentPhysics)
    {
        float safeDistance = Mathf.Max(0f, distance);
        if (safeDistance <= 0.000001f)
            return 0f;

        Vector3 tangentVisual =
            correspondSubject
                ? correspondSubject.MapDirection(tangentPhysics)
                : tangentPhysics;

        if (tangentVisual.sqrMagnitude <= 0.000001f)
            return 0f;

        tangentVisual.Normalize();

        float initialTangentSpeed =
            Vector3.Dot(
                capturedEqualizerLaunchVelocityVisual,
                tangentVisual);

        if (initialTangentSpeed <= 0.0001f)
        {
            initialTangentSpeed =
                Mathf.Max(
                    0.01f,
                    capturedEqualizerLaunchVelocityVisual.magnitude);
        }

        float tangentAcceleration =
            Vector3.Dot(
                Physics.gravity,
                tangentVisual);

        if (Mathf.Abs(tangentAcceleration) <= 0.0001f)
            return safeDistance / initialTangentSpeed;

        float discriminant =
            initialTangentSpeed * initialTangentSpeed +
            2f * tangentAcceleration * safeDistance;

        if (discriminant < 0f)
            return safeDistance / initialTangentSpeed;

        float sqrtD = Mathf.Sqrt(discriminant);
        float t1 = (-initialTangentSpeed + sqrtD) / tangentAcceleration;
        float t2 = (-initialTangentSpeed - sqrtD) / tangentAcceleration;
        float best = float.PositiveInfinity;

        if (t1 >= 0f) best = t1;
        if (t2 >= 0f && t2 < best) best = t2;

        return float.IsInfinity(best) || float.IsNaN(best)
            ? safeDistance / initialTangentSpeed
            : best;
    }
    private float EvaluateEnvelopeAmplitudeAtTime(
        float A0,
        float decayRatePerSecondValue,
        float timeSeconds)
    {
        // Legacy O-model. Do not replace this with the presentation curve.
        // This is the physical/decay side that produces H_n.
        float epsilon = Mathf.Clamp01(canonicalEnergyRatio);

        float amplitude =
            Mathf.Max(0f, A0) *
            epsilon *
            Mathf.Exp(
                -Mathf.Max(0f, decayRatePerSecondValue) *
                Mathf.Max(0f, timeSeconds));

        return Mathf.Max(minimumFreeAmplitude, amplitude);
    }


    private float EvaluatePresentationCeilingR(
        float waveCoordinate)
    {
        float safeMinimum =
            Mathf.Max(0.01f, minimumPresentationCeilingR);

        if (presentationCeilingR == null ||
            presentationCeilingR.length == 0)
        {
            return safeMinimum;
        }

        return
            Mathf.Max(
                safeMinimum,
                presentationCeilingR.Evaluate(
                    Mathf.Max(0f, waveCoordinate)));
    }


    private float EvaluatePresentationWaveCoordinate(
        float timeSeconds)
    {
        float safePeriod =
            Mathf.Max(0.0001f, resolvedContactPeriodSeconds);

        // Actual canonical Upper peaks advance presentationWaveIndex.
        // The predicted time coordinate is used for future Spline samples.
        // max() prevents a late physical turnpoint from moving the presentation
        // envelope backwards after a real wave has already completed.
        float predictedWaveCoordinate =
            Mathf.Max(
                0f,
                Mathf.Max(0f, timeSeconds) / safePeriod -
                0.5f);

        // Upper peaks occur at T/2, 3T/2, 5T/2... .
        // Therefore the first visible peak maps to curve X=0, the second to X=1.
        return
            Mathf.Max(
                presentationWaveIndex,
                predictedWaveCoordinate);
    }


    private float EvaluateLegacyCenterTravelAtTime(
        float A0,
        float decayRatePerSecondValue,
        float timeSeconds,
        float equalizerRadius)
    {
        float radius =
            Mathf.Max(0.0001f, equalizerRadius);

        float legacyAmplitude =
            EvaluateEnvelopeAmplitudeAtTime(
                A0,
                decayRatePerSecondValue,
                timeSeconds);

        // Original O geometry:
        // Upper offset = S(T)R + A(t)
        // Center free travel = Upper offset - 2R
        //                    = (S(T)-2)R + A(t).
        return
            Mathf.Max(
                minimumFreeAmplitude,
                (resolvedEnvelopeRadiusClearanceScale -
                 MinimumPeriodicRadiusClearanceScale) *
                    radius +
                legacyAmplitude);
    }


    private float EvaluatePresentationCenterTravelAtTime(
        float A0,
        float decayRatePerSecondValue,
        float timeSeconds,
        float equalizerRadius,
        out float ceilingR,
        out float legacyTravelR,
        out float lossHnR,
        out float legacyEnergyRetention01,
        out float legacyTimeRetention01,
        out float rawLegacyRetention01,
        out float appliedRetention01)
    {
        float radius =
            Mathf.Max(0.0001f, equalizerRadius);

        float safeTime =
            Mathf.Max(0f, timeSeconds);

        float waveCoordinate =
            EvaluatePresentationWaveCoordinate(
                safeTime);

        ceilingR =
            EvaluatePresentationCeilingR(
                waveCoordinate);

        // Keep the original O/Q absolute geometry as a diagnostic only.
        // It is intentionally NOT used as the resolved presentation height.
        // Q used min(C_n, D_legacy) and the log showed 3.8R ceilings collapsing
        // to roughly 0.1R-0.9R.  R instead extracts only the old decay SHAPE.
        float legacyMeters =
            EvaluateLegacyCenterTravelAtTime(
                A0,
                decayRatePerSecondValue,
                safeTime,
                radius);

        legacyTravelR =
            legacyMeters / radius;

        // Old Envelope:
        //     A_old(t) = H0 * epsilon * exp(-gamma*t)
        // Therefore its dimensionless retention is exactly
        //     q_old = epsilon * exp(-gamma*t).
        // H0 cancels, so a small handoff height cannot collapse a 3-4R
        // presentation ceiling merely because the old absolute geometry was small.
        legacyEnergyRetention01 =
            Mathf.Clamp01(canonicalEnergyRatio);

        legacyTimeRetention01 =
            Mathf.Clamp01(
                Mathf.Exp(
                    -Mathf.Max(0f, decayRatePerSecondValue) *
                    safeTime));

        rawLegacyRetention01 =
            Mathf.Clamp01(
                legacyEnergyRetention01 *
                legacyTimeRetention01);

        float influence =
            Mathf.Clamp01(waveTimeDecayInfluence);

        // Apply only a tunable fraction of the old loss:
        //     q_applied = 1 - influence * (1 - q_old)
        //               = lerp(1, q_old, influence).
        //
        // Example: q_old=0.20, influence=0.35 -> q_applied=0.72.
        // A 3.8R ceiling then resolves to 2.736R instead of 0.76R.
        appliedRetention01 =
            Mathf.Clamp01(
                Mathf.Lerp(
                    1f,
                    rawLegacyRetention01,
                    influence));

        float resolvedR =
            Mathf.Max(
                minimumFreeAmplitude / radius,
                ceilingR * appliedRetention01);

        // Never exceed the human ceiling even when the numerical minimum floor
        // would otherwise be slightly larger.
        resolvedR =
            Mathf.Min(
                ceilingR,
                resolvedR);

        // Floating Rigidbody版では4R-Hnが振幅Authorityです。
        // 周期/実現可能性の都合で振幅を縮めません。Rigidbody側がSpring/Damperと
        // 実衝突によって自然周期を形成します。
        resolvedR = Mathf.Min(ceilingR, resolvedR);

        lossHnR =
            Mathf.Max(
                0f,
                ceilingR - resolvedR);

        return
            resolvedR * radius;
    }


    private void UpdatePresentationRuntimeDiagnostics(
        float elapsedTimeSeconds)
    {
        if (cachedEqualizerRadius <= 0.0001f ||
            cachedA0 <= 0f)
        {
            return;
        }

        float resolvedMeters =
            EvaluatePresentationCenterTravelAtTime(
                cachedA0,
                cachedGamma,
                Mathf.Max(0f, elapsedTimeSeconds),
                cachedEqualizerRadius,
                out presentationCurrentCeilingR,
                out presentationCurrentLegacyTravelR,
                out presentationCurrentLossHnR,
                out presentationCurrentLegacyEnergyRetention01,
                out presentationCurrentLegacyTimeRetention01,
                out presentationCurrentRawLegacyRetention01,
                out presentationCurrentAppliedRetention01);

        presentationCurrentResolvedTravelR =
            resolvedMeters /
            Mathf.Max(0.0001f, cachedEqualizerRadius);
    }
    private float ResolvePresentationHalfPeriodForSpan(
        float spanMeters,
        float canonicalLaunchNormalSpeed,
        float canonicalNormalAccelerationValue,
        float preferredHalfPeriod,
        float fixedDt,
        out int resolvedSteps,
        out float requiredPhaseAcceleration,
        out bool expanded)
    {
        float safeDt = Mathf.Max(0.0001f, fixedDt);
        float preferredHalf =
            Mathf.Max(
                safeDt * MinimumPeriodicHalfCycleFixedSteps,
                preferredHalfPeriod);

        int preferredSteps =
            Mathf.Max(
                MinimumPeriodicHalfCycleFixedSteps,
                Mathf.CeilToInt(
                    preferredHalf / safeDt - 0.00001f));

        int maximumSteps =
            Mathf.Max(
                preferredSteps,
                Mathf.Clamp(
                    maximumPresentationHalfCycleFixedSteps,
                    MinimumPeriodicHalfCycleFixedSteps,
                    64));

        float phaseBudget =
            Mathf.Max(
                1f,
                maximumPresentationPhaseAcceleration);

        presentationPhaseAccelerationBudget =
            phaseBudget;

        resolvedSteps = preferredSteps;
        requiredPhaseAcceleration = 0f;
        expanded = false;

        float safeSpan =
            Mathf.Max(
                minimumFreeAmplitude,
                spanMeters);

        for (int steps = preferredSteps;
             steps <= maximumSteps;
             steps++)
        {
            float h = steps * safeDt;

            float requiredTotalNormalAcceleration =
                2f *
                (safeSpan -
                 canonicalLaunchNormalSpeed * h) /
                (h * h);

            float phaseAcceleration =
                requiredTotalNormalAcceleration +
                Mathf.Max(
                    0f,
                    canonicalNormalAccelerationValue);

            resolvedSteps = steps;
            requiredPhaseAcceleration = phaseAcceleration;

            if (Mathf.Abs(phaseAcceleration) <=
                phaseBudget + 0.0001f)
            {
                expanded =
                    steps > preferredSteps;

                return h;
            }
        }

        expanded =
            resolvedSteps > preferredSteps;

        return
            resolvedSteps * safeDt;
    }


    private void ResolvePeriodicContactPlan(
        float A0,
        float equalizerRadius)
    {
        float fixedDt =
            Mathf.Max(
                0.0001f,
                Time.fixedDeltaTime);

        float fallbackPeriod =
            Mathf.Max(
                fixedDt * MinimumPeriodicHalfCycleFixedSteps * 2f,
                preferredContactPeriodSeconds);

        baseExperimentPeriodSeconds =
            fallbackPeriod;

        nominalExperimentPeriodSeconds =
            fallbackPeriod;

        if (TryReadSlopeCoreMaxGroundSpeed(
                out float currentMaxGroundSpeed))
        {
            plannedMaxGroundSpeedForCycle =
                currentMaxGroundSpeed;
        }
        else
        {
            plannedMaxGroundSpeedForCycle = 0f;
        }

        plannedMaxGroundSpeedRatio = 1f;
        maxGroundSpeedExperimentCycleIndex = 0;

        // Natural oscillator T is measured by Sync from real Upper->Upper contacts.
        // Before the first measured cycle, keep the old Preferred T only as a
        // compatibility/diagnostic fallback.
        resolvedContactPeriodSeconds =
            observedGeometryPeriodValid &&
            observedGeometryPeriodSeconds > 0.0001f
                ? observedGeometryPeriodSeconds
                : fallbackPeriod;

        resolvedHalfPeriodSeconds =
            resolvedContactPeriodSeconds * 0.5f;

        float radius =
            Mathf.Max(
                0.0001f,
                equalizerRadius);

        resolvedReleaseCenterTravelDistance =
            EvaluatePresentationCenterTravelAtTime(
                A0,
                decayRatePerSecond,
                0f,
                radius,
                out float releaseCeilingR,
                out float releaseLegacyR,
                out float releaseLossHnR,
                out float releaseEnergyRetention01,
                out float releaseTimeRetention01,
                out float releaseRawRetention01,
                out float releaseAppliedRetention01);

        // Geometry is now simply:
        // Upper center = Virtual Lower center + (4R-Hn) * N.
        // The old S(T) and phase-acceleration timing resolver no longer own height.
        resolvedEnvelopeRadiusClearanceScale =
            Mathf.Clamp(
                MinimumPeriodicRadiusClearanceScale +
                resolvedReleaseCenterTravelDistance / radius,
                MinimumPeriodicRadiusClearanceScale,
                MaximumPeriodicRadiusClearanceScale);

        resolvedReleaseSurfaceClearance =
            MinimumPeriodicRadiusClearanceScale *
                radius +
            resolvedReleaseCenterTravelDistance;

        float mass =
            ballVisualEqualizer
                ? Mathf.Max(0.0001f, ballVisualEqualizer.mass)
                : 1f;

        resolvedReleaseTargetNormalSpeed =
            sourceEnergyJoule > 0f
                ? Mathf.Sqrt(
                    2f *
                    Mathf.Max(0f, sourceEnergyJoule) /
                    mass)
                : 0f;

        // These remain only for old Inspector/log compatibility.
        resolvedReleasePhaseAcceleration = 0f;
        presentationTimingExpanded = false;
        presentationRequiredPhaseAcceleration = 0f;
        presentationPhaseAccelerationBudget = 0f;
        presentationResolvedHalfCycleFixedSteps =
            Mathf.Max(
                MinimumPeriodicHalfCycleFixedSteps,
                Mathf.RoundToInt(
                    resolvedHalfPeriodSeconds /
                    fixedDt));

        presentationCurrentCeilingR =
            releaseCeilingR;

        presentationCurrentLegacyTravelR =
            releaseLegacyR;

        presentationCurrentResolvedTravelR =
            resolvedReleaseCenterTravelDistance /
            radius;

        presentationCurrentLossHnR =
            releaseLossHnR;

        presentationCurrentLegacyEnergyRetention01 =
            releaseEnergyRetention01;

        presentationCurrentLegacyTimeRetention01 =
            releaseTimeRetention01;

        presentationCurrentRawLegacyRetention01 =
            releaseRawRetention01;

        presentationCurrentAppliedRetention01 =
            releaseAppliedRetention01;
    }



    private float ResolveEnvelopeClearance(
        float equalizerRadius,
        float A0,
        float decayRatePerSecondValue,
        float timeSeconds)
    {
        float radius =
            Mathf.Max(0.0001f, equalizerRadius);

        float resolvedCenterTravel =
            EvaluatePresentationCenterTravelAtTime(
                A0,
                decayRatePerSecondValue,
                timeSeconds,
                radius,
                out _,
                out _,
                out _,
                out _,
                out _,
                out _,
                out _);

        // Surface offset must always contain the sphere diameter 2R.
        return
            MinimumPeriodicRadiusClearanceScale *
                radius +
            resolvedCenterTravel;
    }


    private bool CreateFullSplineEnvelopeMesh(
        float A0,
        float decayRatePerSecondValue,
        float equalizerRadius)
    {
        if (!generatedRoot)
            return false;

        GameObject meshObject =
            new GameObject("NegativeEnvelopeMesh");

        meshObject.layer =
            ballVisualEqualizerCollider
                ? ballVisualEqualizerCollider.gameObject.layer
                : gameObject.layer;

        generatedMeshTransform = meshObject.transform;
        generatedMeshTransform.SetParent(generatedRoot, false);
        generatedMeshTransform.localPosition = Vector3.zero;
        generatedMeshTransform.localRotation = Quaternion.identity;
        generatedMeshTransform.localScale = Vector3.one;

        Mesh mesh =
            BuildFullSplineEnvelopeMeshAsset(
                generatedMeshTransform,
                A0,
                decayRatePerSecondValue,
                equalizerRadius);

        if (!mesh)
        {
            Destroy(meshObject);
            generatedMeshTransform = null;
            return false;
        }

        generatedMeshFilter =
            meshObject.AddComponent<MeshFilter>();

        generatedMeshFilter.sharedMesh =
            mesh;

        generatedMeshCollider =
            meshObject.AddComponent<MeshCollider>();

        generatedMeshCollider.sharedMesh =
            mesh;

        generatedMeshCollider.convex =
            false;

        generatedMeshCollider.isTrigger =
            false;

        meshObject.AddComponent<BallVisualEnvelopeSurfaceMarker>();

        // Floating Rigidbody版ではconcave MeshCollider Trigger Sensorを作りません。
        // Upperの実CollisionそのものをSyncが観測して周期/損失を測ります。
        generatedMeshSensorCollider = null;
        upperEnvelopeSensorEnabled = false;
        upperEnvelopeSensorOccupied = false;

        ConfigureEqualizerOnlyBoundaryCollider(
            generatedMeshCollider);

        ForceEnableEqualizerBoundaryCollision(
            generatedMeshCollider,
            "UpperSolid");

        // No rhythm gate: the physical Upper is continuously available while armed.
        upperEnvelopeSolidRequested = true;
        generatedMeshCollider.enabled = true;
        upperEnvelopeSolidEnabled = true;

        generatedMesh = mesh;

        CaptureLiveSettingsSnapshot();

        Debug.Log(
            $"[ENVELOPE FLOATING RIDE READY] " +
            $"wave={presentationWaveIndex + 1} " +
            $"centerTravel={presentationCurrentResolvedTravelR:F3}R " +
            $"ceiling={presentationCurrentCeilingR:F3}R " +
            $"Hn={presentationCurrentLossHnR:F3}R " +
            $"upperSolid=True " +
            $"physicalLowerCollider=False " +
            $"sensor=False",
            this);

        return true;
    }



    private void ConfigureEqualizerOnlyBoundaryCollider(
        Collider boundary)
    {
        if (!boundary)
            return;

        Collider[] colliders =
            FindObjectsByType<Collider>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);

        foreach (Collider other in colliders)
        {
            if (!other || other == boundary)
                continue;

            bool isEqualizerCollider =
                other == ballVisualEqualizerCollider ||
                (ballVisualEqualizer &&
                 other.attachedRigidbody == ballVisualEqualizer);

            Physics.IgnoreCollision(
                boundary,
                other,
                !isEqualizerCollider);
        }
    }
    /// <summary>
    /// Equalizer専用境界との衝突所有権を再構成します。
    /// Pairwise IgnoreだけではLayer Collision Matrixを上書きできないため、
    /// 最後にEqualizerとのLayer/Pairを明示的に有効化します。
    /// </summary>
    public void RefreshEqualizerBoundaryCollisionOwnership()
    {
        ConfigureEqualizerOnlyBoundaryCollider(
            generatedMeshCollider);

        ForceEnableEqualizerBoundaryCollision(
            generatedMeshCollider,
            "UpperSolid");

        generatedMeshSensorCollider = null;
        upperEnvelopeSensorEnabled = false;
        upperEnvelopeSensorOccupied = false;

        if (generatedMeshCollider)
        {
            generatedMeshCollider.isTrigger = false;
            generatedMeshCollider.enabled = upperEnvelopeSolidRequested;
            upperEnvelopeSolidEnabled = generatedMeshCollider.enabled;
        }
    }



/// <summary>
    /// Boundary -> Equalizer の物理衝突だけは必ず有効にします。
    /// Upper/LowerはEqualizerと同じLayerへ揃え、Layer MatrixとPair Ignoreを
    /// 両方解除します。他Colliderとの衝突は
    /// ConfigureEqualizerOnlyBoundaryCollider() がPair単位で遮断します。
    /// </summary>
    private void ForceEnableEqualizerBoundaryCollision(
        Collider boundary,
        string kind)
    {
        if (!boundary ||
            !ballVisualEqualizerCollider)
        {
            return;
        }

        int equalizerLayer =
            ballVisualEqualizerCollider.gameObject.layer;

        if (boundary.gameObject.layer != equalizerLayer)
        {
            boundary.gameObject.layer =
                equalizerLayer;
        }

        bool rhythmUpperSolid =
            boundary == generatedMeshCollider;

        boundary.enabled =
            rhythmUpperSolid
                ? upperEnvelopeSolidRequested
                : true;

        if (rhythmUpperSolid)
            boundary.isTrigger = false;

        // Layer MatrixがOFFならPhysics.IgnoreCollision(..., false)だけでは
        // 接触イベントは復活しない。ここでLayer Pairを先に有効化する。
        Physics.IgnoreLayerCollision(
            equalizerLayer,
            boundary.gameObject.layer,
            false);

        // Equalizer Sphere <-> Boundary Mesh のPairを最後に必ず許可する。
        Physics.IgnoreCollision(
            boundary,
            ballVisualEqualizerCollider,
            false);

        bool layerIgnored =
            Physics.GetIgnoreLayerCollision(
                equalizerLayer,
                boundary.gameObject.layer);

        bool pairIgnored =
            Physics.GetIgnoreCollision(
                boundary,
                ballVisualEqualizerCollider);

        Debug.Log(
            $"[EQUALIZER BOUNDARY PHYSICS] " +
            $"kind={kind} " +
            $"boundary={boundary.name} " +
            $"boundaryLayer={LayerMask.LayerToName(boundary.gameObject.layer)}({boundary.gameObject.layer}) " +
            $"equalizer={ballVisualEqualizerCollider.name} " +
            $"equalizerLayer={LayerMask.LayerToName(equalizerLayer)}({equalizerLayer}) " +
            $"layerIgnored={layerIgnored} " +
            $"pairIgnored={pairIgnored} " +
            $"enabled={boundary.enabled} " +
            $"trigger={boundary.isTrigger}",
            this);
    }


    // Compatibility surface: physical Lower was removed in AC.
    public void SetLowerGuideTriggerMode(bool trigger, string reason) { }
    public bool LowerGuideTriggerMode => false;
    public bool LowerGuideSolidActive => false;
    public bool LowerGuideTriggerProxyActive => false;

    public bool TryMeasureLowerGuideSpherePressDepth(
        Vector3 sphereCenterWorld,
        float sphereRadiusWorld,
        out float pressDepth,
        out Vector3 outwardNormalWorld,
        out Vector3 surfacePointWorld,
        out int proxyIndex)
    {
        pressDepth = 0f;
        outwardNormalWorld = Vector3.up;
        surfacePointWorld = sphereCenterWorld;
        proxyIndex = -1;
        return false;
    }


    /// <summary>
    /// BallVisualEqualizer自身の現在位置をSplineへ局所射影します。
    /// 「最寄りKnot」ではなく、Spline連続区間をサンプリング＋局所絞り込みして、
    /// Stable-N方向の高さを残したままT/L誤差が最小になる点を選びます。
    ///
    /// centerVisual は Equalizer球がSpline surfaceに接する時の中心位置、
    /// surfaceVisual は実Spline surface位置です。
    /// </summary>
    public bool TryGetEqualizerUnderProjectionFrameVisual(
        out Vector3 centerVisual,
        out Vector3 surfaceVisual,
        out Vector3 tangentVisual,
        out Vector3 normalVisual,
        out float progress01,
        out float clearanceMeters)
    {
        centerVisual = Vector3.zero;
        surfaceVisual = Vector3.zero;
        tangentVisual = Vector3.forward;
        normalVisual = Vector3.up;
        progress01 = 0f;
        clearanceMeters = 0f;

        ResolveReferences();

        if (!slopeCore ||
            !correspondSubject ||
            !ballVisualEqualizer ||
            !ballVisualEqualizerCollider)
        {
            equalizerUnderProjectionValid = false;
            return false;
        }

        if (!TryProjectVisualPointToSplineFrame(
                ballVisualEqualizer.position,
                out centerVisual,
                out surfaceVisual,
                out tangentVisual,
                out normalVisual,
                out progress01,
                out clearanceMeters))
        {
            equalizerUnderProjectionValid = false;
            return false;
        }

        equalizerUnderProgress01 = progress01;
        equalizerUnderSurfaceVisual = surfaceVisual;
        equalizerUnderCenterVisual = centerVisual;
        equalizerUnderTangentVisual = tangentVisual;
        equalizerUnderNormalVisual = normalVisual;
        equalizerUnderClearanceMeters = clearanceMeters;
        equalizerUnderProjectionValid = true;
        return true;
    }


    /// <summary>
    /// 任意のVisual座標点を現在のSpline区間へ射影します。
    /// Equalizer終着点を同じSpline座標へ落とす用途にも使用します。
    /// </summary>
    public bool TryProjectVisualPointToSplineFrameVisual(
        Vector3 pointVisual,
        out Vector3 centerVisual,
        out Vector3 surfaceVisual,
        out Vector3 tangentVisual,
        out Vector3 normalVisual,
        out float progress01)
    {
        return TryProjectVisualPointToSplineFrame(
            pointVisual,
            out centerVisual,
            out surfaceVisual,
            out tangentVisual,
            out normalVisual,
            out progress01,
            out _);
    }


    public float EstimateSplineArcDistanceBetweenProgress(
        float progressA,
        float progressB)
    {
        return EstimateSplineArcDistancePhysics(
            progressA,
            progressB,
            40);
    }


    /// <summary>
    /// BallVisualDriveのTerminal到達計画をEqualizerSyncから受け取る軽量ゲート。
    /// Envelope Meshを毎FixedUpdate recookしません。
    /// 最終区間ではUpper Solidだけを開放し、3者同一点収束をColliderが妨げないようにします。
    /// </summary>
    public void SetArrivalTerminalState(
        bool active,
        float timeToGo,
        float totalTerminalTime)
    {
        arrivalTerminalActive = active;
        arrivalTerminalTimeToGo = Mathf.Max(0f, timeToGo);

        if (!active)
        {
            arrivalTerminalBlend01 = 0f;
            return;
        }

        float total = Mathf.Max(Time.fixedDeltaTime, totalTerminalTime);
        float elapsed01 = 1f - Mathf.Clamp01(arrivalTerminalTimeToGo / total);
        arrivalTerminalBlend01 = Mathf.SmoothStep(0f, 1f, elapsed01);

        // Final 35%ではUpper物理境界を開放する。Sensorは残す。
        if (arrivalTerminalBlend01 >= 0.65f)
            SetUpperEnvelopeSolidEnabled(false, "ArrivalTerminalGate");
    }


    private bool TryProjectVisualPointToSplineFrame(
        Vector3 pointVisual,
        out Vector3 centerVisual,
        out Vector3 surfaceVisual,
        out Vector3 tangentVisual,
        out Vector3 normalVisual,
        out float progress01,
        out float clearanceMeters)
    {
        centerVisual = Vector3.zero;
        surfaceVisual = Vector3.zero;
        tangentVisual = Vector3.forward;
        normalVisual = Vector3.up;
        progress01 = 0f;
        clearanceMeters = 0f;

        ResolveReferences();

        if (!slopeCore ||
            !correspondSubject ||
            !ballVisualEqualizerCollider)
        {
            return false;
        }

        float subjectRadius = ResolveSlopeCoreWorldRadius();
        float equalizerRadius = ResolveEqualizerWorldRadius();

        const int coarseSamples = 32;
        float bestProgress = 0f;
        float bestScore = float.PositiveInfinity;
        bool found = false;

        for (int i = 0; i <= coarseSamples; i++)
        {
            float p = i / (float)coarseSamples;
            if (!TryScoreProjectionCandidate(
                    pointVisual,
                    p,
                    subjectRadius,
                    equalizerRadius,
                    out float score,
                    out _, out _, out _, out _, out _))
            {
                continue;
            }

            if (score < bestScore)
            {
                bestScore = score;
                bestProgress = p;
                found = true;
            }
        }

        if (!found)
            return false;

        // Coarse winnerの前後1区間を三分探索して、Knot密度に依存しない局所射影へ絞る。
        float step = 1f / coarseSamples;
        float lo = Mathf.Clamp01(bestProgress - step);
        float hi = Mathf.Clamp01(bestProgress + step);

        for (int iteration = 0; iteration < 7; iteration++)
        {
            float p1 = Mathf.Lerp(lo, hi, 1f / 3f);
            float p2 = Mathf.Lerp(lo, hi, 2f / 3f);

            bool ok1 = TryScoreProjectionCandidate(
                pointVisual, p1, subjectRadius, equalizerRadius,
                out float s1, out _, out _, out _, out _, out _);
            bool ok2 = TryScoreProjectionCandidate(
                pointVisual, p2, subjectRadius, equalizerRadius,
                out float s2, out _, out _, out _, out _, out _);

            if (!ok1 && !ok2)
                break;

            if (!ok2 || (ok1 && s1 <= s2))
                hi = p2;
            else
                lo = p1;
        }

        progress01 = Mathf.Clamp01((lo + hi) * 0.5f);

        if (!TryScoreProjectionCandidate(
                pointVisual,
                progress01,
                subjectRadius,
                equalizerRadius,
                out _,
                out centerVisual,
                out surfaceVisual,
                out tangentVisual,
                out normalVisual,
                out clearanceMeters))
        {
            return false;
        }

        return true;
    }


    private bool TryScoreProjectionCandidate(
        Vector3 pointVisual,
        float progress01,
        float subjectRadius,
        float equalizerRadius,
        out float score,
        out Vector3 centerVisual,
        out Vector3 surfaceVisual,
        out Vector3 tangentVisual,
        out Vector3 normalVisual,
        out float clearanceMeters)
    {
        score = float.PositiveInfinity;
        centerVisual = Vector3.zero;
        surfaceVisual = Vector3.zero;
        tangentVisual = Vector3.forward;
        normalVisual = Vector3.up;
        clearanceMeters = 0f;

        if (!TryEvaluateSplineSurfacePhysics(
                progress01,
                subjectRadius,
                out _,
                out Vector3 surfacePhysics,
                out Vector3 tangentPhysics,
                out Vector3 normalPhysics))
        {
            return false;
        }

        surfaceVisual = correspondSubject.MapPoint(surfacePhysics);
        tangentVisual = correspondSubject.MapDirection(tangentPhysics);
        normalVisual = correspondSubject.MapDirection(normalPhysics);

        if (normalVisual.sqrMagnitude <= 0.000001f ||
            tangentVisual.sqrMagnitude <= 0.000001f)
        {
            return false;
        }

        normalVisual.Normalize();
        tangentVisual = Vector3.ProjectOnPlane(tangentVisual, normalVisual);
        if (tangentVisual.sqrMagnitude <= 0.000001f)
            return false;
        tangentVisual.Normalize();

        centerVisual = surfaceVisual + normalVisual * equalizerRadius;
        Vector3 delta = pointVisual - centerVisual;
        float signedHeight = Vector3.Dot(delta, normalVisual);
        Vector3 planeError = Vector3.ProjectOnPlane(delta, normalVisual);

        // 「真下」を優先: T/L誤差を主尺度にし、Splineの裏側は強く罰する。
        float behindPenalty = signedHeight < -0.01f
            ? 1000f + signedHeight * signedHeight * 100f
            : 0f;

        score =
            planeError.sqrMagnitude * 8f +
            delta.sqrMagnitude * 0.05f +
            behindPenalty;

        clearanceMeters = Mathf.Max(0f, signedHeight);
        return true;
    }


    /// <summary>
    /// 互換API。今後のVirtual LowerはSubject進歩率ではなく
    /// BallVisualEqualizer自身の直下Spline射影で決めます。
    /// </summary>
    public bool TryGetVirtualLowerTurnpointFrameVisual(
        out Vector3 centerVisual,
        out Vector3 tangentVisual,
        out Vector3 normalVisual)
    {
        return TryGetEqualizerUnderProjectionFrameVisual(
            out centerVisual,
            out _,
            out tangentVisual,
            out normalVisual,
            out _,
            out _);
    }


    /// <summary>
    /// 現在のPresentation waveが要求する中心移動量を返します。
    /// 第1波では4R-Hn、以後3.8R-Hn, 3.3R-Hn...。
    /// 4RをVirtual Lower/Upper間へ実際に噛ませるための正式APIです。
    /// </summary>
    public bool TryGetCurrentPresentationCenterTravel(
        out float centerTravelMeters,
        out float centerTravelR)
    {
        centerTravelMeters = 0f;
        centerTravelR = 0f;

        float radius =
            Mathf.Max(0.0001f, cachedEqualizerRadius);

        if (!envelopeBuilt ||
            cachedA0 <= 0f ||
            radius <= 0.0001f)
        {
            return false;
        }

        float elapsed =
            Mathf.Max(
                0f,
                Time.fixedTime - presentationReleaseFixedTime);

        centerTravelMeters =
            EvaluatePresentationCenterTravelAtTime(
                cachedA0,
                cachedGamma,
                elapsed,
                radius,
                out presentationCurrentCeilingR,
                out presentationCurrentLegacyTravelR,
                out presentationCurrentLossHnR,
                out presentationCurrentLegacyEnergyRetention01,
                out presentationCurrentLegacyTimeRetention01,
                out presentationCurrentRawLegacyRetention01,
                out presentationCurrentAppliedRetention01);

        centerTravelR = centerTravelMeters / radius;
        presentationCurrentResolvedTravelR = centerTravelR;
        return true;
    }


    /// <summary>
    /// Hybrid Wave Controllerへ現在の連続減衰状態を返します。
    /// centerTravelはLower球中心 -> Upper接触時球中心のN方向振幅、
    /// timeEnergyCeiling01はGate侵入後から連続的に縮むEnergy上限です。
    /// Collider衝突によるcanonicalEnergyRatioとは分離し、Sync側でminを取ります。
    /// </summary>

    /// <summary>
    /// Sync側の3-wave spatial modeのauthorityを設定します。
    /// Inspector switchのauthorityはSync側だけに置き、Envelopeは状態をmirrorします。
    /// </summary>
    public void ConfigureSpatialWaveAuthority(
        bool enabled,
        int waveCount)
    {
        int safeCount =
            Mathf.Clamp(
                waveCount,
                1,
                8);

        bool changed =
            spatialWaveAuthorityActive != enabled ||
            spatialWaveAuthorityCount != safeCount;

        spatialWaveAuthorityActive = enabled;
        spatialWaveAuthorityCount = safeCount;

        if (!enabled)
        {
            spatialWaveAuthorityProgress01 = 0f;
            return;
        }

        if (changed)
        {
            spatialWaveAuthorityProgress01 = 0f;
            presentationWaveIndex = 0;

            if (envelopeBuilt)
            {
                UpdatePresentationRuntimeDiagnostics(
                    Mathf.Max(
                        0f,
                        Time.fixedTime -
                        presentationReleaseFixedTime));

                pendingCanonicalGeometryRebuild = true;
            }
        }
    }


    /// <summary>
    /// Release->Terminal targetを「1つのactive stair wave domain」として公開します。
    /// 3-wave modeではこの区間へN波を空間配置します。
    /// </summary>
    public bool TryGetActiveSplineWaveDomain(
        out float releaseProgress01,
        out float targetProgress01,
        out float equalizerProgress01,
        out float activeArcLengthMeters)
    {
        releaseProgress01 =
            capturedReleaseProgress01;

        targetProgress01 =
            capturedTargetProgress01;

        equalizerProgress01 =
            equalizerUnderProgress01;

        activeArcLengthMeters = 0f;

        if (!envelopeBuilt ||
            targetProgress01 <=
            releaseProgress01 + 0.000001f)
        {
            return false;
        }

        if (!equalizerUnderProjectionValid)
        {
            if (!TryGetEqualizerUnderProjectionFrameVisual(
                    out _,
                    out _,
                    out _,
                    out _,
                    out equalizerProgress01,
                    out _))
            {
                return false;
            }
        }
        else
        {
            equalizerProgress01 =
                equalizerUnderProgress01;
        }

        activeArcLengthMeters =
            EstimateSplineArcDistancePhysics(
                releaseProgress01,
                targetProgress01,
                40);

        return activeArcLengthMeters > 0.0001f;
    }


    /// <summary>
    /// Spatial progressだけでPresentation wave indexを進めます。
    /// Physical Upper contactはこのmode中にwave index authorityを持ちません。
    /// </summary>
    public void SetSpatialPresentationProgress(
        float normalizedProgress01)
    {
        if (!spatialWaveAuthorityActive)
            return;

        float p =
            Mathf.Clamp01(
                normalizedProgress01);

        spatialWaveAuthorityProgress01 = p;

        int count =
            Mathf.Max(
                1,
                spatialWaveAuthorityCount);

        int desiredWaveIndex =
            Mathf.Clamp(
                Mathf.FloorToInt(
                    p * count),
                0,
                count - 1);

        if (desiredWaveIndex ==
            presentationWaveIndex)
        {
            return;
        }

        presentationWaveIndex =
            desiredWaveIndex;

        float elapsed =
            Mathf.Max(
                0f,
                Time.fixedTime -
                presentationReleaseFixedTime);

        UpdatePresentationRuntimeDiagnostics(
            elapsed);

        pendingCanonicalGeometryRebuild = true;

        Debug.Log(
            $"[ENVELOPE SPATIAL WAVE] " +
            $"wave={presentationWaveIndex + 1}/{count} " +
            $"domainP={p:F4} " +
            $"elapsed={elapsed:F4}s " +
            $"resolved={presentationCurrentResolvedTravelR:F3}R " +
            $"Hn={presentationCurrentLossHnR:F3}R",
            this);
    }


    public bool SpatialWaveAuthorityActive =>
        spatialWaveAuthorityActive;

    public float SpatialWaveAuthorityProgress01 =>
        spatialWaveAuthorityProgress01;


    /// <summary>
    /// Floating Rigidbody controller用の正式なRide Frame。
    /// lowerCenter = Equalizer直下Splineに接する球中心。
    /// rideCenter  = Spring/Damperの平衡点（現在4R-Hnの中央）。
    /// upperCenter = 実Upper接触時の球中心。
    ///
    /// 重要: このAPIは周期を指示しません。4R-Hn/Spline幾何だけを公開します。
    /// </summary>
    public bool TryGetFloatingRideFrame(
        out Vector3 lowerCenterVisual,
        out Vector3 rideCenterVisual,
        out Vector3 upperCenterVisual,
        out Vector3 tangentVisual,
        out Vector3 normalVisual,
        out float spanMeters,
        out float spanR,
        out float observedPeriodSeconds)
    {
        lowerCenterVisual = Vector3.zero;
        rideCenterVisual = Vector3.zero;
        upperCenterVisual = Vector3.zero;
        tangentVisual = Vector3.forward;
        normalVisual = Vector3.up;
        spanMeters = 0f;
        spanR = 0f;
        observedPeriodSeconds =
            observedGeometryPeriodValid
                ? observedGeometryPeriodSeconds
                : 0f;

        if (!TryGetEqualizerUnderProjectionFrameVisual(
                out lowerCenterVisual,
                out _,
                out tangentVisual,
                out normalVisual,
                out _,
                out _))
        {
            return false;
        }

        if (!TryGetCurrentPresentationCenterTravel(
                out spanMeters,
                out spanR))
        {
            return false;
        }

        if (normalVisual.sqrMagnitude <= 0.000001f)
            return false;

        normalVisual.Normalize();

        tangentVisual =
            Vector3.ProjectOnPlane(
                tangentVisual,
                normalVisual);

        if (tangentVisual.sqrMagnitude <= 0.000001f)
            return false;

        tangentVisual.Normalize();

        rideCenterVisual =
            lowerCenterVisual +
            normalVisual *
            (spanMeters * 0.5f);

        upperCenterVisual =
            lowerCenterVisual +
            normalVisual *
            spanMeters;

        return envelopeBuilt;
    }


    public bool TryGetHybridWaveState(
        out float centerTravelMeters,
        out float centerTravelR,
        out float halfPeriodSeconds,
        out float timeEnergyCeiling01,
        out float gammaPerSecond)
    {
        centerTravelMeters = 0f;
        centerTravelR = 0f;

        float period =
            observedGeometryPeriodValid &&
            observedGeometryPeriodSeconds > 0.0001f
                ? observedGeometryPeriodSeconds
                : Mathf.Max(
                    Time.fixedDeltaTime * 4f,
                    preferredContactPeriodSeconds);

        halfPeriodSeconds =
            period * 0.5f;

        timeEnergyCeiling01 = 1f;

        gammaPerSecond =
            Mathf.Max(
                0f,
                decayRatePerSecond);

        if (!TryGetCurrentPresentationCenterTravel(
                out centerTravelMeters,
                out centerTravelR))
        {
            return false;
        }

        float elapsed =
            Mathf.Max(
                0f,
                Time.fixedTime -
                presentationReleaseFixedTime);

        float rawTimeRetention01 =
            Mathf.Clamp01(
                Mathf.Exp(
                    -gammaPerSecond *
                    elapsed));

        timeEnergyCeiling01 =
            Mathf.Clamp01(
                Mathf.Lerp(
                    1f,
                    rawTimeRetention01,
                    Mathf.Clamp01(
                        waveTimeDecayInfluence)));

        return envelopeBuilt;
    }



    /// <summary>
    /// Hybrid Waveのイベント認識だけに使う動的Capture profileです。
    /// Rigidbody運動、Upper Mesh形状、減衰Energyは変更しません。
    /// FixedUpdateの離散化に合わせ、時間閾値は最低2 physics ticksを確保します。
    /// </summary>
    public bool TryGetHybridWaveEventProfile(
        out float upperCaptureDistanceMeters,
        out float lowerCandidateDistanceMeters,
        out float minimumUpperApproachSpeed,
        out float minimumEventIntervalSeconds)
    {
        upperCaptureDistanceMeters = 0f;
        lowerCandidateDistanceMeters = 0f;
        minimumUpperApproachSpeed = 0.05f;
        minimumEventIntervalSeconds = Mathf.Max(Time.fixedDeltaTime * 2f, 0.04f);

        float radius = Mathf.Max(
            0.0001f,
            cachedEqualizerRadius > 0f
                ? cachedEqualizerRadius
                : ResolveEqualizerWorldRadius());

        if (!TryGetHybridWaveState(
                out float centerTravelMeters,
                out _,
                out float halfPeriodSeconds,
                out _,
                out _))
        {
            return false;
        }

        upperCaptureDistanceMeters = Mathf.Max(
            radius * Mathf.Max(0.10f, hybridUpperCaptureDistanceR),
            centerTravelMeters * 0.10f);

        lowerCandidateDistanceMeters = Mathf.Max(
            radius * Mathf.Max(0.10f, hybridLowerCandidateDistanceR),
            radius * 0.05f);

        float meanHalfWaveSpeed =
            centerTravelMeters /
            Mathf.Max(Time.fixedDeltaTime * 2f, halfPeriodSeconds);

        minimumUpperApproachSpeed = Mathf.Max(
            0.05f,
            meanHalfWaveSpeed * Mathf.Max(0.005f, hybridUpperApproachSpeedRatio));

        minimumEventIntervalSeconds = Mathf.Max(
            Time.fixedDeltaTime * 2f,
            halfPeriodSeconds * 0.25f);

        return envelopeBuilt;
    }


    /// <summary>
    /// D: Equalizer SchedulerからREAD/DERIVEDな実現可能Wave profileを受け取ります。
    /// amplitudeScaleだけをEnvelopeの中心移動量へ掛け、Upper Meshは既存の
    /// double-buffer経路で安全に再生成します。Inspector調整値は増やしません。
    /// </summary>
    public void SetHybridWaveFeasibilityProfile(
        float amplitudeScale01,
        float halfPeriodSeconds)
    {
        // Compatibility only.
        // Floating Rigidbody版ではT都合で4R-Hn振幅を縮めません。
        hybridWaveFeasibilityAmplitudeScale01 = 1f;
        hybridWaveFeasibilityHalfPeriodSeconds =
            Mathf.Max(
                0f,
                halfPeriodSeconds);
        hybridWaveFeasibilityAmplitudeLimited = false;
    }



    public void ResetHybridWaveFeasibilityProfile()
    {
        hybridWaveFeasibilityAmplitudeScale01 = 1f;
        hybridWaveFeasibilityHalfPeriodSeconds = 0f;
        hybridWaveFeasibilityAmplitudeLimited = false;
    }



    public Collider CurrentUpperEnvelopeCollider =>
        generatedMeshCollider;

    public Collider CurrentUpperEnvelopeSensorCollider => null;

    public bool UpperEnvelopeSolidEnabled =>
        generatedMeshCollider &&
        generatedMeshCollider.enabled;

    public bool UpperEnvelopeSensorEnabled => false;

    public bool UpperEnvelopeSensorOccupied => false;


    public void SetUpperEnvelopeSensorOccupied(
        bool occupied)
    {
        // Compatibility no-op. Concave MeshCollider sensor was removed.
        upperEnvelopeSensorOccupied = false;
    }



    public void SetUpperEnvelopeSolidEnabled(
        bool enabled,
        string reason)
    {
        upperEnvelopeSolidRequested = enabled;

        if (!generatedMeshCollider)
            return;

        generatedMeshCollider.isTrigger = false;
        generatedMeshCollider.enabled = enabled;
        upperEnvelopeSolidEnabled = enabled;

        Debug.Log(
            $"[EQUALIZER UPPER SOLID] " +
            $"solid={enabled} " +
            $"wave={presentationWaveIndex + 1} " +
            $"reason={reason}",
            this);
    }



    public Collider CurrentLowerGuideCollider => null;


    /// <summary>
    /// SlopeStickCore.maxGroundSpeedをREAD ONLYで取得します。
    /// </summary>
    private bool TryReadSlopeCoreMaxGroundSpeed(
        out float speed)
    {
        speed = 0f;

        if (!slopeCore)
            ResolveReferences();

        if (!slopeCore ||
            MaxGroundSpeedField == null)
        {
            maxGroundSpeedReadAvailable = false;
            sourceMaxGroundSpeedReadOnly = 0f;
            return false;
        }

        object raw =
            MaxGroundSpeedField.GetValue(
                slopeCore);

        if (!(raw is float value))
        {
            maxGroundSpeedReadAvailable = false;
            sourceMaxGroundSpeedReadOnly = 0f;
            return false;
        }

        speed =
            Mathf.Max(0f, value);

        sourceMaxGroundSpeedReadOnly = speed;
        maxGroundSpeedReadAvailable = true;
        return true;
    }


    public void SubmitObservedGeometryPeriod(
        float observedPeriodSeconds)
    {
        if (float.IsNaN(observedPeriodSeconds) ||
            float.IsInfinity(observedPeriodSeconds) ||
            observedPeriodSeconds <= Time.fixedDeltaTime * 2f)
        {
            return;
        }

        float blend =
            Mathf.Clamp01(
                periodObservationBlend);

        if (!observedGeometryPeriodValid)
        {
            observedGeometryPeriodSeconds =
                observedPeriodSeconds;
        }
        else
        {
            observedGeometryPeriodSeconds =
                Mathf.Lerp(
                    observedGeometryPeriodSeconds,
                    observedPeriodSeconds,
                    blend);
        }

        observedGeometryPeriodValid = true;

        // T is now an observed natural oscillator result, not a master schedule.
        resolvedContactPeriodSeconds =
            Mathf.Max(
                Time.fixedDeltaTime * 4f,
                observedGeometryPeriodSeconds);

        resolvedHalfPeriodSeconds =
            resolvedContactPeriodSeconds * 0.5f;

        nominalExperimentPeriodSeconds =
            Mathf.Max(
                Time.fixedDeltaTime * 4f,
                preferredContactPeriodSeconds);

        observedGeometryPeriodCorrectionRatio =
            resolvedContactPeriodSeconds /
            Mathf.Max(
                0.0001f,
                nominalExperimentPeriodSeconds);
    }



    public void SetMaxGroundSpeedExperimentCycle(
        int cycleIndex)
    {
        // K compatibility API: synthetic decay cycles are disabled.
        // Always expose the current READ ONLY maxGroundSpeed as the planned value.
        maxGroundSpeedExperimentCycleIndex = 0;

        if (TryReadSlopeCoreMaxGroundSpeed(
                out float sourceMaxGroundSpeed))
        {
            plannedMaxGroundSpeedForCycle =
                sourceMaxGroundSpeed;
        }
        else
        {
            plannedMaxGroundSpeedForCycle = 0f;
        }

        plannedMaxGroundSpeedRatio = 1f;
    }


    public void ResetMaxGroundSpeedExperiment()
    {
        observedGeometryPeriodSeconds = 0f;
        observedGeometryPeriodCorrectionRatio = 1f;
        observedGeometryPeriodValid = false;
        nominalExperimentPeriodSeconds = 0f;
        SetMaxGroundSpeedExperimentCycle(0);
    }


    public bool TryEvaluateMaxGroundSpeedDecayExperiment(
        int cycleIndex,
        out float sourceMaxGroundSpeed,
        out float plannedMaxGroundSpeed,
        out float normalizedProgress01)
    {
        // K compatibility API: there is no longer a 16->8 / 24->12 synthetic sweep.
        normalizedProgress01 = 0f;
        maxGroundSpeedExperimentCycleIndex = 0;

        if (!TryReadSlopeCoreMaxGroundSpeed(
                out sourceMaxGroundSpeed))
        {
            plannedMaxGroundSpeed = 0f;
            plannedMaxGroundSpeedForCycle = 0f;
            plannedMaxGroundSpeedRatio = 1f;
            return false;
        }

        plannedMaxGroundSpeed =
            sourceMaxGroundSpeed;
        plannedMaxGroundSpeedForCycle =
            sourceMaxGroundSpeed;
        plannedMaxGroundSpeedRatio = 1f;
        return true;
    }


    public int MaxGroundSpeedDecayCycleCount =>
        0;

    public int MaxGroundSpeedExperimentCycleIndex =>
        maxGroundSpeedExperimentCycleIndex;

    public float PreferredContactPeriodSeconds =>
        preferredContactPeriodSeconds;

    public float BaseExperimentPeriodSeconds =>
        Mathf.Max(0.0001f, preferredContactPeriodSeconds);

    public float ResolvedExperimentPeriodSeconds =>
        resolvedContactPeriodSeconds;

    public float NominalExperimentPeriodSeconds =>
        nominalExperimentPeriodSeconds;

    public float ObservedGeometryPeriodSeconds =>
        observedGeometryPeriodSeconds;

    public float ObservedGeometryPeriodCorrectionRatio =>
        observedGeometryPeriodCorrectionRatio;

    /// <summary>
    /// SlopeStickCore.maxGroundSpeedをその場でREAD ONLY取得します。
    /// ACでは互換/診断APIとして残し、Virtual Lowerの4R距離やTは変更しません。
    /// このAPIからSlopeStickCoreへ書き込みは行いません。
    /// </summary>
    public bool TryGetSourceMaxGroundSpeedReadOnly(
        out float speed)
    {
        return TryReadSlopeCoreMaxGroundSpeed(
            out speed);
    }


    public float SourceMaxGroundSpeedReadOnly =>
        sourceMaxGroundSpeedReadOnly;

    public float PlannedMaxGroundSpeedForCycle =>
        plannedMaxGroundSpeedForCycle;


    public bool TryGetPeriodicContactPlan(
        out float periodSeconds,
        out float halfPeriodSeconds,
        out float timeCostSeconds,
        out float gammaPerSecond,
        out float radiusClearanceScale,
        out float releaseCenterTravelDistance,
        out float releaseTargetNormalSpeed,
        out float releasePhaseAcceleration)
    {
        periodSeconds =
            resolvedContactPeriodSeconds;

        halfPeriodSeconds =
            resolvedHalfPeriodSeconds;

        timeCostSeconds =
            decayTimeCostSeconds;

        gammaPerSecond =
            decayRatePerSecond;

        radiusClearanceScale =
            resolvedEnvelopeRadiusClearanceScale;

        releaseCenterTravelDistance =
            resolvedReleaseCenterTravelDistance;

        releaseTargetNormalSpeed =
            resolvedReleaseTargetNormalSpeed;

        releasePhaseAcceleration =
            resolvedReleasePhaseAcceleration;

        return
            envelopeBuilt &&
            generatedMeshCollider &&
            periodSeconds > 0.0001f &&
            halfPeriodSeconds > 0.0001f;
    }


    // Compatibility API. New code should use TryGetPeriodicContactPlan().
    public bool TryGetForcedOscillationTiming(
        out float timeCostSeconds,
        out float gammaPerSecond)
    {
        timeCostSeconds =
            decayTimeCostSeconds;

        gammaPerSecond =
            decayRatePerSecond;

        return
            envelopeBuilt &&
            generatedMeshCollider &&
            resolvedContactPeriodSeconds > 0.0001f;
    }


    public bool IsUpperEnvelopeCollider(
        Collider collider)
    {
        return
            collider &&
            generatedMeshCollider &&
            collider == generatedMeshCollider;
    }


    public bool IsUpperEnvelopeSensorCollider(
        Collider collider)
    {
        return false;
    }



    public bool IsLowerGuideCollider(
        Collider collider)
    {
        return false;
    }

    public bool IsEqualizerBoundaryCollider(
        Collider collider)
    {
        return IsUpperEnvelopeCollider(collider);
    }


    private Mesh BuildFullSplineEnvelopeMeshAsset(
        Transform meshTransform,
        float A0,
        float decayRatePerSecondValue,
        float equalizerRadius)
    {
        if (!meshTransform ||
            !correspondSubject ||
            !slopeCore)
        {
            return null;
        }

        int sampleCount =
            Mathf.Max(2, segmentCount + 1);

        int vertexCount = sampleCount * 2;
        Vector3[] vertices = new Vector3[vertexCount];
        int[] triangles = new int[(sampleCount - 1) * 6];

        float halfWidth =
            Mathf.Max(0.05f, envelopeWidth * 0.5f);

        float subjectRadius = ResolveSlopeCoreWorldRadius();

        const float startProgress = 0f;
        const float endProgress = 1f;

        Vector3 previousCenterWorld = Vector3.zero;
        bool hasPreviousCenter = false;
        float curvedCenterlineLength = 0f;
        float basePathLength =
            EstimateSplineArcDistancePhysics(
                startProgress,
                endProgress,
                Mathf.Max(32, segmentCount * 2));

        Vector3 releaseTangentPhysics = slopeCore.BallVisualSlopeTangent;
        slopeCore.TryEvaluateBallVisualSectionFramePhysics(
            capturedReleaseProgress01,
            out _,
            out releaseTangentPhysics,
            out _);

        float finalTravelTime = 0f;

        for (int i = 0; i < sampleCount; i++)
        {
            float u = i / (float)(sampleCount - 1);
            float progress = Mathf.Lerp(startProgress, endProgress, u);

            if (!TryEvaluateSplineSurfacePhysics(
                    progress,
                    subjectRadius,
                    out _,
                    out Vector3 surfacePhysics,
                    out Vector3 tangentPhysics,
                    out Vector3 normalPhysics))
            {
                return null;
            }

            float distanceFromRelease =
                progress > capturedReleaseProgress01
                    ? EstimateSplineArcDistancePhysics(
                        capturedReleaseProgress01,
                        progress,
                        16)
                    : 0f;

            float timeSeconds =
                progress > capturedReleaseProgress01
                    ? ResolveTravelTimeForArcDistance(
                        distanceFromRelease,
                        releaseTangentPhysics)
                    : 0f;

            finalTravelTime = Mathf.Max(finalTravelTime, timeSeconds);

            float clearance =
                ResolveEnvelopeClearance(
                    equalizerRadius,
                    A0,
                    decayRatePerSecondValue,
                    timeSeconds);

            Vector3 centerPhysics =
                surfacePhysics +
                normalPhysics * clearance;

            Vector3 widthAxisPhysics =
                Vector3.Cross(
                    normalPhysics,
                    tangentPhysics);

            if (widthAxisPhysics.sqrMagnitude <= 0.000001f)
                return null;

            widthAxisPhysics.Normalize();

            Vector3 leftPhysics =
                centerPhysics -
                widthAxisPhysics * halfWidth;

            Vector3 rightPhysics =
                centerPhysics +
                widthAxisPhysics * halfWidth;

            Vector3 centerWorld =
                correspondSubject.MapPoint(centerPhysics);

            Vector3 leftWorld =
                correspondSubject.MapPoint(leftPhysics);

            Vector3 rightWorld =
                correspondSubject.MapPoint(rightPhysics);

            if (hasPreviousCenter)
            {
                curvedCenterlineLength +=
                    Vector3.Distance(
                        previousCenterWorld,
                        centerWorld);
            }

            previousCenterWorld = centerWorld;
            hasPreviousCenter = true;

            vertices[i * 2 + 0] =
                meshTransform.InverseTransformPoint(leftWorld);

            vertices[i * 2 + 1] =
                meshTransform.InverseTransformPoint(rightWorld);
        }

        int ti = 0;

        for (int i = 0; i < sampleCount - 1; i++)
        {
            int a = i * 2;
            int b = i * 2 + 1;
            int c = (i + 1) * 2;
            int d = (i + 1) * 2 + 1;

            triangles[ti++] = a;
            triangles[ti++] = b;
            triangles[ti++] = d;
            triangles[ti++] = a;
            triangles[ti++] = d;
            triangles[ti++] = c;
        }

        Mesh mesh = new Mesh();
        mesh.name = "NegativeEnvelope_FullSpline_RealTime";
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        latestSlopeBaseLength = basePathLength;
        latestColliderCurveLength = curvedCenterlineLength;
        latestCurveLengthRatio =
            basePathLength > 0.000001f
                ? curvedCenterlineLength / basePathLength
                : 1f;
        latestEnvelopeTravelTimeSeconds = finalTravelTime;

        return mesh;
    }


    // ================================================================
    // Root
    // ================================================================
    private void CreateRoot()
    {
        if (generatedRoot)
            ClearAllGeneratedEnvelopeRoots();

        GameObject root =
            new GameObject(
                "BallVisualEqualizer_NegativeEnvelope");

        root.layer = gameObject.layer;
        generatedRoot = root.transform;
        generatedRoot.SetParent(transform, true);
        generatedEnvelopeRoots.Add(generatedRoot);
    }


    // ================================================================
    // Visual-frame turn mapping
    // ================================================================
    // Envelope energy/decay scalars remain unchanged.  Only Visual-space
    // points/vectors and, when necessary, the generated Root transform are
    // transported by the incremental coordinate map.
    public void ApplyVisualFrameTurnDelta(
        Vector3 pivot,
        Quaternion deltaTurn)
    {
        // Compatibility API only.  Normal specification clears/resynchronizes
        // the active Equalizer before Stage Turn, so no live Envelope transport
        // is performed here. PhysicsRoot geometry is re-mapped on next build.
    }
    // ================================================================
    // Public clear / next generation
    // ================================================================

    /// <summary>
    /// 明示的な完全Clear。
    /// 現在のUpper/Lower Envelopeを完全に削除します。
    /// </summary>
    public void ClearEnvelope()
    {
        ClearAllGeneratedEnvelopeRoots();

        latestEnvelopeGeometryCached =
            false;

        liveSettingsSnapshotValid =
            false;

        armed =
            false;

        envelopeBuilt =
            false;

        sourceEnergyJoule =
            0f;

        canonicalReferenceHeight =
            0f;

        sourceEnergyAxisVisual =
            Vector3.up;

        canonicalEnergyRatio =
            1f;

        canonicalNormalAcceleration =
            0f;

        equalizerLaunchSpeed =
            0f;

        equalizerVerticalLaunchSpeed =
            0f;

        entryApexHeight =
            0f;

        gamma =
            0f;

        resolvedContactPeriodSeconds =
            Mathf.Max(0.04f, preferredContactPeriodSeconds);

        resolvedHalfPeriodSeconds =
            resolvedContactPeriodSeconds * 0.5f;

        resolvedEnvelopeRadiusClearanceScale =
            MinimumPeriodicRadiusClearanceScale;

        resolvedReleaseCenterTravelDistance = 0f;
        resolvedReleaseSurfaceClearance = 0f;
        resolvedReleaseTargetNormalSpeed = 0f;
        resolvedReleasePhaseAcceleration = 0f;

        presentationWaveIndex = 0;
        spatialWaveAuthorityActive = false;
        spatialWaveAuthorityCount = 3;
        spatialWaveAuthorityProgress01 = 0f;
        presentationReleaseFixedTime = 0f;
        presentationCurrentCeilingR = 0f;
        presentationCurrentLegacyTravelR = 0f;
        presentationCurrentResolvedTravelR = 0f;
        presentationCurrentLossHnR = 0f;
        presentationCurrentLegacyEnergyRetention01 = 1f;
        presentationCurrentLegacyTimeRetention01 = 1f;
        presentationCurrentRawLegacyRetention01 = 1f;
        presentationCurrentAppliedRetention01 = 1f;

        pendingCanonicalGeometryRebuild = false;
        ResetHybridWaveFeasibilityProfile();
        decayTimeCostSeconds = 0f;

        equalizerUnderProgress01 = 0f;
        equalizerUnderSurfaceVisual = Vector3.zero;
        equalizerUnderCenterVisual = Vector3.zero;
        equalizerUnderTangentVisual = Vector3.forward;
        equalizerUnderNormalVisual = Vector3.up;
        equalizerUnderClearanceMeters = 0f;
        equalizerUnderProjectionValid = false;
        arrivalTerminalActive = false;
        arrivalTerminalTimeToGo = 0f;
        arrivalTerminalBlend01 = 0f;
    }


    /// <summary>
    /// 平面→斜面などで次のEnvelopeを生成する直前に使用する。
    /// 旧Envelopeは必ず破棄し、物理境界を一世代だけに保つ。
    /// </summary>
    private void PrepareForNextEnvelopeGeneration()
    {
        ClearAllGeneratedEnvelopeRoots();
        latestEnvelopeGeometryCached = false;
        envelopeBuilt = false;
        liveSettingsSnapshotValid = false;
        pendingCanonicalGeometryRebuild = false;
        ResetHybridWaveFeasibilityProfile();
        equalizerUnderProjectionValid = false;
        arrivalTerminalActive = false;
        arrivalTerminalTimeToGo = 0f;
        arrivalTerminalBlend01 = 0f;
    }


    /// <summary>
    /// 過去Envelope GameObjectをDestroyせず、
    /// 「最新Envelope」用参照だけnullへ戻す。
    ///
    /// Appendモードの核心。
    /// </summary>
/// <summary>
    /// 現在まで生成したEnvelope Rootを全て削除する。
    /// Replaceモードで次Slopeへ入る時と、
    /// 明示的Clear時に使用する。
    /// </summary>
    private void ClearAllGeneratedEnvelopeRoots()
    {
        // List外にcurrent Rootがある異常ケースも拾えるよう、
        // currentを先にListへ補完する。
        if (generatedRoot &&
            !generatedEnvelopeRoots.Contains(
                generatedRoot))
        {
            generatedEnvelopeRoots.Add(
                generatedRoot);
        }


        for (int i =
                 generatedEnvelopeRoots.Count - 1;
             i >= 0;
             i--)
        {
            Transform root =
                generatedEnvelopeRoots[i];

            if (!root)
                continue;


            // Root配下の全Colliderを先に切り離す。Upper Sensorは
            // MeshFilterを持たない別Childなので、Filter単位では漏れる。
            MeshCollider[] rootMeshColliders =
                root.GetComponentsInChildren<MeshCollider>(
                    true);

            for (int c = 0;
                 c < rootMeshColliders.Length;
                 c++)
            {
                MeshCollider rootCollider =
                    rootMeshColliders[c];

                if (!rootCollider)
                    continue;

                rootCollider.enabled = false;
                rootCollider.sharedMesh = null;
            }

            // Root配下にある生成Meshアセットも明示的に破棄。
            MeshFilter[] filters =
                root.GetComponentsInChildren<MeshFilter>(
                    true);

            for (int j = 0;
                 j < filters.Length;
                 j++)
            {
                MeshFilter filter =
                    filters[j];

                if (!filter)
                    continue;

                Mesh mesh =
                    filter.sharedMesh;

                filter.sharedMesh =
                    null;

                if (mesh)
                {
                    Destroy(
                        mesh);
                }
            }


            root.gameObject.SetActive(
                false);

            Destroy(
                root.gameObject);
        }


        generatedEnvelopeRoots.Clear();

        generatedRoot =
            null;

        generatedMesh =
            null;

        generatedMeshTransform =
            null;

        generatedMeshFilter =
            null;

        generatedMeshCollider =
            null;

        generatedMeshSensorCollider =
            null;

        upperEnvelopeSolidRequested = false;
        upperEnvelopeSolidEnabled = false;
        upperEnvelopeSensorEnabled = false;
        upperEnvelopeSensorOccupied = false;

        if (pendingUpperEnvelopeMesh)
        {
            Destroy(pendingUpperEnvelopeMesh);
            pendingUpperEnvelopeMesh = null;
        }

        pendingUpperEnvelopeMeshReady = false;
        nextPendingUpperEnvelopeBuildRetryTime = 0f;

    }



    // ================================================================
    // References / radius
    // ================================================================

    private void ResolveReferences()
    {
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

        if (!inSubjectCollider &&
            slopeCore)
        {
            inSubjectCollider =
                slopeCore.GetComponent<
                    Collider>();
        }

        if (!ballVisualEqualizer &&
            ballVisualEqualizerCollider)
        {
            ballVisualEqualizer =
                ballVisualEqualizerCollider.attachedRigidbody;
        }

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

        if (!ballVisualEqualizerCollider &&
            ballVisualEqualizer)
        {
            ballVisualEqualizerCollider =
                ballVisualEqualizer.GetComponent<
                    SphereCollider>();
        }
    }


    private bool ReferencesValid()
    {
        return
            slopeCore &&
            correspondSubject &&
            ballVisualEqualizer &&
            ballVisualEqualizerCollider;
    }


    private float ResolveEqualizerWorldRadius()
    {
        if (!ballVisualEqualizerCollider)
            return 0.5f;

        Vector3 scale =
            ballVisualEqualizerCollider.transform.lossyScale;

        float maximumScale =
            Mathf.Max(
                Mathf.Abs(scale.x),
                Mathf.Abs(scale.y),
                Mathf.Abs(scale.z));

        return
            ballVisualEqualizerCollider.radius *
            maximumScale;
    }


    private float ResolveSlopeCoreWorldRadius()
    {
        if (!slopeCore)
            return 0.5f;

        SphereCollider sphere =
            slopeCore.GetComponent<SphereCollider>();

        if (!sphere)
            return 0.5f;

        Vector3 scale =
            sphere.transform.lossyScale;

        float maximumScale =
            Mathf.Max(
                Mathf.Abs(scale.x),
                Mathf.Abs(scale.y),
                Mathf.Abs(scale.z));

        return
            sphere.radius *
            maximumScale;
    }


    // ================================================================
    // Stable oscillation frame for BallVisualEqualizerSync
    // ================================================================

    /// <summary>
    /// Release時点のSlopeStickCore連続Spline frameをVisual座標で返します。
    ///
    /// この取得ではsourceEnergyAxisVisualをNormalの向き決定に使用しません。
    /// ArcSlab -> Stairwayの特殊遷移でcaller axisへTransport成分が混ざっても、
    /// Stable-Nは実際のSpline surface normalを権威として使います。
    /// </summary>
    public bool TryGetReleaseSurfaceFrameVisual(
        out Vector3 tangentVisual,
        out Vector3 normalVisual)
    {
        ResolveReferences();

        tangentVisual =
            Vector3.zero;

        normalVisual =
            Vector3.zero;

        if (!slopeCore ||
            !correspondSubject)
        {
            return false;
        }

        float releaseProgress01 =
            Mathf.Clamp01(
                slopeCore.BallVisualSlopeProgress01);

        if (!slopeCore.TryEvaluateBallVisualSectionFramePhysics(
                releaseProgress01,
                out _,
                out Vector3 tangentPhysics,
                out Vector3 normalPhysics))
        {
            return false;
        }

        if (tangentPhysics.sqrMagnitude <= 0.000001f ||
            normalPhysics.sqrMagnitude <= 0.000001f)
        {
            return false;
        }

        tangentPhysics.Normalize();

        normalPhysics =
            Vector3.ProjectOnPlane(
                normalPhysics,
                tangentPhysics);

        if (normalPhysics.sqrMagnitude <= 0.000001f)
            return false;

        normalPhysics.Normalize();

        // Re-orthogonalize T against the native surface normal.
        tangentPhysics =
            Vector3.ProjectOnPlane(
                tangentPhysics,
                normalPhysics);

        if (tangentPhysics.sqrMagnitude <= 0.000001f)
            return false;

        tangentPhysics.Normalize();

        tangentVisual =
            correspondSubject.MapDirection(
                tangentPhysics);

        normalVisual =
            correspondSubject.MapDirection(
                normalPhysics);

        if (tangentVisual.sqrMagnitude <= 0.000001f ||
            normalVisual.sqrMagnitude <= 0.000001f)
        {
            tangentVisual = Vector3.zero;
            normalVisual = Vector3.zero;
            return false;
        }

        tangentVisual.Normalize();

        normalVisual =
            Vector3.ProjectOnPlane(
                normalVisual,
                tangentVisual);

        if (normalVisual.sqrMagnitude <= 0.000001f)
        {
            tangentVisual = Vector3.zero;
            normalVisual = Vector3.zero;
            return false;
        }

        normalVisual.Normalize();

        tangentVisual =
            Vector3.ProjectOnPlane(
                tangentVisual,
                normalVisual);

        if (tangentVisual.sqrMagnitude <= 0.000001f)
        {
            tangentVisual = Vector3.zero;
            normalVisual = Vector3.zero;
            return false;
        }

        tangentVisual.Normalize();
        return true;
    }


    /// <summary>
    /// 最新Envelopeを生成したSlope全体のTangent/NormalをVisual座標で返します。
    /// 局所Mesh三角形のNormalではなく、Build時にGram-Schmidtで確定した
    /// 連続なSlope frameです。Equalizerの減衰エネルギー基底に使用します。
    /// </summary>
    public bool TryGetLatestOscillationFrameVisual(
        out Vector3 tangentVisual,
        out Vector3 normalVisual)
    {
        tangentVisual =
            Vector3.zero;

        normalVisual =
            Vector3.zero;

        if (!latestEnvelopeGeometryCached ||
            !correspondSubject)
        {
            return false;
        }

        tangentVisual =
            correspondSubject.MapDirection(
                cachedAxisPhysics);

        normalVisual =
            correspondSubject.MapDirection(
                cachedSlopeNormalPhysics);

        if (tangentVisual.sqrMagnitude <= 0.000001f ||
            normalVisual.sqrMagnitude <= 0.000001f)
        {
            tangentVisual = Vector3.zero;
            normalVisual = Vector3.zero;
            return false;
        }

        tangentVisual.Normalize();

        normalVisual =
            Vector3.ProjectOnPlane(
                normalVisual,
                tangentVisual);

        if (normalVisual.sqrMagnitude <= 0.000001f)
        {
            tangentVisual = Vector3.zero;
            normalVisual = Vector3.zero;
            return false;
        }

        normalVisual.Normalize();

        Vector3 orientationAxis =
            sourceEnergyAxisVisual;

        if (orientationAxis.sqrMagnitude <=
            0.000001f)
        {
            orientationAxis =
                correspondSubject.MapDirection(
                    Vector3.up);
        }

        if (orientationAxis.sqrMagnitude > 0.000001f)
        {
            orientationAxis.Normalize();

            if (Vector3.Dot(
                    normalVisual,
                    orientationAxis) < 0f)
            {
                normalVisual =
                    -normalVisual;
            }
        }

        return true;
    }
    // ================================================================
    // Debug
    // ================================================================

    private void OnDrawGizmosSelected()
    {
        if (!envelopeBuilt)
            return;

        Gizmos.DrawSphere(
            correspondSubject.MapPoint(
                capturedEntryPhysics),
            0.08f);

        Gizmos.DrawSphere(
            correspondSubject.MapPoint(
                capturedLimitPhysics),
            0.08f);
    }

    // Legacy 3-argument overload.
    public bool ArmFromBallVisualEnergy(
        float energyJoule,
        float canonicalReferenceHeightValue,
        Vector3 equalizerLaunchVelocityVisual)
    {
        Vector3 inferredAxis =
            equalizerLaunchVelocityVisual;

        if (inferredAxis.sqrMagnitude <=
            0.000001f)
        {
            inferredAxis =
                Vector3.up;
        }

        return ArmFromBallVisualEnergy(
            energyJoule,
            canonicalReferenceHeightValue,
            equalizerLaunchVelocityVisual,
            inferredAxis);
    }


    // Canonical BallVisualSlopeDrive / Equalizer handoff.
    public bool ArmFromBallVisualEnergy(
        float energyJoule,
        float canonicalReferenceHeightValue,
        Vector3 equalizerLaunchVelocityVisual,
        Vector3 sourceEnergyAxisVisualValue,
        float initialCanonicalEnergyRatio = 1f)
    {
        ResolveReferences();

        if (!ReferencesValid())
        {
            Debug.LogWarning(
                "[ENVELOPE] Spline references are not valid.",
                this);

            return false;
        }

        float safeEnergy =
            Mathf.Max(
                0f,
                energyJoule);

        float safeReferenceHeight =
            Mathf.Max(
                0f,
                canonicalReferenceHeightValue);

        if (safeEnergy <=
            0.000001f)
        {
            Debug.LogWarning(
                "[ENVELOPE] Source Energy E0 is zero.",
                this);

            return false;
        }

        if (safeReferenceHeight <=
            0.000001f)
        {
            Debug.LogWarning(
                "[ENVELOPE] Stable-N reference height H0 is zero.",
                this);

            return false;
        }

        if (equalizerLaunchVelocityVisual.sqrMagnitude <=
            0.000001f)
        {
            Debug.LogWarning(
                "[ENVELOPE] Equalizer launch velocity is zero.",
                this);

            return false;
        }

        Vector3 sourceAxis =
            sourceEnergyAxisVisualValue;

        if (sourceAxis.sqrMagnitude <=
            0.000001f)
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

        capturedEqualizerLaunchVelocityVisual =
            equalizerLaunchVelocityVisual;

        PrepareForNextEnvelopeGeneration();

        presentationWaveIndex = 0;
        presentationReleaseFixedTime = Time.fixedTime;
        presentationCurrentCeilingR = 0f;
        presentationCurrentLegacyTravelR = 0f;
        presentationCurrentResolvedTravelR = 0f;
        presentationCurrentLossHnR = 0f;
        presentationCurrentLegacyEnergyRetention01 = 1f;
        presentationCurrentLegacyTimeRetention01 = 1f;
        presentationCurrentRawLegacyRetention01 = 1f;
        presentationCurrentAppliedRetention01 = 1f;

        sourceEnergyJoule =
            safeEnergy;

        canonicalReferenceHeight =
            safeReferenceHeight;

        sourceEnergyAxisVisual =
            sourceAxis;

        canonicalEnergyRatio =
            Mathf.Clamp01(
                initialCanonicalEnergyRatio);

        float mass =
            Mathf.Max(
                0.0001f,
                ballVisualEqualizer.mass);

        canonicalNormalAcceleration =
            safeEnergy /
            Mathf.Max(
                0.000001f,
                mass *
                safeReferenceHeight);

        equalizerLaunchSpeed =
            equalizerLaunchVelocityVisual.magnitude;

        // Kept as a diagnostic only.  Canonical geometry does not derive H0
        // from World-Y.
        equalizerVerticalLaunchSpeed =
            Vector3.Dot(
                equalizerLaunchVelocityVisual,
                Vector3.up);

        entryApexHeight =
            safeReferenceHeight;

        armed =
            true;

        envelopeBuilt =
            false;

        return TryBuildEnvelopeIfReady();
    }
    public void SetCanonicalDampingEnergyRatio(
        float energyRatio)
    {
        SetCanonicalEnergyRatio(energyRatio);
    }


    public float CanonicalDampingEnergyRatio =>
        canonicalEnergyRatio;


    public void SetCanonicalEnergyRatio(
        float energyRatio)
    {
        // One release may only lose canonical energy. Solver noise must never
        // re-expand the Upper Envelope.
        float requested = Mathf.Clamp01(energyRatio);
        float next = Mathf.Min(canonicalEnergyRatio, requested);

        if (Mathf.Abs(next - canonicalEnergyRatio) <= 0.0005f)
            return;

        canonicalEnergyRatio = next;

        UpdatePresentationRuntimeDiagnostics(
            Mathf.Max(0f, Time.fixedTime - presentationReleaseFixedTime));

        pendingCanonicalGeometryRebuild = true;
    }


    public void NotifyCanonicalUpperPeak()
    {
        if (spatialWaveAuthorityActive)
        {
            Debug.Log(
                $"[ENVELOPE PHYSICAL UPPER OBSERVED] " +
                $"authority=SpatialProgress " +
                $"wave={presentationWaveIndex + 1}/{spatialWaveAuthorityCount} " +
                $"domainP={spatialWaveAuthorityProgress01:F4}",
                this);
            return;
        }

        presentationWaveIndex =
            Mathf.Max(
                0,
                presentationWaveIndex + 1);

        float elapsed =
            Mathf.Max(
                0f,
                Time.fixedTime - presentationReleaseFixedTime);

        UpdatePresentationRuntimeDiagnostics(elapsed);
        pendingCanonicalGeometryRebuild = true;

        Debug.Log(
            $"[ENVELOPE PRESENTATION PEAK] " +
            $"nextWave={presentationWaveIndex + 1} " +
            $"elapsed={elapsed:F4}s " +
            $"ceiling={presentationCurrentCeilingR:F3}R " +
            $"legacy={presentationCurrentLegacyTravelR:F3}R " +
            $"resolved={presentationCurrentResolvedTravelR:F3}R " +
            $"Hn={presentationCurrentLossHnR:F3}R " +
            $"energyRet={presentationCurrentLegacyEnergyRetention01:F4} " +
            $"timeRet={presentationCurrentLegacyTimeRetention01:F4} " +
            $"rawRet={presentationCurrentRawLegacyRetention01:F4} " +
            $"appliedRet={presentationCurrentAppliedRetention01:F4} " +
            $"lossInfluence={waveTimeDecayInfluence:F3} " +
            $"epsilon={canonicalEnergyRatio:F6}",
            this);
    }


    public void ResetPresentationWaveState()
    {
        presentationWaveIndex = 0;
        presentationReleaseFixedTime = Time.fixedTime;

        UpdatePresentationRuntimeDiagnostics(0f);
        pendingCanonicalGeometryRebuild = true;
    }


    public int PresentationWaveIndex =>
        presentationWaveIndex;

    public float PresentationCurrentCeilingR =>
        presentationCurrentCeilingR;

    public float PresentationCurrentLegacyTravelR =>
        presentationCurrentLegacyTravelR;

    public float PresentationCurrentResolvedTravelR =>
        presentationCurrentResolvedTravelR;

    public float PresentationCurrentLossHnR =>
        presentationCurrentLossHnR;

    public float PresentationCurrentRawLegacyRetention01 =>
        presentationCurrentRawLegacyRetention01;

    public float PresentationCurrentAppliedRetention01 =>
        presentationCurrentAppliedRetention01;


    public void CommitDeferredCanonicalGeometryUpdate()
    {
        // Contact callbacks only request an update. FixedUpdate prepares the
        // standby mesh and swaps it during the next Upper-Solid OFF interval.
        if (!Application.isPlaying ||
            !envelopeBuilt ||
            !latestEnvelopeGeometryCached)
        {
            return;
        }

        pendingCanonicalGeometryRebuild = true;
    }
    private bool TryBuildEnvelopeIfReady()
    {
        if (!armed)
            return false;

        if (envelopeBuilt)
            return true;

        if (!ReferencesValid())
            return false;

        if (!slopeCore.BallVisualHasActiveSlopeFrame)
            return false;

        if (!slopeCore.BallVisualIncidentReady)
            return false;

        // Exact Target must still be ahead of the current Spline progress.
        if (!(slopeCore.slopeProgressErrorPercent < 0f))
            return false;

        if (!slopeCore.TryGetBallVisualTargetProgressCenterPhysics(
                out _))
        {
            return false;
        }

        BuildNegativeEnvelope();

        return envelopeBuilt;
    }

}


/// <summary>
/// BallVisualEqualizer側のOnCollisionEnterで
///
/// collision.collider.GetComponent<BallVisualEnvelopeSurfaceMarker>()
///
/// を調べればEnvelope衝突か判定できます。
///
/// 力は一切加えません。
/// </summary>
public sealed class BallVisualEnvelopeSurfaceMarker
    : MonoBehaviour
{

   /* */


}