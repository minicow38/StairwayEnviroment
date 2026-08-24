using System.Collections.Generic;
using UnityEngine;


/// <summary>
/// BallVisualEqualizer専用のUpper/Lower Colliderを生成します。
///
/// 有効な物理仕様は次の1系統だけです。
///   Lower : SlopeStickCoreの連続Spline surface
///   Upper : S(T) * R + A(t)
///   A(t)  : H0 * epsilon * exp(-gamma * t)
///
/// Inspectorで指定する周期Tを唯一の時間スケールとして、
/// Upper/Lowerの実接触間隔 T/2 に対して必要なCollider高さと
/// Stable-N速度/加速度を自動解決します。
///
/// S(T) は周期Tと、以前成功した300-400m/s^2帯の中心値を基準に
/// [2R, 4R] の範囲で自動選択します。短周期では2R側へ寄せて入口衝撃を減らし、
/// 長周期では必要に応じて4Rまで広げます。
///
/// gammaはReleaseからExact LimitまでのSpline移動時間から一度だけ求めます。
/// First Contact方式選択、Curved Offset、World-Y補正は使用しません。
/// SlopeStickCore / CorrespondSubjectはREAD ONLYです。
/// </summary>
[DisallowMultipleComponent]
public sealed class BallVisualNegativeEnvelopeCollider : MonoBehaviour
{
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
    // Periodic contact master
    // 周期TだけをInspector入力とし、Collider高さ・必要速度・位相加速度を
    // すべて同じ幾何モデルから自動解決する。
    // ================================================================

    [Header("Periodic Contact - Master")]
    [Tooltip(
        "Upper -> Lower -> Upper の1周期 T[s]。反対側Colliderへの接触は T/2 ごとです。\n" +
        "Collider高さ(R倍率)、必要Stable-N速度、Phase Drive加速度は自動計算されます。")]
    [Min(0.04f)]
    [SerializeField]
    private float targetContactPeriodSeconds = 0.10f;

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


    private Transform generatedRoot;
    private Mesh generatedMesh;

    // 最新Envelopeだけを保持する。次の生成前に旧Rootは破棄する。
    private readonly List<Transform> generatedEnvelopeRoots =
        new List<Transform>();

    // 現在「最後に生成された」Envelopeだけをリアルタイム更新するための参照。
    private Transform generatedMeshTransform;
    private MeshFilter generatedMeshFilter;
    private MeshCollider generatedMeshCollider;

    // Equalizer専用Lower Boundary。実階段Colliderではなく、
    // SlopeStickCoreの連続Spline surfaceを物理境界として使用する。
    private Transform generatedLowerGuideTransform;
    private MeshFilter generatedLowerGuideFilter;
    private MeshCollider generatedLowerGuideCollider;
    private Mesh generatedLowerGuideMesh;

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

    [Tooltip("Canonical初速から周期Tへ合わせるための初期Phase加速度[m/s^2]。")]
    [SerializeField]
    private float resolvedReleasePhaseAcceleration;
    // Canonical energy may change inside a collision callback. MeshCollider recook
    // is deferred until the Equalizer has left both boundaries.
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
    private float lastTargetContactPeriodSeconds;


    // ================================================================
    // Unity
    // ================================================================

    private void Awake()
    {
        ResolveReferences();
        CaptureLiveSettingsSnapshot();
    }
    private void FixedUpdate()
    {
        TryBuildEnvelopeIfReady();
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

        RebuildLatestGeneratedMesh();
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
               // $"T={resolvedContactPeriodSeconds:F4}s " +
                //$"halfT={resolvedHalfPeriodSeconds:F4}s " +
                $"clearanceScale={resolvedEnvelopeRadiusClearanceScale:F4}R " +
                $"releaseSpan={resolvedReleaseCenterTravelDistance:F4}m " +
                $"targetVN={resolvedReleaseTargetNormalSpeed:F4}m/s " +
                $"phaseA0={resolvedReleasePhaseAcceleration:F4}m/s2 " +
                $"E0={sourceEnergyJoule:F4}J " +
                $"H0={canonicalReferenceHeight:F4}m " +
                $"epsilon={canonicalEnergyRatio:F4}",
                this);
        }
    }
    // ================================================================
// S(E,r,T) - Exponential travel integral
// ================================================================
    private float ResolveDecayTravelIntegralSeconds(
        float decayRatePerSecondValue,
        float timeSeconds)
    {
        float gamma =
            Mathf.Max(
                0f,
                decayRatePerSecondValue);

        float t =
            Mathf.Max(
                0f,
                timeSeconds);

        // gamma -> 0 の極限
        //
        // (1 - exp(-gamma*t)) / gamma -> t
        //
        if (gamma <= 0.000001f)
            return t;

        return
            (1f - Mathf.Exp(-gamma * t)) /
            gamma;
    }
    private void ResolvePeriodicContactPlan2(
    float A0,
    float equalizerRadius)
{
    float fixedDt =
        Mathf.Max(
            0.0001f,
            Time.fixedDeltaTime);

    float minimumPeriod =
        fixedDt *
        MinimumPeriodicHalfCycleFixedSteps *
        2f;

    resolvedContactPeriodSeconds =
        Mathf.Max(
            minimumPeriod,
            Mathf.Max(
                0.0001f,
                targetContactPeriodSeconds));

    resolvedHalfPeriodSeconds =
        resolvedContactPeriodSeconds * 0.5f;


    // ============================================================
    // Geometry
    // ============================================================

    float radius =
        Mathf.Max(
            0.0001f,
            equalizerRadius);

    float mass =
        ballVisualEqualizer
            ? Mathf.Max(
                0.0001f,
                ballVisualEqualizer.mass)
            : 1f;

    float halfT =
        Mathf.Max(
            fixedDt,
            resolvedHalfPeriodSeconds);


    // ============================================================
    // E : current canonical energy
    //
    // E = E0 * epsilon
    // ============================================================

    float epsilon =
        Mathf.Clamp01(
            canonicalEnergyRatio);

    float effectiveEnergy =
        Mathf.Max(
            0f,
            sourceEnergyJoule) *
        epsilon;


    // ============================================================
    // v0 = sqrt(2E/m)
    // ============================================================

    float canonicalLaunchNormalSpeed =
        effectiveEnergy > 0f
            ? Mathf.Sqrt(
                2f *
                effectiveEnergy /
                mass)
            : 0f;


    // ============================================================
    // r : exponential decay coefficient
    //
    // 現在の実装:
    //
    //     exp(-gamma*t)
    //
    // 数学上の
    //
    //     exp(r*t)
    //
    // と比較すると
    //
    //     r = -gamma
    //
    // ============================================================

    float gamma =
        Mathf.Max(
            0f,
            decayRatePerSecond);

    float r =
        -gamma;


    // ============================================================
    // Integral:
    //
    // I(r,T)
    //     = integral[0,T/2] exp(r*t) dt
    //
    // gamma表記では
    //
    //     = (1-exp(-gamma*T/2))/gamma
    //
    // 単位は seconds
    // ============================================================

    float decayTravelIntegral =
        ResolveDecayTravelIntegralSeconds(
            gamma,
            halfT);


    // ============================================================
    // Exponentially decayed natural travel
    //
    // D_ErT = v0 * I(r,T)
    // ============================================================

    float exponentialNormalTravel =
        canonicalLaunchNormalSpeed *
        decayTravelIntegral;


    // ============================================================
    // H0から現在Energyに対応するnormal accelerationを解く
    //
    // E = m*a*H0
    //
    // a = E/(m*H0)
    // ============================================================

    float safeH0 =
        Mathf.Max(
            0.0001f,
            canonicalReferenceHeight);

    float effectiveNormalAcceleration =
        effectiveEnergy /
        Mathf.Max(
            0.000001f,
            mass * safeH0);


    // ============================================================
    // A(T/2)
    //
    // Upper Envelope自身も指数減衰しているので、
    // Sを解く時にはrelease時のA0ではなく
    // 次接触予定時刻T/2のAmplitudeを使う。
    // ============================================================

    float amplitudeAtHalfT =
        EvaluateEnvelopeAmplitudeAtTime(
            A0,
            gamma,
            halfT);


    // ============================================================
    // Reference phase contribution
    //
    // 350 m/s^2 は既存の内部基準。
    // Inspector parameterにはしない。
    // ============================================================

    float referenceTotalTowardAcceleration =
        PeriodicHeightReferencePhaseAcceleration -
        Mathf.Max(
            0f,
            effectiveNormalAcceleration);


    // ============================================================
    // D(E,r,T)
    //
    //     natural exponential travel
    //          +
    //     phase geometry contribution
    // ============================================================

    float periodPlannedCenterTravel =
        exponentialNormalTravel +
        0.5f *
        referenceTotalTowardAcceleration *
        halfT *
        halfT;

    periodPlannedCenterTravel =
        Mathf.Max(
            minimumFreeAmplitude,
            periodPlannedCenterTravel);


    // ============================================================
    // Solve S(E,r,T)
    //
    // Upper:
    //
    //     H = S*R + A(T/2)
    //
    // Equalizer中心の自由距離:
    //
    //     D = (S - 2)R + A(T/2)
    //
    // したがって
    //
    //     S = 2 + (D - A(T/2))/R
    //
    // ============================================================

    float rawScale =
        MinimumPeriodicRadiusClearanceScale +
        (
            periodPlannedCenterTravel -
            amplitudeAtHalfT
        ) /
        radius;


    // ============================================================
    // Current geometry invariant
    //
    // 2R <= S <= 4R
    // ============================================================

    resolvedEnvelopeRadiusClearanceScale =
        Mathf.Clamp(
            rawScale,
            MinimumPeriodicRadiusClearanceScale,
            MaximumPeriodicRadiusClearanceScale);


    // ============================================================
    // Release時点の実Upper高さ
    // ============================================================

    float releaseAmplitude =
        EvaluateEnvelopeAmplitudeAtTime(
            A0,
            gamma,
            0f);

    resolvedReleaseSurfaceClearance =
        resolvedEnvelopeRadiusClearanceScale *
        radius +
        releaseAmplitude;


    resolvedReleaseCenterTravelDistance =
        Mathf.Max(
            minimumFreeAmplitude,
            resolvedReleaseSurfaceClearance -
            2f * radius);


    // ============================================================
    // Target initial normal speed
    //
    // D = v0*I - 1/2*aN*h^2
    //
    // ->
    //
    // v0 =
    // (D + 1/2*aN*h^2) / I
    // ============================================================

    resolvedReleaseTargetNormalSpeed =
        (
            resolvedReleaseCenterTravelDistance +
            0.5f *
            Mathf.Max(
                0f,
                effectiveNormalAcceleration) *
            halfT *
            halfT
        ) /
        Mathf.Max(
            0.0001f,
            decayTravelIntegral);


    // ============================================================
    // Phase acceleration required after exponential natural travel
    // ============================================================

    float requiredTotalNormalAcceleration =
        2f *
        (
            resolvedReleaseCenterTravelDistance -
            exponentialNormalTravel
        ) /
        Mathf.Max(
            0.000001f,
            halfT * halfT);


    resolvedReleasePhaseAcceleration =
        requiredTotalNormalAcceleration +
        Mathf.Max(
            0f,
            effectiveNormalAcceleration);


    // ============================================================
    // Diagnostic
    // ============================================================

    Debug.Log(
        $"[ENVELOPE S(E,r,T)] " +
        $"E={effectiveEnergy:F4}J " +
        $"epsilon={epsilon:F4} " +
        $"r={r:F4}/s " +
        $"gamma={gamma:F4}/s " +
        $"T={resolvedContactPeriodSeconds:F5}s " +
        $"halfT={halfT:F5}s " +
        $"vN0={canonicalLaunchNormalSpeed:F4}m/s " +
        $"Ahalf={amplitudeAtHalfT:F4}m " +
        $"D={periodPlannedCenterTravel:F4}m " +
        $"Sraw={rawScale:F4}R " +
        $"S={resolvedEnvelopeRadiusClearanceScale:F4}R " +
        $"UpperRelease={resolvedReleaseSurfaceClearance:F4}m",
        this);
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
    private bool LiveSettingsChanged()
    {
        if (!liveSettingsSnapshotValid)
            return true;

        return
            segmentCount != lastSegmentCount ||
            Mathf.Abs(envelopeWidth - lastEnvelopeWidth) > 0.00001f ||
            Mathf.Abs(
                targetContactPeriodSeconds -
                lastTargetContactPeriodSeconds) > 0.00001f;
    }
    private void CaptureLiveSettingsSnapshot()
    {
        lastSegmentCount = segmentCount;
        lastEnvelopeWidth = envelopeWidth;
        lastTargetContactPeriodSeconds =
            targetContactPeriodSeconds;
        liveSettingsSnapshotValid = true;
    }
    private void RebuildLatestGeneratedMesh()
    {
        bool lowerGuideGeometryChanged =
            !liveSettingsSnapshotValid ||
            lastSegmentCount != segmentCount ||
            !Mathf.Approximately(lastEnvelopeWidth, envelopeWidth) ||
            !generatedLowerGuideCollider;

        if (!latestEnvelopeGeometryCached ||
            !generatedMeshTransform ||
            !generatedMeshFilter ||
            !generatedMeshCollider)
        {
            CaptureLiveSettingsSnapshot();
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
            Debug.LogWarning(
                "[ENVELOPE LIVE UPDATE] Full Spline mesh build failed.",
                this);
            CaptureLiveSettingsSnapshot();
            return;
        }

        Mesh oldMesh = generatedMesh;
        generatedMeshFilter.sharedMesh = newMesh;

        bool colliderWasEnabled = generatedMeshCollider.enabled;
        generatedMeshCollider.enabled = false;
        generatedMeshCollider.sharedMesh = null;
        generatedMeshCollider.sharedMesh = newMesh;
        generatedMeshCollider.enabled = colliderWasEnabled;
        generatedMesh = newMesh;

        if (oldMesh && oldMesh != newMesh)
            Destroy(oldMesh);

        if (lowerGuideGeometryChanged)
            RebuildCurrentLowerGuideMesh();

        Physics.SyncTransforms();
        CaptureLiveSettingsSnapshot();

        Debug.Log(
            $"[ENVELOPE LIVE UPDATED] " +
           // $"T={resolvedContactPeriodSeconds:F4}s " +
            //$"halfT={resolvedHalfPeriodSeconds:F4}s " +
            $"epsilon={canonicalEnergyRatio:F4} " +
            $"gamma={cachedGamma:F4}/s " +
            $"clearanceScale={resolvedEnvelopeRadiusClearanceScale:F4}R " +
            $"releaseSpan={resolvedReleaseCenterTravelDistance:F4}m " +
            $"targetVN={resolvedReleaseTargetNormalSpeed:F4}m/s " +
            $"width={envelopeWidth:F3} " +
            $"segments={segmentCount}",
            this);
    }


    private void RebuildCurrentLowerGuideMesh()
    {
        if (!generatedLowerGuideTransform ||
            !generatedLowerGuideFilter ||
            !generatedLowerGuideCollider)
        {
            return;
        }

        Mesh newLowerMesh =
            BuildFullSplineLowerGuideMeshAsset(
                generatedLowerGuideTransform);

        if (!newLowerMesh)
        {
            Debug.LogWarning(
                "[EQUALIZER LOWER GUIDE] Live rebuild failed; previous guide is kept.",
                this);
            return;
        }

        Mesh oldLowerMesh =
            generatedLowerGuideMesh;

        generatedLowerGuideFilter.sharedMesh =
            newLowerMesh;

        bool colliderWasEnabled =
            generatedLowerGuideCollider.enabled;

        generatedLowerGuideCollider.enabled = false;
        generatedLowerGuideCollider.sharedMesh = null;
        generatedLowerGuideCollider.sharedMesh = newLowerMesh;
        generatedLowerGuideCollider.enabled = colliderWasEnabled;

        generatedLowerGuideMesh = newLowerMesh;

        if (oldLowerMesh &&
            oldLowerMesh != newLowerMesh)
        {
            Destroy(oldLowerMesh);
        }
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
        float epsilon = Mathf.Clamp01(canonicalEnergyRatio);

        float amplitude =
            Mathf.Max(0f, A0) *
            epsilon *
            Mathf.Exp(
                -Mathf.Max(0f, decayRatePerSecondValue) *
                Mathf.Max(0f, timeSeconds));

        return Mathf.Max(minimumFreeAmplitude, amplitude);
    }
    private void ResolvePeriodicContactPlan(
        float A0,
        float equalizerRadius)
    {
        float fixedDt =
            Mathf.Max(
                0.0001f,
                Time.fixedDeltaTime);

        float minimumPeriod =
            fixedDt *
            MinimumPeriodicHalfCycleFixedSteps *
            2f;

        resolvedContactPeriodSeconds =
            Mathf.Max(
                minimumPeriod,
                Mathf.Max(0.0001f, targetContactPeriodSeconds));

        resolvedHalfPeriodSeconds =
            resolvedContactPeriodSeconds * 0.5f;

        float radius =
            Mathf.Max(
                0.0001f,
                equalizerRadius);

        float mass =
            ballVisualEqualizer
                ? Mathf.Max(0.0001f, ballVisualEqualizer.mass)
                : 1f;

        float canonicalLaunchNormalSpeed =
            sourceEnergyJoule > 0f
                ? Mathf.Sqrt(
                    2f *
                    Mathf.Max(0f, sourceEnergyJoule) /
                    mass)
                : 0f;

        float halfT =
            Mathf.Max(
                fixedDt,
                resolvedHalfPeriodSeconds);

        // 周期Tから高さを一意に決める。
        // 以前減衰振幅が成立したPhase加速度帯の中心 350m/s^2 を
        // 「高さを決めるためだけの基準」とし、T/2で進む中心距離を求める。
        //
        // Phase correction a_phi を350とするとStable-N総加速度は
        //     a_total = a_phi - aN
        // なので
        //     D_T = vN0*h + 1/2*a_total*h^2
        //
        // 実際の制御加速度はこの値に固定せず、Equalizer側が実Collider距離から
        // a = 2(s-v*tau)/tau^2 を毎FixedUpdate解く。
        float referenceTotalTowardAcceleration =
            PeriodicHeightReferencePhaseAcceleration -
            Mathf.Max(0f, canonicalNormalAcceleration);

        float periodPlannedCenterTravel =
            canonicalLaunchNormalSpeed * halfT +
            0.5f *
            referenceTotalTowardAcceleration *
            halfT * halfT;

        periodPlannedCenterTravel =
            Mathf.Max(
                minimumFreeAmplitude,
                periodPlannedCenterTravel);

        // Upper surface offset = S*R + A0
        // Equalizer中心のLower->Upper自由距離は、おおよそ
        //     D = (S - 2)R + A0
        // よって D ~= D_T となるS(T)を逆算する。
        float rawScale =
            MinimumPeriodicRadiusClearanceScale +
            (periodPlannedCenterTravel - Mathf.Max(0f, A0)) /
            radius;

        resolvedEnvelopeRadiusClearanceScale =
            Mathf.Clamp(
                rawScale,
                MinimumPeriodicRadiusClearanceScale,
                MaximumPeriodicRadiusClearanceScale);

        resolvedReleaseSurfaceClearance =
            resolvedEnvelopeRadiusClearanceScale *
            radius +
            Mathf.Max(0f, A0);

        resolvedReleaseCenterTravelDistance =
            Mathf.Max(
                minimumFreeAmplitude,
                resolvedReleaseSurfaceClearance -
                2f * radius);

        // D = v0*h - 1/2*aN*h^2
        // -> v0 = D/h + 1/2*aN*h
        // これが周期Tに対する理論初速。実運動はEqualizer側のSphereCastで
        // 現在の実距離を測り、同じT/2へ収束させる。
        resolvedReleaseTargetNormalSpeed =
            resolvedReleaseCenterTravelDistance / halfT +
            0.5f *
            Mathf.Max(0f, canonicalNormalAcceleration) *
            halfT;

        float requiredTotalNormalAcceleration =
            2f *
            (resolvedReleaseCenterTravelDistance -
             canonicalLaunchNormalSpeed * halfT) /
            (halfT * halfT);

        resolvedReleasePhaseAcceleration =
            requiredTotalNormalAcceleration +
            Mathf.Max(0f, canonicalNormalAcceleration);
    }


    private float ResolveEnvelopeClearance(
        float equalizerRadius,
        float amplitude)
    {
        return
            Mathf.Max(0f, equalizerRadius) *
            resolvedEnvelopeRadiusClearanceScale +
            Mathf.Max(0f, amplitude);
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

        meshObject.layer = gameObject.layer;

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

        generatedMeshFilter = meshObject.AddComponent<MeshFilter>();
        generatedMeshFilter.sharedMesh = mesh;

        generatedMeshCollider = meshObject.AddComponent<MeshCollider>();
        generatedMeshCollider.sharedMesh = mesh;
        generatedMeshCollider.convex = false;
        generatedMeshCollider.isTrigger = false;

        meshObject.AddComponent<BallVisualEnvelopeSurfaceMarker>();

        ConfigureEqualizerOnlyBoundaryCollider(
            generatedMeshCollider);

        generatedMesh = mesh;

        // Upperと同じSpline frameから、Equalizer専用の連続Lower境界を作る。
        // Lowerは振幅を持たず、surfacePhysicsそのものを使用するため
        // n_T ~= 0 となり、T transportを非貫通のために潰す必要がない。
        if (!CreateFullSplineLowerGuideMesh())
        {
            Debug.LogError(
                "[EQUALIZER LOWER GUIDE] Failed to build dedicated lower boundary.",
                this);

            ClearAllGeneratedEnvelopeRoots();
            return false;
        }
        CaptureLiveSettingsSnapshot();

        Debug.Log(
            $"[EQUALIZER LOWER GUIDE CREATED] " +
            $"vertices={(generatedLowerGuideMesh ? generatedLowerGuideMesh.vertexCount : 0)} " +
            $"width={envelopeWidth:F3} " +
            $"segments={segmentCount}",
            this);

        return true;
    }


    private bool CreateFullSplineLowerGuideMesh()
    {
        if (!generatedRoot)
            return false;

        GameObject lowerObject =
            new GameObject("EqualizerLowerGuideMesh");

        lowerObject.layer = gameObject.layer;

        generatedLowerGuideTransform = lowerObject.transform;
        generatedLowerGuideTransform.SetParent(generatedRoot, false);
        generatedLowerGuideTransform.localPosition = Vector3.zero;
        generatedLowerGuideTransform.localRotation = Quaternion.identity;
        generatedLowerGuideTransform.localScale = Vector3.one;

        Mesh lowerMesh =
            BuildFullSplineLowerGuideMeshAsset(
                generatedLowerGuideTransform);

        if (!lowerMesh)
        {
            Destroy(lowerObject);
            generatedLowerGuideTransform = null;
            return false;
        }

        generatedLowerGuideFilter =
            lowerObject.AddComponent<MeshFilter>();
        generatedLowerGuideFilter.sharedMesh = lowerMesh;

        generatedLowerGuideCollider =
            lowerObject.AddComponent<MeshCollider>();
        generatedLowerGuideCollider.sharedMesh = lowerMesh;
        generatedLowerGuideCollider.convex = false;
        generatedLowerGuideCollider.isTrigger = false;

        // LowerGuideはEqualizer専用。BallVisual / Subject / Stageなど
        // 他のColliderには物理的な影響を与えない。
        ConfigureEqualizerOnlyBoundaryCollider(
            generatedLowerGuideCollider);

        generatedLowerGuideMesh = lowerMesh;
        return true;
    }


    private Mesh BuildFullSplineLowerGuideMeshAsset(
        Transform meshTransform)
    {
        if (!meshTransform ||
            !correspondSubject ||
            !slopeCore)
        {
            return null;
        }

        int sampleCount =
            Mathf.Max(2, segmentCount + 1);

        Vector3[] vertices =
            new Vector3[sampleCount * 2];

        int[] triangles =
            new int[(sampleCount - 1) * 6];

        float halfWidth =
            Mathf.Max(0.05f, envelopeWidth * 0.5f);

        float subjectRadius =
            ResolveSlopeCoreWorldRadius();

        const float startProgress = 0f;
        const float endProgress = 1f;

        for (int i = 0;
             i < sampleCount;
             i++)
        {
            float u =
                i / (float)(sampleCount - 1);

            float progress =
                Mathf.Lerp(
                    startProgress,
                    endProgress,
                    u);

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

            Vector3 widthAxisPhysics =
                Vector3.Cross(
                    normalPhysics,
                    tangentPhysics);

            if (widthAxisPhysics.sqrMagnitude <= 0.000001f)
                return null;

            widthAxisPhysics.Normalize();

            // surfacePhysicsは
            // centerPhysics - normalPhysics * SubjectRadius。
            // ここに追加のRオフセットを入れない。SphereCollider自身の半径が
            // LowerGuideとの接触時にEqualizer中心のClearanceを作る。
            Vector3 leftPhysics =
                surfacePhysics -
                widthAxisPhysics * halfWidth;

            Vector3 rightPhysics =
                surfacePhysics +
                widthAxisPhysics * halfWidth;

            Vector3 leftWorld =
                correspondSubject.MapPoint(leftPhysics);

            Vector3 rightWorld =
                correspondSubject.MapPoint(rightPhysics);

            vertices[i * 2 + 0] =
                meshTransform.InverseTransformPoint(leftWorld);

            vertices[i * 2 + 1] =
                meshTransform.InverseTransformPoint(rightWorld);
        }

        int ti = 0;

        for (int i = 0;
             i < sampleCount - 1;
             i++)
        {
            int a = i * 2;
            int b = i * 2 + 1;
            int c = (i + 1) * 2;
            int d = (i + 1) * 2 + 1;

            // Upper Envelopeとは逆巻き。Lowerの表面Normalを+Stable-N側へ向ける。
            triangles[ti++] = a;
            triangles[ti++] = d;
            triangles[ti++] = b;
            triangles[ti++] = a;
            triangles[ti++] = c;
            triangles[ti++] = d;
        }

        Mesh mesh = new Mesh();
        mesh.name = "EqualizerLowerGuide_FullSpline";
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        return mesh;
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
    public void RefreshEqualizerBoundaryCollisionOwnership()
    {
        ConfigureEqualizerOnlyBoundaryCollider(generatedMeshCollider);
        ConfigureEqualizerOnlyBoundaryCollider(generatedLowerGuideCollider);
    }


    public Collider CurrentUpperEnvelopeCollider =>
        generatedMeshCollider;


    public Collider CurrentLowerGuideCollider =>
        generatedLowerGuideCollider;


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
            generatedLowerGuideCollider &&
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
            generatedLowerGuideCollider &&
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


    public bool IsLowerGuideCollider(
        Collider collider)
    {
        return
            collider &&
            generatedLowerGuideCollider &&
            collider == generatedLowerGuideCollider;
    }


    public bool IsEqualizerBoundaryCollider(
        Collider collider)
    {
        return
            IsUpperEnvelopeCollider(collider) ||
            IsLowerGuideCollider(collider);
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

            float amplitude =
                EvaluateEnvelopeAmplitudeAtTime(
                    A0,
                    decayRatePerSecondValue,
                    timeSeconds);

            float clearance =
                ResolveEnvelopeClearance(
                    equalizerRadius,
                    amplitude);

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
            Mathf.Max(0.04f, targetContactPeriodSeconds);

        resolvedHalfPeriodSeconds =
            resolvedContactPeriodSeconds * 0.5f;

        resolvedEnvelopeRadiusClearanceScale =
            MinimumPeriodicRadiusClearanceScale;

        resolvedReleaseCenterTravelDistance = 0f;
        resolvedReleaseSurfaceClearance = 0f;
        resolvedReleaseTargetNormalSpeed = 0f;
        resolvedReleasePhaseAcceleration = 0f;

        pendingCanonicalGeometryRebuild = false;
        decayTimeCostSeconds = 0f;
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

                MeshCollider collider =
                    filter.GetComponent<MeshCollider>();

                if (collider)
                {
                    collider.enabled =
                        false;

                    collider.sharedMesh =
                        null;
                }

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

        generatedLowerGuideMesh =
            null;

        generatedLowerGuideTransform =
            null;

        generatedLowerGuideFilter =
            null;

        generatedLowerGuideCollider =
            null;
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
        pendingCanonicalGeometryRebuild = true;
    }


    public void CommitDeferredCanonicalGeometryUpdate()
    {
        if (!pendingCanonicalGeometryRebuild)
            return;

        pendingCanonicalGeometryRebuild = false;

        if (Application.isPlaying &&
            envelopeBuilt &&
            latestEnvelopeGeometryCached &&
            generatedMeshTransform &&
            generatedMeshFilter &&
            generatedMeshCollider)
        {
            RebuildLatestGeneratedMesh();
        }
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
