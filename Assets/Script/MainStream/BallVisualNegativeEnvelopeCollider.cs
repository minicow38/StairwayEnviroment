using System.Collections.Generic;
using System.Reflection;
using UnityEngine;


/// <summary>
/// BallVisualEqualizer専用のUpper Envelope Colliderだけを生成します。
///
/// 有効な物理仕様は次の1系統だけです。
///   Lower physical impact : 実際のStairway Collider
///   Upper physical impact : S(T) * R + A(t)
///   A(t)                  : H0 * epsilon * exp(-gamma * t)
///
/// Inspectorで指定する周期TはEnvelope幾何と位相観測の基準時間です。
/// Lower側の物理Colliderは生成せず、Equalizerは実階段そのものへ衝突します。
/// T/2は期待位相の基準として保持しますが、階段到達時刻を人工的に強制しません。
///
/// S(T) は周期Tと、以前成功した300-400m/s^2帯の中心値を基準に
/// [2R, 4R] の範囲で自動選択します。短周期では2R側へ寄せて入口衝撃を減らし、
/// 長周期では必要に応じて4Rまで広げます。
///
/// gammaはReleaseからExact LimitまでのSpline移動時間から一度だけ求めます。
/// First Contact方式選択、Curved Offset、World-Y補正は使用しません。
/// SlopeStickCore / CorrespondSubjectはREAD ONLYです.
///
/// BallVisualEqualizerSyncがClean impactでTangential Energyの一部をStable-Nへ
/// 一時転換しても、その転換分はこのEnvelopeのcanonicalEnergyRatioへ戻しません。
/// Envelopeは単調減少するCanonical Damping Ledgerだけを受け取ります。
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
    // Period / envelope geometry master
    // 周期TだけをInspector入力とし、Upper Envelope高さを同じ幾何モデルから
    // 解決する。T/2は位相観測の基準であり、Lower Stairwayへの到達を強制しない。
    // ================================================================

    [Header("Periodic Contact - Master")]
    [Tooltip(
        "実階段 -> Upper Envelope -> 実階段 の観測基準1周期 T[s]。\n" +
        "Upper高さはTから自動計算します。Lower Stairwayへの接触時刻は物理結果をそのまま使います。")]
    [Min(0.04f)]
    [SerializeField]
    private float targetContactPeriodSeconds = 0.10f;

    [Tooltip(
        "実階段から得た空間周期 Δs / actual tangent speed で、maxGround基準Tをどの程度だけ微補正するか。\n" +
        "0なら従来のT(n)=T0*V0/V(n)を厳密維持、0.15程度なら幾何観測を弱く反映します。")]
    [Range(0f, 0.25f)]
    [SerializeField]
    private float periodObservationBlend = 0.15f;

    // Upper EnvelopeをSpline基準面から十分離すため、surface-to-surface基準の
    // clearance scaleは最低2R。以前安定した4Rを上限としてAuto化する。
    private const float MinimumPeriodicRadiusClearanceScale = 2f;
    private const float MaximumPeriodicRadiusClearanceScale = 4f;
    private const int MinimumPeriodicHalfCycleFixedSteps = 2;

    // 以前実際に減衰振幅が成立した300-400m/s^2帯の中心値。
    // Inspectorパラメータにはせず、周期TからUpper高さS(T)を一意に決める
    // 内部幾何基準だけに使う。実階段へのdeadline Driveには使用しない。
    private const float PeriodicHeightReferencePhaseAcceleration = 350f;

    // ================================================================
    // maxGroundSpeed -> period-cycle experiment (READ ONLY)
    // ================================================================
    // SlopeStickCore is never written from this component.  The private
    // serialized maxGroundSpeed is read only as the reference V0.
    // One experiment step is one full oscillation period T, not one frame.
    // After 8 completed periods the diagnostic reference reaches V0 / 2.
    private const int MaxGroundSpeedDecayExperimentCycles = 8;
    private const float MaxGroundSpeedDecayExperimentEndRatio = 0.5f;

    // Geometry observation is a correction, never a replacement for the
    // maxGroundSpeed schedule.  Keep one bad/missed Stair contact from pulling
    // T far away from the planned 16->8 experiment.
    private const float MinimumObservedPeriodCorrectionRatio = 0.85f;
    private const float MaximumObservedPeriodCorrectionRatio = 1.15f;

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


    private Transform generatedRoot;
    private Mesh generatedMesh;

    // 最新Envelopeだけを保持する。次の生成前に旧Rootは破棄する。
    private readonly List<Transform> generatedEnvelopeRoots =
        new List<Transform>();

    // 現在有効なUpper Envelope Collider。
    // Release開始時に生成し、Release終了まで形状は不変です。
    private MeshCollider generatedMeshCollider;

    // Lower側の物理境界は生成しない。
    // EqualizerはScene上の実Stairway Colliderへ直接衝突する。

    [Header("Periodic Contact Runtime - Read Only")]
    [Tooltip("FixedUpdate解像度を考慮した実際の1周期T[s]。")]
    [SerializeField]
    private float resolvedContactPeriodSeconds = 0.10f;

    [Tooltip("位相観測上の半周期 T/2[s]。実階段への到達時刻は強制しません。")]
    [SerializeField]
    private float resolvedHalfPeriodSeconds = 0.05f;

    [Tooltip("自動決定されたUpper基準R倍率 S(T)。")]
    [SerializeField]
    private float resolvedEnvelopeRadiusClearanceScale =
        MinimumPeriodicRadiusClearanceScale;

    [Tooltip("Spline基準面からUpper中心までの幾何学的参照距離[m]。Lower Colliderは生成しません。")]
    [SerializeField]
    private float resolvedReleaseCenterTravelDistance;

    [Tooltip("Spline基準面からUpper surfaceまでの参照高さ[m]。Lower Colliderは生成しません。")]
    [SerializeField]
    private float resolvedReleaseSurfaceClearance;

    [Tooltip("T/2を基準にした理論Stable-N初速[m/s]。診断値で、実階段到達を強制しません。")]
    [SerializeField]
    private float resolvedReleaseTargetNormalSpeed;

    [Tooltip("旧T/2モデル由来の参照Phase加速度[m/s^2]。診断値のみで強制Driveには使いません。")]
    [SerializeField]
    private float resolvedReleasePhaseAcceleration;

    [Header("maxGroundSpeed / Period Experiment Runtime - Read Only")]
    [Tooltip("SlopeStickCore.maxGroundSpeed の実行時READ ONLY値。SlopeStickCoreへは書き戻しません。")]
    [SerializeField]
    private float sourceMaxGroundSpeedReadOnly;

    [Tooltip("0T..8Tの実験番号。Releaseをまたいで保持し、8T以降は8に固定します。")]
    [SerializeField]
    private int maxGroundSpeedExperimentCycleIndex;

    [Tooltip("InspectorのtargetContactPeriodSecondsを0T基準T0として保持します。")]
    [SerializeField]
    private float baseExperimentPeriodSeconds;

    [Tooltip("T0*V0/V(n)だけで得た補正前のnominal周期[s]。")]
    [SerializeField]
    private float nominalExperimentPeriodSeconds;

    [Tooltip("実階段のΔs / actual tangent speedから得た観測周期[s]。次Release以降のTだけを弱く補正します。")]
    [SerializeField]
    private float observedGeometryPeriodSeconds;

    [Tooltip("nominal Tへ掛ける幾何観測補正。0.85..1.15に制限し、1.0が無補正です。")]
    [SerializeField]
    private float observedGeometryPeriodCorrectionRatio = 1f;

    [SerializeField]
    private bool observedGeometryPeriodValid;

    [Tooltip("V(n)=V0*(1/2)^(n/8) で得る周期単位のREAD ONLY参照速度[m/s]。Coreの値は変更しません。")]
    [SerializeField]
    private float plannedMaxGroundSpeedForCycle;

    [Tooltip("plannedMaxGroundSpeedForCycle / sourceMaxGroundSpeedReadOnly。")]
    [SerializeField]
    private float plannedMaxGroundSpeedRatio = 1f;

    [SerializeField]
    private bool maxGroundSpeedReadAvailable;

    // Release開始時に確定したOscillation frameだけを保持する。
    // Active Release中にMeshをrecookしないため、A0/gamma/radius再生成cacheは持たない。
    private bool latestOscillationFrameCached;
    private Vector3 cachedAxisPhysics;
    private Vector3 cachedSlopeNormalPhysics;


    // ================================================================
    // Unity
    // ================================================================

    private void Awake()
    {
        ResolveReferences();
        TryEvaluateMaxGroundSpeedDecayExperiment(
            0,
            out _,
            out _,
            out _);
    }
    private void FixedUpdate()
    {
        TryBuildEnvelopeIfReady();
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

        // Cache only the Release-fixed oscillation frame.
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

        CacheLatestOscillationFrame(
            releaseTangentPhysics,
            stableReleaseNormalPhysics);

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
                $"cycle={maxGroundSpeedExperimentCycleIndex} " +
                $"maxGround0={sourceMaxGroundSpeedReadOnly:F3}m/s " +
                $"planned={plannedMaxGroundSpeedForCycle:F3}m/s " +
                $"speedRatio={plannedMaxGroundSpeedRatio:F5} " +
                $"T0={baseExperimentPeriodSeconds:F4}s " +
                $"Tnom={nominalExperimentPeriodSeconds:F4}s " +
                $"Tcorr={observedGeometryPeriodCorrectionRatio:F5} " +
                $"T={resolvedContactPeriodSeconds:F4}s " +
                $"halfT={resolvedHalfPeriodSeconds:F4}s " +
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
    // ================================================================
    // Envelope mesh
    // ================================================================
    private void CacheLatestOscillationFrame(
        Vector3 axisPhysics,
        Vector3 slopeNormalPhysics)
    {
        cachedAxisPhysics =
            axisPhysics;

        cachedSlopeNormalPhysics =
            slopeNormalPhysics;

        latestOscillationFrameCached =
            true;
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

        // targetContactPeriodSeconds is the 0T baseline T0.
        // The current experiment step changes only the Envelope/phase time scale:
        //
        //     V(n) = V0 * (1/2)^(n/8)
        //     T(n) = T0 * V0 / V(n) = T0 / speedRatio
        //
        // SlopeStickCore.maxGroundSpeed itself is READ ONLY and is never written.
        baseExperimentPeriodSeconds =
            Mathf.Max(
                0.0001f,
                targetContactPeriodSeconds);

        TryEvaluateMaxGroundSpeedDecayExperiment(
            maxGroundSpeedExperimentCycleIndex,
            out _,
            out _,
            out _);

        float experimentSpeedRatio =
            Mathf.Max(
                0.0001f,
                plannedMaxGroundSpeedRatio);

        nominalExperimentPeriodSeconds =
            baseExperimentPeriodSeconds /
            experimentSpeedRatio;

        // maxGroundSpeed schedule remains the dominant law.
        // Geometry contributes only a bounded, low-pass correction ratio
        // learned from accepted real Stairway contacts.
        float correctedExperimentPeriod =
            nominalExperimentPeriodSeconds *
            Mathf.Clamp(
                observedGeometryPeriodCorrectionRatio,
                MinimumObservedPeriodCorrectionRatio,
                MaximumObservedPeriodCorrectionRatio);

        resolvedContactPeriodSeconds =
            Mathf.Max(
                minimumPeriod,
                correctedExperimentPeriod);

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
        // この加速度はUpper高さを決める幾何学的な参照値だけに使用する。
        // Equalizer側でT/2 deadline driveは行わない。
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

        Transform meshTransform =
            meshObject.transform;

        meshTransform.SetParent(generatedRoot, false);
        meshTransform.localPosition = Vector3.zero;
        meshTransform.localRotation = Quaternion.identity;
        meshTransform.localScale = Vector3.one;

        Mesh mesh =
            BuildFullSplineEnvelopeMeshAsset(
                meshTransform,
                A0,
                decayRatePerSecondValue,
                equalizerRadius);

        if (!mesh)
        {
            Destroy(meshObject);
            return false;
        }

        MeshFilter meshFilter =
            meshObject.AddComponent<MeshFilter>();

        meshFilter.sharedMesh =
            mesh;

        generatedMeshCollider = meshObject.AddComponent<MeshCollider>();
        generatedMeshCollider.sharedMesh = mesh;
        generatedMeshCollider.convex = false;
        generatedMeshCollider.isTrigger = false;

        meshObject.AddComponent<BallVisualEnvelopeSurfaceMarker>();

        ConfigureEqualizerOnlyBoundaryCollider(
            generatedMeshCollider);

        generatedMesh = mesh;

        // Lower物理境界は生成しない。実Stairway Colliderが反射面になる。
        // Active Release中はこのMeshをrecookしない。
        Debug.Log(
            $"[EQUALIZER UPPER ENVELOPE CREATED] " +
            $"vertices={(generatedMesh ? generatedMesh.vertexCount : 0)} " +
            $"width={envelopeWidth:F3} " +
            $"segments={segmentCount} " +
            $"lower=RealStairway",
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
    public void RefreshEqualizerBoundaryCollisionOwnership()
    {
        ConfigureEqualizerOnlyBoundaryCollider(generatedMeshCollider);
    }


    public Collider CurrentUpperEnvelopeCollider =>
        generatedMeshCollider;


    /// <summary>
    /// SlopeStickCore.maxGroundSpeedをREAD ONLYで取得します。
    /// SetValueは行わず、Coreの物理状態は変更しません。
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
            Mathf.Max(
                0f,
                value);

        sourceMaxGroundSpeedReadOnly =
            speed;

        maxGroundSpeedReadAvailable =
            true;

        return true;
    }


    /// <summary>
    /// Equalizer側の実験番号を受け取ります。
    /// 物理Coreは変更せず、次回Envelope build時のT(n)だけを切り替えます。
    /// </summary>
    /// <summary>
    /// Canonicalな実Stairway接触から得た幾何周期を次Release以降へ弱く反映します。
    /// current EnvelopeのT/meshは途中変更しません。
    /// observedPeriod = Δs / actual tangent speed。
    /// </summary>
    public void SubmitObservedGeometryPeriod(
        float observedPeriodSeconds)
    {
        if (float.IsNaN(observedPeriodSeconds) ||
            float.IsInfinity(observedPeriodSeconds) ||
            observedPeriodSeconds <= 0.0001f)
        {
            return;
        }

        float nominal =
            Mathf.Max(
                0.0001f,
                nominalExperimentPeriodSeconds > 0.0001f
                    ? nominalExperimentPeriodSeconds
                    : BaseExperimentPeriodSeconds);

        float targetCorrectionRatio =
            Mathf.Clamp(
                observedPeriodSeconds / nominal,
                MinimumObservedPeriodCorrectionRatio,
                MaximumObservedPeriodCorrectionRatio);

        float blend =
            Mathf.Clamp01(periodObservationBlend);

        observedGeometryPeriodSeconds =
            observedPeriodSeconds;

        observedGeometryPeriodCorrectionRatio =
            observedGeometryPeriodValid
                ? Mathf.Lerp(
                    observedGeometryPeriodCorrectionRatio,
                    targetCorrectionRatio,
                    blend)
                : Mathf.Lerp(
                    1f,
                    targetCorrectionRatio,
                    blend);

        observedGeometryPeriodCorrectionRatio =
            Mathf.Clamp(
                observedGeometryPeriodCorrectionRatio,
                MinimumObservedPeriodCorrectionRatio,
                MaximumObservedPeriodCorrectionRatio);

        observedGeometryPeriodValid =
            true;

        Debug.Log(
            $"[ENVELOPE PERIOD OBSERVATION] " +
            $"observedT={observedGeometryPeriodSeconds:F5}s " +
            $"nominalT={nominal:F5}s " +
            $"blend={blend:F3} " +
            $"correction={observedGeometryPeriodCorrectionRatio:F5} " +
            $"apply=NextRelease",
            this);
    }


    public void SetMaxGroundSpeedExperimentCycle(
        int cycleIndex)
    {
        maxGroundSpeedExperimentCycleIndex =
            Mathf.Clamp(
                cycleIndex,
                0,
                MaxGroundSpeedDecayExperimentCycles);

        TryEvaluateMaxGroundSpeedDecayExperiment(
            maxGroundSpeedExperimentCycleIndex,
            out _,
            out _,
            out _);
    }


    /// <summary>
    /// Scene開始時など、実験を0Tへ明示的に戻すときだけ使用します。
    /// ClearEnvelope()では呼ばないため、通常のRejoin/次Releaseで進捗は失われません。
    /// </summary>
    public void ResetMaxGroundSpeedExperiment()
    {
        observedGeometryPeriodSeconds = 0f;
        observedGeometryPeriodCorrectionRatio = 1f;
        observedGeometryPeriodValid = false;
        nominalExperimentPeriodSeconds = 0f;

        SetMaxGroundSpeedExperimentCycle(0);
    }


    /// <summary>
    /// 周期Tを単位にした maxGroundSpeed 16->8 型の減衰実験。
    /// V0はSlopeStickCore.maxGroundSpeedの現在値をREAD ONLYで取得します。
    /// n=0..8について V(n)=V0*(1/2)^(n/8)。8T以降はV0/2で保持します。
    /// これはEqualizer/Envelopeの診断基準であり、SlopeStickCoreへは書き込みません。
    /// </summary>
    public bool TryEvaluateMaxGroundSpeedDecayExperiment(
        int cycleIndex,
        out float sourceMaxGroundSpeed,
        out float plannedMaxGroundSpeed,
        out float normalizedProgress01)
    {
        int clampedCycle =
            Mathf.Clamp(
                cycleIndex,
                0,
                MaxGroundSpeedDecayExperimentCycles);

        normalizedProgress01 =
            clampedCycle /
            (float)MaxGroundSpeedDecayExperimentCycles;

        maxGroundSpeedExperimentCycleIndex =
            clampedCycle;

        if (!TryReadSlopeCoreMaxGroundSpeed(
                out sourceMaxGroundSpeed))
        {
            plannedMaxGroundSpeed = 0f;
            plannedMaxGroundSpeedForCycle = 0f;
            plannedMaxGroundSpeedRatio = 0f;
            return false;
        }

        float ratio =
            Mathf.Pow(
                MaxGroundSpeedDecayExperimentEndRatio,
                normalizedProgress01);

        plannedMaxGroundSpeed =
            sourceMaxGroundSpeed *
            ratio;

        plannedMaxGroundSpeedForCycle =
            plannedMaxGroundSpeed;

        plannedMaxGroundSpeedRatio =
            sourceMaxGroundSpeed > 0.000001f
                ? plannedMaxGroundSpeed /
                  sourceMaxGroundSpeed
                : 0f;

        return true;
    }


    public int MaxGroundSpeedDecayCycleCount =>
        MaxGroundSpeedDecayExperimentCycles;

    public int MaxGroundSpeedExperimentCycleIndex =>
        maxGroundSpeedExperimentCycleIndex;

    public float BaseExperimentPeriodSeconds =>
        Mathf.Max(0.0001f, targetContactPeriodSeconds);

    public float ResolvedExperimentPeriodSeconds =>
        resolvedContactPeriodSeconds;

    public float NominalExperimentPeriodSeconds =>
        nominalExperimentPeriodSeconds;

    public float ObservedGeometryPeriodSeconds =>
        observedGeometryPeriodSeconds;

    public float ObservedGeometryPeriodCorrectionRatio =>
        observedGeometryPeriodCorrectionRatio;


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
    /// 現在のUpper Envelopeを完全に削除します。
    /// </summary>
    public void ClearEnvelope()
    {
        ClearAllGeneratedEnvelopeRoots();

        latestOscillationFrameCached =
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

        // Experiment state intentionally survives physical Envelope clear.
        // Rejoin / next generation must not send 0T..8T back to cycle 0.
        TryEvaluateMaxGroundSpeedDecayExperiment(
            maxGroundSpeedExperimentCycleIndex,
            out _,
            out _,
            out _);
        decayTimeCostSeconds = 0f;
    }


    /// <summary>
    /// 平面→斜面などで次のEnvelopeを生成する直前に使用する。
    /// 旧Envelopeは必ず破棄し、物理境界を一世代だけに保つ。
    /// </summary>
    private void PrepareForNextEnvelopeGeneration()
    {
        ClearAllGeneratedEnvelopeRoots();
        latestOscillationFrameCached = false;
        envelopeBuilt = false;
    }


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

        generatedMeshCollider =
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

        if (!latestOscillationFrameCached ||
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
    /// <summary>
    /// EqualizerのCanonical Damping Ledgerだけを受け取ります。
    /// Clean impactでTangential -> Stable-Nへ一時転換した反射Energyは渡しません。
    /// Active Release中は単調減少のみ許可し、現在のUpper Meshは再生成しません。
    /// </summary>
    public void SetCanonicalDampingEnergyRatio(
        float energyRatio)
    {
        float requested =
            Mathf.Clamp01(
                energyRatio);

        canonicalEnergyRatio =
            Mathf.Min(
                canonicalEnergyRatio,
                requested);
    }


    /// <summary>
    /// 旧API互換。新規コードではSetCanonicalDampingEnergyRatioを使用してください。
    /// </summary>
    public void SetCanonicalEnergyRatio(
        float energyRatio)
    {
        SetCanonicalDampingEnergyRatio(
            energyRatio);
    }


    public float CanonicalDampingEnergyRatio =>
        canonicalEnergyRatio;


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
