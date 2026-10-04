using UnityEngine;
using System.Collections;
using TMPro;
using System.Text.RegularExpressions;
using Sirenix.OdinInspector;

[Searchable]
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody), typeof(SphereCollider), typeof(NearestKnotDetector))]
public sealed class SlopeStickCore : MonoBehaviour
{
    
    public float MaxGroundSpeedReadOnly =>
        Mathf.Max(0f, maxGroundSpeed);

    public float TargetSlopeProgress01ReadOnly =>
        TargetProgress;

    const float Eps = 0.000001f;
    [Min(0f)] [SerializeField] float maxGroundSpeed = 24f;
    // Compactから固定値化
    const float ProbeRadius = .475f;
    [SerializeField]public float GroundAcceleration = 35f;
    const float MaxDeceleration = 80f;
    const float ResponseInverse = 8.333333f;
    const float AccelerationJerk = 600f;

    const float TargetMinDistance = .27f;
    [SerializeField]public float TargetAccelerationLimit = 120f;
    const float PostTargetBlendWidth = .05f;
    const float PostTargetGravityRatio = .50f;

    const float StickJerk = 3000f;
    const float ReleaseHold = .02f;
    const float NaturalReleaseEnd = .90f;
    
    public static readonly string CallForBestScore = "CallForBestScore";


    [SerializeField] NearestKnotDetector knotDetector;
    [SerializeField] LayerMask groundMask = ~0;

    [SerializeField] public RaycastHit currentHit;
    [Header("Travel")]
    [SerializeField] Vector3 travelDirection = Vector3.forward;
    
    [Range(0f, 100f)] [SerializeField] public float targetSlopeProgressPercent = 60f;

    [Header("Coordinate Mapping")]
    [Tooltip("PhysicsRoot上のInSubjectをVisualPlayerRoot側へ写す座標変換担当です。")]
    [SerializeField] CorrespondSubject correspondSubject;
    [Tooltip("Energy target / POP状態をREAD ONLY参照するBallVisual軌道担当です。")]
    [SerializeField] BallVisualSlopeDrive ballVisualSlopeDrive;
    [Tooltip("回転させない物理座標系。InSubjectはこの配下で物理計算します。")]
    [SerializeField] Transform physicsRoot;

    [Header("Map Direction Turn")]
    [Tooltip("回転する表示座標系。InSubject/PhysicsRootは回転させません。")]
    [SerializeField] Transform visualPlayerRoot;
    [Tooltip("VisualPlayerRootを回すワールドPivot。未設定ならCenter1を検索します。")]
    [SerializeField] Transform visualRotationPivot;
    [Tooltip("ONなら入力方向と反対へVisualPlayerRootを回し、マップが逆向きに旋回して見えるようにします。")]
    [SerializeField] bool visualRootTurnsOppositeToInput = true;
    [Tooltip("これ未満の横フリックは旋回として扱いません。MouseUpを待たず、押下中にこの閾値を超えた瞬間にTurn Input Intentを確定します。")]
    [Min(1f)] [SerializeField] float minimumFlickPixels = 10f;

    [Tooltip("Turn Input Intentを保持する最大時間[s]。古いフリックがRecovery/前回Turn終了後に突然実行されることを防ぎます。")]
    [Min(0.05f)] [SerializeField] float turnInputBufferSeconds = 0.25f;

    [Header("Turn Transition / Smooth Handoff")]
    [Tooltip("VisualPlayerRootの90度旋回時間[s]。CorrespondSubjectのTween時間をこの値で上書きします。")]
    [Min(0.05f)] [SerializeField] float turnVisualDurationSeconds = 0.30f;

    [Tooltip("旋回開始直後にInSubjectをKinematic停止する時間[s]。解除後はUTurnなら開始点から新方向へ直接進み、FiveLine系ならCoastへ移ります。")]
    [Min(0f)] [SerializeField] float turnHardFreezeSeconds = 0.12f;

    [Tooltip("Turn Input Intent / HardFreeze / Coastの診断ログを出します。")]
    [SerializeField] bool logTurnTransition = true;

    // 旋回角度は1系統だけ。入力強度や呼び出し元に関係なく必ず90度。
    const float QuarterTurnDegrees = 90f;

    public enum TurnResolutionMode
    {
        None,
        FiveLineAfterPop,
        UTurnBeforeEnergyTarget,
        FiveLineAfterEnergyTarget
    }

    [Header("Turn Policy - Upper Stair / Flat / Energy Target")]
    // Flat Entry / flick位置 / 次の斜面までの距離から旋回ごとに自動計算する。
    // Inspectorの固定m閾値は使わない（旧 energyTargetPassToleranceMeters を廃止）。

    [Tooltip("FiveLine横補正を滑らかに収束させる時間[s]。")]
    [Min(0.02f)] [SerializeField] float fiveLineCorrectionDurationSeconds = 0.10f;

    [Tooltip("FiveLineが一度に許す最大横補正距離[m]。遠距離スナップを防ぎます。")]
    [Min(0.05f)] [SerializeField] float fiveLineMaximumCorrectionMeters = 0.75f;

    [Tooltip("Turn policyとFiveLine選択をログへ出します。")]
    [SerializeField] bool logTurnPolicy = true;

    [Header("Spline Support")]
    [Min(.01f)] [SerializeField] float probeDistance = .85f;
    [Range(1f, 89f)] [SerializeField] float maxSlopeAngle = 75f;

    [Min(0f)] [SerializeField] float supportGraceSeconds = .12f;
    [Min(0f)] [SerializeField] float supportGraceMaxGuideDistance = 1.75f;
    [Min(0f)] [SerializeField] float maxGraceOutwardSpeed = 2f;

    [Header("Stick")]
    [Min(0f)] [SerializeField] float flatStick = 24.6f;
    [Min(0f)] [SerializeField] float maxStick = 1000f;
    [Min(1f)] [SerializeField] float stickSafety = 1.10f;

    [Header("PlayerMotivation")] [SerializeField]
    public bool BeginCommandOnTouch = false;
    [Header("Debug")]
    [SerializeField] bool logCore;

    Rigidbody rb;
    Vector3 direction;

    // ================================================================
    // Initial Visual Frame Pose
    // ================================================================
    // VisualPlayerRoot は旋回時に position と rotation の両方が変化するため、
    // Scene初期状態のワールドPoseをセットで保存して復元する。
    Vector3 initialVisualPlayerRootPosition;
    Quaternion initialVisualPlayerRootRotation = Quaternion.identity;
    bool hasInitialVisualPlayerRootPose;

    // FirstStepInsertSplinePath側が「死亡復帰の再構築」を開始した時だけtrue。
    // 通常のStart() -> delayStart()には復帰専用処理を侵入させない。
    bool restartFramePrepared;

    Vector2 flickStart;
    bool trackingFlick;
    bool flickConsumed;
    float pendingTurnDegrees;
    float pendingTurnQueuedTime = -1f;

    float graceTimer;
    float driveState;
    float stickState;
    bool wasSlope;

    // 旋回が始まってからNearestKnotDetectorが旋回後Splineを捕捉するまでtrue。
    bool waitingForTurnGuide;
    Vector3 turnTargetDirection;

    struct TurnTransitionSnapshot
    {
        public bool valid;
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 velocity;
        public Vector3 angularVelocity;
        public Vector3 direction;
        public bool useGravity;
    }
    
    public bool Earliest=false;

    // Visual旋回全体と、先頭の短いKinematic停止区間を分ける。
    bool turnTransitionActive;
    bool turnBodyFrozen;
    float turnTransitionStartTime = -1f;
    float activeTurnDegrees;
    TurnTransitionSnapshot turnTransitionSnapshot;

    TurnResolutionMode activeTurnMode = TurnResolutionMode.None;
    bool fiveLineCorrectionPending;
    bool fiveLineCorrectionActive;
    float fiveLineCorrectionElapsed;
    Vector3 fiveLineCorrectionSide;
    Vector3 fiveLineCorrectionStartPosition;
    float fiveLineCorrectionTargetOffset;
    NearestKnotDetector.FiveLineGroup fiveLineTargetGroup =
        NearestKnotDetector.FiveLineGroup.Center;

    bool capturedTurnLandingIntentValid;
    BallVisualSlopeDrive.TurnLandingIntent capturedTurnLandingIntent;
    float lastEnergyTargetForwardDistance = float.PositiveInfinity;

    struct FlatEntryFrame
    {
        public bool valid;
        public int splineIndex;
        public Vector3 position;
        public Vector3 direction;
        public float distanceToNextSlope;
    }

    FlatEntryFrame flatEntryFrame;
    FlatEntryFrame pendingFlatEntryFrame;
    bool pendingTurnSnapshotValid;
    bool pendingTurnWasFlat;
    Vector3 pendingTurnFlickPosition;
    Vector3 pendingTurnFlickVelocity;
    bool pendingTurnTargetValid;
    Vector3 pendingTurnTargetPhysics;
    BallVisualSlopeDrive.TurnLandingIntent pendingTurnLandingIntent;
    float dynamicEnergyTargetTolerance = float.NaN;
    float lastFlatTurnLimitProgress = float.NaN;

    public float LastDynamicEnergyTargetTolerance => dynamicEnergyTargetTolerance;
    public float LastFlatTurnLimitProgress => lastFlatTurnLimitProgress;

    // 旧READ ONLY APIとの互換性を維持。Bezier物理Pathは廃止したため常にfalse。
    public bool IsPhysicsTurnPathActive => false;
    public bool IsTurnTransitionActive => turnTransitionActive;
    public bool IsTurnBodyFrozen => turnBodyFrozen;
    public TurnResolutionMode CurrentTurnResolutionMode => activeTurnMode;
    public bool IsFiveLineCorrectionPending => fiveLineCorrectionPending;
    public bool IsFiveLineCorrectionActive => fiveLineCorrectionActive;
    public NearestKnotDetector.FiveLineGroup CurrentFiveLineTargetGroup => fiveLineTargetGroup;
    public float LastEnergyTargetForwardDistance => lastEnergyTargetForwardDistance;

    const float TurnGuideAlignmentMin = 0.8f;

    float TargetProgress => Mathf.Clamp01(targetSlopeProgressPercent * .01f);
    float ReleaseStart => Mathf.Clamp01(TargetProgress + ReleaseHold);
    float ReleaseEnd => Mathf.Max(ReleaseStart, NaturalReleaseEnd);

    struct Surface
    {
        public Vector3 tangent, side, normal;
        public float tangentSpeed, lateralSpeed, outwardSpeed;
        public float gravityAlong, gravitySupport;
        public bool Valid => tangent.sqrMagnitude > Eps;
    }

    // ================================================================
    // BallVisual READ ONLY motion reference
    // ================================================================
    // BallVisualSlopeDrive does not write to Core. Core exposes one stable
    // spline-session plan whose meaning matches the successful SlopeStick3D
    // contract while using NearestKnotDetector as the canonical path model.

    NearestKnotDetector.GuideFrame currentGuide;
    Surface currentSurface;
    bool currentGuideValid;
    bool currentSurfaceValid;
    bool currentSupported;
public float CurrentSplineDriveAccelerationReadOnly =>
    driveState;

public float CurrentSplineStickAccelerationReadOnly =>
    stickState;

public float PredictDesiredSplineDriveReadOnly(
    bool isSlope,
    float tangentSpeed,
    float sectionProgress01,
    float sectionLength,
    float gravityAlong)
{
    tangentSpeed =
        Mathf.Max(
            0f,
            tangentSpeed);

    if (!isSlope)
    {
        return
            SpeedDrive(
                tangentSpeed);
    }

    float target;

    if (sectionProgress01 < TargetProgress)
    {
        float remaining =
            Mathf.Max(
                TargetMinDistance,
                (TargetProgress -
                 sectionProgress01) *
                Mathf.Max(
                    TargetMinDistance,
                    sectionLength));

        target =
            (maxGroundSpeed *
             maxGroundSpeed -
             tangentSpeed *
             tangentSpeed) /
            (2f *
             Mathf.Max(
                 Eps,
                 remaining))
            -
            gravityAlong;

        target =
            Mathf.Clamp(
                target,
                -TargetAccelerationLimit,
                TargetAccelerationLimit);
    }
    else
    {
        target =
            SpeedDrive(
                tangentSpeed);

        float blend =
            SmoothRange01(
                sectionProgress01,
                TargetProgress,
                Mathf.Clamp01(
                    TargetProgress +
                    PostTargetBlendWidth));

        target -=
            Mathf.Max(
                0f,
                gravityAlong) *
            PostTargetGravityRatio *
            blend;

        target =
            Mathf.Min(
                0f,
                target);
    }

    return
        Mathf.Max(
            -MaxDeceleration,
            target);
}

public float AdvancePredictedSplineDriveReadOnly(
    float currentDrive,
    float desiredDrive,
    float deltaTime)
{
    return
        Mathf.MoveTowards(
            currentDrive,
            desiredDrive,
            AccelerationJerk *
            Mathf.Max(
                0f,
                deltaTime));
}
    struct BallVisualSplinePlan
    {
        public bool valid;

        public Vector3 targetCenterPhysics;
        public Vector3 targetTangent;
        public Vector3 targetNormal;
        public Vector3 targetReferenceVelocityPhysics;

        public float targetCurvature;
        public float targetTangentSpeed;

        public float currentProgress01;
        public float progressRate01PerSecond;
        public float distanceToTarget;
    }

    BallVisualSplinePlan ballVisualPlan;

    NearestKnotDetector.GuideFrame ballVisualSessionGuide;
    bool hasBallVisualSession;
    int ballVisualStableFrames;
    float previousBallVisualProgress;
    float ballVisualProgressDelta;

    const int BallVisualStableFramesRequired = 2;
    const float BallVisualMaximumProgressJump01 = .25f;
    const float BallVisualAllowedReverseProgress01 = .005f;

    public Rigidbody Body => rb;

    // READ ONLY handoff telemetry for BallVisualEqualizer.
    // This is the exact inward Stable-N support acceleration that SlopeStickCore
    // is currently applying through -surface.normal * stickState.
    // Equalizer may capture it at Plane -> Stair entry, but never writes it back.
    public float BallVisualNormalSupportAccelerationReadOnly =>
        Mathf.Max(0f, stickState);

    public bool BallVisualNormalSupportAvailableReadOnly =>
        currentSupported && currentSurfaceValid;

    public bool BallVisualIsOnFlat =>
        currentSupported &&
        currentGuideValid &&
        !currentGuide.isSlope;

    public bool BallVisualIsOnSlope =>
        currentSupported &&
        currentGuideValid &&
        currentGuide.isSlope;

    public bool BallVisualIsAir =>
        !currentSupported;

    // "Active slope frame" means that Core has a valid spline basis now.
    // Incident readiness is stricter and is exposed separately below.
    public bool BallVisualHasActiveSlopeFrame =>
        currentSupported &&
        currentGuideValid &&
        currentSurfaceValid &&
        currentGuide.isSlope &&
        !waitingForTurnGuide &&
        !IsPhysicsTurnPathActive &&
        !turnTransitionActive;

    public bool IsWaitingForTurnGuide =>
        waitingForTurnGuide || IsPhysicsTurnPathActive || turnTransitionActive;

    // Exact Incident gate used by BallVisualSlopeDrive.
    // World-Y velocity is intentionally not used: outwardSpeed is measured
    // against the spline surface normal.
    public bool BallVisualIncidentReady
    {
        get
        {
            if (!BallVisualHasActiveSlopeFrame ||
                !ballVisualPlan.valid ||
                ballVisualStableFrames < BallVisualStableFramesRequired)
            {
                return false;
            }

            if (ballVisualPlan.currentProgress01 >= TargetProgress)
                return false;

            if (currentSurface.tangentSpeed <= Eps)
                return false;

            if (currentSurface.outwardSpeed > maxGraceOutwardSpeed)
                return false;

            return true;
        }
    }

    public Vector3 BallVisualSurfaceNormal =>
        currentSurfaceValid
            ? currentSurface.normal
            : (currentGuideValid &&
               currentGuide.normal.sqrMagnitude > Eps
                ? currentGuide.normal.normalized
                : Vector3.up);

    public Vector3 BallVisualSlopeTangent =>
        currentSurfaceValid
            ? currentSurface.tangent
            : NormalizeFlat(
                currentGuideValid
                    ? currentGuide.tangent
                    : direction,
                direction);

    public float BallVisualSlopeProgress01 =>
        ballVisualPlan.valid
            ? ballVisualPlan.currentProgress01
            : (currentGuideValid
                ? currentGuide.sectionProgress01
                : 0f);

    public float BallVisualSlopeProgressRatePercentPerSecond =>
        ballVisualPlan.valid
            ? ballVisualPlan.progressRate01PerSecond * 100f
            : 0f;

    public float BallVisualSlopeSectionLength =>
        currentGuideValid
            ? currentGuide.sectionLength
            : 0f;

    public float BallVisualDistanceToTarget =>
        ballVisualPlan.valid
            ? ballVisualPlan.distanceToTarget
            : 0f;

    public float slopeProgressErrorPercent =>
        ballVisualPlan.valid
            ? (ballVisualPlan.currentProgress01 - TargetProgress) * 100f
            : (currentGuideValid
                ? (currentGuide.sectionProgress01 - TargetProgress) * 100f
                : float.PositiveInfinity);

    // Kept under the old API name so the successful BallVisualSlopeDrive
    // formula can remain unchanged. Unlike the previous Core port, this is
    // planned BEFORE Target crossing from the target spline sample.
    public float CapturedTargetTangentSpeed =>
        ballVisualPlan.valid
            ? ballVisualPlan.targetTangentSpeed
            : 0f;

    public Vector3 BallVisualTargetTangentPhysics =>
        ballVisualPlan.valid
            ? ballVisualPlan.targetTangent
            : BallVisualSlopeTangent;

    public Vector3 BallVisualTargetNormalPhysics =>
        ballVisualPlan.valid
            ? ballVisualPlan.targetNormal
            : BallVisualSurfaceNormal;

    public Vector3 BallVisualTargetReferenceVelocityPhysics =>
        ballVisualPlan.valid
            ? ballVisualPlan.targetReferenceVelocityPhysics
            : Vector3.zero;

    public Vector3 BallVisualTargetProgressCenterPhysics =>
        ballVisualPlan.valid && rb
            ? ballVisualPlan.targetCenterPhysics
            : (rb ? rb.position : transform.position);

    public bool TryGetBallVisualTargetProgressCenterPhysics(
        out Vector3 targetPosition)
    {
        targetPosition =
            BallVisualTargetProgressCenterPhysics;

        return
            BallVisualHasActiveSlopeFrame &&
            ballVisualPlan.valid;
    }

    /// <summary>
    /// BallVisual / Envelope用READ ONLY API。
    /// 現在捕捉しているSpline区間のprogress01位置をPhysics座標で評価します。
    /// Rigidbody中心がGuide lineから持つSide/Normal offsetも同じ区間へ平行移送します。
    /// SlopeStickCoreの状態は変更しません。
    /// </summary>
    public bool TryEvaluateBallVisualSectionFramePhysics(
        float progress01,
        out Vector3 centerPhysics,
        out Vector3 tangentPhysics,
        out Vector3 normalPhysics)
    {
        centerPhysics = rb ? rb.position : transform.position;
        tangentPhysics = BallVisualSlopeTangent;
        normalPhysics = BallVisualSurfaceNormal;

        if (!rb ||
            !knotDetector ||
            !currentGuideValid)
        {
            return false;
        }

        NearestKnotDetector.GuideFrame sectionGuide =
            hasBallVisualSession
                ? ballVisualSessionGuide
                : currentGuide;

        if (!sectionGuide.valid ||
            !knotDetector.TryEvaluateSameSection(
                sectionGuide,
                Mathf.Clamp01(progress01),
                out NearestKnotDetector.GuideSample sample) ||
            !sample.valid)
        {
            return false;
        }

        Vector3 normal =
            sample.normal.sqrMagnitude > Eps
                ? sample.normal.normalized
                : Vector3.up;

        Vector3 tangent =
            Vector3.ProjectOnPlane(
                sample.tangent,
                normal);

        if (tangent.sqrMagnitude <= Eps)
            return false;

        tangent.Normalize();

        if (Vector3.Dot(tangent, direction) < 0f)
            tangent = -tangent;

        Vector3 side =
            Vector3.Cross(
                normal,
                tangent);

        if (side.sqrMagnitude <= Eps)
            return false;

        side.Normalize();
        tangent = Vector3.Cross(side, normal).normalized;

        // Preserve the current Rigidbody-center offset from the spline guide.
        Surface basis =
            currentSurfaceValid
                ? currentSurface
                : BuildSplineSurface(currentGuide);

        float sideOffset = 0f;
        float normalOffset = 0f;

        if (basis.Valid)
        {
            Vector3 currentOffset =
                rb.position -
                currentGuide.point;

            sideOffset =
                Vector3.Dot(
                    currentOffset,
                    basis.side);

            normalOffset =
                Vector3.Dot(
                    currentOffset,
                    basis.normal);
        }

        centerPhysics =
            sample.point +
            side * sideOffset +
            normal * normalOffset;

        tangentPhysics = tangent;
        normalPhysics = normal;

        return true;
    }


    void ResetBallVisualSplineSession()
    {
        ballVisualPlan = default;
        ballVisualSessionGuide = default;
        hasBallVisualSession = false;
        ballVisualStableFrames = 0;
        previousBallVisualProgress = 0f;
        ballVisualProgressDelta = 0f;
    }

    void UpdateBallVisualSplinePlan(
        NearestKnotDetector.GuideFrame guide,
        Surface surface)
    {
        ballVisualPlan = default;

        if (!currentSupported ||
            !guide.valid ||
            !guide.isSlope ||
            !surface.Valid ||
            waitingForTurnGuide ||
            IsPhysicsTurnPathActive ||
            turnTransitionActive ||
            !knotDetector)
        {
            ResetBallVisualSplineSession();
            return;
        }

        bool sameSession =
            hasBallVisualSession &&
            knotDetector.IsSameSection(
                ballVisualSessionGuide,
                guide);

        if (!sameSession)
        {
            ballVisualSessionGuide = guide;
            hasBallVisualSession = true;
            ballVisualStableFrames = 1;
            previousBallVisualProgress =
                guide.sectionProgress01;
            ballVisualProgressDelta = 0f;
            return;
        }

        float progress =
            guide.sectionProgress01;

        float delta =
            progress -
            previousBallVisualProgress;

        previousBallVisualProgress =
            progress;

        bool plausibleProgress =
            delta >= -BallVisualAllowedReverseProgress01 &&
            Mathf.Abs(delta) <=
                BallVisualMaximumProgressJump01;

        if (!plausibleProgress)
        {
            ballVisualStableFrames = 1;
            ballVisualProgressDelta = 0f;
            return;
        }

        ballVisualStableFrames++;
        ballVisualProgressDelta = delta;

        if (ballVisualStableFrames <
            BallVisualStableFramesRequired)
        {
            return;
        }

        if (!knotDetector.TryEvaluateSameSection(
                ballVisualSessionGuide,
                TargetProgress,
                out NearestKnotDetector.GuideSample target) ||
            !target.valid)
        {
            return;
        }

        Vector3 targetNormal =
            target.normal.sqrMagnitude > Eps
                ? target.normal.normalized
                : Vector3.up;

        Vector3 targetTangent =
            Vector3.ProjectOnPlane(
                target.tangent,
                targetNormal);

        if (targetTangent.sqrMagnitude <= Eps)
            return;

        targetTangent.Normalize();

        if (Vector3.Dot(
                targetTangent,
                direction) < 0f)
        {
            targetTangent =
                -targetTangent;
        }

        Vector3 targetSide =
            Vector3.Cross(
                targetNormal,
                targetTangent);

        if (targetSide.sqrMagnitude <= Eps)
            return;

        targetSide.Normalize();

        // Re-orthogonalize the target tangent.
        targetTangent =
            Vector3.Cross(
                targetSide,
                targetNormal).normalized;

        
        Vector3 currentGuideOffset = rb.position - guide.point;

        float sideOffset = Vector3.Dot(currentGuideOffset, surface.side);

        float normalOffset =
            Vector3.Dot(currentGuideOffset, surface.normal);

        Vector3 targetCenter =
            target.point + targetSide * sideOffset + targetNormal * normalOffset;

        float targetCurvature =
            Mathf.Max(0f, Mathf.Max(target.curvature, target.entryCurvature));

        float targetGravitySupport =
            Mathf.Max(0f, Vector3.Dot(Physics.gravity, -targetNormal));

        float targetTangentSpeed =
            maxGroundSpeed;

        if (targetCurvature > Eps)
        {
            // DesiredStick uses:
            // v^2 * curvature * stickSafety - gravitySupport <= maxStick
            // so invert the same inequality at Target Progress.
            float supportedSpeed =
                Mathf.Sqrt(Mathf.Max(0f, (maxStick + targetGravitySupport) / Mathf.Max(Eps, targetCurvature * stickSafety)));

            targetTangentSpeed =
                Mathf.Min(
                    maxGroundSpeed,
                    supportedSpeed);
        }

        float distanceToTarget;

        if (!knotDetector.TryGetDistanceAlongSameSection(
                ballVisualSessionGuide,
                progress,
                TargetProgress,
                out distanceToTarget))
        {
            distanceToTarget =
                Mathf.Abs(
                    TargetProgress -
                    progress) *
                Mathf.Max(
                    0f,
                    guide.sectionLength);
        }

        float progressRate =
            ballVisualProgressDelta /
            Mathf.Max(
                Time.fixedDeltaTime,
                Eps);

        ballVisualPlan =
            new BallVisualSplinePlan
            {
                valid = true,

                targetCenterPhysics =
                    targetCenter,

                targetTangent =
                    targetTangent,

                targetNormal =
                    targetNormal,

                targetReferenceVelocityPhysics =
                    targetTangent *
                    targetTangentSpeed,

                targetCurvature =
                    targetCurvature,

                targetTangentSpeed =
                    targetTangentSpeed,

                currentProgress01 =
                    progress,

                progressRate01PerSecond =
                    progressRate,

                distanceToTarget =
                    distanceToTarget
            };
    }

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        if (!ballVisualSlopeDrive)
            ballVisualSlopeDrive = FindObjectOfType<BallVisualSlopeDrive>(true);


        if (!knotDetector)
            knotDetector = GetComponent<NearestKnotDetector>();

        direction = NormalizeFlat(travelDirection, transform.forward);

        // Start() より前、通常旋回が始まる前のPoseを保存する。
        FindMapFrameReferences();
        BindCoordinateFrames();
        CaptureInitialVisualFramePose();
    }

    // ================================================================
    // Usability
    // ================================================================

    void Start()
    {
        FindMapFrameReferences();
        CaptureInitialVisualFramePose();
        BindCoordinateFrames();
        StartCoroutine(delayStart());
    }

    void Update()
    {
        ReadTurnFlick();
    }

    public void PushStart()
    {
        MainGameManager.TopTitle.SetActive(false);
        MainGameManager.PreviewIconRoot.SetActive(false);
        MainGameManager.TopLiteral.SetActive(false);
        MainGameManager.PlayButton.SetActive(false);
        MainGameManager.Userbility.SetActive(true);
        
        Earliest=true;
        BeginCommandOnTouch = true;
    }

    IEnumerator Recover()
    {
        
        yield return new WaitForSeconds(0.5f);
        MainGameManager.DropOut.SetActive(true);
        yield return new WaitForSeconds(2f);
        MainGameManager.DropOut.SetActive(false);
        MainGameManager.OpenChunkStage = true;
        
         
        MainGameManager.TopTitle.SetActive(true);
        MainGameManager.PreviewIconRoot.SetActive(true);
        MainGameManager.TopLiteral.SetActive(true);
        MainGameManager.PlayButton.SetActive(true);
        MainGameManager.Userbility.SetActive(false);

    }
    public IEnumerator delayStart()
    {
        yield return new WaitForSeconds(0.15f);
        GameObject startSlab =
            GameObject.Find(
                "CollisionStageRoot/__GeneratedPhysics/ArcSlab2_0_Physics");

        if (!startSlab)
        {
            Debug.LogError(
                "[CORE DELAY START] ArcSlab2_0_Physicsが見つかりません。",
                this);
            yield break;
        }

        if (!rb)
        {
            Debug.LogError(
                "[CORE DELAY START] InSubject Rigidbodyがありません。",
                this);
            yield break;
        }

        // ------------------------------------------------------------
        // Soft restart gate
        // ------------------------------------------------------------
        // Start()からもdelayStart()は呼ばれるため、復帰専用処理を
        // 無条件にここへ入れると通常起動のdirection/Controller状態まで変えてしまう。
        // FirstStepInsertSplinePathが死亡復帰時にPrepareForStageRebuild()を
        // 呼んだ場合だけ、下のrestartPreparedがtrueになる。
        bool restartPrepared = restartFramePrepared;
        restartFramePrepared = false;
        MainGameManager.OnDead = false;
        


        Vector3 restart =
            startSlab.transform.position;

        if (restartPrepared)
        {
            // Root PoseはStage/Spline再生成前に既に復元済み。
            // ここでは「遅れて残ったTween」だけを念のため止める。
            // Rootをもう一度復元しないので、再生成後の座標基準を再び動かさない。
            correspondSubject?.CancelVisualFrameTurn(false);

            // KnotDetector.Evaluate()は使わない。
            // 再生成直後のGuide選択差がdirectionへ流れ込むのを避け、
            // 実際の開始Physics板のforwardだけを使う。
            direction = ResolveRestartDirectionSoft(
                startSlab.transform);
        }
        else
        {
            // 通常起動は従来の挙動を維持。
            direction = Vector3.forward;
        }

        MainGameManager.LimitTouchingphase = 9;
        turnTargetDirection = direction;

        // 旋回中に死亡/再構築へ入ってもKinematic状態を持ち越さない。
        if (rb.isKinematic)
            rb.isKinematic = false;

        rb.position =
            new Vector3(
                restart.x,
                restart.y + 2f,
                restart.z);

        // 位置を飛ばすのと同じ瞬間に物理速度を0へ戻す。
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        if (restartPrepared)
        {
            // 復帰専用の状態破棄はこの瞬間だけ。
            // Stage再生成の0.3秒前にdrive/stickを先に0へする処理はやめる。
            ResetRestartTransientStateSoft();

            if (logCore)
            {
                ;
            }
        }

        Physics.SyncTransforms();
        correspondSubject?.SynchronizeNow(true);
    }

    // ================================================================
    // Main
    // ================================================================

    void FixedUpdate()
    {
        // Updateで予約した90度旋回を、物理/Spline観測より先に1回だけ適用する。
        ApplyPendingQuarterTurn();

        // Visual旋回中は通常のSpline Drive / Stick / FiveLineを止める。
        // UTurnはHardFreeze解除時に90度方向転換済み。円弧追従は行わない。
        if (turnTransitionActive)
        {
            AdvanceTurnTransition();
            if (turnBodyFrozen)
            {
                ResetBallVisualSplineSession();
                return;
            }

            direction = turnTargetDirection;
            driveState = 0f;
            ResetBallVisualSplineSession();
            return;
        }

        // Visual旋回後、新Spline捕捉まで旧SplineのDriveを再開しない。
        if (waitingForTurnGuide)
            direction = turnTargetDirection;

        var guide = knotDetector.Evaluate(rb.position);

        // Splineが存在しなければCoreは制御しない。
        if (!guide.valid)
        {
            LoseSupport();
            return;
        }

        currentGuide = guide;
        currentGuideValid = true;

        (bool grounded,RaycastHit hit)= HasGroundSupport();

        if (grounded)
            graceTimer = supportGraceSeconds;
        else
            graceTimer = Mathf.Max(0f, graceTimer - Time.fixedDeltaTime);
        
        if (currentHit.transform != null)
            if (Vector3.Distance(transform.position, currentHit.transform.position) > 12 && !grounded)
            {

                if (!MainGameManager.OnDead)
                {
                    StartCoroutine(Recover());
                }

                AndroidOneOnly.pharseCoin = MainGameManager.Coin;
               AndroidOneOnly.currentScore = MainGameManager.CurrentPointToPlane;
               if (MainGameManager.CurrentPointToPlane > AndroidOneOnly.bestScore)
               {
                   PlayerPrefs.SetInt(CallForBestScore, MainGameManager.CurrentPointToPlane);
                   AndroidOneOnly.bestScore = MainGameManager.CurrentPointToPlane;
               }

               
               MainGameManager.OnDead = true;
                MainGameManager.core.BeginCommandOnTouch = false;

                MainGameManager.core.Earliest=false;
            }

        if (hit.transform != null)
        {
            currentHit = hit;

        }
        
        
        
        float load = grounded ? 0f : SupportLoad(guide);
        bool grace = !grounded && graceTimer > 0f && load <= 1f && CanGrace(guide);

        if (!grounded && !grace)
        {
            if (logCore)
                ;

            LoseSupport();
            return;
        }
        
        currentSupported = true;
        
        Surface surface = BuildSplineSurface(guide);

        if (!surface.Valid)
        {
            currentSurfaceValid = false;
            return;
        }

        currentSurface = surface;
        currentSurfaceValid = true;
        // Slopeへの切替そのものもSpline判定。
        if (guide.isSlope && !wasSlope)
        {
            TrySwitchStageCollidersOnSlopeEntry(hit);
            driveState = 0f;
            TransportToSpline(guide);
        }

        // Collider法線は使わない。
        // tangent / normal は必ずSpline Guideから構築。

        if (!surface.Valid)
        {
            currentSurfaceValid = false;
            return;
        }

        currentSurface = surface;
        currentSurfaceValid = true;
        // ================================================================
// 旋回後：旋回後Splineへ切り替わるまで待つ
// ================================================================
        if (waitingForTurnGuide)
        {
            Vector3 flatGuideTangent =
                Vector3.ProjectOnPlane(
                    guide.tangent,
                    Vector3.up);

           

            float alignment = 0f;

            if (flatGuideTangent.sqrMagnitude > Eps )
            {
                flatGuideTangent.Normalize();
               

                alignment =
                    Mathf.Abs(
                        Vector3.Dot(
                            flatGuideTangent,
                            turnTargetDirection));
            }

            // まだ旋回前Splineを見ている
            if (alignment < TurnGuideAlignmentMin)
            {
                // 旋回前Spline方向へのDriveを止める
                driveState = 0f;

                // 接地維持だけ残す
                stickState =
                    Move(
                        stickState,
                        flatStick,
                        StickJerk);

                rb.AddForce(
                    -surface.normal * stickState,
                    ForceMode.Acceleration);

                // InCorner開始後はtargetDirectionの水平速度を保持したまま、
                // 新Splineが捕捉されるまで旧SplineのDriveだけを止める。
                return;
            }

            // 旋回後Splineを捕捉した。
            waitingForTurnGuide = false;
            driveState = 0f;

            if (activeTurnMode == TurnResolutionMode.UTurnBeforeEnergyTarget)
            {
                // UTurnという「前回旋回の方針」は次の旋回判定まで保持する。
                // 着地点Intentだけは古くなるので、Spline捕捉時点で破棄する。
                capturedTurnLandingIntentValid = false;
                capturedTurnLandingIntent = default;
            }
        }

        // 物理的に安定したFlatの最初のGuideからEntry基準を1回だけ保存する。
        // Turn中は上部の早期returnで更新しない。旧Flatと新Flatを混ぜない。
        UpdateFlatEntryFrame(guide);

        // FiveLineモードではVisual回転とBallVisual POP/Rejoinが終わってから、
        // 下階段側の5ラインへ短い横補正を開始する。
        TryBeginPendingFiveLineCorrection(guide, ref surface);

        if (fiveLineCorrectionActive)
        {
            StepFiveLineCorrection(ref surface);

            stickState = Move(stickState, flatStick, StickJerk);
            rb.AddForce(-surface.normal * stickState, ForceMode.Acceleration);
            driveState = 0f;
            return;
        }

       /* if (Input.GetMouseButtonDown(0))
        {
            MainGameManager.TopTitle.SetActive(false);
            MainGameManager.PreviewIconRoot.SetActive(false);
            MainGameManager.TopLiteral.SetActive(false);
            MainGameManager.PlayButton.SetActive(false);
            MainGameManager.Userbility.SetActive(true);

          BeginCommandOnTouch = true;
        }*/
        // Build the stable read-only Spline plan used by BallVisual.
        // This runs only after turn-guide handoff has completed.
        if (BeginCommandOnTouch == true)
        {
            UpdateBallVisualSplinePlan(
                guide,
                surface);

            float release = guide.isSlope
                ? 1f - SmoothRange01(guide.sectionProgress01, ReleaseStart, ReleaseEnd)
                : 1f;

            float desiredDrive = DesiredDrive(surface, guide, grace) * release;
            driveState = Move(driveState, desiredDrive, AccelerationJerk);

            float curvature = SplineCurvature(guide);

            float desiredStick = DesiredStick(surface, guide, driveState, curvature, grace) * release;
            stickState = Move(stickState, desiredStick, StickJerk);


            Vector3 acceleration =
                surface.tangent * driveState
                - surface.side * surface.lateralSpeed * ResponseInverse
                - surface.normal * stickState;

            rb.AddForce(acceleration, ForceMode.Acceleration);

            wasSlope = guide.isSlope;

            if (logCore)
            {
                ;
            }
        }
    }

    // ================================================================
    // Stage collider handoff
    // ================================================================

    void TrySwitchStageCollidersOnSlopeEntry(RaycastHit hit)
    {
        if (!hit.transform || !visualRotationPivot)
            return;

        // 初回直進中ではなく、Visual座標系が旋回済みのときだけ次区間へColliderを渡す。
        if (visualRotationPivot.rotation == Quaternion.identity)
            return;

        Match matched = Regex.Match(
            hit.transform.name,
            @"^StairWay(\d+)_(\d+)_Physics$");

        if (!matched.Success ||
            !int.TryParse(matched.Groups[1].Value, out int stairIndex))
        {
            return;
        }

        string lane = matched.Groups[2].Value;

        SetMeshColliderEnabled(
            $"StairWay{stairIndex - 1}_{lane}_Render",
            false);

        SetMeshColliderEnabled(
            $"ArcSlab{stairIndex}_{lane}_Render",
            false);

        SetMeshColliderEnabled(
            $"StairWay{stairIndex}_{lane}_Render",
            true);

        SetMeshColliderEnabled(
            $"ArcSlab{stairIndex + 1}_{lane}_Render",
            true);
    }

    static void SetMeshColliderEnabled(
        string objectName,
        bool enabled)
    {
        GameObject target = GameObject.Find(objectName);
        if (!target)
            return;

        MeshCollider meshCollider = target.GetComponent<MeshCollider>();
        if (meshCollider)
            meshCollider.enabled = enabled;
    }

    // ================================================================
    // Collider = Support existence only
    // ================================================================

    (bool ,RaycastHit)HasGroundSupport()
    {
        Vector3 origin = rb.worldCenterOfMass + Vector3.up * .05f;

        if (!Physics.SphereCast(
                origin,
                ProbeRadius,
                Vector3.down,
                out RaycastHit hit,
                probeDistance,
                groundMask,
                QueryTriggerInteraction.Ignore))
            return (false,hit);

        return (Vector3.Angle(hit.normal, Vector3.up) <= maxSlopeAngle,hit);
    }

    // ================================================================
    // Spline basis
    // ================================================================

    Surface BuildSplineSurface(NearestKnotDetector.GuideFrame g)
    {
        Vector3 normal = g.normal.sqrMagnitude > Eps ? g.normal.normalized : Vector3.up;
        Vector3 tangent = Vector3.ProjectOnPlane(g.tangent, normal);

        if (tangent.sqrMagnitude <= Eps)
            tangent = Vector3.ProjectOnPlane(direction, normal);

        if (tangent.sqrMagnitude <= Eps)
            return default;

        tangent.Normalize();

        if (Vector3.Dot(tangent, direction) < 0f)
            tangent = -tangent;

        Vector3 side = Vector3.Cross(normal, tangent);

        if (side.sqrMagnitude <= Eps)
            return default;

        side.Normalize();
        tangent = Vector3.Cross(side, normal).normalized;

        Vector3 velocity = rb.velocity;

        return new Surface
        {
            tangent = tangent,
            side = side,
            normal = normal,

            tangentSpeed = Vector3.Dot(velocity, tangent),
            lateralSpeed = Vector3.Dot(velocity, side),
            outwardSpeed = Mathf.Max(0f, Vector3.Dot(velocity, normal)),

            gravityAlong = Vector3.Dot(Physics.gravity, tangent),
            gravitySupport = Mathf.Max(0f, Vector3.Dot(Physics.gravity, -normal))
        };
    }

    float SplineCurvature(NearestKnotDetector.GuideFrame g)
    {
        float entry = Mathf.Max(0f, g.entryCurvature) *
                      (1f - SmoothRange01(g.sectionProgress01, 0f, .25f));

        return Mathf.Max(0f, Mathf.Max(g.curvature, entry));
    }

    // ================================================================
    // Spline Grace
    // ================================================================

    float SupportLoad(NearestKnotDetector.GuideFrame g)
    {
        float time = supportGraceSeconds > Eps
            ? 1f - Mathf.Clamp01(graceTimer / supportGraceSeconds)
            : float.PositiveInfinity;

        float distance = Ratio(g.distanceToGuide, supportGraceMaxGuideDistance);
        float outward = Ratio(Outward(g), maxGraceOutwardSpeed);

        return Mathf.Max(time, Mathf.Max(distance, outward));
    }

    bool CanGrace(NearestKnotDetector.GuideFrame g)
    {
        if (g.isSlope)
            return g.sectionProgress01 < ReleaseEnd;

        return g.nextIsSlope;
    }

    float Outward(NearestKnotDetector.GuideFrame g)
    {
        Vector3 normal = g.normal.sqrMagnitude > Eps ? g.normal.normalized : Vector3.up;
        return Mathf.Max(0f, Vector3.Dot(rb.velocity, normal));
    }

    static float Ratio(float value, float limit)
    {
        value = Mathf.Max(0f, value);

        if (limit <= Eps)
            return value <= Eps ? 0f : float.PositiveInfinity;

        return value / limit;
    }

    // ================================================================
    // Spline Drive
    // ================================================================

    float DesiredDrive(
        Surface s,
        NearestKnotDetector.GuideFrame g,
        bool grace)
    {
        float target;

        if (!g.isSlope)
        {
            target = SpeedDrive(s.tangentSpeed);
        }
        else if (g.sectionProgress01 < TargetProgress)
        {
            float remaining = Mathf.Max(
                TargetMinDistance,
                (TargetProgress - g.sectionProgress01) *
                Mathf.Max(TargetMinDistance, g.sectionLength));

            target =
                (maxGroundSpeed * maxGroundSpeed - s.tangentSpeed * s.tangentSpeed) /
                (2f * Mathf.Max(Eps, remaining))
                - s.gravityAlong;

            target = Mathf.Clamp(target, -TargetAccelerationLimit, TargetAccelerationLimit);
        }
        else
        {
            target = SpeedDrive(s.tangentSpeed);

            float blend = SmoothRange01(
                g.sectionProgress01,
                TargetProgress,
                Mathf.Clamp01(TargetProgress + PostTargetBlendWidth));

            target -= Mathf.Max(0f, s.gravityAlong) * PostTargetGravityRatio * blend;
            target = Mathf.Min(0f, target);
        }

        if (grace)
            target = Mathf.Min(0f, target);

        return Mathf.Max(-MaxDeceleration, target);
    }

    float SpeedDrive(float speed)
    {
        return Mathf.Clamp(
            (maxGroundSpeed - speed) * ResponseInverse,
            -MaxDeceleration,
            GroundAcceleration);
    }

    // ================================================================
    // Unified Spline Stick
    // ================================================================

    float DesiredStick(
        Surface s,
        NearestKnotDetector.GuideFrame g,
        float tangentialAcceleration,
        float curvature,
        bool grace)
    {
        // Spline曲率から必要な法線加速度を求める。
        float forecast = Mathf.Max(
            0f,
            s.tangentSpeed +
            Mathf.Max(0f, tangentialAcceleration) * supportGraceSeconds * .5f);

        float curveNeed = Mathf.Max(
            0f,
            forecast * forecast * curvature * stickSafety - s.gravitySupport);

        float graceNeed = 0f;

        if (grace && s.outwardSpeed > Eps)
        {
            float remainingTime = Mathf.Max(Time.fixedDeltaTime, graceTimer);

            float remainingDistance = Mathf.Max(
                .001f,
                supportGraceMaxGuideDistance - Mathf.Max(0f, g.distanceToGuide));

            float timeBrake = s.outwardSpeed / remainingTime;
            float distanceBrake = s.outwardSpeed * s.outwardSpeed / (2f * remainingDistance);

            graceNeed = Mathf.Max(
                0f,
                Mathf.Max(timeBrake, distanceBrake) - s.gravitySupport);
        }

        // Flat / Curve / Graceを加算せず最大要求だけ採用。
        return Mathf.Clamp(
            Mathf.Max(flatStick, Mathf.Max(curveNeed, graceNeed)),
            0f,
            maxStick);
    }

    // ================================================================
    // Entry
    // ================================================================

    void TransportToSpline(NearestKnotDetector.GuideFrame g)
    {
        Vector3 before = rb.velocity;

        if (before.sqrMagnitude <= Eps || g.normal.sqrMagnitude <= Eps)
            return;

        Vector3 normal = g.normal.normalized;

        if (Vector3.Dot(before, normal) <= .01f)
            return;

        Vector3 projected = Vector3.ProjectOnPlane(before, normal);

        if (projected.sqrMagnitude <= Eps)
            return;

        rb.AddForce(
            projected.normalized * before.magnitude - before,
            ForceMode.VelocityChange);
    }

    void LoseSupport()
    {
        wasSlope = false;
        driveState = 0f;
        stickState = 0f;

        currentSupported = false;
        currentGuideValid = false;
        currentSurfaceValid = false;
        flatEntryFrame = default;

        ResetBallVisualSplineSession();
    }

    // ================================================================
    // Initial Visual Frame Restore
    // ================================================================

    /// <summary>
    /// まだ旋回していないScene初期状態のVisualPlayerRoot / Pivot Poseを保存します。
    /// Awake() と Start() から呼び、未取得の参照だけを補完します。
    /// </summary>
    void CaptureInitialVisualFramePose()
    {
        FindMapFrameReferences();

        // CorrespondSubjectがVisualPlayerRoot/StageRootの正式な所有者。
        // Bind済みならここで初期Poseを保存させる。
        correspondSubject?.CaptureInitialFramePose();

        // CorrespondSubjectが無いSceneでもDirect fallbackを戻せるように、
        // VisualPlayerRootだけはCore側にも初期world Poseを保存する。
        if (!hasInitialVisualPlayerRootPose && visualPlayerRoot)
        {
            initialVisualPlayerRootPosition =
                visualPlayerRoot.position;

            initialVisualPlayerRootRotation =
                visualPlayerRoot.rotation;

            hasInitialVisualPlayerRootPose = true;
        }

        if (logCore && hasInitialVisualPlayerRootPose)
        {
            ;
        }
    }

    /// <summary>
    /// VisualPlayerRoot と Pivot をScene初期Poseへ復元します。
    /// rotationだけでなくpositionも戻すことで、PhysicsRootとの座標対応を復旧します。
    /// </summary>
    public bool RestoreInitialVisualFrame(
        bool synchronizeImmediately = true)
    {
        FindMapFrameReferences();
        BindCoordinateFrames();
        CaptureInitialVisualFramePose();

        bool restored = false;

        // 正式経路。Tween停止 + VisualPlayerRoot + StageRootをまとめて復元。
        if (correspondSubject)
        {
            restored =
                correspondSubject.RestoreInitialFramePose(false);
        }

        // CorrespondSubjectが無い場合の互換Fallback。
        if (!restored &&
            hasInitialVisualPlayerRootPose &&
            visualPlayerRoot)
        {
            visualPlayerRoot.SetPositionAndRotation(
                initialVisualPlayerRootPosition,
                initialVisualPlayerRootRotation);

            restored = true;
        }

        Physics.SyncTransforms();

        if (synchronizeImmediately)
            correspondSubject?.SynchronizeNow(true);

        if (logCore && visualPlayerRoot)
        {
            ;
        }

        return restored;
    }

    /// <summary>
    /// Stage/Spline再生成の直前に呼ぶリセットです。
    /// 古い旋回TweenとCore内部の旋回待ち状態を破棄し、
    /// Visual/Stage座標系をScene初期Poseへ戻します。Rigidbody位置は変更しません。
    /// </summary>
    public bool PrepareForStageRebuild()
    {
        // Tween途中・HardFreeze途中の状態を再生成へ持ち越さない。
        ClearSmoothTurnStateForStageRebuild();

        FindMapFrameReferences();
        BindCoordinateFrames();
        CaptureInitialVisualFramePose();

        // Root復元はStage/Spline生成「前」に1回だけ行う。
        // RestoreInitialVisualFrame -> CorrespondSubject.RestoreInitialFramePose
        // の中で古いTweenも停止されるため、ここで二重Killはしない。
        bool restored =
            RestoreInitialVisualFrame(false);

        // delayStart()へ「これは死亡復帰である」とだけ渡す。
        // 通常のStart()から来たdelayStart()はこのフラグが立たない。
        restartFramePrepared = true;

        Physics.SyncTransforms();

        if (logCore)
        {
            ;
        }

        return restored;
    }

    /// <summary>
    /// 復帰時に物理位置/velocityを確定した瞬間だけ行う最小リセット。
    /// 古いSpline/旋回状態は捨てるが、通常起動には一切適用しない。
    /// </summary>
    void ResetRestartTransientStateSoft()
    {
        pendingTurnDegrees = 0f;
        pendingTurnQueuedTime = -1f;
        trackingFlick = false;
        flickConsumed = false;
        ballVisualSlopeDrive?.CancelTurnHandoffRequest();

        turnTransitionActive = false;
        turnBodyFrozen = false;
        turnTransitionStartTime = -1f;
        activeTurnDegrees = 0f;
        turnTransitionSnapshot = default;

        if (rb && rb.isKinematic)
            rb.isKinematic = false;


        activeTurnMode = TurnResolutionMode.None;
        fiveLineCorrectionPending = false;
        fiveLineCorrectionActive = false;
        fiveLineCorrectionElapsed = 0f;
        fiveLineCorrectionTargetOffset = 0f;
        capturedTurnLandingIntentValid = false;
        capturedTurnLandingIntent = default;
        lastEnergyTargetForwardDistance = float.PositiveInfinity;
        dynamicEnergyTargetTolerance = float.NaN;
        lastFlatTurnLimitProgress = float.NaN;
        flatEntryFrame = default;
        ClearPendingTurnSnapshot();

        waitingForTurnGuide = false;

        // rb.velocityを0へした同じタイミングでController蓄積も0へ揃える。
        // これにより「Stage再生成時点で先にdrive/stickだけ0になる」時間差を作らない。
        driveState = 0f;
        stickState = 0f;
        graceTimer = 0f;
        wasSlope = false;

        // 再生成前のGuide/Surfaceは再生成後Splineでは無効なので破棄する。
        currentSupported = false;
        currentGuideValid = false;
        currentSurfaceValid = false;
        currentGuide = default;
        currentSurface = default;

        // FiveLineGroup自体は強制Centerへ書き換えない。
        // 復帰以外の後続旋回挙動へ余計なバイアスを残さない。

        // BallVisualに再生成前Spline sessionを持ち越させない。
        ResetBallVisualSplineSession();
    }

    /// <summary>
    /// 復帰開始方向の応急処置版。
    /// KnotDetector.Evaluate()には依存せず、実体Physics板のforwardだけを見る。
    /// </summary>
    Vector3 ResolveRestartDirectionSoft(Transform startSlab)
    {
        if (startSlab)
        {
            Vector3 slabForward =
                Vector3.ProjectOnPlane(
                    startSlab.forward,
                    Vector3.up);

            if (slabForward.sqrMagnitude > Eps)
                return slabForward.normalized;
        }

        // 板forwardが取れない時だけ既存のtravelDirectionへフォールバック。
        return NormalizeFlat(
            travelDirection,
            direction);
    }

    public void SetTravelDirection(Vector3 worldDirection)
    {
        direction = NormalizeFlat(worldDirection, direction);
    }

    // ================================================================
    // Map coordinate direction turn
    // SlopeStick3Dから移植。
    // 物理方向変更はCore、Visual座標変換とSubject同期はCorrespondSubjectが担当。
    // ================================================================

    void FindMapFrameReferences()
    {
        if (!physicsRoot)
        {
            GameObject physics = GameObject.Find("PhysicsRoot");
            if (physics)
                physicsRoot = physics.transform;
        }

        if (!visualPlayerRoot)
        {
            GameObject visualRoot = GameObject.Find("VisualPlayerRoot");
            if (visualRoot)
                visualPlayerRoot = visualRoot.transform;
        }

        if (!visualRotationPivot)
        {
            GameObject center = GameObject.Find("Center1");
            if (center)
                visualRotationPivot = center.transform;
        }

        if (!correspondSubject)
            correspondSubject = FindFirstObjectByType<CorrespondSubject>();
    }

    void BindCoordinateFrames()
    {
        if (!correspondSubject)
            return;

        correspondSubject.Bind(
            rb,
            physicsRoot,
            visualPlayerRoot);
    }

    void ReadTurnFlick()
    {
        if (Input.GetMouseButtonDown(0))
        {
            flickStart = Input.mousePosition;
            
            trackingFlick = true;
            flickConsumed = false;
        }

        if (!trackingFlick)
            return;

        // MouseUpを待たず、押下中に横移動が閾値を超えた最初の1回でIntentを確定する。
        // MouseUpも評価に含めるので、短い高速フリックも拾える。
        if (!flickConsumed &&
            (Input.GetMouseButton(0) || Input.GetMouseButtonUp(0)))
        {
            Vector2 flick =
                (Vector2)Input.mousePosition - flickStart;

            if (Mathf.Abs(flick.x) >= minimumFlickPixels &&
                Mathf.Abs(flick.x) > Mathf.Abs(flick.y)&&Earliest)
            {
                flickConsumed = true;
                QueueQuarterTurn(flick.x);
            }
        }

        if (Input.GetMouseButtonUp(0))
        {
            trackingFlick = false;
            flickConsumed = false;
        }
    }

    public void TurnPlayerAndStageLeft()
    {
        QueueQuarterTurn(-1f);
    }

    public void TurnPlayerAndStageRight()
    {
        QueueQuarterTurn(1f);
    }

    // 既存UI/APIからfloat角度で呼ばれても、符号だけを使い必ず90度へ正規化する。
    public void BeginPlayerAndStageTurn(float playerAngle)
    {
        QueueQuarterTurn(playerAngle);
    }

    void QueueQuarterTurn(float directionSign)
    {
        
        if (!Earliest)
                return;
        if (Mathf.Abs(directionSign) <= Eps)
            return;

        pendingTurnDegrees =
            Mathf.Sign(directionSign) * QuarterTurnDegrees;
        pendingTurnQueuedTime = Time.time;

        // Updateで入力を受理した瞬間のPhysics状態を保存する。
        // Turn Handoff後の位置・Targetで判定をすり替えない。
        CapturePendingTurnSnapshot();

        // BallVisualが独立Pose Authority中なら、通常Recoveryを待たず短いTurn Handoffへ入れる。
        RequestTurnHandoffForPendingIntent();

        if (logCore || logTurnTransition)
        {
            ;
        }
    }

    bool HasPendingTurnIntent =>
        Mathf.Abs(pendingTurnDegrees) > Eps;

    bool IsPendingTurnIntentExpired()
    {
        if (!HasPendingTurnIntent || pendingTurnQueuedTime < 0f)
            return false;

        return
            Time.time - pendingTurnQueuedTime >
            Mathf.Max(0.05f, turnInputBufferSeconds);
    }

    void ExpirePendingTurnIntent()
    {
        if (!HasPendingTurnIntent)
            return;

        if (logCore || logTurnTransition)
        {
            ;
        }

        pendingTurnDegrees = 0f;
        pendingTurnQueuedTime = -1f;
        ClearPendingTurnSnapshot();
        ballVisualSlopeDrive?.CancelTurnHandoffRequest();
    }

    void RequestTurnHandoffForPendingIntent()
    {
        if (!HasPendingTurnIntent || !ballVisualSlopeDrive)
            return;

        if (ballVisualSlopeDrive.OwnsBallVisualPose)
            ballVisualSlopeDrive.RequestTurnHandoff();
    }

    // 新しい旋回を「今このFixedUpdateで開始してよいか」だけ判定する。
    bool IsTurnExecutionBlocked()
    {
        if (turnTransitionActive || waitingForTurnGuide)
            return true;

        if (fiveLineCorrectionActive || fiveLineCorrectionPending)
            return true;

        // Turn Handoffが完了してBallVisualのPose Authorityが同期側へ戻るまで待つ。
        if (ballVisualSlopeDrive &&
            !ballVisualSlopeDrive.IsReadyForTurnTransition)
        {
            return true;
        }

        if (correspondSubject && correspondSubject.IsVisualFrameTurning)
            return true;

        return false;
    }

    void ApplyPendingQuarterTurn()
    {
        if (!HasPendingTurnIntent)
            return;

        // Recovery終了後に古い入力が突然実行されることを防ぐ。
        if (IsPendingTurnIntentExpired())
        {
            ExpirePendingTurnIntent();
            return;
        }

        RequestTurnHandoffForPendingIntent();

        if (IsTurnExecutionBlocked())
            return;

        float turnDegrees =
            Mathf.Sign(pendingTurnDegrees) * QuarterTurnDegrees;

        pendingTurnDegrees = 0f;
        pendingTurnQueuedTime = -1f;
        ballVisualSlopeDrive?.CancelTurnHandoffRequest();

        ApplyQuarterTurn(turnDegrees);
    }

    void ApplyQuarterTurn(float turnDegrees)
    {
        if (!rb || turnTransitionActive)
            return;

        turnDegrees = Mathf.Sign(turnDegrees) * QuarterTurnDegrees;

        if (rb.isKinematic)
        {
            ClearPendingTurnSnapshot();
            Debug.LogError(
                "[CORE TURN TRANSITION FAILED] InSubject Rigidbodyは通常走行時Dynamicである必要があります。",
                rb);
            return;
        }

        Quaternion fullDirectionTurn =
            Quaternion.AngleAxis(turnDegrees, Vector3.up);

        Vector3 directionBefore =
            NormalizeFlat(direction, travelDirection);

        Vector3 velocityBefore = rb.velocity;
        Vector3 angularVelocityBefore = rb.angularVelocity;

        turnTargetDirection =
            NormalizeFlat(
                fullDirectionTurn * directionBefore,
                directionBefore);

        // 旋回種別の判定は一度だけ。Flatではフリック時の凍結Snapshotを使う。
        CaptureTurnLandingIntent();
        activeTurnMode = ResolveTurnResolutionMode(directionBefore);
        ClearPendingTurnSnapshot();
        flatEntryFrame = default; // 次のFlatで新しいEntryを採取する。

        bool fiveLineMode =
            activeTurnMode == TurnResolutionMode.FiveLineAfterPop ||
            activeTurnMode == TurnResolutionMode.FiveLineAfterEnergyTarget;

        // OutCorner側の外ラインに既に乗っている場合は、
        // FiveLineで中央側へ取り直さず、その横位置を旋回後ラインとして残す。
        // InCorner(UTurnBeforeEnergyTarget)には一切入らない。
        bool preserveOutCornerLine =
            fiveLineMode &&
            ShouldPreserveOutCornerLine(turnDegrees);

        fiveLineCorrectionPending =
            fiveLineMode &&
            !preserveOutCornerLine;

        if (!BeginTurnTransition(turnDegrees))
            return;

        // FiveLine補正を使わないので、この旋回だけModeを消費済みにする。
        // ReleaseTurnBodyToMotionではUTurn以外が既存のDirectFiveLineTurnへ流れるため、
        // 位置を飛ばさず速度方向だけ90度変える現在仕様をそのまま使える。
        if (preserveOutCornerLine)
        {
            activeTurnMode = TurnResolutionMode.None;

            if (logCore || logTurnPolicy)
            {
                ;
            }
        }

        // 旋回後Splineへ切り替わるまでは通常Driveを再開しない。
        waitingForTurnGuide = true;
        driveState = 0f;
        ResetBallVisualSplineSession();

        FindMapFrameReferences();
        BindCoordinateFrames();

        float visualAngle =
            turnDegrees *
            (visualRootTurnsOppositeToInput ? -1f : 1f);

        bool visualTurnStarted = false;

        if (correspondSubject && visualPlayerRoot )
        {
            Vector3 pivot =
                visualRotationPivot
                    ? visualRotationPivot.position
                    : visualPlayerRoot.position;

            Quaternion visualTurn =
                Quaternion.AngleAxis(visualAngle, Vector3.up);

            visualTurnStarted =
                correspondSubject.RotateVisualFrameAround(
                    pivot,
                    visualTurn,
                    true,
                    OnVisualTurnProgress,
                    CompleteTurnTransition,
                    turnVisualDurationSeconds);
        }

        // 古いScene向けfallback。Direct回転は同一FixedUpdateで完了する。
        if (!visualTurnStarted)
        {
            visualTurnStarted = RotateVisualMapFrameDirect(visualAngle);

            if (visualTurnStarted)
                CompleteTurnTransition();
        }

        if (!visualTurnStarted)
        {
            AbortTurnTransitionAndRestoreOriginal("VisualTurnStartFailed");
            return;
        }

        if (logCore || logTurnPolicy || logTurnTransition)
        {
            ;
        }
    }

    bool BeginTurnTransition(float turnDegrees)
    {
        if (!rb || rb.isKinematic || turnTransitionActive)
            return false;

        turnTransitionSnapshot = new TurnTransitionSnapshot
        {
            valid = true,
            position = rb.position,
            rotation = rb.rotation,
            velocity = rb.velocity,
            angularVelocity = rb.angularVelocity,
            direction = direction,
            useGravity = rb.useGravity
        };

        activeTurnDegrees =
            Mathf.Sign(turnDegrees) * QuarterTurnDegrees;

        // 旋回冒頭だけ完全停止。BallVisual/SubjectのAuthority切替境界を静かにする。
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.isKinematic = true;

        turnTransitionActive = true;
        turnBodyFrozen = true;
        turnTransitionStartTime = Time.fixedTime;

        ResetBallVisualSplineSession();
        return true;
    }

    // FixedUpdateとCorrespondSubjectのTween tickの両方から呼ばれる時間基準の解除判定。
    void AdvanceTurnTransition()
    {
        if (!turnTransitionActive || !turnBodyFrozen)
            return;

        float hardFreeze = Mathf.Max(0f, turnHardFreezeSeconds);
        float elapsed = Mathf.Max(0f, Time.fixedTime - turnTransitionStartTime);

        if (elapsed + Eps < hardFreeze)
            return;

        ReleaseTurnBodyToMotion("HardFreezeElapsed");
    }

    void OnVisualTurnProgress(float visualProgress01)
    {
        if (!turnTransitionActive)
            return;

        AdvanceTurnTransition();
    }

    void ReleaseTurnBodyToMotion(string reason)
    {
        if (!turnTransitionActive ||
            !turnBodyFrozen ||
            !turnTransitionSnapshot.valid ||
            !rb)
        {
            return;
        }

        TurnTransitionSnapshot snapshot = turnTransitionSnapshot;
        Quaternion physicsTurn =
            Quaternion.AngleAxis(activeTurnDegrees, Vector3.up);

        // Freezeした地点から再開する。
        rb.position = snapshot.position;
        rb.rotation = snapshot.rotation;
        rb.isKinematic = false;
        rb.useGravity = snapshot.useGravity;
        rb.velocity = snapshot.velocity;
        rb.angularVelocity = snapshot.angularVelocity;
        rb.WakeUp();

        turnBodyFrozen = false;
        driveState = 0f;

        bool useUTurn =
            activeTurnMode == TurnResolutionMode.UTurnBeforeEnergyTarget;

        if (useUTurn)
        {
            // 旋回種別の確定時にFiveLine予約済み状態を決定している。
            // ここでは重複したPending解除をせず、InCornerの運動だけを実行する。
            ApplyDirectInCornerTurn(snapshot.velocity);
            rb.angularVelocity = physicsTurn * snapshot.angularVelocity;
        }
        else
        {
            // FiveLine系は位置を飛ばさず、速度/headingだけ新方向へ向けてCoast。
            rb.velocity = snapshot.velocity;
            ApplyDirectFiveLineTurn(activeTurnDegrees);
            rb.angularVelocity = physicsTurn * snapshot.angularVelocity;
        }

        ResetBallVisualSplineSession();
        Physics.SyncTransforms();
        correspondSubject?.ResetDerivedVelocitySample();
        correspondSubject?.SynchronizeNow(true);

        if (logCore || logTurnTransition)
        {
            ;
        }
    }

    void CompleteTurnTransition()
    {
        if (!turnTransitionActive || !turnTransitionSnapshot.valid || !rb)
            return;

        float completedTurnDegrees = activeTurnDegrees;

        // Visual TweenがFreeze時間より短い設定でも必ずDynamicへ戻す。
        if (turnBodyFrozen)
            ReleaseTurnBodyToMotion("VisualTurnCompleted");

        turnTransitionActive = false;
        turnBodyFrozen = false;
        turnTransitionStartTime = -1f;
        activeTurnDegrees = 0f;
        turnTransitionSnapshot = default;

        // waitingForTurnGuideはApplyQuarterTurn開始時からtrueのまま。
        // InCornerは既に新方向へ切り替え済み。Spline捕捉へ進む。
        driveState = 0f;
        ResetBallVisualSplineSession();

        Physics.SyncTransforms();
        correspondSubject?.ResetDerivedVelocitySample();
        correspondSubject?.SynchronizeNow(true);

        if (logCore || logTurnTransition)
        {
            ;
        }
    }

    void AbortTurnTransitionAndRestoreOriginal(string reason)
    {
        TurnTransitionSnapshot snapshot = turnTransitionSnapshot;


        if (rb)
        {
            if (snapshot.valid)
            {
                rb.position = snapshot.position;
                rb.rotation = snapshot.rotation;

                if (rb.isKinematic)
                    rb.isKinematic = false;

                rb.useGravity = snapshot.useGravity;
                rb.velocity = snapshot.velocity;
                rb.angularVelocity = snapshot.angularVelocity;
                direction = NormalizeFlat(snapshot.direction, travelDirection);
                turnTargetDirection = direction;
                rb.WakeUp();
            }
            else if (rb.isKinematic)
            {
                rb.isKinematic = false;
            }
        }

        turnTransitionActive = false;
        turnBodyFrozen = false;
        turnTransitionStartTime = -1f;
        activeTurnDegrees = 0f;
        turnTransitionSnapshot = default;
        waitingForTurnGuide = false;

        activeTurnMode = TurnResolutionMode.None;
        fiveLineCorrectionPending = false;
        fiveLineCorrectionActive = false;
        capturedTurnLandingIntentValid = false;
        capturedTurnLandingIntent = default;
        flatEntryFrame = default;
        ClearPendingTurnSnapshot();

        Physics.SyncTransforms();
        correspondSubject?.ResetDerivedVelocitySample();
        correspondSubject?.SynchronizeNow(true);

        if (logCore || logTurnTransition)
        {
            Debug.LogWarning(
                $"[CORE TURN TRANSITION ABORT] reason={reason} " +
                $"position={(rb ? rb.position : Vector3.zero):F4} " +
                $"velocity={(rb ? rb.velocity : Vector3.zero):F4}",
                this);
        }
    }

    void ClearSmoothTurnStateForStageRebuild()
    {
        pendingTurnDegrees = 0f;
        pendingTurnQueuedTime = -1f;
        trackingFlick = false;
        flickConsumed = false;
        ballVisualSlopeDrive?.CancelTurnHandoffRequest();

        correspondSubject?.CancelVisualFrameTurn(false);

        turnTransitionActive = false;
        turnBodyFrozen = false;
        turnTransitionStartTime = -1f;
        activeTurnDegrees = 0f;
        turnTransitionSnapshot = default;
        waitingForTurnGuide = false;

        activeTurnMode = TurnResolutionMode.None;
        fiveLineCorrectionPending = false;
        fiveLineCorrectionActive = false;
        fiveLineCorrectionElapsed = 0f;
        fiveLineCorrectionTargetOffset = 0f;
        capturedTurnLandingIntentValid = false;
        capturedTurnLandingIntent = default;
        lastEnergyTargetForwardDistance = float.PositiveInfinity;
        dynamicEnergyTargetTolerance = float.NaN;
        lastFlatTurnLimitProgress = float.NaN;
        flatEntryFrame = default;
        ClearPendingTurnSnapshot();

        if (rb && rb.isKinematic)
            rb.isKinematic = false;

        if (rb)
        {
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }

    // 同じFlat上の進行距離で「次の斜面入口へ安全に曲がり終われるか」を判定する。
    // 従来の前後通過判定とは意味が異なるが、既存の3モードと後続の物理実行は維持。
    TurnResolutionMode ResolveTurnResolutionMode(Vector3 directionBefore)
    {
        if (BallVisualIsOnSlope || BallVisualIsAir)
            return TurnResolutionMode.FiveLineAfterPop;

        if (!BallVisualIsOnFlat)
            return TurnResolutionMode.FiveLineAfterPop;

        if (!pendingTurnSnapshotValid || !pendingTurnWasFlat || !pendingTurnTargetValid)
        {
            if (logTurnPolicy)
                ;
            return TurnResolutionMode.FiveLineAfterEnergyTarget;
        }

        Vector3 measuringDirection = pendingFlatEntryFrame.valid
            ? pendingFlatEntryFrame.direction
            : NormalizeFlat(directionBefore, travelDirection);

        lastEnergyTargetForwardDistance = Vector3.Dot(
            pendingTurnTargetPhysics - pendingTurnFlickPosition,
            measuringDirection);

        if (!TryCalculateDynamicEnergyTolerance(out float tolerance))
        {
            if (logTurnPolicy)
                ;
            return TurnResolutionMode.FiveLineAfterEnergyTarget;
        }

        // Apply時に多少進んだ場合も、実際のInCorner開始位置が境界を越えていないか検査。
        float executionProgress = Vector3.Dot(
            rb.position - pendingFlatEntryFrame.position,
            pendingFlatEntryFrame.direction);

        float forwardSpeedAtFlick = Mathf.Max(0f, Vector3.Dot(
            pendingTurnFlickVelocity, pendingFlatEntryFrame.direction));
        float entryNumericalAllowance = Mathf.Max(Eps, forwardSpeedAtFlick * Time.fixedDeltaTime);
        bool clearanceStillAvailable =
            executionProgress >= -entryNumericalAllowance &&
            executionProgress < lastFlatTurnLimitProgress;

        bool useUTurn =
            lastEnergyTargetForwardDistance > tolerance &&
            clearanceStillAvailable;

        if (useUTurn)
        {
            ;
        }
        if (logTurnPolicy)
        {
            ;
        }

        return useUTurn
            ? TurnResolutionMode.UTurnBeforeEnergyTarget
            : TurnResolutionMode.FiveLineAfterEnergyTarget;
    }

    bool TryCalculateDynamicEnergyTolerance(out float tolerance)
    {
        tolerance = float.NaN;
        dynamicEnergyTargetTolerance = float.NaN;
        lastFlatTurnLimitProgress = float.NaN;

        FlatEntryFrame entry = pendingFlatEntryFrame;
        if (!entry.valid || !pendingTurnTargetValid ||
            !IsFiniteFloat(entry.distanceToNextSlope) ||
            entry.distanceToNextSlope <= Eps ||
            !currentGuideValid || currentGuide.splineIndex != entry.splineIndex)
            return false;
        
        //平面入り口からフリックした距離
        float entryToFlick = Vector3.Dot(
            pendingTurnFlickPosition - entry.position, entry.direction);
        //平面入り口からBallVisualからもらった着地点の距離
        float entryToTarget = Vector3.Dot(
            pendingTurnTargetPhysics - entry.position, entry.direction);

        float forwardSpeed = Mathf.Max(0f, Vector3.Dot(
            pendingTurnFlickVelocity, entry.direction));
        float motionAllowance = forwardSpeed * Time.fixedDeltaTime;

        // InCornerでは旧方向にBezier半径Rを進めない。球のプローブ幅と1物理ステップだけ確保。
        float clearance = ProbeRadius + motionAllowance;
        if (!knotDetector)
            return false;
        knotDetector.TryGetNextSlopePosition();

        Vector3 nextSlopePosition = knotDetector.nextSlopePosition;
        if (!IsFiniteVector(nextSlopePosition))
            return false;
        float flatLength = Vector3.Dot(nextSlopePosition - entry.position, entry.direction);
        if (!IsFiniteFloat(flatLength) || flatLength <= Eps)
            return false;

        float latestTurnProgress = flatLength - clearance;

        if (!IsFiniteFloat(entryToFlick) || !IsFiniteFloat(entryToTarget) ||
            latestTurnProgress <= Eps ||
            entryToFlick < motionAllowance ||
            entryToFlick > flatLength + motionAllowance
            )
            return false;

        lastFlatTurnLimitProgress = latestTurnProgress;
        tolerance = entryToTarget - latestTurnProgress;
        dynamicEnergyTargetTolerance = tolerance;
        return IsFiniteFloat(tolerance);
    }
   

    // 同じSplineで次の斜面が確認できた最初のFlat支持フレームを基準にする。
    void UpdateFlatEntryFrame(NearestKnotDetector.GuideFrame guide)
    {
        if (!guide.valid || guide.isSlope)
        {
            flatEntryFrame = default;
            return;
        }

        if (flatEntryFrame.valid && flatEntryFrame.splineIndex == guide.splineIndex)
            return;

        flatEntryFrame = default;
        if (!guide.nextIsSlope ||
            !IsFiniteFloat(guide.distanceToNextSlope) ||
            guide.distanceToNextSlope <= Eps)
            return;

        Vector3 entryDirection = NormalizeFlat(direction, guide.tangent);
        Vector3 guideDirection = NormalizeFlat(guide.tangent, entryDirection);
        // 距離はSpline沿い、投影は直線方向。非直線Flatを誤判定しないため拒否。
        if (Vector3.Dot(entryDirection, guideDirection) < 0.95f)
            return;

        flatEntryFrame = new FlatEntryFrame
        {
            valid = true,
            splineIndex = guide.splineIndex,
            position = rb.position,
            direction = entryDirection,
            distanceToNextSlope = guide.distanceToNextSlope
        };

        if (logTurnPolicy)
            ;
    }

    void CapturePendingTurnSnapshot()
    {
        ClearPendingTurnSnapshot();
        if (!rb)
            return;

        pendingTurnSnapshotValid = true;
        pendingTurnWasFlat = BallVisualIsOnFlat;
        pendingTurnFlickPosition = rb.position;
        pendingTurnFlickVelocity = rb.velocity;
        pendingFlatEntryFrame = flatEntryFrame;
        
        ;

        if (!pendingTurnWasFlat)
            return;

        if (!ballVisualSlopeDrive)
            ballVisualSlopeDrive = FindObjectOfType<BallVisualSlopeDrive>(true);
        if (!ballVisualSlopeDrive)
            return;

        if (!ballVisualSlopeDrive.TryGetTurnLandingIntent(
                out BallVisualSlopeDrive.TurnLandingIntent intent) || !intent.valid)
            return;

        // TargetはVisual座標。座標写像がなければ安全側(FiveLine)へ倒す。
        if (!correspondSubject)
            return;

        Vector3 physicsTarget =
            correspondSubject.InverseMapPoint(intent.positionVisual);

        if (!IsFiniteVector(physicsTarget))
            return;

        pendingTurnTargetValid = true;
        pendingTurnTargetPhysics = physicsTarget;
        pendingTurnLandingIntent = intent;
    }

    void ClearPendingTurnSnapshot()
    {
        pendingTurnSnapshotValid = false;
        pendingTurnWasFlat = false;
        pendingFlatEntryFrame = default;
        pendingTurnFlickPosition = default;
        pendingTurnFlickVelocity = default;
        pendingTurnTargetValid = false;
        pendingTurnTargetPhysics = default;
        pendingTurnLandingIntent = default;
    }

    static bool IsFiniteFloat(float value) =>
        !float.IsNaN(value) && !float.IsInfinity(value);

    static bool IsFiniteVector(Vector3 value) =>
        IsFiniteFloat(value.x) && IsFiniteFloat(value.y) && IsFiniteFloat(value.z);

    void CaptureTurnLandingIntent()
    {
        capturedTurnLandingIntentValid = false;
        capturedTurnLandingIntent = default;
        lastEnergyTargetForwardDistance = float.PositiveInfinity;
        dynamicEnergyTargetTolerance = float.NaN;
        lastFlatTurnLimitProgress = float.NaN;

        if (pendingTurnWasFlat)
        {
            if (pendingTurnTargetValid)
            {
                capturedTurnLandingIntent = pendingTurnLandingIntent;
                capturedTurnLandingIntentValid = true;
            }
            return;
        }

        if (!ballVisualSlopeDrive)
            ballVisualSlopeDrive = FindObjectOfType<BallVisualSlopeDrive>(true);

        if (ballVisualSlopeDrive &&
            ballVisualSlopeDrive.TryGetTurnLandingIntent(
                out BallVisualSlopeDrive.TurnLandingIntent intent) &&
            intent.valid)
        {
            capturedTurnLandingIntent = intent;
            capturedTurnLandingIntentValid = true;
        }
    }

    bool TryGetCapturedLandingIntentPhysics(out Vector3 targetPhysics)
    {
        targetPhysics = rb ? rb.position : transform.position;

        BallVisualSlopeDrive.TurnLandingIntent intent = capturedTurnLandingIntent;
        bool valid = capturedTurnLandingIntentValid;

        // FiveLineAfterPopではターン開始後にEnergy targetが更新される場合があるため、
        // 補正開始時は最新Intentを優先する。
        if (ballVisualSlopeDrive &&
            ballVisualSlopeDrive.TryGetTurnLandingIntent(
                out BallVisualSlopeDrive.TurnLandingIntent latest) &&
            latest.valid)
        {
            intent = latest;
            valid = true;
        }

        if (!valid)
            return false;

        targetPhysics = correspondSubject
            ? correspondSubject.InverseMapPoint(intent.positionVisual)
            : intent.positionVisual;

        return
            !float.IsNaN(targetPhysics.x) &&
            !float.IsNaN(targetPhysics.y) &&
            !float.IsNaN(targetPhysics.z) &&
            !float.IsInfinity(targetPhysics.x) &&
            !float.IsInfinity(targetPhysics.y) &&
            !float.IsInfinity(targetPhysics.z);
    }

    void ApplyDirectFiveLineTurn(float turnDegrees)
    {
        float verticalSpeed = Vector3.Dot(rb.velocity, Vector3.up);
        Vector3 planarVelocity = Vector3.ProjectOnPlane(rb.velocity, Vector3.up);

        if (planarVelocity.sqrMagnitude > Eps * Eps)
        {
            Quaternion turn = Quaternion.AngleAxis(turnDegrees, Vector3.up);
            planarVelocity = turn * planarVelocity;
        }

        rb.velocity = planarVelocity + Vector3.up * verticalSpeed;
        direction = turnTargetDirection;
        rb.WakeUp();
    }

    // InCorner: 旧進行方向へ前進する円弧を作らず、旋回開始位置から新方向へ向かう。
    // 速度の水平大きさとY速度は保持する。FiveLine予約はApplyQuarterTurnで決定済み。
    void ApplyDirectInCornerTurn(Vector3 originalVelocity)
    {
        float verticalSpeed = Vector3.Dot(originalVelocity, Vector3.up);
        float planarSpeed = Vector3.ProjectOnPlane(originalVelocity, Vector3.up).magnitude;
        direction = turnTargetDirection;
        rb.velocity = direction * planarSpeed + Vector3.up * verticalSpeed;
        rb.WakeUp();

        if (logTurnTransition)
            ;
    }

    bool ShouldPreserveOutCornerLine(float turnDegrees)
    {
        if (!rb ||
            !knotDetector ||
            !currentGuideValid ||
            !currentSurfaceValid)
        {
            return false;
        }

        if (!knotDetector.TryGetFiveLineFrame(
                currentGuide,
                currentSurface.side,
                out NearestKnotDetector.FiveLineFrame frame) ||
            !frame.valid)
        {
            return false;
        }

        Vector3 outwardSide =
            -Mathf.Sign(turnDegrees) *
            currentSurface.side.normalized;

        float outerDistance =
            Mathf.Max(
                Vector3.Dot(frame.left - frame.center, outwardSide),
                Vector3.Dot(frame.right - frame.center, outwardSide));

        if (outerDistance <= Eps)
            return false;

        float outwardOffset =
            Vector3.Dot(
                rb.position - frame.center,
                outwardSide);

        // Centerから最外ラインまでの半分より外なら、既存の横ラインを優先する。
        return outwardOffset >= outerDistance * 0.5f;
    }

    // ================================================================
    // FiveLine landing correction
    // ================================================================

    void TryBeginPendingFiveLineCorrection(
        NearestKnotDetector.GuideFrame guide,
        ref Surface surface)
    {
        if (!fiveLineCorrectionPending ||
            fiveLineCorrectionActive ||
            waitingForTurnGuide ||
            turnTransitionActive)
        {
            return;
        }

        if (correspondSubject && correspondSubject.IsVisualFrameTurning)
            return;

        // 上段POP -> 下段ではBallVisualの軌道権威が返るまで待つ。
        // Flat-after-targetでも同じGateを使うことでPose二重所有を避ける。
        if (ballVisualSlopeDrive &&
            !ballVisualSlopeDrive.IsStableForFiveLineCorrection)
        {
            return;
        }

        if (!rb || !knotDetector || !guide.valid || !surface.Valid)
            return;

        // FiveLineは下階段側でのみ適用する。Flat上ではターゲット選択を急がない。
        if (!guide.isSlope)
            return;

        if (!knotDetector.TryGetFiveLineFrame(
                guide,
                surface.side,
                out NearestKnotDetector.FiveLineFrame frame) ||
            !frame.valid)
        {
            return;
        }

        Vector3 side = surface.side.normalized;
        Vector3 selectorPosition = rb.position;

        if (TryGetCapturedLandingIntentPhysics(out Vector3 landingPhysics))
            selectorPosition = landingPhysics;

        NearestKnotDetector.FiveLineGroup selectedGroup =
            NearestKnotDetector.FiveLineGroup.Center;

        Vector3 selectedPoint = frame.center;
        float bestAbsSideError = float.PositiveInfinity;

        for (int i = 0; i < 5; i++)
        {
            NearestKnotDetector.FiveLineGroup group =
                (NearestKnotDetector.FiveLineGroup)i;

            Vector3 point = frame.GetPoint(group);
            float sideError = Mathf.Abs(
                Vector3.Dot(point - selectorPosition, side));

            if (sideError < bestAbsSideError)
            {
                bestAbsSideError = sideError;
                selectedGroup = group;
                selectedPoint = point;
            }
        }

        float rawCorrection =
            Vector3.Dot(selectedPoint - rb.position, side);

        float correction = Mathf.Clamp(
            rawCorrection,
            -Mathf.Max(0.05f, fiveLineMaximumCorrectionMeters),
            Mathf.Max(0.05f, fiveLineMaximumCorrectionMeters));

        fiveLineTargetGroup = selectedGroup;
        fiveLineCorrectionSide = side;
        fiveLineCorrectionStartPosition = rb.position;
        fiveLineCorrectionTargetOffset = correction;
        fiveLineCorrectionElapsed = 0f;
        fiveLineCorrectionActive = Mathf.Abs(correction) > 0.001f;
        fiveLineCorrectionPending = false;

        if (!fiveLineCorrectionActive)
        {
            CompleteFiveLineCorrection(ref surface);
            return;
        }

        if (logCore || logTurnPolicy)
        {
            ;
        }
    }

    void StepFiveLineCorrection(ref Surface surface)
    {
        if (!fiveLineCorrectionActive || !rb || !surface.Valid)
            return;

        float dt = Mathf.Max(Time.fixedDeltaTime, Eps);
        float duration = Mathf.Max(dt, fiveLineCorrectionDurationSeconds);
        float nextElapsed = Mathf.Min(duration, fiveLineCorrectionElapsed + dt);
        float t = Mathf.Clamp01(nextElapsed / duration);
        float smoothT = t * t * (3f - 2f * t);

        float desiredOffset = fiveLineCorrectionTargetOffset * smoothT;
        float currentOffset = Vector3.Dot(
            rb.position - fiveLineCorrectionStartPosition,
            fiveLineCorrectionSide);

        float desiredLateralSpeed =
            (desiredOffset - currentOffset) / dt;

        float currentLateralSpeed =
            Vector3.Dot(rb.velocity, fiveLineCorrectionSide);

        rb.velocity +=
            fiveLineCorrectionSide *
            (desiredLateralSpeed - currentLateralSpeed);

        fiveLineCorrectionElapsed = nextElapsed;
        rb.WakeUp();

        if (t < 1f - Eps)
            return;

        CompleteFiveLineCorrection(ref surface);
    }

    void CompleteFiveLineCorrection(ref Surface surface)
    {
        fiveLineCorrectionActive = false;

        if (rb && fiveLineCorrectionSide.sqrMagnitude > Eps)
        {
            float lateralSpeed =
                Vector3.Dot(rb.velocity, fiveLineCorrectionSide);

            rb.velocity -= fiveLineCorrectionSide * lateralSpeed;
        }

        activeTurnMode = TurnResolutionMode.None;
        capturedTurnLandingIntentValid = false;
        capturedTurnLandingIntent = default;
        ResetBallVisualSplineSession();

        if (surface.Valid)
        {
            surface = BuildSplineSurface(currentGuide);
            currentSurface = surface;
            currentSurfaceValid = surface.Valid;
        }

        if (logCore || logTurnPolicy)
        {
            ;
        }
    }

    bool RotateVisualMapFrameDirect(float visualAngle)
    {
        if (!visualPlayerRoot)
            return false;

        Quaternion worldTurn = Quaternion.AngleAxis(visualAngle, Vector3.up);
        Vector3 pivot = visualRotationPivot
            ? visualRotationPivot.position
            : visualPlayerRoot.position;

        Vector3 relative = visualPlayerRoot.position - pivot;

        visualPlayerRoot.SetPositionAndRotation(
            pivot + worldTurn * relative,
            worldTurn * visualPlayerRoot.rotation);

        return true;
    }

    // ================================================================
    // Helpers
    // ================================================================

    static float Move(float current, float target, float jerk) =>
        Mathf.MoveTowards(current, target, jerk * Time.fixedDeltaTime);

    static Vector3 NormalizeFlat(Vector3 value, Vector3 fallback)
    {
        Vector3 flat = Vector3.ProjectOnPlane(value, Vector3.up);

        if (flat.sqrMagnitude <= Eps)
            flat = Vector3.ProjectOnPlane(fallback, Vector3.up);

        return (flat.sqrMagnitude <= Eps ? Vector3.forward : flat).normalized;
    }

    static float SmoothRange01(float value, float start, float end)
    {
        if (end <= start + Eps)
            return value >= end ? 1f : 0f;

        float t = Mathf.Clamp01(Mathf.InverseLerp(start, end, value));
        return t * t * t * (t * (t * 6f - 15f) + 10f);
    }
    
}

public interface IVisualProxyBindable
{
    void BindVisualProxy(Transform proxy);
    void UnbindVisualProxy();
}