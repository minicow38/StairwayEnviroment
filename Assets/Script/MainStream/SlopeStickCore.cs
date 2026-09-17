using UnityEngine;
using System.Collections;
using System.Text.RegularExpressions;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody), typeof(SphereCollider), typeof(NearestKnotDetector))]
public sealed class SlopeStickCore : MonoBehaviour
{
    const float Eps = 0.000001f;

    [Min(0f)] [SerializeField] float maxGroundSpeed = 24f;

    // Compactから固定値化
    const float ProbeRadius = .475f;
    [SerializeField] public float GroundAcceleration = 35f;
    const float MaxDeceleration = 80f;
    const float ResponseInverse = 8.333333f;
    const float AccelerationJerk = 600f;

    const float TargetMinDistance = .27f;
    [SerializeField] public float TargetAccelerationLimit = 120f;
    const float PostTargetBlendWidth = .05f;
    const float PostTargetGravityRatio = .50f;

    RaycastHit currentHit;
    const float StickJerk = 3000f;
    const float ReleaseHold = .02f;
    const float NaturalReleaseEnd = .90f;

    [SerializeField] NearestKnotDetector knotDetector;
    [SerializeField] LayerMask groundMask = ~0;

    [Header("Travel")] [SerializeField] Vector3 travelDirection = Vector3.forward;

    [Range(0f, 100f)] [SerializeField] public float targetSlopeProgressPercent = 60f;

    [Header("Coordinate Mapping")] [Tooltip("PhysicsRoot上のInSubjectをVisualPlayerRoot側へ写す座標変換担当です。")] [SerializeField]
    CorrespondSubject correspondSubject;

    [Tooltip("Energy target / POP状態をREAD ONLY参照するBallVisual軌道担当です。")] [SerializeField]
    BallVisualSlopeDrive ballVisualSlopeDrive;

    [Tooltip("回転させない物理座標系。InSubjectはこの配下で物理計算します。")] [SerializeField]
    Transform physicsRoot;

    [Header("Map Direction Turn")] [Tooltip("回転する表示座標系。InSubject/PhysicsRootは回転させません。")] [SerializeField]
    Transform visualPlayerRoot;

    [Tooltip("VisualPlayerRootを回すワールドPivot。未設定ならCenter1を検索します。")] [SerializeField]
    Transform visualRotationPivot;

    [Tooltip("ONなら入力方向と反対へVisualPlayerRootを回し、マップが逆向きに旋回して見えるようにします。")] [SerializeField]
    bool visualRootTurnsOppositeToInput = true;

    [Tooltip("これ未満の横フリックは旋回として扱いません。MouseUpを待たず、押下中にこの閾値を超えた瞬間にTurn Input Intentを確定します。")] [Min(1f)] [SerializeField]
    float minimumFlickPixels = 10f;

    [Tooltip("Turn Input Intentを保持する最大時間[s]。古いフリックがRecovery/前回Turn終了後に突然実行されることを防ぎます。")] [Min(0.05f)] [SerializeField]
    float turnInputBufferSeconds = 0.25f;

    // 旋回角度は1系統だけ。入力強度や呼び出し元に関係なく必ず90度。
    const float QuarterTurnDegrees = 90f;

    public enum TurnPostCorrectionMode
    {
        None,
        FiveLineAfterPop
    }

    [Header("Turn Transition / Landing Correction")]
    [Tooltip("VisualPlayerRootの90度旋回時間[s]。SlopeStickCoreからCorrespondSubjectへ明示的に渡します。")]
    [Min(0.05f)]
    [SerializeField]
    float turnVisualDurationSeconds = 0.30f;

    [Tooltip("旋回開始後、InSubjectを完全Kinematic停止する時間[s]。この時間を過ぎたらVisual旋回中でもDynamicへ戻してCoastします。")]
    [Min(0f)]
    [SerializeField]
    float turnHardFreezeSeconds = 0.12f;

    [Tooltip("Slope/Air中の旋回後、BallVisualの着地レーン整理としてFiveLine補正を使います。Flat旋回では使いません。")] [SerializeField]
    bool enableFiveLineAfterPop = true;

    [Tooltip("FiveLine横補正を滑らかに収束させる時間[s]。")] [Min(0.02f)] [SerializeField]
    float fiveLineCorrectionDurationSeconds = 0.10f;

    [Tooltip("FiveLineが一度に許す最大横補正距離[m]。遠距離スナップを防ぎます。")] [Min(0.05f)] [SerializeField]
    float fiveLineMaximumCorrectionMeters = 0.75f;

    [Tooltip("Turn Transition / FiveLineの状態をログへ出します。")] [SerializeField]
    bool logTurnTransition = true;

    [Header("Spline Support")] [Min(.01f)] [SerializeField]
    float probeDistance = .85f;

    [Range(1f, 89f)] [SerializeField] float maxSlopeAngle = 75f;

    [Min(0f)] [SerializeField] float supportGraceSeconds = .12f;
    [Min(0f)] [SerializeField] float supportGraceMaxGuideDistance = 1.75f;
    [Min(0f)] [SerializeField] float maxGraceOutwardSpeed = 2f;

    [Header("Stick")] [Min(0f)] [SerializeField]
    float flatStick = 24.6f;

    [Min(0f)] [SerializeField] float maxStick = 1000f;
    [Min(1f)] [SerializeField] float stickSafety = 1.10f;

    [Header("PlayerMotivation")] [SerializeField]
    public bool BeginCommandOnTouch = false;

    [Header("Debug")] [SerializeField] bool logCore;

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

    // Visual旋回完了後、NearestKnotDetectorが旋回後Splineを捕捉するまでだけtrue。
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

    // TurnTransition全体と、本当にRigidbodyをKinematic停止している区間を分離する。
    // turnTransitionActive: Visual旋回開始～Visual旋回完了まで。
    // turnBodyFrozen: その先頭の短いHardFreeze区間だけ。
    bool turnTransitionActive;
    bool turnBodyFrozen;
    float turnTransitionStartTime = -1f;
    float activeTurnDegrees;
    TurnTransitionSnapshot turnTransitionSnapshot;

    TurnPostCorrectionMode activeTurnPostCorrectionMode = TurnPostCorrectionMode.None;
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

    public bool IsTurnTransitionActive => turnTransitionActive;
    public bool IsTurnBodyFrozen => turnBodyFrozen;

    // 旧外部参照との互換。旧プロパティは従来どおり「旋回処理全体が進行中」を返す。
    // 新コードでは IsTurnTransitionActive / IsTurnBodyFrozen を使い分ける。
    public bool IsTurnFreezeActive => turnTransitionActive;

    public TurnPostCorrectionMode CurrentTurnPostCorrectionMode => activeTurnPostCorrectionMode;
    public bool IsFiveLineCorrectionPending => fiveLineCorrectionPending;
    public bool IsFiveLineCorrectionActive => fiveLineCorrectionActive;
    public NearestKnotDetector.FiveLineGroup CurrentFiveLineTargetGroup => fiveLineTargetGroup;

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
        !turnTransitionActive;

    public bool IsWaitingForTurnGuide =>
        waitingForTurnGuide || turnTransitionActive;

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
                Mathf.Sqrt(Mathf.Max(0f,
                    (maxStick + targetGravitySupport) / Mathf.Max(Eps, targetCurvature * stickSafety)));

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


    public IEnumerator delayStart()
    {
        Material activeMaterial = Resources.Load<Material>("BallCollections/" + AndroidOneOnly.activeBallMaterial);
        GameObject.Find("VisualPlayerRoot/BallVisualEqualizer")
            .GetComponent<MeshRenderer>()
            .material = activeMaterial;

        yield return new WaitForSeconds(0.3f);
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

        // 旋回中に死亡/再構築へ入ってもKinematicを持ち越さない。
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
                Debug.Log(
                    $"[CORE SOFT RESTART APPLIED] " +
                    $"restart={restart:F4} direction={direction:F4}",
                    this);
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

        // TurnTransition中はSpline Drive / Stick / FiveLineを止める。
        // ただしHardFreeze終了後はRigidbody自体をDynamicへ戻すため、
        // このreturn中でもPhysXの慣性・Gravity・Collider応答は継続できる。
        if (turnTransitionActive)
        {
            AdvanceTurnTransition();
            ResetBallVisualSplineSession();
            return;
        }

        // Visual旋回完了後は、旋回後Splineを捕捉するまで新しいheadingを保持する。
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

        (bool grounded, RaycastHit hit) = HasGroundSupport();

        if (grounded)
        {
            graceTimer = supportGraceSeconds;
            currentHit = hit;
        }
        else
            graceTimer = Mathf.Max(0f, graceTimer - Time.fixedDeltaTime);

        if (currentHit.transform != null)
            if (Vector3.Distance(transform.position, currentHit.transform.position) > 12)
            {

                if (!MainGameManager.OnDead)
                {
                    StartCoroutine(Recover());
                }

                MainGameManager.OnDead = true;
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
                Debug.Log(
                    $"[CORE SUPPORT LOST] load={load:F3} dist={guide.distanceToGuide:F3} outward={Outward(guide):F3}");

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

            if (flatGuideTangent.sqrMagnitude > Eps)
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

                // Freeze解除後はtargetDirectionの復元速度を保持したまま、
                // 新Splineが捕捉されるまで旧SplineのDriveだけを止める。
                return;
            }

            // 旋回後Splineを捕捉した。
            waitingForTurnGuide = false;
            driveState = 0f;

            // Flat旋回は後補正を持たないため、Spline捕捉時点でTurn状態を完了する。
            if (!fiveLineCorrectionPending)
            {
                activeTurnPostCorrectionMode = TurnPostCorrectionMode.None;
                capturedTurnLandingIntentValid = false;
                capturedTurnLandingIntent = default;
            }
        }

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

        if (Input.GetMouseButtonDown(0))
        {
            /* MainGameManager.TopTitle.SetActive(false);
             MainGameManager.PreviewIconRoot.SetActive(false);
             MainGameManager.TopLiteral.SetActive(false);
             MainGameManager.PlayButton.SetActive(false);
             MainGameManager.Userbility.SetActive(true);

           BeginCommandOnTouch = true;*/
        }

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
                Debug.Log(
                    $"[CORE] speed={surface.tangentSpeed:F3} " +
                    $"progress={guide.sectionProgress01:F3} " +
                    $"drive={driveState:F3} stick={stickState:F3} " +
                    $"grace={grace} load={load:F3}");
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

    (bool, RaycastHit) HasGroundSupport()
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
            return (false, hit);

        return (Vector3.Angle(hit.normal, Vector3.up) <= maxSlopeAngle, hit);
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
            Debug.Log(
                $"[CORE INITIAL VISUAL FRAME CAPTURED] " +
                $"rootPos={initialVisualPlayerRootPosition:F4} " +
                $"rootRot={initialVisualPlayerRootRotation.eulerAngles:F2}",
                this);
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
            Debug.Log(
                $"[CORE INITIAL VISUAL FRAME RESTORED] " +
                $"rootPos={visualPlayerRoot.position:F4} " +
                $"rootRot={visualPlayerRoot.rotation.eulerAngles:F2} " +
                $"restored={restored}",
                this);
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
        // 旋回Tweenが途中でも、再構築へ旋回前Snapshot/Kinematic状態を持ち越さない。
        ClearTurnTransitionForStageRebuild();

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
            Debug.Log(
                $"[CORE SOFT RESTART PREPARED] restored={restored}",
                this);
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

        activeTurnPostCorrectionMode = TurnPostCorrectionMode.None;
        fiveLineCorrectionPending = false;
        fiveLineCorrectionActive = false;
        fiveLineCorrectionElapsed = 0f;
        fiveLineCorrectionTargetOffset = 0f;
        capturedTurnLandingIntentValid = false;
        capturedTurnLandingIntent = default;

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

        // MouseUpを待たない。押下中に閾値を超えた最初の1回だけTurn Input Intentへ変換する。
        // GetMouseButtonUpも評価対象に残すことで、Down->Upが短い高速フリックも拾う。
        if (!flickConsumed &&
            (Input.GetMouseButton(0) || Input.GetMouseButtonUp(0)))
        {
            Vector2 flick =
                (Vector2)Input.mousePosition - flickStart;

            if (Mathf.Abs(flick.x) >= minimumFlickPixels &&
                Mathf.Abs(flick.x) > Mathf.Abs(flick.y))
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
        if (Mathf.Abs(directionSign) <= Eps)
            return;

        // Turn Input Intentは常に最新1件だけ保持する。
        // ただし永久予約にはせず、turnInputBufferSecondsを超えた古い入力は破棄する。
        pendingTurnDegrees =
            Mathf.Sign(directionSign) * QuarterTurnDegrees;
        pendingTurnQueuedTime = Time.time;

        // BallVisualが独立Pose Authorityを持っている場合は、通常Recovery完了を待つのではなく、
        // Turn専用の短時間Handoff Rejoinを即座に要求する。
        RequestTurnHandoffForPendingIntent();

        if (logCore || logTurnTransition)
        {
            Debug.Log(
                $"[CORE TURN INPUT INTENT] " +
                $"time={Time.fixedTime:F4} " +
                $"angle={pendingTurnDegrees:F1} " +
                $"buffer={turnInputBufferSeconds:F3}s " +
                $"blocked={IsTurnExecutionBlocked()} " +
                $"transition={turnTransitionActive} bodyFrozen={turnBodyFrozen} " +
                $"fiveLineActive={fiveLineCorrectionActive} " +
                $"fiveLinePending={fiveLineCorrectionPending} " +
                $"ballVisualOwnsPose={(ballVisualSlopeDrive && ballVisualSlopeDrive.OwnsBallVisualPose)} " +
                $"turnHandoff={(ballVisualSlopeDrive && ballVisualSlopeDrive.IsTurnHandoffActive)} " +
                $"visualTurning={(correspondSubject && correspondSubject.IsVisualFrameTurning)} " +
                $"direction={direction:F4} " +
                $"velocity={(rb ? rb.velocity : Vector3.zero):F4}",
                this);
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
            Debug.Log(
                $"[CORE TURN INPUT INTENT EXPIRED] " +
                $"time={Time.fixedTime:F4} " +
                $"angle={pendingTurnDegrees:F1} " +
                $"age={(pendingTurnQueuedTime >= 0f ? Time.time - pendingTurnQueuedTime : 0f):F3}s",
                this);
        }

        pendingTurnDegrees = 0f;
        pendingTurnQueuedTime = -1f;
        ballVisualSlopeDrive?.CancelTurnHandoffRequest();
    }

    void RequestTurnHandoffForPendingIntent()
    {
        if (!HasPendingTurnIntent || !ballVisualSlopeDrive)
            return;

        if (ballVisualSlopeDrive.OwnsBallVisualPose)
            ballVisualSlopeDrive.RequestTurnHandoff();
    }

    // 新しい旋回を「今このFixedUpdateで実行してよいか」だけを判定する。
    // 入力の破棄はここでも行わない。
    bool IsTurnExecutionBlocked()
    {
        if (turnTransitionActive)
            return true;

        if (fiveLineCorrectionActive)
            return true;

        if (fiveLineCorrectionPending)
            return true;

        // Turn Input Intent中は、BallVisual側がTurnHandoffを完了して
        // Synchronizedへ戻るまでTurnTransitionを開始しない。
        // 単なるOwnsPose待ちではなく、RequestTurnHandoffForPendingIntent()が
        // 通常Recoveryを短いTurnHandoffRejoinへ切り替える。
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

        // 古い入力をRecovery終了後に突然実行しない。
        if (IsPendingTurnIntentExpired())
        {
            ExpirePendingTurnIntent();
            return;
        }

        // 独立軌道中なら通常Recoveryを待つ代わりにTurn専用Handoffへ短縮する。
        RequestTurnHandoffForPendingIntent();

        // 実行禁止中はIntentを保持したまま待つ。
        if (IsTurnExecutionBlocked())
            return;

        float turnDegrees =
            Mathf.Sign(pendingTurnDegrees) * QuarterTurnDegrees;

        // 実行する瞬間にだけIntentを消費する。
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
            Debug.LogError(
                "[CORE TURN TRANSITION FAILED] InSubject Rigidbodyは通常走行時Dynamicである必要があります。",
                rb);
            return;
        }

        Quaternion physicsTurn =
            Quaternion.AngleAxis(turnDegrees, Vector3.up);

        Vector3 directionBefore =
            NormalizeFlat(direction, travelDirection);

        turnTargetDirection =
            NormalizeFlat(
                physicsTurn * directionBefore,
                directionBefore);

        // Flat旋回はTransitionだけで横ズレを発生させないため後補正しない。
        // Slope/Airだけ、BallVisualの独立軌道後に着地レーン整理としてFiveLineを残す。
        bool needsFiveLineAfterPop =
            enableFiveLineAfterPop &&
            (BallVisualIsOnSlope || BallVisualIsAir);

        activeTurnPostCorrectionMode =
            needsFiveLineAfterPop
                ? TurnPostCorrectionMode.FiveLineAfterPop
                : TurnPostCorrectionMode.None;

        fiveLineCorrectionPending = needsFiveLineAfterPop;

        if (needsFiveLineAfterPop)
        {
            CaptureTurnLandingIntent();
        }
        else
        {
            capturedTurnLandingIntentValid = false;
            capturedTurnLandingIntent = default;
        }

        Vector3 velocityBefore = rb.velocity;
        Vector3 angularVelocityBefore = rb.angularVelocity;

        if (!BeginTurnTransition(turnDegrees))
            return;

        FindMapFrameReferences();
        BindCoordinateFrames();

        float visualAngle =
            turnDegrees *
            (visualRootTurnsOppositeToInput ? -1f : 1f);

        bool visualTurnStarted = false;

        if (correspondSubject && visualPlayerRoot)
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

        // CorrespondSubjectが無い互換SceneではDirect回転を使う。
        // Direct回転は同一FixedUpdateで完了するのでHardFreezeも即解除する。
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

        if (logCore || logTurnTransition)
        {
            Debug.Log(
                $"[CORE TURN TRANSITION BEGIN] " +
                $"time={Time.fixedTime:F4} " +
                $"turn={turnDegrees:F1} " +
                $"visualDuration={turnVisualDurationSeconds:F3}s " +
                $"hardFreeze={turnHardFreezeSeconds:F3}s " +
                $"directionBefore={directionBefore:F4} " +
                $"targetDirection={turnTargetDirection:F4} " +
                $"velocityBefore={velocityBefore:F4} " +
                $"angularVelocityBefore={angularVelocityBefore:F4} " +
                $"postCorrection={activeTurnPostCorrectionMode}",
                this);
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

        // HardFreeze開始。旧速度を同期側に漏らさないため明示的に0化してからKinematicへ。
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.isKinematic = true;

        turnTransitionActive = true;
        turnBodyFrozen = true;
        turnTransitionStartTime = Time.fixedTime;
        waitingForTurnGuide = false;

        ResetBallVisualSplineSession();
        return true;
    }

    // SlopeStickCore.FixedUpdateとCorrespondSubjectのTween tickの両方から呼べる。
    // 時間基準なのでVisual Easeを変更してもHardFreezeの体感時間は変わらない。
    void AdvanceTurnTransition()
    {
        if (!turnTransitionActive || !turnBodyFrozen)
            return;

        float hardFreeze = Mathf.Max(0f, turnHardFreezeSeconds);
        float elapsed = Mathf.Max(0f, Time.fixedTime - turnTransitionStartTime);

        if (elapsed + Eps < hardFreeze)
            return;

        ReleaseTurnBodyToCoast("HardFreezeElapsed");
    }

    void OnVisualTurnProgress(float visualProgress01)
    {
        if (!turnTransitionActive)
            return;

        AdvanceTurnTransition();
    }

    void ReleaseTurnBodyToCoast(string reason)
    {
        if (!turnTransitionActive ||
            !turnBodyFrozen ||
            !turnTransitionSnapshot.valid ||
            !rb)
        {
            return;
        }

        Quaternion physicsTurn =
            Quaternion.AngleAxis(activeTurnDegrees, Vector3.up);

        Vector3 restoredVelocity =
            physicsTurn * turnTransitionSnapshot.velocity;

        Vector3 restoredAngularVelocity =
            physicsTurn * turnTransitionSnapshot.angularVelocity;

        Vector3 restoredDirection =
            NormalizeFlat(
                physicsTurn * turnTransitionSnapshot.direction,
                turnTargetDirection);

        // HardFreeze中は位置/回転を固定したまま。Coast開始時だけDynamicへ戻す。
        rb.position = turnTransitionSnapshot.position;
        rb.rotation = turnTransitionSnapshot.rotation;
        rb.isKinematic = false;
        rb.useGravity = turnTransitionSnapshot.useGravity;
        rb.velocity = restoredVelocity;
        rb.angularVelocity = restoredAngularVelocity;
        rb.WakeUp();

        direction = restoredDirection;
        turnTargetDirection = restoredDirection;
        turnBodyFrozen = false;

        // Visual旋回完了までは通常Spline Driveを再開しない。
        // Rigidbody自身の慣性・Gravity・Collider応答だけでCoastする。
        driveState = 0f;
        ResetBallVisualSplineSession();

        Physics.SyncTransforms();
        correspondSubject?.ResetDerivedVelocitySample();
        correspondSubject?.SynchronizeNow(true);

        if (logCore || logTurnTransition)
        {
            Debug.Log(
                $"[CORE TURN COAST BEGIN] " +
                $"reason={reason} " +
                $"time={Time.fixedTime:F4} " +
                $"elapsed={Mathf.Max(0f, Time.fixedTime - turnTransitionStartTime):F4}s " +
                $"velocity={rb.velocity:F4} " +
                $"speed={rb.velocity.magnitude:F4} " +
                $"direction={direction:F4}",
                this);
        }
    }

    void CompleteTurnTransition()
    {
        if (!turnTransitionActive || !turnTransitionSnapshot.valid || !rb)
            return;

        float completedTurnDegrees = activeTurnDegrees;

        // Visual旋回の方がHardFreezeより短い設定でも、終了時には必ずDynamicへ復帰させる。
        if (turnBodyFrozen)
            ReleaseTurnBodyToCoast("VisualTurnCompleted");

        turnTransitionActive = false;
        turnBodyFrozen = false;
        turnTransitionStartTime = -1f;
        activeTurnDegrees = 0f;
        turnTransitionSnapshot = default;

        // Visualは完了してもNearestKnotDetectorが旧Splineを返す可能性がある。
        // 新Splineを捕捉するまでは旧Spline Driveを再開しないが、
        // Rigidbodyの復元速度は保持されるため移動自体は止めない。
        waitingForTurnGuide = true;
        driveState = 0f;
        ResetBallVisualSplineSession();

        Physics.SyncTransforms();
        correspondSubject?.ResetDerivedVelocitySample();
        correspondSubject?.SynchronizeNow(true);

        if (logCore || logTurnTransition)
        {
            Debug.Log(
                $"[CORE TURN TRANSITION COMPLETE] " +
                $"time={Time.fixedTime:F4} " +
                $"turn={completedTurnDegrees:F1} " +
                $"position={rb.position:F4} " +
                $"velocity={rb.velocity:F4} " +
                $"speed={rb.velocity.magnitude:F4} " +
                $"direction={direction:F4} " +
                $"waitingForGuide={waitingForTurnGuide}",
                this);
        }
    }

    void AbortTurnTransitionAndRestoreOriginal(string reason)
    {
        if (!rb)
        {
            turnTransitionActive = false;
            turnBodyFrozen = false;
            turnTransitionStartTime = -1f;
            activeTurnDegrees = 0f;
            turnTransitionSnapshot = default;
            return;
        }

        TurnTransitionSnapshot snapshot = turnTransitionSnapshot;

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

        turnTransitionActive = false;
        turnBodyFrozen = false;
        turnTransitionStartTime = -1f;
        activeTurnDegrees = 0f;
        turnTransitionSnapshot = default;
        waitingForTurnGuide = false;

        activeTurnPostCorrectionMode = TurnPostCorrectionMode.None;
        fiveLineCorrectionPending = false;
        fiveLineCorrectionActive = false;
        capturedTurnLandingIntentValid = false;
        capturedTurnLandingIntent = default;

        Physics.SyncTransforms();
        correspondSubject?.ResetDerivedVelocitySample();
        correspondSubject?.SynchronizeNow(true);

        if (logCore || logTurnTransition)
        {
            Debug.LogWarning(
                $"[CORE TURN TRANSITION ABORT] reason={reason} " +
                $"position={rb.position:F4} velocity={rb.velocity:F4}",
                this);
        }
    }

    void ClearTurnTransitionForStageRebuild()
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
        waitingForTurnGuide = false;

        activeTurnPostCorrectionMode = TurnPostCorrectionMode.None;
        fiveLineCorrectionPending = false;
        fiveLineCorrectionActive = false;
        fiveLineCorrectionElapsed = 0f;
        fiveLineCorrectionTargetOffset = 0f;
        capturedTurnLandingIntentValid = false;
        capturedTurnLandingIntent = default;

        if (rb && rb.isKinematic)
            rb.isKinematic = false;

        if (rb)
        {
            // Death/rebuildは通常の旋回取消ではなく再配置処理なので、
            // 旋回前速度を復元せず再開地点用の0状態へ揃える。
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }

    void CaptureTurnLandingIntent()
    {
        capturedTurnLandingIntentValid = false;
        capturedTurnLandingIntent = default;

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

        // FiveLineAfterPopではターン開始後にLanding targetが更新される場合があるため、
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
        // Turn Transitionは終了済みでも、BallVisualが独立軌道中ならFiveLineを開始しない。
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

        if (logCore || logTurnTransition)
        {
            Debug.Log(
                $"[CORE FIVE LINE BEGIN] mode={activeTurnPostCorrectionMode} " +
                $"group={selectedGroup} rawCorrection={rawCorrection:F4} " +
                $"appliedCorrection={correction:F4} selector={selectorPosition:F4}",
                this);
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

        activeTurnPostCorrectionMode = TurnPostCorrectionMode.None;
        capturedTurnLandingIntentValid = false;
        capturedTurnLandingIntent = default;
        ResetBallVisualSplineSession();

        if (surface.Valid)
        {
            surface = BuildSplineSurface(currentGuide);
            currentSurface = surface;
            currentSurfaceValid = surface.Valid;
        }

        if (logCore || logTurnTransition)
        {
            Debug.Log(
                $"[CORE FIVE LINE COMPLETE] group={fiveLineTargetGroup} " +
                $"position={rb.position:F4} velocity={rb.velocity:F4}",
                this);
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


#if UNITY_EDITOR
    // ================================================================
    // TurnTransition embedded self check
    // ================================================================
    // Unity Test Framework / NUnitへ依存せず、Play Mode中にInspectorの
    // ContextMenuからTurnTransitionの主要契約を確認するための自己診断。
    // Runtime Player buildではUNITY_EDITORが未定義なので含まれない。
    [ContextMenu("Debug/Turn Transition Self Check")]
    void RunTurnTransitionSelfCheck()
    {
        if (!Application.isPlaying)
        {
            Debug.LogWarning(
                "[TURN TRANSITION SELF CHECK] Play Modeで実行してください。",
                this);
            return;
        }

        if (Time.timeScale <= 0f)
        {
            Debug.LogWarning(
                "[TURN TRANSITION SELF CHECK] Time.timeScaleが0以下です。Tween完了を確認できないため中止します。",
                this);
            return;
        }

        StartCoroutine(RunTurnTransitionSelfCheckCoroutine());
    }

    public void PushStart()
    {
        MainGameManager.TopTitle.SetActive(false);
        MainGameManager.PreviewIconRoot.SetActive(false);
        MainGameManager.TopLiteral.SetActive(false);
        MainGameManager.PlayButton.SetActive(false);
        MainGameManager.Userbility.SetActive(true);

        BeginCommandOnTouch = true;
    }

    IEnumerator Recover()
    {
        yield return new WaitForSeconds(0.5f);
        MainGameManager.DropOut.SetActive(true);
        yield return new WaitForSeconds(2f);
        MainGameManager.DropOut.SetActive(false);
        MainGameManager.OpenChunkStage = true;
    }

    IEnumerator RunTurnTransitionSelfCheckCoroutine()
    {
        int failureCount = 0;

        GameObject physicsRootObject = null;
        GameObject visualRootObject = null;
        GameObject pivotObject = null;
        GameObject inSubjectObject = null;
        GameObject subjectObject = null;

        try
        {
            physicsRootObject =
                new GameObject("PhysicsRoot_TurnTransitionSelfCheck");
            visualRootObject =
                new GameObject("VisualPlayerRoot_TurnTransitionSelfCheck");
            pivotObject =
                new GameObject("TurnPivot_TurnTransitionSelfCheck");
            inSubjectObject =
                new GameObject("InSubject_TurnTransitionSelfCheck");
            subjectObject =
                new GameObject("Subject_TurnTransitionSelfCheck");

            inSubjectObject.transform.SetParent(
                physicsRootObject.transform,
                false);

            Rigidbody inSubjectBody =
                inSubjectObject.AddComponent<Rigidbody>();
            inSubjectBody.useGravity = false;
            inSubjectBody.detectCollisions = false;

            inSubjectObject.AddComponent<SphereCollider>();
            inSubjectObject.AddComponent<NearestKnotDetector>();

            SlopeStickCore testCore =
                inSubjectObject.AddComponent<SlopeStickCore>();

            // 自己診断用インスタンス自身のUpdate/FixedUpdate/Startは使わない。
            // ApplyQuarterTurnをこのクラス内部から直接呼び、TurnTransition契約だけを検査する。
            testCore.enabled = false;

            Rigidbody subjectBody =
                subjectObject.AddComponent<Rigidbody>();
            subjectBody.useGravity = false;
            subjectBody.detectCollisions = false;
            subjectBody.isKinematic = true;

            CorrespondSubject correspond =
                subjectObject.AddComponent<CorrespondSubject>();

            correspond.Bind(
                inSubjectBody,
                physicsRootObject.transform,
                visualRootObject.transform);

            // 同じSlopeStickCore型のprivate fieldなので直接設定できる。
            testCore.correspondSubject = correspond;
            testCore.physicsRoot = physicsRootObject.transform;
            testCore.visualPlayerRoot = visualRootObject.transform;
            testCore.visualRotationPivot = pivotObject.transform;
            testCore.enableFiveLineAfterPop = false;
            testCore.logCore = false;
            testCore.logTurnTransition = false;

            testCore.turnVisualDurationSeconds = 0.16f;
            testCore.turnHardFreezeSeconds = 0.04f;

            testCore.SetTravelDirection(Vector3.forward);

            // Scene内のColliderと接触しないよう十分離れた位置で検証する。
            inSubjectBody.position =
                new Vector3(10000f, 10000f, 10000f);
            inSubjectBody.rotation = Quaternion.identity;
            inSubjectBody.velocity =
                new Vector3(0f, 0f, 10f);
            inSubjectBody.angularVelocity =
                new Vector3(0f, 2f, 0f);

            Vector3 frozenPosition = inSubjectBody.position;

            // FixedUpdate経由ではなく、TurnTransition開始処理を直接検証する。
            testCore.ApplyQuarterTurn(+QuarterTurnDegrees);

            SelfCheck(
                testCore.IsTurnTransitionActive,
                "旋回開始直後にTurnTransitionが有効",
                ref failureCount);

            SelfCheck(
                testCore.IsTurnBodyFrozen && inSubjectBody.isKinematic,
                "HardFreeze中のInSubjectがKinematic",
                ref failureCount);

            SelfCheck(
                Vector3.Distance(
                    inSubjectBody.position,
                    frozenPosition) < 0.0001f,
                "旋回開始時点でInSubject位置を保持",
                ref failureCount);

            // HardFreeze=0.04秒を越え、VisualTurn=0.16秒はまだ完了していない時点。
            yield return new WaitForSeconds(0.07f);
            yield return new WaitForFixedUpdate();

            SelfCheck(
                testCore.IsTurnTransitionActive,
                "Coast中もTurnTransition全体は継続",
                ref failureCount);

            SelfCheck(
                !testCore.IsTurnBodyFrozen && !inSubjectBody.isKinematic,
                "HardFreeze終了後にVisual旋回完了前でもDynamicへ復帰",
                ref failureCount);

            SelfCheck(
                Mathf.Abs(inSubjectBody.velocity.magnitude - 10f) <= 0.10f,
                "Coast開始時に旋回前Speedを復元",
                ref failureCount);

            // VisualTurn完了後まで待つ。
            yield return new WaitForSeconds(0.14f);
            yield return new WaitForFixedUpdate();

            SelfCheck(
                !testCore.IsTurnTransitionActive,
                "Visual旋回完了後にTurnTransition終了",
                ref failureCount);

            SelfCheck(
                !inSubjectBody.isKinematic,
                "Visual旋回完了後もInSubjectはDynamic",
                ref failureCount);

            SelfCheck(
                Mathf.Abs(inSubjectBody.velocity.magnitude - 10f) <= 0.02f,
                "旋回前の速度Magnitudeを保存",
                ref failureCount);

            SelfCheck(
                Mathf.Abs(inSubjectBody.velocity.x - 10f) <= 0.02f,
                "右90度旋回後にX方向へ速度復元",
                ref failureCount);

            SelfCheck(
                Mathf.Abs(inSubjectBody.velocity.z) <= 0.02f,
                "右90度旋回後に旧Z速度を残さない",
                ref failureCount);

            if (failureCount == 0)
            {
                Debug.Log(
                    "[TURN TRANSITION SELF CHECK PASS] HardFreeze→Coast→Completeの主要契約を確認しました。",
                    this);
            }
            else
            {
                Debug.LogError(
                    $"[TURN TRANSITION SELF CHECK FAILED] failures={failureCount}",
                    this);
            }
        }
        finally
        {
            if (physicsRootObject)
                Object.Destroy(physicsRootObject);

            if (visualRootObject)
                Object.Destroy(visualRootObject);

            if (pivotObject)
                Object.Destroy(pivotObject);

            if (inSubjectObject)
                Object.Destroy(inSubjectObject);

            if (subjectObject)
                Object.Destroy(subjectObject);
        }
    }




    void SelfCheck(
        bool condition,
        string message,
        ref int failureCount)
    {
        if (condition)
        {
            Debug.Log(
                $"[TURN TRANSITION SELF CHECK PASS] {message}",
                this);
            return;
        }

        failureCount++;
        Debug.LogError(
            $"[TURN TRANSITION SELF CHECK FAIL] {message}",
            this);
    }
#endif

}
