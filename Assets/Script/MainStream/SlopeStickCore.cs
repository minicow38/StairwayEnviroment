using UnityEngine;
using System.Collections;
using UnityEditor.Rendering;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody), typeof(SphereCollider), typeof(NearestKnotDetector))]
public sealed class SlopeStickCore : MonoBehaviour
{
    const float Eps = 0.000001f;
    [Min(0f)] [SerializeField] float maxGroundSpeed = 24f;
    // Compactから固定値化
    const float ProbeRadius = .475f;
    [SerializeField]public float GroundAcceleration = 35f;
    const float MaxDeceleration = 80f;
    const float ResponseInverse = 8.333333f;
    const float AccelerationJerk = 600f;
    [SerializeField] public float activeTimeScale;

    const float TargetMinDistance = .27f;
    [SerializeField]public float TargetAccelerationLimit = 120f;
    const float PostTargetBlendWidth = .05f;
    const float PostTargetGravityRatio = .50f;

    const float StickJerk = 3000f;
    const float ReleaseHold = .02f;
    const float NaturalReleaseEnd = .90f;

    [SerializeField] public MainGameManager mainGameManager;

    [SerializeField] NearestKnotDetector knotDetector;
    [SerializeField] LayerMask groundMask = ~0;

    [Header("Travel")]
    [SerializeField] Vector3 travelDirection = Vector3.forward;
    
    [Range(0f, 100f)] [SerializeField] public float targetSlopeProgressPercent = 60f;

    [Header("Coordinate Mapping")]
    [Tooltip("PhysicsRoot上のInSubjectをVisualPlayerRoot側へ写す座標変換担当です。")]
    [SerializeField] CorrespondSubject correspondSubject;
    [Tooltip("回転させない物理座標系。InSubjectはこの配下で物理計算します。")]
    [SerializeField] Transform physicsRoot;

    [Header("Map Direction Turn")]
    [Tooltip("回転する表示座標系。InSubject/PhysicsRootは回転させません。")]
    [SerializeField] Transform visualPlayerRoot;
    [Tooltip("VisualPlayerRootを回すワールドPivot。未設定ならCenter1を検索します。")]
    [SerializeField] Transform visualRotationPivot;
    [Tooltip("ONなら入力方向と反対へVisualPlayerRootを回し、マップが逆向きに旋回して見えるようにします。")]
    [SerializeField] bool visualRootTurnsOppositeToInput = true;
    [Tooltip("これ未満の横フリックは旋回として扱いません。")]
    [Min(1f)] [SerializeField] float minimumFlickPixels = 10f;
    [Tooltip("フリック距離を角度段階へ分ける倍率です。")]
    [Min(1.01f)] [SerializeField] float flickStrengthBandRatio = 1.8f;
    [Tooltip("弱いフリックから強いフリックへ順番に割り当てる旋回角度です。")]
    [SerializeField] float[] flickTurnAngleSteps = { 90f, 90f, 90f, 90f };
    [Tooltip("ボタン/API操作時の固定旋回角度です。")]
    [Range(0f, 180f)] [SerializeField] float turnAngle = 45f;

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
    private bool BeginCommandOnTouch = false;
    [Header("Debug")]
    [SerializeField] bool logCore;

    // Compact側のユーザービリティを維持
    public GameObject sub;

    Rigidbody rb;
    Vector3 direction;

    Vector2 flickStart;
    bool trackingFlick;
    float pendingFlickTurnDegrees;

    float graceTimer;
    float driveState;
    float stickState;
    bool wasSlope;
    
    bool waitingForTurnGuide;
    Vector3 turnTargetDirection;

    // 旋回後、新しいGuideとVisual回転が揃った瞬間に1回だけ
    // 5本の横基準へ位置・横速度を整理する。通常移動中は使わない。
    bool postTurnFiveLinePending;

    public bool IsPostTurnFiveLinePending =>
        postTurnFiveLinePending;

    public NearestKnotDetector.FiveLineGroup LastPostTurnFiveLineGroup
    {
        get;
        private set;
    } = NearestKnotDetector.FiveLineGroup.Center;

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
        !postTurnFiveLinePending;

    public bool IsWaitingForTurnGuide =>
        waitingForTurnGuide;

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
            postTurnFiveLinePending ||
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

        if (!knotDetector)
            knotDetector = GetComponent<NearestKnotDetector>();

        direction = NormalizeFlat(travelDirection, transform.forward);
    }

    // ================================================================
    // Usability
    // ================================================================

    void Start()
    {
        mainGameManager = GameObject.Find("GameManager").transform.GetComponent<MainGameManager>();
        if (!sub)
            sub = GameObject.Find("InSubject");
               activeTimeScale =Time.timeScale;

        FindMapFrameReferences();
        BindCoordinateFrames();
        StartCoroutine(delayStart());
    }

    void Update()
    {
        ReadTurnFlick();
    }

   
    IEnumerator delayStart()
    {
        yield return new WaitForSeconds(0.3f);
        //Time.timeScale = 0.25f;
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

        Vector3 restart =
            startSlab.transform.position;

        rb.position =
            new Vector3(
                restart.x,
                restart.y + 2f,
                restart.z);

        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        Physics.SyncTransforms();
        correspondSubject?.SynchronizeNow(true);
    }

    // ================================================================
    // Main
    // ================================================================

    void FixedUpdate()
    {
        if (activeTimeScale != Time.timeScale)
        {
            Debug.Log("");
           
            activeTimeScale = Time.timeScale;
           // Time.timeScale = activeTimeScale;
        }
        // SlopeStick3Dと同じく、描画Updateで予約した方向変更を
        // 物理/Spline観測より先にFixedUpdateで一度だけ適用する。
        ApplyPendingMapDirectionTurn();

        var guide = knotDetector.Evaluate(rb.position);

        // Splineが存在しなければCoreは制御しない。
        if (!guide.valid)
        {
            LoseSupport();
            return;
        }

        currentGuide = guide;
        currentGuideValid = true;

        bool grounded = HasGroundSupport();

        if (grounded)
            graceTimer = supportGraceSeconds;
        else
            graceTimer = Mathf.Max(0f, graceTimer - Time.fixedDeltaTime);

        float load = grounded ? 0f : SupportLoad(guide);
        bool grace = !grounded && graceTimer > 0f && load <= 1f && CanGrace(guide);

        if (!grounded && !grace)
        {
            if (logCore)
                Debug.Log($"[CORE SUPPORT LOST] load={load:F3} dist={guide.distanceToGuide:F3} outward={Outward(guide):F3}");

            LoseSupport();
            return;
        }
        
        currentSupported = true;

        // Slopeへの切替そのものもSpline判定。
        if (guide.isSlope && !wasSlope)
        {
           // mainGameManager.lastTouch=
            driveState = 0f;
            TransportToSpline(guide);
        }

        // Collider法線は使わない。
        // tangent / normal は必ずSpline Guideから構築。
        Surface surface = BuildSplineSurface(guide);

        if (!surface.Valid)
        {
            currentSurfaceValid = false;
            return;
        }

        currentSurface = surface;
        currentSurfaceValid = true;
        // ================================================================
// 旋回後：新しいSplineへ切り替わるまで待つ
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

            // まだ旧Splineを見ている
            if (alignment < TurnGuideAlignmentMin)
            {
                // 旧Spline方向へのDriveを止める
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

                return;
            }

            // 新しいSplineを捕捉した。
            // ただし5ライン補正はVisual側の回転完了後に1回だけ行う。
            waitingForTurnGuide = false;
            driveState = 0f;
        }
        
        ApplyPostTurnFiveLineCorrection(guide, ref surface);

        if (Input.GetMouseButtonDown(0))
        {
          /*  Debug.Log("");
            mainGameManager.TopTitle.SetActive(false);
            mainGameManager.PreviewIconRoot.SetActive(false);
            mainGameManager.TopLiteral.SetActive(false);
            mainGameManager.PlayButton.SetActive(false);*/
            
          BeginCommandOnTouch = true;
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
    // Post-turn five-line correction
    // ================================================================

    void ApplyPostTurnFiveLineCorrection(
        NearestKnotDetector.GuideFrame guide,
        ref Surface surface)
    {
        if (!postTurnFiveLinePending)
            return;

        // Stage / VisualPlayerRoot のTweenが終わるまでは位置を確定しない。
        // Pendingは残すので、完了した最初のFixedUpdateだけが補正点になる。
        if (correspondSubject &&
            correspondSubject.IsVisualFrameTurning)
        {
            return;
        }

        // 「旋回後に1回だけ」を保証するため、ここから先は成功/失敗に関係なく消費する。
        postTurnFiveLinePending = false;

        if (!rb ||
            !knotDetector ||
            !guide.valid ||
            !surface.Valid)
        {
            return;
        }

        if (!knotDetector.TryGetFiveLineFrame(
                guide,
                surface.side,
                out NearestKnotDetector.FiveLineFrame frame) ||
            !frame.valid)
        {
            if (logCore)
            {
                Debug.LogWarning(
                    "[CORE FIVE LINE] 3本の対応Splineを取得できなかったため補正を省略しました。",
                    this);
            }

            return;
        }

        Vector3 side = surface.side.normalized;

        // 階段幅を5点へ量子化する。
        // 左右入力をレーンShiftへ変換する処理ではなく、旋回後の1回限りの整理。
        NearestKnotDetector.FiveLineGroup bestGroup =
            NearestKnotDetector.FiveLineGroup.Center;

        Vector3 bestPoint = frame.center;
        float bestAbsError = float.PositiveInfinity;
        float bestSignedError = 0f;

        for (int i = 0; i < 5; i++)
        {
            NearestKnotDetector.FiveLineGroup group =
                (NearestKnotDetector.FiveLineGroup)i;

            Vector3 point = frame.GetPoint(group);

            float signedError =
                Vector3.Dot(
                    point - rb.position,
                    side);

            float absError = Mathf.Abs(signedError);

            if (absError < bestAbsError)
            {
                bestAbsError = absError;
                bestSignedError = signedError;
                bestPoint = point;
                bestGroup = group;
            }
        }

        // 5ラインの幅から大きく外れている場合は、遠距離スナップをしない。
        // 端のラインから「半レーン分」までは端グループとして許容する。
        float leftX =
            Vector3.Dot(
                frame.left - frame.center,
                side);

        float rightX =
            Vector3.Dot(
                frame.right - frame.center,
                side);

        float currentX =
            Vector3.Dot(
                rb.position - frame.center,
                side);

        float leftStep =
            Mathf.Abs(
                Vector3.Dot(
                    frame.leftCenter - frame.left,
                    side));

        float rightStep =
            Mathf.Abs(
                Vector3.Dot(
                    frame.right - frame.centerRight,
                    side));

        float edgeMargin =
            Mathf.Max(Eps, Mathf.Min(leftStep, rightStep) * 0.5f);

        float minX = Mathf.Min(leftX, rightX) - edgeMargin;
        float maxX = Mathf.Max(leftX, rightX) + edgeMargin;

        if (currentX < minX || currentX > maxX)
        {
            if (logCore)
            {
                Debug.LogWarning(
                    $"[CORE FIVE LINE] 階段幅外のため補正を省略。 " +
                    $"x={currentX:F3} range=[{minX:F3},{maxX:F3}]",
                    this);
            }

            return;
        }

        Vector3 positionBefore = rb.position;
        Vector3 velocityBefore = rb.velocity;

        // 前後位置と接地高さは保持し、横成分だけ最寄り5ラインへ合わせる。
        rb.position =
            rb.position +
            side * bestSignedError;

        // 残った横速度だけをゼロへする。
        // 法線方向の接地運動や、正方向の接線速度は保存する。
        float lateralSpeed =
            Vector3.Dot(
                rb.velocity,
                surface.side);

        rb.velocity -=
            surface.side * lateralSpeed;

        // 旋回後に新しいSplineと逆向きへ進む成分だけ除去する。
        float tangentSpeed =
            Vector3.Dot(
                rb.velocity,
                surface.tangent);

        if (tangentSpeed < 0f)
        {
            rb.velocity -=
                surface.tangent * tangentSpeed;
        }

        rb.WakeUp();

        Physics.SyncTransforms();
        correspondSubject?.SynchronizeNow(true);

        LastPostTurnFiveLineGroup = bestGroup;

        // velocityを変更したので、このFixedUpdateで使うSurface速度成分も更新する。
        surface = BuildSplineSurface(guide);
        currentSurface = surface;
        currentSurfaceValid = surface.Valid;

        // 補正前位置に基づくBallVisual spline sessionは使わない。
        ResetBallVisualSplineSession();

        if (logCore)
        {
            Debug.Log(
                $"[CORE FIVE LINE] group={bestGroup} " +
                $"positionBefore={positionBefore:F4} positionAfter={rb.position:F4} " +
                $"target={bestPoint:F4} lateralCorrection={bestSignedError:F4} " +
                $"velocityBefore={velocityBefore:F4} velocityAfter={rb.velocity:F4}",
                this);
        }
    }

    // ================================================================
    // Collider = Support existence only
    // ================================================================

    bool HasGroundSupport()
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
            return false;

        return Vector3.Angle(hit.normal, Vector3.up) <= maxSlopeAngle;
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
        }

        if (!trackingFlick || !Input.GetMouseButtonUp(0))
            return;

        trackingFlick = false;

        Vector2 flick = (Vector2)Input.mousePosition - flickStart;

        if (Mathf.Abs(flick.x) < minimumFlickPixels ||
            Mathf.Abs(flick.x) <= Mathf.Abs(flick.y))
        {
            return;
        }

        float steppedAngle = GetSteppedFlickTurnAngle(Mathf.Abs(flick.x));

        if (steppedAngle <= 0f)
            return;

        BeginPlayerAndStageTurn(flick.x > 0f ? steppedAngle : -steppedAngle);
    }

    float GetSteppedFlickTurnAngle(float flickPixels)
    {
        if (flickTurnAngleSteps == null || flickTurnAngleSteps.Length == 0)
            return Mathf.Clamp(Mathf.Abs(turnAngle), 0f, 180f);

        float basePixels = Mathf.Max(minimumFlickPixels, 1f);
        float bandRatio = Mathf.Max(flickStrengthBandRatio, 1.01f);
        float normalizedDistance = Mathf.Max(flickPixels, basePixels) / basePixels;

        int stepIndex = Mathf.FloorToInt(Mathf.Log(normalizedDistance, bandRatio));
        stepIndex = Mathf.Clamp(stepIndex, 0, flickTurnAngleSteps.Length - 1);

        return Mathf.Clamp(Mathf.Abs(flickTurnAngleSteps[stepIndex]), 0f, 180f);
    }

    public void TurnPlayerAndStageLeft()
    {
        BeginPlayerAndStageTurn(-Mathf.Abs(turnAngle));
    }

    public void TurnPlayerAndStageRight()
    {
        BeginPlayerAndStageTurn(Mathf.Abs(turnAngle));
    }

    public void BeginPlayerAndStageTurn(float playerAngle)
    {
        float clampedAngle = Mathf.Clamp(playerAngle, -180f, 180f);

        if (Mathf.Abs(clampedAngle) <= .0001f)
            return;

        // Update/UI側ではRigidbodyを変更せず、次のFixedUpdateへ予約する。
        pendingFlickTurnDegrees = clampedAngle;

        if (logCore)
        {
            Debug.Log(
                $"[CORE MAP TURN QUEUED] time={Time.fixedTime:F4} " +
                $"angle={clampedAngle:F3} direction={direction:F4} " +
                $"velocity={rb.velocity:F4}",
                this);
        }
    }

    void ApplyPendingMapDirectionTurn()
    {
        float inputAngle = pendingFlickTurnDegrees;

        if (Mathf.Abs(inputAngle) <= .0001f)
            return;

        pendingFlickTurnDegrees = 0f;
        ApplyMapDirectionTurn(inputAngle);
    }

    void ApplyMapDirectionTurn(float inputAngle)
    {
        if (Mathf.Abs(inputAngle) <= .0001f || !rb)
            return;

        if (rb.isKinematic)
        {
            Debug.LogError(
                "[CORE MAP TURN FAILED] InSubject RigidbodyはDynamicである必要があります。",
                rb);
            return;
        }

        Quaternion directionTurn = Quaternion.AngleAxis(inputAngle, Vector3.up);

        Vector3 directionBefore = NormalizeFlat(direction, travelDirection);
        Vector3 velocityBefore = rb.velocity;

        // InSubjectのTransform/rotationは回さない。
        // 進行基準directionと水平速度だけをワールドY軸回りに旋回する。
        direction = NormalizeFlat(directionTurn * directionBefore, directionBefore);

        float verticalSpeed = Vector3.Dot(velocityBefore, Vector3.up);
        Vector3 planarVelocity = velocityBefore - Vector3.up * verticalSpeed;

        if (planarVelocity.sqrMagnitude > Eps * Eps)
            planarVelocity = directionTurn * planarVelocity;

        rb.velocity = planarVelocity + Vector3.up * verticalSpeed;
        rb.WakeUp();

        // 旧方向の接線加速度を新方向へ持ち越さない。
        // Spline自体はこの直後に現在位置から再Evaluateされる。
        driveState = 0f;
        
        turnTargetDirection = direction;
        waitingForTurnGuide = true;
        postTurnFiveLinePending = true;

        // A turn starts a new logical spline session. Do not let BallVisual
        // reuse a plan anchored to the pre-turn branch.
        ResetBallVisualSplineSession();

        FindMapFrameReferences();
        BindCoordinateFrames();

        float visualAngle =
            inputAngle *
            (visualRootTurnsOppositeToInput ? -1f : 1f);

        bool visualRotated = false;

        if (correspondSubject && visualPlayerRoot)
        {
            Vector3 pivot =
                visualRotationPivot
                    ? visualRotationPivot.position
                    : visualPlayerRoot.position;

            Quaternion visualTurn =
                Quaternion.AngleAxis(
                    visualAngle,
                    Vector3.up);

            visualRotated =
                correspondSubject.RotateVisualFrameAround(
                    pivot,
                    visualTurn,
                    true);
        }
        else
        {
            // CorrespondSubjectがまだSceneに無い場合だけの互換Fallback。
            visualRotated = RotateVisualMapFrameDirect(visualAngle);
        }

        if (logCore)
        {
            Debug.Log(
                $"[CORE MAP TURN APPLIED] time={Time.fixedTime:F4} " +
                $"inputAngle={inputAngle:F3} visualAngle={visualAngle:F3} " +
                $"directionBefore={directionBefore:F4} directionAfter={direction:F4} " +
                $"velocityBefore={velocityBefore:F4} velocityAfter={rb.velocity:F4} " +
                $"verticalPreserved={Mathf.Abs(verticalSpeed - Vector3.Dot(rb.velocity, Vector3.up)) <= .0001f} " +
                $"visualRotated={visualRotated}",
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
}