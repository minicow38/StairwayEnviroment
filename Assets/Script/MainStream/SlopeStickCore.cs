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
    public static event System.Action<Rigidbody, Vector3> AutoHoldEdgeSnapped;
    
    public float MaxGroundSpeed
    {
        get => maxGroundSpeed;
        set => maxGroundSpeed = Mathf.Max(0f, value);
    }
    public float MaxGroundSpeedReadOnly => MaxGroundSpeed;
    public float TargetSlopeProgress01ReadOnly =>
        TargetProgress;

    const float Eps = 0.000001f;
    [Min(0f)] [SerializeField] float maxGroundSpeed = 24f;
    // Compactから固定値化
    const float ProbeRadius = .475f;
    [SerializeField]public float GroundAcceleration = 35f;
    const float MaxDeceleration = 80f;
    const float ResponseInverse = 8.333333f;
    private Vector3 acceleration = Vector3.zero;
    const float AccelerationJerk = 600f;

    float nextFlatMotionLogTime;
    bool straightFlatFallLogged;
    const float TargetMinDistance = .27f;
    [SerializeField]public float TargetAccelerationLimit = 120f;
    const float PostTargetBlendWidth = .05f;
    const float PostTargetGravityRatio = .50f;
    
    float sumTime = 0;
    private float bitInTime = 0.2f;
    private float SubtleLimitime = 0.2f;
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
    [Tooltip("モバイルではDPIからmm換算、PCマウスでは1080p時の基準pxとして使用。方向・距離・速度で判定します。")]
    [Min(1f)] [SerializeField] float minimumFlickPixels = 10f;

    [Tooltip("Turn Input Intentを保持する最大時間[s]。古いフリックがRecovery/前回Turn終了後に突然実行されることを防ぎます。")]
    [Min(0.05f)] [SerializeField] float turnInputBufferSeconds = 0.25f;

    [Tooltip("極限ポイント通過後、階段上でフリックしたときのBallVisual Handoff待ち上限[s]。")]
    [Min(0.25f)] [SerializeField] float slopeExtremeTurnBufferSeconds = 1.0f;

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

    [Header("Pre-Extreme Steering / 極限前の物理的な進路微調整")]
    [Tooltip("1回の横フリックで目標とする横方向の距離。通常の90度旋回とは独立しています。")]
    const float earlyFlickStepMeters = 1.2f;
    [Tooltip("同じ階段上で蓄積する目標オフセットの最大距離。")]
    const float earlyFlickTotalMeters = 3f;
    const float earlyFlickMaxSideSpeed = 7f;
    const float earlyFlickMaxSideAcceleration = 140f;
    const float earlyFlickResponse = 50f;
    const float earlyFlickEndBrakeSeconds = 0.04f;
    [SerializeField] bool logEarlyFlick = true;

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

    public bool StickSlopePharse = false;
    public bool AcceleFirstTurn = false;
    [Header("PlayerMotivation")] [SerializeField]
    public bool BeginCommandOnTouch = false;
    [Header("Debug")]
    [SerializeField] bool logCore;

    [Header("DropOut Distance Diagnostics")]
    [Tooltip("非接地になった瞬間と、最後の接触物のTransformから9m以上離れた間の診断ログを出します。死亡確定ログはこの設定に関係なく出ます。")]
    [SerializeField] bool logDropoutDistance = true;
    [Min(0.05f)] [SerializeField] float dropoutLogIntervalSeconds = 0.25f;
    float nextDropoutLogFixedTime;
    bool previousGroundedForDropoutLog = true;

    [Header("DropOut Cause Audit / 原因調査専用")]
    [Tooltip("旋回時と接地喪失時だけ履歴を出力します。Rigidbodyやステージの動作は変更しません。")]
    [SerializeField] bool logDropoutInvestigation = true;
    public bool LogDropoutInvestigation => logDropoutInvestigation;

    // 直近64物理ステップは数値だけ保持し、接地を失ったときだけログをまとめて出す。
    const int DropAuditCapacity = 64;
    const int DropAuditHistorySteps = 28;
    const int DropAuditPostSteps = 12;
    struct DropAuditSample
    {
        public float time;
        public int frame;
        public Vector3 position, velocity, heading, guidePoint;
        public float guideDistance, crossTrack, normalOffset, lateralSpeed, tangentAlignment;
        public float stickState, stickAppliedLastTick, graceTimer;
        public bool kinematic, turning, waitingGuide, guideValid, slope;
        public bool supportKnown, grounded;
        public string lastHitName, newHitName;
        public TurnResolutionMode turnMode;
    }
    readonly DropAuditSample[] dropAuditHistory = new DropAuditSample[DropAuditCapacity];
    readonly Collider[] dropAuditNearby = new Collider[16];
    int dropAuditNext, dropAuditCount, dropAuditPostRemaining;
    float dropAuditStickAppliedLastTick;


    // 進捗誤差が負から0以上になった瞬間だけ、表示側へ通知する。
    NearestKnotDetector.GuideFrame extremePointSection;
    bool extremePointSectionValid;
    bool extremePointHasPrevious;
    public bool extremePointFiredInSection;
    public bool ExtremePointPassedInCurrentSlope => extremePointFiredInSection;
    float previousExtremePointError;
    float previousExtremePointTarget;

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
    // The press began before the slope's energy target; crossing it mid-drag
    // must not upgrade this gesture to a 90-degree turn while still on the slope.
    bool flickStartedBeforeExtreme;
    // Remember a flat press across slope entry; never reinterpret it as EarlyFlick.
    bool flickStartedOnFlat;
    bool tapEdgePressOnFlat;
    Collider flickPressFlatCollider;
    float flickPressUnscaledTime;

    // Independent from Auto Hold: flat TAP -> any number of pre-target
    // EarlyFlicks -> automatic next-flat turn -> existing next-slope edge chase.
    enum TapEdgeState { None, Armed, Qualified, TurnQueued }
    TapEdgeState tapEdgeState;
    Collider tapEdgeFlatCollider;
    bool tapEdgeFastTurnQueued, tapEdgeHandoffHeld;
    NearestKnotDetector.GuideFrame tapEdgeSlopeGuide;
    float tapEdgeMaxSlopeProgress;

    [Header("Auto Hold / 左右どちらの半分でも押しっぱなし旋回")]
    [SerializeField] bool autoInCornerHoldEnabled = true;
    [Tooltip("階段で押し始めるSlopeReleaseHoldの横移動許容[px]。平面始動のAutoHoldは、次の平面で離すと移動履歴を破棄するため対象外。")] 
    [Min(1f)] [SerializeField] float autoHoldHorizontalTolerancePixels = 10f;
    [Tooltip("階段の中央から進行方向左へ、この距離以上で左半分とみなす[m]。")] 
    [Min(0f)] [SerializeField] float autoHoldLeftHalfMarginMeters = .05f;
    [Tooltip("Auto Holdで物理Colliderの左端へ寄せる最大横速度[m/s]。")]
    [Min(.1f)] [SerializeField] float autoHoldMaxSideSpeed = 3f;
    [Tooltip("Auto Holdで物理Colliderの左端へ寄せる最大横加速度[m/s²]。")] 
    [Min(1f)] [SerializeField] float autoHoldMaxSideAcceleration = 55f;
    [Tooltip("旋回前の物理エッジ推定に使用する横方向の許容幅[m]。")] 
    [Min(.5f)] [SerializeField] float autoHoldExitWidthTolerance = 3.5f;
    [Tooltip("幾何的に方向が不明な通常TapEdge用の旧フォールバック。長押し解放旋回には使用しない。OFF=左、ON=右。")] 
    [SerializeField] bool autoHoldFallbackTurnRight = false;

    bool autoHoldEligible, autoHoldSawSlope, autoHoldLastHalfValid, autoHoldLeftHalf;
    bool autoHoldTurnPending;
    // B ONLY: receiving-flat turn can be authorized by MouseUp/Touch.Ended,
    // never by an old held-input automatic-turn path or FixedUpdate.
    bool flatStartReleaseCommitInProgress;
    // B: movement state belongs to its actual receiving flat until turn begins.
    // This is never an automatic held-input turn or an unrelated rescue motor.
    bool bReleaseHandoffActive;
    Collider bReleaseHandoffFlat;
    Vector3 bReleaseOriginalHeading;
    float bReleaseForwardSpeed;
    float bReleaseHandoffTime = -1f;
    float bReleaseNextLogTime;
    const float BReleaseHandoffSpeedCeiling = 9.5f;
    const float BReleaseHandoffBrakeAcceleration = 90f;
    const float BReleaseHandoffSideAcceleration = 85f;
    // Existing FLAT-start AutoHold now waits for release ON its next real flat.
    // This is NOT a new gesture: it retains the old slope observations and
    // QueueAutoHoldHeldTurn path. The collider is the physical exit anchor.
    bool autoHoldNextFlatReady;
    Collider autoHoldNextFlatCollider;
    float autoHoldNextFlatLastSupportTime = -1f;
    Vector3 autoHoldNextFlatLastSupportPosition;

    // Physical flat evidence from the ordinary FixedUpdate support sample.
    // Only an immediately recent flat can lend an extra 0.08s of press room
    // at its exit. It is NOT a separate gesture or an alternate turn route.
    const float AutoHoldFlatPressGraceSeconds = .08f;
    const float AutoHoldFlatPressGraceSlopeProgress = .14f;
    Collider autoHoldRecentPressFlatCollider;
    float autoHoldRecentPressFlatTime = -1f;
    Vector3 autoHoldRecentPressFlatPosition, autoHoldRecentPressFlatHeading;

    // Slope-start hold: press on a real slope, keep the finger down across
    // its exit, then RELEASE on the immediately following physical flat.
    // It shares Auto Hold's verified physical-edge approach and UTurn handoff,
    // but it never triggers the old automatic flat-entry commit.
    bool slopeReleaseHoldArmed, slopeReleaseHoldNextFlatSeen;
    bool slopeReleaseHoldTurnPending, slopeReleaseHoldHandoffHeld;
    NearestKnotDetector.GuideFrame slopeReleaseHoldStartGuide;
    // Survives ResetAutoHoldObservation while the release turn is queued.
    NearestKnotDetector.GuideFrame slopeReleaseCommittedSlopeGuide;
    float slopeReleaseHoldMaxProgress;
    Collider slopeReleaseHoldNextFlatCollider;
    float slopeReleaseHoldLastFlatSupportTime = -1f;
    Vector3 slopeReleaseHoldLastFlatSupportPosition;

    [Header("Slope Hold: Extreme / Timed Spline Escape")]
    [Tooltip("階段の極限ポイント通過後に開始した長押しも次の平面で離すと受付。")]
    [SerializeField] bool slopeReleaseAllowPostExtremePress = true;
    [Tooltip("Handoff待ちで水平速度がほぼ0になってから、Splineへの安全復帰を開始するまでの秒数。通常の0.12秒Turn Freezeには適用しない。")]
    [Min(.05f)] [SerializeField] float slopeReleaseStoppedSecondsToEscape = .30f;
    [Tooltip("この水平速度[m/s]未満の間だけ停止時間を加算。")]
    [Min(.05f)] [SerializeField] float slopeReleaseStopSpeedThreshold = .40f;
    [Tooltip("逃げ先への側方加速度上限[m/s2]。位置をテレポートしない。")]
    [Min(1f)] [SerializeField] float slopeReleaseEscapeSideAcceleration = 45f;
    [Tooltip("逃げ先Splineに沿って再発進する前方加速度上限[m/s2]。")]
    [Min(1f)] [SerializeField] float slopeReleaseEscapeForwardAcceleration = 35f;

    bool slopeReleaseStartedAfterExtreme;
    bool slopeReleaseExtremeReachedWhileHeld;
    Vector3 slopeReleaseBeforeStopVelocity, slopeReleaseBeforeStopAcceleration;
    Vector3 slopeReleaseLastSampleVelocity, slopeReleaseObservedAcceleration;
    bool slopeReleaseLastSampleValid;
    float slopeReleaseStopStartFixedTime = -1f;
    float slopeReleaseEscapeNextNoLaneLogFixedTime;
    bool slopeReleaseEscapeActive;
    Vector3 slopeReleaseEscapeLinePoint, slopeReleaseEscapeHeading;
    Collider slopeReleaseEscapeFloor;
    float slopeReleaseEscapeStartedFixedTime;
    NearestKnotDetector.FiveLineGroup slopeReleaseEscapeGroup;

    // General near-stall detection. Unlike the dedicated 0.30s Handoff
    // escape, this is available on BOTH flat and stair sections, even when
    // the player never pressed. It only assists after actual loss of progress.
    [Header("Safe Spline Recovery / 平面と階段の失速救済")]
    [SerializeField] bool enableGeneralStallRecovery = true;
    [Tooltip("これ以下の速度で、さらに実際の前進距離が少ない状態を監視 [m/s]")]
    [Min(.1f)] [SerializeField] float stallWatchForwardSpeed = 1.25f;
    [Tooltip("失速が連続した場合に回復を検討する時間 [s]")]
    [Min(.20f)] [SerializeField] float stallWatchSeconds = .75f;
    [Tooltip("監視時間中、この距離以上進めた場合は失速扱いしない [m]")]
    [Min(.05f)] [SerializeField] float stallWatchMaxProgress = .30f;
    [Tooltip("一度退避を試みた後の再発動禁止期間 [s]")]
    [Min(.5f)] [SerializeField] float stallRecoveryCooldown = 2f;
    [Tooltip("安全なSplineに戻す横加速度上限 [m/s2]")]
    [Min(1f)] [SerializeField] float stallRecoverySideAcceleration = 45f;
    [Tooltip("Splineに沿って再発進する加速度上限 [m/s2]")]
    [Min(1f)] [SerializeField] float stallRecoveryForwardAcceleration = 35f;

    bool generalStallRecovering;
    // Separate from ordinary near-stop: a released 90-degree turn can be
    // stranded while waiting for the NEW guide, even on a real stair tread.
    bool stallRecoveryForUncapturedTurn;
    float turnGuideStallWatchSince = -1f;
    float turnGuideStallNextLogAt;
    Vector3 turnGuideStallWatchPosition;
    const float TurnGuideStallAfterReleaseSeconds = .30f;
    const float TurnGuideStallObservationSeconds = .40f;
    const float TurnGuideStallMinProgress = .28f;
    const float TurnGuideStallMaximumPlanarSpeed = 2.50f;
    float stallLowSpeedSince = -1f, stallLastHealthyTime = -1f;
    float stallLastAttemptAt = -1f, stallRecoveryStartedAt;
    float stallNextDiagnosticAt;
    Vector3 stallStartPosition, stallLastMovingVelocity, stallLastMovingAcceleration;
    Vector3 stallPreviousMotionVelocity;
    bool stallPreviousVelocityValid;
    Vector3 stallRecoveryPoint, stallRecoveryHeading;
    bool stallRecoveryWasSlope;
    NearestKnotDetector.FiveLineGroup stallRecoveryLane;

    // Early-stall samples are distinct from the existing 0.75s near-stop
    // timer. A moving-but-not-converging ball must not reset that observation
    // simply by travelling 0.30m along the wrong lane.
    const float StallEarlyCreepSeconds = .40f;
    const float StallEarlyCreepProgress = .20f;
    const float StallEarlyCreepTravel = .30f;
    const float StallEarlyNoConvergenceSeconds = .45f;
    const float StallEarlyCrossTrack = 1.00f;
    const float StallEarlyRequiredConvergence = .10f;
    float stallEarlyWatchSince = -1f, stallEarlyStartCrossTrack;
    Vector3 stallEarlyStartPosition, stallEarlyStartHeading;
    int stallEarlySectionIndex;
    bool stallEarlyWasSlope;

    NearestKnotDetector.GuideFrame autoHoldLastSlopeGuide;
    Vector3 autoHoldLastCenter, autoHoldLastLeft, autoHoldLastForward;
    Vector3 autoHoldExitPoint, autoHoldLastInnerLinePoint;
    float autoHoldLeftOffset, autoHoldInnerErrorAtCommit;
    float autoHoldTurnSign;
    // Cached read-only future-slope geometry: avoids a costly spline scan
    // every physics tick and works even when CurrentGuide is still a slope.
    bool autoHoldPredictedTurnSignValid;
    float autoHoldPredictedTurnSign;
    float autoHoldLastSlopeProgress;
    // Auto Hold only: cached physical edge (no new Inspector settings).
    Vector3 autoHoldTurnLeft, autoHoldEdgeSample;
    float autoHoldNextEdgeSampleTime;
    bool autoHoldEdgeSampleValid;
    // Validated collider and sampling height are retained across turn Handoff.
    Collider autoHoldEdgeCollider;
    float autoHoldEdgeProbeHeight, autoHoldEdgeSurfaceHeight;
    readonly RaycastHit[] autoHoldCastHits = new RaycastHit[16];

    // Only the NEXT flat and NEXT slope following a committed Auto Hold turn.
    // No extra Inspector controls, and no correction during normal flick turns.
    bool postAutoHoldEdgeActive, postAutoHoldNewGuide, postAutoHoldSawSlope;
    bool postAutoHoldHadEdgeSample, postAutoHoldCorrectionActive;
    bool postAutoHoldFastArrival, postAutoHoldFastDeadlineLogged;
    bool postAutoHoldUniversalInCorner;
    bool postAutoHoldUniversalFallbackLogged;
    bool postAutoHoldFastFlatPrescanLogged;
    bool postAutoHoldFastArrivedBeforeThird;
    bool postAutoHoldFastInfeasibleLogged;
    Vector3 postAutoHoldHeading, postAutoHoldEdgeTarget;
    float postAutoHoldStartedAt, postAutoHoldNextProbeTime;
    float postAutoHoldNextFlatProbeFailureLogTime;
    float postAutoHoldLastError, postAutoHoldPeakSlopeError;
    float postAutoHoldMaxSlopeProgress;
    int postAutoHoldSlopeSamples, postAutoHoldCorrections;

    // After a VERIFIED edge arrival, retain a small physical safety margin.
    // This is a separate state: clearing the approach must not remove cruising
    // protection. Never lock the old collider/edge across a 90-degree turn.
    bool edgeCruiseActive, edgeCruiseSampleValid;
    Vector3 edgeCruiseHeading, edgeCruiseTarget;
    Collider edgeCruiseSupportCollider;
    float edgeCruiseNextProbeTime, edgeCruiseSampleTime, edgeCruiseNextLogTime;
    int edgeCruiseCorrections;

    // Android/iPhone: the specific game-play touch that began outside the gauge.
    int activeFlickFingerId = -1;
    float pendingTurnDegrees;
    float pendingTurnQueuedTime = -1f;
    // 階段の極限ポイント通過後にフリックした事実を、Handoff待ち中も保持する。
    bool pendingTurnFromPassedSlope;

    // 旧版の段階的なSphereCastや膨大なログは使用しない。
    // 進路の変更はInSubjectの側方加速度に限定し、Stick力は変更しない。
    int pendingEarlyFlickSteps;
    bool earlyFlickSession;
    bool earlyFlickMoving;
    // Only a flick FIRST recognized after crossing may complete on the remaining slope.
    bool earlyFlickStartedAfterTarget;
    NearestKnotDetector.GuideFrame earlyFlickGuide;
    float earlyFlickInitialOffset;
    float earlyFlickTargetOffset;

    // CorrespondSubjectの表示側が横方向だけ追従するための読み取り専用情報。
    public bool HasPreExtremeLateralSession =>
        earlyFlickSession && currentSurfaceValid && currentGuideValid &&
        currentGuide.isSlope && !MainGameManager.OnDead;
    public Vector3 PreExtremeSidePhysics =>
        currentSurfaceValid ? currentSurface.side : Vector3.zero;

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

    // A new nearest Spline may be perpendicular at a geometric crossing.
    // The player did not request a turn, so never treat that nearest line as
    // the new forward direction. This guard does not fabricate a ground hit.
    const float UncommandedGuideAlignmentMin = 0.65f;
    // No uncommanded seam brake: only input-authorized turn/hold paths may stop the body.
    bool uncommandedGuideGuardLogged;

    // Dropout guard: no additional Inspector settings. Ordinary successful
    // turns continue unchanged; intervention occurs only during an unsafe handoff.
    const float TurnGuideCoastAllowanceSeconds = 0.26f;
    const float TurnGuideBrakeAcceleration = 90f;
    const float TurnGuideAheadMaximumMeters = 4f;
    const float TurnGuideSafeCrawlSpeed = 3f;
    // Intentional turn only: brake translation toward a safe positive speed,
    // damp excess spin without preventing physical rolling, and limit the
    // first stair-tread Stick transient after guide reacquisition.
    const float TurnGuideSpinResponse = 7f;
    const float TurnGuideMaxAngularAcceleration = 65f;
    const float TurnGuideLandingSeconds = .24f;
    const float TurnGuideLandingStickLimit = 35f;
    const float TurnGuideForwardAssistLimit = 48f;
    const float TurnGuideNearSlopeSafeSpeed = 2.8f;
    const float TurnGuideMinimumSupportedSpeed = 1.7f;
    float turnHandoffNextAuditTime;
    float turnGuideCapturedAt = -1f;

    // Reverse-motion protection ONLY for an accepted B-release InCorner turn
    // and the first physically verified descending stair after that turn.
    // Ordinary straights, TapEdge, other flicks and OutCorner are untouched.
    const float PostBDownhillMaxWindow = 4.0f;
    const float PostBDownhillMinimumForwardSpeed = 2.5f;
    const float PostBDownhillMaxAssist = 42f;
    const float PostBDownhillStickLowSpeedCap = 45f;
    bool postBDownhillArmed, postBDownhillSawSlope;
    bool postBDownhillWrongGuideLogged;
    Vector3 postBDownhillCommittedHeading;
    int postBDownhillSectionIndex = -1;
    float postBDownhillArmedAt = -1f;
    float postBDownhillLastSupportTime = -1f;
    float postBDownhillRecentNearSlopeTime = -1f;
    float postBDownhillNextLogTime;
    // Same physical StairWay family survives knot/lane/section switches.
    // The left edge is measured on real floor, then held across a guide jump.
    int postBDownhillStairFamily = -1;
    int postBDownhillLastLoggedSection = -1;
    bool postBEdgeAnchorValid;
    float postBEdgeAnchorLateral;
    float postBEdgeReacquireUntil = -1f;
    float postBEdgeNextLogTime;
    const float PostBEdgeMaxSideSpeed = 5f;
    const float PostBEdgeMaxSideAcceleration = 75f;
    const float PostBEdgeAnchorNoiseMeters = .24f;
    const float PostBEdgeAnchorSlewMetersPerSecond = 5f;

    float turnGuideReleasedAt = -1f;
    bool turnGuideGuardBraking;
    bool turnGuideCrawlLogged;

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
        SubtleLimitime = sumTime + bitInTime;
        FindMapFrameReferences();
        CaptureInitialVisualFramePose();
        BindCoordinateFrames();
        StartCoroutine(delayStart());
    }

    void Update()
    {
        ReadTurnFlick();
    }

    void OnDisable()
    {
        ClearPostBDownhillGuard("lifecycle reset");
        ClearBReleaseHandoff();
        turnGuideCapturedAt = -1f;
        ResetGeneralStallRecovery("component disabled", true);
        ResetSlopeReleaseTimedEscape("component disabled");
        if (slopeReleaseHoldTurnPending)
            ExpirePendingTurnIntent();
        turnGuideReleasedAt = -1f;
        turnGuideGuardBraking = false;
        turnGuideCrawlLogged = false;
        ResetTapEdge();
        ClearPostAutoHoldLeftEdge();
        StopEdgeCruise("component disabled");
        ClearEarlyFlick();
        ResetFlickGesture();
        autoHoldRecentPressFlatCollider = null;
        autoHoldRecentPressFlatTime = -1f;
        correspondSubject?.ResetTurnGestureTracking();
    }

    void OnApplicationPause(bool paused)
    {
        ClearPostBDownhillGuard("lifecycle reset");
        if (!paused) return;
        ClearBReleaseHandoff();
        turnGuideCapturedAt = -1f;
        ResetGeneralStallRecovery("application paused", true);
        ResetSlopeReleaseTimedEscape("application paused");
        if (slopeReleaseHoldTurnPending)
            ExpirePendingTurnIntent();
        ResetTapEdge();
        ClearPostAutoHoldLeftEdge();
        StopEdgeCruise("application paused");
        ClearEarlyFlick();
        ResetFlickGesture();
        autoHoldRecentPressFlatCollider = null;
        autoHoldRecentPressFlatTime = -1f;
        correspondSubject?.ResetTurnGestureTracking();
    }

    public void PushStart()
    {
        MainGameManager.TopTitle.SetActive(false);
        MainGameManager.PreviewIconRoot.SetActive(false);
        MainGameManager.TopLiteral.SetActive(false);
         MainGameManager.PlayButton.SetActive(false);
        MainGameManager.Userbility.SetActive(true);
        MainGameManager.HopingSwipeTurn.SetActive(true);
        
       
        BeginCommandOnTouch = true;

        Earliest=true;
        if (!AcceleFirstTurn)
        {
            StartCoroutine(delayTurn());
        }
    }

    IEnumerator delayTurn()
    {
        yield return new WaitForSeconds(1f);
        if (!AcceleFirstTurn)
        {
            MainGameManager.PlayButton.SetActive(true);
            turnTransitionActive = false;
            waitingForTurnGuide = false;
            BeginCommandOnTouch = false;
        }


    }

    IEnumerator Recover()
    {
        
        yield return new WaitForSeconds(0.5f);
        MainGameManager.DropOut.SetActive(true);
        yield return new WaitForSeconds(2f);
        MainGameManager.DropOut.SetActive(false);
        AcceleFirstTurn = false;
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
        if (logDropoutInvestigation)
            Debug.Log($"[DROP AUDIT RESTART] t={Time.fixedTime:F3} pos={rb.position:F3} " +
                      $"restartPrepared={restartPrepared}", this);

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

    // 判定に使っているTransform原点までの距離と、Collider表面までの距離を
    // 同時に記録して、「長い階段のTransform原点による誤判定」を判別する。
    // ここから物理状態や死亡条件は変更しない。
    void LogDropoutDistanceDiagnostic(string phase, bool grounded, RaycastHit latestHit, float originDistance)
    {
        Transform previousTransform = currentHit.transform;
        Collider previousCollider = currentHit.collider;
        bool hasPreviousCollider = previousCollider != null;
        // Unity's ClosestPoint cannot be safely used for a non-convex mesh.
        bool canQueryClosest = hasPreviousCollider &&
            (!(previousCollider is MeshCollider mesh) || mesh.convex);
        Vector3 closestPoint = canQueryClosest
            ? previousCollider.ClosestPoint(transform.position)
            : Vector3.zero;
        float surfaceDistance = canQueryClosest
            ? Vector3.Distance(transform.position, closestPoint)
            : -1f;

        string message =
            $"[CORE DROPOUT {phase}] fixedTime={Time.fixedTime:F3} frame={Time.frameCount} " +
            $"grounded={grounded} onDeadBefore={MainGameManager.OnDead} " +
            $"originDistance={originDistance:F4} threshold=12.0000 " +
            $"lastHit={(previousTransform != null ? previousTransform.name : "<null>")} " +
            $"lastHitOrigin={(previousTransform != null ? previousTransform.position.ToString("F4") : "<null>")} " +
            $"lastHitPoint={currentHit.point.ToString("F4")} " +
            $"lastCollider={(hasPreviousCollider ? previousCollider.name : "<null>")} " +
            $"closestPoint={(hasPreviousCollider ? closestPoint.ToString("F4") : "<null>")} " +
            $"surfaceDistance={surfaceDistance:F4} " +
            $"latestHit={(latestHit.transform != null ? latestHit.transform.name : "<null>")} " +
            $"latestHitPoint={latestHit.point.ToString("F4")} " +
            $"playerPos={transform.position.ToString("F4")} rbPos={rb.position.ToString("F4")} " +
            $"velocity={rb.velocity.ToString("F4")} speed={rb.velocity.magnitude:F3} " +
            $"graceTimer={graceTimer:F3} supportGrace={supportGraceSeconds:F3} " +
            $"guideDistance={currentGuide.distanceToGuide:F4} guideIsSlope={currentGuide.isSlope} " +
            $"kinematic={rb.isKinematic}";

        /*if (phase == "TRIGGER")
            Debug.LogError(message, this);
        else
            Debug.LogWarning(message, this);*/
    }

    // 比較不能な間は直前サンプルを捨てる。発火済み区間は保持する。
    void PauseExtremePointTracking()
    {
        extremePointHasPrevious = false;
    }

    // 平面に戻ったとき、またはゲーム再構築時だけ再武装する。
    // 階段の中でKnot/Sectionが切り替わっても発火済みフラグを消さない。
    void ResetExtremePointTracking()
    {
        ClearEarlyFlick();
        extremePointSection = default;
        extremePointSectionValid = false;
        extremePointFiredInSection = false;
        extremePointHasPrevious = false;
        previousExtremePointError = 0f;
        previousExtremePointTarget = 0f;
    }

    // 階段 -> 平面を観測した物理フレームでParticleを止める。
    // 非接地やSplineの一時的な欠測ではStopしない。
    void CompleteExtremePointAtFlat()
    {
        if (!extremePointSectionValid)
            return;

        correspondSubject?.StopExtremePointEffect();
        ResetExtremePointTracking();
    }

    void ObserveExtremePoint(NearestKnotDetector.GuideFrame guide)
    {
        if (!guide.valid || !guide.isSlope || !knotDetector || MainGameManager.OnDead)
        {
            PauseExtremePointTracking();
            return;
        }

        bool sameSection = extremePointSectionValid &&
            knotDetector.IsSameSection(extremePointSection, guide);

        if (!sameSection)
        {
            extremePointSection = guide;
            extremePointSectionValid = true;
            // 発火済みフラグは保持。解除するのはFlatへの移行時だけ。
            extremePointHasPrevious = false;
        }

        float target = TargetProgress;
        float error = guide.sectionProgress01 - target;

        // Inspector/UIで設定値が変わっただけでは発火させない。
        bool targetUnchanged = extremePointHasPrevious &&
            Mathf.Abs(previousExtremePointTarget - target) <= Eps;

        bool crossedZero = targetUnchanged &&
            previousExtremePointError < 0f && error >= 0f;
        
        MainGameManager.OnInifnite = crossedZero;

        // 目標が0%なら、斜面の最初の有効なサンプルを発火点とする。
        bool enteredAtZero = !extremePointHasPrevious &&
            target <= Eps && error >= 0f;

        if (!extremePointFiredInSection && (crossedZero || enteredAtZero))
        {
            extremePointFiredInSection = true;
            correspondSubject?.PlayExtremePointEffect();
        }

        previousExtremePointError = error;
        previousExtremePointTarget = target;
        extremePointHasPrevious = true;
    }

    // ================================================================
    // DropOut Cause Audit: observation only (no force, velocity or transform writes)
    // ================================================================
    void CaptureDropAuditBeforeStep()
    {
        if (!logDropoutInvestigation || !rb)
            return;

        Vector3 heading = (turnTransitionActive || waitingForTurnGuide)
            ? turnTargetDirection : direction;
        heading = Vector3.ProjectOnPlane(heading, Vector3.up);
        if (heading.sqrMagnitude > Eps)
            heading.Normalize();

        Vector3 side = heading.sqrMagnitude > Eps
            ? Vector3.Cross(Vector3.up, heading) : Vector3.zero;

        dropAuditHistory[dropAuditNext] = new DropAuditSample
        {
            time = Time.fixedTime,
            frame = Time.frameCount,
            position = rb.position,
            velocity = rb.velocity,
            heading = heading,
            lateralSpeed = Vector3.Dot(rb.velocity, side),
            kinematic = rb.isKinematic,
            turning = turnTransitionActive,
            waitingGuide = waitingForTurnGuide,
            turnMode = activeTurnMode,
            stickState = stickState,
            stickAppliedLastTick = dropAuditStickAppliedLastTick,
            graceTimer = graceTimer,
            lastHitName = currentHit.collider ? currentHit.collider.name : "-",
            newHitName = "-"
        };
        dropAuditNext = (dropAuditNext + 1) % DropAuditCapacity;
        dropAuditCount = Mathf.Min(dropAuditCount + 1, DropAuditCapacity);
        // このFixedUpdateが実際にAddForceしたときだけ、次のサンプルへ記録する。
        dropAuditStickAppliedLastTick = 0f;
    }

    void CaptureDropAuditSupport(NearestKnotDetector.GuideFrame guide, bool grounded, RaycastHit hit)
    {
        if (!logDropoutInvestigation || dropAuditCount == 0)
            return;

        int index = (dropAuditNext + DropAuditCapacity - 1) % DropAuditCapacity;
        DropAuditSample record = dropAuditHistory[index];
        record.supportKnown = true;
        record.grounded = grounded;
        record.guideValid = guide.valid;
        record.slope = guide.isSlope;
        record.graceTimer = graceTimer;
        record.guideDistance = guide.distanceToGuide;
        record.newHitName = hit.collider ? hit.collider.name : "-";

        if (guide.valid)
        {
            record.guidePoint = guide.point;
            Vector3 normal = guide.normal.sqrMagnitude > Eps ? guide.normal.normalized : Vector3.up;
            Vector3 tangent = Vector3.ProjectOnPlane(guide.tangent, normal);
            if (tangent.sqrMagnitude > Eps)
            {
                tangent.Normalize();
                Vector3 side = Vector3.Cross(normal, tangent).normalized;
                record.tangentAlignment = Mathf.Abs(Vector3.Dot(tangent, record.heading));
                record.crossTrack = Vector3.Dot(rb.position - guide.point, side);
                record.normalOffset = Vector3.Dot(rb.position - guide.point, normal);
            }
        }
        dropAuditHistory[index] = record;
    }

    void WriteDropAuditSample(DropAuditSample record, string phase)
    {
        Debug.Log(
            $"[DROP AUDIT {phase}] t={record.time:F3} frame={record.frame} " +
            $"pos={record.position:F3} vel={record.velocity:F3} " +
            $"heading={record.heading:F3} lateralSpeed={record.lateralSpeed:F3} " +
            $"guideValid={record.guideValid} slope={record.slope} " +
            $"guidePoint={record.guidePoint:F3} crossTrack={record.crossTrack:F3} " +
            $"normalOffset={record.normalOffset:F3} guideDist={record.guideDistance:F3} " +
            $"guideHeadingAlignment={record.tangentAlignment:F3} " +
            $"groundKnown={record.supportKnown} grounded={record.grounded} " +
            $"hit={record.lastHitName}->{record.newHitName} " +
            $"stickState={record.stickState:F2} stickAppliedLastTick={record.stickAppliedLastTick:F2} " +
            $"grace={record.graceTimer:F3} kinematic={record.kinematic} " +
            $"turning={record.turning} waitingGuide={record.waitingGuide} mode={record.turnMode}", this);
    }

    void LogDropAuditTurn(string phase, Vector3 extraVelocity, float intendedDegrees = float.NaN)
    {
        if (!logDropoutInvestigation || !rb)
            return;

        Vector3 target = Vector3.ProjectOnPlane(turnTargetDirection, Vector3.up);
        if (target.sqrMagnitude > Eps)
            target.Normalize();
        float lateral = Vector3.Dot(rb.velocity, Vector3.Cross(Vector3.up, target));
        float loggedDegrees = float.IsNaN(intendedDegrees) ? activeTurnDegrees : intendedDegrees;
        Debug.Log(
            $"[DROP TURN {phase}] t={Time.fixedTime:F3} " +
            $"mode={activeTurnMode} degrees={loggedDegrees:F1} " +
            $"pos={rb.position:F3} velocity={rb.velocity:F3} " +
            $"snapshotVelocity={extraVelocity:F3} direction={direction:F3} " +
            $"target={turnTargetDirection:F3} lateralToTarget={lateral:F3} " +
            $"frozen={turnBodyFrozen} waitingGuide={waitingForTurnGuide} " +
            $"kinematic={rb.isKinematic}", this);
    }

    // SphereCastが外れた物理ステップで、Rayと球の重なりを追加診断する。
    // Ray側の5点は「床の端」そのものの計測ではなく、近傍床の存在確認。
    void LogDropAuditGeometry(RaycastHit sphereHit)
    {
        if (!logDropoutInvestigation || !rb)
            return;

        Vector3 origin = rb.worldCenterOfMass + Vector3.up * .05f;
        Vector3 forward = Vector3.ProjectOnPlane(turnTargetDirection, Vector3.up);
        if (forward.sqrMagnitude <= Eps)
            forward = Vector3.ProjectOnPlane(direction, Vector3.up);
        forward = forward.sqrMagnitude > Eps ? forward.normalized : Vector3.forward;
        Vector3 side = Vector3.Cross(Vector3.up, forward).normalized;
        float delta = ProbeRadius * .8f;
        float maxDistance = probeDistance + ProbeRadius + .25f;
        Vector3[] offsets = { Vector3.zero, side * delta, -side * delta, forward * delta, -forward * delta };
        string[] names = { "center", "right", "left", "forward", "back" };
        for (int i = 0; i < offsets.Length; i++)
        {
            bool hit = Physics.Raycast(origin + offsets[i], Vector3.down,
                out RaycastHit ray, maxDistance, groundMask, QueryTriggerInteraction.Ignore);
            Debug.Log(
                $"[DROP GEOMETRY ray-{names[i]}] t={Time.fixedTime:F3} " +
                $"found={hit} collider={(hit ? ray.collider.name : "-")} " +
                $"distance={(hit ? ray.distance : -1f):F3} " +
                $"normal={(hit ? ray.normal : Vector3.zero):F3} " +
                $"origin={(origin + offsets[i]):F3}", this);
        }

        int total = Physics.OverlapSphereNonAlloc(rb.worldCenterOfMass, ProbeRadius + .05f,
            dropAuditNearby, groundMask, QueryTriggerInteraction.Ignore);
        // OverlapSphereは近傍Colliderの診断であり、実際の接触数は表しません。
        Debug.Log(
            $"[DROP GEOMETRY overlap] t={Time.fixedTime:F3} " +
            $"overlapCount={total} capacity={dropAuditNearby.Length} " +
            $"sphereCastHit={(sphereHit.collider ? sphereHit.collider.name : "-")} " +
            $"sphereNormal={sphereHit.normal:F3} sphereHitDistance={sphereHit.distance:F3} " +
            $"maxSlopeAngle={maxSlopeAngle:F2} " +
            $"lastCollider={(currentHit.collider ? currentHit.collider.name : "-")} " +
            $"lastColliderEnabled={(currentHit.collider && currentHit.collider.enabled)} " +
            $"lastColliderType={(currentHit.collider ? currentHit.collider.GetType().Name : "-")}", this);
        for (int i = 0; i < total; i++)
            Debug.Log($"[DROP GEOMETRY nearby] {dropAuditNearby[i].name} layer={dropAuditNearby[i].gameObject.layer}", this);
    }

    void LogDropAuditGuideLost()
    {
        if (!logDropoutInvestigation || !currentGuideValid)
            return;
        Debug.LogWarning($"[DROP AUDIT GUIDE-INVALID] t={Time.fixedTime:F3} " +
            $"pos={rb.position:F3} vel={rb.velocity:F3} " +
            $"lastGuidePoint={currentGuide.point:F3} " +
            $"lastHit={(currentHit.collider ? currentHit.collider.name : "-")}", this);
        int n = Mathf.Min(DropAuditHistorySteps, dropAuditCount);
        int first = (dropAuditNext - n + DropAuditCapacity) % DropAuditCapacity;
        for (int j = 0; j < n; j++)
            WriteDropAuditSample(dropAuditHistory[(first + j) % DropAuditCapacity], "GUIDE-LOST-HISTORY");
        LogDropAuditGeometry(default);
    }

    void FlushDropAuditOnSupportChange(bool justLostGround, RaycastHit hit)
    {
        if (!logDropoutInvestigation || dropAuditCount == 0)
            return;

        int lastIndex = (dropAuditNext + DropAuditCapacity - 1) % DropAuditCapacity;
        if (justLostGround)
        {
            Debug.LogWarning(
                $"[DROP AUDIT LOST] t={Time.fixedTime:F3} " +
                $"mode={activeTurnMode} latestHit={(hit.collider ? hit.collider.name : "-")} " +
                $"lastHit={(currentHit.collider ? currentHit.collider.name : "-")} " +
                $"pos={rb.position:F3} vel={rb.velocity:F3}", this);
            int n = Mathf.Min(DropAuditHistorySteps, dropAuditCount);
            int first = (dropAuditNext - n + DropAuditCapacity) % DropAuditCapacity;
            for (int j = 0; j < n; j++)
                WriteDropAuditSample(dropAuditHistory[(first + j) % DropAuditCapacity], "BEFORE");
            LogDropAuditGeometry(hit);
            dropAuditPostRemaining = DropAuditPostSteps;
        }
        else if (dropAuditPostRemaining > 0)
        {
            WriteDropAuditSample(dropAuditHistory[lastIndex], "AFTER");
            dropAuditPostRemaining--;
        }
    }

    void FixedUpdate()
    {
        CaptureDropAuditBeforeStep();
        CaptureSlopeReleaseBeforeStopMotion();
        sumTime += Time.deltaTime;
        // Updateで予約した90度旋回を、物理/Spline観測より先に1回だけ適用する。
        Debug.Log(
            $"[TOP CHECK] " +
            $"pos={rb.position:F3} " +
            $"velocity={rb.velocity:F3} " +
            $"isKinematic={rb.isKinematic}"
        );
        ApplyPendingQuarterTurn();
        // Reset observation during authorized turn/freeze/handoff/press states.
        if (!generalStallRecovering && GeneralStallHasControlConflict())
        {
            ResetGeneralStallWatch();
            stallPreviousVelocityValid = false;
        }
        // An elapsed stop (rather than a single contact/velocity frame) may
        // authorize a one-time controlled escape onto a REAL supported spline.
        if (slopeReleaseEscapeActive && StepSlopeReleaseTimedEscape())
            return;
        // The rescue motor has its own collision/geometry check every tick.
        // If active, it owns Drive and side force for a bounded duration.
        if (generalStallRecovering && StepGeneralStallRecovery())
            return;

        // If turn Handoff is blocked, do not undo the safe horizontal hold
        // with ordinary spline acceleration later in this same FixedUpdate.
        if (tapEdgeHandoffHeld && tapEdgeFastTurnQueued && HasPendingTurnIntent &&
            tapEdgeState == TapEdgeState.TurnQueued)
        {
            var heldContact = HasGroundSupport();
            if (heldContact.Item1 && heldContact.Item2.collider == tapEdgeFlatCollider)
                return;
        }
        // Unlike normal driving, a queued slope-release turn must not be
        // accelerated off its verified receiving flat while handoff is pending.
        if (slopeReleaseHoldHandoffHeld && slopeReleaseHoldTurnPending &&
            HasPendingTurnIntent)
        {
            var heldContact = HasGroundSupport();
            if (heldContact.Item1 &&
                heldContact.Item2.collider == slopeReleaseHoldNextFlatCollider)
                return;
            slopeReleaseHoldHandoffHeld = false;
        }

        // Visual旋回中は通常のSpline Drive / Stick / FiveLineを止める。
        // UTurnはHardFreeze解除時に90度方向転換済み。円弧追従は行わない。
        if (turnTransitionActive)
        {
            ClearEarlyFlick();
            PauseExtremePointTracking();
            AdvanceTurnTransition();
            if (turnBodyFrozen)
            {
                ResetBallVisualSplineSession();
                return;
            }

            direction = turnTargetDirection;
            driveState = 0f;
            ResetBallVisualSplineSession();

            // HardFreeze後はVisual回転中でもRigidbodyがDynamicになる。
            // 0.30sのTween終了まで無監視で走らせない。
            if (waitingForTurnGuide && !rb.isKinematic)
            {
                var transitioningGuide = knotDetector.Evaluate(rb.position);
                var transitionSupport = HasGroundSupport();
                GuardUncapturedTurnGuide(transitioningGuide,
                    transitionSupport.Item1);
            }
            return;
        }

        // Visual旋回後、新Spline捕捉まで旧SplineのDriveを再開しない。
        if (waitingForTurnGuide)
            direction = turnTargetDirection;

        var guide = knotDetector.Evaluate(rb.position);
        // Evaluate() is geometric proximity, NOT a 90-degree turn command.
        // Reject a sudden perpendicular nearest-line jump before it can
        // redefine lateralSpeed/crossTrack or pull the ball off the slab.
        if (GuardUncommandedGuideJump(guide))
        {
            // A mistaken perpendicular nearest-guide may recur while a ball
            // is physically stranded. Observe the LAST known forward path,
            // but never accept a stale guide more than 1.5m away.
            if (currentGuideValid && currentSurfaceValid &&
                Vector3.Distance(rb.position, currentGuide.point) < 1.5f)
            {
                var knownFloor = HasGroundSupport();
                ObserveGeneralStallRecovery(
                    currentGuide, currentSurface,
                    knownFloor.Item1, knownFloor.Item2);
            }
            else
                ResetGeneralStallWatch();
            return;
        }
        ObserveAutoHoldGuide(guide);

        // Splineが存在しなければCoreは制御しない。
        if (!guide.valid)
        {
            if (waitingForTurnGuide)
            {
                var turnFloor = HasGroundSupport();
                if (ObserveUncapturedTurnGuideStall(guide, turnFloor.Item1))
                    return;
            }
            ResetGeneralStallWatch();
            stallPreviousVelocityValid = false;
            // Guideを見失った間も、旧Velocityで無制限に直進しない。
            if (waitingForTurnGuide)
                GuardUncapturedTurnGuide(guide, HasGroundSupport().Item1);
            LogDropAuditGuideLost();
            LoseSupport();
            return;
        }

        // 平面の最初の有効なGuideで、必ず停止・再武装。
        // BeginCommandOnTouch / 接地状態より前に判定して取りこぼしを防ぐ。
        if (!guide.isSlope)
            CompleteExtremePointAtFlat();

        // Capture previous frame before replacing the spline frame.
        // Its horizontal heading tells us whether this is a true straight
        // flat-to-stair transition or a player-authorized 90-degree turn.
        NearestKnotDetector.GuideFrame previousGuide = currentGuide;
        currentGuide = guide;
        currentGuideValid = true;

        (bool grounded,RaycastHit hit)= HasGroundSupport();
        bool ordinaryStraightFlat = IsUncommandedStraightFlat(guide);
        // A long downward SphereCast is not proof that the ball is still
        // touching a straight slab. Require a real local floor witness.
        if (ordinaryStraightFlat && grounded &&
            !IsCloseGroundRay(hit, rb.worldCenterOfMass))
            grounded = false;
        RememberAutoHoldRecentPhysicalFlat(guide, grounded, hit);

        if (grounded)
        {
            graceTimer = supportGraceSeconds;
        }
        else
        {
            // Airborne travel is not a verified stall, even during Grace.
            graceTimer = Mathf.Max(0f, graceTimer - Time.fixedDeltaTime);
            ResetGeneralStallWatch();
            stallPreviousVelocityValid = false;
        }

        // 死亡判定は従来通り「最後の接触物のTransform原点との距離 > 12 && !grounded」。
        // currentHitはこの下で更新するため、latestHitとlastHitの違いも記録する。
        float lastHitOriginDistance = currentHit.transform != null
            ? Vector3.Distance(transform.position, currentHit.transform.position)
            : -1f;
        
        Debug.Log(
            $"[DROPOUT CHECK] " +
            $"grounded={grounded} " +
            $"distance={lastHitOriginDistance:F3} " +
            $"lastHit={currentHit.transform?.name} " +
            $"newHit={hit.transform?.name} " +
            $"pos={rb.position:F3} " +
            $"velocity={rb.velocity:F3}"
        );
        bool justLostGround = previousGroundedForDropoutLog && !grounded;
        if (logDropoutDistance && !grounded &&
            (justLostGround || lastHitOriginDistance >= 9f) &&
            (justLostGround || Time.fixedTime >= nextDropoutLogFixedTime))
        {
            

            LogDropoutDistanceDiagnostic("WATCH", grounded, hit, lastHitOriginDistance);
            nextDropoutLogFixedTime = Time.fixedTime + Mathf.Max(0.05f, dropoutLogIntervalSeconds);
        }
        CaptureDropAuditSupport(guide, grounded, hit);
        FlushDropAuditOnSupportChange(justLostGround, hit);
        previousGroundedForDropoutLog = grounded;

        if (currentHit.transform != null)
            if (lastHitOriginDistance > 15 && !grounded)
            {

                if (!MainGameManager.OnDead)
                {
                    if (logDropoutInvestigation)
                        Debug.LogWarning($"[DROP AUDIT DEATH] t={Time.fixedTime:F3} " +
                            $"originDistance={lastHitOriginDistance:F3} lastHit={(currentHit.collider ? currentHit.collider.name : "-")} " +
                            $"pos={rb.position:F3} velocity={rb.velocity:F3}", this);
                    // Recover/OnDead変更より前の状態を必ず残す。
                    LogDropoutDistanceDiagnostic("TRIGGER", grounded, hit, lastHitOriginDistance);
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
        // A straight flat with no pending turn MUST NOT continue exerting
        // Spline Stick/Drive while unsupported: preserve rb.velocity and
        // let Unity gravity carry the ball naturally off its edge.
        bool grace = !grounded && !ordinaryStraightFlat &&
            graceTimer > 0f && load <= 1f && CanGrace(guide);

        if (grounded || !ordinaryStraightFlat)
            straightFlatFallLogged = false;
        if (!grounded && !grace)
        {
            if (ordinaryStraightFlat && !straightFlatFallLogged)
            {
                straightFlatFallLogged = true;
                Debug.Log($"[STRAIGHT FLAT FREEFALL] t={Time.fixedTime:F3} " +
                    $"velocity preserved={rb.velocity:F3}; no Stick/Drive/Grace", this);
            }
            ResetUncapturedTurnGuideStallWatch();
            // Safety must execute even if the normal spline controller returns.
            if (waitingForTurnGuide)
                GuardUncapturedTurnGuide(guide, false);
            if (logCore)
                ;

            LoseSupport();
            return;
        }
        
        currentSupported = true;
        
        Surface surface = BuildSplineSurface(guide);

        if (!surface.Valid)
        {
            ResetGeneralStallWatch();
            stallPreviousVelocityValid = false;
            currentSurfaceValid = false;
            PauseExtremePointTracking();
            return;
        }

        currentSurface = surface;
        currentSurfaceValid = true;
        // B-release handoff owns only its original, actually supported flat.
        // Ordinary route, other input and fall detection remain untouched.
        if (StepPendingBReleaseHandoff())
            return;
        // 平面上の前進速度・横速度を調べる
        if (!guide.isSlope && Time.fixedTime >= nextFlatMotionLogTime)
        {
            nextFlatMotionLogTime = Time.fixedTime + 0.1f;

            float forwardSpeed =
                Vector3.Dot(rb.velocity, surface.tangent);

            float lateralSpeed =
                Vector3.Dot(rb.velocity, surface.side);

            float lateralOffset =
                Vector3.Dot(rb.position - guide.point, surface.side);

            Debug.Log(
                $"[FLAT MOTION] " +
                $"forward={forwardSpeed:F2}m/s " +
                $"lateral={lateralSpeed:F2}m/s " +
                $"offset={lateralOffset:F2}m " +
                $"pos={rb.position:F3} " +
                $"velocity={rb.velocity:F3}"
            );
        }
        // Slopeへの切替そのものもSpline判定。
        if (guide.isSlope &&BallVisualIsOnSlope)
        {
            
            if (sumTime >SubtleLimitime)
            {
                SubtleLimitime = sumTime + bitInTime;
            }

        }
        if (guide.isSlope && !wasSlope)
        {
            AcceleFirstTurn = true;
            StickSlopePharse = false;
            TrySwitchStageCollidersOnSlopeEntry(hit);
            // Ordinary straight continuation needs no artificial Drive reset
            // and no normal-velocity projection. Keep the incoming momentum.
            // A real 90-degree turn or non-straight transition retains its
            // existing controlled transport path.
            if (!IsUncommandedStraightSlopeEntry(previousGuide, guide))
            {
                driveState = 0f;
                TransportToSpline(guide);
            }
        }
        if (wasSlope && !guide.isSlope)
        {
            StickSlopePharse = true;
        }
      

        

        // Collider法線は使わない。
        // tangent / normal は必ずSpline Guideから構築。

        if (!surface.Valid)
        {
            ResetGeneralStallWatch();
            stallPreviousVelocityValid = false;
            currentSurfaceValid = false;
            PauseExtremePointTracking();
            return;
        }

        currentSurface = surface;
        currentSurfaceValid = true;
        // A single observer for BOTH stair/flat stalls, after actual contact
        // and guide have been verified, before the ordinary turn-guide guard
        // and Drive channels take control. No change if velocity is healthy.
        // The normal observer rejects perpendicular old guides on purpose.
        // A post-turn orphan gets an independent, target-heading-only route.
        if (waitingForTurnGuide && ObserveUncapturedTurnGuideStall(guide, grounded))
            return;
        if (ObserveGeneralStallRecovery(guide, surface, grounded, hit))
            return;
        // ================================================================
// 旋回後：旋回後Splineへ切り替わるまで待つ
// ================================================================
        if (waitingForTurnGuide)
        {
            // 前のSplineの接線とたまたま平行な遠距離Guideを採用しない。
            // 接地 + 方向 + 距離の3条件を満たして初めてDriveを解禁する。
            float alignment = TurnGuideAxisAlignment(guide);
            bool captured = IsTurnReceivingGuideValid(guide, grounded, hit);
            if (captured && postBDownhillArmed && guide.isSlope &&
                !TryResolvePostBDownhill(guide, surface, hit, out _))
            {
                captured = false;
                if (Time.fixedTime >= postBDownhillNextLogTime)
                {
                    postBDownhillNextLogTime = Time.fixedTime + .25f;
                    Debug.LogWarning($"[B DOWNHILL GUIDE DEFERRED] " +
                        $"t={Time.fixedTime:F3} section={guide.sectionIndex} " +
                        $"distance={guide.distanceToGuide:F2} " +
                        $"floor={(hit.collider ? hit.collider.name : "-")} " +
                        $"heading={postBDownhillCommittedHeading:F3}", this);
                }
            }

            if (!captured)
            {
                PauseExtremePointTracking();
                driveState = 0f;
                GuardUncapturedTurnGuide(guide, grounded);

                // The guide guard has already applied bounded support and
                // roll-consistent spin damping on the REAL floor. No second
                // full-strength spline Stick: it can push X/Z backwards.
                return;
            }

            waitingForTurnGuide = false;
            turnGuideCapturedAt = Time.fixedTime;
            stickState = Mathf.Min(stickState, Mathf.Max(0f, flatStick));
            ResetUncapturedTurnGuideStallWatch();
            turnGuideReleasedAt = -1f;
            turnGuideGuardBraking = false;
            turnGuideCrawlLogged = false;
            if (postAutoHoldEdgeActive)
            {
                postAutoHoldNewGuide = true;
                postAutoHoldNextProbeTime = 0f;
            }
            LogDropAuditTurn("NEW-GUIDE-CAPTURED", rb.velocity);
            // Preserve the bounded forward Drive scalar across the confirmed
            // handoff. Old wrong-spline lateral corrections remain inactive.
            if (guide.isSlope)
                driveState = Mathf.Clamp(driveState, 0f, GroundAcceleration);
            Debug.Log($"[TURN GUIDE CAPTURE HANDOFF] t={Time.fixedTime:F3} " +
                $"forward={Vector3.Dot(rb.velocity, NormalizeFlat(turnTargetDirection, direction)):F2} " +
                $"drive={driveState:F1} stick={stickState:F1} " +
                $"guideDist={guide.distanceToGuide:F2} " +
                $"hit={(hit.collider ? hit.collider.name : "-")} " +
                $"dot={alignment:F2}", this);

            if (activeTurnMode == TurnResolutionMode.UTurnBeforeEnergyTarget)
            {
                capturedTurnLandingIntentValid = false;
                capturedTurnLandingIntent = default;
            }
        }

        // Only the B-release InCorner's first supported descending section
        // may replace a backwards nearest-Spline tangent or receive an
        // anti-uphill correction. All other surfaces use their original basis.
        bool postBDownhillValid = TryStepPostBDownhill(
            guide, grounded, hit, ref surface, out Vector3 postBDownhill);
        if (postBDownhillValid)
        {
            currentSurface = surface;
            currentSurfaceValid = true;
        }

        // 物理的に安定したFlatの最初のGuideからEntry基準を1回だけ保存する。
        // Turn中は上部の早期returnで更新しない。旧Flatと新Flatを混ぜない。
        UpdateFlatEntryFrame(guide);

        // FiveLineモードではVisual回転とBallVisual POP/Rejoinが終わってから、
        // 下階段側の5ラインへ短い横補正を開始する。
        TryBeginPendingFiveLineCorrection(guide, ref surface);

        if (fiveLineCorrectionActive)
        {
            PauseExtremePointTracking();
            StepFiveLineCorrection(ref surface);

            stickState = Move(stickState, flatStick, StickJerk);
            rb.AddForce(-surface.normal * stickState, ForceMode.Acceleration);
            dropAuditStickAppliedLastTick = stickState;
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

            // 最新の物理Spline進行度で -0 -> +0 の境界を検知する。
            ObserveExtremePoint(guide);
            ObserveTapEdgeGuide(guide, grounded);

            float release = guide.isSlope
                ? 1f - SmoothRange01(guide.sectionProgress01, ReleaseStart, ReleaseEnd)
                : 1f;

            float desiredDrive = DesiredDrive(surface, guide, grace) * release;
            driveState = Move(driveState, desiredDrive, AccelerationJerk);

            float curvature = SplineCurvature(guide);

            float desiredStick = DesiredStick(surface, guide, driveState, curvature, grace) * release;
            stickState = Move(stickState, desiredStick, StickJerk);
            bool gentleNewStair = waitingForTurnGuide == false &&
                guide.isSlope && grounded && turnGuideCapturedAt >= 0f &&
                Time.fixedTime - turnGuideCapturedAt < TurnGuideLandingSeconds &&
                Vector3.Dot(rb.velocity,
                    NormalizeFlat(turnTargetDirection, direction)) < 6f;
            if (gentleNewStair)
                stickState = Mathf.Min(stickState, TurnGuideLandingStickLimit);

            // A completed approach hands off to a gentle, separately verified
            // edge-cruise controller. Only the same existing side-force channel
            // is used; gravity, forward Drive and ordinary fall detection stay.
            float lateralAcceleration = StepEarlyFlick(guide, surface, grounded) +
                StepAutoHoldInnerSteering(surface, grounded) +
                StepPostAutoHoldLeftEdge(guide, surface, grounded) +
                StepEdgeCruise(guide, surface, grounded, hit);

            // DriveはSpline接線、微調整はSideのみ、Stickは法線方向の既存値。
            // 上から押さえつける計算を弱めたりキャンセルする処理は追加しない。
            if (postBDownhillValid &&
                Vector3.Dot(rb.velocity, postBDownhill) < 4f)
                stickState = Mathf.Min(stickState,
                    PostBDownhillStickLowSpeedCap);

            acceleration =
                surface.tangent * driveState
                + surface.side * lateralAcceleration
                - surface.normal * stickState;

            if (gentleNewStair &&
                TryAutoHoldGroundRay(rb.worldCenterOfMass, out RaycastHit newFloor) &&
                IsCloseGroundRay(newFloor, rb.worldCenterOfMass))
            {
                // On a 45-degree tread, even moderate Stick has a backwards
                // XZ component. Offset only that net backwards component;
                // preserve the vertical support and ordinary steering.
                Vector3 forward = NormalizeFlat(turnTargetDirection, direction);
                float projected = Vector3.Dot(acceleration, forward);
                float desiredForwardAcceleration =
                    Vector3.Dot(rb.velocity, forward) < 2.5f ? 16f : 2f;
                float compensate = Mathf.Clamp(
                    desiredForwardAcceleration - projected, 0f,
                    TurnGuideForwardAssistLimit);
                acceleration += forward * compensate;
                GuideWaitBalanceSpin(true, newFloor.normal);
            }

            if (postBDownhillValid)
                ApplyPostBDownhillProtection(postBDownhill, ref acceleration);

            rb.AddForce(acceleration, ForceMode.Acceleration);
            dropAuditStickAppliedLastTick = stickState;

            wasSlope = guide.isSlope;

            if (logCore)
            {
                ;
            }
        }
        else
        {
            ClearEarlyFlick();
            PauseExtremePointTracking();
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

    // Used only if the original SphereCast misses (or hits a steep side).
    // This is NOT an invisible supporting wall: independent stage-surface rays
    // must agree on one collider near the ball's actual contact footprint.
    bool TryGetVerifiedGroundRayFallback(out RaycastHit verified)
    {
        verified = default;
        Vector3 centre = rb.worldCenterOfMass;
        if (!TryAutoHoldGroundRay(centre, out RaycastHit middle) ||
            !IsCloseGroundRay(middle, centre))
            return false;

        Vector3 ahead = NormalizeFlat(
            waitingForTurnGuide ? turnTargetDirection : direction,
            travelDirection);
        Vector3 side = Vector3.Cross(Vector3.up, ahead).normalized;
        float d = ProbeRadius * .8f;
        int confirmations = 1;

        for (int i = 0; i < 4; ++i)
        {
            Vector3 offset = i == 0 ? side * d :
                             i == 1 ? -side * d :
                             i == 2 ? ahead * d : -ahead * d;
            Vector3 sample = centre + offset;
            if (TryAutoHoldGroundRay(sample, out RaycastHit candidate) &&
                candidate.collider == middle.collider &&
                IsCloseGroundRay(candidate, sample))
                ++confirmations;
        }

        if (confirmations < 3)
            return false;

        verified = middle;
        if (logDropoutInvestigation)
            Debug.Log($"[DROP SUPPORT RAY-RECOVER] t={Time.fixedTime:F3} " +
                      $"collider={verified.collider.name} rays={confirmations}/5 " +
                      $"distance={verified.distance:F3}", this);
        return true;
    }

    bool IsCloseGroundRay(RaycastHit hit, Vector3 sample)
    {
        if (!IsAutoHoldStageHit(hit, float.NaN))
            return false;
        // The contact must be near the sphere, not a distant slab below it.
        float normalSeparation = Vector3.Dot(sample - hit.point, hit.normal);
        return normalSeparation >= -0.10f &&
               normalSeparation <= ProbeRadius + .30f;
    }

    // Records physical support, not just the closest Spline. The short-lived
    // witness is used ONLY when a new press happens at a flat-to-stair seam.
    void RememberAutoHoldRecentPhysicalFlat(
        NearestKnotDetector.GuideFrame guide, bool grounded, RaycastHit hit)
    {
        if (!grounded || !guide.valid || guide.isSlope ||
            !IsCloseGroundRay(hit, rb.worldCenterOfMass) ||
            Vector3.Angle(hit.normal, Vector3.up) > 18f ||
            Mathf.Abs(hit.point.y - guide.point.y) > .35f ||
            guide.distanceToGuide > Mathf.Max(ProbeRadius * 2f, 1f))
            return;

        autoHoldRecentPressFlatCollider = hit.collider;
        autoHoldRecentPressFlatTime = Time.time;
        autoHoldRecentPressFlatPosition = rb.position;
        autoHoldRecentPressFlatHeading = NormalizeFlat(guide.tangent, direction);
    }

    (bool, RaycastHit) HasGroundSupport()
    {
        Vector3 origin = rb.worldCenterOfMass + Vector3.up * .05f;
        bool sphereFound = Physics.SphereCast(
            origin, ProbeRadius, Vector3.down, out RaycastHit sphereHit,
            probeDistance, groundMask, QueryTriggerInteraction.Ignore);

        if (sphereFound &&
            Vector3.Angle(sphereHit.normal, Vector3.up) <= maxSlopeAngle)
            return (true, sphereHit);

        if (TryGetVerifiedGroundRayFallback(out RaycastHit fallback))
            return (true, fallback);

        return (false, sphereFound ? sphereHit : default);
    }

    // ================================================================
    // Turn guide reacquisition guard (separate from ordinary Spline Drive)
    // ================================================================

    float TurnGuideAxisAlignment(NearestKnotDetector.GuideFrame guide)
    {
        if (!guide.valid)
            return 0f;
        Vector3 tangent = Vector3.ProjectOnPlane(guide.tangent, Vector3.up);
        Vector3 heading = Vector3.ProjectOnPlane(turnTargetDirection, Vector3.up);
        if (tangent.sqrMagnitude <= Eps || heading.sqrMagnitude <= Eps)
            return 0f;
        return Mathf.Abs(Vector3.Dot(tangent.normalized, heading.normalized));
    }

    bool TryGetTurnForwardFloor(Vector3 heading, out RaycastHit support)
    {
        support = default;
        if (!rb) return false;
        Vector3 centre = rb.worldCenterOfMass;
        // The turn runs inside FixedUpdate; keep this forward fan allocation-free.
        for (int i = 0; i < 4; ++i)
        {
            float d = i == 0 ? .28f : i == 1 ? .50f :
                      i == 2 ? .80f : 1.10f;
            Vector3 point = centre + heading * d;
            if (!TryAutoHoldGroundRay(point, out RaycastHit floor) ||
                !IsAutoHoldStageHit(floor, float.NaN) ||
                Vector3.Angle(floor.normal, Vector3.up) > maxSlopeAngle)
                continue;
            // A downhill tread can be below the centre even while reachable.
            // The ray still has a strict range (probeDistance + radius + .25).
            float below = Vector3.Dot(point - floor.point, Vector3.up);
            if (below < -.12f || below > ProbeRadius + 1.10f)
                continue;
            support = floor;
            return true;
        }
        return false;
    }

    // The detector's knot direction may be reversed. A valid receiving
    // guide must still be physically close, forward of the body, and share
    // the floor's actual normal. An axial cosine alone is insufficient.
    bool IsTurnReceivingGuideValid(
        NearestKnotDetector.GuideFrame guide,
        bool grounded, RaycastHit floor)
    {
        if (!waitingForTurnGuide || !guide.valid || !grounded ||
            !floor.collider || !IsCloseGroundRay(floor, rb.worldCenterOfMass) ||
            TurnGuideAxisAlignment(guide) < TurnGuideAlignmentMin ||
            guide.distanceToGuide > Mathf.Max(
                supportGraceMaxGuideDistance, ProbeRadius * 2f))
            return false;
        Vector3 forward = NormalizeFlat(turnTargetDirection, direction);
        Vector3 delta = Vector3.ProjectOnPlane(
            guide.point - rb.position, Vector3.up);
        if (Vector3.Dot(delta, forward) < -ProbeRadius * 2.5f)
            return false;
        if (guide.normal.sqrMagnitude > Eps &&
            Vector3.Dot(floor.normal.normalized, guide.normal.normalized) < .35f)
            return false;
        return true;
    }

    // This state is a *physical route witness*, never an alternate steering
    // mode. It cannot outlive the first receiving downhill stair.
    void ClearPostBDownhillGuard(string reason)
    {
        if (postBDownhillArmed && (logTurnTransition || logDropoutInvestigation))
            Debug.Log($"[B DOWNHILL END] t={Time.fixedTime:F3} reason={reason} " +
                $"section={postBDownhillSectionIndex}", this);
        postBDownhillArmed = false;
        postBDownhillSawSlope = false;
        postBDownhillWrongGuideLogged = false;
        postBDownhillSectionIndex = -1;
        postBDownhillArmedAt = -1f;
        postBDownhillLastSupportTime = -1f;
        postBDownhillRecentNearSlopeTime = -1f;
        postBDownhillCommittedHeading = Vector3.zero;
        postBDownhillStairFamily = -1;
        postBDownhillLastLoggedSection = -1;
        postBEdgeAnchorValid = false;
        postBEdgeAnchorLateral = 0f;
        postBEdgeReacquireUntil = -1f;
        postBEdgeNextLogTime = 0f;
    }

    // StairWay5_0_Physics -> family 5, StairWay5_4_Physics -> family 5.
    // Different detector sectionIndex values are permitted ONLY on this
    // physically shared stair family. Unknown colliders cannot grant it.
    static bool TryGetPostBStairFamily(Collider floor, out int family)
    {
        family = -1;
        if (!floor) return false;
        string name = floor.name;
        const string prefix = "StairWay";
        if (!name.StartsWith(prefix, System.StringComparison.Ordinal))
            return false;
        int index = prefix.Length;
        if (index >= name.Length || name[index] < '0' || name[index] > '9')
            return false;
        int value = 0;
        while (index < name.Length && name[index] >= '0' &&
            name[index] <= '9')
        {
            int digit = name[index] - '0';
            if (value > 1000000) return false;
            value = value * 10 + digit;
            ++index;
        }
        if (index >= name.Length || name[index] != '_') return false;
        family = value;
        return true;
    }

    // Downhill sign comes from Spline+gravity+the committed turn heading.
    // Geometry alone is a SHORT guide-gap witness, never permission to push
    // an unsupported sphere or to call a wrong Spline physically captured.
    bool TryResolvePostBDownhillGeometry(
        NearestKnotDetector.GuideFrame guide, out Vector3 downhill)
    {
        downhill = Vector3.zero;
        if (!postBDownhillArmed || !guide.valid || !guide.isSlope ||
            guide.normal.sqrMagnitude < Eps)
            return false;
        Vector3 normal = guide.normal.normalized;
        if (Vector3.Angle(normal, Vector3.up) < 9f)
            return false;
        Vector3 gravityDown = Vector3.ProjectOnPlane(Physics.gravity, normal);
        Vector3 splineAxis = Vector3.ProjectOnPlane(guide.tangent, normal);
        if (gravityDown.sqrMagnitude < Eps || splineAxis.sqrMagnitude < Eps)
            return false;
        gravityDown.Normalize();
        splineAxis.Normalize();
        float downhillSign = Vector3.Dot(splineAxis, gravityDown);
        if (Mathf.Abs(downhillSign) < .62f)
            return false;
        Vector3 candidate = downhillSign >= 0f ? splineAxis : -splineAxis;
        Vector3 horizontal = Vector3.ProjectOnPlane(candidate, Vector3.up);
        if (horizontal.sqrMagnitude < Eps)
            return false;
        horizontal.Normalize();
        float headingDot = Vector3.Dot(horizontal,
            postBDownhillCommittedHeading);
        if (headingDot < .74f)
        {
            if (!postBDownhillWrongGuideLogged)
            {
                postBDownhillWrongGuideLogged = true;
                Debug.LogWarning($"[B DOWNHILL WRONG GUIDE] " +
                    $"t={Time.fixedTime:F3} headingDot={headingDot:F3} " +
                    $"slope={guide.slopeAngle:F1} " +
                    $"guideSection={guide.sectionIndex} " +
                    $"committed={postBDownhillCommittedHeading:F3} " +
                    $"computed={candidate:F3}; no reversal", this);
            }
            return false;
        }
        downhill = candidate;
        return true;
    }

    // Three independent physical witnesses before changing movement:
    // signed downhill geometry, real floor contact, and matching floor normal.
    bool TryResolvePostBDownhill(
        NearestKnotDetector.GuideFrame guide, Surface surface,
        RaycastHit floor, out Vector3 downhill)
    {
        downhill = Vector3.zero;
        if (!postBDownhillArmed || !surface.Valid || !rb ||
            !floor.collider || !IsCloseGroundRay(floor, rb.worldCenterOfMass) ||
            guide.distanceToGuide > Mathf.Max(3.0f,
                supportGraceMaxGuideDistance + .8f) ||
            Vector3.Dot(surface.normal,
                floor.normal.normalized) < .40f)
            return false;
        return TryResolvePostBDownhillGeometry(guide, out downhill);
    }

    // Armed on a real B release, then activated only by first supported slope.
    // A valid flat on its far end retires the special control permanently.
    bool TryStepPostBDownhill(
        NearestKnotDetector.GuideFrame guide, bool grounded,
        RaycastHit floor, ref Surface surface, out Vector3 downhill)
    {
        downhill = Vector3.zero;
        if (!postBDownhillArmed || turnTransitionActive || waitingForTurnGuide)
            return false;
        // The four-second timeout is only for ACQUIRING the first slope.
        // Once physically captured, remain protected through its actual end.
        if (!postBDownhillSawSlope &&
            Time.fixedTime - postBDownhillArmedAt > PostBDownhillMaxWindow)
        {
            ClearPostBDownhillGuard("first downhill not reached in time");
            return false;
        }
        if (!grounded || !floor.collider)
            return false;

        bool physicallyClose = IsCloseGroundRay(floor, rb.worldCenterOfMass);
        // Confirm the receiving FLAT physically; the detector may still
        // report the previous slope for one or more seam frames.
        if (postBDownhillSawSlope && physicallyClose &&
            Vector3.Angle(floor.normal, Vector3.up) <= 18f &&
            floor.collider.name.StartsWith("ArcSlab",
                System.StringComparison.Ordinal))
        {
            ClearPostBDownhillGuard("actual next flat / downhill complete");
            return false;
        }
        if (!guide.valid || !guide.isSlope || !physicallyClose)
            return false;

        bool knownPhysicalFamily =
            TryGetPostBStairFamily(floor.collider, out int currentFamily);
        if (postBDownhillSawSlope)
        {
            // Do not cross into a DIFFERENT stair merely because the
            // nearest Spline remains parallel. An unknown physical family
            // cannot authorize an across-section continuation either.
            // An unknown collider at a joint cannot authorize a force,
            // but it should not permanently erase the verified stair route.
            if (!knownPhysicalFamily)
                return false;
            if (postBDownhillStairFamily < 0 ||
                currentFamily != postBDownhillStairFamily)
            {
                ClearPostBDownhillGuard("different physical stair family");
                return false;
            }
        }
        // Same stair family is NOT sufficient on its own: the normal,
        // nearby guide and gravity-confirmed downhill axis must still pass.
        if (!TryResolvePostBDownhill(guide, surface, floor, out downhill))
            return false;
        if (postBDownhillSawSlope &&
            postBDownhillLastLoggedSection != guide.sectionIndex)
        {
            postBDownhillLastLoggedSection = guide.sectionIndex;
            Debug.Log($"[B DOWNHILL SAME STAIR SECTION] " +
                $"t={Time.fixedTime:F3} first={postBDownhillSectionIndex} " +
                $"now={guide.sectionIndex} family={currentFamily} " +
                $"floor={floor.collider.name}", this);
        }
        if (!postBDownhillSawSlope)
        {
            if (!knownPhysicalFamily)
                return false; // Unknown geometry must not acquire protection.
            postBDownhillSawSlope = true;
            postBDownhillSectionIndex = guide.sectionIndex;
            postBDownhillStairFamily = currentFamily;
            postBDownhillLastLoggedSection = guide.sectionIndex;
            Debug.Log($"[B DOWNHILL VERIFIED] t={Time.fixedTime:F3} " +
                $"section={guide.sectionIndex} family={currentFamily} " +
                $"downhill={downhill:F3} " +
                $"heading={postBDownhillCommittedHeading:F3} " +
                $"floor={floor.collider.name}", this);
        }
        postBDownhillLastSupportTime = Time.fixedTime;

        // BuildSplineSurface normally follows the horizontal player heading.
        // Only in this physically proven B-handoff may a reversed spline
        // tangent be corrected to the gravity-confirmed descending axis.
        if (Vector3.Dot(surface.tangent, downhill) < 0f)
        {
            surface.tangent = downhill;
            surface.side = Vector3.Cross(surface.normal, downhill).normalized;
            surface.tangentSpeed = Vector3.Dot(rb.velocity, downhill);
            surface.lateralSpeed = Vector3.Dot(rb.velocity, surface.side);
            surface.gravityAlong = Vector3.Dot(Physics.gravity, downhill);
            Debug.LogWarning($"[B DOWNHILL TANGENT FLIPPED] " +
                $"t={Time.fixedTime:F3} section={guide.sectionIndex}", this);
        }
        return true;
    }

    // Velocity sign protection is deliberately restricted to the verified
    // descending tangent. Keep sideways motion, gravity-normal velocity and
    // all steering inputs as they were. No teleport / generic spline snap.
    void ApplyPostBDownhillProtection(Vector3 downhill,
        ref Vector3 totalAcceleration)
    {
        if (!postBDownhillArmed || !postBDownhillSawSlope || !rb ||
            rb.isKinematic) return;

        float before = Vector3.Dot(rb.velocity, downhill);
        if (before < -.10f)
        {
            // Once the NEXT real downhill stair is confirmed, a negative
            // along-stair component is a definite uphill/backwards motion.
            // Replace only that signed component with a tiny downhill crawl;
            // keep lateral and surface-normal velocity exactly as measured.
            rb.velocity += downhill * (.35f - before);
        }
        float now = Vector3.Dot(rb.velocity, downhill);
        float projectedAcceleration = Vector3.Dot(
            totalAcceleration + Physics.gravity, downhill);
        float requested = (PostBDownhillMinimumForwardSpeed - now) / .16f;
        float assist = Mathf.Clamp(requested - projectedAcceleration,
            0f, PostBDownhillMaxAssist);
        if (assist > .01f)
            totalAcceleration += downhill * assist;

        if (before < -.10f || (assist > .01f &&
            Time.fixedTime >= postBDownhillNextLogTime))
        {
            postBDownhillNextLogTime = Time.fixedTime + .12f;
            Debug.Log($"[B DOWNHILL FORWARD RECOVERY] " +
                $"t={Time.fixedTime:F3} before={before:F2} after={now:F2} " +
                $"assist={assist:F1} slope={downhill:F3} " +
                $"heading={postBDownhillCommittedHeading:F3} " +
                $"stick={stickState:F1}", this);
        }
    }

    // Guide waiting is an explicitly authorized 90-degree turn handoff:
    // decelerate safely, ALIGN horizontal velocity to the committed heading,
    // preserve a small forward crawl when supported, and damp excess angular
    // spin relative to rolling. Never run this on an uncommanded flat.
    void GuardUncapturedTurnGuide(
        NearestKnotDetector.GuideFrame guide, bool grounded)
    {
        if (!waitingForTurnGuide || !rb || rb.isKinematic ||
            !BeginCommandOnTouch || MainGameManager.OnDead)
            return;

        Vector3 heading = NormalizeFlat(turnTargetDirection, travelDirection);
        Vector3 planar = Vector3.ProjectOnPlane(rb.velocity, Vector3.up);
        float forward = Vector3.Dot(planar, heading);
        float sinceRelease = turnGuideReleasedAt >= 0f
            ? Mathf.Max(0f, Time.fixedTime - turnGuideReleasedAt) : 0f;
        float axisAlignment = TurnGuideAxisAlignment(guide);
        bool matchingGuide = guide.valid &&
            axisAlignment >= TurnGuideAlignmentMin &&
            guide.distanceToGuide <= Mathf.Max(
                supportGraceMaxGuideDistance, ProbeRadius * 2f);

        RaycastHit floorHit = default;
        bool closeFloor = grounded &&
            TryAutoHoldGroundRay(rb.worldCenterOfMass, out floorHit) &&
            IsCloseGroundRay(floorHit, rb.worldCenterOfMass);
        Vector3 floorNormal = closeFloor ? floorHit.normal : Vector3.up;
        Vector3 receivingSample = rb.worldCenterOfMass +
            heading * (ProbeRadius * 1.5f);
        RaycastHit aheadHit = default;
        bool floorAhead = closeFloor &&
            TryAutoHoldGroundRay(receivingSample, out aheadHit) &&
            IsCloseGroundRay(aheadHit, receivingSample);

        bool tooFar = guide.valid && !matchingGuide &&
            guide.distanceToGuide > supportGraceMaxGuideDistance;
        bool timedOut = sinceRelease >= TurnGuideCoastAllowanceSeconds;
        // The forward floor fan is independent from the flat centre ray.
        // At a descending lip the first ray often sees air even when the
        // next stair's tread is physically within the guide-search region.
        bool forwardFloor = floorAhead ||
            TryGetTurnForwardFloor(heading, out _);
        bool nearReceivingSlope = guide.valid && guide.isSlope &&
            axisAlignment >= TurnGuideAlignmentMin &&
            guide.distanceToGuide <= Mathf.Max(
                supportGraceMaxGuideDistance, ProbeRadius * 2f) &&
            Vector3.Dot(Vector3.ProjectOnPlane(
                guide.point - rb.position, Vector3.up), heading) >=
                -ProbeRadius * 2.5f;
        // B-InCorner only: a valid signed DOWNHILL guide seen within the last
        // .16s is not immediately discarded if a single frame loses it.
        // No force or fake ground is created while airborne; just a bounded
        // forward speed target along the already committed heading.
        bool briefDownhillGuideGap = false;
        if (postBDownhillArmed && !postBDownhillSawSlope)
        {
            if (nearReceivingSlope &&
                TryResolvePostBDownhillGeometry(guide, out _))
                postBDownhillRecentNearSlopeTime = Time.fixedTime;
            else
                nearReceivingSlope = false;
            briefDownhillGuideGap = postBDownhillRecentNearSlopeTime >= 0f &&
                Time.fixedTime - postBDownhillRecentNearSlopeTime <= .16f &&
                !tooFar;
        }
        bool missingFloorAhead = closeFloor && !forwardFloor;
        bool shouldBrake =
            (!closeFloor && sinceRelease >= .08f) ||
            tooFar || missingFloorAhead || (!matchingGuide && timedOut);

        // Zero is reserved for an unsupported, unverified direction. A real
        // forward floor or a nearby correctly aligned descending guide keeps
        // a small forward crawl; sideways sliding always decays separately.
        float safeForward = forwardFloor ? TurnGuideSafeCrawlSpeed :
            (nearReceivingSlope || briefDownhillGuideGap)
                ? TurnGuideNearSlopeSafeSpeed :
            closeFloor ? TurnGuideMinimumSupportedSpeed : 0f;
        float forwardAfter = shouldBrake
            ? Mathf.MoveTowards(forward, safeForward,
                TurnGuideBrakeAcceleration * Time.fixedDeltaTime)
            : Mathf.Max(0f, forward);
        Vector3 sideVelocity = planar - heading * forward;
        Vector3 remainingSide = Vector3.MoveTowards(
            sideVelocity, Vector3.zero,
            (shouldBrake ? BReleaseHandoffSideAcceleration : GroundAcceleration) *
            Time.fixedDeltaTime);
        Vector3 corrected = heading * Mathf.Max(0f, forwardAfter) + remainingSide;

        if ((corrected - planar).sqrMagnitude > Eps)
            rb.velocity = corrected + Vector3.up * rb.velocity.y;

        // Stick alone cannot stop a sphere's spin: damping uses a real torque
        // relative to rolling velocity, NOT an invented zero angularVelocity.
        GuideWaitBalanceSpin(closeFloor, floorNormal);
        if (closeFloor)
        {
            // Stale preturn Stick can be hundreds of m/s². Cap it until the
            // new guide is captured; compensate its reverse XZ projection.
            stickState = Mathf.Min(
                Move(Mathf.Max(0f, stickState), flatStick, StickJerk),
                Mathf.Max(0f, flatStick));
            Vector3 support = -floorNormal.normalized * Mathf.Min(
                Mathf.Max(0f, stickState), Mathf.Max(0f, flatStick));
            float reverseHorizontal = Mathf.Max(0f,
                -Vector3.Dot(support, heading));
            rb.AddForce(support + heading * reverseHorizontal,
                ForceMode.Acceleration);
            dropAuditStickAppliedLastTick = support.magnitude;
        }

        if (Time.fixedTime >= turnHandoffNextAuditTime)
        {
            turnHandoffNextAuditTime = Time.fixedTime + .10f;
            Debug.Log($"[TURN GUIDE HANDOFF STEP] t={Time.fixedTime:F3} " +
                $"forwardBefore={forward:F2} forwardAfter={Vector3.Dot(corrected, heading):F2} " +
                $"side={sideVelocity.magnitude:F2}->{remainingSide.magnitude:F2} " +
                $"target={safeForward:F2} brake={shouldBrake} " +
                $"floor={closeFloor} ahead={forwardFloor} " +
                $"nearSlope={nearReceivingSlope} recentGap={briefDownhillGuideGap} " +
                $"alignment={axisAlignment:F2} " +
                $"stick={stickState:F1} omega={rb.angularVelocity.magnitude:F2}", this);
        }
        if (shouldBrake && !turnGuideGuardBraking)
        {
            turnGuideGuardBraking = true;
            Debug.LogWarning($"[TURN GUIDE BALANCED BRAKE] t={Time.fixedTime:F3} " +
                $"afterRelease={sinceRelease:F3}s target={heading:F3} " +
                $"beforeForward={forward:F3} afterForward={Vector3.Dot(corrected, heading):F3} " +
                $"beforeSide={(planar - heading * forward).magnitude:F3} " +
                $"omega={rb.angularVelocity.magnitude:F3} " +
                $"stick={stickState:F1} closeFloor={closeFloor} floorAhead={floorAhead} " +
                $"alignment={axisAlignment:F3}", this);
        }
        if (shouldBrake && floorAhead &&
            Vector3.Dot(corrected, heading) <= TurnGuideSafeCrawlSpeed + .25f &&
            !turnGuideCrawlLogged)
        {
            turnGuideCrawlLogged = true;
            Debug.Log($"[TURN GUIDE SAFE-CRAWL] t={Time.fixedTime:F3} " +
                $"forward={Vector3.Dot(corrected, heading):F2} " +
                $"floor={aheadHit.collider.name}", this);
        }
    }

    void GuideWaitBalanceSpin(bool floorSupported, Vector3 normal)
    {
        if (!rb || rb.isKinematic) return;
        Vector3 n = normal.sqrMagnitude > Eps ? normal.normalized : Vector3.up;
        Vector3 tangentVelocity = Vector3.ProjectOnPlane(rb.velocity, n);
        Vector3 rollingAngular = floorSupported
            ? Vector3.Cross(n, tangentVelocity) / Mathf.Max(.05f, ProbeRadius)
            : Vector3.zero;
        rollingAngular = Vector3.ClampMagnitude(rollingAngular,
            Mathf.Max(0f, rb.maxAngularVelocity));
        Vector3 angularError = rollingAngular - rb.angularVelocity;
        Vector3 angularAcceleration = Vector3.ClampMagnitude(
            angularError * (floorSupported ? TurnGuideSpinResponse : 3f),
            TurnGuideMaxAngularAcceleration);
        rb.AddTorque(angularAcceleration, ForceMode.Acceleration);
    }


    // On the ordinary route the player's confirmed heading is authoritative.
    // NearestKnotDetector may select a perpendicular branch at a junction even
    // when the player NEVER held/tapped/flicked for a turn. That geometric
    // discontinuity is not permission to brake or freeze the Rigidbody.
    // Leave its velocity intact and coast on the real stage. If there is no
    // floor ahead it may naturally fall: do not silently rescue a missed turn.
    // Explicitly accepted turns and the slope-release hold handoff have their
    // own independent, input-authorized stop/escape paths.
    bool GuardUncommandedGuideJump(NearestKnotDetector.GuideFrame candidate)
    {
        if (!rb || rb.isKinematic || !BeginCommandOnTouch ||
            turnTransitionActive || waitingForTurnGuide || HasPendingTurnIntent ||
            !candidate.valid || !currentGuideValid || !currentGuide.valid)
        {
            uncommandedGuideGuardLogged = false;
            return false;
        }

        Vector3 heading = NormalizeFlat(direction, travelDirection);
        Vector3 oldTangent = NormalizeFlat(currentGuide.tangent, heading);
        Vector3 nextTangent = NormalizeFlat(candidate.tangent, heading);
        // Flat projection permits ordinary slope-to-flat handoffs but
        // rejects a genuinely perpendicular branch selection.
        float oldAlignment = Mathf.Abs(Vector3.Dot(heading, oldTangent));
        float newAlignment = Mathf.Abs(Vector3.Dot(heading, nextTangent));
        float segmentAlignment = Mathf.Abs(Vector3.Dot(oldTangent, nextTangent));
        if (oldAlignment < 0.8f ||
            newAlignment >= UncommandedGuideAlignmentMin ||
            segmentAlignment >= UncommandedGuideAlignmentMin)
        {
            uncommandedGuideGuardLogged = false;
            return false;
        }

        // ProbeDistance can be very large in the scene (e.g. 28m).
        // Require a *close* genuine stage surface, not merely a distant cast.
        Vector3 centre = rb.worldCenterOfMass;
        if (!TryAutoHoldGroundRay(centre, out RaycastHit floor) ||
            !IsCloseGroundRay(floor, centre))
        {
            uncommandedGuideGuardLogged = false;
            return false;  // Normal support/dropout pipeline remains authoritative.
        }

        // Important: this is NOT a player-authorized stop. In particular, do
        // not call MoveTowards(velocity, zero) here. Doing so used to remove
        // ~7.6 m/s *per FixedUpdate* (380 m/s2 at 0.02 s), causing the ball
        // to halt on the flat even when no slope hold was ever committed.
        // Returning true still rejects the unrelated 90-degree spline basis
        // for this tick, avoiding false lateral drive into the wrong lane.
        // Rigidbody velocity, angular velocity, and gravity are untouched.
        Vector3 planar = Vector3.ProjectOnPlane(rb.velocity, Vector3.up);
        // Wrong nearest spline does not invalidate ordinary flat drive.
        currentSupported = true;
        if (!uncommandedGuideGuardLogged)
        {
            uncommandedGuideGuardLogged = true;
            Debug.LogWarning($"[SPLINE UNCOMMANDED TURN COAST] " +
                $"t={Time.fixedTime:F3} oldAlign={oldAlignment:F3} " +
                $"newAlign={newAlignment:F3} segmentAlign={segmentAlignment:F3} " +
                $"oldSpline={currentGuide.splineIndex} " +
                $"candidateSpline={candidate.splineIndex} " +
                $"floor={floor.collider.name} preservedSpeed={planar.magnitude:F2} " +
                $"holdArmed={slopeReleaseHoldArmed} " +
                $"holdTurnPending={slopeReleaseHoldTurnPending} " +
                $"gestureActive={trackingFlick}; no automatic brake", this);
        }
        return true;
    }

    // ================================================================
    // Spline basis
    // ================================================================

    Surface BuildSplineSurface(NearestKnotDetector.GuideFrame g)
    {
        Vector3 normal = g.normal.sqrMagnitude > Eps
            ? g.normal.normalized :
            (!g.isSlope ? Vector3.up :
                (currentSurfaceValid && currentSurface.normal.sqrMagnitude > Eps
                    ? currentSurface.normal.normalized : Vector3.up));
        Vector3 tangent = Vector3.ProjectOnPlane(g.tangent, normal);
        if (tangent.sqrMagnitude <= Eps && !g.isSlope && currentSurfaceValid)
            tangent = Vector3.ProjectOnPlane(currentSurface.tangent, normal);

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
    // Ordinary straight flat: preserve freefall and normal momentum.
    // This is based on XZ heading, so a vertical 45-degree slope transition
    // is NOT misclassified as a horizontal 90-degree curve.
    // ================================================================
    bool IsUncommandedStraightFlat(NearestKnotDetector.GuideFrame flat)
    {
        if (!flat.valid || flat.isSlope || turnTransitionActive ||
            waitingForTurnGuide || HasPendingTurnIntent ||
            fiveLineCorrectionActive || autoHoldTurnPending ||
            slopeReleaseHoldTurnPending)
            return false;

        Vector3 ahead = NormalizeFlat(flat.tangent, direction);
        Vector3 commanded = NormalizeFlat(direction, ahead);
        Vector3 after = flat.nextTangent.sqrMagnitude > Eps
            ? NormalizeFlat(flat.nextTangent, ahead) : ahead;
        return Vector3.Dot(ahead, commanded) > .985f &&
               Vector3.Dot(ahead, after) > .985f;
    }

    bool IsUncommandedStraightSlopeEntry(
        NearestKnotDetector.GuideFrame previousFlat,
        NearestKnotDetector.GuideFrame nextSlope)
    {
        if (!IsUncommandedStraightFlat(previousFlat) ||
            !nextSlope.valid || !nextSlope.isSlope)
            return false;
        return Vector3.Dot(
            NormalizeFlat(previousFlat.tangent, direction),
            NormalizeFlat(nextSlope.tangent, direction)) > .985f;
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
        if (postBDownhillArmed && postBDownhillSawSlope &&
            postBDownhillLastSupportTime >= 0f &&
            Time.fixedTime - postBDownhillLastSupportTime > .25f)
            ClearPostBDownhillGuard("physical slope support lost");
        if (postAutoHoldEdgeActive && postAutoHoldNewGuide &&
            postAutoHoldSawSlope)
            FinishPostAutoHoldLeftEdge("support lost", false);
        // Do not apply an invisible force while airborne. A short seam can
        // regain support; the cruise controller re-verifies before any force.
        edgeCruiseSampleValid = false;
        ClearEarlyFlick();
        PauseExtremePointTracking();
        bool lastWasFlat = currentGuideValid && !currentGuide.isSlope;
        wasSlope = false;
        // An uncommanded seam/freefall never zeroes remembered flat
        // acceleration scalars, even though no support force is applied.
        if (!lastWasFlat)
        {
            driveState = 0f;
            stickState = 0f;
        }

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
        ClearPostBDownhillGuard("lifecycle reset");
        autoHoldRecentPressFlatCollider = null;
        autoHoldRecentPressFlatTime = -1f;
        ResetGeneralStallRecovery("restart", true);
        ResetTapEdge();
        ClearPostAutoHoldLeftEdge();
        correspondSubject?.StopExtremePointEffect();
        ResetExtremePointTracking();
        pendingTurnDegrees = 0f;
        pendingTurnQueuedTime = -1f;
        pendingTurnFromPassedSlope = false;
        autoHoldTurnPending = false;
        slopeReleaseHoldTurnPending = false;
        slopeReleaseHoldHandoffHeld = false;
        ResetFlickGesture();
        correspondSubject?.ResetTurnGestureTracking();
        ballVisualSlopeDrive?.CancelTurnHandoffRequest();

        turnTransitionActive = false;
        turnBodyFrozen = false;
        turnTransitionStartTime = -1f;
        activeTurnDegrees = 0f;
        turnTransitionSnapshot = default;

        if (rb && rb.isKinematic)
            rb.isKinematic = false;


        activeTurnMode = TurnResolutionMode.None;
        autoHoldTurnPending = false;
        slopeReleaseHoldTurnPending = false;
        slopeReleaseHoldHandoffHeld = false;
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

        uncommandedGuideGuardLogged = false;
        waitingForTurnGuide = false;
        turnGuideReleasedAt = -1f;
        turnGuideGuardBraking = false;
        turnGuideCrawlLogged = false;

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

    // The gauge owns which touch started on its UI. The core consumes only
    // game-originated gestures; the CorrespondSubject remains responsible for
    // the visual-frame turn itself. No cooldown timer is introduced.
    void ReadTurnFlick()
    {
        // Input.GetTouch works on both Android and iOS. On mobile never also
        // consume the simulated mouse events, which could double-trigger turns.
        if (Application.isMobilePlatform || Input.touchCount > 0)
        {
            ReadTouchTurnFlick();
            return;
        }

        ReadMouseTurnFlick();
    }

    void ReadTouchTurnFlick()
    {
        int currentTouchIndex = -1;
        int newGameTouchIndex = -1;
        bool currentTouchBlocked = false;

        // Every touch must be forwarded to CockpitGauge (via CorrespondSubject),
        // including UI touches and Ended/Canceled phases.
        for (int i = 0; i < Input.touchCount; i++)
        {
            Touch touch = Input.GetTouch(i);
            bool blocked = correspondSubject &&
                           correspondSubject.ShouldIgnoreTurnTouch(touch);

            if (trackingFlick && touch.fingerId == activeFlickFingerId)
            {
                currentTouchIndex = i;
                currentTouchBlocked = blocked;
            }

            if (touch.phase == TouchPhase.Began && !blocked && newGameTouchIndex < 0)
                newGameTouchIndex = i;
        }

        if (trackingFlick)
        {
            if (currentTouchIndex < 0 || currentTouchBlocked)
            {
                // UIに移った操作・消失した操作は旋回に転用しない。
                if (currentTouchIndex >= 0)
                {
                    Touch blockedTouch = Input.GetTouch(currentTouchIndex);
                    if (blockedTouch.phase == TouchPhase.Ended)
                        LogFlickRelease("touch", blockedTouch.position, flickConsumed, true);
                }
                ResetFlickGesture();
            }
            else
            {
                Touch current = Input.GetTouch(currentTouchIndex);
                ObserveAdaptiveFlickPointer(current.position);
                ObserveAutoHoldPointer(current.position);
                // While pressed, collect samples only; no 90-degree commit.
                bool consumedBeforeRelease = flickConsumed;

                // Normal turns still finalize on release; an already accepted
                // same-flat tap turn cannot be queued a second time.
                if (current.phase == TouchPhase.Ended)
                {
                    TryQueueFlickFromPosition(current.position);
                    LogFlickRelease("touch", current.position, consumedBeforeRelease, false);
                }

                if (current.phase == TouchPhase.Ended ||
                    current.phase == TouchPhase.Canceled)
                {
                    ResetFlickGesture();
                }
                else
                {
                    return;
                }
            }
        }

        // A newly started game touch is accepted immediately, including if
        // an older touch ends during the same frame.
        if (!trackingFlick && newGameTouchIndex >= 0)
        {
            Touch next = Input.GetTouch(newGameTouchIndex);
            BeginFlickGesture(next.position, next.fingerId);
        }
    }

    void ReadMouseTurnFlick()
    {
        bool down = Input.GetMouseButtonDown(0);
        bool up = Input.GetMouseButtonUp(0);
        bool blocked = correspondSubject &&
                       correspondSubject.ShouldIgnoreTurnMouse(Input.mousePosition, down, up);

        if (down)
        {
            if (blocked)
                ResetFlickGesture();
            else
                BeginFlickGesture(Input.mousePosition, -1);
        }

        if (trackingFlick && !blocked && Input.GetMouseButton(0))
        {
            ObserveAdaptiveFlickPointer(Input.mousePosition);
            ObserveAutoHoldPointer(Input.mousePosition);
            // Commit only on MouseUp.
        }

        bool consumedBeforeRelease = flickConsumed;
        // Ordinary mouse flicks still execute on MouseUp, while a qualified
        // same-flat tap flick can already have been accepted during the hold.
        if (trackingFlick && !blocked && up)
            TryQueueFlickFromPosition(Input.mousePosition);

        // 判定前の消費状態と、離した瞬間のGuide/Hitを同時に記録する。
        if (up && trackingFlick)
            LogFlickRelease("mouse", Input.mousePosition, consumedBeforeRelease, blocked);

        if (up || blocked)
            ResetFlickGesture();
    }

    // Gesture samples are collected every frame. The tap+same-flat route alone
    // may confirm early when time-to-slope and a coherent finger motion agree.
    // Physical millimeters make the threshold independent of device pixel density.
    Vector2 adaptiveLastPosition;
    float adaptiveLastTime;
    float adaptivePeakHorizontalMm;
    float adaptivePeakVerticalMm;
    // Rotation recognition uses the entire 2D slide, not X-only samples.
    float adaptivePeakSlideMmPerSecond;
    float adaptivePeakSlideMm;
    float adaptiveTotalSlideMm;
    // Total sideways travel detects prolonged left/right dithering during a drag.
    // It does NOT impose a new time delay on a short purposeful flick.
    float adaptiveTotalHorizontalTravelMm;
    Vector2 adaptiveLastStepDirection;
    int adaptiveMovingSamples;
    int adaptiveConsistentSamples;
    float adaptivePixelsPerMm;
    // Diagnostics only: no Inspector knobs, and reset on every new gesture.
    float adaptiveRequiredMm = 1.5f;
    float adaptiveTimeToSlope = float.PositiveInfinity;
    float adaptiveDistanceToSlope = float.PositiveInfinity;
    float adaptiveForwardSpeed;

    float GesturePixelsPerMm()
    {
        // Unity may report 0 DPI. In that case use a conservative mobile baseline;
        // mouse input uses screen coordinates and should not trust desktop monitor DPI.
        if (!Application.isMobilePlatform && Input.touchCount == 0)
            return Mathf.Max(1f, minimumFlickPixels / 1.5f) *
                Mathf.Clamp(Screen.height / 1080f, 0.75f, 2f);
        float dpi = Screen.dpi;
        return (dpi >= 100f && dpi <= 800f ? dpi : 160f) / 25.4f;
    }

    void BeginAdaptiveFlick(Vector2 position)
    {
        adaptivePixelsPerMm = GesturePixelsPerMm();
        adaptiveLastPosition = position;
        adaptiveLastTime = Time.unscaledTime;
        adaptivePeakHorizontalMm = 0f;
        adaptivePeakVerticalMm = 0f;
        adaptivePeakSlideMmPerSecond = 0f;
        adaptivePeakSlideMm = 0f;
        adaptiveTotalSlideMm = 0f;
        adaptiveTotalHorizontalTravelMm = 0f;
        adaptiveLastStepDirection = Vector2.zero;
        adaptiveMovingSamples = 0;
        adaptiveConsistentSamples = 0;
        adaptiveRequiredMm = 1.5f;
        adaptiveTimeToSlope = float.PositiveInfinity;
        adaptiveDistanceToSlope = float.PositiveInfinity;
        adaptiveForwardSpeed = 0f;
    }

    void ObserveAdaptiveFlickPointer(Vector2 position)
    {
        if (!trackingFlick) return;
        float elapsed = Time.unscaledTime - adaptiveLastTime;
        Vector2 step = position - adaptiveLastPosition;
        float ppm = Mathf.Max(1f, adaptivePixelsPerMm);
        float stepMm = step.magnitude / ppm;
        if (stepMm >= .015f && elapsed > .0001f)
        {
            adaptiveTotalSlideMm += stepMm;
            adaptiveTotalHorizontalTravelMm += Mathf.Abs(step.x) / ppm;
            // Detect purposeful straight motion on ANY axis. Small jitter
            // does not contribute to the directional sample count.
            if (stepMm >= .12f)
            {
                Vector2 stepDirection = step.normalized;
                adaptiveMovingSamples++;
                adaptiveConsistentSamples =
                    adaptiveLastStepDirection != Vector2.zero &&
                    Vector2.Dot(stepDirection, adaptiveLastStepDirection) >= .65f
                        ? adaptiveConsistentSamples + 1 : 1;
                adaptiveLastStepDirection = stepDirection;

                // The peak must be based on segment time, not hold duration.
                adaptivePeakSlideMmPerSecond = Mathf.Max(
                    adaptivePeakSlideMmPerSecond,
                    Mathf.Min(1000f, stepMm / elapsed));
            }
        }
        Vector2 delta = (position - flickStart) / ppm;
        adaptivePeakSlideMm = Mathf.Max(adaptivePeakSlideMm, delta.magnitude);
        adaptivePeakHorizontalMm = Mathf.Max(adaptivePeakHorizontalMm, Mathf.Abs(delta.x));
        adaptivePeakVerticalMm = Mathf.Max(adaptivePeakVerticalMm, Mathf.Abs(delta.y));
        adaptiveLastPosition = position;
        adaptiveLastTime = Time.unscaledTime;
    }

    // Return the time until the current physically supported flat reaches the
    // next slope. Never reduce the threshold using an unverified spline or
    // off-surface position. Infinity means "use the ordinary flick threshold".
    float GetAdaptiveTimeToNextSlope(out float remainingMeters,
        out float forwardMetersPerSecond)
    {
        remainingMeters = float.PositiveInfinity;
        forwardMetersPerSecond = 0f;
        if (!rb || tapEdgeState != TapEdgeState.Armed ||
            !tapEdgeFlatCollider || flickPressFlatCollider != tapEdgeFlatCollider)
            return float.PositiveInfinity;

        var contact = HasGroundSupport();
        if (!contact.Item1 || contact.Item2.collider != tapEdgeFlatCollider ||
            !IsTapEdgeSupportedFlat(contact.Item1, contact.Item2))
            return float.PositiveInfinity;

        Vector3 heading = NormalizeFlat(
            currentGuideValid && !currentGuide.isSlope
                ? currentGuide.tangent
                : (flatEntryFrame.valid ? flatEntryFrame.direction : direction),
            direction);
        forwardMetersPerSecond = Mathf.Max(0f, Vector3.Dot(rb.velocity, heading));

        // NearestKnotDetector supplies distance to the next slope. Near a
        // seam it may momentarily switch guides, so keep the already captured
        // entry frame as a geometrically projected fallback (same spline only).
        if (currentGuideValid && !currentGuide.isSlope &&
            currentGuide.nextIsSlope &&
            IsFiniteFloat(currentGuide.distanceToNextSlope) &&
            currentGuide.distanceToNextSlope >= 0f)
            remainingMeters = currentGuide.distanceToNextSlope;

        if (flatEntryFrame.valid &&
            (!currentGuideValid || currentGuide.splineIndex == flatEntryFrame.splineIndex ||
             currentGuide.isSlope))
        {
            float entryRemaining = flatEntryFrame.distanceToNextSlope -
                Vector3.Dot(rb.position - flatEntryFrame.position,
                    flatEntryFrame.direction);
            if (IsFiniteFloat(entryRemaining) &&
                entryRemaining >= -ProbeRadius * 2f)
                remainingMeters = Mathf.Min(remainingMeters,
                    Mathf.Max(0f, entryRemaining));
        }

        // A generated flat may be split into separate physical slabs even
        // where the guide does not yet expose nextIsSlope. The forward AABB
        // extent is a conservative positional estimate of when the current
        // *same collider* may end; it never authorizes a turn without contact.
        Bounds slabBounds = contact.Item2.collider.bounds;
        float boundingHalfLength =
            Mathf.Abs(heading.x) * slabBounds.extents.x +
            Mathf.Abs(heading.z) * slabBounds.extents.z;
        float toSlabBounds = boundingHalfLength -
            Vector3.Dot(rb.position - slabBounds.center, heading);
        if (IsFiniteFloat(toSlabBounds) && toSlabBounds >= -ProbeRadius)
            remainingMeters = Mathf.Min(remainingMeters,
                Mathf.Max(0f, toSlabBounds));

        if (float.IsInfinity(remainingMeters) || forwardMetersPerSecond <= 0.5f)
            return float.PositiveInfinity;

        // Stay ahead of the flat lip by the ball radius and up to two physics
        // frames of travel. This is a recognition budget, NOT authorization
        // to turn after contact with the original flat has been lost.
        float safetyDistance = ProbeRadius * 1.25f +
            forwardMetersPerSecond * Mathf.Clamp(2f * Time.fixedDeltaTime,
                .015f, .04f);
        return Mathf.Max(0f, remainingMeters - safetyDistance) /
            forwardMetersPerSecond;
    }

    // Only a clear sideways component may request a signed 90-degree turn.
    // Do NOT infer left/right from a vertical swipe or the sign of delta.y.
    // The ordinary adaptive travel/speed thresholds remain unchanged.
    const float FlickMinimumHorizontalIntentMm = .45f;
    const float FlickMinimumHorizontalToVerticalRatio = .45f;
    const float FlickLongDragMinPathMm = 4f;
    const float FlickLongDragSideCoherence = .60f;

    bool HasClearHorizontalTurnIntent(Vector2 deltaMm)
    {
        float sideways = Mathf.Abs(deltaMm.x);
        float vertical = Mathf.Abs(deltaMm.y);
        if (sideways < FlickMinimumHorizontalIntentMm ||
            sideways < vertical * FlickMinimumHorizontalToVerticalRatio)
            return false;

        // A long held drag may wander sideways in both directions while
        // moving mainly up/down. Require only mild NET sideways coherence;
        // short, fast diagonal flicks are unaffected by this extra check.
        if (adaptiveTotalSlideMm >= FlickLongDragMinPathMm &&
            adaptiveMovingSamples >= 3 &&
            adaptiveTotalHorizontalTravelMm > .001f &&
            sideways < adaptiveTotalHorizontalTravelMm * FlickLongDragSideCoherence)
            return false;
        return true;
    }

    float SlideTurnSign(Vector2 deltaMm)
    {
        // Called only after HasClearHorizontalTurnIntent succeeds.
        return Mathf.Sign(deltaMm.x);
    }

    bool TryGetAdaptiveFlick(Vector2 position, out float signedTurnPixels)
    {
        ObserveAdaptiveFlickPointer(position);
        Vector2 pixelDelta = position - flickStart;
        Vector2 deltaMm = pixelDelta / Mathf.Max(1f, adaptivePixelsPerMm);
        float slideMm = deltaMm.magnitude;
        float coherence = slideMm /
            Mathf.Max(slideMm, adaptiveTotalSlideMm);

        bool consistent = adaptiveMovingSamples >= 2 &&
            adaptiveConsistentSamples >= 2;
        // A deliberate 1.2 mm / ~26 mm/s gesture was previously rejected
        // solely because it did not reach the 30 mm/s 'fast' threshold.
        // Keep true taps (no coherent samples) separate from slide turns.
        bool quick = consistent && adaptivePeakSlideMmPerSecond >= 30f;
        bool deliberateShort = consistent &&
            adaptivePeakSlideMmPerSecond >= 20f;
        float requiredMm = quick ? .7f : deliberateShort ? .95f : 1.5f;

        adaptiveTimeToSlope = GetAdaptiveTimeToNextSlope(
            out adaptiveDistanceToSlope, out adaptiveForwardSpeed);
        if (!float.IsInfinity(adaptiveTimeToSlope))
        {
            // Approaching a verified flat lip allows a shorter intentional
            // slide regardless of its angle; support checks are unchanged.
            float urgency = Mathf.InverseLerp(.06f, .30f,
                adaptiveTimeToSlope);
            urgency = urgency * urgency * (3f - 2f * urgency);
            requiredMm = Mathf.Min(requiredMm, Mathf.Lerp(.35f, 1.5f, urgency));
        }
        adaptiveRequiredMm = requiredMm;

        bool reduced = requiredMm < 1.5f - .001f;
        bool intentionalShort = !reduced ||
            (consistent && adaptivePeakSlideMmPerSecond >= 10f);
        bool validSlide = slideMm >= requiredMm &&
                          coherence >= .65f && intentionalShort &&
                          adaptivePeakSlideMm <=
                              Mathf.Max(1.7f * slideMm, 1.5f) &&
                          HasClearHorizontalTurnIntent(deltaMm);

        // QueueQuarterTurn and QueueEarlyFlick both read only the sign.
        // Never assign a sign based on Y when the horizontal intent is weak.
        signedTurnPixels = validSlide
            ? SlideTurnSign(deltaMm) * pixelDelta.magnitude : 0f;
        return validSlide;
    }

    // Same-flat tap flicks are queued only by their pointer release.

    void BeginFlickGesture(Vector2 position, int fingerId)
    {
        // Player intent supersedes any general automatic escape immediately.
        ResetGeneralStallRecovery("new player input", true);
        // Fresh player input always overrides automatic stalled recovery.
        if (slopeReleaseEscapeActive)
            ResetSlopeReleaseTimedEscape("new player input");
        flickStart = position;
        BeginAdaptiveFlick(position);
        flickPressUnscaledTime = Time.unscaledTime;
        trackingFlick = true;
        flickConsumed = false;
        activeFlickFingerId = fingerId;
        // 通常フリック方法Aの開始地形記録は従来どおり保持する。
        flickStartedOnFlat = BallVisualIsOnFlat;
        flickPressFlatCollider = null;
        flickStartedBeforeExtreme = currentGuideValid && currentGuide.isSlope &&
            !extremePointFiredInSection && currentGuide.sectionProgress01 < TargetProgress;

        // 自動InCornerの受付開始は「平面で押した瞬間」だけ。
        // Updateの入力フレームで物理座標を直接Evaluateし、現在の接地状態や
        // 平面→階段の境界通過イベントを受付条件に使わない。
        NearestKnotDetector.GuideFrame pressGuide = default;
        if (knotDetector && rb)
            pressGuide = knotDetector.Evaluate(rb.position);
        bool guideSaysFlat = pressGuide.valid && !pressGuide.isSlope;
        // A press close to the outgoing physical flat is still a FLAT-start
        // hold even when Evaluate() has already selected the first stair knot.
        // A late press on a stair is never upgraded to flat-start AutoHold.
        var pressSupport = HasGroundSupport();
        bool nearPressFloor = pressSupport.Item1 &&
            IsCloseGroundRay(pressSupport.Item2, rb.worldCenterOfMass);
        float sinceLastPhysicalFlat = Time.time - autoHoldRecentPressFlatTime;
        Vector3 pressOffsetFromFlat = rb.position - autoHoldRecentPressFlatPosition;
        float pressForwardTravel = Vector3.Dot(pressOffsetFromFlat,
            autoHoldRecentPressFlatHeading);
        float pressSideTravel = Vector3.ProjectOnPlane(pressOffsetFromFlat,
            Vector3.up).magnitude;
        float maximumGraceTravel = ProbeRadius +
            Mathf.Max(0f, Vector3.Dot(rb.velocity,
                autoHoldRecentPressFlatHeading)) * AutoHoldFlatPressGraceSeconds;
        bool recentFlatWitness = autoHoldRecentPressFlatCollider &&
            autoHoldRecentPressFlatTime >= 0f &&
            sinceLastPhysicalFlat >= 0f &&
            sinceLastPhysicalFlat <= AutoHoldFlatPressGraceSeconds &&
            pressForwardTravel >= -ProbeRadius &&
            pressForwardTravel <= maximumGraceTravel &&
            pressSideTravel <= maximumGraceTravel + ProbeRadius;
        bool sameRealFlat = recentFlatWitness && nearPressFloor &&
            pressSupport.Item2.collider == autoHoldRecentPressFlatCollider &&
            Vector3.Angle(pressSupport.Item2.normal, Vector3.up) <= 18f &&
            (!pressGuide.valid || !pressGuide.isSlope ||
             pressGuide.sectionProgress01 <= AutoHoldFlatPressGraceSlopeProgress);
        bool justExitedFlat = recentFlatWitness && nearPressFloor &&
            pressGuide.valid && pressGuide.isSlope &&
            pressGuide.sectionProgress01 <= AutoHoldFlatPressGraceSlopeProgress &&
            Vector3.Dot(NormalizeFlat(pressGuide.tangent, direction),
                autoHoldRecentPressFlatHeading) > .65f;
        bool autoHoldPressOnFlat = guideSaysFlat || sameRealFlat || justExitedFlat;
        string autoHoldPressSource = guideSaysFlat ? "flat spline" :
            sameRealFlat ? "same physical flat" :
            justExitedFlat ? "0.08s flat-exit grace" : "not a flat press";
        // Right AFTER the extreme point, the nearest guide may jump to the
        // next flat while the actual collider is still a slope. Preserve the
        // last verified slope frame only in this physically confirmed case.
        if (slopeReleaseAllowPostExtremePress &&
            pressGuide.valid && !pressGuide.isSlope &&
            currentGuideValid && currentGuide.isSlope &&
            extremePointFiredInSection && pressSupport.Item1 &&
            Vector3.Angle(pressSupport.Item2.normal, Vector3.up) > 18f &&
            IsAutoHoldStageHit(pressSupport.Item2, float.NaN) &&
            currentGuide.distanceToGuide <=
                Mathf.Max(ProbeRadius * 2f, supportGraceMaxGuideDistance))
        {
            pressGuide = currentGuide;
            // The physically verified stair owns this press. Do not let the
            // earlier (misleading) flat spline classification steal it.
            autoHoldPressOnFlat = false;
            autoHoldPressSource = "post-extreme physical stair";
            SlopeReleaseHoldLog("POST-EXTREME LIP",
                "using last slope guide; physical slope collider is still supporting");
        }
        // Each STAIR TREAD also has a horizontal collider normal. Therefore
        // a horizontal hit alone must not turn an actual slope press into a
        // flat tap. Preserve the old lip fallback only near the slope ENTRY
        // when the previous physics guide was really on a flat.
        bool verifiedSlopeSection = pressGuide.valid && pressGuide.isSlope &&
            pressSupport.Item1 &&
            IsAutoHoldStageHit(pressSupport.Item2, float.NaN) &&
            !sameRealFlat && !justExitedFlat &&
            (pressGuide.sectionProgress01 > .035f ||
             (currentGuideValid && currentGuide.isSlope));
        bool supportedPressFlat = !verifiedSlopeSection &&
            !justExitedFlat && // slope-exit grace must not create a TAP
            IsTapEdgeSupportedFlat(pressSupport.Item1, pressSupport.Item2);
        tapEdgePressOnFlat = supportedPressFlat;
        if (verifiedSlopeSection)
            flickStartedOnFlat = false;
        else
            flickStartedOnFlat |= supportedPressFlat || autoHoldPressOnFlat;
        flickPressFlatCollider = supportedPressFlat
            ? pressSupport.Item2.collider : null;

        // A fresh gesture after the extreme point cancels ONLY the tap path;
        // the regular 90-degree flick and existing held Auto Hold still work.
        if (tapEdgeState == TapEdgeState.TurnQueued ||
            (tapEdgeState == TapEdgeState.Qualified &&
             ((pressGuide.valid &&
               (!pressGuide.isSlope ||
                pressGuide.sectionProgress01 >= TargetProgress)) ||
              (currentGuideValid && extremePointFiredInSection))))
        {
            CancelTapEdge("new input after extreme point");
            // This same post-target tap must NOT immediately re-arm a new
            // left-edge route when released on the next flat.
            tapEdgePressOnFlat = false;
        }

        flatStartReleaseCommitInProgress = false;
        ResetAutoHoldObservation(false);
        // B release-only retains its flat-press arming policy,
        // but both lateral halves and both geometric turn directions are valid.
        // A slope-start release is a separate, release-triggered gesture.
        bool slopePressSupported = pressGuide.valid && pressGuide.isSlope &&
            !autoHoldPressOnFlat &&
            pressSupport.Item1 && IsAutoHoldStageHit(pressSupport.Item2, float.NaN);
        // Explicit post-extreme route. The event flag can be reset at the
        // next flat, so latch it HERE while the original slope is identifiable.
        bool pressAfterExtreme = slopePressSupported &&
            (pressGuide.sectionProgress01 >= TargetProgress ||
             (currentGuideValid && currentGuide.isSlope &&
              knotDetector.IsSameSection(currentGuide, pressGuide) &&
              extremePointFiredInSection));
        bool slopePressAllowed = autoInCornerHoldEnabled && slopePressSupported &&
            tapEdgeState == TapEdgeState.None && // already armed TapEdge owns the next touch
            (!pressAfterExtreme || slopeReleaseAllowPostExtremePress) &&
            !flickStartedOnFlat && BeginCommandOnTouch && Earliest &&
            AcceleFirstTurn && !MainGameManager.OnDead &&
            !HasPendingTurnIntent && !turnTransitionActive &&
            !waitingForTurnGuide && !slopeReleaseEscapeActive &&
            rb && !rb.isKinematic;
        slopeReleaseHoldArmed = slopePressAllowed;
        if (slopeReleaseHoldArmed)
        {
            slopeReleaseHoldStartGuide = pressGuide;
            slopeReleaseHoldMaxProgress = pressGuide.sectionProgress01;
            slopeReleaseStartedAfterExtreme = pressAfterExtreme;
            slopeReleaseExtremeReachedWhileHeld = pressAfterExtreme;
        }
        autoHoldEligible = autoInCornerHoldEnabled &&
            (autoHoldPressOnFlat || slopeReleaseHoldArmed) &&
            tapEdgeState == TapEdgeState.None &&
            !HasPendingTurnIntent && !turnTransitionActive &&
            !waitingForTurnGuide && !generalStallRecovering &&
            BeginCommandOnTouch && Earliest && !MainGameManager.OnDead;
        if (slopeReleaseHoldArmed)
        {
            // Capture the very first slope frame NOW. At high speed there may
            // be no further FixedUpdate on this particular section.
            ObserveAutoHoldSlope(pressGuide);
            SlopeReleaseHoldLog(pressAfterExtreme ? "POST-EXTREME ARMED" : "ARMED",
                $"section={pressGuide.sectionIndex} spline={pressGuide.splineIndex} " +
                $"progress={pressGuide.sectionProgress01:F3} target={TargetProgress:F3} " +
                $"speed={rb.velocity.magnitude:F2} finger={fingerId}");
        }
        else if (autoHoldEligible)
            AutoHoldLog("START", $"flat press started now; finger={fingerId} " +
                        $"source={autoHoldPressSource} guide={pressGuide.splineIndex} " +
                        $"progress={pressGuide.sectionProgress01:F3} " +
                        $"support={(pressSupport.Item2.collider ? pressSupport.Item2.collider.name : "-")} " +
                        $"pos={rb.position:F3}");
        else if (autoInCornerHoldEnabled)
            AutoHoldLog("NOT ARMED", $"pressFlat={autoHoldPressOnFlat} " +
                        $"source={autoHoldPressSource} " +
                        $"guideValid={pressGuide.valid} slope={pressGuide.isSlope} " +
                        $"progress={pressGuide.sectionProgress01:F3} " +
                        $"support={(pressSupport.Item2.collider ? pressSupport.Item2.collider.name : "-")} " +
                        $"nearFloor={nearPressFloor} lastFlatAge={sinceLastPhysicalFlat:F3}s " +
                        $"tapEdge={tapEdgeState} " +
                        $"started={BeginCommandOnTouch} earliest={Earliest} " +
                        $"dead={MainGameManager.OnDead}");

        if (logTurnTransition)
            Debug.Log($"[FLICK PRESS] t={Time.time:F3} input={(fingerId < 0 ? "mouse" : "touch")} " +
                      $"finger={fingerId} slope={currentGuideValid && currentGuide.isSlope} " +
                      $"flat={BallVisualIsOnFlat} progress={(currentGuideValid ? currentGuide.sectionProgress01 : -1f):F3} " +
                      $"pressOnFlat={flickStartedOnFlat} pressBeforeExtreme={flickStartedBeforeExtreme}", this);
    }

    // MouseUp/Touch Endedの入力分類・実行結果と、その瞬間の地形を同時に記録。
    void LogFlickRelease(string input, Vector2 position, bool consumedBeforeRelease, bool blocked)
    {
        if (!logTurnTransition)
            return;

        Vector2 delta = position - flickStart;
        bool slide = TryGetAdaptiveFlick(position, out float signedTurnPixels);
        bool horizontal = Mathf.Abs(delta.x) > Mathf.Abs(delta.y) * 1.10f;
        bool horizontalIntent = HasClearHorizontalTurnIntent(
            delta / Mathf.Max(1f, adaptivePixelsPerMm));
        string hitName = currentHit.collider ? currentHit.collider.name : "none";
        Debug.Log($"[FLICK RELEASE] t={Time.time:F3} input={input} " +
                  $"dx={delta.x:F1}px dy={delta.y:F1}px horizontal={horizontal} " +
                  $"horizontalIntent={horizontalIntent} " +
                  $"slide={slide} turnSign={(signedTurnPixels > 0f ? 1 : signedTurnPixels < 0f ? -1 : 0):+0;-0;0} " +
                  $"adaptiveMm={delta.magnitude / Mathf.Max(1f, adaptivePixelsPerMm):F2} " +
                  $"peakSpeedMmS={adaptivePeakSlideMmPerSecond:F1} " +
                  $"samples={adaptiveMovingSamples} coherent={adaptiveConsistentSamples} " +
                  $"requiredMm={adaptiveRequiredMm:F2} " +
                  $"timeToSlope={(float.IsInfinity(adaptiveTimeToSlope) ? -1f : adaptiveTimeToSlope):F3}s " +
                  $"commit=release tapEdge={tapEdgeState} " +
                  $"pressOnFlat={flickStartedOnFlat} pressBeforeExtreme={flickStartedBeforeExtreme} " +
                  $"consumedBefore={consumedBeforeRelease} consumedAfter={flickConsumed} " +
                  $"blocked={blocked} guideValid={currentGuideValid} " +
                  $"slope={currentGuideValid && currentGuide.isSlope} flat={BallVisualIsOnFlat} " +
                  $"progress={(currentGuideValid ? currentGuide.sectionProgress01 : -1f):F3} " +
                  $"supported={currentSupported} extremePassed={extremePointFiredInSection} " +
                  $"hit={hitName} pendingTurn={HasPendingTurnIntent}", this);
    }

    void TryQueueFlickFromPosition(Vector2 position)
    {
        if (!trackingFlick || flickConsumed)
            return;

        // Keep the original stair's identity even after AutoHold is canceled.
        bool flatStartEarlyFlickOnOriginalStair = false;

        // A held slope-start gesture is committed only by release on the NEXT
        // verified flat. It is not classified as an EarlyFlick or a flat tap.
        if (slopeReleaseHoldArmed)
        {
            ObserveAutoHoldPointer(position);
            if (slopeReleaseHoldArmed &&
                TryCommitSlopeReleaseHoldOnFlat())
                return;
            // An uncommitted stair-start gesture is not an ordinary flick.
            if (slopeReleaseHoldArmed && slopeReleaseHoldNextFlatSeen)
                return;
        }

        // B-only flat-start hold does not decide input ownership by cursor motion.
        // Resolve the *physical release surface* first:
        //   receiving flat -> discard ALL cursor samples; geometric turn only.
        //   original stair -> retain cursor samples for the existing flick path.
        // A previously verified receiving flat with a one-frame ray miss also
        // consumes the slide, but it cannot authorize a turn without a newly
        // VerifiedFlat physical hit at release.
        if (autoHoldEligible && !slopeReleaseHoldArmed && autoHoldSawSlope)
        {
            NearestKnotDetector.GuideFrame releaseGuide = knotDetector
                ? knotDetector.Evaluate(rb.position) : default;
            AutoHoldLandingFloor landing = ObserveFlatStartAutoHoldNextFlat(
                releaseGuide, out string landingReason);
            // A cached flat-ready flag does not prove physical support.
            // If a sphere cast briefly misses, verify the same receiving
            // collider with a fresh ray before authorizing a turn.
            if (landing != AutoHoldLandingFloor.VerifiedFlat &&
                autoHoldNextFlatReady &&
                TryVerifyReceivingFlatAtReleaseSeam(out string rayReason))
            {
                landing = AutoHoldLandingFloor.VerifiedFlat;
                landingReason = rayReason;
                AutoHoldLog("RELEASE SEAM VERIFIED", rayReason);
            }
            bool receivingFlatRelease =
                landing == AutoHoldLandingFloor.VerifiedFlat ||
                (autoHoldNextFlatReady && landing == AutoHoldLandingFloor.Uncertain);
            if (receivingFlatRelease)
            {
                bool committed;
                flatStartReleaseCommitInProgress = true;
                try { committed = TryCommitFlatStartAutoHoldOnRelease(); }
                finally { flatStartReleaseCommitInProgress = false; }
                Vector2 discarded = position - flickStart;
                AutoHoldLog("POINTER DISCARDED",
                    $"release={landing} dx={discarded.x:F1}px " +
                    $"dy={discarded.y:F1}px committed={committed} " +
                    $"reason={landingReason}; no ordinary flick");
                // The release belongs to this AutoHold even if its physical
                // turn was rejected. Do not reinterpret the held swipe as an
                // unintended opposite-direction QuarterTurn/TapEdge.
                flickConsumed = true;
                flickStart = position;
                BeginAdaptiveFlick(position);
                return;
            }
            // A flat-start gesture released on its ORIGINAL stair before
            // the extreme point is eligible for lateral micro-steering ONLY.
            // Compute this while AutoHold's original-slope guide still exists.
            var stairSupport = HasGroundSupport();
            flatStartEarlyFlickOnOriginalStair = flickStartedOnFlat &&
                !autoHoldNextFlatReady && currentGuideValid &&
                currentGuide.isSlope && autoHoldLastSlopeGuide.valid &&
                knotDetector.IsSameSection(autoHoldLastSlopeGuide, currentGuide) &&
                currentGuide.sectionProgress01 < TargetProgress &&
                !extremePointFiredInSection && stairSupport.Item1 &&
                IsAutoHoldStageHit(stairSupport.Item2, float.NaN);
            CancelAutoHold("released on original stair / outside next flat; retain pointer flick");
        }

        Vector2 flick = position - flickStart;
        bool validFlick = TryGetAdaptiveFlick(position, out float signedTurnPixels);
        // Register a real short TAP on flat, never a long-held release or a
        // swipe. Old Auto Hold remains enabled throughout the held gesture.
        // A tap must remain near its start; it may not be an already-qualified flick.
        float tapPixels = 1.5f * Mathf.Max(1f, adaptivePixelsPerMm);
        var releaseSupport = HasGroundSupport();
        bool physicalFlat = releaseSupport.Item1 && releaseSupport.Item2.collider &&
            Vector3.Angle(releaseSupport.Item2.normal, Vector3.up) <= 15f;
        bool stableTapFlatAtRelease = IsTapEdgeSupportedFlat(
            releaseSupport.Item1, releaseSupport.Item2);
        if (tapEdgeState == TapEdgeState.None && tapEdgePressOnFlat &&
            stableTapFlatAtRelease && releaseSupport.Item2.collider == flickPressFlatCollider &&
            BeginCommandOnTouch && Earliest &&
            !MainGameManager.OnDead && !HasPendingTurnIntent &&
            !turnTransitionActive && !waitingForTurnGuide &&
            Time.unscaledTime - flickPressUnscaledTime <= .30f &&
            !validFlick && flick.sqrMagnitude <= tapPixels * tapPixels &&
            adaptivePeakHorizontalMm <= 1.5f && adaptivePeakVerticalMm <= 1.5f)
        {
            tapEdgeState = TapEdgeState.Armed;
            tapEdgeFlatCollider = releaseSupport.Item2.collider;
            TapEdgeLog("ARMED", $"short flat tap collider={tapEdgeFlatCollider.name}");
            return;
        }
        if (!validFlick || !Earliest)
        {
            // Report directional rejection once, on release, without spamming
            // drag samples or changing any of the existing turn acceptance gates.
            if (!validFlick && logTurnTransition &&
                flick.magnitude / Mathf.Max(1f, adaptivePixelsPerMm) >= adaptiveRequiredMm &&
                !HasClearHorizontalTurnIntent(
                    flick / Mathf.Max(1f, adaptivePixelsPerMm)))
                Debug.Log($"[FLICK DIRECTION REJECTED] t={Time.time:F3} " +
                    $"dx={flick.x:F1}px dy={flick.y:F1}px " +
                    $"netHorizontalMm={Mathf.Abs(flick.x) / Mathf.Max(1f, adaptivePixelsPerMm):F2} " +
                    $"horizontalPathMm={adaptiveTotalHorizontalTravelMm:F2} " +
                    $"sideToVertical={Mathf.Abs(flick.x) / Mathf.Max(.01f, Mathf.Abs(flick.y)):F2}; " +
                    $"no left/right turn from vertical or wandering drag", this);
            if (!validFlick && logTurnTransition &&
                adaptiveMovingSamples == 0 && flick.sqrMagnitude <= Eps)
                Debug.Log($"[FLICK NO MOVEMENT] t={Time.time:F3} " +
                    $"input samples=0; a click or a touch without cursor motion " +
                    $"cannot determine a 90-degree turn direction. " +
                    $"slope={(currentGuideValid && currentGuide.isSlope)} " +
                    $"progress={(currentGuideValid ? currentGuide.sectionProgress01 : -1f):F3}", this);
            if (tapEdgeState == TapEdgeState.Armed && logTurnTransition)
                TapEdgeLog("FLICK REJECTED",
                    $"dx={flick.x:F1}px dy={flick.y:F1}px " +
                    $"speedMmS={adaptivePeakSlideMmPerSecond:F1} " +
                    $"samples={adaptiveMovingSamples} " +
                    $"coherent={adaptiveConsistentSamples} " +
                    $"requiredMm={adaptiveRequiredMm:F2} " +
                    $"timeToSlope={(float.IsInfinity(adaptiveTimeToSlope) ? -1f : adaptiveTimeToSlope):F3}s " +
                    $"remaining={adaptiveDistanceToSlope:F2}m " +
                    $"earliest={Earliest}");
            return;
        }

        // Keep the press/release physical collider identity authoritative.
        // At the edge of a flat MeshCollider the sphere cast can touch a beveled
        // triangle (normal > 15 degrees) even while the spline is still flat.
        // Use the existing supported flat guide as a bounded fallback; NEVER
        // accept missing support or a different collider.
        bool supportedTapFlat = stableTapFlatAtRelease;
        // Each stair tread also has a horizontal physical normal. A verified
        // original-stair EarlyFlick cannot be stolen by the generic flat path.
        bool verifiedEarlyStair = flatStartEarlyFlickOnOriginalStair &&
            currentGuideValid && currentGuide.isSlope &&
            currentGuide.sectionProgress01 < TargetProgress &&
            !extremePointFiredInSection;
        if (!verifiedEarlyStair &&
            (BallVisualIsOnFlat || physicalFlat || supportedTapFlat))
        {
            // The tap is still ARMED only for the same live physical slab.
            // A transient slope guide sample must not veto a genuine same-slab
            // tap+flick; leaving the slab or an accepted micro-flick already
            // transitions/cancels the Armed state through the existing policy.
            bool sameFlatTapTurn = tapEdgeState == TapEdgeState.Armed &&
                tapEdgeFlatCollider && flickPressFlatCollider &&
                releaseSupport.Item2.collider == tapEdgeFlatCollider &&
                flickPressFlatCollider == tapEdgeFlatCollider &&
                supportedTapFlat;
            if (tapEdgeState == TapEdgeState.Armed)
            {
                Collider releaseCollider = releaseSupport.Item2.collider;
                float supportAngle = releaseCollider
                    ? Vector3.Angle(releaseSupport.Item2.normal, Vector3.up) : -1f;
                TapEdgeLog(sameFlatTapTurn ? "SAME FLAT MATCH" : "SAME FLAT REJECT",
                    $"tap={(tapEdgeFlatCollider ? tapEdgeFlatCollider.name : "-")}#{(tapEdgeFlatCollider ? tapEdgeFlatCollider.GetInstanceID() : 0)} " +
                    $"press={(flickPressFlatCollider ? flickPressFlatCollider.name : "-")}#{(flickPressFlatCollider ? flickPressFlatCollider.GetInstanceID() : 0)} " +
                    $"release={(releaseCollider ? releaseCollider.name : "-")}#{(releaseCollider ? releaseCollider.GetInstanceID() : 0)} " +
                    $"support={releaseSupport.Item1} angle={supportAngle:F1} physicalFlat={physicalFlat} " +
                    $"stableFlat={supportedTapFlat} guideFlat={BallVisualIsOnFlat} slopeSeen={tapEdgeSlopeGuide.valid}");
            }
            flickConsumed = QueueQuarterTurn(signedTurnPixels, sameFlatTapTurn);
            return;
        }

        // 階段で離した場合は、極限ポイント前に押したジェスチャーを
        // 微小進路変更として分類する（途中で60%を越えても継続）。
        // B is release-only; the original stair remains a micro-flick exception.
        if (currentGuideValid && currentGuide.isSlope &&
            (flickStartedBeforeExtreme || currentGuide.sectionProgress01 < TargetProgress))
        {
            if (CanQueueEarlyFlick(flickStartedBeforeExtreme,
                    flatStartEarlyFlickOnOriginalStair))
            {
                QueueEarlyFlick(signedTurnPixels);
                flickConsumed = true;
                if (flatStartEarlyFlickOnOriginalStair)
                    AutoHoldLog("FLAT-START EARLY FLICK ACCEPTED",
                        $"progress={currentGuide.sectionProgress01:F3}; lateral only");
            }
            return; // 階段でEarlyFlickが拒否されても90度旋回へ誤送しない。
        }

        // 極限ポイント通過後の階段で離した場合など、既存の90度許可条件に従う。
        if (CanAcceptQuarterTurn(out _))
            flickConsumed = QueueQuarterTurn(signedTurnPixels);
    }

    void ResetFlickGesture()
    {
        flatStartReleaseCommitInProgress = false;
        trackingFlick = false;
        flickConsumed = false;
        flickStartedBeforeExtreme = false;
        flickStartedOnFlat = false;
        tapEdgePressOnFlat = false;
        flickPressFlatCollider = null;
        if (autoHoldEligible)
            CancelAutoHold("input released/canceled before valid exit");
        else
            ResetAutoHoldObservation(true);
        activeFlickFingerId = -1;
    }

    void TapEdgeLog(string phase, string details)
    {
        if (logTurnTransition)
            Debug.Log($"[TAP EDGE {phase}] t={Time.fixedTime:F3} {details}", this);
    }

    // The same-flat tap path may be at the lip of a mesh, where a sphere cast
    // sees a small inclined edge triangle instead of the horizontal top.
    // Trust only an actual supported hit together with a still-flat active
    // guide, and never accept a fully unsupported body or an actual slope.
    // No new Inspector setting or physical force is introduced here.
    bool IsTapEdgeSupportedFlat(bool hasSupport, RaycastHit hit)
    {
        if (!hasSupport || !hit.collider)
            return false;
        float angle = Vector3.Angle(hit.normal, Vector3.up);
        return angle <= 15f ||
            (angle <= maxSlopeAngle && currentGuideValid &&
             !currentGuide.isSlope);
    }

    // Only a handoff that actually waited on the tapped flat restores XZ speed.
    // Never inject it after falling or landing on another collider.
    void ReleaseTapEdgeHandoffHold()
    {
        if (!tapEdgeHandoffHeld) return;
        tapEdgeHandoffHeld = false;
        if (!rb || rb.isKinematic || !tapEdgeFlatCollider) return;
        var support = HasGroundSupport();
        if (!support.Item1 || support.Item2.collider != tapEdgeFlatCollider) return;
        Vector3 original = pendingTurnFlickVelocity;
        rb.velocity = new Vector3(original.x, rb.velocity.y, original.z);
        TapEdgeLog("HANDOFF RESUME", $"velocity={rb.velocity:F3}");
    }

    void ResetTapEdge()
    {
        ReleaseTapEdgeHandoffHold();
        tapEdgeState = TapEdgeState.None;
        tapEdgeFlatCollider = null;
        tapEdgeFastTurnQueued = false;
        tapEdgeSlopeGuide = default;
        tapEdgeMaxSlopeProgress = 0f;
    }

    void CancelTapEdge(string reason)
    {
        if (tapEdgeState == TapEdgeState.None) return;
        if (tapEdgeState == TapEdgeState.TurnQueued && HasPendingTurnIntent)
        {
            ReleaseTapEdgeHandoffHold();
            pendingTurnDegrees = 0f;
            pendingTurnQueuedTime = -1f;
            pendingTurnFromPassedSlope = false;
            ClearPendingTurnSnapshot();
            ballVisualSlopeDrive?.CancelTurnHandoffRequest();
        }
        TapEdgeLog("CANCEL", reason);
        ResetTapEdge();
    }

    // Called on the SAME physics guide as the existing drive. No duplicate
    // spline Evaluate, new forces, collider scanning, or micro-move function.
    void ObserveTapEdgeGuide(NearestKnotDetector.GuideFrame guide, bool grounded)
    {
        if (tapEdgeState == TapEdgeState.Armed)
        {
            // The sampled guide can briefly jump to the next spline while
            // physical support remains on the tapped slab. Keep the tap alive.
            if (grounded && tapEdgeFlatCollider &&
                currentHit.collider == tapEdgeFlatCollider)
                return;
            // The tap applies to only the immediately following slope.
            if (guide.isSlope)
                tapEdgeSlopeGuide = guide;
            else if (tapEdgeSlopeGuide.valid && grounded)
                CancelTapEdge("first slope passed without micro flick");
            return;
        }
        if (tapEdgeState != TapEdgeState.Qualified) return;
        if (MainGameManager.OnDead || !BeginCommandOnTouch || !Earliest)
        {
            CancelTapEdge("inactive or dead");
            return;
        }
        if (guide.isSlope)
        {
            tapEdgeSlopeGuide = guide;
            tapEdgeMaxSlopeProgress = Mathf.Max(tapEdgeMaxSlopeProgress,
                guide.sectionProgress01);
            return;
        }
        // Only the next FLAT after the qualified slope. A brief false flat
        // at a slab seam or a loss of support must not trigger a turn.
        if (!grounded || !tapEdgeSlopeGuide.valid ||
            HasPendingTurnIntent || turnTransitionActive || waitingForTurnGuide ||
            rb.isKinematic ||
            Vector3.Dot(NormalizeFlat(guide.tangent, direction),
                NormalizeFlat(tapEdgeSlopeGuide.tangent, direction)) < .80f)
            return;

        Vector3 forward = NormalizeFlat(tapEdgeSlopeGuide.tangent, direction);
        Vector3 exit = tapEdgeSlopeGuide.point +
            tapEdgeSlopeGuide.tangent.normalized * Mathf.Max(0f,
                tapEdgeSlopeGuide.sectionLength *
                (1f - tapEdgeSlopeGuide.sectionProgress01));
        bool passedExit = wasSlope &&
            tapEdgeMaxSlopeProgress >= TargetProgress &&
            Vector3.Dot(rb.position - exit, forward) >= -.4f;
        if (knotDetector.IsSameSection(tapEdgeSlopeGuide, guide) ||
            (!passedExit && tapEdgeMaxSlopeProgress < .95f))
            return;
        Vector3 left = Vector3.Cross(forward, Vector3.up).normalized;
        float sign = ResolveEdgeTurnSign(forward, left);
        pendingTurnDegrees = sign * QuarterTurnDegrees;
        pendingTurnQueuedTime = Time.time;
        pendingTurnFromPassedSlope = false;
        tapEdgeState = TapEdgeState.TurnQueued;
        CapturePendingTurnSnapshot();
        RequestTurnHandoffForPendingIntent();
        TapEdgeLog("TURN QUEUED", $"sign={sign:+0;-0} progress={tapEdgeMaxSlopeProgress:F3}");
    }

    // 独立操作: 通常のフリック方法A(MouseUp/Touch.Ended)は変更しない。
    void AutoHoldLog(string phase, string details)
    {
        if (logTurnTransition)
            Debug.Log($"[AUTO HOLD {phase}] t={Time.fixedTime:F3} {details}", this);
    }

    void ResetAutoHoldObservation(bool cancelHandoff)
    {
        slopeReleaseHoldArmed = false;
        slopeReleaseHoldNextFlatSeen = false;
        slopeReleaseHoldStartGuide = default;
        slopeReleaseHoldMaxProgress = 0f;
        slopeReleaseStartedAfterExtreme = false;
        slopeReleaseExtremeReachedWhileHeld = false;
        // Keep the committed flat's physical collider until handoff execution.
        if (!slopeReleaseHoldTurnPending)
        {
            slopeReleaseHoldNextFlatCollider = null;
            slopeReleaseHoldLastFlatSupportTime = -1f;
            slopeReleaseHoldLastFlatSupportPosition = default;
        }
        autoHoldEligible = false;
        autoHoldSawSlope = false;
        autoHoldLastHalfValid = false;
        autoHoldLeftHalf = false;
        autoHoldNextFlatReady = false;
        autoHoldNextFlatCollider = null;
        autoHoldNextFlatLastSupportTime = -1f;
        autoHoldNextFlatLastSupportPosition = default;
        autoHoldPredictedTurnSignValid = false;
        autoHoldPredictedTurnSign = 0f;
        // A committed turn still needs this slope/flat and physical-edge data
        // after the gesture has been consumed while awaiting BallVisual Handoff.
        if (!autoHoldTurnPending)
        {
            autoHoldLastSlopeGuide = default;
            autoHoldLastCenter = Vector3.zero;
            autoHoldLastLeft = Vector3.zero;
            autoHoldLastForward = Vector3.zero;
            autoHoldExitPoint = Vector3.zero;
            autoHoldLastInnerLinePoint = Vector3.zero;
            autoHoldLeftOffset = 0f;
            autoHoldLastSlopeProgress = 0f;
            autoHoldEdgeSampleValid = false;
            autoHoldNextEdgeSampleTime = 0f;
            autoHoldEdgeCollider = null;
        }
    }

    void CancelAutoHold(string reason)
    {
        if (autoHoldEligible)
        {
            if (slopeReleaseHoldArmed)
                SlopeReleaseHoldLog("CANCEL", reason);
            else
                AutoHoldLog("CANCEL", reason);
        }
        ResetAutoHoldObservation(true);
    }

    void ObserveAutoHoldPointer(Vector2 pointerPosition)
    {
        if (!autoHoldEligible) return;
        // A gesture started on a FLAT owns its entire held interval until
        // release. Mouse travel must not cancel this hold on the original
        // stair: whether those samples are discarded depends on WHERE the
        // player releases. Original stair => ordinary flick; receiving flat
        // => the samples are discarded and only the geometric AutoHold turn
        // can commit. The slope-start hold keeps its existing swipe policy.
        if (!slopeReleaseHoldArmed) return;
        if (Mathf.Abs(pointerPosition.x - flickStart.x) >
            Mathf.Max(1f, autoHoldHorizontalTolerancePixels))
        {
            CancelAutoHold("horizontal swipe");
            return;
        }
        // A purposeful vertical/diagonal slide also belongs to ordinary
        // Flick, not to a release-to-turn hold. Do not cancel on hand jitter:
        // require a coherent adaptive slide plus appreciable Y motion.
        if (Mathf.Abs(pointerPosition.y - flickStart.y) >
                Mathf.Max(1f, autoHoldHorizontalTolerancePixels) * 1.5f &&
            adaptiveMovingSamples >= 2 && adaptiveConsistentSamples >= 2 &&
            TryGetAdaptiveFlick(pointerPosition, out _))
            CancelAutoHold("coherent vertical/diagonal swipe");
    }

    bool IsAutoHoldReleaseAwaitingUpdate()
    {
        if (!trackingFlick || flickConsumed) return false;
        if (Application.isMobilePlatform || Input.touchCount > 0)
        {
            for (int i = 0; i < Input.touchCount; ++i)
            {
                Touch touch = Input.GetTouch(i);
                if (touch.fingerId == activeFlickFingerId)
                    return touch.phase == TouchPhase.Ended;
            }
            return false;
        }
        return Input.GetMouseButtonUp(0);
    }

    bool IsAutoHoldInputStillDown()
    {
        if (!trackingFlick || flickConsumed) return false;
        if (Application.isMobilePlatform || Input.touchCount > 0)
        {
            for (int i = 0; i < Input.touchCount; ++i)
            {
                Touch touch = Input.GetTouch(i);
                if (touch.fingerId != activeFlickFingerId) continue;
                if (touch.phase == TouchPhase.Ended || touch.phase == TouchPhase.Canceled)
                    return false;
                ObserveAutoHoldPointer(touch.position);
                return autoHoldEligible;
            }
            return false;
        }
        if (!Input.GetMouseButton(0))
            return false;
        ObserveAutoHoldPointer(Input.mousePosition);
        return autoHoldEligible;
    }

    // 左右どちらの位置も有効。地形の左軸は横位置の観測とInCorner用のエッジ探索にだけ使用。
    void ObserveAutoHoldSlope(NearestKnotDetector.GuideFrame guide)
    {
        if (!guide.valid || !guide.isSlope || !knotDetector || !rb) return;
        Surface surface = BuildSplineSurface(guide);
        if (!surface.Valid) return;
        bool hasFrame = knotDetector.TryGetFiveLineFrame(
            guide, surface.side,
            out NearestKnotDetector.FiveLineFrame frame) && frame.valid;
        if (!hasFrame && slopeReleaseHoldArmed &&
            slopeReleaseStartedAfterExtreme)
        {
            // At the very last stair, the detector may have advanced to a
            // flat segment while only the held slope's last guide survives.
            // A matching next-flat lane cross-section is an acceptable
            // geometry fallback; the ACTUAL edge is still ray-verified.
            Vector3 forward = NormalizeFlat(direction, guide.tangent);
            hasFrame = knotDetector.TryGetFollowingFlatFiveLineFrame(
                guide, forward, ProbeRadius * 2f, out frame) && frame.valid;
        }
        if (!hasFrame)
        {
            if (!autoHoldSawSlope)
                AutoHoldLog("SLOPE", "waiting for real FiveLine cross-section");
            return;
        }
        Vector3 heading = NormalizeFlat(direction, guide.tangent);
        Vector3 left = Vector3.Cross(heading, Vector3.up).normalized;
        Vector3 candidateLeft = frame.left;
        float candidateDistance = Vector3.Dot(candidateLeft - frame.center, left);
        float rightDistance = Vector3.Dot(frame.right - frame.center, left);
        if (rightDistance > candidateDistance)
        {
            candidateLeft = frame.right;
            candidateDistance = rightDistance;
        }
        if (candidateDistance <= autoHoldLeftHalfMarginMeters) return;

        bool wasObserved = autoHoldSawSlope;
        bool wasLeft = autoHoldLeftHalf;
        autoHoldSawSlope = true;
        autoHoldLastHalfValid = true;
        autoHoldLastCenter = frame.center;
        autoHoldLastLeft = left;
        autoHoldLastForward = heading;
        autoHoldLastInnerLinePoint = candidateLeft;
        autoHoldLeftOffset = Vector3.Dot(rb.position - frame.center, left);
        autoHoldLeftHalf = autoHoldLeftOffset > autoHoldLeftHalfMarginMeters;
        // FiveLine supplies a cross-section for BOTH halves; left/right is diagnostic, not eligibility.
        // The actual target is determined from the active physics surface below.
        autoHoldLastSlopeGuide = guide;
        autoHoldLastSlopeProgress = guide.sectionProgress01;
        if (!wasObserved)
        {
            // Inspect the next *actual* slope after this slope's flat.
            // The current nearest guide can still be on the stairs when a
            // fast hold reaches the flat, so it is not enough by itself.
            autoHoldPredictedTurnSignValid =
                TryResolveAutoHoldTurnFromFollowingSlope(guide,
                    heading, out autoHoldPredictedTurnSign);
        }

        // Approximate geometric exit plane, independent from body Y / ground collision.
        Vector3 alongSlope = guide.tangent.sqrMagnitude > Eps
            ? guide.tangent.normalized : surface.tangent;
        if (Vector3.Dot(alongSlope, heading) < 0f) alongSlope = -alongSlope;
        float remaining = Mathf.Max(0f, guide.sectionLength *
            (1f - Mathf.Clamp01(guide.sectionProgress01)));
        autoHoldExitPoint = guide.point + alongSlope * remaining;

        if (!wasObserved)
            AutoHoldLog("SLOPE", $"section={guide.splineIndex} progress={guide.sectionProgress01:F3}");
        if (autoHoldLeftHalf && (!wasLeft || !wasObserved))
            AutoHoldLog("LEFT HALF", $"offset={autoHoldLeftOffset:F3}m innerLine={candidateDistance:F3}m");
        if (wasLeft && !autoHoldLeftHalf)
            AutoHoldLog("LEFT HALF LOST", $"offset={autoHoldLeftOffset:F3}m");

        // Do NOT request Handoff here. The old automatic turn used to do
        // that at 95% slope progress, even while the player kept pressing.
        // The existing QueueAutoHoldHeldTurn requests it AFTER release.
    }

    // These probes share HasGroundSupport's existing radius, mask, distance,
    // and permitted surface angle. Only stage colliders count as a floor:
    // the visible BallVisual / InSubject rigidbodies must never define an edge.
    bool IsAutoHoldStageHit(RaycastHit hit, float referenceHeight)
    {
        Collider collider = hit.collider;
        return collider && collider.enabled && !collider.isTrigger &&
            !(collider is SphereCollider) && !collider.attachedRigidbody &&
            Vector3.Angle(hit.normal, Vector3.up) <= maxSlopeAngle &&
            (float.IsNaN(referenceHeight) ||
             Mathf.Abs(hit.point.y - referenceHeight) <= ProbeRadius * 1.75f);
    }

    bool TryAutoHoldGroundRay(Vector3 sample, out RaycastHit hit,
        float referenceHeight = float.NaN)
    {
        hit = default;
        int count = Physics.RaycastNonAlloc(sample + Vector3.up * .05f,
            Vector3.down, autoHoldCastHits, probeDistance + ProbeRadius + .25f,
            groundMask, QueryTriggerInteraction.Ignore);
        float closest = float.PositiveInfinity;
        bool found = false;
        for (int i = 0; i < count; ++i)
        {
            RaycastHit candidate = autoHoldCastHits[i];
            if (!IsAutoHoldStageHit(candidate, referenceHeight) ||
                candidate.distance >= closest) continue;
            closest = candidate.distance;
            hit = candidate;
            found = true;
        }
        return found;
    }

    bool TryAutoHoldGroundSphere(Vector3 sample, float referenceHeight)
    {
        int count = Physics.SphereCastNonAlloc(sample + Vector3.up * .05f,
            ProbeRadius, Vector3.down, autoHoldCastHits, probeDistance,
            groundMask, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < count; ++i)
            if (IsAutoHoldStageHit(autoHoldCastHits[i], referenceHeight))
                return true;
        return false;
    }

    // SphereCast can miss an edge when its sweep starts in contact with a
    // MeshCollider or its hit normal is the side of the slab. Do not treat a
    // single missed sphere as proof that the stage is missing. Instead accept
    // only a conservative footprint backed by several same-collider rays.
    bool TryAutoHoldVerifiedFootprint(Vector3 centre, float height,
        Vector3 inward, out string evidence)
    {
        if (TryAutoHoldGroundSphere(centre, height))
        {
            evidence = "sphere";
            return true;
        }

        evidence = "no_central_ray";
        if (!TryAutoHoldGroundRay(centre, out RaycastHit middle, height))
            return false;
        Collider slab = middle.collider;
        if (!slab || !slab.enabled || slab.isTrigger)
            return false;

        inward = Vector3.ProjectOnPlane(inward, Vector3.up);
        if (inward.sqrMagnitude <= Eps)
        {
            evidence = "invalid_inward_axis";
            return false;
        }
        inward.Normalize();
        Vector3 forward = Vector3.Cross(Vector3.up, inward).normalized;
        Vector3 inside = centre + inward * (ProbeRadius * .55f);
        // Same physical floor, forward and backward, not only a single pixel
        // of a thin ledge. This fallback does not approve a missing stage.
        if (!TryAutoHoldFootprintRay(inside, slab, height) ||
            !TryAutoHoldFootprintRay(inside + forward * (ProbeRadius * .30f),
                slab, height) ||
            !TryAutoHoldFootprintRay(inside - forward * (ProbeRadius * .30f),
                slab, height))
        {
            evidence = "footprint_rays_not_supported";
            return false;
        }
        evidence = "verified_three_ray_footprint";
        return true;
    }

    bool TryAutoHoldFootprintRay(Vector3 sample, Collider expected,
        float height)
    {
        return TryAutoHoldGroundRay(sample, out RaycastHit hit, height) &&
            hit.collider == expected &&
            Mathf.Abs(hit.point.y - height) <= ProbeRadius * .5f;
    }

    // The spline supplies ONLY a cross-section and an estimated search width.
    // The final boundary is always determined by actual stage collider hits.
    // No Evaluate() call: that would mutate the detector's active guide tracking.
    bool TryBuildAutoHoldEdgeAnchor(Vector3 left, out Vector3 centre,
        out float range)
    {
        centre = rb.position;
        range = ProbeRadius * 4f;
        if (!knotDetector || !autoHoldLastSlopeGuide.valid) return false;

        NearestKnotDetector.FiveLineFrame frame;
        float pastExit = Vector3.Dot(rb.position - autoHoldExitPoint,
            autoHoldLastForward);
        bool flatApproach = pastExit >= -ProbeRadius * 2f ||
            (currentGuideValid && !currentGuide.isSlope);

        if (flatApproach && knotDetector.TryGetFollowingFlatStart(
                autoHoldLastSlopeGuide, out Vector3 flatStart,
                out Vector3 flatTangent))
        {
            float along = Mathf.Max(ProbeRadius * 2f,
                Vector3.Dot(rb.position - flatStart, autoHoldLastForward));
            if (knotDetector.TryGetFollowingFlatFiveLineFrame(
                    autoHoldLastSlopeGuide, autoHoldLastForward,
                    along, out frame) && frame.valid)
                centre = frame.center;
            else
            {
                // Some newly inserted flats have no complete three-lane
                // cross section yet. Their existing start/tangent still give
                // a valid Y and search origin; the ray finds the actual edge.
                frame = default;
                Vector3 flatForward = NormalizeFlat(flatTangent,
                    autoHoldLastForward);
                float lane = Vector3.Dot(autoHoldLastCenter -
                    autoHoldLastSlopeGuide.point, left);
                centre = flatStart + flatForward * along + left * lane;
            }
        }
        else if (currentGuideValid && currentGuide.isSlope &&
            knotDetector.TryGetFiveLineFrame(currentGuide, currentSurfaceValid
                ? currentSurface.side : Vector3.Cross(Vector3.up, autoHoldLastForward),
                out frame) && frame.valid)
        {
            centre = frame.center;
        }
        else
        {
            // Last measured slope centre is safe as a search *origin*, not
            // as the physical edge. Its Y is from the tracked support section.
            frame = default;
            centre = autoHoldLastCenter;
            centre += autoHoldLastForward *
                Vector3.Dot(rb.position - centre, autoHoldLastForward);
        }

        float width = frame.valid
            ? Mathf.Abs(Vector3.Dot(frame.left - frame.right, left))
            : Mathf.Abs(Vector3.Dot(autoHoldLastInnerLinePoint -
                autoHoldLastCenter, left)) * 2f;
        // Search relative to the SAME lane centre, not to a ball that may
        // already be several metres to the right of it.
        range = Mathf.Max(ProbeRadius * 4f, width + ProbeRadius * 4f);
        // Body centre is ~one radius above the actual flat/slope surface.
        centre.y += ProbeRadius + .25f;
        return width > ProbeRadius;
    }

    bool TryScanAutoHoldLeftEdge(Vector3 probeCentre, Vector3 left,
        float range, out Vector3 target)
    {
        return TryScanAutoHoldLeftEdge(probeCentre, left, range,
            out target, out _);
    }

    bool TryScanAutoHoldLeftEdge(Vector3 probeCentre, Vector3 left,
        float range, out Vector3 target, out string failureStage,
        bool keepFullBallOnSlab = false)
    {
        target = rb.position;
        failureStage = "invalid_side";
        left = Vector3.ProjectOnPlane(left, Vector3.up);
        if (left.sqrMagnitude <= Eps) return false;
        left.Normalize();

        // Coarse Raycasts use the existing ball radius; the six bisections
        // refine the physical lip without scanning hundreds of tiny steps.
        float step = ProbeRadius;
        float originOffset = 0f;
        if (!TryAutoHoldGroundRay(probeCentre, out RaycastHit referenceHit))
        {
            // The centre can be in an actual joint/gap: recover toward the
            // inside first. Never take an unsupported point as the edge.
            bool recovered = false;
            for (float back = step; back <= range; back += step)
            {
                if (!TryAutoHoldGroundRay(probeCentre - left * back,
                        out referenceHit)) continue;
                originOffset = -back;
                recovered = true;
                break;
            }
            if (!recovered)
            {
                failureStage = "no_reference_stage_ray";
                return false;
            }
        }

        float referenceHeight = referenceHit.point.y;
        float lastSupported = 0f;
        float firstUnsupported = -1f;
        bool confirmedBoundary = false;
        for (float d = step; d <= range + ProbeRadius + Eps; d += step)
        {
            if (TryAutoHoldGroundRay(probeCentre +
                    left * (originOffset + d), out _, referenceHeight))
            {
                lastSupported = d;
                firstUnsupported = -1f;
                continue;
            }
            if (firstUnsupported < 0f) firstUnsupported = d;
            // Small slab seams are not necessarily the outside edge.
            // Require an unsupported span wider than the sphere footprint.
            if (d - firstUnsupported < ProbeRadius) continue;
            confirmedBoundary = true;
            break;
        }
        if (!confirmedBoundary)
        {
            failureStage = "no_outer_boundary_within_scan_range";
            return false;
        }

        for (int i = 0; i < 6; ++i)
        {
            float mid = (lastSupported + firstUnsupported) * .5f;
            if (TryAutoHoldGroundRay(probeCentre +
                    left * (originOffset + mid), out _, referenceHeight))
                lastSupported = mid;
            else
                firstUnsupported = mid;
        }
        // Fast same-flat approach: the centre must stay one ball radius
        // inside the measured lip. Ordinary Auto Hold retains its prior inset.
        float inset = keepFullBallOnSlab
            ? ProbeRadius * 1.10f : ProbeRadius * .20f;
        float edgeOffset = originOffset +
            Mathf.Max(0f, lastSupported - inset);
        Vector3 checkedCentre = probeCentre + left * edgeOffset;
        if (!TryAutoHoldGroundRay(checkedCentre,
                out RaycastHit edgeHit, referenceHeight))
        {
            failureStage = "refined_edge_ray_missing";
            return false;
        }
        if (!TryAutoHoldVerifiedFootprint(checkedCentre, referenceHeight,
                -left, out string verification))
        {
            failureStage = "refined_edge_footprint_missing_" + verification;
            return false;
        }
        // A non-null note on success is diagnostic, not an error. The caller
        // logs whether the fallback used a three-ray stage footprint.
        failureStage = keepFullBallOnSlab && verification != "sphere"
            ? verification : null;

        target = rb.position + left *
            Vector3.Dot(checkedCentre - rb.position, left);
        autoHoldEdgeCollider = edgeHit.collider;
        autoHoldEdgeProbeHeight = checkedCentre.y;
        autoHoldEdgeSurfaceHeight = referenceHeight;
        return true;
    }

    bool TryReuseAutoHoldEdge(Vector3 left, out Vector3 target)
    {
        target = rb.position;
        if (!autoHoldEdgeSampleValid || !autoHoldEdgeCollider ||
            !autoHoldEdgeCollider.enabled) return false;
        Vector3 centre = rb.position + left *
            Vector3.Dot(autoHoldEdgeSample - rb.position, left);
        centre.y = autoHoldEdgeProbeHeight;
        if (!TryAutoHoldGroundRay(centre, out RaycastHit hit,
                autoHoldEdgeSurfaceHeight) ||
            hit.collider != autoHoldEdgeCollider ||
            !TryAutoHoldGroundSphere(centre, autoHoldEdgeSurfaceHeight))
            return false;
        target = rb.position + left *
            Vector3.Dot(centre - rb.position, left);
        return true;
    }

    bool TryResolveAutoHoldEdge(Vector3 left, out Vector3 target)
    {
        target = rb.position;
        left = Vector3.ProjectOnPlane(left, Vector3.up);
        if (left.sqrMagnitude <= Eps) return false;
        left.Normalize();
        if (TryBuildAutoHoldEdgeAnchor(left, out Vector3 anchor,
                out float range) &&
            TryScanAutoHoldLeftEdge(anchor, left, range, out target))
            return true;
        // Never extrapolate an unverified edge across a different collider.
        return TryReuseAutoHoldEdge(left, out target);
    }

    // The previous EDGE LOCK measures the PRE-turn lip. This controller
    // deliberately discards that geometry and measures the NEW direction.
    void StartPostAutoHoldLeftEdge(Vector3 newHeading, bool fastArrival = false,
        bool universalInCorner = false)
    {
        StopEdgeCruise("new edge-approach turn");
        ClearPostAutoHoldLeftEdge();
        postAutoHoldFastArrival = fastArrival;
        postAutoHoldUniversalInCorner = universalInCorner;
        postAutoHoldHeading = NormalizeFlat(newHeading, direction);
        postAutoHoldEdgeActive = postAutoHoldHeading.sqrMagnitude > Eps;
        postAutoHoldStartedAt = Time.fixedTime;
        postAutoHoldNextProbeTime = 0f;
        if (postAutoHoldEdgeActive)
            AutoHoldLog("NEXT SLOPE EDGE", fastArrival
                ? "armed after SAME FLAT tap turn; waiting new guide"
                : universalInCorner
                    ? "InCorner target locked by normal UTurn; speed-independent"
                    : "armed after Auto Hold turn; waiting new guide");
    }

    void ClearPostAutoHoldLeftEdge()
    {
        postAutoHoldEdgeActive = false;
        postAutoHoldNewGuide = false;
        postAutoHoldSawSlope = false;
        postAutoHoldHadEdgeSample = false;
        postAutoHoldCorrectionActive = false;
        postAutoHoldFastArrival = false;
        postAutoHoldUniversalInCorner = false;
        postAutoHoldFastDeadlineLogged = false;
        postAutoHoldFastFlatPrescanLogged = false;
        postAutoHoldUniversalFallbackLogged = false;
        postAutoHoldFastArrivedBeforeThird = false;
        postAutoHoldFastInfeasibleLogged = false;
        postAutoHoldHeading = Vector3.zero;
        postAutoHoldEdgeTarget = Vector3.zero;
        postAutoHoldNextProbeTime = 0f;
        postAutoHoldNextFlatProbeFailureLogTime = 0f;
        postAutoHoldLastError = 0f;
        postAutoHoldPeakSlopeError = 0f;
        postAutoHoldMaxSlopeProgress = 0f;
        postAutoHoldSlopeSamples = 0;
        postAutoHoldCorrections = 0;
        postBEdgeAnchorValid = false;
        postBEdgeAnchorLateral = 0f;
        postBEdgeReacquireUntil = -1f;
        postBEdgeNextLogTime = 0f;
    }

    void FinishPostAutoHoldLeftEdge(string reason, bool completedSlope)
    {
        if (!postAutoHoldEdgeActive) return;
        float tolerance = ProbeRadius * .20f;
        bool succeeded = completedSlope && postAutoHoldSawSlope &&
            postAutoHoldSlopeSamples > 0 &&
            postAutoHoldHadEdgeSample &&
            Mathf.Abs(postAutoHoldLastError) <= tolerance;
        AutoHoldLog(succeeded ? "NEXT SLOPE SUCCESS" : "NEXT SLOPE FAILED",
            $"reason={reason} finalError={postAutoHoldLastError:F3}m " +
            $"peakSlopeError={postAutoHoldPeakSlopeError:F3}m " +
            $"samples={postAutoHoldSlopeSamples} corrections={postAutoHoldCorrections}");
        // Completed next-slope approach is the ONLY way to arm continuing
        // edge protection. A failure or a support loss never arms it.
        if (succeeded)
            StartEdgeCruise(postAutoHoldHeading);
        ClearPostAutoHoldLeftEdge();
    }

    // Reuse the exact same Raycast/SphereCast edge solver as pre-turn Auto Hold.
    // Here the cross-section comes from the NEW current guide, not the old slope.
    bool TryResolvePostAutoHoldLeftEdge(
        NearestKnotDetector.GuideFrame guide, Surface surface,
        Vector3 left, out Vector3 target, out string failureReason)
    {
        target = rb.position;
        failureReason = "invalid_guide_or_surface";
        if (!guide.valid || !surface.Valid || !knotDetector)
            return false;

        Vector3 forward = NormalizeFlat(guide.tangent, postAutoHoldHeading);
        if (Vector3.Dot(forward, postAutoHoldHeading) < .75f)
        {
            failureReason = "new_guide_direction_mismatch";
            return false;
        }

        // For a same-flat tap+flick, seek the PHYSICAL lip on the first new
        // supported flat, before crossing into the stair. The ball's centre
        // is already above the slab. Adding another radius to rb.position
        // made the final SphereCast miss the flat at the default probeDistance.
        if ((postAutoHoldFastArrival || postAutoHoldUniversalInCorner) &&
            !guide.isSlope)
        {
            var flatSupport = HasGroundSupport();
            if (!flatSupport.Item1 || !flatSupport.Item2.collider)
                failureReason = "no_physical_flat_support";
            else if (!IsAutoHoldStageHit(flatSupport.Item2, float.NaN))
                failureReason = "non_stage_support";
            else if (Vector3.Angle(flatSupport.Item2.normal, Vector3.up) > 15f)
                failureReason = "contact_not_horizontal";
            else
            {
                Bounds bounds = flatSupport.Item2.collider.bounds;
                float projectedWidth = 2f *
                    (Mathf.Abs(left.x) * bounds.extents.x +
                     Mathf.Abs(left.z) * bounds.extents.z);
                float scanRange = Mathf.Clamp(projectedWidth + ProbeRadius * 4f,
                    ProbeRadius * 4f, ProbeRadius * 40f);
                // Contact point is at the actual floor elevation; never
                // add the probe lift to the already elevated Rigidbody centre.
                Vector3 flatProbe = rb.position;
                flatProbe.y = flatSupport.Item2.point.y + ProbeRadius + .25f;
                if (TryScanAutoHoldLeftEdge(flatProbe, left, scanRange,
                        out target, out string scanFailureStage, true))
                {
                    if (!postAutoHoldFastFlatPrescanLogged)
                    {
                        postAutoHoldFastFlatPrescanLogged = true;
                        AutoHoldLog("FAST EDGE EARLY FLAT SAMPLE",
                            $"collider={flatSupport.Item2.collider.name} " +
                            $"error={Vector3.Dot(target - rb.position, left):F3}m " +
                            $"scanRange={scanRange:F2}m " +
                            $"floorY={flatSupport.Item2.point.y:F3} probeY={flatProbe.y:F3} " +
                            $"verification={(scanFailureStage ?? "sphere")} ");
                    }
                    failureReason = null;
                    return true;
                }
                failureReason = "flat_physical_edge_scan_failed_" + scanFailureStage;
            }
        }

        // Preserve the existing FiveLine-guided physical fallback for slopes,
        // seams, and flats whose own physical lip cannot be confirmed.
        if (!knotDetector.TryGetFiveLineFrame(guide, surface.side,
                out NearestKnotDetector.FiveLineFrame frame) || !frame.valid)
        {
            if (failureReason == "invalid_guide_or_surface")
                failureReason = "five_line_frame_unavailable";
            return false;
        }

        float width = Mathf.Abs(Vector3.Dot(frame.left - frame.right, left));
        if (width <= ProbeRadius)
        {
            if (failureReason == "invalid_guide_or_surface")
                failureReason = "five_line_width_too_small";
            return false;
        }

        float along = Mathf.Clamp(Vector3.Dot(rb.position - frame.center,
            forward), -ProbeRadius * 2f, ProbeRadius * 2f);
        Vector3 probe = frame.center + forward * along;
        probe.y += ProbeRadius + .25f;
        float range = Mathf.Max(ProbeRadius * 4f, width + ProbeRadius * 4f);
        if (!TryScanAutoHoldLeftEdge(probe, left, range,
                out target, out _, true))
        {
            if (postAutoHoldUniversalInCorner)
            {
                // Use the physical LeftSpline as the minimum goal, but only
                // when it has actual support. The inward margin keeps the
                // sphere centre off the mathematically exact outer lip.
                Vector3 leftSpline =
                    Vector3.Dot(frame.left - frame.center, left) >
                    Vector3.Dot(frame.right - frame.center, left)
                        ? frame.left : frame.right;
                Vector3 towardCenter = Vector3.ProjectOnPlane(
                    frame.center - leftSpline, Vector3.up);
                if (towardCenter.sqrMagnitude > Eps)
                {
                    target = leftSpline + towardCenter.normalized *
                        (ProbeRadius * .65f);
                    Vector3 sample = target + Vector3.up *
                        (ProbeRadius * .45f);
                    if (TryAutoHoldGroundRay(sample, out RaycastHit safe) &&
                        IsCloseGroundRay(safe, sample))
                    {
                        failureReason = null;
                        if (!postAutoHoldUniversalFallbackLogged)
                        {
                            postAutoHoldUniversalFallbackLogged = true;
                            Debug.Log($"[INCORNER LEFTSPLINE FALLBACK] " +
                                $"t={Time.fixedTime:F3} left={leftSpline:F3} " +
                                $"target={target:F3} floor={safe.collider.name}", this);
                        }
                        return true;
                    }
                }
                target = rb.position;
            }
            if (failureReason == "invalid_guide_or_surface")
                failureReason = "five_line_physical_edge_scan_failed";
            return false;
        }
        failureReason = null;
        return true;
    }

    // ================================================================
    // Edge cruise: safety after the completed left-edge approach
    // ================================================================
    void StartEdgeCruise(Vector3 heading)
    {
        StopEdgeCruise("replaced");
        edgeCruiseHeading = NormalizeFlat(heading, direction);
        edgeCruiseActive = edgeCruiseHeading.sqrMagnitude > Eps;
        edgeCruiseNextProbeTime = 0f;
        edgeCruiseNextLogTime = 0f;
        if (edgeCruiseActive)
            AutoHoldLog("EDGE CRUISE START",
                "next slope completed; maintaining verified physical lip clearance");
    }

    void StopEdgeCruise(string reason)
    {
        if (edgeCruiseActive)
            AutoHoldLog("EDGE CRUISE STOP",
                $"reason={reason} corrections={edgeCruiseCorrections}");
        edgeCruiseActive = false;
        edgeCruiseSampleValid = false;
        edgeCruiseHeading = Vector3.zero;
        edgeCruiseTarget = Vector3.zero;
        edgeCruiseSupportCollider = null;
        edgeCruiseNextProbeTime = 0f;
        edgeCruiseSampleTime = 0f;
        edgeCruiseNextLogTime = 0f;
        edgeCruiseCorrections = 0;
    }

    float StepEdgeCruise(NearestKnotDetector.GuideFrame guide,
        Surface surface, bool grounded, RaycastHit supportHit)
    {
        if (!edgeCruiseActive) return 0f;
        if (MainGameManager.OnDead || !BeginCommandOnTouch ||
            HasPendingTurnIntent || turnTransitionActive || waitingForTurnGuide ||
            earlyFlickSession || autoHoldEligible || postAutoHoldEdgeActive)
        {
            StopEdgeCruise("player action or turn");
            return 0f;
        }
        if (!guide.valid || !surface.Valid) return 0f;
        Vector3 forward = NormalizeFlat(guide.tangent, edgeCruiseHeading);
        if (Vector3.Dot(forward, edgeCruiseHeading) < .80f)
        {
            StopEdgeCruise("spline heading changed");
            return 0f;
        }

        // Do not suspend gravity or steer an unsupported body. The normal
        // support/DropOut logic remains authoritative, even in edge cruise.
        if (!grounded || !IsAutoHoldStageHit(supportHit, float.NaN))
        {
            edgeCruiseSampleValid = false;
            return 0f;
        }

        Vector3 left = Vector3.Cross(edgeCruiseHeading, Vector3.up).normalized;
        if (left.sqrMagnitude <= Eps) return 0f;
        float vLeft = Vector3.Dot(rb.velocity, left);
        if (Time.fixedTime >= edgeCruiseNextProbeTime ||
            (edgeCruiseSupportCollider &&
             supportHit.collider != edgeCruiseSupportCollider))
        {
            Bounds bounds = supportHit.collider.bounds;
            float width = 2f * (Mathf.Abs(left.x) * bounds.extents.x +
                Mathf.Abs(left.z) * bounds.extents.z);
            float range = Mathf.Clamp(width + ProbeRadius * 4f,
                ProbeRadius * 4f, ProbeRadius * 40f);
            Vector3 probe = rb.position;
            probe.y = supportHit.point.y + ProbeRadius + .25f;

            if (TryScanAutoHoldLeftEdge(probe, left, range,
                    out Vector3 lipTarget, out _, true))
            {
                // Leave slightly more than one full radius inside the lip.
                // Validated against the actual floor, not the Spline width.
                Vector3 safeTarget = lipTarget - left * (ProbeRadius * .24f);
                Vector3 check = probe + left *
                    Vector3.Dot(safeTarget - rb.position, left);
                if (TryAutoHoldGroundRay(check, out RaycastHit safeHit,
                        supportHit.point.y) &&
                    safeHit.collider == supportHit.collider &&
                    IsAutoHoldStageHit(safeHit, supportHit.point.y))
                {
                    edgeCruiseTarget = safeTarget;
                    edgeCruiseSupportCollider = supportHit.collider;
                    edgeCruiseSampleValid = true;
                    edgeCruiseSampleTime = Time.fixedTime;
                }
                else
                    edgeCruiseSampleValid = false;
            }
            else
                edgeCruiseSampleValid = false;

            // Only sample every second physics tick unless outward drift is
            // already significant. No unbounded raycasts every Update frame.
            edgeCruiseNextProbeTime = Time.fixedTime +
                (vLeft > 1f ? Time.fixedDeltaTime :
                    Mathf.Max(Time.fixedDeltaTime, Time.fixedDeltaTime * 2f));
        }

        // A cached lip may only be used on the SAME supported collider, for
        // at most two physics ticks. Across seams, reacquire the new floor.
        bool fresh = edgeCruiseSampleValid &&
            supportHit.collider == edgeCruiseSupportCollider &&
            Time.fixedTime - edgeCruiseSampleTime <=
                Mathf.Max(.04f, Time.fixedDeltaTime * 2.5f);
        if (!fresh)
        {
            if (Time.fixedTime >= edgeCruiseNextLogTime)
            {
                edgeCruiseNextLogTime = Time.fixedTime + .40f;
                AutoHoldLog("EDGE CRUISE PROBE UNAVAILABLE",
                    $"collider={supportHit.collider.name} supported={grounded} " +
                    "no stale-lip force across stage seams");
            }
            return 0f;
        }

        float error = Vector3.Dot(edgeCruiseTarget - rb.position, left);
        // Positive vLeft means moving toward the outside of the left lip.
        // Predict the stopping distance before the ball touches the boundary.
        float maxAcceleration = Mathf.Clamp(Mathf.Max(
            autoHoldMaxSideAcceleration, GroundAcceleration * 2.5f), 45f, 120f);
        float stoppingDistance = vLeft > 0f
            ? vLeft * vLeft / (2f * maxAcceleration) : 0f;
        float predictedError = error - vLeft *
            Mathf.Max(.05f, Time.fixedDeltaTime * 3f);
        bool nearOutside = error <= ProbeRadius * .25f + stoppingDistance +
            Mathf.Max(0f, vLeft) * Time.fixedDeltaTime;

        // Gentle parallel running at the lip, with stronger braking ONLY when
        // outward velocity predicts an overrun. Normal forward speed is untouched.
        float targetSideSpeed = Mathf.Clamp(error *
            (nearOutside ? 8f : 3.5f), -2.5f, 1.1f);
        if (nearOutside && vLeft > .4f)
            targetSideSpeed = Mathf.Min(targetSideSpeed, -Mathf.Min(2f, vLeft));
        float accelLeft = Mathf.Clamp((targetSideSpeed - vLeft) * 12f,
            -maxAcceleration, maxAcceleration);
        if (nearOutside && vLeft > 0f)
            accelLeft = Mathf.Min(accelLeft,
                -Mathf.Min(maxAcceleration, vLeft * 25f));

        if (Mathf.Abs(accelLeft) <= .01f) return 0f;
        edgeCruiseCorrections++;
        if (Time.fixedTime >= edgeCruiseNextLogTime)
        {
            edgeCruiseNextLogTime = Time.fixedTime + .40f;
            AutoHoldLog("EDGE CRUISE GUARD",
                $"error={error:F3}m outwardSpeed={vLeft:F2}m/s " +
                $"predicted={predictedError:F3}m accel={accelLeft:F1}m/s2 " +
                $"collider={supportHit.collider.name} slope={guide.isSlope}");
        }
        // Lateral force only. Never teleport, clamp world position, freeze Y,
        // add invisible walls, or override the existing death logic.
        return accelLeft * Vector3.Dot(left, surface.side);
    }

    // A latched physical left-lip target is not changed when NearestKnot
    // switches to another Spline lane of the same stair. Revalidate it against
    // the ACTUAL supporting stair collider at this forward position first.
    bool IsPostBEdgeAnchorStillSupported(Vector3 left, float signedLateral)
    {
        if (!postBDownhillSawSlope || postBDownhillStairFamily < 0 ||
            !currentHit.collider || !IsAutoHoldStageHit(currentHit, float.NaN) ||
            !TryGetPostBStairFamily(currentHit.collider, out int family) ||
            family != postBDownhillStairFamily)
            return false;

        Vector3 probe = rb.worldCenterOfMass + left *
            (signedLateral - Vector3.Dot(rb.worldCenterOfMass, left));
        probe.y = currentHit.point.y + ProbeRadius + .25f;
        if (!TryAutoHoldGroundRay(probe, out RaycastHit check,
                currentHit.point.y) ||
            !TryGetPostBStairFamily(check.collider, out int checkedFamily) ||
            checkedFamily != family ||
            !TryAutoHoldVerifiedFootprint(probe, check.point.y, -left,
                out _))
            return false;
        return true;
    }

    void StabilizePostBLeftEdgeTarget(Vector3 left)
    {
        if (!postBDownhillSawSlope || !currentHit.collider) return;
        float measured = Vector3.Dot(postAutoHoldEdgeTarget, left);
        if (!postBEdgeAnchorValid)
        {
            postBEdgeAnchorLateral = measured;
            postBEdgeAnchorValid = true;
        }
        else
        {
            float jump = Mathf.Abs(measured - postBEdgeAnchorLateral);
            if (jump > PostBEdgeAnchorNoiseMeters)
            {
                if (IsPostBEdgeAnchorStillSupported(left,
                        postBEdgeAnchorLateral))
                {
                    // The original lip is still physically present. Ignore
                    // a 2.5m lane/section snap in the Guide and do not chase.
                    if (Time.fixedTime >= postBEdgeNextLogTime)
                    {
                        postBEdgeNextLogTime = Time.fixedTime + .25f;
                        Debug.Log($"[B EDGE TARGET HELD] " +
                            $"t={Time.fixedTime:F3} guideJump={jump:F2}m " +
                            $"floor={currentHit.collider.name}", this);
                    }
                }
                else
                {
                    // A newly VERIFIED physical lip is available, but the
                    // old lip disappeared. Slew the reference and BRAKE
                    // sideways during reacquisition, never jump the target.
                    postBEdgeAnchorLateral = Mathf.MoveTowards(
                        postBEdgeAnchorLateral, measured,
                        PostBEdgeAnchorSlewMetersPerSecond *
                        Time.fixedDeltaTime);
                    postBEdgeReacquireUntil = Time.fixedTime + .06f;
                    if (Time.fixedTime >= postBEdgeNextLogTime)
                    {
                        postBEdgeNextLogTime = Time.fixedTime + .25f;
                        Debug.LogWarning($"[B EDGE TARGET REACQUIRE] " +
                            $"t={Time.fixedTime:F3} guideJump={jump:F2}m " +
                            $"floor={currentHit.collider.name}", this);
                    }
                }
            }
            else
            {
                postBEdgeAnchorLateral = Mathf.MoveTowards(
                    postBEdgeAnchorLateral, measured,
                    PostBEdgeAnchorSlewMetersPerSecond * Time.fixedDeltaTime);
            }
        }
        postAutoHoldEdgeTarget = rb.position + left *
            (postBEdgeAnchorLateral - Vector3.Dot(rb.position, left));
    }

    // B-InCorner exclusively: bounded side acceleration, velocity-based
    // stopping distance, no side push while re-locating the physical lip.
    // No X/Z snap, no force along the downhill tangent, no new MonoBehaviour.
    float StepPostBSafeLateral(Vector3 left, Surface surface,
        float error, float lateralVelocity, bool targetVerified)
    {
        float desiredSpeed = 0f;
        float tolerance = ProbeRadius * .20f;
        if (targetVerified && Time.fixedTime >= postBEdgeReacquireUntil)
        {
            float travelToGoal = Mathf.Max(0f, Mathf.Abs(error) - tolerance);
            float towardSpeed = Mathf.Sign(error) * lateralVelocity;
            float stopDistance = towardSpeed > 0f
                ? towardSpeed * towardSpeed /
                    (2f * PostBEdgeMaxSideAcceleration) : 0f;

            if (stopDistance + Mathf.Max(0f, towardSpeed) *
                    Time.fixedDeltaTime >= travelToGoal)
                desiredSpeed = 0f; // begin BRAKING before overshoot
            else
                desiredSpeed = Mathf.Sign(error) * Mathf.Min(
                    PostBEdgeMaxSideSpeed,
                    Mathf.Min(Mathf.Sqrt(2f * PostBEdgeMaxSideAcceleration *
                        travelToGoal), travelToGoal * 6f));
        }
        float requestedAcceleration = (desiredSpeed - lateralVelocity) * 16f;
        float accelerationLeft = Mathf.Clamp(requestedAcceleration,
            -PostBEdgeMaxSideAcceleration, PostBEdgeMaxSideAcceleration);
        if (Time.fixedTime >= postBEdgeNextLogTime &&
            (Mathf.Abs(lateralVelocity) > 2f ||
             Mathf.Abs(error) > ProbeRadius))
        {
            postBEdgeNextLogTime = Time.fixedTime + .18f;
            Debug.Log($"[B EDGE LATERAL BRAKE] " +
                $"t={Time.fixedTime:F3} error={error:F2}m " +
                $"side={lateralVelocity:F2}m/s " +
                $"target={desiredSpeed:F2}m/s " +
                $"sideAccel={accelerationLeft:F1} " +
                $"verified={targetVerified}", this);
        }
        return accelerationLeft * Vector3.Dot(left, surface.side);
    }

    // Only Auto Hold's immediately following flat -> slope is eligible.
    // A correct edge position gets no steering. A measurable error gets only
    // existing lateral AddForce, never a new hard teleport into stair colliders.
    float StepPostAutoHoldLeftEdge(
        NearestKnotDetector.GuideFrame guide, Surface surface, bool grounded)
    {
        if (!postAutoHoldEdgeActive) return 0f;
        if (MainGameManager.OnDead)
        {
            FinishPostAutoHoldLeftEdge("player died", false);
            return 0f;
        }
        if (!BeginCommandOnTouch || HasPendingTurnIntent ||
            turnTransitionActive || waitingForTurnGuide ||
            earlyFlickSession || autoHoldEligible)
        {
            AutoHoldLog("NEXT SLOPE CANCELED", "new player action / inactive");
            ClearPostAutoHoldLeftEdge();
            return 0f;
        }
        if (!postAutoHoldNewGuide || !guide.valid || !surface.Valid)
            return 0f;
        if (!grounded)
        {
            if (postAutoHoldFastArrival && guide.isSlope &&
                !postAutoHoldFastDeadlineLogged && guide.sectionProgress01 >= 3f / 7f)
            {
                postAutoHoldFastDeadlineLogged = true;
                AutoHoldLog("FAST EDGE STEP 3 UNSUPPORTED",
                    $"progress={guide.sectionProgress01:F3} no physical support");
            }
            return 0f; // never steer toward a lip while unsupported
        }

        Vector3 heading = NormalizeFlat(guide.tangent, postAutoHoldHeading);
        if (Vector3.Dot(heading, postAutoHoldHeading) < .75f)
        {
            FinishPostAutoHoldLeftEdge("different travel direction", false);
            return 0f;
        }
        float elapsed = Time.fixedTime - postAutoHoldStartedAt;
        if (elapsed > (postAutoHoldSawSlope ? 12f : 6f))
        {
            FinishPostAutoHoldLeftEdge(postAutoHoldSawSlope
                ? "next slope not completed" : "next slope not reached", false);
            return 0f;
        }

        if (guide.isSlope)
        {
            if (!postAutoHoldSawSlope)
            {
                postAutoHoldSawSlope = true;
                postAutoHoldNextProbeTime = 0f;
                AutoHoldLog("NEXT SLOPE EDGE",
                    $"slope entered section={guide.sectionIndex}");
            }
            postAutoHoldMaxSlopeProgress = Mathf.Max(
                postAutoHoldMaxSlopeProgress, guide.sectionProgress01);
        }
        else if (postAutoHoldSawSlope)
        {
            // On the B-InCorner receiving stair a flat GUIDE can occur at
            // a knot/step seam while the real collider is still StairWay.
            // The downhill guard, not the guide flag, confirms ArcSlab arrival.
            if (postBDownhillArmed && postAutoHoldUniversalInCorner)
                return 0f;
            if (postAutoHoldMaxSlopeProgress >= .65f)
                FinishPostAutoHoldLeftEdge("next slope exited", true);
            return 0f;
        }
        // A same-flat tap+flick may start from any VERIFIED new flat after
        // the turn, instead of waiting for nextIsSlope at the last seam.
        if (!postAutoHoldFastArrival && !postAutoHoldUniversalInCorner &&
            !guide.isSlope && !guide.nextIsSlope)
            return 0f;

        Vector3 left = Vector3.Cross(postAutoHoldHeading, Vector3.up).normalized;
        if (left.sqrMagnitude <= Eps) return 0f;

        // At high forward speed sample every two physics ticks; a new slope
        // triggers immediate resampling. Reuse only within the same guide span.
        if (Time.fixedTime >= postAutoHoldNextProbeTime)
        {
            postAutoHoldHadEdgeSample = TryResolvePostAutoHoldLeftEdge(
                guide, surface, left, out postAutoHoldEdgeTarget,
                out string probeFailureReason);
            postAutoHoldNextProbeTime = Time.fixedTime +
                Mathf.Max(Time.fixedDeltaTime, Time.fixedDeltaTime * 2f);
            if (postAutoHoldHadEdgeSample && postBDownhillArmed &&
                postAutoHoldUniversalInCorner &&
                postBDownhillSawSlope && guide.isSlope)
                StabilizePostBLeftEdgeTarget(left);
            // Only failed early-flat probes are logged, throttled to avoid
            // flooding the console at physics tick frequency.
            if (!postAutoHoldHadEdgeSample && postAutoHoldFastArrival &&
                !guide.isSlope && Time.fixedTime >= postAutoHoldNextFlatProbeFailureLogTime)
            {
                postAutoHoldNextFlatProbeFailureLogTime = Time.fixedTime + .16f;
                AutoHoldLog("FAST EDGE EARLY FLAT PROBE FAILED",
                    $"reason={probeFailureReason} " +
                    $"support={grounded} guideFlat={!guide.isSlope} " +
                    $"collider={(currentHit.collider ? currentHit.collider.name : "-")} " +
                    $"speed={Vector3.Dot(rb.velocity, postAutoHoldHeading):F2}m/s");
            }
        }
        if (!postAutoHoldHadEdgeSample)
        {
            if (postBDownhillArmed && postAutoHoldUniversalInCorner)
                return StepPostBSafeLateral(left, surface, 0f,
                    Vector3.Dot(rb.velocity, left), false);
            if (postAutoHoldFastArrival && guide.isSlope &&
                !postAutoHoldFastDeadlineLogged && guide.sectionProgress01 >= 3f / 7f)
            {
                postAutoHoldFastDeadlineLogged = true;
                AutoHoldLog("FAST EDGE STEP 3 UNAVAILABLE",
                    $"progress={guide.sectionProgress01:F3} no verified edge");
            }
            return 0f;
        }

        float error = Vector3.Dot(postAutoHoldEdgeTarget - rb.position, left);
        float tolerance = ProbeRadius * .20f;
        float lateralVelocity = Vector3.Dot(rb.velocity, left);
        // If the ball is on the lip but moving outward, do not wait until
        // the next tick to begin braking. Otherwise a good edge gets no force.
        float predictedError = error - lateralVelocity *
            (Time.fixedDeltaTime * 2f);
        bool needsCorrection = Mathf.Abs(error) > tolerance ||
            Mathf.Abs(predictedError) > tolerance;
        postAutoHoldLastError = error;
        // Three treads are the DEADLINE, not the start of steering.
        // Count only a measured arrival before that line; confirm it remains
        // at the lip when the third-tread boundary is crossed.
        bool arrived = Mathf.Abs(error) <= tolerance &&
            Mathf.Abs(predictedError) <= tolerance;
        if (postAutoHoldFastArrival && !postAutoHoldFastArrivedBeforeThird &&
            arrived && (!guide.isSlope || guide.sectionProgress01 < 3f / 7f))
        {
            postAutoHoldFastArrivedBeforeThird = true;
            AutoHoldLog("FAST EDGE EARLY ARRIVAL",
                $"slope={guide.isSlope} progress={guide.sectionProgress01:F3} " +
                $"error={error:F3}m lateral={lateralVelocity:F2}m/s");
        }
        if (postAutoHoldFastArrival && guide.isSlope &&
            !postAutoHoldFastDeadlineLogged && guide.sectionProgress01 >= 3f / 7f)
        {
            postAutoHoldFastDeadlineLogged = true;
            AutoHoldLog(postAutoHoldFastArrivedBeforeThird && arrived
                    ? "FAST EDGE STEP 3 SUCCESS" : "FAST EDGE STEP 3 MISSED",
                $"progress={guide.sectionProgress01:F3} error={error:F3}m " +
                $"lateral={lateralVelocity:F2}m/s predictedError={predictedError:F3}m " +
                $"early={postAutoHoldFastArrivedBeforeThird} " +
                $"corrections={postAutoHoldCorrections}");
        }
        if (postAutoHoldSawSlope)
        {
            postAutoHoldSlopeSamples++;
            postAutoHoldPeakSlopeError = Mathf.Max(postAutoHoldPeakSlopeError,
                Mathf.Abs(error));
        }

        if (postBDownhillArmed && postAutoHoldUniversalInCorner)
        {
            // Original B-InCorner only: do not apply the 220m/s2 universal
            // quintic controller. The slow 5m/s side target is based on the
            // PHYSICAL lip, with braking distance and measured side speed.
            if (needsCorrection || Mathf.Abs(lateralVelocity) > .15f)
                postAutoHoldCorrections++;
            return StepPostBSafeLateral(left, surface, error,
                lateralVelocity, true);
        }
        if (!needsCorrection)
        {
            postAutoHoldCorrectionActive = false;
            return 0f;
        }
        if (!postAutoHoldCorrectionActive)
        {
            AutoHoldLog("NEXT SLOPE CORRECT",
                $"error={error:F3}m slope={guide.isSlope} collider=" +
                (autoHoldEdgeCollider ? autoHoldEdgeCollider.name : "-"));
            postAutoHoldCorrectionActive = true;
        }
        postAutoHoldCorrections++;

        // Same-flat tap + turn: a physically bounded trajectory controller.
        // Never demand 700-1200 m/s2 from the ball just because only a few
        // centimetres of the stair remain. If the deadline is impossible,
        // report it and still steer as safely and quickly as the caps permit.
        if (postAutoHoldFastArrival)
        {
            const float maxFastSideSpeed = 24f;        // m/s
            const float maxFastSideAcceleration = 220f; // m/s2
            const float earlyTreadProgress = 2.5f / 7f;
            float remaining = guide.isSlope
                ? Mathf.Max(0f, (earlyTreadProgress - guide.sectionProgress01) *
                    Mathf.Max(0f, guide.sectionLength))
                : Mathf.Max(0f, guide.distanceToNextSlope) +
                    earlyTreadProgress * 9.9f;
            float forward = Mathf.Max(1f,
                Vector3.Dot(rb.velocity, postAutoHoldHeading));
            float budget = Mathf.Max(Time.fixedDeltaTime * 2f,
                remaining / forward * .80f);

            float absError = Mathf.Abs(error);
            float initialPeakSpeed = Mathf.Sqrt(
                maxFastSideAcceleration * absError);
            float minimumTimeFromRest = initialPeakSpeed <= maxFastSideSpeed
                ? 2f * initialPeakSpeed / maxFastSideAcceleration
                : 2f * maxFastSideSpeed / maxFastSideAcceleration +
                  (absError - maxFastSideSpeed * maxFastSideSpeed /
                   maxFastSideAcceleration) / maxFastSideSpeed;

            if (postAutoHoldCorrections == 1)
            {
                AutoHoldLog("FAST EDGE BEFORE STEP 3 PLAN",
                    $"error={error:F2}m remaining={remaining:F2}m " +
                    $"budget={budget:F3}s minFromRest={minimumTimeFromRest:F3}s " +
                    $"speedCap={maxFastSideSpeed:F1}m/s " +
                    $"accelCap={maxFastSideAcceleration:F0}m/s2 " +
                    $"feasibleFromRest={minimumTimeFromRest <= budget}");
            }
            if (!postAutoHoldFastInfeasibleLogged &&
                minimumTimeFromRest > budget * 1.05f)
            {
                postAutoHoldFastInfeasibleLogged = true;
                AutoHoldLog("FAST EDGE TIME BUDGET RISK",
                    $"fromRestEstimate={minimumTimeFromRest:F3}s " +
                    $"available={budget:F3}s error={error:F2}m " +
                    $"initialLateral={lateralVelocity:F2}m/s " +
                    "bounded acceleration; estimate assumes zero initial lateral speed");
            }

            // The stop distance governs WHEN to start braking; a tiny
            // position error alone is not a sufficient arrival condition.
            float speedToward = Mathf.Sign(error) * lateralVelocity;
            float stoppingDistance = speedToward > 0f
                ? speedToward * speedToward / (2f * maxFastSideAcceleration)
                : 0f;
            float speedByDistance = Mathf.Sqrt(
                2f * maxFastSideAcceleration * absError);
            float speedByTime = absError / Mathf.Max(.12f, budget) * 1.4f;
            float targetSpeed = Mathf.Min(maxFastSideSpeed,
                Mathf.Min(speedByDistance, speedByTime));
            float desiredLateralSpeed = Mathf.Abs(error) <= tolerance
                ? 0f : Mathf.Sign(error) * targetSpeed;
            float speedGain = Mathf.Clamp(4f / Mathf.Max(.12f, budget),
                10f, 26f);
            float accelerationLeft = (desiredLateralSpeed - lateralVelocity) *
                speedGain;
            if (speedToward > 0f &&
                stoppingDistance >= Mathf.Max(0f, absError - tolerance))
                accelerationLeft = -Mathf.Sign(lateralVelocity) *
                    maxFastSideAcceleration;

            // The existing surface.side is the ONLY force output channel.
            return Mathf.Clamp(
                accelerationLeft * Vector3.Dot(left, surface.side),
                -maxFastSideAcceleration, maxFastSideAcceleration);
        }

        if (postAutoHoldUniversalInCorner)
        {
            // Receding-horizon quintic from measured position/side speed to
            // the confirmed *physical* InCorner lip: e(T)=v(T)=a(T)=0.
            // Unlike selecting the nearest FiveLine, this target is not
            // changed to Center/Out just because acceleration is high.
            const float maxSideAcceleration = 220f;
            float maxSideSpeed = Mathf.Max(autoHoldMaxSideSpeed,
                maxGroundSpeed * 0.75f);
            float forward = Mathf.Max(1f,
                Vector3.Dot(rb.velocity, postAutoHoldHeading));
            float distance = guide.isSlope
                ? Mathf.Max(0f, guide.sectionLength *
                    (1f - guide.sectionProgress01))
                : guide.nextIsSlope
                    ? Mathf.Max(0f, guide.distanceToNextSlope) + 3f
                    : Mathf.Max(1.5f, maxGroundSpeed * .25f);
            float horizon = Mathf.Clamp(distance / forward, .12f, 1.2f);
            float v0 = lateralVelocity;
            float T2 = horizon * horizon;
            float T3 = T2 * horizon;
            float T4 = T3 * horizon;
            float T5 = T4 * horizon;
            float c3 = (10f * error - 6f * v0 * horizon) / T3;
            float c4 = (-15f * error + 8f * v0 * horizon) / T4;
            float c5 = (6f * error - 3f * v0 * horizon) / T5;
            float look = Mathf.Min(Time.fixedDeltaTime * 3f, horizon * .25f);
            float l2 = look * look;
            float l3 = l2 * look;
            float l4 = l3 * look;
            float targetSpeed = v0 + 3f * c3 * l2 +
                4f * c4 * l3 + 5f * c5 * l4;
            targetSpeed = Mathf.Clamp(targetSpeed,
                -maxSideSpeed, maxSideSpeed);

            // Stop distance is a hard safety constraint when the remaining
            // horizon is physically infeasible. Never apply unbounded force.
            float toward = Mathf.Sign(error) * v0;
            float stopping = toward > 0f
                ? toward * toward / (2f * maxSideAcceleration) : 0f;
            if (stopping >= Mathf.Max(0f, Mathf.Abs(error) - tolerance))
                targetSpeed = 0f;
            if (Mathf.Abs(error) <= tolerance)
                targetSpeed = 0f;
            float accel = Mathf.Clamp(
                (targetSpeed - v0) / Mathf.Max(look, Time.fixedDeltaTime),
                -maxSideAcceleration, maxSideAcceleration);
            return accel * Vector3.Dot(left, surface.side);
        }

        bool largeError = Mathf.Abs(error) > ProbeRadius;
        float maxSpeed = largeError
            ? Mathf.Max(autoHoldMaxSideSpeed, maxGroundSpeed * .25f)
            : autoHoldMaxSideSpeed;
        float maxAccel = largeError
            ? Mathf.Max(autoHoldMaxSideAcceleration, GroundAcceleration * 3f)
            : autoHoldMaxSideAcceleration;
        const float response = 12f;
        const float positionGain = 8f;
        // Auto Hold's ordinary slow approach remains unchanged.
        float brakingSpeed = Mathf.Sqrt(2f * maxAccel * Mathf.Abs(error));
        float desired = Mathf.Abs(error) <= tolerance ? 0f :
            Mathf.Sign(error) * Mathf.Min(maxSpeed,
                Mathf.Min(Mathf.Abs(error) * positionGain, brakingSpeed));
        float desiredSideSpeed = Vector3.Dot(left * desired, surface.side);
        return Mathf.Clamp((desiredSideSpeed - surface.lateralSpeed) * response,
            -maxAccel, maxAccel);
    }

    // Continuous natural approach on the existing AddForce lateral channel.
    // The flat-entry time and lateral braking distance provide the target speed;
    // the existing Inspector speed/acceleration remain its upper bounds.
    float StepAutoHoldInnerSteering(Surface surface, bool grounded)
    {
        // A pressed gesture may begin on EITHER side. However, left-lip
        // steering is an InCorner-specific assist, not a requirement for
        // authorizing the turn. Never pull a right-half OutCorner into the
        // left lip merely because its hold started on a stair.
        // No guessed left-lip approach while the next turn direction is unknown.
        // Guessing left can damage an OutCorner even before the finger is released.
        bool innerSteeringEligible = autoHoldPredictedTurnSignValid &&
            autoHoldPredictedTurnSign < 0f;
        bool held = autoHoldEligible && autoHoldSawSlope &&
            !autoHoldNextFlatReady && // no forced edge chase while choosing release time
            autoHoldLastHalfValid && innerSteeringEligible &&
            IsAutoHoldInputStillDown();
        bool innerTurnPending = autoHoldTurnPending && autoHoldTurnSign < 0f;
        if (!grounded || !surface.Valid || (!held && !innerTurnPending))
            return 0f;
        // On the next verified flat we can resolve the upcoming turn from
        // the actual next-slope direction. OutCorner must preserve its lane.
        if (held && currentGuideValid && !currentGuide.isSlope &&
            TryResolveAutoHoldTurnFromCurrentFlat(out float flatTurnSign) &&
            flatTurnSign > 0f)
            return 0f;

        Vector3 left = autoHoldTurnPending ? autoHoldTurnLeft : autoHoldLastLeft;
        left = Vector3.ProjectOnPlane(left, Vector3.up);
        if (left.sqrMagnitude <= Eps) return 0f;
        left.Normalize();
        if (!autoHoldEdgeSampleValid || Time.fixedTime >= autoHoldNextEdgeSampleTime)
        {
            if (TryResolveAutoHoldEdge(left, out Vector3 sampled))
            {
                autoHoldEdgeSample = sampled;
                autoHoldEdgeSampleValid = true;
            }
            else
                autoHoldEdgeSampleValid = false;
            // At high speed, refresh before the body travels more than two
            // probe radii; at low speed avoid unnecessary geometry queries.
            float groundSpeed = Vector3.ProjectOnPlane(rb.velocity,
                Vector3.up).magnitude;
            autoHoldNextEdgeSampleTime = Time.fixedTime +
                Mathf.Min(.08f, ProbeRadius * 2f / Mathf.Max(1f, groundSpeed));
        }
        if (!autoHoldEdgeSampleValid) return 0f;

        float remaining = Vector3.Dot(autoHoldEdgeSample - rb.position, left);
        float distanceToExit = Mathf.Max(0f,
            Vector3.Dot(autoHoldExitPoint - rb.position, autoHoldLastForward));
        float forwardSpeed = Mathf.Max(1f,
            Vector3.Dot(rb.velocity, autoHoldLastForward));
        float timeRemaining = Mathf.Max(.12f, distanceToExit / forwardSpeed);
        // This route begins later (on the slope, not on the preceding flat).
        // Give it a bounded distance/time-responsive side budget. The original
        // flat-start Auto Hold's Inspector caps remain completely unchanged.
        float sideAccel = slopeReleaseHoldArmed
            ? Mathf.Max(autoHoldMaxSideAcceleration, GroundAcceleration * 2f)
            : autoHoldMaxSideAcceleration;
        float sideSpeedCap = slopeReleaseHoldArmed
            ? Mathf.Max(autoHoldMaxSideSpeed, Mathf.Min(8f, maxGroundSpeed * .32f))
            : autoHoldMaxSideSpeed;
        float stopSpeed = Mathf.Sqrt(2f * sideAccel * Mathf.Abs(remaining));
        float speed = Mathf.Min(sideSpeedCap,
            Mathf.Min(Mathf.Abs(remaining) / timeRemaining, stopSpeed));
        float desiredSideSpeed = Vector3.Dot(left *
            (Mathf.Sign(remaining) * speed), surface.side);
        return Mathf.Clamp((desiredSideSpeed - surface.lateralSpeed) * 12f,
            -sideAccel, sideAccel);
    }

    // Only committed Auto Hold performs final edge alignment. A long shift
    // must be supported all along its cross-section; normal flicks are untouched.
    void SetAutoHoldTurnAtPhysicalEdge()
    {
        Vector3 left = Vector3.ProjectOnPlane(autoHoldTurnLeft,
            Vector3.up);
        if (left.sqrMagnitude <= Eps) return;
        left.Normalize();
        if (!TryResolveAutoHoldEdge(left, out Vector3 target))
        {
            AutoHoldLog("EDGE UNAVAILABLE", "no confirmed stage edge / cache invalid");
            return;
        }
        Vector3 delta = Vector3.ProjectOnPlane(target - rb.position, Vector3.up);
        int steps = Mathf.Max(1,
            Mathf.CeilToInt(delta.magnitude / (ProbeRadius * .5f)));
        for (int i = 1; i <= steps; ++i)
        {
            Vector3 sample = rb.position + delta * (i / (float)steps);
            sample.y = autoHoldEdgeProbeHeight;
            if (TryAutoHoldGroundSphere(sample, autoHoldEdgeSurfaceHeight))
                continue;
            AutoHoldLog("EDGE UNAVAILABLE", "unsafe gap on final alignment path");
            return;
        }
        if (delta.sqrMagnitude > Eps)
        {
            rb.position += delta;
            Physics.SyncTransforms();
            AutoHoldEdgeSnapped?.Invoke(rb, delta);
        }
        // The leftmost support is meaningless if sideways inertia remains.
        rb.velocity -= left * Vector3.Dot(rb.velocity, left);
        AutoHoldLog("EDGE LOCK", $"shift={delta.magnitude:F3}m collider={(autoHoldEdgeCollider ? autoHoldEdgeCollider.name : "-")} pos={rb.position:F3}");
    }

    // Read-only lookahead through the REAL section after the held slope.
    // Works even if the detector's current nearest guide is still on a stair
    // or the incoming ball has already passed the physical next-flat lip.
    bool TryResolveAutoHoldTurnFromFollowingSlope(
        NearestKnotDetector.GuideFrame heldSlope, Vector3 previousHeading,
        out float turnSign)
    {
        turnSign = 0f;
        if (!knotDetector || !heldSlope.valid || !heldSlope.isSlope ||
            !knotDetector.TryGetFollowingFlatStart(
                heldSlope, out Vector3 flatStart, out Vector3 flatTangent) ||
            !knotDetector.TryEvaluateSameSection(heldSlope, 1f,
                out NearestKnotDetector.GuideSample lastSlopeSample) ||
            !lastSlopeSample.valid)
            return false;

        // The first flat segment is exactly after the last segment of the
        // held slope. TryGetFollowingFlatStart verified that it exists.
        NearestKnotDetector.GuideFrame followingFlat =
            new NearestKnotDetector.GuideFrame
            {
                valid = true,
                isSlope = false,
                splineIndex = lastSlopeSample.splineIndex,
                segmentIndex = lastSlopeSample.segmentIndex + 1,
                tangent = flatTangent,
                point = flatStart
            };
        if (!knotDetector.TryEvaluateForwardSlopeSection(
                followingFlat, 0f,
                out NearestKnotDetector.GuideSample futureSlope) ||
            !futureSlope.valid)
            return false;

        Vector3 forward = NormalizeFlat(previousHeading, heldSlope.tangent);
        Vector3 nextForward = NormalizeFlat(futureSlope.tangent, forward);
        float signedAngle = Vector3.SignedAngle(
            forward, nextForward, Vector3.up);
        // Require a real quarter-turn candidate; do not infer from slight
        // ramp deviations or from whichever side the body happens to occupy.
        if (Mathf.Abs(signedAngle) < 50f ||
            Mathf.Abs(signedAngle) > 130f)
            return false;

        turnSign = signedAngle > 0f ? 1f : -1f;
        return true;
    }

    // At release, the receiving FLAT is an independent way to read the
    // immediately following slope. Verify the incoming heading and elevation
    // before using it; a geometrically nearby but unrelated lane is not proof.
    bool TryResolveAutoHoldTurnFromCurrentFlat(out float sign)
    {
        sign = 0f;
        if (!knotDetector || !autoHoldLastSlopeGuide.valid || !rb)
            return false;
        NearestKnotDetector.GuideFrame flat = knotDetector.CurrentGuide;
        if (!flat.valid || flat.isSlope ||
            !knotDetector.TryGetFollowingFlatStart(autoHoldLastSlopeGuide,
                out Vector3 flatStart, out Vector3 flatTangent))
            return false;

        Vector3 incoming = NormalizeFlat(autoHoldLastForward, direction);
        if (Vector3.Dot(NormalizeFlat(flat.tangent, incoming), incoming) < .85f ||
            Vector3.Dot(NormalizeFlat(flatTangent, incoming), incoming) < .85f ||
            Mathf.Abs(flat.point.y - flatStart.y) > ProbeRadius + .50f ||
            Vector3.Dot(flat.point - flatStart, incoming) < -.75f ||
            !knotDetector.TryEvaluateForwardSlopeSection(flat, 0f,
                out NearestKnotDetector.GuideSample nextSlope) ||
            !nextSlope.valid)
            return false;

        Vector3 outgoing = NormalizeFlat(nextSlope.tangent, incoming);
        float angle = Vector3.SignedAngle(incoming, outgoing, Vector3.up);
        if (Mathf.Abs(angle) < 50f || Mathf.Abs(angle) > 130f)
            return false;

        sign = angle > 0f ? 1f : -1f;
        return true;
    }

    // Never choose a turn from the player's left/right position or the
    // Inspector fallback when a FLAT-start (or slope-start) hold is released.
    // The old fallback defaulted to LEFT and occasionally inverted a real
    // right-hand corner. Require actual upcoming spline geometry instead.
    bool TryResolveAutoHoldTurnSign(out float sign, out string source)
    {
        sign = 0f;
        source = "no verified next slope";
        bool liveFlat = TryResolveAutoHoldTurnFromCurrentFlat(out float flatSign);
        bool heldSlope = TryResolveAutoHoldTurnFromFollowingSlope(
            autoHoldLastSlopeGuide, autoHoldLastForward, out float slopeSign);

        if (liveFlat && heldSlope && flatSign != slopeSign)
        {
            source = "conflicting flat / held-slope geometry";
            AutoHoldLog("TURN DIRECTION CONFLICT",
                $"flat={flatSign:+0;-0} heldSlope={slopeSign:+0;-0} " +
                $"spline={autoHoldLastSlopeGuide.splineIndex}; turn withheld");
            return false;
        }

        if (liveFlat)
        {
            sign = flatSign;
            source = "verified next-flat spline";
        }
        else if (heldSlope)
        {
            sign = slopeSign;
            source = "held-slope next-section spline";
        }
        else if (autoHoldPredictedTurnSignValid)
        {
            sign = autoHoldPredictedTurnSign;
            source = "slope-entry geometry cache";
        }
        else
            return false;

        if (autoHoldPredictedTurnSignValid &&
            autoHoldPredictedTurnSign != sign)
            AutoHoldLog("TURN DIRECTION REFRESHED",
                $"old={autoHoldPredictedTurnSign:+0;-0} new={sign:+0;-0} " +
                $"source={source}");
        return true;
    }

    float ResolveEdgeTurnSign(Vector3 forward, Vector3 left)
    {
        // Both input methods use one geometric turn-direction decision.
        if (knotDetector && rb)
        {
            bool foundNextSlope = knotDetector.TryGetNextSlopePosition();
            Vector3 target = knotDetector.nextSlopePosition;
            Vector3 toNext = Vector3.ProjectOnPlane(
                target - rb.position, Vector3.up);
            if (foundNextSlope && IsFiniteVector(target) &&
                IsFiniteVector(toNext))
            {
                float ahead = Vector3.Dot(toNext, forward);
                float towardLeft = Vector3.Dot(toNext, left);
                if (ahead > -1f && Mathf.Abs(towardLeft) > 4f &&
                    Mathf.Abs(towardLeft) > Mathf.Abs(ahead) * .65f)
                    return towardLeft > 0f ? -1f : 1f;
            }
        }
        return autoHoldFallbackTurnRight ? 1f : -1f;
    }

    // ================================================================
    // Global near-stall safety. Never deliberately stops the ball.
    // Does not apply to intentional HardFreeze / Handoff / new player turns.
    // ================================================================
    bool GeneralStallHasControlConflict()
    {
        // Never compete with an intentional finger gesture, edge approach,
        // or an ACCEPTED turn/Handoff. The ordinary motor keeps control then.
        return !enableGeneralStallRecovery || !rb || rb.isKinematic ||
            !BeginCommandOnTouch || MainGameManager.OnDead ||
            turnTransitionActive || turnBodyFrozen ||
            slopeReleaseEscapeActive || slopeReleaseHoldHandoffHeld ||
            tapEdgeHandoffHeld || fiveLineCorrectionActive ||
            postAutoHoldEdgeActive || edgeCruiseActive || trackingFlick ||
            HasPendingTurnIntent;
    }

    void ResetGeneralStallWatch()
    {
        stallLowSpeedSince = -1f;
        stallEarlyWatchSince = -1f;
    }

    // A rolling observation for two *early* signals. Flat or stair is fine,
    // but actual support, plausible direction, and a low-speed envelope are
    // verified by the caller. No recovery is launched by this function.
    bool ObserveEarlyGeneralStall(
        NearestKnotDetector.GuideFrame guide, Surface surface,
        Vector3 velocity, float forwardSpeed, float planarSpeed,
        out string reason, out float elapsed, out float progress)
    {
        reason = null;
        elapsed = 0f;
        progress = 0f;
        if (forwardSpeed < -.65f || forwardSpeed > 2.25f ||
            planarSpeed > 2.50f ||
            Mathf.Abs(Vector3.Dot(velocity, surface.side)) > 1.80f ||
            Mathf.Abs(Vector3.Dot(velocity, surface.normal)) > 1.50f)
        {
            stallEarlyWatchSince = -1f;
            return false;
        }

        Vector3 heading = NormalizeFlat(surface.tangent, direction);
        if (stallEarlyWatchSince >= 0f &&
            (stallEarlySectionIndex != guide.sectionIndex ||
             stallEarlyWasSlope != guide.isSlope ||
             Vector3.Dot(stallEarlyStartHeading, heading) < .90f))
            stallEarlyWatchSince = -1f;

        float crossTrack = Mathf.Abs(Vector3.Dot(
            rb.position - guide.point, surface.side));
        if (stallEarlyWatchSince < 0f)
        {
            stallEarlyWatchSince = Time.fixedTime;
            stallEarlyStartPosition = rb.position;
            stallEarlyStartCrossTrack = crossTrack;
            stallEarlyStartHeading = heading;
            stallEarlySectionIndex = guide.sectionIndex;
            stallEarlyWasSlope = guide.isSlope;
            return false;
        }

        elapsed = Time.fixedTime - stallEarlyWatchSince;
        Vector3 moved = rb.position - stallEarlyStartPosition;
        progress = Vector3.Dot(moved, surface.tangent);
        float groundTravel = Vector3.ProjectOnPlane(
            moved, surface.normal).magnitude;

        // A: barely any *actual movement* (not merely low tangent speed).
        // This prevents a ball sliding sideways at speed from being called stuck.
        if (elapsed >= StallEarlyCreepSeconds &&
            Mathf.Abs(progress) < StallEarlyCreepProgress &&
            groundTravel < StallEarlyCreepTravel)
        {
            reason = "early-creep";
            return true;
        }

        // B: moving slowly along a line but staying >=1m away from it.
        // Do not pull a rapidly drifting ball toward the nearest wrong spline.
        if (elapsed >= StallEarlyNoConvergenceSeconds)
        {
            if (stallEarlyStartCrossTrack >= StallEarlyCrossTrack &&
                crossTrack >= StallEarlyCrossTrack &&
                stallEarlyStartCrossTrack - crossTrack <
                    StallEarlyRequiredConvergence)
            {
                reason = "early-no-convergence";
                return true;
            }

            // A healthy window becomes the new rolling reference. This keeps
            // a long straight run from accumulating a stale offset sample.
            stallEarlyWatchSince = Time.fixedTime;
            stallEarlyStartPosition = rb.position;
            stallEarlyStartCrossTrack = crossTrack;
            stallEarlyStartHeading = heading;
            stallEarlySectionIndex = guide.sectionIndex;
            stallEarlyWasSlope = guide.isSlope;
        }
        return false;
    }

    void ResetUncapturedTurnGuideStallWatch()
    {
        turnGuideStallWatchSince = -1f;
    }

    bool UncapturedTurnGuideStallHasControlConflict()
    {
        // An InCorner may arm postAutoHoldEdgeActive WHILE waiting for the new
        // guide. That edge approach cannot act until postAutoHoldNewGuide is
        // true, so it must not veto the only possible recovery at this stage.
        return !enableGeneralStallRecovery || !rb || rb.isKinematic ||
            !BeginCommandOnTouch || MainGameManager.OnDead ||
            turnTransitionActive || turnBodyFrozen ||
            slopeReleaseEscapeActive || slopeReleaseHoldHandoffHeld ||
            tapEdgeHandoffHeld || fiveLineCorrectionActive ||
            (postAutoHoldEdgeActive && postAutoHoldNewGuide) ||
            edgeCruiseActive || trackingFlick || HasPendingTurnIntent;
    }

    // A completed turn can remain in waitingForTurnGuide indefinitely because
    // Evaluate() still sees a perpendicular OLD spline. Do not lift that
    // safety guard. Instead, find a FORWARD-aligned receiving spline without
    // changing detector tracking, and demand real, close supporting floor.
    bool ObserveUncapturedTurnGuideStall(
        NearestKnotDetector.GuideFrame nearestGuide, bool grounded)
    {
        if (!waitingForTurnGuide ||
            UncapturedTurnGuideStallHasControlConflict() || !grounded ||
            turnGuideReleasedAt < 0f || !knotDetector)
        {
            ResetUncapturedTurnGuideStallWatch();
            return false;
        }
        // Let the existing handoff complete normally if Evaluate() has
        // finally acquired the matching receiving guide this very frame.
        if (nearestGuide.valid &&
            TurnGuideAxisAlignment(nearestGuide) >= TurnGuideAlignmentMin &&
            nearestGuide.distanceToGuide <= Mathf.Max(
                supportGraceMaxGuideDistance, ProbeRadius * 2f))
        {
            ResetUncapturedTurnGuideStallWatch();
            return false;
        }
        float sinceRelease = Time.fixedTime - turnGuideReleasedAt;
        if (sinceRelease < TurnGuideStallAfterReleaseSeconds)
            return false;
        Vector3 planar = Vector3.ProjectOnPlane(rb.velocity, Vector3.up);
        if (planar.magnitude > TurnGuideStallMaximumPlanarSpeed ||
            Mathf.Abs(rb.velocity.y) > 1.50f)
        {
            ResetUncapturedTurnGuideStallWatch();
            return false;
        }

        // HasGroundSupport() alone may report a ray hit many metres below the
        // ball (ProbeDistance can be 28). Require ACTUAL nearby stage floor.
        Vector3 centre = rb.worldCenterOfMass;
        if (!TryAutoHoldGroundRay(centre, out RaycastHit closeFloor) ||
            !IsCloseGroundRay(closeFloor, centre))
        {
            ResetUncapturedTurnGuideStallWatch();
            return false;
        }
        Vector3 target = NormalizeFlat(turnTargetDirection, direction);
        if (turnGuideStallWatchSince < 0f)
        {
            turnGuideStallWatchSince = Time.fixedTime;
            turnGuideStallWatchPosition = rb.position;
            GeneralStallLog("TURN GUIDE WATCH",
                $"sinceRelease={sinceRelease:F2}s " +
                $"speed={planar.magnitude:F2} floor={closeFloor.collider.name} " +
                $"target={target:F2}");
            return false;
        }
        float elapsed = Time.fixedTime - turnGuideStallWatchSince;
        float progress = Vector3.Dot(
            rb.position - turnGuideStallWatchPosition, target);
        if (progress >= TurnGuideStallMinProgress)
        {
            // Real progress along the intended heading; observe anew.
            turnGuideStallWatchSince = Time.fixedTime;
            turnGuideStallWatchPosition = rb.position;
            return false;
        }
        if (elapsed < TurnGuideStallObservationSeconds ||
            (stallLastAttemptAt >= 0f &&
             Time.fixedTime - stallLastAttemptAt < stallRecoveryCooldown))
            return false;

        stallLastAttemptAt = Time.fixedTime;
        if (!knotDetector.TryFindGuideAlongHeading(
                rb.position, target, 3.6f,
                out NearestKnotDetector.GuideFrame candidate))
        {
            LogTurnGuideStallBlocked("no forward-aligned spline within 3.6m",
                elapsed, progress);
            return false;
        }
        Surface receiving = BuildSplineSurface(candidate);
        if (!receiving.Valid ||
            Vector3.Dot(NormalizeFlat(candidate.tangent, target), target) < .82f ||
            candidate.distanceToGuide > 3.6f)
        {
            LogTurnGuideStallBlocked("receiving guide direction / distance",
                elapsed, progress);
            return false;
        }
        if (!TrySelectGeneralSafeLane(candidate, receiving,
                out Vector3 lanePoint,
                out NearestKnotDetector.FiveLineGroup lane))
        {
            LogTurnGuideStallBlocked("no physically supported FiveLine corridor",
                elapsed, progress);
            return false;
        }

        // Reuse precisely the normal, bounded force motor. No teleport and
        // no global 90-degree turn. Keep waitingForTurnGuide until tracking
        // really acquires the receiving spline.
        stallLastMovingVelocity = planar;
        // Keep historical acceleration until a new observation is measured.
        stallRecoveryPoint = lanePoint;
        stallRecoveryLane = lane;
        stallRecoveryHeading = candidate.tangent;
        stallRecoveryWasSlope = candidate.isSlope;
        stallRecoveryStartedAt = Time.fixedTime;
        stallRecoveryForUncapturedTurn = true;
        generalStallRecovering = true;
        ResetGeneralStallWatch();
        ResetUncapturedTurnGuideStallWatch();
        GeneralStallLog("START",
            $"reason=turn-guide-unclaimed " +
            $"sinceRelease={sinceRelease:F2}s observed={elapsed:F2}s " +
            $"progress={progress:F2}m " +
            $"targetHeading={target:F2} lane={lane} " +
            $"section={candidate.sectionIndex} " +
            $"floor={closeFloor.collider.name} target={lanePoint:F2}");
        return true;
    }

    void LogTurnGuideStallBlocked(string why, float elapsed, float progress)
    {
        if (Time.fixedTime < turnGuideStallNextLogAt)
            return;
        turnGuideStallNextLogAt = Time.fixedTime + 1.0f;
        GeneralStallLog("TURN GUIDE BLOCKED",
            $"reason={why} elapsed={elapsed:F2}s progress={progress:F2}m " +
            $"pos={rb.position:F2} target={turnTargetDirection:F2}");
    }

    // First prove a genuine stall: low speed, almost no progress, real ground,
    // and no competing operation. Then and ONLY then search for a receiving
    // line. This path must never fire on a jump, drop, slow but moving slope,
    // hard freeze, or normal fast turn.
    bool ObserveGeneralStallRecovery(
        NearestKnotDetector.GuideFrame guide, Surface surface,
        bool grounded, RaycastHit support)
    {
        if (waitingForTurnGuide)
            return false; // New guide recovery has its own verified path.
        if (GeneralStallHasControlConflict() || !grounded ||
            !IsAutoHoldStageHit(support, float.NaN) ||
            !guide.valid || !surface.Valid ||
            guide.distanceToGuide > Mathf.Max(2.5f, ProbeRadius * 4f))
        {
            ResetGeneralStallWatch();
            stallPreviousVelocityValid = false;
            return false;
        }

        Vector3 heading = NormalizeFlat(surface.tangent, direction);
        // A perpendicular closest-line swap is NOT a player's turn.
        // A stalled ball must not be rescued onto the wrong 90deg branch.
        Vector3 expected = waitingForTurnGuide
            ? NormalizeFlat(turnTargetDirection, direction)
            : NormalizeFlat(direction, travelDirection);
        if (Vector3.Dot(heading, expected) < .78f)
        {
            ResetGeneralStallWatch();
            stallPreviousVelocityValid = false;
            return false;
        }

        Vector3 velocity = rb.velocity;
        float forwardSpeed = Vector3.Dot(velocity, surface.tangent);
        float planarSpeed = Vector3.ProjectOnPlane(velocity, Vector3.up).magnitude;
        // Do not infer the pre-stall acceleration from a velocity freshly
        // overwritten by Handoff. Keep the last physically moving sample.
        if (forwardSpeed >= 2.5f && planarSpeed >= 2.5f)
        {
            if (stallPreviousVelocityValid)
            {
                Vector3 measured = (velocity - stallPreviousMotionVelocity) /
                    Mathf.Max(.001f, Time.fixedDeltaTime);
                if (IsFiniteVector(measured) && measured.sqrMagnitude < 160f * 160f)
                    stallLastMovingAcceleration = Vector3.Lerp(
                        stallLastMovingAcceleration, measured, .20f);
            }
            else
                stallLastMovingAcceleration = Vector3.ClampMagnitude(
                    acceleration, 90f);
            stallPreviousMotionVelocity = velocity;
            stallPreviousVelocityValid = true;
            stallLastMovingVelocity = velocity;
            stallLastHealthyTime = Time.fixedTime;
        }
        else
            stallPreviousVelocityValid = false;

        // These early detectors share the existing safety corridor and motor.
        // Keep the original 0.75s low-speed route as a separate fallback.
        bool earlyStuck = ObserveEarlyGeneralStall(
            guide, surface, velocity, forwardSpeed, planarSpeed,
            out string cause, out float elapsed, out float distanceProgress);

        // Require low TOTAL velocity as well: a fast sideways drift must not
        // be mistaken for a stuck ball just because tangentSpeed is small.
        bool nearlyStopped = forwardSpeed <= stallWatchForwardSpeed &&
            forwardSpeed > -.65f &&
            planarSpeed <= Mathf.Max(1.8f, stallWatchForwardSpeed + .5f) &&
            Mathf.Abs(Vector3.Dot(velocity, surface.normal)) <= 1.5f;
        if (!nearlyStopped)
            stallLowSpeedSince = -1f;

        if (!earlyStuck)
        {
            if (!nearlyStopped)
                return false;
            if (stallLowSpeedSince < 0f)
            {
                stallLowSpeedSince = Time.fixedTime;
                stallStartPosition = rb.position;
                GeneralStallLog("WATCH",
                    $"surface={(guide.isSlope ? "STAIR" : "FLAT")} " +
                    $"speed={forwardSpeed:F2}m/s (observe for {stallWatchSeconds:F2}s)");
                return false;
            }

            elapsed = Time.fixedTime - stallLowSpeedSince;
            distanceProgress = Vector3.Dot(
                rb.position - stallStartPosition, surface.tangent);
            if (Mathf.Abs(distanceProgress) >= stallWatchMaxProgress)
            {
                // The ball still made measurable forward progress, but
                // this does NOT reset the independent convergence monitor.
                stallLowSpeedSince = Time.fixedTime;
                stallStartPosition = rb.position;
                return false;
            }
            if (elapsed < stallWatchSeconds)
                return false;
            cause = "legacy-near-stop";
        }

        if (stallLastAttemptAt >= 0f &&
            Time.fixedTime - stallLastAttemptAt < stallRecoveryCooldown)
            return false;

        // A body that has never moved can be physically blocked at spawn;
        // require a longer time before the first recovery attempt.
        if (stallLastHealthyTime < 0f && elapsed < stallWatchSeconds + .60f)
            return false;
        // No impulse from an old, unrelated turn: discard stale history.
        if (stallLastHealthyTime < 0f ||
            Time.fixedTime - stallLastHealthyTime > 2.0f)
        {
            stallLastMovingVelocity = surface.tangent * Mathf.Max(0f, forwardSpeed);
            stallLastMovingAcceleration = Vector3.zero;
        }
        stallLastAttemptAt = Time.fixedTime;
        if (!TrySelectGeneralSafeLane(guide, surface, out Vector3 lanePoint,
            out NearestKnotDetector.FiveLineGroup lane))
        {
            if (Time.fixedTime >= stallNextDiagnosticAt)
            {
                stallNextDiagnosticAt = Time.fixedTime + 1.5f;
                GeneralStallLog("NO SAFE LANE",
                    $"section={guide.sectionIndex} slope={guide.isSlope} " +
                    $"reason={cause} observed={elapsed:F2}s " +
                    $"progress={distanceProgress:F2}m " +
                    "physical support/forward corridor not verified");
            }
            return false; // Continue ordinary physics; do not freeze.
        }

        // This is a REAL verified stall. It preempts any UNCOMMITTED hold;
        // otherwise a later MouseUp could unexpectedly request a 90deg turn.
        if (autoHoldEligible)
            CancelAutoHold("general stall recovery took priority");
        stallRecoveryPoint = lanePoint;
        stallRecoveryLane = lane;
        stallRecoveryHeading = surface.tangent;
        stallRecoveryWasSlope = guide.isSlope;
        stallRecoveryStartedAt = Time.fixedTime;
        stallRecoveryForUncapturedTurn = false;
        generalStallRecovering = true;
        ResetGeneralStallWatch();
        GeneralStallLog("START",
            $"surface={(guide.isSlope ? "STAIR" : "FLAT")} " +
            $"reason={cause} lane={lane} observed={elapsed:F2}s " +
            $"progress={distanceProgress:F2}m " +
            $"beforeVel={stallLastMovingVelocity:F2} " +
            $"beforeAccel={stallLastMovingAcceleration:F2} " +
            $"target={lanePoint:F2} (no teleport/no forced stop)");
        return true;
    }

    // Stair treads are horizontal MeshCollider patches even when the Spline
    // segment slopes. Allow DIFFERENT colliders on successive treads, but
    // verify that every lateral sample is near the expected physical surface.
    bool GeneralStallFootprintSupported(
        Vector3 fromCenter, Vector3 floorTarget,
        NearestKnotDetector.GuideFrame baseGuide, Surface surface,
        bool verifyAhead)
    {
        if (!IsFiniteVector(floorTarget) || !surface.Valid)
            return false;
        float normalHeight = Mathf.Clamp(
            Vector3.Dot(fromCenter - baseGuide.point, surface.normal),
            .15f, ProbeRadius + .45f);
        Vector3 targetCenter = floorTarget + surface.normal * normalHeight;
        Vector3 sideShift = Vector3.ProjectOnPlane(
            targetCenter - fromCenter, surface.tangent);
        if (sideShift.magnitude > 3.25f)
            return false;
        // Every footprint sample must have a real collider reasonably close
        // to the expected spline height. A far-away floor is not safe.
        int count = Mathf.Clamp(Mathf.CeilToInt(sideShift.magnitude / .35f), 2, 10);
        for (int i = 0; i <= count; ++i)
        {
            float t = i / (float)count;
            Vector3 sample = Vector3.Lerp(fromCenter, targetCenter, t);
            Vector3 expectedFloor = Vector3.Lerp(
                baseGuide.point, floorTarget, t);
            if (!TryAutoHoldGroundRay(sample, out RaycastHit hit) ||
                !IsAutoHoldStageHit(hit, float.NaN) ||
                Mathf.Abs(hit.point.y - expectedFloor.y) > .75f)
                return false;
        }
        if (!verifyAhead)
            return true;
        Vector3 aheadCenter = targetCenter + surface.tangent * .50f;
        Vector3 aheadFloor = floorTarget + surface.tangent * .50f;
        return TryAutoHoldGroundRay(aheadCenter, out RaycastHit next) &&
            IsAutoHoldStageHit(next, float.NaN) &&
            Mathf.Abs(next.point.y - aheadFloor.y) <= .85f;
    }

    bool TrySelectGeneralSafeLane(
        NearestKnotDetector.GuideFrame guide, Surface surface,
        out Vector3 lanePoint, out NearestKnotDetector.FiveLineGroup lane)
    {
        lanePoint = default;
        lane = NearestKnotDetector.FiveLineGroup.Center;
        if (!knotDetector || !rb ||
            !knotDetector.TryGetFiveLineFrame(
                guide, surface.side,
                out NearestKnotDetector.FiveLineFrame frame) || !frame.valid)
            return false;
        float best = float.PositiveInfinity;
        Vector3 side = surface.side;
        float previousVSide = Vector3.Dot(stallLastMovingVelocity, side);
        float previousASide = Vector3.Dot(stallLastMovingAcceleration, side);
        // A short prediction measures which way the body was drifting before
        // losing speed. Center remains preferred unless moving to it is unsafe.
        float predicted = Vector3.Dot(rb.position - frame.center, side) +
            previousVSide * .18f + .5f * previousASide * .18f * .18f;
        for (int i = 0; i < 5; ++i)
        {
            var group = (NearestKnotDetector.FiveLineGroup)i;
            Vector3 candidate = frame.GetPoint(group);
            if (!GeneralStallFootprintSupported(
                rb.position, candidate, guide, surface, true))
                continue;
            float offset = Vector3.Dot(candidate - frame.center, side);
            float current = Vector3.Dot(rb.position - frame.center, side);
            float score = Mathf.Abs(offset) * .80f +
                Mathf.Abs(offset - predicted) * .35f +
                Mathf.Abs(offset - current) * .28f;
            if (score >= best) continue;
            best = score;
            lanePoint = candidate;
            lane = group;
        }
        return !float.IsInfinity(best);
    }

    NearestKnotDetector.GuideFrame ReadSafeReceivingTurnGuide()
    {
        if (!waitingForTurnGuide || !knotDetector)
            return default;
        return knotDetector.TryFindGuideAlongHeading(
            rb.position, turnTargetDirection, 3.6f,
            out NearestKnotDetector.GuideFrame guide) ? guide : default;
    }

    bool StepGeneralStallRecovery()
    {
        if (!generalStallRecovering || !rb)
            return false;
        if (stallRecoveryForUncapturedTurn
                ? UncapturedTurnGuideStallHasControlConflict()
                : GeneralStallHasControlConflict())
        {
            ResetGeneralStallRecovery("player / turn / handoff took priority", true);
            return false;
        }
        var contact = HasGroundSupport();
        if (!contact.Item1 || !IsAutoHoldStageHit(contact.Item2, float.NaN))
        {
            ResetGeneralStallRecovery("actual ground lost", true);
            return false;
        }
        // During post-turn rescue an extended HasGroundSupport ray is NOT
        // enough: the body must really be sitting close to a stage collider.
        if (stallRecoveryForUncapturedTurn &&
            (!waitingForTurnGuide ||
             !TryAutoHoldGroundRay(rb.worldCenterOfMass,
                 out RaycastHit closeFloor) ||
             !IsCloseGroundRay(closeFloor, rb.worldCenterOfMass)))
        {
            ResetGeneralStallRecovery("turn-guide floor / phase lost", true);
            return false;
        }
        // Re-evaluate the live guide each tick. A new 90deg section, a drop,
        // or a stale old segment must never be treated as permission to turn.
        var live = stallRecoveryForUncapturedTurn
            ? ReadSafeReceivingTurnGuide()
            : knotDetector.Evaluate(rb.position);
        if (!live.valid || live.isSlope != stallRecoveryWasSlope ||
            Vector3.Dot(NormalizeFlat(live.tangent, direction),
                NormalizeFlat(stallRecoveryHeading, direction)) < .80f ||
            live.distanceToGuide > (stallRecoveryForUncapturedTurn ? 3.6f : 2.5f))
        {
            ResetGeneralStallRecovery("route changed / guide out of reach", true);
            return false;
        }
        Surface surface = BuildSplineSurface(live);
        // A newly rescued ball can still be moving: carry the selected lane
        // forward along the LIVE section instead of chasing a stale world
        // point from the first frame of recovery.
        if (!surface.Valid ||
            !knotDetector.TryGetFiveLineFrame(live, surface.side,
                out NearestKnotDetector.FiveLineFrame liveFrame) ||
            !liveFrame.valid)
        {
            ResetGeneralStallRecovery("receiving lane no longer valid", true);
            return false;
        }
        Vector3 targetNow = liveFrame.GetPoint(stallRecoveryLane);
        if (!GeneralStallFootprintSupported(
                rb.position, targetNow, live, surface, true))
        {
            ResetGeneralStallRecovery("corridor no longer supported", true);
            return false;
        }
        stallRecoveryPoint = targetNow;

        Vector3 side = surface.side;
        float error = Vector3.Dot(stallRecoveryPoint - rb.position, side);
        float vSide = Vector3.Dot(rb.velocity, side);
        float forward = Vector3.Dot(rb.velocity, surface.tangent);
        float sideLimit = Mathf.Clamp(stallRecoverySideAcceleration, 1f, 55f);
        float sideTargetSpeed = Mathf.Sign(error) *
            Mathf.Min(2.5f, Mathf.Sqrt(2f * sideLimit * Mathf.Abs(error)));
        float sideAccel = Mathf.Clamp(
            (sideTargetSpeed - vSide) * 9f, -sideLimit, sideLimit);
        float savedSpeed = Mathf.Max(0f,
            Vector3.Dot(stallLastMovingVelocity, surface.tangent));
        float savedAccel = Mathf.Max(0f,
            Vector3.Dot(stallLastMovingAcceleration, surface.tangent));
        float targetSpeed = Mathf.Clamp(
            2.5f + savedSpeed * .15f + savedAccel * .04f,
            2.5f, Mathf.Min(maxGroundSpeed, 6.5f));
        float driveLimit = Mathf.Clamp(
            stallRecoveryForwardAcceleration, 1f, 45f);
        float forwardAccel = Mathf.Clamp(
            (targetSpeed - forward) * 7f, -driveLimit, driveLimit);

        // Keep the shared ground / extreme-point observer current while the
        // bounded recovery motor temporarily owns forces. Otherwise a fast
        // stair recovery might skip the 60% event or leave stale support.
        currentGuide = live;
        currentGuideValid = true;
        currentSurface = surface;
        currentSurfaceValid = true;
        currentSupported = true;
        currentHit = contact.Item2;
        graceTimer = supportGraceSeconds;
        previousGroundedForDropoutLog = true;
        if (live.isSlope)
            ObserveExtremePoint(live);
        else
            CompleteExtremePointAtFlat();
        UpdateFlatEntryFrame(live);

        // Ground ahead was verified above for the selected lane; check the
        // *current* step too. Keep gravity and normal stick, no position snap.
        rb.AddForce(surface.tangent * forwardAccel +
            side * sideAccel - surface.normal *
            Mathf.Min(35f, Mathf.Max(0f, flatStick)), ForceMode.Acceleration);
        if (live.isSlope) driveState = 0f;
        float duration = Time.fixedTime - stallRecoveryStartedAt;
        bool reachedLane = forward >= 2.2f && Mathf.Abs(error) < .30f &&
            Mathf.Abs(vSide) < 1.0f;
        if (stallRecoveryForUncapturedTurn && reachedLane &&
            live.distanceToGuide <= 1.60f &&
            knotDetector.TryFocusGuideAfterSafeRecovery(
                live, rb.position, turnTargetDirection, 1.60f))
            GeneralStallLog("TURN GUIDE FOCUSED",
                $"section={live.sectionIndex} spline={live.splineIndex} " +
                $"distance={live.distanceToGuide:F2}m " +
                "next FixedUpdate will verify NEW-GUIDE-CAPTURED");
        if (reachedLane || duration >= 1.15f)
        {
            ResetGeneralStallRecovery("forward speed restored / bounded duration", true);
            return true;
        }
        return true;
    }

    void ResetGeneralStallRecovery(string why, bool clearHistory)
    {
        if (generalStallRecovering)
            GeneralStallLog("END", $"reason={why} position={(rb ? rb.position : Vector3.zero):F2}");
        generalStallRecovering = false;
        stallRecoveryForUncapturedTurn = false;
        ResetGeneralStallWatch();
        if (clearHistory)
        {
            stallPreviousVelocityValid = false;
            stallLastHealthyTime = -1f;
            stallLastMovingVelocity = Vector3.zero;
            stallLastMovingAcceleration = Vector3.zero;
        }
    }

    void GeneralStallLog(string stage, string message)
    {
        if (logTurnTransition)
            Debug.Log($"[STALL SPLINE {stage}] t={Time.fixedTime:F3} {message}", this);
    }

    // Keep only a light, bounded history of velocity changes while moving.
    // The remembered acceleration is taken BEFORE horizontal Hold forces V=0.
    void CaptureSlopeReleaseBeforeStopMotion()
    {
        if (!rb || rb.isKinematic || MainGameManager.OnDead ||
            turnTransitionActive || slopeReleaseHoldHandoffHeld ||
            slopeReleaseEscapeActive)
        {
            slopeReleaseLastSampleValid = false;
            return;
        }
        Vector3 planar = Vector3.ProjectOnPlane(rb.velocity, Vector3.up);
        if (planar.magnitude < .8f)
        {
            slopeReleaseLastSampleValid = false;
            return;
        }
        if (slopeReleaseLastSampleValid)
        {
            Vector3 a = (planar - slopeReleaseLastSampleVelocity) /
                Mathf.Max(.001f, Time.fixedDeltaTime);
            if (a.sqrMagnitude < 250f * 250f && IsFiniteVector(a))
                slopeReleaseObservedAcceleration = Vector3.Lerp(
                    slopeReleaseObservedAcceleration, a, .25f);
        }
        else
            slopeReleaseObservedAcceleration = Vector3.ProjectOnPlane(
                acceleration, Vector3.up);
        slopeReleaseLastSampleVelocity = planar;
        slopeReleaseLastSampleValid = true;
    }

    // A candidate is a cached lane on the IMMEDIATELY FOLLOWING flat, not an
    // arbitrary globally nearest spline. Every lateral sample and an ahead
    // sample must be on the same physically verified floor collider.
    bool TryChooseSlopeReleaseEscapeLane(
        out Vector3 linePoint, out Vector3 heading,
        out NearestKnotDetector.FiveLineGroup chosenGroup)
    {
        linePoint = default;
        heading = default;
        chosenGroup = NearestKnotDetector.FiveLineGroup.Center;
        if (!rb || !knotDetector || !slopeReleaseHoldNextFlatCollider ||
            !slopeReleaseHoldNextFlatCollider.enabled ||
            !knotDetector.TryGetFollowingFlatStart(
                slopeReleaseCommittedSlopeGuide, out Vector3 flatStart,
                out Vector3 flatTangent))
            return false;

        heading = NormalizeFlat(flatTangent, autoHoldLastForward);
        if (Vector3.Dot(heading,
            NormalizeFlat(autoHoldLastForward, direction)) < .85f)
            return false;
        float along = Mathf.Max(.55f,
            Vector3.Dot(rb.position - flatStart, heading) + .35f);
        if (!knotDetector.TryGetFollowingFlatFiveLineFrame(
                slopeReleaseCommittedSlopeGuide, heading, along,
                out NearestKnotDetector.FiveLineFrame frame) || !frame.valid)
        {
            // A recently generated corridor may not have three parallel
            // lanes cached yet. The very same slope's directly following
            // flat segment is a real spline too; validate its whole footprint
            // rather than inventing a center line or teleporting to it.
            Vector3 onlyLine = flatStart + heading * along;
            if (!IsSlopeReleaseEscapePathSupported(
                    rb.position, onlyLine, heading,
                    slopeReleaseHoldNextFlatCollider))
                return false;
            linePoint = onlyLine;
            chosenGroup = NearestKnotDetector.FiveLineGroup.Center;
            return true;
        }

        Vector3 side = Vector3.Cross(Vector3.up, heading).normalized;
        float beforeLateralSpeed = Vector3.Dot(slopeReleaseBeforeStopVelocity, side);
        float beforeLateralAccel = Vector3.Dot(slopeReleaseBeforeStopAcceleration, side);
        // Short forecast: a strong previous outward push suggests an inward
        // recovery lane. Score remains center-biased to avoid an edge trap.
        const float forecastSeconds = .24f;
        float predicted = Vector3.Dot(rb.position - frame.center, side) +
            beforeLateralSpeed * forecastSeconds +
            .5f * beforeLateralAccel * forecastSeconds * forecastSeconds;
        predicted = Mathf.Clamp(predicted, -3.5f, 3.5f);
        float bestScore = float.PositiveInfinity;
        for (int i = 0; i < 3; ++i)
        {
            NearestKnotDetector.FiveLineGroup group = i == 0
                ? NearestKnotDetector.FiveLineGroup.Left
                : i == 1 ? NearestKnotDetector.FiveLineGroup.Center
                    : NearestKnotDetector.FiveLineGroup.Right;
            Vector3 candidate = frame.GetPoint(group);
            // Verify floor under lane, and no gap while moving across to it.
            if (!IsSlopeReleaseEscapePathSupported(
                    rb.position, candidate, heading,
                    slopeReleaseHoldNextFlatCollider))
                continue;
            float offset = Vector3.Dot(candidate - frame.center, side);
            float now = Vector3.Dot(rb.position - frame.center, side);
            float score = Mathf.Abs(offset - predicted) * .60f +
                Mathf.Abs(offset) * .85f + // center has safer edge clearance
                Mathf.Abs(offset - now) * .20f;
            if (score >= bestScore) continue;
            bestScore = score;
            linePoint = candidate;
            chosenGroup = group;
        }
        return !float.IsInfinity(bestScore);
    }

    bool IsSlopeReleaseEscapePathSupported(
        Vector3 from, Vector3 target, Vector3 heading, Collider slab)
    {
        if (!slab || !slab.enabled) return false;
        Vector3 destination = target + Vector3.up * (ProbeRadius + .30f);
        Vector3 planarDelta = Vector3.ProjectOnPlane(destination - from, Vector3.up);
        // Don't jump laterally over a gap or validate a distant unrelated line.
        if (planarDelta.magnitude > 4f) return false;
        int samples = Mathf.Max(2,
            Mathf.CeilToInt(planarDelta.magnitude / (ProbeRadius * .60f)));
        for (int i = 0; i <= samples; i++)
        {
            Vector3 point = Vector3.Lerp(from, destination,
                i / (float)samples);
            if (!TryAutoHoldGroundRay(point, out RaycastHit hit) ||
                hit.collider != slab ||
                Vector3.Angle(hit.normal, Vector3.up) > 18f)
                return false;
        }
        Vector3 ahead = destination + heading * (ProbeRadius + .35f);
        return TryAutoHoldGroundRay(ahead, out RaycastHit front) &&
            front.collider == slab &&
            Vector3.Angle(front.normal, Vector3.up) <= 18f;
    }

    bool TryStartSlopeReleaseTimedEscape()
    {
        if (!slopeReleaseHoldHandoffHeld || !slopeReleaseHoldTurnPending ||
            slopeReleaseEscapeActive || !rb || rb.isKinematic ||
            MainGameManager.OnDead)
            return false;
        if (!TryChooseSlopeReleaseEscapeLane(
                out Vector3 target, out Vector3 heading,
                out NearestKnotDetector.FiveLineGroup group))
        {
            if (Time.fixedTime >= slopeReleaseEscapeNextNoLaneLogFixedTime)
            {
                slopeReleaseEscapeNextNoLaneLogFixedTime = Time.fixedTime + .25f;
                SlopeReleaseHoldLog("ESCAPE NO SAFE LANE",
                    $"stopped={Time.fixedTime - slopeReleaseStopStartFixedTime:F3}s " +
                    "no continuous same-collider footprint; turn still pending");
            }
            return false;
        }
        // Stop waiting for an unavailable turn, preserving a safe way forward.
        // NEVER re-use old handoff position, force a blind 90deg turn, or snap
        // the ball onto a spline without a matching real floor.
        Collider stableFloor = slopeReleaseHoldNextFlatCollider;
        SlopeReleaseHoldLog("TIMED ESCAPE",
            $"stopped={Time.fixedTime - slopeReleaseStopStartFixedTime:F3}s " +
            $"lane={group} beforeVelocity={slopeReleaseBeforeStopVelocity:F3} " +
            $"beforeAccel={slopeReleaseBeforeStopAcceleration:F3} " +
            $"target={target:F3}; pending turn canceled for safety");
        ExpirePendingTurnIntent(true);
        ClearPostAutoHoldLeftEdge();
        StopEdgeCruise("timed slope release escape");
        ResetBallVisualSplineSession();
        slopeReleaseEscapeFloor = stableFloor;
        slopeReleaseEscapeLinePoint = target;
        slopeReleaseEscapeHeading = heading;
        slopeReleaseEscapeGroup = group;
        slopeReleaseEscapeStartedFixedTime = Time.fixedTime;
        slopeReleaseEscapeActive = true;
        return true;
    }

    bool StepSlopeReleaseTimedEscape()
    {
        if (!slopeReleaseEscapeActive || !rb) return false;
        var contact = HasGroundSupport();
        if (MainGameManager.OnDead || rb.isKinematic ||
            !contact.Item1 || contact.Item2.collider != slopeReleaseEscapeFloor ||
            Vector3.Angle(contact.Item2.normal, Vector3.up) > 18f)
        {
            ResetSlopeReleaseTimedEscape("support lost / state changed");
            return false; // run the ordinary dropout/support logic instead
        }
        Vector3 heading = slopeReleaseEscapeHeading;
        Vector3 side = Vector3.Cross(Vector3.up, heading).normalized;
        float error = Vector3.Dot(slopeReleaseEscapeLinePoint - rb.position, side);
        float vSide = Vector3.Dot(rb.velocity, side);
        float vForward = Vector3.Dot(rb.velocity, heading);
        float sidewaysCap = Mathf.Min(65f,
            Mathf.Max(1f, slopeReleaseEscapeSideAcceleration));
        float sideSpeed = Mathf.Sign(error) * Mathf.Min(3f,
            Mathf.Min(Mathf.Abs(error) * 5f,
                Mathf.Sqrt(2f * sidewaysCap * Mathf.Abs(error))));
        float sideAccel = Mathf.Clamp((sideSpeed - vSide) * 12f,
            -sidewaysCap, sidewaysCap);
        float formerForwardSpeed = Mathf.Max(0f,
            Vector3.Dot(slopeReleaseBeforeStopVelocity, heading));
        float formerForwardAcceleration = Mathf.Max(0f,
            Vector3.Dot(slopeReleaseBeforeStopAcceleration, heading));
        float desiredForwardSpeed = Mathf.Clamp(
            2.5f + formerForwardSpeed * .15f +
            formerForwardAcceleration * .04f, 2.5f, 6.5f);
        float forwardAccel = Mathf.Clamp(
            (desiredForwardSpeed - vForward) * 8f,
            -slopeReleaseEscapeForwardAcceleration,
            slopeReleaseEscapeForwardAcceleration);

        // Recheck the actual receiving slab just ahead, each physics frame.
        // This prevents the recovery motor from accelerating off the next lip.
        Vector3 ahead = rb.position + heading *
            Mathf.Max(ProbeRadius + .25f,
                Mathf.Max(0f, vForward) * .12f + ProbeRadius);
        if (!TryAutoHoldGroundRay(ahead, out RaycastHit front) ||
            front.collider != slopeReleaseEscapeFloor ||
            Vector3.Angle(front.normal, Vector3.up) > 18f)
            forwardAccel = Mathf.Min(0f, forwardAccel);
        if (!IsSlopeReleaseEscapePathSupported(rb.position,
            slopeReleaseEscapeLinePoint + heading *
                Vector3.Dot(rb.position - slopeReleaseEscapeLinePoint, heading),
            heading, slopeReleaseEscapeFloor))
            sideAccel = -Mathf.Clamp(vSide * 10f, -sidewaysCap, sidewaysCap);

        rb.AddForce(heading * forwardAccel + side * sideAccel -
            Vector3.up * Mathf.Max(0f, flatStick), ForceMode.Acceleration);
        direction = heading;
        // Keep flat drive history; controlled escape supplies its own force.
        // Resume regular spline drive once the supported recovery lane is
        // reached and the ball has recovered some forward speed.
        float elapsed = Time.fixedTime - slopeReleaseEscapeStartedFixedTime;
        if ((Mathf.Abs(error) < .30f && Mathf.Abs(vSide) < .9f &&
             vForward >= 2f) || elapsed > 1.20f)
        {
            ResetSlopeReleaseTimedEscape("lane reached / bounded escape duration");
            return true;
        }
        return true;
    }

    void ResetSlopeReleaseTimedEscape(string reason)
    {
        if (slopeReleaseEscapeActive)
            SlopeReleaseHoldLog("ESCAPE END",
                $"reason={reason} lane={slopeReleaseEscapeGroup} " +
                $"pos={(rb ? rb.position : Vector3.zero):F3}");
        slopeReleaseEscapeActive = false;
        slopeReleaseEscapeFloor = null;
        slopeReleaseCommittedSlopeGuide = default;
        slopeReleaseStopStartFixedTime = -1f;
    }

    // Dedicated diagnostics, so a test run can be filtered without mixing
    // it with existing flat-start Auto Hold or TAP EDGE logs.
    void SlopeReleaseHoldLog(string phase, string details)
    {
        if (logTurnTransition)
            Debug.Log($"[SLOPE RELEASE HOLD {phase}] t={Time.time:F3} {details}", this);
    }

    void ObserveSlopeReleaseHoldGuide(NearestKnotDetector.GuideFrame guide)
    {
        if (!slopeReleaseHoldArmed || !knotDetector || !rb || !guide.valid)
            return;

        if (guide.isSlope)
        {
            // A second slope means the one immediately following the held
            // slope's flat was missed. Never arm a later accidental release.
            // Three lane splines are different section IDs even when they
            // represent the SAME physical stair. The left-edge approach can
            // naturally change the nearest lane; do not treat it as the next
            // stair merely because splineIndex changed.
            bool sameLogicalSlope = knotDetector.IsSameSection(
                slopeReleaseHoldStartGuide, guide);
            if (!sameLogicalSlope)
            {
                Vector3 from = NormalizeFlat(
                    slopeReleaseHoldStartGuide.tangent, direction);
                Vector3 to = NormalizeFlat(guide.tangent, direction);
                bool aligned = Vector3.Dot(from, to) > .90f;
                bool nearbyProgress = Mathf.Abs(guide.sectionProgress01 -
                    slopeReleaseHoldMaxProgress) <= .40f;
                bool samePhysicalCorridor = Vector3.Distance(
                    Vector3.ProjectOnPlane(guide.point, Vector3.up),
                    Vector3.ProjectOnPlane(rb.position, Vector3.up)) <=
                    Mathf.Max(4f, autoHoldExitWidthTolerance + ProbeRadius);
                sameLogicalSlope = aligned && nearbyProgress &&
                    samePhysicalCorridor;
            }
            if (slopeReleaseHoldNextFlatSeen || !sameLogicalSlope)
            {
                CancelAutoHold("reached another slope without releasing on the next flat");
                return;
            }
            slopeReleaseHoldMaxProgress = Mathf.Max(
                slopeReleaseHoldMaxProgress, guide.sectionProgress01);
            if (guide.sectionProgress01 >= TargetProgress)
                slopeReleaseExtremeReachedWhileHeld = true;
            ObserveAutoHoldSlope(guide);
            return;
        }

        if (slopeReleaseHoldNextFlatSeen)
        {
            // Refresh the last REAL flat support sample during the hold.
            // A short airborne release immediately afterwards may reuse it.
            var stillSupported = HasGroundSupport();
            if (stillSupported.Item1 &&
                stillSupported.Item2.collider == slopeReleaseHoldNextFlatCollider &&
                IsAutoHoldStageHit(stillSupported.Item2, float.NaN) &&
                Vector3.Angle(stillSupported.Item2.normal, Vector3.up) <= 18f)
            {
                slopeReleaseHoldLastFlatSupportTime = Time.time;
                slopeReleaseHoldLastFlatSupportPosition = rb.position;
            }
            return;
        }
        if (!autoHoldSawSlope || !autoHoldLastHalfValid)
            return;

        Vector3 oldHeading = autoHoldLastForward;
        Vector3 flatHeading = NormalizeFlat(guide.tangent, oldHeading);
        float forwardAtExit = Vector3.Dot(
            rb.position - autoHoldExitPoint, oldHeading);
        bool validNextFlat = Vector3.Dot(flatHeading, oldHeading) > .8f &&
            forwardAtExit >= -.4f &&
            (slopeReleaseHoldMaxProgress >= .80f || forwardAtExit >= .15f);
        if (!validNextFlat)
            return;

        // A nearest Spline is not enough. Require an actual stage collider
        // supporting the body on a horizontal surface (not BallVisual/Render).
        var support = HasGroundSupport();
        bool verifiedFlat = support.Item1 &&
            IsAutoHoldStageHit(support.Item2, float.NaN) &&
            Vector3.Angle(support.Item2.normal, Vector3.up) <= 18f;
        if (!verifiedFlat)
            return;

        // Verify that this flat follows the held slope in the cached section,
        // not a nearby branch with a coincidentally aligned tangent.
        if (!knotDetector.TryGetFollowingFlatStart(
                slopeReleaseHoldStartGuide, out Vector3 flatStart,
                out Vector3 nextFlatTangent) ||
            Vector3.Dot(NormalizeFlat(nextFlatTangent, oldHeading),
                oldHeading) <= .8f ||
            Vector3.Dot(rb.position - flatStart, oldHeading) < -.45f)
            return;

        slopeReleaseHoldNextFlatSeen = true;
        slopeReleaseHoldNextFlatCollider = support.Item2.collider;
        slopeReleaseHoldLastFlatSupportTime = Time.time;
        slopeReleaseHoldLastFlatSupportPosition = rb.position;
        SlopeReleaseHoldLog("NEXT FLAT VERIFIED",
            $"collider={slopeReleaseHoldNextFlatCollider.name} " +
            $"progress={slopeReleaseHoldMaxProgress:F3} " +
            $"exit={forwardAtExit:F3}m " +
            $"leftOffset={autoHoldLeftOffset:F3}m " +
            $"speed={rb.velocity.magnitude:F2}");
    }

    bool IsSlopeReleaseHoldOnVerifiedFlat(bool grounded, RaycastHit hit)
    {
        if (!slopeReleaseHoldNextFlatCollider || !rb)
            return false;
        if (grounded &&
            hit.collider == slopeReleaseHoldNextFlatCollider &&
            IsAutoHoldStageHit(hit, float.NaN) &&
            Vector3.Angle(hit.normal, Vector3.up) <= 18f)
            return true;

        // From the successful flat->slope->flat hold experiments: a ball can
        // briefly be above its landing flat. Permit ONLY a very short grace
        // after confirmed support and only while a stage floor is still below.
        if (slopeReleaseHoldLastFlatSupportTime < 0f ||
            Time.time - slopeReleaseHoldLastFlatSupportTime > .10f ||
            !slopeReleaseHoldNextFlatCollider.enabled ||
            !knotDetector || !knotDetector.CurrentGuide.valid ||
            knotDetector.CurrentGuide.isSlope)
            return false;
        Vector3 delta = rb.position - slopeReleaseHoldLastFlatSupportPosition;
        if (Vector3.ProjectOnPlane(delta, Vector3.up).magnitude > 2.8f ||
            Mathf.Abs(delta.y) > 1.2f)
            return false;
        return TryAutoHoldGroundRay(rb.worldCenterOfMass,
            out RaycastHit nearbyFloor) &&
            nearbyFloor.collider == slopeReleaseHoldNextFlatCollider &&
            Vector3.Angle(nearbyFloor.normal, Vector3.up) <= 18f;
    }

    bool TryCommitSlopeReleaseHoldOnFlat()
    {
        if (!slopeReleaseHoldArmed || !autoHoldEligible || !rb ||
            rb.isKinematic || MainGameManager.OnDead ||
            !BeginCommandOnTouch || !Earliest || !AcceleFirstTurn ||
            turnTransitionActive || waitingForTurnGuide || HasPendingTurnIntent)
            return false;

        // Update may receive MouseUp/Touch.Ended before the next FixedUpdate.
        // Re-evaluate at the physical release position to avoid a lost frame.
        if (knotDetector)
            ObserveSlopeReleaseHoldGuide(knotDetector.Evaluate(rb.position));
        if (!slopeReleaseHoldNextFlatSeen || !slopeReleaseHoldNextFlatCollider)
            return false;

        var support = HasGroundSupport();
        if (!IsSlopeReleaseHoldOnVerifiedFlat(support.Item1, support.Item2))
        {
            SlopeReleaseHoldLog("RELEASE REJECTED",
                $"not on verified next flat; hit={(support.Item2.collider ? support.Item2.collider.name : "-")}");
            return false;
        }

        // Geometry, not pressed half, selects the corner. A right/out turn
        // is accepted too, without LEFT-lip positioning or left-edge snap.
        if (!TryResolveAutoHoldTurnSign(out float sign, out string signSource))
        {
            SlopeReleaseHoldLog("RELEASE REJECTED",
                "next turn direction is not verified; no guessed turn");
            return false;
        }
        bool innerTurn = sign < 0f;
        autoHoldTurnSign = sign;
        autoHoldTurnLeft = innerTurn ? autoHoldLastLeft : Vector3.zero;
        autoHoldInnerErrorAtCommit = Vector3.Dot(
            rb.position - autoHoldLastInnerLinePoint, autoHoldLastLeft);
        autoHoldEdgeSampleValid = false;
        if (innerTurn && TryResolveAutoHoldEdge(autoHoldTurnLeft, out Vector3 edge))
        {
            autoHoldEdgeSample = edge;
            autoHoldEdgeSampleValid = true;
        }

        // The original long-hold commit preserves its edge snapshot through
        // Handoff, and the post-turn correction is limited to the next slope.
        slopeReleaseHoldTurnPending = true;
        autoHoldTurnPending = true;
        slopeReleaseCommittedSlopeGuide = slopeReleaseHoldStartGuide;
        slopeReleaseStopStartFixedTime = -1f;
        slopeReleaseEscapeNextNoLaneLogFixedTime = 0f;
        slopeReleaseBeforeStopVelocity = rb.velocity;
        // Use the actual measured pre-stop acceleration when available,
        // blended with this frame's physical drive/steering demand.
        Vector3 commanded = Vector3.ProjectOnPlane(acceleration, Vector3.up);
        slopeReleaseBeforeStopAcceleration = slopeReleaseLastSampleValid
            ? Vector3.Lerp(commanded, slopeReleaseObservedAcceleration, .40f)
            : commanded;
        slopeReleaseBeforeStopAcceleration = Vector3.ClampMagnitude(
            slopeReleaseBeforeStopAcceleration, 120f);
        bool committedAfterExtreme = slopeReleaseStartedAfterExtreme;
        bool reachedExtreme = slopeReleaseExtremeReachedWhileHeld;

        pendingTurnFromPassedSlope = false;
        pendingTurnDegrees = sign * QuarterTurnDegrees;
        pendingTurnQueuedTime = Time.time;
        ResetTapEdge();
        StopEdgeCruise("slope release hold queued");
        ClearPostAutoHoldLeftEdge();
        flickConsumed = true;
        CapturePendingTurnSnapshot();
        RequestTurnHandoffForPendingIntent();
        SlopeReleaseHoldLog("TURN QUEUED",
            $"corner={(innerTurn ? "InCorner" : "OutCorner")} " +
            $"half={(autoHoldLeftHalf ? "LEFT" : "RIGHT")} " +
            $"sign={sign:+0;-0} signSource={signSource} angle=90 " +
            $"postExtremePress={committedAfterExtreme} " +
            $"extremeReached={reachedExtreme} " +
            $"leftOffset={autoHoldLeftOffset:F3}m " +
            $"innerError={autoHoldInnerErrorAtCommit:F3}m " +
            $"velocity={rb.velocity:F3} " +
            $"preStopAccel={slopeReleaseBeforeStopAcceleration:F3}");
        ResetAutoHoldObservation(false);
        return true;
    }

    // Physical receiving-flat evidence is authoritative for flat-start holds.
    // The nearest Spline may already point into the following stair even while
    // the ball is still supported by the actual receiving slab.
    enum AutoHoldLandingFloor { VerifiedFlat, Uncertain, LeftFlat }

    bool TryGetAutoHoldReceivingFlat(out Vector3 flatStart, out Vector3 forward)
    {
        flatStart = default;
        forward = Vector3.zero;
        if (!knotDetector || !rb || !autoHoldLastSlopeGuide.valid ||
            !knotDetector.TryGetFollowingFlatStart(autoHoldLastSlopeGuide,
                out flatStart, out Vector3 flatTangent))
            return false;
        forward = NormalizeFlat(autoHoldLastForward, direction);
        return Vector3.Dot(NormalizeFlat(flatTangent, forward), forward) > .80f;
    }

    // Diagnostic ONLY. Passing a mathematical Spline entry is not evidence
    // that the physical flat slab has ended, and must never veto a turn.
    float AutoHoldPastNextSplineEntryMeters(Vector3 flatStart, Vector3 forward)
    {
        if (!knotDetector ||
            !knotDetector.TryEvaluateSameSection(autoHoldLastSlopeGuide, 1f,
                out NearestKnotDetector.GuideSample lastSlope) ||
            !lastSlope.valid)
            return float.NaN;
        var followingFlat = new NearestKnotDetector.GuideFrame
        {
            valid = true,
            isSlope = false,
            splineIndex = lastSlope.splineIndex,
            segmentIndex = lastSlope.segmentIndex + 1,
            point = flatStart,
            tangent = forward
        };
        if (!knotDetector.TryEvaluateForwardSlopeSection(followingFlat, 0f,
                out NearestKnotDetector.GuideSample followingSlope) ||
            !followingSlope.valid ||
            Vector3.Dot(followingSlope.point - flatStart, forward) < .60f)
            return float.NaN;
        return Vector3.Dot(rb.position - followingSlope.point, forward);
    }

    // Preview one physics step ahead at the present speed. This is a
    // diagnostic of remaining platform support, NOT an extra veto while
    // the sphere still has real receiving-flat support under its centre.
    bool HasAutoHoldFlatForwardPreview(Vector3 flatStart, Vector3 forward)
    {
        float ahead = Mathf.Max(0f, Vector3.Dot(rb.velocity, forward)) *
            Time.fixedDeltaTime + ProbeRadius;
        Vector3 sample = rb.worldCenterOfMass + forward * ahead;
        return TryAutoHoldGroundRay(sample, out RaycastHit floor, flatStart.y) &&
            IsCloseGroundRay(floor, sample) &&
            Vector3.Angle(floor.normal, Vector3.up) <= 18f &&
            Mathf.Abs(floor.point.y - flatStart.y) <= .32f;
    }

    // One common judgement for FixedUpdate (hold) and Update (release).
    // A wide probeDistance cannot turn a floor many metres below into a
    // verified flat: we require a close physical hit and landing-slab height.
    AutoHoldLandingFloor ClassifyFlatStartAutoHoldNextFlat(
        NearestKnotDetector.GuideFrame guide, out RaycastHit physicalFloor,
        out string reason)
    {
        physicalFloor = default;
        reason = "awaiting receiving flat";
        if (!autoHoldEligible || slopeReleaseHoldArmed ||
            !autoHoldSawSlope || !autoHoldLastHalfValid || !rb ||
            !TryGetAutoHoldReceivingFlat(out Vector3 flatStart,
                out Vector3 forward))
            return AutoHoldLandingFloor.Uncertain;

        float fromExit = Vector3.Dot(rb.position - autoHoldExitPoint, forward);
        float fromStart = Vector3.Dot(rb.position - flatStart, forward);
        bool nextFlatGuide = guide.valid && !guide.isSlope &&
            Vector3.Dot(NormalizeFlat(guide.tangent, forward), forward) > .80f;
        bool exitedSlope = autoHoldLastSlopeProgress >= .80f && fromExit >= .15f;
        // The current physics slab can begin a few centimetres BEFORE its
        // spline flat-start sample. Let a near-entry flat-guide candidate
        // reach the mandatory physical floor + entry-anchor verification.
        bool flatEntrySeam = nextFlatGuide &&
            fromExit >= -ProbeRadius * .5f &&
            fromStart >= -ProbeRadius * .5f;
        if (!autoHoldNextFlatReady &&
            !(((nextFlatGuide || exitedSlope) &&
                fromExit >= .05f && fromStart >= .05f) || flatEntrySeam))
            return AutoHoldLandingFloor.Uncertain;

        // A stair's horizontal tread can also have an UP normal. The actual
        // supporting floor must be near the receiving flat's Y plane.
        const float maximumLandingHeightError = .32f;
        Vector3 centre = rb.worldCenterOfMass;
        bool centreRay = TryAutoHoldGroundRay(centre, out RaycastHit under,
                flatStart.y) && IsCloseGroundRay(under, centre) &&
            Vector3.Angle(under.normal, Vector3.up) <= 18f &&
            Mathf.Abs(under.point.y - flatStart.y) <= maximumLandingHeightError;
        var support = HasGroundSupport();
        bool closeSupport = support.Item1 &&
            IsCloseGroundRay(support.Item2, centre);
        bool supportFlat = closeSupport &&
            Vector3.Angle(support.Item2.normal, Vector3.up) <= 18f &&
            Mathf.Abs(support.Item2.point.y - flatStart.y) <=
                maximumLandingHeightError;

        if (centreRay || supportFlat)
        {
            physicalFloor = centreRay ? under : support.Item2;
            // At first arrival, anchor to the exact following flat, not a
            // final horizontal tread of the preceding stairway.
            if (!autoHoldNextFlatReady)
            {
                Vector3 probe = flatStart + forward * (ProbeRadius * 1.6f);
                probe.y = flatStart.y + ProbeRadius + .35f;
                if (!TryAutoHoldGroundRay(probe, out RaycastHit entryFloor,
                        flatStart.y) ||
                    Vector3.Angle(entryFloor.normal, Vector3.up) > 18f ||
                    Mathf.Abs(entryFloor.point.y - flatStart.y) >
                        maximumLandingHeightError)
                {
                    reason = "receiving flat anchor not confirmed";
                    return AutoHoldLandingFloor.Uncertain;
                }
                // Different adjacent MeshColliders on one continuous level
                // can represent the same receiving platform. Do not demand
                // reference equality when both are directly verified.
                if (physicalFloor.collider != entryFloor.collider)
                {
                    Vector3 behind = centre - forward * (ProbeRadius * .5f);
                    if (!TryAutoHoldGroundRay(behind, out RaycastHit bridge,
                            flatStart.y) ||
                        !IsCloseGroundRay(bridge, behind) ||
                        Mathf.Abs(bridge.point.y - flatStart.y) >
                            maximumLandingHeightError ||
                        (bridge.collider != physicalFloor.collider &&
                         bridge.collider != entryFloor.collider))
                    {
                        reason = "receiving flat colliders not continuous";
                        return AutoHoldLandingFloor.Uncertain;
                    }
                }
            }
            else if (physicalFloor.collider != autoHoldNextFlatCollider)
            {
                // Permit a seam to the next same-height slab only while
                // physical support samples remain spatially continuous.
                float since = Time.time - autoHoldNextFlatLastSupportTime;
                float moved = Vector3.ProjectOnPlane(rb.position -
                    autoHoldNextFlatLastSupportPosition, Vector3.up).magnitude;
                float allowedTravel = Mathf.Max(ProbeRadius * 2f,
                    rb.velocity.magnitude * Mathf.Max(0f, since) + ProbeRadius);
                // Once the actual receiving flat has been proven, a different
                // collider on its same-height surface is not automatically a
                // new stair. A valid aligned FLAT guide plus close physics
                // contact is stronger evidence than collider identity.
                bool stillOnFlatGuide = guide.valid && !guide.isSlope &&
                    Vector3.Dot(NormalizeFlat(guide.tangent, forward),
                        forward) > .80f &&
                    fromStart >= -ProbeRadius * .5f;
                if (!stillOnFlatGuide &&
                    (!autoHoldNextFlatCollider ||
                     !autoHoldNextFlatCollider.enabled ||
                     autoHoldNextFlatLastSupportTime < 0f || since > .12f ||
                     moved > allowedTravel))
                {
                    reason = "new slab not verified continuous with receiving flat";
                    return AutoHoldLandingFloor.Uncertain;
                }
            }
            reason = physicalFloor.collider == autoHoldNextFlatCollider
                ? "close physical receiving slab"
                : "close continuous physical slab";
            return AutoHoldLandingFloor.VerifiedFlat;
        }

        // A nearby slope/other-height tread is evidence of departure ONLY
        // after the receiving flat was verified. Before that, it may simply
        // be the last step of the ORIGINAL held stairway.
        if (closeSupport &&
            (Vector3.Angle(support.Item2.normal, Vector3.up) > 18f ||
             Mathf.Abs(support.Item2.point.y - flatStart.y) >
                 maximumLandingHeightError))
        {
            if (!autoHoldNextFlatReady)
            {
                bool newSlope = guide.valid && guide.isSlope &&
                    !knotDetector.IsSameSection(autoHoldLastSlopeGuide, guide);
                if (!newSlope || fromStart < ProbeRadius * 3f)
                {
                    reason = "still approaching receiving flat from original stair";
                    return AutoHoldLandingFloor.Uncertain;
                }
            }
            else
            {
                // A sphere cast can briefly favor the first next-stair
                // tread while a small part of the sphere remains on the
                // platform. Do not cancel until this rear floor disappears.
                Vector3 behind = centre - forward * (ProbeRadius * .5f);
                if (TryAutoHoldGroundRay(behind, out RaycastHit rear,
                        flatStart.y) && IsCloseGroundRay(rear, behind) &&
                    Vector3.Angle(rear.normal, Vector3.up) <= 18f &&
                    Mathf.Abs(rear.point.y - flatStart.y) <=
                        maximumLandingHeightError)
                {
                    reason = "platform edge contact; checking next frame";
                    return AutoHoldLandingFloor.Uncertain;
                }
            }
            reason = "physically entered a different-height/slope surface";
            return AutoHoldLandingFloor.LeftFlat;
        }

        // Do not cancel on a one-frame ray/sphere miss near a MeshCollider
        // seam. This grace NEVER authorizes a release turn: release needs
        // VerifiedFlat at the CURRENT physical position.
        if (autoHoldNextFlatReady && autoHoldNextFlatLastSupportTime >= 0f)
        {
            float missing = Time.time - autoHoldNextFlatLastSupportTime;
            float moved = Vector3.ProjectOnPlane(rb.position -
                autoHoldNextFlatLastSupportPosition, Vector3.up).magnitude;
            // Hold only: brief ray misses can span several FixedUpdate ticks
            // at full speed. RELEASE still requires current VerifiedFlat.
            if (missing > .16f ||
                moved > Mathf.Max(ProbeRadius * 1.5f,
                    rb.velocity.magnitude * .16f + ProbeRadius))
            {
                reason = "physical receiving flat support lost";
                return AutoHoldLandingFloor.LeftFlat;
            }
            reason = "brief physical floor miss; keep hold pending";
        }
        return AutoHoldLandingFloor.Uncertain;
    }

    AutoHoldLandingFloor ObserveFlatStartAutoHoldNextFlat(
        NearestKnotDetector.GuideFrame guide, out string reason)
    {
        AutoHoldLandingFloor state = ClassifyFlatStartAutoHoldNextFlat(
            guide, out RaycastHit floor, out reason);
        if (state != AutoHoldLandingFloor.VerifiedFlat)
            return state;
        if (!autoHoldNextFlatReady)
        {
            autoHoldNextFlatReady = true;
            autoHoldNextFlatCollider = floor.collider;
            bool preview = TryGetAutoHoldReceivingFlat(
                out Vector3 flatStart, out Vector3 forward) &&
                HasAutoHoldFlatForwardPreview(flatStart, forward);
            AutoHoldLog("NEXT FLAT READY",
                $"collider={floor.collider.name} " +
                $"physicalY={floor.point.y:F2} reason={reason} " +
                $"forwardPreview={preview} " +
                $"half={(autoHoldLeftHalf ? "LEFT" : "RIGHT")} " +
                $"speed={rb.velocity.magnitude:F2}; waiting for finger release");
        }
        autoHoldNextFlatLastSupportTime = Time.time;
        autoHoldNextFlatLastSupportPosition = rb.position;
        return state;
    }

    void ClearBReleaseHandoff()
    {
        bReleaseHandoffActive = false;
        bReleaseHandoffFlat = null;
        bReleaseOriginalHeading = Vector3.zero;
        bReleaseForwardSpeed = 0f;
        bReleaseHandoffTime = -1f;
        bReleaseNextLogTime = 0f;
    }

    bool TryGetBReleaseReceivingSupport(out RaycastHit verified)
    {
        verified = default;
        if (!bReleaseHandoffActive || !bReleaseHandoffFlat ||
            !bReleaseHandoffFlat.enabled || !rb)
            return false;

        Vector3 centre = rb.worldCenterOfMass;
        Vector3 forward = NormalizeFlat(bReleaseOriginalHeading, direction);
        Vector3 side = Vector3.Cross(Vector3.up, forward).normalized;
        for (int i = 0; i < 4; ++i)
        {
            Vector3 offset = i == 0 ? Vector3.zero :
                i == 1 ? -forward * (ProbeRadius * .35f) :
                i == 2 ? side * (ProbeRadius * .35f) :
                         -side * (ProbeRadius * .35f);
            Vector3 sample = centre + offset;
            Ray ray = new Ray(sample + Vector3.up * .45f, Vector3.down);
            if (bReleaseHandoffFlat.Raycast(ray, out RaycastHit hit,
                    probeDistance + ProbeRadius + .95f) &&
                hit.collider == bReleaseHandoffFlat &&
                IsCloseGroundRay(hit, sample) &&
                Vector3.Angle(hit.normal, Vector3.up) <= 18f)
            {
                verified = hit;
                return true;
            }
        }
        return false;
    }

    // Waiting for BallVisual/Handoff is NOT an opportunity to steer with an
    // old perpendicular spline or to brake both components to zero.
    // This handles only the genuine, still-supported receiving flat.
    bool StepPendingBReleaseHandoff()
    {
        if (!bReleaseHandoffActive || !autoHoldTurnPending ||
            !HasPendingTurnIntent || turnTransitionActive ||
            waitingForTurnGuide || !BeginCommandOnTouch ||
            !rb || rb.isKinematic)
            return false;
        if (!TryGetBReleaseReceivingSupport(out RaycastHit floor))
        {
            // The original flat ended. A deferred B turn must not fire in air
            // or on an unrelated stair, even if a visual tween finishes later.
            AutoHoldLog("B HANDOFF SUPPORT LOST",
                $"pos={rb.position:F3} expected={(bReleaseHandoffFlat ? bReleaseHandoffFlat.name : "-")}");
            ExpirePendingTurnIntent();
            ClearBReleaseHandoff();
            return false;
        }

        Vector3 heading = NormalizeFlat(bReleaseOriginalHeading, direction);
        Vector3 planar = Vector3.ProjectOnPlane(rb.velocity, Vector3.up);
        float along = Vector3.Dot(planar, heading);
        float target = Mathf.Clamp(bReleaseForwardSpeed, 0f,
            Mathf.Min(BReleaseHandoffSpeedCeiling,
                Mathf.Max(0f, maxGroundSpeed)));
        float forward = Mathf.MoveTowards(along, target,
            BReleaseHandoffBrakeAcceleration * Time.fixedDeltaTime);
        Vector3 sideVelocity = planar - heading * along;
        Vector3 side = Vector3.MoveTowards(sideVelocity, Vector3.zero,
            BReleaseHandoffSideAcceleration * Time.fixedDeltaTime);
        rb.velocity = heading * Mathf.Max(0f, forward) + side +
            Vector3.up * rb.velocity.y;
        GuideWaitBalanceSpin(true, floor.normal);
        // Physics gravity/contact own the supported flat; no duplicate Stick.
        stickState = Mathf.Min(stickState, Mathf.Max(0f, flatStick));
        if (Time.fixedTime >= bReleaseNextLogTime)
        {
            bReleaseNextLogTime = Time.fixedTime + .08f;
            Debug.Log($"[B RELEASE HANDOFF] t={Time.fixedTime:F3} " +
                $"forward={forward:F2} target={target:F2} " +
                $"sideBefore={sideVelocity.magnitude:F2} sideAfter={side.magnitude:F2} " +
                $"floor={floor.collider.name} queuedAt={bReleaseHandoffTime:F3}", this);
        }
        return true;
    }

    // Direct B receiving-flat ray: recent support allows rechecking,
    // but cannot authorize a turn without a current collider intersection.
    bool TryVerifyReceivingFlatAtReleaseSeam(out string reason)
    {
        reason = "receiving-flat floor not reverified";
        if (!autoHoldNextFlatReady || !autoHoldNextFlatCollider ||
            !autoHoldNextFlatCollider.enabled || !rb ||
            autoHoldNextFlatLastSupportTime < 0f ||
            Time.time - autoHoldNextFlatLastSupportTime > .14f ||
            !TryGetAutoHoldReceivingFlat(out Vector3 flatStart,
                out Vector3 forward))
            return false;

        if (Vector3.Dot(rb.position - flatStart, forward) < -ProbeRadius)
            return false;
        Vector3 centre = rb.worldCenterOfMass;
        Vector3 side = Vector3.Cross(Vector3.up, forward).normalized;
        Vector3[] offsets =
        {
            Vector3.zero, -forward * (ProbeRadius * .45f),
            side * (ProbeRadius * .45f), -side * (ProbeRadius * .45f)
        };
        foreach (Vector3 offset in offsets)
        {
            Vector3 sample = centre + offset;
            Ray ray = new Ray(sample + Vector3.up * .45f, Vector3.down);
            if (!autoHoldNextFlatCollider.Raycast(ray, out RaycastHit hit,
                    ProbeRadius + 1.2f) ||
                hit.collider != autoHoldNextFlatCollider ||
                !IsCloseGroundRay(hit, sample) ||
                Vector3.Angle(hit.normal, Vector3.up) > 18f ||
                Mathf.Abs(hit.point.y - flatStart.y) > .32f)
                continue;
            reason = "ray verified original receiving-flat collider";
            return true;
        }
        return false;
    }

    // B-only entry: authorized inside a mouse/touch release transaction.
    bool TryCommitFlatStartAutoHoldOnRelease()
    {
        if (!flatStartReleaseCommitInProgress ||
            !autoHoldEligible || slopeReleaseHoldArmed ||
            !autoHoldSawSlope || !trackingFlick || flickConsumed ||
            !BeginCommandOnTouch || !Earliest || !AcceleFirstTurn ||
            MainGameManager.OnDead || !rb || rb.isKinematic ||
            turnTransitionActive || waitingForTurnGuide ||
            HasPendingTurnIntent || generalStallRecovering)
            return false;

        // Release can arrive before FixedUpdate. Use the SAME physical-floor
        // classifier here, but never accept its Uncertain short grace.
        NearestKnotDetector.GuideFrame releaseGuide = knotDetector
            ? knotDetector.Evaluate(rb.position) : default;
        AutoHoldLandingFloor landing = ObserveFlatStartAutoHoldNextFlat(
            releaseGuide, out string floorReason);
        if (landing != AutoHoldLandingFloor.VerifiedFlat &&
            autoHoldNextFlatReady &&
            TryVerifyReceivingFlatAtReleaseSeam(out string seamReason))
        {
            landing = AutoHoldLandingFloor.VerifiedFlat;
            floorReason = seamReason;
            AutoHoldLog("RELEASE SEAM VERIFIED", floorReason);
        }
        if (landing != AutoHoldLandingFloor.VerifiedFlat ||
            !autoHoldNextFlatReady)
        {
            var currentFloor = HasGroundSupport();
            AutoHoldLog("RELEASE REJECTED",
                $"state={landing} reason={floorReason} " +
                $"expected={(autoHoldNextFlatCollider ? autoHoldNextFlatCollider.name : "-")} " +
                $"actual={(currentFloor.Item2.collider ? currentFloor.Item2.collider.name : "-")} " +
                $"pos={rb.position:F3}; no turn/flick fallback");
            return false;
        }
        // Spline entry may lie INSIDE the physical slab. Record its distance
        // to diagnose the old false veto; never use it to reject verified floor.
        if (TryGetAutoHoldReceivingFlat(out Vector3 flatStart,
                out Vector3 forward))
        {
            float overrun = AutoHoldPastNextSplineEntryMeters(flatStart, forward);
            bool preview = HasAutoHoldFlatForwardPreview(flatStart, forward);
            AutoHoldLog("RELEASE VERIFIED",
                $"physicalFlat={(autoHoldNextFlatCollider ? autoHoldNextFlatCollider.name : "-")} " +
                $"splineEntryOverrun={overrun:F2}m forwardPreview={preview} " +
                $"pos={rb.position:F3}; reason={floorReason}");
        }

        float lateral = Vector3.Dot(rb.position - autoHoldLastCenter,
            autoHoldLastLeft);
        autoHoldLeftHalf = lateral > autoHoldLeftHalfMarginMeters;
        QueueAutoHoldHeldTurn(lateral);
        bool committed = autoHoldTurnPending && HasPendingTurnIntent;
        if (committed)
            AutoHoldLog("RELEASE COMMIT",
                $"sign={autoHoldTurnSign:+0;-0} lateral={lateral:F3} " +
                $"pos={rb.position:F3}; one turn queued");
        return committed;
    }

    void ObserveAutoHoldGuide(NearestKnotDetector.GuideFrame guide)
    {
        if (!autoHoldEligible) return;
        if (!autoInCornerHoldEnabled || !BeginCommandOnTouch || !Earliest ||
            MainGameManager.OnDead || turnTransitionActive || waitingForTurnGuide ||
            HasPendingTurnIntent || generalStallRecovering)
        {
            CancelAutoHold("input/status changed");
            return;
        }
        // In FixedUpdate the mouse button (or touch) can already be released
        // before Update processes MouseUp/Ended. Do not erase the sequence
        // just before the release handler has a chance to commit it.
        if (!IsAutoHoldInputStillDown() && !IsAutoHoldReleaseAwaitingUpdate())
        {
            CancelAutoHold("input no longer down / canceled");
            return;
        }
        if (slopeReleaseHoldArmed)
        {
            // The existing stair-start release method keeps its own owner.
            ObserveSlopeReleaseHoldGuide(guide);
            return;
        }
        if (autoHoldNextFlatReady)
        {
            // Never cancel merely because the nearest Spline advanced to
            // the next stair. Ask the same physical classifier as release.
            AutoHoldLandingFloor state = ObserveFlatStartAutoHoldNextFlat(
                guide, out string reason);
            if (state == AutoHoldLandingFloor.LeftFlat)
                CancelAutoHold("left physical receiving flat: " + reason);
            return;
        }
        if (guide.valid && guide.isSlope)
        {
            if (autoHoldSawSlope && autoHoldLastSlopeGuide.valid &&
                !knotDetector.IsSameSection(autoHoldLastSlopeGuide, guide))
            {
                // Preserve the ORIGINAL held slope; a nearest-guide jump
                // must not replace the cached receiving-flat geometry.
                AutoHoldLandingFloor state = ObserveFlatStartAutoHoldNextFlat(
                    guide, out string reason);
                if (state == AutoHoldLandingFloor.LeftFlat)
                    CancelAutoHold("entered physical next stair: " + reason);
                return;
            }
            ObserveAutoHoldSlope(guide);
        }
        if (!autoHoldSawSlope || !autoHoldLastHalfValid) return;
        // B observes the receiving flat only. Held input cannot auto-turn.
        // The release callback is the sole physical commit point.
        AutoHoldLandingFloor landing = ObserveFlatStartAutoHoldNextFlat(
            guide, out string floorReason);
        if (landing == AutoHoldLandingFloor.LeftFlat)
            CancelAutoHold("left physical receiving flat: " + floorReason);
    }

    // Both physical halves can authorize a hold. A left geometric turn uses
    // the successful InCorner UTurn; a right turn takes the OutCorner FiveLine
    // path, preserving lateral position instead of snapping to the LEFT lip.
    void QueueAutoHoldHeldTurn(float lateral)
    {
        // A execution is removed. The surviving observer is B's precondition;
        // only MouseUp / Touch.Ended may queue a physical turn here.
        if (!flatStartReleaseCommitInProgress)
        {
            AutoHoldLog("A AUTO EXECUTION BLOCKED", "no B release event");
            return;
        }
        if (!autoHoldEligible || !autoHoldSawSlope || !autoHoldLastHalfValid ||
            !rb || rb.isKinematic || HasPendingTurnIntent || turnTransitionActive ||
            waitingForTurnGuide)
        {
            CancelAutoHold("turn blocked");
            return;
        }
        if (!TryResolveAutoHoldTurnSign(out float resolvedSign,
                out string signSource))
        {
            AutoHoldLog("RELEASE REJECTED",
                "next turn direction is not verified; no guessed turn");
            return;
        }
        autoHoldTurnSign = resolvedSign;
        bool innerTurn = autoHoldTurnSign < 0f;
        autoHoldInnerErrorAtCommit = Vector3.Dot(
            rb.position - autoHoldLastInnerLinePoint, autoHoldLastLeft);
        autoHoldTurnLeft = innerTurn ? autoHoldLastLeft : Vector3.zero;
        autoHoldEdgeSampleValid = false;
        // Only the InCorner route uses a pre-turn LEFT physical edge.
        if (innerTurn && TryResolveAutoHoldEdge(autoHoldTurnLeft, out Vector3 edge))
        {
            autoHoldEdgeSample = edge;
            autoHoldEdgeSampleValid = true;
        }
        pendingTurnDegrees = autoHoldTurnSign * QuarterTurnDegrees;
        pendingTurnQueuedTime = Time.time;
        pendingTurnFromPassedSlope = false;
        autoHoldTurnPending = true;
        ResetTapEdge(); // Long Hold wins; no competing tap turn.
        flickConsumed = true; // A release must never generate another turn.
        CapturePendingTurnSnapshot();
        // This is the ONLY place an authorized B-release handoff is armed.
        bReleaseHandoffFlat = autoHoldNextFlatCollider;
        bReleaseOriginalHeading = NormalizeFlat(direction, autoHoldLastForward);
        bReleaseForwardSpeed = Mathf.Clamp(Vector3.Dot(
            rb.velocity, bReleaseOriginalHeading), 0f,
            Mathf.Max(0f, maxGroundSpeed));
        bReleaseHandoffTime = Time.fixedTime;
        bReleaseHandoffActive = bReleaseHandoffFlat &&
            bReleaseForwardSpeed > .1f;
        bReleaseNextLogTime = 0f;
        RequestTurnHandoffForPendingIntent();
        AutoHoldLog(innerTurn ? "INCORNER COMMIT" : "OUTCORNER COMMIT",
            $"direction={autoHoldTurnSign:+0;-0} signSource={signSource} " +
            $"half={(autoHoldLeftHalf ? "LEFT" : "RIGHT")} " +
            $"leftOffset={lateral:F3} innerError={autoHoldInnerErrorAtCommit:F3} " +
            $"pos={rb.position:F3} vel={rb.velocity:F3}");
        ResetAutoHoldObservation(false);
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

    // ================================================================
    // Pre-extreme steering: lateral acceleration only, no position teleport
    // ================================================================
    bool CanQueueEarlyFlick(bool pressStartedBeforeExtreme,
        bool verifiedFlatStartOriginalStair = false)
    {
        return (!flickStartedOnFlat || verifiedFlatStartOriginalStair) &&
               rb && !rb.isKinematic && BeginCommandOnTouch && Earliest &&
               !MainGameManager.OnDead && currentSupported &&
               currentGuideValid && currentSurfaceValid && currentGuide.isSlope &&
               (pressStartedBeforeExtreme ||
                (currentGuide.sectionProgress01 < TargetProgress && !extremePointFiredInSection)) &&
               !turnTransitionActive && !waitingForTurnGuide &&
               !fiveLineCorrectionActive && !fiveLineCorrectionPending &&
               !HasPendingTurnIntent &&
               (!correspondSubject || !correspondSubject.IsVisualFrameTurning);
    }

    void QueueEarlyFlick(float x)
    {
        if (Mathf.Abs(x) <= Eps) return;
        // Accepted gestures retain the tap reservation, even after repeated
        // right-left-right adjustments or an early press released after target.
        if (tapEdgeState == TapEdgeState.Armed ||
            tapEdgeState == TapEdgeState.Qualified)
        {
            if (tapEdgeState == TapEdgeState.Armed)
                TapEdgeLog("QUALIFIED", "first pre-extreme micro flick");
            tapEdgeState = TapEdgeState.Qualified;
            tapEdgeSlopeGuide = currentGuide;
            tapEdgeMaxSlopeProgress = Mathf.Max(tapEdgeMaxSlopeProgress,
                currentGuide.sectionProgress01);
        }
        if (flickStartedBeforeExtreme && currentGuide.sectionProgress01 >= TargetProgress)
            earlyFlickStartedAfterTarget = true;
        pendingEarlyFlickSteps = Mathf.Clamp(
            pendingEarlyFlickSteps + (x > 0f ? 1 : -1), -2, 2);
        if (logEarlyFlick)
            Debug.Log($"[EARLY FLICK] accepted x={x:F1} progress={currentGuide.sectionProgress01:F3}", this);
    }

    void ClearEarlyFlick()
    {
        pendingEarlyFlickSteps = 0;
        earlyFlickSession = false;
        earlyFlickMoving = false;
        earlyFlickStartedAfterTarget = false;
        earlyFlickGuide = default;
        earlyFlickInitialOffset = 0f;
        earlyFlickTargetOffset = 0f;
    }

    float StepEarlyFlick(NearestKnotDetector.GuideFrame guide, Surface surface, bool grounded)
    {
        float normalDamping = -surface.lateralSpeed * ResponseInverse;

        if (!grounded || !guide.isSlope || waitingForTurnGuide || turnTransitionActive ||
            (!earlyFlickStartedAfterTarget &&
             (guide.sectionProgress01 >= TargetProgress || extremePointFiredInSection)))
        {
            if (earlyFlickMoving && logEarlyFlick)
                Debug.Log($"[EARLY FLICK END] sideOffset=" +
                          $"{Vector3.Dot(rb.position - guide.point, surface.side):F3}m " +
                          $"target={earlyFlickTargetOffset:F3}m", this);
            // Normal early steering stops at the target; late-started steering
            // can run only until the slope ends. Keep visual history until flat.
            pendingEarlyFlickSteps = 0;
            earlyFlickMoving = false;
            return normalDamping;
        }
        if (earlyFlickSession &&
            (!knotDetector || !knotDetector.IsSameSection(earlyFlickGuide, guide)))
        {
            ClearEarlyFlick();
            return normalDamping;
        }

        float sideOffset = Vector3.Dot(rb.position - guide.point, surface.side);
        int steps = pendingEarlyFlickSteps;
        pendingEarlyFlickSteps = 0;
        if (steps != 0)
        {
            if (!earlyFlickSession)
            {
                earlyFlickGuide = guide;
                earlyFlickSession = true;
                earlyFlickInitialOffset = sideOffset;
                earlyFlickTargetOffset = sideOffset;
            }
            float maxTotal = Mathf.Max(0.2f, earlyFlickTotalMeters);
            earlyFlickTargetOffset = Mathf.Clamp(
                earlyFlickTargetOffset + steps * Mathf.Max(0.1f, earlyFlickStepMeters),
                earlyFlickInitialOffset - maxTotal,
                earlyFlickInitialOffset + maxTotal);
            earlyFlickMoving = true;
            if (logEarlyFlick)
            {
                float remainingError = earlyFlickTargetOffset - sideOffset;
                Debug.Log($"[EARLY FLICK] target={earlyFlickTargetOffset:F3} current={sideOffset:F3}", this);
                if (Mathf.Abs(remainingError) <= .03f)
                    Debug.Log($"[EARLY FLICK LIMIT] Target is already within 3cm of current position. " +
                              $"remaining={remainingError:F3}m, totalLimit={maxTotal:F3}m", this);
            }
        }
        if (!earlyFlickMoving) return normalDamping;

        float difference = earlyFlickTargetOffset - sideOffset;
        float gain = Mathf.Max(1f, earlyFlickResponse);
        float maxSpeed = Mathf.Max(.2f, earlyFlickMaxSideSpeed);
        float maxAccel = Mathf.Max(1f, earlyFlickMaxSideAcceleration);

        float desiredSpeed = Mathf.Clamp(difference * gain, -maxSpeed, maxSpeed);
        float endProgress = earlyFlickStartedAfterTarget ? 1f : TargetProgress;
        float timeLeft = (endProgress - guide.sectionProgress01) *
                         Mathf.Max(0f, guide.sectionLength) /
                         Mathf.Max(0.1f, surface.tangentSpeed);
        desiredSpeed *= Mathf.Clamp01(timeLeft / Mathf.Max(.02f, earlyFlickEndBrakeSeconds));

        // 目標を越える惰性を抑える。追加の固定時間ロックは使用しない。
        float approaching = Mathf.Sign(difference) * surface.lateralSpeed;
        if (approaching > 0f && Mathf.Abs(difference) <=
            approaching * approaching / (2f * maxAccel) + approaching * .01f)
            desiredSpeed = 0f;

        float lateralAccel = Mathf.Clamp(
            (desiredSpeed - surface.lateralSpeed) * gain, -maxAccel, maxAccel);
        if (Mathf.Abs(difference) < .02f && Mathf.Abs(surface.lateralSpeed) < .12f)
            earlyFlickMoving = false;
        return lateralAccel;
    }

    // 極限ポイント通過済みの階段か、従来の平面旋回かを先に確定する。
    // 通過瞬間のOnInifniteは使わず、区間内で保持されるフラグを使う。
    // A single authorization policy for physics and external visual turns.
    // Do not use the old flat-only StickBeforeNextTurn latch.
    public bool CanAcceptVisualFrameTurn => CanAcceptQuarterTurn(out _);

    bool CanAcceptQuarterTurn(out bool passedSlopeTurn)
    {
        passedSlopeTurn = false;
        if (MainGameManager.OnDead)
            return false;

        bool slopeSection =
            (currentGuideValid && currentGuide.isSlope) ||
            (extremePointSectionValid && !BallVisualIsOnFlat);

        if (slopeSection)
        {
            // A dropped -/+ target-crossing sample must not prevent a valid turn.
            // Keep the extreme-point effect's crossing event independent.
            passedSlopeTurn = extremePointFiredInSection ||
                (currentSupported && currentGuideValid && currentGuide.isSlope &&
                 currentGuide.sectionProgress01 >= TargetProgress);
            return passedSlopeTurn;
        }

        return BallVisualIsOnFlat && AcceleFirstTurn;
    }

    bool QueueQuarterTurn(float directionSign, bool sameFlatTapTurn = false)
    {
        if (!Earliest)
            return false;
        if (Mathf.Abs(directionSign) <= Eps)
            return false;

        // The physical slab is authoritative for this specific tap+flick.
        // Normal turns retain their existing authorization policy.
        bool canTurn = CanAcceptQuarterTurn(out bool passedSlopeTurn);
        if (sameFlatTapTurn && AcceleFirstTurn &&
            BeginCommandOnTouch && !MainGameManager.OnDead)
        {
            canTurn = true;
            passedSlopeTurn = false;
        }
        if (!canTurn)
        {
            if (logTurnTransition)
                Debug.Log($"[CORE TURN DENIED] slope={currentGuideValid && currentGuide.isSlope} " +
                          $"extremePassed={extremePointFiredInSection} " +
                          $"acceleFirst={AcceleFirstTurn} stickSlopePhase={StickSlopePharse} " +
                          $"flat={BallVisualIsOnFlat} progress={(currentGuideValid ? currentGuide.sectionProgress01 : -1f):F3}", this);
            return false;
        }

        if (sameFlatTapTurn)
        {
            tapEdgeState = TapEdgeState.TurnQueued;
            tapEdgeFastTurnQueued = true;
            TapEdgeLog("SAME FLAT TURN QUEUED",
                $"collider={tapEdgeFlatCollider.name} sign={Mathf.Sign(directionSign):+0;-0}");
        }
        else
            CancelTapEdge("ordinary 90-degree turn");
        StopEdgeCruise("90-degree turn queued");
        ClearPostAutoHoldLeftEdge();
        pendingTurnFromPassedSlope = passedSlopeTurn;
        if (passedSlopeTurn && logTurnTransition)
            Debug.Log("[CORE TURN AFTER EXTREME QUEUED] waiting for safe handoff", this);
        pendingTurnDegrees =
            Mathf.Sign(directionSign) * QuarterTurnDegrees;
        pendingTurnQueuedTime = Time.time;

        // Updateで入力を受理した瞬間のPhysics状態を保存する。
        // Turn Handoff後の位置・Targetで判定をすり替えない。
        CapturePendingTurnSnapshot();

        // BallVisualが独立Pose Authority中なら、通常Recoveryを待たず短いTurn Handoffへ入れる。
        RequestTurnHandoffForPendingIntent();

        return true;
    }

    bool HasPendingTurnIntent =>
        Mathf.Abs(pendingTurnDegrees) > Eps;

    bool IsPendingTurnIntentExpired()
    {
        if (!HasPendingTurnIntent || pendingTurnQueuedTime < 0f)
            return false;

        float bufferSeconds = slopeReleaseHoldTurnPending
            ? Mathf.Max(.5f, slopeExtremeTurnBufferSeconds)
            : autoHoldTurnPending
            ? Mathf.Max(2f, slopeExtremeTurnBufferSeconds)
            : pendingTurnFromPassedSlope
                ? Mathf.Max(0.25f, slopeExtremeTurnBufferSeconds)
                : Mathf.Max(0.05f, turnInputBufferSeconds);
        return Time.time - pendingTurnQueuedTime > bufferSeconds;
    }

    void ExpirePendingTurnIntent(bool escapingToSafeSpline = false)
    {
        if (!HasPendingTurnIntent)
            return;
        if (autoHoldTurnPending)
            ClearBReleaseHandoff();

        if (pendingTurnFromPassedSlope && logTurnTransition)
        {
            bool handoffReady = !ballVisualSlopeDrive ||
                                ballVisualSlopeDrive.IsReadyForTurnTransition;
            Debug.LogWarning(
                $"[CORE TURN AFTER EXTREME EXPIRED] " +
                $"wait={Time.time - pendingTurnQueuedTime:F3}s " +
                $"handoffReady={handoffReady} " +
                $"turnActive={turnTransitionActive} " +
                $"waitingGuide={waitingForTurnGuide}", this);
        }

        if (tapEdgeState == TapEdgeState.TurnQueued)
            CancelTapEdge("pending Handoff expired");
        if (autoHoldTurnPending)
        {
            if (slopeReleaseHoldTurnPending)
                SlopeReleaseHoldLog(
                    escapingToSafeSpline ? "TURN ABANDONED FOR SAFE LANE" : "CANCEL",
                    escapingToSafeSpline
                        ? "stalled beyond configured seconds; verified spline escape takes priority"
                        : "pending Handoff expired / invalidated");
            else
                AutoHoldLog("CANCEL", "pending Handoff expired / invalidated");
            autoHoldTurnPending = false;
            slopeReleaseHoldTurnPending = false;
            ResetAutoHoldObservation(true);
        }
        autoHoldTurnPending = false;
        slopeReleaseHoldTurnPending = false;
        slopeReleaseHoldHandoffHeld = false;
        slopeReleaseStopStartFixedTime = -1f;
        slopeReleaseCommittedSlopeGuide = default;
        slopeReleaseHoldNextFlatCollider = null;
        slopeReleaseHoldLastFlatSupportTime = -1f;
        slopeReleaseHoldLastFlatSupportPosition = default;
        pendingTurnDegrees = 0f;
        pendingTurnQueuedTime = -1f;
        pendingTurnFromPassedSlope = false;
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

        // Authorization was checked when the flick was queued. Changing from
        // slope to flat during Handoff must not invalidate that accepted input.
        if (MainGameManager.OnDead)
        {
            ExpirePendingTurnIntent();
            return;
        }

        RequestTurnHandoffForPendingIntent();

        if (IsTurnExecutionBlocked())
        {
            if (slopeReleaseHoldTurnPending && rb && !rb.isKinematic &&
                slopeReleaseHoldNextFlatCollider)
            {
                var contact = HasGroundSupport();
                if (contact.Item1 &&
                    contact.Item2.collider == slopeReleaseHoldNextFlatCollider &&
                    IsAutoHoldStageHit(contact.Item2, float.NaN) &&
                    Vector3.Angle(contact.Item2.normal, Vector3.up) <= 18f)
                {
                    if (!slopeReleaseHoldHandoffHeld)
                    {
                        slopeReleaseStopStartFixedTime = -1f;
                        SlopeReleaseHoldLog("HANDOFF FLAT HOLD",
                            $"savedVelocity={slopeReleaseBeforeStopVelocity:F3} " +
                            $"preStopAccel={slopeReleaseBeforeStopAcceleration:F3} " +
                            $"pos={rb.position:F3}");
                    }
                    slopeReleaseHoldHandoffHeld = true;
                    float horizontalSpeed = Vector3.ProjectOnPlane(
                        rb.velocity, Vector3.up).magnitude;
                    if (horizontalSpeed <= slopeReleaseStopSpeedThreshold)
                    {
                        if (slopeReleaseStopStartFixedTime < 0f)
                            slopeReleaseStopStartFixedTime = Time.fixedTime;
                    }
                    else
                        slopeReleaseStopStartFixedTime = -1f;
                    float stoppedFor = slopeReleaseStopStartFixedTime < 0f
                        ? 0f : Time.fixedTime - slopeReleaseStopStartFixedTime;
                    if (stoppedFor >= slopeReleaseStoppedSecondsToEscape &&
                        TryStartSlopeReleaseTimedEscape())
                        return;
                    // Do not zero X/Z speed while waiting on a real flat.
                    return;
                }
            }
            if (tapEdgeFastTurnQueued && tapEdgeState == TapEdgeState.TurnQueued &&
                rb && !rb.isKinematic && tapEdgeFlatCollider)
            {
                var contact = HasGroundSupport();
                if (contact.Item2.collider == tapEdgeFlatCollider &&
                    IsTapEdgeSupportedFlat(contact.Item1, contact.Item2))
                {
                    if (!tapEdgeHandoffHeld)
                        TapEdgeLog("HANDOFF HOLD", $"position={rb.position:F3}");
                    tapEdgeHandoffHeld = true;
                    // Keep natural flat momentum during release handoff.
                    return;
                }
            }
            return;
        }

        if (tapEdgeFastTurnQueued && tapEdgeState == TapEdgeState.TurnQueued &&
            logTurnTransition)
            TapEdgeLog("HANDOFF READY",
                $"wait={Mathf.Max(0f, Time.time - pendingTurnQueuedTime):F3}s " +
                $"visualReady={!ballVisualSlopeDrive || ballVisualSlopeDrive.IsReadyForTurnTransition} " +
                $"correspondReady={!correspondSubject || !correspondSubject.IsVisualFrameTurning}");

        // For the same-flat gesture do not commit the turn after leaving the
        // original slab. In particular, never turn while falling in midair.
        if (tapEdgeFastTurnQueued && tapEdgeState == TapEdgeState.TurnQueued)
        {
            var contact = HasGroundSupport();
            if (contact.Item2.collider != tapEdgeFlatCollider ||
                !IsTapEdgeSupportedFlat(contact.Item1, contact.Item2))
            {
                TapEdgeLog("TURN CANCELED SUPPORT",
                    $"position={rb.position:F3} supported={contact.Item1} " +
                    $"collider={(contact.Item2.collider ? contact.Item2.collider.name : "-")} " +
                    $"angle={(contact.Item2.collider ? Vector3.Angle(contact.Item2.normal, Vector3.up) : -1f):F1} " +
                    $"guideFlat={BallVisualIsOnFlat}");
                ExpirePendingTurnIntent();
                return;
            }
            ReleaseTapEdgeHandoffHold();
        }

        if (slopeReleaseHoldTurnPending)
        {
            // Handoff may require several physics frames. Do not start a
            // high-speed right-angle turn after leaving the receiving flat.
            var flatSupport = HasGroundSupport();
            bool stillOnNextFlat = IsSlopeReleaseHoldOnVerifiedFlat(
                flatSupport.Item1, flatSupport.Item2);
            if (!stillOnNextFlat)
            {
                SlopeReleaseHoldLog("TURN CANCELED SUPPORT",
                    $"after handoff: pos={rb.position:F3} " +
                    $"hit={(flatSupport.Item2.collider ? flatSupport.Item2.collider.name : "-")}");
                ExpirePendingTurnIntent();
                return;
            }
            if (slopeReleaseHoldHandoffHeld)
            {
                // Restore a SAFE planar component only when the original flat
                // still supports the body. Old sideways drift is not restored.
                float savedSpeed = Vector3.ProjectOnPlane(
                    pendingTurnFlickVelocity, Vector3.up).magnitude;
                Vector3 forward = NormalizeFlat(autoHoldLastForward, direction);
                rb.velocity = forward * Mathf.Min(savedSpeed, maxGroundSpeed) +
                    Vector3.up * rb.velocity.y;
                slopeReleaseHoldHandoffHeld = false;
                slopeReleaseStopStartFixedTime = -1f;
                SlopeReleaseHoldLog("HANDOFF READY",
                    $"restoredSpeed={Mathf.Min(savedSpeed, maxGroundSpeed):F2} " +
                    $"wait={Time.time - pendingTurnQueuedTime:F3}s");
            }
        }

        if (autoHoldTurnPending && bReleaseHandoffActive &&
            !TryGetBReleaseReceivingSupport(out _))
        {
            AutoHoldLog("B TURN CANCELED SUPPORT",
                "original receiving flat no longer supports the body");
            ExpirePendingTurnIntent();
            ClearBReleaseHandoff();
            return;
        }

        float turnDegrees =
            Mathf.Sign(pendingTurnDegrees) * QuarterTurnDegrees;

        pendingTurnDegrees = 0f;
        pendingTurnQueuedTime = -1f;
        ballVisualSlopeDrive?.CancelTurnHandoffRequest();

        ApplyQuarterTurn(turnDegrees);
        // The accepted turn's physics snapshot now owns this motion; the
        // queued B state must never leak into a later unrelated turn.
        ClearBReleaseHandoff();
        pendingTurnFromPassedSlope = false;
        autoHoldTurnPending = false;
        slopeReleaseHoldTurnPending = false;
        slopeReleaseHoldHandoffHeld = false;
        slopeReleaseStopStartFixedTime = -1f;
        slopeReleaseCommittedSlopeGuide = default;
        slopeReleaseHoldNextFlatCollider = null;
        slopeReleaseHoldLastFlatSupportTime = -1f;
        slopeReleaseHoldLastFlatSupportPosition = default;
        autoHoldEdgeSampleValid = false;
        autoHoldEdgeCollider = null;
    }

    void ApplyQuarterTurn(float turnDegrees)
    {
        if (!rb || turnTransitionActive)
            return;

        ClearPostBDownhillGuard("new turn");
        ClearEarlyFlick();
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

        // Only committed Auto Hold can force final edge alignment.
        // Position must be corrected BEFORE the freeze snapshot is captured.
        bool verifiedB = autoHoldTurnPending && bReleaseHandoffActive &&
            TryGetBReleaseReceivingSupport(out _);
        if (autoHoldTurnPending && turnDegrees < 0f)
            SetAutoHoldTurnAtPhysicalEdge();
        if (verifiedB && turnDegrees < 0f &&
            Vector3.Dot(bReleaseOriginalHeading, directionBefore) > .95f)
        {
            // Edge locking removes sideways inertia, including nearly all
            // horizontal motion when the old guide is momentarily sideways.
            // Rebuild ONLY the confirmed old-heading component, with a strict
            // turn speed ceiling, before the one-time 90-degree rotation.
            float naturalForward = Mathf.Max(0f,
                Vector3.Dot(rb.velocity, directionBefore));
            float boundedRelease = Mathf.Min(bReleaseForwardSpeed,
                Mathf.Min(BReleaseHandoffSpeedCeiling,
                    Mathf.Max(0f, maxGroundSpeed)));
            float restored = Mathf.Max(naturalForward, boundedRelease);
            Vector3 prior = rb.velocity;
            rb.velocity = directionBefore * restored +
                Vector3.up * prior.y;
            Debug.Log($"[B RELEASE TURN SNAPSHOT] t={Time.fixedTime:F3} " +
                $"releaseForward={bReleaseForwardSpeed:F2} " +
                $"before={prior:F3} restored={rb.velocity:F3} " +
                $"heading={directionBefore:F3}", this);
        }

        Vector3 velocityBefore = rb.velocity;
        Vector3 angularVelocityBefore = rb.angularVelocity;

        turnTargetDirection =
            NormalizeFlat(
                fullDirectionTurn * directionBefore,
                directionBefore);

        // 旋回種別の判定は一度だけ。Flatではフリック時の凍結Snapshotを使う。
        CaptureTurnLandingIntent();
        bool tapEdgeSelected = tapEdgeState == TapEdgeState.TurnQueued;
        activeTurnMode = autoHoldTurnPending
            ? (turnDegrees < 0f
                ? TurnResolutionMode.UTurnBeforeEnergyTarget
                : TurnResolutionMode.FiveLineAfterEnergyTarget)
            : tapEdgeSelected
                ? TurnResolutionMode.UTurnBeforeEnergyTarget
            : pendingTurnFromPassedSlope
                ? TurnResolutionMode.FiveLineAfterPop
                : ResolveTurnResolutionMode(directionBefore);
        bool autoHoldSelected = autoHoldTurnPending;
        ClearPendingTurnSnapshot();
        LogDropAuditTurn("ACCEPTED", velocityBefore, turnDegrees);

        if (pendingTurnFromPassedSlope && logTurnTransition)
            Debug.Log("[CORE TURN AFTER EXTREME] slope flick accepted, mode=FiveLineAfterPop", this);
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
        {
            if (autoHoldSelected)
                AutoHoldLog("CANCEL", "could not begin physics turn");
            if (tapEdgeSelected) CancelTapEdge("could not begin physics turn");
            return;
        }
        if (verifiedB && turnDegrees < 0f &&
            activeTurnMode == TurnResolutionMode.UTurnBeforeEnergyTarget)
        {
            postBDownhillArmed = true;
            postBDownhillSawSlope = false;
            postBDownhillWrongGuideLogged = false;
            postBDownhillCommittedHeading = NormalizeFlat(
                turnTargetDirection, direction);
            postBDownhillArmedAt = Time.fixedTime;
            postBDownhillSectionIndex = -1;
            postBDownhillLastSupportTime = -1f;
            postBDownhillRecentNearSlopeTime = -1f;
            postBDownhillNextLogTime = 0f;
            postBDownhillStairFamily = -1;
            postBDownhillLastLoggedSection = -1;
            postBEdgeAnchorValid = false;
            postBEdgeReacquireUntil = -1f;
            Debug.Log($"[B DOWNHILL ARMED] t={Time.fixedTime:F3} " +
                $"heading={postBDownhillCommittedHeading:F3}; " +
                $"first receiving stair only", this);
        }
        bool normalInCornerTurn = !autoHoldSelected && !tapEdgeSelected &&
            activeTurnMode == TurnResolutionMode.UTurnBeforeEnergyTarget;
        bool heldOutCorner = autoHoldSelected && turnDegrees > 0f;
        if (!heldOutCorner && (autoHoldSelected || tapEdgeSelected || normalInCornerTurn))
        {
            StartPostAutoHoldLeftEdge(turnTargetDirection,
                tapEdgeSelected && tapEdgeFastTurnQueued,
                (normalInCornerTurn || autoHoldSelected) && !tapEdgeSelected);
            if (autoHoldSelected)
            {
                if (slopeReleaseHoldTurnPending)
                    SlopeReleaseHoldLog("TURN EXECUTED",
                        $"pos={rb.position:F3} heading={turnTargetDirection:F3} " +
                        $"velocityBefore={velocityBefore:F3} " +
                        $"innerError={autoHoldInnerErrorAtCommit:F3}");
                else
                    AutoHoldLog("INCORNER EXECUTED", $"innerErrorAtCommit={autoHoldInnerErrorAtCommit:F3} " +
                        $"executionPos={rb.position:F3} queuedSign={autoHoldTurnSign:+0;-0}");
            }
            else if (tapEdgeSelected)
                TapEdgeLog("TURN EXECUTED", $"position={rb.position:F3}");
            else if (normalInCornerTurn && logTurnTransition)
                Debug.Log($"[INCORNER GOAL LOCKED] mode={activeTurnMode} " +
                    $"target=next physical left-edge / leftSpline " +
                    $"speed={rb.velocity.magnitude:F2}", this);
        }
        else
        {
            // An OutCorner must NOT be forced onto the post-turn LEFT edge.
            // Its FiveLine continuation handles the chosen lateral corridor.
            ClearPostAutoHoldLeftEdge();
            if (heldOutCorner)
                AutoHoldLog("OUTCORNER EXECUTED",
                    $"mode={activeTurnMode} " +
                    $"position={rb.position:F3} direction={turnTargetDirection:F3} " +
                    $"velocityBefore={velocityBefore:F3}");
        }
        if (tapEdgeSelected) ResetTapEdge();

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
        turnGuideCapturedAt = -1f;
        ResetUncapturedTurnGuideStallWatch();
        uncommandedGuideGuardLogged = false;
        turnGuideReleasedAt = -1f;
        turnGuideGuardBraking = false;
        turnGuideCrawlLogged = false;
        if (currentGuideValid && currentGuide.isSlope) driveState = 0f;
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
                    turnVisualDurationSeconds,
                    true); // SlopeStickCoreの許可判定を通過済み
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
          
            direction = direction,
            velocity = rb.velocity,
            angularVelocity = rb.angularVelocity,
            useGravity = rb.useGravity
        };
        activeTurnDegrees =
            Mathf.Sign(turnDegrees) * QuarterTurnDegrees;

        // The turn snapshot stores velocity and angular velocity. Pause pose
        // integration briefly without ever erasing those physical values.
        rb.isKinematic = true;

        turnTransitionActive = true;
        turnBodyFrozen = true;
        turnTransitionStartTime = Time.fixedTime;
        LogDropAuditTurn("FREEZE", turnTransitionSnapshot.velocity);

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
        
        if(BeginCommandOnTouch)
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
        LogDropAuditTurn("RELEASE-BEGIN", snapshot.velocity);
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
        turnGuideReleasedAt = Time.fixedTime;
        turnHandoffNextAuditTime = 0f;
        turnGuideGuardBraking = false;
        turnGuideCrawlLogged = false;
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
        LogDropAuditTurn("RELEASE-END", snapshot.velocity);

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
        {
            ReleaseTurnBodyToMotion("VisualTurnCompleted");
        }

        LogDropAuditTurn("VISUAL-COMPLETE", turnTransitionSnapshot.velocity);
        turnTransitionActive = false;
        turnBodyFrozen = false;
        turnTransitionStartTime = -1f;
        activeTurnDegrees = 0f;
        turnTransitionSnapshot = default;

        // waitingForTurnGuideはApplyQuarterTurn開始時からtrueのまま。
        // InCornerは既に新方向へ切り替え済み。Spline捕捉へ進む。
        if (currentGuideValid && currentGuide.isSlope) driveState = 0f;
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
        ClearPostBDownhillGuard("turn aborted");
        ResetGeneralStallRecovery("turn aborted", true);
        ResetSlopeReleaseTimedEscape("turn aborted");
        ResetTapEdge();
        ClearPostAutoHoldLeftEdge();
        StopEdgeCruise("turn aborted");
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
        uncommandedGuideGuardLogged = false;
        waitingForTurnGuide = false;
        turnGuideReleasedAt = -1f;
        turnGuideGuardBraking = false;
        turnGuideCrawlLogged = false;

        activeTurnMode = TurnResolutionMode.None;
        autoHoldTurnPending = false;
        slopeReleaseHoldTurnPending = false;
        slopeReleaseHoldHandoffHeld = false;
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
        ClearPostBDownhillGuard("lifecycle reset");
        ClearBReleaseHandoff();
        turnGuideCapturedAt = -1f;
        ResetGeneralStallRecovery("stage rebuilt", true);
        ResetSlopeReleaseTimedEscape("stage rebuilt");
        slopeReleaseLastSampleValid = false;
        ResetTapEdge();
        ClearPostAutoHoldLeftEdge();
        StopEdgeCruise("stage rebuilt");
        correspondSubject?.StopExtremePointEffect();
        ResetExtremePointTracking();
        pendingTurnDegrees = 0f;
        pendingTurnQueuedTime = -1f;
        pendingTurnFromPassedSlope = false;
        autoHoldTurnPending = false;
        slopeReleaseHoldTurnPending = false;
        slopeReleaseHoldHandoffHeld = false;
        ResetFlickGesture();
        correspondSubject?.ResetTurnGestureTracking();
        ballVisualSlopeDrive?.CancelTurnHandoffRequest();

        correspondSubject?.CancelVisualFrameTurn(false);

        turnTransitionActive = false;
        turnBodyFrozen = false;
        turnTransitionStartTime = -1f;
        activeTurnDegrees = 0f;
        turnTransitionSnapshot = default;
        uncommandedGuideGuardLogged = false;
        waitingForTurnGuide = false;
        turnGuideReleasedAt = -1f;
        turnGuideGuardBraking = false;
        turnGuideCrawlLogged = false;

        activeTurnMode = TurnResolutionMode.None;
        autoHoldTurnPending = false;
        slopeReleaseHoldTurnPending = false;
        slopeReleaseHoldHandoffHeld = false;
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

        // No valid snapshot: never erase the live body's velocity.
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
        Vector3 newSide = Vector3.Cross(Vector3.up,
            NormalizeFlat(turnTargetDirection, direction)).normalized;
        float sideBefore = Vector3.Dot(planarVelocity, newSide);
        float permittedSide = Mathf.Max(.8f,
            Mathf.Min(2f, maxGroundSpeed * .04f));
        float sideAfter = Mathf.Clamp(sideBefore,
            -permittedSide, permittedSide);
        planarVelocity += newSide * (sideAfter - sideBefore);
        rb.velocity = planarVelocity + Vector3.up * verticalSpeed;
        if (Mathf.Abs(sideBefore - sideAfter) > .01f && logTurnTransition)
            Debug.Log($"[TURN LATERAL CAP] before={sideBefore:F2}m/s " +
                $"after={sideAfter:F2}m/s cap={permittedSide:F2}m/s " +
                $"mode={activeTurnMode}", this);
        LogDropAuditTurn("FIVELINE-VELOCITY-SET", rb.velocity);
        // 旋回開始前に許可済み。90度の速度変更とheadingは常に揃える。
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
        LogDropAuditTurn("INCORNER-VELOCITY-SET", originalVelocity);
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