/*using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using DG.Tweening;

[RequireComponent(typeof(Rigidbody), typeof(ReliableBaselineSampler))]
public class SlopeStickBall3D3 : MonoBehaviour
{
    // Balanced Virtual Fillet + Analytic Limit Capture版。
    // Slope Progress改訂：進行方向のワールド座標を廃止し、実斜面Colliderの入口0%～出口100%で統一。
    // 写真の2位Approach、3位条件付き通常吸着、4位Analytic Extra Stick、
    // 5位早めSphereCast接地判定を残し、同じ軸の補助を加算せず調停する。
    [SerializeField] Rigidbody rb;

    [Header("Auto Progress")]
    [Tooltip("自動進行するワールド方向です。(1,0,0)=X正方向、(0,0,1)=Z正方向")]
    [SerializeField] Vector3 initialHeading = Vector3.right;

    [Header("Player + Stage Simultaneous Turn")]
    [Tooltip("入力方向を決めるカメラです。Side Turn ExperimentがONなら横フリックだけを使うため未設定でも動作します。")]
    [SerializeField] Transform cameraTransform;

    [Tooltip("プレイヤーの向きを表示するTransformです。未設定ならこのGameObjectを使用します。")]
    [SerializeField] Transform headingTransform;

    [SerializeField, Min(1f)] float minFlickPixels = 10f;
    [SerializeField, Range(0f, 180f)] float maxTurnPerFlick = 90f;

    [Tooltip("ONなら横フリックだけで左右へ曲がります。")]
    [SerializeField] bool useSideTurnExperiment = true;

    [Tooltip("90度旋回を許可している場合だけ、初期方向と旋回後方向をX/Z軸へスナップします。")]
    [SerializeField] bool snapHeadingOnSideTest = true;

    [Tooltip("フリック距離を対数的な強度帯へ分ける倍率です。")]
    [SerializeField, Min(1.1f)] float flickStrengthBandRatio = 1.8f;

    [Tooltip("弱いフリックから強いフリックへ割り当てる旋回角度です。")]
    [SerializeField] float[] flickTurnAngleSteps = {
        12.5f, 20f, 30f, 45f
    };

    [SerializeField, Min(.01f)] float headingTurnDuration = .45f;
    [SerializeField, Min(.01f)] float minimumHeadingTurnDuration = .12f;
    [SerializeField] Ease headingTurnEase = Ease.InOutCubic;
    [SerializeField, Range(0f, 180f)] float maxPlayerTurnAngle = 45f;

    [Tooltip("回転させるステージ全体のRootです。未設定ならStage Pivotを使用します。")]
    [SerializeField] Transform stageRoot;

    [Tooltip("ステージとプレイヤーが回る中心です。未設定ならCenter1を検索します。")]
    [SerializeField] Transform stagePivot;

    [Tooltip("ONならプレイヤーの旋回方向と逆方向へステージを回します。")]
    [SerializeField] bool stageTurnsOppositeToPlayer = true;

    [Tooltip("プレイヤーがStage Rootの子でない場合、Stage Pivotを中心にプレイヤー位置も同時に公転させます。")]
    [SerializeField] bool movePlayerAroundStagePivot = true;

    [Header("Initial Reliable Baseline Snapshot")]
    [Tooltip("初回静止・法線急変・VelocityChange直前・DOTween回転直前の4ピースを1サイクルとして管理するSamplerです。")]
    [SerializeField] ReliableBaselineSampler reliableBaselineSampler;

    [Header("Temporary Motion Snapshot")]
    [Tooltip("接地法線が急変した時とSoft HandoffのVelocityChange直前に、速度・角速度の完全ベクトルと符号付き面成分を保存します。")]
    [SerializeField] bool useSurfaceMotionSnapshot = true;

    [Tooltip("前回の接地法線からこの角度以上変わった時に、法線急変として一時保存します。")]
    [SerializeField, Range(.1f, 89f)] float surfaceNormalChangeTriggerAngle = 5f;

    [Tooltip("新しい面の法線との差がこの角度以内なら、面が安定していると判断します。")]
    [SerializeField, Range(.1f, 15f)] float surfaceSnapshotStableNormalTolerance = 2f;

    [Tooltip("面がこのFixedUpdate数だけ連続して安定した後、一時保存した状態を再構成します。")]
    [SerializeField, Range(1, 10)] int surfaceSnapshotStableFrames = 3;

    [Tooltip("保存直後の同一FixedUpdateで復元しないための最低保持時間です。")]
    [SerializeField, Min(0f)] float surfaceSnapshotMinimumSeconds = .06f;

    [Tooltip("保存した符号付き前後速度を、新しい面の進行方向へ戻します。")]
    [SerializeField] bool restoreSurfaceTangentSpeed = true;

    [Tooltip("保存した横速度を戻す割合です。0なら斜面入口で横滑りを復元しません。")]
    [SerializeField, Range(0f, 1f)] float surfaceLateralVelocityRestoreRatio = 0f;

    [Tooltip("ONなら保存時の法線速度を戻さず、復元時点で面へ向いている法線速度だけを維持します。")]
    [SerializeField] bool preserveCurrentInwardNormalSpeedOnSurfaceRestore = true;

    [Tooltip("保存した転がり角速度を、新しい面の転がり軸へ戻します。")]
    [SerializeField] bool restoreSurfaceAngularSpeed = true;

    [Tooltip("転がり軸以外の角速度成分を戻す割合です。0なら不要な縦回転・ヨー回転を復元しません。")]
    [SerializeField, Range(0f, 1f)] float surfaceCrossAngularRestoreRatio = 0f;

    [Header("DOTween Stage Turn Snapshot")]
    [Tooltip("DOTweenによるステージ回転の開始前に完全な運動状態を保存し、完了後のFixedUpdateで全符号付き成分を新しい面座標へ再構成します。")]
    [SerializeField] bool useStageTurnMotionSnapshot = true;

    [SerializeField] bool restoreStageTurnTangentSpeed = true;
    [SerializeField] bool restoreStageTurnAngularSpeed = true;
    [SerializeField] bool debugMotionSnapshot = true;

    public float sphereRadius = .5f, groundProbeDistance = .25f;
    public LayerMask groundMask = ~0;
    [Range(0, 89)] public float maxSlopeAngle = 75;

    public float maxGroundSpeed = 15;
    public float maxGroundAcceleration = 50;
    public float airAcceleration = 6;

    public float targetNormalSpeed = -2;
    public float normalSnapSharpness = 25;
    public float baseStickAcceleration = 24.6f;
    public float extraStickAcceleration = 20;

    [Header("Unified Natural Adhesion")]
    [Tooltip("通常吸着と入口Approach吸着を、法線方向の1回のAddForceへ統合します。")]
    [SerializeField] bool useUnifiedNaturalAdhesion = true;
    [Tooltip("法線方向へ加える最大加速度です。位置戻しではなくAddForceの安全上限です。")]
    [SerializeField, Min(.1f)] float normalAccelerationLimit = 180f;

    [Header("Flat -> Slope Natural Approach")]
    [SerializeField] bool useFlatToSlopeApproach = true;
    [SerializeField, Range(0f, 15f)] float flatToSlopeFlatMaxAngle = 4f;
    [SerializeField, Range(3f, 80f)] float flatToSlopeSlopeMinAngle = 8f;
    [SerializeField, Min(.25f)] float flatToSlopeLookAheadDistance = 5f;
    [SerializeField, Min(.02f)] float flatToSlopeProbeStep = .05f;
    [SerializeField, Min(.1f)] float flatToSlopeProbeHeight = 1.5f;
    [SerializeField, Min(.2f)] float flatToSlopeProbeDistance = 4.5f;
    [SerializeField, Min(.1f)] float flatToSlopeTargetEntrySpeed = 10f;
    [SerializeField, Min(.1f)] float flatToSlopeLongitudinalAccelerationLimit = 35f;
    [SerializeField, Min(.1f)] float flatToSlopeLateralAccelerationLimit = 22f;
    [SerializeField, Range(-2f, -.01f)] float flatToSlopeTargetNormalSpeed = -.35f;
    [SerializeField, Min(0f)] float flatToSlopeApproachStickAcceleration = 15f;
    [SerializeField, Min(.05f)] float flatToSlopeStickStartDistance = 2f;

    [Header("Flat -> Slope Curvature Check")]
    [SerializeField, Min(.05f)] float flatToSlopeVirtualTransitionLength = .8f;
    [SerializeField, Min(.05f)] float flatToSlopeVirtualMinimumRadius = .45f;
    [SerializeField] bool flatToSlopeEnforceCurvatureSpeedLimit = false;
    [SerializeField, Range(.5f, 1f)] float flatToSlopeCurvatureSafety = .9f;

    [Header("ProbeGround Flat -> Slope Soft Handoff")]
    [Tooltip("ProbeGroundが平面の直後に斜面を検出した瞬間だけ、小さなVelocityChangeを1回加えます。走行中の位置・速度を直接代入しません。")]
    [SerializeField] bool useSoftSlopeEntryHandoff = true;

    [Tooltip("この角度以下を平面として記憶します。")]
    [SerializeField, Range(0f, 15f)] float softHandoffFlatMaxAngle = 4f;

    [Tooltip("この角度以上を斜面候補として扱います。")]
    [SerializeField, Range(1f, 80f)] float softHandoffSlopeMinAngle = 8f;

    [Tooltip("エッジや側面の異常法線を除外する斜面角度上限です。45度斜面なら50～55度程度を推奨します。")]
    [SerializeField, Range(3f, 89f)] float softHandoffSlopeMaxAngle = 50f;

    [Tooltip("平面を離れて短時間AIRになっても、平面→斜面の遷移として認識する記憶時間です。")]
    [SerializeField, Min(0f)] float softHandoffFlatMemorySeconds = .20f;

    [Tooltip("斜面判定を何FixedUpdate連続で確認してから補正するかです。1なら接触した瞬間、2なら誤検出に強くなります。")]
    [SerializeField, Range(1, 4)] int softHandoffSlopeConfirmFrames = 1;

    [Tooltip("複数フレーム確認時に、法線がこの角度以内なら同じ斜面候補として平均化します。")]
    [SerializeField, Range(1f, 60f)] float softHandoffNormalConsistencyAngle = 25f;

    [Tooltip("重力から求めた下り方向と進行方向の内積がこの値以下なら、逆向き・横向き斜面として無視します。")]
    [SerializeField, Range(-1f, 1f)] float softHandoffMinimumForwardDot = .10f;

    [Tooltip("補正後、少なくともこの時間は次の平面で再Armしません。")]
    [SerializeField, Min(0f)] float softHandoffMinimumRearmSeconds = .12f;

    [Tooltip("次の平面へこの時間連続して接地したら、次の平面→斜面用に再Armします。")]
    [SerializeField, Min(0f)] float softHandoffFlatRearmSeconds = .04f;

    [Tooltip("平面法線から斜面法線へ向きを変える仮想遷移長です。球の直径の何倍で滑らかに曲げるかを指定します。値を大きくすると柔らかく、小さくすると強くなります。")]
    [SerializeField, Min(.25f)] float softHandoffVirtualTransitionDiameters = 2f;

    [Tooltip("幾何学的な速度変化量を、通常地上加速度で1 FixedUpdateに出せる量以下へ安全制限します。")]
    [SerializeField] bool softHandoffUseGroundAccelerationSafety = true;

    [Tooltip("接線速度を入口目標へ寄せる割合です。")]
    [SerializeField, Range(0f, 1f)] float softHandoffTangentialCorrection = .25f;

    [Tooltip("斜面接線から外れた横速度を弱める割合です。")]
    [SerializeField, Range(0f, 1f)] float softHandoffLateralCorrection = .35f;

    [Tooltip("斜面から外向きの法線速度を面側へ寄せる割合です。")]
    [SerializeField, Range(0f, 1f)] float softHandoffNormalCorrection = .55f;

    [Tooltip("補正後、通常物理へ滑らかに戻す短い時間です。")]
    [SerializeField, Min(.02f)] float softHandoffReleaseSeconds = .10f;

    [Tooltip("解放中に使う補助加速度の上限です。")]
    [SerializeField, Min(0f)] float softHandoffReleaseAccelerationLimit = 10f;

    [SerializeField, Range(0f, 1f)] float softHandoffReleaseStrength = .35f;

    [SerializeField] bool debugSoftSlopeEntryHandoff = true;
    [SerializeField] bool drawSoftSlopeEntryHandoff = true;

    [Header("Balanced Virtual Fillet (Ranks 2-5)")]
    [Tooltip("2位Approachの中で、平面法線から斜面法線へ5次関数で参照面を滑らかに回します。独立した強制軌道は追加しません。")]
    [SerializeField] bool useBalancedVirtualFillet = true;
    [Tooltip("2位Approachの進行方向へ仮想接線を混ぜる割合です。1でも速度や位置は直接変更しません。")]
    [SerializeField, Range(0f, 1f)] float virtualFilletTangentBlend = .70f;
    [Tooltip("3位の条件付き通常吸着で使う法線へ、仮想法線を混ぜる割合です。")]
    [SerializeField, Range(0f, 1f)] float virtualFilletNormalBlend = .55f;
    [Tooltip("v²κから求めた向心加速度不足のうち、法線補助候補へ採用する割合です。Approach/Analyticとは加算せず最大値を選びます。")]
    [SerializeField, Range(0f, 1f)] float virtualFilletCurveAssistRatio = .65f;
    [Tooltip("仮想曲率補助が法線方向へ要求できる最大加速度です。")]
    [SerializeField, Min(0f)] float virtualFilletCurveAccelerationLimit = 40f;
    [Tooltip("仮想フィレット終端で4位Analytic Extra Stickを残す割合です。0ならフィレット側へ完全に譲り、1なら従来どおりです。")]
    [SerializeField, Range(0f, 1f)] float virtualFilletAnalyticExtraRemain = .35f;
    [Tooltip("5位の早めSphereCast判定は残しつつ、このCast距離以内で法線吸着を100%使用します。")]
    [SerializeField, Min(0f)] float adhesionFullContactCastDistance = .15f;
    [Tooltip("このCast距離以上では、早期接地判定による法線吸着を0%にします。移動と前方探索は継続します。")]
    [SerializeField, Min(.01f)] float adhesionFadeEndCastDistance = .60f;
    [SerializeField] bool drawBalancedVirtualFillet = true;

    [Header("Analytic Limit Capture (Natural AddForce Only)")]
    [Tooltip("前方のmargin=0候補を一度だけ固定し、同じ点へ向けて接線速度だけをAddForceで整えます。位置・速度の直接変更や通過後の引き戻しは行いません。")]
    [SerializeField] bool useAnalyticLimitCapture = true;
    [Tooltip("候補点がこの距離以内へ入る、または次のFixedUpdateでこの範囲へ入る予測になった時点で固定します。")]
    [SerializeField, Min(.05f)] float analyticLimitArmDistance = 2f;
    [Tooltip("固定点をこの距離だけ通過したら計画を完了します。完了後は逆向きの力を加えません。")]
    [SerializeField, Min(.001f)] float analyticLimitPassEpsilon = .04f;
    [Tooltip("完了した極限点からこの距離だけ先へ進んだ後、次の候補探索を再開します。")]
    [SerializeField, Min(.05f)] float analyticLimitRearmDistance = .5f;
    [Tooltip("極限点で目標にするmarginです。0が理論上の境界です。僅かに正へすると支持余裕を残せます。")]
    [SerializeField, Range(-.25f, .5f)] float analyticLimitTargetMargin = 0f;
    [Tooltip("通常/Approach加速度から、極限速度へ必要な接線加速度へ置き換える最大割合です。加算はしません。")]
    [SerializeField, Range(0f, 1f)] float analyticLimitAccelerationBlend = .72f;
    [Tooltip("極限へ合わせる接線加減速度の絶対上限です。")]
    [SerializeField, Min(.1f)] float analyticLimitTangentialAccelerationLimit = 18f;
    [Tooltip("極限用接線加速度が1秒あたりに変化できる最大量です。急な制御切替を抑えます。")]
    [SerializeField, Min(.1f)] float analyticLimitTangentialJerkLimit = 90f;
    [Tooltip("極限捕捉中にAnalytic Extra Stickを残す割合です。0でも4位の検知は残り、法線力への寄与だけを譲ります。")]
    [SerializeField, Range(0f, 1f)] float analyticLimitExtraStickRemain = .18f;
    [Tooltip("曲率がこれ未満の点では、v²κの分母が不安定になるため極限計画を確定しません。")]
    [SerializeField, Min(.000001f)] float analyticLimitMinimumCurvature = .0005f;
    [Tooltip("極限目標速度が通常速度に対して過大にならないための安全倍率です。")]
    [SerializeField, Range(1f, 2f)] float analyticLimitTargetSpeedCeilingMultiplier = 1.2f;
    [SerializeField] bool debugAnalyticLimitCapture = true;
    [SerializeField] bool drawCapturedAnalyticLimit = true;

    [Header("Flat -> Slope Approach Debug")]
    [SerializeField] bool debugFlatToSlopeApproach = false;
    [SerializeField, Min(.02f)] float debugFlatToSlopeApproachInterval = .10f;
    [SerializeField] bool drawFlatToSlopeApproach = true;

    [Header("Slope Progress (Inspector Adjustable)")]
    [Tooltip("ONなら実際の斜面Collider上の位置から0～100%を自動計算します。OFFならCurrent Slope Progress Percentを解析・極限制御へそのまま使用します。")]
    [SerializeField] bool calculateSlopeProgressFromPosition = true;

    [Tooltip("斜面入口=0%、斜面出口=100%。自動計算がOFFのときはInspectorから設定した値がMargin、離脱点探索、Analytic Limitへ使われます。ONのときは実測値の表示欄になります。")]
    [SerializeField, Range(0f, 100f)] float currentSlopeProgressPercent = 0f;

    public bool useAnalyticTrackAssist;
    public float derivativeStep = .05f;
    public float lookAheadDistance = 4;
    public float analyticSharpness = 3;
    public int takeoffSearchSegments = 20;
    public float analyticAmplitude = 2;
    public float analyticFrequency = .5f;
    public float analyticYOffset;

    public bool drawGroundNormal = true;
    public bool drawTakeoffPoint = true;

    [Header("Limit Debug Log")]
    [Tooltip("currentMarginの極限状況をUnity Consoleへ表示します。")]
    [SerializeField] bool debugLimitLog = true;

    [Tooltip("ONなら接地中の全FixedUpdateで追跡ログを表示します。通常はOFF推奨です。")]
    [SerializeField] bool debugLimitEveryFixedStep = false;

    [Tooltip("通常の追跡ログを出す最短間隔です。")]
    [SerializeField, Min(.01f)] float debugLimitLogInterval = .10f;

    [Tooltip("abs(currentMargin)がこの値以下なら±0近傍として扱います。")]
    [SerializeField, Min(0f)] float debugLimitNearZero = .05f;

    [Tooltip("前回の通常ログからmarginがこの値以上変化した場合、間隔を待たず表示します。")]
    [SerializeField, Min(0f)] float debugLimitMeaningfulDelta = .10f;

    [Tooltip("接地と空中が切り替わった瞬間をConsoleへ表示します。")]
    [SerializeField] bool debugGroundStateChange = true;

    Vector3 restart;
    Vector3 headingDir = Vector3.right;
    Vector3 targetHeadingDir = Vector3.right;
    Vector3 groundNormal = Vector3.up;

    Vector2 flickStart;
    bool hasFlickStart;
    bool hasQueuedTurn;
    float queuedTurnAngle;
    float lastPlayerTurnSign = 1f;
    Tween headingTween;

    public float currentTangentSpeed;

    bool movementReady;
    bool isGrounded;
    bool hasTakeoffPoint;

    RaycastHit currentGroundHit;
    FlatToSlopeApproachState flatToSlopeApproachState;

    enum MotionSnapshotReason
    {
        None,
        AbruptSurfaceNormalChange,
        BeforeSoftHandoffVelocityChange
    }

    struct MotionSnapshotData
    {
        // 保存時点の完全な物理状態。分解値に不具合があった場合も検証できる正本です。
        public Vector3 velocity;
        public Vector3 angularVelocity;

        // 保存元の直交面座標系。
        public Vector3 sourceNormal;
        public Vector3 sourceHeading;
        public Vector3 sourceSide;

        // 速度を保存元座標へ符号付きで分解した値。
        public float forwardSpeed;
        public float lateralSpeed;
        public float normalSpeed;

        // 角速度を転がり軸・面法線軸・進行軸へ符号付きで分解した値。
        public float rollingAngularSpeed;
        public float yawAngularSpeed;
        public float headingAngularSpeed;

        public bool wasGrounded;
        public Collider sourceCollider;
        public float savedTime;
    }

    struct SurfaceMotionSnapshot
    {
        public bool valid;
        public MotionSnapshotReason reason;

        // 法線急変とVelocityChange直前の両方で、
        // 最初のReliableBaselineSamplerが確定した同一データを参照する。
        public MotionSnapshotData motion;

        public Vector3 targetNormal;
        public Vector3 targetHeading;

        // motion.savedTimeは初回Baseline取得時刻なので、
        // 面遷移側の待機時間はイベント開始時刻で別管理する。
        public float startedTime;
        public int stableFrames;
    }

    struct StageTurnMotionSnapshot
    {
        public bool valid;
        public bool pendingRestore;
        public MotionSnapshotData motion;
        public Vector3 targetNormal;
        public Vector3 targetHeading;
    }

    SurfaceMotionSnapshot surfaceMotionSnapshot;
    StageTurnMotionSnapshot stageTurnMotionSnapshot;

    MotionSnapshotData initialReliableBaselineSnapshot;
    bool hasInitialReliableBaselineSnapshot;

    public bool HasInitialReliableBaselineSnapshot => hasInitialReliableBaselineSnapshot;

    // 法線急変・VelocityChange直前・DOTween回転が共通参照する初回Baselineです。
    public Vector3 SharedReliableBaselineVelocity => initialReliableBaselineSnapshot.velocity;
    public Vector3 SharedReliableBaselineAngularVelocity => initialReliableBaselineSnapshot.angularVelocity;

    public bool HasUsableSavedBaselineFile => reliableBaselineSampler && reliableBaselineSampler.HasUsableSavedFile;

    bool stageTurnInProgress;
    bool hasPreviousSurfaceNormal;
    Vector3 previousSurfaceNormal = Vector3.up;

    struct SoftSlopeHandoffPlan
    {
        public bool valid;
        public bool applied;
        public Collider flatCollider;
        public Collider slopeCollider;
        public Vector3 boundaryPoint;
        public Vector3 flatDirection;
        public Vector3 flatNormal;
        public Vector3 slopeNormal;
        public Vector3 slopeTangent;
        public float targetEntrySpeed;
        public float targetNormalSpeed;
        public float armedTime;
        public float crossedTime;
    }

    SoftSlopeHandoffPlan softSlopeHandoffPlan;
    Vector3 currentSoftHandoffVelocityChange;
    Vector3 currentSoftHandoffReleaseAcceleration;
    float currentSoftHandoffReleaseWeight;

    float softHandoffLastFlatTime = float.NegativeInfinity;
    float softHandoffLastAppliedTime = float.NegativeInfinity;
    float softHandoffFlatRearmStartTime = float.NegativeInfinity;
    Collider softHandoffLastFlatCollider;
    Vector3 softHandoffLastFlatDirection = Vector3.forward;
    Vector3 softHandoffLastFlatNormal = Vector3.up;

    bool softHandoffLatched;
    int softHandoffSlopeConfirmCount;
    Collider softHandoffPendingSlopeCollider;
    Vector3 softHandoffPendingSlopeNormal = Vector3.up;
    Vector3 softHandoffPendingSlopePoint;

    float currentMargin;

    // 内部計算用の正規化値。0=斜面入口、1=斜面出口。
    // InspectorではcurrentSlopeProgressPercentを0～100%で調整する。
    // ワールド原点からの進行座標は保持しない。
    float currentSlopeProgress;
    float currentDistanceToSlopeStart;
    float takeoffProgress;
    Vector3 analyticTakeoffWorldPoint;
    SlopeProgressFrame currentSlopeProgressFrame;

    float nextFlatToSlopeApproachLogTime;
    float currentAdhesionWeight = 1f;
    float currentVirtualCurveAssist;
    float currentAnalyticExtra;
    float currentApproachStick;
    float currentLimitRemaining;
    float currentLimitTargetSpeed;
    float currentLimitAcceleration;
    float currentLimitInfluence;
    float currentLimitPredictionStick;

    enum AnalyticLimitPhase
    {
        Searching, Armed, Captured, Passed
    }

    struct SlopeProgressFrame
    {
        public bool valid;
        public Collider slopeCollider;
        public Vector3 normal;
        public Vector3 direction;
        public Vector3 startPoint;
        public Vector3 endPoint;
        public float length;
    }

    struct AnalyticLimitPlan
    {
        public bool valid;
        public AnalyticLimitPhase phase;
        public SlopeProgressFrame slopeFrame;
        public Vector3 direction;
        public float progress;
        public Vector3 worldPoint;
        public float capturedTime;
        public float captureMargin;
        public float targetSpeed;
        public float lastTangentialAcceleration;
        public float predictionStickAtCapture;
    }

    AnalyticLimitPlan analyticLimitPlan;

    struct FlatToSlopeApproachState
    {
        public bool valid;
        public Collider flatCollider;
        public Collider slopeCollider;
        public Vector3 boundaryPoint;
        public Vector3 slopePoint;
        public Vector3 flatNormal;
        public Vector3 slopeNormal;
        public Vector3 flatDirection;
        public Vector3 slopeDirection;
        public Vector3 controlNormal;
        public Vector3 controlTangent;
        public Vector3 filletStartPoint;
        public float remainingDistance;
        public float influence;
        public float targetEntrySpeed;
        public float targetNormalSpeed;
        public float approachStick;
        public float normalAngle;
        public float availableRadius;
        public float curvatureSpeedLimit;
        public float filletRawPhase;
        public float filletSmoothPhase;
        public float virtualCurvature;
        public float curveAssist;
        public float contactWeight;
        public float analyticExtra;
    }

    float nextLimitLogTime;
    float previousMargin;
    float previousLoggedMargin;
    bool hasPreviousMargin;
    bool hasPreviousLoggedMargin;
    bool wasNearZero;

    bool previousGrounded;
    bool hasGroundStateSample;

    public Vector3 HeadingDir => headingDir;
    public float CurrentMargin => currentMargin;
    public bool HasSlopeProgress => currentSlopeProgressFrame.valid;
    public float CurrentSlopeProgress => currentSlopeProgress;
    public float CurrentSlopeProgressPercent => currentSlopeProgressPercent;
    public bool CalculateSlopeProgressFromPosition => calculateSlopeProgressFromPosition;

    void Reset()
    {
        rb = GetComponent<Rigidbody>();
    }

    /// <summary>
    /// 画像で指定された「AddForceのみの非強制プリセット」を既存コンポーネントへ適用します。
    /// Inspectorに保存済みの値はソース初期値の変更だけでは更新されないため、
    /// コンポーネントの三点メニューまたは右クリックから実行してください。
    /// </summary>
    [ContextMenu("Apply Shown Non-Forced Preset")]
    void ApplyShownNonForcedPreset()
    {
        maxGroundSpeed = 15f;
        maxGroundAcceleration = 50f;
        airAcceleration = 6f;

        targetNormalSpeed = -2f;
        normalSnapSharpness = 25f;
        baseStickAcceleration = 24.6f;
        extraStickAcceleration = 20f;
        normalAccelerationLimit = 180f;

        flatToSlopeLookAheadDistance = 5f;
        flatToSlopeProbeStep = .05f;
        flatToSlopeProbeHeight = 1.5f;
        flatToSlopeProbeDistance = 4.5f;

        flatToSlopeTargetEntrySpeed = 10f;
        flatToSlopeLongitudinalAccelerationLimit = 35f;
        flatToSlopeLateralAccelerationLimit = 22f;
        flatToSlopeTargetNormalSpeed = -.35f;
        flatToSlopeApproachStickAcceleration = 15f;
        flatToSlopeStickStartDistance = 2f;

        useSoftSlopeEntryHandoff = true;
        softHandoffFlatMaxAngle = 4f;
        softHandoffSlopeMinAngle = 8f;
        softHandoffSlopeMaxAngle = 50f;
        softHandoffFlatMemorySeconds = .20f;
        softHandoffSlopeConfirmFrames = 1;
        softHandoffNormalConsistencyAngle = 25f;
        softHandoffMinimumForwardDot = .10f;
        softHandoffMinimumRearmSeconds = .12f;
        softHandoffFlatRearmSeconds = .04f;
        softHandoffVirtualTransitionDiameters = 2f;
        softHandoffUseGroundAccelerationSafety = true;
        softHandoffTangentialCorrection = .25f;
        softHandoffLateralCorrection = .35f;
        softHandoffNormalCorrection = .55f;
        softHandoffReleaseSeconds = .10f;
        softHandoffReleaseAccelerationLimit = 10f;
        softHandoffReleaseStrength = .35f;

        virtualFilletTangentBlend = .70f;
        virtualFilletNormalBlend = .55f;
        virtualFilletCurveAssistRatio = .65f;
        virtualFilletCurveAccelerationLimit = 40f;
        adhesionFullContactCastDistance = .15f;
        adhesionFadeEndCastDistance = .60f;

        useUnifiedNaturalAdhesion = true;
        useFlatToSlopeApproach = true;
        useBalancedVirtualFillet = true;
        useAnalyticLimitCapture = true;

#if UNITY_EDITOR
        UnityEditor.EditorUtility.SetDirty(this);
#endif

        Debug.Log("[SlopeStickBall3D] Non-forced parameter preset applied. " + "Runtime motion remains AddForce-based; only startup reset writes position/velocity.", this);
    }

    void OnValidate()
    {
        targetNormalSpeed = Mathf.Min(-.01f, targetNormalSpeed);
        flatToSlopeTargetNormalSpeed = Mathf.Min(-.01f, flatToSlopeTargetNormalSpeed);
        flatToSlopeProbeStep = Mathf.Max(.02f, flatToSlopeProbeStep);
        flatToSlopeLookAheadDistance = Mathf.Max(flatToSlopeProbeStep, flatToSlopeLookAheadDistance);
        flatToSlopeStickStartDistance = Mathf.Max(.05f, flatToSlopeStickStartDistance);
        flatToSlopeVirtualTransitionLength = Mathf.Max(.05f, flatToSlopeVirtualTransitionLength);
        flatToSlopeVirtualMinimumRadius = Mathf.Max(.05f, flatToSlopeVirtualMinimumRadius);
        softHandoffFlatMaxAngle = Mathf.Clamp(softHandoffFlatMaxAngle, 0f, 15f);
        softHandoffSlopeMinAngle = Mathf.Max(softHandoffFlatMaxAngle + .1f, softHandoffSlopeMinAngle);
        softHandoffSlopeMaxAngle = Mathf.Max(softHandoffSlopeMinAngle + .1f, softHandoffSlopeMaxAngle);
        softHandoffFlatMemorySeconds = Mathf.Max(0f, softHandoffFlatMemorySeconds);
        softHandoffSlopeConfirmFrames = Mathf.Clamp(softHandoffSlopeConfirmFrames, 1, 4);
        softHandoffMinimumRearmSeconds = Mathf.Max(0f, softHandoffMinimumRearmSeconds);
        softHandoffFlatRearmSeconds = Mathf.Max(0f, softHandoffFlatRearmSeconds);
        softHandoffVirtualTransitionDiameters = Mathf.Max(.25f, softHandoffVirtualTransitionDiameters);
        softHandoffReleaseSeconds = Mathf.Max(.02f, softHandoffReleaseSeconds);
        softHandoffReleaseAccelerationLimit = Mathf.Max(0f, softHandoffReleaseAccelerationLimit);
        adhesionFullContactCastDistance = Mathf.Max(0f, adhesionFullContactCastDistance);
        adhesionFadeEndCastDistance = Mathf.Max(adhesionFullContactCastDistance + .01f, adhesionFadeEndCastDistance);
        analyticLimitArmDistance = Mathf.Max(.05f, analyticLimitArmDistance);
        analyticLimitPassEpsilon = Mathf.Max(.001f, analyticLimitPassEpsilon);
        analyticLimitRearmDistance = Mathf.Max(analyticLimitPassEpsilon + .01f, analyticLimitRearmDistance);
        analyticLimitTangentialAccelerationLimit = Mathf.Max(.1f, analyticLimitTangentialAccelerationLimit);
        analyticLimitTangentialJerkLimit = Mathf.Max(.1f, analyticLimitTangentialJerkLimit);
        analyticLimitMinimumCurvature = Mathf.Max(.000001f, analyticLimitMinimumCurvature);
        surfaceNormalChangeTriggerAngle = Mathf.Clamp(surfaceNormalChangeTriggerAngle, .1f, 89f);
        surfaceSnapshotStableNormalTolerance = Mathf.Clamp(surfaceSnapshotStableNormalTolerance, .1f, 15f);
        surfaceSnapshotStableFrames = Mathf.Clamp(surfaceSnapshotStableFrames, 1, 10);
        surfaceSnapshotMinimumSeconds = Mathf.Max(0f, surfaceSnapshotMinimumSeconds);
        currentSlopeProgressPercent = Mathf.Clamp(currentSlopeProgressPercent, 0f, 100f);
    }

    void Awake()
    {
        if (!rb)
            rb = GetComponent<Rigidbody>();

        if (!headingTransform)
            headingTransform = transform;

        if (!reliableBaselineSampler)
            reliableBaselineSampler = GetComponent<ReliableBaselineSampler>();

        Vector3 startHeading = useSideTurnExperiment ? SnapToCardinalXZ(initialHeading) : Flat(initialHeading, headingTransform.forward);

        SetHeading(startHeading, false);
    }

    void Start()
    {
        FindStageReferences();
        StartCoroutine(DelayStart());
    }

    void Update()
    {
        // 初回静止サンプリングが完了するまでは、入力による旋回も予約しない。
        // この間はRigidbody本来の重力・衝突・摩擦だけで運動する。
        if (!movementReady || !rb || !hasInitialReliableBaselineSnapshot)
            return;

        ReadFlick();
    }

    void OnDestroy()
    {
        headingTween?.Kill();
    }

    void FixedUpdate()
    {
        if (!movementReady || !rb)
            return;

        isGrounded = ProbeGround(out RaycastHit hit);
        LogGroundStateChange(isGrounded, hit);

        currentGroundHit = isGrounded ? hit : default;

        Vector3 detectedNormal = isGrounded ? hit.normal.normalized : Vector3.up;
        groundNormal = detectedNormal;

        // 初回静止サンプリングが必要数へ到達するまでは、ボール制御を一切加えない。
        // Rigidbodyの重力・衝突・摩擦・反発だけをそのまま進行させる。
        if (UpdateInitialReliableBaselineSnapshot(hit))
        {
            if (reliableBaselineSampler &&
                reliableBaselineSampler.InitialRestSampleCount >= reliableBaselineSampler.RequiredInitialRestSampleCount &&
                !reliableBaselineSampler.FirstSamplingInCollect)
            {
                reliableBaselineSampler.FirstSamplingInCollect = true;
                Debug.Log("初回静止サンプリングが必要数に到達しました。", this);
            }

            DisableBallControllerDuringInitialBaseline();
            return;
        }

        // 初回基準が完成した次のFixedUpdateから、入力・自動加速・吸着・斜面補正を開始する。
        ApplyQueuedTurnAtFixedBoundary();

        bool restoredStageTurn = RestoreCompletedStageTurnMotionSnapshot();

        if (restoredStageTurn)
        {
            previousSurfaceNormal = detectedNormal;
            hasPreviousSurfaceNormal = isGrounded;
        }
        else
        {
            DetectAbruptSurfaceNormalChange(detectedNormal);
        }

        UpdateProbeGroundSoftSlopeHandoff(hit);
        

        if (isGrounded)
        {
            SolveGround(headingDir);
        }
        else
        {
            flatToSlopeApproachState = default;
            UpdateAnalyticLimitPlanWithoutGround();
            SolveAir(headingDir);
        }

        UpdateSurfaceMotionSnapshotCompletion();
    }

    void FindStageReferences()
    {
        if (!stagePivot)
        {
            GameObject center = GameObject.Find("Center1");

            if (center)
                stagePivot = center.transform;
        }

        if (!stageRoot)
            stageRoot = stagePivot;
    }

    void ReadFlick()
    {
        if (!rb)
            return;

        if (UnityEngine.Input.GetMouseButtonDown(0))
        {
            flickStart = UnityEngine.Input.mousePosition;
            hasFlickStart = true;
        }

        Vector3 tangentVelocity = Vector3.ProjectOnPlane(rb.velocity, groundNormal);

        currentTangentSpeed = tangentVelocity.magnitude;

        if (!UnityEngine.Input.GetMouseButtonUp(0) || !hasFlickStart)
        {
            return;
        }

        hasFlickStart = false;

        Vector2 flick = (Vector2)UnityEngine.Input.mousePosition - flickStart;

        float flickPixels = useSideTurnExperiment ? Mathf.Abs(flick.x) : flick.magnitude;

        if (flickPixels < minFlickPixels)
            return;

        Vector3 current = Flat(headingDir, headingTransform ? headingTransform.forward : transform.forward);

        Vector3 input;

        if (useSideTurnExperiment)
        {
            // 横方向が縦方向より弱いフリックは無視する。
            if (Mathf.Abs(flick.x) <= Mathf.Abs(flick.y))
                return;

            input = flick.x > 0f ? -Side(current) : Side(current);
        }
        else
        {
            GetCameraGroundBasis(out Vector3 forward, out Vector3 right);

            input = right * flick.x + forward * flick.y;

            if (input.sqrMagnitude <= 1e-6f)
                return;

            input.Normalize();
        }

        float rawAngle = Vector3.SignedAngle(current, input, Vector3.up);

        float turnAngle = GetLogarithmicTurnAngle(rawAngle, flickPixels);

        if (Mathf.Abs(turnAngle) > .001f)
            QueueTurnForNextPhysicsStep(turnAngle);
    }

    void QueueTurnForNextPhysicsStep(float turnAngle)
    {
        queuedTurnAngle = turnAngle;
        hasQueuedTurn = true;

        if (Mathf.Abs(turnAngle) > .001f)
            lastPlayerTurnSign = Mathf.Sign(turnAngle);
    }

    void ApplyQueuedTurnAtFixedBoundary()
    {
        if (!hasQueuedTurn)
            return;

        float turnAngle = queuedTurnAngle;

        hasQueuedTurn = false;
        queuedTurnAngle = 0f;

        Vector3 current = Flat(headingDir, headingTransform ? headingTransform.forward : transform.forward);

        SetHeading(Quaternion.AngleAxis(turnAngle, Vector3.up) * current);
    }

    float GetLogarithmicTurnAngle(float signedInputAngle, float flickPixels)
    {
        if (Mathf.Abs(signedInputAngle) <= .001f)
            return 0f;

        float maxAngle = Mathf.Min(maxTurnPerFlick, maxPlayerTurnAngle);

        if (maxAngle <= 0f)
            return 0f;

        if (flickTurnAngleSteps == null || flickTurnAngleSteps.Length == 0)
        {
            return Mathf.Sign(signedInputAngle) * Mathf.Min(12.5f, maxAngle);
        }

        float basePixels = Mathf.Max(minFlickPixels, 1f);

        float ratio = Mathf.Max(flickStrengthBandRatio, 1.01f);

        int index = Mathf.FloorToInt(Mathf.Log(Mathf.Max(flickPixels, basePixels) / basePixels, ratio));

        index = Mathf.Clamp(index, 0, flickTurnAngleSteps.Length - 1);

        float angle = Mathf.Clamp(flickTurnAngleSteps[index], Mathf.Min(12.5f, maxAngle), maxAngle);

        return Mathf.Sign(signedInputAngle) * angle;
    }

    void SetHeading(Vector3 requestedDirection, bool rotate = true)
    {
        Vector3 requested = Flat(requestedDirection, headingDir);

        bool snapToCardinal = useSideTurnExperiment && snapHeadingOnSideTest && maxPlayerTurnAngle >= 90f - .001f;

        if (snapToCardinal)
            requested = SnapToCardinalXZ(requested);

        if (!rotate)
        {
            targetHeadingDir = requested;
            ApplyHeading(requested);

            SetHeadingRotation(headingTransform, Quaternion.LookRotation(headingDir, Vector3.up));

            return;
        }

        Vector3 start = Flat(headingDir, requested);

        float angle = Mathf.Clamp(Vector3.SignedAngle(start, requested, Vector3.up), -maxPlayerTurnAngle, maxPlayerTurnAngle);

        targetHeadingDir = Flat(Quaternion.AngleAxis(angle, Vector3.up) * start, start);

        RotateHeadingAndStage(start, angle);
    }

    void RotateHeadingAndStage(Vector3 startDir, float playerAngle)
    {
        FindStageReferences();

        Transform target = headingTransform ? headingTransform : transform;

        if (!target)
            return;

        // 連続フリック時は、前回の保存状態を現在の途中状態へ戻してから新しく保存する。
        RestoreInterruptedStageTurnMotionSnapshot();
        headingTween?.Kill();

        Quaternion headingStart = target.rotation;

        Quaternion headingEnd = Quaternion.AngleAxis(playerAngle, Vector3.up) * headingStart;

        bool canRotateStage = stageRoot && stagePivot;

        bool playerIsStageChild = canRotateStage && transform.IsChildOf(stageRoot);

        Vector3 pivot = canRotateStage ? stagePivot.position : Vector3.zero;

        Vector3 stageStartPosition = canRotateStage ? stageRoot.position : Vector3.zero;

        Quaternion stageStartRotation = canRotateStage ? stageRoot.rotation : Quaternion.identity;

        Vector3 playerStartPosition = rb ? rb.position : transform.position;

        float stageMultiplier = stageTurnsOppositeToPlayer ? -1f : 1f;

        float duration = GetHeadingTurnDuration(playerAngle);

        SaveStageTurnMotionSnapshot(startDir);

        void Apply(float angle)
        {
            Quaternion playerTurn = Quaternion.AngleAxis(angle, Vector3.up);

            Quaternion stageTurn = Quaternion.AngleAxis(angle * stageMultiplier, Vector3.up);

            ApplyHeading(playerTurn * startDir);

            SetHeadingRotation(target, playerTurn * headingStart);

            ApplyStageAndPlayerOrbit(stageTurn, canRotateStage, playerIsStageChild, pivot, stageStartPosition, stageStartRotation, playerStartPosition);
        }

        Quaternion finalStageTurn = Quaternion.AngleAxis(playerAngle * stageMultiplier, Vector3.up);

        if (duration <= 0f || Mathf.Abs(playerAngle) <= .001f)
        {
            Apply(playerAngle);
            SetHeadingRotation(target, headingEnd);
            MarkStageTurnMotionSnapshotCompleted(finalStageTurn, targetHeadingDir);
            return;
        }

        headingTween = DOTween.To(() => 0f, Apply, playerAngle, duration).SetEase(headingTurnEase).SetUpdate(UpdateType.Fixed).OnComplete(() =>
        {
            ApplyHeading(targetHeadingDir);

            SetHeadingRotation(target, headingEnd);

            ApplyStageAndPlayerOrbit(finalStageTurn, canRotateStage, playerIsStageChild, pivot, stageStartPosition, stageStartRotation, playerStartPosition);

            MarkStageTurnMotionSnapshotCompleted(finalStageTurn, targetHeadingDir);
        });
    }

    void ApplyHeading(Vector3 direction)
    {
        headingDir = Flat(direction, targetHeadingDir);
    }

    float GetHeadingTurnDuration(float angle)
    {
        float max = Mathf.Max(maxPlayerTurnAngle, .001f);

        float min = Mathf.Min(12.5f, max);

        float t = max <= min + .001f ? 1f : Mathf.InverseLerp(min, max, Mathf.Abs(angle));

        return Mathf.Lerp(Mathf.Min(minimumHeadingTurnDuration, headingTurnDuration), headingTurnDuration, t);
    }

    void ApplyStageAndPlayerOrbit(Quaternion stageTurn, bool canRotateStage, bool playerIsStageChild, Vector3 pivot, Vector3 stageStartPosition, Quaternion stageStartRotation, Vector3 playerStartPosition)
    {
        if (!canRotateStage)
            return;

        stageRoot.SetPositionAndRotation(pivot + stageTurn * (stageStartPosition - pivot), stageTurn * stageStartRotation);

        if (!movePlayerAroundStagePivot || playerIsStageChild)
        {
            return;
        }

        Vector3 position = pivot + stageTurn * (playerStartPosition - pivot);

        if (rb && !rb.isKinematic)
            rb.MovePosition(position);
        else if (rb)
            rb.position = position;
        else
            transform.position = position;
    }

    void SetHeadingRotation(Transform target, Quaternion rotation)
    {
        if (target == transform && rb && !rb.isKinematic)
        {
            rb.MoveRotation(rotation);
        }
        else if (target)
        {
            target.rotation = rotation;
        }
    }

    void GetCameraGroundBasis(out Vector3 forward, out Vector3 right)
    {
        forward = cameraTransform ? Vector3.ProjectOnPlane(cameraTransform.forward, Vector3.up) : Vector3.forward;

        forward = Flat(forward, Vector3.forward);

        right = Flat(Vector3.Cross(Vector3.up, forward), Vector3.right);
    }

    static Vector3 Flat(Vector3 direction, Vector3 fallback)
    {
        direction = Vector3.ProjectOnPlane(direction, Vector3.up);

        if (direction.sqrMagnitude > 1e-6f)
            return direction.normalized;

        fallback = Vector3.ProjectOnPlane(fallback, Vector3.up);

        return fallback.sqrMagnitude > 1e-6f ? fallback.normalized : Vector3.right;
    }

    public static Vector3 Side(Vector3 direction)
    {
        direction = Flat(direction, Vector3.right);

        return new Vector3(-direction.z, 0f, direction.x);
    }

    static Vector3 SnapToCardinalXZ(Vector3 direction)
    {
        direction = Flat(direction, Vector3.right);

        if (Mathf.Abs(direction.x) >= Mathf.Abs(direction.z))
        {
            return direction.x >= 0f ? Vector3.right : Vector3.left;
        }

        return direction.z >= 0f ? Vector3.forward : Vector3.back;
    }

    static Vector3 FlatDirection(Vector3 direction, Vector3 fallback)
    {
        direction = Vector3.ProjectOnPlane(direction, Vector3.up);

        if (direction.sqrMagnitude > 1e-6f)
            return direction.normalized;

        fallback = Vector3.ProjectOnPlane(fallback, Vector3.up);

        if (fallback.sqrMagnitude > 1e-6f)
            return fallback.normalized;

        return Vector3.right;
    }

    bool UpdateInitialReliableBaselineSnapshot(RaycastHit hit)
    {
        if (hasInitialReliableBaselineSnapshot)
            return false;

        if (!reliableBaselineSampler)
        {
            reliableBaselineSampler = GetComponent<ReliableBaselineSampler>();

            if (!reliableBaselineSampler)
            {
                Debug.LogError("[SlopeStickBall3D] ReliableBaselineSampler component is missing. Ball control remains disabled and Rigidbody physics continues normally.", this);
                return true;
            }
        }

        bool stillSampling = reliableBaselineSampler.Tick(rb, isGrounded, hit, headingDir, currentGroundHit.collider, out ReliableBaselineSampler.BaselineResult result);

        if (!result.valid)
            return stillSampling;

        initialReliableBaselineSnapshot = ConvertBaselineResult(result);
        hasInitialReliableBaselineSnapshot = true;

        // この平均法線を、その後の法線急変判定の出発点にする。
        previousSurfaceNormal = result.normal;
        hasPreviousSurfaceNormal = true;

        // 基準保存より前の別Snapshotは破棄する。
        surfaceMotionSnapshot = default;
        stageTurnMotionSnapshot = default;
        stageTurnInProgress = false;

        return true;
    }

    static MotionSnapshotData ConvertBaselineResult(ReliableBaselineSampler.BaselineResult result)
    {
        return new MotionSnapshotData
        {
            velocity = result.velocity,
            angularVelocity = result.angularVelocity,
            sourceNormal = result.normal,
            sourceHeading = result.heading,
            sourceSide = result.side,
            forwardSpeed = result.forwardSpeed,
            lateralSpeed = result.lateralSpeed,
            normalSpeed = result.normalSpeed,
            rollingAngularSpeed = result.rollingAngularSpeed,
            yawAngularSpeed = result.yawAngularSpeed,
            headingAngularSpeed = result.headingAngularSpeed,
            wasGrounded = true,
            sourceCollider = result.sourceCollider,
            savedTime = result.savedTime
        };
    }

    void DisableBallControllerDuringInitialBaseline()
    {
        // 入力を持ち越さない。サンプリング終了直後に予約旋回が発生することを防ぐ。
        hasFlickStart = false;
        hasQueuedTurn = false;
        queuedTurnAngle = 0f;

        // 万一、再サンプリング開始時に旋回Tweenが残っていた場合も停止する。
        headingTween?.Kill();
        headingTween = null;
        stageTurnInProgress = false;

        // コントローラー内部の補助状態だけを消す。
        // rb.velocity、rb.angularVelocity、rb.position、rb.rotationには一切触れない。
        flatToSlopeApproachState = default;
        softSlopeHandoffPlan = default;
        surfaceMotionSnapshot = default;
        stageTurnMotionSnapshot = default;
        currentSoftHandoffVelocityChange = Vector3.zero;
        currentSoftHandoffReleaseAcceleration = Vector3.zero;
        currentSoftHandoffReleaseWeight = 0f;
        currentApproachStick = 0f;
        currentVirtualCurveAssist = 0f;
        currentAnalyticExtra = 0f;
        currentLimitAcceleration = 0f;
        currentLimitInfluence = 0f;
        currentLimitPredictionStick = 0f;
        currentAdhesionWeight = 0f;
        currentTangentSpeed = Vector3.ProjectOnPlane(rb.velocity, groundNormal).magnitude;
        ResetAnalyticLimitPlan();
    }

    void DetectAbruptSurfaceNormalChange(Vector3 detectedNormal)
    {
        if (!isGrounded)
            return;

        detectedNormal = NormalizeOrFallback(detectedNormal, Vector3.up);

        if (!hasPreviousSurfaceNormal)
        {
            previousSurfaceNormal = detectedNormal;
            hasPreviousSurfaceNormal = true;
            return;
        }

        // ステージ回転中の法線変化はStageTurnSnapshotが担当する。
        if (stageTurnInProgress || stageTurnMotionSnapshot.pendingRestore)
        {
            previousSurfaceNormal = detectedNormal;
            return;
        }

        float normalChange = Vector3.Angle(previousSurfaceNormal, detectedNormal);

        if (useSurfaceMotionSnapshot && normalChange >= surfaceNormalChangeTriggerAngle)
        {
            Vector3 newHeading = ProjectHeadingOnSurface(headingDir, detectedNormal);

            BeginOrRetargetSurfaceMotionSnapshot(previousSurfaceNormal, detectedNormal, newHeading, MotionSnapshotReason.AbruptSurfaceNormalChange);
        }

        previousSurfaceNormal = detectedNormal;
    }

    void BeginOrRetargetSurfaceMotionSnapshot(Vector3 sourceNormal, Vector3 targetNormal, Vector3 targetHeading, MotionSnapshotReason reason)
    {
        if (!useSurfaceMotionSnapshot || !rb || !HasInitialReliableBaselineSnapshot)
            return;

        if (!reliableBaselineSampler)
            reliableBaselineSampler = GetComponent<ReliableBaselineSampler>();

        if (!reliableBaselineSampler)
            return;

        sourceNormal = NormalizeOrFallback(sourceNormal, groundNormal);
        targetNormal = NormalizeOrFallback(targetNormal, groundNormal);
        Vector3 sourceHeading = ProjectHeadingOnSurface(headingDir, sourceNormal);
        targetHeading = ProjectHeadingOnSurface(targetHeading, targetNormal);

        ReliableBaselineSampler.CyclePieceType pieceType =
            reason == MotionSnapshotReason.BeforeSoftHandoffVelocityChange ? ReliableBaselineSampler.CyclePieceType.BeforeVelocityChange : ReliableBaselineSampler.CyclePieceType.AbruptNormalChange;

        // 今回のイベントを「現在サイクルのパズルピース」としてSamplerへ渡す。
        // Samplerは InitialGroundedRest → AbruptNormalChange →
        // BeforeVelocityChange → BeforeDOTweenTurn の順で1サイクルを完成させる。
        reliableBaselineSampler.CaptureCyclePiece(pieceType, rb, true, sourceNormal, sourceHeading, currentGroundHit.collider);

        // 未完成サイクルの途中値は信用しない。
        // JSONはReliableBaselineSamplerの起動時に一度だけ読み込み、
        // ここではメモリ上の信用済みサイクルから対応ピースを取得する。
        // ファイルが未生成・空・破損・無効ならSnapshot復元を行わず、
        // 現在のrb.velocity / rb.angularVelocityによる通常物理を継続する。
        if (!reliableBaselineSampler.TryGetTrustedPiece(pieceType, out ReliableBaselineSampler.BaselineResult trustedResult))
        {
            return;
        }

        // DOTween回転の保存が有効な間は、別Snapshotによる二重復元を行わない。
        if (stageTurnInProgress || stageTurnMotionSnapshot.valid || stageTurnMotionSnapshot.pendingRestore)
            return;

        MotionSnapshotData motion = ConvertBaselineResult(trustedResult);

        if (!surfaceMotionSnapshot.valid)
        {
            surfaceMotionSnapshot = new SurfaceMotionSnapshot
            {
                valid = true,
                reason = reason,
                motion = motion,
                targetNormal = targetNormal,
                targetHeading = targetHeading,
                startedTime = Time.fixedTime,
                stableFrames = 0
            };
        }
        else
        {
            // 法線急変とVelocityChange直前は別のピースなので、
            // 対応する完成済みピースへ差し替える。
            surfaceMotionSnapshot.reason = reason;
            surfaceMotionSnapshot.motion = motion;
            surfaceMotionSnapshot.targetNormal = targetNormal;
            surfaceMotionSnapshot.targetHeading = targetHeading;
            surfaceMotionSnapshot.startedTime = Time.fixedTime;
            surfaceMotionSnapshot.stableFrames = 0;
        }

        if (debugMotionSnapshot)
        {
            Debug.Log($"[Surface Trusted Cycle Piece Armed] reason={reason} " + $"trustedCycles={reliableBaselineSampler.TrustedCycleCount} time={motion.savedTime:F4} " +
                $"velocity={motion.velocity:F4} angularVelocity={motion.angularVelocity:F4} " + $"forward={motion.forwardSpeed:F4} lateral={motion.lateralSpeed:F4} normal={motion.normalSpeed:F4} " +
                $"roll={motion.rollingAngularSpeed:F4} yaw={motion.yawAngularSpeed:F4} " + $"headingSpin={motion.headingAngularSpeed:F4}", this);
        }
    }

    void UpdateSurfaceMotionSnapshotCompletion()
    {
        if (!surfaceMotionSnapshot.valid || stageTurnInProgress || stageTurnMotionSnapshot.valid)
            return;

        if (!isGrounded)
        {
            surfaceMotionSnapshot.stableFrames = 0;
            return;
        }

        if (Time.fixedTime - surfaceMotionSnapshot.startedTime < surfaceSnapshotMinimumSeconds)
            return;

        // Soft Handoffの短いRelease補正が終了してから、保存したリズムへ戻す。
        if (currentSoftHandoffReleaseWeight > .0001f)
        {
            surfaceMotionSnapshot.stableFrames = 0;
            return;
        }

        float normalError = Vector3.Angle(groundNormal, surfaceMotionSnapshot.targetNormal);

        if (normalError <= surfaceSnapshotStableNormalTolerance)
            surfaceMotionSnapshot.stableFrames++;
        else
            surfaceMotionSnapshot.stableFrames = 0;

        if (surfaceMotionSnapshot.stableFrames < surfaceSnapshotStableFrames)
            return;

        ReconstructSurfaceMotionFromSnapshot(surfaceMotionSnapshot.motion, groundNormal, surfaceMotionSnapshot.targetHeading,
            $"Surface/{surfaceMotionSnapshot.reason}");

        surfaceMotionSnapshot = default;
    }

    void SaveStageTurnMotionSnapshot(Vector3 startHeading)
    {
        if (!HasInitialReliableBaselineSnapshot)
        {
            stageTurnInProgress = false;
            stageTurnMotionSnapshot = default;
            return;
        }

        stageTurnInProgress = true;

        if (!useStageTurnMotionSnapshot || !rb)
        {
            stageTurnMotionSnapshot = default;
            return;
        }

        if (!reliableBaselineSampler)
            reliableBaselineSampler = GetComponent<ReliableBaselineSampler>();

        Vector3 sourceNormal = NormalizeOrFallback(groundNormal, Vector3.up);
        Vector3 sourceHeading = ProjectHeadingOnSurface(startHeading, sourceNormal);

        if (!reliableBaselineSampler)
        {
            stageTurnMotionSnapshot = default;
            return;
        }

        // DOTween開始直前を4番目のピースとして回収する。
        // この呼び出しで4ピースが揃えば、Sampler内で1サイクルが完成し、
        // 検証後に信用済み結果へサイクル単位で逐次平均される。
        reliableBaselineSampler.CaptureCyclePiece(ReliableBaselineSampler.CyclePieceType.BeforeDOTweenTurn, rb, isGrounded, sourceNormal, sourceHeading, currentGroundHit.collider);

        // JSONを再読込せず、起動時に読み込んだ信用済みサイクルのキャッシュを使用する。
        if (!reliableBaselineSampler.TryGetTrustedPiece(ReliableBaselineSampler.CyclePieceType.BeforeDOTweenTurn, out ReliableBaselineSampler.BaselineResult trustedResult))
        {
            // 保存済みJSONファイルが未生成・空・破損・無効なら回転自体は続行するが、
            // Snapshot復元は行わず現在の物理状態を維持する。
            stageTurnMotionSnapshot = default;
            return;
        }

        // 面遷移Snapshotが残っている場合は、完成済みピースで現在面へ戻してから
        // 回転用Snapshotへ切り替える。
        if (surfaceMotionSnapshot.valid)
        {
            ReconstructSurfaceMotionFromSnapshot(surfaceMotionSnapshot.motion, groundNormal, headingDir, "Surface/InterruptedByStageTurn");
            surfaceMotionSnapshot = default;
        }

        MotionSnapshotData motion = ConvertBaselineResult(trustedResult);

        stageTurnMotionSnapshot = new StageTurnMotionSnapshot
        {
            valid = true,
            pendingRestore = false,
            motion = motion,
            targetNormal = sourceNormal,
            targetHeading = sourceHeading
        };

        if (debugMotionSnapshot)
        {
            Debug.Log($"[Stage Turn Trusted Cycle Piece Armed] " + $"trustedCycles={reliableBaselineSampler.TrustedCycleCount} time={motion.savedTime:F4} " +
                $"velocity={motion.velocity:F4} angularVelocity={motion.angularVelocity:F4} " + $"forward={motion.forwardSpeed:F4} lateral={motion.lateralSpeed:F4} normal={motion.normalSpeed:F4}", this);
        }
    }

    void MarkStageTurnMotionSnapshotCompleted(Quaternion finalStageTurn, Vector3 finalHeading)
    {
        stageTurnInProgress = false;

        if (!stageTurnMotionSnapshot.valid)
            return;

        // motion.sourceNormalは初回Baselineの法線で固定されているため、
        // 回転開始時に保存した現在面(targetNormal)を回転させる。
        Vector3 stageTurnStartNormal = stageTurnMotionSnapshot.targetNormal;
        stageTurnMotionSnapshot.targetNormal = NormalizeOrFallback(finalStageTurn * stageTurnStartNormal, stageTurnStartNormal);
        stageTurnMotionSnapshot.targetHeading = ProjectHeadingOnSurface(finalHeading, stageTurnMotionSnapshot.targetNormal);
        stageTurnMotionSnapshot.pendingRestore = true;

        if (debugMotionSnapshot)
        {
            Debug.Log($"[Stage Turn Snapshot Awaiting Restore] time={Time.fixedTime:F4} " + $"targetNormal={stageTurnMotionSnapshot.targetNormal:F4} targetHeading={stageTurnMotionSnapshot.targetHeading:F4}", this);
        }
    }

    bool RestoreCompletedStageTurnMotionSnapshot()
    {
        if (!stageTurnMotionSnapshot.valid || !stageTurnMotionSnapshot.pendingRestore)
            return false;

        Vector3 targetNormal = isGrounded ? NormalizeOrFallback(groundNormal, stageTurnMotionSnapshot.targetNormal) : NormalizeOrFallback(stageTurnMotionSnapshot.targetNormal, Vector3.up);

        Vector3 targetHeading = ProjectHeadingOnSurface(stageTurnMotionSnapshot.targetHeading, targetNormal);

        ReconstructStageTurnMotionFromSnapshot(stageTurnMotionSnapshot.motion, targetNormal, targetHeading, "StageTurn/Completed");

        stageTurnMotionSnapshot = default;
        stageTurnInProgress = false;
        return true;
    }

    void RestoreInterruptedStageTurnMotionSnapshot()
    {
        if (!stageTurnMotionSnapshot.valid)
            return;

        Vector3 targetNormal = NormalizeOrFallback(groundNormal, stageTurnMotionSnapshot.motion.sourceNormal);
        Vector3 targetHeading = ProjectHeadingOnSurface(headingDir, targetNormal);

        ReconstructStageTurnMotionFromSnapshot(stageTurnMotionSnapshot.motion, targetNormal, targetHeading, "StageTurn/Interrupted");

        stageTurnMotionSnapshot = default;
        stageTurnInProgress = false;
    }

    MotionSnapshotData CaptureMotionSnapshotData(Vector3 sourceNormal, Vector3 sourceHeading)
    {
        sourceNormal = NormalizeOrFallback(sourceNormal, Vector3.up);
        sourceHeading = ProjectHeadingOnSurface(sourceHeading, sourceNormal);

        Vector3 sourceSide = Vector3.Cross(sourceNormal, sourceHeading);

        if (sourceSide.sqrMagnitude <= 1e-8f)
            sourceSide = ProjectHeadingOnSurface(Vector3.right, sourceNormal);
        else
            sourceSide.Normalize();

        Vector3 velocity = rb.velocity;
        Vector3 angularVelocity = rb.angularVelocity;

        return new MotionSnapshotData
        {
            velocity = velocity,
            angularVelocity = angularVelocity,
            sourceNormal = sourceNormal,
            sourceHeading = sourceHeading,
            sourceSide = sourceSide,
            forwardSpeed = Vector3.Dot(velocity, sourceHeading),
            lateralSpeed = Vector3.Dot(velocity, sourceSide),
            normalSpeed = Vector3.Dot(velocity, sourceNormal),
            rollingAngularSpeed = Vector3.Dot(angularVelocity, sourceSide),
            yawAngularSpeed = Vector3.Dot(angularVelocity, sourceNormal),
            headingAngularSpeed = Vector3.Dot(angularVelocity, sourceHeading),
            wasGrounded = isGrounded,
            sourceCollider = currentGroundHit.collider,
            savedTime = Time.fixedTime
        };
    }

    void ReconstructSurfaceMotionFromSnapshot(MotionSnapshotData motion, Vector3 targetNormal, Vector3 targetHeading, string reason)
    {
        if (!rb)
            return;

        BuildSurfaceBasis(targetNormal, targetHeading, out Vector3 rebuiltNormal, out Vector3 rebuiltHeading, out Vector3 targetSide);
        targetNormal = rebuiltNormal;
        targetHeading = rebuiltHeading;

        if (restoreSurfaceTangentSpeed)
        {
            float restoredNormalSpeed = motion.normalSpeed;

            if (preserveCurrentInwardNormalSpeedOnSurfaceRestore)
            {
                // 面から外へ向かう正の速度は復元しない。現在の面向き速度が負の場合だけ維持する。
                restoredNormalSpeed = Mathf.Min(Vector3.Dot(rb.velocity, targetNormal), 0f);
            }

            Vector3 targetVelocity = targetHeading * motion.forwardSpeed + targetSide * (motion.lateralSpeed * surfaceLateralVelocityRestoreRatio) + targetNormal * restoredNormalSpeed;

            ApplyVelocitySnapshotCorrection(targetVelocity);
        }

        if (restoreSurfaceAngularSpeed)
        {
            Vector3 targetAngularVelocity =
                targetSide * motion.rollingAngularSpeed + targetNormal * (motion.yawAngularSpeed * surfaceCrossAngularRestoreRatio) +
                targetHeading * (motion.headingAngularSpeed * surfaceCrossAngularRestoreRatio);

            rb.angularVelocity = targetAngularVelocity;
        }

        LogMotionSnapshotReconstruction(reason, motion, targetNormal, targetHeading);
    }

    void ReconstructStageTurnMotionFromSnapshot(MotionSnapshotData motion, Vector3 targetNormal, Vector3 targetHeading, string reason)
    {
        if (!rb)
            return;

        BuildSurfaceBasis(targetNormal, targetHeading, out Vector3 rebuiltNormal, out Vector3 rebuiltHeading, out Vector3 targetSide);
        targetNormal = rebuiltNormal;
        targetHeading = rebuiltHeading;

        if (restoreStageTurnTangentSpeed)
        {
            // DOTween回転では前後・横・法線の全符号付き速度成分を新しい面座標へ移す。
            Vector3 targetVelocity = targetHeading * motion.forwardSpeed + targetSide * motion.lateralSpeed + targetNormal * motion.normalSpeed;

            ApplyVelocitySnapshotCorrection(targetVelocity);
        }

        if (restoreStageTurnAngularSpeed)
        {
            // 角速度も全成分を新しい転がり軸・面法線軸・進行軸へ移す。
            rb.angularVelocity = targetSide * motion.rollingAngularSpeed + targetNormal * motion.yawAngularSpeed + targetHeading * motion.headingAngularSpeed;
        }

        LogMotionSnapshotReconstruction(reason, motion, targetNormal, targetHeading);
    }

    void ApplyVelocitySnapshotCorrection(Vector3 targetVelocity)
    {
        Vector3 deltaVelocity = targetVelocity - rb.velocity;

        if (deltaVelocity.sqrMagnitude > 1e-8f)
            rb.AddForce(deltaVelocity, ForceMode.VelocityChange);
    }

    static void BuildSurfaceBasis(Vector3 normalInput, Vector3 headingInput, out Vector3 normal, out Vector3 heading, out Vector3 side)
    {
        normal = NormalizeOrFallback(normalInput, Vector3.up);
        heading = ProjectHeadingOnSurface(headingInput, normal);
        side = Vector3.Cross(normal, heading);

        if (side.sqrMagnitude <= 1e-8f)
            side = ProjectHeadingOnSurface(Vector3.right, normal);
        else
            side.Normalize();
    }

    void LogMotionSnapshotReconstruction(string reason, MotionSnapshotData motion, Vector3 targetNormal, Vector3 targetHeading)
    {
        if (!debugMotionSnapshot)
            return;

        Debug.Log($"[Motion Snapshot Reconstructed] reason={reason} time={Time.fixedTime:F4} " + $"savedVelocity={motion.velocity:F4} savedAngularVelocity={motion.angularVelocity:F4} " +
            $"targetNormal={targetNormal:F4} targetHeading={targetHeading:F4} " + $"velocity={rb.velocity:F4} angularVelocity={rb.angularVelocity:F4}", this);
    }

    static Vector3 NormalizeOrFallback(Vector3 value, Vector3 fallback)
    {
        if (value.sqrMagnitude > 1e-8f)
            return value.normalized;

        return fallback.sqrMagnitude > 1e-8f ? fallback.normalized : Vector3.up;
    }

    static Vector3 ProjectHeadingOnSurface(Vector3 heading, Vector3 normal)
    {
        normal = NormalizeOrFallback(normal, Vector3.up);
        Vector3 projected = Vector3.ProjectOnPlane(heading, normal);

        if (projected.sqrMagnitude > 1e-8f)
            return projected.normalized;

        projected = Vector3.ProjectOnPlane(Vector3.forward, normal);

        if (projected.sqrMagnitude > 1e-8f)
            return projected.normalized;

        return Vector3.ProjectOnPlane(Vector3.right, normal).normalized;
    }

    bool ProbeGround(out RaycastHit hit)
    {
        Vector3 origin =
            transform.position + Vector3.up * .02f;

        bool found = Physics.SphereCast(
            origin,
            sphereRadius * .95f,
            Vector3.down,
            out hit,
            sphereRadius + groundProbeDistance,
            groundMask,
            QueryTriggerInteraction.Ignore);

        if (!found)
            return false;

        float surfaceAngle =
            Vector3.Angle(hit.normal, Vector3.up);

        // 歩行可能角度を超える面は地面から除外
        if (surfaceAngle > maxSlopeAngle)
        {
            hit = default;
            return false;
        }

        // 平面も斜面も接地として返す
        return true;
    }

    void SolveGround(Vector3 move)
    {
        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-6f);
        float snapRate = 1f - Mathf.Exp(-normalSnapSharpness * dt);

        Vector3 velocity = rb.velocity;
        Vector3 tangentVelocity = Vector3.ProjectOnPlane(velocity, groundNormal);
        Vector3 desiredVelocity = Vector3.ProjectOnPlane(move, groundNormal);

        Vector3 movementAxis = desiredVelocity.sqrMagnitude > 1e-6f ? desiredVelocity.normalized : Vector3.ProjectOnPlane(headingDir, groundNormal).normalized;

        if (movementAxis.sqrMagnitude <= 1e-6f)
            movementAxis = FlatDirection(headingDir, transform.forward);

        float tangentSpeed = Mathf.Abs(Vector3.Dot(tangentVelocity, movementAxis));

        desiredVelocity = desiredVelocity.sqrMagnitude > 1e-6f ? desiredVelocity.normalized * maxGroundSpeed : Vector3.zero;

        Vector3 baseMoveAcceleration = Vector3.ClampMagnitude((desiredVelocity - tangentVelocity) / dt, maxGroundAcceleration);

        bool approachActive = TryBuildFlatToSlopeApproach(movementAxis, out FlatToSlopeApproachState approach);

        Vector3 controlNormal = approachActive ? approach.controlNormal : groundNormal;

        if (controlNormal.sqrMagnitude <= 1e-6f)
            controlNormal = groundNormal;
        else
            controlNormal.Normalize();

        Vector3 moveAcceleration = baseMoveAcceleration;

        if (approachActive)
        {
            Vector3 approachAcceleration = CalculateFlatToSlopeApproachAcceleration(velocity, approach, dt);

            // 2位Approachは通常加速へ足し重ねず、入口へ近づくほど置き換える。
            moveAcceleration = Vector3.Lerp(baseMoveAcceleration, approachAcceleration, approach.influence);

            float combinedLimit = Mathf.Max(maxGroundAcceleration, Mathf.Sqrt(flatToSlopeLongitudinalAccelerationLimit *
                    flatToSlopeLongitudinalAccelerationLimit + flatToSlopeLateralAccelerationLimit * flatToSlopeLateralAccelerationLimit));

            moveAcceleration = Vector3.ClampMagnitude(moveAcceleration, combinedLimit);
        }

        currentAdhesionWeight = GetAdhesionContactWeight(currentGroundHit);
        currentVirtualCurveAssist = approachActive ? CalculateVirtualFilletCurveAssist(velocity, approach) : 0f;
        currentApproachStick = approachActive ? approach.approachStick : 0f;

        // 極限探索に使用する支持量は、2位Approachと仮想曲率のうち
        // 大きい方だけを採用する。4位Analytic Extraは循環参照を避けるため
        // この予測段階では含めない。
        float predictionSupplementalStick = Mathf.Max(currentApproachStick, currentVirtualCurveAssist);

        currentLimitPredictionStick = baseStickAcceleration + predictionSupplementalStick;

        float analyticExtra = 0f;

        // currentSlopeProgressはワールド座標ではなく、実Collider上の0～1進捗率。
        // 平面上でApproachが有効な間は次の斜面を0%として先読みし、
        // 斜面接地中は実際のCollider投影長に対する割合を使用する。
        bool hasSlopeProgressFrame = TryBuildCurrentSlopeProgressFrame(approachActive, approach, movementAxis, out SlopeProgressFrame slopeFrame);

        currentSlopeProgressFrame = hasSlopeProgressFrame ? slopeFrame : default;
        currentSlopeProgress = ResolveCurrentSlopeProgress(
            currentSlopeProgressFrame,
            rb.position);
        currentDistanceToSlopeStart = approachActive ? Mathf.Max(0f, approach.remainingDistance) : 0f;

        if (useAnalyticTrackAssist && hasSlopeProgressFrame)
        {
            float predictionMargin = MarginAtSlopeProgress(
                currentSlopeProgress, slopeFrame.length, tangentSpeed, currentLimitPredictionStick, derivativeStep);

            // 捕捉時ログと初期計画には、現在フレームの予測marginを使用する。
            currentMargin = predictionMargin;

            analyticExtra = extraStickAcceleration * Mathf.Exp(-analyticSharpness * Mathf.Max(predictionMargin, 0f));

            hasTakeoffPoint = FindTakeoffPointAhead(
                currentSlopeProgress, slopeFrame.length, tangentSpeed, currentLimitPredictionStick, derivativeStep, out takeoffProgress);
            if (hasTakeoffPoint)
            {
                analyticTakeoffWorldPoint = CalculateAnalyticWorldPoint(slopeFrame, takeoffProgress);
            }

            UpdateAnalyticLimitPlanDetection(
                slopeFrame, currentSlopeProgress, currentDistanceToSlopeStart, tangentSpeed,
                moveAcceleration, currentLimitPredictionStick, dt);
            if (analyticLimitPlan.phase == AnalyticLimitPhase.Captured)
            {
                Debug.Log("");
            }
            // Captured中は再探索した次の根へ切り替えず、同じ極限進捗率を表示・制御する。
            if (analyticLimitPlan.valid && analyticLimitPlan.phase == AnalyticLimitPhase.Captured)
            {
                hasTakeoffPoint = true;
                takeoffProgress = analyticLimitPlan.progress;
                analyticTakeoffWorldPoint = analyticLimitPlan.worldPoint;
            }
        }
        else
        {
            currentMargin = 999f;
            hasTakeoffPoint = false;
            takeoffProgress = 0f;

            if (!useAnalyticTrackAssist ||
                !hasSlopeProgressFrame ||
                !analyticLimitPlan.valid ||
                analyticLimitPlan.slopeFrame.slopeCollider != slopeFrame.slopeCollider)
            {
                ResetAnalyticLimitPlan();
            }
        }

        float weightedAnalytic = analyticExtra;

        if (approachActive)
        {
            float analyticFilletWeight = Mathf.Lerp(1f, virtualFilletAnalyticExtraRemain, approach.filletSmoothPhase);

            weightedAnalytic *= analyticFilletWeight;
        }

        // 極限点へ近づくほど4位Extra Stickの法線寄与を譲らせる。
        // 検知式自体は維持するため、4位の役割を削除していない。
        weightedAnalytic *= Mathf.Lerp(1f, analyticLimitExtraStickRemain, currentLimitInfluence);

        float supplementalStick = Mathf.Max(weightedAnalytic, Mathf.Max(currentApproachStick, currentVirtualCurveAssist));

        float totalStick = baseStickAcceleration + supplementalStick;

        currentAnalyticExtra = weightedAnalytic;

        // ログへ出すmarginは、実際に今回使用する法線支持量で再評価する。
        if (useAnalyticTrackAssist && currentSlopeProgressFrame.valid)
        {
            currentMargin = MarginAtSlopeProgress(
                currentSlopeProgress, currentSlopeProgressFrame.length, tangentSpeed, totalStick, derivativeStep);
        }

        Vector3 limitAxis = approachActive ? approach.controlTangent : movementAxis;

        moveAcceleration = ApplyAnalyticLimitAcceleration(moveAcceleration, velocity, controlNormal, limitAxis, totalStick, dt);

        float effectiveTargetNormalSpeed = targetNormalSpeed;

        if (approachActive)
        {
            effectiveTargetNormalSpeed = Mathf.Lerp(targetNormalSpeed, approach.targetNormalSpeed, approach.influence);

            approach.curveAssist = currentVirtualCurveAssist;
            approach.contactWeight = currentAdhesionWeight;
            approach.analyticExtra = weightedAnalytic;
        }

        float controlNormalSpeed = Vector3.Dot(velocity, controlNormal);

        Vector3 normalAcceleration;

        if (useUnifiedNaturalAdhesion)
        {
            normalAcceleration = CalculateUnifiedGroundStickAcceleration(controlNormal, controlNormalSpeed, effectiveTargetNormalSpeed, totalStick, snapRate, dt, currentAdhesionWeight);
        }
        else
        {
            normalAcceleration = CalculateLegacyGroundStickAcceleration(controlNormal, controlNormalSpeed, effectiveTargetNormalSpeed, totalStick, snapRate, dt, currentAdhesionWeight);
        }

        currentSoftHandoffReleaseAcceleration = CalculateSoftSlopeHandoffReleaseAcceleration(velocity, dt);

        // 接線側（2位＋極限）・法線側・境界通過後の短い解放補助を
        // 最後に一度だけAddForceする。
        rb.AddForce(moveAcceleration + normalAcceleration + currentSoftHandoffReleaseAcceleration, ForceMode.Acceleration);

        flatToSlopeApproachState = approachActive ? approach : default;

        if (approachActive)
        {
            LogFlatToSlopeApproach(approach, velocity, controlNormalSpeed);
        }

        if (useAnalyticTrackAssist)
        {
            LogLimitState(tangentSpeed, controlNormalSpeed, totalStick);
        }
    }

    void UpdateProbeGroundSoftSlopeHandoff(RaycastHit hit)
    {
        currentSoftHandoffVelocityChange = Vector3.zero;

        if (!useSoftSlopeEntryHandoff)
        {
            ResetProbeGroundSoftHandoffState();
            return;
        }

        float now = Time.fixedTime;
        float slopeAngle = isGrounded ? Vector3.Angle(hit.normal, Vector3.up) : 0f;

        bool currentIsFlat = isGrounded && hit.collider && slopeAngle <= softHandoffFlatMaxAngle;

        bool currentIsSlope = isGrounded && hit.collider && slopeAngle >= softHandoffSlopeMinAngle && slopeAngle <= softHandoffSlopeMaxAngle;

        if (currentIsFlat)
        {
            softHandoffLastFlatTime = now;
            softHandoffLastFlatCollider = hit.collider;
            softHandoffLastFlatNormal = hit.normal.normalized;
            softHandoffLastFlatDirection = Vector3.ProjectOnPlane(headingDir, hit.normal);

            if (softHandoffLastFlatDirection.sqrMagnitude <= 1e-6f)
            {
                softHandoffLastFlatDirection = FlatDirection(headingDir, transform.forward);
            }
            else
            {
                softHandoffLastFlatDirection.Normalize();
            }

            ResetPendingSoftSlopeCandidate();

            if (softHandoffLatched && now - softHandoffLastAppliedTime >= softHandoffMinimumRearmSeconds)
            {
                if (float.IsNegativeInfinity(softHandoffFlatRearmStartTime))
                {
                    softHandoffFlatRearmStartTime = now;
                }

                if (now - softHandoffFlatRearmStartTime >= softHandoffFlatRearmSeconds)
                {
                    softHandoffLatched = false;
                    softSlopeHandoffPlan = default;
                    currentSoftHandoffReleaseAcceleration = Vector3.zero;
                    currentSoftHandoffReleaseWeight = 0f;
                    softHandoffFlatRearmStartTime = float.NegativeInfinity;

                    if (debugSoftSlopeEntryHandoff)
                    {
                        Debug.Log($"[Soft Slope Handoff Rearmed] " + $"time={now:F4}s " + $"flat={hit.collider.name} " + $"slopeAngle={slopeAngle:F3}", this);
                    }
                }
            }
            else if (!softHandoffLatched)
            {
                softHandoffFlatRearmStartTime = float.NegativeInfinity;
            }

            return;
        }

        softHandoffFlatRearmStartTime = float.NegativeInfinity;

        if (!currentIsSlope)
        {
            // AIRを挟んでも最後の平面時刻は維持する。
            // ただし斜面の連続確認フレームは一旦切る。
            ResetPendingSoftSlopeCandidate();
            return;
        }
        else
        {
            Debug.Log("");
        }

        if (softHandoffLatched)
        {
            ResetPendingSoftSlopeCandidate();
            return;
        }

        float flatAge = now - softHandoffLastFlatTime;
        bool recentlyWasFlat = flatAge >= 0f && flatAge <= softHandoffFlatMemorySeconds;

        if (!recentlyWasFlat)
        {
            ResetPendingSoftSlopeCandidate();
            return;
        }

        Vector3 slopeNormal = hit.normal.normalized;
        Vector3 downhillDirection = Vector3.ProjectOnPlane(Physics.gravity, slopeNormal);

        if (downhillDirection.sqrMagnitude <= 1e-6f)
        {
            ResetPendingSoftSlopeCandidate();
            return;
        }

        downhillDirection.Normalize();

        Vector3 referenceForward = softHandoffLastFlatDirection.sqrMagnitude > 1e-6f ? softHandoffLastFlatDirection.normalized : FlatDirection(headingDir, transform.forward);

        if (Vector3.Dot(downhillDirection, referenceForward) < softHandoffMinimumForwardDot)
        {
            ResetPendingSoftSlopeCandidate();
            return;
        }

        bool sameCandidate = softHandoffPendingSlopeCollider == hit.collider && softHandoffSlopeConfirmCount > 0 && Vector3.Angle(softHandoffPendingSlopeNormal, slopeNormal) <= softHandoffNormalConsistencyAngle;

        if (sameCandidate)
        {
            float blend = 1f / (softHandoffSlopeConfirmCount + 1f);

            softHandoffPendingSlopeNormal = Vector3.Slerp(softHandoffPendingSlopeNormal, slopeNormal, blend).normalized;

            softHandoffPendingSlopePoint = Vector3.Lerp(softHandoffPendingSlopePoint, hit.point, blend);

            softHandoffSlopeConfirmCount++;
        }
        else
        {
            softHandoffPendingSlopeCollider = hit.collider;
            softHandoffPendingSlopeNormal = slopeNormal;
            softHandoffPendingSlopePoint = hit.point;
            softHandoffSlopeConfirmCount = 1;
        }

        if (softHandoffSlopeConfirmCount < softHandoffSlopeConfirmFrames)
        {
            return;
        }

        Vector3 confirmedNormal = softHandoffPendingSlopeNormal.normalized;

        Vector3 slopeTangent = Vector3.ProjectOnPlane(Physics.gravity, confirmedNormal);

        if (slopeTangent.sqrMagnitude <= 1e-6f)
        {
            slopeTangent = Vector3.ProjectOnPlane(referenceForward, confirmedNormal);
        }

        if (slopeTangent.sqrMagnitude <= 1e-6f)
        {
            ResetPendingSoftSlopeCandidate();
            return;
        }

        slopeTangent.Normalize();

        if (Vector3.Dot(slopeTangent, referenceForward) < 0f)
            slopeTangent = -slopeTangent;

        softSlopeHandoffPlan = new SoftSlopeHandoffPlan
        {
            valid = true, applied = true, flatCollider = softHandoffLastFlatCollider, slopeCollider = softHandoffPendingSlopeCollider,
            boundaryPoint = softHandoffPendingSlopePoint, flatDirection = referenceForward, flatNormal = softHandoffLastFlatNormal,
            slopeNormal = confirmedNormal, slopeTangent = slopeTangent, targetEntrySpeed = flatToSlopeTargetEntrySpeed,
            targetNormalSpeed = flatToSlopeTargetNormalSpeed, armedTime = now, crossedTime = now
        };

        softHandoffLatched = true;
        softHandoffLastAppliedTime = now;

        if (debugSoftSlopeEntryHandoff)
        {
            string flatName = softHandoffLastFlatCollider ? softHandoffLastFlatCollider.name : "(memory)";

            Debug.Log($"[Soft Slope Handoff Probe Detected] " + $"time={now:F4}s " + $"flat={flatName} " + $"slope={hit.collider.name} " +
                $"flatAge={flatAge:F4}s " + $"slopeAngle={slopeAngle:F3}deg " + $"confirmFrames={softHandoffSlopeConfirmCount}", this);
        }

        ApplySoftSlopeHandoffVelocityChange();
        ResetPendingSoftSlopeCandidate();
    }

    void ResetPendingSoftSlopeCandidate()
    {
        softHandoffSlopeConfirmCount = 0;
        softHandoffPendingSlopeCollider = null;
        softHandoffPendingSlopeNormal = Vector3.up;
        softHandoffPendingSlopePoint = Vector3.zero;
    }

    void ResetProbeGroundSoftHandoffState()
    {
        softSlopeHandoffPlan = default;
        currentSoftHandoffVelocityChange = Vector3.zero;
        currentSoftHandoffReleaseAcceleration = Vector3.zero;
        currentSoftHandoffReleaseWeight = 0f;

        softHandoffLastFlatTime = float.NegativeInfinity;
        softHandoffLastAppliedTime = float.NegativeInfinity;
        softHandoffFlatRearmStartTime = float.NegativeInfinity;
        softHandoffLastFlatCollider = null;
        softHandoffLastFlatNormal = Vector3.up;
        softHandoffLastFlatDirection = FlatDirection(headingDir, transform.forward);

        softHandoffLatched = false;
        ResetPendingSoftSlopeCandidate();
    }

    void ApplySoftSlopeHandoffVelocityChange()
    {
        Vector3 slopeNormal = softSlopeHandoffPlan.slopeNormal.normalized;
        Vector3 slopeTangent = Vector3.ProjectOnPlane(softSlopeHandoffPlan.slopeTangent, slopeNormal);

        if (slopeTangent.sqrMagnitude <= 1e-6f)
            return;

        slopeTangent.Normalize();

        Vector3 velocity = rb.velocity;
        Vector3 planarVelocity = Vector3.ProjectOnPlane(velocity, slopeNormal);

        float tangentSpeed = Vector3.Dot(planarVelocity, slopeTangent);

        Vector3 lateralVelocity = planarVelocity - slopeTangent * tangentSpeed;

        float desiredTangentSpeed = Mathf.Lerp(tangentSpeed, softSlopeHandoffPlan.targetEntrySpeed, softHandoffTangentialCorrection);

        Vector3 tangentialDelta = slopeTangent * (desiredTangentSpeed - tangentSpeed);

        Vector3 lateralDelta = -lateralVelocity * softHandoffLateralCorrection;

        float normalSpeed = Vector3.Dot(velocity, slopeNormal);

        float targetNormalSpeed = softSlopeHandoffPlan.targetNormalSpeed;

        Vector3 normalDelta = Vector3.zero;

        if (normalSpeed > targetNormalSpeed)
        {
            normalDelta = slopeNormal * (targetNormalSpeed - normalSpeed) * softHandoffNormalCorrection;
        }

        Vector3 requestedDeltaVelocity = tangentialDelta + lateralDelta + normalDelta;

        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-4f);
        float speed = velocity.magnitude;

        Vector3 flatNormal = softSlopeHandoffPlan.flatNormal.sqrMagnitude > 1e-6f ? softSlopeHandoffPlan.flatNormal.normalized : Vector3.up;

        float transitionAngleRadians = Vector3.Angle(flatNormal, slopeNormal) * Mathf.Deg2Rad;

        // 速度ベクトルを角度θだけ完全に回すのに必要な速度差:
        // |Δv_full| = 2 |v| sin(θ / 2)
        float fullRedirectDelta = 2f * speed * Mathf.Sin(transitionAngleRadians * .5f);

        // 仮想遷移長Lを球直径の倍数で決める。
        // 遷移時間T = L / |v|
        float sphereDiameter = Mathf.Max(2f * sphereRadius, .01f);
        float transitionLength = Mathf.Max(sphereDiameter * softHandoffVirtualTransitionDiameters, .05f);

        float transitionTime = transitionLength / Mathf.Max(speed, .10f);

        // 1 FixedUpdateで進める割合。フレームレートに依存しにくい
        // 指数応答 α = 1 - exp(-dt / T)
        float responseFraction = 1f - Mathf.Exp(-dt / Mathf.Max(transitionTime, dt));

        float geometricDeltaBudget = fullRedirectDelta * responseFraction;

        // 一般地上加速度で1 FixedUpdateに出せる速度変化を安全上限にする。
        float accelerationDeltaBudget = Mathf.Max(0f, maxGroundAcceleration) * dt;

        float adaptiveDeltaBudget = softHandoffUseGroundAccelerationSafety ? Mathf.Min(geometricDeltaBudget, accelerationDeltaBudget) : geometricDeltaBudget;

        Vector3 deltaVelocity = Vector3.ClampMagnitude(requestedDeltaVelocity, adaptiveDeltaBudget);

        if (deltaVelocity.sqrMagnitude <= 1e-8f)
            return;

        // VelocityChangeで速度を変更する直前のリズムを保存する。
        BeginOrRetargetSurfaceMotionSnapshot(flatNormal, slopeNormal, slopeTangent, MotionSnapshotReason.BeforeSoftHandoffVelocityChange);

        rb.AddForce(deltaVelocity, ForceMode.VelocityChange);

        currentSoftHandoffVelocityChange = deltaVelocity;

        if (debugSoftSlopeEntryHandoff)
        {
            Debug.Log($"[Soft Slope Handoff Applied] " + $"time={Time.fixedTime:F4}s " + $"deltaV={deltaVelocity:F5} " + $"magnitude={deltaVelocity.magnitude:F4} " + $"adaptiveBudget={adaptiveDeltaBudget:F4} " +
                $"angle={transitionAngleRadians * Mathf.Rad2Deg:F3}deg " + $"transitionLength={transitionLength:F3}m " +
                $"transitionTime={transitionTime:F4}s " + $"normalSpeed={normalSpeed:F4} " + $"tangentSpeed={tangentSpeed:F4}", this);
        }
    }

    Vector3 CalculateSoftSlopeHandoffReleaseAcceleration(Vector3 velocity, float dt)
    {
        currentSoftHandoffReleaseWeight = 0f;

        if (!useSoftSlopeEntryHandoff || !softSlopeHandoffPlan.valid || !softSlopeHandoffPlan.applied || softHandoffReleaseAccelerationLimit <= 0f || softHandoffReleaseStrength <= 0f)
        {
            currentSoftHandoffReleaseAcceleration = Vector3.zero;
            return Vector3.zero;
        }

        float elapsed = Time.fixedTime - softSlopeHandoffPlan.crossedTime;

        if (elapsed < 0f || elapsed >= softHandoffReleaseSeconds)
            return Vector3.zero;

        float raw = Mathf.Clamp01(elapsed / Mathf.Max(softHandoffReleaseSeconds, dt));

        float weight = 1f - QuinticSmooth01(raw);
        currentSoftHandoffReleaseWeight = weight;

        Vector3 slopeNormal = softSlopeHandoffPlan.slopeNormal.normalized;
        Vector3 slopeTangent = Vector3.ProjectOnPlane(softSlopeHandoffPlan.slopeTangent, slopeNormal);

        if (slopeTangent.sqrMagnitude <= 1e-6f)
            return Vector3.zero;

        slopeTangent.Normalize();

        Vector3 planarVelocity = Vector3.ProjectOnPlane(velocity, slopeNormal);

        float tangentSpeed = Vector3.Dot(planarVelocity, slopeTangent);

        Vector3 lateralVelocity = planarVelocity - slopeTangent * tangentSpeed;

        float responseTime = Mathf.Max(softHandoffReleaseSeconds, dt);

        Vector3 tangential = slopeTangent * (softSlopeHandoffPlan.targetEntrySpeed - tangentSpeed) / responseTime * softHandoffTangentialCorrection;

        Vector3 lateral = -lateralVelocity / responseTime * softHandoffLateralCorrection;

        float normalSpeed = Vector3.Dot(velocity, slopeNormal);

        Vector3 normal = Vector3.zero;

        if (normalSpeed > softSlopeHandoffPlan.targetNormalSpeed)
        {
            normal = slopeNormal * (softSlopeHandoffPlan.targetNormalSpeed - normalSpeed) / responseTime * softHandoffNormalCorrection;
        }

        Vector3 acceleration = Vector3.ClampMagnitude(tangential + lateral + normal, softHandoffReleaseAccelerationLimit);

        return acceleration * weight * softHandoffReleaseStrength;
    }

    Vector3 CalculateUnifiedGroundStickAcceleration(Vector3 controlNormal, float normalSpeed, float effectiveTargetNormalSpeed, float totalStick, float snapRate, float dt, float contactWeight)
    {
        // 3位の条件付き通常吸着。
        // すでに目標値以上の速さで参照面へ向かっている場合は追加しない。
        if (normalSpeed <= effectiveTargetNormalSpeed || contactWeight <= 0f)
        {
            return Vector3.zero;
        }

        float snappedNormalSpeed = Mathf.Lerp(normalSpeed, effectiveTargetNormalSpeed, snapRate);

        float normalAcceleration = (snappedNormalSpeed - normalSpeed) / dt - totalStick;

        normalAcceleration = Mathf.Clamp(normalAcceleration, -normalAccelerationLimit, 0f);

        return controlNormal * normalAcceleration * contactWeight;
    }

    Vector3 CalculateLegacyGroundStickAcceleration(Vector3 controlNormal, float normalSpeed, float effectiveTargetNormalSpeed, float totalStick, float snapRate, float dt, float contactWeight)
    {
        if (contactWeight <= 0f)
            return Vector3.zero;

        float targetSpeed = Mathf.Min(normalSpeed, effectiveTargetNormalSpeed);

        float snappedNormalSpeed = Mathf.Lerp(normalSpeed, targetSpeed, snapRate);

        float normalAcceleration = (snappedNormalSpeed - normalSpeed) / dt - totalStick;

        normalAcceleration = Mathf.Clamp(normalAcceleration, -normalAccelerationLimit, 0f);

        return controlNormal * normalAcceleration * contactWeight;
    }

    float GetAdhesionContactWeight(RaycastHit hit)
    {
        // 5位の早めSphereCast接地判定そのものは残す。
        // ただし遠いヒットで法線吸着を100%発生させない。
        if (!hit.collider)
            return 0f;

        float fullDistance = Mathf.Max(0f, adhesionFullContactCastDistance);

        float fadeEnd = Mathf.Max(fullDistance + .01f, adhesionFadeEndCastDistance);

        return 1f - Mathf.InverseLerp(fullDistance, fadeEnd, hit.distance);
    }

    float CalculateVirtualFilletCurveAssist(Vector3 velocity, FlatToSlopeApproachState state)
    {
        if (!useBalancedVirtualFillet || state.filletSmoothPhase <= 0f || state.virtualCurvature <= 0f || virtualFilletCurveAssistRatio <= 0f)
        {
            return 0f;
        }

        float speed = Mathf.Abs(Vector3.Dot(velocity, state.controlTangent));

        float requiredInwardAcceleration = speed * speed * state.virtualCurvature;

        Vector3 inwardNormal = -state.controlNormal;

        float gravityInwardAcceleration = Mathf.Max(0f, Vector3.Dot(Physics.gravity, inwardNormal));

        float deficit = Mathf.Max(0f, requiredInwardAcceleration - gravityInwardAcceleration);

        return Mathf.Min(virtualFilletCurveAccelerationLimit, deficit * virtualFilletCurveAssistRatio);
    }

    static float QuinticSmooth01(float t)
    {
        t = Mathf.Clamp01(t);
        return t * t * t * (t * (t * 6f - 15f) + 10f);
    }

    static float QuinticSmoothDerivative01(float t)
    {
        t = Mathf.Clamp01(t);
        float oneMinusT = 1f - t;
        return 30f * t * t * oneMinusT * oneMinusT;
    }

    bool TryBuildCurrentSlopeProgressFrame(
        bool approachActive,
        FlatToSlopeApproachState approach,
        Vector3 movementAxis,
        out SlopeProgressFrame frame)
    {
        frame = default;

        if (approachActive && approach.valid && approach.slopeCollider)
        {
            return TryBuildSlopeProgressFrame(
                approach.slopeCollider,
                approach.boundaryPoint,
                approach.slopeNormal,
                approach.slopeDirection,
                out frame);
        }

        if (!isGrounded || !currentGroundHit.collider)
            return false;

        float slopeAngle = Vector3.Angle(groundNormal, Vector3.up);

        if (slopeAngle < flatToSlopeSlopeMinAngle)
            return false;

        Vector3 slopeDirection = Vector3.ProjectOnPlane(Physics.gravity, groundNormal);

        if (slopeDirection.sqrMagnitude <= 1e-6f)
            return false;

        slopeDirection.Normalize();

        Vector3 forward = Vector3.ProjectOnPlane(movementAxis, groundNormal);

        if (forward.sqrMagnitude > 1e-6f && Vector3.Dot(slopeDirection, forward) < 0f)
            slopeDirection = -slopeDirection;

        return TryBuildSlopeProgressFrame(
            currentGroundHit.collider,
            currentGroundHit.point,
            groundNormal,
            slopeDirection,
            out frame);
    }

    bool TryBuildSlopeProgressFrame(
        Collider slopeCollider,
        Vector3 surfaceReferencePoint,
        Vector3 slopeNormal,
        Vector3 slopeDirection,
        out SlopeProgressFrame frame)
    {
        frame = default;

        if (!slopeCollider)
            return false;

        slopeNormal = NormalizeOrFallback(slopeNormal, Vector3.up);
        slopeDirection = Vector3.ProjectOnPlane(slopeDirection, slopeNormal);

        if (slopeDirection.sqrMagnitude <= 1e-6f)
            return false;

        slopeDirection.Normalize();

        if (!TryGetColliderProjectionRange(slopeCollider, slopeDirection, out float minimum, out float maximum))
            return false;

        float length = maximum - minimum;

        if (length <= 1e-4f)
            return false;

        float referenceCoordinate = Vector3.Dot(surfaceReferencePoint, slopeDirection);
        Vector3 startPoint = surfaceReferencePoint + slopeDirection * (minimum - referenceCoordinate);
        Vector3 endPoint = surfaceReferencePoint + slopeDirection * (maximum - referenceCoordinate);

        frame = new SlopeProgressFrame
        {
            valid = true,
            slopeCollider = slopeCollider,
            normal = slopeNormal,
            direction = slopeDirection,
            startPoint = startPoint,
            endPoint = endPoint,
            length = length
        };

        return true;
    }

    static bool TryGetColliderProjectionRange(
        Collider collider,
        Vector3 worldDirection,
        out float minimum,
        out float maximum)
    {
        minimum = 0f;
        maximum = 0f;

        if (!collider || worldDirection.sqrMagnitude <= 1e-8f)
            return false;

        worldDirection.Normalize();

        if (TryGetColliderLocalBounds(collider, out Bounds localBounds))
        {
            Transform colliderTransform = collider.transform;
            Vector3 worldCenter = colliderTransform.TransformPoint(localBounds.center);
            Vector3 extents = localBounds.extents;

            Vector3 worldExtentX = colliderTransform.TransformVector(Vector3.right * extents.x);
            Vector3 worldExtentY = colliderTransform.TransformVector(Vector3.up * extents.y);
            Vector3 worldExtentZ = colliderTransform.TransformVector(Vector3.forward * extents.z);

            float projectedCenter = Vector3.Dot(worldCenter, worldDirection);
            float projectedExtent =
                Mathf.Abs(Vector3.Dot(worldExtentX, worldDirection)) +
                Mathf.Abs(Vector3.Dot(worldExtentY, worldDirection)) +
                Mathf.Abs(Vector3.Dot(worldExtentZ, worldDirection));

            minimum = projectedCenter - projectedExtent;
            maximum = projectedCenter + projectedExtent;
            return maximum - minimum > 1e-4f;
        }

        // 未知のCollider型だけはworld AABBで近似する。
        Bounds worldBounds = collider.bounds;
        Vector3 worldExtents = worldBounds.extents;
        float centerProjection = Vector3.Dot(worldBounds.center, worldDirection);
        float extentProjection =
            Mathf.Abs(worldDirection.x) * worldExtents.x +
            Mathf.Abs(worldDirection.y) * worldExtents.y +
            Mathf.Abs(worldDirection.z) * worldExtents.z;

        minimum = centerProjection - extentProjection;
        maximum = centerProjection + extentProjection;
        return maximum - minimum > 1e-4f;
    }

    static bool TryGetColliderLocalBounds(Collider collider, out Bounds bounds)
    {
        if (collider is BoxCollider box)
        {
            bounds = new Bounds(box.center, box.size);
            return true;
        }

        if (collider is MeshCollider mesh && mesh.sharedMesh)
        {
            bounds = mesh.sharedMesh.bounds;
            return true;
        }

        if (collider is SphereCollider sphere)
        {
            bounds = new Bounds(sphere.center, Vector3.one * sphere.radius * 2f);
            return true;
        }

        if (collider is CapsuleCollider capsule)
        {
            Vector3 size = Vector3.one * capsule.radius * 2f;
            float axisLength = Mathf.Max(capsule.height, capsule.radius * 2f);

            if (capsule.direction == 0)
                size.x = axisLength;
            else if (capsule.direction == 1)
                size.y = axisLength;
            else
                size.z = axisLength;

            bounds = new Bounds(capsule.center, size);
            return true;
        }

        bounds = default;
        return false;
    }

    static float EvaluateSlopeProgressUnclamped(SlopeProgressFrame frame, Vector3 worldPosition)
    {
        if (!frame.valid || frame.length <= 1e-4f)
            return 0f;

        return Vector3.Dot(worldPosition - frame.startPoint, frame.direction) / frame.length;
    }

    float ResolveCurrentSlopeProgress(
        SlopeProgressFrame frame,
        Vector3 worldPosition)
    {
        float measuredProgress = frame.valid
            ? Mathf.Clamp01(EvaluateSlopeProgressUnclamped(frame, worldPosition))
            : 0f;

        if (calculateSlopeProgressFromPosition)
            currentSlopeProgressPercent = measuredProgress * 100f;

        currentSlopeProgressPercent = Mathf.Clamp(
            currentSlopeProgressPercent,
            0f,
            100f);

        return currentSlopeProgressPercent * .01f;
    }

    Vector3 CalculateAnalyticWorldPoint(SlopeProgressFrame frame, float progress)
    {
        if (!frame.valid)
            return transform.position;

        return Vector3.LerpUnclamped(frame.startPoint, frame.endPoint, Mathf.Clamp01(progress));
    }

    void ResetAnalyticLimitPlan()
    {
        analyticLimitPlan = default;
        analyticLimitPlan.phase = AnalyticLimitPhase.Searching;
        currentLimitRemaining = 0f;
        currentLimitTargetSpeed = 0f;
        currentLimitAcceleration = 0f;
        currentLimitInfluence = 0f;
    }

    void UpdateAnalyticLimitPlanWithoutGround()
    {
        currentLimitAcceleration = 0f;
        currentLimitInfluence = 0f;

        if (!analyticLimitPlan.valid)
            return;

        if (analyticLimitPlan.phase == AnalyticLimitPhase.Armed)
        {
            ResetAnalyticLimitPlan();
            return;
        }

        currentSlopeProgressFrame = analyticLimitPlan.slopeFrame;
        float progress = ResolveCurrentSlopeProgress(
            currentSlopeProgressFrame,
            rb.position);
        currentSlopeProgress = progress;
        currentDistanceToSlopeStart = 0f;

        UpdateAnalyticLimitPassAndRearm(progress);
    }

    void UpdateAnalyticLimitPlanDetection(
        SlopeProgressFrame slopeFrame,
        float progress,
        float distanceToSlopeStart,
        float speed,
        Vector3 plannedMoveAcceleration,
        float predictionStick,
        float dt)
    {
        currentLimitAcceleration = 0f;
        currentLimitInfluence = 0f;

        if (!useAnalyticLimitCapture || !useAnalyticTrackAssist || !slopeFrame.valid)
        {
            ResetAnalyticLimitPlan();
            return;
        }

        if (analyticLimitPlan.valid)
        {
            if (analyticLimitPlan.slopeFrame.slopeCollider != slopeFrame.slopeCollider)
            {
                ResetAnalyticLimitPlan();
            }
            else
            {
                // 同じColliderがステージ回転した場合も、0～1の進捗率を保ったまま枠だけ更新する。
                analyticLimitPlan.slopeFrame = slopeFrame;
                analyticLimitPlan.direction = slopeFrame.direction;
                analyticLimitPlan.worldPoint = CalculateAnalyticWorldPoint(slopeFrame, analyticLimitPlan.progress);
            }
        }

        UpdateAnalyticLimitPassAndRearm(progress);

        if (analyticLimitPlan.valid &&
            (analyticLimitPlan.phase == AnalyticLimitPhase.Captured || analyticLimitPlan.phase == AnalyticLimitPhase.Passed))
        {
            currentLimitRemaining =
                distanceToSlopeStart +
                (analyticLimitPlan.progress - progress) * slopeFrame.length;
            currentLimitInfluence = GetAnalyticLimitInfluence(progress, distanceToSlopeStart);
            return;
        }

        if (!hasTakeoffPoint)
        {
            if (analyticLimitPlan.valid && analyticLimitPlan.phase == AnalyticLimitPhase.Armed)
                ResetAnalyticLimitPlan();

            return;
        }

        float aheadDistance =
            distanceToSlopeStart +
            (takeoffProgress - progress) * slopeFrame.length;
        float observeDistance = Mathf.Max(
            analyticLimitArmDistance * 2f,
            analyticLimitArmDistance + speed * dt * 4f);

        if (aheadDistance < -analyticLimitPassEpsilon || aheadDistance > observeDistance)
        {
            if (analyticLimitPlan.valid && analyticLimitPlan.phase == AnalyticLimitPhase.Armed)
                ResetAnalyticLimitPlan();

            return;
        }

        analyticLimitPlan.valid = true;
        analyticLimitPlan.phase = AnalyticLimitPhase.Armed;
        analyticLimitPlan.slopeFrame = slopeFrame;
        analyticLimitPlan.direction = slopeFrame.direction;
        analyticLimitPlan.progress = takeoffProgress;
        analyticLimitPlan.worldPoint = analyticTakeoffWorldPoint;
        analyticLimitPlan.predictionStickAtCapture = predictionStick;

        float forwardAcceleration = Vector3.Dot(plannedMoveAcceleration, analyticLimitPlan.direction);
        float predictedTravel = Mathf.Max(0f, speed * dt + .5f * forwardAcceleration * dt * dt);
        float predictedAheadDistance = aheadDistance - predictedTravel;
        bool captureNow =
            aheadDistance <= analyticLimitArmDistance ||
            predictedAheadDistance <= analyticLimitArmDistance;

        if (!captureNow)
            return;

        if (!TryCalculateAnalyticLimitSpeed(
                takeoffProgress, slopeFrame.length, predictionStick, out float targetSpeed))
        {
            ResetAnalyticLimitPlan();
            return;
        }

        analyticLimitPlan.phase = AnalyticLimitPhase.Captured;
        analyticLimitPlan.capturedTime = Time.fixedTime;
        analyticLimitPlan.captureMargin = currentMargin;
        analyticLimitPlan.targetSpeed = targetSpeed;
        analyticLimitPlan.lastTangentialAcceleration = forwardAcceleration;
        currentLimitRemaining = aheadDistance;
        currentLimitTargetSpeed = targetSpeed;
        currentLimitInfluence = GetAnalyticLimitInfluence(progress, distanceToSlopeStart);

        if (debugAnalyticLimitCapture)
        {
            Debug.Log(
                $"[Analytic Limit Captured] " +
                $"time={Time.fixedTime:F4}s " +
                $"slopeProgress={takeoffProgress * 100f:F3}% " +
                $"aheadDistance={aheadDistance:F6} " +
                $"predictedAheadDistance={predictedAheadDistance:F6} " +
                $"speed={speed:F6} " +
                $"targetSpeed={targetSpeed:F6} " +
                $"predictionStick={predictionStick:F6}", this);
        }
    }

    void UpdateAnalyticLimitPassAndRearm(float progress)
    {
        if (!analyticLimitPlan.valid || !analyticLimitPlan.slopeFrame.valid)
            return;

        float remainingDistance =
            (analyticLimitPlan.progress - progress) * analyticLimitPlan.slopeFrame.length;

        currentLimitRemaining = remainingDistance;

        if (analyticLimitPlan.phase == AnalyticLimitPhase.Captured &&
            remainingDistance <= -analyticLimitPassEpsilon)
        {
            analyticLimitPlan.phase = AnalyticLimitPhase.Passed;
            analyticLimitPlan.lastTangentialAcceleration = 0f;
            currentLimitAcceleration = 0f;
            currentLimitInfluence = 0f;

            if (debugAnalyticLimitCapture)
            {
                Debug.Log(
                    $"[Analytic Limit Passed] " +
                    $"time={Time.fixedTime:F4}s " +
                    $"slopeProgress={analyticLimitPlan.progress * 100f:F3}% " +
                    $"remainingDistance={remainingDistance:F6} " +
                    $"margin={currentMargin:F8}", this);
            }
        }

        float traveledAfterTarget =
            (progress - analyticLimitPlan.progress) * analyticLimitPlan.slopeFrame.length;

        if (analyticLimitPlan.phase == AnalyticLimitPhase.Passed &&
            traveledAfterTarget >= analyticLimitRearmDistance)
        {
            ResetAnalyticLimitPlan();
        }
    }

    float GetAnalyticLimitInfluence(float progress, float distanceToSlopeStart = 0f)
    {
        if (!analyticLimitPlan.valid ||
            analyticLimitPlan.phase != AnalyticLimitPhase.Captured ||
            !analyticLimitPlan.slopeFrame.valid)
        {
            return 0f;
        }

        float remainingDistance =
            distanceToSlopeStart +
            (analyticLimitPlan.progress - progress) * analyticLimitPlan.slopeFrame.length;

        float raw = 1f - Mathf.Clamp01(
            remainingDistance / Mathf.Max(analyticLimitArmDistance, .001f));

        return QuinticSmooth01(raw);
    }

    bool TryGetAnalyticTrackTerms(
        float slopeProgress,
        float slopeLength,
        float h,
        out float gravitySupport,
        out float curvature)
    {
        slopeLength = Mathf.Max(slopeLength, 1e-4f);
        h = Mathf.Max(1e-4f, h);

        // currentSlopeProgressは無次元だが、微分と曲率はメートル単位で必要。
        // s = progress * slopeLength と等価変換し、既存のderivativeStepの単位を維持する。
        float localDistance = Mathf.Clamp01(slopeProgress) * slopeLength;

        float nextHeight = H(localDistance + h);
        float height = H(localDistance);
        float previousHeight = H(localDistance - h);

        float firstDerivative = (nextHeight - previousHeight) / (2f * h);
        float secondDerivative = (nextHeight - 2f * height + previousHeight) / (h * h);

        curvature = Mathf.Max(
            0f,
            -secondDerivative /
            Mathf.Pow(1f + firstDerivative * firstDerivative, 1.5f));

        gravitySupport =
            Mathf.Abs(Physics.gravity.y) /
            Mathf.Sqrt(1f + firstDerivative * firstDerivative);

        return curvature >= analyticLimitMinimumCurvature;
    }

    bool TryCalculateAnalyticLimitSpeed(
        float slopeProgress,
        float slopeLength,
        float stick,
        out float targetSpeed)
    {
        targetSpeed = 0f;

        if (!TryGetAnalyticTrackTerms(
                slopeProgress, slopeLength, derivativeStep,
                out float gravitySupport, out float curvature))
        {
            return false;
        }

        float numerator =
            gravitySupport + Mathf.Max(0f, stick) - analyticLimitTargetMargin;

        if (numerator <= 0f)
            return false;

        targetSpeed = Mathf.Sqrt(numerator / curvature);

        float referenceSpeed =
            Mathf.Max(.1f, Mathf.Max(maxGroundSpeed, flatToSlopeTargetEntrySpeed));

        targetSpeed = Mathf.Min(
            targetSpeed,
            referenceSpeed * analyticLimitTargetSpeedCeilingMultiplier);

        return true;
    }

    Vector3 ApplyAnalyticLimitAcceleration(
        Vector3 moveAcceleration,
        Vector3 velocity,
        Vector3 controlNormal,
        Vector3 requestedAxis,
        float totalStick,
        float dt)
    {
        currentLimitAcceleration = 0f;

        if (!useAnalyticLimitCapture ||
            !analyticLimitPlan.valid ||
            analyticLimitPlan.phase != AnalyticLimitPhase.Captured ||
            !analyticLimitPlan.slopeFrame.valid)
        {
            return moveAcceleration;
        }

        bool usingCurrentSlopeFrame =
            currentSlopeProgressFrame.valid &&
            currentSlopeProgressFrame.slopeCollider == analyticLimitPlan.slopeFrame.slopeCollider;

        float progress = usingCurrentSlopeFrame
            ? currentSlopeProgress
            : ResolveCurrentSlopeProgress(
                analyticLimitPlan.slopeFrame,
                rb.position);

        float distanceToSlopeStart = usingCurrentSlopeFrame
            ? Mathf.Max(0f, currentDistanceToSlopeStart)
            : 0f;

        currentSlopeProgressFrame = analyticLimitPlan.slopeFrame;
        currentSlopeProgress = Mathf.Clamp01(progress);

        UpdateAnalyticLimitPassAndRearm(progress);

        if (!analyticLimitPlan.valid ||
            analyticLimitPlan.phase != AnalyticLimitPhase.Captured)
        {
            return moveAcceleration;
        }

        float remainingDistance =
            distanceToSlopeStart +
            (analyticLimitPlan.progress - progress) * analyticLimitPlan.slopeFrame.length;

        if (remainingDistance <= 0f)
        {
            // 通過後の位置戻し・逆向き制御は行わない。
            return moveAcceleration;
        }

        Vector3 axis = Vector3.ProjectOnPlane(requestedAxis, controlNormal);

        if (axis.sqrMagnitude <= 1e-6f)
            axis = Vector3.ProjectOnPlane(analyticLimitPlan.direction, controlNormal);

        if (axis.sqrMagnitude <= 1e-6f)
            return moveAcceleration;

        axis.Normalize();

        if (Vector3.Dot(axis, analyticLimitPlan.direction) < 0f)
            axis = -axis;

        if (!TryCalculateAnalyticLimitSpeed(
                analyticLimitPlan.progress,
                analyticLimitPlan.slopeFrame.length,
                totalStick,
                out float targetSpeed))
        {
            return moveAcceleration;
        }

        analyticLimitPlan.targetSpeed = targetSpeed;
        currentLimitTargetSpeed = targetSpeed;
        currentLimitRemaining = remainingDistance;
        currentLimitInfluence = GetAnalyticLimitInfluence(progress, distanceToSlopeStart);

        Vector3 controlVelocity = Vector3.ProjectOnPlane(velocity, controlNormal);
        float currentSpeed = Mathf.Max(0f, Vector3.Dot(controlVelocity, axis));
        float controlDistance = Mathf.Max(remainingDistance, .04f);

        // 既存の等加速度式をそのまま使い、割合差だけを実距離へ戻す。
        float requiredAcceleration =
            (targetSpeed * targetSpeed - currentSpeed * currentSpeed) /
            (2f * controlDistance);

        requiredAcceleration = Mathf.Clamp(
            requiredAcceleration,
            -analyticLimitTangentialAccelerationLimit,
            analyticLimitTangentialAccelerationLimit);

        float jerkLimitedAcceleration = Mathf.MoveTowards(
            analyticLimitPlan.lastTangentialAcceleration,
            requiredAcceleration,
            analyticLimitTangentialJerkLimit * dt);

        analyticLimitPlan.lastTangentialAcceleration = jerkLimitedAcceleration;

        float existingAxisAcceleration = Vector3.Dot(moveAcceleration, axis);
        float blend = currentLimitInfluence * analyticLimitAccelerationBlend;
        float selectedAxisAcceleration = Mathf.Lerp(
            existingAxisAcceleration,
            jerkLimitedAcceleration,
            blend);

        currentLimitAcceleration = selectedAxisAcceleration;

        Vector3 result =
            moveAcceleration + axis * (selectedAxisAcceleration - existingAxisAcceleration);

        float combinedLimit = Mathf.Max(
            maxGroundAcceleration,
            Mathf.Sqrt(
                analyticLimitTangentialAccelerationLimit * analyticLimitTangentialAccelerationLimit +
                flatToSlopeLateralAccelerationLimit * flatToSlopeLateralAccelerationLimit));

        return Vector3.ClampMagnitude(result, combinedLimit);
    }

    bool TryBuildFlatToSlopeApproach(Vector3 intendedForward, out FlatToSlopeApproachState state)
    {
        state = default;

        if (!useFlatToSlopeApproach || !currentGroundHit.collider)
        {
            return false;
        }

        float currentSlopeAngle = Vector3.Angle(groundNormal, Vector3.up);

        if (currentSlopeAngle > flatToSlopeFlatMaxAngle)
            return false;

        Vector3 flatForward = Vector3.ProjectOnPlane(intendedForward, groundNormal);

        if (flatForward.sqrMagnitude <= 1e-6f)
            return false;

        flatForward.Normalize();

        float previousDistance = 0f;
        RaycastHit slopeHit = default;
        float slopeDistance = 0f;

        for (float distance = flatToSlopeProbeStep; distance <= flatToSlopeLookAheadDistance + 1e-4f; distance += flatToSlopeProbeStep)
        {
            if (!ProbeFlatToSlopeSurface(distance, flatForward, out RaycastHit sample))
            {
                previousDistance = distance;
                continue;
            }

            if (!IsForwardDownSlope(sample, flatForward, out _))
            {
                previousDistance = distance;
                continue;
            }

            slopeHit = sample;
            slopeDistance = distance;
            break;
        }

        if (!slopeHit.collider)
            return false;

        float low = Mathf.Max(0f, previousDistance);

        float high = slopeDistance;
        RaycastHit refinedSlope = slopeHit;

        for (int i = 0; i < 12; i++)
        {
            float middle = .5f * (low + high);

            if (ProbeFlatToSlopeSurface(middle, flatForward, out RaycastHit middleHit) && IsForwardDownSlope(middleHit, flatForward, out _))
            {
                high = middle;
                refinedSlope = middleHit;
            }
            else
            {
                low = middle;
            }
        }

        Vector3 slopeNormal = refinedSlope.normal.normalized;
        Vector3 slopeDirection = Vector3.ProjectOnPlane(Physics.gravity, slopeNormal);

        if (slopeDirection.sqrMagnitude <= 1e-6f)
            return false;

        slopeDirection.Normalize();

        if (Vector3.Dot(slopeDirection, flatForward) <= 0f)
            return false;

        Vector3 approachFlatDirection = Vector3.ProjectOnPlane(slopeDirection, groundNormal);

        if (approachFlatDirection.sqrMagnitude <= 1e-6f)
            approachFlatDirection = flatForward;
        else
            approachFlatDirection.Normalize();

        if (Vector3.Dot(approachFlatDirection, flatForward) < 0f)
            approachFlatDirection = -approachFlatDirection;

        float normalAngle = Vector3.Angle(groundNormal, slopeNormal);

        float angleRadians = Mathf.Max(normalAngle * Mathf.Deg2Rad, .001f);

        float availableRadius = Mathf.Max(flatToSlopeVirtualMinimumRadius, flatToSlopeVirtualTransitionLength / angleRadians);

        float stickInfluence = 1f - Mathf.Clamp01(high / Mathf.Max(flatToSlopeStickStartDistance, .001f));

        stickInfluence = Mathf.SmoothStep(0f, 1f, stickInfluence);

        float approachStick = flatToSlopeApproachStickAcceleration * stickInfluence;

        float availableInwardAcceleration = Mathf.Max(.01f, -Vector3.Dot(Physics.gravity, groundNormal) + baseStickAcceleration + approachStick);

        float curvatureSpeedLimit = Mathf.Sqrt(availableInwardAcceleration * availableRadius) * flatToSlopeCurvatureSafety;

        float configuredTargetSpeed = Mathf.Max(.1f, flatToSlopeTargetEntrySpeed);

        float targetEntrySpeed = flatToSlopeEnforceCurvatureSpeedLimit ? Mathf.Min(configuredTargetSpeed, curvatureSpeedLimit) : configuredTargetSpeed;

        float approachInfluence = 1f - Mathf.Clamp01(high / Mathf.Max(flatToSlopeLookAheadDistance, .001f));

        approachInfluence = Mathf.SmoothStep(0f, 1f, approachInfluence);

        float filletLength = Mathf.Max(flatToSlopeVirtualTransitionLength, .05f);

        float filletRawPhase = useBalancedVirtualFillet ? 1f - Mathf.Clamp01(high / filletLength) : 0f;

        float filletSmoothPhase = QuinticSmooth01(filletRawPhase);

        Vector3 fullVirtualNormal = Vector3.Slerp(groundNormal, slopeNormal, filletSmoothPhase).normalized;

        Vector3 controlNormal = Vector3.Slerp(groundNormal, fullVirtualNormal, virtualFilletNormalBlend).normalized;

        Vector3 rawVirtualTangent = Vector3.Slerp(approachFlatDirection, slopeDirection, filletSmoothPhase * virtualFilletTangentBlend);

        Vector3 controlTangent = Vector3.ProjectOnPlane(rawVirtualTangent, controlNormal);

        if (controlTangent.sqrMagnitude <= 1e-6f)
            controlTangent = approachFlatDirection;
        else
            controlTangent.Normalize();

        if (Vector3.Dot(controlTangent, flatForward) < 0f)
            controlTangent = -controlTangent;

        float appliedAngleRadians = angleRadians * virtualFilletNormalBlend;

        float virtualCurvature = useBalancedVirtualFillet ? appliedAngleRadians * QuinticSmoothDerivative01(filletRawPhase) / filletLength : 0f;

        Vector3 boundaryPoint = currentGroundHit.point + flatForward * high;

        Vector3 filletStartPoint = boundaryPoint - flatForward * filletLength;

        state = new FlatToSlopeApproachState
        {
            valid = true, flatCollider = currentGroundHit.collider, slopeCollider = refinedSlope.collider, boundaryPoint = boundaryPoint,
            slopePoint = refinedSlope.point, flatNormal = groundNormal, slopeNormal = slopeNormal, flatDirection = approachFlatDirection,
            slopeDirection = slopeDirection, controlNormal = controlNormal, controlTangent = controlTangent, filletStartPoint = filletStartPoint,
            remainingDistance = Mathf.Max(high, .001f), influence = approachInfluence, targetEntrySpeed = targetEntrySpeed,
            targetNormalSpeed = flatToSlopeTargetNormalSpeed, approachStick = approachStick, normalAngle = normalAngle,
            availableRadius = availableRadius, curvatureSpeedLimit = curvatureSpeedLimit, filletRawPhase = filletRawPhase,
            filletSmoothPhase = filletSmoothPhase, virtualCurvature = virtualCurvature, curveAssist = 0f, contactWeight = 0f, analyticExtra = 0f
        };

        return true;
    }

    Vector3 CalculateFlatToSlopeApproachAcceleration(Vector3 velocity, FlatToSlopeApproachState state, float dt)
    {
        Vector3 controlVelocity = Vector3.ProjectOnPlane(velocity, state.controlNormal);

        float signedSpeed = Vector3.Dot(controlVelocity, state.controlTangent);

        float remaining = Mathf.Max(state.remainingDistance, .02f);

        float longitudinalAcceleration;

        if (signedSpeed < 0f)
        {
            longitudinalAcceleration = flatToSlopeLongitudinalAccelerationLimit;
        }
        else
        {
            // 2位の距離ベース収束。
            // a=(ve²-v²)/(2s)で必要分だけ加減速し、仮想接線へ沿わせる。
            longitudinalAcceleration = (state.targetEntrySpeed * state.targetEntrySpeed - signedSpeed * signedSpeed) / (2f * remaining);
        }

        longitudinalAcceleration = Mathf.Clamp(longitudinalAcceleration, -flatToSlopeLongitudinalAccelerationLimit, flatToSlopeLongitudinalAccelerationLimit);

        Vector3 longitudinal = state.controlTangent * longitudinalAcceleration;

        Vector3 lateralVelocity = controlVelocity - state.controlTangent * signedSpeed;

        float arrivalSeconds = remaining / Mathf.Max(Mathf.Abs(signedSpeed), .5f);

        arrivalSeconds = Mathf.Max(arrivalSeconds, dt);

        Vector3 lateralAcceleration = -lateralVelocity / arrivalSeconds;

        lateralAcceleration = Vector3.ClampMagnitude(lateralAcceleration, flatToSlopeLateralAccelerationLimit);

        return longitudinal + lateralAcceleration;
    }

    bool ProbeFlatToSlopeSurface(float forwardDistance, Vector3 forward, out RaycastHit hit)
    {
        Vector3 origin = rb.position + forward * forwardDistance + Vector3.up * flatToSlopeProbeHeight;

        float probeRadius = Mathf.Max(.03f, sphereRadius * .25f);

        if (!Physics.SphereCast(origin, probeRadius, Vector3.down, out hit, flatToSlopeProbeHeight + flatToSlopeProbeDistance, groundMask, QueryTriggerInteraction.Ignore))
        {
            hit = default;
            return false;
        }

        if (!hit.collider || hit.collider.attachedRigidbody == rb)
        {
            hit = default;
            return false;
        }

        float slopeAngle = Vector3.Angle(hit.normal, Vector3.up);

        if (slopeAngle > maxSlopeAngle)
        {
            hit = default;
            return false;
        }

        return true;
    }

    bool IsForwardDownSlope(RaycastHit hit, Vector3 intendedForward, out Vector3 downhillDirection)
    {
        downhillDirection = Vector3.ProjectOnPlane(Physics.gravity, hit.normal);

        float slopeAngle = Vector3.Angle(hit.normal, Vector3.up);

        if (slopeAngle < flatToSlopeSlopeMinAngle || downhillDirection.sqrMagnitude <= 1e-6f)
        {
            return false;
        }

        downhillDirection.Normalize();

        return Vector3.Dot(downhillDirection, intendedForward) > .1f;
    }

    void LogFlatToSlopeApproach(FlatToSlopeApproachState state, Vector3 tangentVelocity, float normalSpeed)
    {
        if (!debugFlatToSlopeApproach || Time.fixedTime < nextFlatToSlopeApproachLogTime)
        {
            return;
        }

        float signedSpeed = Vector3.Dot(tangentVelocity, state.controlTangent);

        Debug.Log($"[Flat To Slope Approach]\n" + $"time={Time.fixedTime:F4}s\n" + $"remaining={state.remainingDistance:F6}\n" +
            $"influence={state.influence:F6}\n" + $"speed={signedSpeed:F6}\n" + $"targetEntrySpeed={state.targetEntrySpeed:F6}\n" +
            $"normalSpeed={normalSpeed:F6}\n" + $"targetNormalSpeed={state.targetNormalSpeed:F6}\n" + $"approachStick={state.approachStick:F6}\n" +
            $"filletRawPhase={state.filletRawPhase:F6}\n" + $"filletSmoothPhase={state.filletSmoothPhase:F6}\n" + $"virtualCurvature={state.virtualCurvature:F6}\n" + $"curveAssist={state.curveAssist:F6}\n" +
            $"analyticExtraUsed={state.analyticExtra:F6}\n" + $"contactWeight={state.contactWeight:F6}\n" + $"normalAngle={state.normalAngle:F6}\n" +
            $"availableRadius={state.availableRadius:F6}\n" + $"curvatureSpeedLimit={state.curvatureSpeedLimit:F6}", this);

        nextFlatToSlopeApproachLogTime = Time.fixedTime + debugFlatToSlopeApproachInterval;
    }

    void SolveAir(Vector3 move)
    {
        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-6f);

        Vector3 horizontalVelocity = Vector3.ProjectOnPlane(rb.velocity, Vector3.up);

        Vector3 desiredHorizontalVelocity = FlatDirection(move, headingDir) * maxGroundSpeed;

        Vector3 acceleration = (desiredHorizontalVelocity - horizontalVelocity) / dt;

        Vector3 airAccelerationVector = Vector3.ClampMagnitude(acceleration, airAcceleration);

        currentSoftHandoffReleaseAcceleration = CalculateSoftSlopeHandoffReleaseAcceleration(rb.velocity, dt);

        rb.AddForce(airAccelerationVector + currentSoftHandoffReleaseAcceleration, ForceMode.Acceleration);

        currentMargin = -1f;
        hasTakeoffPoint = analyticLimitPlan.valid && analyticLimitPlan.phase == AnalyticLimitPhase.Captured;

        if (hasTakeoffPoint)
        {
            takeoffProgress = analyticLimitPlan.progress;
            analyticTakeoffWorldPoint = analyticLimitPlan.worldPoint;
        }

        currentAdhesionWeight = 0f;
        currentVirtualCurveAssist = 0f;
        currentAnalyticExtra = 0f;
        currentApproachStick = 0f;
        currentLimitAcceleration = 0f;
    }

    float H(float localDistance)
    {
        return analyticYOffset +
               analyticAmplitude * Mathf.Cos(localDistance * analyticFrequency);
    }

    static bool Cross(float a, float b)
    {
        return a > 0f && b <= 0f ||
               a < 0f && b >= 0f ||
               Mathf.Approximately(b, 0f);
    }

    float MarginAtSlopeProgress(
        float slopeProgress,
        float slopeLength,
        float speed,
        float stick,
        float h)
    {
        TryGetAnalyticTrackTerms(
            slopeProgress,
            slopeLength,
            h,
            out float gravitySupport,
            out float curvature);

        return gravitySupport + stick - speed * speed * curvature;
    }

    bool FindTakeoffPointAhead(
        float startProgress,
        float slopeLength,
        float speed,
        float stick,
        float h,
        out float progress)
    {
        progress = Mathf.Clamp01(startProgress);
        slopeLength = Mathf.Max(slopeLength, 1e-4f);

        if (lookAheadDistance <= 0f || takeoffSearchSegments <= 0)
            return false;

        float maximumProgress = Mathf.Min(
            1f,
            progress + lookAheadDistance / slopeLength);

        float progressSpan = maximumProgress - progress;

        if (progressSpan <= 1e-6f)
            return false;

        float step = progressSpan / takeoffSearchSegments;
        float leftProgress = progress;
        float leftMargin = MarginAtSlopeProgress(
            leftProgress, slopeLength, speed, stick, h);

        for (int i = 1; i <= takeoffSearchSegments; i++)
        {
            float rightProgress = progress + step * i;
            float rightMargin = MarginAtSlopeProgress(
                rightProgress, slopeLength, speed, stick, h);

            if (!Cross(leftMargin, rightMargin))
            {
                leftProgress = rightProgress;
                leftMargin = rightMargin;
                continue;
            }

            float a = leftProgress;
            float b = rightProgress;
            float marginA = leftMargin;

            for (int j = 0; j < 24; j++)
            {
                float middle = (a + b) * .5f;
                float middleMargin = MarginAtSlopeProgress(
                    middle, slopeLength, speed, stick, h);

                if (Cross(marginA, middleMargin))
                {
                    b = middle;
                }
                else
                {
                    a = middle;
                    marginA = middleMargin;
                }
            }

            progress = Mathf.Clamp01((a + b) * .5f);
            return true;
        }

        return false;
    }

    void LogLimitState(float tangentSpeed, float normalSpeed, float totalStick)
    {
        if (!debugLimitLog)
            return;

        float now = Time.fixedTime;
        float absMargin = Mathf.Abs(currentMargin);
        float delta = hasPreviousMargin ? currentMargin - previousMargin : 0f;
        bool nearZero = absMargin <= debugLimitNearZero;
        bool crossedZero = hasPreviousMargin &&
            (previousMargin > 0f && currentMargin <= 0f ||
             previousMargin < 0f && currentMargin >= 0f);

        string side = GetLimitSide(currentMargin);
        string trend = GetLimitTrend(previousMargin, currentMargin, hasPreviousMargin);
        float slopePercent = currentSlopeProgress * 100f;

        if (nearZero && !wasNearZero)
        {
            Debug.Log(
                $"[Limit NearZero Enter] " +
                $"time={now:F4}s " +
                $"side={side} " +
                $"margin={currentMargin:F8} " +
                $"absMargin={absMargin:F8} " +
                $"trend={trend} " +
                $"slopeProgress={slopePercent:F3}% " +
                $"tangentSpeed={tangentSpeed:F6}", this);
        }

        if (crossedZero)
        {
            Debug.LogWarning(
                $"[Limit Cross] " +
                $"time={now:F4}s " +
                $"margin={previousMargin:F8} -> {currentMargin:F8} " +
                $"delta={delta:F8} " +
                $"slopeProgress={slopePercent:F3}% " +
                $"tangentSpeed={tangentSpeed:F6} " +
                $"takeoffFound={hasTakeoffPoint} " +
                $"takeoffProgress={takeoffProgress * 100f:F3}%", this);
        }

        bool intervalReached = now >= nextLimitLogTime;
        bool meaningfulChange =
            !hasPreviousLoggedMargin ||
            Mathf.Abs(currentMargin - previousLoggedMargin) >= debugLimitMeaningfulDelta;
        bool shouldTrace =
            debugLimitEveryFixedStep || intervalReached || meaningfulChange || nearZero || crossedZero;

        if (shouldTrace)
        {
            float takeoffAheadDistance =
                hasTakeoffPoint && currentSlopeProgressFrame.valid
                    ? (takeoffProgress - currentSlopeProgress) * currentSlopeProgressFrame.length
                    : 0f;

            Debug.Log(
                $"[Limit Trace]\n" +
                $"time={now:F4}s\n" +
                $"side={side}\n" +
                $"trend={trend}\n" +
                $"margin={currentMargin:F8}\n" +
                $"absMargin={absMargin:F8}\n" +
                $"delta={delta:F8}\n" +
                $"slopeProgress={slopePercent:F3}%\n" +
                $"slopeLength={currentSlopeProgressFrame.length:F6}\n" +
                $"distanceToSlopeStart={currentDistanceToSlopeStart:F6}\n" +
                $"slopeCollider={(currentSlopeProgressFrame.slopeCollider ? currentSlopeProgressFrame.slopeCollider.name : "(none)")}\n" +
                $"tangentSpeed={tangentSpeed:F6}\n" +
                $"normalSpeed={normalSpeed:F6}\n" +
                $"baseStick={baseStickAcceleration:F6}\n" +
                $"analyticExtra={currentAnalyticExtra:F6}\n" +
                $"approachStick={currentApproachStick:F6}\n" +
                $"virtualCurveAssist={currentVirtualCurveAssist:F6}\n" +
                $"adhesionWeight={currentAdhesionWeight:F6}\n" +
                $"totalStick={totalStick:F6}\n" +
                $"limitPhase={analyticLimitPlan.phase}\n" +
                $"limitRemainingDistance={currentLimitRemaining:F6}\n" +
                $"limitTargetSpeed={currentLimitTargetSpeed:F6}\n" +
                $"limitAcceleration={currentLimitAcceleration:F6}\n" +
                $"limitInfluence={currentLimitInfluence:F6}\n" +
                $"limitPredictionStick={currentLimitPredictionStick:F6}\n" +
                $"takeoffFound={hasTakeoffPoint}\n" +
                $"takeoffProgress={takeoffProgress * 100f:F3}%\n" +
                $"takeoffAheadDistance={takeoffAheadDistance:F6}", this);

            nextLimitLogTime = now + debugLimitLogInterval;
            previousLoggedMargin = currentMargin;
            hasPreviousLoggedMargin = true;
        }

        previousMargin = currentMargin;
        hasPreviousMargin = true;
        wasNearZero = nearZero;
    }

    void LogGroundStateChange(bool groundedNow, RaycastHit hit)
    {
        if (!hasGroundStateSample)
        {
            previousGrounded = groundedNow;
            hasGroundStateSample = true;
            return;
        }

        if (groundedNow == previousGrounded)
            return;

        if (debugGroundStateChange)
        {
            if (groundedNow)
            {
                float slopeAngle = Vector3.Angle(hit.normal, Vector3.up);

                string surfaceName = hit.collider ? hit.collider.name : "(unknown)";

                Debug.Log($"[Ground State] AIR -> GROUND " + $"time={Time.fixedTime:F4}s " + $"position={rb.position:F5} " + $"velocity={rb.velocity:F5} " + $"surface={surfaceName} " + $"slopeAngle={slopeAngle:F4}deg " +
                    $"lastMargin={currentMargin:F8}", this);
            }
            else
            {
                Debug.LogWarning($"[Ground State] GROUND -> AIR " + $"time={Time.fixedTime:F4}s " + $"position={rb.position:F5} " +
                    $"velocity={rb.velocity:F5} " + $"lastMargin={currentMargin:F8} " + $"takeoffFound={hasTakeoffPoint} " + $"takeoffProgress={takeoffProgress * 100f:F3}%", this);
            }
        }

        previousGrounded = groundedNow;
    }

    string GetLimitSide(float margin)
    {
        if (margin > debugLimitNearZero)
            return "+側（支持余裕あり）";

        if (margin >= 0f)
            return "+0近傍";

        if (margin >= -debugLimitNearZero)
            return "-0近傍";

        return "-側（離脱条件側）";
    }

    static string GetLimitTrend(float previous, float current, bool hasPrevious)
    {
        if (!hasPrevious)
            return "初回計測";

        float previousAbs = Mathf.Abs(previous);

        float currentAbs = Mathf.Abs(current);

        if (currentAbs < previousAbs - 1e-5f)
            return "0へ接近";

        if (currentAbs > previousAbs + 1e-5f)
            return "0から離反";

        return "ほぼ一定";
    }

    void OnDrawGizmosSelected()
    {
        if (drawGroundNormal)
        {
            Gizmos.color = isGrounded ? Color.green : Color.gray;

            Gizmos.DrawLine(transform.position, transform.position + groundNormal * 1.5f);
        }

        // 自動進行方向を青線で表示する。
        Vector3 previewHeading = Application.isPlaying ? headingDir : FlatDirection(initialHeading, transform.forward);

        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.position, transform.position + previewHeading * 2f);

        if (drawTakeoffPoint && useAnalyticTrackAssist && hasTakeoffPoint)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(analyticTakeoffWorldPoint, .15f);
            Gizmos.DrawLine(transform.position, analyticTakeoffWorldPoint);
        }

        if (drawCapturedAnalyticLimit && analyticLimitPlan.valid && (analyticLimitPlan.phase == AnalyticLimitPhase.Armed || analyticLimitPlan.phase == AnalyticLimitPhase.Captured))
        {
            Gizmos.color = analyticLimitPlan.phase == AnalyticLimitPhase.Captured ? Color.green : Color.yellow;

            Gizmos.DrawWireSphere(analyticLimitPlan.worldPoint, .22f);
            Gizmos.DrawLine(transform.position, analyticLimitPlan.worldPoint);
        }

        if (drawSoftSlopeEntryHandoff && softSlopeHandoffPlan.valid)
        {
            Gizmos.color = softSlopeHandoffPlan.applied ? Color.green : Color.yellow;

            Gizmos.DrawWireSphere(softSlopeHandoffPlan.boundaryPoint, .18f);
            Gizmos.DrawLine(transform.position, softSlopeHandoffPlan.boundaryPoint);

            Gizmos.color = Color.white;
            Gizmos.DrawLine(softSlopeHandoffPlan.boundaryPoint, softSlopeHandoffPlan.boundaryPoint + softSlopeHandoffPlan.slopeTangent * 1.25f);

            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(softSlopeHandoffPlan.boundaryPoint, softSlopeHandoffPlan.boundaryPoint + softSlopeHandoffPlan.slopeNormal * 1.0f);
        }

        if (drawFlatToSlopeApproach && flatToSlopeApproachState.valid)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(flatToSlopeApproachState.boundaryPoint, .12f);
            Gizmos.DrawLine(transform.position, flatToSlopeApproachState.boundaryPoint);

            Gizmos.color = Color.magenta;
            Gizmos.DrawSphere(flatToSlopeApproachState.slopePoint, .10f);
            Gizmos.DrawLine(flatToSlopeApproachState.slopePoint, flatToSlopeApproachState.slopePoint + flatToSlopeApproachState.slopeNormal);

            if (drawBalancedVirtualFillet && useBalancedVirtualFillet)
            {
                Gizmos.color = new Color(1f, .5f, 0f);
                Gizmos.DrawSphere(flatToSlopeApproachState.filletStartPoint, .08f);
                Gizmos.DrawLine(flatToSlopeApproachState.filletStartPoint, flatToSlopeApproachState.boundaryPoint);

                Gizmos.color = Color.cyan;
                Gizmos.DrawLine(transform.position, transform.position + flatToSlopeApproachState.controlNormal * 1.25f);

                Gizmos.color = Color.white;
                Gizmos.DrawLine(transform.position, transform.position + flatToSlopeApproachState.controlTangent * 1.5f);
            }
        }
    }

    IEnumerator DelayStart()
    {
        movementReady = false;

        yield return new WaitForSeconds(.8f);

        GameObject startSlab = GameObject.Find("ArcSlab4");

        if (!startSlab)
        {
            Debug.LogWarning("ArcSlab4が見つからないため、現在位置から自動進行を開始します。", this);

            surfaceMotionSnapshot = default;
            stageTurnMotionSnapshot = default;
            initialReliableBaselineSnapshot = default;
            hasInitialReliableBaselineSnapshot = false;

            if (!reliableBaselineSampler)
                reliableBaselineSampler = GetComponent<ReliableBaselineSampler>();

            reliableBaselineSampler?.ClearCompletedResult();

            stageTurnInProgress = false;
            hasPreviousSurfaceNormal = false;
            previousSurfaceNormal = Vector3.up;

            movementReady = true;
            yield break;
        }

        restart = startSlab.transform.position;

        rb.position = new Vector3(restart.x, restart.y + 2f, restart.z);

        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        Physics.SyncTransforms();

        headingTween?.Kill();
        hasFlickStart = false;
        hasQueuedTurn = false;
        queuedTurnAngle = 0f;

        currentGroundHit = default;
        flatToSlopeApproachState = default;
        softSlopeHandoffPlan = default;
        currentSoftHandoffVelocityChange = Vector3.zero;
        currentSoftHandoffReleaseAcceleration = Vector3.zero;
        currentSoftHandoffReleaseWeight = 0f;
        nextFlatToSlopeApproachLogTime = 0f;
        if (calculateSlopeProgressFromPosition)
            currentSlopeProgressPercent = 0f;

        currentSlopeProgress = Mathf.Clamp01(currentSlopeProgressPercent * .01f);
        currentDistanceToSlopeStart = 0f;
        takeoffProgress = 0f;
        currentSlopeProgressFrame = default;
        analyticTakeoffWorldPoint = Vector3.zero;
        currentAdhesionWeight = 0f;
        currentVirtualCurveAssist = 0f;
        currentAnalyticExtra = 0f;
        currentApproachStick = 0f;
        currentLimitPredictionStick = 0f;
        ResetAnalyticLimitPlan();

        surfaceMotionSnapshot = default;
        stageTurnMotionSnapshot = default;

        initialReliableBaselineSnapshot = default;
        hasInitialReliableBaselineSnapshot = false;

        if (!reliableBaselineSampler)
            reliableBaselineSampler = GetComponent<ReliableBaselineSampler>();

        reliableBaselineSampler?.ClearCompletedResult();

        stageTurnInProgress = false;
        hasPreviousSurfaceNormal = false;
        previousSurfaceNormal = Vector3.up;

        nextLimitLogTime = 0f;
        previousMargin = 0f;
        previousLoggedMargin = 0f;
        hasPreviousMargin = false;
        hasPreviousLoggedMargin = false;
        wasNearZero = false;
        previousGrounded = false;
        hasGroundStateSample = false;

        movementReady = true;
    }

}*/
