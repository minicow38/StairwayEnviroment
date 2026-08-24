using UnityEngine;
 using Sirenix.OdinInspector;
[Searchable]
[RequireComponent(typeof(Rigidbody))]
public class BallVisualSlopeDrive6 : MonoBehaviour
{
    private enum VisualPhase
    {
        Waiting,
        LockedToSubject,
        SlopeControlled,
        Released,
        Rejoining
    }

    // 極限ポイントから1回だけ跳ね、Flat着地までのタイムコストを測る研究用状態。
    private enum LimitExperimentPhase
    {
        Waiting,
        ApproachingLimit,

        // Unified Pre-Limit Guidance
        // Flat -> POP -> Apex -> Missile Descent -> Limit
        PreLimitAscent,
        PreLimitChase,

        // Unified Post-Limit Guidance
        PostLimitAscent,
        PostLimitChase,
        TerminalRejoin,

        // 旧A/B比較経路を残す
        Ballistic,
        FlatContact,

        Settled
    }

    [Header("References")]
    [SerializeField] private Rigidbody subjectBody;
    [SerializeField] private SlopeStick3D slopeStick;
    [SerializeField] private Rigidbody rb;
    [SerializeField] public float decisiveValue = 0f;
    [SerializeField] public CorrespondSubject respondSubject;


    [Header("Control Field")]
    [Tooltip("Subjectの実適用StickをBallVisualへ反映する倍率")]
    [Min(0f)]
    [SerializeField] private float stickScale = 0.10f;

    [Tooltip("Subjectの接線方向人工加速度を反映する倍率")]
    [Min(0f)]
    [SerializeField] private float tangentialAccelerationScale = 1f;

    [Tooltip("BallVisualへ適用する総加速度の上限")]
    [Min(0f)]
    [SerializeField] private float maximumControlAcceleration = 300f;

    [Tooltip("BallVisualのStick加速度が1秒間に変化できる上限")]
    [Min(0f)]
    [SerializeField] private float maximumBallStickJerk = 600f;

    [Header("Rolling")]
    [Tooltip("BallVisualの球半径")]
    [Min(0.001f)]
    [SerializeField] private float ballRadius = 0.5f;

    [Tooltip("目標角速度へ近づける強さ")]
    [Min(0f)]
    [SerializeField] private float rollingTorqueGain = 15f;

    [Tooltip("このStick量で回転補助を最大にする")]
    [Min(0.001f)]
    [SerializeField] private float stickForFullRollingControl = 100f;

    [Tooltip("回転補助に使う最大角加速度")]
    [Min(0f)]
    [SerializeField] private float maximumAngularAcceleration = 100f;

    [Tooltip("空中またはRelease中にも残す回転補助率")]
    [Range(0f, 1f)]
    [SerializeField] private float minimumRollingControl = 0.05f;

    [Header("Step Motion")]
    [Tooltip("実際の階段衝突に加えて、段周期の小さな上下演出を使う")]
    [SerializeField] private bool useProceduralStepPulse;

    [Tooltip("階段一段分の進行距離")]
    [Min(0.01f)]
    [SerializeField] private float stepLength = 1f;

    [Tooltip("段をまたぐ瞬間に加える法線方向の速度変化")]
    [Min(0f)]
    [SerializeField] private float stepPulseVelocity = 0.25f;

    [Tooltip("Stickがこの値以上なら段差パルスをほぼ抑える")]
    [Min(0.001f)]
    [SerializeField] private float stickForFullBounceSuppression = 100f;

    [Header("Synchronization")]
    [Tooltip("平面中は最終的にSubjectへ固定する")]
    [SerializeField] private bool lockOnFlat = true;

    [Tooltip("斜面入口でSubjectの速度を継承する")]
    [SerializeField] private bool inheritSubjectVelocityOnSlopeEntry = true;

    [Header("Natural Rejoin")]
    [Tooltip("Flat到達後も階段出口の慣性と最後の跳ねを残す時間")]
    [Min(0f)]
    [SerializeField] private float rejoinFreeSeconds = 0.05f;

    [Tooltip("位置・速度誤差を収束させる基準応答時間")]
    [Min(0.02f)]
    [SerializeField] private float rejoinResponseSeconds = 0.25f;

    [Tooltip("位置補正を0から徐々に立ち上げる時間")]
    [Min(0.01f)]
    [SerializeField] private float rejoinPositionBlendSeconds = 0.12f;

    [Tooltip("上下方向の位置・速度補正率。0なら上下は重力と接触へ任せる")]
    [Range(0f, 1f)]
    [SerializeField] private float rejoinVerticalWeight = 0.30f;

    [Tooltip("復帰加速度の絶対上限")]
    [Min(0f)]
    [SerializeField] private float maximumRejoinAcceleration = 40f;

    [Tooltip("復帰加速度が1秒間に変化できる上限")]
    [Min(0f)]
    [SerializeField] private float rejoinJerkLimit = 240f;

    [Tooltip("復帰中に実速度由来の回転へ近づける強さ")]
    [Min(0f)]
    [SerializeField] private float rejoinAngularGain = 8f;

    [Tooltip("復帰中の最大角加速度")]
    [Min(0f)]
    [SerializeField] private float maximumRejoinAngularAcceleration = 60f;

    [Tooltip("この距離以下なら完全同期候補")]
    [Min(0.001f)]
    [SerializeField] private float rejoinLockDistance = 0.10f;

    [Tooltip("この速度差以下なら完全同期候補")]
    [Min(0.001f)]
    [SerializeField] private float rejoinLockVelocityError = 0.75f;

    [Tooltip("同期条件を連続して満たすFixedUpdate数")]
    [Min(1)]
    [SerializeField] private int rejoinStableFramesRequired = 3;

    [Header("Debug")]
    [SerializeField] private bool enableDebugLog = true;

    [Min(1)]
    [SerializeField] private int logEveryFixedFrames = 10;

    private Rigidbody ballBody;
    private SphereCollider ballCollider;

    private VisualPhase phase = VisualPhase.Waiting;
    private VisualPhase previousLoggedPhase = VisualPhase.Waiting;

    private bool wasOnSlope;
    private bool rejoinPending;
    private bool hasEnteredSlopeAtLeastOnce;

    private int fixedFrameCounter;

    private Vector3 currentSurfaceNormal = Vector3.up;
    private Vector3 currentSlopeTangent = Vector3.forward;

    private Vector3 debugAppliedLinearAcceleration;
    private Vector3 debugTargetAngularVelocity;
    private Vector3 debugAppliedAngularAcceleration;

    private float ballStickState;

    private float debugAppliedStick;
    private float debugAppliedTangentialAcceleration;
    private float debugReleaseWeight;
    private float debugRollingWeight;

    private float accumulatedSlopeDistance;
    private int previousStepIndex;

    private float rejoinElapsed;
    private float debugRejoinPositionWeight;
    private Vector3 rejoinAccelerationState;
    private int rejoinStableFrames;

    private Vector3 previousLockedSubjectPosition;
    private Vector3 lockedRollingAngularVelocity;
    public float totalTime = 0, sumTime = 0;
    [SerializeField] public float stamp = 0;

    [Header("Limit Bounce Experiment")]
    [Tooltip("極限ポイント→1回Bounce→Flat着地→Settled の研究モードを有効にする")]
    [SerializeField] private bool enableLimitBounceExperiment = true;

    [Tooltip("旧方式の極限Bounce上向き速度[m/s]。Energy + Contact BounceをOFFにした時だけ使う")]
    [Min(0f)]
    [SerializeField] private float limitBounceDeltaV = 3f;

    [Header("Energy + Subject Relative POP")]
    [Tooltip("1st/2nd POPの高さを固定Y速度ではなくSubject相対Energyから作る")]
    [SerializeField] private bool useSubjectRelativeEnergyPop = true;

    [Tooltip("1st POPもSubject相対の目標高さから初速を逆算する")]
    [SerializeField] private bool useEnergyHeightForFirstPop = true;

    [Tooltip("1st POPのSubject相対Apex高さ[m]")]
    [Min(0f)]
    [SerializeField] private float preLimitTargetHeightRelativeToSubject = 0.45f;

    [Tooltip("2nd POPのSubject相対Apex高さ[m]")]
    [Min(0f)]
    [SerializeField] private float postLimitTargetHeightRelativeToSubject = 0.45f;

    [Header("Unified POP Algebra - v^2 Bridge")]
    [Tooltip("Subject/SlopeStick側の速度二乗をPOP高さへ反映する。h = eta * v^2 / (2 g_rel)")]
    [SerializeField] private bool useSlopeStickVelocitySquaredHeightScale = true;

    [Tooltip("固定Target Heightとv^2由来Heightの混合率。0=従来固定高さ、1=完全v^2同期")]
    [Range(0f, 1f)]
    [SerializeField] private float slopeStickEnergyHeightBlend = 1f;

    [Tooltip("1st POP: Qpop/Qsubject。24m/s時に約0.45mなら0.0153前後")]
    [Min(0f)]
    [SerializeField] private float firstPopVelocitySquaredRatio = 0.01533f;

    [Tooltip("2nd POP: Qpop/Qsubject。1stと同じなら高さのエネルギー尺度が連続する")]
    [Min(0f)]
    [SerializeField] private float secondPopVelocitySquaredRatio = 0.01533f;

    [Tooltip("v^2同期で生成する最小Subject相対Apex高さ[m]")]
    [Min(0f)]
    [SerializeField] private float minimumEnergyScaledPopHeight = 0.08f;

    [Tooltip("v^2同期で生成する最大Subject相対Apex高さ[m]")]
    [Min(0.01f)]
    [SerializeField] private float maximumEnergyScaledPopHeight = 1.50f;

    [Header("Subject Relative Gravity")]
    [Tooltip("ONならg_rel = -(g - a_subject)・up をPOP高さ計算へ使う。研究用。OFFならUnity重力のみ")]
    [SerializeField] private bool useSubjectRelativeGravity = false;

    [Tooltip("Subject加速度推定の指数平滑化強度")]
    [Min(0.1f)]
    [SerializeField] private float subjectAccelerationSmoothing = 12f;

    [Tooltip("数値不安定を避けるSubject相対重力の下限[m/s^2]")]
    [Min(0.1f)]
    [SerializeField] private float minimumSubjectRelativeGravity = 2f;

    [Tooltip("数値不安定を避けるSubject相対重力の上限[m/s^2]")]
    [Min(0.1f)]
    [SerializeField] private float maximumSubjectRelativeGravity = 30f;

    [Header("Limit Contact Direction")]
    [Tooltip("2nd POP方向へ階段接触角を反映する。0=真上、1=接触反射方向")]
    [Range(0f, 1f)]
    [SerializeField] private float postLimitContactDirectionWeight = 0.85f;

    [Tooltip("接触法線方向の反発係数。1=完全反射(Vector3.Reflect相当)、0=法線相対速度を消す")]
    [Range(0f, 1f)]
    [SerializeField] private float postLimitNormalRestitution = 0.65f;

    [Tooltip("60%の-0/+0跨ぎを補間して得た法線を2nd POP方向へ優先使用する")]
    [SerializeField] private bool preferInterpolatedLimitNormal = true;

    [Tooltip("2nd POP方向に必ず残す最小上向き成分。浅すぎる角度による過大速度を防ぐ")]
    [Range(0.05f, 1f)]
    [SerializeField] private float minimumPostLimitUpDot = 0.35f;

    [Tooltip("接触角が浅い場合でも許可するSubject相対Launch速度の上限[m/s]")]
    [Min(0.1f)]
    [SerializeField] private float maximumPostLimitRelativeLaunchSpeed = 20f;

    [Tooltip("最後の実階段接触法線を2nd POPへ使える時間[s]。期限切れ時はSlope法線へFallback")]
    [Min(0f)]
    [SerializeField] private float stairContactMemorySeconds = 0.30f;

    [Header("Limit Continuity")]
    [Tooltip("Pre-Limit到着がTolerance内なら極限で位置/速度を強制同期せず、そのまま2nd POPへ繋ぐ")]
    [SerializeField] private bool preferNaturalLimitContinuity = true;

    [Tooltip("Tolerance外の時だけ研究用Fallbackとして極限でSubjectへ位置/速度をHard Syncする")]
    [SerializeField] private bool hardSyncLimitWhenArrivalOutsideTolerance = true;

    [Tooltip("Settled判定に使う上下速度の上限")]
    [Min(0f)]
    [SerializeField] private float settleVerticalSpeed = 0.25f;

    [Tooltip("Settled判定に使う1 FixedUpdateあたりの速度変化量上限")]
    [Min(0f)]
    [SerializeField] private float settleVelocityChange = 0.30f;

    [Tooltip("Settled条件を連続して満たすFixedUpdate数")]
    [Min(1)]
    [SerializeField] private int settleStableFramesRequired = 5;

    [Header("Unified Pre-Limit Guidance")]
    [Tooltip("FlatからSlopeへ向かうPOP→Apex→ミサイル下降→Limit到着を有効にする。SlopeStick3Dは読み取り専用")]
    [SerializeField] private bool useUnifiedPreLimitGuidance = true;

    [Tooltip("Flat上で次Slope入口までの予想到達時間がこの値以下になったらPOPを開始する[s]")]
    [Min(0.01f)]
    [SerializeField] private float preLimitLaunchLeadSeconds = 0.10f;

    [Tooltip("次Slope予測が取れず実Slopeへ入った場合でも入口からPre-Limit飛翔を開始する")]
    [SerializeField] private bool preLimitAllowSlopeEntryFallbackLaunch = true;

    [Tooltip("Flat→Slope POP直後の上向き速度[m/s]。加算量ではなくY速度そのもの")]
    [Min(0f)]
    [SerializeField] private float preLimitLaunchUpSpeed = 3f;

    [Tooltip("Slope Progress速度がまだ取れない間、Slope入口→Limitに必要と仮定する時間[s]")]
    [Min(0.05f)]
    [SerializeField] private float preLimitFallbackEntryToLimitSeconds = 0.42f;

    [Tooltip("Progress速度をTime Taskへ採用する最小値[%/s]")]
    [Min(0.1f)]
    [SerializeField] private float preLimitMinimumProgressRatePercentPerSecond = 3f;

    [Tooltip("Progress速度の平滑化強度。大きいほど最新値へ早く追従する")]
    [Min(0.1f)]
    [SerializeField] private float preLimitProgressRateSmoothing = 10f;

    [Tooltip("Subjectの予想到着時刻そのものを平滑化する強度。Tgoではなくdeadlineを平滑化する")]
    [Min(0.1f)]
    [SerializeField] private float preLimitDeadlineSmoothing = 12f;

    [Tooltip("Time-To-Goの最小値[s]")]
    [Min(0.01f)]
    [SerializeField] private float preLimitMinimumTimeToGo = 0.04f;

    [Tooltip("Time-To-Goの最大値[s]。通常区間の基準上限。長いSlopeでは下のAbsolute上限まで自動延長する")]
    [Min(0.05f)]
    [SerializeField] private float preLimitMaximumTimeToGo = 1.20f;

    [Tooltip("Slopeが5倍など長い場合にTime Taskを延長できる絶対上限[s]")]
    [Min(0.2f)]
    [SerializeField] private float preLimitAbsoluteMaximumTimeToGo = 8f;

    [Tooltip("SubjectのProgress変化から推定する『1%あたり何m進むか』の平滑化強度")]
    [Min(0.1f)]
    [SerializeField] private float preLimitProgressDistanceScaleSmoothing = 10f;

    [Tooltip("-0(=Target Progress)の推定ワールド位置を平滑化する強度")]
    [Min(0.1f)]
    [SerializeField] private float preLimitZeroTargetSmoothing = 12f;

    [Tooltip("残りProgressがこの値[%]以下なら、-0推定位置を遅延なく最新値へSnapして着地点の遅れをなくす")]
    [Min(0.01f)]
    [SerializeField] private float preLimitZeroTargetSnapPercent = 2f;

    [Tooltip("上昇中にSubjectへ寄せる平面位置ゲイン")]
    [Min(0f)]
    [SerializeField] private float preLimitAscentPositionGain = 4f;

    [Tooltip("上昇中にSubjectへ寄せる平面速度ゲイン")]
    [Min(0f)]
    [SerializeField] private float preLimitAscentVelocityGain = 2f;

    [Tooltip("上昇弧を壊さないための最大人工加速度[m/s^2]")]
    [Min(0f)]
    [SerializeField] private float preLimitAscentMaximumAcceleration = 20f;

    [Tooltip("上昇中の最大Jerk[m/s^3]")]
    [Min(0f)]
    [SerializeField] private float preLimitAscentMaximumJerk = 220f;

    [Tooltip("上向き速度がこの値以下ならApex通過とみなす[m/s]")]
    [Min(0f)]
    [SerializeField] private float preLimitApexVerticalSpeedThreshold = 0.10f;

    [Tooltip("Apex検出を待てる最大時間[s]")]
    [Min(0.05f)]
    [SerializeField] private float preLimitMaximumAscentSeconds = 0.60f;

    [Tooltip("残り時間がこの値以下ならApex前でもミサイル下降型Time Taskへ切り替える[s]")]
    [Min(0.05f)]
    [SerializeField] private float preLimitForceMissileTimeToGo = 0.24f;

    [Tooltip("Apex後の3Dミサイル下降型終端誘導の最大人工加速度[m/s^2]")]
    [Min(0f)]
    [SerializeField] private float preLimitMissileMaximumAcceleration = 100f;

    [Tooltip("Apex後のミサイル下降型終端誘導の最大Jerk[m/s^3]")]
    [Min(0f)]
    [SerializeField] private float preLimitMissileMaximumJerk = 800f;

    [Tooltip("ONならApex後は上下を含む3D誘導。OFFなら水平面のみ")]
    [SerializeField] private bool preLimitMissileUse3D = true;

    [Tooltip("Limit前の飛翔中はColliderをTriggerにして、階段衝突で放物線Time Taskを崩さない")]
    [SerializeField] private bool preLimitUseTriggerDuringFlight = true;

    [Tooltip("Limit同期直前の位置誤差がこの値以下なら物理誘導到着成功とみなす[m]")]
    [Min(0.001f)]
    [SerializeField] private float preLimitArrivalPositionTolerance = 0.20f;

    [Tooltip("Limit同期直前の速度誤差がこの値以下なら物理誘導到着成功とみなす[m/s]")]
    [Min(0.001f)]
    [SerializeField] private float preLimitArrivalVelocityTolerance = 1.0f;

    [Header("Unified Post-Limit Guidance")]
    [Tooltip("Limit同期→RePOP→上昇→Apex→追尾下降→Subject Flat→Terminal Rejoinを一本の状態機械で扱う")]
    [SerializeField] private bool useUnifiedPostLimitGuidance = true;

    [Tooltip("RePOP上昇中の平面方向位置ゲイン")]
    [Min(0f)]
    [SerializeField] private float postLimitAscentPositionGain = 5f;

    [Tooltip("RePOP上昇中の平面方向速度ゲイン")]
    [Min(0f)]
    [SerializeField] private float postLimitAscentVelocityGain = 2f;

    [Tooltip("RePOP上昇中の最大人工加速度[m/s^2]")]
    [Min(0f)]
    [SerializeField] private float postLimitAscentMaximumAcceleration = 24f;

    [Tooltip("RePOP上昇中の最大Jerk[m/s^3]")]
    [Min(0f)]
    [SerializeField] private float postLimitAscentMaximumJerk = 240f;

    [Tooltip("上向き速度がこの値以下になったらApex通過とみなす[m/s]")]
    [Min(0f)]
    [SerializeField] private float postLimitApexVerticalSpeedThreshold = 0.10f;

    [Tooltip("Apex検出を待つ最大時間[s]")]
    [Min(0.05f)]
    [SerializeField] private float postLimitMaximumAscentSeconds = 0.60f;

    [Tooltip("Apex後の3D追尾位置ゲイン")]
    [Min(0f)]
    [SerializeField] private float postLimitChasePositionGain = 7f;

    [Tooltip("Apex後の3D追尾速度ゲイン")]
    [Min(0f)]
    [SerializeField] private float postLimitChaseVelocityGain = 3f;

    [Tooltip("Apex後の最大人工加速度[m/s^2]")]
    [Min(0f)]
    [SerializeField] private float postLimitChaseMaximumAcceleration = 48f;

    [Tooltip("Apex後の最大Jerk[m/s^3]")]
    [Min(0f)]
    [SerializeField] private float postLimitChaseMaximumJerk = 420f;

    [Tooltip("Apex後に何秒先のSubjectをFuture Shadowとして狙うか[s]")]
    [Min(0f)]
    [SerializeField] private float postLimitChaseLeadSeconds = 0.18f;

    [Tooltip("Subject側がFlatへ到達したらFinal Terminalを開始する")]
    [SerializeField] private bool terminalStartFromSubjectFlat = true;

    [Tooltip("Subject Flatを待てる最大時間[s]。分類欠落時のフォールバック")]
    [Min(0.2f)]
    [SerializeField] private float postLimitMaximumWaitForSubjectFlatSeconds = 2.50f;

    [Header("Terminal Rejoin - Time Optimal Guidance")]
    [Tooltip("Final Terminalで上下を含む3D誘導を使う")]
    [SerializeField] private bool unifiedTerminalUse3D = true;

    [Tooltip("物理的な同期成功判定はBallVisualがFlatへ接触した後だけ許可する")]
    [SerializeField] private bool requireBallFlatContactBeforePhysicalSync = true;

    [Tooltip("Terminal開始から同期までの時間予算[s]")]
    [Min(0.05f)]
    [SerializeField] private float terminalTimeBudget = 0.30f;

    [Tooltip("Time-To-Goの最小値[s]")]
    [Min(0.01f)]
    [SerializeField] private float terminalMinimumTimeToGo = 0.04f;

    [Tooltip("Terminal誘導の最大人工加速度[m/s^2]")]
    [Min(0f)]
    [SerializeField] private float maximumTerminalAcceleration = 80f;

    [Tooltip("Terminal誘導の最大Jerk[m/s^3]")]
    [Min(0f)]
    [SerializeField] private float maximumTerminalJerk = 600f;

    [Tooltip("同期候補とする位置誤差[m]")]
    [Min(0.001f)]
    [SerializeField] private float terminalPositionTolerance = 0.05f;

    [Tooltip("同期候補とする速度誤差[m/s]")]
    [Min(0.001f)]
    [SerializeField] private float terminalVelocityTolerance = 0.20f;

    [Tooltip("同期条件を連続して満たすFixedUpdate数")]
    [Min(1)]
    [SerializeField] private int terminalStableFramesRequired = 2;

    [Tooltip("時間予算切れ時に最後だけSubjectへ強制同期する")]
    [SerializeField] private bool forceTerminalSyncAtDeadline = true;

    [Header("Limit Bounce Experiment Runtime")]
    [SerializeField] private LimitExperimentPhase experimentPhase = LimitExperimentPhase.Waiting;
    [SerializeField] private float experimentLimitTime = -1f;
    [SerializeField] private float experimentBounceAppliedTime = -1f;
    [SerializeField] private float experimentFlatContactTime = -1f;
    [SerializeField] private float experimentSettledTime = -1f;
    [SerializeField] private float experimentLimitToFlatSeconds = -1f;
    [SerializeField] private float experimentLimitToSettledSeconds = -1f;
    [SerializeField] private float experimentEnergyBeforeBounce;
    [SerializeField] private float experimentPlannedEnergyAfterBounce;
    [SerializeField] private float experimentEnergyAtFlatContact;
    [SerializeField] private float experimentFlatImpactImpulse;
    [SerializeField] private Vector3 experimentLimitBallPosition;
    [SerializeField] private Vector3 experimentLimitBallVelocity;
    [SerializeField] private Vector3 experimentLimitReferencePosition;
    [SerializeField] private Vector3 experimentLimitReferenceVelocity;

    [Header("Energy + Contact Bounce Runtime")]
    [SerializeField] private bool limitHardSyncApplied;
    [SerializeField] private Vector3 limitIncomingRelativePosition;
    [SerializeField] private Vector3 limitIncomingRelativeVelocity;
    [SerializeField] private bool hasStairContactSample;
    [SerializeField] private float lastStairContactTime = -1f;
    [SerializeField] private Vector3 lastStairContactPoint;
    [SerializeField] private Vector3 lastStairContactNormal = Vector3.up;
    [SerializeField] private Vector3 lastStairContactRelativeVelocity;
    [SerializeField] private Vector3 resolvedLimitBounceDirection = Vector3.up;
    [SerializeField] private float resolvedLimitBounceRelativeSpeed;
    [SerializeField] private float experimentRelativeEnergyBeforeBounce;
    [SerializeField] private float experimentRelativeEnergyPlannedAfterBounce;

    [Header("Unified POP Algebra Runtime")]
    [SerializeField] private float resolvedFirstPopTargetHeight;
    [SerializeField] private float resolvedSecondPopTargetHeight;
    [SerializeField] private float subjectVelocitySquaredEnergyScale;
    [SerializeField] private float subjectReferenceSpeedForPop;
    [SerializeField] private float resolvedSubjectRelativeGravity;
    [SerializeField] private Vector3 estimatedMappedSubjectAcceleration;

    [Header("Interpolated Limit Boundary Runtime")]
    [SerializeField] private float limitCrossingAlpha;
    [SerializeField] private Vector3 limitCrossingBallPosition;
    [SerializeField] private Vector3 limitCrossingBallVelocity;
    [SerializeField] private Vector3 limitCrossingSubjectPosition;
    [SerializeField] private Vector3 limitCrossingSubjectVelocity;
    [SerializeField] private Vector3 limitCrossingSurfaceNormal = Vector3.up;
    [SerializeField] private bool limitCrossingSurfaceNormalValid;

    [Header("Unified Pre-Limit Runtime")]
    [SerializeField] private float preLimitGuidanceStartTime = -1f;
    [SerializeField] private float preLimitApexTime = -1f;
    [SerializeField] private float preLimitElapsed;
    [SerializeField] private float preLimitTimeToGo;
    [SerializeField] private float preLimitPredictedDeadlineTime = -1f;
    [SerializeField] private float preLimitSmoothedProgressRate;
    [SerializeField] private float preLimitLastProgressError;
    [SerializeField] private float preLimitLastProgressSampleTime = -1f;
    [SerializeField] private bool preLimitHasProgressSample;
    [SerializeField] private bool preLimitHadPositiveUpSpeed;
    [SerializeField] private float preLimitUpSpeed;
    [SerializeField] private Vector3 preLimitAscentAccelerationState;
    [SerializeField] private Vector3 preLimitMissileAccelerationState;
    [SerializeField] private float preLimitPeakAscentAcceleration;
    [SerializeField] private float preLimitPeakMissileAcceleration;
    [SerializeField] private Vector3 preLimitTargetPosition;
    [SerializeField] private Vector3 preLimitTargetVelocity;

    [Header("Unified Pre-Limit -0 Target Runtime")]
    [SerializeField] private float preLimitMetersPerProgressPercent;
    [SerializeField] private float preLimitEstimatedSlopeLength;
    [SerializeField] private float preLimitEstimatedRemainingDistance;
    [SerializeField] private Vector3 preLimitZeroTargetPosition;
    [SerializeField] private bool preLimitZeroTargetValid;
    [SerializeField] private Vector3 preLimitLastMappedSubjectPosition;
    [SerializeField] private bool preLimitHasMappedSubjectPositionSample;

    [SerializeField] private float preLimitArrivalPositionError = -1f;
    [SerializeField] private float preLimitArrivalVelocityError = -1f;
    [SerializeField] private bool preLimitArrivalWithinTolerance;

    [Header("Unified Post-Limit Runtime")]
    [SerializeField] private float experimentEstimatedLimitCrossingTime = -1f;
    [SerializeField] private float limitPreSyncPositionError = -1f;
    [SerializeField] private float limitPostSyncPositionError = -1f;
    [SerializeField] private float postLimitGuidanceStartTime = -1f;
    [SerializeField] private float postLimitApexTime = -1f;
    [SerializeField] private float postLimitSubjectFlatTime = -1f;
    [SerializeField] private float postLimitElapsed;
    [SerializeField] private float postLimitUpSpeed;
    [SerializeField] private bool postLimitHadPositiveUpSpeed;
    [SerializeField] private bool postLimitSubjectFlatCaptured;
    [SerializeField] private bool postLimitBallFlatCaptured;
    [SerializeField] private Vector3 postLimitAscentAccelerationState;
    [SerializeField] private Vector3 postLimitChaseAccelerationState;
    [SerializeField] private float postLimitPeakAscentAcceleration;
    [SerializeField] private float postLimitPeakChaseAcceleration;

    [Header("Unified Terminal Runtime")]
    [SerializeField] private float terminalRejoinStartTime = -1f;
    [SerializeField] private float terminalRejoinCompleteTime = -1f;
    [SerializeField] private float terminalRejoinCostSeconds = -1f;
    [SerializeField] private float terminalLimitToSyncSeconds = -1f;
    [SerializeField] private float terminalElapsed;
    [SerializeField] private float terminalTimeToGo;
    [SerializeField] private Vector3 terminalTargetPosition;
    [SerializeField] private Vector3 terminalTargetVelocity;
    [SerializeField] private Vector3 terminalAccelerationState;
    [SerializeField] private Vector3 previousTerminalAccelerationState;
    [SerializeField] private float terminalPositionError = -1f;
    [SerializeField] private float terminalVelocityError = -1f;
    [SerializeField] private float terminalPeakAcceleration;
    [SerializeField] private float terminalPeakJerk;
    [SerializeField] private int terminalStableFrames;
    [SerializeField] private bool terminalForcedSyncAtDeadline;

    private bool hasPreviousProgressSample;
    private float previousProgressError;
    private float previousProgressTime;
    private Vector3 previousProgressBallPosition;
    private Vector3 previousProgressBallVelocity;
    private Vector3 previousProgressSubjectPosition;
    private Vector3 previousProgressSubjectVelocity;
    private Vector3 previousProgressSurfaceNormal = Vector3.up;
    private bool hasPreviousProgressBoundarySample;

    private Vector3 previousMappedSubjectVelocity;
    private bool hasMappedSubjectVelocitySample;

    // 1st POP ballistic plan. These are runtime states, not tuning parameters.
    private bool preLimitBallisticPlanValid;
    private Vector3 preLimitBallisticStartPosition;
    private Vector3 preLimitBallisticLaunchVelocity;
    private Vector3 preLimitBallisticImpactVelocity;
    private Vector3 preLimitLockedTargetPosition;
    private float preLimitBallisticFlightSeconds;

    private Vector3 previousExperimentVelocity;
    private int experimentSettledFrames;

    private void Awake()
    {
        ballBody = GetComponent<Rigidbody>();
        ballCollider = GetComponent<SphereCollider>();
    }

    private void Start()
    {
        rb = transform.GetComponent<Rigidbody>();
       // rb.useGravity = false;
        respondSubject = GameObject.Find("subject").transform.GetComponent<CorrespondSubject>();
        if (slopeStick == null)
        {
            GameObject subjectObject = GameObject.Find("/PlayerRoot/subject");

            if (subjectObject != null)
            {
                slopeStick = subjectObject.GetComponent<SlopeStick3D>();
                Debug.Log("");
            }
        }

        if (subjectBody == null && slopeStick != null)
            subjectBody = slopeStick.GetComponent<Rigidbody>();

        if (subjectBody == null || slopeStick == null || ballBody == null)
        {
            Debug.LogError(
                "[BALL VISUAL] 必要な参照が設定されていません。",
                this
            );

            enabled = false;
            return;
        }

        LockToSubjectImmediate(true, "InitialLock");
    }

    private void FixedUpdate()
    {
        fixedFrameCounter++;

        bool isFlat = slopeStick.BallVisualIsOnFlat;
        bool isOnSlope = slopeStick.BallVisualIsOnSlope;

        UpdateControlBasis();
        UpdateSubjectRelativeKinematics();

        if (enableLimitBounceExperiment)
        {
            FixedUpdateLimitBounceExperiment(isFlat, isOnSlope);
            return;
        }

        FixedUpdateLegacy(isFlat, isOnSlope);
    }

    // 仕様改訂前の挙動をA/B比較できるように残す。
    private void FixedUpdateLegacy(bool isFlat, bool isOnSlope)
    {
        if (slopeStick.groundKind == GroundKind.Flat)
        {
            Vector3 subjectVelocity =
                respondSubject.Mappedvelocity;

            Vector3 ballVelocity =
                rb.velocity;

            Vector3 subjectDirection =
                Vector3.ProjectOnPlane(subjectVelocity, Vector3.up);

            Vector3 ballDirection =
                Vector3.ProjectOnPlane(ballVelocity, Vector3.up);

            if (subjectDirection != ballDirection)
                Debug.Log("");

            if (ballCollider != null)
                ballCollider.isTrigger = true;
        }

        if (isFlat)
        {
            ProcessFlat();

            wasOnSlope = false;
            WriteDebugLog(isFlat, isOnSlope);

            rb.position = respondSubject.MappedPosition;
            rb.rotation = respondSubject.MappedRotation;
            return;
        }

        if (totalTime > sumTime + stamp || sumTime == 0)
            sumTime = totalTime;

        if (isOnSlope)
        {
            ProcessSlope();

            if (slopeStick.slopeProgressErrorPercent <= 0)
                Debug.Log("");

            rb.position = respondSubject.MappedPosition;
            rb.rotation = respondSubject.MappedRotation;

            wasOnSlope = true;
            WriteDebugLog(isFlat, isOnSlope);
            return;
        }

        ProcessReleased();

        wasOnSlope = false;
        WriteDebugLog(isFlat, isOnSlope);
    }

    private void FixedUpdateLimitBounceExperiment(bool isFlat, bool isOnSlope)
    {
        // -------------------------------------------------------------
        // Unified Pre-Limit Guidance
        // SlopeStick3Dは観測のみ。Subject/InSubjectへは一切書き込まない。
        // -------------------------------------------------------------
        // 1st POPは極限点を実座標で取得できるまで撃たない。
        // Flat上のForward予測だけでは60%点そのものを保証できないため、
        // Active SlopeFrameが得られた最初の斜面FixedUpdateで開始する。
        if (useUnifiedPreLimitGuidance &&
            experimentPhase == LimitExperimentPhase.Waiting &&
            isOnSlope &&
            slopeStick.BallVisualHasActiveSlopeFrame)
        {
            BeginLimitBounceExperiment();
        }

        if (useUnifiedPreLimitGuidance &&
            (experimentPhase == LimitExperimentPhase.PreLimitAscent ||
             experimentPhase == LimitExperimentPhase.PreLimitChase))
        {
            // Subject側のSlope Progressが実際に観測可能になった後だけ0跨ぎを見る。
            if (isOnSlope || slopeStick.ReadyForLimitCapture)
                ObserveLimitCrossing();

            // ObserveLimitCrossing()がこのFixedUpdateでLimitへ遷移させた場合は
            // Pre-Limit Forceを重ねず、そのまま既存Post-Limitへ渡す。
            if (experimentPhase == LimitExperimentPhase.PostLimitAscent)
            {
                EnsureExperimentColliderIsSolid();
                ProcessUnifiedPostLimitAscent();
                WriteDebugLog(isFlat, isOnSlope);
                return;
            }

            if (experimentPhase == LimitExperimentPhase.Ballistic)
            {
                EnsureExperimentColliderIsSolid();
                ProcessBallisticExperiment(isFlat);
                WriteDebugLog(isFlat, isOnSlope);
                return;
            }

            UpdateUnifiedPreLimitTimeTask(isFlat, isOnSlope);

            if (experimentPhase == LimitExperimentPhase.PreLimitAscent)
                ProcessUnifiedPreLimitAscent();
            else if (experimentPhase == LimitExperimentPhase.PreLimitChase)
                ProcessUnifiedPreLimitChase();

            WriteDebugLog(isFlat, isOnSlope);
            return;
        }

        // Unified Post-Limit GuidanceはLimit後を一本の状態機械として扱う。
        if (useUnifiedPostLimitGuidance)
        {
            if (experimentPhase == LimitExperimentPhase.PostLimitAscent)
            {
                EnsureExperimentColliderIsSolid();
                ProcessUnifiedPostLimitAscent();
                WriteDebugLog(isFlat, isOnSlope);
                return;
            }

            if (experimentPhase == LimitExperimentPhase.PostLimitChase)
            {
                EnsureExperimentColliderIsSolid();

                // Subject側がFlatへ入った瞬間をFinal Terminalの基準にする。
                if (ShouldStartUnifiedTerminal(isFlat))
                    BeginUnifiedTerminalRejoin();

                if (experimentPhase == LimitExperimentPhase.PostLimitChase)
                    ProcessUnifiedPostLimitChase();

                WriteDebugLog(isFlat, isOnSlope);
                return;
            }

            if (experimentPhase == LimitExperimentPhase.TerminalRejoin)
            {
                EnsureExperimentColliderIsSolid();
                ProcessUnifiedTerminalRejoin();
                WriteDebugLog(isFlat, isOnSlope);
                return;
            }

            if (experimentPhase == LimitExperimentPhase.Settled)
            {
                // 次のSlopeに入るまではSubjectと同じVisual座標を保持する。
                if (!isOnSlope)
                {
                    HoldExperimentSettledWithSubject();
                    wasOnSlope = false;
                    WriteDebugLog(isFlat, isOnSlope);
                    return;
                }

                // 新しいSlopeへ入った時だけ次の実験を開始可能にする。
                experimentPhase = LimitExperimentPhase.Waiting;
                wasOnSlope = false;
            }
        }
        else
        {
            // 旧A/B比較経路。
            if (experimentPhase == LimitExperimentPhase.Ballistic ||
                experimentPhase == LimitExperimentPhase.FlatContact)
            {
                EnsureExperimentColliderIsSolid();
                ProcessBallisticExperiment(isFlat);
                ObserveSettledState(isFlat);
                WriteDebugLog(isFlat, isOnSlope);
                return;
            }
        }

        // Subject側Progressの0跨ぎを、Slope判定とは独立して観測する。
        // 短いAirが混じってもLimit検知を継続できる。
        if (experimentPhase == LimitExperimentPhase.ApproachingLimit)
        {
            ObserveLimitCrossing();

            if (useUnifiedPostLimitGuidance &&
                experimentPhase == LimitExperimentPhase.PostLimitAscent)
            {
                EnsureExperimentColliderIsSolid();
                ProcessUnifiedPostLimitAscent();
                WriteDebugLog(isFlat, isOnSlope);
                return;
            }

            if (!useUnifiedPostLimitGuidance &&
                experimentPhase == LimitExperimentPhase.Ballistic)
            {
                EnsureExperimentColliderIsSolid();
                ProcessBallisticExperiment(isFlat);
                WriteDebugLog(isFlat, isOnSlope);
                return;
            }
        }

        if (isFlat)
        {
            // 実験開始前のFlatは従来どおりSubjectへ同期させる。
            ProcessFlat();

            wasOnSlope = false;
            rb.position = respondSubject.MappedPosition;
            rb.rotation = respondSubject.MappedRotation;

            if (ballCollider != null)
                ballCollider.isTrigger = true;

            WriteDebugLog(isFlat, isOnSlope);
            return;
        }

        if (totalTime > sumTime + stamp || sumTime == 0)
            sumTime = totalTime;

        if (isOnSlope)
        {
            EnsureExperimentColliderIsSolid();
            ProcessSlope();

            // Limit前はBallVisual自身の位置を保持し、
            // 回転だけSubjectのVisual座標へ合わせる。
            if (experimentPhase == LimitExperimentPhase.ApproachingLimit)
                rb.rotation = respondSubject.MappedRotation;

            wasOnSlope = true;
            WriteDebugLog(isFlat, isOnSlope);
            return;
        }

        // Limit前に短いAirが入った場合は実験状態を保持したまま自由運動させる。
        ProcessReleased();
        wasOnSlope = false;
        WriteDebugLog(isFlat, isOnSlope);
    }

    void Update()
    {
        totalTime += Time.deltaTime;
    }

    private void ProcessFlat()
    {
        if (!lockOnFlat)
        {
            EnsureDynamicFromLockedState();
            ResetRejoinRuntime();
            rejoinPending = false;

            SetPhase(
                VisualPhase.Waiting,
                "FlatLockDisabled"
            );

            return;
        }

       if (rejoinPending)
        {
            ProcessRejoin();
           return;
        }

        if (phase != VisualPhase.LockedToSubject || !ballBody.isKinematic)
            LockToSubjectImmediate(false, "FlatImmediateLock");

        ProcessLockedFlat();
    }

    private void ProcessSlope()
    {
        if (!wasOnSlope)
        {
            if (ballBody.isKinematic)
                ReleaseFromSubject();

            ResetRejoinRuntime();
            BeginSlopeSession();

            // 短いAirからSlopeへ戻っただけなら実験をリセットしない。
            if (enableLimitBounceExperiment &&
                (experimentPhase == LimitExperimentPhase.Waiting ||
                 experimentPhase == LimitExperimentPhase.Settled))
            {
                if (!useUnifiedPreLimitGuidance)
                {
                    BeginLimitBounceExperiment();
                }
            }
        }

        // Pre-Limit飛翔中は専用Time Taskだけを使う。
        // 旧Slope Stick/接線Controlを重ねると放物線とdeadlineが崩れる。
        if (useUnifiedPreLimitGuidance &&
            (experimentPhase == LimitExperimentPhase.PreLimitAscent ||
             experimentPhase == LimitExperimentPhase.PreLimitChase))
        {
            SetPhase(
                VisualPhase.SlopeControlled,
                "UnifiedPreLimitGuidance"
            );
            return;
        }

        ApplySubjectControlField();
        ApplyRollingTorque();

        // 初期研究版では人工StepPulseを混ぜない。
        if (!enableLimitBounceExperiment && useProceduralStepPulse)
            ApplyProceduralStepPulse();

        SetPhase(
            VisualPhase.SlopeControlled,
            "SlopeControlField"
        );
    }

    private void ProcessReleased()
    {
        if (ballBody.isKinematic)
        {
            EnsureDynamicFromLockedState();

            // 一度でも斜面区間を通過している場合は、次にFlatへ戻った時に
            // いきなり固定せずNatural Rejoinへ入る。
            if (hasEnteredSlopeAtLeastOnce)
                rejoinPending = true;
        }

        UpdateBallStickState(0f);

        debugAppliedLinearAcceleration = Vector3.zero;
        debugAppliedStick = ballStickState;
        debugAppliedTangentialAcceleration = 0f;
        debugReleaseWeight = 0f;

        // 空中では現在の回転慣性をそのまま残す。
        debugTargetAngularVelocity = ballBody.angularVelocity;
        debugAppliedAngularAcceleration = Vector3.zero;
        debugRollingWeight = 0f;
        debugRejoinPositionWeight = 0f;

        SetPhase(
            VisualPhase.Released,
            "NoFlatOrSlopeSupport"
        );
    }
    private void ApplyPlanarVelocityMatch()
    {
        Vector3 targetVelocity =
            respondSubject.mappedvelocity;

        Vector3 velocityError =
            targetVelocity - ballBody.velocity;

        Vector3 planarVelocityError =
            Vector3.ProjectOnPlane(
                velocityError,
                Vector3.up
            );

        float response =
            Mathf.Max(rejoinResponseSeconds, 0.02f);

        Vector3 lateralAcceleration =
            planarVelocityError *
            (2f / response);

        lateralAcceleration =
            Vector3.ClampMagnitude(
                lateralAcceleration,
                maximumRejoinAcceleration
            );

        ballBody.AddForce(
            lateralAcceleration,
            ForceMode.Acceleration
        );
    }
    private void ProcessRejoin()
    {
        if (phase != VisualPhase.Rejoining)
            BeginRejoin();

        float dt = Mathf.Max(Time.fixedDeltaTime, 0.000001f);
        rejoinElapsed += dt;

        Vector3 positionError = subjectBody.position - ballBody.position;
        Vector3 velocityError = subjectBody.velocity - ballBody.velocity;

        Vector3 planarPositionError =
            Vector3.ProjectOnPlane(positionError, Vector3.up);

        Vector3 verticalPositionError =
            Vector3.Project(positionError, Vector3.up) *
            rejoinVerticalWeight;

        Vector3 weightedPositionError =
            planarPositionError + verticalPositionError;

        Vector3 planarVelocityError =
            Vector3.ProjectOnPlane(velocityError, Vector3.up);

        Vector3 verticalVelocityError =
            Vector3.Project(velocityError, Vector3.up) *
            rejoinVerticalWeight;

        Vector3 weightedVelocityError =
            planarVelocityError + verticalVelocityError;

        float activeSeconds =
            Mathf.Max(0f, rejoinElapsed - rejoinFreeSeconds);

        debugRejoinPositionWeight = Mathf.SmoothStep(
            0f,
            1f,
            Mathf.Clamp01(
                activeSeconds /
                Mathf.Max(rejoinPositionBlendSeconds, 0.001f)
            )
        );

       
        float omega =
            2f /
            Mathf.Max(rejoinResponseSeconds, 0.02f);

        Vector3 desiredAcceleration =
            weightedPositionError *
            omega *
            omega *
            debugRejoinPositionWeight +
            weightedVelocityError *
            2f *
            omega;

        desiredAcceleration = Vector3.ClampMagnitude(
            desiredAcceleration,
            maximumRejoinAcceleration
        );

        rejoinAccelerationState = Vector3.MoveTowards(
            rejoinAccelerationState,
            desiredAcceleration,
            rejoinJerkLimit * dt
        );

        if (rejoinElapsed > rejoinFreeSeconds)
        {
           ballBody.AddForce(
                rejoinAccelerationState,
                ForceMode.Acceleration
            );
        }

        ApplyRejoinRollingTorque();

        debugAppliedLinearAcceleration =
            rejoinElapsed > rejoinFreeSeconds
                ? rejoinAccelerationState
                : Vector3.zero;

        debugAppliedStick = 0f;
        debugAppliedTangentialAcceleration = 0f;
        debugReleaseWeight = 0f;

        bool positionReady =
            positionError.magnitude <= rejoinLockDistance;

        bool velocityReady =
            velocityError.magnitude <= rejoinLockVelocityError;

        if (positionReady && velocityReady &&
            rejoinElapsed >= rejoinFreeSeconds)
        {
            rejoinStableFrames++;
        }
        else
        {
            rejoinStableFrames = 0;
        }

        if (rejoinStableFrames >=
            Mathf.Max(1, rejoinStableFramesRequired))
        {
            CompleteRejoin();
        }
    }

    private void BeginRejoin()
    {
        if (ballBody.isKinematic)
            EnsureDynamicFromLockedState();

        ResetRejoinRuntime();

        SetPhase(
            VisualPhase.Rejoining,
            "FlatNaturalRejoin"
        );
    }

    private void CompleteRejoin()
    {
        Quaternion preservedRotation = ballBody.rotation;

        ballBody.velocity = Vector3.zero;
        ballBody.angularVelocity = Vector3.zero;
        //ballBody.isKinematic = true;

        ballBody.position = subjectBody.position;
        ballBody.rotation = preservedRotation;

        previousLockedSubjectPosition = subjectBody.position;
        lockedRollingAngularVelocity =
            CalculateRollingAngularVelocity(
                subjectBody.velocity,
                Vector3.up
            );

        rejoinPending = false;
        ResetRejoinRuntime();
        ResetSlopeVisualRuntime();

        SetPhase(
            VisualPhase.LockedToSubject,
            "RejoinCompleted"
        );
    }

    private void ProcessLockedFlat()
    {
        if (!ballBody.isKinematic)
        {
            LockToSubjectImmediate(
                false,
                "LockedFlatRecovered"
            );
        }

        Vector3 subjectMovement =
            subjectBody.position - previousLockedSubjectPosition;

        // DelayStartやリセットなどの大きなテレポートでは、
        // その距離を転がり量として扱わない。
        if (subjectMovement.sqrMagnitude > 4f)
        {
            ballBody.position = subjectBody.position;
            previousLockedSubjectPosition = subjectBody.position;
            lockedRollingAngularVelocity = Vector3.zero;
        }
        else
        {
            ballBody.position = subjectBody.position;

            ApplyLockedRolling(
                subjectMovement,
                Vector3.up
            );

            previousLockedSubjectPosition = subjectBody.position;
        }

        debugAppliedLinearAcceleration = Vector3.zero;
        debugAppliedAngularAcceleration = Vector3.zero;
        debugTargetAngularVelocity = lockedRollingAngularVelocity;
        debugAppliedStick = 0f;
        debugAppliedTangentialAcceleration = 0f;
        debugReleaseWeight = 0f;
        debugRollingWeight = 1f;
        debugRejoinPositionWeight = 0f;
        rejoinStableFrames = 0;
    }

    private void LockToSubjectImmediate(
        bool matchSubjectRotation,
        string reason
    )
    {
        if (!ballBody.isKinematic)
        {
            ballBody.velocity = Vector3.zero;
            ballBody.angularVelocity = Vector3.zero;
           // ballBody.isKinematic = true;
        }

        ballBody.position = subjectBody.position;

        if (matchSubjectRotation)
            ballBody.rotation = subjectBody.rotation;

        previousLockedSubjectPosition = subjectBody.position;
        lockedRollingAngularVelocity =
            CalculateRollingAngularVelocity(
                subjectBody.velocity,
                Vector3.up
            );

        ResetSlopeVisualRuntime();
        ResetRejoinRuntime();

        SetPhase(
            VisualPhase.LockedToSubject,
            reason
        );
    }

    private void ReleaseFromSubject()
    {
        // Locked中は既に位置が一致しているため、位置だけを再確認する。
        // 回転姿勢はBallVisual自身の転がり姿勢を保持する。
        ballBody.position = subjectBody.position;
        ballBody.isKinematic = false;

        if (inheritSubjectVelocityOnSlopeEntry)
        {
            ballBody.velocity = subjectBody.velocity;

            ballBody.angularVelocity =
                lockedRollingAngularVelocity.sqrMagnitude > 0.000001f
                    ? lockedRollingAngularVelocity
                    : subjectBody.angularVelocity;
        }
        else
        {
            ballBody.velocity = Vector3.zero;
            ballBody.angularVelocity = Vector3.zero;
        }
    }

    private void EnsureDynamicFromLockedState()
    {
        if (!ballBody.isKinematic)
            return;

        ballBody.isKinematic = false;
        ballBody.velocity = subjectBody.velocity;

        ballBody.angularVelocity =
            lockedRollingAngularVelocity.sqrMagnitude > 0.000001f
                ? lockedRollingAngularVelocity
                : subjectBody.angularVelocity;
    }

    private void BeginSlopeSession()
    {
        accumulatedSlopeDistance = 0f;
        previousStepIndex = 0;

        hasEnteredSlopeAtLeastOnce = true;
        rejoinPending = true;

        SetPhase(
            VisualPhase.SlopeControlled,
            "SlopeEntry"
        );
    }

    private void ResetSlopeVisualRuntime()
    {
        accumulatedSlopeDistance = 0f;
        previousStepIndex = 0;
        ballStickState = 0f;

        debugAppliedLinearAcceleration = Vector3.zero;
        debugAppliedAngularAcceleration = Vector3.zero;
        debugTargetAngularVelocity = Vector3.zero;
        debugAppliedStick = 0f;
        debugAppliedTangentialAcceleration = 0f;
        debugReleaseWeight = 0f;
        debugRollingWeight = 0f;
    }

    private void ResetRejoinRuntime()
    {
        rejoinElapsed = 0f;
        debugRejoinPositionWeight = 0f;
        rejoinAccelerationState = Vector3.zero;
        rejoinStableFrames = 0;
    }

    private void UpdateControlBasis()
    {
        Vector3 normal =
            slopeStick.BallVisualSurfaceNormal;

        Vector3 tangent =
            slopeStick.BallVisualSlopeTangent;

        // PhysicsRoot側の方向をVisualPlayerRoot側へ変換する
        if (respondSubject != null)
        {
            normal = respondSubject.MapDirection(normal);
            tangent = respondSubject.MapDirection(tangent);
        }

        if (normal.sqrMagnitude > 0.000001f)
            currentSurfaceNormal = normal.normalized;

        // 変換後の法線に対して、接線をもう一度面上へ投影する
        tangent = Vector3.ProjectOnPlane(
            tangent,
            currentSurfaceNormal
        );

        if (tangent.sqrMagnitude > 0.000001f)
            currentSlopeTangent = tangent.normalized;
    }

    private void ApplySubjectControlField()
    {
        float subjectAppliedStick = Mathf.Max(
            0f,
            slopeStick.BallVisualAppliedStickAcceleration
        );

        float targetStick =
            subjectAppliedStick * stickScale;

        UpdateBallStickState(targetStick);

        float appliedTangentialAcceleration =
            slopeStick.BallVisualAppliedTangentialAcceleration;

        float releaseWeight = Mathf.Clamp01(
            slopeStick.BallVisualReleaseWeight
        );

        debugAppliedStick = ballStickState;

        debugAppliedTangentialAcceleration =
            appliedTangentialAcceleration *
            tangentialAccelerationScale;

        debugReleaseWeight = releaseWeight;

        Vector3 stickAcceleration =
            -currentSurfaceNormal * ballStickState;

        Vector3 tangentAcceleration =
            currentSlopeTangent *
            debugAppliedTangentialAcceleration;

        Vector3 totalAcceleration =
            stickAcceleration + tangentAcceleration;

        totalAcceleration = Vector3.ClampMagnitude(
            totalAcceleration,
            maximumControlAcceleration
        );

        debugAppliedLinearAcceleration = totalAcceleration;
        
       ballBody.AddForce(
            totalAcceleration,
            ForceMode.Acceleration
        );
    }

    private void UpdateBallStickState(float targetStick)
    {
        float dt = Mathf.Max(
            Time.fixedDeltaTime,
            0.000001f
        );

        ballStickState = Mathf.MoveTowards(
            ballStickState,
            Mathf.Max(0f, targetStick),
            maximumBallStickJerk * dt
        );
    }

    private void ApplyRollingTorque()
    {
        Vector3 tangentVelocity = Vector3.ProjectOnPlane(
            ballBody.velocity,
            currentSurfaceNormal
        );

        if (tangentVelocity.sqrMagnitude < 0.000001f)
        {
            debugTargetAngularVelocity = Vector3.zero;
            debugAppliedAngularAcceleration = Vector3.zero;
            return;
        }

        Vector3 targetAngularVelocity =
            Vector3.Cross(
                currentSurfaceNormal,
                tangentVelocity
            ) /
            Mathf.Max(ballRadius, 0.001f);

        debugTargetAngularVelocity = targetAngularVelocity;

        Vector3 angularVelocityError =
            targetAngularVelocity - ballBody.angularVelocity;

        float stickWeight = Mathf.Clamp01(
            debugAppliedStick /
            Mathf.Max(stickForFullRollingControl, 0.001f)
        );

        float controlledWeight =
            stickWeight * debugReleaseWeight;

        float rollingWeight = Mathf.Lerp(
            minimumRollingControl,
            1f,
            controlledWeight
        );

        debugRollingWeight = rollingWeight;

        Vector3 angularAcceleration =
            angularVelocityError *
            rollingTorqueGain *
            rollingWeight;

        angularAcceleration = Vector3.ClampMagnitude(
            angularAcceleration,
            maximumAngularAcceleration
        );

        debugAppliedAngularAcceleration = angularAcceleration;

        ballBody.AddTorque(
            angularAcceleration,
            ForceMode.Acceleration
        );
    }

    private void ApplyRejoinRollingTorque()
    {
        Vector3 targetAngularVelocity =
            CalculateRollingAngularVelocity(
                ballBody.velocity,
                Vector3.up
            );

        Vector3 angularVelocityError =
            targetAngularVelocity - ballBody.angularVelocity;

        Vector3 angularAcceleration =
            angularVelocityError * rejoinAngularGain;

        angularAcceleration = Vector3.ClampMagnitude(
            angularAcceleration,
            maximumRejoinAngularAcceleration
        );

        ballBody.AddTorque(
            angularAcceleration,
            ForceMode.Acceleration
        );

        debugTargetAngularVelocity = targetAngularVelocity;
        debugAppliedAngularAcceleration = angularAcceleration;
        debugRollingWeight = 1f;
    }

    private Vector3 CalculateRollingAngularVelocity(
        Vector3 worldVelocity,
        Vector3 surfaceNormal
    )
    {
        Vector3 tangentVelocity = Vector3.ProjectOnPlane(
            worldVelocity,
            surfaceNormal
        );

        if (tangentVelocity.sqrMagnitude < 0.000001f)
            return Vector3.zero;

        return Vector3.Cross(
                   surfaceNormal.normalized,
                   tangentVelocity
               ) /
               Mathf.Max(ballRadius, 0.001f);
    }

    private void ApplyLockedRolling(
        Vector3 movement,
        Vector3 surfaceNormal
    )
    {
        Vector3 tangentMovement = Vector3.ProjectOnPlane(
            movement,
            surfaceNormal
        );

        float distance = tangentMovement.magnitude;

        if (distance < 0.000001f)
        {
            lockedRollingAngularVelocity = Vector3.zero;
            return;
        }

        Vector3 rotationAxis = Vector3.Cross(
            surfaceNormal,
            tangentMovement.normalized
        );

        if (rotationAxis.sqrMagnitude < 0.000001f)
        {
            lockedRollingAngularVelocity = Vector3.zero;
            return;
        }

        float angleRadians =
            distance /
            Mathf.Max(ballRadius, 0.001f);

        Quaternion deltaRotation = Quaternion.AngleAxis(
            angleRadians * Mathf.Rad2Deg,
            rotationAxis.normalized
        );

        ballBody.rotation = deltaRotation * ballBody.rotation;

        float dt = Mathf.Max(Time.fixedDeltaTime, 0.000001f);

        lockedRollingAngularVelocity =
            rotationAxis.normalized *
            (angleRadians / dt);
    }

    void OnCollisionEnter(Collision col)
    {
        if (enableLimitBounceExperiment &&
            col.transform.CompareTag("plane"))
        {
            RegisterFlatContact(
                col.impulse.magnitude,
                "OnCollisionEnter"
            );
        }

        if (col.transform.CompareTag("stairway"))
        {
            CaptureStairContact(col, "OnCollisionEnter");
        }

        if (col.transform.CompareTag("plane"))
            Debug.Log("");

        Debug.Log("");
    }

    void OnTriggerEnter(Collider col)
    {
        // TriggerではContactPoint/normalを取得できないため、
        // ここでは階段接触角サンプルとして採用しない。
        Debug.Log("");
    }

    void OnCollisionStay(Collision col)
    {
        if (col.transform.CompareTag("stairway"))
        {
            CaptureStairContact(col, "OnCollisionStay");
        }
    }
    

    private void BeginLimitBounceExperiment()
    {
        experimentPhase = LimitExperimentPhase.ApproachingLimit;

        hasPreviousProgressSample = false;
        previousProgressError = 0f;
        previousProgressTime = Time.fixedTime;
        previousProgressBallPosition = ballBody.position;
        previousProgressBallVelocity = ballBody.velocity;
        previousExperimentVelocity = ballBody.velocity;

        experimentLimitTime = -1f;
        experimentEstimatedLimitCrossingTime = -1f;
        limitPreSyncPositionError = -1f;
        limitPostSyncPositionError = -1f;
        experimentBounceAppliedTime = -1f;
        experimentFlatContactTime = -1f;
        experimentSettledTime = -1f;
        experimentLimitToFlatSeconds = -1f;
        experimentLimitToSettledSeconds = -1f;

        experimentEnergyBeforeBounce = 0f;
        experimentPlannedEnergyAfterBounce = 0f;
        experimentEnergyAtFlatContact = 0f;
        experimentFlatImpactImpulse = 0f;

        experimentLimitBallPosition = Vector3.zero;
        experimentLimitBallVelocity = Vector3.zero;
        experimentLimitReferencePosition = Vector3.zero;
        experimentLimitReferenceVelocity = Vector3.zero;

        limitHardSyncApplied = false;
        limitIncomingRelativePosition = Vector3.zero;
        limitIncomingRelativeVelocity = Vector3.zero;
        resolvedLimitBounceDirection = Vector3.up;
        resolvedLimitBounceRelativeSpeed = 0f;
        experimentRelativeEnergyBeforeBounce = 0f;
        experimentRelativeEnergyPlannedAfterBounce = 0f;

        resolvedFirstPopTargetHeight = 0f;
        resolvedSecondPopTargetHeight = 0f;
        subjectVelocitySquaredEnergyScale = 0f;
        subjectReferenceSpeedForPop = 0f;
        resolvedSubjectRelativeGravity = GetSubjectRelativeGravityMagnitude();

        limitCrossingAlpha = 0f;
        limitCrossingBallPosition = Vector3.zero;
        limitCrossingBallVelocity = Vector3.zero;
        limitCrossingSubjectPosition = Vector3.zero;
        limitCrossingSubjectVelocity = Vector3.zero;
        limitCrossingSurfaceNormal = Vector3.up;
        limitCrossingSurfaceNormalValid = false;
        hasPreviousProgressBoundarySample = false;

        // Contact sampleは直前のFlat/階段境界から引き継げるため、
        // ここでは消さない。stairContactMemorySecondsで鮮度だけ判定する。

        experimentSettledFrames = 0;

        preLimitGuidanceStartTime = -1f;
        preLimitApexTime = -1f;
        preLimitElapsed = 0f;
        preLimitTimeToGo = Mathf.Max(
            preLimitFallbackEntryToLimitSeconds,
            preLimitMinimumTimeToGo
        );
        preLimitPredictedDeadlineTime = -1f;
        preLimitSmoothedProgressRate = 0f;
        preLimitLastProgressError = 0f;
        preLimitLastProgressSampleTime = -1f;
        preLimitHasProgressSample = false;
        preLimitHadPositiveUpSpeed = false;
        preLimitUpSpeed = 0f;
        preLimitAscentAccelerationState = Vector3.zero;
        preLimitMissileAccelerationState = Vector3.zero;
        preLimitPeakAscentAcceleration = 0f;
        preLimitPeakMissileAcceleration = 0f;
        preLimitTargetPosition = Vector3.zero;
        preLimitTargetVelocity = Vector3.zero;

        preLimitMetersPerProgressPercent = 0f;
        preLimitEstimatedSlopeLength = 0f;
        preLimitEstimatedRemainingDistance = 0f;
        preLimitZeroTargetPosition = Vector3.zero;
        preLimitZeroTargetValid = false;
        preLimitLastMappedSubjectPosition = Vector3.zero;
        preLimitHasMappedSubjectPositionSample = false;

        preLimitArrivalPositionError = -1f;
        preLimitArrivalVelocityError = -1f;
        preLimitArrivalWithinTolerance = false;

        preLimitBallisticPlanValid = false;
        preLimitBallisticStartPosition = Vector3.zero;
        preLimitBallisticLaunchVelocity = Vector3.zero;
        preLimitBallisticImpactVelocity = Vector3.zero;
        preLimitLockedTargetPosition = Vector3.zero;
        preLimitBallisticFlightSeconds = 0f;

        postLimitGuidanceStartTime = -1f;
        postLimitApexTime = -1f;
        postLimitSubjectFlatTime = -1f;
        postLimitElapsed = 0f;
        postLimitUpSpeed = 0f;
        postLimitHadPositiveUpSpeed = false;
        postLimitSubjectFlatCaptured = false;
        postLimitBallFlatCaptured = false;
        postLimitAscentAccelerationState = Vector3.zero;
        postLimitChaseAccelerationState = Vector3.zero;
        postLimitPeakAscentAcceleration = 0f;
        postLimitPeakChaseAcceleration = 0f;

        terminalRejoinStartTime = -1f;
        terminalRejoinCompleteTime = -1f;
        terminalRejoinCostSeconds = -1f;
        terminalLimitToSyncSeconds = -1f;
        terminalElapsed = 0f;
        terminalTimeToGo = Mathf.Max(
            terminalTimeBudget,
            terminalMinimumTimeToGo
        );
        terminalTargetPosition = Vector3.zero;
        terminalTargetVelocity = Vector3.zero;
        terminalAccelerationState = Vector3.zero;
        previousTerminalAccelerationState = Vector3.zero;
        terminalPositionError = -1f;
        terminalVelocityError = -1f;
        terminalPeakAcceleration = 0f;
        terminalPeakJerk = 0f;
        terminalStableFrames = 0;
        terminalForcedSyncAtDeadline = false;

        if (useUnifiedPreLimitGuidance)
            BeginUnifiedPreLimitGuidance();
        else
            EnsureExperimentColliderIsSolid();

        Debug.Log(
            $"[LIMIT EXPERIMENT] Begin " +
            $"time={Time.fixedTime:F4} " +
            $"unifiedPreLimit={useUnifiedPreLimitGuidance} " +
            $"unifiedPostLimit={useUnifiedPostLimitGuidance}",
            this
        );
    }

    private bool IsLimitApproachPhase()
    {
        return
            experimentPhase == LimitExperimentPhase.ApproachingLimit ||
            experimentPhase == LimitExperimentPhase.PreLimitAscent ||
            experimentPhase == LimitExperimentPhase.PreLimitChase;
    }

    private bool ShouldLaunchUnifiedPreLimitFromFlat()
    {
        // New rule: 1st POP must aim at the exact Target Progress center.
        // The current SlopeStick3D API exposes that exact point only after
        // the active SlopeFrame exists, so Flat pre-fire is intentionally disabled.
        return false;
    }

    private void BeginUnifiedPreLimitGuidance()
    {
        if (experimentPhase != LimitExperimentPhase.ApproachingLimit)
            return;

        if (!TryResolveExactPreLimitTarget(out Vector3 exactLimitTarget))
        {
            // Exact 60% target is mandatory for 1st POP. Retry from Waiting.
            experimentPhase = LimitExperimentPhase.Waiting;
            return;
        }

        if (ballBody.isKinematic)
            ballBody.isKinematic = false;

        ballBody.useGravity = true;

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = respondSubject.Mappedvelocity;

        // Start exactly from Subject at the first active slope frame.
        // This is deliberately later than the old Flat pre-fire so that the
        // Subject's downward slope velocity is inherited before the POP is made.
        ballBody.position = subjectPosition;

        resolvedFirstPopTargetHeight =
            useSubjectRelativeEnergyPop && useEnergyHeightForFirstPop
                ? ResolvePopTargetHeight(
                    preLimitTargetHeightRelativeToSubject,
                    firstPopVelocitySquaredRatio)
                : Mathf.Max(0f, preLimitTargetHeightRelativeToSubject);

        float firstPopRelativeUpSpeed =
            useSubjectRelativeEnergyPop && useEnergyHeightForFirstPop
                ? CalculateVerticalSpeedForHeight(resolvedFirstPopTargetHeight)
                : Mathf.Max(0f, preLimitLaunchUpSpeed);

        // -------------------------------------------------------------
        // Decomposed 1st POP
        // 1) v^2 controls only relative height -> relative upward speed.
        // 2) Subject velocity supplies the base slope/downward motion.
        // 3) Vertical ballistic equation determines the natural flight time.
        // 4) Horizontal velocity is solved linearly from distance / flight time.
        // No 3D terminal 1/T^2 dive is used.
        // -------------------------------------------------------------
        float launchVerticalSpeed =
            subjectVelocity.y + firstPopRelativeUpSpeed;

        float flightSeconds = SolveBallisticTimeToVerticalTarget(
            subjectPosition.y,
            exactLimitTarget.y,
            launchVerticalSpeed);

        if (!(flightSeconds > 0f) ||
            float.IsNaN(flightSeconds) ||
            float.IsInfinity(flightSeconds))
        {
            flightSeconds = Mathf.Max(
                preLimitFallbackEntryToLimitSeconds,
                preLimitMinimumTimeToGo);
        }

        Vector3 planarDisplacement = Vector3.ProjectOnPlane(
            exactLimitTarget - subjectPosition,
            Vector3.up);

        Vector3 planarLaunchVelocity =
            planarDisplacement / Mathf.Max(flightSeconds, 0.0001f);

        Vector3 launchVelocity =
            planarLaunchVelocity +
            Vector3.up * launchVerticalSpeed;

        Vector3 impactVelocity =
            planarLaunchVelocity +
            Vector3.up *
            (launchVerticalSpeed + Physics.gravity.y * flightSeconds);

        ballBody.velocity = launchVelocity;

        preLimitBallisticPlanValid = true;
        preLimitBallisticStartPosition = subjectPosition;
        preLimitBallisticLaunchVelocity = launchVelocity;
        preLimitBallisticImpactVelocity = impactVelocity;
        preLimitLockedTargetPosition = exactLimitTarget;
        preLimitBallisticFlightSeconds = flightSeconds;

        preLimitTargetPosition = exactLimitTarget;
        preLimitTargetVelocity = impactVelocity;
        preLimitZeroTargetPosition = exactLimitTarget;
        preLimitZeroTargetValid = true;

        Vector3 planarVelocity = Vector3.ProjectOnPlane(
            subjectVelocity,
            Vector3.up);

        Vector3 inheritedAngularVelocity = lockedRollingAngularVelocity;
        if (inheritedAngularVelocity.sqrMagnitude <= 0.000001f)
        {
            inheritedAngularVelocity = CalculateRollingAngularVelocity(
                planarVelocity,
                Vector3.up);
        }

        ballBody.angularVelocity = inheritedAngularVelocity;
        debugTargetAngularVelocity = inheritedAngularVelocity;
        debugAppliedAngularAcceleration = Vector3.zero;
        debugRollingWeight = 0f;

        if (preLimitUseTriggerDuringFlight && ballCollider != null)
            ballCollider.isTrigger = true;

        preLimitGuidanceStartTime = Time.fixedTime;
        preLimitElapsed = 0f;
        preLimitApexTime = -1f;
        preLimitPredictedDeadlineTime =
            preLimitGuidanceStartTime + flightSeconds;
        preLimitTimeToGo = flightSeconds;

        preLimitUpSpeed = firstPopRelativeUpSpeed;
        preLimitHadPositiveUpSpeed =
            preLimitUpSpeed > preLimitApexVerticalSpeedThreshold;

        preLimitAscentAccelerationState = Vector3.zero;
        preLimitMissileAccelerationState = Vector3.zero;

        experimentPhase = LimitExperimentPhase.PreLimitAscent;

        SetPhase(
            VisualPhase.SlopeControlled,
            "UnifiedPreLimitExactLimitPOP");

        if (enableDebugLog)
        {
            float impactPlanarSpeed =
                Vector3.ProjectOnPlane(impactVelocity, Vector3.up).magnitude;
            float impactDownSpeed = Mathf.Max(0f, -impactVelocity.y);
            float impactAngle = Mathf.Atan2(
                impactDownSpeed,
                Mathf.Max(0.0001f, impactPlanarSpeed)) * Mathf.Rad2Deg;

            Debug.Log(
                $"[FIRST POP DECOMPOSED] " +
                $"height={resolvedFirstPopTargetHeight:F4}m " +
                $"relativeUp={firstPopRelativeUpSpeed:F4} " +
                $"subjectVelocity={subjectVelocity:F4} " +
                $"launchVelocity={launchVelocity:F4} " +
                $"flight={flightSeconds:F4}s " +
                $"target={exactLimitTarget:F4} " +
                $"plannedImpactVelocity={impactVelocity:F4} " +
                $"plannedImpactAngle={impactAngle:F3}deg",
                this);
        }
    }

    private void UpdateUnifiedPreLimitTimeTask(
        bool isFlat,
        bool isOnSlope)
    {
        float dt = Mathf.Max(Time.fixedDeltaTime, 0.000001f);
        float now = Time.fixedTime;
        float currentError = slopeStick.slopeProgressErrorPercent;

        bool slopeProgressActive =
            isOnSlope || slopeStick.ReadyForLimitCapture;

        if (slopeProgressActive)
        {
            if (preLimitHasProgressSample)
            {
                float sampleDt = Mathf.Max(
                    0.000001f,
                    now - preLimitLastProgressSampleTime);

                float rawProgressRate =
                    (currentError - preLimitLastProgressError) /
                    sampleDt;

                if (rawProgressRate >=
                        preLimitMinimumProgressRatePercentPerSecond &&
                    rawProgressRate <= 1000f &&
                    !float.IsNaN(rawProgressRate) &&
                    !float.IsInfinity(rawProgressRate))
                {
                    float blend = 1f - Mathf.Exp(
                        -preLimitProgressRateSmoothing * dt);

                    preLimitSmoothedProgressRate =
                        preLimitSmoothedProgressRate > 0f
                            ? Mathf.Lerp(
                                preLimitSmoothedProgressRate,
                                rawProgressRate,
                                blend)
                            : rawProgressRate;
                }
            }

            float sectionLength =
                Mathf.Max(0f, slopeStick.BallVisualSlopeSectionLength);

            if (sectionLength > 0.0001f)
            {
                preLimitEstimatedSlopeLength = sectionLength;
                preLimitMetersPerProgressPercent = sectionLength * 0.01f;
                preLimitEstimatedRemainingDistance =
                    Mathf.Max(0f, -currentError) *
                    preLimitMetersPerProgressPercent;
            }

            preLimitLastProgressError = currentError;
            preLimitLastProgressSampleTime = now;
            preLimitHasProgressSample = true;
        }

        // Once fired, the 1st POP endpoint and deadline never chase Subject.
        // The exact 60% point is frozen in the ballistic plan.
        if (preLimitBallisticPlanValid)
        {
            preLimitTargetPosition = preLimitLockedTargetPosition;
            preLimitZeroTargetPosition = preLimitLockedTargetPosition;
            preLimitZeroTargetValid = true;
            preLimitTargetVelocity = preLimitBallisticImpactVelocity;

            preLimitTimeToGo = Mathf.Max(
                0f,
                preLimitPredictedDeadlineTime - now);
        }
    }

    private void SwitchUnifiedPreLimitToMissile(
        string reason)
    {
        if (experimentPhase !=
            LimitExperimentPhase.PreLimitAscent)
        {
            return;
        }

        preLimitApexTime =
            Time.fixedTime;

        preLimitMissileAccelerationState =
            preLimitAscentAccelerationState;

        experimentPhase =
            LimitExperimentPhase.PreLimitChase;

        Debug.Log(
            $"[UNIFIED PRE LIMIT] BallisticDescentStart " +
            $"reason={reason} " +
            $"time={preLimitApexTime:F4} " +
            $"elapsed={(preLimitApexTime - preLimitGuidanceStartTime):F4}s " +
            $"Tgo={preLimitTimeToGo:F4}s " +
            $"deadline={preLimitPredictedDeadlineTime:F4} " +
            $"upSpeed={preLimitUpSpeed:F4} " +
            $"ballPos={ballBody.position:F4} " +
            $"targetPos={preLimitTargetPosition:F4}",
            this
        );
    }

    private void ProcessUnifiedPreLimitAscent()
    {
        if (experimentPhase != LimitExperimentPhase.PreLimitAscent)
            return;

        preLimitElapsed = Mathf.Max(
            0f,
            Time.fixedTime - preLimitGuidanceStartTime);

        ApplyPreLimitBallisticPlanarCorrection(
            ref preLimitAscentAccelerationState,
            preLimitAscentMaximumAcceleration,
            preLimitAscentMaximumJerk);

        preLimitPeakAscentAcceleration = Mathf.Max(
            preLimitPeakAscentAcceleration,
            preLimitAscentAccelerationState.magnitude);

        Vector3 subjectVelocity = respondSubject.Mappedvelocity;
        preLimitUpSpeed = Vector3.Dot(
            ballBody.velocity - subjectVelocity,
            Vector3.up);

        if (preLimitUpSpeed > preLimitApexVerticalSpeedThreshold)
            preLimitHadPositiveUpSpeed = true;

        bool crossedRelativeApex =
            preLimitHadPositiveUpSpeed &&
            preLimitUpSpeed <= preLimitApexVerticalSpeedThreshold;

        bool ascentTimedOut =
            preLimitElapsed >= preLimitMaximumAscentSeconds;

        if (crossedRelativeApex)
            SwitchUnifiedPreLimitToMissile("RelativeApex");
        else if (ascentTimedOut)
            SwitchUnifiedPreLimitToMissile("AscentTimeout");
    }

    private void ProcessUnifiedPreLimitChase()
    {
        if (experimentPhase != LimitExperimentPhase.PreLimitChase)
            return;

        preLimitElapsed = Mathf.Max(
            0f,
            Time.fixedTime - preLimitGuidanceStartTime);

        // Old implementation used a 3D cubic terminal law whose position term
        // grew as 1/Tgo^2. The new chase keeps the same ballistic vertical
        // motion and corrects only planar numerical drift toward the locked 60% point.
        ApplyPreLimitBallisticPlanarCorrection(
            ref preLimitMissileAccelerationState,
            Mathf.Min(
                preLimitAscentMaximumAcceleration,
                preLimitMissileMaximumAcceleration),
            Mathf.Min(
                preLimitAscentMaximumJerk,
                preLimitMissileMaximumJerk));

        preLimitPeakMissileAcceleration = Mathf.Max(
            preLimitPeakMissileAcceleration,
            preLimitMissileAccelerationState.magnitude);

        if (enableDebugLog &&
            fixedFrameCounter % Mathf.Max(1, logEveryFixedFrames) == 0)
        {
            Vector3 targetError =
                preLimitLockedTargetPosition - ballBody.position;

            Debug.Log(
                $"[UNIFIED PRE LIMIT BALLISTIC] " +
                $"elapsed={preLimitElapsed:F4}s " +
                $"Tgo={preLimitTimeToGo:F4}s " +
                $"progressError={slopeStick.slopeProgressErrorPercent:F3}% " +
                $"targetError={targetError:F4} " +
                $"planarCorrection={preLimitMissileAccelerationState.magnitude:F3} " +
                $"ballPos={ballBody.position:F4} " +
                $"targetPos={preLimitLockedTargetPosition:F4} " +
                $"ballVel={ballBody.velocity:F4} " +
                $"impactVel={preLimitBallisticImpactVelocity:F4}",
                this);
        }
    }

    private bool TryResolveExactPreLimitTarget(out Vector3 targetPosition)
    {
        targetPosition = Vector3.zero;

        if (slopeStick == null ||
            !slopeStick.BallVisualHasActiveSlopeFrame)
        {
            return false;
        }

        Vector3 physicsTarget =
            slopeStick.BallVisualTargetProgressCenterPhysics;

        // CorrespondSubjectの既存MapDirectionだけで点を写す。
        // 物理Subject中心からTargetまでの差分をVisual座標へ回し、
        // MappedPositionへ足すので、新しい座標変換APIは不要。
        if (respondSubject != null && subjectBody != null)
        {
            Vector3 physicsOffset =
                physicsTarget - subjectBody.position;

            targetPosition =
                respondSubject.MappedPosition +
                respondSubject.MapDirection(physicsOffset);
        }
        else
        {
            targetPosition = physicsTarget;
        }

        return
            !float.IsNaN(targetPosition.x) &&
            !float.IsNaN(targetPosition.y) &&
            !float.IsNaN(targetPosition.z) &&
            !float.IsInfinity(targetPosition.x) &&
            !float.IsInfinity(targetPosition.y) &&
            !float.IsInfinity(targetPosition.z);
    }

    private float SolveBallisticTimeToVerticalTarget(
        float startY,
        float targetY,
        float launchVerticalSpeed)
    {
        // targetY = startY + vy*t + 0.5*g*t^2
        float a = 0.5f * Physics.gravity.y;
        float b = launchVerticalSpeed;
        float c = startY - targetY;

        if (Mathf.Abs(a) <= 0.000001f)
        {
            if (Mathf.Abs(b) <= 0.000001f)
                return -1f;

            float linearTime = -c / b;
            return linearTime > 0f ? linearTime : -1f;
        }

        float discriminant = b * b - 4f * a * c;
        if (discriminant < 0f)
            return -1f;

        float sqrtD = Mathf.Sqrt(discriminant);
        float denominator = 2f * a;
        float t0 = (-b - sqrtD) / denominator;
        float t1 = (-b + sqrtD) / denominator;

        float best = float.PositiveInfinity;
        float minimum = Mathf.Max(
            Time.fixedDeltaTime * 0.5f,
            0.0001f);

        if (t0 >= minimum)
            best = t0;
        if (t1 >= minimum && t1 < best)
            best = t1;

        return float.IsInfinity(best) ? -1f : best;
    }

    private void ApplyPreLimitBallisticPlanarCorrection(
        ref Vector3 accelerationState,
        float maximumAcceleration,
        float maximumJerk)
    {
        if (!preLimitBallisticPlanValid)
            return;

        float dt = Mathf.Max(Time.fixedDeltaTime, 0.000001f);
        float t = Mathf.Clamp(
            preLimitElapsed,
            0f,
            Mathf.Max(preLimitBallisticFlightSeconds, 0.0001f));

        Vector3 plannedPosition =
            preLimitBallisticStartPosition +
            preLimitBallisticLaunchVelocity * t +
            0.5f * Physics.gravity * (t * t);

        Vector3 plannedVelocity =
            preLimitBallisticLaunchVelocity +
            Physics.gravity * t;

        // The vertical channel is pure POP + gravity.
        // Only planar drift is corrected, so no artificial steep dive can form.
        Vector3 planarPositionError = Vector3.ProjectOnPlane(
            plannedPosition - ballBody.position,
            Vector3.up);

        Vector3 planarVelocityError = Vector3.ProjectOnPlane(
            plannedVelocity - ballBody.velocity,
            Vector3.up);

        Vector3 desiredAcceleration =
            planarPositionError * preLimitAscentPositionGain +
            planarVelocityError * preLimitAscentVelocityGain;

        desiredAcceleration = Vector3.ClampMagnitude(
            desiredAcceleration,
            Mathf.Max(0f, maximumAcceleration));

        accelerationState = Vector3.MoveTowards(
            accelerationState,
            desiredAcceleration,
            Mathf.Max(0f, maximumJerk) * dt);

        ballBody.AddForce(
            accelerationState,
            ForceMode.Acceleration);

        debugAppliedLinearAcceleration = accelerationState;
        debugAppliedStick = 0f;
        debugAppliedTangentialAcceleration = 0f;
        debugReleaseWeight = 0f;
        debugRejoinPositionWeight =
            preLimitBallisticFlightSeconds > 0.0001f
                ? Mathf.Clamp01(
                    preLimitElapsed / preLimitBallisticFlightSeconds)
                : 1f;
    }

    private void ObserveLimitCrossing()
    {
        float currentError =
            slopeStick.slopeProgressErrorPercent;

        float currentTime =
            Time.fixedTime;

        Vector3 currentBallPosition =
            ballBody.position;

        Vector3 currentBallVelocity =
            ballBody.velocity;

        Vector3 currentSubjectPosition =
            respondSubject.MappedPosition;

        Vector3 currentSubjectVelocity =
            respondSubject.Mappedvelocity;

        Vector3 currentLimitNormal =
            currentSurfaceNormal.sqrMagnitude > 0.000001f
                ? currentSurfaceNormal.normalized
                : Vector3.up;

        if (!hasPreviousProgressSample)
        {
            previousProgressError = currentError;
            previousProgressTime = currentTime;
            previousProgressBallPosition = currentBallPosition;
            previousProgressBallVelocity = currentBallVelocity;
            previousProgressSubjectPosition = currentSubjectPosition;
            previousProgressSubjectVelocity = currentSubjectVelocity;
            previousProgressSurfaceNormal = currentLimitNormal;
            hasPreviousProgressSample = true;
            hasPreviousProgressBoundarySample = true;
            return;
        }

        bool crossedZero =
            (previousProgressError < 0f && currentError >= 0f) ||
            (previousProgressError > 0f && currentError <= 0f) ||
            (Mathf.Abs(currentError) <= 0.000001f &&
             Mathf.Abs(previousProgressError) > 0.000001f);

        if (crossedZero &&
            IsLimitApproachPhase())
        {
            float denominator =
                currentError - previousProgressError;

            float alpha =
                Mathf.Abs(denominator) > 0.000001f
                    ? -previousProgressError / denominator
                    : 1f;

            alpha = Mathf.Clamp01(alpha);
            limitCrossingAlpha = alpha;

            // ---------------------------------------------------------
            // 真の 0 crossing をFixedUpdate間で補間する。
            // ここで得た状態は「-0の入射条件」。Rigidbodyを過去へ戻さない。
            // ---------------------------------------------------------
            experimentEstimatedLimitCrossingTime =
                Mathf.Lerp(
                    previousProgressTime,
                    currentTime,
                    alpha
                );

            limitCrossingBallPosition =
                Vector3.Lerp(
                    previousProgressBallPosition,
                    currentBallPosition,
                    alpha
                );

            limitCrossingBallVelocity =
                Vector3.Lerp(
                    previousProgressBallVelocity,
                    currentBallVelocity,
                    alpha
                );

            limitCrossingSubjectPosition =
                Vector3.Lerp(
                    previousProgressSubjectPosition,
                    currentSubjectPosition,
                    alpha
                );

            limitCrossingSubjectVelocity =
                Vector3.Lerp(
                    previousProgressSubjectVelocity,
                    currentSubjectVelocity,
                    alpha
                );

            Vector3 interpolatedNormal =
                Vector3.Slerp(
                    previousProgressSurfaceNormal.sqrMagnitude > 0.000001f
                        ? previousProgressSurfaceNormal.normalized
                        : currentLimitNormal,
                    currentLimitNormal,
                    alpha
                );

            if (interpolatedNormal.sqrMagnitude > 0.000001f)
            {
                limitCrossingSurfaceNormal =
                    interpolatedNormal.normalized;

                if (Vector3.Dot(
                        limitCrossingSurfaceNormal,
                        Vector3.up) < 0f)
                {
                    limitCrossingSurfaceNormal =
                        -limitCrossingSurfaceNormal;
                }

                limitCrossingSurfaceNormalValid = true;
            }
            else
            {
                limitCrossingSurfaceNormal = currentLimitNormal;
                limitCrossingSurfaceNormalValid = true;
            }

            // +0処理自体は現在FixedUpdateで行うが、空間上の基準は
            // 必ず真のTarget Progress(60%)点に置く。
            experimentLimitTime =
                currentTime;

            experimentLimitReferencePosition =
                preLimitBallisticPlanValid
                    ? preLimitLockedTargetPosition
                    : limitCrossingSubjectPosition;

            experimentLimitReferenceVelocity =
                limitCrossingSubjectVelocity;

            // Arrival品質も「現在Subject位置」ではなく極限点そのものに対して測る。
            limitPreSyncPositionError =
                Vector3.Distance(
                    limitCrossingBallPosition,
                    experimentLimitReferencePosition
                );

            preLimitArrivalPositionError =
                limitPreSyncPositionError;

            preLimitArrivalVelocityError =
                Vector3.Distance(
                    limitCrossingBallVelocity,
                    limitCrossingSubjectVelocity
                );

            // -0 -> 0 -> +0 のうち、-0のSubject相対入射状態。
            limitIncomingRelativePosition =
                limitCrossingBallPosition -
                experimentLimitReferencePosition;

            limitIncomingRelativeVelocity =
                limitCrossingBallVelocity -
                experimentLimitReferenceVelocity;

            preLimitArrivalWithinTolerance =
                preLimitArrivalPositionError <=
                    preLimitArrivalPositionTolerance &&
                preLimitArrivalVelocityError <=
                    preLimitArrivalVelocityTolerance;

            if (useUnifiedPreLimitGuidance)
            {
                Debug.Log(
                    $"[UNIFIED PRE LIMIT ARRIVAL] " +
                    $"estimatedCrossingTime={experimentEstimatedLimitCrossingTime:F4} " +
                    $"actualLimitFrame={currentTime:F4} " +
                    $"alpha={limitCrossingAlpha:F4} " +
                    $"predictedDeadline={preLimitPredictedDeadlineTime:F4} " +
                    $"deadlineError={(currentTime - preLimitPredictedDeadlineTime):F4}s " +
                    $"positionError={preLimitArrivalPositionError:F4} " +
                    $"velocityError={preLimitArrivalVelocityError:F4} " +
                    $"withinTolerance={preLimitArrivalWithinTolerance} " +
                    $"incomingRelative={limitIncomingRelativeVelocity:F4} " +
                    $"limitNormal={limitCrossingSurfaceNormal:F4} " +
                    $"peakAscentAccel={preLimitPeakAscentAcceleration:F3} " +
                    $"peakMissileAccel={preLimitPeakMissileAcceleration:F3} " +
                    $"ball0={limitCrossingBallPosition:F4} " +
                    $"subject0={limitCrossingSubjectPosition:F4}",
                    this
                );
            }

            SynchronizeBallExactlyAtLimit();
            ApplySingleLimitBounce();
        }

        previousProgressError = currentError;
        previousProgressTime = currentTime;
        previousProgressBallPosition = currentBallPosition;
        previousProgressBallVelocity = currentBallVelocity;
        previousProgressSubjectPosition = currentSubjectPosition;
        previousProgressSubjectVelocity = currentSubjectVelocity;
        previousProgressSurfaceNormal = currentLimitNormal;
        hasPreviousProgressBoundarySample = true;
    }

    private void SynchronizeBallExactlyAtLimit()
    {
        if (ballBody.isKinematic)
            ballBody.isKinematic = false;

        // -------------------------------------------------------------
        // Limit RePOP Natural Spin
        // -------------------------------------------------------------
        // Limitで位置・並進速度はSubjectへ正確に同期するが、
        // BallVisualがPre-Limit飛翔中に持っていた回転姿勢と角速度は
        // RePOP後へそのまま持ち越す。
        //
        // ここでMappedRotation / Subject angularVelocityへ上書きすると、
        // Flat -> POP -> Apex -> Missile Descentで蓄積・継承してきた
        // BallVisual自身の回転がLimit境界で消えてしまう。
        Quaternion preservedRotation =
            ballBody.rotation;

        Vector3 preservedAngularVelocity =
            ballBody.angularVelocity;

        // 何らかの理由で角速度がほぼ0になっている場合だけ、
        // 現在のLimit並進速度と現在面法線から自然な転がり角速度を再構築する。
        if (preservedAngularVelocity.sqrMagnitude <= 0.000001f)
        {
            Vector3 fallbackNormal =
                currentSurfaceNormal.sqrMagnitude > 0.000001f
                    ? currentSurfaceNormal.normalized
                    : Vector3.up;

            preservedAngularVelocity =
                CalculateRollingAngularVelocity(
                    experimentLimitReferenceVelocity,
                    fallbackNormal
                );
        }

        // -------------------------------------------------------------
        // Natural Limit Continuity
        // -------------------------------------------------------------
        // 到着精度が十分なら、1st POP自身が作った位置/速度を残し、
        // 極限でのワープを消してそのまま2nd POPへ繋ぐ。
        // Tolerance外でもHard Sync先は現在Subject位置ではなく、
        // ロック済みの真のTarget Progress(60%)点に限定する。
        bool shouldHardSync =
            !preferNaturalLimitContinuity ||
            (!preLimitArrivalWithinTolerance &&
             hardSyncLimitWhenArrivalOutsideTolerance);

        limitHardSyncApplied = shouldHardSync;

        if (shouldHardSync)
        {
            ballBody.position =
                experimentLimitReferencePosition;

            ballBody.velocity =
                experimentLimitReferenceVelocity;
        }

        // 回転系の所有権は常にBallVisualへ残す。
        ballBody.rotation =
            preservedRotation;

        ballBody.angularVelocity =
            preservedAngularVelocity;

        debugTargetAngularVelocity =
            preservedAngularVelocity;

        debugAppliedAngularAcceleration =
            Vector3.zero;

        debugRollingWeight =
            0f;

        experimentLimitBallPosition =
            ballBody.position;

        experimentLimitBallVelocity =
            ballBody.velocity;

        limitPostSyncPositionError =
            Vector3.Distance(
                ballBody.position,
                experimentLimitReferencePosition
            );
    }

    private void ApplySingleLimitBounce()
    {
        if (!IsLimitApproachPhase())
            return;

        EnsureExperimentColliderIsSolid();

        if (ballBody.isKinematic)
            ballBody.isKinematic = false;

        ballBody.useGravity = true;

        Vector3 velocityBefore =
            ballBody.velocity;

        Vector3 subjectVelocity =
            experimentLimitReferenceVelocity;

        Vector3 plannedVelocityAfter;

        if (useSubjectRelativeEnergyPop)
        {
            // ---------------------------------------------------------
            // 2nd POP Direction = Contact Angle
            // 2nd POP Height    = Subject Relative Energy
            // ---------------------------------------------------------
            resolvedLimitBounceDirection =
                ResolvePostLimitBounceDirection(
                    limitIncomingRelativeVelocity
                );

            // 1st POPと同じ式を通す。
            // 方向だけがContact由来、速度二乗/高さはSubject相対Energy由来。
            plannedVelocityAfter =
                SolveSubjectRelativePopVelocity(
                    subjectVelocity,
                    resolvedLimitBounceDirection,
                    postLimitTargetHeightRelativeToSubject,
                    secondPopVelocitySquaredRatio,
                    out resolvedSecondPopTargetHeight,
                    out resolvedLimitBounceRelativeSpeed
                );
        }
        else
        {
            // 旧方式。比較用に残す。
            Vector3 planarVelocity =
                Vector3.ProjectOnPlane(
                    subjectVelocity,
                    Vector3.up
                );

            plannedVelocityAfter =
                planarVelocity +
                Vector3.up * limitBounceDeltaV;

            resolvedLimitBounceDirection = Vector3.up;
            resolvedLimitBounceRelativeSpeed = limitBounceDeltaV;
        }

        Vector3 deltaVelocity =
            plannedVelocityAfter -
            velocityBefore;

        // 旧ログとの比較用: World並進KE。
        experimentEnergyBeforeBounce =
            CalculateTranslationalEnergy(
                velocityBefore
            );

        experimentPlannedEnergyAfterBounce =
            CalculateTranslationalEnergy(
                plannedVelocityAfter
            );

        // 新しい主診断値: Subject基準の相対力学Energy。
        experimentRelativeEnergyBeforeBounce =
            CalculateSubjectRelativeMechanicalEnergy(
                experimentLimitReferencePosition +
                    limitIncomingRelativePosition,
                subjectVelocity +
                    limitIncomingRelativeVelocity,
                experimentLimitReferencePosition,
                subjectVelocity
            );

        experimentRelativeEnergyPlannedAfterBounce =
            CalculateSubjectRelativeMechanicalEnergy(
                ballBody.position,
                plannedVelocityAfter,
                experimentLimitReferencePosition,
                subjectVelocity
            );

        experimentBounceAppliedTime =
            Time.fixedTime;

        ballBody.AddForce(
            deltaVelocity,
            ForceMode.VelocityChange
        );

        // RePOPではvelocityだけを変更する。
        // rotation / angularVelocityは1st POPから連続させる。
        debugTargetAngularVelocity =
            ballBody.angularVelocity;

        debugAppliedAngularAcceleration =
            Vector3.zero;

        debugRollingWeight =
            0f;

        if (useUnifiedPostLimitGuidance)
        {
            postLimitGuidanceStartTime =
                experimentBounceAppliedTime;

            postLimitElapsed = 0f;

            // ApexはWorld YではなくSubject相対Yで判定する。
            postLimitUpSpeed =
                Vector3.Dot(
                    plannedVelocityAfter - subjectVelocity,
                    Vector3.up
                );

            postLimitHadPositiveUpSpeed =
                postLimitUpSpeed >
                postLimitApexVerticalSpeedThreshold;

            postLimitAscentAccelerationState =
                Vector3.zero;

            postLimitChaseAccelerationState =
                Vector3.zero;

            experimentPhase =
                LimitExperimentPhase.PostLimitAscent;

            SetPhase(
                VisualPhase.SlopeControlled,
                "UnifiedPostLimitRePOP"
            );
        }
        else
        {
            experimentPhase =
                LimitExperimentPhase.Ballistic;
        }

        bool freshContact =
            HasFreshStairContactSample();

        Debug.Log(
            $"[LIMIT REPOP ENERGY CONTACT] " +
            $"estimatedCrossingTime={experimentEstimatedLimitCrossingTime:F4} " +
            $"syncTime={experimentLimitTime:F4} " +
            $"hardSync={limitHardSyncApplied} " +
            $"freshContact={freshContact} " +
            $"contactAge={(hasStairContactSample ? Time.fixedTime - lastStairContactTime : -1f):F4}s " +
            $"contactNormal={lastStairContactNormal:F4} " +
            $"incomingRelative={limitIncomingRelativeVelocity:F4} " +
            $"bounceDirection={resolvedLimitBounceDirection:F4} " +
            $"relativeSpeed={resolvedLimitBounceRelativeSpeed:F4} " +
            $"targetHeight={resolvedSecondPopTargetHeight:F4}m " +
            $"subjectSpeed={subjectReferenceSpeedForPop:F4} " +
            $"subjectQ={subjectVelocitySquaredEnergyScale:F4} " +
            $"gRel={resolvedSubjectRelativeGravity:F4} " +
            $"velocityBefore={velocityBefore:F4} " +
            $"subjectVelocity={subjectVelocity:F4} " +
            $"plannedVelocityAfter={plannedVelocityAfter:F4} " +
            $"deltaV={deltaVelocity:F4} " +
            $"worldKEBefore={experimentEnergyBeforeBounce:F4} " +
            $"worldKEAfter={experimentPlannedEnergyAfterBounce:F4} " +
            $"relativeEBefore={experimentRelativeEnergyBeforeBounce:F4} " +
            $"relativeEAfter={experimentRelativeEnergyPlannedAfterBounce:F4}",
            this
        );
    }

    private void ProcessBallisticExperiment(bool isFlat)
    {
        // Limit以降は意図的に何も制御しない。
        // Gravity + Rigidbody + 実Colliderだけで飛翔・衝突させる。
        // Flat到達時刻はOnCollisionEnter("plane")だけを採用する。
        // Subject側の状態変化を着地と誤認しないため、isFlatはここでは判定に使わない。
        debugAppliedLinearAcceleration = Vector3.zero;
        debugAppliedStick = 0f;
        debugAppliedTangentialAcceleration = 0f;
        debugReleaseWeight = 0f;
        debugRejoinPositionWeight = 0f;
    }

    private void ProcessUnifiedPostLimitAscent()
    {
        if (experimentPhase != LimitExperimentPhase.PostLimitAscent)
            return;

        float dt =
            Mathf.Max(Time.fixedDeltaTime, 0.000001f);

        postLimitElapsed =
            Mathf.Max(
                0f,
                Time.fixedTime - postLimitGuidanceStartTime
            );

        Vector3 subjectPosition =
            respondSubject.MappedPosition;

        Vector3 subjectVelocity =
            respondSubject.Mappedvelocity;

        Vector3 positionError =
            subjectPosition - ballBody.position;

        Vector3 velocityError =
            subjectVelocity - ballBody.velocity;

        // 上昇そのものはRePOP + Gravityに任せる。
        // この区間では平面方向だけ弱くSubjectへ寄せ、
        // 上昇弧を壊さない。
        Vector3 planarPositionError =
            Vector3.ProjectOnPlane(
                positionError,
                Vector3.up
            );

        Vector3 planarVelocityError =
            Vector3.ProjectOnPlane(
                velocityError,
                Vector3.up
            );

        Vector3 desiredAcceleration =
            planarPositionError *
            postLimitAscentPositionGain +
            planarVelocityError *
            postLimitAscentVelocityGain;

        desiredAcceleration =
            Vector3.ClampMagnitude(
                desiredAcceleration,
                postLimitAscentMaximumAcceleration
            );

        postLimitAscentAccelerationState =
            Vector3.MoveTowards(
                postLimitAscentAccelerationState,
                desiredAcceleration,
                postLimitAscentMaximumJerk * dt
            );

        ballBody.AddForce(
            postLimitAscentAccelerationState,
            ForceMode.Acceleration
        );

        postLimitPeakAscentAcceleration =
            Mathf.Max(
                postLimitPeakAscentAcceleration,
                postLimitAscentAccelerationState.magnitude
            );

        postLimitUpSpeed =
            Vector3.Dot(
                ballBody.velocity - subjectVelocity,
                Vector3.up
            );

        if (postLimitUpSpeed >
            postLimitApexVerticalSpeedThreshold)
        {
            postLimitHadPositiveUpSpeed = true;
        }

        // RePOPのVelocityChangeは同じFixedUpdate内では
        // Rigidbody.velocityへまだ反映されていない場合がある。
        // そのためRePOPした同一フレームではApex判定しない。
        bool canDetectApex =
            postLimitElapsed >=
            Mathf.Max(Time.fixedDeltaTime * 0.5f, 0.000001f);

        bool crossedApex =
            canDetectApex &&
            postLimitHadPositiveUpSpeed &&
            postLimitUpSpeed <=
            postLimitApexVerticalSpeedThreshold;

        bool ascentTimedOut =
            postLimitElapsed >=
            postLimitMaximumAscentSeconds;

        debugAppliedLinearAcceleration =
            postLimitAscentAccelerationState;

        debugAppliedStick = 0f;
        debugAppliedTangentialAcceleration = 0f;
        debugReleaseWeight = 0f;
        debugRejoinPositionWeight = 0f;

        if (!crossedApex && !ascentTimedOut)
            return;

        postLimitApexTime =
            Time.fixedTime;

        experimentPhase =
            LimitExperimentPhase.PostLimitChase;

        // 上昇から下降追尾へJerkを連続させる。
        postLimitChaseAccelerationState =
            postLimitAscentAccelerationState;

        Debug.Log(
            $"[UNIFIED POST LIMIT] Apex " +
            $"time={postLimitApexTime:F4} " +
            $"limitToApex={(postLimitApexTime - experimentLimitTime):F4}s " +
            $"upSpeed={postLimitUpSpeed:F4} " +
            $"timeout={ascentTimedOut} " +
            $"ballPos={ballBody.position:F4} " +
            $"subjectPos={subjectPosition:F4}",
            this
        );
    }

    private void ProcessUnifiedPostLimitChase()
    {
        if (experimentPhase != LimitExperimentPhase.PostLimitChase)
            return;

        float dt =
            Mathf.Max(Time.fixedDeltaTime, 0.000001f);

        postLimitElapsed =
            Mathf.Max(
                0f,
                Time.fixedTime - postLimitGuidanceStartTime
            );

        Vector3 subjectPosition =
            respondSubject.MappedPosition;

        Vector3 subjectVelocity =
            respondSubject.Mappedvelocity;

        // 現在地点ではなく、少し先のSubjectをFuture Shadowとして狙う。
        Vector3 shadowTargetPosition =
            subjectPosition +
            subjectVelocity *
            postLimitChaseLeadSeconds;

        Vector3 positionError =
            shadowTargetPosition -
            ballBody.position;

        Vector3 velocityError =
            subjectVelocity -
            ballBody.velocity;

        // Apex後は3Dで追う。
        // GravityはRigidbody側で既に作用しているため、
        // ここは軌道誤差を詰める人工加速度だけを加える。
        Vector3 desiredAcceleration =
            positionError *
            postLimitChasePositionGain +
            velocityError *
            postLimitChaseVelocityGain;

        desiredAcceleration =
            Vector3.ClampMagnitude(
                desiredAcceleration,
                postLimitChaseMaximumAcceleration
            );

        postLimitChaseAccelerationState =
            Vector3.MoveTowards(
                postLimitChaseAccelerationState,
                desiredAcceleration,
                postLimitChaseMaximumJerk * dt
            );

        ballBody.AddForce(
            postLimitChaseAccelerationState,
            ForceMode.Acceleration
        );

        postLimitPeakChaseAcceleration =
            Mathf.Max(
                postLimitPeakChaseAcceleration,
                postLimitChaseAccelerationState.magnitude
            );

        debugAppliedLinearAcceleration =
            postLimitChaseAccelerationState;

        debugAppliedStick = 0f;
        debugAppliedTangentialAcceleration = 0f;
        debugReleaseWeight = 0f;
        debugRejoinPositionWeight = 0f;

        if (enableDebugLog &&
            fixedFrameCounter %
            Mathf.Max(1, logEveryFixedFrames) == 0)
        {
            Debug.Log(
                $"[UNIFIED POST LIMIT] Chase " +
                $"elapsed={postLimitElapsed:F4}s " +
                $"ballPos={ballBody.position:F4} " +
                $"subjectPos={subjectPosition:F4} " +
                $"shadowTarget={shadowTargetPosition:F4} " +
                $"posError={positionError.magnitude:F4} " +
                $"velError={velocityError.magnitude:F4} " +
                $"accel={postLimitChaseAccelerationState.magnitude:F4}",
                this
            );
        }
    }

    private bool ShouldStartUnifiedTerminal(bool isFlat)
    {
        if (experimentPhase != LimitExperimentPhase.PostLimitChase)
            return false;

        postLimitElapsed =
            Mathf.Max(
                0f,
                Time.fixedTime - postLimitGuidanceStartTime
            );

        bool subjectFlat =
            isFlat ||
            slopeStick.groundKind == GroundKind.Flat;

        if (terminalStartFromSubjectFlat && subjectFlat)
        {
            if (!postLimitSubjectFlatCaptured)
            {
                postLimitSubjectFlatCaptured = true;
                postLimitSubjectFlatTime = Time.fixedTime;

                Debug.Log(
                    $"[UNIFIED POST LIMIT] SubjectFlat " +
                    $"time={postLimitSubjectFlatTime:F4} " +
                    $"limitToSubjectFlat={(postLimitSubjectFlatTime - experimentLimitTime):F4}s " +
                    $"subjectPos={respondSubject.MappedPosition:F4} " +
                    $"subjectVel={respondSubject.Mappedvelocity:F4} " +
                    $"ballPos={ballBody.position:F4} " +
                    $"ballVel={ballBody.velocity:F4}",
                    this
                );
            }

            return true;
        }

        if (!terminalStartFromSubjectFlat)
            return true;

        // Subject側Flat分類が一瞬欠落しても状態機械を永久停止させない。
        return
            postLimitElapsed >=
            postLimitMaximumWaitForSubjectFlatSeconds;
    }

    private void BeginUnifiedTerminalRejoin()
    {
        if (experimentPhase != LimitExperimentPhase.PostLimitChase)
            return;

        terminalRejoinStartTime =
            Time.fixedTime;

        terminalRejoinCompleteTime = -1f;
        terminalRejoinCostSeconds = -1f;
        terminalLimitToSyncSeconds = -1f;
        terminalElapsed = 0f;
        terminalTimeToGo =
            Mathf.Max(
                terminalTimeBudget,
                terminalMinimumTimeToGo
            );

        terminalTargetPosition =
            respondSubject.MappedPosition;

        terminalTargetVelocity =
            respondSubject.Mappedvelocity;

        // Chaseの加速度状態から開始してJerkを連続させる。
        terminalAccelerationState =
            postLimitChaseAccelerationState;

        previousTerminalAccelerationState =
            terminalAccelerationState;

        terminalPositionError = -1f;
        terminalVelocityError = -1f;
        terminalPeakAcceleration =
            terminalAccelerationState.magnitude;
        terminalPeakJerk = 0f;
        terminalStableFrames = 0;
        terminalForcedSyncAtDeadline = false;

        experimentPhase =
            LimitExperimentPhase.TerminalRejoin;

        SetPhase(
            VisualPhase.Rejoining,
            "UnifiedTerminalRejoin"
        );

        Debug.Log(
            $"[UNIFIED TERMINAL] Begin " +
            $"time={terminalRejoinStartTime:F4} " +
            $"budget={terminalTimeBudget:F4}s " +
            $"ballFlat={postLimitBallFlatCaptured} " +
            $"ballPos={ballBody.position:F4} " +
            $"subjectPos={terminalTargetPosition:F4} " +
            $"ballVel={ballBody.velocity:F4} " +
            $"subjectVel={terminalTargetVelocity:F4}",
            this
        );
    }

    private void ProcessUnifiedTerminalRejoin()
    {
        if (experimentPhase != LimitExperimentPhase.TerminalRejoin)
            return;

        float dt =
            Mathf.Max(Time.fixedDeltaTime, 0.000001f);

        terminalElapsed =
            Mathf.Max(
                0f,
                Time.fixedTime - terminalRejoinStartTime
            );

        float rawTimeToGo =
            terminalTimeBudget -
            terminalElapsed;

        terminalTimeToGo =
            Mathf.Max(
                terminalMinimumTimeToGo,
                rawTimeToGo
            );

        Vector3 subjectPosition =
            respondSubject.MappedPosition;

        Vector3 subjectVelocity =
            respondSubject.Mappedvelocity;

        // Tgo秒後のSubjectをRendezvous Pointとする。
        // 現段階ではSubject加速度を未来へ外挿せず、
        // 現在速度一定のFuture Shadowだけを使う。
        terminalTargetPosition =
            subjectPosition +
            subjectVelocity *
            terminalTimeToGo;

        terminalTargetVelocity =
            subjectVelocity;

        Vector3 positionToDeadline =
            terminalTargetPosition -
            ballBody.position;

        Vector3 velocityNow =
            ballBody.velocity;

        float safeTimeToGo =
            Mathf.Max(
                terminalTimeToGo,
                terminalMinimumTimeToGo
            );

        // p(T)=P, v(T)=V を満たす3次終端軌道の
        // 「現在時刻で必要な総加速度」。
        Vector3 requiredTotalAcceleration =
            6f * positionToDeadline /
            (safeTimeToGo * safeTimeToGo) -
            (4f * velocityNow +
             2f * terminalTargetVelocity) /
            safeTimeToGo;

        // RigidbodyにはGravityが別途作用しているため、
        // AddForceすべき人工加速度はGravity分を差し引く。
        Vector3 desiredArtificialAcceleration;

        if (unifiedTerminalUse3D)
        {
            desiredArtificialAcceleration =
                requiredTotalAcceleration -
                Physics.gravity;
        }
        else
        {
            desiredArtificialAcceleration =
                Vector3.ProjectOnPlane(
                    requiredTotalAcceleration,
                    Vector3.up
                );
        }

        desiredArtificialAcceleration =
            Vector3.ClampMagnitude(
                desiredArtificialAcceleration,
                maximumTerminalAcceleration
            );

        previousTerminalAccelerationState =
            terminalAccelerationState;

        terminalAccelerationState =
            Vector3.MoveTowards(
                terminalAccelerationState,
                desiredArtificialAcceleration,
                maximumTerminalJerk * dt
            );

        ballBody.AddForce(
            terminalAccelerationState,
            ForceMode.Acceleration
        );

        float jerkMagnitude =
            (terminalAccelerationState -
             previousTerminalAccelerationState).magnitude /
            dt;

        terminalPeakAcceleration =
            Mathf.Max(
                terminalPeakAcceleration,
                terminalAccelerationState.magnitude
            );

        terminalPeakJerk =
            Mathf.Max(
                terminalPeakJerk,
                jerkMagnitude
            );

        Vector3 currentPositionError =
            subjectPosition -
            ballBody.position;

        Vector3 currentVelocityError =
            subjectVelocity -
            ballBody.velocity;

        if (!unifiedTerminalUse3D)
        {
            currentPositionError =
                Vector3.ProjectOnPlane(
                    currentPositionError,
                    Vector3.up
                );

            currentVelocityError =
                Vector3.ProjectOnPlane(
                    currentVelocityError,
                    Vector3.up
                );
        }

        terminalPositionError =
            currentPositionError.magnitude;

        terminalVelocityError =
            currentVelocityError.magnitude;

        bool contactEligible =
            !requireBallFlatContactBeforePhysicalSync ||
            postLimitBallFlatCaptured;

        bool positionReady =
            terminalPositionError <=
            terminalPositionTolerance;

        bool velocityReady =
            terminalVelocityError <=
            terminalVelocityTolerance;

        if (contactEligible &&
            positionReady &&
            velocityReady)
        {
            terminalStableFrames++;
        }
        else
        {
            terminalStableFrames = 0;
        }

        debugAppliedLinearAcceleration =
            terminalAccelerationState;

        debugAppliedStick = 0f;
        debugAppliedTangentialAcceleration = 0f;
        debugReleaseWeight = 0f;
        debugRejoinPositionWeight =
            Mathf.Clamp01(
                terminalElapsed /
                Mathf.Max(terminalTimeBudget, 0.000001f)
            );

        if (terminalStableFrames >=
            Mathf.Max(1, terminalStableFramesRequired))
        {
            CompleteUnifiedTerminalRejoin(false);
            return;
        }

        if (rawTimeToGo <= 0f &&
            forceTerminalSyncAtDeadline)
        {
            CompleteUnifiedTerminalRejoin(true);
            return;
        }

        if (enableDebugLog &&
            fixedFrameCounter %
            Mathf.Max(1, logEveryFixedFrames) == 0)
        {
            Debug.Log(
                $"[UNIFIED TERMINAL] " +
                $"elapsed={terminalElapsed:F4}s " +
                $"Tgo={terminalTimeToGo:F4}s " +
                $"ballFlat={postLimitBallFlatCaptured} " +
                $"posError={terminalPositionError:F4} " +
                $"velError={terminalVelocityError:F4} " +
                $"accel={terminalAccelerationState.magnitude:F4} " +
                $"peakAccel={terminalPeakAcceleration:F4} " +
                $"targetPos={terminalTargetPosition:F4}",
                this
            );
        }
    }

    private void CompleteUnifiedTerminalRejoin(
        bool forcedAtDeadline
    )
    {
        if (experimentPhase != LimitExperimentPhase.TerminalRejoin)
            return;

        Vector3 subjectPosition =
            respondSubject.MappedPosition;

        Vector3 subjectVelocity =
            respondSubject.Mappedvelocity;

        float preSyncPositionError =
            Vector3.Distance(
                ballBody.position,
                subjectPosition
            );

        float preSyncVelocityError =
            Vector3.Distance(
                ballBody.velocity,
                subjectVelocity
            );

        terminalForcedSyncAtDeadline =
            forcedAtDeadline;

        terminalRejoinCompleteTime =
            Time.fixedTime;

        terminalRejoinCostSeconds =
            terminalRejoinCompleteTime -
            terminalRejoinStartTime;

        terminalLimitToSyncSeconds =
            terminalRejoinCompleteTime -
            experimentLimitTime;

        // 計測終了境界でのみ完全同期する。
        // 途中の誘導精度はpreSyncErrorとしてログへ残す。
        ballBody.position =
            subjectPosition;

        ballBody.velocity =
            subjectVelocity;

        ballBody.rotation =
            respondSubject.MappedRotation;

        if (subjectBody != null && respondSubject != null)
        {
            ballBody.angularVelocity =
                respondSubject.MapDirection(
                    subjectBody.angularVelocity
                );
        }

        // Settled中は次のSlope開始までSubjectと完全一致させる。
        ballBody.useGravity = false;

        if (ballCollider != null)
            ballCollider.isTrigger = true;

        terminalPositionError = 0f;
        terminalVelocityError = 0f;
        terminalAccelerationState = Vector3.zero;
        previousTerminalAccelerationState = Vector3.zero;
        terminalStableFrames = 0;

        experimentSettledTime =
            terminalRejoinCompleteTime;

        experimentLimitToSettledSeconds =
            terminalLimitToSyncSeconds;

        experimentPhase =
            LimitExperimentPhase.Settled;

        rejoinPending = false;
        wasOnSlope = false;

        SetPhase(
            VisualPhase.LockedToSubject,
            forcedAtDeadline
                ? "UnifiedTerminalForcedSync"
                : "UnifiedTerminalPhysicalSync"
        );

        Debug.Log(
            $"[UNIFIED TERMINAL COMPLETE] " +
            $"forced={forcedAtDeadline} " +
            $"terminalCost={terminalRejoinCostSeconds:F4}s " +
            $"limitToSync={terminalLimitToSyncSeconds:F4}s " +
            $"ballFlat={postLimitBallFlatCaptured} " +
            $"preSyncPosError={preSyncPositionError:F4} " +
            $"preSyncVelError={preSyncVelocityError:F4} " +
            $"peakAscentAccel={postLimitPeakAscentAcceleration:F4} " +
            $"peakChaseAccel={postLimitPeakChaseAcceleration:F4} " +
            $"peakTerminalAccel={terminalPeakAcceleration:F4} " +
            $"peakTerminalJerk={terminalPeakJerk:F4}",
            this
        );
    }

    private void HoldExperimentSettledWithSubject()
    {
        if (ballBody.isKinematic)
            ballBody.isKinematic = false;

        ballBody.useGravity = false;

        ballBody.position =
            respondSubject.MappedPosition;

        ballBody.velocity =
            respondSubject.Mappedvelocity;

        ballBody.rotation =
            respondSubject.MappedRotation;

        if (subjectBody != null && respondSubject != null)
        {
            ballBody.angularVelocity =
                respondSubject.MapDirection(
                    subjectBody.angularVelocity
                );
        }

        if (ballCollider != null)
            ballCollider.isTrigger = true;

        debugAppliedLinearAcceleration = Vector3.zero;
        debugAppliedStick = 0f;
        debugAppliedTangentialAcceleration = 0f;
        debugReleaseWeight = 0f;
        debugRejoinPositionWeight = 1f;
    }

    private void RegisterFlatContact(
        float impulseMagnitude,
        string source
    )
    {
        // Unified経路ではFlat接触は「誘導開始条件」ではなく計測イベント。
        if (useUnifiedPostLimitGuidance)
        {
            bool unifiedFlightPhase =
                experimentPhase == LimitExperimentPhase.PostLimitAscent ||
                experimentPhase == LimitExperimentPhase.PostLimitChase ||
                experimentPhase == LimitExperimentPhase.TerminalRejoin;

            if (!unifiedFlightPhase)
                return;

            if (postLimitBallFlatCaptured)
                return;

            postLimitBallFlatCaptured = true;

            experimentFlatContactTime =
                Time.fixedTime;

            experimentLimitToFlatSeconds =
                experimentFlatContactTime -
                experimentLimitTime;

            experimentEnergyAtFlatContact =
                CalculateTranslationalEnergy(
                    ballBody.velocity
                );

            experimentFlatImpactImpulse =
                impulseMagnitude;

            previousExperimentVelocity =
                ballBody.velocity;

            Debug.Log(
                $"[UNIFIED POST LIMIT] BallFlatContact " +
                $"source={source} " +
                $"time={experimentFlatContactTime:F4} " +
                $"limitToFlat={experimentLimitToFlatSeconds:F4}s " +
                $"phase={experimentPhase} " +
                $"velocity={ballBody.velocity:F4} " +
                $"KE={experimentEnergyAtFlatContact:F4} " +
                $"impulse={experimentFlatImpactImpulse:F4}",
                this
            );

            return;
        }

        // 旧A/B比較経路。
        if (experimentPhase != LimitExperimentPhase.Ballistic)
            return;

        experimentFlatContactTime =
            Time.fixedTime;

        experimentLimitToFlatSeconds =
            experimentFlatContactTime -
            experimentLimitTime;

        experimentEnergyAtFlatContact =
            CalculateTranslationalEnergy(
                ballBody.velocity
            );

        experimentFlatImpactImpulse =
            impulseMagnitude;

        previousExperimentVelocity =
            ballBody.velocity;

        experimentSettledFrames = 0;

        experimentPhase =
            LimitExperimentPhase.FlatContact;

        Debug.Log(
            $"[LIMIT EXPERIMENT] FlatContact " +
            $"source={source} " +
            $"time={experimentFlatContactTime:F4} " +
            $"limitToFlat={experimentLimitToFlatSeconds:F4}s " +
            $"velocity={ballBody.velocity:F4} " +
            $"KE={experimentEnergyAtFlatContact:F4} " +
            $"impulse={experimentFlatImpactImpulse:F4}",
            this
        );
    }

    private void ObserveSettledState(bool isFlat)
    {
        if (experimentPhase != LimitExperimentPhase.FlatContact)
            return;

        Vector3 velocity =
            ballBody.velocity;

        float verticalSpeed =
            Mathf.Abs(velocity.y);

        float velocityChange =
            (velocity - previousExperimentVelocity).magnitude;

        bool settled =
            isFlat &&
            verticalSpeed <= settleVerticalSpeed &&
            velocityChange <= settleVelocityChange;

        if (settled)
            experimentSettledFrames++;
        else
            experimentSettledFrames = 0;

        previousExperimentVelocity = velocity;

        if (experimentSettledFrames <
            Mathf.Max(1, settleStableFramesRequired))
        {
            return;
        }

        experimentSettledTime =
            Time.fixedTime;

        experimentLimitToSettledSeconds =
            experimentSettledTime - experimentLimitTime;

        experimentPhase =
            LimitExperimentPhase.Settled;

        Debug.Log(
            $"[LIMIT EXPERIMENT COMPLETE] " +
            $"limitToFlat={experimentLimitToFlatSeconds:F4}s " +
            $"limitToSettled={experimentLimitToSettledSeconds:F4}s " +
            $"KEBefore={experimentEnergyBeforeBounce:F4} " +
            $"KEPlannedAfter={experimentPlannedEnergyAfterBounce:F4} " +
            $"KEAtFlat={experimentEnergyAtFlatContact:F4} " +
            $"flatImpulse={experimentFlatImpactImpulse:F4}",
            this
        );
    }

    private void EnsureExperimentColliderIsSolid()
    {
        if (ballCollider != null && ballCollider.isTrigger)
            ballCollider.isTrigger = false;
    }

    private void CaptureStairContact(
        Collision collision,
        string source)
    {
        if (collision == null ||
            collision.contactCount <= 0)
        {
            return;
        }

        Vector3 normalSum = Vector3.zero;
        Vector3 pointSum = Vector3.zero;
        int validContacts = 0;

        for (int i = 0; i < collision.contactCount; i++)
        {
            ContactPoint contact = collision.GetContact(i);

            if (contact.normal.sqrMagnitude <= 0.000001f)
                continue;

            normalSum += contact.normal.normalized;
            pointSum += contact.point;
            validContacts++;
        }

        if (validContacts <= 0)
            return;

        Vector3 averagedNormal =
            normalSum / validContacts;

        if (averagedNormal.sqrMagnitude <= 0.000001f)
            return;

        averagedNormal.Normalize();

        // Colliderの向きや接触側によって法線が逆を向いた場合だけ上側へ揃える。
        if (Vector3.Dot(averagedNormal, Vector3.up) < 0f)
            averagedNormal = -averagedNormal;

        Vector3 subjectVelocity =
            respondSubject != null
                ? respondSubject.Mappedvelocity
                : Vector3.zero;

        hasStairContactSample = true;
        lastStairContactTime = Time.fixedTime;
        lastStairContactPoint =
            pointSum / validContacts;
        lastStairContactNormal =
            averagedNormal;
        lastStairContactRelativeVelocity =
            ballBody.velocity -
            subjectVelocity;

        if (enableDebugLog)
        {
            Debug.Log(
                $"[STAIR CONTACT SAMPLE] " +
                $"source={source} " +
                $"time={lastStairContactTime:F4} " +
                $"point={lastStairContactPoint:F4} " +
                $"normal={lastStairContactNormal:F4} " +
                $"relativeVelocity={lastStairContactRelativeVelocity:F4}",
                this
            );
        }
    }

    private bool HasFreshStairContactSample()
    {
        if (!hasStairContactSample ||
            lastStairContactTime < 0f)
        {
            return false;
        }

        return
            Time.fixedTime - lastStairContactTime <=
            stairContactMemorySeconds;
    }

    private Vector3 ResolvePostLimitBounceDirection(
        Vector3 incomingRelativeVelocity)
    {
        // -------------------------------------------------------------
        // 0地点の法線を優先。
        // Pre-Limit飛翔中がTriggerでも、SlopeStick観測をVisual座標へ写した
        // 60% crossing法線が残るため、方向が古いContact sampleへ依存しない。
        // -------------------------------------------------------------
        Vector3 contactNormal;

        if (preferInterpolatedLimitNormal &&
            limitCrossingSurfaceNormalValid)
        {
            contactNormal =
                limitCrossingSurfaceNormal;
        }
        else if (HasFreshStairContactSample())
        {
            contactNormal =
                lastStairContactNormal;
        }
        else
        {
            contactNormal =
                currentSurfaceNormal;
        }

        if (contactNormal.sqrMagnitude <= 0.000001f)
            contactNormal = Vector3.up;

        contactNormal.Normalize();

        if (Vector3.Dot(contactNormal, Vector3.up) < 0f)
            contactNormal = -contactNormal;

        Vector3 contactOutgoingDirection;

        if (incomingRelativeVelocity.sqrMagnitude > 0.000001f)
        {
            // u+ = u- - (1 + e)(u-・n)n
            // e=1 ならVector3.Reflectと一致。
            float normalSpeed =
                Vector3.Dot(
                    incomingRelativeVelocity,
                    contactNormal
                );

            Vector3 reflected =
                incomingRelativeVelocity -
                (1f + Mathf.Clamp01(postLimitNormalRestitution)) *
                normalSpeed *
                contactNormal;

            if (reflected.sqrMagnitude > 0.000001f &&
                Vector3.Dot(reflected, contactNormal) > 0f)
            {
                contactOutgoingDirection =
                    reflected.normalized;
            }
            else
            {
                contactOutgoingDirection =
                    contactNormal;
            }
        }
        else
        {
            contactOutgoingDirection =
                contactNormal;
        }

        // 接触角は方向だけを担当する。
        // 高さ/速度の大きさはSolveSubjectRelativePopVelocityが担当する。
        Vector3 direction =
            Vector3.Slerp(
                Vector3.up,
                contactOutgoingDirection,
                Mathf.Clamp01(
                    postLimitContactDirectionWeight
                )
            );

        if (direction.sqrMagnitude <= 0.000001f)
            direction = Vector3.up;

        direction.Normalize();

        return EnsureMinimumUpwardComponent(
            direction,
            minimumPostLimitUpDot
        );
    }

    private Vector3 EnsureMinimumUpwardComponent(
        Vector3 direction,
        float minimumUpDot)
    {
        direction =
            direction.sqrMagnitude > 0.000001f
                ? direction.normalized
                : Vector3.up;

        float minUp =
            Mathf.Clamp(
                minimumUpDot,
                0.05f,
                1f
            );

        float currentUp =
            Vector3.Dot(
                direction,
                Vector3.up
            );

        if (currentUp >= minUp)
            return direction;

        Vector3 planar =
            Vector3.ProjectOnPlane(
                direction,
                Vector3.up
            );

        if (planar.sqrMagnitude <= 0.000001f)
            return Vector3.up;

        planar.Normalize();

        float planarMagnitude =
            Mathf.Sqrt(
                Mathf.Max(
                    0f,
                    1f - minUp * minUp
                )
            );

        return (
            planar * planarMagnitude +
            Vector3.up * minUp
        ).normalized;
    }

    private Vector3 SolveSubjectRelativePopVelocity(
        Vector3 subjectVelocity,
        Vector3 launchDirection,
        float configuredTargetHeight,
        float velocitySquaredRatio,
        out float resolvedTargetHeight,
        out float resolvedRelativeSpeed)
    {
        Vector3 direction =
            launchDirection.sqrMagnitude > 0.000001f
                ? launchDirection.normalized
                : Vector3.up;

        direction =
            EnsureMinimumUpwardComponent(
                direction,
                minimumPostLimitUpDot
            );

        resolvedTargetHeight =
            ResolvePopTargetHeight(
                configuredTargetHeight,
                velocitySquaredRatio
            );

        resolvedRelativeSpeed =
            CalculateRelativeLaunchSpeedForHeight(
                resolvedTargetHeight,
                direction
            );

        return
            subjectVelocity +
            direction * resolvedRelativeSpeed;
    }

    private float ResolvePopTargetHeight(
        float configuredTargetHeight,
        float velocitySquaredRatio)
    {
        float baseHeight =
            Mathf.Max(0f, configuredTargetHeight);

        resolvedSubjectRelativeGravity =
            GetSubjectRelativeGravityMagnitude();

        subjectReferenceSpeedForPop =
            ResolveSubjectReferenceSpeedForPop();

        subjectVelocitySquaredEnergyScale =
            subjectReferenceSpeedForPop *
            subjectReferenceSpeedForPop;

        if (!useSlopeStickVelocitySquaredHeightScale ||
            slopeStickEnergyHeightBlend <= 0f ||
            subjectVelocitySquaredEnergyScale <= 0.000001f)
        {
            return baseHeight;
        }

        // Qpop = eta * Qsubject
        // 2 g_rel h = eta * v_subject^2
        // h = eta * v_subject^2 / (2 g_rel)
        float energyHeight =
            Mathf.Max(0f, velocitySquaredRatio) *
            subjectVelocitySquaredEnergyScale /
            (2f * Mathf.Max(
                resolvedSubjectRelativeGravity,
                0.0001f));

        float minHeight =
            Mathf.Max(0f, minimumEnergyScaledPopHeight);

        float maxHeight =
            Mathf.Max(
                minHeight,
                maximumEnergyScaledPopHeight
            );

        energyHeight =
            Mathf.Clamp(
                energyHeight,
                minHeight,
                maxHeight
            );

        return Mathf.Lerp(
            baseHeight,
            energyHeight,
            Mathf.Clamp01(slopeStickEnergyHeightBlend)
        );
    }

    private float ResolveSubjectReferenceSpeedForPop()
    {
        Vector3 subjectVelocity =
            respondSubject != null
                ? respondSubject.Mappedvelocity
                : Vector3.zero;

        Vector3 tangent =
            currentSlopeTangent;

        float mappedSubjectTangentSpeed = 0f;

        if (tangent.sqrMagnitude > 0.000001f)
        {
            mappedSubjectTangentSpeed =
                Mathf.Abs(
                    Vector3.Dot(
                        subjectVelocity,
                        tangent.normalized
                    )
                );
        }

        float planarSubjectSpeed =
            Vector3.ProjectOnPlane(
                subjectVelocity,
                Vector3.up
            ).magnitude;

        float slopeStickTargetSpeed =
            slopeStick != null
                ? Mathf.Max(
                    0f,
                    slopeStick.CapturedTargetTangentSpeed
                  )
                : 0f;

        // 実Subject速度を第一候補にし、入口などで接線値が取れない時だけ
        // SlopeStickが保持するTarget速度をエネルギー尺度へ使う。
        float actualSpeed =
            Mathf.Max(
                mappedSubjectTangentSpeed,
                planarSubjectSpeed
            );

        return
            actualSpeed > 0.05f
                ? actualSpeed
                : slopeStickTargetSpeed;
    }

    private float CalculateVerticalSpeedForHeight(
        float targetHeight)
    {
        float gravity =
            GetSubjectRelativeGravityMagnitude();

        return Mathf.Sqrt(
            2f *
            gravity *
            Mathf.Max(0f, targetHeight)
        );
    }

    private float CalculateRelativeLaunchSpeedForHeight(
        float targetHeight,
        Vector3 launchDirection)
    {
        Vector3 direction =
            launchDirection.sqrMagnitude > 0.000001f
                ? launchDirection.normalized
                : Vector3.up;

        float upwardDot =
            Mathf.Max(
                minimumPostLimitUpDot,
                Vector3.Dot(
                    direction,
                    Vector3.up
                )
            );

        // h = vy^2 / (2 g_rel)
        // vy = |vRelative| * direction.y
        // -> |vRelative| = sqrt(2 g_rel h) / direction.y
        float requiredRelativeSpeed =
            CalculateVerticalSpeedForHeight(
                targetHeight
            ) /
            Mathf.Max(upwardDot, 0.0001f);

        return Mathf.Min(
            requiredRelativeSpeed,
            maximumPostLimitRelativeLaunchSpeed
        );
    }

    private void UpdateSubjectRelativeKinematics()
    {
        if (respondSubject == null)
            return;

        Vector3 currentVelocity =
            respondSubject.Mappedvelocity;

        float dt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f
            );

        if (hasMappedSubjectVelocitySample)
        {
            Vector3 rawAcceleration =
                (currentVelocity -
                 previousMappedSubjectVelocity) /
                dt;

            float blend =
                1f -
                Mathf.Exp(
                    -Mathf.Max(
                        0.1f,
                        subjectAccelerationSmoothing
                    ) * dt
                );

            estimatedMappedSubjectAcceleration =
                Vector3.Lerp(
                    estimatedMappedSubjectAcceleration,
                    rawAcceleration,
                    blend
                );
        }
        else
        {
            estimatedMappedSubjectAcceleration =
                Vector3.zero;

            hasMappedSubjectVelocitySample = true;
        }

        previousMappedSubjectVelocity =
            currentVelocity;
    }

    private float GetSubjectRelativeGravityMagnitude()
    {
        float worldGravity =
            GetVerticalGravityMagnitude();

        if (!useSubjectRelativeGravity ||
            !hasMappedSubjectVelocitySample)
        {
            return worldGravity;
        }

        Vector3 relativeAcceleration =
            Physics.gravity -
            estimatedMappedSubjectAcceleration;

        float relativeGravity =
            -Vector3.Dot(
                relativeAcceleration,
                Vector3.up
            );

        float minGravity =
            Mathf.Max(
                0.1f,
                minimumSubjectRelativeGravity
            );

        float maxGravity =
            Mathf.Max(
                minGravity,
                maximumSubjectRelativeGravity
            );

        if (float.IsNaN(relativeGravity) ||
            float.IsInfinity(relativeGravity))
        {
            return worldGravity;
        }

        return Mathf.Clamp(
            relativeGravity,
            minGravity,
            maxGravity
        );
    }

    private float CalculateSubjectRelativeMechanicalEnergy(
        Vector3 ballPosition,
        Vector3 ballVelocity,
        Vector3 subjectPosition,
        Vector3 subjectVelocity
    )
    {
        Vector3 relativeVelocity =
            ballVelocity -
            subjectVelocity;

        float relativeHeight =
            Vector3.Dot(
                ballPosition - subjectPosition,
                Vector3.up
            );

        float gravity =
            GetSubjectRelativeGravityMagnitude();

        float kinetic =
            0.5f *
            ballBody.mass *
            relativeVelocity.sqrMagnitude;

        float potential =
            ballBody.mass *
            gravity *
            relativeHeight;

        return kinetic + potential;
    }

    private float GetVerticalGravityMagnitude()
    {
        float verticalGravity =
            -Vector3.Dot(
                Physics.gravity,
                Vector3.up
            );

        if (verticalGravity <= 0.0001f)
            verticalGravity = Physics.gravity.magnitude;

        return Mathf.Max(
            verticalGravity,
            0.0001f
        );
    }

    private float CalculateTranslationalEnergy(
        Vector3 velocity
    )
    {
        return
            0.5f *
            ballBody.mass *
            velocity.sqrMagnitude;
    }

    private void ApplyProceduralStepPulse()
    {
        float forwardSpeed = Vector3.Dot(
            ballBody.velocity,
            currentSlopeTangent
        );

        float forwardDistance =
            Mathf.Max(0f, forwardSpeed) *
            Time.fixedDeltaTime;

        accumulatedSlopeDistance += forwardDistance;

        int currentStepIndex = Mathf.FloorToInt(
            accumulatedSlopeDistance /
            Mathf.Max(stepLength, 0.01f)
        );

        if (currentStepIndex <= previousStepIndex)
            return;

        previousStepIndex = currentStepIndex;

        float bounceSuppression = Mathf.Clamp01(
            debugAppliedStick /
            Mathf.Max(stickForFullBounceSuppression, 0.001f)
        );

        float bounceWeight = 1f - bounceSuppression;
        float releaseBounceWeight = 1f - debugReleaseWeight;

        float finalBounceWeight = Mathf.Clamp01(
            bounceWeight + releaseBounceWeight
        );

        Vector3 pulse =
            currentSurfaceNormal *
            stepPulseVelocity *
            finalBounceWeight;

        ballBody.AddForce(
            pulse,
            ForceMode.VelocityChange
        );
    }

    private void SetPhase(
        VisualPhase nextPhase,
        string reason
    )
    {
        phase = nextPhase;

        if (!enableDebugLog || phase == previousLoggedPhase)
            return;

        Debug.Log(
            $"[BALL VISUAL PHASE] " +
            $"{previousLoggedPhase} -> {phase} " +
            $"reason={reason} " +
            $"time={Time.fixedTime:F3}",
            this
        );

        previousLoggedPhase = phase;
    }

    private void WriteDebugLog(
        bool isFlat,
        bool isOnSlope
    )
    {
        if (!enableDebugLog)
            return;

        int interval =
            Mathf.Max(1, logEveryFixedFrames);

        if (fixedFrameCounter % interval != 0)
            return;

        // BallVisualはVisual座標系にいるため、
        // 比較対象もCorrespondSubjectでVisual側へ写像したSubject状態を使う。
        Vector3 subjectPosition =
            respondSubject != null
                ? respondSubject.MappedPosition
                : subjectBody.position;

        Vector3 subjectVelocity =
            respondSubject != null
                ? respondSubject.Mappedvelocity
                : subjectBody.velocity;

        Vector3 positionError =
            subjectPosition -
            ballBody.position;

        Vector3 velocityError =
            subjectVelocity -
            ballBody.velocity;

        Debug.Log(
            $"[BALL VISUAL CONTROL] " +
            $"time={Time.fixedTime:F3} " +
            $"phase={phase} " +
            $"experimentPhase={experimentPhase} " +
            $"flat={isFlat} slope={isOnSlope} " +
            $"kinematic={ballBody.isKinematic} " +
            $"mass={ballBody.mass:F3} " +
            $"rejoinPending={rejoinPending} " +
            $"rejoinElapsed={rejoinElapsed:F3} " +
            $"rejoinPositionWeight={debugRejoinPositionWeight:F3} " +
            $"rejoinStableFrames={rejoinStableFrames} " +
            $"subjectPos={subjectPosition:F4} " +
            $"ballPos={ballBody.position:F4} " +
            $"positionError={positionError:F4} " +
            $"positionErrorMag={positionError.magnitude:F4} " +
            $"subjectVelocity={subjectVelocity:F4} " +
            $"ballVelocity={ballBody.velocity:F4} " +
            $"velocityError={velocityError:F4} " +
            $"velocityErrorMag={velocityError.magnitude:F4} " +
            $"normal={currentSurfaceNormal:F4} " +
            $"tangent={currentSlopeTangent:F4} " +
            $"stick={debugAppliedStick:F3} " +
            $"tangentAccel={debugAppliedTangentialAcceleration:F3} " +
            $"releaseWeight={debugReleaseWeight:F3} " +
            $"linearAccel={debugAppliedLinearAcceleration:F4} " +
            $"targetAngular={debugTargetAngularVelocity:F4} " +
            $"ballAngular={ballBody.angularVelocity:F4} " +
            $"angularAccel={debugAppliedAngularAcceleration:F4} " +
            $"rollingWeight={debugRollingWeight:F3}",
            this
        );
    }

    private void OnDrawGizmosSelected()
    {
        if (ballBody == null)
            return;

        Vector3 origin = ballBody.position;

        Gizmos.DrawRay(
            origin,
            currentSurfaceNormal * 2f
        );

        Gizmos.DrawRay(
            origin,
            currentSlopeTangent * 2f
        );

        Gizmos.DrawRay(
            origin,
            debugAppliedLinearAcceleration * 0.02f
        );
    }
}
