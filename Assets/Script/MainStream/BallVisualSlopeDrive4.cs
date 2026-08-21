using UnityEngine;
 using Sirenix.OdinInspector;
[Searchable]
[RequireComponent(typeof(Rigidbody))]
public class BallVisualSlopeDrive4 : MonoBehaviour
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

    [Tooltip("極限ポイント離脱直後の上向き速度[m/s]。現在速度への加算量ではなく、再POP後のY速度そのもの")]
    [Min(0f)]
    [SerializeField] private float limitBounceDeltaV = 3f;

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
        if (useUnifiedPreLimitGuidance &&
            experimentPhase == LimitExperimentPhase.Waiting &&
            isFlat &&
            ShouldLaunchUnifiedPreLimitFromFlat())
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
                if (!useUnifiedPreLimitGuidance ||
                    preLimitAllowSlopeEntryFallbackLaunch)
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

        if (col.transform.CompareTag("plane"))
            Debug.Log("");

        Debug.Log("");
    }

    void OnTriggerEnter(Collider col)
    {
        Debug.Log("");
    }

    void OnCollisionStay(Collision col)
    {
        if (col.transform.CompareTag("stairway"))
        {
            Debug.Log("");
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
        if (!slopeStick ||
            !slopeStick.BallVisualHasForwardSlope)
        {
            return false;
        }

        float distance =
            slopeStick.BallVisualDistanceToNextSlopeEntry;

        if (float.IsNaN(distance) ||
            float.IsInfinity(distance) ||
            distance < 0f)
        {
            return false;
        }

        Vector3 subjectVelocity =
            respondSubject.Mappedvelocity;

        Vector3 mappedTangent =
            slopeStick.BallVisualSlopeTangent;

        if (respondSubject != null &&
            mappedTangent.sqrMagnitude > 0.000001f)
        {
            mappedTangent =
                respondSubject.MapDirection(mappedTangent);
        }

        Vector3 flatDirection =
            mappedTangent.sqrMagnitude > 0.000001f
                ? Vector3.ProjectOnPlane(
                    mappedTangent,
                    Vector3.up
                  ).normalized
                : Vector3.ProjectOnPlane(
                    subjectVelocity,
                    Vector3.up
                  ).normalized;

        float forwardSpeed =
            flatDirection.sqrMagnitude > 0.000001f
                ? Mathf.Max(
                    0.10f,
                    Vector3.Dot(
                        subjectVelocity,
                        flatDirection
                    )
                  )
                : Mathf.Max(
                    0.10f,
                    Vector3.ProjectOnPlane(
                        subjectVelocity,
                        Vector3.up
                    ).magnitude
                  );

        float timeToEntry =
            distance / forwardSpeed;

        return
            timeToEntry <=
            preLimitLaunchLeadSeconds;
    }

    private void BeginUnifiedPreLimitGuidance()
    {
        if (experimentPhase !=
            LimitExperimentPhase.ApproachingLimit)
        {
            return;
        }

        if (ballBody.isKinematic)
            ballBody.isKinematic = false;

        ballBody.useGravity = true;

        // FlatでSubjectと一致していたVisual座標からPOPする。
        // 書き換えるのはBallVisualだけ。
        Vector3 subjectPosition =
            respondSubject.MappedPosition;

        Vector3 subjectVelocity =
            respondSubject.Mappedvelocity;

        ballBody.position =
            subjectPosition;

        // FlatでBallVisual自身が蓄積した回転姿勢をそのまま保持する。
        // SubjectのMappedRotationへ戻すと、それまでの転がり量が消えるため
        // Pre-Limit POPではrotationを書き換えない。

        Vector3 planarVelocity =
            Vector3.ProjectOnPlane(
                subjectVelocity,
                Vector3.up
            );

        Vector3 launchVelocity =
            planarVelocity +
            Vector3.up *
            preLimitLaunchUpSpeed;

        ballBody.velocity =
            launchVelocity;

        // Flat上で蓄積した転がり角速度を、そのまま空中へ引き継ぐ。
        // lockedRollingAngularVelocityが無い場合だけ、
        // POP直前の水平速度から転がり角速度を再構築する。
        Vector3 inheritedAngularVelocity =
            lockedRollingAngularVelocity;

        if (inheritedAngularVelocity.sqrMagnitude <= 0.000001f)
        {
            inheritedAngularVelocity =
                CalculateRollingAngularVelocity(
                    planarVelocity,
                    Vector3.up
                );
        }

        ballBody.angularVelocity =
            inheritedAngularVelocity;

        debugTargetAngularVelocity =
            inheritedAngularVelocity;

        debugAppliedAngularAcceleration =
            Vector3.zero;

        // Pre-Limitの空中区間ではTorqueを追加せず、
        // Rigidbodyの角運動量として自然に回転を継続させる。
        debugRollingWeight = 0f;

        if (preLimitUseTriggerDuringFlight &&
            ballCollider != null)
        {
            ballCollider.isTrigger = true;
        }

        preLimitGuidanceStartTime =
            Time.fixedTime;

        preLimitElapsed = 0f;
        preLimitApexTime = -1f;
        preLimitUpSpeed = launchVelocity.y;
        preLimitHadPositiveUpSpeed =
            launchVelocity.y >
            preLimitApexVerticalSpeedThreshold;

        preLimitAscentAccelerationState =
            Vector3.zero;

        preLimitMissileAccelerationState =
            Vector3.zero;

        experimentPhase =
            LimitExperimentPhase.PreLimitAscent;

        UpdateUnifiedPreLimitTimeTask(
            slopeStick.BallVisualIsOnFlat,
            slopeStick.BallVisualIsOnSlope
        );

        SetPhase(
            VisualPhase.SlopeControlled,
            "UnifiedPreLimitPOP"
        );

        Debug.Log(
            $"[UNIFIED PRE LIMIT] Begin " +
            $"time={preLimitGuidanceStartTime:F4} " +
            $"launchVelocity={launchVelocity:F4} " +
            $"Tgo={preLimitTimeToGo:F4}s " +
            $"deadline={preLimitPredictedDeadlineTime:F4} " +
            $"subjectPos={subjectPosition:F4} " +
            $"subjectVel={subjectVelocity:F4} " +
            $"forwardSlope={slopeStick.BallVisualHasForwardSlope} " +
            $"distanceToEntry={slopeStick.BallVisualDistanceToNextSlopeEntry:F4}",
            this
        );
    }

    private void UpdateUnifiedPreLimitTimeTask(
        bool isFlat,
        bool isOnSlope)
    {
        float dt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f
            );

        float now =
            Time.fixedTime;

        Vector3 subjectPosition =
            respondSubject.MappedPosition;

        Vector3 subjectVelocity =
            respondSubject.Mappedvelocity;

        float rawTimeToGo = -1f;
        float currentError =
            slopeStick.slopeProgressErrorPercent;

        bool slopeProgressActive =
            isOnSlope ||
            slopeStick.ReadyForLimitCapture;

        float rawProgressRate = 0f;
        bool validProgressRate = false;

        Vector3 mappedSlopeTangent =
            slopeStick.BallVisualSlopeTangent;

        if (respondSubject != null &&
            mappedSlopeTangent.sqrMagnitude > 0.000001f)
        {
            mappedSlopeTangent =
                respondSubject.MapDirection(
                    mappedSlopeTangent
                );
        }

        if (mappedSlopeTangent.sqrMagnitude > 0.000001f)
            mappedSlopeTangent.Normalize();

        // -------------------------------------------------------------
        // 1) Subject Progressを時計として使う。
        //    error : 負側 -> -0 -> 0
        // -------------------------------------------------------------
        if (slopeProgressActive)
        {
            if (preLimitHasProgressSample)
            {
                float sampleDt =
                    Mathf.Max(
                        0.000001f,
                        now -
                        preLimitLastProgressSampleTime
                    );

                rawProgressRate =
                    (currentError -
                     preLimitLastProgressError) /
                    sampleDt;

                validProgressRate =
                    rawProgressRate >=
                        preLimitMinimumProgressRatePercentPerSecond &&
                    rawProgressRate <= 1000f &&
                    !float.IsNaN(rawProgressRate) &&
                    !float.IsInfinity(rawProgressRate);

                if (validProgressRate)
                {
                    float rateBlend =
                        1f -
                        Mathf.Exp(
                            -preLimitProgressRateSmoothing *
                            dt
                        );

                    preLimitSmoothedProgressRate =
                        preLimitSmoothedProgressRate > 0f
                            ? Mathf.Lerp(
                                preLimitSmoothedProgressRate,
                                rawProgressRate,
                                rateBlend
                              )
                            : rawProgressRate;
                }
            }

            if (currentError < 0f &&
                preLimitSmoothedProgressRate >=
                    preLimitMinimumProgressRatePercentPerSecond)
            {
                rawTimeToGo =
                    (-currentError) /
                    preLimitSmoothedProgressRate;
            }
        }

        // -------------------------------------------------------------
        // 2) 「1% = 何mか」を実測する。
        //
        //    ここがSlope長さ1倍/5倍/10倍を吸収する部分。
        //    長さ5倍なら同じ1%でも移動距離が約5倍になるため、
        //    metersPerProgressPercentも自動的に約5倍になる。
        // -------------------------------------------------------------
        if (slopeProgressActive &&
            currentError < 0f &&
            mappedSlopeTangent.sqrMagnitude > 0.000001f)
        {
            float metersPerPercentSample = -1f;

            // A. Progress速度と実Subject速度から推定。
            if (preLimitSmoothedProgressRate >=
                preLimitMinimumProgressRatePercentPerSecond)
            {
                float subjectSpeedAlongSlope =
                    Mathf.Max(
                        0f,
                        Vector3.Dot(
                            subjectVelocity,
                            mappedSlopeTangent
                        )
                    );

                if (subjectSpeedAlongSlope > 0.0001f)
                {
                    metersPerPercentSample =
                        subjectSpeedAlongSlope /
                        preLimitSmoothedProgressRate;
                }
            }

            // B. 前FixedUpdateの実移動量 / Progress変化からも推定。
            //    Aより直接的なので、妥当ならこちらを優先する。
            if (preLimitHasMappedSubjectPositionSample &&
                preLimitHasProgressSample)
            {
                float deltaProgress =
                    currentError -
                    preLimitLastProgressError;

                Vector3 subjectDelta =
                    subjectPosition -
                    preLimitLastMappedSubjectPosition;

                float alongDistance =
                    Vector3.Dot(
                        subjectDelta,
                        mappedSlopeTangent
                    );

                if (deltaProgress > 0.00001f &&
                    alongDistance > 0.000001f)
                {
                    float directSample =
                        alongDistance /
                        deltaProgress;

                    if (!float.IsNaN(directSample) &&
                        !float.IsInfinity(directSample) &&
                        directSample > 0f)
                    {
                        metersPerPercentSample =
                            directSample;
                    }
                }
            }

            if (metersPerPercentSample > 0f &&
                !float.IsNaN(metersPerPercentSample) &&
                !float.IsInfinity(metersPerPercentSample))
            {
                float scaleBlend =
                    1f -
                    Mathf.Exp(
                        -preLimitProgressDistanceScaleSmoothing *
                        dt
                    );

                preLimitMetersPerProgressPercent =
                    preLimitMetersPerProgressPercent > 0f
                        ? Mathf.Lerp(
                            preLimitMetersPerProgressPercent,
                            metersPerPercentSample,
                            scaleBlend
                          )
                        : metersPerPercentSample;
            }

            // -0までの残距離 = 残Progress[%] * 実測[m/%]
            if (preLimitMetersPerProgressPercent > 0f)
            {
                float remainingPercent =
                    Mathf.Max(
                        0f,
                        -currentError
                    );

                preLimitEstimatedRemainingDistance =
                    remainingPercent *
                    preLimitMetersPerProgressPercent;

                // 100%区間へ換算した推定Slope長。
                preLimitEstimatedSlopeLength =
                    100f *
                    preLimitMetersPerProgressPercent;

                Vector3 rawZeroTarget =
                    subjectPosition +
                    mappedSlopeTangent *
                    preLimitEstimatedRemainingDistance;

                if (!preLimitZeroTargetValid)
                {
                    preLimitZeroTargetPosition =
                        rawZeroTarget;

                    preLimitZeroTargetValid =
                        true;
                }
                else
                {
                    // -0直前では平滑化遅れを残さず、
                    // 実測から得た最新の0位置へ直接合わせる。
                    // remainingPercent はこの外側スコープで既に計算済みなので再利用する。
                    if (remainingPercent <=
                        preLimitZeroTargetSnapPercent)
                    {
                        preLimitZeroTargetPosition =
                            rawZeroTarget;
                    }
                    else
                    {
                        float targetBlend =
                            1f -
                            Mathf.Exp(
                                -preLimitZeroTargetSmoothing *
                                dt
                            );

                        preLimitZeroTargetPosition =
                            Vector3.Lerp(
                                preLimitZeroTargetPosition,
                                rawZeroTarget,
                                targetBlend
                            );
                    }
                }
            }
        }

        // -------------------------------------------------------------
        // 3) Flat上では既存Forward Slope距離から入口時間を作る。
        //    Slope長そのものは、SubjectがSlopeへ入った後に2)で実測する。
        // -------------------------------------------------------------
        if (rawTimeToGo <= 0f &&
            isFlat &&
            slopeStick.BallVisualHasForwardSlope)
        {
            float distance =
                Mathf.Max(
                    0f,
                    slopeStick.BallVisualDistanceToNextSlopeEntry
                );

            Vector3 horizontalVelocity =
                Vector3.ProjectOnPlane(
                    subjectVelocity,
                    Vector3.up
                );

            float flatSpeed =
                Mathf.Max(
                    0.10f,
                    horizontalVelocity.magnitude
                );

            float timeToEntry =
                distance /
                flatSpeed;

            rawTimeToGo =
                timeToEntry +
                preLimitFallbackEntryToLimitSeconds;
        }

        // -------------------------------------------------------------
        // 4) Progress時計がまだ無い入口だけ既存deadlineを使う。
        // -------------------------------------------------------------
        if (rawTimeToGo <= 0f)
        {
            if (preLimitPredictedDeadlineTime > now)
            {
                rawTimeToGo =
                    preLimitPredictedDeadlineTime -
                    now;
            }
            else
            {
                rawTimeToGo =
                    preLimitFallbackEntryToLimitSeconds;
            }
        }

        // 長いSlopeでは通常の1.2s上限を自動延長する。
        // ただし暴走防止としてAbsolute上限は残す。
        float effectiveMaximumTimeToGo =
            Mathf.Max(
                preLimitMaximumTimeToGo,
                preLimitMinimumTimeToGo
            );

        if (slopeProgressActive &&
            currentError < 0f &&
            preLimitSmoothedProgressRate >=
                preLimitMinimumProgressRatePercentPerSecond)
        {
            float progressBasedTime =
                (-currentError) /
                preLimitSmoothedProgressRate;

            effectiveMaximumTimeToGo =
                Mathf.Max(
                    effectiveMaximumTimeToGo,
                    Mathf.Min(
                        preLimitAbsoluteMaximumTimeToGo,
                        progressBasedTime
                    )
                );
        }

        effectiveMaximumTimeToGo =
            Mathf.Min(
                effectiveMaximumTimeToGo,
                preLimitAbsoluteMaximumTimeToGo
            );

        rawTimeToGo =
            Mathf.Clamp(
                rawTimeToGo,
                preLimitMinimumTimeToGo,
                effectiveMaximumTimeToGo
            );

        float rawDeadline =
            now + rawTimeToGo;

        if (preLimitPredictedDeadlineTime < 0f)
        {
            preLimitPredictedDeadlineTime =
                rawDeadline;
        }
        else
        {
            float currentRemaining =
                Mathf.Max(
                    0f,
                    preLimitPredictedDeadlineTime -
                    now
                );

            // Flat時の短いFallback(例0.42s)で始めた後、
            // 実際は5倍Slopeで2s必要だった、と判明した場合は
            // 古い短Deadlineを引きずらず即座に延長する。
            bool discoveredLongerSlopeTask =
                slopeProgressActive &&
                preLimitSmoothedProgressRate >=
                    preLimitMinimumProgressRatePercentPerSecond &&
                rawTimeToGo >
                    currentRemaining +
                    Mathf.Max(0.05f, dt * 2f);

            if (discoveredLongerSlopeTask)
            {
                preLimitPredictedDeadlineTime =
                    rawDeadline;
            }
            else
            {
                float deadlineBlend =
                    1f -
                    Mathf.Exp(
                        -preLimitDeadlineSmoothing *
                        dt
                    );

                preLimitPredictedDeadlineTime =
                    Mathf.Lerp(
                        preLimitPredictedDeadlineTime,
                        rawDeadline,
                        deadlineBlend
                    );
            }
        }

        preLimitTimeToGo =
            Mathf.Clamp(
                preLimitPredictedDeadlineTime -
                now,
                preLimitMinimumTimeToGo,
                effectiveMaximumTimeToGo
            );

        // -------------------------------------------------------------
        // 5) Target Position
        //
        // 旧:
        //   SubjectPosition + SubjectVelocity * Tgo
        //
        // 新:
        //   SubjectのProgress実測から逆算した「-0そのものの位置」
        //
        // これによりSlope長さが5倍でもTargetが5倍先へ自動移動する。
        // -------------------------------------------------------------
        if (preLimitZeroTargetValid &&
            slopeProgressActive &&
            currentError < 0f)
        {
            preLimitTargetPosition =
                preLimitZeroTargetPosition;
        }
        else
        {
            // Flat / Progress未確定時だけ従来予測をFallbackとして使う。
            preLimitTargetPosition =
                subjectPosition +
                subjectVelocity *
                preLimitTimeToGo;
        }

        preLimitTargetVelocity =
            subjectVelocity;

        // 次FixedUpdateの直接距離サンプル用。
        preLimitLastMappedSubjectPosition =
            subjectPosition;

        preLimitHasMappedSubjectPositionSample =
            slopeProgressActive;

        // Progress履歴は、このメソッドの最後で更新する。
        // これにより上のdeltaProgressが「前回->今回」を正しく使える。
        if (slopeProgressActive)
        {
            preLimitLastProgressError =
                currentError;

            preLimitLastProgressSampleTime =
                now;

            preLimitHasProgressSample =
                true;
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
            $"[UNIFIED PRE LIMIT] MissileStart " +
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
        if (experimentPhase !=
            LimitExperimentPhase.PreLimitAscent)
        {
            return;
        }

        float dt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f
            );

        preLimitElapsed =
            Mathf.Max(
                0f,
                Time.fixedTime -
                preLimitGuidanceStartTime
            );

        Vector3 subjectPosition =
            respondSubject.MappedPosition;

        Vector3 subjectVelocity =
            respondSubject.Mappedvelocity;

        Vector3 positionError =
            subjectPosition -
            ballBody.position;

        Vector3 velocityError =
            subjectVelocity -
            ballBody.velocity;

        // 上下はPOP + Gravityに任せる。
        // 上昇中は平面方向だけを弱く補正して放物線を残す。
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
            preLimitAscentPositionGain +
            planarVelocityError *
            preLimitAscentVelocityGain;

        desiredAcceleration =
            Vector3.ClampMagnitude(
                desiredAcceleration,
                preLimitAscentMaximumAcceleration
            );

        preLimitAscentAccelerationState =
            Vector3.MoveTowards(
                preLimitAscentAccelerationState,
                desiredAcceleration,
                preLimitAscentMaximumJerk *
                dt
            );

        ballBody.AddForce(
            preLimitAscentAccelerationState,
            ForceMode.Acceleration
        );

        preLimitPeakAscentAcceleration =
            Mathf.Max(
                preLimitPeakAscentAcceleration,
                preLimitAscentAccelerationState.magnitude
            );

        preLimitUpSpeed =
            Vector3.Dot(
                ballBody.velocity,
                Vector3.up
            );

        if (preLimitUpSpeed >
            preLimitApexVerticalSpeedThreshold)
        {
            preLimitHadPositiveUpSpeed = true;
        }

        bool crossedApex =
            preLimitElapsed >=
                Mathf.Max(
                    Time.fixedDeltaTime * 0.5f,
                    0.000001f
                ) &&
            preLimitHadPositiveUpSpeed &&
            preLimitUpSpeed <=
                preLimitApexVerticalSpeedThreshold;

        bool deadlinePriority =
            preLimitTimeToGo <=
            preLimitForceMissileTimeToGo;

        bool ascentTimedOut =
            preLimitElapsed >=
            preLimitMaximumAscentSeconds;

        debugAppliedLinearAcceleration =
            preLimitAscentAccelerationState;

        debugAppliedStick = 0f;
        debugAppliedTangentialAcceleration = 0f;
        debugReleaseWeight = 0f;
        debugRejoinPositionWeight = 0f;

        if (crossedApex)
        {
            SwitchUnifiedPreLimitToMissile(
                "Apex"
            );
        }
        else if (deadlinePriority)
        {
            SwitchUnifiedPreLimitToMissile(
                "DeadlinePriority"
            );
        }
        else if (ascentTimedOut)
        {
            SwitchUnifiedPreLimitToMissile(
                "AscentTimeout"
            );
        }
    }

    private void ProcessUnifiedPreLimitChase()
    {
        if (experimentPhase !=
            LimitExperimentPhase.PreLimitChase)
        {
            return;
        }

        float dt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f
            );

        preLimitElapsed =
            Mathf.Max(
                0f,
                Time.fixedTime -
                preLimitGuidanceStartTime
            );

        float safeTimeToGo =
            Mathf.Max(
                preLimitTimeToGo,
                preLimitMinimumTimeToGo
            );

        Vector3 positionToDeadline =
            preLimitTargetPosition -
            ballBody.position;

        Vector3 velocityNow =
            ballBody.velocity;

        // p(T)=P, v(T)=V の3次終端誘導。
        // Tgoが小さくなるほど1/T^2項が強くなり、
        // Apex後に鋭いミサイル下降型へ移行する。
        Vector3 requiredTotalAcceleration =
            6f * positionToDeadline /
            (safeTimeToGo * safeTimeToGo) -
            (4f * velocityNow +
             2f * preLimitTargetVelocity) /
            safeTimeToGo;

        Vector3 desiredArtificialAcceleration;

        if (preLimitMissileUse3D)
        {
            // Rigidbody Gravityは別途Unityが加える。
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
                preLimitMissileMaximumAcceleration
            );

        preLimitMissileAccelerationState =
            Vector3.MoveTowards(
                preLimitMissileAccelerationState,
                desiredArtificialAcceleration,
                preLimitMissileMaximumJerk *
                dt
            );

        ballBody.AddForce(
            preLimitMissileAccelerationState,
            ForceMode.Acceleration
        );

        preLimitPeakMissileAcceleration =
            Mathf.Max(
                preLimitPeakMissileAcceleration,
                preLimitMissileAccelerationState.magnitude
            );

        debugAppliedLinearAcceleration =
            preLimitMissileAccelerationState;

        debugAppliedStick = 0f;
        debugAppliedTangentialAcceleration = 0f;
        debugReleaseWeight = 0f;
        debugRejoinPositionWeight =
            1f -
            Mathf.Clamp01(
                preLimitTimeToGo /
                Mathf.Max(
                    preLimitMaximumTimeToGo,
                    0.000001f
                )
            );

        if (enableDebugLog &&
            fixedFrameCounter %
            Mathf.Max(
                1,
                logEveryFixedFrames
            ) == 0)
        {
            Debug.Log(
                $"[UNIFIED PRE LIMIT MISSILE] " +
                $"elapsed={preLimitElapsed:F4}s " +
                $"Tgo={preLimitTimeToGo:F4}s " +
                $"deadline={preLimitPredictedDeadlineTime:F4} " +
                $"progressError={slopeStick.slopeProgressErrorPercent:F3}% " +
                $"progressRate={preLimitSmoothedProgressRate:F3}%/s " +
                $"metersPerPercent={preLimitMetersPerProgressPercent:F4} " +
                $"estimatedSlopeLength={preLimitEstimatedSlopeLength:F3}m " +
                $"remainingDistance={preLimitEstimatedRemainingDistance:F3}m " +
                $"zeroTargetValid={preLimitZeroTargetValid} " +
                $"accel={preLimitMissileAccelerationState.magnitude:F3} " +
                $"ballPos={ballBody.position:F4} " +
                $"targetPos={preLimitTargetPosition:F4} " +
                $"ballVel={ballBody.velocity:F4} " +
                $"targetVel={preLimitTargetVelocity:F4}",
                this
            );
        }
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

        if (!hasPreviousProgressSample)
        {
            previousProgressError = currentError;
            previousProgressTime = currentTime;
            previousProgressBallPosition = currentBallPosition;
            previousProgressBallVelocity = currentBallVelocity;
            hasPreviousProgressSample = true;
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

            // 30% crossingの補間時刻は診断値として保存する。
            // Post-Limitの実際の時計t=0は「BallVisualとSubjectを同期した今このFixedUpdate」
            // に統一する。これで過去の補間時刻と現在のRigidbody状態を混在させない。
            experimentEstimatedLimitCrossingTime =
                Mathf.Lerp(
                    previousProgressTime,
                    currentTime,
                    alpha
                );

            experimentLimitTime =
                currentTime;

            // 同期直前のBallVisual状態。
            Vector3 preSyncPosition =
                currentBallPosition;

            // Limit基準はCorrespondSubjectが返す現在のVisual側Subject状態。
            // Subject/CorrespondSubjectへは一切書き込まない。
            experimentLimitReferencePosition =
                respondSubject.MappedPosition;

            experimentLimitReferenceVelocity =
                respondSubject.Mappedvelocity;

            limitPreSyncPositionError =
                Vector3.Distance(
                    preSyncPosition,
                    experimentLimitReferencePosition
                );

            preLimitArrivalPositionError =
                limitPreSyncPositionError;

            preLimitArrivalVelocityError =
                Vector3.Distance(
                    currentBallVelocity,
                    experimentLimitReferenceVelocity
                );

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
                    $"predictedDeadline={preLimitPredictedDeadlineTime:F4} " +
                    $"deadlineError={(currentTime - preLimitPredictedDeadlineTime):F4}s " +
                    $"positionError={preLimitArrivalPositionError:F4} " +
                    $"velocityError={preLimitArrivalVelocityError:F4} " +
                    $"withinTolerance={preLimitArrivalWithinTolerance} " +
                    $"peakAscentAccel={preLimitPeakAscentAcceleration:F3} " +
                    $"peakMissileAccel={preLimitPeakMissileAcceleration:F3} " +
                    $"ballPos={currentBallPosition:F4} " +
                    $"subjectPos={experimentLimitReferencePosition:F4}",
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

        // Limitの同期境界では位置と並進速度だけをSubjectへ一致させる。
        // 回転系の所有権はBallVisualに残す。
        ballBody.position =
            experimentLimitReferencePosition;

        ballBody.velocity =
            experimentLimitReferenceVelocity;

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

        // SynchronizeBallExactlyAtLimit()直後なので、
        // velocityBeforeはSubjectのLimit状態と一致している。
        Vector3 velocityBefore =
            ballBody.velocity;

        // Subjectから受け継いだ水平速度を残し、
        // Y速度は「+Δv」ではなく必ず+limitBounceDeltaVへ設定する。
        Vector3 planarVelocity =
            Vector3.ProjectOnPlane(
                velocityBefore,
                Vector3.up
            );

        Vector3 plannedVelocityAfter =
            planarVelocity +
            Vector3.up * limitBounceDeltaV;

        Vector3 deltaVelocity =
            plannedVelocityAfter -
            velocityBefore;

        experimentEnergyBeforeBounce =
            CalculateTranslationalEnergy(
                velocityBefore
            );

        experimentPlannedEnergyAfterBounce =
            CalculateTranslationalEnergy(
                plannedVelocityAfter
            );

        experimentBounceAppliedTime =
            Time.fixedTime;

        ballBody.AddForce(
            deltaVelocity,
            ForceMode.VelocityChange
        );

        // RePOPではvelocityのY成分だけを変更する。
        // rotation / angularVelocityには触れず、
        // SynchronizeBallExactlyAtLimit()で保持した自然回転をそのまま継続する。
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
            postLimitUpSpeed =
                plannedVelocityAfter.y;

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
            // 旧A/B比較経路。
            experimentPhase =
                LimitExperimentPhase.Ballistic;
        }

        Debug.Log(
            $"[LIMIT REPOP] " +
            $"estimatedCrossingTime={experimentEstimatedLimitCrossingTime:F4} " +
            $"syncTime={experimentLimitTime:F4} " +
            $"detectionDelay={(experimentLimitTime - experimentEstimatedLimitCrossingTime):F4} " +
            $"bounceAppliedTime={experimentBounceAppliedTime:F4} " +
            $"preSyncError={limitPreSyncPositionError:F4} " +
            $"postSyncError={limitPostSyncPositionError:F6} " +
            $"ballPos={experimentLimitBallPosition:F4} " +
            $"syncedVelocity={velocityBefore:F4} " +
            $"plannedVelocityAfter={plannedVelocityAfter:F4} " +
            $"deltaV={deltaVelocity:F4} " +
            $"upSpeed={plannedVelocityAfter.y:F4} " +
            $"referencePos={experimentLimitReferencePosition:F4} " +
            $"referenceVel={experimentLimitReferenceVelocity:F4} " +
            $"KEBefore={experimentEnergyBeforeBounce:F4} " +
            $"KEPlannedAfter={experimentPlannedEnergyAfterBounce:F4} " +
            $"unified={useUnifiedPostLimitGuidance}",
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
                ballBody.velocity,
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