using UnityEngine;
using Sirenix.OdinInspector;

[Searchable]
[DefaultExecutionOrder(200)]
[RequireComponent(typeof(Rigidbody), typeof(SphereCollider))]
public class BallVisualSlopeDrive : MonoBehaviour
{
    public const string RuntimeBuildId = "BallVisualSlopeDrive-CleanTurnPolicy-v5";

    private enum MotionPhase
    {
        Waiting,
        Incident,
        MissileAscent,
        MissileChase,
        TerminalRejoin,
        ContinuousRejoin,
        Settled
    }
    
    public bool guttnJurge=false;

    [Header("References")]
    [SerializeField] private SlopeStickCore slopeCore;

    [SerializeField] private BallVisualEqualizerSync BallVisualEqualizer;

    [SerializeField] private CorrespondSubject respondSubject;

    [Header("Turn Landing Intent")]
    [Tooltip("IncidentのEnergy targetを旋回判定へREAD ONLY公開する保持時間[s]。")]
    [Min(0.1f)]
    [SerializeField] private float turnLandingIntentRetentionSeconds = 3.0f;

    [Header("1st POP - Incident Method")]
    [Tooltip("1st POPのSubject相対Apex高さ[m]")]
    [Min(0f)]
    [SerializeField] private float preLimitTargetHeightRelativeToSubject = 0.45f;

    [Header("Post Turn 1st POP Guard")]
    [Tooltip(
        "VisualPlayerRootの回転が完了した直後、最初のIncidentだけ" +
        "POP計算用のv0 sin(theta)を制限する。通常階段には適用しない。")]
    [SerializeField] private bool limitFirstPopAfterTurn = true;

    [Tooltip(
        "回転後1回だけ使用するPOP計算用Incident Normal Speed上限[m/s]。" +
        "異常なv0 sin(theta)だけを安全Clampする。通常階段には適用しない。")]
    [Min(0f)]
    [SerializeField] private float postTurnMaximumIncidentNormalSpeed = 2.40f;

    [Tooltip(
        "回転後1回だけ使用するBallVisualのSubject相対Up速度上限[m/s]。" +
        "回転後は通常の1.70m/s最低可視POPとDecorationを無効化し、" +
        "この値を直接の上限として小さな自然な浮きだけ残す。")]
    [Min(0f)]
    [SerializeField] private float postTurnMaximumRelativeUpSpeed = 0.50f;

    [Tooltip(
        "回転後最初のIncidentだけ、Equalizerへ渡すIncident Normal Energyも" +
        "現在のmaxGroundSpeedに比例した上限へ正規化する。通常Incidentには適用しない。")]
    [SerializeField] private bool normalizeEqualizerEnergyAfterTurn = true;

    [Tooltip(
        "回転後Equalizer Normal速度上限 = maxGroundSpeed × この比率。" +
        "既定値8/24=0.333333なので、maxGroundSpeed 24→8m/s、32→10.667m/s、64→21.333m/s。")]
    [Range(0.01f, 1f)]
    [SerializeField] private float postTurnEqualizerNormalSpeedRatioToMaxGround =
        8f / 24f;

    [Tooltip(
        "maxGroundSpeedのREAD ONLY取得に失敗した場合だけ使用するFallback[m/s]。" +
        "通常はBallVisualEqualizerSync経由で現在値を取得する。")]
    [Min(1f)]
    [SerializeField] private float postTurnFallbackMaxGroundSpeed = 24f;

    [Tooltip(
        "回転と判定するVisual Frameの累積角度[deg]。" +
        "微小なMapDirection揺れではGuardをArmしない。")]
    [Min(1f)]
    [SerializeField] private float postTurnDetectionMinimumAccumulatedAngleDeg = 20f;

    [Header("BallVisual -> Equalizer Canonical Coupling")]
    [Tooltip(
        "BallVisualから渡すSource Energyを1としたときの、" +
        "BallVisualEqualizer / NegativeEnvelope共通のStable N方向基準高さ[m]。" +
        "Equalizer加速度とEnvelope振幅はこの高さから自動導出します。")]
    [Min(0.01f)]
    [SerializeField] private float oscillationReferenceHeight = 0.45f;

    [Tooltip("h = eta * v^2 / (2g) の eta")]
    [Min(0f)]
    [SerializeField] private float firstPopVelocitySquaredRatio = 0.01533f;

    [Tooltip("v^2同期で生成する最小POP高さ[m]")]
    [Min(0f)]
    [SerializeField] private float minimumEnergyScaledPopHeight = 0.08f;

    [Tooltip("v^2同期で生成する最大POP高さ[m]")]
    [Min(0.01f)]
    [SerializeField] private float maximumEnergyScaledPopHeight = 1.50f;

    [Tooltip("入射法の平面位置ドリフト補正ゲイン")]
    [Min(0f)]
    [SerializeField] private float incidentPlanarPositionGain = 4f;

    [Tooltip("入射法の平面速度ドリフト補正ゲイン")]
    [Min(0f)]
    [SerializeField] private float incidentPlanarVelocityGain = 2f;

    [Tooltip("入射法の平面補正最大加速度[m/s^2]")]
    [Min(0f)]
    [SerializeField] private float incidentMaximumPlanarAcceleration = 20f;

    [Tooltip("入射法の平面補正最大Jerk[m/s^3]")]
    [Min(0f)]
    [SerializeField] private float incidentMaximumPlanarJerk = 220f;

    [Tooltip("Limit前はColliderをTriggerにして階段衝突で入射弧を崩さない")]
    [SerializeField] private bool incidentUseTriggerDuringFlight = true;

    [Header("2nd POP - Missile Method")]
    [Tooltip("Limitでミサイル降下法へ渡す上向きY速度[m/s]")]
    [Min(0f)]
    [SerializeField] private float secondPopUpSpeed = 3f;

    [Tooltip("2nd POP上昇中の平面位置ゲイン")]
    [Min(0f)]
    [SerializeField] private float missileAscentPositionGain = 5f;

    [Tooltip("2nd POP上昇中の平面速度ゲイン")]
    [Min(0f)]
    [SerializeField] private float missileAscentVelocityGain = 2f;

    [Tooltip("2nd POP上昇中の最大人工加速度[m/s^2]")]
    [Min(0f)]
    [SerializeField] private float missileAscentMaximumAcceleration = 24f;

    [Tooltip("2nd POP上昇中の最大Jerk[m/s^3]")]
    [Min(0f)]
    [SerializeField] private float missileAscentMaximumJerk = 240f;

    [Tooltip("2nd POP Apex判定の上向き速度[m/s]")]
    [Min(0f)]
    [SerializeField] private float missileApexVerticalSpeedThreshold = 0.10f;

    [Tooltip("2nd POP Apex検出待ち最大時間[s]")]
    [Min(0.05f)]
    [SerializeField] private float missileMaximumAscentSeconds = 0.60f;

    [Tooltip("Apex後のFuture Shadow位置ゲイン")]
    [Min(0f)]
    [SerializeField] private float missileChasePositionGain = 7f;

    [Tooltip("Apex後の速度ゲイン")]
    [Min(0f)]
    [SerializeField] private float missileChaseVelocityGain = 3f;

    [Tooltip("Apex後の最大人工加速度[m/s^2]")]
    [Min(0f)]
    [SerializeField] private float missileChaseMaximumAcceleration = 48f;

    [Tooltip("Apex後の最大Jerk[m/s^3]")]
    [Min(0f)]
    [SerializeField] private float missileChaseMaximumJerk = 420f;

    [Tooltip("Future Shadowの先読み時間[s]")]
    [Min(0f)]
    [SerializeField] private float missileChaseLeadSeconds = 0.18f;

    [Tooltip("Subject Flat分類を待つ最大時間[s]")]
    [Min(0.2f)]
    [SerializeField] private float missileMaximumWaitForSubjectFlatSeconds = 2.50f;

    [Header("Terminal Rejoin")]
    [Tooltip("Terminal開始から完全同期までの時間予算[s]")]
    [Min(0.05f)]
    [SerializeField] private float terminalTimeBudget = 0.30f;

    [Tooltip("Terminal Time-To-Goの最小値[s]")]
    [Min(0.01f)]
    [SerializeField] private float terminalMinimumTimeToGo = 0.04f;

    [Tooltip("Terminal最大人工加速度[m/s^2]")]
    [Min(0f)]
    [SerializeField] private float maximumTerminalAcceleration = 80f;

    [Tooltip("Terminal最大Jerk[m/s^3]")]
    [Min(0f)]
    [SerializeField] private float maximumTerminalJerk = 600f;

    [Tooltip("完全同期候補の位置誤差[m]")]
    [Min(0.001f)]
    [SerializeField] private float terminalPositionTolerance = 0.05f;

    [Tooltip("完全同期候補の速度誤差[m/s]")]
    [Min(0.001f)]
    [SerializeField] private float terminalVelocityTolerance = 0.20f;

    [Tooltip("同期条件を連続して満たすFixedUpdate数")]
    [Min(1)]
    [SerializeField] private int terminalStableFramesRequired = 2;

    [Header("Terminal Recovery / Continuous Relative Rejoin")]

    [Tooltip("Terminalの初期時間切れ後も、Acceleration/Jerk budget内で回収可能なら物理Terminalを延長します。")]
    [SerializeField] private bool enableTerminalFeasibilityRecovery = true;

    [Tooltip("通常terminalTimeBudgetに追加して許す物理Recovery時間[s]。ここまでで回収不能ならContinuous Relative Rejoinへ移行します。")]
    [Min(0f)]
    [SerializeField] private float maximumExtendedTerminalSeconds = 0.60f;

    [Tooltip("回収可能性を評価する候補Time-To-Go刻み[s]。")]
    [Range(0.01f, 0.10f)]
    [SerializeField] private float terminalRecoveryProbeStepSeconds = 0.04f;

    [Tooltip("maximumTerminalAccelerationの何割までをRecovery feasibleとして許すか。")]
    [Range(0.5f, 1f)]
    [SerializeField] private float terminalRecoveryAccelerationSafety01 = 0.92f;

    [Tooltip("候補時間の何割以内にJerk制約で必要加速度へ到達できることを要求するか。")]
    [Range(0.3f, 1f)]
    [SerializeField] private float terminalRecoveryJerkTimeSafety01 = 0.85f;

    [Tooltip("物理的に回収不能な場合、衝突を一時停止してSubject相対Hermite Recoveryへ移行します。")]
    [SerializeField] private bool enableEmergencyVisualRejoin = true;

    [Tooltip("Continuous Relative Hermiteの最短時間[s]。")]
    [Min(0.05f)]
    [SerializeField] private float emergencyVisualMinimumDuration = 0.18f;

    [Tooltip("Continuous Relative Hermiteの最長時間[s]。World終点の再予測/Retargetは行いません。")]
    [Min(0.10f)]
    [SerializeField] private float emergencyVisualMaximumDuration = 0.75f;

    [Tooltip("Continuous Rejoin時の見かけ上の相対回収速度目安[m/s]。相対距離からHermite時間を決めます。")]
    [Min(1f)]
    [SerializeField] private float emergencyVisualPreferredCatchUpSpeed = 28f;

    [Tooltip("最終Micro Sync位置許容[m]。0.01mの固定安全上限より小さい値だけ有効です。")]
    [Min(0.001f)]
    [SerializeField] private float emergencyVisualPositionTolerance = 0.03f;

    [Tooltip("最終Micro Sync速度許容[m/s]。0.05m/sの固定安全上限より小さい値だけ有効です。")]
    [Min(0.001f)]
    [SerializeField] private float emergencyVisualVelocityTolerance = 0.30f;

    [Header("Debug")]
    [SerializeField] private bool enableDebugLog = true;
    [Min(1)]
    [SerializeField] private int logEveryFixedFrames = 10;

    private Rigidbody ballBody;
    private Rigidbody inSubjectBody;
    private SphereCollider ballCollider;

    private MotionPhase motionPhase = MotionPhase.Waiting;
    private int fixedFrameCounter;

    // VisualPlayerRootの回転Tween中に、途中角度の座標系でIncidentを開始しない。
    // Inspector調整値にはせず、方式境界の安全条件として固定する。
    private const float VisualFrameStableAngleEpsilonDeg = 0.05f;
    private const int VisualFrameStableFixedFramesRequired = 2;

    // v4: 最終的な直接一致は浮動小数点の丸め相当だけ許す。
    private const float FinalMicroSyncPositionTolerance = 0.01f;
    private const float FinalMicroSyncVelocityTolerance = 0.05f;
    private Vector3 previousVisualFrameForward;
    private bool hasPreviousVisualFrameForward;
    private int visualFrameStableFrames;
    private bool visualFrameStable;

    // 回転TweenをVisual frameの累積角で検出し、
    // 回転完了後の「次の正常Incident 1回だけ」を規制対象にする。
    private float visualFrameTurnAccumulatedDegrees;
    private bool postTurnPopGuardPending;

    private Vector3 currentSurfaceNormal = Vector3.up;
    private Vector3 currentSlopeTangent = Vector3.forward;

    // ---------- Incident runtime ----------
    private float incidentStartTime = -1f;
    private float incidentElapsed;
    private Vector3 incidentPlanarAccelerationState;

    // 入射法の主原則:
    // 1) Exact LimitはSlopeStickCoreからREAD ONLYで受け取る。
    // 2) SlopeStick3D成功版と同じく、Natural Flight -> Time-Cost ->
    //    Visible Decoration の順で「読めるAir」を作る。
    // 3) Energy / XZ Separation はAirを最短化する目的ではなく、
    //    decorated flightが暴走しないための安全範囲として使う。
    // 4) Apex前後でYへ人工加速度を入れず、初速+Gravityだけで進む。
    // Inspector項目は追加しない。
    private const float IncidentMinimumFlightSeconds = 0.08f;
    private const float IncidentEnergyMinimumHeightRetention = 0.86f;
    private const float IncidentVisibilityMinimumRelativeUpSpeed = 1.70f;
    private const float IncidentPlanarSeparationHardRatio = 1.15f;
    private const float IncidentMaximumSpin = 30f;

    // SlopeStick3D成功版の「Time-Costを詰め切らず、POPとして読めるAirを
    // 30〜75msだけ返す」設計をSpline Exact Target上へ移植する。
    private const float IncidentPopDecorationPreferredUpRatio = 0.72f;
    private const float IncidentPopDecorationMinimumRelativeUpSpeed = 1.70f;
    private const float IncidentPopDecorationMaximumRelativeUpSpeed = 2.60f;
    private const float IncidentPopDecorationFlightRatio = 0.22f;
    private const float IncidentPopDecorationMinimumExtraSeconds = 0.030f;
    private const float IncidentPopDecorationMaximumExtraSeconds = 0.075f;

    public bool RawTimeJurge = false;

    private bool incidentPlanValid;
    private Vector3 incidentStartPosition;
    private Vector3 incidentLaunchVelocity;
    private Vector3 incidentImpactVelocity;
    private Vector3 incidentTargetPosition;
    private Vector3 incidentPlanarTravelDirection = Vector3.forward;
    private float incidentFlightSeconds;
    private float incidentAllowedPlanarSeparation;
    private float incidentMaximumObservedPlanarSeparation;

    private bool hasPreviousIncidentSample;
    private float previousIncidentTime;
    private Vector3 previousIncidentBallPosition;
    private Vector3 previousIncidentBallVelocity;
    private Vector3 previousIncidentSubjectPosition;
    private Vector3 previousIncidentSubjectVelocity;

    // ---------- Limit handoff runtime ----------
    private float limitCrossingTime = -1f;
    private float limitCrossingAlpha;
    private Vector3 limitCrossingPosition;
    private Vector3 limitIncomingVelocity;
    private Vector3 limitReferencePosition;
    private Vector3 limitReferenceVelocity;

    // ---------- Missile runtime ----------
    private float missileStartTime = -1f;
    private float missileApexTime = -1f;
    private float missileElapsed;
    private bool missileHadPositiveUpSpeed;
    private Vector3 missileAscentAccelerationState;
    private Vector3 missileChaseAccelerationState;
    private bool subjectFlatCaptured;
    private float subjectFlatTime = -1f;
    private bool ballFlatCaptured;
    private float ballFlatTime = -1f;

    // ---------- Terminal runtime ----------
    private float terminalStartTime = -1f;
    private float terminalElapsed;
    private float terminalTimeToGo;
    private Vector3 terminalAccelerationState;
    private Vector3 previousTerminalAccelerationState;
    private int terminalStableFrames;

    private float terminalActiveTimeBudget;
    private bool terminalExtendedRecoveryActive;
    private int terminalRecoveryExtensionCount;

    // ---------- Continuous Relative Rejoin runtime ----------
    // Recovery中はWorld終点を追いかけない。
    // VisualPlayerRoot座標系で「Subjectからの相対差」だけをHermiteで0へ収束させる。
    private float continuousRejoinStartTime = -1f;
    private float continuousRejoinDuration;

    private Vector3 continuousRejoinStartLocalOffset;
    private Vector3 continuousRejoinStartLocalVelocity;
    private Quaternion continuousRejoinStartRelativeRotation = Quaternion.identity;

    private Vector3 continuousRejoinCurrentLocalOffset;
    private Vector3 continuousRejoinCurrentLocalVelocity;
    private Vector3 continuousRejoinCurrentWorldPosition;
    private Vector3 continuousRejoinCurrentWorldVelocity;
    private Quaternion continuousRejoinCurrentWorldRotation = Quaternion.identity;

    // VisualPlayerRoot自体が回転しているときの輸送速度 omega x r を復元するためのsample。
    private bool hasContinuousFrameSample;
    private Vector3 previousContinuousFramePosition;
    private Quaternion previousContinuousFrameRotation = Quaternion.identity;
    private Vector3 continuousFrameLinearVelocityWorld;
    private Vector3 continuousFrameAngularVelocityWorld;

    // 1コマ不連続を数値で検出するための診断値。
    private bool hasContinuousTrajectorySample;
    private Vector3 previousContinuousWorldPosition;
    private Vector3 previousContinuousWorldVelocity;
    private float previousContinuousSampleTime = -1f;
    private float currentContinuityResidualMeters;
    private float maximumContinuityResidualMeters;

    private bool continuousRejoinWaitingForTurnEndLogged;

    public enum BallVisualPoseAuthority
    {
        Synchronized,
        PhysicalDrive,
        ContinuousRejoin
    }

    public BallVisualPoseAuthority PoseAuthority
    {
        get
        {
            if (motionPhase == MotionPhase.ContinuousRejoin)
                return BallVisualPoseAuthority.ContinuousRejoin;

            if (motionPhase == MotionPhase.Waiting ||
                motionPhase == MotionPhase.Settled)
            {
                return BallVisualPoseAuthority.Synchronized;
            }

            return BallVisualPoseAuthority.PhysicalDrive;
        }
    }

    // Waiting/Settled以外ではBallVisualSlopeDriveがPose権威を持つ。
    // CorrespondSubjectはこのtrueの間、BallVisualへ位置/回転/速度を書いてはいけない。
    public bool OwnsBallVisualPose =>
        PoseAuthority != BallVisualPoseAuthority.Synchronized;

    public bool IsContinuousRejoining =>
        motionPhase == MotionPhase.ContinuousRejoin;

    public float CurrentContinuityResidualMeters =>
        currentContinuityResidualMeters;

    public float MaximumContinuityResidualMeters =>
        maximumContinuityResidualMeters;

    public enum TurnLandingIntentSource
    {
        None,
        IncidentEnergyTarget,
        MissileFutureShadow
    }

    [System.Serializable]
    public struct TurnLandingIntent
    {
        public bool valid;
        public TurnLandingIntentSource source;
        public Vector3 positionVisual;
        public Vector3 velocityVisual;
        [Range(0f, 1f)] public float confidence01;
        public bool energyBacked;
        public float ageSeconds;
    }

    /// <summary>
    /// Turn planner用READ ONLY API。
    /// 最優先はIncidentで確定したEnergy target。
    /// それが期限切れの場合だけMissile Future Shadowをfallbackにする。
    /// このAPIはBallVisual / Subject / Equalizerの状態を書き換えない。
    /// </summary>
    public bool TryGetTurnLandingIntent(out TurnLandingIntent intent)
    {
        intent = default;

        float incidentAge =
            incidentStartTime >= 0f
                ? Mathf.Max(0f, Time.fixedTime - incidentStartTime)
                : float.PositiveInfinity;

        if (incidentPlanValid &&
            incidentAge <= Mathf.Max(0.1f, turnLandingIntentRetentionSeconds) &&
            IsFinite(incidentTargetPosition))
        {
            intent = new TurnLandingIntent
            {
                valid = true,
                source = TurnLandingIntentSource.IncidentEnergyTarget,
                positionVisual = incidentTargetPosition,
                velocityVisual = incidentImpactVelocity,
                confidence01 = 1f,
                energyBacked = true,
                ageSeconds = incidentAge
            };

            return true;
        }

        bool missilePhase =
            motionPhase == MotionPhase.MissileAscent ||
            motionPhase == MotionPhase.MissileChase ||
            motionPhase == MotionPhase.TerminalRejoin;

        if (missilePhase && respondSubject)
        {
            Vector3 subjectVelocity = ReadMappedInSubjectVelocity();
            Vector3 shadowTarget =
                respondSubject.MappedPosition +
                subjectVelocity * missileChaseLeadSeconds;

            if (IsFinite(shadowTarget))
            {
                intent = new TurnLandingIntent
                {
                    valid = true,
                    source = TurnLandingIntentSource.MissileFutureShadow,
                    positionVisual = shadowTarget,
                    velocityVisual = subjectVelocity,
                    confidence01 = 0.75f,
                    energyBacked = false,
                    ageSeconds = 0f
                };

                return true;
            }
        }

        return false;
    }

    public bool IsStableForFiveLineCorrection =>
        !OwnsBallVisualPose;

    // =====================================================================
    // BallVisual global trajectory ownership
    // =====================================================================
    //
    // Equalizer may temporarily express local Envelope / stair damping, but
    // BallVisual remains the only global path owner. Equalizer needs only the
    // current phase name for diagnostics; it no longer borrows BallVisual's
    // Terminal acceleration / jerk / time budget.
    public string GlobalTrajectoryPhase =>
        motionPhase.ToString();

    public float OscillationReferenceHeight =>
        Mathf.Max(
            0.01f,
            oscillationReferenceHeight);

    private void Awake()
    {
        ballBody = GetComponent<Rigidbody>();
        ballCollider = GetComponent<SphereCollider>();
    }

    private void Start()
    {
        // 実行中のファイル世代をログだけで確認できる識別子。
        Debug.Log($"[BALL VISUAL DRIVE BUILD] {RuntimeBuildId}", this);

        if (BallVisualEqualizer == null)
        {
            GameObject equalizerObject =
                GameObject.Find("/VisualPlayerRoot/BallVisualEqualizer");

            if (equalizerObject != null)
                BallVisualEqualizer =
                    equalizerObject.GetComponent<BallVisualEqualizerSync>();
        }
        if (respondSubject == null)
        {
            GameObject subjectVisual = GameObject.Find("subject");
            if (subjectVisual != null)
                respondSubject = subjectVisual.GetComponent<CorrespondSubject>();
        }

        if (slopeCore == null)
            slopeCore = FindFirstObjectByType<SlopeStickCore>();

        if (slopeCore != null)
            inSubjectBody = slopeCore.Body != null
                ? slopeCore.Body
                : slopeCore.GetComponent<Rigidbody>();

        if (ballBody == null || ballCollider == null || slopeCore == null || respondSubject == null || inSubjectBody == null)
        {
            Debug.LogError("[BALL VISUAL] 必要な参照が設定されていません。", this);
            enabled = false;
            return;
        }

        SyncCompletelyToSubject("InitialSync");
    }

    void Update()
    {
        if (BallVisualEqualizer != null)
            BallVisualEqualizer.Equalize();
    }
    private void FixedUpdate()
    {
        fixedFrameCounter++;
        UpdateControlBasis();
        UpdateVisualFrameStability();
        UpdateContinuousFrameAngularVelocitySample();

        bool isFlat = slopeCore.BallVisualIsOnFlat;
        bool isOnSlope = slopeCore.BallVisualIsOnSlope;
        bool canBeginIncident = CanBeginIncident(isOnSlope);

        // Continuous Rejoin中はBallVisualの位置権威をこのクラスだけが持つ。
        // CorrespondSubject / PhysX contact / Gravity / Slope Driveからの書込みは禁止。
        if (motionPhase == MotionPhase.ContinuousRejoin)
        {
            ProcessContinuousRejoin();
            WriteDebugLog();
            return;
        }

        // Settledは「次の有効Incident入口」まで完全同期区間。
        // 単にisOnSlopeになっただけでは抜けない。
        // これにより回転途中のArcSlab誤検出(progress > Target)で
        // Incidentを誤発火させない。
        if (motionPhase == MotionPhase.Settled)
        {
            if (!canBeginIncident)
            {
                HoldSettledSync();
                LogIncidentEntryGateIfNeeded(isOnSlope);
                WriteDebugLog();
                return;
            }

            motionPhase = MotionPhase.Waiting;
        }

        // Waitingは同期区間。
        // Incident開始条件を満たしたFixedUpdateだけIncidentEntry同期へ渡す。
        // 開始しない間はSubjectへ完全一致を維持する。
        if (motionPhase == MotionPhase.Waiting)
        {
            if (canBeginIncident)
                BeginIncidentMethod();
            else
                SyncCompletelyToSubject("WaitingSync");

            LogIncidentEntryGateIfNeeded(isOnSlope);
            WriteDebugLog();
            return;
        }

        // 入射法区間。位置/速度のSubject hard syncは禁止。
        if (motionPhase == MotionPhase.Incident)
        {
            ObserveIncidentLimitCrossing();

            if (motionPhase == MotionPhase.MissileAscent)
            {
                ProcessMissileAscent();
                WriteDebugLog();
                return;
            }

            ProcessIncident();
            WriteDebugLog();
            return;
        }

        // ミサイル降下法区間。Terminalへ入るまでhard syncしない。
        if (motionPhase == MotionPhase.MissileAscent)
        {
            ProcessMissileAscent();
            WriteDebugLog();
            return;
        }

        if (motionPhase == MotionPhase.MissileChase)
        {
            if (ShouldBeginTerminalRejoin(isFlat))
                BeginTerminalRejoin();

            if (motionPhase == MotionPhase.MissileChase)
                ProcessMissileChase();
            else
                ProcessTerminalRejoin();

            WriteDebugLog();
            return;
        }

        if (motionPhase == MotionPhase.TerminalRejoin)
        {
            ProcessTerminalRejoin();
            WriteDebugLog();
        }
    }

    // =====================================================================
    // Incident entry gate / visual-frame stability
    // =====================================================================

    private void UpdateVisualFrameStability()
    {
        if (respondSubject == null)
        {
            visualFrameStable = false;
            visualFrameStableFrames = 0;
            hasPreviousVisualFrameForward = false;
            visualFrameTurnAccumulatedDegrees = 0f;
            return;
        }

        Vector3 mappedForward =
            respondSubject.MapDirection(Vector3.forward);

        mappedForward =
            Vector3.ProjectOnPlane(mappedForward, Vector3.up);

        if (mappedForward.sqrMagnitude <= 0.000001f ||
            !IsFinite(mappedForward))
        {
            visualFrameStable = false;
            visualFrameStableFrames = 0;
            hasPreviousVisualFrameForward = false;
            visualFrameTurnAccumulatedDegrees = 0f;
            return;
        }

        mappedForward.Normalize();

        if (!hasPreviousVisualFrameForward)
        {
            previousVisualFrameForward = mappedForward;
            hasPreviousVisualFrameForward = true;
            visualFrameStableFrames = 0;
            visualFrameStable = false;
            visualFrameTurnAccumulatedDegrees = 0f;
            return;
        }

        bool wasStable = visualFrameStable;

        float angleChange =
            Vector3.Angle(
                previousVisualFrameForward,
                mappedForward);

        previousVisualFrameForward = mappedForward;

        if (angleChange <= VisualFrameStableAngleEpsilonDeg)
        {
            visualFrameStableFrames++;
        }
        else
        {
            // Tween中の1 FixedUpdateごとの角度を積算する。
            // 0.05deg以下の微小揺れは回転量として数えない。
            visualFrameTurnAccumulatedDegrees += angleChange;
            visualFrameStableFrames = 0;
        }

        visualFrameStable =
            visualFrameStableFrames >=
            VisualFrameStableFixedFramesRequired;

        // 回転中(false) -> 再安定(true)へ戻った瞬間だけArm。
        // Guardは次にBeginIncidentMethodが正常成立した1回だけ消費する。
        if (!wasStable &&
            visualFrameStable &&
            visualFrameTurnAccumulatedDegrees >=
                Mathf.Max(1f, postTurnDetectionMinimumAccumulatedAngleDeg))
        {
            postTurnPopGuardPending = true;

            if (enableDebugLog)
            {
                Debug.Log(
                    $"[POST TURN POP GUARD ARMED] " +
                    $"turnAccum={visualFrameTurnAccumulatedDegrees:F3}deg " +
                    $"normalSpeedCap={postTurnMaximumIncidentNormalSpeed:F3}m/s",
                    this);
            }

            visualFrameTurnAccumulatedDegrees = 0f;
        }
        else if (visualFrameStable &&
                 visualFrameTurnAccumulatedDegrees > 0f &&
                 visualFrameTurnAccumulatedDegrees <
                    Mathf.Max(1f, postTurnDetectionMinimumAccumulatedAngleDeg))
        {
            // 回転閾値未満の微小な座標系変化は持ち越さない。
            visualFrameTurnAccumulatedDegrees = 0f;
        }
    }

    private bool CanBeginIncident(bool isOnSlope)
    {
        if (!isOnSlope)
            return false;

        if (!slopeCore.BallVisualHasActiveSlopeFrame)
            return false;

        // Core must have a stable same-section Spline plan with an exact
        // Target Progress sample before the successful Incident formula starts.
        if (!slopeCore.BallVisualIncidentReady)
            return false;

        // Coreが旋回後の新Splineを捕捉するまではIncidentを開始しない。
        // 旧branch tangentで入射法を組み立てることを防ぐ。
        if (slopeCore.IsWaitingForTurnGuide)
            return false;

        // Target Progressより後ろ側(+側)からIncidentを撃たない。
        // 右回転直後にArcSlabがprogress=100%としてSlope判定された
        // ケースをここで除外する。
        if (!(slopeCore.slopeProgressErrorPercent < 0f))
            return false;

        // VisualPlayerRootのTween回転中はMapDirection/Targetの座標系が
        // FixedUpdateごとに変化するため、回転が安定するまで開始しない。
        if (!visualFrameStable)
            return false;

        return true;
    }

    private void LogIncidentEntryGateIfNeeded(bool isOnSlope)
    {
        if (!enableDebugLog ||
            !isOnSlope ||
            !slopeCore.BallVisualHasActiveSlopeFrame ||
            fixedFrameCounter % Mathf.Max(1, logEveryFixedFrames) != 0)
        {
            return;
        }

        Vector3 mappedForward =
            respondSubject != null
                ? respondSubject.MapDirection(Vector3.forward)
                : Vector3.forward;

        Debug.Log(
            $"[INCIDENT ENTRY GATE] " +
            $"allowed={CanBeginIncident(isOnSlope)} " +
            $"progressError={slopeCore.slopeProgressErrorPercent:F3}% " +
            $"coreIncidentReady={slopeCore.BallVisualIncidentReady} " +
            $"visualFrameStable={visualFrameStable} " +
            $"coreTurnGuideWait={slopeCore.IsWaitingForTurnGuide} " +
            $"stableFrames={visualFrameStableFrames}/{VisualFrameStableFixedFramesRequired} " +
            $"mappedForward={mappedForward:F4}",
            this);
    }

    // =====================================================================
    // Synchronization ownership
    // =====================================================================

    private void SyncCompletelyToSubject(string reason)
    {
        if (ballBody.isKinematic)
            ballBody.isKinematic = false;

        ballBody.useGravity = false;
        ballBody.position = respondSubject.MappedPosition;
        ballBody.velocity = ReadMappedInSubjectVelocity();
        ballBody.rotation = respondSubject.MappedRotation;
        ballBody.angularVelocity = respondSubject.MapDirection(inSubjectBody.angularVelocity);

        if (ballCollider != null)
            ballCollider.isTrigger = true;

        if (enableDebugLog && reason != "WaitingSync")
        {
            Debug.Log(
                $"[SYNC CHECKPOINT] reason={reason} " +
                $"time={Time.fixedTime:F4} " +
                $"position={ballBody.position:F4} " +
                $"velocity={ballBody.velocity:F4}",
                this);
        }
    }

    private void HoldSettledSync()
    {
        SyncCompletelyToSubject("WaitingSync");
    }

    private Vector3 ReadMappedInSubjectVelocity()
    {
        // SlopeStickCore / InSubjectはREAD ONLY。
        // 同一FixedUpdateでSlopeStickCoreが更新した最新Rigidbody速度をVisual座標へ写す。
        return respondSubject.MapDirection(inSubjectBody.velocity);
    }

    // =====================================================================
    // 1st POP / Incident Method
    // =====================================================================

    private void ResolveIncidentVelocityComponents(
        Vector3 velocity,
        out float v0,
        out float tangentSpeed,
        out float inwardNormalSpeed,
        out float incidenceAngleDeg,
        out Vector3 tangentVelocity,
        out Vector3 inwardNormalVelocity)
    {
        Vector3 normal =
            currentSurfaceNormal.sqrMagnitude > 0.000001f
                ? currentSurfaceNormal.normalized
                : Vector3.up;

        Vector3 tangent =
            currentSlopeTangent.sqrMagnitude > 0.000001f
                ? currentSlopeTangent.normalized
                : Vector3.ProjectOnPlane(
                    velocity,
                    normal);

        // tangent と normal を必ず直交させる。
        tangent =
            Vector3.ProjectOnPlane(
                tangent,
                normal);

        if (tangent.sqrMagnitude <= 0.000001f)
        {
            tangent =
                Vector3.ProjectOnPlane(
                    velocity,
                    normal);
        }

        if (tangent.sqrMagnitude <= 0.000001f)
        {
            Vector3 fallbackForward =
                respondSubject != null
                    ? respondSubject.MapDirection(Vector3.forward)
                    : Vector3.forward;

            tangent =
                Vector3.ProjectOnPlane(
                    fallbackForward,
                    normal);
        }

        if (tangent.sqrMagnitude <= 0.000001f)
            tangent = Vector3.forward;
        else
            tangent.Normalize();

        // 斜面接線を実際の進行方向へ揃える。
        if (Vector3.Dot(tangent, velocity) < 0f)
            tangent = -tangent;

        v0 =
            velocity.magnitude;

        // v0 cos(theta)
        tangentSpeed =
            Vector3.Dot(
                velocity,
                tangent);

        // 外向きnormalに対して、
        // Colliderへ食い込む速度だけを正値として取る。
        //
        // v0 sin(theta) = max(0, -v dot n)
        inwardNormalSpeed =
            Mathf.Max(
                0f,
                -Vector3.Dot(
                    velocity,
                    normal));

        tangentVelocity =
            tangent *
            tangentSpeed;

        inwardNormalVelocity =
            -normal *
            inwardNormalSpeed;

        incidenceAngleDeg =
            Mathf.Atan2(
                inwardNormalSpeed,
                Mathf.Max(
                    0.0001f,
                    Mathf.Abs(tangentSpeed))) *
            Mathf.Rad2Deg;
    }


    private void BeginIncidentMethod()
    {
        if (motionPhase != MotionPhase.Waiting ||
            !CanBeginIncident(slopeCore.BallVisualIsOnSlope) ||
            !TryResolveExactIncidentTarget(out Vector3 exactTarget))
        {
            return;
        }

        // 方式入口だけで完全同期。LimitまではBallVisualSlopeDriveが軌道を所有する。
        SyncCompletelyToSubject("IncidentEntry");
        ballBody.useGravity = true;

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();

        // ================================================================
        // Flat -> Slope incident decomposition
        //
        // v0
        // ├─ v0 cos(theta) : slope-tangent component
        // └─ v0 sin(theta) : inward surface-normal component
        //
        // currentSurfaceNormal / currentSlopeTangent are already mapped
        // into VisualPlayerRoot coordinates by UpdateControlBasis().
        // ================================================================

        ResolveIncidentVelocityComponents(
            subjectVelocity,
            out float incidentV0,
            out float incidentTangentSpeed,
            out float incidentNormalSpeed,
            out float incidentAngleDeg,
            out Vector3 incidentTangentVelocity,
            out Vector3 incidentInwardNormalVelocity);

        // ================================================================
        // Post-turn 1st POP guard
        //
        // 回転後の最初のIncidentだけ、BallVisual POP計算に使う
        // Normal成分を上限規制する。
        // Equalizer Energy側は後段でmaxGroundSpeed正規化した別の速度を使う。
        // subjectVelocity / incidentNormalSpeed本体は変更しないため、
        // InSubjectや斜面物理の運動量は一切書き換えない。
        // ================================================================
        bool usePostTurnPopGuard =
            limitFirstPopAfterTurn &&
            postTurnPopGuardPending;

        float incidentNormalSpeedForPop =
            usePostTurnPopGuard
                ? Mathf.Min(
                    incidentNormalSpeed,
                    Mathf.Max(0f, postTurnMaximumIncidentNormalSpeed))
                : incidentNormalSpeed;

        Vector3 planarDisplacement = Vector3.ProjectOnPlane(
            exactTarget - subjectPosition,
            Vector3.up);

        float referenceSpeed = ResolveSubjectReferenceSpeed();

        // Colliderへ食い込む v0 sin(theta) を、
        // 衝突後に外向き法線へ返したときの world-up 成分。
        //
        // これを1st Popの第一候補とする。
        Vector3 reflectedNormalVelocity =
            currentSurfaceNormal.normalized *
            incidentNormalSpeedForPop;

        float incidenceDerivedRelativeUp =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    reflectedNormalVelocity,
                    Vector3.up));

        // 入射法線成分が成立しないケース
        // （下り斜面・すでに斜面接線へ整列済み等）では、
        // 既存の v^2 同期則へFallbackしてIncidentを失わない。
        float rawPreferredRelativeUp =
            incidenceDerivedRelativeUp > 0.0001f
                ? incidenceDerivedRelativeUp
                : ResolveIncidentPreferredRelativeUpSpeed(
                    referenceSpeed);

        // 回転直後だけ、通常のPOP装飾へ入る前にSubject相対Upを直接規制する。
        float preferredRelativeUp =
            usePostTurnPopGuard
                ? Mathf.Min(
                    rawPreferredRelativeUp,
                    Mathf.Max(0f, postTurnMaximumRelativeUpSpeed))
                : rawPreferredRelativeUp;

        float preferredHeight = RelativeUpToHeight(preferredRelativeUp);

        if (enableDebugLog)
        {
            float reconstructedSpeed =
                Mathf.Sqrt(
                    incidentTangentSpeed * incidentTangentSpeed +
                    incidentNormalSpeed * incidentNormalSpeed);

            Debug.Log(
                $"[INCIDENT DECOMPOSITION] " +
                $"v0={incidentV0:F4}m/s " +
                $"v0cos={incidentTangentSpeed:F4}m/s " +
                $"v0sin={incidentNormalSpeed:F4}m/s " +
                $"popV0sin={incidentNormalSpeedForPop:F4}m/s " +
                $"postTurnGuard={usePostTurnPopGuard} " +
                $"angle={incidentAngleDeg:F3}deg " +
                $"reconstructed={reconstructedSpeed:F4}m/s " +
                $"tangentVelocity={incidentTangentVelocity:F4} " +
                $"inwardNormalVelocity={incidentInwardNormalVelocity:F4} " +
                $"reflectedNormalVelocity={reflectedNormalVelocity:F4} " +
                $"incidenceDerivedUp={incidenceDerivedRelativeUp:F4}m/s " +
                $"rawPreferredUp={rawPreferredRelativeUp:F4}m/s " +
                $"preferredUp={preferredRelativeUp:F4}m/s " +
                $"surfaceNormal={currentSurfaceNormal:F4} " +
                $"slopeTangent={currentSlopeTangent:F4}",
                this);
        }

        // ================================================================
        // SlopeStick3D-success style Air timing on top of Spline Exact Target
        // ================================================================

        // まず入射法そのものが要求する自然な放物線時間。
        float naturalFlight = SolveIncidentTimeFromRelativeUp(
            subjectPosition.y,
            exactTarget.y,
            subjectVelocity.y,
            preferredRelativeUp);

        if (!IsValidPositiveTime(naturalFlight))
            return;

        // Subjectが同じTargetへ進む代表ETAを使ったTime-Cost。
        // 現在Spline版の「最大速度から作る最低時間」ではなく、
        // 成功版と同じくSubject自身のclosing speedを主基準にする。
        float timeCostFlight = ResolveIncidentTimeCostFlightSeconds(
            subjectPosition,
            exactTarget,
            subjectVelocity,
            naturalFlight);

        // Time-Costを詰め切らず、30〜75msの範囲だけ
        // POPとして読めるAir余白を戻す。
        // 回転直後は「POPとして読ませるためのAir余白」を追加しない。
        float decoratedFlight =
            usePostTurnPopGuard
                ? naturalFlight
                : ResolveIncidentDecoratedFlightSeconds(
                    subjectPosition,
                    exactTarget,
                    subjectVelocity,
                    preferredRelativeUp,
                    timeCostFlight,
                    naturalFlight);

        // 1〜2 FixedUpdateしかない極端な近距離Limitは
        // 視認用Energyを追加しない。
        bool extremelyShortIncident =
            naturalFlight <= Mathf.Max(0.05f, Time.fixedDeltaTime * 2f);

        // 回転後Guardでは通常の「最低可視POP=1.70m/s」を無効化する。
        float visibleRelativeUp =
            (extremelyShortIncident || usePostTurnPopGuard)
                ? 0f
                : IncidentVisibilityMinimumRelativeUpSpeed;

        float minimumRelativeUp;
        float maximumRelativeUp;

        if (usePostTurnPopGuard)
        {
            // 回転後だけEnergy許容幅を一点に固定し、
            // Separation/Decorationが1.70〜2.60m/sへ再膨張させることを禁止する。
            minimumRelativeUp = preferredRelativeUp;
            maximumRelativeUp = preferredRelativeUp;
        }
        else
        {
            // 通常階段は従来どおり。
            minimumRelativeUp = Mathf.Max(
                visibleRelativeUp,
                preferredRelativeUp *
                Mathf.Sqrt(IncidentEnergyMinimumHeightRetention));

            maximumRelativeUp = extremelyShortIncident
                ? preferredRelativeUp
                : Mathf.Max(
                    preferredRelativeUp,
                    IncidentPopDecorationMaximumRelativeUpSpeed);
        }

        float energyTimeA = SolveIncidentTimeFromRelativeUp(
            subjectPosition.y,
            exactTarget.y,
            subjectVelocity.y,
            minimumRelativeUp);

        float energyTimeB = SolveIncidentTimeFromRelativeUp(
            subjectPosition.y,
            exactTarget.y,
            subjectVelocity.y,
            maximumRelativeUp);

        if (!IsValidPositiveTime(energyTimeA) ||
            !IsValidPositiveTime(energyTimeB))
        {
            return;
        }

        float energyTimeMin = Mathf.Min(energyTimeA, energyTimeB);
        float energyTimeMax = Mathf.Max(energyTimeA, energyTimeB);

        Vector3 forecastPlanarVelocity = ResolveIncidentForecastPlanarVelocity(
            planarDisplacement,
            subjectVelocity);

        incidentAllowedPlanarSeparation =
            ResolveIncidentAllowedPlanarSeparation(subjectVelocity);

        bool hasSeparationRange = TrySolveIncidentSeparationTimeRange(
            planarDisplacement,
            forecastPlanarVelocity,
            incidentAllowedPlanarSeparation,
            out float separationTimeMin,
            out float separationTimeMax,
            out float separationOptimalTime);

        // 数値安全上の最低時間。Naturalがそれより短ければNaturalを尊重する。
        float numericalMinimum = Mathf.Min(
            naturalFlight,
            Mathf.Max(
                IncidentMinimumFlightSeconds,
                Time.fixedDeltaTime * 2f));

        float baseMin = Mathf.Max(
            numericalMinimum,
            energyTimeMin);

        float baseMax = energyTimeMax;

        bool energyConflict = baseMin > baseMax;

        // まず成功版のdecorated Airを採用する。
        // Energyは「最短Tを選ぶ材料」ではなく安全Clampとして働く。
        float flightSeconds = energyConflict
            ? baseMax
            : Mathf.Clamp(
                decoratedFlight,
                baseMin,
                baseMax);

        bool separationConflict = false;

        if (!energyConflict && hasSeparationRange)
        {
            float safeMin = Mathf.Max(baseMin, separationTimeMin);
            float safeMax = Mathf.Min(baseMax, separationTimeMax);

            if (safeMin <= safeMax)
            {
                // 安全区間内なら最短へ寄せず、decoratedFlightを可能な限り維持する。
                flightSeconds = Mathf.Clamp(
                    flightSeconds,
                    safeMin,
                    safeMax);
            }
            else
            {
                // EnergyとXZ安全区間が交差しない場合だけ、
                // Energy範囲内でXZ分離が最小になる時間へ退避する。
                separationConflict = true;
                flightSeconds = Mathf.Clamp(
                    separationOptimalTime,
                    baseMin,
                    baseMax);
            }
        }
        else if (!energyConflict && !hasSeparationRange)
        {
            // XZ安全区間を解析的に作れない場合も、
            // decorated AirをEnergy範囲内で維持する。
            // 実走行中は既存の弱いXZコリドー補正へ任せる。
            flightSeconds = Mathf.Clamp(
                flightSeconds,
                baseMin,
                baseMax);
        }

        if (!IsValidPositiveTime(flightSeconds))
            return;

        float launchVerticalSpeed = SolveVerticalLaunchSpeedForTime(
            subjectPosition.y,
            exactTarget.y,
            flightSeconds);

        if (float.IsNaN(launchVerticalSpeed) ||
            float.IsInfinity(launchVerticalSpeed))
        {
            return;
        }

        Vector3 planarLaunchVelocity =
            planarDisplacement / Mathf.Max(flightSeconds, 0.0001f);

        Vector3 launchVelocity =
            planarLaunchVelocity + Vector3.up * launchVerticalSpeed;

        Vector3 impactVelocity =
            planarLaunchVelocity +
            Vector3.up * (launchVerticalSpeed + Physics.gravity.y * flightSeconds);

        float effectiveRelativeUp = Mathf.Max(
            0f,
            launchVerticalSpeed - subjectVelocity.y);

        float effectivePopHeight = RelativeUpToHeight(effectiveRelativeUp);

        float predictedPlanarSeparation = PredictIncidentPlanarSeparation(
            planarDisplacement,
            forecastPlanarVelocity,
            flightSeconds);

        incidentMaximumObservedPlanarSeparation = 0f;
        ballBody.velocity = launchVelocity;

        ballBody.angularVelocity = Vector3.ClampMagnitude(
            CalculateRollingAngularVelocity(planarLaunchVelocity, Vector3.up),
            IncidentMaximumSpin);

        if (incidentUseTriggerDuringFlight && ballCollider != null)
            ballCollider.isTrigger = true;

        incidentPlanarTravelDirection = planarDisplacement.sqrMagnitude > 0.000001f
            ? planarDisplacement.normalized
            : Vector3.ProjectOnPlane(launchVelocity, Vector3.up).normalized;

        if (incidentPlanarTravelDirection.sqrMagnitude <= 0.000001f)
            incidentPlanarTravelDirection = Vector3.forward;

        incidentPlanValid = true;

        if (usePostTurnPopGuard)
        {
            // 成功したIncident 1回でのみ消費する。
            // 途中で数値安全returnした場合はPendingを残し、次回へ持ち越す。
            postTurnPopGuardPending = false;

            if (enableDebugLog)
            {
                Debug.Log(
                    $"[POST TURN POP GUARD CONSUMED] " +
                    $"rawV0sin={incidentNormalSpeed:F4}m/s " +
                    $"limitedV0sin={incidentNormalSpeedForPop:F4}m/s " +
                    $"rawPreferredUp={rawPreferredRelativeUp:F4}m/s " +
                    $"limitedPreferredUp={preferredRelativeUp:F4}m/s " +
                    $"effectivePopHeight={effectivePopHeight:F4}m",
                    this);
            }
        }

        incidentStartTime = Time.fixedTime;
        incidentElapsed = 0f;
        incidentFlightSeconds = flightSeconds;
        incidentPlanarAccelerationState = Vector3.zero;

        ballFlatCaptured = false;
        ballFlatTime = -1f;
        subjectFlatCaptured = false;
        subjectFlatTime = -1f;

        incidentStartPosition = subjectPosition;
        incidentLaunchVelocity = launchVelocity;
        incidentImpactVelocity = impactVelocity;
        incidentTargetPosition = exactTarget;
        
        // ================================================================
// BallVisual -> BallVisualEqualizer Energy Handoff
// ================================================================

// BallVisualのワールド全運動Energyではなく、
// Subjectに対するPOP相対運動だけをEnergyとして取り出す。
//
// uB = vBallVisual - vSubject
//
// E_POP = 1/2 * mB * |uB|^2
        Vector3 ballVisualRelativePopVelocity =
            incidentLaunchVelocity -
            subjectVelocity;

        // ================================================================
        // Incident normal energy
        //
        // E_perp = 1/2 m (v0 sin(theta))^2
        //
        // 斜面Colliderへ実際に食い込む成分だけを、
        // Equalizer / Negative Envelopeの第一Energy源にする。
        // ================================================================

        // ================================================================
        // Post-turn Equalizer Normal normalization
        //
        // 通常Incident:
        //     equalizerIncidentNormalSpeed = incidentNormalSpeed
        //
        // 回転後最初のIncidentだけ:
        //     vN,max = maxGroundSpeed * ratio
        //     vN,E   = min(raw vN, vN,max)
        //
        // maxGroundSpeed自体はREAD ONLY。
        // BallVisualの0.50m/s見た目POP Guardとは独立させるため、
        // Equalizerには高速域に応じた適切なNormal Energyを残す。
        // ================================================================
        float resolvedPostTurnMaxGroundSpeed = 0f;
        bool resolvedPostTurnMaxGroundSpeedFromSource = false;

        if (usePostTurnPopGuard &&
            normalizeEqualizerEnergyAfterTurn &&
            BallVisualEqualizer)
        {
            resolvedPostTurnMaxGroundSpeedFromSource =
                BallVisualEqualizer.TryGetSourceMaxGroundSpeedReadOnly(
                    out resolvedPostTurnMaxGroundSpeed);
        }

        if (usePostTurnPopGuard &&
            normalizeEqualizerEnergyAfterTurn &&
            (!resolvedPostTurnMaxGroundSpeedFromSource ||
             resolvedPostTurnMaxGroundSpeed <= 0.0001f))
        {
            resolvedPostTurnMaxGroundSpeed =
                Mathf.Max(
                    1f,
                    postTurnFallbackMaxGroundSpeed);
        }

        float postTurnEqualizerNormalSpeedLimit =
            usePostTurnPopGuard && normalizeEqualizerEnergyAfterTurn
                ? Mathf.Max(
                    0.01f,
                    resolvedPostTurnMaxGroundSpeed *
                    Mathf.Clamp(
                        postTurnEqualizerNormalSpeedRatioToMaxGround,
                        0.01f,
                        1f))
                : float.PositiveInfinity;

        float equalizerIncidentNormalSpeed =
            usePostTurnPopGuard && normalizeEqualizerEnergyAfterTurn
                ? Mathf.Min(
                    incidentNormalSpeed,
                    postTurnEqualizerNormalSpeedLimit)
                : incidentNormalSpeed;

        float incidentNormalEnergy =
            0.5f *
            ballBody.mass *
            equalizerIncidentNormalSpeed *
            equalizerIncidentNormalSpeed;

        float legacyRelativePopEnergy =
            0.5f *
            ballBody.mass *
            ballVisualRelativePopVelocity.sqrMagnitude;

        // v0 sin(theta) が物理的に成立する時はNormal Energyを採用。
        // ほぼ0なら、Equalizer Release自体が消失しないよう
        // 従来のSubject相対POP EnergyへFallbackする。
        bool usingIncidentNormalEnergy =
            incidentNormalEnergy > 0.000001f;

        float ballVisualEnergy =
            usingIncidentNormalEnergy
                ? incidentNormalEnergy
                : legacyRelativePopEnergy;

        // v0sin≈0でLegacyRelativePopへFallbackした場合も、
        // 回転後1回だけは同じmaxGroundSpeed正規化上限を越えないようにする。
        if (usePostTurnPopGuard &&
            normalizeEqualizerEnergyAfterTurn &&
            !float.IsInfinity(postTurnEqualizerNormalSpeedLimit))
        {
            float postTurnMaximumEqualizerEnergy =
                0.5f *
                ballBody.mass *
                postTurnEqualizerNormalSpeedLimit *
                postTurnEqualizerNormalSpeedLimit;

            ballVisualEnergy =
                Mathf.Min(
                    ballVisualEnergy,
                    postTurnMaximumEqualizerEnergy);
        }


// BallVisual source EnergyをCanonical unit (=1) として、
        // Equalizer / Envelopeへ同じEnergy・同じStable-N基準高さを渡す。
        //
        // Energyの向きはIncident Normalを第一候補とし、
        // Legacy fallback時だけ相対POP方向を使う。
        Vector3 equalizerEnergySourceAxis =
            usingIncidentNormalEnergy
                ? currentSurfaceNormal
                : (ballVisualRelativePopVelocity.sqrMagnitude >
                   0.000001f
                    ? ballVisualRelativePopVelocity.normalized
                    : currentSurfaceNormal);

        float equalizerReferenceHeight =
            Mathf.Max(
                0.01f,
                oscillationReferenceHeight);

        Vector3 equalizerLaunchVelocity =
            ResolveEqualizerLaunchVelocity(
                ballVisualEnergy,
                subjectVelocity,
                equalizerEnergySourceAxis);


        // Equalizer同期解除。
        // ここで渡す4量は同じCanonical structureを表す:
        //
        //   E0 : BallVisual source Energy
        //   H0 : Stable-N reference height
        //   v  : E0から再構成したEqualizer release velocity
        //   n0 : Energy source axis
        //
        // Equalizer / Envelope側は
        //
        //   aN = E0 / (m H0)
        //   epsilon = E / E0
        //   A = H0 * epsilon
        //
        // を共通規則として使用する。
        bool equalizerReleased =
            BallVisualEqualizer.ReleaseToEnvelopeSimulation(
                equalizerLaunchVelocity,
                ballVisualEnergy,
                equalizerReferenceHeight,
                equalizerEnergySourceAxis);


        if (enableDebugLog)
        {
            Debug.Log(
                $"[EQUALIZER HANDOFF] " +
                $"released={equalizerReleased} " +
                $"sourceEnergy={ballVisualEnergy:F4}J " +
                $"energyMode={(usingIncidentNormalEnergy ? "IncidentNormal" : "LegacyRelativePop")} " +
                $"incidentNormalEnergy={incidentNormalEnergy:F4}J " +
                $"legacyRelativePopEnergy={legacyRelativePopEnergy:F4}J " +
                $"v0cos={incidentTangentSpeed:F4}m/s " +
                $"v0sin={incidentNormalSpeed:F4}m/s " +
                $"energyV0sin={equalizerIncidentNormalSpeed:F4}m/s " +
                $"postTurnMaxGround={(usePostTurnPopGuard && normalizeEqualizerEnergyAfterTurn ? resolvedPostTurnMaxGroundSpeed : 0f):F4}m/s " +
                $"postTurnEqualizerCap={(usePostTurnPopGuard && normalizeEqualizerEnergyAfterTurn ? postTurnEqualizerNormalSpeedLimit : 0f):F4}m/s " +
                $"postTurnMaxGroundSource={resolvedPostTurnMaxGroundSpeedFromSource} " +
                $"postTurnBallVisualV0sin={incidentNormalSpeedForPop:F4}m/s " +
                $"postTurnGuard={usePostTurnPopGuard} " +
                $"incidentAngle={incidentAngleDeg:F3}deg " +
                $"sourcePopHeight={effectivePopHeight:F4}m " +
                $"canonicalReferenceHeight={equalizerReferenceHeight:F4}m " +
                $"sourceEnergyAxis={equalizerEnergySourceAxis:F4} " +
                $"subjectVelocity={subjectVelocity:F4} " +
                $"relativePopVelocity={ballVisualRelativePopVelocity:F4} " +
                $"equalizerLaunchVelocity={equalizerLaunchVelocity:F4}",
                this);
        }
        hasPreviousIncidentSample = false;
        previousIncidentTime = Time.fixedTime;
        previousIncidentBallPosition = ballBody.position;
        previousIncidentBallVelocity = ballBody.velocity;
        previousIncidentSubjectPosition = respondSubject.MappedPosition;
        previousIncidentSubjectVelocity = ReadMappedInSubjectVelocity();

        motionPhase = MotionPhase.Incident;

        if (enableDebugLog)
        {
            float impactPlanarSpeed = Vector3.ProjectOnPlane(
                impactVelocity,
                Vector3.up).magnitude;

            float impactAngle = Mathf.Atan2(
                Mathf.Max(0f, -impactVelocity.y),
                Mathf.Max(0.0001f, impactPlanarSpeed)) * Mathf.Rad2Deg;

            float heightEnergyRetention = preferredHeight > 0.0001f
                ? effectivePopHeight / preferredHeight
                : 1f;

            string separationRangeText = hasSeparationRange
                ? $"[{separationTimeMin:F4},{separationTimeMax:F4}]"
                : "None";

            Debug.Log(
                $"[FIRST POP INCIDENT SPLINE-DECORATED] " +
                $"preferredUp={preferredRelativeUp:F4} " +
                $"energyUp=[{minimumRelativeUp:F4},{maximumRelativeUp:F4}] " +
                $"heightPreference={preferredHeight:F4}m " +
                $"effectivePopHeight={effectivePopHeight:F4}m " +
                $"heightEnergyRetention={heightEnergyRetention:F3} " +
                $"naturalFlight={naturalFlight:F4}s " +
                $"timeCostFlight={timeCostFlight:F4}s " +
                $"decoratedFlight={decoratedFlight:F4}s " +
                $"decorationExtra={(decoratedFlight - timeCostFlight):F4}s " +
                $"energyTime=[{energyTimeMin:F4},{energyTimeMax:F4}] " +
                $"separationTime={separationRangeText}s " +
                $"energyConflict={energyConflict} " +
                $"separationConflict={separationConflict} " +
                $"flight={flightSeconds:F4}s " +
                $"allowedXZSeparation={incidentAllowedPlanarSeparation:F4}m " +
                $"predictedXZSeparation={predictedPlanarSeparation:F4}m " +
                $"subjectVelocity={subjectVelocity:F4} " +
                $"launchVelocity={launchVelocity:F4} " +
                $"target={exactTarget:F4} " +
                $"plannedImpactVelocity={impactVelocity:F4} " +
                $"plannedImpactAngle={impactAngle:F3}deg",
                this);
        }
    }
    
    private Vector3 ResolveEqualizerLaunchVelocity(
        float sourceEnergyJoule,
        Vector3 subjectVelocity,
        Vector3 sourceEnergyAxisVisual)
    {
        if (BallVisualEqualizer == null)
            return subjectVelocity;

        // ------------------------------------------------------------
        // Canonical BallVisual -> Equalizer Energy map
        //
        // E0 = 1/2 mE |uE|^2
        //
        // |uE| = sqrt(2 E0 / mE)
        //
        // Source EnergyはTransportへ分配せず、
        // Stable-Nへ投影される1本のPOP modeとして渡す。
        // TransportはSubject velocityが担当する。
        // ------------------------------------------------------------

        float equalizerMass =
            Mathf.Max(
                0.0001f,
                BallVisualEqualizer.EqualizerMass);

        float safeEnergy =
            Mathf.Max(
                0f,
                sourceEnergyJoule);

        if (safeEnergy <= 0.000001f)
            return subjectVelocity;

        Vector3 sourceAxis =
            sourceEnergyAxisVisual;

        if (sourceAxis.sqrMagnitude <= 0.000001f)
            sourceAxis = currentSurfaceNormal;

        if (sourceAxis.sqrMagnitude <= 0.000001f)
            sourceAxis = Vector3.up;

        sourceAxis.Normalize();

        // Canonical upper half-space.
        if (Vector3.Dot(sourceAxis, Vector3.up) < 0f)
            sourceAxis = -sourceAxis;

        float relativeSpeed =
            Mathf.Sqrt(
                2f *
                safeEnergy /
                equalizerMass);

        Vector3 equalizerRelativeVelocity =
            sourceAxis *
            relativeSpeed;

        Vector3 equalizerLaunchVelocity =
            subjectVelocity +
            equalizerRelativeVelocity;

        return IsFinite(equalizerLaunchVelocity)
            ? equalizerLaunchVelocity
            : subjectVelocity;
    }

    private void ProcessIncident()
    {
        if (!incidentPlanValid)
            return;

        if (incidentUseTriggerDuringFlight &&
            ballCollider != null &&
            !ballCollider.isTrigger)
        {
            ballCollider.isTrigger = true;
        }

        incidentElapsed = Mathf.Max(
            0f,
            Time.fixedTime - incidentStartTime);

        // 純粋入射法: Apex前後で制御則を変えない。
        // Yは初速+Gravity、人工制御は弱いXZ補正と回転だけ。
        ApplyIncidentPlanarCorrection();
        ApplyIncidentRotationFollow();
    }


    private void ApplyIncidentPlanarCorrection()
    {
        float dt = Mathf.Max(Time.fixedDeltaTime, 0.000001f);
        float t = Mathf.Clamp(
            incidentElapsed,
            0f,
            Mathf.Max(incidentFlightSeconds, 0.0001f));

        Vector3 plannedPosition =
            incidentStartPosition +
            incidentLaunchVelocity * t +
            0.5f * Physics.gravity * (t * t);

        Vector3 plannedVelocity =
            incidentLaunchVelocity + Physics.gravity * t;

        Vector3 planarPositionError = Vector3.ProjectOnPlane(
            plannedPosition - ballBody.position,
            Vector3.up);

        Vector3 planarVelocityError = Vector3.ProjectOnPlane(
            plannedVelocity - ballBody.velocity,
            Vector3.up);

        Vector3 desiredAcceleration =
            planarPositionError * incidentPlanarPositionGain +
            planarVelocityError * incidentPlanarVelocityGain;

        // 演出上の自由な先行は残すが、Subjectから極端に離れた時だけ
        // XZコリドーへ弱く戻す。Yには一切触れない。
        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();

        Vector3 planarSeparation = Vector3.ProjectOnPlane(
            ballBody.position - subjectPosition,
            Vector3.up);

        float separationMagnitude = planarSeparation.magnitude;
        incidentMaximumObservedPlanarSeparation = Mathf.Max(
            incidentMaximumObservedPlanarSeparation,
            separationMagnitude);

        float hardSeparation =
            incidentAllowedPlanarSeparation * IncidentPlanarSeparationHardRatio;

        if (separationMagnitude > hardSeparation &&
            separationMagnitude > 0.0001f)
        {
            Vector3 outwardDirection = planarSeparation / separationMagnitude;
            float excess = separationMagnitude - hardSeparation;

            Vector3 planarRelativeVelocity = Vector3.ProjectOnPlane(
                ballBody.velocity - subjectVelocity,
                Vector3.up);

            float outwardRelativeSpeed = Mathf.Max(
                0f,
                Vector3.Dot(planarRelativeVelocity, outwardDirection));

            // 既存Planar補正の範囲内だけで働く安全補正。
            // Hard Syncではなく、距離超過と外向き相対速度だけを減らす。
            desiredAcceleration +=
                -outwardDirection *
                (excess * incidentPlanarPositionGain +
                 outwardRelativeSpeed * incidentPlanarVelocityGain);
        }

        desiredAcceleration = Vector3.ClampMagnitude(
            desiredAcceleration,
            incidentMaximumPlanarAcceleration);

        incidentPlanarAccelerationState = Vector3.MoveTowards(
            incidentPlanarAccelerationState,
            desiredAcceleration,
            incidentMaximumPlanarJerk * dt);

        ballBody.AddForce(
            incidentPlanarAccelerationState,
            ForceMode.Acceleration);
    }

    private void ApplyIncidentRotationFollow()
    {
        Vector3 targetAngularVelocity = Vector3.ClampMagnitude(
            CalculateRollingAngularVelocity(
                ballBody.velocity,
                Vector3.up),
            IncidentMaximumSpin);

        Vector3 angularVelocityError =
            targetAngularVelocity - ballBody.angularVelocity;

        // 1st POP内部だけで回転追従上限を閉じる。
        // 2nd POP/Missileの設定には依存させない。
        float maximumAngularAcceleration = Mathf.Max(
            20f,
            IncidentMaximumSpin * 2.4f);

        Vector3 angularAcceleration = Vector3.ClampMagnitude(
            angularVelocityError * 12f,
            maximumAngularAcceleration);

        ballBody.AddTorque(
            angularAcceleration,
            ForceMode.Acceleration);
    }

    private float ResolveIncidentTimeCostFlightSeconds(
        Vector3 subjectPosition,
        Vector3 exactTarget,
        Vector3 subjectVelocity,
        float naturalFlightSeconds)
    {
        Vector3 planarDisplacement = Vector3.ProjectOnPlane(
            exactTarget - subjectPosition,
            Vector3.up);

        float planarDistance = planarDisplacement.magnitude;
        if (planarDistance <= 0.0001f)
            return naturalFlightSeconds;

        Vector3 travelDirection =
            planarDisplacement / planarDistance;

        Vector3 planarSubjectVelocity = Vector3.ProjectOnPlane(
            subjectVelocity,
            Vector3.up);

        float closingSpeed = Vector3.Dot(
            planarSubjectVelocity,
            travelDirection);

        float referenceSpeed = Mathf.Max(
            ResolveSubjectReferenceSpeed(),
            planarSubjectVelocity.magnitude);

        if (closingSpeed <= 0.05f)
            closingSpeed = referenceSpeed;

        if (closingSpeed <= 0.05f)
            return naturalFlightSeconds;

        float subjectEta =
            planarDistance / closingSpeed;

        // 極端な短時間化でplanarLaunchVelocityだけが暴走しないよう、
        // Subject基準速度の1.35倍から最低Flightを作る。
        float representativeMaximumPlanarSpeed = Mathf.Max(
            6f,
            referenceSpeed * 1.35f);

        float minimumByPlanarSpeed =
            planarDistance / representativeMaximumPlanarSpeed;

        float minimumFlight = Mathf.Min(
            naturalFlightSeconds,
            Mathf.Max(
                IncidentMinimumFlightSeconds,
                Time.fixedDeltaTime * 2f,
                minimumByPlanarSpeed));

        // Subject ETAがNaturalより短い時だけTime-Cost短縮。
        return Mathf.Clamp(
            subjectEta,
            minimumFlight,
            naturalFlightSeconds);
    }

    private float ResolveIncidentDecoratedFlightSeconds(
        Vector3 subjectPosition,
        Vector3 exactTarget,
        Vector3 subjectVelocity,
        float preferredRelativeUpSpeed,
        float timeCostFlightSeconds,
        float naturalFlightSeconds)
    {
        // 1〜2 FixedUpdateしかない極端な近距離Limitは
        // POP装飾よりLimit連続性を優先する。
        if (naturalFlightSeconds <=
            Mathf.Max(0.05f, Time.fixedDeltaTime * 2f))
        {
            return timeCostFlightSeconds;
        }

        float planarSubjectSpeed = Vector3.ProjectOnPlane(
            subjectVelocity,
            Vector3.up).magnitude;

        // 高速になるほど最低POPもsqrtで緩やかに増やし、
        // 2.6m/sで必ず頭打ちにする。
        float speedScale = Mathf.Sqrt(
            Mathf.Max(1f, planarSubjectSpeed / 12f));

        float adaptiveMinimumRelativeUpSpeed = Mathf.Min(
            IncidentPopDecorationMaximumRelativeUpSpeed,
            IncidentPopDecorationMinimumRelativeUpSpeed * speedScale);

        float visibleRelativeUpSpeed = Mathf.Clamp(
            Mathf.Max(
                preferredRelativeUpSpeed * IncidentPopDecorationPreferredUpRatio,
                adaptiveMinimumRelativeUpSpeed),
            IncidentPopDecorationMinimumRelativeUpSpeed,
            IncidentPopDecorationMaximumRelativeUpSpeed);

        float visibleFlightSeconds = SolveIncidentTimeFromRelativeUp(
            subjectPosition.y,
            exactTarget.y,
            subjectVelocity.y,
            visibleRelativeUpSpeed);

        if (!IsValidPositiveTime(visibleFlightSeconds))
            return timeCostFlightSeconds;

        // SlopeStick3D成功版と同じ30〜75msの追加時間予算。
        float extraBudget = Mathf.Clamp(
            naturalFlightSeconds * IncidentPopDecorationFlightRatio,
            IncidentPopDecorationMinimumExtraSeconds,
            IncidentPopDecorationMaximumExtraSeconds);

        float maximumDecoratedFlight =
            timeCostFlightSeconds + extraBudget;

        // visibleFlight全部を強制せず、Time-CostからextraBudget以内だけ延ばす。
        return Mathf.Max(
            timeCostFlightSeconds,
            Mathf.Min(visibleFlightSeconds, maximumDecoratedFlight));
    }

    private float ResolveIncidentPreferredRelativeUpSpeed(float referenceSpeed)
    {
        float gravity = GetVerticalGravityMagnitude();

        if (referenceSpeed <= 0.05f)
        {
            return Mathf.Sqrt(
                2f * gravity *
                Mathf.Max(0f, preLimitTargetHeightRelativeToSubject));
        }

        // h = eta*v^2/(2g) と u = sqrt(2gh) から
        // u = sqrt(eta)*v。高さを経由せず直接相対Up速度を作る。
        float preferredUp =
            Mathf.Sqrt(Mathf.Max(0f, firstPopVelocitySquaredRatio)) *
            referenceSpeed;

        float minimumUp = Mathf.Sqrt(
            2f * gravity *
            Mathf.Max(0f, minimumEnergyScaledPopHeight));

        float maximumUp = Mathf.Sqrt(
            2f * gravity *
            Mathf.Max(
                minimumEnergyScaledPopHeight,
                maximumEnergyScaledPopHeight));

        return Mathf.Clamp(
            preferredUp,
            minimumUp,
            maximumUp);
    }

    private float RelativeUpToHeight(float relativeUpSpeed)
    {
        float gravity = GetVerticalGravityMagnitude();
        float up = Mathf.Max(0f, relativeUpSpeed);

        return up * up /
               (2f * Mathf.Max(gravity, 0.0001f));
    }

    private float ResolveIncidentAllowedPlanarSeparation(Vector3 subjectVelocity)
    {
        float radius = ResolveBallRadius();
        float planarSpeed = Vector3.ProjectOnPlane(
            subjectVelocity,
            Vector3.up).magnitude;

        float minimum = 1.7f * radius;
        float maximum = 2.9f * radius;

        // 旧 0.85 + 0.022*v (R=.5, dt=.02) を
        // Ball半径とFixedDeltaTimeへ正規化した同値式。
        return Mathf.Clamp(
            minimum +
            1.1f * planarSpeed * Mathf.Max(Time.fixedDeltaTime, 0.0001f),
            minimum,
            maximum);
    }

    private float ResolveBallRadius()
    {
        if (ballCollider == null)
            return 0.5f;

        Vector3 scale = ballCollider.transform.lossyScale;
        float maximumScale = Mathf.Max(
            Mathf.Abs(scale.x),
            Mathf.Abs(scale.y),
            Mathf.Abs(scale.z));

        return ballCollider.radius *
               Mathf.Max(maximumScale, 0.0001f);
    }

    private Vector3 ResolveIncidentForecastPlanarVelocity(
        Vector3 planarDisplacement,
        Vector3 subjectVelocity)
    {
        Vector3 currentPlanarVelocity = Vector3.ProjectOnPlane(
            subjectVelocity,
            Vector3.up);

        if (planarDisplacement.sqrMagnitude <= 0.000001f)
            return currentPlanarVelocity;

        Vector3 travelDirection = planarDisplacement.normalized;
        float currentClosingSpeed = Mathf.Max(
            0f,
            Vector3.Dot(currentPlanarVelocity, travelDirection));

        float tangentPlanarRatio = 0f;
        if (currentSlopeTangent.sqrMagnitude > 0.000001f)
        {
            Vector3 tangent = currentSlopeTangent.normalized;
            tangentPlanarRatio = Vector3.ProjectOnPlane(
                tangent,
                Vector3.up).magnitude;
        }

        float plannedClosingSpeed = Mathf.Max(
            currentClosingSpeed,
            Mathf.Max(0f, slopeCore.CapturedTargetTangentSpeed) *
            tangentPlanarRatio);

        Vector3 lateralVelocity =
            currentPlanarVelocity -
            travelDirection * currentClosingSpeed;

        return travelDirection * plannedClosingSpeed +
               lateralVelocity;
    }

    private static bool TrySolveIncidentSeparationTimeRange(
        Vector3 planarDisplacement,
        Vector3 forecastPlanarVelocity,
        float allowedSeparation,
        out float minimumTime,
        out float maximumTime,
        out float optimalTime)
    {
        float a = forecastPlanarVelocity.sqrMagnitude;

        if (a <= 0.000001f)
        {
            optimalTime = 0f;
            minimumTime = 0f;
            maximumTime = float.PositiveInfinity;
            return planarDisplacement.magnitude <= allowedSeparation;
        }

        float dot = Vector3.Dot(
            planarDisplacement,
            forecastPlanarVelocity);

        // |D - V*T|^2 はTについて凸二次関数。
        // optimalTimeはSubjectとのXZ距離が最小になる時刻。
        optimalTime = Mathf.Max(0f, dot / a);

        float b = -2f * dot;
        float c =
            planarDisplacement.sqrMagnitude -
            allowedSeparation * allowedSeparation;

        float discriminant = b * b - 4f * a * c;

        if (discriminant < 0f)
        {
            minimumTime = 0f;
            maximumTime = 0f;
            return false;
        }

        float sqrtD = Mathf.Sqrt(discriminant);
        float inv2A = 1f / (2f * a);

        float t0 = (-b - sqrtD) * inv2A;
        float t1 = (-b + sqrtD) * inv2A;

        if (t0 > t1)
        {
            float tmp = t0;
            t0 = t1;
            t1 = tmp;
        }

        minimumTime = Mathf.Max(0f, t0);
        maximumTime = t1;

        return maximumTime >= minimumTime &&
               maximumTime >= 0f;
    }

    private static float PredictIncidentPlanarSeparation(
        Vector3 planarDisplacement,
        Vector3 forecastPlanarVelocity,
        float flightSeconds)
    {
        return (
            planarDisplacement -
            forecastPlanarVelocity * Mathf.Max(0f, flightSeconds)
        ).magnitude;
    }

    private float SolveIncidentTimeFromRelativeUp(
        float startY,
        float targetY,
        float subjectVerticalSpeed,
        float relativeUpSpeed)
    {
        float gravity = GetVerticalGravityMagnitude();
        if (gravity <= 0.000001f)
            return -1f;

        float launchVerticalSpeed =
            subjectVerticalSpeed + relativeUpSpeed;

        float deltaY = targetY - startY;
        float discriminant =
            launchVerticalSpeed * launchVerticalSpeed -
            2f * gravity * deltaY;

        if (discriminant < 0f)
            return -1f;

        float sqrtD = Mathf.Sqrt(discriminant);
        float t0 =
            (launchVerticalSpeed - sqrtD) / gravity;
        float t1 =
            (launchVerticalSpeed + sqrtD) / gravity;

        float minimum = Mathf.Max(
            Time.fixedDeltaTime * 0.5f,
            0.0001f);

        float best = float.PositiveInfinity;

        if (t0 >= minimum)
            best = t0;

        if (t1 >= minimum && t1 < best)
            best = t1;

        return float.IsInfinity(best)
            ? -1f
            : best;
    }

private static bool IsValidPositiveTime(float value)
    {
        return value > 0f &&
               !float.IsNaN(value) &&
               !float.IsInfinity(value);
    }


    private float SolveVerticalLaunchSpeedForTime(
        float startY,
        float targetY,
        float flightSeconds)
    {
        float t = Mathf.Max(
            flightSeconds,
            0.0001f);

        // targetY = startY + vy*t + 0.5*g*t^2
        return
            (targetY - startY - 0.5f * Physics.gravity.y * t * t) /
            t;
    }



    private void ObserveIncidentLimitCrossing()
    {
        if (!incidentPlanValid)
            return;

        Vector3 travelDirection = incidentPlanarTravelDirection;
        if (travelDirection.sqrMagnitude <= 0.000001f)
            return;

        travelDirection.Normalize();

        float currentTime = Time.fixedTime;
        Vector3 currentBallPosition = ballBody.position;
        Vector3 currentBallVelocity = ballBody.velocity;
        Vector3 currentSubjectPosition = respondSubject.MappedPosition;
        Vector3 currentSubjectVelocity = ReadMappedInSubjectVelocity();

        if (!hasPreviousIncidentSample)
        {
            hasPreviousIncidentSample = true;
            previousIncidentTime = currentTime;
            previousIncidentBallPosition = currentBallPosition;
            previousIncidentBallVelocity = currentBallVelocity;
            previousIncidentSubjectPosition = currentSubjectPosition;
            previousIncidentSubjectVelocity = currentSubjectVelocity;
            return;
        }

        // Limitは静止して収束する点ではなく、Target Progressを通過する境界面。
        // Yを含む3D距離で停止判定せず、進行XZ方向の符号反転を連続時間補間する。
        // 計画弾道自体はExact TargetのYへ解いているため、正常時はこの通過点が
        // Exact Targetへ自然に重なる。
        float previousRemaining = Vector3.Dot(
            incidentTargetPosition - previousIncidentBallPosition,
            travelDirection);

        float currentRemaining = Vector3.Dot(
            incidentTargetPosition - currentBallPosition,
            travelDirection);

        bool crossed =
            previousRemaining > 0f &&
            currentRemaining <= 0f;

        if (!crossed)
        {
            previousIncidentTime = currentTime;
            previousIncidentBallPosition = currentBallPosition;
            previousIncidentBallVelocity = currentBallVelocity;
            previousIncidentSubjectPosition = currentSubjectPosition;
            previousIncidentSubjectVelocity = currentSubjectVelocity;
            return;
        }

        float denominator = previousRemaining - currentRemaining;
        limitCrossingAlpha = Mathf.Abs(denominator) > 0.000001f
            ? Mathf.Clamp01(previousRemaining / denominator)
            : 1f;

        limitCrossingTime = Mathf.Lerp(
            previousIncidentTime,
            currentTime,
            limitCrossingAlpha);

        limitCrossingPosition = Vector3.Lerp(
            previousIncidentBallPosition,
            currentBallPosition,
            limitCrossingAlpha);

        limitIncomingVelocity = Vector3.Lerp(
            previousIncidentBallVelocity,
            currentBallVelocity,
            limitCrossingAlpha);

        Vector3 crossingSubjectPosition = Vector3.Lerp(
            previousIncidentSubjectPosition,
            currentSubjectPosition,
            limitCrossingAlpha);

        limitReferencePosition = incidentTargetPosition;
        limitReferenceVelocity = Vector3.Lerp(
            previousIncidentSubjectVelocity,
            currentSubjectVelocity,
            limitCrossingAlpha);

        // FixedUpdate内の真のcrossing位置へ戻すだけで、SubjectへTeleportしない。
        ballBody.position = limitCrossingPosition;
        ballBody.velocity = limitIncomingVelocity;

        if (ballCollider != null)
            ballCollider.isTrigger = false;

        if (enableDebugLog)
        {
            float targetError = Vector3.Distance(
                limitCrossingPosition,
                incidentTargetPosition);

            float verticalError =
                incidentTargetPosition.y - limitCrossingPosition.y;

            float relativeHeightAtLimit =
                limitCrossingPosition.y - crossingSubjectPosition.y;

            Vector3 relativeVelocityAtLimit =
                limitIncomingVelocity - limitReferenceVelocity;

            float velocityError = Vector3.Distance(
                limitIncomingVelocity,
                incidentImpactVelocity);

            Debug.Log(
                $"[INCIDENT -> MISSILE BOUNDARY] " +
                $"time={limitCrossingTime:F4} " +
                $"alpha={limitCrossingAlpha:F4} " +
                $"crossPos={limitCrossingPosition:F4} " +
                $"target={incidentTargetPosition:F4} " +
                $"positionError={targetError:F4} " +
                $"verticalError={verticalError:F4} " +
                $"relativeHeightAtLimit={relativeHeightAtLimit:F4} " +
                $"relativeVelocityAtLimit={relativeVelocityAtLimit:F4} " +
                $"maxObservedXZSeparation={incidentMaximumObservedPlanarSeparation:F4}m " +
                $"incomingVelocity={limitIncomingVelocity:F4} " +
                $"plannedImpactVelocity={incidentImpactVelocity:F4} " +
                $"velocityError={velocityError:F4} " +
                $"incidentCost={(limitCrossingTime - incidentStartTime):F4}s",
                this);
        }

        BeginMissileMethod();
    }


    private bool TryResolveExactIncidentTarget(out Vector3 targetPosition)
    {
        targetPosition = Vector3.zero;

        if (slopeCore == null ||
            !slopeCore.TryGetBallVisualTargetProgressCenterPhysics(
                out Vector3 physicsTarget))
        {
            return false;
        }

        // PhysicsRoot側の球中心Targetを、現在のSubject表示座標へ写す。
        Vector3 physicsOffset =
            physicsTarget - inSubjectBody.position;

        targetPosition =
            respondSubject.MappedPosition +
            respondSubject.MapDirection(physicsOffset);

        return IsFinite(targetPosition);
    }

    private float ResolveSubjectReferenceSpeed()
    {
        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();

        float tangentSpeed = 0f;
        if (currentSlopeTangent.sqrMagnitude > 0.000001f)
        {
            tangentSpeed = Mathf.Abs(Vector3.Dot(
                subjectVelocity,
                currentSlopeTangent.normalized));
        }

        float planarSpeed = Vector3.ProjectOnPlane(
            subjectVelocity,
            Vector3.up).magnitude;

        float actualSpeed = Mathf.Max(tangentSpeed, planarSpeed);

        return actualSpeed > 0.05f
            ? actualSpeed
            : Mathf.Max(0f, slopeCore.CapturedTargetTangentSpeed);
    }



    // =====================================================================
    // 2nd POP / Missile Method
    // =====================================================================

    private void BeginMissileMethod()
    {
        // 入射法の-0速度を消してから別の補正を重ねるのではなく、
        // 方式間境界で一回だけMissile初速へ変換する。
        Vector3 velocityBefore = ballBody.velocity;
        Vector3 planarReferenceVelocity = Vector3.ProjectOnPlane(
            limitReferenceVelocity,
            Vector3.up);

        Vector3 plannedVelocityAfter =
            planarReferenceVelocity +
            Vector3.up * secondPopUpSpeed;

        Vector3 deltaVelocity = plannedVelocityAfter - velocityBefore;
        ballBody.AddForce(deltaVelocity, ForceMode.VelocityChange);

        missileStartTime = Time.fixedTime;
        missileApexTime = -1f;
        missileElapsed = 0f;
        missileHadPositiveUpSpeed = plannedVelocityAfter.y > missileApexVerticalSpeedThreshold;
        missileAscentAccelerationState = Vector3.zero;
        missileChaseAccelerationState = Vector3.zero;
        subjectFlatCaptured = false;
        subjectFlatTime = -1f;
        ballFlatCaptured = false;
        ballFlatTime = -1f;

        motionPhase = MotionPhase.MissileAscent;

        if (enableDebugLog)
        {
            float incomingPlanar = Vector3.ProjectOnPlane(velocityBefore, Vector3.up).magnitude;
            float outgoingPlanar = Vector3.ProjectOnPlane(plannedVelocityAfter, Vector3.up).magnitude;
            float incomingAngle = Mathf.Atan2(
                Mathf.Max(0f, -velocityBefore.y),
                Mathf.Max(0.0001f, incomingPlanar)) * Mathf.Rad2Deg;
            float outgoingAngle = Mathf.Atan2(
                Mathf.Max(0f, plannedVelocityAfter.y),
                Mathf.Max(0.0001f, outgoingPlanar)) * Mathf.Rad2Deg;

            Debug.Log(
                $"[MISSILE ENTRY SYNC] " +
                $"time={missileStartTime:F4} " +
                $"position={ballBody.position:F4} " +
                $"velocityBefore={velocityBefore:F4} " +
                $"referenceVelocity={limitReferenceVelocity:F4} " +
                $"plannedVelocityAfter={plannedVelocityAfter:F4} " +
                $"incomingAngle={incomingAngle:F3}deg " +
                $"outgoingAngle={outgoingAngle:F3}deg",
                this);
        }
    }

    private void ProcessMissileAscent()
    {
        if (motionPhase != MotionPhase.MissileAscent)
            return;

        float dt = Mathf.Max(Time.fixedDeltaTime, 0.000001f);
        missileElapsed = Mathf.Max(0f, Time.fixedTime - missileStartTime);

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();

        Vector3 planarPositionError = Vector3.ProjectOnPlane(
            subjectPosition - ballBody.position,
            Vector3.up);

        Vector3 planarVelocityError = Vector3.ProjectOnPlane(
            subjectVelocity - ballBody.velocity,
            Vector3.up);

        Vector3 desiredAcceleration =
            planarPositionError * missileAscentPositionGain +
            planarVelocityError * missileAscentVelocityGain;

        desiredAcceleration = Vector3.ClampMagnitude(
            desiredAcceleration,
            missileAscentMaximumAcceleration);

        missileAscentAccelerationState = Vector3.MoveTowards(
            missileAscentAccelerationState,
            desiredAcceleration,
            missileAscentMaximumJerk * dt);

        ballBody.AddForce(
            missileAscentAccelerationState,
            ForceMode.Acceleration);

        float upSpeed = ballBody.velocity.y;
        if (upSpeed > missileApexVerticalSpeedThreshold)
            missileHadPositiveUpSpeed = true;

        bool apex = missileHadPositiveUpSpeed && upSpeed <= missileApexVerticalSpeedThreshold;
        bool timeout = missileElapsed >= missileMaximumAscentSeconds;

        if (apex || timeout)
        {
            missileApexTime = Time.fixedTime;
            missileChaseAccelerationState = missileAscentAccelerationState;
            motionPhase = MotionPhase.MissileChase;

            if (enableDebugLog)
            {
                Debug.Log(
                    $"[MISSILE APEX] time={missileApexTime:F4} " +
                    $"reason={(apex ? "Apex" : "Timeout")} " +
                    $"ballPos={ballBody.position:F4} " +
                    $"subjectPos={subjectPosition:F4}",
                    this);
            }
        }
    }


    private void ProcessMissileChase()
    {
        if (motionPhase != MotionPhase.MissileChase)
            return;

        float dt = Mathf.Max(Time.fixedDeltaTime, 0.000001f);
        missileElapsed = Mathf.Max(0f, Time.fixedTime - missileStartTime);

        // Keep the successful BallVisual motion formula unchanged.
        Vector3 subjectPosition =
            respondSubject.MappedPosition;

        Vector3 subjectVelocity =
            ReadMappedInSubjectVelocity();

        Vector3 shadowTargetPosition =
            subjectPosition +
            subjectVelocity *
            missileChaseLeadSeconds;

        Vector3 positionError =
            shadowTargetPosition -
            ballBody.position;

        Vector3 velocityError =
            subjectVelocity -
            ballBody.velocity;

        Vector3 desiredAcceleration =
            positionError * missileChasePositionGain +
            velocityError * missileChaseVelocityGain;

        // 実Flat接触後はY所有権をCollider/Gravityへ返す。
        if (ballFlatCaptured)
            desiredAcceleration = Vector3.ProjectOnPlane(desiredAcceleration, Vector3.up);

        desiredAcceleration = Vector3.ClampMagnitude(
            desiredAcceleration,
            missileChaseMaximumAcceleration);

        missileChaseAccelerationState = Vector3.MoveTowards(
            missileChaseAccelerationState,
            desiredAcceleration,
            missileChaseMaximumJerk * dt);

        if (ballFlatCaptured)
            missileChaseAccelerationState = Vector3.ProjectOnPlane(
                missileChaseAccelerationState,
                Vector3.up);

        ballBody.AddForce(
            missileChaseAccelerationState,
            ForceMode.Acceleration);
    }

    private bool ShouldBeginTerminalRejoin(bool isFlat)
    {
        if (motionPhase != MotionPhase.MissileChase)
            return false;

        missileElapsed = Mathf.Max(0f, Time.fixedTime - missileStartTime);

        bool subjectFlat = isFlat || slopeCore.BallVisualIsOnFlat;

        if (subjectFlat)
        {
            if (!subjectFlatCaptured)
            {
                subjectFlatCaptured = true;
                subjectFlatTime = Time.fixedTime;

                if (enableDebugLog)
                {
                    Debug.Log(
                        $"[TERMINAL ENTRY WINDOW] reason=SubjectFlat " +
                        $"time={subjectFlatTime:F4} " +
                        $"ballPos={ballBody.position:F4} " +
                        $"subjectPos={respondSubject.MappedPosition:F4}",
                        this);
                }
            }

            return true;
        }

        return missileElapsed >= missileMaximumWaitForSubjectFlatSeconds;
    }

    // =====================================================================
    // Terminal Rejoin / final synchronization zone
    // =====================================================================

    private void BeginTerminalRejoin()
    {
        if (motionPhase != MotionPhase.MissileChase)
            return;

        terminalStartTime = Time.fixedTime;
        terminalElapsed = 0f;
        terminalActiveTimeBudget = Mathf.Max(terminalTimeBudget, terminalMinimumTimeToGo);
        terminalTimeToGo = terminalActiveTimeBudget;
        terminalAccelerationState = missileChaseAccelerationState;
        previousTerminalAccelerationState = terminalAccelerationState;
        terminalStableFrames = 0;
        terminalExtendedRecoveryActive = false;
        terminalRecoveryExtensionCount = 0;

        motionPhase = MotionPhase.TerminalRejoin;
    }

    private void ProcessTerminalRejoin()
    {
        if (motionPhase != MotionPhase.TerminalRejoin)
            return;

        float dt = Mathf.Max(Time.fixedDeltaTime, 0.000001f);
        terminalElapsed = Mathf.Max(0f, Time.fixedTime - terminalStartTime);

        float rawTimeToGo = terminalActiveTimeBudget - terminalElapsed;
        terminalTimeToGo = Mathf.Max(terminalMinimumTimeToGo, rawTimeToGo);

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();

        // Terminalの物理誘導式は維持する。
        // ここでの役割は「連続加速度で追いつく」ことであり、位置代入ではない。
        Vector3 terminalTargetPosition =
            subjectPosition +
            subjectVelocity *
            terminalTimeToGo;

        Vector3 terminalTargetVelocity =
            subjectVelocity;

        Vector3 positionToDeadline =
            terminalTargetPosition -
            ballBody.position;

        Vector3 velocityNow = ballBody.velocity;
        float safeT = Mathf.Max(terminalTimeToGo, terminalMinimumTimeToGo);

        Vector3 requiredTotalAcceleration =
            6f * positionToDeadline / (safeT * safeT) -
            (4f * velocityNow + 2f * terminalTargetVelocity) / safeT;

        Vector3 desiredArtificialAcceleration =
            requiredTotalAcceleration - Physics.gravity;

        // Flat接触後はYを接触物理へ返す。
        if (ballFlatCaptured)
        {
            desiredArtificialAcceleration =
                Vector3.ProjectOnPlane(
                    desiredArtificialAcceleration,
                    Vector3.up);
        }

        desiredArtificialAcceleration =
            Vector3.ClampMagnitude(
                desiredArtificialAcceleration,
                maximumTerminalAcceleration);

        previousTerminalAccelerationState = terminalAccelerationState;
        terminalAccelerationState = Vector3.MoveTowards(
            terminalAccelerationState,
            desiredArtificialAcceleration,
            maximumTerminalJerk * dt);

        if (ballFlatCaptured)
        {
            terminalAccelerationState = Vector3.ProjectOnPlane(
                terminalAccelerationState,
                Vector3.up);
        }

        ballBody.AddForce(
            terminalAccelerationState,
            ForceMode.Acceleration);

        // -------------------------------------------------------------
        // 1) Control convergence: Flat後はXZだけでTerminal制御の収束を見る。
        // 2) Final sync safety: 最終同期はXYZ全体 + Visual旋回終了を必須にする。
        // -------------------------------------------------------------
        Vector3 fullPositionError =
            subjectPosition - ballBody.position;
        Vector3 fullVelocityError =
            subjectVelocity - ballBody.velocity;

        Vector3 controlPositionError = fullPositionError;
        Vector3 controlVelocityError = fullVelocityError;

        if (ballFlatCaptured)
        {
            controlPositionError =
                Vector3.ProjectOnPlane(
                    controlPositionError,
                    Vector3.up);

            controlVelocityError =
                Vector3.ProjectOnPlane(
                    controlVelocityError,
                    Vector3.up);
        }

        bool controlReady =
            ballFlatCaptured &&
            controlPositionError.magnitude <= terminalPositionTolerance &&
            controlVelocityError.magnitude <= terminalVelocityTolerance;

        bool visualTurnActive =
            respondSubject != null &&
            respondSubject.IsVisualFrameTurning;

        // v4では最終直接一致を「丸め」レベルに限定する。
        float finalPositionTolerance =
            Mathf.Min(
                FinalMicroSyncPositionTolerance,
                Mathf.Max(0.000001f, emergencyVisualPositionTolerance));

        float finalVelocityTolerance =
            Mathf.Min(
                FinalMicroSyncVelocityTolerance,
                Mathf.Max(0.000001f, emergencyVisualVelocityTolerance));

        bool finalSyncSafe =
            !visualTurnActive &&
            fullPositionError.magnitude <= finalPositionTolerance &&
            fullVelocityError.magnitude <= finalVelocityTolerance;

        bool ready = controlReady && finalSyncSafe;
        terminalStableFrames = ready ? terminalStableFrames + 1 : 0;

        if (terminalStableFrames >= Mathf.Max(1, terminalStableFramesRequired))
        {
            CompleteTerminalRejoin(false);
            return;
        }

        if (rawTimeToGo <= 0f || RawTimeJurge)
        {
            // 旋回途中ではSettledへ確定しない。
            // Terminalの連続加速度だけを継続し、Visual frameが確定するまで待つ。
            if (visualTurnActive)
            {
                terminalActiveTimeBudget =
                    Mathf.Max(
                        terminalActiveTimeBudget,
                        terminalElapsed + terminalMinimumTimeToGo);

                terminalTimeToGo = terminalMinimumTimeToGo;

                if (enableDebugLog &&
                    fixedFrameCounter % Mathf.Max(1, logEveryFixedFrames) == 0)
                {
                    Debug.Log(
                        $"[TERMINAL TURN WAIT] time={Time.fixedTime:F4} " +
                        $"posError3D={fullPositionError.magnitude:F4} " +
                        $"velError3D={fullVelocityError.magnitude:F4}",
                        this);
                }

                return;
            }

            // 物理的に追いつけるならAcceleration/Jerk budget内でTerminalを延長する。
            if (TryExtendTerminalRecovery(
                    subjectPosition,
                    subjectVelocity,
                    out float recoveryHorizon))
            {
                terminalActiveTimeBudget =
                    terminalElapsed + recoveryHorizon;

                terminalTimeToGo =
                    Mathf.Max(
                        terminalMinimumTimeToGo,
                        recoveryHorizon);

                terminalExtendedRecoveryActive = true;
                terminalRecoveryExtensionCount++;

                if (enableDebugLog)
                {
                    Debug.Log(
                        $"[TERMINAL RECOVERY EXTEND] " +
                        $"count={terminalRecoveryExtensionCount} " +
                        $"time={Time.fixedTime:F4} " +
                        $"horizon={recoveryHorizon:F4}s " +
                        $"posError3D={fullPositionError.magnitude:F4} " +
                        $"velError3D={fullVelocityError.magnitude:F4}",
                        this);
                }

                return;
            }

            // ここでもhard snapは禁止。
            // 物理Terminalで回収不能なら、現在のSubject相対位置/速度を連続条件としてRelative Hermiteへ渡す。
            BeginContinuousRejoin(
                "TerminalBudgetExhausted_NoHardSnap");
            return;
        }

        if (enableDebugLog &&
            fixedFrameCounter % Mathf.Max(1, logEveryFixedFrames) == 0)
        {
            Debug.Log(
                $"[TERMINAL REJOIN] elapsed={terminalElapsed:F4}s " +
                $"Tgo={terminalTimeToGo:F4}s " +
                $"ballFlat={ballFlatCaptured} " +
                $"controlPosError={controlPositionError.magnitude:F4} " +
                $"controlVelError={controlVelocityError.magnitude:F4} " +
                $"finalPosError3D={fullPositionError.magnitude:F4} " +
                $"finalVelError3D={fullVelocityError.magnitude:F4} " +
                $"turning={visualTurnActive} " +
                $"verticalOwner={(ballFlatCaptured ? "PhysicsContact" : "Guidance")}",
                this);
        }
    }

    private bool TryExtendTerminalRecovery(
        Vector3 subjectPosition,
        Vector3 subjectVelocity,
        out float recoveryHorizon)
    {
        recoveryHorizon = 0f;

        if (!enableTerminalFeasibilityRecovery)
            return false;

        float maximumTotalTerminalTime =
            Mathf.Max(
                terminalTimeBudget,
                terminalTimeBudget +
                Mathf.Max(0f, maximumExtendedTerminalSeconds));

        float remainingRecoveryBudget =
            maximumTotalTerminalTime - terminalElapsed;

        if (remainingRecoveryBudget < terminalMinimumTimeToGo)
            return false;

        float step =
            Mathf.Max(
                0.01f,
                terminalRecoveryProbeStepSeconds);

        float firstCandidate =
            Mathf.Max(
                terminalMinimumTimeToGo,
                step);

        float maximumAccelerationForRecovery =
            Mathf.Max(0f, maximumTerminalAcceleration) *
            Mathf.Clamp01(terminalRecoveryAccelerationSafety01);

        float maximumJerk =
            Mathf.Max(0f, maximumTerminalJerk);

        for (float candidateT = firstCandidate;
             candidateT <= remainingRecoveryBudget + 0.0001f;
             candidateT += step)
        {
            Vector3 futureSubjectPosition =
                subjectPosition +
                subjectVelocity * candidateT;

            Vector3 positionToDeadline =
                futureSubjectPosition -
                ballBody.position;

            Vector3 requiredTotalAcceleration =
                6f * positionToDeadline /
                (candidateT * candidateT) -
                (4f * ballBody.velocity +
                 2f * subjectVelocity) /
                candidateT;

            Vector3 desiredArtificialAcceleration =
                requiredTotalAcceleration -
                Physics.gravity;

            // Flat捕捉後はYを物理接触へ返しているので、
            // Recovery feasibilityも同じ制御自由度だけを評価する。
            if (ballFlatCaptured)
            {
                desiredArtificialAcceleration =
                    Vector3.ProjectOnPlane(
                        desiredArtificialAcceleration,
                        Vector3.up);
            }

            bool accelerationFeasible =
                desiredArtificialAcceleration.magnitude <=
                maximumAccelerationForRecovery + 0.0001f;

            float jerkArrivalTime = 0f;

            if (maximumJerk > 0.0001f)
            {
                jerkArrivalTime =
                    (desiredArtificialAcceleration -
                     terminalAccelerationState).magnitude /
                    maximumJerk;
            }
            else if ((desiredArtificialAcceleration -
                      terminalAccelerationState).sqrMagnitude >
                     0.000001f)
            {
                jerkArrivalTime = float.PositiveInfinity;
            }

            bool jerkFeasible =
                jerkArrivalTime <=
                candidateT *
                Mathf.Clamp01(terminalRecoveryJerkTimeSafety01);

            if (accelerationFeasible && jerkFeasible)
            {
                recoveryHorizon = candidateT;
                return true;
            }
        }

        return false;
    }


    private Transform ContinuousRejoinFrame =>
        respondSubject != null
            ? respondSubject.VisualPlayerRoot
            : null;


    private Vector3 WorldVectorToContinuousLocal(Vector3 worldVector)
    {
        Transform frame = ContinuousRejoinFrame;

        if (frame != null)
            return frame.InverseTransformVector(worldVector);

        return respondSubject != null
            ? respondSubject.InverseMapDirection(worldVector)
            : worldVector;
    }


    private Vector3 ContinuousLocalVectorToWorld(Vector3 localVector)
    {
        Transform frame = ContinuousRejoinFrame;

        if (frame != null)
            return frame.TransformVector(localVector);

        return respondSubject != null
            ? respondSubject.MapDirection(localVector)
            : localVector;
    }


    private void UpdateContinuousFrameAngularVelocitySample()
    {
        Transform frame = ContinuousRejoinFrame;

        if (frame == null)
        {
            hasContinuousFrameSample = false;
            continuousFrameLinearVelocityWorld = Vector3.zero;
            continuousFrameAngularVelocityWorld = Vector3.zero;
            return;
        }

        Vector3 currentPosition = frame.position;
        Quaternion currentRotation = frame.rotation;

        if (!hasContinuousFrameSample)
        {
            hasContinuousFrameSample = true;
            previousContinuousFramePosition = currentPosition;
            previousContinuousFrameRotation = currentRotation;
            continuousFrameLinearVelocityWorld = Vector3.zero;
            continuousFrameAngularVelocityWorld = Vector3.zero;
            return;
        }

        float dt = Mathf.Max(0.0001f, Time.fixedDeltaTime);

        continuousFrameLinearVelocityWorld =
            (currentPosition - previousContinuousFramePosition) / dt;

        Quaternion delta =
            currentRotation *
            Quaternion.Inverse(previousContinuousFrameRotation);

        delta = NormalizeQuaternionSafe(delta);
        delta.ToAngleAxis(out float angleDeg, out Vector3 axis);

        if (angleDeg > 180f)
            angleDeg -= 360f;

        if (Mathf.Abs(angleDeg) <= 0.0001f ||
            axis.sqrMagnitude <= 0.000001f)
        {
            continuousFrameAngularVelocityWorld = Vector3.zero;
        }
        else
        {
            continuousFrameAngularVelocityWorld =
                axis.normalized *
                (angleDeg * Mathf.Deg2Rad / dt);
        }

        previousContinuousFramePosition = currentPosition;
        previousContinuousFrameRotation = currentRotation;
    }


    private Vector3 ReadContinuousMappedSubjectVelocity()
    {
        Vector3 directMappedVelocity =
            ReadMappedInSubjectVelocity();

        Transform frame = ContinuousRejoinFrame;
        if (frame == null || !hasContinuousFrameSample)
            return directMappedVelocity;

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 radiusFromFrameOrigin =
            subjectPosition - frame.position;

        return
            directMappedVelocity +
            continuousFrameLinearVelocityWorld +
            Vector3.Cross(
                continuousFrameAngularVelocityWorld,
                radiusFromFrameOrigin);
    }


    private static Quaternion NormalizeQuaternionSafe(Quaternion q)
    {
        float mag = Mathf.Sqrt(
            q.x * q.x +
            q.y * q.y +
            q.z * q.z +
            q.w * q.w);

        if (mag <= 0.000001f)
            return Quaternion.identity;

        float inv = 1f / mag;
        return new Quaternion(
            q.x * inv,
            q.y * inv,
            q.z * inv,
            q.w * inv);
    }


    private void BeginContinuousRejoin(string reason)
    {
        if (motionPhase != MotionPhase.TerminalRejoin)
            return;

        if (!enableEmergencyVisualRejoin)
        {
            // RecoveryをOFFにしても大距離hard snapへは戻さない。
            // Terminalを最小Time-To-Goで継続し、物理収束だけを許可する。
            terminalActiveTimeBudget =
                Mathf.Max(
                    terminalActiveTimeBudget,
                    terminalElapsed + terminalMinimumTimeToGo);

            if (enableDebugLog)
            {
                Debug.LogWarning(
                    $"[CONTINUOUS REJOIN DISABLED] " +
                    $"reason={reason} " +
                    $"time={Time.fixedTime:F4} " +
                    $"Terminal will continue without hard snap.",
                    this);
            }

            return;
        }

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadContinuousMappedSubjectVelocity();
        Quaternion subjectRotation = respondSubject.MappedRotation;

        Vector3 worldOffset =
            ballBody.position - subjectPosition;

        Vector3 worldRelativeVelocity =
            ballBody.velocity - subjectVelocity;

        // 回転座標系の輸送速度 omega x r を引いたものが、
        // VisualPlayerRootローカルにおける純粋な相対速度 dr/dt。
        Vector3 frameTransportVelocity =
            Vector3.Cross(
                continuousFrameAngularVelocityWorld,
                worldOffset);

        continuousRejoinStartLocalOffset =
            WorldVectorToContinuousLocal(worldOffset);

        continuousRejoinStartLocalVelocity =
            WorldVectorToContinuousLocal(
                worldRelativeVelocity - frameTransportVelocity);

        continuousRejoinStartRelativeRotation =
            NormalizeQuaternionSafe(
                Quaternion.Inverse(subjectRotation) *
                ballBody.rotation);

        float relativeDistance = worldOffset.magnitude;
        float preferredSpeed =
            Mathf.Max(
                1f,
                emergencyVisualPreferredCatchUpSpeed);

        float durationFromDistance =
            relativeDistance / preferredSpeed;

        continuousRejoinDuration =
            Mathf.Clamp(
                Mathf.Max(
                    emergencyVisualMinimumDuration,
                    durationFromDistance),
                Mathf.Max(0.05f, emergencyVisualMinimumDuration),
                Mathf.Max(
                    emergencyVisualMinimumDuration,
                    emergencyVisualMaximumDuration));

        continuousRejoinStartTime = Time.fixedTime;
        continuousRejoinCurrentLocalOffset =
            continuousRejoinStartLocalOffset;
        continuousRejoinCurrentLocalVelocity =
            continuousRejoinStartLocalVelocity;
        continuousRejoinCurrentWorldPosition =
            ballBody.position;
        continuousRejoinCurrentWorldVelocity =
            ballBody.velocity;
        continuousRejoinCurrentWorldRotation =
            ballBody.rotation;

        hasContinuousTrajectorySample = false;
        previousContinuousSampleTime = -1f;
        currentContinuityResidualMeters = 0f;
        maximumContinuityResidualMeters = 0f;
        continuousRejoinWaitingForTurnEndLogged = false;

        // EqualizerはBallVisualの下流。BallVisual自体のPoseはこのクラスだけが所有する。
        if (BallVisualEqualizer != null)
        {
            BallVisualEqualizer.BeginEmergencyVisualRecovery(
                continuousRejoinDuration);
        }

        ballBody.useGravity = false;
        ballBody.detectCollisions = false;

        if (ballCollider != null)
            ballCollider.isTrigger = true;

        ballBody.isKinematic = true;

        terminalAccelerationState = Vector3.zero;
        previousTerminalAccelerationState = Vector3.zero;
        terminalStableFrames = 0;

        motionPhase = MotionPhase.ContinuousRejoin;

        if (enableDebugLog)
        {
            Debug.Log(
                $"[CONTINUOUS REJOIN BEGIN] " +
                $"reason={reason} " +
                $"time={Time.fixedTime:F4} " +
                $"duration={continuousRejoinDuration:F4}s " +
                $"relativeDistance={relativeDistance:F4} " +
                $"relativeSpeed={worldRelativeVelocity.magnitude:F4}",
                this);
        }
    }


    private void ProcessContinuousRejoin()
    {
        if (motionPhase != MotionPhase.ContinuousRejoin)
            return;

        float duration =
            Mathf.Max(
                0.0001f,
                continuousRejoinDuration);

        float elapsed =
            Mathf.Max(
                0f,
                Time.fixedTime - continuousRejoinStartTime);

        float t = Mathf.Clamp01(elapsed / duration);

        continuousRejoinCurrentLocalOffset =
            EvaluateHermitePosition(
                continuousRejoinStartLocalOffset,
                continuousRejoinStartLocalVelocity,
                Vector3.zero,
                Vector3.zero,
                duration,
                t);

        continuousRejoinCurrentLocalVelocity =
            EvaluateHermiteVelocity(
                continuousRejoinStartLocalOffset,
                continuousRejoinStartLocalVelocity,
                Vector3.zero,
                Vector3.zero,
                duration,
                t);

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadContinuousMappedSubjectVelocity();
        Quaternion subjectRotation = respondSubject.MappedRotation;

        Vector3 worldOffset =
            ContinuousLocalVectorToWorld(
                continuousRejoinCurrentLocalOffset);

        Vector3 worldLocalDerivative =
            ContinuousLocalVectorToWorld(
                continuousRejoinCurrentLocalVelocity);

        // d(Rr)/dt = omega x (Rr) + R dr/dt
        Vector3 frameTransportVelocity =
            Vector3.Cross(
                continuousFrameAngularVelocityWorld,
                worldOffset);

        continuousRejoinCurrentWorldPosition =
            subjectPosition + worldOffset;

        continuousRejoinCurrentWorldVelocity =
            subjectVelocity +
            frameTransportVelocity +
            worldLocalDerivative;

        float rotationT =
            t * t * (3f - 2f * t);

        Quaternion relativeRotation =
            Quaternion.Slerp(
                continuousRejoinStartRelativeRotation,
                Quaternion.identity,
                rotationT);

        continuousRejoinCurrentWorldRotation =
            NormalizeQuaternionSafe(
                subjectRotation * relativeRotation);

        ballBody.MovePosition(
            continuousRejoinCurrentWorldPosition);

        ballBody.MoveRotation(
            continuousRejoinCurrentWorldRotation);

        UpdateContinuousTrajectoryDiagnostics(
            continuousRejoinCurrentWorldPosition,
            continuousRejoinCurrentWorldVelocity);

        if (t < 1f)
            return;

        bool visualTurnActive =
            respondSubject != null &&
            respondSubject.IsVisualFrameTurning;

        // t=1では相対位置/速度は数学的に0。
        // 旋回中、またはVisual frameが2 FixedUpdate安定するまではWorld終点を再計画せず、
        // そのままr=0を維持して座標系だけ追従する。
        if (visualTurnActive || !visualFrameStable)
        {
            if (enableDebugLog &&
                !continuousRejoinWaitingForTurnEndLogged)
            {
                continuousRejoinWaitingForTurnEndLogged = true;
                Debug.Log(
                    $"[CONTINUOUS REJOIN TURN HOLD] " +
                    $"time={Time.fixedTime:F4} " +
                    $"turning={visualTurnActive} " +
                    $"frameStable={visualFrameStable} " +
                    $"relativePos={continuousRejoinCurrentLocalOffset.magnitude:F6} " +
                    $"relativeVel={continuousRejoinCurrentLocalVelocity.magnitude:F6}",
                    this);
            }

            return;
        }

        // frame停止後の最終速度は通常同期系と同じcanonical mapped velocityへ戻す。
        subjectVelocity = ReadMappedInSubjectVelocity();
        continuousRejoinCurrentWorldVelocity = subjectVelocity;

        if (TryFinalizeRejoinMicroSync(
                "ContinuousRelativeHermite",
                subjectPosition,
                subjectVelocity,
                continuousRejoinCurrentWorldPosition,
                continuousRejoinCurrentWorldVelocity,
                requestedForced: false,
                out float positionError,
                out float velocityError,
                out _))
        {
            if (enableDebugLog)
            {
                Debug.Log(
                    $"[CONTINUOUS REJOIN COMPLETE] " +
                    $"time={Time.fixedTime:F4} " +
                    $"posError={positionError:F6} " +
                    $"velError={velocityError:F6} " +
                    $"maxResidual={maximumContinuityResidualMeters:F6}",
                    this);
            }

            return;
        }

        // v4ではWorld終点のRetargetを行わない。
        // t=1を保持して次FixedUpdateでも同じr=0条件で最終Micro Syncを再評価する。
        if (enableDebugLog)
        {
            Debug.LogWarning(
                $"[CONTINUOUS REJOIN FINALIZE WAIT] " +
                $"time={Time.fixedTime:F4} " +
                $"posError={positionError:F6} " +
                $"velError={velocityError:F6}",
                this);
        }
    }


    private void UpdateContinuousTrajectoryDiagnostics(
        Vector3 currentPosition,
        Vector3 currentVelocity)
    {
        float now = Time.fixedTime;

        if (!hasContinuousTrajectorySample)
        {
            hasContinuousTrajectorySample = true;
            previousContinuousWorldPosition = currentPosition;
            previousContinuousWorldVelocity = currentVelocity;
            previousContinuousSampleTime = now;
            currentContinuityResidualMeters = 0f;
            return;
        }

        float dt = now - previousContinuousSampleTime;

        if (dt <= 0.000001f)
        {
            currentContinuityResidualMeters = 0f;
            return;
        }

        Vector3 integratedDisplacement =
            0.5f *
            (previousContinuousWorldVelocity + currentVelocity) *
            dt;

        Vector3 actualDisplacement =
            currentPosition - previousContinuousWorldPosition;

        currentContinuityResidualMeters =
            (actualDisplacement - integratedDisplacement).magnitude;

        maximumContinuityResidualMeters =
            Mathf.Max(
                maximumContinuityResidualMeters,
                currentContinuityResidualMeters);

        previousContinuousWorldPosition = currentPosition;
        previousContinuousWorldVelocity = currentVelocity;
        previousContinuousSampleTime = now;
    }


    private static Vector3 EvaluateHermitePosition(
        Vector3 p0,
        Vector3 v0,
        Vector3 p1,
        Vector3 v1,
        float duration,
        float t)
    {
        float t2 = t * t;
        float t3 = t2 * t;

        float h00 = 2f * t3 - 3f * t2 + 1f;
        float h10 = t3 - 2f * t2 + t;
        float h01 = -2f * t3 + 3f * t2;
        float h11 = t3 - t2;

        return
            h00 * p0 +
            h10 * (v0 * duration) +
            h01 * p1 +
            h11 * (v1 * duration);
    }


    private static Vector3 EvaluateHermiteVelocity(
        Vector3 p0,
        Vector3 v0,
        Vector3 p1,
        Vector3 v1,
        float duration,
        float t)
    {
        float safeDuration = Mathf.Max(0.0001f, duration);
        float t2 = t * t;

        float dh00 = 6f * t2 - 6f * t;
        float dh10 = 3f * t2 - 4f * t + 1f;
        float dh01 = -6f * t2 + 6f * t;
        float dh11 = 3f * t2 - 2f * t;

        return
            (dh00 * p0 +
             dh10 * (v0 * safeDuration) +
             dh01 * p1 +
             dh11 * (v1 * safeDuration)) /
            safeDuration;
    }


    /// <summary>
    /// 最終の位置・速度一致を許可する唯一の入口。
    /// 大誤差またはVisual旋回中はfalseを返し、Rigidbody位置を変更しない。
    /// </summary>
    private bool TryFinalizeRejoinMicroSync(
        string source,
        Vector3 subjectPosition,
        Vector3 subjectVelocity,
        Vector3 observedBallPosition,
        Vector3 observedBallVelocity,
        bool requestedForced,
        out float preSyncPositionError,
        out float preSyncVelocityError,
        out bool visualTurnActive)
    {
        preSyncPositionError =
            Vector3.Distance(
                observedBallPosition,
                subjectPosition);

        preSyncVelocityError =
            Vector3.Distance(
                observedBallVelocity,
                subjectVelocity);

        visualTurnActive =
            respondSubject != null &&
            respondSubject.IsVisualFrameTurning;

        float finalPositionTolerance =
            Mathf.Min(
                FinalMicroSyncPositionTolerance,
                Mathf.Max(0.000001f, emergencyVisualPositionTolerance));

        float finalVelocityTolerance =
            Mathf.Min(
                FinalMicroSyncVelocityTolerance,
                Mathf.Max(0.000001f, emergencyVisualVelocityTolerance));

        bool safeToFinalize =
            !visualTurnActive &&
            preSyncPositionError <= finalPositionTolerance &&
            preSyncVelocityError <= finalVelocityTolerance;

        if (!safeToFinalize)
        {
            if (enableDebugLog)
            {
                Debug.LogWarning(
                    $"[FINAL SYNC BLOCKED] source={source} " +
                    $"requestedForced={requestedForced} " +
                    $"time={Time.fixedTime:F4} " +
                    $"turning={visualTurnActive} " +
                    $"preSyncPosError={preSyncPositionError:F4} " +
                    $"preSyncVelError={preSyncVelocityError:F4}",
                    this);
            }

            return false;
        }

        // ここだけがTerminal/ContinuousRejoin後にpositionを直接合わせる場所。
        // 1cm以下・速度差も0.05m/s以下・Visual旋回終了後なので、
        // 見た目上のテレポートではなく最終丸めだけになる。
        ballBody.isKinematic = false;
        ballBody.detectCollisions = true;
        ballBody.useGravity = false;

        ballBody.position = subjectPosition;
        ballBody.velocity = subjectVelocity;
        ballBody.rotation = respondSubject.MappedRotation;
        ballBody.angularVelocity =
            respondSubject.MapDirection(
                inSubjectBody.angularVelocity);

        if (ballCollider != null)
            ballCollider.isTrigger = true;

        if (BallVisualEqualizer != null)
            BallVisualEqualizer.ResumeSynchronization();

        terminalAccelerationState = Vector3.zero;
        previousTerminalAccelerationState = Vector3.zero;
        terminalStableFrames = 0;
        terminalExtendedRecoveryActive = false;

        motionPhase = MotionPhase.Settled;

        if (enableDebugLog)
        {
            Debug.Log(
                $"[FINAL SYNC CHECKPOINT] forced=False " +
                $"source={source} " +
                $"requestedForced={requestedForced} " +
                $"time={Time.fixedTime:F4} " +
                $"turning=False " +
                $"preSyncPosError={preSyncPositionError:F4} " +
                $"preSyncVelError={preSyncVelocityError:F4}",
                this);
        }

        return true;
    }


    private void CompleteTerminalRejoin(bool requestedForced)
    {
        if (motionPhase != MotionPhase.TerminalRejoin)
            return;

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();

        if (TryFinalizeRejoinMicroSync(
                "Terminal",
                subjectPosition,
                subjectVelocity,
                ballBody.position,
                ballBody.velocity,
                requestedForced,
                out float preSyncPositionError,
                out float preSyncVelocityError,
                out bool visualTurnActive))
        {
            return;
        }

        // 旋回途中だけが理由なら、座標系が確定するまでTerminalを維持する。
        if (visualTurnActive)
        {
            terminalActiveTimeBudget =
                Mathf.Max(
                    terminalActiveTimeBudget,
                    terminalElapsed + terminalMinimumTimeToGo);

            terminalTimeToGo = terminalMinimumTimeToGo;
            terminalStableFrames = 0;
            return;
        }

        // 旋回終了済みで誤差が大きい場合もhard snapは禁止。
        // 現在位置/速度を連続初期条件としてContinuous Relative Hermiteへ移譲する。
        BeginContinuousRejoin(
            requestedForced
                ? "UnsafeForcedTerminalComplete"
                : $"UnsafeTerminalComplete_Pos{preSyncPositionError:F3}_Vel{preSyncVelocityError:F3}");
    }

    // =====================================================================
    // Landing contact
    // =====================================================================

    private void OnCollisionEnter(Collision collision)
    {
        TryRegisterLandingContact(collision, "OnCollisionEnter");
    }

    private void OnCollisionStay(Collision collision)
    {
        // Enterが方式切替と同FixedUpdateで取りこぼされた場合の保険。
        TryRegisterLandingContact(collision, "OnCollisionStay");
    }

    private void TryRegisterLandingContact(Collision collision, string source)
    {
        if (ballFlatCaptured || collision == null || collision.contactCount <= 0)
            return;

        bool postLimitPhase =
            motionPhase == MotionPhase.MissileAscent ||
            motionPhase == MotionPhase.MissileChase ||
            motionPhase == MotionPhase.TerminalRejoin;

        if (!postLimitPhase)
            return;

        Transform t = collision.transform;

        if (t.CompareTag("stairway"))
            return;

        bool flatIdentity = t.CompareTag("plane") || HierarchyNameContains(t, "ArcSlab");
        if (!flatIdentity)
            return;

        Vector3 normalSum = Vector3.zero;
        int count = 0;

        for (int i = 0; i < collision.contactCount; i++)
        {
            Vector3 n = collision.GetContact(i).normal;
            if (n.sqrMagnitude <= 0.000001f)
                continue;

            normalSum += n.normalized;
            count++;
        }

        if (count <= 0)
            return;

        Vector3 averageNormal = (normalSum / count).normalized;
        if (Vector3.Dot(averageNormal, Vector3.up) < 0.50f)
            return;

        ballFlatCaptured = true;
        ballFlatTime = Time.fixedTime;

        // 接触した瞬間に過去のY誘導状態を消す。
        missileChaseAccelerationState = Vector3.ProjectOnPlane(
            missileChaseAccelerationState,
            Vector3.up);

        terminalAccelerationState = Vector3.ProjectOnPlane(
            terminalAccelerationState,
            Vector3.up);

        previousTerminalAccelerationState = Vector3.ProjectOnPlane(
            previousTerminalAccelerationState,
            Vector3.up);

        if (enableDebugLog)
        {
            Debug.Log(
                $"[BALL FLAT CONTACT] source={source} " +
                $"time={ballFlatTime:F4} " +
                $"collider={t.name} " +
                $"normal={averageNormal:F4} " +
                $"velocity={ballBody.velocity:F4}",
                this);
        }
    }

    private static bool HierarchyNameContains(Transform transform, string token)
    {
        for (Transform t = transform; t != null; t = t.parent)
        {
            if (t.name.IndexOf(token, System.StringComparison.OrdinalIgnoreCase) >= 0)
                return true;
        }

        return false;
    }

    // =====================================================================
    // Mapping / basis / utilities
    // =====================================================================

    private void UpdateControlBasis()
    {
        Vector3 normal = slopeCore.BallVisualSurfaceNormal;
        Vector3 tangent = slopeCore.BallVisualSlopeTangent;

        normal = respondSubject.MapDirection(normal);
        tangent = respondSubject.MapDirection(tangent);

        if (normal.sqrMagnitude > 0.000001f)
            currentSurfaceNormal = normal.normalized;

        tangent = Vector3.ProjectOnPlane(tangent, currentSurfaceNormal);
        if (tangent.sqrMagnitude > 0.000001f)
            currentSlopeTangent = tangent.normalized;
    }

    private Vector3 CalculateRollingAngularVelocity(
        Vector3 worldVelocity,
        Vector3 surfaceNormal)
    {
        Vector3 planarVelocity = Vector3.ProjectOnPlane(worldVelocity, surfaceNormal);
        const float ballRadius = 0.5f;

        if (planarVelocity.sqrMagnitude <= 0.000001f)
            return Vector3.zero;

        return Vector3.Cross(
            surfaceNormal.normalized,
            planarVelocity) / ballRadius;
    }

    private float GetVerticalGravityMagnitude()
    {
        return Mathf.Max(0.0001f, -Physics.gravity.y);
    }

    private static bool IsFinite(Vector3 value)
    {
        return
            !float.IsNaN(value.x) && !float.IsNaN(value.y) && !float.IsNaN(value.z) &&
            !float.IsInfinity(value.x) && !float.IsInfinity(value.y) && !float.IsInfinity(value.z);
    }

    private void WriteDebugLog()
    {
        if (!enableDebugLog || fixedFrameCounter % Mathf.Max(1, logEveryFixedFrames) != 0)
            return;

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();

        Vector3 observedBallPosition =
            IsContinuousRejoining
                ? continuousRejoinCurrentWorldPosition
                : ballBody.position;

        Vector3 observedBallVelocity =
            IsContinuousRejoining
                ? continuousRejoinCurrentWorldVelocity
                : ballBody.velocity;

        float incidentTargetError =
            incidentPlanValid &&
            motionPhase == MotionPhase.Incident
                ? Vector3.Distance(ballBody.position, incidentTargetPosition)
                : -1f;

        float incidentTimeToGo =
            incidentPlanValid &&
            motionPhase == MotionPhase.Incident
                ? incidentFlightSeconds - incidentElapsed
                : -1f;

        Debug.Log(
            $"[BALL VISUAL CONTROL] " +
            $"time={Time.fixedTime:F3} " +
            $"motionPhase={motionPhase} " +
            $"authority={PoseAuthority} " +
            $"ballFlat={ballFlatCaptured} " +
            $"incidentTgo={incidentTimeToGo:F4} " +
            $"incidentTargetError={incidentTargetError:F4} " +
            $"subjectPos={subjectPosition:F4} " +
            $"ballPos={observedBallPosition:F4} " +
            $"positionError={(subjectPosition - observedBallPosition).magnitude:F4} " +
            $"subjectVel={subjectVelocity:F4} " +
            $"ballVel={observedBallVelocity:F4} " +
            $"velocityError={(subjectVelocity - observedBallVelocity).magnitude:F4} " +
            $"continuityResidual={currentContinuityResidualMeters:F6}",
            this);
    }

    private void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying)
            return;

        if (incidentPlanValid)
        {
            Gizmos.DrawWireSphere(incidentTargetPosition, 0.15f);
            Gizmos.DrawLine(incidentStartPosition, incidentTargetPosition);
        }

        if (limitCrossingTime >= 0f)
            Gizmos.DrawWireSphere(limitCrossingPosition, 0.12f);
    }
}
