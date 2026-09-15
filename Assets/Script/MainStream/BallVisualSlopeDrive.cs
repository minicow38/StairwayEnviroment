using UnityEngine;
using Sirenix.OdinInspector;

[Searchable]
[DefaultExecutionOrder(200)]
[RequireComponent(typeof(Rigidbody), typeof(SphereCollider))]
public class BallVisualSlopeDrive : MonoBehaviour
{
    public const string RuntimeBuildId = "BallVisualSlopeDrive-NaturalEntryConnect-v6";

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
    
    [Header("References")]
    [SerializeField] private SlopeStickCore slopeCore;

    [SerializeField] private BallVisualEqualizerSync BallVisualEqualizer;

    [SerializeField] private CorrespondSubject respondSubject;

    [Header("Turn Landing Intent")]
    [Tooltip("Natural EntryのTarget Progress intentを旋回判定へREAD ONLY公開する保持時間[s]。")]
    [Min(0.1f)]
    [SerializeField] private float turnLandingIntentRetentionSeconds = 3.0f;

    [Header("Natural Plane -> Stair Connect")]
    [Tooltip(
        "ON: Plane -> Stair入口では予定放物線/人工POP/Incident加速度を作らず、" +
        "直前までのBallVisual Rigidbody速度 + Gravity + 実Colliderだけで自然遷移します。")]
    [SerializeField] private bool useNaturalPlaneToStairEntry = true;

    [Tooltip(
        "BallVisualEqualizerへ渡す4R-Hn/Hybrid波形の基準高さ[m]。" +
        "入口初速を生成する値ではなく、Hybrid側の波形スケールだけに使います。")]
    [Min(0.01f)]
    [SerializeField] private float oscillationReferenceHeight = 0.45f;

    [Header("Terminal Compatibility")]
    [Tooltip("Legacy terminal timing override. Keep false during normal play.")]
    public bool RawTimeJurge = false;

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

    private Vector3 currentSurfaceNormal = Vector3.up;
    private Vector3 currentSlopeTangent = Vector3.forward;

    // ---------- Natural entry runtime ----------
    // Plane -> Stair入口は軌道を計画しない。
    // この状態はTarget Progress crossingを観測するための最小情報だけを保持する。
    private float incidentStartTime = -1f;
    private float incidentElapsed;
    private bool incidentPlanValid;
    private Vector3 incidentStartPosition;
    private Vector3 incidentTargetPosition;
    private Vector3 incidentPlanarTravelDirection = Vector3.forward;
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
        IncidentEnergyTarget, // legacy numeric alias
        NaturalEntryTarget = IncidentEnergyTarget,
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
    /// 最優先はNatural EntryのTarget Progress位置。
    /// 入口ではEnergy/Impact velocityを計画しないため、速度は現在の実測Rigidbody値を返します。
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
                source = TurnLandingIntentSource.NaturalEntryTarget,
                positionVisual = incidentTargetPosition,
                velocityVisual = ballBody != null
                    ? ballBody.velocity
                    : ReadMappedInSubjectVelocity(),
                confidence01 = 0.90f,
                energyBacked = false,
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
        // Entry開始条件を満たしたFixedUpdateだけNatural Rigidbody区間へ渡す。
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

        // Natural Entry区間。位置/速度のSubject hard syncは禁止。
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
            return;
        }

        mappedForward.Normalize();

        if (!hasPreviousVisualFrameForward)
        {
            previousVisualFrameForward = mappedForward;
            hasPreviousVisualFrameForward = true;
            visualFrameStableFrames = 0;
            visualFrameStable = false;
            return;
        }

        float angleChange =
            Vector3.Angle(
                previousVisualFrameForward,
                mappedForward);

        previousVisualFrameForward = mappedForward;

        if (angleChange <= VisualFrameStableAngleEpsilonDeg)
            visualFrameStableFrames++;
        else
            visualFrameStableFrames = 0;

        visualFrameStable =
            visualFrameStableFrames >=
            VisualFrameStableFixedFramesRequired;
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
    // Natural Plane -> Stair Entry
    // =====================================================================
    // No planned projectile, no POP synthesis, no SlopeStick acceleration feed.
    // BallVisual simply leaves the synchronized phase with its current Rigidbody
    // state. Gravity and real collisions own the entry trajectory.

    private void BeginIncidentMethod()
    {
        if (motionPhase != MotionPhase.Waiting ||
            !CanBeginIncident(slopeCore.BallVisualIsOnSlope) ||
            !TryResolveExactIncidentTarget(out Vector3 exactTarget))
        {
            return;
        }

        if (!useNaturalPlaneToStairEntry)
            return;

        // Waitingの直前FixedUpdateまでSubjectへ同期されているため、
        // ここではposition/velocityを書き直さない。現在値をそのまま初期条件にする。
        if (ballBody.isKinematic)
            ballBody.isKinematic = false;

        ballBody.useGravity = true;

        // Natural entryでは実Colliderを無効化しない。
        if (ballCollider != null)
            ballCollider.isTrigger = false;

        incidentStartTime = Time.fixedTime;
        incidentElapsed = 0f;
        incidentStartPosition = ballBody.position;
        incidentTargetPosition = exactTarget;

        Vector3 planarDisplacement =
            Vector3.ProjectOnPlane(
                exactTarget - ballBody.position,
                Vector3.up);

        incidentPlanarTravelDirection =
            planarDisplacement.sqrMagnitude > 0.000001f
                ? planarDisplacement.normalized
                : Vector3.ProjectOnPlane(
                    currentSlopeTangent,
                    Vector3.up).normalized;

        if (incidentPlanarTravelDirection.sqrMagnitude <= 0.000001f)
            incidentPlanarTravelDirection = Vector3.forward;

        incidentPlanValid = true;
        incidentMaximumObservedPlanarSeparation = 0f;

        hasPreviousIncidentSample = true;
        previousIncidentTime = Time.fixedTime;
        previousIncidentBallPosition = ballBody.position;
        previousIncidentBallVelocity = ballBody.velocity;
        previousIncidentSubjectPosition = respondSubject.MappedPosition;
        previousIncidentSubjectVelocity = ReadMappedInSubjectVelocity();

        bool connectArmed = false;

        if (BallVisualEqualizer != null)
        {
            connectArmed =
                BallVisualEqualizer.BeginNaturalConnectFromBallVisual(
                    Mathf.Max(0.01f, oscillationReferenceHeight),
                    currentSurfaceNormal,
                    currentSlopeTangent);
        }

        motionPhase = MotionPhase.Incident;

        if (enableDebugLog)
        {
            Debug.Log(
                $"[NATURAL ENTRY BEGIN] " +
                $"time={Time.fixedTime:F4} " +
                $"position={ballBody.position:F4} " +
                $"velocity={ballBody.velocity:F4} " +
                $"gravity={Physics.gravity:F4} " +
                $"target={incidentTargetPosition:F4} " +
                $"normal={currentSurfaceNormal:F4} " +
                $"tangent={currentSlopeTangent:F4} " +
                $"equalizerConnectArmed={connectArmed} " +
                $"colliderSolid={(ballCollider == null || !ballCollider.isTrigger)}",
                this);
        }
    }


    private void ProcessIncident()
    {
        if (!incidentPlanValid)
            return;

        incidentElapsed =
            Mathf.Max(
                0f,
                Time.fixedTime - incidentStartTime);

        // Translation is intentionally force-free here.
        // No AddForce / VelocityChange / planned projectile correction is used.
        // Only Unity gravity and actual collider impulses change linear velocity.
        if (ballCollider != null && ballCollider.isTrigger)
            ballCollider.isTrigger = false;

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 planarSeparation =
            Vector3.ProjectOnPlane(
                ballBody.position - subjectPosition,
                Vector3.up);

        incidentMaximumObservedPlanarSeparation =
            Mathf.Max(
                incidentMaximumObservedPlanarSeparation,
                planarSeparation.magnitude);

        // Rotation follow is presentation-only torque; it does not generate
        // Plane -> Stair translational jump energy.
        ApplyIncidentRotationFollow();
    }


    private void ApplyIncidentRotationFollow()
    {
        Vector3 targetAngularVelocity = Vector3.ClampMagnitude(
            CalculateRollingAngularVelocity(
                ballBody.velocity,
                Vector3.up),
            30f);

        Vector3 angularVelocityError =
            targetAngularVelocity - ballBody.angularVelocity;

        Vector3 angularAcceleration = Vector3.ClampMagnitude(
            angularVelocityError * 12f,
            72f);

        ballBody.AddTorque(
            angularAcceleration,
            ForceMode.Acceleration);
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
        // Natural EntryではYをTargetへ解かない。
        // crossingは進行方向のTarget Progress面だけで判定し、Yは実Rigidbody結果を尊重する。
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

            Debug.Log(
                $"[NATURAL ENTRY -> MISSILE BOUNDARY] " +
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
                $"naturalEntryCost={(limitCrossingTime - incidentStartTime):F4}s",
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

        float incidentAge =
            incidentPlanValid &&
            motionPhase == MotionPhase.Incident
                ? incidentElapsed
                : -1f;

        Debug.Log(
            $"[BALL VISUAL CONTROL] " +
            $"time={Time.fixedTime:F3} " +
            $"motionPhase={motionPhase} " +
            $"authority={PoseAuthority} " +
            $"ballFlat={ballFlatCaptured} " +
            $"naturalEntryAge={incidentAge:F4} " +
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
