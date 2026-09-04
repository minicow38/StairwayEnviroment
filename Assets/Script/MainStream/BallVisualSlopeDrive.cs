using UnityEngine;
using Sirenix.OdinInspector;

[Searchable]
[DefaultExecutionOrder(200)]
[RequireComponent(typeof(Rigidbody), typeof(SphereCollider))]
public class BallVisualSlopeDrive : MonoBehaviour
{
    private enum VisualPhase
    {
        WaitingSync,
        Incident,
        Missile,
        TerminalRejoin,
        SettledSync
    }

    private enum MotionPhase
    {
        Waiting,
        Incident,
        MissileAscent,
        MissileChase,
        TerminalRejoin,
        Settled
    }

    public int regionTurn = 0;

    [Header("References")]
    [SerializeField] private SlopeStickCore slopeCore;

    [SerializeField] private BallVisualEqualizerSync BallVisualEqualizer;

    [SerializeField] private CorrespondSubject respondSubject;

    [Header("1st POP - Incident Method")]
    [Tooltip("1st POPのSubject相対Apex高さ[m]")]
    [Min(0f)]
    [SerializeField] private float preLimitTargetHeightRelativeToSubject = 0.45f;

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

    [Header("Debug")]
    [SerializeField] private bool enableDebugLog = true;
    [Min(1)]
    [SerializeField] private int logEveryFixedFrames = 10;

    private Rigidbody ballBody;
    private Rigidbody inSubjectBody;
    private SphereCollider ballCollider;

    private VisualPhase visualPhase = VisualPhase.WaitingSync;
    private MotionPhase motionPhase = MotionPhase.Waiting;
    private int fixedFrameCounter;

    // VisualPlayerRootの回転Tween中に、途中角度の座標系でIncidentを開始しない。
    // Inspector調整値にはせず、方式境界の安全条件として固定する。
    private const float VisualFrameStableAngleEpsilonDeg = 0.05f;
    private const int VisualFrameStableFixedFramesRequired = 2;
    private Vector3 previousVisualFrameForward;
    private bool hasPreviousVisualFrameForward;
    private int visualFrameStableFrames;
    private bool visualFrameStable;

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

        bool isFlat = slopeCore.BallVisualIsOnFlat;
        bool isOnSlope = slopeCore.BallVisualIsOnSlope;
        bool canBeginIncident = CanBeginIncident(isOnSlope);

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
            visualPhase = VisualPhase.WaitingSync;
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
        visualPhase = VisualPhase.SettledSync;
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
            incidentNormalSpeed;

        float incidenceDerivedRelativeUp =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    reflectedNormalVelocity,
                    Vector3.up));

        // 入射法線成分が成立しないケース
        // （下り斜面・すでに斜面接線へ整列済み等）では、
        // 既存の v^2 同期則へFallbackしてIncidentを失わない。
        float preferredRelativeUp =
            incidenceDerivedRelativeUp > 0.0001f
                ? incidenceDerivedRelativeUp
                : ResolveIncidentPreferredRelativeUpSpeed(
                    referenceSpeed);

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
                $"angle={incidentAngleDeg:F3}deg " +
                $"reconstructed={reconstructedSpeed:F4}m/s " +
                $"tangentVelocity={incidentTangentVelocity:F4} " +
                $"inwardNormalVelocity={incidentInwardNormalVelocity:F4} " +
                $"reflectedNormalVelocity={reflectedNormalVelocity:F4} " +
                $"incidenceDerivedUp={incidenceDerivedRelativeUp:F4}m/s " +
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
        float decoratedFlight = ResolveIncidentDecoratedFlightSeconds(
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

        float visibleRelativeUp = extremelyShortIncident
            ? 0f
            : IncidentVisibilityMinimumRelativeUpSpeed;

        // Preferred POP高さの86%以上は維持する。
        float minimumRelativeUp = Mathf.Max(
            visibleRelativeUp,
            preferredRelativeUp * Mathf.Sqrt(IncidentEnergyMinimumHeightRetention));

        // 成功版のDecorationは最大2.6m/sまでの相対Upを許可していた。
        // preferredRelativeUpがそれ以上なら当然そちらを上限として尊重する。
        float maximumRelativeUp = extremelyShortIncident
            ? preferredRelativeUp
            : Mathf.Max(
                preferredRelativeUp,
                IncidentPopDecorationMaximumRelativeUpSpeed);

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

        float incidentNormalEnergy =
            0.5f *
            ballBody.mass *
            incidentNormalSpeed *
            incidentNormalSpeed;

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
        visualPhase = VisualPhase.Incident;

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
        visualPhase = VisualPhase.Missile;

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
        terminalTimeToGo = Mathf.Max(terminalTimeBudget, terminalMinimumTimeToGo);
        terminalAccelerationState = missileChaseAccelerationState;
        previousTerminalAccelerationState = terminalAccelerationState;
        terminalStableFrames = 0;

        motionPhase = MotionPhase.TerminalRejoin;
        visualPhase = VisualPhase.TerminalRejoin;
    }

    private void ProcessTerminalRejoin()
    {
        if (motionPhase != MotionPhase.TerminalRejoin)
            return;

        float dt = Mathf.Max(Time.fixedDeltaTime, 0.000001f);
        terminalElapsed = Mathf.Max(0f, Time.fixedTime - terminalStartTime);

        float rawTimeToGo = terminalTimeBudget - terminalElapsed;
        terminalTimeToGo = Mathf.Max(terminalMinimumTimeToGo, rawTimeToGo);

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();

        // Keep the successful Terminal formula unchanged.
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

        // Flat接触後はTerminalもXZだけ。Yは物理接触へ完全返却。
        if (ballFlatCaptured)
            desiredArtificialAcceleration = Vector3.ProjectOnPlane(
                desiredArtificialAcceleration,
                Vector3.up);

        desiredArtificialAcceleration = Vector3.ClampMagnitude(
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

        Vector3 positionError = subjectPosition - ballBody.position;
        Vector3 velocityError = subjectVelocity - ballBody.velocity;

        // Flat接触後の同期成立判定もXZだけを見る。
        if (ballFlatCaptured)
        {
            positionError = Vector3.ProjectOnPlane(positionError, Vector3.up);
            velocityError = Vector3.ProjectOnPlane(velocityError, Vector3.up);
        }

        bool ready =
            ballFlatCaptured &&
            positionError.magnitude <= terminalPositionTolerance &&
            velocityError.magnitude <= terminalVelocityTolerance;

        terminalStableFrames = ready ? terminalStableFrames + 1 : 0;

        if (terminalStableFrames >= Mathf.Max(1, terminalStableFramesRequired))
        {
            CompleteTerminalRejoin(false);
            return;
        }

        // 決められたTerminal時間の最後でだけ完全同期を許可。
        if (rawTimeToGo <= 0f)
        {
            CompleteTerminalRejoin(true);
            return;
        }

        if (enableDebugLog && fixedFrameCounter % Mathf.Max(1, logEveryFixedFrames) == 0)
        {
            Debug.Log(
                $"[TERMINAL REJOIN] elapsed={terminalElapsed:F4}s " +
                $"Tgo={terminalTimeToGo:F4}s " +
                $"ballFlat={ballFlatCaptured} " +
                $"posError={positionError.magnitude:F4} " +
                $"velError={velocityError.magnitude:F4} " +
                $"verticalOwner={(ballFlatCaptured ? "PhysicsContact" : "Guidance")}",
                this);
        }
    }

    private void CompleteTerminalRejoin(bool forced)
    {
        if (regionTurn == 1)
        {
            Debug.Log("");
        }
        if (motionPhase != MotionPhase.TerminalRejoin)
            return;

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();

        float preSyncPositionError = Vector3.Distance(ballBody.position, subjectPosition);
        float preSyncVelocityError = Vector3.Distance(ballBody.velocity, subjectVelocity);

        // この一箇所だけがMissile後の完全同期点。
        ballBody.position = subjectPosition;
       //ballBody.velocity = subjectVelocity;
        ballBody.rotation = respondSubject.MappedRotation;
        ballBody.angularVelocity = respondSubject.MapDirection(inSubjectBody.angularVelocity);
        ballBody.useGravity = false;

        if (ballCollider != null)
            ballCollider.isTrigger = true;

        // Equalizerは次のStage Turnより前に必ずBallVisualへ戻す。
        // Turn中に独立したStable-N Hopperを持ち越さないことで、
        // CorrespondSubjectの座標写像とEqualizer物理を競合させない。
       /* if (BallVisualEqualizer != null)
            BallVisualEqualizer.ResumeSynchronization();*/

        terminalAccelerationState = Vector3.zero;
        previousTerminalAccelerationState = Vector3.zero;
        terminalStableFrames = 0;

        motionPhase = MotionPhase.Settled;
        visualPhase = VisualPhase.SettledSync;
        regionTurn++;
        if (enableDebugLog)
        {
            Debug.Log(
                $"[FINAL SYNC CHECKPOINT] forced={forced} " +
                $"time={Time.fixedTime:F4} " +
                $"preSyncPosError={preSyncPositionError:F4} " +
                $"preSyncVelError={preSyncVelocityError:F4}",
                this);
        }
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
            $"visualPhase={visualPhase} " +
            $"motionPhase={motionPhase} " +
            $"ballFlat={ballFlatCaptured} " +
            $"incidentTgo={incidentTimeToGo:F4} " +
            $"incidentTargetError={incidentTargetError:F4} " +
            $"subjectPos={subjectPosition:F4} " +
            $"ballPos={ballBody.position:F4} " +
            $"positionError={(subjectPosition - ballBody.position).magnitude:F4} " +
            $"subjectVel={subjectVelocity:F4} " +
            $"ballVel={ballBody.velocity:F4} " +
            $"velocityError={(subjectVelocity - ballBody.velocity).magnitude:F4}",
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
