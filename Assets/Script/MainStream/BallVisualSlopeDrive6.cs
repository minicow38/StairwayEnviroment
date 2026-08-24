using UnityEngine;
using Sirenix.OdinInspector;

[Searchable]
[RequireComponent(typeof(Rigidbody))]
public class BallVisualSlopeDrive7 : MonoBehaviour
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
        IncidentAscent,
        IncidentDescent,
        MissileAscent,
        MissileChase,
        TerminalRejoin,
        Settled
    }

    [Header("References")]
    [SerializeField] private SlopeStick3D slopeStick;
    [SerializeField] private CorrespondSubject respondSubject;

    [Header("1st POP - Incident Method")]
    [Tooltip("1st POPのSubject相対Apex高さ[m]")]
    [Min(0f)]
    [SerializeField] private float preLimitTargetHeightRelativeToSubject = 0.45f;

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

    [Tooltip("相対上向き速度がこの値以下ならApex通過")]
    [Min(0f)]
    [SerializeField] private float incidentApexVerticalSpeedThreshold = 0.10f;

    [Tooltip("Apex検出待ちの最大時間[s]")]
    [Min(0.05f)]
    [SerializeField] private float incidentMaximumAscentSeconds = 0.60f;

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
    private float incidentApexTime = -1f;
    private float incidentElapsed;
    private bool incidentHadPositiveUpSpeed;
    private Vector3 incidentPlanarAccelerationState;

    private bool incidentPlanValid;
    private Vector3 incidentStartPosition;
    private Vector3 incidentLaunchVelocity;
    private Vector3 incidentImpactVelocity;
    private Vector3 incidentTargetPosition;
    private float incidentFlightSeconds;
    private float incidentResolvedHeight;

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

    private void Awake()
    {
        ballBody = GetComponent<Rigidbody>();
        ballCollider = GetComponent<SphereCollider>();
    }

    private void Start()
    {
        if (respondSubject == null)
        {
            GameObject subjectVisual = GameObject.Find("subject");
            if (subjectVisual != null)
                respondSubject = subjectVisual.GetComponent<CorrespondSubject>();
        }

        if (slopeStick == null)
        {
            GameObject subjectObject = GameObject.Find("/PlayerRoot/subject");
            if (subjectObject != null)
                slopeStick = subjectObject.GetComponent<SlopeStick3D>();
        }

        if (slopeStick != null)
            inSubjectBody = slopeStick.GetComponent<Rigidbody>();

        if (ballBody == null || ballCollider == null || slopeStick == null || respondSubject == null || inSubjectBody == null)
        {
            Debug.LogError("[BALL VISUAL] 必要な参照が設定されていません。", this);
            enabled = false;
            return;
        }

        SyncCompletelyToSubject("InitialSync");
    }

    private void FixedUpdate()
    {
        fixedFrameCounter++;
        UpdateControlBasis();
        UpdateVisualFrameStability();

        bool isFlat = slopeStick.BallVisualIsOnFlat;
        bool isOnSlope = slopeStick.BallVisualIsOnSlope;
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
        if (motionPhase == MotionPhase.IncidentAscent ||
            motionPhase == MotionPhase.IncidentDescent)
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

        if (!slopeStick.BallVisualHasActiveSlopeFrame)
            return false;

        // Target Progressより後ろ側(+側)からIncidentを撃たない。
        // 右回転直後にArcSlabがprogress=100%としてSlope判定された
        // ケースをここで除外する。
        if (!(slopeStick.slopeProgressErrorPercent < 0f))
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
            !slopeStick.BallVisualHasActiveSlopeFrame ||
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
            $"progressError={slopeStick.slopeProgressErrorPercent:F3}% " +
            $"visualFrameStable={visualFrameStable} " +
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
        // SlopeStick3D / InSubjectはREAD ONLY。
        // 同一FixedUpdateでSlopeStick3Dが更新した最新Rigidbody速度をVisual座標へ写す。
        return respondSubject.MapDirection(inSubjectBody.velocity);
    }

    // =====================================================================
    // 1st POP / Incident Method
    // =====================================================================

    private void BeginIncidentMethod()
    {
        if (motionPhase != MotionPhase.Waiting)
            return;

        // FixedUpdate側の入口判定を別経路から呼ばれても迂回させない。
        if (!CanBeginIncident(slopeStick.BallVisualIsOnSlope))
            return;

        if (!TryResolveExactIncidentTarget(out Vector3 exactTarget))
            return;

        // 決められた区間入口でのみ完全同期。
        SyncCompletelyToSubject("IncidentEntry");

        ballBody.useGravity = true;

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();

        incidentResolvedHeight = ResolveIncidentHeight();
        float relativeUpSpeed = CalculateVerticalSpeedForHeight(incidentResolvedHeight);
        float launchVerticalSpeed = subjectVelocity.y + relativeUpSpeed;

        float flightSeconds = SolveBallisticTimeToVerticalTarget(
            subjectPosition.y,
            exactTarget.y,
            launchVerticalSpeed);

        if (!(flightSeconds > 0f) || float.IsNaN(flightSeconds) || float.IsInfinity(flightSeconds))
            return;

        Vector3 planarDisplacement = Vector3.ProjectOnPlane(
            exactTarget - subjectPosition,
            Vector3.up);

        Vector3 planarLaunchVelocity =
            planarDisplacement / Mathf.Max(flightSeconds, 0.0001f);

        Vector3 launchVelocity =
            planarLaunchVelocity + Vector3.up * launchVerticalSpeed;

        Vector3 impactVelocity =
            planarLaunchVelocity +
            Vector3.up * (launchVerticalSpeed + Physics.gravity.y * flightSeconds);

        ballBody.velocity = launchVelocity;

        Vector3 planarSubjectVelocity = Vector3.ProjectOnPlane(subjectVelocity, Vector3.up);
        ballBody.angularVelocity = CalculateRollingAngularVelocity(planarSubjectVelocity, Vector3.up);

        if (incidentUseTriggerDuringFlight && ballCollider != null)
            ballCollider.isTrigger = true;

        incidentPlanValid = true;
        incidentStartTime = Time.fixedTime;
        incidentApexTime = -1f;
        incidentElapsed = 0f;
        incidentHadPositiveUpSpeed = relativeUpSpeed > incidentApexVerticalSpeedThreshold;
        incidentPlanarAccelerationState = Vector3.zero;

        incidentStartPosition = subjectPosition;
        incidentLaunchVelocity = launchVelocity;
        incidentImpactVelocity = impactVelocity;
        incidentTargetPosition = exactTarget;
        incidentFlightSeconds = flightSeconds;

        hasPreviousIncidentSample = false;
        previousIncidentTime = Time.fixedTime;
        previousIncidentBallPosition = ballBody.position;
        previousIncidentBallVelocity = ballBody.velocity;
        previousIncidentSubjectPosition = respondSubject.MappedPosition;
        previousIncidentSubjectVelocity = ReadMappedInSubjectVelocity();

        motionPhase = MotionPhase.IncidentAscent;
        visualPhase = VisualPhase.Incident;

        if (enableDebugLog)
        {
            float impactPlanarSpeed = Vector3.ProjectOnPlane(impactVelocity, Vector3.up).magnitude;
            float impactDownSpeed = Mathf.Max(0f, -impactVelocity.y);
            float impactAngle = Mathf.Atan2(
                impactDownSpeed,
                Mathf.Max(0.0001f, impactPlanarSpeed)) * Mathf.Rad2Deg;

            Debug.Log(
                $"[FIRST POP INCIDENT] " +
                $"height={incidentResolvedHeight:F4}m " +
                $"subjectVelocity={subjectVelocity:F4} " +
                $"launchVelocity={launchVelocity:F4} " +
                $"flight={flightSeconds:F4}s " +
                $"target={exactTarget:F4} " +
                $"plannedImpactVelocity={impactVelocity:F4} " +
                $"plannedImpactAngle={impactAngle:F3}deg",
                this);
        }
    }

    private void ProcessIncident()
    {
        if (!incidentPlanValid)
            return;

        incidentElapsed = Mathf.Max(0f, Time.fixedTime - incidentStartTime);
        ApplyIncidentPlanarCorrection();

        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();
        float relativeUpSpeed = Vector3.Dot(
            ballBody.velocity - subjectVelocity,
            Vector3.up);

        if (relativeUpSpeed > incidentApexVerticalSpeedThreshold)
            incidentHadPositiveUpSpeed = true;

        if (motionPhase == MotionPhase.IncidentAscent)
        {
            bool relativeApex =
                incidentHadPositiveUpSpeed &&
                relativeUpSpeed <= incidentApexVerticalSpeedThreshold;

            bool timeout = incidentElapsed >= incidentMaximumAscentSeconds;

            if (relativeApex || timeout)
            {
                incidentApexTime = Time.fixedTime;
                motionPhase = MotionPhase.IncidentDescent;

                if (enableDebugLog)
                {
                    Debug.Log(
                        $"[INCIDENT APEX] time={incidentApexTime:F4} " +
                        $"reason={(relativeApex ? "RelativeApex" : "Timeout")} " +
                        $"ballPos={ballBody.position:F4}",
                        this);
                }
            }
        }
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

    private void ObserveIncidentLimitCrossing()
    {
        if (!incidentPlanValid)
            return;

        Vector3 planarTravel = Vector3.ProjectOnPlane(
            incidentTargetPosition - incidentStartPosition,
            Vector3.up);

        if (planarTravel.sqrMagnitude <= 0.000001f)
            planarTravel = Vector3.ProjectOnPlane(incidentLaunchVelocity, Vector3.up);

        if (planarTravel.sqrMagnitude <= 0.000001f)
            return;

        Vector3 travelDirection = planarTravel.normalized;
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

        float previousRemaining = Vector3.Dot(
            incidentTargetPosition - previousIncidentBallPosition,
            travelDirection);

        float currentRemaining = Vector3.Dot(
            incidentTargetPosition - currentBallPosition,
            travelDirection);

        bool crossed = previousRemaining > 0f && currentRemaining <= 0f;

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

        limitReferencePosition = incidentTargetPosition;
        limitReferenceVelocity = Vector3.Lerp(
            previousIncidentSubjectVelocity,
            currentSubjectVelocity,
            limitCrossingAlpha);

        // 決められた方式間境界でのみ同期。
        // 位置はFixedUpdate内の真のcrossing位置へ戻すだけで、SubjectへTeleportしない。
        ballBody.position = limitCrossingPosition;
        ballBody.velocity = limitIncomingVelocity;

        if (ballCollider != null)
            ballCollider.isTrigger = false;

        if (enableDebugLog)
        {
            float targetError = Vector3.Distance(limitCrossingPosition, incidentTargetPosition);
            float velocityError = Vector3.Distance(limitIncomingVelocity, incidentImpactVelocity);

            Debug.Log(
                $"[INCIDENT -> MISSILE BOUNDARY] " +
                $"time={limitCrossingTime:F4} alpha={limitCrossingAlpha:F4} " +
                $"crossPos={limitCrossingPosition:F4} " +
                $"target={incidentTargetPosition:F4} " +
                $"positionError={targetError:F4} " +
                $"incomingVelocity={limitIncomingVelocity:F4} " +
                $"plannedImpactVelocity={incidentImpactVelocity:F4} " +
                $"velocityError={velocityError:F4}",
                this);
        }

        BeginMissileMethod();
    }

    private bool TryResolveExactIncidentTarget(out Vector3 targetPosition)
    {
        targetPosition = Vector3.zero;

        if (!slopeStick.BallVisualHasActiveSlopeFrame)
            return false;

        Vector3 physicsTarget = slopeStick.BallVisualTargetProgressCenterPhysics;
        Vector3 physicsOffset = physicsTarget - inSubjectBody.position;

        targetPosition =
            respondSubject.MappedPosition +
            respondSubject.MapDirection(physicsOffset);

        return IsFinite(targetPosition);
    }

    private float SolveBallisticTimeToVerticalTarget(
        float startY,
        float targetY,
        float launchVerticalSpeed)
    {
        float a = 0.5f * Physics.gravity.y;
        float b = launchVerticalSpeed;
        float c = startY - targetY;

        if (Mathf.Abs(a) <= 0.000001f)
        {
            if (Mathf.Abs(b) <= 0.000001f)
                return -1f;

            float t = -c / b;
            return t > 0f ? t : -1f;
        }

        float discriminant = b * b - 4f * a * c;
        if (discriminant < 0f)
            return -1f;

        float sqrtD = Mathf.Sqrt(discriminant);
        float denominator = 2f * a;
        float t0 = (-b - sqrtD) / denominator;
        float t1 = (-b + sqrtD) / denominator;

        float minimum = Mathf.Max(Time.fixedDeltaTime * 0.5f, 0.0001f);
        float best = float.PositiveInfinity;

        if (t0 >= minimum)
            best = t0;
        if (t1 >= minimum && t1 < best)
            best = t1;

        return float.IsInfinity(best) ? -1f : best;
    }

    private float ResolveIncidentHeight()
    {
        float speed = ResolveSubjectReferenceSpeed();
        float gravity = GetVerticalGravityMagnitude();

        float energyHeight =
            Mathf.Max(0f, firstPopVelocitySquaredRatio) *
            speed * speed /
            (2f * Mathf.Max(gravity, 0.0001f));

        energyHeight = Mathf.Clamp(
            energyHeight,
            minimumEnergyScaledPopHeight,
            maximumEnergyScaledPopHeight);

        // ベンチマーク中はv^2式を主役にする。
        // speedが取れない時だけ既定高さへFallback。
        return speed > 0.05f
            ? energyHeight
            : Mathf.Max(0f, preLimitTargetHeightRelativeToSubject);
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
            : Mathf.Max(0f, slopeStick.CapturedTargetTangentSpeed);
    }

    private float CalculateVerticalSpeedForHeight(float targetHeight)
    {
        return Mathf.Sqrt(
            2f *
            GetVerticalGravityMagnitude() *
            Mathf.Max(0f, targetHeight));
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

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();

        Vector3 shadowTargetPosition =
            subjectPosition + subjectVelocity * missileChaseLeadSeconds;

        Vector3 positionError = shadowTargetPosition - ballBody.position;
        Vector3 velocityError = subjectVelocity - ballBody.velocity;

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

        bool subjectFlat = isFlat || slopeStick.groundKind == GroundKind.Flat;

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

        Vector3 terminalTargetPosition =
            subjectPosition + subjectVelocity * terminalTimeToGo;

        Vector3 positionToDeadline = terminalTargetPosition - ballBody.position;
        Vector3 velocityNow = ballBody.velocity;
        float safeT = Mathf.Max(terminalTimeToGo, terminalMinimumTimeToGo);

        Vector3 requiredTotalAcceleration =
            6f * positionToDeadline / (safeT * safeT) -
            (4f * velocityNow + 2f * subjectVelocity) / safeT;

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
        if (motionPhase != MotionPhase.TerminalRejoin)
            return;

        Vector3 subjectPosition = respondSubject.MappedPosition;
        Vector3 subjectVelocity = ReadMappedInSubjectVelocity();

        float preSyncPositionError = Vector3.Distance(ballBody.position, subjectPosition);
        float preSyncVelocityError = Vector3.Distance(ballBody.velocity, subjectVelocity);

        // この一箇所だけがMissile後の完全同期点。
        ballBody.position = subjectPosition;
        ballBody.velocity = subjectVelocity;
        ballBody.rotation = respondSubject.MappedRotation;
        ballBody.angularVelocity = respondSubject.MapDirection(inSubjectBody.angularVelocity);
        ballBody.useGravity = false;

        if (ballCollider != null)
            ballCollider.isTrigger = true;

        terminalAccelerationState = Vector3.zero;
        previousTerminalAccelerationState = Vector3.zero;
        terminalStableFrames = 0;

        motionPhase = MotionPhase.Settled;
        visualPhase = VisualPhase.SettledSync;

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
        Vector3 normal = slopeStick.BallVisualSurfaceNormal;
        Vector3 tangent = slopeStick.BallVisualSlopeTangent;

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

        Debug.Log(
            $"[BALL VISUAL CONTROL] " +
            $"time={Time.fixedTime:F3} " +
            $"visualPhase={visualPhase} " +
            $"motionPhase={motionPhase} " +
            $"ballFlat={ballFlatCaptured} " +
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

