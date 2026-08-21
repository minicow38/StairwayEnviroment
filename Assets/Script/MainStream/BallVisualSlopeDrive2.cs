 /*"using UnityEngine;
 using Sirenix.OdinInspector;
[Searchable]
[RequireComponent(typeof(Rigidbody))]
public class BallVisualSlopeDrive2 : MonoBehaviour
{
    private enum VisualPhase
    {
        Waiting,
        LockedToSubject,
        SlopeControlled,
        Released,
        Rejoining
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

    private void Awake()
    {
        ballBody = GetComponent<Rigidbody>();
    }
    private void ApplyMappedSynchronization()
    {
        if (respondSubject == null || rb == null)
            return;

        // DOTweenによって動いている表示座標へ位置・姿勢を合わせる。
        rb.position = respondSubject.MappedPosition;
        rb.rotation = respondSubject.MappedRotation;

        if (respondSubject.IsVisualFrameTurning)
        {
            // Mappedvelocityは使わない。
            // VisualPlayerRootの公転速度までRigidbodyへ入れてしまうため。
            rb.velocity =
                respondSubject.MappedPhysicalVelocity;

            rb.angularVelocity =
                respondSubject.MappedPhysicalAngularVelocity;
        }
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
            {
                Debug.Log("");
            }
           transform.GetComponent<SphereCollider>().isTrigger = true;
            

        }

        UpdateControlBasis();

        if (isFlat)
        {
            ProcessFlat();

            wasOnSlope = false;

            ApplyMappedSynchronization();

            WriteDebugLog(isFlat, isOnSlope);
            return;
        }
        if (totalTime > sumTime + stamp|| sumTime==0)
        {
            sumTime = totalTime;
        }

        if (isOnSlope)
        {
            ProcessSlope();

            if (slopeStick.slopeProgressErrorPercent <= 0)
            {
                Debug.Log("");
            }

            ApplyMappedSynchronization();

            wasOnSlope = true;
            WriteDebugLog(isFlat, isOnSlope);
            return;
        }
       

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
    }

    // 一時的に停止
     ApplySubjectControlField();

    ApplyRollingTorque();

    if (useProceduralStepPulse)
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

        Vector3 targetPosition =
            respondSubject.MappedPosition;

        Vector3 targetVelocity =
            respondSubject.Mappedvelocity;

        Vector3 positionError =
            targetPosition - ballBody.position;

        Vector3 velocityError =
            targetVelocity - ballBody.velocity;

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
        if (col.transform.CompareTag("plane"))
        {
            Debug.Log("");
        }
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

        int interval = Mathf.Max(1, logEveryFixedFrames);

        if (fixedFrameCounter % interval != 0)
            return;

        Vector3 positionError =
            subjectBody.position - ballBody.position;

        Vector3 velocityError =
            subjectBody.velocity - ballBody.velocity;

        Debug.Log(
            $"[BALL VISUAL CONTROL] " +
            $"time={Time.fixedTime:F3} " +
            $"phase={phase} " +
            $"flat={isFlat} slope={isOnSlope} " +
            $"kinematic={ballBody.isKinematic} " +
            $"mass={ballBody.mass:F3} " +
            $"rejoinPending={rejoinPending} " +
            $"rejoinElapsed={rejoinElapsed:F3} " +
            $"rejoinPositionWeight={debugRejoinPositionWeight:F3} " +
            $"rejoinStableFrames={rejoinStableFrames} " +
            $"subjectPos={subjectBody.position:F4} " +
            $"ballPos={ballBody.position:F4} " +
            $"positionError={positionError:F4} " +
            $"positionErrorMag={positionError.magnitude:F4} " +
            $"subjectVelocity={subjectBody.velocity:F4} " +
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
*/