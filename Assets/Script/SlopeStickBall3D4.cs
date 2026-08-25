using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody), typeof(SphereCollider))]
public sealed class SlopeStickBall3D4 : MonoBehaviour
{
    const string ImplementationVersion = "AdaptiveCriticalAdhesion-NaturalRelease-v6-2026-07-21";

    enum GroundKind
    {
        Air,
        Flat,
        Slope
    }

    enum GroundObservationSource
    {
        None,
        CollisionContact,
        SphereCast
    }

    enum SlopeProgressSide
    {
        Invalid,
        NegativeSide,
        NegativeNearZero,
        Exact,
        PositiveNearZero,
        PositiveSide
    }

    enum TargetProgressPreconditionPhase
    {
        Inactive,
        Observing,
        Preconditioning,
        Completed
    }

    enum SurfaceSamplePurpose
    {
        Progress,
        Entry,
        Exit
    }

    [System.Serializable]
    struct SlopeFrame
    {
        public bool valid;
        public Collider collider;
        public Collider[] sampleColliders;
        public Transform trackRoot;
        public Vector3 axis;
        public Vector3 entryPoint;
        public Vector3 exitPoint;
        public float projectedLength;
        public bool hasEntryConnectedSurface;
        public bool hasExitConnectedSurface;
        public Collider entryConnectedCollider;
        public Collider exitConnectedCollider;
        public Vector3 entryConnectedPoint;
        public Vector3 exitConnectedPoint;
        public Vector3 entryConnectedNormal;
        public Vector3 exitConnectedNormal;
        public float entryBoundaryCurvature;
        public float exitBoundaryCurvature;
        public float representativeCurvature;
    }

    struct SurfaceSample
    {
        public bool valid;
        public float progress;
        public Vector3 point;
        public Vector3 normal;
        public Vector3 tangent;
        public float curvature;
        public float gravitySupport;
        public float availableNormalAcceleration;
        public float criticalSpeed;
        public float distanceAhead;
        public float effectiveMaximumDeceleration;
        public float normalDemandRatio;
    }

    struct GroundObservation
    {
        public bool valid;
        public Collider collider;
        public Vector3 point;
        public Vector3 normal;
        public float distance;
        public GroundObservationSource source;
    }

    [Header("References")]
    [SerializeField] Rigidbody rb;
    [SerializeField] Transform cameraTransform;
    [SerializeField] Transform startTransform;

    private float sumTime = 0;
    public float fitTime = 0;
    [SerializeField] public float subtleTime = 0;

    [Header("Start / Auto Progress")]
    [SerializeField] bool resetToStartOnPlay;
    [SerializeField] Vector3 initialHeading = Vector3.right;
    [SerializeField] bool useAutoProgress = true;
    [Range(0f, 1f)] [SerializeField] float steeringStrength = 0.75f;

    [Header("Ground Detection")]
    [Min(0.01f)] [SerializeField] float sphereRadius = 0.5f;
    [Min(0f)] [SerializeField] float groundProbeDistance = 0.25f;
    [SerializeField] LayerMask groundMask = ~0;
    [Range(0f, 89f)] [SerializeField] float maxSlopeAngle = 75f;
    [Range(0f, 20f)] [SerializeField] float minimumSlopeAngle = 2f;

    [Header("Support Surface Latch")]
    [Tooltip("Unity物理が生成した実ContactをSphereCastより先に接地観測として採用します。")]
    [SerializeField] bool useCollisionContactsForGround = true;
    [Tooltip("Collision Contactをコールバック直後のFixedUpdateで読むための追加保持ステップ数です。0が最小です。")]
    [Range(0, 1)] [SerializeField] int collisionContactMemorySteps = 0;
    [Tooltip("接地観測を失ってもSlopeFrameだけを保持するFixedUpdate数です。猶予中もGroundKindはAirで、BaseStickは加えません。")]
    [Range(0, 3)] [SerializeField] int slopeTrackingGraceFrames = 1;
    [SerializeField] bool logSupportSurfaceLatch = true;

    [Header("Forward Slope Detection")]
    [Min(0.1f)] [SerializeField] float forwardSlopeProbeDistance = 8f;
    [Min(2)] [SerializeField] int forwardSlopeProbeSegments = 24;
    [Min(0.1f)] [SerializeField] float forwardProbeHeight = 3f;
    [Min(0.1f)] [SerializeField] float forwardProbeDownDistance = 8f;
    
   // [SerializeField] bool useTargetProgressPreconditioning = true;

   
   [Tooltip("Target Progress制御で使用できる正方向の最大人工加速度です。")]
   [Min(0f)]
   [SerializeField] float targetProgressMaximumArtificialAcceleration = 6f;

   
    [Min(0f)] [SerializeField] float maxGroundSpeed = 16f;
    [Min(0f)] [SerializeField] float maxGroundAcceleration = 45f;
    [Min(0f)] [SerializeField] float airAcceleration = 24f;

    [Header("Adaptive Critical Adhesion")]
    [Tooltip("平面、曲率無効時、またはAdaptive Adhesion無効時に使用する基準吸着加速度です。曲率が有効な斜面では固定値として速度を縛らず、必要吸着力をFixedUpdateごとに逆算します。")]
    [Min(0f)] [SerializeField] float baseStickAcceleration = 10f;
    [Tooltip("現在速度・曲率・重力支持から、Critical RatioがTarget値になるBase StickをFixedUpdateごとに逆算します。")]
    [SerializeField] bool useAdaptiveCriticalBaseStick = true;
    [Tooltip("Adaptive Base Stickの物理安全上限です。Max Ground Speedを上げた際にこの上限へ達すると、安全に保持できる速度までAllowed Speedが制限されます。")]
    [Min(0.1f)] [SerializeField] float maximumAdaptiveBaseStickAcceleration = 1000f;
    [Tooltip("現在速度から少し先の速度を予測して吸着力を先行させる時間です。高加速時の1 FixedUpdate遅れによる跳ねを防ぎます。")]
    [Range(0f, 0.25f)] [SerializeField] float adaptiveStickPredictionSeconds = 0.06f;
    [Tooltip("Adaptive Base Stickを増やす最大Jerkです。斜面初回接触では必要値を即時採用し、その後の増加だけに使用します。")]
    [Min(0.1f)] [SerializeField] float adaptiveStickRiseJerkLimit = 5000f;
    [Tooltip("Adaptive Base Stickを減らす最大Jerkです。速度低下時の急な法線力解除を抑えます。")]
    [Min(0.1f)] [SerializeField] float adaptiveStickFallJerkLimit = 1200f;
    [Tooltip("面から外向きの法線速度をこの時間で0へ近づける補助です。Critical計算を主成分とし、跳ね始めだけを抑えます。")]
    [Min(0.01f)] [SerializeField] float adaptiveOutwardNormalResponseSeconds = 0.05f;
    [SerializeField] bool logAdaptiveCriticalBaseStick = true;

    [Header("Critical Boundary Tracking")]
    [SerializeField] bool useCriticalBoundaryTracking = true;
    [Tooltip("1.0が理論離脱限界です。Unityの離散誤差を考慮して1未満にします。")]
    [Range(0.80f, 0.999f)] [SerializeField] float targetCriticalRatio = 0.98f;
    [Tooltip("前方斜面上で臨界速度を評価する点数です。")]
    [Range(4, 64)] [SerializeField] int criticalSampleCount = 24;
    [Tooltip("前方を何秒分まで評価するか。実距離は速度から自動計算します。")]
    [Min(0.05f)] [SerializeField] float criticalLookAheadSeconds = 0.75f;
    [Tooltip("接線方向の最大減速度。制動距離の逆算にも同じ値を使用します。")]
    [Min(0.1f)] [SerializeField] float maximumCriticalDeceleration = 30f;
    [Tooltip("接線方向の最大加速度。")]
    [Min(0.1f)] [SerializeField] float maximumCriticalAcceleration = 20f;
    [Tooltip("速度誤差を加速度へ変換する応答時間です。")]
    [Min(0.02f)] [SerializeField] float criticalResponseSeconds = 0.20f;
    [Tooltip("接線加速度が変化できる最大Jerkです。")]
    [Min(0.1f)] [SerializeField] float criticalJerkLimit = 100f;
    [Tooltip("これ未満の曲率は直線として扱います。")]
    [Min(0.000001f)] [SerializeField] float minimumCurvature = 0.0005f;
    [Tooltip("曲率計算に使うProgress差。斜面長から実距離へ変換されます。")]
    [Range(0.001f, 0.2f)] [SerializeField] float curvatureProgressStep = 0.025f;

    [Header("Representative Section Curvature")]
    [Tooltip("現在の1枚のEntry/Exitは維持したまま、入口前と出口後の接続面から代表曲率を作ります。")]
    [SerializeField] bool useRepresentativeSectionCurvature = true;
    [Tooltip("設定時は、このTransform配下のColliderだけを接続面候補にします。未設定時は幾何的な連続条件だけで選びます。")]
    [SerializeField] Transform curvatureTrackRoot;
    [Tooltip("Entryの手前、Exitの先へどれだけ進めて接続面を探すか。")]
    [Min(0.05f)] [SerializeField] float connectedSurfaceProbeOffset = 0.5f;
    [Tooltip("境界点から接続面Hitまで許容する最大距離。")]
    [Min(0.05f)] [SerializeField] float connectedSurfaceMaximumGap = 2f;
    [Tooltip("板境界の角を有限曲率へ変換するときの最小距離。小さすぎる曲率スパイクを防ぎます。")]
    [Min(0.01f)] [SerializeField] float minimumBoundaryCurvatureDistance = 0.5f;
    [Tooltip("代表曲率の上限。板境界で無限大に近づくことを防ぎます。")]
    [Min(0.001f)] [SerializeField] float maximumRepresentativeCurvature = 1f;
    [Tooltip("曲率によって法線加速度容量を使い切った場合でも残す最小減速度。")]
    [Min(0f)] [SerializeField] float minimumCurvatureAdjustedDeceleration = 2f;

    [Header("Critical Success")]
    [Tooltip("Critical Ratioが目標値からこの範囲内なら成功候補です。")]
    [Range(0.001f, 0.2f)] [SerializeField] float criticalRatioTolerance = 0.03f;
    [Tooltip("成功判定に必要な連続維持時間です。")]
    [Min(0f)] [SerializeField] float criticalHoldSeconds = 0.15f;

    [Header("Slope Progress Target Observation")]
    [Tooltip("斜面上の目標地点です。観測の基準であると同時に、下の事前調整が有効なら目標Critical Ratioへ近づける基準にも使います。")]
    [Range(0f, 100f)] [SerializeField] float targetSlopeProgressPercent = 30f;
    [Tooltip("目標の手前/直後を-0/+0として分類する範囲です。")]
    [Range(0.01f, 5f)] [SerializeField] float progressNearZeroTolerancePercent = 0.5f;
    [Tooltip("極限捕捉準備完了とみなす、同一SlopeFrame上の連続観測数です。")]
    [Range(1, 60)] [SerializeField] int requiredStableSlopeFrames = 5;
    [SerializeField] bool logProgressTargetCrossing = true;

    [Header("Target Progress Preconditioning")]
    [Tooltip("同一SlopeFrameを安定観測した後、targetProgressでtargetCriticalRatioへ近づくために接線方向だけを事前調整します。新しいAddForceは増やさず、既存の接線成分を置き換えます。")]
    
    [SerializeField] bool useTargetProgressPreconditioning = true;
    
    [Tooltip("事前調整が使用できる人工的な最大減速度です。重力の斜面方向成分を相殺しながら目標速度へ近づけるため、45度下降面では2より大きい値が必要です。")]
    [Min(0.1f)] [SerializeField] float targetProgressMaximumArtificialDeceleration = 12f;
    [Tooltip("事前調整加速度の変化上限です。急激な切り替えを避けつつ、短い斜面でも間に合う値にします。")]
    [Min(0.1f)] [SerializeField] float targetProgressJerkLimit = 40f;
    [Tooltip("targetProgress直前で残距離除算が発散しないための最小距離です。")]
    [Min(0.01f)] [SerializeField] float targetProgressMinimumDistance = 0.25f;
    [Tooltip("必要な正味加速度から斜面方向の重力加速度を差し引き、実際にAddForceで必要な最小人工加速度を求めます。")]
    [SerializeField] bool compensateTargetProgressGravity = true;
    [SerializeField] bool logTargetProgressPreconditioning = true;

    [Header("Natural Artificial Release")]
    [Tooltip("targetProgress通過後、人工加速度を滑らかに減らし、Natural Release Progressで完全にUnity物理へ渡します。")]
    [SerializeField] bool useNaturalArtificialRelease = true;
    [Tooltip("このProgress以上では人工接線加速度・Base Stick・Air Accelerationを復活させません。")]
    [Range(0f, 100f)] [SerializeField] float naturalReleaseProgressPercent = 50f;
    [Tooltip("targetProgress通過後もBase Stickを100%維持するProgress幅です。30%目標なら2で32%まで維持します。")]
    [Range(0f, 20f)] [SerializeField] float releaseHoldAfterTargetPercent = 2f;
    [Tooltip("現在速度に対して確保したい解放時間です。速度×時間から必要な解放距離を求めます。")]
    [Min(0.05f)] [SerializeField] float releaseNominalSeconds = 0.45f;
    [Tooltip("低速時でも確保する最小解放Progress幅です。")]
    [Range(0.1f, 50f)] [SerializeField] float minimumReleaseWidthPercent = 8f;
    [Tooltip("設計速度の下限をMax Ground Speedに対する比率で定めます。Base Stick自体へMax Ground Speedを直接掛けません。")]
    [Range(0.05f, 1f)] [SerializeField] float releaseSpeedFloorRatio = 0.35f;
    [SerializeField] bool logNaturalArtificialRelease = true;

    [Header("Runtime Readout")]
    [SerializeField] GroundKind groundKind;
    [SerializeField] bool hasForwardSlope;
    [SerializeField] string activeSlopeName;
    [Range(0f, 100f)] [SerializeField] float currentSlopeProgressPercent;
    [Range(0f, 100f)] [SerializeField] float controllingSampleProgressPercent;
    [SerializeField] float currentTangentSpeed;
    [SerializeField] float currentAllowedSpeed;
    [SerializeField] float currentLocalCriticalSpeed;
    [SerializeField] float currentCriticalRatio;
    [SerializeField] float currentCurvature;
    [SerializeField] float currentAvailableNormalAcceleration;
    [SerializeField] float currentRequiredNormalAcceleration;
    [SerializeField] float currentTangentialAcceleration;
    [SerializeField] float currentNormalCorrection;
    [SerializeField] float currentRepresentativeCurvature;
    [SerializeField] float currentEffectiveMaximumDeceleration;
    [SerializeField] float currentNormalDemandRatio;
    [SerializeField] bool criticalStateMaintained;
    [SerializeField] float criticalMaintainedSeconds;

    [Header("Support / Progress Runtime Readout")]
    [SerializeField] GroundObservationSource currentGroundObservationSource;
    [SerializeField] string currentGroundColliderName;
    [SerializeField] int consecutiveGroundMissFrames;
    [SerializeField] int stableSlopeContactFrames;
    [SerializeField] bool slopeProgressObservationValid;
    [SerializeField] float previousSlopeProgressPercent;
    [SerializeField] float slopeProgressDeltaPercent;
    [SerializeField] float slopeProgressErrorPercent;
    [SerializeField] SlopeProgressSide slopeProgressSide;
    [SerializeField] bool crossedTargetThisFrame;
    [SerializeField] bool readyForLimitCapture;
    [SerializeField] float estimatedTargetCrossingTime;

    [Header("Target Progress Preconditioning Runtime")]
    [SerializeField] TargetProgressPreconditionPhase targetProgressPhase;
    [SerializeField] bool targetProgressPlanValid;
    [SerializeField] string targetProgressPlanColliderName;
    [SerializeField] float capturedTargetProgressPercent;
    [SerializeField] float capturedTargetTangentSpeed;
    [SerializeField] float capturedTargetCurvature;
    [SerializeField] float capturedTargetAvailableNormalAcceleration;
    [SerializeField] float targetProgressRemainingDistance;
    [SerializeField] float targetProgressRequiredNetAcceleration;
    [SerializeField] float targetProgressGravityAcceleration;
    [SerializeField] float targetProgressDesiredArtificialAcceleration;
    [SerializeField] float targetProgressAppliedArtificialAcceleration;

    [Header("Natural Artificial Release Runtime")]
    [SerializeField] bool naturalReleasePlanValid;
    [SerializeField] bool naturalMotionReleased;
    [SerializeField] string naturalReleasePlanColliderName;
    [SerializeField] float capturedReleaseStartProgressPercent;
    [SerializeField] float capturedNaturalReleaseProgressPercent;
    [SerializeField] float capturedReleaseDesignSpeed;
    [Range(0f, 1f)] [SerializeField] float currentArtificialReleaseWeight = 1f;
    [Range(0f, 1f)] [SerializeField] float currentCumulativeReleaseRatio;

    [Header("Adaptive Critical Adhesion Runtime")]
    [SerializeField] float currentAdaptivePredictedSpeed;
    [SerializeField] float currentAdaptiveDesiredBaseStickAcceleration;
    [SerializeField] float currentAdaptiveStickCapacity;
    [SerializeField] float currentAdaptiveOutwardDampingAcceleration;
    [SerializeField] float currentEffectiveBaseStickAcceleration;
    [SerializeField] bool adaptiveStickSaturated;

    [Header("Debug")]
    [SerializeField] bool writeRuntimeLog = true;
    [Min(1)] [SerializeField] int logEveryFixedFrames = 10;
    [SerializeField] bool logStateChanges = true;
    [SerializeField] bool logCriticalSuccess = true;
    [SerializeField] bool logCriticalRisk = true;
    [SerializeField] bool logSessionSummary = true;
    [Tooltip("Ratioがこの値以上なら理論離脱域として警告します。通常は1です。")]
    [Min(0.01f)] [SerializeField] float criticalRiskRatio = 1f;
    [Tooltip("危険警告を繰り返す最短間隔です。")]
    [Min(0f)] [SerializeField] float criticalRiskLogInterval = 0.25f;
    [SerializeField] bool drawDebugGizmos = true;

    [Header("Entry / Exit Ray Visualization")]
    [Tooltip("entryGuessとexitGuessから実際に飛ばす5本ずつのRayをLineRendererで表示します。")]
    [SerializeField] bool showEntryExitRayLines = true;
    [Tooltip("各斜面ColliderについてEntry/Exitを最初の1回だけ保存します。別の斜面では別のLineRenderer群を生成します。")]
    [SerializeField] bool freezeEntryExitRayLinesAfterCapture = true;
    [Tooltip("斜面ごとにEntry 5本＋Exit 5本を保持し、全斜面を同時表示します。")]
    [SerializeField] bool keepVisualizationForEverySlope = true;
    [Tooltip("対象斜面へ当たらなかったRayも最大距離まで表示します。")]
    [SerializeField] bool showMissedRayFullLength = true;
    [Min(0.001f)] [SerializeField] float slopeRayLineWidth = 0.03f;
    [SerializeField] Material slopeRayLineMaterial;
    [SerializeField] Color entryRayHitColor = Color.cyan;
    [SerializeField] Color exitRayHitColor = Color.yellow;
    [SerializeField] Color missedRayColor = Color.red;

    Vector2 input;
    Vector3 heading;
    Vector3 groundNormal = Vector3.up;
    public Vector3 restart;
    GroundObservation currentGroundObservation;
    GroundObservation latestCollisionGroundObservation;
    SlopeFrame slopeFrame;
    SlopeFrame forwardSlopeFrame;
    int fixedFrameCounter;
    bool wasGrounded;
    float lastCriticalRatioBeforeAir;
    Vector3 controllingSamplePoint;

    readonly ContactPoint[] collisionContactBuffer = new ContactPoint[16];
    float latestCollisionContactFixedTime = float.NegativeInfinity;
    float latestCollisionGroundScore = float.NegativeInfinity;
    float latestCollisionScoreFixedTime = float.NegativeInfinity;
    Collider trackedSlopeCollider;
    Collider progressObservationCollider;
    bool hasPreviousSlopeProgressObservation;
    float lastObservedSlopeProgressPercent;
    float previousSlopeProgressErrorPercent;
    bool limitCaptureReadyLogged;
    GroundObservationSource previousGroundObservationSource;
    Collider previousGroundObservationCollider;

    Collider targetProgressPlanCollider;
    float previousTargetProgressAppliedAcceleration;
    bool targetProgressCompletionPending;
    bool targetProgressPlanFailureLogged;

    Collider naturalReleasePlanCollider;
    bool naturalReleaseLatchLogged;
    Collider adaptiveStickActiveCollider;
    float previousAdaptiveBaseStickAcceleration;
    bool adaptiveStickSaturationLogged;

    sealed class SlopeSectionRayVisual
    {
        public Transform root;
        public readonly LineRenderer[] entryRays = new LineRenderer[5];
        public readonly LineRenderer[] exitRays = new LineRenderer[5];
        public LineRenderer entryGuessToHit;
        public LineRenderer exitGuessToHit;
        public LineRenderer measuredSection;
        public bool entryCaptured;
        public bool exitCaptured;
        public Vector3 entryPoint;
        public Vector3 exitPoint;
        public bool hasEntryPoint;
        public bool hasExitPoint;
    }

    Transform slopeRayDebugRoot;
    Material runtimeSlopeRayMaterial;
    readonly Dictionary<int, SlopeSectionRayVisual> slopeRayVisuals = new Dictionary<int, SlopeSectionRayVisual>();

    GroundKind previousLoggedGroundKind = GroundKind.Air;
    bool previousLoggedForwardSlope;
    bool previousLoggedCriticalControlActive;
    bool previousLoggedInCriticalTolerance;
    bool previousLoggedCriticalMaintained;
    string previousLoggedSlopeName = string.Empty;
    float lastCriticalRiskLogTime = float.NegativeInfinity;
    float sessionMaximumCriticalRatio;
    float sessionMinimumCriticalError = float.PositiveInfinity;
    float sessionLongestMaintainedSeconds;
    int sessionCriticalSuccessCount;
    int sessionGroundToAirCount;
    int sessionSlopeDetectionCount;

    public float CurrentSlopeProgressPercent => currentSlopeProgressPercent;
    public float TargetSlopeProgressPercent => targetSlopeProgressPercent;
    public float SlopeProgressErrorPercent => slopeProgressErrorPercent;
    public bool ReadyForLimitCapture => readyForLimitCapture;
    public float CurrentCriticalRatio => currentCriticalRatio;
    public bool CriticalStateMaintained => criticalStateMaintained;
    public bool TargetProgressPlanValid => targetProgressPlanValid;
    public float CapturedTargetTangentSpeed => capturedTargetTangentSpeed;
    public bool NaturalMotionReleased => naturalMotionReleased;
    public float CurrentArtificialReleaseWeight => currentArtificialReleaseWeight;

    void Reset()
    {
        rb = GetComponent<Rigidbody>();
        initialHeading = Vector3.forward;
    }

    IEnumerator DelayStart()
    {
        yield return new WaitForSeconds(.8f);
        GameObject startSlab = GameObject.Find("ArcSlab4");
        restart = startSlab.transform.position;
        rb.position = new Vector3(restart.x, restart.y + 2f, restart.z);
    }

    void Awake()
    {
        if (!rb)
            rb = GetComponent<Rigidbody>();

        heading = NormalizeFlat(initialHeading, Vector3.forward);
        rb.maxAngularVelocity = 100f;
        currentEffectiveMaximumDeceleration = maximumCriticalDeceleration;
        currentSlopeProgressPercent = 0f;
        previousSlopeProgressPercent = 0f;
        slopeProgressSide = SlopeProgressSide.Invalid;
        currentGroundObservationSource = GroundObservationSource.None;
        ResetTargetProgressPreconditioning("Awake", false);
        ResetNaturalArtificialRelease("Awake", false);

        if (writeRuntimeLog)
        {
            Debug.Log($"[SLOPE STICK VERSION] {ImplementationVersion}", this);
        }
    }

    void Start()
    {
        if (resetToStartOnPlay && startTransform)
            ResetBallToStart();
        StartCoroutine(DelayStart());
    }

    void Update()
    {
        input = new Vector2(Input.GetAxisRaw("Horizontal"), Input.GetAxisRaw("Vertical"));
    }

    void FixedUpdate()
    {
        float dt = Mathf.Max(Time.fixedDeltaTime, 0.000001f);
        bool grounded = ProbeGround(out currentGroundObservation);

        UpdateSupportObservationDebug(grounded, currentGroundObservation);

        if (grounded)
        {
            consecutiveGroundMissFrames = 0;
            groundNormal = currentGroundObservation.normal.normalized;
            float angle = Vector3.Angle(groundNormal, Vector3.up);
            Debug.Log("");

            groundKind = angle >= minimumSlopeAngle ? GroundKind.Slope : GroundKind.Flat;
        }
        else
        {
            consecutiveGroundMissFrames++;
            groundNormal = Vector3.up;
            groundKind = GroundKind.Air;

            if (consecutiveGroundMissFrames <= slopeTrackingGraceFrames)
            {
                if (writeRuntimeLog && logSupportSurfaceLatch)
                {
                    Debug.Log($"[SLOPE TRACKING GRACE] time={Time.fixedTime:F4}s " + $"missFrames={consecutiveGroundMissFrames} " + $"graceFrames={slopeTrackingGraceFrames} " +
                        $"trackedSlope={(trackedSlopeCollider ? trackedSlopeCollider.name : "None")} " + $"ground=Air stickApplied=false", this);
                }
            }
            else
            {
                ClearSlopeTracking("GroundObservationLost");
            }
        }

        if (wasGrounded && !grounded)
        {
            lastCriticalRatioBeforeAir = currentCriticalRatio;
            sessionGroundToAirCount++;

            if (writeRuntimeLog)
            {
                string judgement = ClassifyAirTransition(lastCriticalRatioBeforeAir);

                Debug.LogWarning($"[CRITICAL AIR TRANSITION] judgement={judgement} " + $"time={Time.fixedTime:F4}s " + $"progress={currentSlopeProgressPercent:F3}% " +
                    $"ratio={lastCriticalRatioBeforeAir:F6} " + $"target={targetCriticalRatio:F6} " + $"error={Mathf.Abs(lastCriticalRatioBeforeAir - targetCriticalRatio):F6} " +
                    $"speed={currentTangentSpeed:F6} " + $"allowed={currentAllowedSpeed:F6} " + $"localCritical={currentLocalCriticalSpeed:F6} " + $"curvature={currentCurvature:F8} " +
                    $"available={currentAvailableNormalAcceleration:F6} " + $"required={currentRequiredNormalAcceleration:F6} " + $"maintainedBeforeAir={previousLoggedCriticalMaintained}",
                    this);
            }
        }

        Vector3 desiredMove = GetDesiredMoveWorld();
        sumTime += Time.deltaTime;

        if (grounded)
        {
            if (groundKind == GroundKind.Slope)
            {
                Debug.Log("");
            }

            if (groundKind == GroundKind.Flat)
            {
                UpdateForwardSlopeFromFlat(desiredMove, currentGroundObservation.normal);
            }
            else
            {
                UpdateCurrentSlopeFrame(currentGroundObservation, desiredMove);
            }

            SolveGround(desiredMove, dt);
        }
        else
        {
            SolveAir(desiredMove, dt);
            criticalStateMaintained = false;
            criticalMaintainedSeconds = 0f;
        }

        UpdateSlopeProgressObservation(grounded, currentGroundObservation, dt);
        UpdateCriticalDebugEvents(grounded);
        UpdateSessionStatistics();

        wasGrounded = grounded;
        WriteLogIfNeeded();
    }

    public void ResetBallToStart()
    {
        if (!startTransform || !rb)
            return;

        rb.position = startTransform.position;
        rb.rotation = startTransform.rotation;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        heading = NormalizeFlat(initialHeading, startTransform.forward);
        slopeFrame = default;
        forwardSlopeFrame = default;
        trackedSlopeCollider = null;
        currentGroundObservation = default;
        latestCollisionGroundObservation = default;
        latestCollisionContactFixedTime = float.NegativeInfinity;
        latestCollisionGroundScore = float.NegativeInfinity;
        latestCollisionScoreFixedTime = float.NegativeInfinity;
        consecutiveGroundMissFrames = 0;
        currentTangentialAcceleration = 0f;
        criticalMaintainedSeconds = 0f;
        criticalStateMaintained = false;
        ResetSlopeProgressObservation();
        ResetTargetProgressPreconditioning("BallReset", false);
        ResetNaturalArtificialRelease("BallReset", false);
    }

    void OnCollisionEnter(Collision collision)
    {
        CaptureCollisionGround(collision);
    }

    void OnCollisionStay(Collision collision)
    {
        CaptureCollisionGround(collision);
    }

    void CaptureCollisionGround(Collision collision)
    {
        if (!useCollisionContactsForGround || collision == null)
            return;

        float callbackFixedTime = Time.fixedTime;
        if (Mathf.Abs(callbackFixedTime - latestCollisionScoreFixedTime) > 0.000001f)
        {
            latestCollisionScoreFixedTime = callbackFixedTime;
            latestCollisionGroundScore = float.NegativeInfinity;
        }

        int count = collision.GetContacts(collisionContactBuffer);
        bool found = false;
        float bestScore = float.NegativeInfinity;
        GroundObservation best = default;

        for (int i = 0; i < count; i++)
        {
            ContactPoint contact = collisionContactBuffer[i];
            Collider candidate = contact.otherCollider;

            if (!candidate || candidate.attachedRigidbody == rb)
                candidate = contact.thisCollider;

            if (!candidate || candidate.attachedRigidbody == rb || candidate.isTrigger || !LayerIsGround(candidate.gameObject.layer))
            {
                continue;
            }

            Vector3 normal = contact.normal.normalized;
            if (normal.sqrMagnitude < 0.999f)
                continue;

            Vector3 towardBody = rb.worldCenterOfMass - contact.point;
            if (Vector3.Dot(normal, towardBody) < 0f)
                normal = -normal;

            float angle = Vector3.Angle(normal, Vector3.up);
            if (angle > maxSlopeAngle)
                continue;

            float score = Vector3.Dot(normal, Vector3.up);

            if (candidate == trackedSlopeCollider)
                score += 4f;
            if (slopeFrame.valid && candidate == slopeFrame.collider)
                score += 3f;
            if (forwardSlopeFrame.valid && candidate == forwardSlopeFrame.collider)
                score += 2f;

            score -= towardBody.magnitude * 0.01f;

            if (score <= bestScore)
                continue;

            bestScore = score;
            found = true;
            best = new GroundObservation
            {
                valid = true,
                collider = candidate,
                point = contact.point,
                normal = normal,
                distance = 0f,
                source = GroundObservationSource.CollisionContact
            };
        }

        if (!found || bestScore <= latestCollisionGroundScore)
            return;

        latestCollisionGroundScore = bestScore;
        latestCollisionGroundObservation = best;
        latestCollisionContactFixedTime = callbackFixedTime;
    }

    bool ProbeGround(out GroundObservation observation)
    {
        observation = default;

        if (useCollisionContactsForGround && TryGetRecentCollisionGround(out observation))
        {
            return true;
        }

        Vector3 origin = rb.position + Vector3.up * 0.02f;
        bool found = Physics.SphereCast(origin, sphereRadius * 0.95f, Vector3.down, out RaycastHit hit, sphereRadius + groundProbeDistance, groundMask,
            QueryTriggerInteraction.Ignore);

        if (!found || !hit.collider)
            return false;

        Vector3 normal = hit.normal.normalized;
        if (normal.sqrMagnitude < 0.999f || Vector3.Angle(normal, Vector3.up) > maxSlopeAngle)
        {
            return false;
        }

        if (sumTime > fitTime)
        {
            fitTime = sumTime + subtleTime;
        }

        observation = new GroundObservation
        {
            valid = true,
            collider = hit.collider,
            point = hit.point,
            normal = normal,
            distance = hit.distance,
            source = GroundObservationSource.SphereCast
        };

        return true;
    }

    bool TryGetRecentCollisionGround(out GroundObservation observation)
    {
        observation = default;

        if (!latestCollisionGroundObservation.valid || !latestCollisionGroundObservation.collider)
        {
            return false;
        }

        float maximumAge = (1f + collisionContactMemorySteps) * Mathf.Max(Time.fixedDeltaTime, 0.000001f) + 0.0001f;

        if (Time.fixedTime - latestCollisionContactFixedTime > maximumAge)
        {
            latestCollisionGroundObservation = default;
            latestCollisionContactFixedTime = float.NegativeInfinity;
            latestCollisionGroundScore = float.NegativeInfinity;
            return false;
        }

        observation = latestCollisionGroundObservation;
        return true;
    }

    bool LayerIsGround(int layer)
    {
        return (groundMask.value & (1 << layer)) != 0;
    }

    void UpdateSupportObservationDebug(bool grounded, GroundObservation observation)
    {
        currentGroundObservationSource = grounded ? observation.source : GroundObservationSource.None;
        currentGroundColliderName = grounded && observation.collider ? observation.collider.name : string.Empty;

        if (!writeRuntimeLog || !logSupportSurfaceLatch)
            return;

        Collider currentCollider = grounded ? observation.collider : null;
        GroundObservationSource currentSource = grounded ? observation.source : GroundObservationSource.None;

        if (currentSource == previousGroundObservationSource && currentCollider == previousGroundObservationCollider)
        {
            return;
        }

        Debug.Log($"[SUPPORT OBSERVATION] time={Time.fixedTime:F4}s " + $"grounded={grounded} " + $"source={currentSource} " +
            $"collider={(currentCollider ? currentCollider.name : "None")} " + $"point={(grounded ? observation.point.ToString("F5") : "N/A")} " +
            $"normal={(grounded ? observation.normal.ToString("F5") : "N/A")}", this);

        previousGroundObservationSource = currentSource;
        previousGroundObservationCollider = currentCollider;
    }

    void ClearSlopeTracking(string reason)
    {
        bool hadTracking = trackedSlopeCollider || slopeFrame.valid || forwardSlopeFrame.valid;

        ResetTargetProgressPreconditioning(reason, hadTracking);

        trackedSlopeCollider = null;
        slopeFrame = default;
        forwardSlopeFrame = default;
        hasForwardSlope = false;
        activeSlopeName = string.Empty;
        currentSlopeProgressPercent = 0f;
        ResetSlopeProgressObservation();

        if (hadTracking && writeRuntimeLog && logSupportSurfaceLatch)
        {
            Debug.Log($"[SLOPE TRACKING CLEARED] time={Time.fixedTime:F4}s " + $"reason={reason}", this);
        }
    }

    Vector3 GetDesiredMoveWorld()
    {
        if (useAutoProgress)
        {
            Vector3 forward = NormalizeFlat(heading, initialHeading);
            Vector3 right = Vector3.Cross(Vector3.up, forward).normalized;
            Vector3 desired = forward + right * input.x * steeringStrength;
            return desired.sqrMagnitude > 0.000001f ? desired.normalized : forward;
        }

        Vector3 forwardDirection = Vector3.forward;
        Vector3 rightDirection = Vector3.right;

        if (cameraTransform)
        {
            forwardDirection = NormalizeFlat(cameraTransform.forward, Vector3.forward);
            rightDirection = NormalizeFlat(cameraTransform.right, Vector3.right);
        }

        Vector3 move = forwardDirection * input.y + rightDirection * input.x;
        return move.sqrMagnitude > 1f ? move.normalized : move;
    }

    void UpdateForwardSlopeFromFlat(Vector3 desiredMove, Vector3 flatNormal)
    {
        hasForwardSlope = TryDetectForwardSlope(flatNormal, desiredMove, out RaycastHit slopeHit, out SlopeFrame detectedFrame);

        if (hasForwardSlope)
        {
            forwardSlopeFrame = detectedFrame;
            activeSlopeName = slopeHit.collider ? slopeHit.collider.name : string.Empty;
        }
        else
        {
            forwardSlopeFrame = default;
            activeSlopeName = string.Empty;
        }
    }

    bool TryDetectForwardSlope(Vector3 flatNormal, Vector3 desiredMove, out RaycastHit slopeHit, out SlopeFrame detectedFrame)
    {
        slopeHit = default;
        detectedFrame = default;

        Vector3 forward = Vector3.ProjectOnPlane(desiredMove, flatNormal);
        if (forward.sqrMagnitude < 0.000001f)
            forward = Vector3.ProjectOnPlane(heading, flatNormal);
        if (forward.sqrMagnitude < 0.000001f)
            return false;
        forward.Normalize();

        int segments = Mathf.Max(2, forwardSlopeProbeSegments);
        for (int i = 1; i <= segments; i++)
        {
            float distance = forwardSlopeProbeDistance * i / segments;
            Vector3 horizontalPoint = rb.position + forward * distance;
            Vector3 origin = horizontalPoint + Vector3.up * forwardProbeHeight;

            if (!Physics.Raycast(origin, Vector3.down, out RaycastHit hit, forwardProbeDownDistance, groundMask, QueryTriggerInteraction.Ignore))
            {
                continue;
            }

            float angle = Vector3.Angle(hit.normal, Vector3.up);
            if (angle < minimumSlopeAngle || angle > maxSlopeAngle)
                continue;

            if (forwardSlopeFrame.valid && forwardSlopeFrame.collider == hit.collider)
            {
                slopeHit = hit;
                detectedFrame = forwardSlopeFrame;
                return true;
            }

            if (!BuildSlopeFrame(hit.collider, hit.normal, forward, out detectedFrame))
            {
                continue;
            }

            if (detectedFrame.valid == true)
            {
                if (groundKind == GroundKind.Slope)
                {
                    Debug.Log("");
                }
            }

            slopeHit = hit;
            return true;
        }

        return false;
    }

    void UpdateCurrentSlopeFrame(GroundObservation observation, Vector3 desiredMove)
    {
        if (!observation.valid || !observation.collider)
            return;

        Collider contactedCollider = observation.collider;

        if (slopeFrame.valid && slopeFrame.collider == contactedCollider)
        {
            trackedSlopeCollider = contactedCollider;
            activeSlopeName = contactedCollider.name;
            return;
        }

        if (forwardSlopeFrame.valid && forwardSlopeFrame.collider == contactedCollider)
        {
            slopeFrame = forwardSlopeFrame;
            trackedSlopeCollider = contactedCollider;
            activeSlopeName = contactedCollider.name;

            if (writeRuntimeLog && logSupportSurfaceLatch)
            {
                Debug.Log($"[SLOPE FRAME PROMOTED] time={Time.fixedTime:F4}s " + $"collider={contactedCollider.name} " + $"entry={slopeFrame.entryPoint:F5} " +
                    $"exit={slopeFrame.exitPoint:F5} " + $"length={slopeFrame.projectedLength:F6}", this);
            }

            return;
        }

        Vector3 forward = Vector3.ProjectOnPlane(desiredMove, observation.normal);
        if (forward.sqrMagnitude < 0.000001f)
        {
            forward = Vector3.ProjectOnPlane(heading, observation.normal);
        }

        if (BuildSlopeFrame(contactedCollider, observation.normal, forward, out SlopeFrame built))
        {
            slopeFrame = built;
            trackedSlopeCollider = contactedCollider;
            activeSlopeName = contactedCollider.name;
        }
        else
        {
            Debug.Log("");
        }
    }

    bool BuildSlopeFrame(Collider slopeCollider, Vector3 referenceNormalInput, Vector3 desiredForward, out SlopeFrame frame)
    {
        frame = default;

        if (!slopeCollider)
            return false;

        // 1区間を「現在検出した1枚の斜面Collider」として扱います。
        // 傾きの異なるtrackRoot配下の全Colliderをまとめると、入口側の局所axisを
        // 全体終端まで延長してしまうため、exitGuessが実コースから外れます。
        Collider[] frameColliders = { slopeCollider };

        Vector3 referenceNormal = referenceNormalInput.normalized;
        if (referenceNormal.sqrMagnitude < 0.999f)
            return false;

        // currentSlopeProgressPercent は、この1枚の斜面Colliderの入口0%～出口100%を表します。
        // プレイヤーの入力方向を斜面へ投影すると、斜面を斜め横切るaxisになり、
        // 45度面でもaxisの傾斜が45度未満になる可能性があります。
        // そこで、面法線から「斜面上の最急勾配方向」を一意に求めます。
        Vector3 axis = Vector3.ProjectOnPlane(Vector3.up, referenceNormal);
        if (axis.sqrMagnitude < 0.000001f)
            return false;

        axis.Normalize();

        // 最急勾配方向には上り・下りの2方向があるため、実際の進行方向へ向きをそろえます。
        Vector3 desiredFlat = Vector3.ProjectOnPlane(desiredForward, Vector3.up);
        if (desiredFlat.sqrMagnitude < 0.000001f)
            desiredFlat = Vector3.ProjectOnPlane(heading, Vector3.up);

        Vector3 axisFlat = Vector3.ProjectOnPlane(axis, Vector3.up);
        if (desiredFlat.sqrMagnitude > 0.000001f && axisFlat.sqrMagnitude > 0.000001f && Vector3.Dot(axisFlat.normalized, desiredFlat.normalized) < 0f)
        {
            axis = -axis;
        }

        Bounds sectionBounds = slopeCollider.bounds;
        Vector3[] corners = GetBoundsCorners(sectionBounds);
        float minimum = float.PositiveInfinity;
        float maximum = float.NegativeInfinity;

        for (int i = 0; i < corners.Length; i++)
        {
            float coordinate = Vector3.Dot(corners[i], axis);
            minimum = Mathf.Min(minimum, coordinate);
            maximum = Mathf.Max(maximum, coordinate);
        }

        float length = maximum - minimum;
        if (length <= 0.0001f)
            return false;

        Vector3 center = sectionBounds.center;
        float centerCoordinate = Vector3.Dot(center, axis);

        // axisに直交し、Collider中心を通る基準点です。
        // entryGuess / exitGuess は、この基準点からaxis方向のスカラー座標で配置します。
        Vector3 axisOrigin = center - axis * centerCoordinate;

        // Colliderの完全な端ではRaycastが境界から外れやすいため、端からごく少しだけ内側を使います。
        // 一辺分は入れず、区間長の0.5%を基本として0.005～0.02に制限します。
        float endpointInset = Mathf.Clamp(length * 0.005f, 0.005f, 0.02f);

        Vector3 entryGuess = axisOrigin + axis * (minimum + endpointInset);
        Vector3 exitGuess = axisOrigin + axis * (maximum - endpointInset);

        frame.valid = true;
        frame.collider = slopeCollider;
        frame.sampleColliders = frameColliders;
        frame.trackRoot = slopeCollider.transform;
        frame.axis = axis;
        frame.entryPoint = entryGuess;
        frame.exitPoint = exitGuess;
        frame.projectedLength = Mathf.Max(Vector3.Dot(exitGuess - entryGuess, axis), 0.0001f);

        const float sameSectionNormalTolerance = 3f;

        // 始点はreferenceHit.normalと同じ角度の面だけを承認します。
        if (groundKind == GroundKind.Slope)
        {
            Debug.Log("");
        }
        if (!TrySampleFrameSurface(frame, entryGuess, referenceNormal, sameSectionNormalTolerance, out Vector3 entryPoint, out Vector3 entryNormal, SurfaceSamplePurpose.Entry))
        {
            if (writeRuntimeLog)
            {
                Debug.LogWarning($"[CRITICAL FRAME INVALID] collider={slopeCollider.name} " + $"reason=EntrySampleFailed " + $"entryGuess={entryGuess:F5} " +
                    $"referenceNormal={referenceNormal:F5}", this);
            }

            frame = default;
            return false;
        }

        // 終点は、始点で実測した法線と同じ角度の面だけを承認します。
        if (!TrySampleFrameSurface(frame, exitGuess, entryNormal, sameSectionNormalTolerance, out Vector3 exitPoint, out Vector3 exitNormal, SurfaceSamplePurpose.Exit))
        {
            if (writeRuntimeLog)
            {
                Debug.LogWarning($"[CRITICAL FRAME INVALID] collider={slopeCollider.name} " + $"reason=ExitSampleFailedOrAngleMismatch " + $"exitGuess={exitGuess:F5} " +
                    $"entryNormal={entryNormal:F5}", this);
            }

            frame = default;
            return false;
        }

        float endpointNormalDifference = Vector3.Angle(entryNormal, exitNormal);

        if (endpointNormalDifference > sameSectionNormalTolerance)
        {
            if (writeRuntimeLog)
            {
                Debug.LogWarning($"[FRAME SECTION REJECTED] collider={slopeCollider.name} " + $"entryPoint={entryPoint:F5} " + $"exitPoint={exitPoint:F5} " +
                    $"entryNormal={entryNormal:F5} " + $"exitNormal={exitNormal:F5} " + $"normalDifference={endpointNormalDifference:F4} " + $"limit={sameSectionNormalTolerance:F4}", this);
            }

            frame = default;
            return false;
        }

        // 同じ1枚の斜面を最急勾配方向に測れているかを確認します。
        // 45度面なら surfaceAngle / axisAngle / endpointAngle がすべて約45度になります。
        Vector3 endpointDelta = exitPoint - entryPoint;
        float endpointHorizontalDistance = Vector3.ProjectOnPlane(endpointDelta, Vector3.up).magnitude;

        float surfaceAngle = Vector3.Angle(entryNormal, Vector3.up);
        float axisAngle = Mathf.Atan2(Mathf.Abs(axis.y), Vector3.ProjectOnPlane(axis, Vector3.up).magnitude) * Mathf.Rad2Deg;
        float endpointAngle = Mathf.Atan2(Mathf.Abs(endpointDelta.y), endpointHorizontalDistance) * Mathf.Rad2Deg;
        float axisNormalDot = Mathf.Abs(Vector3.Dot(axis.normalized, entryNormal.normalized));

        const float endpointDirectionTolerance = 2f;
        if (axisNormalDot > 0.001f || Mathf.Abs(endpointAngle - axisAngle) > endpointDirectionTolerance)
        {
            if (writeRuntimeLog)
            {
                Debug.LogWarning($"[FRAME SECTION REJECTED] collider={slopeCollider.name} " + $"reason=AxisOrEndpointAngleMismatch " + $"surfaceAngle={surfaceAngle:F4} " +
                    $"axisAngle={axisAngle:F4} " + $"endpointAngle={endpointAngle:F4} " + $"axisNormalDot={axisNormalDot:F6} " + $"entryPoint={entryPoint:F5} " + $"exitPoint={exitPoint:F5}",
                    this);
            }

            frame = default;
            return false;
        }

        float projectedLength = Vector3.Dot(endpointDelta, axis);

        if (projectedLength <= 0.0001f)
        {
            if (writeRuntimeLog)
            {
                Debug.LogWarning($"[FRAME SECTION REJECTED] collider={slopeCollider.name} " + $"reason=NonPositiveLength " + $"entryPoint={entryPoint:F5} " +
                    $"exitPoint={exitPoint:F5} " + $"projectedLength={projectedLength:F6}", this);
            }

            frame = default;
            return false;
        }

        frame.entryPoint = entryPoint;
        frame.exitPoint = exitPoint;
        frame.projectedLength = projectedLength;

        PopulateRepresentativeSectionCurvature(ref frame, entryNormal, exitNormal);

        if (writeRuntimeLog)
        {
            Debug.Log($"[FRAME SECTION APPROVED] collider={slopeCollider.name} " + $"entryPoint={entryPoint:F5} " + $"exitPoint={exitPoint:F5} " + $"entryNormal={entryNormal:F5} " +
                $"exitNormal={exitNormal:F5} " + $"normalDifference={endpointNormalDifference:F4} " + $"surfaceAngle={surfaceAngle:F4} " + $"axisAngle={axisAngle:F4} " +
                $"endpointAngle={endpointAngle:F4} " + $"axisNormalDot={axisNormalDot:F6} " + $"length={frame.projectedLength:F6} " +
                $"entryConnected={(frame.entryConnectedCollider ? frame.entryConnectedCollider.name : "None")} " +
                $"exitConnected={(frame.exitConnectedCollider ? frame.exitConnectedCollider.name : "None")} " + $"entryBoundaryCurvature={frame.entryBoundaryCurvature:F8} " +
                $"exitBoundaryCurvature={frame.exitBoundaryCurvature:F8} " + $"representativeCurvature={frame.representativeCurvature:F8}", this);
        }

        return true;
    }

    bool FrameContainsCollider(SlopeFrame frame, Collider candidate)
    {
        // SlopeFrameは現在の1枚だけを表します。
        // 親Root配下の別Colliderは、この区間の候補面として承認しません。
        return frame.valid && candidate && frame.collider == candidate;
    }

    void PopulateRepresentativeSectionCurvature(ref SlopeFrame frame, Vector3 entryNormal, Vector3 exitNormal)
    {
        frame.hasEntryConnectedSurface = false;
        frame.hasExitConnectedSurface = false;
        frame.entryConnectedCollider = null;
        frame.exitConnectedCollider = null;
        frame.entryConnectedPoint = Vector3.zero;
        frame.exitConnectedPoint = Vector3.zero;
        frame.entryConnectedNormal = Vector3.up;
        frame.exitConnectedNormal = Vector3.up;
        frame.entryBoundaryCurvature = 0f;
        frame.exitBoundaryCurvature = 0f;
        frame.representativeCurvature = 0f;

        if (!useRepresentativeSectionCurvature || !frame.valid)
            return;

        if (TrySampleConnectedSurface(frame, frame.entryPoint, -1f, out Vector3 entryConnectedPoint, out Vector3 entryConnectedNormal, out Collider entryConnectedCollider))
        {
            frame.hasEntryConnectedSurface = true;
            frame.entryConnectedPoint = entryConnectedPoint;
            frame.entryConnectedNormal = entryConnectedNormal;
            frame.entryConnectedCollider = entryConnectedCollider;
            frame.entryBoundaryCurvature = CalculateBoundaryCurvature(entryConnectedNormal, entryNormal, frame.axis, Vector3.Distance(entryConnectedPoint, frame.entryPoint),
                entryNormal);
        }

        if (TrySampleConnectedSurface(frame, frame.exitPoint, 1f, out Vector3 exitConnectedPoint, out Vector3 exitConnectedNormal, out Collider exitConnectedCollider))
        {
            frame.hasExitConnectedSurface = true;
            frame.exitConnectedPoint = exitConnectedPoint;
            frame.exitConnectedNormal = exitConnectedNormal;
            frame.exitConnectedCollider = exitConnectedCollider;
            frame.exitBoundaryCurvature = CalculateBoundaryCurvature(exitNormal, exitConnectedNormal, frame.axis, Vector3.Distance(frame.exitPoint, exitConnectedPoint), exitNormal);
        }

        frame.representativeCurvature = Mathf.Clamp(Mathf.Max(frame.entryBoundaryCurvature, frame.exitBoundaryCurvature), 0f,
            Mathf.Max(minimumCurvature, maximumRepresentativeCurvature));
    }

    bool TrySampleConnectedSurface(SlopeFrame frame, Vector3 boundaryPoint, float directionSign, out Vector3 point, out Vector3 normal, out Collider sampledCollider)
    {
        point = Vector3.zero;
        normal = Vector3.up;
        sampledCollider = null;

        if (!frame.valid || !frame.collider)
            return false;

        directionSign = directionSign < 0f ? -1f : 1f;
        Vector3 guess = boundaryPoint + frame.axis * directionSign * Mathf.Max(0.05f, connectedSurfaceProbeOffset);

        Bounds sectionBounds = frame.collider.bounds;
        float castHeight = Mathf.Max(forwardProbeHeight, sectionBounds.extents.y + sphereRadius + 1f);
        float castDistance = castHeight + sectionBounds.size.y + sphereRadius + forwardProbeDownDistance + connectedSurfaceMaximumGap + 2f;

        Vector3 side = Vector3.Cross(Vector3.up, frame.axis);
        if (side.sqrMagnitude < 0.000001f)
            side = Vector3.Cross(Vector3.forward, frame.axis);
        side = side.sqrMagnitude > 0.000001f ? side.normalized : Vector3.right;

        float lateralOffset = Mathf.Clamp(sphereRadius * 0.1f, 0.05f, 0.35f);

        Vector3[] offsets =
        {
            Vector3.zero,
            side * lateralOffset, -side * lateralOffset,
            frame.axis * lateralOffset, -frame.axis * lateralOffset
        };

        float maximumGap = Mathf.Max(0.05f, connectedSurfaceMaximumGap);
        float bestScore = float.PositiveInfinity;
        RaycastHit bestHit = default;
        bool found = false;

        for (int offsetIndex = 0; offsetIndex < offsets.Length; offsetIndex++)
        {
            Vector3 origin = guess + offsets[offsetIndex] + Vector3.up * castHeight;
            RaycastHit[] hits = Physics.RaycastAll(origin, Vector3.down, castDistance, groundMask, QueryTriggerInteraction.Ignore);

            for (int hitIndex = 0; hitIndex < hits.Length; hitIndex++)
            {
                RaycastHit hit = hits[hitIndex];
                Collider candidate = hit.collider;

                if (!candidate || candidate == frame.collider || candidate.isTrigger || candidate.attachedRigidbody == rb)
                {
                    continue;
                }

                if (curvatureTrackRoot && candidate.transform != curvatureTrackRoot && !candidate.transform.IsChildOf(curvatureTrackRoot))
                {
                    continue;
                }

                Vector3 candidateNormal = hit.normal.normalized;
                if (candidateNormal.sqrMagnitude < 0.999f)
                    continue;

                float surfaceAngle = Vector3.Angle(candidateNormal, Vector3.up);
                if (surfaceAngle > maxSlopeAngle)
                    continue;

                Vector3 delta = hit.point - boundaryPoint;
                float forwardDistance = Vector3.Dot(delta, frame.axis) * directionSign;
                float lateralDistance = Mathf.Abs(Vector3.Dot(delta, side));
                float directDistance = delta.magnitude;

                if (forwardDistance < -0.1f || forwardDistance > maximumGap || lateralDistance > maximumGap || directDistance > maximumGap * 1.5f)
                {
                    continue;
                }

                Vector3 candidateTangent = Vector3.ProjectOnPlane(frame.axis, candidateNormal);
                if (candidateTangent.sqrMagnitude < 0.000001f)
                    continue;

                float targetForwardDistance = Mathf.Max(0.05f, connectedSurfaceProbeOffset);
                float score = Mathf.Abs(forwardDistance - targetForwardDistance) + lateralDistance * 2f + directDistance * 0.25f;

                if (score >= bestScore)
                    continue;

                bestScore = score;
                bestHit = hit;
                found = true;
            }
        }

        if (!found)
            return false;

        point = bestHit.point;
        normal = bestHit.normal.normalized;
        sampledCollider = bestHit.collider;
        return true;
    }

    float CalculateBoundaryCurvature(Vector3 beforeNormal, Vector3 afterNormal, Vector3 travelAxis, float measuredDistance, Vector3 referenceNormal)
    {
        Vector3 beforeTangent = Vector3.ProjectOnPlane(travelAxis, beforeNormal);
        Vector3 afterTangent = Vector3.ProjectOnPlane(travelAxis, afterNormal);

        if (beforeTangent.sqrMagnitude < 0.000001f || afterTangent.sqrMagnitude < 0.000001f)
        {
            return 0f;
        }

        beforeTangent.Normalize();
        afterTangent.Normalize();
        if (Vector3.Dot(beforeTangent, afterTangent) < 0f)
            afterTangent = -afterTangent;

        float regularizedDistance = Mathf.Max(measuredDistance, Mathf.Max(0.01f, minimumBoundaryCurvatureDistance));
        Vector3 tangentDerivative = (afterTangent - beforeTangent) / regularizedDistance;
        float signedCurvature = Vector3.Dot(tangentDerivative, -referenceNormal.normalized);

        return Mathf.Clamp(Mathf.Max(0f, signedCurvature), 0f, Mathf.Max(minimumCurvature, maximumRepresentativeCurvature));
    }

    float CalculateCurvatureAdjustedMaximumDeceleration(float speed, float curvature, float availableNormalAcceleration, out float normalDemandRatio)
    {
        speed = Mathf.Max(0f, speed);
        curvature = Mathf.Max(0f, curvature);
        float maximumDeceleration = Mathf.Max(0.1f, maximumCriticalDeceleration);
        float minimumDeceleration = Mathf.Clamp(minimumCurvatureAdjustedDeceleration, 0f, maximumDeceleration);

        if (curvature <= minimumCurvature || availableNormalAcceleration <= 0.0001f)
        {
            normalDemandRatio = 0f;
            return maximumDeceleration;
        }

        float requiredNormalAcceleration = speed * speed * curvature;
        normalDemandRatio = requiredNormalAcceleration / availableNormalAcceleration;
        float clampedUsage = Mathf.Clamp01(normalDemandRatio);

        // 法線方向と接線方向の加速度容量を楕円として配分します。
        // 曲率による法線需要が増えるほど、制動距離の逆算に使う最大減速度を下げます。
        float remainingTangentialRatio = Mathf.Sqrt(Mathf.Max(0f, 1f - clampedUsage * clampedUsage));

        return Mathf.Lerp(minimumDeceleration, maximumDeceleration, remainingTangentialRatio);
    }

    void SolveGround(Vector3 move, float dt)
    {
        Vector3 velocity = rb.velocity;
        Vector3 tangentVelocity = Vector3.ProjectOnPlane(velocity, groundNormal);
        Vector3 desiredDirection = Vector3.ProjectOnPlane(move, groundNormal);

        if (desiredDirection.sqrMagnitude < 0.000001f)
            desiredDirection = Vector3.ProjectOnPlane(heading, groundNormal);
        desiredDirection = desiredDirection.sqrMagnitude > 0.000001f ? desiredDirection.normalized : Vector3.zero;

        SlopeFrame evaluationFrame = default;
        float startProgress = 0f;
        float distanceToSlopeEntry = 0f;

        if (groundKind == GroundKind.Slope && slopeFrame.valid)
        {
            evaluationFrame = slopeFrame;
            startProgress = CalculateProgress(slopeFrame, rb.position);
            currentSlopeProgressPercent = startProgress * 100f;
        }
        else if (groundKind == GroundKind.Flat && hasForwardSlope && forwardSlopeFrame.valid)
        {
            evaluationFrame = forwardSlopeFrame;
            startProgress = 0f;
            currentSlopeProgressPercent = 0f;
            distanceToSlopeEntry = Mathf.Max(0f, Vector3.Dot(evaluationFrame.entryPoint - rb.position, NormalizeFlat(move, heading)));
        }
        else
        {
            currentSlopeProgressPercent = 0f;
        }

        float forwardSpeed = desiredDirection.sqrMagnitude > 0f ? Mathf.Max(0f, Vector3.Dot(tangentVelocity, desiredDirection)) : tangentVelocity.magnitude;
        currentTangentSpeed = forwardSpeed;

        // 実斜面へ接触した時だけ新しい解放セッションを開始します。
        // 50%解放後にAirやFlatへ移ってもLatchは保持し、人工力を復活させません。
        UpdateNaturalArtificialReleaseState(evaluationFrame, startProgress);

        float allowedSpeed = maxGroundSpeed;
        SurfaceSample controllingSample = default;

        if (useCriticalBoundaryTracking && evaluationFrame.valid)
        {
            allowedSpeed = CalculateAllowedSpeedEnvelope(evaluationFrame, startProgress, distanceToSlopeEntry, forwardSpeed, out controllingSample);
        }

        currentAllowedSpeed = Mathf.Clamp(allowedSpeed, 0f, maxGroundSpeed);
        controllingSamplePoint = controllingSample.valid ? controllingSample.point : Vector3.zero;

        if (controllingSample.valid)
        {
            controllingSampleProgressPercent = controllingSample.progress * 100f;
            currentCurvature = controllingSample.curvature;
            currentLocalCriticalSpeed = controllingSample.criticalSpeed;
            currentAvailableNormalAcceleration = controllingSample.availableNormalAcceleration;
            currentEffectiveMaximumDeceleration = controllingSample.effectiveMaximumDeceleration;
            currentNormalDemandRatio = controllingSample.normalDemandRatio;
        }
        else
        {
            controllingSampleProgressPercent = 0f;
            currentCurvature = 0f;
            currentLocalCriticalSpeed = maxGroundSpeed;
            currentAvailableNormalAcceleration = 0f;
            currentEffectiveMaximumDeceleration = maximumCriticalDeceleration;
            currentNormalDemandRatio = 0f;
        }

        currentRepresentativeCurvature = evaluationFrame.valid ? evaluationFrame.representativeCurvature : 0f;

        Vector3 movementAcceleration = CalculateTangentialAcceleration(tangentVelocity, desiredDirection, currentAllowedSpeed, dt,
            useCriticalBoundaryTracking && evaluationFrame.valid, currentEffectiveMaximumDeceleration);

        movementAcceleration = ApplyTargetProgressPreconditioning(movementAcceleration, tangentVelocity, desiredDirection, evaluationFrame, startProgress, dt);

        // 接線制御とAdaptive Base Stickは同じRelease Weightを参照します。
        // 接線制御は直接Weightを掛け、Base Stickは速度・曲率から必要量を逆算した後、
        // Release Weightで定めた容量内へ収めます。固定Base Stickで速度を4m/s付近へ縛りません。
        float artificialReleaseWeight = GetCurrentArtificialReleaseWeight(evaluationFrame, startProgress);

        movementAcceleration *= artificialReleaseWeight;
        currentTangentialAcceleration *= artificialReleaseWeight;

        currentNormalCorrection = 0f;
        float actualInwardNormalAcceleration = CalculateAdaptiveCriticalBaseStickAcceleration(evaluationFrame, startProgress, forwardSpeed, movementAcceleration,
            desiredDirection, artificialReleaseWeight, dt);

        Vector3 normalAcceleration = -groundNormal * actualInwardNormalAcceleration;
        Vector3 totalArtificialAcceleration = movementAcceleration + normalAcceleration;

        if (totalArtificialAcceleration.sqrMagnitude > 0.00000001f)
        {
            rb.AddForce(totalArtificialAcceleration, ForceMode.Acceleration);
        }

        UpdateCriticalRatioAtCurrentPosition(evaluationFrame, startProgress, forwardSpeed, actualInwardNormalAcceleration, dt);

        CompleteTargetProgressPreconditioningIfPending();
    }

    Vector3 ApplyTargetProgressPreconditioning(Vector3 existingMovementAcceleration, Vector3 tangentVelocity, Vector3 desiredDirection, SlopeFrame evaluationFrame,
        float currentProgress, float dt)
    {
        targetProgressRemainingDistance = 0f;
        targetProgressRequiredNetAcceleration = 0f;
        targetProgressGravityAcceleration = 0f;
        targetProgressDesiredArtificialAcceleration = 0f;
        targetProgressAppliedArtificialAcceleration = 0f;

        if (!useTargetProgressPreconditioning || groundKind != GroundKind.Slope || !evaluationFrame.valid || !evaluationFrame.collider)
        {
            return existingMovementAcceleration;
        }

        if (!EnsureTargetProgressPreconditioningPlan(evaluationFrame, currentProgress))
        {
            return existingMovementAcceleration;
        }

        if (targetProgressPhase != TargetProgressPreconditionPhase.Preconditioning || !targetProgressPlanValid || targetProgressPlanCollider != evaluationFrame.collider)
        {
            return existingMovementAcceleration;
        }

        float targetProgress = Mathf.Clamp01(capturedTargetProgressPercent * 0.01f);
        float remainingProgress = targetProgress - currentProgress;

        if (remainingProgress <= 0f)
        {
            targetProgressPhase = TargetProgressPreconditionPhase.Completed;
            targetProgressCompletionPending = true;
            previousTargetProgressAppliedAcceleration = 0f;
            return existingMovementAcceleration;
        }

        Vector3 travelTangent = Vector3.ProjectOnPlane(evaluationFrame.axis, groundNormal);
        if (travelTangent.sqrMagnitude < 0.000001f)
            return existingMovementAcceleration;

        travelTangent.Normalize();
        if (desiredDirection.sqrMagnitude > 0.000001f && Vector3.Dot(travelTangent, desiredDirection) < 0f)
        {
            travelTangent = -travelTangent;
        }

        float currentSpeed = Mathf.Max(0f, Vector3.Dot(tangentVelocity, travelTangent));

        targetProgressRemainingDistance = remainingProgress * Mathf.Max(evaluationFrame.projectedLength, 0.0001f);

        float safeDistance = Mathf.Max(targetProgressRemainingDistance, targetProgressMinimumDistance);

        targetProgressRequiredNetAcceleration = (capturedTargetTangentSpeed * capturedTargetTangentSpeed - currentSpeed * currentSpeed) / (2f * safeDistance);

        targetProgressGravityAcceleration = compensateTargetProgressGravity ? Vector3.Dot(Physics.gravity, travelTangent) : 0f;

        targetProgressDesiredArtificialAcceleration = targetProgressRequiredNetAcceleration - targetProgressGravityAcceleration;

        // 目標より遅い場合も人工加速はしません。

        targetProgressAppliedArtificialAcceleration = Mathf.MoveTowards(previousTargetProgressAppliedAcceleration, targetProgressDesiredArtificialAcceleration,
            Mathf.Max(0.1f, targetProgressJerkLimit) * dt);

        previousTargetProgressAppliedAcceleration = targetProgressAppliedArtificialAcceleration;

        // 既存接線力へ加算しません。
        // 進行軸成分だけを目標事前調整へ置き換え、横方向の補正は保持します。
        float existingAlongTravel = Vector3.Dot(existingMovementAcceleration, travelTangent);
        Vector3 preservedOtherAcceleration = existingMovementAcceleration - travelTangent * existingAlongTravel;

        currentTangentialAcceleration = targetProgressAppliedArtificialAcceleration;

        return preservedOtherAcceleration + travelTangent * targetProgressAppliedArtificialAcceleration;
    }

    bool EnsureTargetProgressPreconditioningPlan(SlopeFrame evaluationFrame, float currentProgress)
    {
        if (!useTargetProgressPreconditioning || !evaluationFrame.valid || !evaluationFrame.collider)
        {
            return false;
        }

        if (targetProgressPlanCollider && targetProgressPlanCollider != evaluationFrame.collider)
        {
            ResetTargetProgressPreconditioning("SlopeChanged", targetProgressPlanValid || targetProgressPhase == TargetProgressPreconditionPhase.Preconditioning);
        }

        if (!targetProgressPlanCollider)
        {
            targetProgressPlanCollider = evaluationFrame.collider;
            targetProgressPlanColliderName = evaluationFrame.collider.name;
            targetProgressPhase = TargetProgressPreconditionPhase.Observing;
        }

        if (targetProgressPhase == TargetProgressPreconditionPhase.Completed)
            return false;

        if (targetProgressPlanValid)
            return true;

        bool observationStable = slopeProgressObservationValid && readyForLimitCapture && stableSlopeContactFrames >= requiredStableSlopeFrames &&
            progressObservationCollider == evaluationFrame.collider;

        if (!observationStable)
        {
            targetProgressPhase = TargetProgressPreconditionPhase.Observing;
            return false;
        }

        float targetProgress = Mathf.Clamp01(targetSlopeProgressPercent * 0.01f);

        if (currentProgress >= targetProgress)
        {
            capturedTargetProgressPercent = targetSlopeProgressPercent;
            targetProgressPhase = TargetProgressPreconditionPhase.Completed;
            targetProgressCompletionPending = true;
            EnsureNaturalArtificialReleasePlan(evaluationFrame, currentTangentSpeed);
            return false;
        }

        if (!TryEvaluateSurface(evaluationFrame, targetProgress, out SurfaceSample targetSample) || targetSample.curvature <= minimumCurvature ||
            targetSample.availableNormalAcceleration <= 0.0001f)
        {
            targetProgressPhase = TargetProgressPreconditionPhase.Observing;

            if (!targetProgressPlanFailureLogged && writeRuntimeLog && logTargetProgressPreconditioning)
            {
                targetProgressPlanFailureLogged = true;
                Debug.LogWarning($"[TARGET PROGRESS PLAN WAITING] time={Time.fixedTime:F4}s " + $"collider={evaluationFrame.collider.name} " +
                    $"target={targetSlopeProgressPercent:F3}% " + $"reason=TargetSurfaceModelInvalid", this);
            }

            return false;
        }

        targetProgressPlanFailureLogged = false;
        targetProgressPlanValid = true;
        targetProgressPhase = TargetProgressPreconditionPhase.Preconditioning;
        capturedTargetProgressPercent = targetSlopeProgressPercent;
        capturedTargetTangentSpeed = Mathf.Clamp(targetSample.criticalSpeed, 0f, maxGroundSpeed);
        capturedTargetCurvature = targetSample.curvature;
        capturedTargetAvailableNormalAcceleration = targetSample.availableNormalAcceleration;
        previousTargetProgressAppliedAcceleration = 0f;

        EnsureNaturalArtificialReleasePlan(evaluationFrame, Mathf.Max(currentTangentSpeed, capturedTargetTangentSpeed));

        if (writeRuntimeLog && logTargetProgressPreconditioning)
        {
            Debug.Log($"[TARGET PROGRESS PLAN CAPTURED] time={Time.fixedTime:F4}s " + $"collider={evaluationFrame.collider.name} " +
                $"currentProgress={currentProgress * 100f:F3}% " + $"targetProgress={capturedTargetProgressPercent:F3}% " + $"targetSpeed={capturedTargetTangentSpeed:F6} " +
                $"targetCurvature={capturedTargetCurvature:F8} " + $"targetAvailable={capturedTargetAvailableNormalAcceleration:F6} " + $"targetRatio={targetCriticalRatio:F6} " +
                $"additionalAddForce=false", this);
        }

        return true;
    }

    void CompleteTargetProgressPreconditioningIfPending()
    {
        if (!targetProgressCompletionPending)
            return;

        targetProgressCompletionPending = false;
        previousTargetProgressAppliedAcceleration = 0f;
        targetProgressAppliedArtificialAcceleration = 0f;

        if (writeRuntimeLog && logTargetProgressPreconditioning)
        {
            Debug.Log($"[TARGET PROGRESS PRECONDITION COMPLETED] time={Time.fixedTime:F4}s " +
                $"collider={(targetProgressPlanCollider ? targetProgressPlanCollider.name : "None")} " + $"targetProgress={capturedTargetProgressPercent:F3}% " +
                $"actualProgress={currentSlopeProgressPercent:F3}% " + $"actualSpeed={currentTangentSpeed:F6} " + $"targetSpeed={capturedTargetTangentSpeed:F6} " +
                $"actualRatio={currentCriticalRatio:F6} " + $"targetRatio={targetCriticalRatio:F6} " + $"targetControlReleased=true " +
                $"naturalReleaseWeight={currentArtificialReleaseWeight:F6} " + $"naturalMotionReleased={naturalMotionReleased} pullback=false", this);
        }
    }

    void UpdateNaturalArtificialReleaseState(SlopeFrame evaluationFrame, float currentProgress)
    {
        if (!useNaturalArtificialRelease)
        {
            currentArtificialReleaseWeight = 1f;
            currentCumulativeReleaseRatio = 0f;
            return;
        }

        // Forward検出だけでは新セッションに切り替えません。
        // 実際に斜面へ接触した時だけ、前区間のRelease Latchを解除します。
        if (groundKind == GroundKind.Slope && evaluationFrame.valid && evaluationFrame.collider)
        {
            if (naturalReleasePlanCollider && naturalReleasePlanCollider != evaluationFrame.collider)
            {
                ResetNaturalArtificialRelease("NewSlopeContact", true);
            }

            if (!naturalReleasePlanCollider)
            {
                naturalReleasePlanCollider = evaluationFrame.collider;
                naturalReleasePlanColliderName = evaluationFrame.collider.name;
            }
        }

        if (!naturalReleasePlanValid || naturalMotionReleased || !evaluationFrame.valid || evaluationFrame.collider != naturalReleasePlanCollider)
        {
            return;
        }

        float progressPercent = currentProgress * 100f;
        if (progressPercent < capturedNaturalReleaseProgressPercent)
            return;

        naturalMotionReleased = true;
        currentArtificialReleaseWeight = 0f;
        currentCumulativeReleaseRatio = 1f;
        currentEffectiveBaseStickAcceleration = 0f;
        currentAdaptivePredictedSpeed = 0f;
        currentAdaptiveDesiredBaseStickAcceleration = 0f;
        currentAdaptiveStickCapacity = 0f;
        currentAdaptiveOutwardDampingAcceleration = 0f;
        adaptiveStickSaturated = false;
        previousAdaptiveBaseStickAcceleration = 0f;
        adaptiveStickActiveCollider = null;
        previousTargetProgressAppliedAcceleration = 0f;
        targetProgressAppliedArtificialAcceleration = 0f;

        if (!naturalReleaseLatchLogged && writeRuntimeLog && logNaturalArtificialRelease)
        {
            naturalReleaseLatchLogged = true;
            Debug.Log($"[NATURAL ARTIFICIAL RELEASE LATCHED] time={Time.fixedTime:F4}s " + $"collider={(naturalReleasePlanCollider ? naturalReleasePlanCollider.name : "None")} " +
                $"progress={progressPercent:F3}% " + $"releasePoint={capturedNaturalReleaseProgressPercent:F3}% " + $"baseStick=0 tangentialArtificial=0 airArtificial=0 " +
                $"handoff=GravityContactFrictionRestitution", this);
        }
    }

    bool EnsureNaturalArtificialReleasePlan(SlopeFrame frame, float speedCandidate)
    {
        if (!useNaturalArtificialRelease || !frame.valid || !frame.collider)
        {
            return false;
        }

        if (naturalReleasePlanCollider && naturalReleasePlanCollider != frame.collider)
        {
            ResetNaturalArtificialRelease("ReleasePlanSlopeChanged", true);
        }

        naturalReleasePlanCollider = frame.collider;
        naturalReleasePlanColliderName = frame.collider.name;

        if (naturalReleasePlanValid)
            return true;

        float releaseEnd = Mathf.Clamp(naturalReleaseProgressPercent, 0f, 100f);
        float minimumStart = Mathf.Clamp(targetSlopeProgressPercent + releaseHoldAfterTargetPercent, 0f, releaseEnd);
        float availableWidth = Mathf.Max(0f, releaseEnd - minimumStart);

        if (availableWidth <= 0.0001f && writeRuntimeLog && logNaturalArtificialRelease)
        {
            Debug.LogWarning($"[NATURAL RELEASE CONFIG INVALID] targetProgress={targetSlopeProgressPercent:F3}% " + $"hold={releaseHoldAfterTargetPercent:F3}% " +
                $"releaseEnd={releaseEnd:F3}% reason=ReleaseIntervalIsZero " + $"recommended=Target30_Release50", this);
        }

        float safeMaxGroundSpeed = Mathf.Max(0.01f, maxGroundSpeed);
        float speedFloor = safeMaxGroundSpeed * Mathf.Clamp01(releaseSpeedFloorRatio);
        float designSpeed = Mathf.Clamp(Mathf.Max(speedCandidate, capturedTargetTangentSpeed, speedFloor), 0f, safeMaxGroundSpeed);

        float desiredWidth = designSpeed * Mathf.Max(0.05f, releaseNominalSeconds) / Mathf.Max(0.0001f, frame.projectedLength) * 100f;

        float minimumWidth = Mathf.Min(Mathf.Max(0.1f, minimumReleaseWidthPercent), availableWidth);
        float releaseWidth = availableWidth > 0f ? Mathf.Clamp(desiredWidth, minimumWidth, availableWidth) : 0f;

        capturedReleaseStartProgressPercent = releaseEnd - releaseWidth;
        capturedNaturalReleaseProgressPercent = releaseEnd;
        capturedReleaseDesignSpeed = designSpeed;
        naturalReleasePlanValid = true;
        naturalReleaseLatchLogged = false;

        if (writeRuntimeLog && logNaturalArtificialRelease)
        {
            Debug.Log($"[NATURAL ARTIFICIAL RELEASE PLAN CAPTURED] time={Time.fixedTime:F4}s " + $"collider={frame.collider.name} " +
                $"targetProgress={targetSlopeProgressPercent:F3}% " + $"fullStickUntil={minimumStart:F3}% " + $"releaseStart={capturedReleaseStartProgressPercent:F3}% " +
                $"releaseEnd={capturedNaturalReleaseProgressPercent:F3}% " + $"designSpeed={capturedReleaseDesignSpeed:F6} " + $"maxGroundSpeed={maxGroundSpeed:F6} " +
                $"releaseWidth={releaseWidth:F3}% " + $"baseStickMode={(useAdaptiveCriticalBaseStick ? "AdaptiveCritical" : "FixedFallback")} " +
                $"adaptiveMaximum={maximumAdaptiveBaseStickAcceleration:F6} baseStickEnd=0", this);
        }

        return true;
    }

    float GetCurrentArtificialReleaseWeight(SlopeFrame frame, float progress)
    {
        float weight = EvaluateArtificialReleaseWeight(frame.collider, progress * 100f);

        currentArtificialReleaseWeight = weight;
        currentCumulativeReleaseRatio = 1f - weight;
        return weight;
    }

    float CalculateRequiredBaseStickForSpeed(float speed, float curvature, float gravitySupport)
    {
        if (curvature <= minimumCurvature)
            return 0f;

        float safeRatio = Mathf.Clamp(targetCriticalRatio, 0.01f, 0.999f);
        float requiredAvailableNormalAcceleration = speed * speed * curvature / safeRatio;

        return Mathf.Max(0f, requiredAvailableNormalAcceleration - gravitySupport);
    }

    float GetPredictedBaseStickCapacity(SlopeFrame frame, float progress, float curvature, float gravitySupport)
    {
        float releaseWeight = EvaluateArtificialReleaseWeight(frame.collider, progress * 100f);

        if (!useAdaptiveCriticalBaseStick)
            return baseStickAcceleration * releaseWeight;

        float requiredAtMaxGroundSpeed = CalculateRequiredBaseStickForSpeed(Mathf.Max(0f, maxGroundSpeed), curvature, gravitySupport);

        float uncappedCapacity = requiredAtMaxGroundSpeed * releaseWeight;
        return Mathf.Min(uncappedCapacity, Mathf.Max(0.1f, maximumAdaptiveBaseStickAcceleration));
    }

    float CalculateAdaptiveCriticalBaseStickAcceleration(SlopeFrame frame, float progress, float currentSpeed, Vector3 movementAcceleration, Vector3 desiredDirection,
        float releaseWeight, float dt)
    {
        currentAdaptivePredictedSpeed = currentSpeed;
        currentAdaptiveDesiredBaseStickAcceleration = 0f;
        currentAdaptiveStickCapacity = 0f;
        currentAdaptiveOutwardDampingAcceleration = 0f;
        adaptiveStickSaturated = false;

        if (naturalMotionReleased || releaseWeight <= 0f)
        {
            previousAdaptiveBaseStickAcceleration = 0f;
            currentEffectiveBaseStickAcceleration = 0f;
            return 0f;
        }

        // 平面または曲率を評価できない箇所では、従来値を接触安定用のFallbackとして残します。
        if (groundKind != GroundKind.Slope || !frame.valid || !TryEvaluateSurface(frame, progress, out SurfaceSample sample) || sample.curvature <= minimumCurvature)
        {
            float fallback = baseStickAcceleration * releaseWeight;
            previousAdaptiveBaseStickAcceleration = fallback;
            currentEffectiveBaseStickAcceleration = fallback;
            adaptiveStickActiveCollider = frame.collider;
            return fallback;
        }

        if (!useAdaptiveCriticalBaseStick)
        {
            float fixedStick = baseStickAcceleration * releaseWeight;
            previousAdaptiveBaseStickAcceleration = fixedStick;
            currentEffectiveBaseStickAcceleration = fixedStick;
            adaptiveStickActiveCollider = frame.collider;
            return fixedStick;
        }

        Vector3 travelTangent = Vector3.ProjectOnPlane(desiredDirection.sqrMagnitude > 0.000001f ? desiredDirection : sample.tangent, sample.normal);
        if (travelTangent.sqrMagnitude < 0.000001f)
            travelTangent = sample.tangent;
        travelTangent.Normalize();
        if (Vector3.Dot(travelTangent, sample.tangent) < 0f)
            travelTangent = -travelTangent;

        float positiveGravityAcceleration = Mathf.Max(0f, Vector3.Dot(Physics.gravity, travelTangent));
        float positiveArtificialAcceleration = Mathf.Max(0f, Vector3.Dot(movementAcceleration, travelTangent));
        float predictionSeconds = Mathf.Max(dt, adaptiveStickPredictionSeconds);

        float predictedSpeed = Mathf.Clamp(currentSpeed + (positiveGravityAcceleration + positiveArtificialAcceleration) * predictionSeconds, 0f, Mathf.Max(0f, maxGroundSpeed));

        float requiredStick = CalculateRequiredBaseStickForSpeed(predictedSpeed, sample.curvature, sample.gravitySupport);

        float capacity = GetPredictedBaseStickCapacity(frame, progress, sample.curvature, sample.gravitySupport);

        float outwardNormalSpeed = Mathf.Max(0f, Vector3.Dot(rb.velocity, sample.normal));
        float outwardDampingAcceleration = outwardNormalSpeed / Mathf.Max(dt, adaptiveOutwardNormalResponseSeconds);

        float desiredStick = Mathf.Min(requiredStick + outwardDampingAcceleration, capacity);

        bool newSlopeContact = adaptiveStickActiveCollider != frame.collider;
        float appliedStick;
        if (newSlopeContact)
        {
            // 斜面の最初のFixedUpdateだけはJerk待ちをせず必要値を即時採用します。
            appliedStick = desiredStick;
            adaptiveStickActiveCollider = frame.collider;
        }
        else
        {
            float jerkLimit = desiredStick >= previousAdaptiveBaseStickAcceleration ? adaptiveStickRiseJerkLimit : adaptiveStickFallJerkLimit;
            appliedStick = Mathf.MoveTowards(previousAdaptiveBaseStickAcceleration, desiredStick, Mathf.Max(0.1f, jerkLimit) * dt);
        }

        appliedStick = Mathf.Clamp(appliedStick, 0f, capacity);
        previousAdaptiveBaseStickAcceleration = appliedStick;

        currentAdaptivePredictedSpeed = predictedSpeed;
        currentAdaptiveDesiredBaseStickAcceleration = requiredStick;
        currentAdaptiveStickCapacity = capacity;
        currentAdaptiveOutwardDampingAcceleration = outwardDampingAcceleration;
        currentEffectiveBaseStickAcceleration = appliedStick;
        adaptiveStickSaturated = requiredStick + outwardDampingAcceleration > capacity + 0.001f;

        if (adaptiveStickSaturated)
        {
            if (!adaptiveStickSaturationLogged && writeRuntimeLog && logAdaptiveCriticalBaseStick)
            {
                adaptiveStickSaturationLogged = true;
                Debug.LogWarning($"[ADAPTIVE CRITICAL STICK SATURATED] time={Time.fixedTime:F4}s " + $"collider={frame.collider.name} progress={progress * 100f:F3}% " +
                    $"speed={currentSpeed:F6} predictedSpeed={predictedSpeed:F6} " + $"required={requiredStick:F6} damping={outwardDampingAcceleration:F6} " +
                    $"capacity={capacity:F6} maxGroundSpeed={maxGroundSpeed:F6} " + $"maximumAdaptive={maximumAdaptiveBaseStickAcceleration:F6}", this);
            }
        }
        else
        {
            adaptiveStickSaturationLogged = false;
        }

        return appliedStick;
    }

    float EvaluateArtificialReleaseWeight(Collider frameCollider, float progressPercent)
    {
        if (!useNaturalArtificialRelease)
            return 1f;

        // Latch後はAir、Flat、SphereCastの揺れでも人工力を復活させません。
        if (naturalMotionReleased)
            return 0f;

        if (!naturalReleasePlanValid || !naturalReleasePlanCollider || frameCollider != naturalReleasePlanCollider)
        {
            return 1f;
        }

        if (progressPercent <= capturedReleaseStartProgressPercent)
            return 1f;
        if (progressPercent >= capturedNaturalReleaseProgressPercent)
            return 0f;

        float u = Mathf.InverseLerp(capturedReleaseStartProgressPercent, capturedNaturalReleaseProgressPercent, progressPercent);

        // 30u^2(1-u)^2を0～uまで累積したSmootherStepです。
        // 開始端と終了端で傾き・曲率が0になり、人工力を連続的に解放します。
        float cumulativeRelease = u * u * u * (u * (u * 6f - 15f) + 10f);

        return 1f - cumulativeRelease;
    }

    void ResetNaturalArtificialRelease(string reason, bool writeResetLog)
    {
        bool hadPlanOrLatch = naturalReleasePlanValid || naturalMotionReleased || naturalReleasePlanCollider;

        if (writeResetLog && hadPlanOrLatch && writeRuntimeLog && logNaturalArtificialRelease)
        {
            Debug.Log($"[NATURAL ARTIFICIAL RELEASE RESET] time={Time.fixedTime:F4}s " + $"collider={(naturalReleasePlanCollider ? naturalReleasePlanCollider.name : "None")} " +
                $"released={naturalMotionReleased} reason={reason}", this);
        }

        naturalReleasePlanValid = false;
        naturalMotionReleased = false;
        naturalReleasePlanCollider = null;
        naturalReleasePlanColliderName = string.Empty;
        capturedReleaseStartProgressPercent = 0f;
        capturedNaturalReleaseProgressPercent = 0f;
        capturedReleaseDesignSpeed = 0f;
        currentArtificialReleaseWeight = 1f;
        currentCumulativeReleaseRatio = 0f;
        currentAdaptivePredictedSpeed = 0f;
        currentAdaptiveDesiredBaseStickAcceleration = 0f;
        currentAdaptiveStickCapacity = 0f;
        currentAdaptiveOutwardDampingAcceleration = 0f;
        currentEffectiveBaseStickAcceleration = baseStickAcceleration;
        adaptiveStickSaturated = false;
        previousAdaptiveBaseStickAcceleration = 0f;
        adaptiveStickActiveCollider = null;
        adaptiveStickSaturationLogged = false;
        naturalReleaseLatchLogged = false;
    }

    void ResetTargetProgressPreconditioning(string reason, bool writeResetLog)
    {
        bool hadActivePlan = targetProgressPlanValid || targetProgressPhase == TargetProgressPreconditionPhase.Preconditioning;

        if (writeResetLog && hadActivePlan && writeRuntimeLog && logTargetProgressPreconditioning)
        {
            Debug.Log($"[TARGET PROGRESS PRECONDITION RESET] time={Time.fixedTime:F4}s " + $"collider={(targetProgressPlanCollider ? targetProgressPlanCollider.name : "None")} " +
                $"phase={targetProgressPhase} reason={reason}", this);
        }

        targetProgressPhase = TargetProgressPreconditionPhase.Inactive;
        targetProgressPlanValid = false;
        targetProgressPlanCollider = null;
        targetProgressPlanColliderName = string.Empty;
        capturedTargetProgressPercent = 0f;
        capturedTargetTangentSpeed = 0f;
        capturedTargetCurvature = 0f;
        capturedTargetAvailableNormalAcceleration = 0f;
        targetProgressRemainingDistance = 0f;
        targetProgressRequiredNetAcceleration = 0f;
        targetProgressGravityAcceleration = 0f;
        targetProgressDesiredArtificialAcceleration = 0f;
        targetProgressAppliedArtificialAcceleration = 0f;
        previousTargetProgressAppliedAcceleration = 0f;
        targetProgressCompletionPending = false;
        targetProgressPlanFailureLogged = false;
    }

    Vector3 CalculateTangentialAcceleration(Vector3 tangentVelocity, Vector3 desiredDirection, float allowedSpeed, float dt, bool criticalControlActive,
        float maximumDeceleration)
    {
        if (desiredDirection.sqrMagnitude < 0.000001f)
            return Vector3.zero;

        Vector3 desiredVelocity = desiredDirection * allowedSpeed;
        Vector3 raw = (desiredVelocity - tangentVelocity) / dt;

        if (!criticalControlActive)
        {
            currentTangentialAcceleration = Vector3.Dot(raw, desiredDirection);
            return Vector3.ClampMagnitude(raw, maxGroundAcceleration);
        }

        float currentForward = Vector3.Dot(tangentVelocity, desiredDirection);
        float speedError = allowedSpeed - currentForward;
        float desiredForwardAcceleration = speedError / Mathf.Max(criticalResponseSeconds, dt);
        desiredForwardAcceleration = Mathf.Clamp(desiredForwardAcceleration, -Mathf.Max(0.1f, maximumDeceleration), maximumCriticalAcceleration);

        currentTangentialAcceleration = Mathf.MoveTowards(currentTangentialAcceleration, desiredForwardAcceleration, criticalJerkLimit * dt);

        Vector3 lateralVelocity = tangentVelocity - desiredDirection * currentForward;
        Vector3 lateralAcceleration = Vector3.ClampMagnitude(-lateralVelocity / dt, maxGroundAcceleration);

        return desiredDirection * currentTangentialAcceleration + lateralAcceleration;
    }

    float CalculateAllowedSpeedEnvelope(SlopeFrame frame, float startProgress, float distanceToSlopeEntry, float currentSpeed, out SurfaceSample controllingSample)
    {
        controllingSample = default;
        float allowedSpeed = maxGroundSpeed;
        float dynamicLookAheadDistance = Mathf.Max(sphereRadius * 2f, currentSpeed * criticalLookAheadSeconds);
        float progressDistance = dynamicLookAheadDistance / Mathf.Max(frame.projectedLength, 0.0001f);
        float endProgress = Mathf.Clamp01(startProgress + progressDistance);
        int count = Mathf.Max(4, criticalSampleCount);

        for (int i = 0; i <= count; i++)
        {
            float t = i / (float)count;
            float progress = Mathf.Lerp(startProgress, endProgress, t);

            if (!TryEvaluateSurface(frame, progress, out SurfaceSample sample))
                continue;

            float slopeDistance = Mathf.Max(0f, (progress - startProgress) * frame.projectedLength);
            sample.distanceAhead = distanceToSlopeEntry + slopeDistance;
            sample.effectiveMaximumDeceleration = CalculateCurvatureAdjustedMaximumDeceleration(currentSpeed, sample.curvature, sample.availableNormalAcceleration,
                out sample.normalDemandRatio);

            float reachableAllowedSpeed = Mathf.Sqrt(Mathf.Max(0f, sample.criticalSpeed * sample.criticalSpeed + 2f * sample.effectiveMaximumDeceleration * sample.distanceAhead));

            reachableAllowedSpeed = Mathf.Min(reachableAllowedSpeed, maxGroundSpeed);

            if (!controllingSample.valid || reachableAllowedSpeed < allowedSpeed)
            {
                allowedSpeed = reachableAllowedSpeed;
                controllingSample = sample;
            }
        }

        return allowedSpeed;
    }

    bool TryEvaluateSurface(SlopeFrame frame, float progress, out SurfaceSample sample)
    {
        sample = default;
        progress = Mathf.Clamp01(progress);

        if (!TrySampleAtProgress(frame, progress, out Vector3 point, out Vector3 normal))
        {
            return false;
        }

        Vector3 tangent = Vector3.ProjectOnPlane(frame.axis, normal);
        if (tangent.sqrMagnitude < 0.000001f)
            return false;
        tangent.Normalize();

        float curvature = 0f;

        // 1枚の板内部は法線一定なので局所曲率が0になります。
        // 接続面を取得できた区間では、Entry/Exitは1枚単位のまま、
        // 境界から作った代表曲率を区間全体の予測値として使用します。
        if (useRepresentativeSectionCurvature && frame.representativeCurvature > minimumCurvature)
        {
            curvature = frame.representativeCurvature;
        }
        else
        {
            float step = Mathf.Max(curvatureProgressStep, 0.001f);
            float previousProgress = Mathf.Clamp01(progress - step);
            float nextProgress = Mathf.Clamp01(progress + step);

            if (!Mathf.Approximately(previousProgress, nextProgress) && TrySampleAtProgress(frame, previousProgress, out Vector3 previousPoint, out Vector3 previousNormal) &&
                TrySampleAtProgress(frame, nextProgress, out Vector3 nextPoint, out Vector3 nextNormal))
            {
                Vector3 previousTangent = Vector3.ProjectOnPlane(frame.axis, previousNormal);
                Vector3 nextTangent = Vector3.ProjectOnPlane(frame.axis, nextNormal);

                if (previousTangent.sqrMagnitude > 0.000001f && nextTangent.sqrMagnitude > 0.000001f)
                {
                    previousTangent.Normalize();
                    nextTangent.Normalize();
                    if (Vector3.Dot(previousTangent, nextTangent) < 0f)
                        nextTangent = -nextTangent;

                    float arcDistance = Mathf.Max(Vector3.Distance(previousPoint, nextPoint), 0.0001f);
                    Vector3 tangentDerivative = (nextTangent - previousTangent) / arcDistance;
                    float signedCurvature = Vector3.Dot(tangentDerivative, -normal);
                    curvature = Mathf.Max(0f, signedCurvature);
                }
            }
        }

        curvature = Mathf.Max(0f, curvature);

        float gravitySupport = Mathf.Max(0f, Vector3.Dot(Physics.gravity, -normal));
        float predictedBaseStickCapacity = GetPredictedBaseStickCapacity(frame, progress, curvature, gravitySupport);
        float available = gravitySupport + predictedBaseStickCapacity;
        float criticalSpeed = curvature > minimumCurvature ? Mathf.Sqrt(Mathf.Max(0f, targetCriticalRatio * available / curvature)) : maxGroundSpeed;

        sample.valid = true;
        sample.progress = progress;
        sample.point = point;
        sample.normal = normal;
        sample.tangent = tangent;
        sample.curvature = curvature;
        sample.gravitySupport = gravitySupport;
        sample.availableNormalAcceleration = available;
        sample.criticalSpeed = Mathf.Min(criticalSpeed, maxGroundSpeed);
        sample.effectiveMaximumDeceleration = maximumCriticalDeceleration;
        sample.normalDemandRatio = 0f;
        return true;
    }

    void UpdateCriticalRatioAtCurrentPosition(SlopeFrame frame, float progress, float tangentSpeed, float actualInwardNormalAcceleration, float dt)
    {
        if (groundKind == GroundKind.Slope)
        {
            Debug.Log("");
        }

        if (!frame.valid || !TryEvaluateSurface(frame, progress, out SurfaceSample currentSample))
        {
            if (groundKind == GroundKind.Slope)
            {
                Debug.Log("");
            }
            currentCriticalRatio = 0f;
            currentRequiredNormalAcceleration = 0f;
            criticalStateMaintained = false;
            criticalMaintainedSeconds = 0f;
            return;
        }

        float available = currentSample.gravitySupport + actualInwardNormalAcceleration;
        float required = tangentSpeed * tangentSpeed * currentSample.curvature;

        currentAvailableNormalAcceleration = available;
        currentRequiredNormalAcceleration = required;
        currentCriticalRatio = available > 0.0001f ? required / available : 0f;

        bool inTolerance = groundKind == GroundKind.Slope && currentSample.curvature > minimumCurvature &&
            Mathf.Abs(currentCriticalRatio - targetCriticalRatio) <= criticalRatioTolerance;

        if (inTolerance)
            criticalMaintainedSeconds += dt;
        else
            criticalMaintainedSeconds = 0f;

        criticalStateMaintained = criticalMaintainedSeconds >= criticalHoldSeconds;
    }

    bool TrySampleAtProgress(SlopeFrame frame, float progress, out Vector3 point, out Vector3 normal)
    {
        if (groundKind == GroundKind.Slope)
        {
            Debug.Log("");
        }

        Vector3 guess = Vector3.Lerp(frame.entryPoint, frame.exitPoint, Mathf.Clamp01(progress));
        if (groundKind == GroundKind.Slope)
        {
            Debug.Log("");
        }
        return TrySampleFrameSurface(frame, guess, out point, out normal, SurfaceSamplePurpose.Progress);
    }

    bool TrySampleFrameSurface(SlopeFrame frame, Vector3 guess, out Vector3 point, out Vector3 normal, SurfaceSamplePurpose purpose)
    {
        // 通常の途中サンプリングでは角度を限定しません。
        return TrySampleFrameSurface(frame, guess, Vector3.zero, 180f, out point, out normal, purpose);
    }

    bool TrySampleFrameSurface(SlopeFrame frame, Vector3 guess, Vector3 expectedNormal, float maxNormalAngleDifference, out Vector3 point, out Vector3 normal,
        SurfaceSamplePurpose purpose)
    {
        point = Vector3.zero;
        normal = Vector3.up;

        if (!frame.valid || frame.sampleColliders == null || frame.sampleColliders.Length == 0)
        {
            return false;
        }

        bool requireNormalMatch = expectedNormal.sqrMagnitude > 0.000001f && maxNormalAngleDifference < 180f;

        if (requireNormalMatch)
            expectedNormal.Normalize();

        Bounds sectionBounds = frame.collider.bounds;

        float castHeight = Mathf.Max(forwardProbeHeight, sectionBounds.extents.y + sphereRadius + 1f);

        float castDistance = castHeight + sectionBounds.size.y + sphereRadius + forwardProbeDownDistance + 2f;

        Vector3 side = Vector3.Cross(Vector3.up, frame.axis);
        if (side.sqrMagnitude < 0.000001f)
            side = Vector3.Cross(Vector3.forward, frame.axis);

        side = side.sqrMagnitude > 0.000001f ? side.normalized : Vector3.right;

        float lateralOffset = Mathf.Max(sphereRadius * 0.2f, 0.02f);

        Vector3[] offsets =
        {
            Vector3.zero,
            side * lateralOffset, -side * lateralOffset,
            frame.axis * lateralOffset, -frame.axis * lateralOffset
        };

        bool found = false;
        float bestScore = float.PositiveInfinity;
        float bestNormalAngle = float.PositiveInfinity;
        RaycastHit bestHit = default;
        bool visualizeThisCall = showEntryExitRayLines && (purpose == SurfaceSamplePurpose.Entry || purpose == SurfaceSamplePurpose.Exit) &&
            ShouldCaptureSlopeRay(frame.collider, purpose);

        for (int offsetIndex = 0; offsetIndex < offsets.Length; offsetIndex++)
        {
            Vector3 origin = guess + offsets[offsetIndex] + Vector3.up * castHeight;

            RaycastHit[] hits = Physics.RaycastAll(origin, Vector3.down, castDistance, groundMask, QueryTriggerInteraction.Ignore);

            if (visualizeThisCall)
            {
                bool hitTargetSlope = false;
                float nearestTargetDistance = float.PositiveInfinity;
                Vector3 displayEnd = origin + Vector3.down * castDistance;

                for (int displayHitIndex = 0; displayHitIndex < hits.Length; displayHitIndex++)
                {
                    RaycastHit displayHit = hits[displayHitIndex];

                    if (!FrameContainsCollider(frame, displayHit.collider))
                        continue;

                    float displaySlopeAngle = Vector3.Angle(displayHit.normal, Vector3.up);

                    if (displaySlopeAngle < minimumSlopeAngle || displaySlopeAngle > maxSlopeAngle)
                    {
                        continue;
                    }

                    if (displayHit.distance >= nearestTargetDistance)
                        continue;

                    nearestTargetDistance = displayHit.distance;
                    displayEnd = displayHit.point;
                    hitTargetSlope = true;
                }

                bool visible = hitTargetSlope || showMissedRayFullLength;

                SetEntryExitRayLine(frame.collider, purpose, offsetIndex, origin, displayEnd, hitTargetSlope, visible);
            }

            if (writeRuntimeLog)
            {
                /* Debug.Log(
                    $"[FRAME SAMPLE RAY] sep={sep} " +
                    $"offset={offsetIndex} " +
                    $"guess={guess:F5} " +
                    $"offsetValue={offsets[offsetIndex]:F5} " +
                    $"origin={origin:F5} " +
                    $"hitCount={hits.Length}",
                    this);*/
            }

            for (int hitIndex = 0; hitIndex < hits.Length; hitIndex++)
            {
                RaycastHit hit = hits[hitIndex];

                if (!FrameContainsCollider(frame, hit.collider))
                    continue;

                Vector3 candidateNormal = hit.normal.normalized;
                float normalAngle = requireNormalMatch ? Vector3.Angle(expectedNormal, candidateNormal) : 0f;

                bool angleAccepted = !requireNormalMatch || normalAngle <= maxNormalAngleDifference;

                if (!angleAccepted)
                    continue;

                float horizontalError = Vector3.ProjectOnPlane(hit.point - guess, Vector3.up).sqrMagnitude;

                float verticalError = Mathf.Abs(hit.point.y - guess.y);

                float score = horizontalError * 10f + verticalError + normalAngle * 0.1f;

                if (score >= bestScore)
                    continue;

                bestScore = score;
                bestNormalAngle = normalAngle;
                bestHit = hit;
                found = true;
            }
        }

        if (visualizeThisCall)
            MarkSlopeRayCaptured(frame.collider, purpose);

        if (!found)
        {
            if (writeRuntimeLog)
            {
                /*Debug.LogWarning(
                    $"[FRAME SAMPLE REJECTED] sep={sep} " +
                    $"guess={guess:F5} " +
                    $"expectedNormal={expectedNormal:F5} " +
                    $"angleLimit={maxNormalAngleDifference:F4} " +
                    $"reason=NoAcceptedSurface",
                    this);*/
            }

            return false;
        }

        point = bestHit.point;
        normal = bestHit.normal.normalized;

        if (visualizeThisCall)
        {
            SetGuessToMeasuredPointLine(frame.collider, purpose, guess, point);
        }

        return normal.sqrMagnitude > 0.999f;
    }

    void EnsureSlopeRayDebugRoot()
    {
        if (slopeRayDebugRoot)
            return;

        GameObject existing = GameObject.Find("SlopeRayDebugRoot");
        GameObject rootObject = existing ? existing : new GameObject("SlopeRayDebugRoot");

        slopeRayDebugRoot = rootObject.transform;
        slopeRayDebugRoot.SetParent(null, true);
        slopeRayDebugRoot.position = Vector3.zero;
        slopeRayDebugRoot.rotation = Quaternion.identity;
        slopeRayDebugRoot.localScale = Vector3.one;
    }

    bool ShouldCaptureSlopeRay(Collider slopeCollider, SurfaceSamplePurpose purpose)
    {
        if (!slopeCollider)
            return false;

        if (!freezeEntryExitRayLinesAfterCapture)
            return true;

        SlopeSectionRayVisual visual = GetOrCreateSlopeRayVisual(slopeCollider);

        return purpose == SurfaceSamplePurpose.Entry ? !visual.entryCaptured : !visual.exitCaptured;
    }

    SlopeSectionRayVisual GetOrCreateSlopeRayVisual(Collider slopeCollider)
    {
        EnsureSlopeRayDebugRoot();

        int key = keepVisualizationForEverySlope ? slopeCollider.GetInstanceID() : 0;

        if (slopeRayVisuals.TryGetValue(key, out SlopeSectionRayVisual visual) && visual != null && visual.root)
        {
            return visual;
        }

        visual = new SlopeSectionRayVisual();

        string safeName = slopeCollider ? slopeCollider.name : "UnknownSlope";

        GameObject sectionObject = new GameObject($"SlopeRaySection_{safeName}_{key}");

        sectionObject.transform.SetParent(slopeRayDebugRoot, false);
        sectionObject.transform.localPosition = Vector3.zero;
        sectionObject.transform.localRotation = Quaternion.identity;
        sectionObject.transform.localScale = Vector3.one;
        visual.root = sectionObject.transform;

        for (int i = 0; i < 5; i++)
        {
            visual.entryRays[i] = CreateSlopeRayLine(visual.root, $"EntryRay_{i}");

            visual.exitRays[i] = CreateSlopeRayLine(visual.root, $"ExitRay_{i}");
        }

        visual.entryGuessToHit = CreateSlopeRayLine(visual.root, "EntryGuess_To_MeasuredPoint");

        visual.exitGuessToHit = CreateSlopeRayLine(visual.root, "ExitGuess_To_MeasuredPoint");

        visual.measuredSection = CreateSlopeRayLine(visual.root, "MeasuredEntry_To_Exit");

        slopeRayVisuals[key] = visual;
        return visual;
    }

    LineRenderer CreateSlopeRayLine(Transform parent, string objectName)
    {
        GameObject lineObject = new GameObject(objectName);
        lineObject.transform.SetParent(parent, false);
        lineObject.transform.localPosition = Vector3.zero;
        lineObject.transform.localRotation = Quaternion.identity;
        lineObject.transform.localScale = Vector3.one;

        LineRenderer line = lineObject.AddComponent<LineRenderer>();
        line.useWorldSpace = true;
        line.positionCount = 2;
        line.startWidth = slopeRayLineWidth;
        line.endWidth = slopeRayLineWidth;
        line.numCapVertices = 2;
        line.enabled = false;

        if (slopeRayLineMaterial)
        {
            line.material = slopeRayLineMaterial;
        }
        else
        {
            if (!runtimeSlopeRayMaterial)
            {
                Shader shader = Shader.Find("Sprites/Default");
                if (!shader)
                    shader = Shader.Find("Universal Render Pipeline/Unlit");

                if (shader)
                    runtimeSlopeRayMaterial = new Material(shader);
            }

            if (runtimeSlopeRayMaterial)
                line.material = runtimeSlopeRayMaterial;
        }

        return line;
    }

    void SetEntryExitRayLine(Collider slopeCollider, SurfaceSamplePurpose purpose, int offsetIndex, Vector3 origin, Vector3 end, bool hitTargetSlope, bool visible)
    {
        if (!slopeCollider || (purpose != SurfaceSamplePurpose.Entry && purpose != SurfaceSamplePurpose.Exit))
        {
            return;
        }

        SlopeSectionRayVisual visual = GetOrCreateSlopeRayVisual(slopeCollider);

        LineRenderer[] targetLines = purpose == SurfaceSamplePurpose.Entry ? visual.entryRays : visual.exitRays;

        if (offsetIndex < 0 || offsetIndex >= targetLines.Length)
            return;

        LineRenderer line = targetLines[offsetIndex];
        if (!line)
            return;

        line.enabled = showEntryExitRayLines && visible;
        if (!line.enabled)
            return;

        Color lineColor = hitTargetSlope ? (purpose == SurfaceSamplePurpose.Entry ? entryRayHitColor : exitRayHitColor) : missedRayColor;

        ConfigureWorldLine(line, origin, end, lineColor, slopeRayLineWidth);
    }

    void SetGuessToMeasuredPointLine(Collider slopeCollider, SurfaceSamplePurpose purpose, Vector3 guess, Vector3 measuredPoint)
    {
        if (!slopeCollider)
            return;

        SlopeSectionRayVisual visual = GetOrCreateSlopeRayVisual(slopeCollider);
        LineRenderer line = purpose == SurfaceSamplePurpose.Entry ? visual.entryGuessToHit : visual.exitGuessToHit;

        Color color = purpose == SurfaceSamplePurpose.Entry ? entryRayHitColor : exitRayHitColor;

        ConfigureWorldLine(line, guess, measuredPoint, color, slopeRayLineWidth * 2f);

        if (purpose == SurfaceSamplePurpose.Entry)
        {
            visual.entryPoint = measuredPoint;
            visual.hasEntryPoint = true;
        }
        else
        {
            visual.exitPoint = measuredPoint;
            visual.hasExitPoint = true;
        }

        if (visual.hasEntryPoint && visual.hasExitPoint)
        {
            ConfigureWorldLine(visual.measuredSection, visual.entryPoint, visual.exitPoint, Color.white, slopeRayLineWidth * 2.5f);
        }
    }

    void ConfigureWorldLine(LineRenderer line, Vector3 start, Vector3 end, Color color, float width)
    {
        if (!line)
            return;

        line.enabled = showEntryExitRayLines;
        line.useWorldSpace = true;
        line.positionCount = 2;
        line.startWidth = width;
        line.endWidth = width;
        line.startColor = color;
        line.endColor = color;
        line.SetPosition(0, start);
        line.SetPosition(1, end);
    }

    void MarkSlopeRayCaptured(Collider slopeCollider, SurfaceSamplePurpose purpose)
    {
        if (!slopeCollider || !freezeEntryExitRayLinesAfterCapture)
            return;

        SlopeSectionRayVisual visual = GetOrCreateSlopeRayVisual(slopeCollider);

        if (purpose == SurfaceSamplePurpose.Entry)
            visual.entryCaptured = true;
        else if (purpose == SurfaceSamplePurpose.Exit)
            visual.exitCaptured = true;
    }

    [ContextMenu("Reset All Entry Exit Ray Lines")]
    public void ResetEntryExitRayLines()
    {
        foreach (KeyValuePair<int, SlopeSectionRayVisual> pair in slopeRayVisuals)
        {
            SlopeSectionRayVisual visual = pair.Value;
            if (visual != null && visual.root)
                Destroy(visual.root.gameObject);
        }

        slopeRayVisuals.Clear();
    }

    float CalculateProgress(SlopeFrame frame, Vector3 position)
    {
        float local = Vector3.Dot(position - frame.entryPoint, frame.axis);
        return Mathf.Clamp01(local / Mathf.Max(frame.projectedLength, 0.0001f));
    }

    void UpdateSlopeProgressObservation(bool grounded, GroundObservation observation, float dt)
    {
        crossedTargetThisFrame = false;
        estimatedTargetCrossingTime = 0f;

        bool valid = grounded && groundKind == GroundKind.Slope && slopeFrame.valid && observation.valid && observation.collider == slopeFrame.collider;

        if (!valid)
        {
            slopeProgressObservationValid = false;
            slopeProgressSide = SlopeProgressSide.Invalid;
            stableSlopeContactFrames = 0;
            readyForLimitCapture = false;
            return;
        }

        float current = currentSlopeProgressPercent;
        float error = current - targetSlopeProgressPercent;
        slopeProgressErrorPercent = error;
        slopeProgressSide = ClassifySlopeProgressSide(error);
        slopeProgressObservationValid = true;

        bool sameSession = hasPreviousSlopeProgressObservation && progressObservationCollider == observation.collider;

        if (!sameSession)
        {
            progressObservationCollider = observation.collider;
            hasPreviousSlopeProgressObservation = true;
            lastObservedSlopeProgressPercent = current;
            previousSlopeProgressPercent = current;
            slopeProgressDeltaPercent = 0f;
            previousSlopeProgressErrorPercent = error;
            stableSlopeContactFrames = 1;
            readyForLimitCapture = stableSlopeContactFrames >= requiredStableSlopeFrames;
            limitCaptureReadyLogged = false;

            if (writeRuntimeLog && logProgressTargetCrossing)
            {
                Debug.Log($"[SLOPE SESSION START] time={Time.fixedTime:F4}s " + $"collider={observation.collider.name} " +
                    $"progress={current:F3}% target={targetSlopeProgressPercent:F3}% " + $"error={error:F3}% side={slopeProgressSide} " + $"source={observation.source}", this);
            }

            return;
        }

        previousSlopeProgressPercent = lastObservedSlopeProgressPercent;
        slopeProgressDeltaPercent = current - lastObservedSlopeProgressPercent;

        bool plausibleForwardObservation = slopeProgressDeltaPercent >= -0.5f && Mathf.Abs(slopeProgressDeltaPercent) <= 25f;

        if (plausibleForwardObservation)
        {
            stableSlopeContactFrames++;
        }
        else
        {
            stableSlopeContactFrames = 1;
            limitCaptureReadyLogged = false;

            if (writeRuntimeLog && logProgressTargetCrossing)
            {
                Debug.LogWarning($"[PROGRESS OBSERVATION RESET] time={Time.fixedTime:F4}s " + $"collider={observation.collider.name} " + $"previous={previousSlopeProgressPercent:F3}% " +
                    $"current={current:F3}% " + $"delta={slopeProgressDeltaPercent:F3}%", this);
            }
        }

        readyForLimitCapture = stableSlopeContactFrames >= requiredStableSlopeFrames;

        if (readyForLimitCapture && !limitCaptureReadyLogged)
        {
            limitCaptureReadyLogged = true;

            if (writeRuntimeLog && logProgressTargetCrossing)
            {
                Debug.Log($"[LIMIT CAPTURE OBSERVATION READY] time={Time.fixedTime:F4}s " + $"collider={observation.collider.name} " + $"stableFrames={stableSlopeContactFrames} " +
                    $"progress={current:F3}% " + $"target={targetSlopeProgressPercent:F3}% " + $"error={error:F3}% side={slopeProgressSide} " + $"forcesChanged=false", this);
            }
        }

        bool crossedNegativeToPositive = previousSlopeProgressErrorPercent < 0f && error >= 0f;
        bool crossedPositiveToNegative = previousSlopeProgressErrorPercent > 0f && error <= 0f;

        crossedTargetThisFrame = crossedNegativeToPositive || crossedPositiveToNegative;

        if (crossedTargetThisFrame)
        {
            float denominator = error - previousSlopeProgressErrorPercent;
            float alpha = Mathf.Abs(denominator) > 0.000001f ? Mathf.Clamp01(-previousSlopeProgressErrorPercent / denominator) : 1f;

            estimatedTargetCrossingTime = Time.fixedTime - dt + alpha * dt;

            if (writeRuntimeLog && logProgressTargetCrossing)
            {
                string direction = crossedNegativeToPositive ? "NegativeToPositive" : "PositiveToNegative";

                Debug.Log($"[PROGRESS TARGET CROSSED] time={Time.fixedTime:F4}s " + $"estimatedCrossingTime={estimatedTargetCrossingTime:F6}s " +
                    $"collider={observation.collider.name} " + $"previous={previousSlopeProgressPercent:F3}% " + $"current={current:F3}% " + $"target={targetSlopeProgressPercent:F3}% " +
                    $"previousError={previousSlopeProgressErrorPercent:F3}% " + $"currentError={error:F3}% " + $"direction={direction} " + $"targetPhase={targetProgressPhase} " +
                    $"targetControlApplied={Mathf.Abs(targetProgressAppliedArtificialAcceleration) > 0.0001f}", this);
            }
        }

        lastObservedSlopeProgressPercent = current;
        previousSlopeProgressErrorPercent = error;
    }

    SlopeProgressSide ClassifySlopeProgressSide(float error)
    {
        const float exactTolerance = 0.01f;
        float nearTolerance = Mathf.Max(exactTolerance, progressNearZeroTolerancePercent);

        if (Mathf.Abs(error) <= exactTolerance)
            return SlopeProgressSide.Exact;
        if (error < 0f && Mathf.Abs(error) <= nearTolerance)
            return SlopeProgressSide.NegativeNearZero;
        if (error > 0f && error <= nearTolerance)
            return SlopeProgressSide.PositiveNearZero;
        return error < 0f ? SlopeProgressSide.NegativeSide : SlopeProgressSide.PositiveSide;
    }

    void ResetSlopeProgressObservation()
    {
        progressObservationCollider = null;
        hasPreviousSlopeProgressObservation = false;
        lastObservedSlopeProgressPercent = 0f;
        previousSlopeProgressPercent = 0f;
        slopeProgressDeltaPercent = 0f;
        slopeProgressErrorPercent = 0f;
        previousSlopeProgressErrorPercent = 0f;
        slopeProgressSide = SlopeProgressSide.Invalid;
        slopeProgressObservationValid = false;
        crossedTargetThisFrame = false;
        readyForLimitCapture = false;
        stableSlopeContactFrames = 0;
        estimatedTargetCrossingTime = 0f;
        limitCaptureReadyLogged = false;
    }

    void SolveAir(Vector3 move, float dt)
    {
        // 50%でLatchした後は空中方向制御も復活させず、重力だけに任せます。
        if (!useNaturalArtificialRelease || !naturalMotionReleased)
        {
            Vector3 horizontalVelocity = Vector3.ProjectOnPlane(rb.velocity, Vector3.up);
            Vector3 desired = Vector3.ProjectOnPlane(move, Vector3.up);

            if (desired.sqrMagnitude > 0.000001f)
                desired.Normalize();

            desired *= maxGroundSpeed;
            Vector3 acceleration = Vector3.ClampMagnitude((desired - horizontalVelocity) / dt, airAcceleration);

            rb.AddForce(acceleration, ForceMode.Acceleration);
        }

        currentTangentialAcceleration = 0f;
        currentNormalCorrection = 0f;
        currentAdaptivePredictedSpeed = 0f;
        currentAdaptiveDesiredBaseStickAcceleration = 0f;
        currentAdaptiveStickCapacity = 0f;
        currentAdaptiveOutwardDampingAcceleration = 0f;
        currentEffectiveBaseStickAcceleration = 0f;
        adaptiveStickSaturated = false;
        currentCriticalRatio = 0f;
        currentRequiredNormalAcceleration = 0f;
        currentAvailableNormalAcceleration = 0f;
        currentCurvature = 0f;
        currentRepresentativeCurvature = 0f;
        currentEffectiveMaximumDeceleration = maximumCriticalDeceleration;
        currentNormalDemandRatio = 0f;
        currentAllowedSpeed = maxGroundSpeed;
        currentLocalCriticalSpeed = maxGroundSpeed;
    }

    void UpdateCriticalDebugEvents(bool grounded)
    {
        if (!writeRuntimeLog || !logStateChanges)
        {
            CacheDebugState();
            return;
        }

        if (groundKind != previousLoggedGroundKind)
        {
            Debug.Log($"[CRITICAL GROUND STATE] time={Time.fixedTime:F4}s " + $"{previousLoggedGroundKind} -> {groundKind} " +
                $"surface={activeSlopeName} progress={currentSlopeProgressPercent:F3}%", this);
        }

        if (hasForwardSlope && !previousLoggedForwardSlope)
        {
            sessionSlopeDetectionCount++;

            Debug.Log($"[CRITICAL SLOPE DETECTED] time={Time.fixedTime:F4}s " + $"slope={activeSlopeName} " + $"entry={forwardSlopeFrame.entryPoint:F5} " +
                $"exit={forwardSlopeFrame.exitPoint:F5} " + $"length={forwardSlopeFrame.projectedLength:F6} " +
                $"colliderCount={(forwardSlopeFrame.sampleColliders != null ? forwardSlopeFrame.sampleColliders.Length : 0)} " + $"currentSpeed={currentTangentSpeed:F6}", this);
        }
        else if (!hasForwardSlope && previousLoggedForwardSlope)
        {
            Debug.Log($"[CRITICAL FORWARD SLOPE LOST] time={Time.fixedTime:F4}s " + $"previousSlope={previousLoggedSlopeName}", this);
        }

        if (!string.Equals(activeSlopeName, previousLoggedSlopeName))
        {
            Debug.Log($"[CRITICAL SLOPE CHANGED] time={Time.fixedTime:F4}s " + $"from={previousLoggedSlopeName} to={activeSlopeName} " +
                $"progress={currentSlopeProgressPercent:F3}%", this);
        }

        bool criticalControlActive = grounded && useCriticalBoundaryTracking && ((groundKind == GroundKind.Slope && slopeFrame.valid) ||
            (groundKind == GroundKind.Flat && hasForwardSlope && forwardSlopeFrame.valid));

        if (criticalControlActive && !previousLoggedCriticalControlActive)
        {
            Debug.Log($"[CRITICAL CONTROL START] time={Time.fixedTime:F4}s " + $"ground={groundKind} slope={activeSlopeName} " + $"progress={currentSlopeProgressPercent:F3}% " +
                $"controlProgress={controllingSampleProgressPercent:F3}% " + $"speed={currentTangentSpeed:F6} allowed={currentAllowedSpeed:F6}", this);
        }
        else if (!criticalControlActive && previousLoggedCriticalControlActive)
        {
            Debug.Log($"[CRITICAL CONTROL END] time={Time.fixedTime:F4}s " + $"ground={groundKind} slope={activeSlopeName} " + $"lastRatio={currentCriticalRatio:F6}", this);
        }

        bool ratioIsValid = grounded && groundKind == GroundKind.Slope && currentCurvature > minimumCurvature && currentAvailableNormalAcceleration > 0.0001f;

        bool inTolerance = ratioIsValid && Mathf.Abs(currentCriticalRatio - targetCriticalRatio) <= criticalRatioTolerance;

        if (inTolerance && !previousLoggedInCriticalTolerance)
        {
            Debug.Log($"[CRITICAL ENTER] time={Time.fixedTime:F4}s " + $"progress={currentSlopeProgressPercent:F3}% " +
                $"ratio={currentCriticalRatio:F6} target={targetCriticalRatio:F6} " + $"error={Mathf.Abs(currentCriticalRatio - targetCriticalRatio):F6} " +
                $"speed={currentTangentSpeed:F6} " + $"localCritical={currentLocalCriticalSpeed:F6} " + $"curvature={currentCurvature:F8}", this);
        }
        else if (!inTolerance && previousLoggedInCriticalTolerance)
        {
            Debug.Log($"[CRITICAL EXIT] time={Time.fixedTime:F4}s " + $"progress={currentSlopeProgressPercent:F3}% " +
                $"ratio={currentCriticalRatio:F6} target={targetCriticalRatio:F6} " + $"error={Mathf.Abs(currentCriticalRatio - targetCriticalRatio):F6} " +
                $"held={criticalMaintainedSeconds:F4}s", this);
        }

        if (logCriticalSuccess && criticalStateMaintained && !previousLoggedCriticalMaintained)
        {
            sessionCriticalSuccessCount++;

            Debug.Log($"[CRITICAL SUCCESS] time={Time.fixedTime:F4}s " + $"slope={activeSlopeName} " + $"progress={currentSlopeProgressPercent:F3}% " +
                $"controlProgress={controllingSampleProgressPercent:F3}% " + $"ratio={currentCriticalRatio:F6} target={targetCriticalRatio:F6} " +
                $"error={Mathf.Abs(currentCriticalRatio - targetCriticalRatio):F6} " + $"held={criticalMaintainedSeconds:F4}s " +
                $"speed={currentTangentSpeed:F6} allowed={currentAllowedSpeed:F6} " + $"localCritical={currentLocalCriticalSpeed:F6} " + $"curvature={currentCurvature:F8} " +
                $"available={currentAvailableNormalAcceleration:F6} " + $"required={currentRequiredNormalAcceleration:F6}", this);
        }

        if (logCriticalSuccess && !criticalStateMaintained && previousLoggedCriticalMaintained)
        {
            Debug.LogWarning($"[CRITICAL MAINTENANCE LOST] time={Time.fixedTime:F4}s " + $"slope={activeSlopeName} " + $"progress={currentSlopeProgressPercent:F3}% " +
                $"ratio={currentCriticalRatio:F6} target={targetCriticalRatio:F6} " + $"error={Mathf.Abs(currentCriticalRatio - targetCriticalRatio):F6} " +
                $"speed={currentTangentSpeed:F6} allowed={currentAllowedSpeed:F6}", this);
        }

        if (logCriticalRisk && ratioIsValid && currentCriticalRatio >= criticalRiskRatio && Time.fixedTime - lastCriticalRiskLogTime >= criticalRiskLogInterval)
        {
            lastCriticalRiskLogTime = Time.fixedTime;

            Debug.LogWarning($"[CRITICAL RISK] time={Time.fixedTime:F4}s " + $"slope={activeSlopeName} " + $"progress={currentSlopeProgressPercent:F3}% " +
                $"ratio={currentCriticalRatio:F6} riskThreshold={criticalRiskRatio:F6} " + $"speed={currentTangentSpeed:F6} allowed={currentAllowedSpeed:F6} " +
                $"localCritical={currentLocalCriticalSpeed:F6} " + $"curvature={currentCurvature:F8} " + $"available={currentAvailableNormalAcceleration:F6} " +
                $"required={currentRequiredNormalAcceleration:F6}", this);
        }

        CacheDebugState();
    }

    void CacheDebugState()
    {
        previousLoggedGroundKind = groundKind;
        previousLoggedForwardSlope = hasForwardSlope;
        previousLoggedCriticalControlActive = useCriticalBoundaryTracking && ((groundKind == GroundKind.Slope && slopeFrame.valid) ||
            (groundKind == GroundKind.Flat && hasForwardSlope && forwardSlopeFrame.valid));
        previousLoggedInCriticalTolerance = groundKind == GroundKind.Slope && currentCurvature > minimumCurvature && currentAvailableNormalAcceleration > 0.0001f &&
            Mathf.Abs(currentCriticalRatio - targetCriticalRatio) <= criticalRatioTolerance;
        previousLoggedCriticalMaintained = criticalStateMaintained;
        previousLoggedSlopeName = activeSlopeName;
    }

    void UpdateSessionStatistics()
    {
        if (groundKind != GroundKind.Slope || currentCurvature <= minimumCurvature || currentAvailableNormalAcceleration <= 0.0001f)
        {
            return;
        }

        sessionMaximumCriticalRatio = Mathf.Max(sessionMaximumCriticalRatio, currentCriticalRatio);

        sessionMinimumCriticalError = Mathf.Min(sessionMinimumCriticalError, Mathf.Abs(currentCriticalRatio - targetCriticalRatio));

        sessionLongestMaintainedSeconds = Mathf.Max(sessionLongestMaintainedSeconds, criticalMaintainedSeconds);
    }

    string ClassifyAirTransition(float ratio)
    {
        if (ratio >= criticalRiskRatio)
            return "MODEL_PREDICTED_LIMIT_OR_OVER";

        if (ratio > 0f && Mathf.Abs(ratio - targetCriticalRatio) <= criticalRatioTolerance)
        {
            return "AIRBORNE_WHILE_NEAR_TARGET_CRITICAL";
        }

        if (ratio <= 0f || currentCurvature <= minimumCurvature)
            return "MODEL_INVALID_OR_NO_CURVATURE";

        return "AIRBORNE_BELOW_PREDICTED_LIMIT";
    }

    void OnDestroy()
    {
        if (runtimeSlopeRayMaterial)
            Destroy(runtimeSlopeRayMaterial);

        if (slopeRayDebugRoot)
            Destroy(slopeRayDebugRoot.gameObject);
    }

    void OnDisable()
    {
        if (!writeRuntimeLog || !logSessionSummary)
            return;

        string minimumErrorText = float.IsPositiveInfinity(sessionMinimumCriticalError) ? "N/A" : sessionMinimumCriticalError.ToString("F6");

        Debug.Log($"[CRITICAL SESSION SUMMARY] successes={sessionCriticalSuccessCount} " + $"groundToAir={sessionGroundToAirCount} " +
            $"slopeDetections={sessionSlopeDetectionCount} " + $"maxRatio={sessionMaximumCriticalRatio:F6} " + $"minTargetError={minimumErrorText} " +
            $"longestMaintained={sessionLongestMaintainedSeconds:F4}s " + $"targetRatio={targetCriticalRatio:F6} " + $"tolerance={criticalRatioTolerance:F6} " +
            $"requiredHold={criticalHoldSeconds:F4}s", this);
    }

    void WriteLogIfNeeded()
    {
        if (!writeRuntimeLog)
            return;

        fixedFrameCounter++;
        if (fixedFrameCounter % Mathf.Max(1, logEveryFixedFrames) != 0)
            return;

        Debug.Log($"[CRITICAL TRACE] time={Time.fixedTime:F4}s " + $"ground={groundKind} slope={activeSlopeName} forwardSlope={hasForwardSlope} " +
            $"progress={currentSlopeProgressPercent:F3}% " + $"controlProgress={controllingSampleProgressPercent:F3}% " +
            $"speed={currentTangentSpeed:F6} allowed={currentAllowedSpeed:F6} " + $"localCritical={currentLocalCriticalSpeed:F6} " + $"curvature={currentCurvature:F8} " +
            $"sectionCurvature={currentRepresentativeCurvature:F8} " + $"effectiveDeceleration={currentEffectiveMaximumDeceleration:F6} " +
            $"normalDemandRatio={currentNormalDemandRatio:F6} " + $"available={currentAvailableNormalAcceleration:F6} " + $"required={currentRequiredNormalAcceleration:F6} " +
            $"ratio={currentCriticalRatio:F6} targetRatio={targetCriticalRatio:F6} " + $"tangentAcceleration={currentTangentialAcceleration:F6} " +
            $"normalCorrection={currentNormalCorrection:F6} " + $"ratioError={Mathf.Abs(currentCriticalRatio - targetCriticalRatio):F6} " +
            $"modelValid={(groundKind == GroundKind.Slope && currentCurvature > minimumCurvature && currentAvailableNormalAcceleration > 0.0001f)} " +
            $"maintained={criticalStateMaintained} hold={criticalMaintainedSeconds:F4}s " + $"supportSource={currentGroundObservationSource} " +
            $"supportCollider={currentGroundColliderName} " + $"targetProgress={targetSlopeProgressPercent:F3}% " + $"progressError={slopeProgressErrorPercent:F3}% " +
            $"progressSide={slopeProgressSide} " + $"progressDelta={slopeProgressDeltaPercent:F3}% " + $"stableSlopeFrames={stableSlopeContactFrames} " +
            $"observationValid={slopeProgressObservationValid} " + $"crossedTarget={crossedTargetThisFrame} " + $"readyForLimitCapture={readyForLimitCapture} " +
            $"targetPhase={targetProgressPhase} " + $"targetPlanValid={targetProgressPlanValid} " + $"capturedTargetSpeed={capturedTargetTangentSpeed:F6} " +
            $"targetRemainingDistance={targetProgressRemainingDistance:F6} " + $"targetRequiredNetAcceleration={targetProgressRequiredNetAcceleration:F6} " +
            $"targetGravityAcceleration={targetProgressGravityAcceleration:F6} " + $"targetArtificialAcceleration={targetProgressAppliedArtificialAcceleration:F6} " +
            $"naturalReleasePlanValid={naturalReleasePlanValid} " + $"naturalReleaseStart={capturedReleaseStartProgressPercent:F3}% " +
            $"naturalReleaseEnd={capturedNaturalReleaseProgressPercent:F3}% " + $"naturalReleaseWeight={currentArtificialReleaseWeight:F6} " +
            $"cumulativeRelease={currentCumulativeReleaseRatio:F6} " + $"adaptivePredictedSpeed={currentAdaptivePredictedSpeed:F6} " +
            $"adaptiveDesiredStick={currentAdaptiveDesiredBaseStickAcceleration:F6} " + $"adaptiveStickCapacity={currentAdaptiveStickCapacity:F6} " +
            $"adaptiveOutwardDamping={currentAdaptiveOutwardDampingAcceleration:F6} " + $"effectiveBaseStick={currentEffectiveBaseStickAcceleration:F6} " +
            $"adaptiveStickSaturated={adaptiveStickSaturated} " + $"naturalMotionReleased={naturalMotionReleased}", this);
    }

    void OnDrawGizmosSelected()
    {
        if (!drawDebugGizmos)
            return;

        if (slopeFrame.valid)
        {
            Gizmos.DrawWireSphere(slopeFrame.entryPoint, 0.16f);
            Gizmos.DrawWireSphere(slopeFrame.exitPoint, 0.16f);
            Gizmos.DrawLine(slopeFrame.entryPoint, slopeFrame.exitPoint);
        }

        if (forwardSlopeFrame.valid)
        {
            Gizmos.DrawWireSphere(forwardSlopeFrame.entryPoint, 0.12f);
            Gizmos.DrawWireSphere(forwardSlopeFrame.exitPoint, 0.12f);
            Gizmos.DrawLine(forwardSlopeFrame.entryPoint, forwardSlopeFrame.exitPoint);
        }

        if (controllingSamplePoint != Vector3.zero)
            Gizmos.DrawWireSphere(controllingSamplePoint, 0.22f);

        if (slopeFrame.valid)
        {
            Vector3 targetPoint = Vector3.Lerp(slopeFrame.entryPoint, slopeFrame.exitPoint, Mathf.Clamp01(targetSlopeProgressPercent / 100f));
            Gizmos.DrawWireSphere(targetPoint, 0.25f);
        }
    }

    static Vector3 NormalizeFlat(Vector3 value, Vector3 fallback)
    {
        value = Vector3.ProjectOnPlane(value, Vector3.up);
        if (value.sqrMagnitude < 0.000001f)
            value = Vector3.ProjectOnPlane(fallback, Vector3.up);
        return value.sqrMagnitude > 0.000001f ? value.normalized : Vector3.forward;
    }

    static Vector3[] GetBoundsCorners(Bounds bounds)
    {
        Vector3 min = bounds.min;
        Vector3 max = bounds.max;
        return new[]
        {
            new Vector3(min.x, min.y, min.z),
            new Vector3(min.x, min.y, max.z),
            new Vector3(min.x, max.y, min.z),
            new Vector3(min.x, max.y, max.z),
            new Vector3(max.x, min.y, min.z),
            new Vector3(max.x, min.y, max.z),
            new Vector3(max.x, max.y, min.z),
            new Vector3(max.x, max.y, max.z)
        };
    }
}
