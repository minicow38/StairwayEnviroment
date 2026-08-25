using UnityEngine;
using System.Collections;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody), typeof(SphereCollider), typeof(NearestKnotDetector))]
public sealed class SlopeStickCore : MonoBehaviour
{
    const float Eps = 0.000001f;

    [SerializeField] NearestKnotDetector knotDetector;
    [SerializeField] LayerMask groundMask = ~0;

    [Header("Travel")]
    [SerializeField] Vector3 travelDirection = Vector3.forward;
    [Min(0f)] [SerializeField] float maxGroundSpeed = 24f;
    [Range(0f, 100f)] [SerializeField] float targetSlopeProgressPercent = 60f;

    [Header("Ground Probe / Grace")]
    [Min(.01f)] [SerializeField] float probeRadius = .475f;
    [Min(.01f)] [SerializeField] float probeDistance = .85f;
    [Range(1f, 89f)] [SerializeField] float maxSlopeAngle = 75f;
    [Min(0f)] [SerializeField] float supportGraceSeconds = .12f;
    [Min(0f)] [SerializeField] float supportGraceMaxGuideDistance = 1.75f;
    [Min(0f)] [SerializeField] float maxGraceOutwardSpeed = 2f;

    [Header("Longitudinal")]
    [Min(0f)] [SerializeField] float groundAcceleration = 35f;
    [Min(0f)] [SerializeField] float maxDeceleration = 80f;
    [Min(.01f)] [SerializeField] float responseInverse = 8.333333f;
    [Min(.01f)] [SerializeField] float lateralResponseInverse = 8.333333f;
    [Min(0f)] [SerializeField] float accelerationJerk = 600f;
    [Min(.01f)] [SerializeField] float targetMinDistance = .27f;
    [Min(0f)] [SerializeField] float targetAccelerationLimit = 120f;
    [Range(0f, 1f)] [SerializeField] float postTargetBlendWidth = .05f;
    [Range(0f, 1f)] [SerializeField] float postTargetGravityRatio = .50f;

    [Header("Stick")]
    [Min(0f)] [SerializeField] float flatStick = 24.6f;
    [Min(0f)] [SerializeField] float maxStick = 1000f;
    [Min(0f)] [SerializeField] float stickPredictionSeconds = .06f;
    [Range(.01f, 1f)] [SerializeField] float criticalRatio = .98f;
    [Min(1f)] [SerializeField] float stickSafety = 1.08f;
    [Min(0f)] [SerializeField] float outwardResponseInverse = 15f;
    [Min(0f)] [SerializeField] float stickJerk = 3000f;

    [Header("Release")]
    [Range(0f, .25f)] [SerializeField] float releaseHold = .02f;
    [Range(0f, 1f)] [SerializeField] float naturalReleaseEnd = .90f;

    [Header("Debug")]
    [SerializeField] bool logCore;

    public bool Started { get; private set; }
    public bool IsGrounded { get; private set; }
    public bool IsOnFlat { get; private set; }
    public bool IsOnSlope { get; private set; }
    public bool IsAir => Started && !IsGrounded;
    public bool BallVisualIsOnFlat => IsOnFlat;
    public bool BallVisualIsOnSlope => IsOnSlope;
    public bool BallVisualIsAir => IsAir;

    public NearestKnotDetector.GuideFrame CurrentGuide { get; private set; }
    public Vector3 PathTangent => CurrentGuide.valid ? CurrentGuide.tangent : direction;
    public Vector3 PathNormal => CurrentGuide.valid ? CurrentGuide.normal : Vector3.up;
    public float PathCurvature => CurrentGuide.valid ? CurrentGuide.curvature : 0f;
    public float SlopeProgress01 => CurrentGuide.valid ? CurrentGuide.sectionProgress01 : 0f;

    // -----------------------------------------------------------------
    // BallVisualSlopeDrive compatibility telemetry
    // -----------------------------------------------------------------
    // すべて読み取り専用。BallVisual側からCoreの状態を書き換えない。
    //
    // 現行BallVisualSlopeDriveが参照しているAPI名をそのまま公開する。
    // 法線・接線はGuideの理想値ではなく、CoreがそのFixedUpdateで
    // 実際のForce計算に使ったSurfaceを優先して返す。
    public Vector3 BallVisualSurfaceNormal =>
        ballVisualSurfaceNormal.sqrMagnitude > Eps
            ? ballVisualSurfaceNormal
            : PathNormal;

    public Vector3 BallVisualSlopeTangent =>
        ballVisualSlopeTangent.sqrMagnitude > Eps
            ? ballVisualSlopeTangent
            : PathTangent;

    // 実際にCoreのForceへ使われたStick / 接線加速度。
    public float BallVisualAppliedStickAcceleration => stickState;
    public float BallVisualAppliedTangentialAcceleration => driveState;

    // 1 = 人工制御が有効、0 = Release完了。
    // BallVisualSlopeDriveのBallVisualReleaseWeightと同じ意味。
    public float BallVisualReleaseWeight => releaseWeightState;

    // Target地点を0%とする進捗誤差。
    // Target=60%, Progress=55% -> -5%
    // Target=60%, Progress=60% ->  0%
    // Target=60%, Progress=70% -> +10%
    public float slopeProgressErrorPercent =>
        (SlopeProgress01 - TargetProgress) * 100f;

    // BallVisual側でTargetとProgressを別々に観測したい場合の公開値。
    public float TargetSlopeProgress01 => TargetProgress;
    public float TargetSlopeProgressPercent => targetSlopeProgressPercent;

    // 現行BallVisualSlopeDriveの旧SlopeStick3D互換API。
    // GroundKind enumは既存プロジェクト側の定義を再利用する。
    public GroundKind groundKind =>
        IsOnSlope
            ? GroundKind.Slope
            : IsOnFlat
                ? GroundKind.Flat
                : GroundKind.Air;

    Rigidbody rb;
    Vector3 direction;
    bool wasOnSlope;
    float graceTimer;
    float driveState;
    float stickState;

    // BallVisualSlopeDriveへ公開する観測値。
    float releaseWeightState;
    Vector3 ballVisualSurfaceNormal = Vector3.up;
    Vector3 ballVisualSlopeTangent = Vector3.forward;

    float TargetProgress => Mathf.Clamp01(targetSlopeProgressPercent * .01f);
    float ReleaseStart => Mathf.Clamp01(TargetProgress + releaseHold);
    float ReleaseEnd => Mathf.Max(ReleaseStart, naturalReleaseEnd);

    struct Surface
    {
        public bool valid;
        public Vector3 tangent;
        public Vector3 side;
        public Vector3 normal;
        public float tangentSpeed;
        public float lateralSpeed;
        public float outwardSpeed;
        public float gravityAlong;
        public float gravitySupport;
    }

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (!knotDetector) knotDetector = GetComponent<NearestKnotDetector>();
        direction = NormalizeFlat(travelDirection, transform.forward);

        ballVisualSurfaceNormal = Vector3.up;
        ballVisualSlopeTangent = direction;
        releaseWeightState = 0f;

       /* Debug.Log(
            $"[CORE PARAM] " +
            $"maxSpeed={maxGroundSpeed:F3} " +
            $"targetProgress={targetSlopeProgressPercent:F1} " +
            $"groundAccel={groundAcceleration:F3} " +
            $"response={responseInverse:F3} " +
            $"accelJerk={accelerationJerk:F3}");*/
    }

    IEnumerator Start()
    {
        yield return new WaitForSeconds(.8f);
        GameObject arc = GameObject.Find("StairWay2_0_Physics");
        rb.position = new Vector3(
            arc.transform.position.x,
            arc.transform.position.y + 2f,
            arc.transform.position.z);
    }

    void FixedUpdate()
    {
        bool measuredGrounded = ProbeGround(out Vector3 contactNormal);
        IsGrounded = measuredGrounded;

        // 初回着地までは完全にUnity重力だけ。
        if (!Started)
        {
            IsOnFlat = IsOnSlope = false;
            if (!measuredGrounded) return;
            Started = true;
        }

        CurrentGuide = knotDetector ? knotDetector.Evaluate(rb.position) : default;
        if (!CurrentGuide.valid)
        {
            LoseSupport();
            return;
        }

        graceTimer = measuredGrounded
            ? supportGraceSeconds
            : Mathf.Max(0f, graceTimer - Time.fixedDeltaTime);

        bool grace = CanUseGrace(measuredGrounded, CurrentGuide);
        if (!measuredGrounded && !grace)
        {
            if (logCore)
                Debug.Log(
                    $"[CORE SUPPORT LOST] outward={GuideOutward(CurrentGuide):F3} " +
                    $"guideDist={CurrentGuide.distanceToGuide:F3}");

            LoseSupport();
            return;
        }

        IsGrounded = true;
        IsOnSlope = CurrentGuide.isSlope;
        IsOnFlat = !CurrentGuide.isSlope;

        // 過去ログでoutwardを10～16m/s級から0へ落としているため、これは残す。
        if (CurrentGuide.isSlope && !wasOnSlope)
        {
            driveState = 0f; // 入口で旧Flat加速状態だけ捨てる。Stickは持ち越す。
            TransportToSlope(CurrentGuide.normal);
        }

        Vector3 supportNormal = measuredGrounded ? contactNormal : CurrentGuide.normal;
        Surface s = BuildSurface(supportNormal, CurrentGuide.tangent);
        if (!s.valid) return;

        // このFixedUpdateでCoreが実際に制御へ使用した基底を保存する。
        // BallVisualはこの値をVisual座標へMapして再利用できる。
        ballVisualSurfaceNormal = s.normal;
        ballVisualSlopeTangent = s.tangent;

        float release = CurrentGuide.isSlope
            ? 1f - SmoothRange01(CurrentGuide.sectionProgress01, ReleaseStart, ReleaseEnd)
            : 1f;

        releaseWeightState = Mathf.Clamp01(release);

        float desiredA = DesiredDrive(s, CurrentGuide, grace) * release;
        driveState = MoveForce(driveState, desiredA, accelerationJerk);

        // 増強DetectorのentryCurvatureをそのまま入口25%だけ使う。
        float curvature = CurrentGuide.isSlope
            ? Mathf.Max(0f, CurrentGuide.entryCurvature) *
              (1f - SmoothRange01(CurrentGuide.sectionProgress01, 0f, .25f))
            : 0f;
        float stick = DesiredStick(s, driveState, curvature);

        if (grace)
            stick = Mathf.Max(
                stick,
                Mathf.Min(maxStick, flatStick + s.outwardSpeed * outwardResponseInverse));

        stick *= release;
        stickState = MoveForce(stickState, stick, stickJerk);

        Vector3 acceleration =
            s.tangent * driveState
            - s.side * (s.lateralSpeed * lateralResponseInverse)
            - s.normal * stickState;

        rb.AddForce(acceleration, ForceMode.Acceleration);

        wasOnSlope = CurrentGuide.isSlope;

        if (logCore)
            Debug.Log(
                $"[CORE] speed={s.tangentSpeed:F3} total={rb.velocity.magnitude:F3} " +
                $"slope={CurrentGuide.isSlope} progress={CurrentGuide.sectionProgress01:F3} " +
                $"entryK={CurrentGuide.entryCurvature:F4} usedK={curvature:F4} " +
                $"drive={driveState:F3} stick={stickState:F3} grace={grace}");
        

        Debug.Log(
            $"[CORE DRIVE] " +
            $"speed={s.tangentSpeed:F3} " +
            $"max={maxGroundSpeed:F3} " +
            $"desiredA={desiredA:F3} " +
            $"driveState={driveState:F3} " +
            $"progress={CurrentGuide.sectionProgress01:F3}");
    }

    bool ProbeGround(out Vector3 normal)
    {
        normal = Vector3.up;
        Vector3 origin = rb.worldCenterOfMass + Vector3.up * .05f;

        if (!Physics.SphereCast(
                origin,
                probeRadius,
                Vector3.down,
                out RaycastHit hit,
                probeDistance,
                groundMask,
                QueryTriggerInteraction.Ignore) ||
            Vector3.Angle(hit.normal, Vector3.up) > maxSlopeAngle)
            return false;

        normal = hit.normal.normalized;
        return true;
    }

    bool CanUseGrace(bool measuredGrounded, NearestKnotDetector.GuideFrame g)
    {
        if (measuredGrounded ||
            graceTimer <= 0f ||
            g.distanceToGuide > supportGraceMaxGuideDistance)
            return false;

        // Release後は支持を延命しない。
        if (g.isSlope && g.sectionProgress01 >= ReleaseEnd)
            return false;

        // Detectorが次Slopeを既に返すので旧TransitionKind判定は不要。
        if (!g.isSlope && !g.nextIsSlope)
            return false;

        return GuideOutward(g) <= maxGraceOutwardSpeed;
    }

    float GuideOutward(NearestKnotDetector.GuideFrame g)
    {
        Vector3 n = g.normal.sqrMagnitude > Eps ? g.normal.normalized : Vector3.up;
        return Mathf.Max(0f, Vector3.Dot(rb.velocity, n));
    }

    void LoseSupport()
    {
        IsGrounded = IsOnFlat = IsOnSlope = false;

        // Graceで救えなかった離脱だけを完全Airとして扱う。
        // 過去ログではAirAccelerationがSupport復帰に寄与しなかったため、追加Forceは行わない。
        wasOnSlope = false;
        driveState = 0f;
        stickState = 0f;

        // Airでは人工制御を0としてBallVisualへ公開する。
        releaseWeightState = 0f;
    }

    Surface BuildSurface(Vector3 normalInput, Vector3 preferredTangent)
    {
        Vector3 normal = normalInput.sqrMagnitude > Eps
            ? normalInput.normalized
            : Vector3.up;

        Vector3 tangent = Vector3.ProjectOnPlane(preferredTangent, normal);
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

        Vector3 v = rb.velocity;
        return new Surface
        {
            valid = true,
            tangent = tangent,
            side = side,
            normal = normal,
            tangentSpeed = Vector3.Dot(v, tangent),
            lateralSpeed = Vector3.Dot(v, side),
            outwardSpeed = Mathf.Max(0f, Vector3.Dot(v, normal)),
            gravityAlong = Vector3.Dot(Physics.gravity, tangent),
            gravitySupport = Mathf.Max(0f, Vector3.Dot(Physics.gravity, -normal))
        };
    }

    float DesiredDrive(
        Surface s,
        NearestKnotDetector.GuideFrame g,
        bool grace)
    {
        float target;

        if (!g.isSlope)
        {
            target = Mathf.Clamp(
                (maxGroundSpeed - s.tangentSpeed) * responseInverse,
                -maxDeceleration,
                groundAcceleration);
        }
        else if (g.sectionProgress01 < TargetProgress)
        {
            float remaining = Mathf.Max(
                targetMinDistance,
                (TargetProgress - g.sectionProgress01) *
                Mathf.Max(targetMinDistance, g.sectionLength));

            target = Mathf.Clamp(
                (maxGroundSpeed * maxGroundSpeed - s.tangentSpeed * s.tangentSpeed) /
                (2f * Mathf.Max(.0001f, remaining)) - s.gravityAlong,
                -targetAccelerationLimit,
                targetAccelerationLimit);
        }
        else
        {
            target = Mathf.Clamp(
                (maxGroundSpeed - s.tangentSpeed) * responseInverse,
                -maxDeceleration,
                groundAcceleration);

            float blend = SmoothRange01(
                g.sectionProgress01,
                TargetProgress,
                Mathf.Clamp01(TargetProgress + postTargetBlendWidth));

            target -= Mathf.Max(0f, s.gravityAlong) * postTargetGravityRatio * blend;
            target = Mathf.Min(0f, target);
        }

        if (grace)
            target = Mathf.Min(0f, target);

        return Mathf.Max(-maxDeceleration, target);
    }

    float DesiredStick(Surface s, float tangentialA, float curvature)
    {
        if (curvature <= Eps)
            return flatStick;

        float forecast = Mathf.Max(
            0f,
            s.tangentSpeed + Mathf.Max(0f, tangentialA) * stickPredictionSeconds);

        float required =
            forecast * forecast * curvature / Mathf.Max(.01f, criticalRatio)
            - s.gravitySupport;

        return Mathf.Clamp(
            Mathf.Max(flatStick, required * stickSafety
                + s.outwardSpeed * outwardResponseInverse),
            0f,
            maxStick);
    }

    void TransportToSlope(Vector3 normal)
    {
        if (normal.sqrMagnitude <= Eps)
            return;

        normal.Normalize();
        Vector3 before = rb.velocity;
        if (before.sqrMagnitude <= Eps)
            return;

        float outwardBefore = Mathf.Max(0f, Vector3.Dot(before, normal));
        if (outwardBefore <= .01f)
            return;

        Vector3 projected = Vector3.ProjectOnPlane(before, normal);
        if (projected.sqrMagnitude <= Eps)
            return;

        Vector3 after = projected.normalized * before.magnitude;
        rb.AddForce(after - before, ForceMode.VelocityChange);

        if (logCore)
            Debug.Log(
                $"[ENTRY TRANSPORT] speed={before.magnitude:F3} " +
                $"outwardBefore={outwardBefore:F3} " +
                $"outwardAfter={Mathf.Max(0f, Vector3.Dot(after, normal)):F3}");
    }

    public void SetTravelDirection(Vector3 worldDirection) =>
        direction = NormalizeFlat(worldDirection, direction);

    static float MoveForce(float current, float target, float jerk) =>
        Mathf.MoveTowards(current, target, jerk * Time.fixedDeltaTime);

    static Vector3 NormalizeFlat(Vector3 value, Vector3 fallback)
    {
        Vector3 flat = Vector3.ProjectOnPlane(value, Vector3.up);
        if (flat.sqrMagnitude <= Eps)
            flat = Vector3.ProjectOnPlane(fallback, Vector3.up);

        return (flat.sqrMagnitude <= Eps ? Vector3.forward : flat).normalized;
    }

    static float SmoothRange01(float value, float start, float end) =>
        end <= start + Eps
            ? (value >= end ? 1f : 0f)
            : SmootherStep01(Mathf.InverseLerp(start, end, value));

    static float SmootherStep01(float value)
    {
        float t = Mathf.Clamp01(value);
        return t * t * t * (t * (t * 6f - 15f) + 10f);
    }
}