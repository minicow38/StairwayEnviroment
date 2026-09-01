using UnityEngine;
using UnityEngine.Serialization;
using System.Collections;
using System.Collections.Generic;
// using DG.Tweening; // No-stage-rotation test: DOTween stage/player rotation is disabled.
using Sirenix.OdinInspector;
using Unity.VisualScripting;
using System.Text.RegularExpressions;
using Unity.Mathematics;
public enum GroundKind
{
    Air,
    Flat,
    Slope
}
[Searchable]
[RequireComponent(typeof(Rigidbody), typeof(SphereCollider), typeof(ReliableBaselineSampler))]
public sealed class SlopeStick3D : MonoBehaviour
{
    // const string ImplementationVersion = "UnifiedInvariantController-v15.1-SafeAlgebraicCleanup-2026-07-25";

    // Flat -> Slope の局所Sequenceだけに適用する固定条件です。
    // Inspector項目を増やさず、過去ログで再現した1 FixedUpdate欠落と入口25%未満へ限定します。
    const int LocalSlopeEntryBridgeMaximumMissFrames = 1;
    const float LocalSlopeEntrySettlingEndProgress = 0.25f;
    const float LocalSlopeEntrySpeedDeadband = 0.05f;

    

    enum GroundObservationSource
    {
        None,
        CollisionContact,
        SphereCast,
        SlopeEntryBridge
    }

    enum LocalFlatToSlopeSequencePhase
    {
        Inactive,
        FlatApproach,
        SlopeEntrySettling,
        NormalSlope
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
        public Vector3 axis;
        public Vector3 entryPoint;
        public Vector3 exitPoint;
        public float projectedLength;
        public float representativeCurvature;
    }

    struct SurfaceSample
    {
        public bool valid;
        public Vector3 point;
        public Vector3 normal;
        public Vector3 tangent;
        public float curvature;
        public float gravitySupport;
        public ContactInvariant contact;
        public float distanceAhead;
        public float effectiveMaximumDeceleration;
    }

    struct GroundObservation
    {
        public bool valid;
        public Collider collider;
        public Vector3 point;
        public Vector3 normal;
        public GroundObservationSource source;
    }

    enum ArtificialControlPhase
    {
        Uncontrolled,
        Controlled,
        Releasing,
        Released
    }

    /// <summary>
    /// 面上の局所直交基底です。
    /// tangent + side - normal の3成分を一度だけ構成し、
    /// Drive / Side / Stickを同じ座標系で扱います。
    /// </summary>
    struct SurfaceBasis
    {
        public bool valid;
        public Vector3 tangent;
        public Vector3 side;
        public Vector3 normal;

        public static SurfaceBasis Create(Vector3 normalInput, Vector3 preferredTangent, Vector3 fallbackTangent)
        {
            SurfaceBasis basis = default;
            Vector3 normal = normalInput.sqrMagnitude > 0.000001f ? normalInput.normalized : Vector3.up;
            Vector3 tangent = Vector3.ProjectOnPlane(preferredTangent, normal);

            if (tangent.sqrMagnitude < 0.000001f)
                tangent = Vector3.ProjectOnPlane(fallbackTangent, normal);
            if (tangent.sqrMagnitude < 0.000001f)
                return basis;

            tangent.Normalize();

            Vector3 side = Vector3.Cross(normal, tangent);
            if (side.sqrMagnitude < 0.000001f)
                return basis;
            side.Normalize();

            // 数値誤差を除いて相互直交させます。
            tangent = Vector3.Cross(side, normal).normalized;

            basis.valid = true;
            basis.tangent = tangent;
            basis.side = side;
            basis.normal = normal;
            return basis;
        }

        public Vector3 Compose(float tangentAcceleration, float sideAcceleration, float inwardNormalAcceleration)
        {
            return tangent * tangentAcceleration + side * sideAcceleration - normal * inwardNormalAcceleration;
        }
    }

    /// <summary>
    /// v²κ、法線容量、Critical Ratio、Critical Speed、Required Stickを
    /// 同じ式から一度に導出した状態です。
    /// </summary>
    struct ContactInvariant
    {
        public bool curvatureValid;
        public float requiredNormalAcceleration;
        public float availableNormalAcceleration;
        public float criticalRatio;
        public float criticalSpeed;
        public float requiredStickForTargetRatio;
    }

    /// <summary>
    /// 局所基底上の人工加速度です。AddForceはこの型を介して一か所だけで実行します。
    /// </summary>
    struct ControlCommand
    {
        public SurfaceBasis basis;
        public float tangentAcceleration;
        public float sideAcceleration;
        public float inwardNormalAcceleration;

        public bool valid => basis.valid;

        public Vector3 ToWorld()
        {
            return valid ? basis.Compose(tangentAcceleration, sideAcceleration, inwardNormalAcceleration) : Vector3.zero;
        }

    }

    #region Canonical 500-Line Physics View

    const float epsilon = 0.000001f;

    /// <summary>
    /// 500行版と3800行版が共有する面上の観測値です。
    /// tangent / side / normal と速度・重力を一度だけ同じ座標系へ分解します。
    /// </summary>
    struct Surface
    {
        public SurfaceBasis basis;
        public float tangentSpeed;
        public float sideSpeed;
        public float outwardSpeed;
        public float gravityAlong;
        public float gravitySupport;

        public bool valid => basis.valid;
        public Vector3 tangent => basis.tangent;
        public Vector3 side => basis.side;
        public Vector3 normal => basis.normal;
    }

    Surface BuildSurface(Vector3 normalInput, Vector3 preferredTangent, Vector3 alignmentAxis)
    {
        Vector3 fallback = heading.sqrMagnitude > epsilon ? heading : initialHeading;
        SurfaceBasis basis = SurfaceBasis.Create(normalInput, preferredTangent, fallback);

        if (!basis.valid)
            return default;

        if (alignmentAxis.sqrMagnitude > epsilon && Vector3.Dot(basis.tangent, alignmentAxis) < 0f)
        {
            basis.tangent = -basis.tangent;
            basis.side = -basis.side;
        }

        Vector3 velocity = rb.velocity;

        return new Surface
        {
            basis = basis,
            tangentSpeed = Vector3.Dot(velocity, basis.tangent),
            sideSpeed = Vector3.Dot(velocity, basis.side),
            outwardSpeed = Mathf.Max(0f, Vector3.Dot(velocity, basis.normal)),
            gravityAlong = Vector3.Dot(Physics.gravity, basis.tangent),
            gravitySupport = Mathf.Max(0f, Vector3.Dot(Physics.gravity, -basis.normal))
        };
    }

    Vector3 ApplySurfaceAcceleration(
        SurfaceBasis basis,
        float tangentAcceleration,
        float sideAcceleration,
        float inwardAcceleration)
    {
        if (!basis.valid)
            return Vector3.zero;

        Vector3 acceleration = basis.Compose(
            tangentAcceleration,
            sideAcceleration,
            Mathf.Max(0f, inwardAcceleration)
        );

        if (acceleration.sqrMagnitude > epsilon * epsilon)
            rb.AddForce(acceleration, ForceMode.Acceleration);

        return acceleration;
    }

    static float RequiredAcceleration(float currentSpeed, float targetSpeed, float distance)
    {
        return (targetSpeed * targetSpeed - currentSpeed * currentSpeed) /
            (2f * Mathf.Max(0.0001f, distance));
    }

    static float ReachableSpeed(float terminalSpeed, float deceleration, float distance)
    {
        return Mathf.Sqrt(
            Mathf.Max(
                0f,
                terminalSpeed * terminalSpeed +
                2f * Mathf.Max(0f, deceleration) * Mathf.Max(0f, distance)
            )
        );
    }

    static float MoveToward(float current, float target, float riseJerk, float fallJerk, float dt)
    {
        float jerk = target >= current ? riseJerk : fallJerk;
        return Mathf.MoveTowards(current, target, Mathf.Max(0f, jerk) * Mathf.Max(0f, dt));
    }

    static float SmoothRange01(float value, float start, float end)
    {
        if (end <= start + epsilon)
            return value >= end ? 1f : 0f;

        return SmootherStep01(Mathf.InverseLerp(start, end, value));
    }

    static float SmootherStep01(float value)
    {
        float t = Mathf.Clamp01(value);
        return t * t * t * (t * (t * 6f - 15f) + 10f);
    }

    static float Progress(SlopeFrame frame, Vector3 position)
    {
        if (!frame.valid)
            return 0f;

        float distance = Vector3.Dot(position - frame.entryPoint, frame.axis);
        return Mathf.Clamp01(distance / Mathf.Max(frame.projectedLength, 0.0001f));
    }

    float RequiredStick(float speed, float curvature, float gravitySupport)
    {
        float safeRatio = Mathf.Clamp(targetCriticalRatio, 0.01f, 0.999f);
        return Mathf.Max(
            0f,
            speed * speed * Mathf.Max(0f, curvature) / safeRatio -
            Mathf.Max(0f, gravitySupport)
        );
    }

    float SafeSpeed(float curvature, float gravitySupport, float stickCapacity)
    {
        float safeCurvature = Mathf.Max(minimumCurvature, curvature);
        float safeRatio = Mathf.Clamp(targetCriticalRatio, 0.01f, 0.999f);

        return Mathf.Sqrt(
            Mathf.Max(
                0f,
                safeRatio *
                (Mathf.Max(0f, gravitySupport) + Mathf.Max(0f, stickCapacity)) /
                safeCurvature
            )
        );
    }

    static float BridgeRecoveryAcceleration(
        float gap,
        float outwardSpeed,
        float dt,
        float limit)
    {
        float horizon = Mathf.Max(dt * 2f, 0.04f);
        float positionRecovery =
            2f * (Mathf.Max(0f, gap) + Mathf.Max(0f, outwardSpeed) * horizon) /
            Mathf.Max(epsilon, horizon * horizon);

        return Mathf.Clamp(
            Mathf.Max(positionRecovery, Mathf.Max(0f, outwardSpeed) / horizon),
            0f,
            Mathf.Max(0f, limit)
        );
    }

    #endregion


    [Header("References")]
    [SerializeField] Rigidbody rb;

    [SerializeField] public Rigidbody rbClone;

    [Header("BallVisual Synchronization Debug")]
    [SerializeField] bool logBallVisualDelayedStartSynchronization = true;
    [SerializeField] bool resetBallVisualVelocityAtDelayedStart = true;

    [SerializeField] Transform cameraTransform;
    [SerializeField] Transform startTransform;

    [Header("PhysicsRoot / VisualPlayerRoot Coordinate Frames")]
    [Tooltip("回転させない物理計算座標系です。InSubjectと物理用ステージをこの配下へ置きます。")]
    [SerializeField] Transform physicsRoot;
    [FormerlySerializedAs("stageRoot")]
    [Tooltip("回転する表示座標系です。VisualStageとSubjectをこの座標系側へ置きます。")]
    [SerializeField] Transform visualPlayerRoot;
    [FormerlySerializedAs("stagePivot")]
    [Tooltip("VisualPlayerRootを回転させるワールド空間上の中心です。未設定ならCenter1を検索します。")]
    [SerializeField] Transform visualRotationPivot;
    [Tooltip("PhysicsRoot上のInSubjectをVisualPlayerRoot座標へ写すSubject代理体です。")]
    [SerializeField] CorrespondSubject correspondSubject;
    [FormerlySerializedAs("stageTurnsOppositeToPlayer")]
    [Tooltip("ONなら入力角度と反対方向へVisualPlayerRootを回します。InSubjectの物理状態はどちらでも回転しません。")]
    [SerializeField] bool visualRootTurnsOppositeToInput = true;
    [Tooltip("ResetBallToStartでVisualPlayerRootを開始時の姿勢へ戻します。")]
    [SerializeField] bool resetVisualFrameOnBallReset = true;
    [Tooltip("PhysicsRootが実行中に回転した場合にエラーを記録します。")]
    [SerializeField] bool validatePhysicsRootRemainsInertial = true;
    [Tooltip("これ未満の横フリックは旋回として扱いません。刻み判定の基準距離にも使用します。")]
    [SerializeField, Min(1f)] float minimumFlickPixels = 10f;
    [Tooltip("フリック距離を対数的な強度帯へ分ける倍率です。1.8なら基準距離の1.8倍ごとに次の角度へ進みます。")]
    [SerializeField, Min(1.01f)] float flickStrengthBandRatio = 1.8f;
    [Tooltip("弱いフリックから強いフリックへ順番に割り当てる旋回角度です。")]
    [SerializeField] float[] flickTurnAngleSteps =
    {
        90f,
        90f,
        90f,
        90f
    };
    
    [Tooltip("ボタン操作時の固定旋回角度です。フリック操作ではFlick Turn Angle Stepsを使用します。")]
    [SerializeField, Range(0f, 180f)] float turnAngle = 45f;

    [Header("Reliable Baseline Sampler - 3 Point")]
    [SerializeField] ReliableBaselineSampler reliableBaselineSampler;
    [Tooltip("前回の接地法線からこの角度以上変化した時にAbruptNormalChangeを保存します。")]
    [SerializeField, Range(.1f, 89f)] float baselineAbruptNormalAngle = 5f;
    [Tooltip("InitialGroundedRestが完成するまでは既存の移動・吸着制御を実行しません。")]
    [SerializeField] bool pauseControllerUntilInitialRest = true;

    [Header("Reliable Baseline JSON Replay")]
    [Tooltip("完成済みJSONがある場合、3状態の各タイミングで保存済みvelocity / angularVelocityをワールド値のまま適用します。")]
    [SerializeField] bool replayTrustedBaselineExactly = false;
    [Tooltip("JSON値を適用した時にConsoleへ記録します。")]
    [SerializeField] bool logTrustedBaselineReplay = true;

    [Header("Start / Auto Progress")]
    [SerializeField] bool resetToStartOnPlay;
    [SerializeField] Vector3 initialHeading = Vector3.right;
    [SerializeField] bool useAutoProgress = true;
    [Range(0f, 1f)] [SerializeField] float steeringStrength = 0.75f;

    [Header("Ground Detection")]
    [Min(0.01f)] [SerializeField] float sphereRadius = 0.5f;
    [Min(0f)] [SerializeField] float groundProbeDistance = 0.45f;
    [SerializeField] LayerMask groundMask = ~0;
    [Range(0f, 89f)] [SerializeField] float maxSlopeAngle = 75f;
    [Range(0f, 20f)] [SerializeField] float minimumSlopeAngle = 2f;

    [Header("Support Surface Latch")]
    [Tooltip("Unity物理が生成した実ContactをSphereCastより先に接地観測として採用します。")]
    [SerializeField] bool useCollisionContactsForGround = false;
    [Tooltip("Collision Contactをコールバック直後のFixedUpdateで読むための追加保持ステップ数です。0が最小です。")]
    [Range(0, 1)] [SerializeField] int collisionContactMemorySteps = 1;
    [Tooltip("接地観測が一時的に抜けたとき、同じSlopeFrameを保持するFixedUpdate数です。")]
    [Range(0, 12)] [SerializeField] int slopeTrackingGraceFrames = 3;
    [SerializeField] bool logSupportSurfaceLatch = true;

    [Header("Local Flat -> Slope Sequence Patch")]
    [Tooltip("平面接近時の残距離逆算制動と、斜面入口25%未満で発生した1 FixedUpdateだけの接地観測欠落を局所補間します。通常の平面・斜面中央・出口には作用しません。")]
    [SerializeField] bool useLocalFlatToSlopeSequencePatch = true;
    [Tooltip("局所Sequenceの予測制動と支持面BridgeをConsoleへ記録します。")]
    [SerializeField] bool logLocalFlatToSlopeSequencePatch = true;

    [Header("Slope Entry Velocity Transport + Stick Preload")]
    [Tooltip("Flatで予測した同一Slopeへ初めて接触した時だけ、過大な外向き法線速度を斜面接平面側へ輸送します。小さな自然バウンドは残します。")]
    [SerializeField] bool useSlopeEntryVelocityTransport = true;
    [Tooltip("自然バウンド閾値を超えた外向き法線速度のうち、接平面へ輸送する割合です。1でも自然バウンド閾値までは残します。")]
    [Range(0f, 1f)] [SerializeField] float slopeEntryVelocityTransportWeight = 0.80f;
    [Tooltip("この値以下の外向き法線速度は軽快な入口バウンドとして変更しません。")]
    [Min(0f)] [SerializeField] float slopeEntryNaturalBounceSpeed = 0.65f;
    [Tooltip("Entry TransportとAdaptive Stick Preloadを許可する斜面入口Progressです。0.12は入口12%までです。")]
    [Range(0.01f, 0.25f)] [SerializeField] float slopeEntryControlMaximumProgress = 0.12f;
    [Tooltip("斜面の最初の実接触では、Flat予測中のFallback状態に関係なくAdaptive Critical Stick必要値を即時プリロードします。")]
    [SerializeField] bool useAdaptiveCriticalEntryStickPreload = true;
    [Tooltip("入口で理論必要Stickへ掛ける小さな安全余裕です。最終値は既存Adaptive Stick容量で制限されます。")]
    [Range(1f, 1.25f)] [SerializeField] float adaptiveEntryStickPreloadSafetyMargin = 1.05f;
    [SerializeField] bool logSlopeEntryTransportAndPreload = true;

    [Header("Forward Slope Detection")]
    [Min(0.1f)] [SerializeField] float forwardSlopeProbeDistance = 8f;
    [Min(2)] [SerializeField] int forwardSlopeProbeSegments = 24;
    [Min(0.1f)] [SerializeField] float forwardProbeHeight = 3f;
    [Min(0.1f)] [SerializeField] float forwardProbeDownDistance = 8f;

    // [SerializeField] bool useTargetProgressPreconditioning = true;

    [Tooltip("Target Progress制御で使用できる正方向の最大人工加速度です。")]
    [Min(0f)]
    [SerializeField] float targetProgressMaximumArtificialAcceleration = 4f;

    [Min(0f)] [SerializeField] float maxGroundSpeed = 32f;
    [Min(0f)] [SerializeField] float maxGroundAcceleration = 45f;
    [Min(0f)] [SerializeField] float airAcceleration = 6f;

    [Header("Adaptive Critical Adhesion")]
    [Tooltip("平面、曲率無効時、またはAdaptive Adhesion無効時に使用する基準吸着加速度です。曲率が有効な斜面では固定値として速度を縛らず、必要吸着力をFixedUpdateごとに逆算します。")]
    [Min(0f)] [SerializeField] float baseStickAcceleration = 24.6f;
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
    [Min(0.01f)] [SerializeField] float adaptiveOutwardNormalResponseSeconds = 0.08f;

    [Header("Deductive Adaptive Stick Calibration")]
    [Tooltip("ONなら理論式 v^2*kappa/rho - gravitySupport で求めた必要Stickへ、最後の校正倍率だけを掛けます。絶対Stick値を手動調整する代わりに、通常は1.0付近だけを微調整します。")]
    [SerializeField] bool useDeductiveAdaptiveStickCalibration = true;
    [Tooltip("理論必要Stickへの最終校正倍率。1=理論値そのまま。通常のベンチマークでは0.95〜1.05程度だけを触る想定です。")]
    [Range(0.5f, 1.5f)] [SerializeField] float adaptiveStickCalibration = 1f;

    [SerializeField] bool logAdaptiveCriticalBaseStick = true;

    [Header("Critical Boundary Tracking")]
    [SerializeField] bool useCriticalBoundaryTracking = true;
    [Tooltip("1.0が理論離脱限界です。Unityの離散誤差を考慮して1未満にします。")]
    [Range(0.80f, 0.999f)] [SerializeField] float targetCriticalRatio = 0.98f;
    [Tooltip("前方斜面上で臨界速度を評価する点数です。")]
    [Range(4, 64)] [SerializeField] int criticalSampleCount = 24;
    [Tooltip("前方を何秒分まで評価するか。実距離は速度から自動計算します。")]
    [Min(0.05f)] [SerializeField] float criticalLookAheadSeconds = 0.15f;
    [Tooltip("接線方向の最大減速度。制動距離の逆算にも同じ値を使用します。")]
    [Min(0.1f)] [SerializeField] float maximumCriticalDeceleration = 30f;
    [Tooltip("接線方向の最大加速度。")]
    [Min(0.1f)] [SerializeField] float maximumCriticalAcceleration = 35f;
    [Tooltip("速度誤差を加速度へ変換する応答時間です。")]
    [Min(0.02f)] [SerializeField] float criticalResponseSeconds = 0.09f;
    [Tooltip("接線加速度が変化できる最大Jerkです。")]
    [Min(0.1f)] [SerializeField] float criticalJerkLimit = 320f;
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
    [Min(0f)] [SerializeField] float criticalHoldSeconds = 0.06f;

    [Header("Slope Progress Target Observation")]
    [Tooltip("斜面上の目標地点です。観測の基準であると同時に、下の事前調整が有効なら目標Critical Ratioへ近づける基準にも使います。")]
    [Range(0f, 100f)] [SerializeField] float targetSlopeProgressPercent = 60f;
    [Tooltip("目標の手前/直後を-0/+0として分類する範囲です。")]
    [Range(0.01f, 5f)] [SerializeField] float progressNearZeroTolerancePercent = 0.5f;
    [Tooltip("極限捕捉準備完了とみなす、同一SlopeFrame上の連続観測数です。")]
    [Range(1, 60)] [SerializeField] int requiredStableSlopeFrames = 1;
    [SerializeField] bool logProgressTargetCrossing = true;

    [Header("Target Progress Preconditioning")]
    [Tooltip("同一SlopeFrameを安定観測した後、targetProgressでtargetCriticalRatioへ近づくために接線方向だけを事前調整します。新しいAddForceは増やさず、既存の接線成分を置き換えます。")]

    [SerializeField] bool useTargetProgressPreconditioning = true;

    [Tooltip("事前調整が使用できる人工的な最大減速度です。重力の斜面方向成分を相殺しながら目標速度へ近づけるため、45度下降面では2より大きい値が必要です。")]
    [Min(0.1f)] [SerializeField] float targetProgressMaximumArtificialDeceleration = 12f;
    [Tooltip("事前調整加速度の変化上限です。急激な切り替えを避けつつ、短い斜面でも間に合う値にします。")]
    [Min(0.1f)] [SerializeField] float targetProgressJerkLimit = 180f;
    [Tooltip("targetProgress直前で残距離除算が発散しないための最小距離です。")]
    [Min(0.01f)] [SerializeField] float targetProgressMinimumDistance = 0.27f;
    [Tooltip("必要な正味加速度から斜面方向の重力加速度を差し引き、実際にAddForceで必要な最小人工加速度を求めます。")]
    [SerializeField] bool compensateTargetProgressGravity = true;
    [SerializeField] bool logTargetProgressPreconditioning = true;

    [Header("Automatic Speed-Homogeneous Control (up to Max Ground Speed 32)")]
    [Tooltip("Max Ground Speed、斜面長、残距離、重力、曲率からTarget Progress用加速度・Jerk・Adaptive Stick上限を区間ごとに自動捕捉します。")]
    [SerializeField] bool useAutomaticSpeedHomogeneousControl = true;
    [Tooltip("自動算出した正方向接線加速度の絶対安全上限です。通常は速度・残距離から必要な値だけを使用します。")]
    [Min(10f)] [SerializeField] float automaticTangentialAccelerationHardLimit = 180f;
    [Tooltip("Soft Overspeed Governorが使用できる負方向接線加速度の絶対安全上限です。")]
    [Min(10f)] [SerializeField] float automaticTangentialBrakeHardLimit = 180f;
    [Tooltip("速度32で必要な吸着力を自動拡張した後にも超えない最終安全上限です。")]
    [Min(100f)] [SerializeField] float automaticAdaptiveStickHardLimit = 1600f;
    [Tooltip("理論上必要な接線加速度へ加える固定安全余裕です。")]
    [Range(1f, 1.5f)] [SerializeField] float automaticControlAccelerationMargin = 1.15f;
    [Tooltip("速度二乗・曲率から逆算したAdaptive Stick容量へ加える安全余裕です。")]
    [Range(1f, 1.5f)] [SerializeField] float automaticAdaptiveStickSafetyMargin = 1.08f;
    [Tooltip("Overspeedを何FixedUpdate分の進行距離で滑らかに戻すかを指定します。")]
    [Range(2, 8)] [SerializeField] int automaticOverspeedRecoveryFixedSteps = 3;
    [SerializeField] bool logAutomaticSpeedHomogeneousControl = true;

    [Header("Natural Artificial Release")]
    [Tooltip("targetProgress通過後、人工加速度を滑らかに減らし、Natural Release Progressで完全にUnity物理へ渡します。")]
    [SerializeField] bool useNaturalArtificialRelease = true;
    [Tooltip("このProgress以上では人工接線加速度・Base Stick・Air Accelerationを復活させません。")]
    [Range(0f, 100f)] [SerializeField] float naturalReleaseProgressPercent = 90f;
    [Tooltip("targetProgress通過後もBase Stickを100%維持するProgress幅です。30%目標なら2で32%まで維持します。")]
    [Range(0f, 20f)] [SerializeField] float releaseHoldAfterTargetPercent = 2f;
    [Tooltip("現在速度に対して確保したい解放時間です。速度×時間から必要な解放距離を求めます。")]
    [Min(0.05f)] [SerializeField] float releaseNominalSeconds = 0.13f;
    [Tooltip("低速時でも確保する最小解放Progress幅です。")]
    [Range(0.1f, 50f)] [SerializeField] float minimumReleaseWidthPercent = 8f;
    [Tooltip("設計速度の下限をMax Ground Speedに対する比率で定めます。Base Stick自体へMax Ground Speedを直接掛けません。")]
    [Range(0.05f, 1f)] [SerializeField] float releaseSpeedFloorRatio = 0.35f;
    [SerializeField] bool logNaturalArtificialRelease = true;

    [Header("Runtime Readout")]
    [SerializeField]public GroundKind groundKind;
    [SerializeField] bool hasForwardSlope;
    [SerializeField] string activeSlopeName;
    float slopeProgressStatePercent;
    [SerializeField] float tangentSpeedState;
    [SerializeField] float currentAllowedSpeed;
    [SerializeField] float criticalRatioState;
    [SerializeField] float curvatureState;
    [SerializeField] float availableNormalState;
    [SerializeField] float requiredNormalState;
    [SerializeField] float currentTangentialAcceleration;
    [SerializeField] float currentEffectiveMaximumDeceleration;
    [SerializeField] bool criticalStateMaintained;
    [SerializeField] float criticalMaintainedSeconds;
    [SerializeField] ArtificialControlPhase artificialControlPhase;
    [SerializeField] Vector3 currentAppliedArtificialAcceleration;

    [Header("Support / Progress Runtime Readout")]
    [SerializeField] int consecutiveGroundMissFrames;
    [SerializeField] int stableSlopeContactFrames;
    [SerializeField] bool slopeProgressObservationValid;
    [SerializeField] float previousSlopeProgressPercent;
    [SerializeField] float slopeProgressDeltaPercent;
    [SerializeField] SlopeProgressSide slopeProgressSide;
    [SerializeField] bool crossedTargetThisFrame;
    [SerializeField] bool readyForLimitCapture;
    [SerializeField] float estimatedTargetCrossingTime;

    [Header("Local Flat -> Slope Sequence Runtime")]
    [SerializeField] LocalFlatToSlopeSequencePhase localFlatToSlopeSequencePhase;
    [SerializeField] bool slopeEntrySupportBridgeActive;
    [SerializeField] float slopeEntrySupportBridgeGap;
    [SerializeField] float slopeEntryPredictiveBrakeAcceleration;
    [SerializeField] float slopeEntryPredictiveTargetSpeed;

    [Header("Slope Entry Transport + Preload Runtime")]
    [SerializeField] bool slopeEntryVelocityTransportAppliedThisFrame;
    [SerializeField] float slopeEntryOutwardSpeedBeforeTransport;
    [SerializeField] float slopeEntryOutwardSpeedAfterTransport;
    [SerializeField] Vector3 slopeEntryTransportDeltaVelocity;
    [SerializeField] bool adaptiveEntryStickPreloadAppliedThisFrame;
    [SerializeField] float adaptiveEntryStickPreloadState;

    [Header("Target Progress Preconditioning Runtime")]
    [SerializeField] TargetProgressPreconditionPhase targetProgressPhase;
    [SerializeField] bool targetProgressPlanValid;
    [SerializeField] float capturedTargetProgressPercent;
    [SerializeField] float capturedTargetTangentSpeed;

    [Header("Automatic Speed-Homogeneous Runtime")]
    [SerializeField] float capturedAutomaticPositiveAccelerationLimit;
    [SerializeField] float capturedAutomaticNegativeAccelerationLimit;
    [SerializeField] float capturedAutomaticJerkLimit;

    [Header("Natural Artificial Release Runtime")]
    [SerializeField] bool naturalReleasePlanValid;
    [SerializeField] bool naturalMotionReleased;
    [SerializeField] float releaseStartProgressState;
    [SerializeField] float naturalReleaseProgressState;
    float artificialReleaseState = 1f;

    [Header("Adaptive Critical Adhesion Runtime")]
    [SerializeField] float effectiveBaseStickState;
    [SerializeField] bool adaptiveStickSaturated;
    [SerializeField] float theoreticalRequiredStickState;
    [SerializeField] float calibratedRequiredStickState;
    [SerializeField] float adaptivePredictedSpeedState;
    [SerializeField] float adaptiveGravitySupportState;

    [Header("Debug")]
    [SerializeField] bool writeRuntimeLog = true;
    [Min(1)] [SerializeField] int logEveryFixedFrames = 30;
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
    [Tooltip("entryGuessとexitGuessから実際に飛ばす中央1本のRayをLineRendererで表示します。")]
    [SerializeField] bool showEntryExitRayLines = true;
    [Tooltip("各斜面ColliderについてEntry/Exitの中央Rayを最初の1回だけ保存します。別の斜面では別のLineRendererを生成します。")]
    [SerializeField] bool freezeEntryExitRayLinesAfterCapture = true;
    [Tooltip("斜面ごとにEntry 1本＋Exit 1本を保持し、全斜面を同時表示します。")]
    [SerializeField] bool keepVisualizationForEverySlope = true;
    [Tooltip("対象斜面へ当たらなかったRayも最大距離まで表示します。")]
    [SerializeField] bool showMissedRayFullLength = true;
    [Min(0.001f)] [SerializeField] float slopeRayLineWidth = 0.03f;
    [SerializeField] Material slopeRayLineMaterial;
    [SerializeField] Color entryRayHitColor = Color.cyan;
    [SerializeField] Color exitRayHitColor = Color.yellow;
    [SerializeField] Color missedRayColor = Color.red;
    
    
    public float BallVisualBaseStickAcceleration =>
        baseStickAcceleration;

    Vector2 input;
    Vector3 heading;
    Vector2 flickStart;
    bool trackingFlick;
    // Tween turnTween; // DOTween rotation disabled.
    float pendingFlickTurnDegrees;
    bool flickTurnAppliedThisFixedUpdate;
    Vector3 initialVisualPlayerRootPosition;
    Quaternion initialVisualPlayerRootRotation = Quaternion.identity;
    Quaternion initialPhysicsRootRotation = Quaternion.identity;
    bool hasInitialVisualFramePose;
    bool hasInitialPhysicsRootRotation;
    bool physicsRootRotationErrorLogged;
    Vector3 groundNormal = Vector3.up;
    Vector3 previousBaselineNormal = Vector3.up;
    bool hasPreviousBaselineNormal;
    bool baselineInitialRestReady;
    bool initialGroundedRestReplayApplied;
    bool abruptNormalReplayApplied;
    bool beforeDOTweenTurnReplayApplied;
    public Vector3 restart;
    GroundObservation currentGroundObservation;
    GroundObservation latestCollisionGroundObservation;
    SlopeFrame slopeFrame;
    SlopeFrame forwardSlopeFrame;
    int fixedFrameCounter;
    bool wasGrounded;
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

    GroundObservation lastMeasuredSlopeObservation;
    float lastMeasuredSlopeProgress;
    float previousSlopeEntryPredictiveBrakeAcceleration;
    Collider slopeEntryBrakeLoggedCollider;
    bool slopeEntryBrakeLogged;

    Collider anticipatedSlopeEntryCollider;
    Collider slopeEntryVelocityTransportCollider;
    Collider adaptiveEntryStickPreloadCollider;

    Collider targetProgressPlanCollider;
    float previousTargetProgressAppliedAcceleration;
    float previousOverspeedGovernorAcceleration;
    bool targetProgressCompletionPending;
    bool targetProgressPlanFailureLogged;

    Collider naturalReleasePlanCollider;
    bool naturalReleaseLatchLogged;
    Collider adaptiveStickActiveCollider;
    float previousAdaptiveBaseStickAcceleration;
    bool adaptiveStickSaturationLogged;
    
    
    public bool BallVisualHasForwardSlope =>
        groundKind == GroundKind.Flat &&
        hasForwardSlope &&
        forwardSlopeFrame.valid;

    public float BallVisualDistanceToNextSlopeEntry 
    { get; private set; }= float.PositiveInfinity;

    sealed class SlopeSectionRayVisual
    {
        public Transform root;
        public LineRenderer entryRay;
        public LineRenderer exitRay;
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

    GameObject limitPointSphereObject;

    GroundKind previousLoggedGroundKind = GroundKind.Air;
    bool previousLoggedCriticalMaintained;
    string previousLoggedSlopeName = string.Empty;
    float lastCriticalRiskLogTime = float.NegativeInfinity;
    float sessionMinimumCriticalError = float.PositiveInfinity;
    float sessionLongestMaintainedSeconds;
    int sessionCriticalSuccessCount;
    int sessionGroundToAirCount;
    int sessionSlopeDetectionCount;
    float previousAppliedTangentialAcceleration;
    float sessionMaximumControlledRatio;
    float sessionMaximumReleasingRatio;
    float sessionMaximumReleasedRatio;

    [SerializeField] public float slopeProgressErrorPercent;
    

    public float TargetSlopeProgressPercent => targetSlopeProgressPercent;

    // -----------------------------------------------------------------
    // BallVisual pre-limit deadline telemetry
    // READ ONLY: BallVisualSlopeDriveがLimit到達時刻と位置を予測するためだけに使う。
    // SlopeStick3Dの物理状態を外部から変更するsetterは持たせない。
    // -----------------------------------------------------------------
    public bool BallVisualHasActiveSlopeFrame =>
        slopeFrame.valid &&
        slopeFrame.collider != null &&
        slopeFrame.projectedLength > 0.0001f;

    public float BallVisualSlopeProgress01 =>
        BallVisualHasActiveSlopeFrame
            ? CalculateProgress(slopeFrame, rb.position)
            : 0f;

    // SlopeStick3D自身が観測したProgress差をFixedUpdate時間で割った値。
    // 1フレームのRigidbody加速度ではなく、Targetへ向かう進捗時計として使う。
    public float BallVisualSlopeProgressRatePercentPerSecond
    {
        get
        {
            float dt = Mathf.Max(Time.fixedDeltaTime, 0.000001f);

            if (!slopeProgressObservationValid ||
                Mathf.Abs(slopeProgressDeltaPercent) > 25f)
            {
                return 0f;
            }

            return slopeProgressDeltaPercent / dt;
        }
    }

    public float BallVisualSlopeSectionLength =>
        BallVisualHasActiveSlopeFrame
            ? slopeFrame.projectedLength
            : 0f;

    // 現在のInSubject中心位置からTarget Progressまで、
    // slopeFrame.axis上の残距離だけ進めた「球中心のTarget位置」。
    // surface pointではなくrb.positionを基準にするため、球半径分の法線ズレを作らない。
    public Vector3 BallVisualTargetProgressCenterPhysics
    {
        get
        {
            if (!BallVisualHasActiveSlopeFrame || !rb)
                return rb ? rb.position : transform.position;

            float currentProgress =
                CalculateProgress(slopeFrame, rb.position);

            float targetProgress =
                Mathf.Clamp01(targetSlopeProgressPercent * 0.01f);

            float remainingDistance =
                (targetProgress - currentProgress) *
                slopeFrame.projectedLength;

            Vector3 axis =
                slopeFrame.axis.sqrMagnitude > 0.000001f
                    ? slopeFrame.axis.normalized
                    : Vector3.forward;

            return rb.position + axis * remainingDistance;
        }
    }

    public bool ReadyForLimitCapture => readyForLimitCapture;
    public bool CriticalStateMaintained => criticalStateMaintained;
    public bool TargetProgressPlanValid => targetProgressPlanValid;
    public float CapturedTargetTangentSpeed => capturedTargetTangentSpeed;
    public bool NaturalMotionReleased => naturalMotionReleased; 
    SphereCollider inSubjectSphere;
    private SphereCollider equalizerSphere;
    void Reset()
    {
        inSubjectSphere =
            transform.GetComponent<SphereCollider>();
        equalizerSphere =
            GameObject.Find("VisualPlayerRoot/BallVisualEqualizer")
                .GetComponent<SphereCollider>();

        Physics.IgnoreCollision(transform.GetComponent<SphereCollider>(),
            GameObject.Find("BallVisual").transform.GetComponent<SphereCollider>());
        Physics.IgnoreCollision(transform.GetComponent<SphereCollider>(),
            GameObject.Find("BallVisualEqualizer").transform.GetComponent<SphereCollider>());
        Physics.IgnoreCollision(
            inSubjectSphere,
            equalizerSphere,
            true);
        initialHeading = Vector3.forward;
    }

    Vector3 ResolvePhysicsPoint(Transform source)
    {
        if (!source)
            return Vector3.zero;

        if (correspondSubject &&
            visualPlayerRoot &&
            (source == visualPlayerRoot || source.IsChildOf(visualPlayerRoot)))
        {
            return correspondSubject.InverseMapPoint(source.position);
        }

        return source.position;
    }

    Quaternion ResolvePhysicsRotation(Transform source)
    {
        if (!source)
            return Quaternion.identity;

        if (correspondSubject &&
            visualPlayerRoot &&
            (source == visualPlayerRoot || source.IsChildOf(visualPlayerRoot)))
        {
            return correspondSubject.InverseMapRotation(source.rotation);
        }

        return source.rotation;
    }

    IEnumerator DelayStart(float time)
    {
        yield return new WaitForSeconds(time);
        initialHeading = new Vector3(0, 0, 1);
        Time.timeScale = 0.5f;

        GameObject startSlab = GameObject.Find("CollisionStageRoot/__GeneratedPhysics/ArcSlab2_0_Physics");
        if (!startSlab)
        {
            Debug.LogError("[BALL VISUAL DELAY START] ArcSlab4が見つかりません。", this);
            yield break;
        }

        if (!rb)
        {
            Debug.LogError("[BALL VISUAL DELAY START] InSubject Rigidbodyがありません。", this);
            yield break;
        }

        if (!rbClone)
        {
            Debug.LogError(
                "[BALL VISUAL DELAY START] rbCloneが未設定です。" +
                "BallVisualSlopeDriveのStartまたはInspectorでBallVisualのRigidbodyを設定してください。",
                this
            );
            yield break;
        }

        if (logBallVisualDelayedStartSynchronization)
            LogBallVisualSynchronization("BEFORE_DELAYED_POSITION_SET");

        restart = ResolvePhysicsPoint(startSlab.transform);
        Vector3 synchronizedPosition = new Vector3(restart.x, restart.y + 2f, restart.z);
        
        //開始地点　 開始地点　開始地点
        rb.position = synchronizedPosition;
       

        Vector3 mappedVisualPosition = correspondSubject
            ? correspondSubject.MapPoint(synchronizedPosition)
            : synchronizedPosition;

        Quaternion mappedVisualRotation = correspondSubject
            ? correspondSubject.MapRotation(rb.rotation)
            : rb.rotation;

        rbClone.position = mappedVisualPosition;
        rbClone.rotation = mappedVisualRotation;

        if (resetBallVisualVelocityAtDelayedStart)
        {
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rbClone.velocity = Vector3.zero;
            rbClone.angularVelocity = Vector3.zero;
        }

        Physics.SyncTransforms();
        correspondSubject?.SynchronizeNow(true);

        if (logBallVisualDelayedStartSynchronization)
            LogBallVisualSynchronization("AFTER_DELAYED_POSITION_SET");
    }

    public void LogBallVisualSynchronization(string source)
    {
        if (!rb || !rbClone)
        {
            Debug.LogWarning(
                $"[SUBJECT/BALL VISUAL SYNC] source={source} time={Time.fixedTime:F4} " +
                $"subjectBody={(rb ? rb.name : "NULL")} ballBody={(rbClone ? rbClone.name : "NULL")}",
                this
            );
            return;
        }

        Vector3 mappedInSubjectPosition = correspondSubject
            ? correspondSubject.MappedPosition
            : rb.position;

        Vector3 mappedInSubjectVelocity = correspondSubject
            ? correspondSubject.Mappedvelocity
            : rb.velocity;

        Vector3 mappedInSubjectAngularVelocity = correspondSubject
            ? correspondSubject.MappedAngularVelocity
            : rb.angularVelocity;

        Vector3 positionError = mappedInSubjectPosition - rbClone.position;
        Vector3 velocityError = mappedInSubjectVelocity - rbClone.velocity;
        Vector3 angularVelocityError = mappedInSubjectAngularVelocity - rbClone.angularVelocity;

        Debug.Log(
            $"[INSUBJECT/BALL VISUAL SYNC] source={source} time={Time.fixedTime:F4} " +
            $"mappedInSubjectPos={mappedInSubjectPosition:F5} ballPos={rbClone.position:F5} " +
            $"positionError={positionError:F5} positionErrorMag={positionError.magnitude:F6} " +
            $"mappedInSubjectVelocity={mappedInSubjectVelocity:F5} ballVelocity={rbClone.velocity:F5} " +
            $"velocityError={velocityError:F5} velocityErrorMag={velocityError.magnitude:F6} " +
            $"mappedInSubjectAngular={mappedInSubjectAngularVelocity:F5} ballAngular={rbClone.angularVelocity:F5} " +
            $"angularError={angularVelocityError:F5} angularErrorMag={angularVelocityError.magnitude:F6}",
            this
        );
    }

    void Start()
    {
        inSubjectSphere =
            transform.GetComponent<SphereCollider>();
        equalizerSphere =
            GameObject.Find("VisualPlayerRoot/BallVisualEqualizer")
                .GetComponent<SphereCollider>();

        Physics.IgnoreCollision(transform.GetComponent<SphereCollider>(),
            GameObject.Find("BallVisual").transform.GetComponent<SphereCollider>());
        Physics.IgnoreCollision(inSubjectSphere, equalizerSphere, true);
        if (!rb)
            rb = GetComponent<Rigidbody>();

        if (!reliableBaselineSampler)
            reliableBaselineSampler = GetComponent<ReliableBaselineSampler>();

        FindCoordinateFrameReferences();
        BindCoordinateFrames();
        CaptureInitialCoordinateFrameState();

        baselineInitialRestReady = reliableBaselineSampler && reliableBaselineSampler.HasInitialGroundedRest;

        heading = NormalizeFlat(initialHeading, Vector3.forward);
        rb.maxAngularVelocity = 100f;
        currentEffectiveMaximumDeceleration = maximumCriticalDeceleration;
        slopeProgressStatePercent = 0f;
        previousSlopeProgressPercent = 0f;
        slopeProgressSide = SlopeProgressSide.Invalid;
        ResetTargetProgressPreconditioning("Awake", false);
        ResetNaturalArtificialRelease("Awake", false);
        if (resetToStartOnPlay && startTransform)
            ResetBallToStart();
        //StartCoroutine(DelayStart());
        if (writeRuntimeLog)
        {
           // Debug.Log($"[SLOPE STICK VERSION] {ImplementationVersion}", this);
        }
    }

    

    void Update()
    {
        if (inSubjectSphere != null && equalizerSphere != null)
        {
            Debug.Log(
                $"[IGNORE CHECK] " +
                $"A={inSubjectSphere.name} id={inSubjectSphere.GetInstanceID()} " +
                $"B={equalizerSphere.name} id={equalizerSphere.GetInstanceID()} " +
                $"ignored={Physics.GetIgnoreCollision(inSubjectSphere, equalizerSphere)}");
        }

        input = new Vector2(Input.GetAxisRaw("Horizontal"), Input.GetAxisRaw("Vertical"));
        ReadTurnFlick();
    }

    void FindCoordinateFrameReferences()
    {
        if (!physicsRoot)
        {
            GameObject foundPhysicsRoot = GameObject.Find("PhysicsRoot");

            if (foundPhysicsRoot)
                physicsRoot = foundPhysicsRoot.transform;
        }

        if (!visualPlayerRoot)
        {
            GameObject foundVisualRoot = GameObject.Find("VisualPlayerRoot");

            if (foundVisualRoot)
                visualPlayerRoot = foundVisualRoot.transform;
        }

        if (!visualRotationPivot)
        {
            GameObject center = GameObject.Find("Center1");

            if (center)
                visualRotationPivot = center.transform;
        }

        if (!correspondSubject)
            correspondSubject = FindFirstObjectByType<CorrespondSubject>();
    }

    void BindCoordinateFrames()
    {
        if (correspondSubject)
        {
            correspondSubject.Bind(
                rb,
                physicsRoot,
                visualPlayerRoot
            );
        }

        if (physicsRoot &&
            transform != physicsRoot &&
            !transform.IsChildOf(physicsRoot))
        {
            Debug.LogWarning(
                "[PHYSICS/VISUAL FRAME] SlopeStick3DのInSubjectはPhysicsRoot配下へ置くことを推奨します。",
                this
            );
        }

        if (correspondSubject && physicsRoot &&
            correspondSubject.transform.IsChildOf(physicsRoot))
        {
            Debug.LogWarning(
                "[PHYSICS/VISUAL FRAME] SubjectはPhysicsRootではなくVisualPlayerRoot側へ置いてください。",
                correspondSubject
            );
        }
    }

    void CaptureInitialCoordinateFrameState()
    {
        if (visualPlayerRoot)
        {
            initialVisualPlayerRootPosition = visualPlayerRoot.position;
            initialVisualPlayerRootRotation = visualPlayerRoot.rotation;
            hasInitialVisualFramePose = true;
        }

        if (physicsRoot)
        {
            initialPhysicsRootRotation = physicsRoot.rotation;
            hasInitialPhysicsRootRotation = true;
        }

        correspondSubject?.SynchronizeNow(true);
    }

    void ValidatePhysicsRootInvariant()
    {
        if (!validatePhysicsRootRemainsInertial ||
            !physicsRoot ||
            !hasInitialPhysicsRootRotation ||
            physicsRootRotationErrorLogged)
        {
            return;
        }

        float angle = Quaternion.Angle(
            initialPhysicsRootRotation,
            physicsRoot.rotation
        );

        if (angle <= 0.001f)
            return;

        physicsRootRotationErrorLogged = true;
        Debug.LogError(
            $"[PHYSICS ROOT ROTATED] PhysicsRootは慣性系として回転させません。" +
            $" initial={initialPhysicsRootRotation.eulerAngles:F3}" +
            $" current={physicsRoot.rotation.eulerAngles:F3}" +
            $" deltaAngle={angle:F6}",
            physicsRoot
        );
    }
    
    public bool BallVisualIsOnFlat =>
        groundKind == GroundKind.Flat;

    public bool BallVisualIsOnSlope
    {
        get
        {
            return groundKind == GroundKind.Slope &&
                   slopeFrame.valid &&
                   slopeFrame.collider != null;
        }
    }

   

    public Vector3 BallVisualSlopeTangent
    {
        get
        {
            if (!slopeFrame.valid ||
                slopeFrame.axis.sqrMagnitude <
                0.000001f)
            {
                return Vector3.zero;
            }

            return slopeFrame.axis.normalized;
        }
    }

    public Vector3 BallVisualSurfaceNormal
    {
        get
        {
            if (groundNormal.sqrMagnitude < 0.000001f)
                return Vector3.up;

            return groundNormal.normalized;
        }
    }

    public float BallVisualAppliedStickAcceleration =>
        effectiveBaseStickState;

    public float BallVisualTheoreticalRequiredStickAcceleration =>
        theoreticalRequiredStickState;

    public float BallVisualCalibratedRequiredStickAcceleration =>
        calibratedRequiredStickState;

    public float BallVisualPredictedTangentSpeed =>
        adaptivePredictedSpeedState;

    public float BallVisualGravitySupportAcceleration =>
        adaptiveGravitySupportState;

    public float BallVisualAdaptiveStickCalibration =>
        useDeductiveAdaptiveStickCalibration
            ? adaptiveStickCalibration
            : 1f;

    public float BallVisualAppliedTangentialAcceleration =>
        currentTangentialAcceleration;

    public float BallVisualReleaseWeight =>
        artificialReleaseState;
    void ReadTurnFlick()
    {
        if (Input.GetMouseButtonDown(0))
        {
            flickStart = Input.mousePosition;
            trackingFlick = true;
        }

        if (!trackingFlick || !Input.GetMouseButtonUp(0))
            return;

        trackingFlick = false;

        Vector2 flick = (Vector2)Input.mousePosition - flickStart;

        if (Mathf.Abs(flick.x) < minimumFlickPixels || Mathf.Abs(flick.x) <= Mathf.Abs(flick.y))
        {
            return;
        }

        float steppedAngle = GetSteppedFlickTurnAngle(Mathf.Abs(flick.x));

        if (steppedAngle <= 0f)
            return;

        BeginPlayerAndStageTurn(flick.x > 0f ? steppedAngle : -steppedAngle);
    }

    float GetSteppedFlickTurnAngle(float flickPixels)
    {
        if (flickTurnAngleSteps == null || flickTurnAngleSteps.Length == 0)
        {
            return Mathf.Clamp(Mathf.Abs(turnAngle), 0f, 180f);
        }

        float basePixels = Mathf.Max(minimumFlickPixels, 1f);

        float bandRatio = Mathf.Max(flickStrengthBandRatio, 1.01f);

        float normalizedDistance = Mathf.Max(flickPixels, basePixels) / basePixels;

        int stepIndex = Mathf.FloorToInt(Mathf.Log(normalizedDistance, bandRatio));

        stepIndex = Mathf.Clamp(stepIndex, 0, flickTurnAngleSteps.Length - 1);

        return Mathf.Clamp(Mathf.Abs(flickTurnAngleSteps[stepIndex]), 0f, 180f);
    }

    public void TurnPlayerAndStageLeft()
    {
        BeginPlayerAndStageTurn(-Mathf.Abs(turnAngle));
    }

    public void TurnPlayerAndStageRight()
    {
        BeginPlayerAndStageTurn(Mathf.Abs(turnAngle));
    }

#if false
    // -------------------------------------------------------------------------
    // Original DOTween player/stage rotation path.
    // Disabled for the no-stage-rotation test. This block is intentionally kept
    // so the former implementation can be restored after the physics test.
    public void BeginPlayerAndStageTurn(float playerAngle)
    {
        FindStageTurnReferences();

        // ReliableBaselineSamplerの3点目を、Tween開始前の物理状態で保存します。
        CaptureBeforeDOTweenTurnBaseline();

        Transform playerTarget = headingTransform ? headingTransform : transform;
        
        Vector3 cloneStartPosition =
            rbClone ? rbClone.position : Vector3.zero;

        Vector3 startHeading = NormalizeFlat(heading, initialHeading);
        
  

        // turnTween?.Kill(); // DOTween rotation disabled.
        Quaternion playerStartRotation = playerTarget.rotation;

        Quaternion playerEndRotation = Quaternion.AngleAxis(playerAngle, Vector3.up) * playerStartRotation;

        bool canRotateStage = stageRoot && stagePivot;

        bool playerIsStageChild = canRotateStage && transform.IsChildOf(stageRoot);

        Vector3 pivot = canRotateStage ? stagePivot.position : Vector3.zero;

        Vector3 stageStartPosition = canRotateStage ? stageRoot.position : Vector3.zero;

        Quaternion stageStartRotation = canRotateStage ? stageRoot.rotation : Quaternion.identity;

        Vector3 playerStartPosition =
            rb ? rb.position : transform.position;

        Vector3 velocityBeforeTurn =
            rb ? rb.velocity : Vector3.zero;

        float stageMultiplier =
            stageTurnsOppositeToPlayer ? -1f : 1f;

        // turnTween?.Kill(); // DOTween rotation disabled.

        void Apply(float angle)
        {
            Quaternion PlayerTurn = Quaternion.AngleAxis(angle, Vector3.up);


            Quaternion stageTurn = Quaternion.AngleAxis(angle * stageMultiplier, Vector3.up);

            heading = NormalizeFlat(PlayerTurn * startHeading, startHeading);

            SetPlayerHeadingRotation(playerTarget,PlayerTurn * playerStartRotation);
            ApplyStageAndPlayerOrbit(stageTurn, canRotateStage, playerIsStageChild, pivot, stageStartPosition, stageStartRotation, playerStartPosition, cloneStartPosition);
        }

        turnTween = DOTween
            .To(() => 0f, Apply, playerAngle, turnDuration)
            .SetEase(turnEase)
            .SetUpdate(UpdateType.Fixed)
            .OnComplete(() =>
            {
                Apply(playerAngle);

                Quaternion turnRotation =
                    Quaternion.AngleAxis(
                        playerAngle,
                        Vector3.up
                    );

                heading = NormalizeFlat(
                    turnRotation * startHeading,
                    startHeading
                );

                SetPlayerHeadingRotation(
                    playerTarget,
                    playerEndRotation
                );

                // 回転前の速度を、回転後の進行方向へ一度だけ合わせる
                if (rb)
                    rb.velocity =
                        turnRotation * velocityBeforeTurn;
            });
    }
    void ApplyStageAndPlayerOrbit(
        Quaternion stageTurn,
        bool canRotateStage,
        bool playerIsStageChild,
        Vector3 pivot,
        Vector3 stageStartPosition,
        Quaternion stageStartRotation,
        Vector3 playerStartPosition,
        Vector3 cloneStartPosition)
    {
        if (!canRotateStage)
            return;

        stageRoot.SetPositionAndRotation(
            pivot + stageTurn * (stageStartPosition - pivot),
            stageTurn * stageStartRotation
        );

        if (movePlayerAroundStagePivot && !playerIsStageChild)
        {
            Vector3 playerPosition =
                pivot + stageTurn * (playerStartPosition - pivot);

            if (rb && !rb.isKinematic)
                rb.MovePosition(playerPosition);
            else if (rb)
                rb.position = playerPosition;
            else
                transform.position = playerPosition;
        }

        if (rbClone)
        {
            Vector3 clonePosition =
                pivot + stageTurn * (cloneStartPosition - pivot);

            if (!rbClone.isKinematic)
                rbClone.MovePosition(clonePosition);
            else
                rbClone.position = clonePosition;
        }
    }

    void SetPlayerHeadingRotation(
        Transform playerTarget,
        Quaternion rotation)
    {
        if (playerTarget == transform &&
            rb &&
            !rb.isKinematic)
        {
            rb.MoveRotation(rotation);
        }
        else if (playerTarget)
        {
            playerTarget.rotation = rotation;
        }
    }

#endif

    /// <summary>
    /// フリック角度を次のFixedUpdateへ予約します。
    /// InSubjectの姿勢は回しませんが、headingと水平速度は旋回させます。
    /// </summary>
    public void BeginPlayerAndStageTurn(float playerAngle)
    {
        float clampedAngle = Mathf.Clamp(playerAngle, -180f, 180f);

        if (Mathf.Abs(clampedAngle) <= 0.0001f)
            return;

        pendingFlickTurnDegrees = clampedAngle;

        if (writeRuntimeLog)
        {
            Debug.Log(
                $"[FLICK TURN QUEUED] time={Time.fixedTime:F4}s " +
                $"inputAngle={clampedAngle:F3} physicsHeading={heading:F5} " +
                $"inSubjectVelocity={(rb ? rb.velocity.ToString("F5") : "N/A")}",
                this
            );
        }
    }

    /// <summary>
    /// InSubjectのTransform/Rigidbody姿勢は変更せず、
    /// 進行基準headingとvelocityの水平成分だけを旋回させます。
    /// </summary>
    void ApplyFlickDirectionTurnToInSubject(float inputAngle)
    {
        Quaternion directionTurn =
            Quaternion.AngleAxis(inputAngle, Vector3.up);

        Vector3 headingBefore =
            NormalizeFlat(heading, initialHeading);

        heading = NormalizeFlat(
            directionTurn * headingBefore,
            headingBefore
        );

        if (!rb)
        {
            Debug.LogError(
                "[INSUBJECT DIRECTION TURN FAILED] Rigidbodyが未設定です。",
                this
            );
            return;
        }

        if (rb.isKinematic)
        {
            Debug.LogError(
                "[INSUBJECT DIRECTION TURN FAILED] " +
                "InSubjectのRigidbodyがKinematicです。Dynamicにしてください。",
                rb
            );
            return;
        }

        Vector3 velocityBefore = rb.velocity;
        float verticalSpeed = Vector3.Dot(velocityBefore, Vector3.up);
        Vector3 planarVelocity =
            velocityBefore - Vector3.up * verticalSpeed;

        if (planarVelocity.sqrMagnitude > epsilon * epsilon)
            planarVelocity = directionTurn * planarVelocity;

        rb.velocity =
            planarVelocity + Vector3.up * verticalSpeed;

        rb.WakeUp();

        if (writeRuntimeLog)
        {
            Debug.Log(
                $"[INSUBJECT DIRECTION TURN APPLIED] time={Time.fixedTime:F4}s " +
                $"angle={inputAngle:F3} " +
                $"headingBefore={headingBefore:F5} headingAfter={heading:F5} " +
                $"velocityBefore={velocityBefore:F5} velocityAfter={rb.velocity:F5} " +
                $"speedBefore={velocityBefore.magnitude:F5} " +
                $"speedAfter={rb.velocity.magnitude:F5}",
                rb
            );
        }
    }

    /// <summary>
    /// Updateで予約されたフリックを、FixedUpdateの先頭で一度だけ取り出します。
    /// Rigidbodyの速度変更を描画フレーム側で行わないための受け渡し関数です。
    /// </summary>
    void ApplyPendingFlickTurn()
    {
        float inputAngle = pendingFlickTurnDegrees;

        if (Mathf.Abs(inputAngle) <= 0.0001f)
            return;

        pendingFlickTurnDegrees = 0f;
        ApplyFlickTurn(inputAngle);
    }

    /// <summary>
    /// FixedUpdateからだけ呼ばれる、フリック旋回の実処理です。
    /// 1. InSubjectのheadingと水平速度を入力方向へ旋回します。
    /// 2. 旧方向に依存する斜面予測を破棄します。
    /// 3. VisualPlayerRootを入力と反対方向へ回し、ステージが回って見えるようにします。
    ///
    /// InSubjectのposition / rotation / angularVelocityと、カメラのrotationは変更しません。
    /// </summary>
    void ApplyFlickTurn(float inputAngle)
    {
        if (Mathf.Abs(inputAngle) <= 0.0001f)
            return;

        flickTurnAppliedThisFixedUpdate = true;

        Vector3 inSubjectPositionBefore =
            rb ? rb.position : Vector3.zero;

        Quaternion inSubjectRotationBefore =
            rb ? rb.rotation : Quaternion.identity;

        Vector3 inSubjectVelocityBefore =
            rb ? rb.velocity : Vector3.zero;

        Vector3 inSubjectAngularBefore =
            rb ? rb.angularVelocity : Vector3.zero;

        Vector3 headingBefore = heading;
        float verticalSpeedBefore =
            Vector3.Dot(inSubjectVelocityBefore, Vector3.up);

        ApplyFlickDirectionTurnToInSubject(inputAngle);
        InvalidateDirectionDependentSlopePrediction();
        
        ResetNaturalArtificialRelease(
            "WorldDirectionTurn",
            true
        );

        FindCoordinateFrameReferences();
        BindCoordinateFrames();

        float visualAngle =
            inputAngle *
            (visualRootTurnsOppositeToInput ? -1f : 1f);

        bool visualRotated = false;

        if (visualPlayerRoot)
        {
            Quaternion visualTurn =
                Quaternion.AngleAxis(visualAngle, Vector3.up);

            Vector3 pivot = visualRotationPivot
                ? visualRotationPivot.position
                : visualPlayerRoot.position;

            visualRotated = correspondSubject
                ? correspondSubject.RotateVisualFrameAround(
                    pivot,
                    visualTurn,
                    true
                )
                : RotateVisualRootWithoutSubject(
                    pivot,
                    visualTurn
                );
        }
        else
        {
            Debug.LogWarning(
                "[VISUAL FRAME TURN] VisualPlayerRootは未設定ですが、" +
                "InSubjectの進行方向変更は適用済みです。",
                this
            );
        }

        // BallCameraFollowはSubjectの位置だけを追います。
        // ここでcameraTransform.rotationを変更すると、ステージ回転と相殺されます。

        Vector3 inSubjectVelocityAfter =
            rb ? rb.velocity : Vector3.zero;

        float verticalSpeedAfter =
            Vector3.Dot(inSubjectVelocityAfter, Vector3.up);

        if (writeRuntimeLog)
        {
            Debug.Log(
                $"[FLICK TURN APPLIED] time={Time.fixedTime:F4}s " +
                $"inputAngle={inputAngle:F3} visualAngle={visualAngle:F3} " +
                $"visualRotated={visualRotated} " +
                $"physicsRootRotation={(physicsRoot ? physicsRoot.rotation.eulerAngles.ToString("F3") : "N/A")} " +
                $"visualRootRotation={(visualPlayerRoot ? visualPlayerRoot.rotation.eulerAngles.ToString("F3") : "N/A")} " +
                $"inSubjectPositionUnchanged={Approximately(inSubjectPositionBefore, rb ? rb.position : Vector3.zero)} " +
                $"inSubjectRotationUnchanged={Approximately(inSubjectRotationBefore, rb ? rb.rotation : Quaternion.identity)} " +
                $"inSubjectAngularUnchanged={Approximately(inSubjectAngularBefore, rb ? rb.angularVelocity : Vector3.zero)} " +
                $"headingChanged={!Approximately(headingBefore, heading)} " +
                $"velocityChanged={!Approximately(inSubjectVelocityBefore, inSubjectVelocityAfter)} " +
                $"verticalSpeedPreserved={Mathf.Abs(verticalSpeedBefore - verticalSpeedAfter) <= 0.0001f}",
                this
            );
        }
    }

    bool RotateVisualRootWithoutSubject(
        Vector3 pivot,
        Quaternion worldTurn)
    {
        if (!visualPlayerRoot)
            return false;

        Vector3 relative = visualPlayerRoot.position - pivot;

        visualPlayerRoot.SetPositionAndRotation(
            pivot + worldTurn * relative,
            worldTurn * visualPlayerRoot.rotation
        );

        return true;
    }

    static bool Approximately(Vector3 a, Vector3 b)
    {
        return (a - b).sqrMagnitude <= 0.00000001f;
    }

    static bool Approximately(Quaternion a, Quaternion b)
    {
        return Quaternion.Angle(a, b) <= 0.0001f;
    }

    void InvalidateDirectionDependentSlopePrediction()
    {
        // A frame detected from the former heading must not be reused after turn.
        forwardSlopeFrame = default;
        hasForwardSlope = false;
        anticipatedSlopeEntryCollider = null;
        slopeEntryPredictiveBrakeAcceleration = 0f;
        slopeEntryPredictiveTargetSpeed = 0f;
        previousSlopeEntryPredictiveBrakeAcceleration = 0f;
        slopeEntryBrakeLoggedCollider = null;
        slopeEntryBrakeLogged = false;
        slopeEntryVelocityTransportCollider = null;
        adaptiveEntryStickPreloadCollider = null;

        // Do not carry tangent acceleration/Jerk from the old direction.
        currentTangentialAcceleration = 0f;
        previousAppliedTangentialAcceleration = 0f;
        previousTargetProgressAppliedAcceleration = 0f;
        previousOverspeedGovernorAcceleration = 0f;

        // The requested use case turns on a flat section. Clear the old slope
        // session there, then let this same FixedUpdate detect the new forward
        // slope from the updated heading.
        if (groundKind != GroundKind.Slope)
            ClearSlopeTracking("WorldDirectionTurn");
    }

    bool TryApplyTrustedBaselineExactly(ReliableBaselineSampler.CyclePieceType pieceType)
    {
        // フリック適用と同じFixedUpdateでは、旧ワールド速度を復元しない。
        if (flickTurnAppliedThisFixedUpdate)
            return false;

        if (!replayTrustedBaselineExactly || !reliableBaselineSampler || !reliableBaselineSampler.HasTrustedCycle || !rb)
        {
            return false;
        }

        if (!reliableBaselineSampler.TryGetTrustedPiece(pieceType, out ReliableBaselineSampler.BaselineResult saved) || !saved.valid)
        {
            return false;
        }

        // 「そのまま」の再現試験なので、現在面への再投影や補間は行わない。
        // JSONへ保存されたワールド空間の速度・角速度を完全に同じ値で適用する。
        rb.velocity = saved.velocity;
        //rb.angularVelocity = saved.angularVelocity;

        if (logTrustedBaselineReplay)
        {
            Debug.Log($"[BASELINE JSON REPLAY] type={pieceType} " + $"time={Time.fixedTime:F4}s " + $"velocity={saved.velocity:F5} " + $"angular={saved.angularVelocity:F5}", this);
        }

        return true;
    }

    void ApplyInitialGroundedRestReplayIfNeeded(bool grounded)
    {
        if (initialGroundedRestReplayApplied || !grounded || !currentGroundObservation.valid)
        {
            return;
        }

        if (TryApplyTrustedBaselineExactly(ReliableBaselineSampler.CyclePieceType.InitialGroundedRest))
        {
            initialGroundedRestReplayApplied = true;
        }
    }

    bool UpdateReliableBaselineSampling(bool grounded)
    {
        if (!reliableBaselineSampler || baselineInitialRestReady)
            return false;

        Collider sourceCollider = grounded && currentGroundObservation.valid ? currentGroundObservation.collider : null;

        Vector3 surfaceNormal = grounded && currentGroundObservation.valid ? currentGroundObservation.normal : Vector3.up;

        bool stillSampling = reliableBaselineSampler.Tick(rb, grounded, surfaceNormal, heading, sourceCollider, out ReliableBaselineSampler.BaselineResult result);

        if (result.valid)
        {
            baselineInitialRestReady = true;
            previousBaselineNormal = result.normal;
            hasPreviousBaselineNormal = true;
        }

        return pauseControllerUntilInitialRest && stillSampling;
    }

    void CaptureAbruptNormalChangeIfNeeded(bool grounded)
    {
        if (!reliableBaselineSampler || !baselineInitialRestReady || !grounded || !currentGroundObservation.valid)
        {
            if (!grounded)
                hasPreviousBaselineNormal = false;

            return;
        }

        Vector3 currentNormal = currentGroundObservation.normal.normalized;

        if (!hasPreviousBaselineNormal)
        {
            previousBaselineNormal = currentNormal;
            hasPreviousBaselineNormal = true;
            return;
        }

        float normalChange = Vector3.Angle(previousBaselineNormal, currentNormal);

        if (normalChange >= baselineAbruptNormalAngle)
        {
            if (reliableBaselineSampler.HasTrustedCycle)
            {
                if (!abruptNormalReplayApplied && TryApplyTrustedBaselineExactly(ReliableBaselineSampler.CyclePieceType.AbruptNormalChange))
                {
                    abruptNormalReplayApplied = true;
                }
            }
            else
            {
                reliableBaselineSampler.CaptureCyclePiece(ReliableBaselineSampler.CyclePieceType.AbruptNormalChange, rb, true, previousBaselineNormal, heading,
                    currentGroundObservation.collider);
            }
        }

        previousBaselineNormal = currentNormal;
    }

    /// <summary>
    /// DOTween開始直前に呼び出す3点Baseline用フックです。
    /// 既存のDOTween回転メソッドの先頭へこの呼び出しを1行追加できます。
    /// </summary>
    public bool CaptureBeforeDOTweenTurnBaseline()
    {
        if (!reliableBaselineSampler || !baselineInitialRestReady || !rb)
        {
            return false;
        }

        if (reliableBaselineSampler.HasTrustedCycle)
        {
            if (beforeDOTweenTurnReplayApplied)
                return false;

            bool applied = TryApplyTrustedBaselineExactly(ReliableBaselineSampler.CyclePieceType.BeforeDOTweenTurn);

            if (applied)
                beforeDOTweenTurnReplayApplied = true;

            return applied;
        }

        Collider sourceCollider = currentGroundObservation.valid ? currentGroundObservation.collider : null;

        return reliableBaselineSampler.CaptureCyclePiece(ReliableBaselineSampler.CyclePieceType.BeforeDOTweenTurn, rb, groundKind != GroundKind.Air, groundNormal, heading,
            sourceCollider);
    }

    void FixedUpdate()
    {
        float dt = Mathf.Max(Time.fixedDeltaTime, 0.000001f);
        fixedFrameCounter++;
        flickTurnAppliedThisFixedUpdate = false;

        ValidatePhysicsRootInvariant();

        // Updateで予約したフリックを、物理観測より先に一度だけ適用します。
        // InSubjectの姿勢・PhysicsRoot・カメラは回転させません。
        ApplyPendingFlickTurn();

        slopeEntryVelocityTransportAppliedThisFrame = false;
        slopeEntryTransportDeltaVelocity = Vector3.zero;
        adaptiveEntryStickPreloadAppliedThisFrame = false;
        
        bool measuredGrounded = ProbeGround(out GroundObservation measuredObservation);
        
        currentGroundObservation = measuredObservation;
        slopeEntrySupportBridgeActive = false;
        slopeEntrySupportBridgeGap = 0f;
        

        // SphereCast / CollisionContactが1 FixedUpdateだけ欠落しても、
        // 直前と同じ斜面・入口25%未満・近接状態なら支持面を時間方向に補間します。
        if (!measuredGrounded && TryBuildSlopeEntrySupportBridge(consecutiveGroundMissFrames + 1, dt, out GroundObservation bridgeObservation))
        {
            currentGroundObservation = bridgeObservation;
            slopeEntrySupportBridgeActive = true;
        }

        bool grounded = measuredGrounded || slopeEntrySupportBridgeActive;
        UpdateSupportObservationDebug(grounded, currentGroundObservation);

        // 完成済みJSONを読み込んだ再現試験では、最初の有効接地フレームで
        // InitialGroundedRestの保存値を一度だけ、そのまま適用する。
        ApplyInitialGroundedRestReplayIfNeeded(grounded);

        if (UpdateReliableBaselineSampling(grounded))
        {
            wasGrounded = grounded;
            return;
        }

        CaptureAbruptNormalChangeIfNeeded(grounded);

        if (grounded)
        {
            // 実測接地が戻った時だけMiss数を0へ戻します。Bridge中は1のまま保持します。
            consecutiveGroundMissFrames = measuredGrounded ? 0 : consecutiveGroundMissFrames + 1;
            groundNormal = currentGroundObservation.normal.normalized;
            float angle = Vector3.Angle(groundNormal, Vector3.up);
            groundKind = angle >= minimumSlopeAngle ? GroundKind.Slope : GroundKind.Flat;
        }
        else
        {
           /* if (rb.velocity.y < -25f)
            {
                StartCoroutine(DelayStart());
            }*/
            
            consecutiveGroundMissFrames++;
            groundNormal = Vector3.up;
            groundKind = GroundKind.Air;

            int effectiveSlopeTrackingGraceFrames = Mathf.Max(0, slopeTrackingGraceFrames);

            if (consecutiveGroundMissFrames <= effectiveSlopeTrackingGraceFrames)
            {
                if (writeRuntimeLog && logSupportSurfaceLatch)
                {
                    Debug.Log($"[SLOPE TRACKING GRACE] " + $"time={Time.fixedTime:F4}s " + $"missFrames={consecutiveGroundMissFrames} " +
                        $"graceFrames={effectiveSlopeTrackingGraceFrames} " + $"graceSeconds={(effectiveSlopeTrackingGraceFrames * dt):F6} " +
                        $"trackedSlope={(trackedSlopeCollider ? trackedSlopeCollider.name : "None")} " + $"ground=Air supportBridge=false", this);
                }
            }
            else
            {
                ClearSlopeTracking("GroundObservationLost");
            }
        }

        if (wasGrounded && !grounded)
            sessionGroundToAirCount++;

        Vector3 desiredMove = GetDesiredMoveWorld();

        if (grounded)
        {
            if (groundKind == GroundKind.Flat)
            {
                UpdateForwardSlopeFromFlat(desiredMove, currentGroundObservation.normal);
            }
            else
            {
                UpdateCurrentSlopeFrame(currentGroundObservation, desiredMove);

                if (measuredGrounded)
                    RememberMeasuredSlopeSupport(currentGroundObservation);
            }

            UpdateLocalFlatToSlopeSequencePhase();
            ApplySlopeEntryVelocityTransportOnce();
            SolveGround(desiredMove, dt);
        }
        else
        {
            localFlatToSlopeSequencePhase = LocalFlatToSlopeSequencePhase.Inactive;
            slopeEntryPredictiveBrakeAcceleration = 0f;
            slopeEntryPredictiveTargetSpeed = 0f;
            previousSlopeEntryPredictiveBrakeAcceleration = 0f;
            SolveAir(desiredMove, dt);
            criticalStateMaintained = false;
            criticalMaintainedSeconds = 0f;
        }

        UpdateSlopeProgressObservation(grounded, currentGroundObservation, dt);
        UpdateLimitPointSphere();
        UpdateCanonicalDiagnostics(dt);

        wasGrounded = grounded;
    }

    public void ResetBallToStart()
    {
        if (!startTransform || !rb)
            return;

        rb.position = ResolvePhysicsPoint(startTransform);
        rb.rotation = ResolvePhysicsRotation(startTransform);
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        heading = NormalizeFlat(
            initialHeading,
            ResolvePhysicsRotation(startTransform) * Vector3.forward
        );
        hasPreviousBaselineNormal = false;
        initialGroundedRestReplayApplied = false;
        abruptNormalReplayApplied = false;
        beforeDOTweenTurnReplayApplied = false;
        pendingFlickTurnDegrees = 0f;
        slopeFrame = default;
        forwardSlopeFrame = default;
        trackedSlopeCollider = null;
        currentGroundObservation = default;
        latestCollisionGroundObservation = default;
        latestCollisionContactFixedTime = float.NegativeInfinity;
        latestCollisionGroundScore = float.NegativeInfinity;
        latestCollisionScoreFixedTime = float.NegativeInfinity;
        lastMeasuredSlopeObservation = default;
        lastMeasuredSlopeProgress = 0f;
        localFlatToSlopeSequencePhase = LocalFlatToSlopeSequencePhase.Inactive;
        slopeEntrySupportBridgeActive = false;
        slopeEntrySupportBridgeGap = 0f;
        slopeEntryPredictiveBrakeAcceleration = 0f;
        slopeEntryPredictiveTargetSpeed = 0f;
        previousSlopeEntryPredictiveBrakeAcceleration = 0f;
        slopeEntryBrakeLoggedCollider = null;
        slopeEntryBrakeLogged = false;
        anticipatedSlopeEntryCollider = null;
        slopeEntryVelocityTransportCollider = null;
        adaptiveEntryStickPreloadCollider = null;
        slopeEntryVelocityTransportAppliedThisFrame = false;
        slopeEntryOutwardSpeedBeforeTransport = 0f;
        slopeEntryOutwardSpeedAfterTransport = 0f;
        slopeEntryTransportDeltaVelocity = Vector3.zero;
        adaptiveEntryStickPreloadAppliedThisFrame = false;
        adaptiveEntryStickPreloadState = 0f;
        consecutiveGroundMissFrames = 0;
        currentTangentialAcceleration = 0f;
        previousAppliedTangentialAcceleration = 0f;
        currentAppliedArtificialAcceleration = Vector3.zero;
        artificialControlPhase = ArtificialControlPhase.Uncontrolled;
        criticalMaintainedSeconds = 0f;
        criticalStateMaintained = false;
        ResetSlopeProgressObservation();
        ResetTargetProgressPreconditioning("BallReset", false);
        ResetNaturalArtificialRelease("BallReset", false);

        if (resetVisualFrameOnBallReset &&
            visualPlayerRoot &&
            hasInitialVisualFramePose)
        {
            visualPlayerRoot.SetPositionAndRotation(
                initialVisualPlayerRootPosition,
                initialVisualPlayerRootRotation
            );
        }

        Physics.SyncTransforms();
        correspondSubject?.SynchronizeNow(true);
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.transform.CompareTag("stairway"))
        {
            Debug.Log("");
        }
        if (collision.transform.CompareTag("plane"))
        {
            Debug.Log("");
        }
       
        int slopeLayer = LayerMask.NameToLayer("Slope");

        if (collision.gameObject.layer == slopeLayer)
        {
            Debug.Log("Slopeレイヤーに衝突しました");
        }
        
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

            if (!candidate || candidate.attachedRigidbody == rb || candidate.isTrigger ||
                (groundMask.value & (1 << candidate.gameObject.layer)) == 0)
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

            if (IsSameLogicalSlope(candidate, trackedSlopeCollider))
                score += 4f;
            if (slopeFrame.valid &&
                IsSameLogicalSlope(candidate, slopeFrame.collider))
                score += 3f;
            if (forwardSlopeFrame.valid &&
                IsSameLogicalSlope(candidate, forwardSlopeFrame.collider))
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

        observation = new GroundObservation
        {
            valid = true,
            collider = hit.collider,
            point = hit.point,
            normal = normal,
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

    void UpdateSupportObservationDebug(bool grounded, GroundObservation observation)
    {

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
        slopeProgressStatePercent = 0f;
        lastMeasuredSlopeObservation = default;
        lastMeasuredSlopeProgress = 0f;
        localFlatToSlopeSequencePhase = LocalFlatToSlopeSequencePhase.Inactive;
        slopeEntrySupportBridgeActive = false;
        slopeEntrySupportBridgeGap = 0f;
        slopeEntryPredictiveBrakeAcceleration = 0f;
        slopeEntryPredictiveTargetSpeed = 0f;
        previousSlopeEntryPredictiveBrakeAcceleration = 0f;
        slopeEntryBrakeLoggedCollider = null;
        slopeEntryBrakeLogged = false;
        anticipatedSlopeEntryCollider = null;
        slopeEntryVelocityTransportCollider = null;
        adaptiveEntryStickPreloadCollider = null;
        ResetSlopeProgressObservation();

        if (hadTracking && writeRuntimeLog && logSupportSurfaceLatch)
        {
            Debug.Log($"[SLOPE TRACKING CLEARED] time={Time.fixedTime:F4}s " + $"reason={reason}", this);
        }
    }

    bool TryBuildSlopeEntrySupportBridge(int nextMissFrames, float dt, out GroundObservation observation)
    {
        observation = default;

        if (!useLocalFlatToSlopeSequencePatch || nextMissFrames != LocalSlopeEntryBridgeMaximumMissFrames || !wasGrounded ||
            localFlatToSlopeSequencePhase != LocalFlatToSlopeSequencePhase.SlopeEntrySettling || naturalMotionReleased ||
            !slopeFrame.valid || !slopeFrame.collider || !trackedSlopeCollider ||
            !IsSameLogicalSlope(trackedSlopeCollider, slopeFrame.collider) ||
            !lastMeasuredSlopeObservation.valid ||
            !IsSameLogicalSlope(lastMeasuredSlopeObservation.collider, slopeFrame.collider))
        {
            return false;
        }

        float progress = CalculateProgress(slopeFrame, rb.position);
        if (progress >= LocalSlopeEntrySettlingEndProgress)
            return false;

        Vector3 normal = lastMeasuredSlopeObservation.normal.normalized;
        if (normal.sqrMagnitude < 0.999f)
            return false;

        float forwardSpeed = Vector3.Dot(rb.velocity, slopeFrame.axis);
        if (forwardSpeed <= 0.01f)
            return false;

        Vector3 center = rb.worldCenterOfMass;
        float centerPlaneDistance = Vector3.Dot(center - lastMeasuredSlopeObservation.point, normal);
        float gap = Mathf.Max(0f, centerPlaneDistance - sphereRadius);
        float outwardSpeed = Mathf.Max(0f, Vector3.Dot(rb.velocity, normal));

        // 1 FixedUpdate先に外向き速度で増える距離までを、同じ面の観測欠落として許容します。
        float maximumBridgeGap = Mathf.Max(0.02f, groundProbeDistance + outwardSpeed * dt);
        if (gap > maximumBridgeGap)
            return false;

        observation = new GroundObservation
        {
            valid = true,
            collider = slopeFrame.collider,
            point = center - normal * centerPlaneDistance,
            normal = normal,
            source = GroundObservationSource.SlopeEntryBridge
        };

        slopeEntrySupportBridgeGap = gap;
        return true;
    }

    void RememberMeasuredSlopeSupport(GroundObservation observation)
    {
        if (!observation.valid ||
            !observation.collider ||
            !slopeFrame.valid ||
            !IsSameLogicalSlope(observation.collider, slopeFrame.collider))
        {
            return;
        }

        lastMeasuredSlopeObservation = observation;
        lastMeasuredSlopeProgress = CalculateProgress(slopeFrame, rb.position);
    }

    void UpdateLocalFlatToSlopeSequencePhase()
    {
        if (!useLocalFlatToSlopeSequencePatch)
        {
            localFlatToSlopeSequencePhase = LocalFlatToSlopeSequencePhase.Inactive;
            return;
        }

        if (groundKind == GroundKind.Flat && hasForwardSlope && forwardSlopeFrame.valid)
        {
            localFlatToSlopeSequencePhase = LocalFlatToSlopeSequencePhase.FlatApproach;
            return;
        }

        if (groundKind == GroundKind.Slope && slopeFrame.valid)
        {
            float progress = CalculateProgress(slopeFrame, rb.position);
            localFlatToSlopeSequencePhase = progress < LocalSlopeEntrySettlingEndProgress
                ? LocalFlatToSlopeSequencePhase.SlopeEntrySettling
                : LocalFlatToSlopeSequencePhase.NormalSlope;
            return;
        }

        localFlatToSlopeSequencePhase = LocalFlatToSlopeSequencePhase.Inactive;
    }

    void ApplySlopeEntryVelocityTransportOnce()
    {
        if (!useSlopeEntryVelocityTransport ||
            !rb ||
            groundKind != GroundKind.Slope ||
            !slopeFrame.valid ||
            !slopeFrame.collider ||
            IsSameLogicalSlope(slopeEntryVelocityTransportCollider, slopeFrame.collider) ||
            !IsSameLogicalSlope(anticipatedSlopeEntryCollider, slopeFrame.collider))
        {
            return;
        }

        float progress = CalculateProgress(slopeFrame, rb.position);
        if (progress > Mathf.Clamp(slopeEntryControlMaximumProgress, 0.01f, 0.25f))
            return;

        // 境界の接触法線はFlat寄りへ揺れる場合があるため、
        // SlopeFrame内部を少しだけ進めた点から安定した斜面法線を取得します。
        float sampleProgress = Mathf.Clamp01(
            Mathf.Max(progress, Mathf.Min(0.02f, slopeEntryControlMaximumProgress))
        );

        if (!TryEvaluateSurface(slopeFrame, sampleProgress, out SurfaceSample entrySample))
            return;

        Vector3 entryNormal = entrySample.normal.normalized;
        Vector3 entryTangent = entrySample.tangent.normalized;

        if (Vector3.Dot(entryTangent, slopeFrame.axis) < 0f)
            entryTangent = -entryTangent;

        Vector3 velocityBefore = rb.velocity;
        float forwardSpeed = Vector3.Dot(velocityBefore, entryTangent);

        // 逆走・静止・斜面から内側へ向かっている場合には輸送しません。
        if (forwardSpeed <= 0.01f)
            return;

        float outwardSpeed = Mathf.Max(
            0f,
            Vector3.Dot(velocityBefore, entryNormal)
        );

        float naturalBounceSpeed = Mathf.Max(
            0f,
            slopeEntryNaturalBounceSpeed
        );

        // 閾値以下は演出として残します。閾値を超えた分だけ、
        // transportWeightに従って一回だけ接平面側へ戻します。
        if (outwardSpeed <= naturalBounceSpeed + 0.0001f)
            return;

        float transportWeight = Mathf.Clamp01(
            slopeEntryVelocityTransportWeight
        );

        float retainedOutwardSpeed = Mathf.Min(
            outwardSpeed,
            Mathf.Max(
                naturalBounceSpeed,
                outwardSpeed * (1f - transportWeight)
            )
        );

        float removedOutwardSpeed =
            outwardSpeed - retainedOutwardSpeed;

        Vector3 deltaVelocity =
            -entryNormal * removedOutwardSpeed;

        // 連続制御力ではなく、Flat座標系からSlope接平面座標系への
        // 境界一回だけの速度状態輸送です。
        rb.velocity = velocityBefore + deltaVelocity;

        slopeEntryVelocityTransportCollider =
            slopeFrame.collider;

        slopeEntryVelocityTransportAppliedThisFrame = true;
        slopeEntryOutwardSpeedBeforeTransport = outwardSpeed;
        slopeEntryOutwardSpeedAfterTransport =
            Mathf.Max(0f, Vector3.Dot(rb.velocity, entryNormal));
        slopeEntryTransportDeltaVelocity = deltaVelocity;

        if (writeRuntimeLog && logSlopeEntryTransportAndPreload)
        {
            Debug.Log(
                $"[SLOPE ENTRY VELOCITY TRANSPORT] " +
                $"time={Time.fixedTime:F4}s " +
                $"collider={slopeFrame.collider.name} " +
                $"progress={progress * 100f:F3}% " +
                $"normal={entryNormal:F5} " +
                $"before={velocityBefore:F5} " +
                $"after={rb.velocity:F5} " +
                $"outwardBefore={slopeEntryOutwardSpeedBeforeTransport:F6} " +
                $"outwardAfter={slopeEntryOutwardSpeedAfterTransport:F6} " +
                $"deltaV={deltaVelocity:F5}",
                this
            );
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
            anticipatedSlopeEntryCollider = detectedFrame.collider;
            activeSlopeName = slopeHit.collider
                ? GetLogicalSlopeKey(slopeHit.collider)
                : string.Empty;
        }
        else
        {
            forwardSlopeFrame = default;
            anticipatedSlopeEntryCollider = null;
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

            if (forwardSlopeFrame.valid &&
                IsSameLogicalSlope(forwardSlopeFrame.collider, hit.collider))
            {
                slopeHit = hit;
                detectedFrame = forwardSlopeFrame;
                return true;
            }

            if (!BuildSlopeFrame(hit.collider, hit.normal, forward, out detectedFrame))
            {
                continue;
            }

            slopeHit = hit;
            return true;
        }

        return false;
    }

    static string GetLogicalSlopeKey(Collider slopeCollider)
    {
        if (!slopeCollider)
            return string.Empty;

        // StairWay6_0_Physics ～ StairWay6_4_Physics -> StairWay6_Physics
        // scale=1 の _0_Physics も同じ規則で一つの論理斜面になります。
        return Regex.Replace(
            slopeCollider.name,
            @"_\d+_Physics$",
            "_Physics"
        );
    }

    static bool IsSameLogicalSlope(Collider first, Collider second)
    {
        if (!first || !second)
            return false;

        if (first == second)
            return true;

        string firstKey = GetLogicalSlopeKey(first);
        string secondKey = GetLogicalSlopeKey(second);

        return !string.IsNullOrEmpty(firstKey) &&
               firstKey == secondKey;
    }

    Transform ResolveLogicalSlopeSearchRoot(Collider slopeCollider)
    {
        if (!slopeCollider)
            return null;

        if (curvatureTrackRoot &&
            (slopeCollider.transform == curvatureTrackRoot ||
             slopeCollider.transform.IsChildOf(curvatureTrackRoot)))
        {
            return curvatureTrackRoot;
        }

        Transform current = slopeCollider.transform;
        while (current)
        {
            if (current.name == "__GeneratedPhysics")
                return current;

            current = current.parent;
        }

        return slopeCollider.transform.parent
            ? slopeCollider.transform.parent
            : slopeCollider.transform;
    }

    bool TryGetLogicalSlopeProjectedRange(
        Collider slopeCollider,
        Vector3 axis,
        out float minimum,
        out float maximum,
        out Vector3 axisOrigin)
    {
        minimum = float.PositiveInfinity;
        maximum = float.NegativeInfinity;
        axisOrigin = Vector3.zero;

        if (!slopeCollider || axis.sqrMagnitude < 0.999f)
            return false;

        Vector3 referenceCenter = slopeCollider.bounds.center;
        float referenceCoordinate = Vector3.Dot(referenceCenter, axis);
        axisOrigin = referenceCenter - axis * referenceCoordinate;

        Transform searchRoot = ResolveLogicalSlopeSearchRoot(slopeCollider);
        Collider[] candidates = searchRoot
            ? searchRoot.GetComponentsInChildren<Collider>(true)
            : new[] { slopeCollider };

        bool found = false;

        for (int i = 0; i < candidates.Length; i++)
        {
            Collider candidate = candidates[i];

            if (!candidate ||
                !candidate.enabled ||
                candidate.isTrigger ||
                !IsSameLogicalSlope(slopeCollider, candidate))
            {
                continue;
            }

            Bounds bounds = candidate.bounds;
            Vector3 extents = bounds.extents;
            float centerCoordinate = Vector3.Dot(bounds.center, axis);
            float projectedHalfExtent =
                Mathf.Abs(axis.x) * extents.x +
                Mathf.Abs(axis.y) * extents.y +
                Mathf.Abs(axis.z) * extents.z;

            minimum = Mathf.Min(
                minimum,
                centerCoordinate - projectedHalfExtent
            );

            maximum = Mathf.Max(
                maximum,
                centerCoordinate + projectedHalfExtent
            );

            found = true;
        }

        return found &&
               !float.IsInfinity(minimum) &&
               !float.IsInfinity(maximum) &&
               maximum > minimum + 0.0001f;
    }

    void UpdateCurrentSlopeFrame(GroundObservation observation, Vector3 desiredMove)
    {
        if (!observation.valid || !observation.collider)
            return;

        Collider contactedCollider = observation.collider;

        if (slopeFrame.valid &&
            IsSameLogicalSlope(slopeFrame.collider, contactedCollider))
        {
            trackedSlopeCollider = contactedCollider;
            activeSlopeName = GetLogicalSlopeKey(contactedCollider);
            return;
        }

        if (forwardSlopeFrame.valid &&
            IsSameLogicalSlope(forwardSlopeFrame.collider, contactedCollider))
        {
            slopeFrame = forwardSlopeFrame;
            trackedSlopeCollider = contactedCollider;
            activeSlopeName = GetLogicalSlopeKey(contactedCollider);

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
            activeSlopeName = GetLogicalSlopeKey(contactedCollider);
        }
        else
        {
        }
    }

    bool BuildSlopeFrame(Collider slopeCollider, Vector3 referenceNormalInput, Vector3 desiredForward, out SlopeFrame frame)
    {
        frame = default;

        if (!slopeCollider)
            return false;

        // _0_Physics ～ _N_Physics が同じ論理名を持つ場合は、
        // それら全部を「一つの斜面」として扱います。
        // 進捗0%は論理斜面の最初のEntry、100%は最後のExitです。

        Vector3 referenceNormal = referenceNormalInput.normalized;
        if (referenceNormal.sqrMagnitude < 0.999f)
            return false;

        // slopeProgressStatePercent は、論理斜面全体の入口0%～出口100%を表します。
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

        // ここで1枚のboundsではなく、同じ論理斜面に属する
        // _0_Physics ～ _N_Physics 全体の投影範囲を取得します。
        if (!TryGetLogicalSlopeProjectedRange(
                slopeCollider,
                axis,
                out float minimum,
                out float maximum,
                out Vector3 axisOrigin))
        {
            return false;
        }

        float length = maximum - minimum;

        if (length <= 0.0001f)
            return false;

        // axisOrigin は現在Colliderの中心を通るaxis直交面上の基準点です。
        // 同じ論理斜面の各Colliderはaxis方向に並ぶため、
        // minimum / maximumだけを全体範囲へ広げます。

        // Colliderの完全な端ではRaycastが境界から外れやすいため、端からごく少しだけ内側を使います。
        // 一辺分は入れず、区間長の0.5%を基本として0.005～0.02に制限します。
        float endpointInset = Mathf.Clamp(length * 0.005f, 0.005f, 0.02f);

        Vector3 entryGuess = axisOrigin + axis * (minimum + endpointInset);
        Vector3 exitGuess = axisOrigin + axis * (maximum - endpointInset);

        frame.valid = true;
        frame.collider = slopeCollider;
        frame.axis = axis;
        frame.entryPoint = entryGuess;
        frame.exitPoint = exitGuess;
        frame.projectedLength = Mathf.Max(Vector3.Dot(exitGuess - entryGuess, axis), 0.0001f);

        const float sameSectionNormalTolerance = 3f;

        // 始点はreferenceHit.normalと同じ角度の面だけを承認します。
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

        Vector3 endpointDelta = exitPoint - entryPoint;

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

        return true;
    }

    void PopulateRepresentativeSectionCurvature(ref SlopeFrame frame, Vector3 entryNormal, Vector3 exitNormal)
    {
        frame.representativeCurvature = 0f;

        if (!useRepresentativeSectionCurvature || !frame.valid)
            return;

        float entryBoundaryCurvature = 0f;
        float exitBoundaryCurvature = 0f;

        if (TrySampleConnectedSurface(frame, frame.entryPoint, -1f, out Vector3 entryConnectedPoint, out Vector3 entryConnectedNormal))
        {
            entryBoundaryCurvature = CalculateBoundaryCurvature(
                entryConnectedNormal,
                entryNormal,
                frame.axis,
                Vector3.Distance(entryConnectedPoint, frame.entryPoint),
                entryNormal);
        }

        if (TrySampleConnectedSurface(frame, frame.exitPoint, 1f, out Vector3 exitConnectedPoint, out Vector3 exitConnectedNormal))
        {
            exitBoundaryCurvature = CalculateBoundaryCurvature(
                exitNormal,
                exitConnectedNormal,
                frame.axis,
                Vector3.Distance(frame.exitPoint, exitConnectedPoint),
                exitNormal);
        }

        frame.representativeCurvature = Mathf.Clamp(
            Mathf.Max(entryBoundaryCurvature, exitBoundaryCurvature),
            0f,
            Mathf.Max(minimumCurvature, maximumRepresentativeCurvature));
    }

    bool TrySampleConnectedSurface(SlopeFrame frame, Vector3 boundaryPoint, float directionSign, out Vector3 point, out Vector3 normal)
    {
        point = Vector3.zero;
        normal = Vector3.up;

        if (!frame.valid || !frame.collider)
            return false;

        directionSign = directionSign < 0f ? -1f : 1f;
        Vector3 guess = boundaryPoint + frame.axis * directionSign * Mathf.Max(0.05f, connectedSurfaceProbeOffset);
        Bounds sectionBounds = frame.collider.bounds;
        float castHeight = Mathf.Max(forwardProbeHeight, sectionBounds.extents.y + sphereRadius + 1f);
        float castDistance = castHeight + sectionBounds.size.y + sphereRadius + forwardProbeDownDistance + connectedSurfaceMaximumGap + 2f;
        Vector3 origin = guess + Vector3.up * castHeight;
        RaycastHit[] hits = Physics.RaycastAll(origin, Vector3.down, castDistance, groundMask, QueryTriggerInteraction.Ignore);

        Vector3 side = Vector3.Cross(Vector3.up, frame.axis);
        if (side.sqrMagnitude < 0.000001f)
            side = Vector3.Cross(Vector3.forward, frame.axis);
        side = side.sqrMagnitude > 0.000001f ? side.normalized : Vector3.right;

        float maximumGap = Mathf.Max(0.05f, connectedSurfaceMaximumGap);
        float bestScore = float.PositiveInfinity;
        RaycastHit bestHit = default;
        bool found = false;

        for (int hitIndex = 0; hitIndex < hits.Length; hitIndex++)
        {
            RaycastHit hit = hits[hitIndex];
            Collider candidate = hit.collider;

            if (!candidate ||
                IsSameLogicalSlope(candidate, frame.collider) ||
                candidate.isTrigger ||
                candidate.attachedRigidbody == rb)
            {
                continue;
            }

            if (curvatureTrackRoot && candidate.transform != curvatureTrackRoot && !candidate.transform.IsChildOf(curvatureTrackRoot))
                continue;

            Vector3 candidateNormal = hit.normal.normalized;
            if (candidateNormal.sqrMagnitude < 0.999f)
                continue;

            float surfaceTilt = Vector3.Angle(candidateNormal, Vector3.up);
            if (surfaceTilt > maxSlopeAngle)
                continue;

            Vector3 delta = hit.point - boundaryPoint;
            float forwardDistance = Vector3.Dot(delta, frame.axis) * directionSign;
            float lateralDistance = Mathf.Abs(Vector3.Dot(delta, side));
            float directDistance = delta.magnitude;

            if (forwardDistance < -0.1f || forwardDistance > maximumGap || lateralDistance > maximumGap || directDistance > maximumGap * 1.5f)
                continue;

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

        if (!found)
            return false;

        point = bestHit.point;
        normal = bestHit.normal.normalized;
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

    float CalculateCurvatureAdjustedMaximumDeceleration(float speed, float curvature, float availableNormalAcceleration)
    {
        speed = Mathf.Max(0f, speed);
        curvature = Mathf.Max(0f, curvature);
        float maximumDeceleration = Mathf.Max(0.1f, maximumCriticalDeceleration);
        float minimumDeceleration = Mathf.Clamp(minimumCurvatureAdjustedDeceleration, 0f, maximumDeceleration);

        if (curvature <= minimumCurvature || availableNormalAcceleration <= 0.0001f)
        {
            return maximumDeceleration;
        }

        float normalRequirement = speed * speed * curvature;
        float normalDemand = normalRequirement / availableNormalAcceleration;
        float clampedUsage = Mathf.Clamp01(normalDemand);

        // 法線方向と接線方向の加速度容量を楕円として配分します。
        // 曲率による法線需要が増えるほど、制動距離の逆算に使う最大減速度を下げます。
        float remainingTangentialRatio = Mathf.Sqrt(Mathf.Max(0f, 1f - clampedUsage * clampedUsage));

        return Mathf.Lerp(minimumDeceleration, maximumDeceleration, remainingTangentialRatio);
    }

    // Canonical 500-line flow:
    // Observe -> BuildSurface -> Allowed/Target speed -> RequiredAcceleration
    // -> Release -> RequiredStick -> Bridge support -> ApplySurfaceAcceleration.
    void SolveGround(Vector3 move, float dt)
    {
        Vector3 velocity = rb.velocity;
        Vector3 tangentVelocity = Vector3.ProjectOnPlane(velocity, groundNormal);
        Vector3 desiredDirection = Vector3.ProjectOnPlane(move, groundNormal);

        if (desiredDirection.sqrMagnitude < 0.000001f)
            desiredDirection = Vector3.ProjectOnPlane(heading, groundNormal);
        desiredDirection = desiredDirection.sqrMagnitude > 0.000001f ? desiredDirection.normalized : Vector3.zero;

        SlopeFrame evaluationFrame = default;
        float progressAtStart = 0f;
        float distanceToSlopeEntry = 0f;
        BallVisualDistanceToNextSlopeEntry = float.PositiveInfinity;

        if (groundKind == GroundKind.Slope && slopeFrame.valid)
        {
            evaluationFrame = slopeFrame;
            progressAtStart = CalculateProgress(slopeFrame, rb.position);
            slopeProgressStatePercent = progressAtStart * 100f;
        }
        else if (groundKind == GroundKind.Flat && hasForwardSlope && forwardSlopeFrame.valid)
        {
            evaluationFrame = forwardSlopeFrame;
            progressAtStart = 0f;
            slopeProgressStatePercent = 0f;
            distanceToSlopeEntry = Mathf.Max(0f, Vector3.Dot(evaluationFrame.entryPoint - rb.position, NormalizeFlat(move, heading)));
            BallVisualDistanceToNextSlopeEntry = distanceToSlopeEntry;
        }
        else
        {
            slopeProgressStatePercent = 0f;
        }

        Vector3 preferredTangent = desiredDirection;
        Surface surface = BuildSurface(
            groundNormal,
            preferredTangent,
            desiredDirection.sqrMagnitude > epsilon ? desiredDirection : heading
        );
        SurfaceBasis basis = surface.basis;

        float forwardSpeed = surface.valid
            ? Mathf.Max(0f, surface.tangentSpeed)
            : desiredDirection.sqrMagnitude > 0f
                ? Mathf.Max(0f, Vector3.Dot(tangentVelocity, desiredDirection))
                : tangentVelocity.magnitude;
        tangentSpeedState = forwardSpeed;

        // 実斜面へ接触した時だけ新しい解放セッションを開始します。
        UpdateNaturalArtificialReleaseState(evaluationFrame, progressAtStart);

        float allowedSpeed = maxGroundSpeed;
        SurfaceSample controllingSample = default;

        if (useCriticalBoundaryTracking && evaluationFrame.valid)
            allowedSpeed = CalculateAllowedSpeedEnvelope(evaluationFrame, progressAtStart, distanceToSlopeEntry, forwardSpeed, out controllingSample);

        currentAllowedSpeed = Mathf.Clamp(allowedSpeed, 0f, maxGroundSpeed);
        controllingSamplePoint = controllingSample.valid ? controllingSample.point : Vector3.zero;

        if (controllingSample.valid)
        {
            curvatureState = controllingSample.curvature;
            availableNormalState = controllingSample.contact.availableNormalAcceleration;
            currentEffectiveMaximumDeceleration = controllingSample.effectiveMaximumDeceleration;
        }
        else
        {
            curvatureState = 0f;
            availableNormalState = 0f;
            currentEffectiveMaximumDeceleration = maximumCriticalDeceleration;
        }

        ControlCommand command = BuildGroundCommand(tangentVelocity, desiredDirection, evaluationFrame, progressAtStart, distanceToSlopeEntry, basis, dt);

        ApplyGroundCommand(command, evaluationFrame, progressAtStart, tangentSpeedState, dt);
        CompleteTargetProgressPreconditioningIfPending();
    }

    static float StepJerkLimitedActuator(float current, float target, float jerkLimit, float dt)
    {
        return MoveToward(current, target, jerkLimit, jerkLimit, dt);
    }

    static float RequiredAccelerationForSpeedChange(float currentSpeed, float targetSpeed, float distance)
    {
        return RequiredAcceleration(currentSpeed, targetSpeed, distance);
    }

    static float ReachableSpeedBeforeConstraint(float terminalSpeed, float deceleration, float distance)
    {
        return ReachableSpeed(terminalSpeed, deceleration, distance);
    }

    /// <summary>
    /// ベクトルの指定軸成分だけを置き換え、直交成分を保存します。
    /// Target制御と速度Governorが共有する線形代数上の正準操作です。
    /// </summary>
    static Vector3 ReplaceAxisComponent(Vector3 value, Vector3 unitAxis, float newComponent)
    {
        float oldComponent = Vector3.Dot(value, unitAxis);
        return value + unitAxis * (newComponent - oldComponent);
    }

    /// <summary>
    /// 面上の進行接線を一度だけ構成し、参照方向と同じ向きへそろえます。
    /// </summary>
    bool TryBuildTravelTangent(Vector3 primaryDirection, SlopeFrame frame, Vector3 orientationReference, out Vector3 travelTangent)
    {
        travelTangent = Vector3.ProjectOnPlane(primaryDirection, groundNormal);

        if (travelTangent.sqrMagnitude < 0.000001f && frame.valid)
            travelTangent = Vector3.ProjectOnPlane(frame.axis, groundNormal);
        if (travelTangent.sqrMagnitude < 0.000001f)
            return false;

        travelTangent.Normalize();

        if (orientationReference.sqrMagnitude > 0.000001f && Vector3.Dot(travelTangent, orientationReference) < 0f)
            travelTangent = -travelTangent;

        return true;
    }

    void CaptureAutomaticSpeedScaledLimits(SlopeFrame frame, float progressNow, float targetProgress, float targetSpeed)
    {
        capturedAutomaticPositiveAccelerationLimit = Mathf.Max(0f, targetProgressMaximumArtificialAcceleration);
        capturedAutomaticNegativeAccelerationLimit = Mathf.Max(0.1f, targetProgressMaximumArtificialDeceleration);
        capturedAutomaticJerkLimit = Mathf.Max(0.1f, targetProgressJerkLimit);

        if (!useAutomaticSpeedHomogeneousControl || !frame.valid || targetProgress <= progressNow)
            return;

        float slopeLength = Mathf.Max(frame.projectedLength, 0.0001f);
        float controlDistance = Mathf.Max((targetProgress - progressNow) * slopeLength, sphereRadius * 2f);
        float safeTargetSpeed = Mathf.Max(0.1f, Mathf.Min(targetSpeed, maxGroundSpeed));
        float safeCurrentSpeed = Mathf.Max(0f, tangentSpeedState);
        float averageSpeed = Mathf.Max((safeCurrentSpeed + safeTargetSpeed) * 0.5f, safeTargetSpeed * 0.5f);

        float controlSeconds = Mathf.Max(controlDistance / Mathf.Max(averageSpeed, 0.1f), Time.fixedDeltaTime * 2f);

        Vector3 referenceDirection = heading.sqrMagnitude > 0.000001f ? heading : frame.axis;
        if (!TryBuildTravelTangent(frame.axis, frame, referenceDirection, out Vector3 travelTangent))
            return;

        float netAccelNeeded = RequiredAccelerationForSpeedChange(safeCurrentSpeed, safeTargetSpeed, controlDistance);

        float gravityAlongSurface = compensateTargetProgressGravity ? Vector3.Dot(Physics.gravity, travelTangent) : 0f;

        float requiredArtificialAcceleration = netAccelNeeded - gravityAlongSurface;
        float margin = Mathf.Max(1f, automaticControlAccelerationMargin);
        float requiredMagnitudeWithMargin = Mathf.Abs(requiredArtificialAcceleration) * margin;

        float positiveLimit = Mathf.Max(targetProgressMaximumArtificialAcceleration, requiredMagnitudeWithMargin);
        float negativeLimit = Mathf.Max(targetProgressMaximumArtificialDeceleration, requiredMagnitudeWithMargin);

        capturedAutomaticPositiveAccelerationLimit = Mathf.Min(positiveLimit, Mathf.Max(0.1f, automaticTangentialAccelerationHardLimit));

        capturedAutomaticNegativeAccelerationLimit = Mathf.Min(negativeLimit, Mathf.Max(0.1f, automaticTangentialBrakeHardLimit));

        float accelerationForJerk = Mathf.Max(capturedAutomaticPositiveAccelerationLimit, capturedAutomaticNegativeAccelerationLimit);
        
        float rawRiseSeconds = controlSeconds * 0.35f;

        float riseSeconds =
            Mathf.Clamp(
                rawRiseSeconds,
                0.06f,
                0.18f
            );
        
        float automaticJerk = accelerationForJerk / Mathf.Max(riseSeconds, Time.fixedDeltaTime);
        capturedAutomaticJerkLimit = Mathf.Max(targetProgressJerkLimit, automaticJerk);

        if (writeRuntimeLog && logAutomaticSpeedHomogeneousControl)
        {
            Debug.Log($"[AUTOMATIC SPEED CONTROL CAPTURED] time={Time.fixedTime:F4}s " + $"collider={(frame.collider ? frame.collider.name : "None")} " +
                $"maxGroundSpeed={maxGroundSpeed:F6} currentSpeed={safeCurrentSpeed:F6} targetSpeed={safeTargetSpeed:F6} " +
                $"controlDistance={controlDistance:F6} controlSeconds={controlSeconds:F6} " + $"positiveLimit={capturedAutomaticPositiveAccelerationLimit:F6} " +
                $"negativeLimit={capturedAutomaticNegativeAccelerationLimit:F6} " + $"jerkLimit={capturedAutomaticJerkLimit:F6}",
                this);
        }
    }

    Vector3 ApplyTargetProgressPreconditioning(Vector3 existingMovementAcceleration, Vector3 tangentVelocity, Vector3 desiredDirection, SlopeFrame evaluationFrame,
        float progressNow,
        float dt)
    {
        if (!useTargetProgressPreconditioning || groundKind != GroundKind.Slope || !evaluationFrame.valid || !evaluationFrame.collider)
            return existingMovementAcceleration;

        if (!EnsureTargetProgressPreconditioningPlan(evaluationFrame, progressNow))
            return existingMovementAcceleration;

        if (targetProgressPhase != TargetProgressPreconditionPhase.Preconditioning ||
            !targetProgressPlanValid ||
            !IsSameLogicalSlope(targetProgressPlanCollider, evaluationFrame.collider))
        {
            return existingMovementAcceleration;
        }

        if (!TryBuildTravelTangent(evaluationFrame.axis, evaluationFrame, desiredDirection, out Vector3 travelTangent))
            return existingMovementAcceleration;

        float targetProgress = Mathf.Clamp01(capturedTargetProgressPercent * 0.01f);
        float remainingProgress = targetProgress - progressNow;
        float existingAlongTravel = Vector3.Dot(existingMovementAcceleration, travelTangent);

        // Target通過時は通常制御の正方向成分を復活させず、負方向・横方向だけを連続的に引き継ぎます。
        if (remainingProgress <= 0f)
        {
            targetProgressPhase = TargetProgressPreconditionPhase.Completed;
            targetProgressCompletionPending = true;
            previousTargetProgressAppliedAcceleration = 0f;

            float transferredAlongTravel = Mathf.Min(existingAlongTravel, 0f);
            currentTangentialAcceleration = transferredAlongTravel;
            return ReplaceAxisComponent(existingMovementAcceleration, travelTangent, transferredAlongTravel);
        }

        float currentSpeed = Mathf.Max(0f, Vector3.Dot(tangentVelocity, travelTangent));
        float remainingDistance = remainingProgress * Mathf.Max(evaluationFrame.projectedLength, 0.0001f);
        float safeDistance = Mathf.Max(remainingDistance, targetProgressMinimumDistance);
        float netAcceleration = RequiredAccelerationForSpeedChange(currentSpeed, capturedTargetTangentSpeed, safeDistance);
        float gravityAcceleration = compensateTargetProgressGravity ? Vector3.Dot(Physics.gravity, travelTangent) : 0f;
        float desiredArtificialAcceleration = netAcceleration - gravityAcceleration;

        float positiveLimit = useAutomaticSpeedHomogeneousControl ? capturedAutomaticPositiveAccelerationLimit : targetProgressMaximumArtificialAcceleration;
        float negativeLimit = useAutomaticSpeedHomogeneousControl ? capturedAutomaticNegativeAccelerationLimit : targetProgressMaximumArtificialDeceleration;

        desiredArtificialAcceleration = Mathf.Clamp(desiredArtificialAcceleration, -Mathf.Max(0.1f, negativeLimit), Mathf.Max(0f, positiveLimit));

        // 目標速度以上では正方向の人工加速度を禁止し、速度超過を増幅させません。
        if (currentSpeed >= capturedTargetTangentSpeed)
            desiredArtificialAcceleration = Mathf.Min(desiredArtificialAcceleration, 0f);

        float effectiveJerkLimit = useAutomaticSpeedHomogeneousControl ? capturedAutomaticJerkLimit : targetProgressJerkLimit;
        float appliedArtificialAcceleration = StepJerkLimitedActuator(previousTargetProgressAppliedAcceleration, desiredArtificialAcceleration,
            Mathf.Max(0.1f, effectiveJerkLimit), dt);

        previousTargetProgressAppliedAcceleration = appliedArtificialAcceleration;
        currentTangentialAcceleration = appliedArtificialAcceleration;

        return ReplaceAxisComponent(existingMovementAcceleration, travelTangent, appliedArtificialAcceleration);
    }

    Vector3 ApplySoftMaxGroundSpeedGovernor(Vector3 movementAcceleration, Vector3 tangentVelocity, Vector3 desiredDirection, SlopeFrame evaluationFrame, float dt)
    {

        if (!useAutomaticSpeedHomogeneousControl || desiredDirection.sqrMagnitude < 0.000001f || maxGroundSpeed <= 0f)
        {
            previousOverspeedGovernorAcceleration = 0f;
            return movementAcceleration;
        }

        // Natural Release完了後は速度Governorも停止し、Unityの重力・接触・摩擦へ完全に渡します。
        if (naturalMotionReleased)
        {
            previousOverspeedGovernorAcceleration = 0f;
            return movementAcceleration;
        }

        if (!TryBuildTravelTangent(desiredDirection, evaluationFrame, desiredDirection, out Vector3 travelTangent))
            return movementAcceleration;

        float currentSpeed = Mathf.Max(0f, Vector3.Dot(tangentVelocity, travelTangent));
        float existingAlongTravel = Vector3.Dot(movementAcceleration, travelTangent);

        // 上限到達時点から正方向の人工加速を禁止します。
        if (currentSpeed >= maxGroundSpeed && existingAlongTravel > 0f)
            existingAlongTravel = 0f;

        float effectiveJerkLimit = targetProgressPlanValid && capturedAutomaticJerkLimit > 0f ? capturedAutomaticJerkLimit
            : Mathf.Max(criticalJerkLimit, automaticTangentialBrakeHardLimit / 0.10f);

        if (currentSpeed <= maxGroundSpeed)
        {
            previousOverspeedGovernorAcceleration = StepJerkLimitedActuator(previousOverspeedGovernorAcceleration, 0f, Mathf.Max(0.1f, effectiveJerkLimit), dt);

            // 上限直下で正負加速度が交互に切り替わるチャタリングを防ぐ1%ヒステリシスです。
            float positiveAccelerationReleaseSpeed = Mathf.Max(0f, maxGroundSpeed * 0.99f);
            bool holdPositiveAcceleration = currentSpeed >= positiveAccelerationReleaseSpeed;

            float governedAlongTravel;
            if (holdPositiveAcceleration)
            {
                governedAlongTravel = Mathf.Min(existingAlongTravel, previousOverspeedGovernorAcceleration);
                governedAlongTravel = Mathf.Min(governedAlongTravel, 0f);
            }
            else if (previousOverspeedGovernorAcceleration < -0.0001f)
            {
                governedAlongTravel = Mathf.Min(existingAlongTravel, previousOverspeedGovernorAcceleration);
            }
            else
            {
                governedAlongTravel = existingAlongTravel;
            }

            currentTangentialAcceleration = governedAlongTravel;
            return ReplaceAxisComponent(movementAcceleration, travelTangent, governedAlongTravel);
        }

        float recoveryDistance = Mathf.Max(sphereRadius * 2f, currentSpeed * dt * Mathf.Max(2, automaticOverspeedRecoveryFixedSteps));
        float requiredGovernorAcceleration = RequiredAccelerationForSpeedChange(currentSpeed, maxGroundSpeed, recoveryDistance);
        requiredGovernorAcceleration = Mathf.Clamp(requiredGovernorAcceleration, -Mathf.Max(0.1f, automaticTangentialBrakeHardLimit), 0f);

        previousOverspeedGovernorAcceleration = StepJerkLimitedActuator(previousOverspeedGovernorAcceleration, requiredGovernorAcceleration,
            Mathf.Max(0.1f, effectiveJerkLimit), dt);
        float governedAcceleration = Mathf.Min(existingAlongTravel, previousOverspeedGovernorAcceleration);
        governedAcceleration = Mathf.Min(governedAcceleration, 0f);
        currentTangentialAcceleration = governedAcceleration;

        return ReplaceAxisComponent(movementAcceleration, travelTangent, governedAcceleration);
    }

    bool EnsureTargetProgressPreconditioningPlan(SlopeFrame evaluationFrame, float progressNow)
    {
        if (!useTargetProgressPreconditioning || !evaluationFrame.valid || !evaluationFrame.collider)
        {
            return false;
        }

        if (targetProgressPlanCollider &&
            !IsSameLogicalSlope(targetProgressPlanCollider, evaluationFrame.collider))
        {
            ResetTargetProgressPreconditioning("SlopeChanged", targetProgressPlanValid || targetProgressPhase == TargetProgressPreconditionPhase.Preconditioning);
        }

        if (!targetProgressPlanCollider)
        {
            targetProgressPlanCollider = evaluationFrame.collider;
            targetProgressPhase = TargetProgressPreconditionPhase.Observing;
        }

        if (targetProgressPhase == TargetProgressPreconditionPhase.Completed)
            return false;

        if (targetProgressPlanValid)
            return true;

        bool observationStable = slopeProgressObservationValid && readyForLimitCapture && stableSlopeContactFrames >= requiredStableSlopeFrames &&
            IsSameLogicalSlope(progressObservationCollider, evaluationFrame.collider);

        if (!observationStable)
        {
            targetProgressPhase = TargetProgressPreconditionPhase.Observing;
            return false;
        }

        float targetProgress = Mathf.Clamp01(targetSlopeProgressPercent * 0.01f);

        if (progressNow >= targetProgress)
        {
            capturedTargetProgressPercent = targetSlopeProgressPercent;
            targetProgressPhase = TargetProgressPreconditionPhase.Completed;
            targetProgressCompletionPending = true;
            EnsureNaturalArtificialReleasePlan(evaluationFrame, tangentSpeedState);
            return false;
        }

        if (!TryEvaluateSurface(evaluationFrame, targetProgress, out SurfaceSample targetSample) || targetSample.curvature <= minimumCurvature ||
            targetSample.contact.availableNormalAcceleration <= 0.0001f)
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
        capturedTargetTangentSpeed = Mathf.Clamp(targetSample.contact.criticalSpeed, 0f, maxGroundSpeed);
        previousTargetProgressAppliedAcceleration = 0f;
        previousOverspeedGovernorAcceleration = 0f;

        CaptureAutomaticSpeedScaledLimits(evaluationFrame, progressNow, targetProgress, capturedTargetTangentSpeed);

        EnsureNaturalArtificialReleasePlan(evaluationFrame, Mathf.Max(tangentSpeedState, capturedTargetTangentSpeed));

        if (writeRuntimeLog && logTargetProgressPreconditioning)
        {
            Debug.Log($"[TARGET PROGRESS PLAN CAPTURED] time={Time.fixedTime:F4}s " + $"collider={evaluationFrame.collider.name} " +
                $"progressNow={progressNow * 100f:F3}% " + $"targetProgress={capturedTargetProgressPercent:F3}% " + $"targetSpeed={capturedTargetTangentSpeed:F6} " +
                $"targetCurvature={targetSample.curvature:F8} " + $"targetAvailable={targetSample.contact.availableNormalAcceleration:F6} " + $"targetRatio={targetCriticalRatio:F6} " +
                $"automaticControl={useAutomaticSpeedHomogeneousControl} " + $"autoPositiveLimit={capturedAutomaticPositiveAccelerationLimit:F6} " +
                $"autoNegativeLimit={capturedAutomaticNegativeAccelerationLimit:F6} " + $"autoJerkLimit={capturedAutomaticJerkLimit:F6} additionalAddForce=false", this);
        }

        return true;
    }

    void CompleteTargetProgressPreconditioningIfPending()
    {
        if (!targetProgressCompletionPending)
            return;

        targetProgressCompletionPending = false;
        previousTargetProgressAppliedAcceleration = 0f;

        if (writeRuntimeLog && logTargetProgressPreconditioning)
        {
            Debug.Log($"[TARGET PROGRESS PRECONDITION COMPLETED] time={Time.fixedTime:F4}s " +
                $"collider={(targetProgressPlanCollider ? targetProgressPlanCollider.name : "None")} " + $"targetProgress={capturedTargetProgressPercent:F3}% " +
                $"actualProgress={slopeProgressStatePercent:F3}% " + $"actualSpeed={tangentSpeedState:F6} " + $"targetSpeed={capturedTargetTangentSpeed:F6} " +
                $"actualRatio={criticalRatioState:F6} " + $"targetRatio={targetCriticalRatio:F6} " + $"targetControlReleased=true " +
                $"naturalReleaseWeight={artificialReleaseState:F6} " + $"naturalMotionReleased={naturalMotionReleased} pullback=false", this);
        }
    }

    void UpdateNaturalArtificialReleaseState(SlopeFrame evaluationFrame, float progressNow)
    {
        if (!useNaturalArtificialRelease)
        {
            artificialReleaseState = 1f;
            return;
        }

        // Forward検出だけでは新セッションに切り替えません。
        // 実際に斜面へ接触した時だけ、前区間のRelease Latchを解除します。
        if (groundKind == GroundKind.Slope && evaluationFrame.valid && evaluationFrame.collider)
        {
            if (naturalReleasePlanCollider &&
                !IsSameLogicalSlope(naturalReleasePlanCollider, evaluationFrame.collider))
            {
                ResetNaturalArtificialRelease("NewSlopeContact", true);
            }

            if (!naturalReleasePlanCollider)
            {
                naturalReleasePlanCollider = evaluationFrame.collider;
            }
        }

        if (!naturalReleasePlanValid ||
            naturalMotionReleased ||
            !evaluationFrame.valid ||
            !IsSameLogicalSlope(evaluationFrame.collider, naturalReleasePlanCollider))
        {
            return;
        }

        float progressPercent = progressNow * 100f;
        if (progressPercent < naturalReleaseProgressState)
            return;

        naturalMotionReleased = true;
        artificialReleaseState = 0f;
        effectiveBaseStickState = 0f;
        adaptiveStickSaturated = false;
        previousAdaptiveBaseStickAcceleration = 0f;
        adaptiveStickActiveCollider = null;
        previousTargetProgressAppliedAcceleration = 0f;
        previousOverspeedGovernorAcceleration = 0f;

        if (!naturalReleaseLatchLogged && writeRuntimeLog && logNaturalArtificialRelease)
        {
            naturalReleaseLatchLogged = true;
            Debug.Log($"[NATURAL ARTIFICIAL RELEASE LATCHED] time={Time.fixedTime:F4}s " + $"collider={(naturalReleasePlanCollider ? naturalReleasePlanCollider.name : "None")} " +
                $"progress={progressPercent:F3}% " + $"releasePoint={naturalReleaseProgressState:F3}% " + $"baseStick=0 tangentialArtificial=0 airArtificial=0 " +
                $"handoff=GravityContactFrictionRestitution", this);
        }
    }

    bool EnsureNaturalArtificialReleasePlan(SlopeFrame frame, float speedCandidate)
    {
        if (!useNaturalArtificialRelease || !frame.valid || !frame.collider)
        {
            return false;
        }

        if (naturalReleasePlanCollider &&
            !IsSameLogicalSlope(naturalReleasePlanCollider, frame.collider))
        {
            ResetNaturalArtificialRelease("ReleasePlanSlopeChanged", true);
        }

        naturalReleasePlanCollider = frame.collider;

        if (naturalReleasePlanValid)
            return true;

        float releaseEnd = Mathf.Clamp(naturalReleaseProgressPercent, 0f, 100f);
        float minimumStart = Mathf.Clamp(targetSlopeProgressPercent + releaseHoldAfterTargetPercent, 0f, releaseEnd);
        float availableWidth = Mathf.Max(0f, releaseEnd - minimumStart);

        if (availableWidth <= 0.0001f && writeRuntimeLog && logNaturalArtificialRelease)
        {
            Debug.LogWarning($"[NATURAL RELEASE CONFIG INVALID] targetProgress={targetSlopeProgressPercent:F3}% " + $"hold={releaseHoldAfterTargetPercent:F3}% " +
                $"releaseEnd={releaseEnd:F3}% reason=ReleaseIntervalIsZero " + $"recommended=Target60_Release90", this);
        }

        float safeMaxGroundSpeed = Mathf.Max(0.01f, maxGroundSpeed);
        float speedFloor = safeMaxGroundSpeed * Mathf.Clamp01(releaseSpeedFloorRatio);
        float designSpeed = Mathf.Clamp(Mathf.Max(speedCandidate, capturedTargetTangentSpeed, speedFloor), 0f, safeMaxGroundSpeed);

        float desiredWidth = designSpeed * Mathf.Max(0.05f, releaseNominalSeconds) / Mathf.Max(0.0001f, frame.projectedLength) * 100f;

        float minimumWidth = Mathf.Min(Mathf.Max(0.1f, minimumReleaseWidthPercent), availableWidth);
        float releaseWidth = availableWidth > 0f ? Mathf.Clamp(desiredWidth, minimumWidth, availableWidth) : 0f;

        releaseStartProgressState = releaseEnd - releaseWidth;
        naturalReleaseProgressState = releaseEnd;
        naturalReleasePlanValid = true;
        naturalReleaseLatchLogged = false;

        if (writeRuntimeLog && logNaturalArtificialRelease)
        {
            Debug.Log($"[NATURAL ARTIFICIAL RELEASE PLAN CAPTURED] time={Time.fixedTime:F4}s " + $"collider={frame.collider.name} " +
                $"targetProgress={targetSlopeProgressPercent:F3}% " + $"fullStickUntil={minimumStart:F3}% " + $"releaseStart={releaseStartProgressState:F3}% " +
                $"releaseEnd={naturalReleaseProgressState:F3}% " + $"designSpeed={designSpeed:F6} " + $"maxGroundSpeed={maxGroundSpeed:F6} " +
                $"releaseWidth={releaseWidth:F3}% " + $"baseStickMode={(useAdaptiveCriticalBaseStick ? "AdaptiveCritical" : "FixedFallback")} " +
                $"adaptiveMaximum={maximumAdaptiveBaseStickAcceleration:F6} baseStickEnd=0", this);
        }

        return true;
    }

    ContactInvariant EvaluateContactInvariant(
        float speed,
        float curvature,
        float gravitySupport,
        float appliedStick)
    {
        float safeSpeed = Mathf.Max(0f, speed);
        float safeCurvature = Mathf.Max(0f, curvature);
        float safeGravitySupport = Mathf.Max(0f, gravitySupport);
        float safeAppliedStick = Mathf.Max(0f, appliedStick);
        float availableNormalAcceleration = safeGravitySupport + safeAppliedStick;

        ContactInvariant invariant = default;
        invariant.curvatureValid = safeCurvature > minimumCurvature;
        invariant.availableNormalAcceleration = availableNormalAcceleration;

        if (!invariant.curvatureValid)
        {
            invariant.criticalSpeed = Mathf.Max(0f, maxGroundSpeed);
            return invariant;
        }

        invariant.requiredNormalAcceleration = safeSpeed * safeSpeed * safeCurvature;
        invariant.criticalRatio = availableNormalAcceleration > epsilon
            ? invariant.requiredNormalAcceleration / availableNormalAcceleration
            : 0f;
        invariant.criticalSpeed = SafeSpeed(
            safeCurvature,
            safeGravitySupport,
            safeAppliedStick
        );
        invariant.requiredStickForTargetRatio = RequiredStick(
            safeSpeed,
            safeCurvature,
            safeGravitySupport
        );

        return invariant;
    }

    float CalculateRequiredBaseStickForSpeed(
        float speed,
        float curvature,
        float gravitySupport)
    {
        return RequiredStick(speed, curvature, gravitySupport);
    }

    float CalibrateAdaptiveStick(float theoreticalStick)
    {
        float calibration =
            useDeductiveAdaptiveStickCalibration
                ? Mathf.Clamp(adaptiveStickCalibration, 0.5f, 1.5f)
                : 1f;

        return Mathf.Max(0f, theoreticalStick) * calibration;
    }

    float GetEffectiveAdaptiveStickSafetyLimit(float curvature, float gravitySupport, float designSpeed)
    {
        float manualLimit = Mathf.Max(0.1f, maximumAdaptiveBaseStickAcceleration);

        if (!useAutomaticSpeedHomogeneousControl || curvature <= minimumCurvature)
        {
            return manualLimit;
        }

        float safeDesignSpeed = Mathf.Max(0f, designSpeed);
        float requiredAtDesignSpeed =
            CalibrateAdaptiveStick(
                CalculateRequiredBaseStickForSpeed(
                    safeDesignSpeed,
                    curvature,
                    gravitySupport
                )
            );
        float automaticallyRequiredLimit = requiredAtDesignSpeed * Mathf.Max(1f, automaticAdaptiveStickSafetyMargin);
        float hardLimit = Mathf.Max(0.1f, automaticAdaptiveStickHardLimit);

        return Mathf.Min(Mathf.Max(manualLimit, automaticallyRequiredLimit), hardLimit);
    }

    float GetPredictedBaseStickCapacity(SlopeFrame frame, float progress, float curvature, float gravitySupport, float designSpeed = -1f)
    {
        float releaseFactor = EvaluateArtificialReleaseWeight(frame.collider, progress * 100f);

        // 前SlopeのNatural Release latchは、Flat上で別の次Slopeを予測するときの
        // Stick容量には引き継がない。実際のDrive/StickのRelease状態自体は変更しない。
        if (groundKind == GroundKind.Flat && hasForwardSlope && naturalMotionReleased &&
            naturalReleasePlanCollider && frame.collider && !IsSameLogicalSlope(frame.collider, naturalReleasePlanCollider))
        {
            releaseFactor = 1f;
        }

        if (!useAdaptiveCriticalBaseStick)
            return baseStickAcceleration * releaseFactor;

        float effectiveDesignSpeed = designSpeed >= 0f ? Mathf.Max(maxGroundSpeed, designSpeed) : Mathf.Max(0f, maxGroundSpeed);

        float requiredAtDesignSpeed =
            CalibrateAdaptiveStick(
                CalculateRequiredBaseStickForSpeed(
                    effectiveDesignSpeed,
                    curvature,
                    gravitySupport
                )
            );
        float capacityMargin = useAutomaticSpeedHomogeneousControl ? Mathf.Max(1f, automaticAdaptiveStickSafetyMargin) : 1f;

        float uncappedCapacity = requiredAtDesignSpeed * capacityMargin * releaseFactor;
        float safetyCeiling = GetEffectiveAdaptiveStickSafetyLimit(curvature, gravitySupport, effectiveDesignSpeed);
        return Mathf.Min(uncappedCapacity, safetyCeiling);
    }

    float CalculateAdaptiveCriticalBaseStickAcceleration(SlopeFrame frame, float progress, float currentSpeed, Vector3 movementAcceleration, Vector3 desiredDirection,
        float releaseFactor, float dt)
    {
        adaptiveStickSaturated = false;
        theoreticalRequiredStickState = 0f;
        calibratedRequiredStickState = 0f;
        adaptivePredictedSpeedState = Mathf.Max(0f, currentSpeed);
        adaptiveGravitySupportState = 0f;

        if (naturalMotionReleased || releaseFactor <= 0f)
        {
            previousAdaptiveBaseStickAcceleration = 0f;
            effectiveBaseStickState = 0f;
            return 0f;
        }

        // 平面または曲率を評価できない箇所では、従来値を接触安定用のFallbackとして残します。
        if (groundKind != GroundKind.Slope || !frame.valid || !TryEvaluateSurface(frame, progress, out SurfaceSample sample) || sample.curvature <= minimumCurvature)
        {
            float fallback = baseStickAcceleration * releaseFactor;
            previousAdaptiveBaseStickAcceleration = fallback;
            effectiveBaseStickState = fallback;

            // Flat上のforwardSlopeFrameは「予測対象」であり、実接触済みColliderではありません。
            // ここでactiveへ登録すると、最初のSlope FixedUpdateが新規接触と判定されず、
            // 必要Stickの即時プリロードがJerk待ちになってしまいます。
            adaptiveStickActiveCollider =
                groundKind == GroundKind.Slope
                    ? frame.collider
                    : null;

            return fallback;
        }

        if (!useAdaptiveCriticalBaseStick)
        {
            float fixedStick = baseStickAcceleration * releaseFactor;
            previousAdaptiveBaseStickAcceleration = fixedStick;
            effectiveBaseStickState = fixedStick;
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

        float speedForecast = Mathf.Max(0f, currentSpeed + (positiveGravityAcceleration + positiveArtificialAcceleration) * predictionSeconds);
        float predictedSpeedSafetyCeiling = useAutomaticSpeedHomogeneousControl
            ? Mathf.Max(maxGroundSpeed, currentSpeed) * Mathf.Sqrt(Mathf.Max(1f, automaticAdaptiveStickSafetyMargin)) : Mathf.Max(0f, maxGroundSpeed);
        speedForecast = Mathf.Min(speedForecast, predictedSpeedSafetyCeiling);

        float theoreticalRequiredStick =
            CalculateRequiredBaseStickForSpeed(
                speedForecast,
                sample.curvature,
                sample.gravitySupport
            );

        float requiredStick =
            CalibrateAdaptiveStick(theoreticalRequiredStick);

        theoreticalRequiredStickState = theoreticalRequiredStick;
        calibratedRequiredStickState = requiredStick;
        adaptivePredictedSpeedState = speedForecast;
        adaptiveGravitySupportState = sample.gravitySupport;

        float stickLimit = GetPredictedBaseStickCapacity(frame, progress, sample.curvature, sample.gravitySupport, Mathf.Max(maxGroundSpeed, speedForecast));

        float outwardNormalSpeed = Mathf.Max(0f, Vector3.Dot(rb.velocity, sample.normal));
        float outwardDamping = outwardNormalSpeed / Mathf.Max(dt, adaptiveOutwardNormalResponseSeconds);

        float stickTarget = Mathf.Min(requiredStick + outwardDamping, stickLimit);

        bool newSlopeContact =
            !IsSameLogicalSlope(adaptiveStickActiveCollider, frame.collider);

        bool entryPreloadWindow =
            useAdaptiveCriticalEntryStickPreload &&
            !IsSameLogicalSlope(adaptiveEntryStickPreloadCollider, frame.collider) &&
            progress <= Mathf.Clamp(
                slopeEntryControlMaximumProgress,
                0.01f,
                0.25f
            );

        float stickApplied;

        if (entryPreloadWindow)
        {
            // Flat予測中にfallback Stickが保存されていても、それを入口初期値とはみなしません。
            // 理論必要Stick＋外向き速度減衰を、既存容量の範囲内で最初の実接触へ即時プリロードします。
            float preloadTarget = Mathf.Min(
                requiredStick *
                Mathf.Max(1f, adaptiveEntryStickPreloadSafetyMargin) +
                outwardDamping,
                stickLimit
            );

            stickApplied = Mathf.Max(
                previousAdaptiveBaseStickAcceleration,
                stickTarget,
                preloadTarget
            );

            adaptiveStickActiveCollider = frame.collider;
            adaptiveEntryStickPreloadCollider = frame.collider;
            adaptiveEntryStickPreloadAppliedThisFrame = true;
            adaptiveEntryStickPreloadState = stickApplied;

            if (writeRuntimeLog && logSlopeEntryTransportAndPreload)
            {
                Debug.Log(
                    $"[ADAPTIVE ENTRY STICK PRELOAD] " +
                    $"time={Time.fixedTime:F4}s " +
                    $"collider={frame.collider.name} " +
                    $"progress={progress * 100f:F3}% " +
                    $"speed={currentSpeed:F6} " +
                    $"speedForecast={speedForecast:F6} " +
                    $"requiredTheory={theoreticalRequiredStick:F6} " +
                    $"requiredCalibrated={requiredStick:F6} " +
                    $"calibration={BallVisualAdaptiveStickCalibration:F4} " +
                    $"outwardDamping={outwardDamping:F6} " +
                    $"target={stickTarget:F6} " +
                    $"preloaded={stickApplied:F6} " +
                    $"limit={stickLimit:F6}",
                    this
                );
            }
        }
        else if (newSlopeContact)
        {
            // Airからの直接着地など、Flat->Slope入口以外の新規斜面接触でも
            // 最初のFixedUpdateだけはJerk待ちをせず必要値を採用します。
            stickApplied = stickTarget;
            adaptiveStickActiveCollider = frame.collider;
        }
        else
        {
            float jerkLimit =
                stickTarget >= previousAdaptiveBaseStickAcceleration
                    ? adaptiveStickRiseJerkLimit
                    : adaptiveStickFallJerkLimit;

            stickApplied = StepJerkLimitedActuator(
                previousAdaptiveBaseStickAcceleration,
                stickTarget,
                Mathf.Max(0.1f, jerkLimit),
                dt
            );
        }

        stickApplied = Mathf.Clamp(stickApplied, 0f, stickLimit);
        previousAdaptiveBaseStickAcceleration = stickApplied;
        effectiveBaseStickState = stickApplied;
        adaptiveStickSaturated = requiredStick + outwardDamping > stickLimit + 0.001f;

        if (adaptiveStickSaturated)
        {
            if (!adaptiveStickSaturationLogged && writeRuntimeLog && logAdaptiveCriticalBaseStick)
            {
                adaptiveStickSaturationLogged = true;
                Debug.LogWarning($"[ADAPTIVE CRITICAL STICK SATURATED] time={Time.fixedTime:F4}s " + $"collider={frame.collider.name} progress={progress * 100f:F3}% " +
                    $"speed={currentSpeed:F6} speedForecast={speedForecast:F6} " +
                    $"requiredTheory={theoreticalRequiredStick:F6} requiredCalibrated={requiredStick:F6} " +
                    $"calibration={BallVisualAdaptiveStickCalibration:F4} damping={outwardDamping:F6} " +
                    $"stickLimit={stickLimit:F6} maxGroundSpeed={maxGroundSpeed:F6} " +
                    $"maximumAdaptive={maximumAdaptiveBaseStickAcceleration:F6}", this);
            }
        }
        else
        {
            adaptiveStickSaturationLogged = false;
        }

        return stickApplied;
    }

    float EvaluateArtificialReleaseWeight(Collider frameCollider, float progressPercent)
    {
        if (!useNaturalArtificialRelease)
            return 1f;

        // Latch後はAir、Flat、SphereCastの揺れでも人工力を復活させません。
        if (naturalMotionReleased)
            return 0f;

        if (!naturalReleasePlanValid ||
            !naturalReleasePlanCollider ||
            !IsSameLogicalSlope(frameCollider, naturalReleasePlanCollider))
        {
            return 1f;
        }

        if (progressPercent <= releaseStartProgressState)
            return 1f;
        if (progressPercent >= naturalReleaseProgressState)
            return 0f;

        return 1f - SmoothRange01(
            progressPercent,
            releaseStartProgressState,
            naturalReleaseProgressState
        );
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
        releaseStartProgressState = 0f;
        naturalReleaseProgressState = 0f;
        artificialReleaseState = 1f;
        effectiveBaseStickState = baseStickAcceleration;
        adaptiveStickSaturated = false;
        previousAdaptiveBaseStickAcceleration = 0f;
        adaptiveStickActiveCollider = null;
        adaptiveEntryStickPreloadCollider = null;
        adaptiveEntryStickPreloadAppliedThisFrame = false;
        adaptiveEntryStickPreloadState = 0f;
        adaptiveStickSaturationLogged = false;
        naturalReleaseLatchLogged = false;
        previousOverspeedGovernorAcceleration = 0f;
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
        capturedTargetProgressPercent = 0f;
        capturedTargetTangentSpeed = 0f;
        previousTargetProgressAppliedAcceleration = 0f;
        capturedAutomaticPositiveAccelerationLimit = 0f;
        capturedAutomaticNegativeAccelerationLimit = 0f;
        capturedAutomaticJerkLimit = 0f;
        previousOverspeedGovernorAcceleration = 0f;
        targetProgressCompletionPending = false;
        targetProgressPlanFailureLogged = false;
    }

    Vector3 ApplyFlatToSlopePredictiveBrake(Vector3 movementAcceleration, Vector3 tangentVelocity, Vector3 desiredDirection, SlopeFrame evaluationFrame,
        float distanceToSlopeEntry, SurfaceBasis basis, float dt)
    {
        slopeEntryPredictiveBrakeAcceleration = 0f;
        slopeEntryPredictiveTargetSpeed = 0f;

        if (!useLocalFlatToSlopeSequencePatch || localFlatToSlopeSequencePhase != LocalFlatToSlopeSequencePhase.FlatApproach ||
            groundKind != GroundKind.Flat || !hasForwardSlope || !evaluationFrame.valid || distanceToSlopeEntry <= 0f || !basis.valid ||
            !TryBuildTravelTangent(desiredDirection, evaluationFrame, basis.tangent, out Vector3 travelTangent))
        {
            previousSlopeEntryPredictiveBrakeAcceleration = 0f;
            slopeEntryBrakeLogged = false;
            slopeEntryBrakeLoggedCollider = null;
            return movementAcceleration;
        }

        float currentSpeed = Mathf.Max(0f, Vector3.Dot(tangentVelocity, travelTangent));
        float entryTargetSpeed = Mathf.Min(maxGroundSpeed, currentAllowedSpeed);

        if (TryEvaluateSurface(evaluationFrame, 0f, out SurfaceSample entrySample))
            entryTargetSpeed = Mathf.Min(entryTargetSpeed, entrySample.contact.criticalSpeed);

        entryTargetSpeed = Mathf.Clamp(entryTargetSpeed, 0f, maxGroundSpeed);
        slopeEntryPredictiveTargetSpeed = entryTargetSpeed;

        if (currentSpeed <= entryTargetSpeed + LocalSlopeEntrySpeedDeadband)
        {
            previousSlopeEntryPredictiveBrakeAcceleration = 0f;
            slopeEntryBrakeLogged = false;
            return movementAcceleration;
        }

        // v_target² = v² + 2 a d を入口残距離dについて逆算します。
        // これは上限超過後の固定リミッターではなく、境界到達時の速度条件を合わせる予測制御です。
        float safeDistance = Mathf.Max(distanceToSlopeEntry, Mathf.Max(sphereRadius, currentSpeed * dt));
        float requiredNetAcceleration = RequiredAccelerationForSpeedChange(currentSpeed, entryTargetSpeed, safeDistance);
        float gravityAlongTravel = Vector3.Dot(Physics.gravity, travelTangent);
        float desiredArtificialAcceleration = Mathf.Min(0f, requiredNetAcceleration - gravityAlongTravel);

        float maximumBrake = Mathf.Min(Mathf.Max(0.1f, maximumCriticalDeceleration), Mathf.Max(0.1f, automaticTangentialBrakeHardLimit));
        desiredArtificialAcceleration = Mathf.Clamp(desiredArtificialAcceleration, -maximumBrake, 0f);

        // 距離逆算値をそのままON/OFFせず、既存Jerk尺度で連続化します。
        float jerkLimit = targetProgressPlanValid && capturedAutomaticJerkLimit > 0f
            ? capturedAutomaticJerkLimit
            : Mathf.Max(criticalJerkLimit, maximumBrake / Mathf.Max(0.08f, dt * 4f));

        float appliedBrake = StepJerkLimitedActuator(previousSlopeEntryPredictiveBrakeAcceleration, desiredArtificialAcceleration,
            Mathf.Max(0.1f, jerkLimit), dt);
        previousSlopeEntryPredictiveBrakeAcceleration = appliedBrake;

        float existingAlongTravel = Vector3.Dot(movementAcceleration, travelTangent);
        float selectedAlongTravel = Mathf.Min(existingAlongTravel, appliedBrake);
        selectedAlongTravel = Mathf.Min(selectedAlongTravel, 0f);
        slopeEntryPredictiveBrakeAcceleration = selectedAlongTravel;
        currentTangentialAcceleration = selectedAlongTravel;

        if (logLocalFlatToSlopeSequencePatch && writeRuntimeLog &&
            (!slopeEntryBrakeLogged ||
             !IsSameLogicalSlope(slopeEntryBrakeLoggedCollider, evaluationFrame.collider)))
        {
            slopeEntryBrakeLogged = true;
            slopeEntryBrakeLoggedCollider = evaluationFrame.collider;
            Debug.Log($"[FLAT TO SLOPE PREDICTIVE BRAKE] time={Time.fixedTime:F4}s collider={evaluationFrame.collider.name} " +
                $"distance={distanceToSlopeEntry:F6} currentSpeed={currentSpeed:F6} targetSpeed={entryTargetSpeed:F6} " +
                $"requiredNet={requiredNetAcceleration:F6} gravityAlong={gravityAlongTravel:F6} applied={selectedAlongTravel:F6}", this);
        }

        return ReplaceAxisComponent(movementAcceleration, travelTangent, selectedAlongTravel);
    }

    Vector3 ApplySlopeEntryBridgeTangentialGuard(Vector3 movementAcceleration, Vector3 desiredDirection, SlopeFrame evaluationFrame, SurfaceBasis basis)
    {
        if (!slopeEntrySupportBridgeActive || !basis.valid ||
            !TryBuildTravelTangent(desiredDirection, evaluationFrame, basis.tangent, out Vector3 travelTangent))
        {
            return movementAcceleration;
        }

        // 支持観測が仮想補間中の1 FixedUpdateだけは、離脱を増やす正方向Driveを禁止します。
        // 負方向制動と横速度補正は保持します。
        float existingAlongTravel = Vector3.Dot(movementAcceleration, travelTangent);
        float guardedAlongTravel = Mathf.Min(existingAlongTravel, 0f);
        currentTangentialAcceleration = guardedAlongTravel;
        return ReplaceAxisComponent(movementAcceleration, travelTangent, guardedAlongTravel);
    }

    float ApplySlopeEntryBridgeSupport(float calculatedStick, float dt)
    {
        if (!slopeEntrySupportBridgeActive || !lastMeasuredSlopeObservation.valid)
            return calculatedStick;

        Vector3 normal = lastMeasuredSlopeObservation.normal.normalized;
        float outwardSpeed = Mathf.Max(0f, Vector3.Dot(rb.velocity, normal));
        float supportLimit = Mathf.Min(
            Mathf.Max(0.1f, maximumAdaptiveBaseStickAcceleration),
            Mathf.Max(0.1f, automaticAdaptiveStickHardLimit)
        );
        float recoveryAcceleration = BridgeRecoveryAcceleration(
            slopeEntrySupportBridgeGap,
            outwardSpeed,
            dt,
            supportLimit
        );
        float supportTarget = Mathf.Max(
            calculatedStick,
            previousAdaptiveBaseStickAcceleration,
            recoveryAcceleration
        );
        float appliedStick = Mathf.Clamp(supportTarget, 0f, supportLimit);

        previousAdaptiveBaseStickAcceleration = appliedStick;
        effectiveBaseStickState = appliedStick;
        adaptiveStickSaturated = supportTarget > supportLimit + 0.001f;

        if (logLocalFlatToSlopeSequencePatch && writeRuntimeLog)
        {
            Debug.Log($"[SLOPE ENTRY SUPPORT BRIDGE] time={Time.fixedTime:F4}s collider={lastMeasuredSlopeObservation.collider.name} " +
                $"progress={lastMeasuredSlopeProgress * 100f:F3}% gap={slopeEntrySupportBridgeGap:F6} outwardSpeed={outwardSpeed:F6} " +
                $"calculatedStick={calculatedStick:F6} recoveryStick={supportTarget:F6} appliedStick={appliedStick:F6}", this);
        }

        return appliedStick;
    }

    ControlCommand BuildGroundCommand(Vector3 tangentVelocity, Vector3 desiredDirection, SlopeFrame evaluationFrame, float progress,
        float distanceToSlopeEntry, SurfaceBasis basis, float dt)
    {
        if (!basis.valid)
            return default;

        Vector3 movementAcceleration = CalculateTangentialAcceleration(tangentVelocity, desiredDirection, currentAllowedSpeed, dt,
            useCriticalBoundaryTracking && evaluationFrame.valid, currentEffectiveMaximumDeceleration);
        movementAcceleration = ApplyTargetProgressPreconditioning(movementAcceleration, tangentVelocity, desiredDirection, evaluationFrame, progress, dt);
        movementAcceleration = ApplySoftMaxGroundSpeedGovernor(movementAcceleration, tangentVelocity, desiredDirection, evaluationFrame, dt);

        float releaseFactor = EvaluateArtificialReleaseWeight(evaluationFrame.collider, progress * 100f);
        artificialReleaseState = releaseFactor;
        movementAcceleration *= releaseFactor;
        currentTangentialAcceleration *= releaseFactor;

        // Natural Releaseで通常Driveが0になっていても、前方斜面へ入るための負方向予測制動だけは局所的に許可します。
        movementAcceleration = ApplyFlatToSlopePredictiveBrake(movementAcceleration, tangentVelocity, desiredDirection, evaluationFrame,
            distanceToSlopeEntry, basis, dt);

        // 観測補間中は加速せず、同じ支持面へ戻す法線制御を優先します。
        movementAcceleration = ApplySlopeEntryBridgeTangentialGuard(movementAcceleration, desiredDirection, evaluationFrame, basis);

        float stickAcceleration = CalculateAdaptiveCriticalBaseStickAcceleration(evaluationFrame, progress, tangentSpeedState, movementAcceleration,
            desiredDirection, releaseFactor, dt);
        stickAcceleration = ApplySlopeEntryBridgeSupport(stickAcceleration, dt);

        return new ControlCommand
        {
            basis = basis,
            tangentAcceleration = Vector3.Dot(movementAcceleration, basis.tangent),
            sideAcceleration = Vector3.Dot(movementAcceleration, basis.side),
            inwardNormalAcceleration = Mathf.Max(0f, stickAcceleration)
        };
    }

    void ApplyGroundCommand(
        ControlCommand command,
        SlopeFrame evaluationFrame,
        float progress,
        float tangentSpeed,
        float dt)
    {
        currentAppliedArtificialAcceleration = ApplySurfaceAcceleration(
            command.basis,
            command.tangentAcceleration,
            command.sideAcceleration,
            command.inwardNormalAcceleration
        );

        UpdateCriticalRatioAtCurrentPosition(
            evaluationFrame,
            progress,
            tangentSpeed,
            command.inwardNormalAcceleration,
            dt
        );
    }

    ArtificialControlPhase DetermineArtificialControlPhase()
    {
        if (groundKind == GroundKind.Air)
            return ArtificialControlPhase.Uncontrolled;
        if (naturalMotionReleased)
            return ArtificialControlPhase.Released;
        if (artificialReleaseState < 0.9999f)
            return ArtificialControlPhase.Releasing;
        return ArtificialControlPhase.Controlled;
    }

    void UpdateCanonicalDiagnostics(float dt)
    {
        artificialControlPhase = DetermineArtificialControlPhase();
        float tangentialJerk = (currentTangentialAcceleration - previousAppliedTangentialAcceleration) / Mathf.Max(dt, 0.000001f);
        previousAppliedTangentialAcceleration = currentTangentialAcceleration;

        bool modelValid = groundKind == GroundKind.Slope && curvatureState > minimumCurvature && availableNormalState > 0.0001f;
        if (modelValid)
        {
            float error = Mathf.Abs(criticalRatioState - targetCriticalRatio);
            sessionMinimumCriticalError = Mathf.Min(sessionMinimumCriticalError, error);
            sessionLongestMaintainedSeconds = Mathf.Max(sessionLongestMaintainedSeconds, criticalMaintainedSeconds);

            switch (artificialControlPhase)
            {
                case ArtificialControlPhase.Controlled:
                    sessionMaximumControlledRatio = Mathf.Max(sessionMaximumControlledRatio, criticalRatioState);
                    break;
                case ArtificialControlPhase.Releasing:
                    sessionMaximumReleasingRatio = Mathf.Max(sessionMaximumReleasingRatio, criticalRatioState);
                    break;
                case ArtificialControlPhase.Released:
                    sessionMaximumReleasedRatio = Mathf.Max(sessionMaximumReleasedRatio, criticalRatioState);
                    break;
            }

        }

        if (groundKind == GroundKind.Slope && previousLoggedGroundKind != GroundKind.Slope)
            sessionSlopeDetectionCount++;

        if (criticalStateMaintained && !previousLoggedCriticalMaintained)
            sessionCriticalSuccessCount++;

        if (writeRuntimeLog && logStateChanges && (groundKind != previousLoggedGroundKind || activeSlopeName != previousLoggedSlopeName))
        {
            Debug.Log($"[UNIFIED GROUND STATE] time={Time.fixedTime:F4}s {previousLoggedGroundKind}->{groundKind} " +
                $"slope={(string.IsNullOrEmpty(activeSlopeName) ? "None" : activeSlopeName)} phase={artificialControlPhase} " +
                $"progress={slopeProgressStatePercent:F3}%", this);
        }

        if (writeRuntimeLog && logCriticalSuccess && criticalStateMaintained && !previousLoggedCriticalMaintained)
        {
            Debug.Log($"[UNIFIED CRITICAL SUCCESS] time={Time.fixedTime:F4}s slope={activeSlopeName} phase={artificialControlPhase} " +
                $"progress={slopeProgressStatePercent:F3}% speed={tangentSpeedState:F6} ratio={criticalRatioState:F6} " +
                $"hold={criticalMaintainedSeconds:F4}s", this);
        }

        if (writeRuntimeLog && logCriticalRisk && modelValid && artificialControlPhase != ArtificialControlPhase.Released &&
            criticalRatioState >= criticalRiskRatio && Time.fixedTime - lastCriticalRiskLogTime >= criticalRiskLogInterval)
        {
            lastCriticalRiskLogTime = Time.fixedTime;
            Debug.LogWarning($"[UNIFIED CRITICAL RISK] time={Time.fixedTime:F4}s slope={activeSlopeName} phase={artificialControlPhase} " +
                $"progress={slopeProgressStatePercent:F3}% speed={tangentSpeedState:F6} allowed={currentAllowedSpeed:F6} " +
                $"curvature={curvatureState:F8} available={availableNormalState:F6} required={requiredNormalState:F6} ratio={criticalRatioState:F6}", this);
        }

        if (writeRuntimeLog && fixedFrameCounter % Mathf.Max(1, logEveryFixedFrames) == 0)
        {
            Debug.Log($"[UNIFIED TRACE] time={Time.fixedTime:F4}s ground={groundKind} phase={artificialControlPhase} " +
                $"slope={(string.IsNullOrEmpty(activeSlopeName) ? "None" : activeSlopeName)} progress={slopeProgressStatePercent:F3}% " +
                $"speed={tangentSpeedState:F6} targetSpeed={capturedTargetTangentSpeed:F6} " +
                $"allowed={currentAllowedSpeed:F6} acceleration={currentTangentialAcceleration:F6} jerk={tangentialJerk:F6} " +
                $"curvature={curvatureState:F8} requiredNormal={requiredNormalState:F6} availableNormal={availableNormalState:F6} " +
                $"ratio={criticalRatioState:F6} stick={effectiveBaseStickState:F6} releaseWeight={artificialReleaseState:F6} " +
                $"applied={currentAppliedArtificialAcceleration:F5}", this);
        }

        previousLoggedGroundKind = groundKind;
        previousLoggedCriticalMaintained = criticalStateMaintained;
        previousLoggedSlopeName = activeSlopeName;
    }

    Vector3 CalculateTangentialAcceleration(Vector3 tangentVelocity, Vector3 desiredDirection, float allowedSpeed, float dt, bool criticalControlActive, float maximumDeceleration)
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

        currentTangentialAcceleration = StepJerkLimitedActuator(currentTangentialAcceleration, desiredForwardAcceleration, criticalJerkLimit, dt);

        Vector3 lateralVelocity = tangentVelocity - desiredDirection * currentForward;
        Vector3 lateralAcceleration = Vector3.ClampMagnitude(-lateralVelocity / dt, maxGroundAcceleration);

        return desiredDirection * currentTangentialAcceleration + lateralAcceleration;
    }

    float CalculateAllowedSpeedEnvelope(SlopeFrame frame, float progressAtStart, float distanceToSlopeEntry, float currentSpeed, out SurfaceSample controllingSample)
    {
        controllingSample = default;
        float allowedSpeed = maxGroundSpeed;
        float dynamicLookAheadDistance = Mathf.Max(sphereRadius * 2f, currentSpeed * criticalLookAheadSeconds);
        float progressDistance = dynamicLookAheadDistance / Mathf.Max(frame.projectedLength, 0.0001f);
        float endProgress = Mathf.Clamp01(progressAtStart + progressDistance);
        int count = Mathf.Max(4, criticalSampleCount);

        for (int i = 0; i <= count; i++)
        {
            float t = i / (float)count;
            float progress = Mathf.Lerp(progressAtStart, endProgress, t);

            if (!TryEvaluateSurface(frame, progress, out SurfaceSample sample))
                continue;

            float slopeDistance = Mathf.Max(0f, (progress - progressAtStart) * frame.projectedLength);
            sample.distanceAhead = distanceToSlopeEntry + slopeDistance;
            sample.effectiveMaximumDeceleration = CalculateCurvatureAdjustedMaximumDeceleration(
                currentSpeed,
                sample.curvature,
                sample.contact.availableNormalAcceleration);

            float reachableAllowedSpeed = ReachableSpeedBeforeConstraint(sample.contact.criticalSpeed, sample.effectiveMaximumDeceleration, sample.distanceAhead);

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
        ContactInvariant contact = EvaluateContactInvariant(0f, curvature, gravitySupport, predictedBaseStickCapacity);
        contact.criticalSpeed = Mathf.Min(contact.criticalSpeed, maxGroundSpeed);

        sample.valid = true;
        sample.point = point;
        sample.normal = normal;
        sample.tangent = tangent;
        sample.curvature = curvature;
        sample.gravitySupport = gravitySupport;
        sample.contact = contact;
        sample.effectiveMaximumDeceleration = maximumCriticalDeceleration;
        return true;
    }

    void UpdateCriticalRatioAtCurrentPosition(SlopeFrame frame, float progress, float tangentSpeed, float actualInwardNormalAcceleration, float dt)
    {
        if (!frame.valid || !TryEvaluateSurface(frame, progress, out SurfaceSample currentSample))
        {
            criticalRatioState = 0f;
            requiredNormalState = 0f;
            criticalStateMaintained = false;
            criticalMaintainedSeconds = 0f;
            return;
        }

        ContactInvariant contact = EvaluateContactInvariant(tangentSpeed, currentSample.curvature, currentSample.gravitySupport, actualInwardNormalAcceleration);
        availableNormalState = contact.availableNormalAcceleration;
        requiredNormalState = contact.requiredNormalAcceleration;
        criticalRatioState = contact.criticalRatio;

        bool inTolerance = groundKind == GroundKind.Slope && contact.curvatureValid &&
            Mathf.Abs(criticalRatioState - targetCriticalRatio) <= criticalRatioTolerance;

        // Bridgeは実Contactではないため成功時間へ加算しません。
        // ただし過去の実Contactで積み上げた時間を1 FixedUpdateだけ失わせもしません。
        if (slopeEntrySupportBridgeActive)
        {
            criticalStateMaintained = criticalMaintainedSeconds >= criticalHoldSeconds;
            return;
        }

        criticalMaintainedSeconds = inTolerance ? criticalMaintainedSeconds + dt : 0f;
        criticalStateMaintained = criticalMaintainedSeconds >= criticalHoldSeconds;
    }

    bool TrySampleAtProgress(SlopeFrame frame, float progress, out Vector3 point, out Vector3 normal)
    {
        Vector3 guess = Vector3.Lerp(frame.entryPoint, frame.exitPoint, Mathf.Clamp01(progress));
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

        if (!frame.valid || !frame.collider)
            return false;

        bool requireNormalMatch = expectedNormal.sqrMagnitude > 0.000001f && maxNormalAngleDifference < 180f;

        if (requireNormalMatch)
            expectedNormal.Normalize();

        Bounds sectionBounds = frame.collider.bounds;
        float castHeight = Mathf.Max(forwardProbeHeight, sectionBounds.extents.y + sphereRadius + 1f);
        float castDistance = castHeight + sectionBounds.size.y + sphereRadius + forwardProbeDownDistance + 2f;
        Vector3 origin = guess + Vector3.up * castHeight;
        RaycastHit[] hits = Physics.RaycastAll(origin, Vector3.down, castDistance, groundMask, QueryTriggerInteraction.Ignore);

        bool found = false;
        float bestScore = float.PositiveInfinity;
        RaycastHit bestHit = default;
        bool visualizeThisCall = showEntryExitRayLines && (purpose == SurfaceSamplePurpose.Entry || purpose == SurfaceSamplePurpose.Exit) &&
            ShouldCaptureSlopeRay(frame.collider, purpose);
        bool hitTargetSlope = false;
        float nearestTargetDistance = float.PositiveInfinity;
        Vector3 displayEnd = origin + Vector3.down * castDistance;

        for (int hitIndex = 0; hitIndex < hits.Length; hitIndex++)
        {
            RaycastHit hit = hits[hitIndex];

            // SlopeFrameは同じ論理斜面の _0_Physics ～ _N_Physics を
            // 一つの連続区間として扱います。
            if (!frame.valid ||
                !hit.collider ||
                !IsSameLogicalSlope(frame.collider, hit.collider))
            {
                continue;
            }

            float slopeAngle = Vector3.Angle(hit.normal, Vector3.up);

            if (visualizeThisCall && slopeAngle >= minimumSlopeAngle && slopeAngle <= maxSlopeAngle && hit.distance < nearestTargetDistance)
            {
                nearestTargetDistance = hit.distance;
                displayEnd = hit.point;
                hitTargetSlope = true;
            }

            Vector3 candidateNormal = hit.normal.normalized;
            float normalAngle = requireNormalMatch ? Vector3.Angle(expectedNormal, candidateNormal) : 0f;

            if (requireNormalMatch && normalAngle > maxNormalAngleDifference)
                continue;

            float horizontalError = Vector3.ProjectOnPlane(hit.point - guess, Vector3.up).sqrMagnitude;
            float verticalError = Mathf.Abs(hit.point.y - guess.y);
            float score = horizontalError * 10f + verticalError + normalAngle * 0.1f;

            if (score >= bestScore)
                continue;

            bestScore = score;
            bestHit = hit;
            found = true;
        }

        if (visualizeThisCall)
        {
            SetEntryExitRayLine(frame.collider, purpose, origin, displayEnd, hitTargetSlope, hitTargetSlope || showMissedRayFullLength);
            MarkSlopeRayCaptured(frame.collider, purpose);
        }

        if (!found)
            return false;

        point = bestHit.point;
        normal = bestHit.normal.normalized;

        if (visualizeThisCall)
            SetGuessToMeasuredPointLine(frame.collider, purpose, guess, point);

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

        visual.entryRay = CreateSlopeRayLine(visual.root, "EntryRay");
        visual.exitRay = CreateSlopeRayLine(visual.root, "ExitRay");

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

    void SetEntryExitRayLine(Collider slopeCollider, SurfaceSamplePurpose purpose, Vector3 origin, Vector3 end, bool hitTargetSlope, bool visible)
    {
        if (!slopeCollider || (purpose != SurfaceSamplePurpose.Entry && purpose != SurfaceSamplePurpose.Exit))
            return;

        SlopeSectionRayVisual visual = GetOrCreateSlopeRayVisual(slopeCollider);
        LineRenderer line = purpose == SurfaceSamplePurpose.Entry ? visual.entryRay : visual.exitRay;

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
        return Progress(frame, position);
    }

    void UpdateSlopeProgressObservation(bool grounded, GroundObservation observation, float dt)
    {
        crossedTargetThisFrame = false;
        estimatedTargetCrossingTime = 0f;

        bool valid =
            grounded &&
            groundKind == GroundKind.Slope &&
            slopeFrame.valid &&
            observation.valid &&
            IsSameLogicalSlope(observation.collider, slopeFrame.collider);

        if (!valid)
        {
            slopeProgressObservationValid = false;
            slopeProgressSide = SlopeProgressSide.Invalid;
            stableSlopeContactFrames = 0;
            readyForLimitCapture = false;
            return;
        }

        float current = slopeProgressStatePercent;
        float error = current - targetSlopeProgressPercent;
       
        slopeProgressErrorPercent = error;
        

        slopeProgressErrorPercent=current - targetSlopeProgressPercent;
        slopeProgressSide = ClassifySlopeProgressSide(error);
        slopeProgressObservationValid = true;

        bool sameSession =
            hasPreviousSlopeProgressObservation &&
            IsSameLogicalSlope(progressObservationCollider, observation.collider);

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
                    $"targetControlApplied={targetProgressPhase == TargetProgressPreconditionPhase.Preconditioning && Mathf.Abs(previousTargetProgressAppliedAcceleration) > 0.0001f}", this);
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
        Vector3 appliedAirAcceleration = Vector3.zero;

        // Release Latch後は空中方向制御も復活させず、重力だけに任せます。
        if (!useNaturalArtificialRelease || !naturalMotionReleased)
        {
            Vector3 horizontalVelocity = Vector3.ProjectOnPlane(rb.velocity, Vector3.up);
            Vector3 desired = Vector3.ProjectOnPlane(move, Vector3.up);

            if (desired.sqrMagnitude > 0.000001f)
                desired.Normalize();

            desired *= maxGroundSpeed;
            appliedAirAcceleration = Vector3.ClampMagnitude((desired - horizontalVelocity) / dt, airAcceleration);
            rb.AddForce(appliedAirAcceleration, ForceMode.Acceleration);
        }

        currentTangentialAcceleration = 0f;
        currentAppliedArtificialAcceleration = appliedAirAcceleration;
        slopeEntrySupportBridgeActive = false;
        slopeEntrySupportBridgeGap = 0f;
        effectiveBaseStickState = 0f;
        adaptiveStickSaturated = false;
        criticalRatioState = 0f;
        requiredNormalState = 0f;
        availableNormalState = 0f;
        curvatureState = 0f;
        currentEffectiveMaximumDeceleration = maximumCriticalDeceleration;
        currentAllowedSpeed = maxGroundSpeed;
    }

    void OnDestroy()
    {
        // turnTween?.Kill(); // DOTween rotation disabled.

        if (runtimeSlopeRayMaterial)
            Destroy(runtimeSlopeRayMaterial);

        if (slopeRayDebugRoot)
            Destroy(slopeRayDebugRoot.gameObject);

        if (limitPointSphereObject)
            Destroy(limitPointSphereObject);
    }

    void OnDisable()
    {
        if (!writeRuntimeLog || !logSessionSummary)
            return;

        string minimumErrorText = float.IsPositiveInfinity(sessionMinimumCriticalError) ? "N/A" : sessionMinimumCriticalError.ToString("F6");
        float maximumRatioBeforeRelease = Mathf.Max(sessionMaximumControlledRatio, sessionMaximumReleasingRatio);

        Debug.Log($"[UNIFIED SESSION SUMMARY] successes={sessionCriticalSuccessCount} groundToAir={sessionGroundToAirCount} " +
            $"slopeDetections={sessionSlopeDetectionCount} maxControlledRatio={sessionMaximumControlledRatio:F6} " +
            $"maxReleasingRatio={sessionMaximumReleasingRatio:F6} maxReleasedRatio={sessionMaximumReleasedRatio:F6} " +
            $"maxRatioBeforeRelease={maximumRatioBeforeRelease:F6} minTargetError={minimumErrorText} " +
            $"longestMaintained={sessionLongestMaintainedSeconds:F4}s targetRatio={targetCriticalRatio:F6} " +
            $"tolerance={criticalRatioTolerance:F6} requiredHold={criticalHoldSeconds:F4}s", this);
    }

    void UpdateLimitPointSphere()
    {
        if (!slopeFrame.valid)
        {
            if (limitPointSphereObject)
                limitPointSphereObject.SetActive(false);

            return;
        }

        // 物理核の Progress() と完全に同じ距離単位を使います。
        // entryPoint = 0%、exitPoint = 100%。
        // projectedLength が「斜面入口から斜面出口までの1距離単位」です。
        float target01 = Mathf.Clamp01(targetSlopeProgressPercent * 0.01f);
        float distanceFromEntry = slopeFrame.projectedLength * target01;

        Vector3 physicsLimitPoint =
            slopeFrame.entryPoint +
            slopeFrame.axis * distanceFromEntry;

        Vector3 visualLimitPoint = correspondSubject
            ? correspondSubject.MapPoint(physicsLimitPoint)
            : physicsLimitPoint;

        if (!limitPointSphereObject)
        {
            limitPointSphereObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            limitPointSphereObject.name = "SlopeStick3D_LimitPointSphere";
            limitPointSphereObject.transform.localScale = Vector3.one * 0.35f;

            Collider sphereCollider = limitPointSphereObject.GetComponent<Collider>();
            if (sphereCollider)
                Destroy(sphereCollider);

            Renderer sphereRenderer = limitPointSphereObject.GetComponent<Renderer>();
            if (sphereRenderer)
                sphereRenderer.material.color = Color.magenta;
        }

        limitPointSphereObject.transform.position =
            new Vector3(
                visualLimitPoint.x,
                visualLimitPoint.y + 0.5f,
                visualLimitPoint.z
            );

        if (!limitPointSphereObject.activeSelf)
            limitPointSphereObject.SetActive(true);
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

    

}
