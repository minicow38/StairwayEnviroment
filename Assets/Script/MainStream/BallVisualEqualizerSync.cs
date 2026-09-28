using System.Collections.Generic;
using UnityEngine;
using Sirenix.OdinInspector;

/// <summary>
/// GuiltyStairway - Floating Rigidbody Equalizer.
///
/// Web/Toyful-style responsibility split:
///   Envelope:
///     - Virtual Lower Spline frame
///     - physical Upper boundary
///     - 4R-Hn amplitude / decay
///     - observed Upper->Upper natural period
///
///   Sync:
///     - Rigidbody Spring/Damper around the 4R-Hn carrier target
///     - real PhysX Upper collision
///     - Plane -> Stair release uses measured BallVisual Rigidbody state; no projectile reconstruction
///     - Natural Connect averages entry velocity/frame and C2-blends Hybrid authority
///     - Stable-N passive damping is split between controller and Rigidbody force paths
///     - height / wave count remain spatial-wave authorities
///     - Spatial Curvature Adaptation may only REDUCE the requested carrier height
///       when the current/design speed makes that curvature infeasible
///     - Physical Upper uses a low-friction runtime PhysicMaterial owned by Envelope
///     - Upper reflection is measured on the first separating FixedUpdate; optional
///       governor only removes excess Stable-N rebound and never changes T/L velocity
///     - Continuous Normal Support Handoff closes only the Plane->Stair entry
///       authority gap; it ends before the sustained Upper/Lower oscillation
///     - maxGroundSpeed is READ ONLY and contributes only to short-horizon
///       feasibility anticipation; it never increases Normal acceleration/jerk budgets
///     - T/2 is a soft feed-forward authority and never changes the spatial wave count
///     - velocity-deficit + position-lag catch-up in the transport plane
///
/// Two selectable timing modes are supported:
///   1) NaturalObserved:
///      T is an observed result of the Rigidbody oscillator.
///   2) ThreeWavesPerStair:
///      the active stair Spline domain (release -> terminal target) owns exactly
///      N spatial waves. Temporal T then follows actual progress/speed.
///
/// There is NO Transform animation while released.
/// SlopeStickCore.maxGroundSpeed is READ ONLY.
/// </summary>
[Searchable]
[DisallowMultipleComponent]
public sealed class BallVisualEqualizerSync : MonoBehaviour
{
    // ========================================================================
    // MOTION PRESERVATION CONTRACT — DO NOT TRIM WITHOUT AN A/B MOTION TEST
    // ========================================================================
    //
    // Baseline:
    //   BallVisualEqualizerSync_safe_trim_02
    //
    // Required visual/physical behavior:
    //   The first stair wave must begin with a strong -Stable-N response.
    //   On the current stair frame, -N contains both DOWN and BACKWARD
    //   components, so the ball is visibly pushed back after the first launch.
    //
    // The following control chain is protected:
    //
    //   SlopeStickCore normal support
    //       -> CaptureContinuousNormalSupportSample()
    //       -> ArmContinuousNormalSupportHandoff()
    //       -> ResolveContinuousNormalSupportAcceleration()
    //
    //   Spatial wave phase
    //       -> targetNormalVelocity / targetNormalAcceleration
    //       -> spring + damper + spatial feed-forward
    //       -> ResolveNormalAuthority()
    //       -> normalTargetAccelerationAfterAuthority
    //       -> rideAccelerationState
    //       -> Rigidbody AddForce
    //
    //   Collision state:
    //       Physical Upper terminates the entry support bridge.
    //       Physical Lower rearms the next Upper and owns lower-impact retention.
    //
    // Protected means:
    //   - Do not delete these fields/methods as "diagnostics".
    //   - Do not inline or merge them merely to reduce line count.
    //   - Do not replace persistent state with temporary locals.
    //   - Do not change signs, authority blending, FixedUpdate ordering, or
    //     ForceMode without an explicit motion-comparison test.
    //   - Fine tuning of these values is allowed later, but structure/state
    //     must remain available for tuning.
    // ========================================================================

    public const string RuntimeBuildId = "ContinuousNormalSupportUpperContact-20260920-D";
    public const string ObservationTelemetryBuildId = "UpperReflectionTelemetry-v1";

    public enum EqualizerPhase
    {
        Synchronized,
        ReleaseArmed,
        FreeFlight,
        LowerContact,
        HopperFlight,
        UpperContact,
        Reacquiring
    }

    public enum WaveTimingMode
    {
        NaturalObserved,
        ThreeWavesPerStair
    }

    public enum NormalAuthorityZone
    {
        LogicalEnvelope,
        HandoffToPhysics,
        PhysicalFree,
        PhysicalStairContact
    }


    public enum EqualizerControlMode
    {
        LegacyHybrid,
        Gen3VirtualLower
    }

    public enum Gen3EventPhase
    {
        Synchronized, Flight, LowerApproach, VirtualLowerContact,
        UpperApproach, PhysicalUpperImpact, Reacquire
    }


    [System.Serializable]
    private struct OscillationFrame
    {
        public bool valid;
        public Vector3 tangent;
        public Vector3 normal;
        public Vector3 lateral;
    }

    [System.Serializable]
    private struct ReleaseFrame
    {
        public Vector3 position;
        public Vector3 subjectPosition;
        public Vector3 velocity;
        public Vector3 angularVelocity;
        public float sourceEnergy;
        public float referenceHeight;
        public Vector3 sourceAxis;
    }

// ================================================================
// References
// ================================================================

    [Header("References")] [SerializeField]
    private Rigidbody ballVisual;

    [SerializeField] private SlopeStickCore slopeCore;
    [SerializeField] private CorrespondSubject correspondSubject;
    [SerializeField] private Transform subjectTransform;
    [SerializeField] private Rigidbody subjectBody;

    [SerializeField] private Rigidbody ballVisualEqualizer;
    [SerializeField] private SphereCollider ballVisualEqualizerCollider;
    [SerializeField] private BallVisualNegativeEnvelopeCollider negativeEnvelope;

// ================================================================
// Wave timing mode switch
// ================================================================

    [Header("Wave Timing Mode")]
    [Tooltip(
        "NaturalObserved: 現在のFloating Rigidbody自然振動。Tは観測結果。\n" +
        "ThreeWavesPerStair: Release->TerminalのSpline区間へ空間的にN波を割り当てる。\n" +
        "速度が変化すると時間周期Tは自動的に変化します。")]
    [SerializeField]
    private WaveTimingMode waveTimingMode = WaveTimingMode.ThreeWavesPerStair;

    [Tooltip("ThreeWavesPerStair時の空間波数N。Inspectorから独立して指定します。")] [Range(1, 8)] [SerializeField]
    private int spatialWavesPerStair = 6;

    [Tooltip(
        "4R-Hn Envelopeの何割を実Carrier波の希望Lower->Upper高さに使うか。\n" +
        "実際は加速度Budgetにより自動的に小さくなることがあります。")]
    [Range(0.05f, 0.8f)]
    [SerializeField]
    private float spatialCarrierHeightFractionOfEnvelope = 0.22f;


// ================================================================
// Spatial half-period guidance
// ================================================================

    [Header("Spatial Wave - Soft T/2 Guidance")]
    [Tooltip(
        "ON: Spatial position phase (and therefore wave count) is still owned by Spline progress, while " +
        "velocity/acceleration feed-forward is softly biased toward the preferred T/2.")]
    [SerializeField]
    private bool usePreferredHalfPeriodGuidance = true;

    [Tooltip("Preferred Lower<->Upper half period T/2 [s]. This does not change the spatial wave count.")]
    [Min(0.01f)]
    [SerializeField]
    private float preferredHalfPeriodSeconds = 0.08f;

    [Tooltip(
        "How strongly T/2 biases feed-forward. 0 = pure spatial derivative, 1 = preferred T/2 angular speed. " +
        "Target position always remains the spatial N-wave carrier.")]
    [Range(0f, 1f)]
    [SerializeField]
    private float preferredHalfPeriodFeedForwardBlend01 = 0.35f;

    [Tooltip(
        "Spatial Waveが使ってよいStable-N最大加速度[m/s^2]。\n" +
        "空間N波を高速で見せるためNaturalモードより大きくできます。")]
    [Min(50f)]
    [SerializeField]
    private float spatialWaveAccelerationBudget = 1800f;

    [Tooltip("Spatial WaveのStable-N加速度Jerk上限[m/s^3]。")] [Min(100f)] [SerializeField]
    private float spatialWaveJerkBudget = 12000f;

    [Tooltip(
        "Spatial Waveの参照波加速度feed-forward倍率。1で理論参照加速度を全て使います。")]
    [Range(0f, 1.5f)]
    [SerializeField]
    private float spatialWaveFeedForward = 1f;

    [Header("Spatial Curvature Adaptation")]
    [Tooltip(
        "ON: 現在のUpper / Virtual Lower / 4R-Hn / Energy / Decayを変更せず、" +
        "Envelopeが要求したCarrier高さを曲率Feasibilityで必要な場合だけ下方向へClampします。")]
    [SerializeField]
    private bool useSpatialCurvatureAdaptation = true;

    [Tooltip(
        "速度比のREAD ONLY診断基準[m/s]。Acceleration/Jerk budgetを速度で増幅する用途には使いません。")]
    [Min(1f)]
    [SerializeField]
    private float spatialReferenceTangentSpeed = 16f;

    [Tooltip(
        "曲率上限 kappaMax = aNmax * safety / vDesign^2 に掛ける安全率。" +
        "aNmaxには既存spatialWaveAccelerationBudgetをそのまま使います。")]
    [Range(0.25f, 1f)]
    [SerializeField]
    private float spatialCurvatureSafety01 = 0.85f;

    [Tooltip(
        "maxGroundSpeedへ向かう短期速度をFeasibilityだけで先読みする時間[s]。" +
        "Upper位置、Lower位置、Decay、波Phaseは先送りしません。最低1 FixedUpdate。")]
    [Range(0.02f, 0.20f)]
    [SerializeField]
    private float spatialCurvatureLookAheadSeconds = 0.06f;

    [Tooltip(
        "実測接線速度からmaxGroundSpeed側の短期予測速度へ何割寄せて曲率判定するか。" +
        "0=実測速度のみ、1=look-ahead予測を全使用。")]
    [Range(0f, 1f)]
    [SerializeField]
    private float spatialCurvatureMaxGroundSpeedBlend01 = 0.35f;

    [Tooltip(
        "Spatial進捗加速度dp/dtの微分項をFeedForwardへ含めます。\n" +
        "H''(p)*pDot^2 + H'(p)*pDDot の完全な空間FeedForwardになります。")]
    [SerializeField]
    private bool includeSpatialProgressAccelerationFeedForward = true;

    [Tooltip(
        "pDDot[1/s^2]の数値ノイズ上限。Spline射影の小さな揺れを巨大なNormal加速度へ変換しないためのGuardです。")]
    [Min(1f)]
    [SerializeField]
    private float maximumSpatialDomainAccelerationRate = 80f;

    [Tooltip(
        "Spatial N-waveモードでSubjectのTerminal到着時刻へ間に合わせるために許すmaxGroundSpeed超過率。\n" +
        "0.75なら最大1.75倍。SlopeStickCoreへは書き込みません。")]
    [Range(0f, 2f)]
    [SerializeField]
    private float spatialCatchUpSpeedHeadroom01 = 0.75f;

    [Tooltip("Spatial N-waveモードのCatch-up最大加速度[m/s^2]。")] [Min(1f)] [SerializeField]
    private float spatialMaximumCatchUpAcceleration = 180f;

    [Tooltip("Spatial N-waveモードでGoal速度自体を追従させる最大加速度[m/s^2]。")] [Min(1f)] [SerializeField]
    private float spatialGoalVelocityAcceleration = 160f;


// ================================================================
// Natural Plane -> Stair Connect
// ================================================================

    [Header("Natural Entry Connect")]
    [Tooltip(
        "Natural Entry開始後、BallVisualの実Rigidbody速度とStable frameを平均するFixedUpdate数。" +
        "この区間ではEqualizerはBallVisualへ同期したままで、力を追加しません。")]
    [Range(2, 12)]
    [SerializeField]
    private int naturalConnectAverageFrames = 5;

    [Tooltip(
        "平均化に要求する最小時間[s]。FixedUpdate数と両方を満たした時点でHybridへReleaseします。")]
    [Min(0.02f)]
    [SerializeField]
    private float naturalConnectAverageSeconds = 0.08f;

    [Tooltip(
        "Normal速度がほぼ0でもConnectを永久待機しないための最大Sampling時間[s]。")]
    [Min(0.05f)]
    [SerializeField]
    private float naturalConnectMaximumWaitSeconds = 0.18f;

    [Tooltip(
        "実測状態からHybrid 50%制御へ移る時間[s]。" +
        "authority = 10u^3 - 15u^4 + 6u^5 で位置/速度/加速度の端点を滑らかにします。")]
    [Min(0.02f)]
    [SerializeField]
    private float naturalConnectBlendSeconds = 0.12f;

    [Tooltip(
        "Envelope geometryをArmするためだけに使う最小Normal速度[m/s]。" +
        "BallVisualEqualizerの初速へは絶対に注入しません。")]
    [Min(0.001f)]
    [SerializeField]
    private float naturalConnectMinimumEnvelopeNormalSpeed = 0.05f;

    [Header("Continuous Normal Support Handoff")]
    [Tooltip(
        "ON: Plane->StairのNatural Connect Release直後だけ、SlopeStickCoreが直前まで実際に使っていた" +
        "Normal押さえ込み加速度をREAD ONLYで継承します。既存Equalizer authorityが0->1へ開くのと逆位相で" +
        "1->0へ消えるため、実Upper/Virtual Lower/4R-Hn/Decay/RejoinのAuthorityには残りません。")]
    [SerializeField]
    private bool useContinuousNormalSupportHandoff = true;

    [Tooltip(
        "異常なStick値を入口へ持ち込まないための安全上限[m/s^2]。" +
        "通常のFlatStick(約24.6)はそのまま通し、極端な瞬間値だけを切ります。")]
    [Min(1f)]
    [SerializeField]
    private float continuousNormalSupportMaximumAcceleration = 120f;

// ================================================================
// Floating Ride Spring / Damper
// ================================================================

    [Header("Floating Ride - Rigidbody Spring/Damper")]
    [Tooltip(
        "WebのrideSpringStrength相当。Stable-N位置誤差[m]をAccelerationへ変換します。\n" +
        "ForceMode.Accelerationなので mass 非依存の s^-2 相当です。")]
    [Min(1f)]
    [SerializeField]
    private float rideSpringStrength = 420f;

    [Tooltip(
        "WebのrideSpringDamper相当。Stable-N相対速度[m/s]へ掛ける減衰係数[s^-1]。")]
    [Min(0f)]
    [SerializeField]
    private float rideSpringDamper = 18f;


    [Header("Hybrid Stable-N Passive Damping")]
    [Tooltip(
        "ON: only the passive -C*vN part of the Damper is split. " +
        "Spring, target-velocity drive, spatial feed-forward, H and wave count remain unchanged.")]
    [SerializeField]
    private bool useHybridPassiveNormalDamping = true;

    [Tooltip(
        "Share of passive Stable-N damping applied through a dedicated Rigidbody AddForce path. " +
        "0.5 = 50% controller + 50% Rigidbody damping. T/L transport is never damped by this value.")]
    [Range(0f, 1f)]
    [SerializeField]
    private float rigidbodyPassiveDampingShare01 = 0.50f;

    [Tooltip(
        "Stable-Nに掛かるUnity重力を何割相殺するか。\n" +
        "1でFloating Controllerと同様に重力を相殺してSpringが平衡点を所有します。")]
    [Range(0f, 1.5f)]
    [SerializeField]
    private float gravityCompensation = 1f;

    [Tooltip("Stable-N Spring/Damperの最大加速度[m/s^2]。")] [Min(1f)] [SerializeField]
    private float maximumRideAcceleration = 450f;

    [Tooltip("Stable-N加速度の最大変化率[m/s^3]。")] [Min(1f)] [SerializeField]
    private float maximumRideJerk = 3000f;

    [Tooltip(
        "4R-Hn内のSpring平衡点。0=Lower、0.5=中央、1=Upper。\n" +
        "通常は0.5。上下へ自然に振幅させるため中央を使います。")]
    [Range(0.05f, 0.95f)]
    [SerializeField]
    private float rideEquilibrium01 = 0.5f;


// ================================================================
// Logical -> Physical safety handoff
// ================================================================

    [Header("Physical Stair Safety Handoff")]
    [Tooltip(
        "ON: 実StairWayへ予期せず接触した場合だけPhysical authorityへ渡すSafetyです。\n" +
        "通常のStable-N波形制御とHybrid dampingはLogical authority内で継続します。")]
    [SerializeField]
    private bool enableLogicalPhysicalHandoff = true;

    [Tooltip(
        "予期しない実StairWay接触やProjection discontinuityでPhysical authorityへ渡す際の最大Jerk[m/s^3]。")]
    [Min(1f)]
    [SerializeField]
    private float authorityHandoffMaxJerk = 900f;

    [Tooltip(
        "実StairWay衝突後、上向き相対Normal速度がこの値を超えたらPhysical Lower impactの\n" +
        "Energy retentionを確定します。[m/s]")]
    [Min(0f)]
    [SerializeField]
    private float physicalLowerMeasurementOutgoingSpeed = 0.05f;

    [Tooltip(
        "Physical Lowerとして受理する最低下降相対Normal速度[m/s]。\n" +
        "Accepted Upper後かつ vN < -この値 のStairWay接触だけをPhysical Lowerにします。")]
    [Min(0f)]
    [SerializeField]
    private float physicalLowerMinimumDescendingSpeed = 0.25f;

    [Header("Virtual Lower Support Frame Guard")]
    [Tooltip(
        "Virtual Lower射影中心が1 FixedUpdateでこのR数より大きく飛んだ場合、\n" +
        "Visual Frame回転/射影切替として扱い、その差分を速度へ変換しません。")]
    [Min(0.25f)]
    [SerializeField]
    private float supportKinematicsMaximumCenterJumpR = 2.5f;

    [Tooltip(
        "Virtual Lower中心差分から見た見かけ速度の上限[m/s]。\n" +
        "超過時はSupport速度をMapped Subject速度へ戻してFrame Sampleをリセットします。")]
    [Min(1f)]
    [SerializeField]
    private float supportKinematicsMaximumMeasuredSpeed = 80f;

    [Tooltip(
        "Mapped Subject速度から求めるSupport加速度の上限[m/s^2]。\n" +
        "Visual Frame回転を巨大なNormal feed-forwardへ変換しないためのGuardです。")]
    [Min(1f)]
    [SerializeField]
    private float supportKinematicsMaximumMeasuredAcceleration = 600f;

// ================================================================
// Goal velocity / catch-up
// ================================================================

    [Header("Transport - Goal Velocity / Catch-up")]
    [Tooltip(
        "WebのgoalVelへMoveTowardsする加速度。Goal自体を急変させないための値[m/s^2]。")]
    [Min(1f)]
    [SerializeField]
    private float goalVelocityAcceleration = 80f;

    [Tooltip(
        "Subjectとの進行方向位置遅れ[m]を追加Goal速度へ戻す時間[s]。\n" +
        "階段/Upper衝突でタイムロスすると lag/time がGoal速度へ加算されます。")]
    [Min(0.03f)]
    [SerializeField]
    private float catchUpPositionTime = 0.25f;

    [Tooltip(
        "neededAccel=(goalVel-rb.velocity)/FixedDeltaTime の最大値[m/s^2]。")]
    [Min(1f)]
    [SerializeField]
    private float maximumCatchUpAcceleration = 90f;

    [Tooltip("Transport加速度の最大変化率[m/s^3]。")] [Min(1f)] [SerializeField]
    private float maximumTransportJerk = 1400f;

    [Tooltip(
        "maxGroundSpeedを超えてCatch-upする時の追加許容量。\n" +
        "例0.35なら一時的に1.35倍まで許可。SlopeStickCoreへは書き込みません。")]
    [Range(0f, 1f)]
    [SerializeField]
    private float catchUpSpeedHeadroom01 = 0.35f;

    [Tooltip(
        "Tangent以外のTransport面ズレを戻す弱いSpring[s^-2]。Stable-Nには作用しません。")]
    [Min(0f)]
    [SerializeField]
    private float lateralSpringStrength = 12f;

    [Tooltip("横方向相対速度のDamper[s^-1]。")] [Min(0f)] [SerializeField]
    private float lateralDamper = 5f;

// ================================================================
// Upper impact observation
// ================================================================

    [Header("Minimal Measurement")]
    [SerializeField] private bool logMeasurements;

    [Header("Upper Impact Observation")]
    [Tooltip(
        "Upper->Upper実測周期として採用する最小時間[s]。\n" +
        "0ならFixedDeltaTime*2を使用。")]
    [Min(0f)]
    [SerializeField]
    private float minimumObservedCycleSeconds = 0f;

    [Tooltip(
        "Upperを一度受理した後、4R-Hnのこの高さ率より下へ戻るまで次Upperを受理しません。\n" +
        "Mesh swap / 接触継続による1 FixedUpdate毎の誤カウントを防ぎます。")]
    [Range(0.05f, 0.75f)]
    [SerializeField]
    private float upperPeakRearmHeight01 = 0.35f;

    [Header("Upper Reflection Analysis / Governor")]

    [Tooltip(
        "実Upperが作った反射が指定Restitution上限を超えた場合だけ、Stable-N成分の余剰分を削ります。\n" +
        "Tangent/Lateral速度、Upper Collider形状、Virtual Lowerには触れません。\n" +
        "初期検証ではfalseのまま測定だけ行うことを推奨します。")]
    [SerializeField]
    private bool enableUpperReflectionGovernor = false;

    [Tooltip(
        "vOut <= vIn * cap の片側上限制御。0.85なら実反射速度を最大85%まで許可します。")]
    [Range(0f, 1f)]
    [SerializeField]
    private float upperReflectionRestitutionCap01 = 0.85f;

    [Tooltip(
        "これ未満の入射速度ではGovernorを適用しません[m/s]。微小接触ノイズ除外用。")]
    [Min(0f)]
    [SerializeField]
    private float upperReflectionGovernorMinimumIncomingSpeed = 1.0f;

    [Tooltip(
        "OnCollisionEnter後、最初のseparating velocityを待つ最大FixedUpdate数。")]
    [Range(1, 6)]
    [SerializeField]
    private int upperReflectionResolveMaxFixedSteps = 3;

    [Header("Post Upper Down-Stair Direction Memory")]
    [Tooltip(
        "ON: 最初の実Upper接触後、衝突Normalの跳ね返りはPhysXへ任せたまま、" +
        "接触時に確定したWorld-spaceの階段下降Tangentだけを短時間保持します。\n" +
        "Transform/Visual回転からは再計算しないため、Upper後の階段方向成分が回転表示に引きずられません。")]
    [SerializeField]
    private bool usePostUpperDirectionalMemory = true;

    [Tooltip(
        "maxGroundSpeedの何割をUpper後の最低Tangent目標速度として使うか。\n" +
        "現在速度/Subject速度の方が大きい場合はそちらを保持するため、速度を不自然に下げません。")]
    [Range(0f, 1f)]
    [SerializeField]
    private float postUpperDirectionalReferenceSpeedFraction = 0.72f;

    [Tooltip(
        "Upper後のTangent速度不足を埋める1次遅れの時定数[s]。小さいほど階段方向が強く立ち上がります。")]
    [Min(0.02f)]
    [SerializeField]
    private float postUpperDirectionalResponseTime = 0.08f;

    [Tooltip(
        "Upper後に追加するWorld-space Tangent加速度の上限[m/s^2]。Normal反射には加えません。")]
    [Min(0f)]
    [SerializeField]
    private float postUpperDirectionalMaximumAcceleration = 90f;

    [Tooltip(
        "方向メモリ強度が半分になる時間[s]。Physical Lowerまで完全停止条件を置かず、指数的に自然減衰します。")]
    [Min(0.02f)]
    [SerializeField]
    private float postUpperDirectionalHalfLifeSeconds = 0.20f;

    [Tooltip(
        "Upper接触時World frameのLateral速度を穏やかに消す割合。0なら横方向には触れません。")]
    [Range(0f, 1f)]
    [SerializeField]
    private float postUpperLateralAlignmentShare01 = 0.35f;

    [Tooltip(
        "Lateral速度を0へ寄せる1次遅れ時定数[s]。Tangent強化とは独立です。")]
    [Min(0.02f)]
    [SerializeField]
    private float postUpperLateralResponseTime = 0.12f;

    [Header("Hybrid Rough / Smooth Assist")]
    [Tooltip(
        "既存のStable-N加速度がAcceleration Budgetのこの割合を超え始めたら、" +
        "Post Upper Smooth Assistを徐々に混ぜ始めます。\n" +
        "低加速度域では従来の荒っぽい挙動をそのまま残します。")]
    [Range(0f, 1f)]
    [SerializeField]
    private float postUpperSmoothAssistStartBudgetRatio = 0.30f;

    [Tooltip(
        "この割合付近でSmooth Assistをほぼ100%まで混ぜます。\n" +
        "Startとの間はSmoothStepで連続的に補間し、厳しいON/OFF条件にはしません。")]
    [Range(0.05f, 1.5f)]
    [SerializeField]
    private float postUpperSmoothAssistFullBudgetRatio = 0.72f;

    [Tooltip(
        "spatialSpeedRatioが1を超えたとき、Smooth Assistをどれだけ早く強めるか。\n" +
        "0なら速度比を使わず、1なら超過分を強く反映します。")]
    [Range(0f, 1f)]
    [SerializeField]
    private float postUpperSmoothAssistSpeedRatioInfluence = 0.35f;

    [Header("3-Wave Micro Modal Oscillation")]
    [Tooltip(
        "ON: 既存3waveの頂点/谷を変更せず、その間だけ衝突励起の小振動をStable-Nへ加えます。\n" +
        "最終的には既存Acceleration/Jerk/Authority制限を必ず通ります。")]
    [SerializeField]
    private bool useMicroModalOscillation = true;

    [Tooltip(
        "小振動が使える最大Normal加速度を、既存Acceleration Budgetの割合で指定します。\n" +
        "0.08なら最大8%。3wave本体より十分小さく保つための上限です。")]
    [Range(0f, 0.25f)]
    [SerializeField]
    private float microModalAccelerationShare01 = 0.08f;

    [Tooltip("Plane->Stair Natural Connect直後に入れる弱い初期励起。1st wave前半の細振動用。")]
    [Range(0f, 1.5f)]
    [SerializeField]
    private float microModalEntryKick = 0.22f;

    [Tooltip("Physical Upperで各Modalへ入れる速度Impulse倍率。ApexそのものではGate=0なので形は壊しません。")]
    [Range(0f, 1.5f)]
    [SerializeField]
    private float microModalUpperKick = 0.55f;

    [Tooltip("Physical Lowerで各Modalへ入れる速度Impulse倍率。次の斜面区間へ小振動を残します。")]
    [Range(0f, 1.5f)]
    [SerializeField]
    private float microModalLowerKick = 0.42f;

    [Tooltip("Impact速度を無次元化する基準[m/s]。小さいほど弱い衝突でも細振動が強くなります。")]
    [Min(0.1f)]
    [SerializeField]
    private float microModalImpactReferenceSpeed = 6.0f;

    [Tooltip(
        "既存3waveの targetNormalVelocity と実測 rideRelativeNormalVelocity の差を、" +
        "Micro Modalへ連続的に戻す強さ。0なら従来のImpact励起だけです。")]
    [Range(0f, 0.75f)]
    [SerializeField]
    private float microModalVelocityResidualDriveGain = 0.18f;

    [Tooltip(
        "速度残差を無次元化する基準[m/s]。残差がこの値付近になるとMicro補強が明確になります。\n" +
        "hard thresholdではなく連続飽和なので、小さい残差でも不連続にON/OFFしません。")]
    [Min(0.05f)]
    [SerializeField]
    private float microModalVelocityResidualReferenceSpeed = 2.0f;

    [Tooltip("各Micro Modeの減衰比。小さいほど細振動が長く残ります。")]
    [Range(0.02f, 0.45f)]
    [SerializeField]
    private float microModalDampingRatio = 0.14f;

    [Tooltip(
        "弱いDuffing硬化係数。0で線形Modal。大きくすると衝突直後だけ少し硬く/速い小振動になります。")]
    [Range(0f, 0.30f)]
    [SerializeField]
    private float microModalDuffingBeta = 0.07f;

    [Tooltip("Main 3wave角速度に対するMode-1の非整数比。")]
    [Range(1f, 8f)]
    [SerializeField]
    private float microModalFrequencyRatio1 = 2.6f;

    [Tooltip("Main 3wave角速度に対するMode-2の非整数比。")]
    [Range(1f, 8f)]
    [SerializeField]
    private float microModalFrequencyRatio2 = 4.1f;

    [Tooltip("Main 3wave角速度に対するMode-3の非整数比。")]
    [Range(1f, 8f)]
    [SerializeField]
    private float microModalFrequencyRatio3 = 5.7f;

    [Tooltip(
        "Micro Modeの最大周波数[Hz]。FixedUpdateのサンプリング限界より高い周波数は内部でさらにSoft Compressします。")]
    [Range(4f, 18f)]
    [SerializeField]
    private float microModalMaximumFrequencyHz = 10f;

    [Tooltip(
        "Upper/Lower境界へ近く、かつStable-N速度が小さいgrazing領域で細振動を少し強調する割合。\n" +
        "Apex/LowerのゼロGateは維持するため、境界そのものへ力は追加しません。")]
    [Range(0f, 0.5f)]
    [SerializeField]
    private float microModalGrazingBoost01 = 0.12f;

    [Tooltip("境界から何割の距離までgrazing補強を効かせるか。")]
    [Range(0.02f, 0.35f)]
    [SerializeField]
    private float microModalGrazingDistance01 = 0.14f;

    [Tooltip("grazing判定のStable-N速度スケール[m/s]。")]
    [Min(0.05f)]
    [SerializeField]
    private float microModalGrazingVelocityScale = 0.80f;

    [Header("Physical Lower Impact Observation")]
    [Tooltip(
        "Authority Handoff中はUpper衝突をEnvelope Energyへ二重計上せず、\n" +
        "実StairWay Lower衝突の前後速度から次波Energy retentionを測ります。")]
    [SerializeField]
    private bool applyPhysicalLowerImpactEnergyLoss = true;

    [Tooltip("Stair衝突Energy retentionの測定下限。Solverノイズによる0化を防ぎます。")] [Range(0f, 1f)] [SerializeField]
    private float minimumPhysicalLowerEnergyRetention01 = 0.05f;

    [Tooltip(
        "Physical Lower後、OnCollisionExitを待たずに上向きStable-N速度を追跡する最大FixedUpdate数。\n" +
        "24m/sでは接触時間が短いため、既定6フレーム以内でEnergyを確定します。")]
    [Range(2, 12)]
    [SerializeField]
    private int physicalLowerEnergyResolveMaxFixedSteps = 6;

    [Header("Terminal Spatial Transport Assist")]
    [Tooltip(
        "Spatial Wave終盤だけTransport平面のCatch-upを増やします。Normal波形/Lower減衰には触れません。")]
    [SerializeField]
    private bool enableTerminalSpatialTransportAssist = true;

    [Tooltip("Spatial domainのこの進捗からTerminal transport assistを開始します。")] [Range(0.5f, 0.95f)] [SerializeField]
    private float terminalSpatialTransportAssistStartProgress01 = 0.80f;

    [Tooltip("Terminal付近でCatch-up acceleration/Jerkへ掛ける最大倍率。")] [Range(1f, 2f)] [SerializeField]
    private float terminalSpatialTransportAssistMultiplier = 1.50f;

// ================================================================
// Unified Legacy / Gen3 control
// ================================================================

    [Header("Unified Equalizer Control Mode")]
    [SerializeField]
    private EqualizerControlMode equalizerControlMode = EqualizerControlMode.LegacyHybrid;

    [Header("Gen3 - Poincare / ILC")]
    [Range(1f, 6f)] [SerializeField] private float gen3BaseSpanR = 4.0f;
    [Range(0.25f, 1f)] [SerializeField] private float gen3SpanRetention01 = 0.82f;
    [Range(0.25f, 4f)] [SerializeField] private float gen3MinimumSpanR = 1.25f;
    [Range(0f, 0.75f)] [SerializeField] private float gen3IlcGain = 0.18f;
    [Range(0f, 2f)] [SerializeField] private float gen3IlcClampR = 0.75f;
    [Range(0f, 0.25f)] [SerializeField] private float gen3IlcDeadBandR = 0.04f;

    [Header("Gen3 - Virtual Lower")]
    [Range(0f, 0.5f)] [SerializeField] private float gen3LowerActivationR = 0.06f;
    [Range(0f, 2f)] [SerializeField] private float gen3LowerLeadSteps = 1.0f;
    [Range(0.5f, 20f)] [SerializeField] private float gen3LowerFrequencyHz = 6.5f;
    [Range(0f, 2f)] [SerializeField] private float gen3LowerDampingRatio = 0.90f;
    [Range(0f, 1f)] [SerializeField] private float gen3ImplicitShare01 = 0.72f;
    [Range(1f, 2.5f)] [SerializeField] private float gen3HuntExponent = 1.5f;
    [Range(0f, 0.5f)] [SerializeField] private float gen3HuntDissipation = 0.08f;
    [Range(0.02f, 1f)] [SerializeField] private float gen3HuntReferenceCompressionR = 0.25f;
    [Range(0f, 0.25f)] [SerializeField] private float gen3ImpedanceStartR = 0.01f;
    [Range(0.02f, 1f)] [SerializeField] private float gen3ImpedanceFullR = 0.25f;
    [Range(10f, 800f)] [SerializeField] private float gen3MaximumLowerAcceleration = 320f;
    [Range(50f, 10000f)] [SerializeField] private float gen3MaximumNormalJerk = 6000f;
    [Range(0.05f, 1f)] [SerializeField] private float gen3AdaptiveSubstepChiMax = 0.25f;
    [Range(1, 16)] [SerializeField] private int gen3MaximumInternalSubsteps = 8;
    [Range(0.1f, 1f)] [SerializeField] private float gen3ReferenceGovernorSafety01 = 0.92f;

    [Header("Gen3 - Energy Tank")]
    [SerializeField] private bool gen3UseEnergyTank = true;
    [Range(0.25f, 4f)] [SerializeField] private float gen3EnergyTankCapacityMultiplier = 1.25f;
    [Min(0.001f)] [SerializeField] private float gen3MinimumEnergyTankCapacityJoule = 0.25f;
    [Range(0f, 1f)] [SerializeField] private float gen3InitialEnergyTankFill01 = 0.15f;
    [Range(0f, 1f)] [SerializeField] private float gen3EnergyRecoveryEfficiency01 = 0.90f;

// ================================================================
// Emergency Visual Reacquire
// ================================================================

    [Header("Emergency Visual Reacquire")]
    [Tooltip("この距離[m]以下だけ通常のCopyBallVisualPoseによる最終一致を許可します。")] [Min(0.001f)] [SerializeField]
    private float resumeSynchronizationSnapDistance = 0.05f;

    [Tooltip("Equalizer相対Hermiteの最短時間[s]。")] [Min(0.05f)] [SerializeField]
    private float emergencyReacquireMinimumDuration = 0.18f;

    [Tooltip("Equalizer相対Hermiteの最長時間[s]。")] [Min(0.10f)] [SerializeField]
    private float emergencyReacquireMaximumDuration = 0.75f;

    [Tooltip("BallVisualとの距離からRecovery時間を決める見かけ上の相対回収速度[m/s]。")] [Min(1f)] [SerializeField]
    private float emergencyReacquirePreferredRelativeSpeed = 24f;

// ================================================================
// Runtime diagnostics
// ================================================================

    [Header("Runtime - Read Only")] [SerializeField]
    private bool synchronized = true;

    [SerializeField] private EqualizerPhase phase = EqualizerPhase.Synchronized;

    [Header("Emergency Reacquire Runtime - Read Only")] [SerializeField]
    private bool emergencyVisualRecoveryActive;

    [Header("Post Turn Hermite Bridge Runtime - Read Only")] [SerializeField]
    private bool postTurnHermiteBridgeActive;

    [SerializeField] private float emergencyVisualRecoveryDuration;
    [SerializeField] private float emergencyVisualRecoveryProgress01;
    [SerializeField] private float emergencyVisualRecoveryInitialDistance;

    [SerializeField] private int waveCycleIndex;


    [SerializeField] private float observedNaturalPeriodSeconds;

    [SerializeField] private float rideTargetHeight;
    [SerializeField] private float rideActualHeight;
    [SerializeField] private float ridePositionError;
    [SerializeField] private float rideRelativeNormalVelocity;
    [SerializeField] private float rideSupportNormalVelocity;
    [SerializeField] private float rideSupportNormalAcceleration;
    [SerializeField] private float springAcceleration;
    [SerializeField] private float damperAcceleration;
    [SerializeField] private float gravityCompensationAcceleration;

    [SerializeField] private float requiredCatchUpSpeed;

    [Header("Spatial Wave Runtime - Read Only")]
    [Tooltip("Equalizer実位置から求めた生の空間進捗。Collider反発で後退することがあります。")]
    [SerializeField]
    private float spatialRawDomainProgress01;

    [Tooltip("Presentation用の単調増加進捗。ThreeWavesPerStair中は後退しません。")] [SerializeField]
    private float spatialDomainProgress01;

    [Tooltip("単調増加進捗の実増加率[1/s]。後退保持中は0です。")] [SerializeField]
    private float spatialDomainAdvanceRate01PerSecond;

    [Tooltip("単調進捗速度の時間微分[1/s^2]。H'(p)*pDDot feed-forwardに使います。")] [SerializeField]
    private float spatialDomainAccelerationRate01PerSecond2;

    // [MOTION-PROTECTED] Spatial speed reference.
    // Keep the explicit ratio available for later tuning/comparison.
    [Tooltip("曲率Feasibilityで使ったdesignSpeed / spatialReferenceTangentSpeed。")] [SerializeField]
    private float spatialSpeedRatio = 1f;

    [Tooltip("互換診断。Adaptive版では速度によるAcceleration budget増幅を廃止したため常に1。")]
    [SerializeField] private float spatialAccelerationBudgetScale = 1f;

    [Tooltip("互換診断。Adaptive版では速度によるJerk budget増幅を廃止したため常に1。")]
    [SerializeField] private float spatialJerkBudgetScale = 1f;

    [Header("Spatial Curvature Adaptation Runtime - Read Only")]
    [SerializeField] private float spatialCurvatureMeasuredForwardSpeed;
    [SerializeField] private float spatialCurvatureDesignSpeed;
    [SerializeField] private float spatialCurvatureLookAheadDistance;
    [SerializeField] private float spatialCurvatureWaveNumberPerMeter;
    [SerializeField] private float spatialCurvatureKappaMaxPerMeter;
    [SerializeField] private float spatialCurvatureRequestedHeightMeters;
    [SerializeField] private float spatialCurvatureAllowedHeightMeters;
    [SerializeField, Range(0f, 1f)] private float spatialCurvatureClampRatio01 = 1f;
    [SerializeField] private bool spatialCurvatureLimited;

    [SerializeField] private float spatialFeedForwardVelocity;
    [SerializeField] private float spatialFeedForwardAcceleration;
    [SerializeField, Range(0f, 1f)] private float terminalSpatialTransportAssist01;

    [Tooltip("Raw進捗が過去最大より後ろにあり、Monotonic Hold中ならTRUE。Upper Colliderの状態は変更しません。")]
    [SerializeField]
    private bool spatialMonotonicHoldActive;

    [SerializeField] private float spatialWavePhase01;
    [SerializeField] private float spatialReferencePeriodSeconds;
    [SerializeField] private float spatialCarrierHeightMeters;
    [SerializeField] private float spatialCarrierFeasibility01 = 1f;

    [Header("Spatial Half-Period Runtime - Read Only")]
    [SerializeField] private float spatialPathHalfPeriodSeconds;
    [SerializeField] private float spatialGuidedHalfPeriodSeconds;
    [SerializeField] private float spatialPathAngularSpeed;
    [SerializeField] private float spatialGuidedAngularSpeed;
    [SerializeField, Range(0f, 1f)] private float spatialHalfPeriodFeedForwardBlendApplied01;

    [SerializeField] private float spatialCarrierHeightBeforeFeasibility;
    [SerializeField] private float spatialSubjectTimeToGo;
    [SerializeField] private float spatialRequiredArrivalSpeed;
    [SerializeField] private float spatialArrivalFeasibility01 = 1f;
    // [MOTION-PROTECTED] Physical Upper/Lower cycle gate.
    [SerializeField] private bool upperPeakArmed = true;

    [Header("Logical / Physical Authority Runtime - Read Only")] [SerializeField]
    private NormalAuthorityZone normalAuthorityZone = NormalAuthorityZone.LogicalEnvelope;

    [SerializeField, Range(0f, 1f)] private float normalLogicalAuthority01 = 1f;
    [SerializeField] private float normalTargetAccelerationBeforeAuthority;
    // [MOTION-PROTECTED] Final Stable-N command after logical/physical authority.
    [SerializeField] private float normalTargetAccelerationAfterAuthority;
    [SerializeField] private float normalActiveJerkLimit;

    // [MOTION-PROTECTED] Upper -> descending -> Physical Lower acceptance state.
    [SerializeField] private bool physicalUpperSeenSinceLastLower;
    [SerializeField] private bool physicalLowerContactActive;
    [SerializeField] private bool rideSupportFrameContinuous = true;
    [SerializeField] private float lastPhysicalLowerIncomingNormalSpeed;
    [SerializeField] private float lastPhysicalLowerOutgoingNormalSpeed;
    [SerializeField] private float lastPhysicalLowerEnergyRetention01 = 1f;

    [Header("Release Energy Runtime - Read Only")] [SerializeField]
    private float sourceCanonicalNormalSpeed;

    [SerializeField] private float logicalLaunchNormalSpeed;
    [SerializeField] private float logicalLaunchEnergyJoule;
    [SerializeField] private float logicalLaunchAppliedMultiplier = 1f;
    [SerializeField] private float logicalLaunchReferenceTangentSpeed;

    [Header("Natural Connect Runtime - Read Only")]
    [SerializeField] private bool naturalConnectSamplingActive;
    [SerializeField] private bool naturalConnectBlendActive;
    [SerializeField] private int naturalConnectSampleCount;
    [SerializeField] private float naturalConnectSamplingElapsed;
    [SerializeField] private float naturalConnectBlendElapsed;
    // [MOTION-PROTECTED] 0 -> 1 Natural Connect authority opening.
    // This also controls how the inherited entry support fades away.
    [SerializeField, Range(0f, 1f)] private float naturalConnectControlAuthority01 = 1f;
    [SerializeField] private Vector3 naturalConnectAveragedVelocity;
    [SerializeField] private Vector3 naturalConnectAveragedSubjectVelocity;
    [SerializeField] private Vector3 naturalConnectAveragedNormal = Vector3.up;
    [SerializeField] private Vector3 naturalConnectAveragedTangent = Vector3.forward;
    [SerializeField] private float naturalConnectMeasuredRelativeNormalSpeed;
    [SerializeField] private float naturalConnectMeasuredEnergyJoule;

    // [MOTION-PROTECTED] First-wave backward/downward entry support.
    // Do not remove even when individual members also appear in Inspector.
    [Header("Continuous Normal Support Runtime - Read Only")]
    [SerializeField] private bool continuousNormalSupportCaptured;
    [SerializeField] private bool continuousNormalSupportActive;
    [SerializeField] private float continuousNormalSupportCapturedAcceleration;
    [SerializeField, Range(0f, 1f)] private float continuousNormalSupportWeight01;
    [SerializeField] private float continuousNormalSupportAppliedAcceleration;
    [SerializeField] private string continuousNormalSupportEndReason = "None";

    // [MOTION-PROTECTED] Effective Stable-N damping gain.
    [SerializeField] private float effectiveRideSpringDamper;


    // [MOTION-PROTECTED] Passive Stable-N damping split.
    [Header("Hybrid Damping Runtime - Read Only")]
    [SerializeField] private float activeTargetNormalDriveAcceleration;
    [SerializeField] private float totalPassiveNormalDampingAcceleration;
    [SerializeField] private float controllerPassiveDampingAcceleration;
    [SerializeField] private float rigidbodyPassiveDampingAcceleration;
    [SerializeField] private float rigidbodyPassiveDampingAppliedAcceleration;
    [SerializeField, Range(0f, 1f)] private float rigidbodyPassiveDampingShareApplied01;


    [SerializeField] private float lastUpperIncomingNormalSpeed;

    [Header("Upper Reflection Observation - READ ONLY")]
    [SerializeField] private int upperReflectionCount;
    [SerializeField] private float lastUpperReflectionFixedTime = -1f;

    [SerializeField] private float lastUpperOutgoingNormalSpeed;
    [SerializeField] private float lastImpactEnergyRetention01 = 1f;

    // [MOTION-PROTECTED] Post-Upper world-space directional memory.
    [Header("Post Upper Direction Runtime - Read Only")]
    [SerializeField] private bool postUpperDirectionalMemoryActive;
    [SerializeField] private Vector3 postUpperCapturedTangent = Vector3.forward;
    [SerializeField] private Vector3 postUpperCapturedNormal = Vector3.up;
    [SerializeField] private Vector3 postUpperCapturedLateral = Vector3.right;
    [SerializeField] private float postUpperDirectionalTargetSpeed;
    [SerializeField, Range(0f, 1f)] private float postUpperDirectionalWeight01;
    [SerializeField] private float postUpperDirectionalAppliedAcceleration;
    [SerializeField] private float postUpperLateralAppliedAcceleration;

    [SerializeField] private float postUpperHybridAccelerationDemand;
    [SerializeField] private float postUpperHybridAccelerationBudget;
    [SerializeField] private float postUpperHybridAccelerationRatio;
    [SerializeField, Range(0f, 1f)] private float postUpperHybridSmoothAssist01;

    [Header("Gen3 Runtime - Read Only")]
    [SerializeField] private Gen3EventPhase gen3EventPhase = Gen3EventPhase.Synchronized;
    [SerializeField] private float gen3CurrentCycleTargetSpanR = 4f;
    [SerializeField] private float gen3ObservedUpperSpanR;
    [SerializeField] private float gen3IlcCorrectionR;
    [SerializeField] private float gen3Impedance01;
    [SerializeField] private int gen3AdaptiveSubsteps = 1;
    [SerializeField] private float gen3GovernedNormalSpeed;
    [SerializeField] private float gen3NormalAccelerationState;
    [SerializeField] private float gen3VirtualLowerForceNewton;
    [SerializeField] private float gen3VirtualLowerPenetrationMeters;
    [SerializeField] private float gen3EnergyTankJoule;
    [SerializeField] private float gen3EnergyTankCapacityJoule;
    [SerializeField, Range(0f, 1f)] private float gen3PassivityScale01 = 1f;

// ================================================================
// Runtime state
// ================================================================

    private const float Epsilon = 0.000001f;

    private OscillationFrame oscillationFrame;
    private ReleaseFrame releaseFrame;

    // [MOTION-PROTECTED] FixedUpdate-persistent Stable-N acceleration state.
    private Vector3 rideAccelerationState;
    private Vector3 transportAccelerationState;
    private Vector3 goalPlanarVelocityState;

    // 3-wave micro-modal state.
    // q = normalized modal displacement, qDot = normalized modal velocity.
    // These are intentionally NOT serialized diagnostics.
    private float microMode1Q;
    private float microMode1QDot;
    private float microMode2Q;
    private float microMode2QDot;
    private float microMode3Q;
    private float microMode3QDot;

    // Exact targetNormalVelocity resolved by the existing 3-wave controller
    // in the current FixedUpdate. This is deliberately not an independent
    // wave authority; it only drives the subordinate Micro Modal layer.
    private float microTargetNormalVelocityReference;

    private bool microPreviousNaturalConnectBlendActive;
    private bool microPreviousUpperSeenSinceLastLower;
    private bool microPreviousLowerContactActive;
    private int microImpactSerial;

    private float naturalConnectSamplingStartTime = -1f;
    private float naturalConnectBlendStartTime = -1f;
    private float naturalConnectReferenceHeight;
    private Vector3 naturalConnectPreferredNormal = Vector3.up;
    private Vector3 naturalConnectPreferredTangent = Vector3.forward;
    private Vector3 naturalConnectVelocitySum;
    private Vector3 naturalConnectSubjectVelocitySum;
    private Vector3 naturalConnectNormalSum;
    private Vector3 naturalConnectTangentSum;

    private float emergencyVisualRecoveryStartTime = -1f;
    private Vector3 emergencyInitialRelativeOffset;
    private Vector3 emergencyInitialRelativeVelocity;
    private Quaternion emergencyInitialRotation = Quaternion.identity;

// Virtual Lower is a projection point in Visual space.
// Its world-position finite difference is NOT physical support velocity:
// mapped Subject velocity owns the support kinematics, while the projected
// center is used only to validate frame/projection continuity.
    private bool rideSupportKinematicsValid;
    private Vector3 previousRideSupportCenter;
    private Vector3 previousRideSupportVelocity;
    private float previousRideSupportSampleTime;

    private float lastUpperContactFixedTime = -1f;

    private bool pendingUpperImpactEnergyMeasurement;
    private float pendingUpperIncomingNormalSpeed;
    private int pendingUpperReflectionResolveFrames;

    private float postUpperDirectionalMemoryStartFixedTime = -1f;

    private readonly HashSet<Collider> physicalLowerContacts =
        new HashSet<Collider>();

    private readonly HashSet<Collider> gen3IgnoredPhysicalColliders =
        new HashSet<Collider>();
    private bool gen3VirtualLowerWasActive;

    private bool pendingPhysicalLowerImpactEnergyMeasurement;
    private float pendingPhysicalLowerIncomingNormalSpeed;
    private int pendingPhysicalLowerEnergyFrames;
    private float pendingPhysicalLowerBestOutgoingNormalSpeed;
    private float previousSpatialDomainAdvanceRate01PerSecond;

    private NormalAuthorityZone previousNormalAuthorityZone =
        NormalAuthorityZone.LogicalEnvelope;

// ================================================================
// Public compatibility
// ================================================================

    public Rigidbody Body => ballVisualEqualizer;

    public int OscillationCycleIndex => waveCycleIndex;

    public float PlannedMaxGroundSpeedForCycle =>
        negativeEnvelope
            ? negativeEnvelope.PlannedMaxGroundSpeedForCycle
            : 0f;

    /// <summary>
    /// SlopeStickCore.maxGroundSpeed の現在値をREAD ONLYで取得する。
    /// BallVisualSlopeDriveの回転後1回だけのEqualizer Energy正規化に使用する。
    /// このAPIからSlopeStickCoreへ書き込みは行わない。
    /// </summary>
    public bool TryGetSourceMaxGroundSpeedReadOnly(
        out float maxGroundSpeed)
    {
        maxGroundSpeed = 0f;

        if (!negativeEnvelope)
            return false;

        if (!negativeEnvelope.TryGetSourceMaxGroundSpeedReadOnly(
                out maxGroundSpeed))
        {
            maxGroundSpeed = 0f;
            return false;
        }

        if (maxGroundSpeed <= Epsilon)
        {
            maxGroundSpeed = 0f;
            return false;
        }

        return true;
    }

    public bool IsSynchronized => synchronized;
    public bool IsEmergencyVisualRecoveryActive => emergencyVisualRecoveryActive;
    public bool IsPostTurnHermiteBridgeActive => postTurnHermiteBridgeActive;

    // Physical Upper reflection telemetry. These are observation-only APIs;
    // reading them never changes Equalizer physics or envelope authority.
    public int UpperReflectionCount => upperReflectionCount;
    public float LastUpperReflectionFixedTime => lastUpperReflectionFixedTime;
    public float LastUpperReflectionIncomingNormalSpeed => lastUpperIncomingNormalSpeed;
    public float LastUpperReflectionOutgoingNormalSpeed => lastUpperOutgoingNormalSpeed;

    public EqualizerPhase Phase => phase;

    public float EqualizerMass =>
        ballVisualEqualizer
            ? Mathf.Max(0.0001f, ballVisualEqualizer.mass)
            : 1f;

    public float ObservedNaturalPeriodSeconds =>
        observedNaturalPeriodSeconds;


    public WaveTimingMode TimingMode => waveTimingMode;

    public bool ThreeWavesPerStairEnabled =>
        waveTimingMode == WaveTimingMode.ThreeWavesPerStair;

    public bool LogicalPhysicalHandoffEnabled => enableLogicalPhysicalHandoff;
    public NormalAuthorityZone CurrentNormalAuthorityZone => normalAuthorityZone;
    public float NormalLogicalAuthority01 => normalLogicalAuthority01;
    public float LastPhysicalLowerEnergyRetention01 => lastPhysicalLowerEnergyRetention01;
    public float SpatialReferencePeriodSeconds => spatialReferencePeriodSeconds;
    public float SpatialCarrierHeightMeters => spatialCarrierHeightMeters;
    public float SpatialCarrierFeasibility01 => spatialCarrierFeasibility01;
    public bool SpatialCurvatureAdaptationEnabled => useSpatialCurvatureAdaptation;
    public float SpatialCurvatureDesignSpeed => spatialCurvatureDesignSpeed;
    public float SpatialCurvatureKappaMaxPerMeter => spatialCurvatureKappaMaxPerMeter;
    public float SpatialCurvatureRequestedHeightMeters => spatialCurvatureRequestedHeightMeters;
    public float SpatialCurvatureAllowedHeightMeters => spatialCurvatureAllowedHeightMeters;
    public float SpatialCurvatureClampRatio01 => spatialCurvatureClampRatio01;
    public bool SpatialCurvatureLimited => spatialCurvatureLimited;
    public float SpatialSubjectTimeToGo => spatialSubjectTimeToGo;
    public float SpatialRequiredArrivalSpeed => spatialRequiredArrivalSpeed;
    public float SpatialArrivalFeasibility01 => spatialArrivalFeasibility01;
    public EqualizerControlMode ControlMode => equalizerControlMode;
    public Gen3EventPhase Gen3Phase => gen3EventPhase;
    public float Gen3CycleTargetSpanR => gen3CurrentCycleTargetSpanR;
    public float Gen3ObservedUpperSpanR => gen3ObservedUpperSpanR;
    public float Gen3IlcCorrectionR => gen3IlcCorrectionR;
    public float Gen3Impedance01 => gen3Impedance01;
    public int Gen3InternalSubsteps => gen3AdaptiveSubsteps;
    public float Gen3EnergyTankJoule => gen3EnergyTankJoule;

// ================================================================
// Unity
// ================================================================

    private void Start()
    {


        ResolveReferences();
        RefreshVisualCollisionOwnership();

        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            Debug.LogError(
                "[EQUALIZER] BallVisual / BallVisualEqualizer reference is missing.",
                this);
            return;
        }
       // Physics.IgnoreCollision(ballVisualEqualizerCollider,GameObject.Find("BallVisualEqualizerTest").transform.GetComponent<SphereCollider>());


        EnterSynchronizedState();
    }


    private void FixedUpdate()
    {
        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        if (emergencyVisualRecoveryActive)
        {
            ResetMicroModalOscillation();
            UpdateEmergencyVisualRecovery();
            return;
        }

        if (naturalConnectSamplingActive)
        {
            ResetMicroModalOscillation();
            UpdateNaturalConnectSampling();
            return;
        }

        if (synchronized)
        {
            ResetMicroModalOscillation();
            CopyBallVisualPose();
            return;
        }

        // Detect entry / Upper / Lower event edges before any authority transition
        // can close them during this FixedUpdate.
        UpdateMicroModalImpactExcitation();

        UpdateNaturalConnectBlend();
        UpdateWaveTimingAuthority();

        // Physical Upper reflection is measured on the first separating frame.
        // Optional governor only removes excess Stable-N rebound.
        ResolvePendingUpperImpactEnergyLoss();

        if (equalizerControlMode == EqualizerControlMode.Gen3VirtualLower)
        {
            UpdateGen3VirtualLower();
        }
        else
        {
            ResolvePendingPhysicalLowerImpactEnergyLoss();
            UpdateFloatingRideSpring();
        }

        // Upper後の階段方向成分は、衝突時に固定したWorld-space Tangentを
        // 短時間だけ補強する。Normal反射はPhysX/既存制御へ残す。
        ApplyPostUpperDirectionalMemory();

        // Tangential/transport catch-up is shared. Gen3 only replaces Normal/Lower authority.
        ApplyGoalVelocityCatchUp();
    }

// ================================================================
// References / Subject mapping
// ================================================================

    private void ResolveReferences()
    {
        if (!ballVisualEqualizer)
        {
            ballVisualEqualizer =
                GetComponent<Rigidbody>();

            if (!ballVisualEqualizer)
            {
                GameObject obj =
                    GameObject.Find(
                        "BallVisualEqualizer");

                if (obj)
                {
                    ballVisualEqualizer =
                        obj.GetComponent<Rigidbody>();
                }
            }
        }

        if (!ballVisualEqualizerCollider &&
            ballVisualEqualizer)
        {
            ballVisualEqualizerCollider =
                ballVisualEqualizer.GetComponent<SphereCollider>();
        }

        if (!ballVisual)
        {
            GameObject obj =
                GameObject.Find(
                    "BallVisual");

            if (obj)
            {
                ballVisual =
                    obj.GetComponent<Rigidbody>();
            }
        }

        if (!slopeCore)
        {
            slopeCore =
                FindFirstObjectByType<SlopeStickCore>();
        }

        if (!correspondSubject)
        {
            correspondSubject =
                FindFirstObjectByType<CorrespondSubject>();
        }

        if (correspondSubject)
        {
            if (correspondSubject.SubjectBody)
            {
                subjectBody =
                    correspondSubject.SubjectBody;

                subjectTransform =
                    subjectBody.transform;
            }
            else if (!subjectTransform)
            {
                subjectTransform =
                    correspondSubject.transform;
            }
        }

        if (!subjectTransform)
        {
            GameObject obj =
                GameObject.Find("Subject");

            if (!obj)
                obj = GameObject.Find("subject");

            if (obj)
            {
                subjectTransform =
                    obj.transform;

                subjectBody =
                    obj.GetComponent<Rigidbody>();
            }
        }

        if (!negativeEnvelope)
        {
            negativeEnvelope =
                FindFirstObjectByType<BallVisualNegativeEnvelopeCollider>();
        }

    }


    private bool HasMappedSubject =>
        correspondSubject &&
        correspondSubject.InSubjectBody;


    private Vector3 ReadSubjectPositionVisual()
    {
        if (HasMappedSubject)
            return correspondSubject.MappedPosition;

        if (subjectTransform)
            return subjectTransform.position;

        return ballVisual
            ? ballVisual.position
            : transform.position;
    }


    private Vector3 ReadSubjectVelocityVisual()
    {
        if (HasMappedSubject)
            return correspondSubject.MappedVelocity;

        if (subjectBody &&
            subjectBody != ballVisual)
        {
            return subjectBody.velocity;
        }

        return ballVisual
            ? ballVisual.velocity
            : Vector3.zero;
    }

// ================================================================
// Synchronization
// ================================================================

    public void Equalize()
    {
        // Emergency中はFixedUpdateのHermiteだけが位置権威。
        // Update側からCopyBallVisualPoseしてRecovery曲線を壊さない。
        if (emergencyVisualRecoveryActive)
            return;

        if (synchronized)
            CopyBallVisualPose();
    }


    private void CopyBallVisualPose()
    {
        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        ballVisualEqualizer.transform.SetPositionAndRotation(
            ballVisual.position,
            ballVisual.rotation);
    }


    private void EnterSynchronizedState()
    {
        if (!ballVisualEqualizer ||
            !ballVisual)
        {
            return;
        }

        emergencyVisualRecoveryActive = false;
        emergencyVisualRecoveryProgress01 = 0f;

        synchronized = true;
        phase = EqualizerPhase.Synchronized;

        waveCycleIndex = 0;
        observedNaturalPeriodSeconds = 0f;
        lastUpperContactFixedTime = -1f;
        ResetGen3State(0f);
        gen3EventPhase = Gen3EventPhase.Synchronized;

        upperPeakArmed = true;
        physicalUpperSeenSinceLastLower = false;

        physicalLowerContacts.Clear();
        physicalLowerContactActive = false;
        rideSupportFrameContinuous = true;
        pendingPhysicalLowerImpactEnergyMeasurement = false;
        pendingPhysicalLowerIncomingNormalSpeed = 0f;
        pendingPhysicalLowerEnergyFrames = 0;
        pendingPhysicalLowerBestOutgoingNormalSpeed = 0f;
        lastPhysicalLowerIncomingNormalSpeed = 0f;
        lastPhysicalLowerOutgoingNormalSpeed = 0f;
        lastPhysicalLowerEnergyRetention01 = 1f;
        normalAuthorityZone = NormalAuthorityZone.LogicalEnvelope;
        previousNormalAuthorityZone = NormalAuthorityZone.LogicalEnvelope;
        normalLogicalAuthority01 = 1f;
        normalTargetAccelerationBeforeAuthority = 0f;
        normalTargetAccelerationAfterAuthority = 0f;
        normalActiveJerkLimit = 0f;
        sourceCanonicalNormalSpeed = 0f;
        logicalLaunchNormalSpeed = 0f;
        logicalLaunchEnergyJoule = 0f;
        logicalLaunchAppliedMultiplier = 1f;
        logicalLaunchReferenceTangentSpeed = 0f;
        ResetNaturalConnectRuntime();
        naturalConnectControlAuthority01 = 1f;
        effectiveRideSpringDamper = rideSpringDamper;
        activeTargetNormalDriveAcceleration = 0f;
        totalPassiveNormalDampingAcceleration = 0f;
        controllerPassiveDampingAcceleration = 0f;
        rigidbodyPassiveDampingAcceleration = 0f;
        rigidbodyPassiveDampingAppliedAcceleration = 0f;
        rigidbodyPassiveDampingShareApplied01 = 0f;

        spatialRawDomainProgress01 = 0f;
        spatialDomainProgress01 = 0f;
        spatialDomainAdvanceRate01PerSecond = 0f;
        spatialDomainAccelerationRate01PerSecond2 = 0f;
        previousSpatialDomainAdvanceRate01PerSecond = 0f;
        spatialSpeedRatio = 1f;
        spatialAccelerationBudgetScale = 1f;
        spatialJerkBudgetScale = 1f;
        spatialCurvatureMeasuredForwardSpeed = 0f;
        spatialCurvatureDesignSpeed = 0f;
        spatialCurvatureLookAheadDistance = 0f;
        spatialCurvatureWaveNumberPerMeter = 0f;
        spatialCurvatureKappaMaxPerMeter = 0f;
        spatialCurvatureRequestedHeightMeters = 0f;
        spatialCurvatureAllowedHeightMeters = 0f;
        spatialCurvatureClampRatio01 = 1f;
        spatialCurvatureLimited = false;
        spatialFeedForwardVelocity = 0f;
        spatialFeedForwardAcceleration = 0f;
        terminalSpatialTransportAssist01 = 0f;
        spatialMonotonicHoldActive = false;
        spatialWavePhase01 = 0f;
        spatialReferencePeriodSeconds = 0f;
        spatialPathHalfPeriodSeconds = 0f;
        spatialGuidedHalfPeriodSeconds = 0f;
        spatialPathAngularSpeed = 0f;
        spatialGuidedAngularSpeed = 0f;
        spatialHalfPeriodFeedForwardBlendApplied01 = 0f;
        spatialCarrierHeightMeters = 0f;
        spatialCarrierFeasibility01 = 1f;
        spatialCarrierHeightBeforeFeasibility = 0f;
        spatialSubjectTimeToGo = 0f;
        spatialRequiredArrivalSpeed = 0f;
        spatialArrivalFeasibility01 = 1f;

        pendingUpperImpactEnergyMeasurement = false;

        pendingUpperReflectionResolveFrames = 0;
        pendingUpperIncomingNormalSpeed = 0f;
        ResetPostUpperDirectionalMemory();

        rideAccelerationState = Vector3.zero;
        transportAccelerationState = Vector3.zero;
        goalPlanarVelocityState = Vector3.zero;
        ResetRideSupportKinematics();

        oscillationFrame = default;

        if (negativeEnvelope)
        {
            negativeEnvelope.ConfigureSpatialWaveAuthority(
                false,
                Mathf.Max(
                    1,
                    spatialWavesPerStair));
        }

        ballVisualEqualizer.useGravity = false;
        ballVisualEqualizer.detectCollisions = false;

        if (!ballVisualEqualizer.isKinematic)
        {
            ballVisualEqualizer.velocity =
                Vector3.zero;

            ballVisualEqualizer.angularVelocity =
                Vector3.zero;
        }

        ballVisualEqualizer.isKinematic =
            true;

        CopyBallVisualPose();

    }

// ================================================================
// Natural Entry Connect / Release
// ================================================================

    /// <summary>
    /// Plane -> Stair入口で呼ぶ新しい唯一の入口。
    /// BallVisualの現在Rigidbody状態をNフレーム平均し、その実測速度をそのままEqualizerへ渡します。
    /// 初速生成、Projectile逆算、E->v再構成は行いません。
    /// </summary>
    public bool BeginNaturalConnectFromBallVisual(
        float envelopeEntryHeight,
        Vector3 preferredNormal,
        Vector3 preferredTangent)
    {
        ResolveReferences();

        if (postTurnHermiteBridgeActive)
        {
            return false;
        }

        if (!ballVisual ||
            !ballVisualEqualizer ||
            !negativeEnvelope)
        {
            Debug.LogError(
                "[EQUALIZER] Natural Connect references are missing.",
                this);
            return false;
        }

        if (!synchronized)
            ReacquireForNextIncident();

        naturalConnectReferenceHeight =
            Mathf.Max(0.01f, envelopeEntryHeight);

        naturalConnectPreferredNormal =
            preferredNormal.sqrMagnitude > Epsilon
                ? preferredNormal.normalized
                : Vector3.up;

        if (Vector3.Dot(naturalConnectPreferredNormal, Vector3.up) < 0f)
            naturalConnectPreferredNormal = -naturalConnectPreferredNormal;

        naturalConnectPreferredTangent =
            Vector3.ProjectOnPlane(
                preferredTangent,
                naturalConnectPreferredNormal);

        if (naturalConnectPreferredTangent.sqrMagnitude <= Epsilon)
        {
            naturalConnectPreferredTangent =
                Vector3.ProjectOnPlane(
                    ballVisual.velocity,
                    naturalConnectPreferredNormal);
        }

        if (naturalConnectPreferredTangent.sqrMagnitude <= Epsilon)
            naturalConnectPreferredTangent = Vector3.forward;

        naturalConnectPreferredTangent.Normalize();

        naturalConnectSamplingActive = true;
        naturalConnectBlendActive = false;
        naturalConnectSampleCount = 0;
        naturalConnectSamplingStartTime = Time.fixedTime;
        naturalConnectBlendStartTime = -1f;
        naturalConnectSamplingElapsed = 0f;
        naturalConnectBlendElapsed = 0f;
        naturalConnectControlAuthority01 = 0f;
        naturalConnectVelocitySum = Vector3.zero;
        naturalConnectSubjectVelocitySum = Vector3.zero;
        naturalConnectNormalSum = Vector3.zero;
        naturalConnectTangentSum = Vector3.zero;
        naturalConnectAveragedVelocity = ballVisual.velocity;
        naturalConnectAveragedSubjectVelocity = ReadSubjectVelocityVisual();
        naturalConnectAveragedNormal = naturalConnectPreferredNormal;
        naturalConnectAveragedTangent = naturalConnectPreferredTangent;
        naturalConnectMeasuredRelativeNormalSpeed = 0f;
        naturalConnectMeasuredEnergyJoule = 0f;

        ResetContinuousNormalSupportHandoff();
        CaptureContinuousNormalSupportSample();

        // Sampling中はEqualizerをBallVisualへ完全同期。ここでは独立Forceを出さない。
        // 押さえ込み値はREAD ONLYで記録するだけで、ReleaseまではForceを追加しない。
        synchronized = true;
        phase = EqualizerPhase.Synchronized;
        CopyBallVisualPose();


        return true;
    }


    // Compatibility only. New BallVisualSlopeDrive does not use Energy reconstruction.
    [System.Obsolete("Use BeginNaturalConnectFromBallVisual. Projectile/Energy release reconstruction is retired.")]
    public bool ReleaseToEnvelopeSimulation(
        Vector3 equalizerLaunchVelocity,
        float sourceEnergyJoule,
        float envelopeEntryHeight)
    {
        Vector3 inferredAxis =
            equalizerLaunchVelocity - ReadSubjectVelocityVisual();

        if (inferredAxis.sqrMagnitude <= Epsilon)
            inferredAxis = Vector3.up;

        return BeginNaturalConnectFromBallVisual(
            envelopeEntryHeight,
            inferredAxis,
            equalizerLaunchVelocity);
    }


    [System.Obsolete("Use BeginNaturalConnectFromBallVisual. Projectile/Energy release reconstruction is retired.")]
    public bool ReleaseToEnvelopeSimulation(
        Vector3 equalizerLaunchVelocity,
        float sourceEnergyJoule,
        float envelopeEntryHeight,
        Vector3 sourceEnergyAxisVisual)
    {
        return BeginNaturalConnectFromBallVisual(
            envelopeEntryHeight,
            sourceEnergyAxisVisual,
            equalizerLaunchVelocity);
    }


    #region PROTECTED_01_NaturalConnectSampling
    // [MOTION-PROTECTED] UpdateNaturalConnectSampling
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void UpdateNaturalConnectSampling()
    {
        if (!naturalConnectSamplingActive ||
            !ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        CopyBallVisualPose();
        CaptureContinuousNormalSupportSample();

        Vector3 sampleNormal = naturalConnectPreferredNormal;
        Vector3 sampleTangent = naturalConnectPreferredTangent;

        if (negativeEnvelope &&
            negativeEnvelope.TryGetReleaseSurfaceFrameVisual(
                out Vector3 releaseTangent,
                out Vector3 releaseNormal))
        {
            if (releaseNormal.sqrMagnitude > Epsilon)
                sampleNormal = releaseNormal.normalized;

            if (Vector3.Dot(sampleNormal, naturalConnectPreferredNormal) < 0f)
                sampleNormal = -sampleNormal;

            Vector3 projectedTangent =
                Vector3.ProjectOnPlane(
                    releaseTangent,
                    sampleNormal);

            if (projectedTangent.sqrMagnitude > Epsilon)
                sampleTangent = projectedTangent.normalized;
        }

        if (Vector3.Dot(sampleTangent, naturalConnectPreferredTangent) < 0f)
            sampleTangent = -sampleTangent;

        Vector3 sampleVelocity = ballVisual.velocity;
        Vector3 sampleSubjectVelocity = ReadSubjectVelocityVisual();

        naturalConnectVelocitySum += sampleVelocity;
        naturalConnectSubjectVelocitySum += sampleSubjectVelocity;
        naturalConnectNormalSum += sampleNormal;
        naturalConnectTangentSum += sampleTangent;
        naturalConnectSampleCount++;

        float inverseCount =
            1f / Mathf.Max(1, naturalConnectSampleCount);

        naturalConnectAveragedVelocity =
            naturalConnectVelocitySum * inverseCount;

        naturalConnectAveragedSubjectVelocity =
            naturalConnectSubjectVelocitySum * inverseCount;

        naturalConnectAveragedNormal =
            naturalConnectNormalSum.sqrMagnitude > Epsilon
                ? naturalConnectNormalSum.normalized
                : naturalConnectPreferredNormal;

        naturalConnectAveragedTangent =
            Vector3.ProjectOnPlane(
                naturalConnectTangentSum,
                naturalConnectAveragedNormal);

        if (naturalConnectAveragedTangent.sqrMagnitude <= Epsilon)
            naturalConnectAveragedTangent = naturalConnectPreferredTangent;
        else
            naturalConnectAveragedTangent.Normalize();

        Vector3 averagedRelativeVelocity =
            naturalConnectAveragedVelocity -
            naturalConnectAveragedSubjectVelocity;

        naturalConnectMeasuredRelativeNormalSpeed =
            Vector3.Dot(
                averagedRelativeVelocity,
                naturalConnectAveragedNormal);

        naturalConnectMeasuredEnergyJoule =
            0.5f *
            EqualizerMass *
            naturalConnectMeasuredRelativeNormalSpeed *
            naturalConnectMeasuredRelativeNormalSpeed;

        naturalConnectSamplingElapsed =
            Mathf.Max(
                0f,
                Time.fixedTime - naturalConnectSamplingStartTime);

        bool enoughFrames =
            naturalConnectSampleCount >=
            Mathf.Max(2, naturalConnectAverageFrames);

        bool enoughTime =
            naturalConnectSamplingElapsed >=
            Mathf.Max(0.02f, naturalConnectAverageSeconds);

        bool maximumWaitReached =
            naturalConnectSamplingElapsed >=
            Mathf.Max(
                naturalConnectAverageSeconds,
                naturalConnectMaximumWaitSeconds);

        if ((enoughFrames && enoughTime) || maximumWaitReached)
            CommitNaturalConnectRelease();
    }
    #endregion // PROTECTED_01_NaturalConnectSampling


    #region PROTECTED_02_NaturalConnectRelease
    // [MOTION-PROTECTED] CommitNaturalConnectRelease
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void CommitNaturalConnectRelease()
    {
        if (!naturalConnectSamplingActive ||
            !ballVisual ||
            !ballVisualEqualizer ||
            !negativeEnvelope)
        {
            return;
        }

        Vector3 normal = naturalConnectAveragedNormal;
        if (normal.sqrMagnitude <= Epsilon)
            normal = naturalConnectPreferredNormal;
        if (normal.sqrMagnitude <= Epsilon)
            normal = Vector3.up;
        normal.Normalize();
        if (Vector3.Dot(normal, Vector3.up) < 0f)
            normal = -normal;

        Vector3 tangent =
            Vector3.ProjectOnPlane(
                naturalConnectAveragedTangent,
                normal);

        if (tangent.sqrMagnitude <= Epsilon)
        {
            tangent =
                Vector3.ProjectOnPlane(
                    naturalConnectAveragedVelocity,
                    normal);
        }

        if (tangent.sqrMagnitude <= Epsilon)
            tangent = Vector3.forward;
        tangent.Normalize();

        Vector3 lateral =
            Vector3.Cross(normal, tangent).normalized;

        Vector3 measuredVelocity =
            naturalConnectAveragedVelocity;

        Vector3 measuredSubjectVelocity =
            naturalConnectAveragedSubjectVelocity;

        float measuredRelativeNormalSpeed =
            Vector3.Dot(
                measuredVelocity - measuredSubjectVelocity,
                normal);

        float measuredNormalMagnitude =
            Mathf.Abs(measuredRelativeNormalSpeed);

        float envelopeNormalSpeed =
            Mathf.Max(
                measuredNormalMagnitude,
                Mathf.Max(0.001f, naturalConnectMinimumEnvelopeNormalSpeed));

        // Geometry arm floor only. This Energy is never converted back into
        // Rigidbody initial speed, so it cannot create an artificial jump.
        float envelopeEnergyJoule =
            0.5f *
            EqualizerMass *
            envelopeNormalSpeed *
            envelopeNormalSpeed;

        naturalConnectMeasuredRelativeNormalSpeed =
            measuredRelativeNormalSpeed;

        naturalConnectMeasuredEnergyJoule =
            0.5f *
            EqualizerMass *
            measuredRelativeNormalSpeed *
            measuredRelativeNormalSpeed;

        sourceCanonicalNormalSpeed = measuredNormalMagnitude;
        logicalLaunchNormalSpeed = measuredNormalMagnitude;
        logicalLaunchEnergyJoule = envelopeEnergyJoule;
        logicalLaunchAppliedMultiplier = 1f;
        logicalLaunchReferenceTangentSpeed =
            Mathf.Abs(Vector3.Dot(measuredVelocity, tangent));

        CopyBallVisualPose();

        Vector3 releasePosition = ballVisual.position;
        Quaternion releaseRotation = ballVisual.rotation;
        Vector3 releaseAngularVelocity = ballVisual.angularVelocity;

        bool envelopeReady =
            negativeEnvelope.ArmFromBallVisualEnergy(
                envelopeEnergyJoule,
                Mathf.Max(0.01f, naturalConnectReferenceHeight),
                measuredVelocity,
                normal,
                1f);

        if (!envelopeReady)
        {
            Debug.LogWarning(
                "[EQUALIZER] Natural Connect envelope arm failed.",
                this);

            naturalConnectSamplingActive = false;
            EnterSynchronizedState();
            return;
        }

        if (negativeEnvelope.TryGetLatestOscillationFrameVisual(
                out Vector3 envelopeTangent,
                out Vector3 envelopeNormal))
        {
            if (envelopeNormal.sqrMagnitude > Epsilon)
                normal = envelopeNormal.normalized;

            if (Vector3.Dot(normal, Vector3.up) < 0f)
                normal = -normal;

            if (envelopeTangent.sqrMagnitude > Epsilon)
            {
                tangent =
                    Vector3.ProjectOnPlane(
                        envelopeTangent,
                        normal);

                if (tangent.sqrMagnitude > Epsilon)
                    tangent.Normalize();
            }

            lateral = Vector3.Cross(normal, tangent).normalized;
        }

        oscillationFrame =
            new OscillationFrame
            {
                valid = true,
                tangent = tangent,
                normal = normal,
                lateral = lateral
            };

        releaseFrame =
            new ReleaseFrame
            {
                position = releasePosition,
                subjectPosition = ReadSubjectPositionVisual(),
                velocity = measuredVelocity,
                angularVelocity = releaseAngularVelocity,
                sourceEnergy = naturalConnectMeasuredEnergyJoule,
                referenceHeight = Mathf.Max(0.01f, naturalConnectReferenceHeight),
                sourceAxis = normal
            };

        synchronized = false;
        phase = EqualizerPhase.ReleaseArmed;

        ballVisualEqualizer.isKinematic = false;
        ballVisualEqualizer.detectCollisions = true;
        ballVisualEqualizer.useGravity = true;
        ballVisualEqualizer.position = releasePosition;
        ballVisualEqualizer.rotation = releaseRotation;

        // Critical rule: measured average velocity is copied directly.
        // No E->sqrt(2E/m), POP multiplier or vertical reconstruction here.
        ballVisualEqualizer.velocity = measuredVelocity;
        ballVisualEqualizer.angularVelocity = releaseAngularVelocity;
        ballVisualEqualizer.collisionDetectionMode =
            CollisionDetectionMode.ContinuousDynamic;
        ballVisualEqualizer.solverIterations =
            Mathf.Max(ballVisualEqualizer.solverIterations, 12);
        ballVisualEqualizer.solverVelocityIterations =
            Mathf.Max(ballVisualEqualizer.solverVelocityIterations, 4);
        ballVisualEqualizer.WakeUp();

        rideAccelerationState = Vector3.zero;
        transportAccelerationState = Vector3.zero;
        goalPlanarVelocityState =
            Vector3.ProjectOnPlane(measuredVelocity, normal);
        ResetRideSupportKinematics();

        waveCycleIndex = 0;
        observedNaturalPeriodSeconds = 0f;
        lastUpperContactFixedTime = -1f;
        ResetGen3State(naturalConnectMeasuredEnergyJoule);
        gen3EventPhase = Gen3EventPhase.Flight;
        pendingUpperImpactEnergyMeasurement = false;
        pendingUpperReflectionResolveFrames = 0;
        pendingUpperIncomingNormalSpeed = 0f;

        physicalLowerContacts.Clear();
        upperPeakArmed = true;
        physicalUpperSeenSinceLastLower = false;
        physicalLowerContactActive = false;
        rideSupportFrameContinuous = true;
        pendingPhysicalLowerImpactEnergyMeasurement = false;
        pendingPhysicalLowerIncomingNormalSpeed = 0f;
        pendingPhysicalLowerEnergyFrames = 0;
        pendingPhysicalLowerBestOutgoingNormalSpeed = 0f;
        lastPhysicalLowerIncomingNormalSpeed = 0f;
        lastPhysicalLowerOutgoingNormalSpeed = 0f;
        lastPhysicalLowerEnergyRetention01 = 1f;
        normalAuthorityZone = NormalAuthorityZone.LogicalEnvelope;
        previousNormalAuthorityZone = NormalAuthorityZone.LogicalEnvelope;
        normalLogicalAuthority01 = 0f;
        normalTargetAccelerationBeforeAuthority = 0f;
        normalTargetAccelerationAfterAuthority = 0f;
        normalActiveJerkLimit = 0f;
        effectiveRideSpringDamper = rideSpringDamper;
        activeTargetNormalDriveAcceleration = 0f;
        totalPassiveNormalDampingAcceleration = 0f;
        controllerPassiveDampingAcceleration = 0f;
        rigidbodyPassiveDampingAcceleration = 0f;
        rigidbodyPassiveDampingAppliedAcceleration = 0f;
        rigidbodyPassiveDampingShareApplied01 = 0f;

        naturalConnectSamplingActive = false;
        naturalConnectBlendActive = true;
        naturalConnectBlendStartTime = Time.fixedTime;
        naturalConnectBlendElapsed = 0f;
        naturalConnectControlAuthority01 = 0f;
        ArmContinuousNormalSupportHandoff();

        UpdateWaveTimingAuthority();

        negativeEnvelope.SetUpperEnvelopeSolidEnabled(
            true,
            "NaturalConnectRelease");

        negativeEnvelope.RefreshEqualizerBoundaryCollisionOwnership();
        if (equalizerControlMode == EqualizerControlMode.Gen3VirtualLower)
            RefreshGen3CollisionOwnership();
        phase = EqualizerPhase.FreeFlight;

        if (logMeasurements)
        {
            Debug.Log(
                $"[EQ] entry vN={measuredRelativeNormalSpeed:F3}m/s " +
                $"E={naturalConnectMeasuredEnergyJoule:F3}J " +
                $"H={naturalConnectReferenceHeight:F3}m",
                this);
        }

    }
    #endregion // PROTECTED_02_NaturalConnectRelease


    #region PROTECTED_03_NaturalConnectBlend
    // [MOTION-PROTECTED] UpdateNaturalConnectBlend
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void UpdateNaturalConnectBlend()
    {
        if (!naturalConnectBlendActive)
        {
            naturalConnectControlAuthority01 = 1f;
            naturalConnectBlendElapsed = 0f;
            return;
        }

        naturalConnectBlendElapsed =
            Mathf.Max(
                0f,
                Time.fixedTime - naturalConnectBlendStartTime);

        float duration =
            Mathf.Max(0.02f, naturalConnectBlendSeconds);

        float u =
            Mathf.Clamp01(
                naturalConnectBlendElapsed / duration);

        // Quintic smootherstep: 10u^3 - 15u^4 + 6u^5.
        // value, first derivative and second derivative are zero at both ends.
        float u2 = u * u;
        float u3 = u2 * u;
        naturalConnectControlAuthority01 =
            u3 * (10f - 15f * u + 6f * u2);

        if (u >= 1f)
        {
            naturalConnectBlendActive = false;
            naturalConnectControlAuthority01 = 1f;
            CompleteContinuousNormalSupportHandoff("NaturalConnectComplete");

        }
    }
    #endregion // PROTECTED_03_NaturalConnectBlend


    private void ResetNaturalConnectRuntime()
    {
        naturalConnectSamplingActive = false;
        naturalConnectBlendActive = false;
        naturalConnectSampleCount = 0;
        naturalConnectSamplingStartTime = -1f;
        naturalConnectBlendStartTime = -1f;
        naturalConnectSamplingElapsed = 0f;
        naturalConnectBlendElapsed = 0f;
        naturalConnectReferenceHeight = 0f;
        naturalConnectVelocitySum = Vector3.zero;
        naturalConnectSubjectVelocitySum = Vector3.zero;
        naturalConnectNormalSum = Vector3.zero;
        naturalConnectTangentSum = Vector3.zero;
        naturalConnectAveragedVelocity = Vector3.zero;
        naturalConnectAveragedSubjectVelocity = Vector3.zero;
        naturalConnectAveragedNormal = Vector3.up;
        naturalConnectAveragedTangent = Vector3.forward;
        naturalConnectMeasuredRelativeNormalSpeed = 0f;
        naturalConnectMeasuredEnergyJoule = 0f;
        ResetContinuousNormalSupportHandoff();
    }

    #region PROTECTED_04_ResetContinuousNormalSupport
    // [MOTION-PROTECTED] ResetContinuousNormalSupportHandoff
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void ResetContinuousNormalSupportHandoff()
    {
        continuousNormalSupportCaptured = false;
        continuousNormalSupportActive = false;
        continuousNormalSupportCapturedAcceleration = 0f;
        continuousNormalSupportWeight01 = 0f;
        continuousNormalSupportAppliedAcceleration = 0f;
        continuousNormalSupportEndReason = "None";
    }
    #endregion // PROTECTED_04_ResetContinuousNormalSupport


    #region PROTECTED_05_CaptureContinuousNormalSupport
    // [MOTION-PROTECTED] CaptureContinuousNormalSupportSample
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void CaptureContinuousNormalSupportSample()
    {
        if (!useContinuousNormalSupportHandoff || !slopeCore)
            return;

        if (!slopeCore.BallVisualNormalSupportAvailableReadOnly)
            return;

        float sampledAcceleration =
            Mathf.Clamp(
                slopeCore.BallVisualNormalSupportAccelerationReadOnly,
                0f,
                Mathf.Max(1f, continuousNormalSupportMaximumAcceleration));

        if (sampledAcceleration <= Epsilon)
            return;

        // Sampling中にReleaseが始まってStickが減っても、入口直前に実際に
        // 存在したSupportを失わないよう最大観測値を保持する。
        continuousNormalSupportCapturedAcceleration =
            Mathf.Max(
                continuousNormalSupportCapturedAcceleration,
                sampledAcceleration);
        continuousNormalSupportCaptured = true;
    }
    #endregion // PROTECTED_05_CaptureContinuousNormalSupport


    #region PROTECTED_06_ArmContinuousNormalSupport
    // [MOTION-PROTECTED] ArmContinuousNormalSupportHandoff
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void ArmContinuousNormalSupportHandoff()
    {
        continuousNormalSupportActive =
            useContinuousNormalSupportHandoff &&
            continuousNormalSupportCaptured &&
            continuousNormalSupportCapturedAcceleration > Epsilon;

        continuousNormalSupportWeight01 =
            continuousNormalSupportActive ? 1f : 0f;
        continuousNormalSupportAppliedAcceleration = 0f;
        continuousNormalSupportEndReason =
            continuousNormalSupportActive ? "Active" : "NoCapturedSupport";
    }
    #endregion // PROTECTED_06_ArmContinuousNormalSupport


    #region PROTECTED_07_CompleteContinuousNormalSupport
    // [MOTION-PROTECTED] CompleteContinuousNormalSupportHandoff
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void CompleteContinuousNormalSupportHandoff(string reason)
    {
        continuousNormalSupportActive = false;
        continuousNormalSupportWeight01 = 0f;
        continuousNormalSupportAppliedAcceleration = 0f;
        continuousNormalSupportEndReason =
            string.IsNullOrEmpty(reason) ? "Complete" : reason;
    }
    #endregion // PROTECTED_07_CompleteContinuousNormalSupport


    #region PROTECTED_08_ResolveContinuousNormalSupport
    // [MOTION-PROTECTED] ResolveContinuousNormalSupportAcceleration
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private float ResolveContinuousNormalSupportAcceleration(
        float safetyAuthority01)
    {
        if (!continuousNormalSupportActive ||
            physicalUpperSeenSinceLastLower)
        {
            continuousNormalSupportWeight01 = 0f;
            continuousNormalSupportAppliedAcceleration = 0f;
            return 0f;
        }

        // Existing Equalizer authority opens 0->1. Entry support uses the exact
        // complement 1->0, so total Normal ownership never has the old zero gap
        // and never stacks two full controllers at once.
        float weight01 =
            1f - Mathf.Clamp01(naturalConnectControlAuthority01);

        continuousNormalSupportWeight01 = weight01;

        float acceleration =
            -continuousNormalSupportCapturedAcceleration *
            weight01 *
            Mathf.Clamp01(safetyAuthority01);

        continuousNormalSupportAppliedAcceleration = acceleration;
        return acceleration;
    }
    #endregion // PROTECTED_08_ResolveContinuousNormalSupport


    private void ApplyContinuousNormalSupportForGen3(Vector3 normal)
    {
        if (!continuousNormalSupportActive ||
            !ballVisualEqualizer ||
            normal.sqrMagnitude <= Epsilon)
        {
            return;
        }

        // Gen3 replaces the normal Lower controller, so the entry bridge is
        // applied separately only on the initial free-flight leg. It is never
        // active after the first Physical Upper.
        float acceleration =
            ResolveContinuousNormalSupportAcceleration(1f);

        if (Mathf.Abs(acceleration) <= Epsilon)
            return;

        ballVisualEqualizer.AddForce(
            normal.normalized * acceleration,
            ForceMode.Acceleration);
    }

// ================================================================
// Gen3 Virtual Lower / Poincare / ILC
// ================================================================

    private void ResetGen3State(float availableEnergyJoule)
    {
        gen3EventPhase = synchronized ? Gen3EventPhase.Synchronized : Gen3EventPhase.Flight;
        gen3IlcCorrectionR = 0f;
        gen3ObservedUpperSpanR = 0f;
        gen3CurrentCycleTargetSpanR = ComputeGen3CycleTargetSpanR(0);
        gen3Impedance01 = 0f;
        gen3AdaptiveSubsteps = 1;
        gen3GovernedNormalSpeed = 0f;
        gen3NormalAccelerationState = 0f;
        gen3VirtualLowerForceNewton = 0f;
        gen3VirtualLowerPenetrationMeters = 0f;
        gen3PassivityScale01 = 1f;
        gen3VirtualLowerWasActive = false;

        gen3EnergyTankCapacityJoule = Mathf.Max(
            gen3MinimumEnergyTankCapacityJoule,
            Mathf.Max(0f, availableEnergyJoule) * Mathf.Max(0.25f, gen3EnergyTankCapacityMultiplier));

        gen3EnergyTankJoule =
            gen3EnergyTankCapacityJoule * Mathf.Clamp01(gen3InitialEnergyTankFill01);
    }

    private float ComputeGen3CycleTargetSpanR(int cycleIndex)
    {
        int n = Mathf.Max(0, cycleIndex);
        float baseSpan = Mathf.Max(0.01f, gen3BaseSpanR);
        float retention = Mathf.Clamp(gen3SpanRetention01, 0.01f, 1f);
        float nominal = baseSpan * Mathf.Pow(retention, n);
        float minimum = Mathf.Min(baseSpan, Mathf.Max(0.01f, gen3MinimumSpanR));
        return Mathf.Clamp(nominal, minimum, baseSpan);
    }

    private float ComputeGen3Impedance01(float penetrationMeters, float radius)
    {
        float safeRadius = Mathf.Max(0.0001f, radius);
        float start = Mathf.Max(0f, gen3ImpedanceStartR) * safeRadius;
        float full = Mathf.Max(start + 0.0001f, gen3ImpedanceFullR * safeRadius);
        float u = Mathf.InverseLerp(start, full, Mathf.Max(0f, penetrationMeters));
        float u2 = u * u;
        float u3 = u2 * u;
        return u3 * (10f - 15f * u + 6f * u2);
    }

    private float ComputeGen3ReferenceGovernedNormalSpeed(
        float requestedPositiveSpeed,
        float availableDistanceMeters,
        float maximumAcceleration)
    {
        float distance = Mathf.Max(0f, availableDistanceMeters);
        float acceleration = Mathf.Max(0f, maximumAcceleration);
        float kinematicLimit = Mathf.Sqrt(Mathf.Max(0f, 2f * acceleration * distance));
        return Mathf.Min(
            Mathf.Max(0f, requestedPositiveSpeed),
            kinematicLimit * Mathf.Clamp01(gen3ReferenceGovernorSafety01));
    }

    private static float ApplyGen3NormalJerkLimit(
        float previousAcceleration,
        float requestedAcceleration,
        float maximumJerk,
        float dt)
    {
        float maxDelta = Mathf.Max(0f, maximumJerk) * Mathf.Max(0f, dt);
        return previousAcceleration + Mathf.Clamp(
            requestedAcceleration - previousAcceleration,
            -maxDelta,
            maxDelta);
    }

    private static float Gen3ImplicitSoftConstraintForce(
        float x,
        float normalVelocity,
        float mass,
        float dt,
        float frequencyHz,
        float dampingRatio)
    {
        if (x >= 0f || mass <= Epsilon || dt <= Epsilon)
            return 0f;

        float omega = 2f * Mathf.PI * Mathf.Max(0.01f, frequencyHz);
        float k = mass * omega * omega;
        float c = 2f * Mathf.Max(0f, dampingRatio) * mass * omega;
        float denominator = mass / dt + k * dt + c;
        if (denominator <= Epsilon)
            return 0f;

        float vNext = (mass * normalVelocity / dt - k * x) / denominator;
        float impulse = mass * (vNext - normalVelocity);
        return Mathf.Max(0f, impulse / dt);
    }

    private static float Gen3HuntCrossleyForce(
        float penetration,
        float compressionSpeed,
        float mass,
        float radius,
        float frequencyHz,
        float exponent,
        float dissipation,
        float referenceCompressionR)
    {
        if (penetration <= 0f || mass <= Epsilon)
            return 0f;

        float p = Mathf.Clamp(exponent, 1f, 2.5f);
        float omega = 2f * Mathf.PI * Mathf.Max(0.01f, frequencyHz);
        float linearK = mass * omega * omega;
        float referenceCompression = Mathf.Max(
            0.001f,
            radius * Mathf.Max(0.02f, referenceCompressionR));
        float nonlinearK = linearK / Mathf.Pow(referenceCompression, p - 1f);
        float elastic = nonlinearK * Mathf.Pow(penetration, p);
        float dampingFactor = Mathf.Max(
            0f,
            1f + Mathf.Max(0f, dissipation) * compressionSpeed);
        return Mathf.Max(0f, elastic * dampingFactor);
    }

    private Vector3 ApplyGen3PassivityLimit(
        Vector3 requestedForce,
        Vector3 relativeVelocity,
        float dt)
    {
        if (!gen3UseEnergyTank || requestedForce.sqrMagnitude <= Epsilon || dt <= Epsilon)
        {
            gen3PassivityScale01 = 1f;
            return requestedForce;
        }

        float power = Vector3.Dot(requestedForce, relativeVelocity);
        if (power <= 0f)
        {
            float recovered = -power * dt * Mathf.Clamp01(gen3EnergyRecoveryEfficiency01);
            gen3EnergyTankJoule = Mathf.Min(
                gen3EnergyTankCapacityJoule,
                gen3EnergyTankJoule + recovered);
            gen3PassivityScale01 = 1f;
            return requestedForce;
        }

        float requestedEnergy = power * dt;
        if (requestedEnergy <= Epsilon)
        {
            gen3PassivityScale01 = 1f;
            return requestedForce;
        }

        float scale = Mathf.Clamp01(gen3EnergyTankJoule / requestedEnergy);
        gen3EnergyTankJoule = Mathf.Max(
            0f,
            gen3EnergyTankJoule - requestedEnergy * scale);
        gen3PassivityScale01 = scale;
        return requestedForce * scale;
    }

    private float SolveGen3AdaptiveLowerImpulse(
        float initialConstraintCoordinate,
        float initialNormalVelocity,
        float mass,
        float dt,
        float radius,
        float availableDistanceMeters,
        out float impedance01,
        out int substeps,
        out float governedNormalSpeed)
    {
        impedance01 = 0f;
        governedNormalSpeed = Mathf.Max(0f, initialNormalVelocity);

        if (mass <= Epsilon || dt <= Epsilon || radius <= Epsilon)
        {
            substeps = 1;
            return 0f;
        }

        float chi = Mathf.Abs(initialNormalVelocity) * dt / radius;
        float chiMax = Mathf.Max(0.01f, gen3AdaptiveSubstepChiMax);
        substeps = Mathf.Clamp(
            Mathf.CeilToInt(Mathf.Max(1f, chi / chiMax)),
            1,
            Mathf.Max(1, gen3MaximumInternalSubsteps));

        float subDt = dt / substeps;
        float x = initialConstraintCoordinate;
        float v = initialNormalVelocity;
        float accelerationState = gen3NormalAccelerationState;
        float accumulatedImpulse = 0f;
        float maxAcceleration = Mathf.Max(0f, gen3MaximumLowerAcceleration);

        float targetSpan = Mathf.Max(0.25f, gen3CurrentCycleTargetSpanR);
        float ilcGainMultiplier = Mathf.Clamp(
            1f + gen3IlcCorrectionR / targetSpan,
            0.65f,
            1.35f);

        for (int i = 0; i < substeps; i++)
        {
            float penetration = Mathf.Max(0f, -x);
            float z = ComputeGen3Impedance01(penetration, radius);
            impedance01 = Mathf.Max(impedance01, z);

            if (penetration <= Epsilon || z <= Epsilon)
            {
                x += v * subDt;
                continue;
            }

            float implicitForce = Gen3ImplicitSoftConstraintForce(
                x,
                v,
                mass,
                subDt,
                gen3LowerFrequencyHz,
                gen3LowerDampingRatio);

            float huntForce = Gen3HuntCrossleyForce(
                penetration,
                Mathf.Max(0f, -v),
                mass,
                radius,
                gen3LowerFrequencyHz,
                gen3HuntExponent,
                gen3HuntDissipation,
                gen3HuntReferenceCompressionR);

            float rawForce = Mathf.Lerp(
                huntForce,
                implicitForce,
                Mathf.Clamp01(gen3ImplicitShare01));

            rawForce *=
                z * ilcGainMultiplier * Mathf.Clamp01(naturalConnectControlAuthority01);

            float requestedAcceleration = Mathf.Clamp(
                rawForce / mass,
                0f,
                maxAcceleration);

            accelerationState = ApplyGen3NormalJerkLimit(
                accelerationState,
                requestedAcceleration,
                gen3MaximumNormalJerk,
                subDt);

            float vRequested = v + accelerationState * subDt;
            if (vRequested > 0f)
            {
                governedNormalSpeed = ComputeGen3ReferenceGovernedNormalSpeed(
                    vRequested,
                    availableDistanceMeters,
                    maxAcceleration);
                vRequested = Mathf.Min(vRequested, governedNormalSpeed);
            }

            float deltaV = Mathf.Max(0f, vRequested - v);
            accumulatedImpulse += mass * deltaV;
            v += deltaV;
            x += v * subDt;
        }

        gen3NormalAccelerationState = Mathf.Max(0f, accelerationState);
        return Mathf.Max(0f, accumulatedImpulse);
    }

    private void UpdateGen3VirtualLower()
    {
        if (!negativeEnvelope || !ballVisualEqualizer)
            return;

        if (!negativeEnvelope.TryGetFloatingRideFrame(
                out Vector3 lowerCenter,
                out _,
                out Vector3 upperCenter,
                out Vector3 tangent,
                out Vector3 normal,
                out float spanMeters,
                out float spanR,
                out float observedPeriod))
        {
            gen3EventPhase = Gen3EventPhase.Flight;
            return;
        }

        if (normal.sqrMagnitude <= Epsilon)
            return;

        normal.Normalize();
        tangent = Vector3.ProjectOnPlane(tangent, normal);
        if (tangent.sqrMagnitude <= Epsilon)
            return;
        tangent.Normalize();

        Vector3 lateral = Vector3.Cross(normal, tangent).normalized;
        oscillationFrame = new OscillationFrame
        {
            valid = true,
            tangent = tangent,
            normal = normal,
            lateral = lateral
        };

        if (observedPeriod > 0f)
            observedNaturalPeriodSeconds = observedPeriod;

        Vector3 supportVelocity = ReadSubjectVelocityVisual();
        Vector3 relativeVelocity = ballVisualEqualizer.velocity - supportVelocity;
        float vN = Vector3.Dot(relativeVelocity, normal);
        float radius = ResolveEqualizerWorldRadius();
        float signedHeight = Vector3.Dot(ballVisualEqualizer.position - lowerCenter, normal);

        rideActualHeight = signedHeight;
        rideRelativeNormalVelocity = vN;

        float activation = Mathf.Max(
            Mathf.Max(0f, gen3LowerActivationR) * radius,
            Mathf.Max(0f, -vN) * Time.fixedDeltaTime * Mathf.Max(0f, gen3LowerLeadSteps));

        float x = signedHeight - activation;
        float penetration = Mathf.Max(0f, -x);
        gen3VirtualLowerPenetrationMeters = penetration;

        if (penetration <= Epsilon)
        {
            ApplyContinuousNormalSupportForGen3(normal);

            gen3VirtualLowerForceNewton = 0f;
            gen3Impedance01 = 0f;
            gen3AdaptiveSubsteps = 1;
            gen3NormalAccelerationState = Mathf.MoveTowards(
                gen3NormalAccelerationState,
                0f,
                Mathf.Max(0f, gen3MaximumNormalJerk) * Time.fixedDeltaTime);

            gen3EventPhase = vN < 0f
                ? Gen3EventPhase.LowerApproach
                : Gen3EventPhase.UpperApproach;
            phase = vN >= 0f
                ? EqualizerPhase.HopperFlight
                : EqualizerPhase.FreeFlight;
            normalAuthorityZone = NormalAuthorityZone.PhysicalFree;
            normalLogicalAuthority01 = 0f;
            gen3VirtualLowerWasActive = false;
            return;
        }

        float targetSpanMeters = Mathf.Min(
            Mathf.Max(radius * gen3MinimumSpanR, spanMeters),
            Mathf.Max(radius * gen3MinimumSpanR, gen3CurrentCycleTargetSpanR * radius));

        float availableDistance = Mathf.Max(
            radius * 0.05f,
            targetSpanMeters - Mathf.Max(0f, signedHeight));

        float impulse = SolveGen3AdaptiveLowerImpulse(
            x,
            vN,
            EqualizerMass,
            Time.fixedDeltaTime,
            radius,
            availableDistance,
            out float impedance01,
            out int substeps,
            out float governedSpeed);

        gen3Impedance01 = impedance01;
        gen3AdaptiveSubsteps = substeps;
        gen3GovernedNormalSpeed = governedSpeed;

        if (impulse > Epsilon)
        {
            Vector3 requestedForce = normal * (impulse / Mathf.Max(Epsilon, Time.fixedDeltaTime));
            requestedForce = ApplyGen3PassivityLimit(
                requestedForce,
                relativeVelocity,
                Time.fixedDeltaTime);

            Vector3 finalImpulse = requestedForce * Time.fixedDeltaTime;
            float maxImpulse = EqualizerMass * Mathf.Max(0f, gen3MaximumLowerAcceleration) * Time.fixedDeltaTime;
            finalImpulse = Vector3.ClampMagnitude(finalImpulse, maxImpulse);

            if (Vector3.Dot(finalImpulse, normal) > 0f)
            {
                ballVisualEqualizer.AddForce(finalImpulse, ForceMode.Impulse);
                gen3VirtualLowerForceNewton = finalImpulse.magnitude / Mathf.Max(Epsilon, Time.fixedDeltaTime);
            }
        }

        gen3EventPhase = Gen3EventPhase.VirtualLowerContact;
        phase = EqualizerPhase.LowerContact;
        normalAuthorityZone = NormalAuthorityZone.LogicalEnvelope;
        normalLogicalAuthority01 = Mathf.Clamp01(gen3Impedance01);

        // No Physical Lower is needed in Gen3. Virtual contact rearms the real Upper.
        upperPeakArmed = true;
        physicalUpperSeenSinceLastLower = false;
        physicalLowerContactActive = false;
        gen3VirtualLowerWasActive = true;
    }

    private void RegisterGen3UpperCycleObservation()
    {
        if (equalizerControlMode != EqualizerControlMode.Gen3VirtualLower ||
            !negativeEnvelope || !ballVisualEqualizer)
            return;

        if (!negativeEnvelope.TryGetFloatingRideFrame(
                out Vector3 lowerCenter,
                out _,
                out _,
                out _,
                out Vector3 normal,
                out _,
                out _,
                out _))
            return;

        if (normal.sqrMagnitude <= Epsilon)
            return;
        normal.Normalize();

        float radius = ResolveEqualizerWorldRadius();
        if (radius <= Epsilon)
            return;

        gen3ObservedUpperSpanR = Mathf.Max(
            0f,
            Vector3.Dot(ballVisualEqualizer.position - lowerCenter, normal) / radius);

        float errorR = gen3CurrentCycleTargetSpanR - gen3ObservedUpperSpanR;
        if (Mathf.Abs(errorR) > Mathf.Max(0f, gen3IlcDeadBandR))
        {
            gen3IlcCorrectionR = Mathf.Clamp(
                gen3IlcCorrectionR + Mathf.Max(0f, gen3IlcGain) * errorR,
                -Mathf.Max(0f, gen3IlcClampR),
                Mathf.Max(0f, gen3IlcClampR));
        }

        gen3CurrentCycleTargetSpanR = ComputeGen3CycleTargetSpanR(waveCycleIndex);
        gen3EventPhase = Gen3EventPhase.PhysicalUpperImpact;

        if (logMeasurements)
        {
            Debug.Log(
                $"[EQ] gen3 cycle={waveCycleIndex} " +
                $"upper={gen3ObservedUpperSpanR:F3}R " +
                $"target={gen3CurrentCycleTargetSpanR:F3}R " +
                $"ilc={gen3IlcCorrectionR:F3}R",
                this);
        }

    }

    private void RefreshGen3CollisionOwnership()
    {
        if (equalizerControlMode != EqualizerControlMode.Gen3VirtualLower || !ballVisualEqualizerCollider)
            return;

        Collider[] sceneColliders = FindObjectsByType<Collider>(
            FindObjectsInactive.Exclude,
            FindObjectsSortMode.None);

        for (int i = 0; i < sceneColliders.Length; i++)
        {
            Collider other = sceneColliders[i];
            if (!other || other == ballVisualEqualizerCollider)
                continue;

            bool isUpper = negativeEnvelope && negativeEnvelope.IsUpperEnvelopeCollider(other);
            Physics.IgnoreCollision(ballVisualEqualizerCollider, other, !isUpper);
            if (!isUpper)
                gen3IgnoredPhysicalColliders.Add(other);
        }

        if (negativeEnvelope)
            negativeEnvelope.RefreshEqualizerBoundaryCollisionOwnership();
    }

    private void RestoreGen3CollisionOwnership()
    {
        if (!ballVisualEqualizerCollider)
            return;

        foreach (Collider other in gen3IgnoredPhysicalColliders)
        {
            if (other)
                Physics.IgnoreCollision(ballVisualEqualizerCollider, other, false);
        }
        gen3IgnoredPhysicalColliders.Clear();
    }

// ================================================================
// Floating Ride Spring
// ================================================================

    #region PROTECTED_09_FloatingRideSpringDamper
    // [MOTION-PROTECTED] UpdateFloatingRideSpring
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void UpdateFloatingRideSpring()
    {
        if (!negativeEnvelope ||
            !ballVisualEqualizer)
        {
            return;
        }

        if (!negativeEnvelope.TryGetFloatingRideFrame(
                out Vector3 lowerCenter,
                out _,
                out Vector3 upperCenter,
                out Vector3 tangent,
                out Vector3 normal,
                out float spanMeters,
                out float spanR,
                out float observedPeriod))
        {
            return;
        }

        if (normal.sqrMagnitude <= Epsilon)
            return;

        normal.Normalize();

        tangent =
            Vector3.ProjectOnPlane(
                tangent,
                normal);

        if (tangent.sqrMagnitude <= Epsilon)
            return;

        tangent.Normalize();

        Vector3 lateral =
            Vector3.Cross(
                normal,
                tangent).normalized;

        oscillationFrame =
            new OscillationFrame
            {
                valid = true,
                tangent = tangent,
                normal = normal,
                lateral = lateral
            };

        if (observedPeriod > 0f)
            observedNaturalPeriodSeconds = observedPeriod;

        Vector3 subjectVelocity =
            ReadSubjectVelocityVisual();

        rideSupportFrameContinuous =
            SampleRideSupportKinematics(
                lowerCenter,
                subjectVelocity,
                out Vector3 supportVelocity,
                out Vector3 supportAcceleration);

        float supportNormalVelocity =
            Vector3.Dot(
                supportVelocity,
                normal);

        float supportNormalAcceleration =
            Vector3.Dot(
                supportAcceleration,
                normal);

        rideSupportNormalVelocity =
            supportNormalVelocity;

        rideSupportNormalAcceleration =
            supportNormalAcceleration;

        float equalizerNormalVelocity =
            Vector3.Dot(
                ballVisualEqualizer.velocity,
                normal);

        rideRelativeNormalVelocity =
            equalizerNormalVelocity -
            supportNormalVelocity;

        rideActualHeight =
            Vector3.Dot(
                ballVisualEqualizer.position -
                lowerCenter,
                normal);

        // Legacy no-handoff mode may rearm from the projected Lower reference height.
        // This is only a phase/rearm reference and never pretends to be a physical impact.
        if (!enableLogicalPhysicalHandoff &&
            !upperPeakArmed &&
            spanMeters > Epsilon &&
            rideActualHeight <=
            spanMeters *
            Mathf.Clamp01(upperPeakRearmHeight01))
        {
            upperPeakArmed = true;
        }

        float targetNormalVelocity = 0f;
        float targetNormalAcceleration = 0f;
        Vector3 targetCenter;

        bool spatialMode =
            waveTimingMode ==
            WaveTimingMode.ThreeWavesPerStair;

        if (spatialMode &&
            TryResolveSpatialCurvatureAdaptiveReference(
                lowerCenter,
                spanMeters,
                tangent,
                normal,
                out targetCenter,
                out targetNormalVelocity,
                out targetNormalAcceleration))
        {
            // target / diagnostics are resolved by spatial progress.
        }
        else
        {
            float equilibrium01 =
                Mathf.Clamp01(
                    rideEquilibrium01);

            targetCenter =
                Vector3.Lerp(
                    lowerCenter,
                    upperCenter,
                    equilibrium01);

            rideTargetHeight =
                spanMeters *
                equilibrium01;

            spatialRawDomainProgress01 = 0f;
            spatialDomainProgress01 = 0f;
            spatialDomainAdvanceRate01PerSecond = 0f;
            spatialMonotonicHoldActive = false;
            spatialWavePhase01 = 0f;
            spatialReferencePeriodSeconds = 0f;
            spatialPathHalfPeriodSeconds = 0f;
            spatialGuidedHalfPeriodSeconds = 0f;
            spatialPathAngularSpeed = 0f;
            spatialGuidedAngularSpeed = 0f;
            spatialHalfPeriodFeedForwardBlendApplied01 = 0f;
            spatialCarrierHeightMeters = 0f;
            spatialCarrierFeasibility01 = 1f;
            spatialCarrierHeightBeforeFeasibility = 0f;
            spatialCurvatureMeasuredForwardSpeed = 0f;
            spatialCurvatureDesignSpeed = 0f;
            spatialCurvatureLookAheadDistance = 0f;
            spatialCurvatureWaveNumberPerMeter = 0f;
            spatialCurvatureKappaMaxPerMeter = 0f;
            spatialCurvatureRequestedHeightMeters = 0f;
            spatialCurvatureAllowedHeightMeters = 0f;
            spatialCurvatureClampRatio01 = 1f;
            spatialCurvatureLimited = false;
        }

        // ============================================================
        // Hybrid Stable-N controller
        // No Lower-specific duplicate Spring/Damper path is stacked here.
        // H / N / T/2 guidance remains in the normal carrier controller.
        // ============================================================

        ridePositionError =
            Vector3.Dot(
                targetCenter -
                ballVisualEqualizer.position,
                normal);

        springAcceleration =
            ridePositionError *
            Mathf.Max(
                0f,
                rideSpringStrength);

        effectiveRideSpringDamper =
            Mathf.Max(
                0f,
                rideSpringDamper);

        float activeTargetNormalVelocity =
            targetNormalVelocity;

        float effectiveDamperGain =
            effectiveRideSpringDamper;

        // C(vTarget-vN) = C*vTarget - C*vN.
        // Only passive -C*vN is split. Height/Spring/target drive/feed-forward
        // remain intact; Natural Connect authority is applied later to the
        // complete logical Normal command.
        activeTargetNormalDriveAcceleration =
            activeTargetNormalVelocity *
            effectiveDamperGain;

        totalPassiveNormalDampingAcceleration =
            -rideRelativeNormalVelocity *
            effectiveDamperGain;

        rigidbodyPassiveDampingShareApplied01 =
            useHybridPassiveNormalDamping
                ? Mathf.Clamp01(rigidbodyPassiveDampingShare01)
                : 0f;

        controllerPassiveDampingAcceleration =
            totalPassiveNormalDampingAcceleration *
            (1f - rigidbodyPassiveDampingShareApplied01);

        rigidbodyPassiveDampingAcceleration =
            totalPassiveNormalDampingAcceleration *
            rigidbodyPassiveDampingShareApplied01;

        damperAcceleration =
            activeTargetNormalDriveAcceleration +
            controllerPassiveDampingAcceleration +
            rigidbodyPassiveDampingAcceleration;

        float gravityAlongNormal =
            ballVisualEqualizer.useGravity
                ? Vector3.Dot(
                    Physics.gravity,
                    normal)
                : 0f;

        gravityCompensationAcceleration =
            -gravityAlongNormal *
            Mathf.Clamp(
                gravityCompensation,
                0f,
                1.5f);

        float desiredScalarAcceleration =
            springAcceleration +
            damperAcceleration +
            gravityCompensationAcceleration;

        if (spatialMode)
        {
            desiredScalarAcceleration +=
                targetNormalAcceleration *
                Mathf.Clamp(
                    spatialWaveFeedForward,
                    0f,
                    1.5f);
        }

        // Curvature adaptation reduces the requested carrier height instead of
        // increasing the force budget with speed.  Keep the existing physical
        // acceleration budget fixed across maxGroundSpeed tests.
        float accelerationLimit =
            spatialMode
                ? Mathf.Max(50f, spatialWaveAccelerationBudget)
                : Mathf.Max(1f, maximumRideAcceleration);

        // ------------------------------------------------------------
        // Impact-excited micro-modal substructure.
        //
        // IMPORTANT:
        // - Main 3-wave carrier is not modified.
        // - G(phi)=sin^2(2*pi*phi) makes the added acceleration exactly
        //   zero at Lower (0/1) and Upper/Apex (0.5).
        // - The result still passes the EXISTING acceleration clamp,
        //   Logical/Physical authority and jerk limiter below.
        // ------------------------------------------------------------
        if (spatialMode)
        {
            // Use the SAME targetNormalVelocity that already drives the
            // original C(vTarget-vN) damper. Micro Modal does not invent a
            // second target velocity; it only reacts to the residual.
            microTargetNormalVelocityReference =
                targetNormalVelocity;

            desiredScalarAcceleration +=
                ResolveMicroModalAcceleration(
                    spanMeters,
                    rideActualHeight,
                    rideRelativeNormalVelocity,
                    accelerationLimit);
        }


        // Relative-coordinate controller:
        //   x_rel = xEqualizer - xSupport
        //   aEqualizer = aSupport + aRelativeCommand
        //
        // Without this term a rotating/moving Visual frame can make the
        // relative error grow even though the Spring/Damper error itself is
        // computed correctly.
        desiredScalarAcceleration +=
            Mathf.Clamp(
                supportNormalAcceleration,
                -accelerationLimit,
                accelerationLimit);

        desiredScalarAcceleration =
            Mathf.Clamp(
                desiredScalarAcceleration,
                -accelerationLimit,
                accelerationLimit);

        float dt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f);

        float logicalJerkLimit =
            spatialMode
                ? Mathf.Max(100f, spatialWaveJerkBudget)
                : Mathf.Max(1f, maximumRideJerk);


        normalTargetAccelerationBeforeAuthority =
            desiredScalarAcceleration;

        float authority01 = 1f;
        float activeJerkLimit = logicalJerkLimit;
        NormalAuthorityZone resolvedZone =
            NormalAuthorityZone.LogicalEnvelope;

        if (enableLogicalPhysicalHandoff)
        {
            ResolveNormalAuthority(
                rideActualHeight,
                logicalJerkLimit,
                rideSupportFrameContinuous,
                out authority01,
                out activeJerkLimit,
                out resolvedZone);
        }

        // Safety/physical handoff remains the outer authority. Natural Connect
        // then crossfades ownership inside that safety envelope:
        //   inherited SlopeStick support : 1 -> 0
        //   existing Equalizer control  : 0 -> 1
        // This closes the entry authority hole without adding a permanent press.
        float safetyAuthority01 = Mathf.Clamp01(authority01);
        float connectAuthority01 = Mathf.Clamp01(naturalConnectControlAuthority01);
        authority01 = safetyAuthority01 * connectAuthority01;

        float inheritedSupportAcceleration =
            ResolveContinuousNormalSupportAcceleration(safetyAuthority01);

        normalLogicalAuthority01 = authority01;
        normalActiveJerkLimit = activeJerkLimit;
        normalAuthorityZone = resolvedZone;
        normalTargetAccelerationAfterAuthority =
            Mathf.Clamp(
                desiredScalarAcceleration * authority01 +
                inheritedSupportAcceleration,
                -accelerationLimit,
                accelerationLimit);

        Vector3 desiredAcceleration =
            normal * normalTargetAccelerationAfterAuthority;

        bool physicalOwnsNormal =
            enableLogicalPhysicalHandoff &&
            (resolvedZone == NormalAuthorityZone.PhysicalFree ||
             resolvedZone == NormalAuthorityZone.PhysicalStairContact);

        if (physicalOwnsNormal)
        {
            // Unexpected real StairWay contact is a safety ownership transfer.
            // Projected Lower geometry by itself never reaches this branch.
            rideAccelerationState = Vector3.zero;
        }
        else
        {
            rideAccelerationState =
                Vector3.MoveTowards(
                    rideAccelerationState,
                    desiredAcceleration,
                    Mathf.Max(1f, activeJerkLimit) * dt);

            rideAccelerationState =
                Vector3.Project(rideAccelerationState, normal);
        }

        rigidbodyPassiveDampingAppliedAcceleration = 0f;

        if (!physicalOwnsNormal)
        {
            float realizedNormalAcceleration =
                Vector3.Dot(rideAccelerationState, normal);

            // Scale the dedicated damping contribution by the same jerk-limited
            // realization ratio as the total command. The two AddForce calls sum
            // exactly to rideAccelerationState, so enabling Hybrid50 does not
            // secretly increase the total Stable-N acceleration.
            float requestedAfterAuthority =
                normalTargetAccelerationAfterAuthority;

            float realizationRatio =
                Mathf.Abs(requestedAfterAuthority) > Epsilon
                    ? Mathf.Clamp01(
                        Mathf.Abs(realizedNormalAcceleration) /
                        Mathf.Abs(requestedAfterAuthority))
                    : 0f;

            rigidbodyPassiveDampingAppliedAcceleration =
                rigidbodyPassiveDampingAcceleration *
                normalLogicalAuthority01 *
                realizationRatio;

            float controllerAppliedAcceleration =
                realizedNormalAcceleration -
                rigidbodyPassiveDampingAppliedAcceleration;

            ballVisualEqualizer.AddForce(
                normal * controllerAppliedAcceleration,
                ForceMode.Acceleration);

            ballVisualEqualizer.AddForce(
                normal * rigidbodyPassiveDampingAppliedAcceleration,
                ForceMode.Acceleration);
        }

        if (normalAuthorityZone != previousNormalAuthorityZone)
        {

            previousNormalAuthorityZone = normalAuthorityZone;
        }


        phase =
            physicalLowerContactActive
                ? EqualizerPhase.LowerContact
                : rideRelativeNormalVelocity >= 0f
                    ? EqualizerPhase.HopperFlight
                    : EqualizerPhase.FreeFlight;
    }
    #endregion // PROTECTED_09_FloatingRideSpringDamper


    #region PROTECTED_10_NormalAuthority
    // [MOTION-PROTECTED] ResolveNormalAuthority
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void ResolveNormalAuthority(
        float signedHeightFromVirtualLower,
        float logicalJerkLimit,
        bool supportFrameContinuous,
        out float logicalAuthority01,
        out float activeJerkLimit,
        out NormalAuthorityZone zone)
    {
        // Hybrid damping remains inside LogicalEnvelope authority.
        // This method only protects against projection discontinuity and
        // unexpected real StairWay contact.

        if (!supportFrameContinuous)
        {
            logicalAuthority01 = 0f;
            activeJerkLimit = Mathf.Max(1f, authorityHandoffMaxJerk);
            zone = NormalAuthorityZone.HandoffToPhysics;
            return;
        }

        if (physicalLowerContactActive)
        {
            logicalAuthority01 = 0f;
            activeJerkLimit = Mathf.Max(1f, authorityHandoffMaxJerk);
            zone = NormalAuthorityZone.PhysicalStairContact;
            return;
        }

        logicalAuthority01 = 1f;
        activeJerkLimit = Mathf.Max(1f, logicalJerkLimit);
        zone = NormalAuthorityZone.LogicalEnvelope;
    }
    #endregion // PROTECTED_10_NormalAuthority


// ================================================================
// 3-Wave Micro Modal Oscillation
// ================================================================

    private void ResetMicroModalOscillation()
    {
        microMode1Q = 0f;
        microMode1QDot = 0f;
        microMode2Q = 0f;
        microMode2QDot = 0f;
        microMode3Q = 0f;
        microMode3QDot = 0f;
        microTargetNormalVelocityReference = 0f;
        microImpactSerial = 0;

        // Synchronize edge latches to the CURRENT state so a reset never
        // fabricates a new entry/impact on the following FixedUpdate.
        microPreviousNaturalConnectBlendActive =
            naturalConnectBlendActive;
        microPreviousUpperSeenSinceLastLower =
            physicalUpperSeenSinceLastLower;
        microPreviousLowerContactActive =
            physicalLowerContactActive;
    }


    private void UpdateMicroModalImpactExcitation()
    {
        bool compatibleMode =
            useMicroModalOscillation &&
            equalizerControlMode == EqualizerControlMode.LegacyHybrid &&
            waveTimingMode == WaveTimingMode.ThreeWavesPerStair &&
            !postTurnHermiteBridgeActive;

        if (!compatibleMode)
        {
            ResetMicroModalOscillation();
            return;
        }

        bool entryActive =
            naturalConnectBlendActive;

        bool upperActive =
            physicalUpperSeenSinceLastLower;

        bool lowerActive =
            physicalLowerContactActive;

        // First-wave pre-Upper micro motion:
        // use the already measured Natural Connect relative-N speed.
        if (entryActive &&
            !microPreviousNaturalConnectBlendActive)
        {
            ExciteMicroModalOscillation(
                Mathf.Abs(
                    naturalConnectMeasuredRelativeNormalSpeed),
                microModalEntryKick,
                1f);
        }

        // Physical Upper event.
        // The modal state receives the impact energy, but Apex Guard will make
        // its instantaneous output zero at phi=0.5.
        if (upperActive &&
            !microPreviousUpperSeenSinceLastLower)
        {
            float incoming =
                Mathf.Max(
                    Mathf.Abs(lastUpperIncomingNormalSpeed),
                    ReadMicroRelativeNormalSpeed());

            ExciteMicroModalOscillation(
                incoming,
                microModalUpperKick,
                1f);
        }

        // Physical Lower event. Flip the principal polarity so the next slope
        // does not become a repeated copy of the Upper-excited response.
        if (lowerActive &&
            !microPreviousLowerContactActive)
        {
            float incoming =
                Mathf.Max(
                    Mathf.Abs(lastPhysicalLowerIncomingNormalSpeed),
                    ReadMicroRelativeNormalSpeed());

            ExciteMicroModalOscillation(
                incoming,
                microModalLowerKick,
                -1f);
        }

        microPreviousNaturalConnectBlendActive =
            entryActive;
        microPreviousUpperSeenSinceLastLower =
            upperActive;
        microPreviousLowerContactActive =
            lowerActive;
    }


    private float ReadMicroRelativeNormalSpeed()
    {
        if (!ballVisualEqualizer ||
            !oscillationFrame.valid ||
            oscillationFrame.normal.sqrMagnitude <= Epsilon)
        {
            return 0f;
        }

        Vector3 normal =
            oscillationFrame.normal.normalized;

        float equalizerVN =
            Vector3.Dot(
                ballVisualEqualizer.velocity,
                normal);

        float supportVN =
            Vector3.Dot(
                ReadSubjectVelocityVisual(),
                normal);

        return
            Mathf.Abs(
                equalizerVN -
                supportVN);
    }


    private void ExciteMicroModalOscillation(
        float impactNormalSpeed,
        float kickGain,
        float eventPolarity)
    {
        if (!useMicroModalOscillation ||
            impactNormalSpeed <= Epsilon ||
            kickGain <= 0f)
        {
            return;
        }

        float normalizedImpact =
            Mathf.Clamp(
                impactNormalSpeed /
                Mathf.Max(
                    0.1f,
                    microModalImpactReferenceSpeed),
                0f,
                1.75f);

        float kick =
            normalizedImpact *
            Mathf.Max(
                0f,
                kickGain);

        float dt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f);

        float omega1 =
            ResolveMicroModeAngularSpeed(
                microModalFrequencyRatio1,
                dt);

        float omega2 =
            ResolveMicroModeAngularSpeed(
                microModalFrequencyRatio2,
                dt);

        float omega3 =
            ResolveMicroModeAngularSpeed(
                microModalFrequencyRatio3,
                dt);

        if (omega1 <= Epsilon &&
            omega2 <= Epsilon &&
            omega3 <= Epsilon)
        {
            return;
        }

        // Deterministic, non-integer modal mixture.
        // Alternating the second mode prevents Upper/Lower responses from
        // collapsing into the same repeated waveform while remaining replayable.
        float alternating =
            (microImpactSerial & 1) == 0
                ? 1f
                : -1f;

        float polarity =
            eventPolarity >= 0f
                ? 1f
                : -1f;

        microMode1QDot +=
            polarity *
            kick *
            omega1 *
            0.55f;

        microMode2QDot +=
            -polarity *
            alternating *
            kick *
            omega2 *
            0.32f;

        microMode3QDot +=
            polarity *
            kick *
            omega3 *
            0.18f;

        microImpactSerial++;
    }


    private float ResolveMicroModalAcceleration(
        float spanMeters,
        float actualHeightMeters,
        float relativeNormalVelocity,
        float accelerationLimit)
    {
        if (!useMicroModalOscillation ||
            accelerationLimit <= Epsilon)
        {
            return 0f;
        }

        float dt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f);

        // ------------------------------------------------------------
        // Main-wave phase gate.
        //
        // phi=0   Lower  -> 0
        // phi=0.5 Upper  -> 0
        // phi=1   Lower  -> 0
        //
        // This gate is used BOTH for Micro output and for residual drive.
        // Therefore targetNormalVelocity coupling cannot push the apex itself.
        // ------------------------------------------------------------
        float phase01 =
            Mathf.Repeat(
                spatialWavePhase01,
                1f);

        float theta =
            2f *
            Mathf.PI *
            phase01;

        float apexWave =
            Mathf.Sin(
                theta);

        float apexGuard01 =
            apexWave *
            apexWave;

        // ------------------------------------------------------------
        // Residual coupling to the ORIGINAL 3-wave velocity target.
        //
        // eV = vTarget - vActual
        //
        // The main controller already uses exactly this quantity through
        // C(vTarget-vN). Here it is only a weak secondary excitation source.
        // Smooth rational saturation avoids a hard threshold:
        //
        // eNorm = eV / sqrt(eV^2 + vRef^2)
        //
        // -> small residual : almost zero Micro drive
        // -> large residual : smoothly approaches +/-1
        // ------------------------------------------------------------
        float velocityResidual =
            microTargetNormalVelocityReference -
            relativeNormalVelocity;

        float residualReferenceSpeed =
            Mathf.Max(
                0.05f,
                microModalVelocityResidualReferenceSpeed);

        float residualNormalized =
            velocityResidual /
            Mathf.Sqrt(
                velocityResidual *
                velocityResidual +
                residualReferenceSpeed *
                residualReferenceSpeed);

        float residualDrive =
            residualNormalized *
            Mathf.Clamp(
                microModalVelocityResidualDriveGain,
                0f,
                0.75f) *
            apexGuard01;

        float omega1 =
            ResolveMicroModeAngularSpeed(
                microModalFrequencyRatio1,
                dt);

        float omega2 =
            ResolveMicroModeAngularSpeed(
                microModalFrequencyRatio2,
                dt);

        float omega3 =
            ResolveMicroModeAngularSpeed(
                microModalFrequencyRatio3,
                dt);

        float damping =
            Mathf.Clamp(
                microModalDampingRatio,
                0.02f,
                0.45f);

        float duffingBeta =
            Mathf.Clamp(
                microModalDuffingBeta,
                0f,
                0.30f);

        // Non-integer modes receive different residual-drive polarity/weight.
        // Because targetNormalVelocity reverses naturally across the wave,
        // this produces a smooth but less repetitive "tricky" micro response
        // without adding random noise or a second phase clock.
        StepMicroModalMode(
            ref microMode1Q,
            ref microMode1QDot,
            omega1,
            damping,
            duffingBeta,
            residualDrive * 0.42f,
            dt);

        StepMicroModalMode(
            ref microMode2Q,
            ref microMode2QDot,
            omega2,
            damping,
            duffingBeta,
            -residualDrive * 0.27f,
            dt);

        StepMicroModalMode(
            ref microMode3Q,
            ref microMode3QDot,
            omega3,
            damping,
            duffingBeta,
            residualDrive * 0.16f,
            dt);

        // Soft grazing emphasis.
        // It does NOT inject a separate force; it only lets the already
        // existing modal response persist slightly more near a slow boundary.
        float height01 =
            spanMeters > Epsilon
                ? Mathf.Clamp01(
                    actualHeightMeters /
                    spanMeters)
                : 0.5f;

        float nearestBoundaryDistance01 =
            Mathf.Min(
                height01,
                1f - height01);

        float distanceScale =
            Mathf.Max(
                0.01f,
                microModalGrazingDistance01);

        float distanceRatio =
            nearestBoundaryDistance01 /
            distanceScale;

        float distanceWeight =
            Mathf.Exp(
                -distanceRatio *
                distanceRatio);

        float velocityScale =
            Mathf.Max(
                0.05f,
                microModalGrazingVelocityScale);

        float velocityRatio =
            Mathf.Abs(
                relativeNormalVelocity) /
            velocityScale;

        float velocityWeight =
            Mathf.Exp(
                -velocityRatio *
                velocityRatio);

        float grazingWeight01 =
            distanceWeight *
            velocityWeight;

        float grazingBoost =
            1f +
            grazingWeight01 *
            Mathf.Clamp(
                microModalGrazingBoost01,
                0f,
                0.5f);

        float modalSignal =
            microMode1Q * 0.55f +
            microMode2Q * 0.30f +
            microMode3Q * 0.15f;

        modalSignal =
            Mathf.Clamp(
                modalSignal,
                -1.25f,
                1.25f);

        float microBudget =
            accelerationLimit *
            Mathf.Clamp(
                microModalAccelerationShare01,
                0f,
                0.25f);

        float microAcceleration =
            modalSignal *
            apexGuard01 *
            grazingBoost *
            microBudget;

        return
            Mathf.Clamp(
                microAcceleration,
                -microBudget,
                microBudget);
    }


    private float ResolveMicroModeAngularSpeed(
        float frequencyRatio,
        float dt)
    {
        float mainAngularSpeed =
            Mathf.Abs(
                spatialGuidedAngularSpeed);

        if (mainAngularSpeed <= Epsilon)
        {
            mainAngularSpeed =
                Mathf.Abs(
                    spatialPathAngularSpeed);
        }

        if (mainAngularSpeed <= Epsilon &&
            spatialReferencePeriodSeconds > Epsilon)
        {
            mainAngularSpeed =
                2f *
                Mathf.PI /
                spatialReferencePeriodSeconds;
        }

        if (mainAngularSpeed <= Epsilon &&
            observedNaturalPeriodSeconds > Epsilon)
        {
            mainAngularSpeed =
                2f *
                Mathf.PI /
                observedNaturalPeriodSeconds;
        }

        // Plane->Stair first half-wave fallback, before the spatial solver has
        // published an angular speed for the new section.
        if (mainAngularSpeed <= Epsilon &&
            preferredHalfPeriodSeconds > Epsilon)
        {
            mainAngularSpeed =
                Mathf.PI /
                preferredHalfPeriodSeconds;
        }

        if (mainAngularSpeed <= Epsilon)
            return 0f;

        float rawFrequencyHz =
            mainAngularSpeed /
            (2f * Mathf.PI) *
            Mathf.Max(
                1f,
                frequencyRatio);

        // FixedUpdate-safe soft compression:
        // instead of hard-clamping several high modes to the same frequency,
        // f = fMax * (1-exp(-fRaw/fMax)) keeps their ordering while asymptoting.
        float fixedStepSafeHz =
            0.18f /
            Mathf.Max(
                dt,
                0.000001f);

        float maximumFrequencyHz =
            Mathf.Max(
                1f,
                Mathf.Min(
                    Mathf.Max(
                        1f,
                        microModalMaximumFrequencyHz),
                    fixedStepSafeHz));

        float compressedFrequencyHz =
            maximumFrequencyHz *
            (1f -
             Mathf.Exp(
                 -rawFrequencyHz /
                 maximumFrequencyHz));

        return
            2f *
            Mathf.PI *
            compressedFrequencyHz;
    }


    private static void StepMicroModalMode(
        ref float q,
        ref float qDot,
        float omega,
        float dampingRatio,
        float duffingBeta,
        float externalDrive,
        float dt)
    {
        if (omega <= Epsilon ||
            dt <= Epsilon)
        {
            q = 0f;
            qDot = 0f;
            return;
        }

        float q2 =
            q *
            q;

        float nonlinearPosition =
            q +
            duffingBeta *
            q *
            q2;

        float qAcceleration =
            -2f *
            dampingRatio *
            omega *
            qDot -
            omega *
            omega *
            nonlinearPosition +
            omega *
            omega *
            externalDrive;

        // Semi-implicit Euler is intentionally used here:
        // velocity first, then displacement. With the frequency compression
        // above it remains well behaved at ordinary Unity FixedUpdate rates.
        qDot +=
            qAcceleration *
            dt;

        q +=
            qDot *
            dt;

        if (!IsFinite(q) ||
            !IsFinite(qDot))
        {
            q = 0f;
            qDot = 0f;
            return;
        }

        q =
            Mathf.Clamp(
                q,
                -1.5f,
                1.5f);

        float maximumModalVelocity =
            Mathf.Max(
                1f,
                omega *
                2.0f);

        qDot =
            Mathf.Clamp(
                qDot,
                -maximumModalVelocity,
                maximumModalVelocity);

        // Kill denormally-small tails so an old section can never leave
        // invisible residual state for many seconds.
        if (Mathf.Abs(q) < 0.00001f &&
            Mathf.Abs(qDot) < 0.0001f)
        {
            q = 0f;
            qDot = 0f;
        }
    }


    private float ResolveEqualizerWorldRadius()
    {
        if (!ballVisualEqualizerCollider)
            return 0.5f;

        Vector3 scale = ballVisualEqualizerCollider.transform.lossyScale;
        float maximumScale = Mathf.Max(
            Mathf.Abs(scale.x),
            Mathf.Max(Mathf.Abs(scale.y), Mathf.Abs(scale.z)));

        return Mathf.Max(
            0.0001f,
            ballVisualEqualizerCollider.radius * maximumScale);
    }


    private void ResetRideSupportKinematics()
    {
        rideSupportKinematicsValid = false;
        previousRideSupportCenter = Vector3.zero;
        previousRideSupportVelocity = Vector3.zero;
        previousRideSupportSampleTime = Time.fixedTime;
        rideSupportNormalVelocity = 0f;
        rideSupportNormalAcceleration = 0f;
        rideSupportFrameContinuous = true;
    }


    private bool SampleRideSupportKinematics(
        Vector3 supportCenter,
        Vector3 fallbackVelocity,
        out Vector3 supportVelocity,
        out Vector3 supportAcceleration)
    {
        // IMPORTANT:
        // Virtual Lower is a projection point in the rotating Visual frame.
        // Its world-position finite difference must NOT be treated directly as
        // physical support velocity. The physical reference velocity is the
        // mapped Subject velocity; supportCenter is used only for continuity
        // validation of the projection/frame.
        supportVelocity = fallbackVelocity;
        supportAcceleration = Vector3.zero;

        float sampleTime = Time.fixedTime;

        if (!rideSupportKinematicsValid)
        {
            rideSupportKinematicsValid = true;
            previousRideSupportCenter = supportCenter;
            previousRideSupportVelocity = fallbackVelocity;
            previousRideSupportSampleTime = sampleTime;
            return true;
        }

        float sampleDt = sampleTime - previousRideSupportSampleTime;
        if (sampleDt <= Epsilon)
            return true;

        Vector3 centerDelta = supportCenter - previousRideSupportCenter;
        float centerJumpMeters = centerDelta.magnitude;
        float apparentCenterSpeed = centerJumpMeters / sampleDt;

        float radius = ResolveEqualizerWorldRadius();
        float maximumCenterJump =
            Mathf.Max(
                0.05f,
                radius * Mathf.Max(0.25f, supportKinematicsMaximumCenterJumpR));

        float maximumMeasuredSpeed =
            Mathf.Max(1f, supportKinematicsMaximumMeasuredSpeed);

        bool projectionOrFrameJump =
            !IsFinite(supportCenter) ||
            !IsFinite(fallbackVelocity) ||
            !IsFinite(apparentCenterSpeed) ||
            centerJumpMeters > maximumCenterJump ||
            apparentCenterSpeed > maximumMeasuredSpeed;

        if (projectionOrFrameJump)
        {
            previousRideSupportCenter = supportCenter;
            previousRideSupportVelocity = fallbackVelocity;
            previousRideSupportSampleTime = sampleTime;


            return false;
        }

        Vector3 measuredAcceleration =
            (fallbackVelocity - previousRideSupportVelocity) / sampleDt;

        if (IsFinite(measuredAcceleration) &&
            measuredAcceleration.magnitude <=
            Mathf.Max(1f, supportKinematicsMaximumMeasuredAcceleration))
        {
            supportAcceleration = measuredAcceleration;
        }

        previousRideSupportCenter = supportCenter;
        previousRideSupportVelocity = fallbackVelocity;
        previousRideSupportSampleTime = sampleTime;
        return true;
    }


    #region PROTECTED_11_SpatialCurvatureAndWaveReference
    // [MOTION-PROTECTED] TryResolveSpatialCurvatureAdaptiveReference
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private bool TryResolveSpatialCurvatureAdaptiveReference(
        Vector3 lowerCenter,
        float envelopeSpanMeters,
        Vector3 tangent,
        Vector3 normal,
        out Vector3 targetCenter,
        out float targetNormalVelocity,
        out float targetNormalAcceleration)
    {
        targetCenter = lowerCenter;
        targetNormalVelocity = 0f;
        targetNormalAcceleration = 0f;

        if (!negativeEnvelope ||
            envelopeSpanMeters <= Epsilon)
        {
            return false;
        }

        if (!negativeEnvelope.TryGetActiveSplineWaveDomain(
                out float releaseProgress01,
                out float targetProgress01,
                out float equalizerProgress01,
                out float activeArcLengthMeters))
        {
            return false;
        }

        float progressRange =
            targetProgress01 -
            releaseProgress01;

        if (progressRange <= Epsilon ||
            activeArcLengthMeters <= Epsilon)
        {
            return false;
        }

        // ------------------------------------------------------------
        // Monotonic spatial phase guard
        // ------------------------------------------------------------
        // Equalizer本体はPhysical Upper反射でSpline上を後退してよい。
        // Presentation位相だけを巻き戻さない。旧Trigger Pulse実験は、
        // 実Upperの物理Authorityを一時変更するためAdaptive仕様から除外する。
        float previousMonotonicProgress01 =
            spatialDomainProgress01;

        spatialRawDomainProgress01 =
            Mathf.Clamp01(
                (equalizerProgress01 -
                 releaseProgress01) /
                progressRange);

        spatialDomainProgress01 =
            Mathf.Max(
                previousMonotonicProgress01,
                spatialRawDomainProgress01);

        float spatialDt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f);

        spatialDomainAdvanceRate01PerSecond =
            Mathf.Max(
                0f,
                (spatialDomainProgress01 -
                 previousMonotonicProgress01) /
                spatialDt);

        float rawDomainAccelerationRate =
            (spatialDomainAdvanceRate01PerSecond -
             previousSpatialDomainAdvanceRate01PerSecond) /
            spatialDt;

        spatialDomainAccelerationRate01PerSecond2 =
            Mathf.Clamp(
                rawDomainAccelerationRate,
                -Mathf.Max(1f, maximumSpatialDomainAccelerationRate),
                Mathf.Max(1f, maximumSpatialDomainAccelerationRate));

        previousSpatialDomainAdvanceRate01PerSecond =
            spatialDomainAdvanceRate01PerSecond;

        spatialMonotonicHoldActive =
            spatialRawDomainProgress01 <
            spatialDomainProgress01 - 0.000001f;

        // No collider state change is performed here. This guard is purely
        // logical and therefore cannot alter real Upper restitution/contact.

        int waveCount =
            Mathf.Max(
                1,
                spatialWavesPerStair);

        float totalWavePhase =
            spatialDomainProgress01 *
            waveCount;

        int completedWaveCount =
            Mathf.Clamp(
                Mathf.FloorToInt(
                    totalWavePhase),
                0,
                waveCount - 1);

        spatialWavePhase01 =
            totalWavePhase -
            Mathf.Floor(
                totalWavePhase);

        if (spatialDomainProgress01 >= 0.999999f)
            spatialWavePhase01 = 0f;

        waveCycleIndex =
            completedWaveCount;

        negativeEnvelope.SetSpatialPresentationProgress(
            spatialDomainProgress01);

        float equalizerForwardSpeed =
            Vector3.Dot(
                Vector3.ProjectOnPlane(
                    ballVisualEqualizer.velocity,
                    normal),
                tangent);

        Vector3 subjectVelocityVisual =
            ReadSubjectVelocityVisual();

        float subjectForwardSpeed =
            Vector3.Dot(
                Vector3.ProjectOnPlane(
                    subjectVelocityVisual,
                    normal),
                tangent);

        float measuredForwardSpeed =
            Mathf.Max(
                0.25f,
                Mathf.Max(
                    Mathf.Abs(equalizerForwardSpeed),
                    Mathf.Abs(subjectForwardSpeed)));

        // ------------------------------------------------------------
        // Selective Spatial Curvature Adaptation
        // ------------------------------------------------------------
        // Existing Envelope remains the requested-height authority.  This layer
        // may only reduce that request when the geometric curvature would need
        // more normal acceleration than the existing fixed budget can provide.
        // It never moves Upper/Lower, never changes Energy/Decay, and never
        // increases acceleration/jerk budget as speed rises.
        float lookAheadSeconds =
            Mathf.Max(
                Time.fixedDeltaTime,
                spatialCurvatureLookAheadSeconds);

        float maxGroundSpeed =
            slopeCore
                ? Mathf.Max(0f, slopeCore.MaxGroundSpeedReadOnly)
                : measuredForwardSpeed;

        float groundAcceleration =
            slopeCore
                ? Mathf.Max(0f, slopeCore.GroundAcceleration)
                : 0f;

        float projectedTowardMaxGround =
            Mathf.MoveTowards(
                measuredForwardSpeed,
                Mathf.Max(measuredForwardSpeed, maxGroundSpeed),
                groundAcceleration * lookAheadSeconds);

        float designSpeed =
            Mathf.Max(
                measuredForwardSpeed,
                Mathf.Lerp(
                    measuredForwardSpeed,
                    projectedTowardMaxGround,
                    Mathf.Clamp01(spatialCurvatureMaxGroundSpeedBlend01)));

        float spatialWaveNumber =
            2f * Mathf.PI * waveCount /
            Mathf.Max(0.05f, activeArcLengthMeters);

        float requestedCarrierHeight =
            Mathf.Min(
                envelopeSpanMeters * 0.95f,
                envelopeSpanMeters *
                Mathf.Clamp(
                    spatialCarrierHeightFractionOfEnvelope,
                    0.05f,
                    0.8f));

        float kappaMax =
            Mathf.Max(1f, spatialWaveAccelerationBudget) *
            Mathf.Clamp01(spatialCurvatureSafety01) /
            Mathf.Max(0.01f, designSpeed * designSpeed);

        // y(s)=A/2*(1-cos(k*s)) -> |y''|max=A*k^2/2.
        // Therefore A <= 2*kappaMax/k^2.  This is a one-way limiter:
        // adaptation can shrink A, but can never create extra height.
        float curvatureAllowedHeight =
            spatialWaveNumber > Epsilon
                ? 2f * kappaMax /
                  Mathf.Max(
                      Epsilon,
                      spatialWaveNumber * spatialWaveNumber)
                : requestedCarrierHeight;

        float resolvedCarrierHeight =
            useSpatialCurvatureAdaptation
                ? Mathf.Min(
                    requestedCarrierHeight,
                    Mathf.Max(0f, curvatureAllowedHeight))
                : requestedCarrierHeight;

        spatialCurvatureMeasuredForwardSpeed = measuredForwardSpeed;
        spatialCurvatureDesignSpeed = designSpeed;
        spatialCurvatureLookAheadDistance = designSpeed * lookAheadSeconds;
        spatialCurvatureWaveNumberPerMeter = spatialWaveNumber;
        spatialCurvatureKappaMaxPerMeter = kappaMax;
        spatialCurvatureRequestedHeightMeters = requestedCarrierHeight;
        spatialCurvatureAllowedHeightMeters = curvatureAllowedHeight;
        spatialCurvatureClampRatio01 =
            requestedCarrierHeight > Epsilon
                ? Mathf.Clamp01(
                    resolvedCarrierHeight / requestedCarrierHeight)
                : 1f;
        spatialCurvatureLimited =
            useSpatialCurvatureAdaptation &&
            resolvedCarrierHeight + 0.0001f < requestedCarrierHeight;

        spatialSpeedRatio =
            designSpeed /
            Mathf.Max(1f, spatialReferenceTangentSpeed);

        // Old 24m/s normalization multiplied budgets by v^2/v^3.  That mostly
        // cancelled the very curvature reduction we want at high speed.
        spatialAccelerationBudgetScale = 1f;
        spatialJerkBudgetScale = 1f;

        // Temporal diagnostics/feed-forward still reflect ACTUAL progress speed;
        // maxGroundSpeed anticipation is used only by the feasibility limiter.
        float spatialFrequency =
            waveCount *
            measuredForwardSpeed /
            activeArcLengthMeters;

        spatialReferencePeriodSeconds =
            1f /
            Mathf.Max(0.0001f, spatialFrequency);

        spatialCarrierHeightBeforeFeasibility = requestedCarrierHeight;
        spatialCarrierHeightMeters = resolvedCarrierHeight;
        spatialCarrierFeasibility01 = spatialCurvatureClampRatio01;

        float theta =
            2f *
            Mathf.PI *
            spatialWavePhase01;

        float wave01 =
            0.5f *
            (1f -
             Mathf.Cos(
                 theta));

        rideTargetHeight =
            spatialCarrierHeightMeters *
            wave01;

        targetCenter =
            lowerCenter +
            normal *
            rideTargetHeight;

        // Spatial position authority:
        //   H(theta)=A/2*(1-cos(theta)), theta=2*pi*N*p.
        // p (therefore N waves per stair) remains the only position phase.
        //
        // Soft T/2 guidance affects only feed-forward angular speed. It can
        // improve temporal feel without ever changing the N-wave position phase.
        float pDot =
            spatialDomainAdvanceRate01PerSecond;

        float pDDot =
            includeSpatialProgressAccelerationFeedForward
                ? spatialDomainAccelerationRate01PerSecond2
                : 0f;

        spatialPathAngularSpeed =
            2f * Mathf.PI * waveCount * pDot;

        float pathAngularAcceleration =
            2f * Mathf.PI * waveCount * pDDot;

        spatialPathHalfPeriodSeconds =
            spatialPathAngularSpeed > Epsilon
                ? Mathf.PI / spatialPathAngularSpeed
                : 0f;

        float preferredAngularSpeed =
            Mathf.PI /
            Mathf.Max(0.01f, preferredHalfPeriodSeconds);

        // Never run the preferred clock while path progress is stopped. This
        // avoids velocity feed-forward fighting a frozen spatial target.
        spatialHalfPeriodFeedForwardBlendApplied01 =
            usePreferredHalfPeriodGuidance && pDot > 0.0001f
                ? Mathf.Clamp01(preferredHalfPeriodFeedForwardBlend01)
                : 0f;

        spatialGuidedAngularSpeed =
            Mathf.Lerp(
                spatialPathAngularSpeed,
                preferredAngularSpeed,
                spatialHalfPeriodFeedForwardBlendApplied01);

        float guidedAngularAcceleration =
            pathAngularAcceleration *
            (1f - spatialHalfPeriodFeedForwardBlendApplied01);

        spatialGuidedHalfPeriodSeconds =
            spatialGuidedAngularSpeed > Epsilon
                ? Mathf.PI / spatialGuidedAngularSpeed
                : 0f;

        float halfAmplitude =
            0.5f * spatialCarrierHeightMeters;

        targetNormalVelocity =
            halfAmplitude *
            Mathf.Sin(theta) *
            spatialGuidedAngularSpeed;

        targetNormalAcceleration =
            halfAmplitude *
            (Mathf.Cos(theta) *
             spatialGuidedAngularSpeed *
             spatialGuidedAngularSpeed +
             Mathf.Sin(theta) *
             guidedAngularAcceleration);

        spatialFeedForwardVelocity =
            targetNormalVelocity;

        spatialFeedForwardAcceleration =
            targetNormalAcceleration;

        return true;
    }
    #endregion // PROTECTED_11_SpatialCurvatureAndWaveReference

// ================================================================
// Goal velocity catch-up
// ================================================================

    #region PROTECTED_12_TransportCatchUp
    // [MOTION-PROTECTED] ApplyGoalVelocityCatchUp
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void ApplyGoalVelocityCatchUp()
    {
        if (!oscillationFrame.valid ||
            !ballVisualEqualizer)
        {
            return;
        }

        Vector3 normal =
            oscillationFrame.normal.normalized;

        Vector3 tangent =
            Vector3.ProjectOnPlane(
                oscillationFrame.tangent,
                normal);

        if (tangent.sqrMagnitude <= Epsilon)
            return;

        tangent.Normalize();

        Vector3 lateral =
            oscillationFrame.lateral;

        if (lateral.sqrMagnitude <= Epsilon)
        {
            lateral =
                Vector3.Cross(
                    normal,
                    tangent);
        }

        if (lateral.sqrMagnitude > Epsilon)
            lateral.Normalize();

        float dt =
            Mathf.Max(
                Time.fixedDeltaTime,
                0.000001f);

        Vector3 subjectPosition =
            ReadSubjectPositionVisual();

        Vector3 subjectVelocity =
            ReadSubjectVelocityVisual();

        Vector3 planeGap =
            Vector3.ProjectOnPlane(
                subjectPosition -
                ballVisualEqualizer.position,
                normal);

        Vector3 subjectPlanarVelocity =
            Vector3.ProjectOnPlane(
                subjectVelocity,
                normal);

        Vector3 equalizerPlanarVelocity =
            Vector3.ProjectOnPlane(
                ballVisualEqualizer.velocity,
                normal);

        Vector3 rawGoalVelocity =
            subjectPlanarVelocity +
            planeGap /
            Mathf.Max(
                0.03f,
                catchUpPositionTime);

        bool spatialMode =
            waveTimingMode ==
            WaveTimingMode.ThreeWavesPerStair;

        terminalSpatialTransportAssist01 = 0f;
        float terminalTransportMultiplier = 1f;

        if (spatialMode &&
            enableTerminalSpatialTransportAssist)
        {
            float start =
                Mathf.Clamp(
                    terminalSpatialTransportAssistStartProgress01,
                    0.5f,
                    0.95f);

            terminalSpatialTransportAssist01 =
                Mathf.SmoothStep(
                    0f,
                    1f,
                    Mathf.InverseLerp(
                        start,
                        1f,
                        spatialDomainProgress01));

            terminalTransportMultiplier =
                Mathf.Lerp(
                    1f,
                    Mathf.Max(1f, terminalSpatialTransportAssistMultiplier),
                    terminalSpatialTransportAssist01);
        }

        // In Spatial Wave mode, position lag alone is not enough.
        // Recompute the speed required for Equalizer to reach the same Spline
        // terminal at the Subject's current estimated arrival time.
        if (spatialMode)
        {
            ApplySpatialArrivalDeadlineToGoal(
                ref rawGoalVelocity,
                subjectPosition,
                subjectPlanarVelocity,
                tangent,
                lateral);
        }
        else
        {
            spatialSubjectTimeToGo = 0f;
            spatialRequiredArrivalSpeed = 0f;
            spatialArrivalFeasibility01 = 1f;
        }

        requiredCatchUpSpeed =
            Vector3.Dot(
                rawGoalVelocity,
                tangent);

        if (negativeEnvelope.TryGetSourceMaxGroundSpeedReadOnly(
                out float maxGroundSpeed) &&
            maxGroundSpeed > Epsilon)
        {
            float headroom =
                spatialMode
                    ? Mathf.Max(
                        0f,
                        spatialCatchUpSpeedHeadroom01)
                    : Mathf.Clamp01(
                        catchUpSpeedHeadroom01);

            float maximumGoalSpeed =
                maxGroundSpeed *
                (1f + headroom);

            float tangentGoal =
                Mathf.Clamp(
                    Vector3.Dot(
                        rawGoalVelocity,
                        tangent),
                    -maximumGoalSpeed,
                    maximumGoalSpeed);

            Vector3 lateralGoal =
                lateral.sqrMagnitude > Epsilon
                    ? lateral *
                      Vector3.Dot(
                          rawGoalVelocity,
                          lateral)
                    : Vector3.zero;

            rawGoalVelocity =
                tangent *
                tangentGoal +
                lateralGoal;

            requiredCatchUpSpeed =
                tangentGoal;

            if (spatialMode &&
                spatialRequiredArrivalSpeed > Epsilon)
            {
                spatialArrivalFeasibility01 =
                    Mathf.Clamp01(
                        maximumGoalSpeed /
                        spatialRequiredArrivalSpeed);
            }
        }

        float goalAccel =
            spatialMode
                ? Mathf.Max(
                    1f,
                    spatialGoalVelocityAcceleration *
                    terminalTransportMultiplier)
                : Mathf.Max(
                    1f,
                    goalVelocityAcceleration);

        goalPlanarVelocityState =
            Vector3.MoveTowards(
                goalPlanarVelocityState,
                rawGoalVelocity,
                goalAccel *
                dt);

        Vector3 neededAcceleration =
            (goalPlanarVelocityState -
             equalizerPlanarVelocity) /
            dt;

        if (lateral.sqrMagnitude > Epsilon)
        {
            float lateralGap =
                Vector3.Dot(
                    planeGap,
                    lateral);

            float lateralRelativeVelocity =
                Vector3.Dot(
                    ballVisualEqualizer.velocity -
                    subjectVelocity,
                    lateral);

            neededAcceleration +=
                lateral *
                (lateralGap *
                 Mathf.Max(
                     0f,
                     lateralSpringStrength) -
                 lateralRelativeVelocity *
                 Mathf.Max(
                     0f,
                     lateralDamper));
        }

        neededAcceleration =
            Vector3.ProjectOnPlane(
                neededAcceleration,
                normal);

        float catchUpAccelLimit =
            spatialMode
                ? Mathf.Max(
                    1f,
                    spatialMaximumCatchUpAcceleration *
                    terminalTransportMultiplier)
                : Mathf.Max(
                    1f,
                    maximumCatchUpAcceleration);

        neededAcceleration =
            Vector3.ClampMagnitude(
                neededAcceleration,
                catchUpAccelLimit);

        transportAccelerationState =
            Vector3.MoveTowards(
                transportAccelerationState,
                neededAcceleration,
                Mathf.Max(
                    1f,
                    maximumTransportJerk *
                    terminalTransportMultiplier) *
                dt);

        transportAccelerationState =
            Vector3.ProjectOnPlane(
                transportAccelerationState,
                normal);

        float connectTransportAuthority =
            Mathf.Clamp01(naturalConnectControlAuthority01);

        ballVisualEqualizer.AddForceAtPosition(
            transportAccelerationState *
            connectTransportAuthority *
            EqualizerMass,
            ballVisualEqualizer.worldCenterOfMass,
            ForceMode.Force);

    }
    #endregion // PROTECTED_12_TransportCatchUp


    private void ApplySpatialArrivalDeadlineToGoal(
        ref Vector3 rawGoalVelocity,
        Vector3 subjectPosition,
        Vector3 subjectPlanarVelocity,
        Vector3 tangent,
        Vector3 lateral)
    {
        if (!negativeEnvelope)
            return;

        if (!negativeEnvelope.TryGetActiveSplineWaveDomain(
                out _,
                out float targetProgress01,
                out float equalizerProgress01,
                out _))
        {
            return;
        }

        if (!negativeEnvelope.TryProjectVisualPointToSplineFrameVisual(
                subjectPosition,
                out _,
                out _,
                out _,
                out _,
                out float subjectProgress01))
        {
            return;
        }

        float subjectRemaining =
            subjectProgress01 >=
            targetProgress01 - 0.000001f
                ? 0f
                : negativeEnvelope.EstimateSplineArcDistanceBetweenProgress(
                    subjectProgress01,
                    targetProgress01);

        float equalizerRemaining =
            equalizerProgress01 >=
            targetProgress01 - 0.000001f
                ? 0f
                : negativeEnvelope.EstimateSplineArcDistanceBetweenProgress(
                    equalizerProgress01,
                    targetProgress01);

        subjectRemaining =
            Mathf.Max(
                0f,
                subjectRemaining);

        equalizerRemaining =
            Mathf.Max(
                0f,
                equalizerRemaining);

        float forwardSubjectSpeed =
            Mathf.Abs(
                Vector3.Dot(
                    subjectPlanarVelocity,
                    tangent));

        spatialSubjectTimeToGo =
            subjectRemaining /
            Mathf.Max(
                0.5f,
                forwardSubjectSpeed);

        spatialSubjectTimeToGo =
            Mathf.Max(
                Time.fixedDeltaTime * 2f,
                spatialSubjectTimeToGo);

        spatialRequiredArrivalSpeed =
            equalizerRemaining /
            spatialSubjectTimeToGo;

        float directionSign =
            Vector3.Dot(
                subjectPlanarVelocity,
                tangent) >= 0f
                ? 1f
                : -1f;

        float signedRequiredSpeed =
            spatialRequiredArrivalSpeed *
            directionSign;

        float currentGoalTangent =
            Vector3.Dot(
                rawGoalVelocity,
                tangent);

        float deadlineGoalTangent =
            directionSign >= 0f
                ? Mathf.Max(
                    currentGoalTangent,
                    signedRequiredSpeed)
                : Mathf.Min(
                    currentGoalTangent,
                    signedRequiredSpeed);

        Vector3 lateralGoal =
            lateral.sqrMagnitude > Epsilon
                ? lateral *
                  Vector3.Dot(
                      rawGoalVelocity,
                      lateral)
                : Vector3.zero;

        rawGoalVelocity =
            tangent *
            deadlineGoalTangent +
            lateralGoal;
    }

    #region PROTECTED_12B_PostUpperDirectionalMemory
    // [MOTION-PROTECTED] Upper後の階段方向メモリ。
    //
    // Normal:
    //   実衝突の反射はPhysX / Upper Reflection側が所有。
    //
    // Tangent/Lateral:
    //   Upper接触時にWorld-spaceで確定したframeだけを使う。
    //   Transform/Visualの回転から再計算しない。
    //
    // Tangent速度:
    //   dvT/dt = (vTarget - vT) / tau
    //
    // Weight:
    //   w(t) = exp(-ln(2) * t / halfLife)
    //
    // 厳密なphase一致や位置閾値を要求せず、Accepted Upperを起点に
    // 徐々に効き、Physical Lowerまたは明示的Rejoinで終了する。
    private void ResetPostUpperDirectionalMemory()
    {
        postUpperDirectionalMemoryActive = false;
        postUpperDirectionalMemoryStartFixedTime = -1f;
        postUpperDirectionalTargetSpeed = 0f;
        postUpperDirectionalWeight01 = 0f;
        postUpperDirectionalAppliedAcceleration = 0f;
        postUpperLateralAppliedAcceleration = 0f;
        postUpperHybridAccelerationDemand = 0f;
        postUpperHybridAccelerationBudget = 0f;
        postUpperHybridAccelerationRatio = 0f;
        postUpperHybridSmoothAssist01 = 0f;
        postUpperCapturedTangent = Vector3.forward;
        postUpperCapturedNormal = Vector3.up;
        postUpperCapturedLateral = Vector3.right;
    }


    private void ArmPostUpperDirectionalMemory(Collision collision)
    {
        if (!usePostUpperDirectionalMemory ||
            !ballVisualEqualizer)
        {
            ResetPostUpperDirectionalMemory();
            return;
        }

        Vector3 normal = Vector3.up;
        Vector3 tangent = Vector3.zero;

        if (oscillationFrame.valid)
        {
            normal = oscillationFrame.normal;
            tangent = oscillationFrame.tangent;
        }
        else if (collision != null &&
                 collision.contactCount > 0)
        {
            normal = collision.GetContact(0).normal;
            tangent = naturalConnectAveragedTangent;
        }

        if (normal.sqrMagnitude <= Epsilon)
            normal = Vector3.up;

        normal.Normalize();

        tangent =
            Vector3.ProjectOnPlane(
                tangent,
                normal);

        // Formal frameが瞬間的に取れなくても、現在の運動から緩くfallbackする。
        if (tangent.sqrMagnitude <= Epsilon)
        {
            tangent =
                Vector3.ProjectOnPlane(
                    ReadSubjectVelocityVisual(),
                    normal);
        }

        if (tangent.sqrMagnitude <= Epsilon)
        {
            tangent =
                Vector3.ProjectOnPlane(
                    ballVisualEqualizer.velocity,
                    normal);
        }

        if (tangent.sqrMagnitude <= Epsilon)
            return;

        tangent.Normalize();

        // 重力成分がある斜面では「重力と同じ向き」をdown-stairとする。
        // ほぼ平面なら現在の進行方向と符号を合わせる。
        float gravityDot =
            Vector3.Dot(
                tangent,
                Physics.gravity);

        if (gravityDot < -Epsilon)
        {
            tangent = -tangent;
        }
        else if (Mathf.Abs(gravityDot) <= Epsilon)
        {
            Vector3 observedPlanarVelocity =
                Vector3.ProjectOnPlane(
                    ReadSubjectVelocityVisual(),
                    normal);

            if (observedPlanarVelocity.sqrMagnitude <= Epsilon)
            {
                observedPlanarVelocity =
                    Vector3.ProjectOnPlane(
                        ballVisualEqualizer.velocity,
                        normal);
            }

            if (observedPlanarVelocity.sqrMagnitude > Epsilon &&
                Vector3.Dot(
                    tangent,
                    observedPlanarVelocity) < 0f)
            {
                tangent = -tangent;
            }
        }

        Vector3 lateral =
            Vector3.Cross(
                normal,
                tangent);

        if (lateral.sqrMagnitude <= Epsilon)
            return;

        lateral.Normalize();

        postUpperCapturedNormal = normal;
        postUpperCapturedTangent = tangent;
        postUpperCapturedLateral = lateral;

        float currentTangentSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    ballVisualEqualizer.velocity,
                    tangent));

        float subjectTangentSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    ReadSubjectVelocityVisual(),
                    tangent));

        float sourceReferenceSpeed = 0f;

        if (negativeEnvelope)
        {
            negativeEnvelope.TryGetSourceMaxGroundSpeedReadOnly(
                out sourceReferenceSpeed);
        }

        float referenceFloor =
            Mathf.Max(
                0f,
                sourceReferenceSpeed) *
            Mathf.Clamp01(
                postUpperDirectionalReferenceSpeedFraction);

        // 速度を落とす制御にはしない。
        // 現在/Subject/maxGround由来のうち最も大きい下降Tangentを保持対象にする。
        postUpperDirectionalTargetSpeed =
            Mathf.Max(
                currentTangentSpeed,
                subjectTangentSpeed,
                referenceFloor);

        postUpperDirectionalMemoryStartFixedTime =
            Time.fixedTime;

        postUpperDirectionalWeight01 = 1f;
        postUpperDirectionalAppliedAcceleration = 0f;
        postUpperLateralAppliedAcceleration = 0f;
        postUpperHybridAccelerationDemand = 0f;
        postUpperHybridAccelerationBudget = 0f;
        postUpperHybridAccelerationRatio = 0f;
        postUpperHybridSmoothAssist01 = 0f;
        postUpperDirectionalMemoryActive = true;
    }


    private void ApplyPostUpperDirectionalMemory()
    {
        if (!postUpperDirectionalMemoryActive ||
            !usePostUpperDirectionalMemory ||
            !ballVisualEqualizer)
        {
            return;
        }

        Vector3 tangent =
            postUpperCapturedTangent;

        Vector3 lateral =
            postUpperCapturedLateral;

        if (tangent.sqrMagnitude <= Epsilon)
            return;

        tangent.Normalize();

        float elapsed =
            Mathf.Max(
                0f,
                Time.fixedTime -
                postUpperDirectionalMemoryStartFixedTime);

        float halfLife =
            Mathf.Max(
                0.02f,
                postUpperDirectionalHalfLifeSeconds);

        float weight01 =
            Mathf.Exp(
                -0.69314718056f *
                elapsed /
                halfLife);

        postUpperDirectionalWeight01 =
            Mathf.Clamp01(
                weight01);

        // ------------------------------------------------------------
        // Hybrid authority:
        //
        // Low/medium Stable-N acceleration:
        //   keep the original rough Equalizer response untouched.
        //
        // High Stable-N acceleration:
        //   progressively add the smooth Post-Upper tangent/lateral assist.
        //
        // No hard phase/angle/restitution gate is added here.  The blend is
        // driven by the already-existing protected acceleration state.
        // ------------------------------------------------------------
        Vector3 capturedNormal =
            postUpperCapturedNormal.sqrMagnitude > Epsilon
                ? postUpperCapturedNormal.normalized
                : Vector3.up;

        float commandedNormalAcceleration =
            Mathf.Abs(
                normalTargetAccelerationAfterAuthority);

        float realizedNormalAcceleration =
            Mathf.Abs(
                Vector3.Dot(
                    rideAccelerationState,
                    capturedNormal));

        float accelerationDemand =
            Mathf.Max(
                commandedNormalAcceleration,
                realizedNormalAcceleration);

        float accelerationBudget =
            waveTimingMode == WaveTimingMode.ThreeWavesPerStair
                ? Mathf.Max(
                    50f,
                    spatialWaveAccelerationBudget)
                : Mathf.Max(
                    1f,
                    maximumRideAcceleration);

        float accelerationRatio =
            accelerationDemand /
            Mathf.Max(
                1f,
                accelerationBudget);

        float startRatio =
            Mathf.Clamp(
                postUpperSmoothAssistStartBudgetRatio,
                0f,
                1.45f);

        float fullRatio =
            Mathf.Clamp(
                postUpperSmoothAssistFullBudgetRatio,
                startRatio + 0.05f,
                1.5f);

        float blendT =
            Mathf.InverseLerp(
                startRatio,
                fullRatio,
                accelerationRatio);

        // Cubic SmoothStep: 3t^2 - 2t^3.
        // The derivative is zero at both ends, avoiding a hard switch.
        float accelerationAssist01 =
            blendT *
            blendT *
            (3f - 2f * blendT);

        // Preserve the original spatialSpeedRatio calculation as a mild
        // high-speed pressure term only.  At ratio <= 1 it does not suppress
        // roughness; above 1 it lets the smooth safety layer arrive earlier.
        float speedPressure =
            Mathf.Max(
                0f,
                spatialSpeedRatio - 1f);

        float speedGain =
            1f +
            speedPressure *
            Mathf.Clamp01(
                postUpperSmoothAssistSpeedRatioInfluence);

        float smoothAssist01 =
            Mathf.Clamp01(
                accelerationAssist01 *
                speedGain *
                Mathf.Clamp01(
                    naturalConnectControlAuthority01));

        postUpperHybridAccelerationDemand =
            accelerationDemand;

        postUpperHybridAccelerationBudget =
            accelerationBudget;

        postUpperHybridAccelerationRatio =
            accelerationRatio;

        postUpperHybridSmoothAssist01 =
            smoothAssist01;

        float currentTangentSpeed =
            Vector3.Dot(
                ballVisualEqualizer.velocity,
                tangent);

        float speedDeficit =
            Mathf.Max(
                0f,
                postUpperDirectionalTargetSpeed -
                currentTangentSpeed);

        float tangentAcceleration =
            speedDeficit /
            Mathf.Max(
                0.02f,
                postUpperDirectionalResponseTime);

        tangentAcceleration =
            Mathf.Clamp(
                tangentAcceleration,
                0f,
                Mathf.Max(
                    0f,
                    postUpperDirectionalMaximumAcceleration));

        tangentAcceleration *=
            postUpperDirectionalWeight01 *
            postUpperHybridSmoothAssist01;

        postUpperDirectionalAppliedAcceleration =
            tangentAcceleration;

        float lateralAcceleration = 0f;

        if (lateral.sqrMagnitude > Epsilon &&
            postUpperLateralAlignmentShare01 > 0f)
        {
            lateral.Normalize();

            float lateralSpeed =
                Vector3.Dot(
                    ballVisualEqualizer.velocity,
                    lateral);

            lateralAcceleration =
                -lateralSpeed /
                Mathf.Max(
                    0.02f,
                    postUpperLateralResponseTime);

            lateralAcceleration *=
                Mathf.Clamp01(
                    postUpperLateralAlignmentShare01) *
                postUpperDirectionalWeight01 *
                postUpperHybridSmoothAssist01;

            float lateralLimit =
                Mathf.Max(
                    0f,
                    postUpperDirectionalMaximumAcceleration);

            lateralAcceleration =
                Mathf.Clamp(
                    lateralAcceleration,
                    -lateralLimit,
                    lateralLimit);
        }

        postUpperLateralAppliedAcceleration =
            lateralAcceleration;

        Vector3 acceleration =
            tangent *
            tangentAcceleration +
            lateral *
            lateralAcceleration;

        if (acceleration.sqrMagnitude <= Epsilon)
            return;

        // Normal成分を含めないので、Upperの反射/跳ねは既存物理へ残る。
        ballVisualEqualizer.AddForce(
            acceleration,
            ForceMode.Acceleration);
    }
    #endregion // PROTECTED_12B_PostUpperDirectionalMemory


// ================================================================
// Upper collision -> measured T / measured energy loss
// ================================================================

    #region PROTECTED_13_PhysicalUpperEntry
    // [MOTION-PROTECTED] OnCollisionEnter
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void OnCollisionEnter(
        Collision collision)
    {
        if (synchronized ||
            collision == null ||
            collision.contactCount <= 0)
        {
            return;
        }

        if (equalizerControlMode == EqualizerControlMode.LegacyHybrid &&
            enableLogicalPhysicalHandoff &&
            negativeEnvelope &&
            negativeEnvelope.IsPhysicalLowerCandidateCollider(collision.collider))
        {
            RegisterPhysicalLowerContact(collision);
            return;
        }

        if (!negativeEnvelope ||
            !negativeEnvelope.IsUpperEnvelopeCollider(collision.collider))
        {
            return;
        }

        // Accept one Physical Upper per real Stair Lower cycle in Hybrid mode.
        if (!upperPeakArmed)
            return;

        upperPeakArmed = false;
        physicalUpperSeenSinceLastLower = true;
        phase = EqualizerPhase.UpperContact;

        // Upper接触時の階段下降World frameを記憶する。
        // 角度・Restitution・特定Wave値などの厳しい成立条件は置かない。
        ArmPostUpperDirectionalMemory(collision);

        // The entry bridge must never survive a real Upper impact. From this
        // exact contact onward PhysX + the existing 4R-Hn/Virtual-Lower system
        // own the oscillation. No inherited SlopeStick press remains.
        CompleteContinuousNormalSupportHandoff("FirstPhysicalUpper");
        if (naturalConnectBlendActive)
        {
            naturalConnectBlendActive = false;
            naturalConnectControlAuthority01 = 1f;
        }

        float now = Time.fixedTime;
        float minimumPeriod =
            minimumObservedCycleSeconds > 0f
                ? minimumObservedCycleSeconds
                : Time.fixedDeltaTime * 2f;

        if (lastUpperContactFixedTime >= 0f)
        {
            float measuredPeriod = now - lastUpperContactFixedTime;

            if (measuredPeriod >= minimumPeriod)
            {
                observedNaturalPeriodSeconds = measuredPeriod;
                negativeEnvelope.SubmitObservedGeometryPeriod(measuredPeriod);
            }
        }

        lastUpperContactFixedTime = now;

        if (equalizerControlMode == EqualizerControlMode.Gen3VirtualLower)
        {
            waveCycleIndex++;
            negativeEnvelope.NotifyCanonicalUpperPeak();
            RegisterGen3UpperCycleObservation();
        }
        else if (waveTimingMode == WaveTimingMode.NaturalObserved)
        {
            waveCycleIndex++;
            negativeEnvelope.NotifyCanonicalUpperPeak();
        }

        if (oscillationFrame.valid)
        {
            Vector3 normal = oscillationFrame.normal.normalized;
            float incoming = Mathf.Abs(
                Vector3.Dot(collision.relativeVelocity, normal));

            if (incoming > Epsilon)
            {
                pendingUpperImpactEnergyMeasurement = true;
                pendingUpperIncomingNormalSpeed = incoming;
                pendingUpperReflectionResolveFrames = 0;
                lastUpperIncomingNormalSpeed = incoming;
            }
        }

        // PhysX owns the actual Upper contact. Build D measures the first
        // separating Stable-N velocity after this callback. Optional governor may
        // remove excess rebound, but Upper never writes Canonical Energy/Hn;
        // Stair Lower remains the decay-energy authority.
    }
    #endregion // PROTECTED_13_PhysicalUpperEntry


    private void OnCollisionStay(Collision collision)
    {
        if (synchronized || collision == null || collision.contactCount <= 0)
            return;

        if (equalizerControlMode == EqualizerControlMode.LegacyHybrid &&
            enableLogicalPhysicalHandoff &&
            negativeEnvelope &&
            negativeEnvelope.IsPhysicalLowerCandidateCollider(collision.collider))
        {
            RegisterPhysicalLowerContact(collision);
        }
    }


    #region PROTECTED_14_PhysicalLowerAcceptance
    // [MOTION-PROTECTED] RegisterPhysicalLowerContact
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void RegisterPhysicalLowerContact(Collision collision)
    {
        if (collision == null ||
            !collision.collider ||
            !ballVisualEqualizer ||
            !oscillationFrame.valid)
        {
            return;
        }

        // Envelope only classifies the collider as a StairWay candidate.
        // Sync owns the temporal acceptance rule:
        //   accepted Upper -> descending -> StairWay contact.
        // An ascending/stale Stair contact must never steal Logical authority.
        if (!physicalUpperSeenSinceLastLower)
            return;

        Vector3 normal = oscillationFrame.normal.normalized;
        float supportNormalVelocity =
            Vector3.Dot(ReadSubjectVelocityVisual(), normal);
        float relativeNormalVelocity =
            Vector3.Dot(ballVisualEqualizer.velocity, normal) -
            supportNormalVelocity;

        float minimumDescendingSpeed =
            Mathf.Max(0f, physicalLowerMinimumDescendingSpeed);

        if (relativeNormalVelocity >= -minimumDescendingSpeed)
            return;

        physicalLowerContacts.Add(collision.collider);
        physicalLowerContactActive = physicalLowerContacts.Count > 0;
        phase = EqualizerPhase.LowerContact;

        if (pendingPhysicalLowerImpactEnergyMeasurement)
            return;

        float collisionRelative = Mathf.Abs(
            Vector3.Dot(collision.relativeVelocity, normal));
        float descendingSpeed = -relativeNormalVelocity;
        float incoming = Mathf.Max(collisionRelative, descendingSpeed);

        pendingPhysicalLowerIncomingNormalSpeed = incoming;
        lastPhysicalLowerIncomingNormalSpeed = incoming;
        pendingPhysicalLowerImpactEnergyMeasurement = incoming > Epsilon;
        pendingPhysicalLowerEnergyFrames = 0;
        pendingPhysicalLowerBestOutgoingNormalSpeed = 0f;

        // This real descending Stair impact closes the current physical wave.
        // Only now may the next Physical Upper be armed.
        physicalUpperSeenSinceLastLower = false;
        upperPeakArmed = true;
        ResetPostUpperDirectionalMemory();

    }
    #endregion // PROTECTED_14_PhysicalLowerAcceptance


    #region PROTECTED_15_UpperReflectionResolution
    // [MOTION-PROTECTED] ResolvePendingUpperImpactEnergyLoss
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void ResolvePendingUpperImpactEnergyLoss()
    {
        if (!pendingUpperImpactEnergyMeasurement ||
            !ballVisualEqualizer ||
            !negativeEnvelope ||
            !oscillationFrame.valid)
        {
            return;
        }

        pendingUpperReflectionResolveFrames++;

        Vector3 normal =
            oscillationFrame.normal.normalized;

        float supportNormalVelocity =
            Vector3.Dot(
                ReadSubjectVelocityVisual(),
                normal);

        float relativeNormalVelocity =
            Vector3.Dot(
                ballVisualEqualizer.velocity,
                normal) -
            supportNormalVelocity;

        // Lower -> Upper is +N. The first real reflected/separating state
        // therefore has negative relative vN.
        if (relativeNormalVelocity >= -0.01f)
        {
            if (pendingUpperReflectionResolveFrames >=
                Mathf.Max(
                    1,
                    upperReflectionResolveMaxFixedSteps))
            {
                pendingUpperImpactEnergyMeasurement = false;
                pendingUpperIncomingNormalSpeed = 0f;
                pendingUpperReflectionResolveFrames = 0;
            }

            return;
        }

        float incoming =
            Mathf.Max(
                Epsilon,
                pendingUpperIncomingNormalSpeed);

        float rawOutgoing =
            Mathf.Max(
                0f,
                -relativeNormalVelocity);

        float rawRestitution =
            rawOutgoing / incoming;

        float governedOutgoing =
            rawOutgoing;

        bool governorApplied = false;

        if (enableUpperReflectionGovernor &&
            incoming >=
                Mathf.Max(
                    0f,
                    upperReflectionGovernorMinimumIncomingSpeed))
        {
            float allowedOutgoing =
                incoming *
                Mathf.Clamp01(
                    upperReflectionRestitutionCap01);

            if (rawOutgoing >
                allowedOutgoing + Epsilon)
            {
                float excessOutgoing =
                    rawOutgoing -
                    allowedOutgoing;

                // raw vN is negative while leaving Upper.
                // +N removes only excess separating speed.
                ballVisualEqualizer.AddForce(
                    normal * excessOutgoing,
                    ForceMode.VelocityChange);

                governedOutgoing =
                    allowedOutgoing;

                governorApplied = true;
            }
        }

        lastUpperOutgoingNormalSpeed =
            governedOutgoing;

        lastImpactEnergyRetention01 =
            Mathf.Clamp01(
                (governedOutgoing *
                 governedOutgoing) /
                (incoming *
                 incoming));

        // Monotonic real-physics event telemetry for FutureSpline shadow scoring.
        // Do not reset this counter during normal synchronization/rejoin; prediction
        // probes compare a frozen issue-time count with the later count.
        upperReflectionCount++;
        lastUpperReflectionFixedTime = Time.fixedTime;

        pendingUpperImpactEnergyMeasurement =
            false;

        pendingUpperIncomingNormalSpeed =
            0f;

        pendingUpperReflectionResolveFrames =
            0;
    }
    #endregion // PROTECTED_15_UpperReflectionResolution


    #region PROTECTED_16_PhysicalLowerEnergyResolution
    // [MOTION-PROTECTED] ResolvePendingPhysicalLowerImpactEnergyLoss
    // Preserve behavior/state/order. Fine tuning is allowed; deletion is not.
    private void ResolvePendingPhysicalLowerImpactEnergyLoss()
    {
        if (!enableLogicalPhysicalHandoff ||
            !pendingPhysicalLowerImpactEnergyMeasurement ||
            !ballVisualEqualizer ||
            !negativeEnvelope ||
            !oscillationFrame.valid)
        {
            return;
        }

        Vector3 normal =
            oscillationFrame.normal.normalized;

        float supportNormalVelocity =
            Vector3.Dot(
                ReadSubjectVelocityVisual(),
                normal);

        float outgoing =
            Vector3.Dot(
                ballVisualEqualizer.velocity,
                normal) -
            supportNormalVelocity;

        pendingPhysicalLowerEnergyFrames++;

        if (outgoing > pendingPhysicalLowerBestOutgoingNormalSpeed)
            pendingPhysicalLowerBestOutgoingNormalSpeed = outgoing;

        float outgoingThreshold =
            Mathf.Max(
                0f,
                physicalLowerMeasurementOutgoingSpeed);

        bool clearUpwardRebound =
            outgoing > outgoingThreshold;

        bool contactReleased =
            !physicalLowerContactActive &&
            pendingPhysicalLowerEnergyFrames >= 1;

        bool timeout =
            pendingPhysicalLowerEnergyFrames >=
            Mathf.Max(
                2,
                physicalLowerEnergyResolveMaxFixedSteps);

        if (!clearUpwardRebound &&
            !contactReleased &&
            !timeout)
        {
            return;
        }

        float resolvedOutgoing =
            clearUpwardRebound
                ? outgoing
                : Mathf.Max(
                    0f,
                    pendingPhysicalLowerBestOutgoingNormalSpeed);

        string resolveReason =
            clearUpwardRebound
                ? "UpwardSignReversal"
                : contactReleased
                    ? "ContactReleased"
                    : "FixedStepTimeout";

        pendingPhysicalLowerImpactEnergyMeasurement = false;
        lastPhysicalLowerOutgoingNormalSpeed =
            resolvedOutgoing;

        float incoming =
            Mathf.Max(
                Epsilon,
                pendingPhysicalLowerIncomingNormalSpeed);

        float retention =
            Mathf.Clamp01(
                (resolvedOutgoing * resolvedOutgoing) /
                (incoming * incoming));

        retention =
            Mathf.Max(
                Mathf.Clamp01(
                    minimumPhysicalLowerEnergyRetention01),
                retention);

        lastPhysicalLowerEnergyRetention01 =
            retention;

        if (applyPhysicalLowerImpactEnergyLoss)
        {
            negativeEnvelope.SubmitPhysicalLowerImpactEnergyRetention(
                retention,
                $"DescendingAfterUpperStairPhysX/{resolveReason}");
        }

        if (logMeasurements)
        {
            Debug.Log(
                $"[EQ] cycle={waveCycleIndex} " +
                $"T={observedNaturalPeriodSeconds:F3}s " +
                $"lowerE={retention:F3} " +
                $"upperE={lastImpactEnergyRetention01:F3}",
                this);
        }


        pendingPhysicalLowerIncomingNormalSpeed = 0f;
        pendingPhysicalLowerEnergyFrames = 0;
        pendingPhysicalLowerBestOutgoingNormalSpeed = 0f;
    }
    #endregion // PROTECTED_16_PhysicalLowerEnergyResolution

    private void OnCollisionExit(
        Collision collision)
    {
        if (collision != null && collision.collider)
        {
            physicalLowerContacts.Remove(collision.collider);
            physicalLowerContactActive = physicalLowerContacts.Count > 0;
        }

        if (!synchronized)
        {
            if (physicalLowerContactActive)
                phase = EqualizerPhase.LowerContact;
            else if (rideRelativeNormalVelocity >= 0f)
                phase = EqualizerPhase.HopperFlight;
            else
                phase = EqualizerPhase.FreeFlight;
        }
    }

    private void UpdateWaveTimingAuthority()
    {
        if (!negativeEnvelope)
            return;

        negativeEnvelope.ConfigureSpatialWaveAuthority(
            waveTimingMode ==
            WaveTimingMode.ThreeWavesPerStair,
            Mathf.Max(
                1,
                spatialWavesPerStair));
    }


// ================================================================
// Regain compatibility
// ================================================================

    public bool TryGetRegainOscillationFrame(
        out Vector3 equalizerReleasePositionVisual,
        out Vector3 subjectReleasePositionVisual,
        out Vector3 oscillationNormalVisual)
    {
        equalizerReleasePositionVisual =
            releaseFrame.position;

        subjectReleasePositionVisual =
            releaseFrame.subjectPosition;

        oscillationNormalVisual =
            oscillationFrame.valid
                ? oscillationFrame.normal
                : Vector3.up;

        return
            !synchronized &&
            ballVisualEqualizer &&
            oscillationFrame.valid;
    }

    private static bool IsFinite(float value)
    {
        return
            !float.IsNaN(value) &&
            !float.IsInfinity(value);
    }


    private static bool IsFinite(Vector3 value)
    {
        return
            !float.IsNaN(value.x) &&
            !float.IsNaN(value.y) &&
            !float.IsNaN(value.z) &&
            !float.IsInfinity(value.x) &&
            !float.IsInfinity(value.y) &&
            !float.IsInfinity(value.z);
    }


// ================================================================
// Reacquire / external compatibility
// ================================================================

    private void ReacquireForNextIncident()
    {
        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        if (negativeEnvelope)
        {
            negativeEnvelope.SetUpperEnvelopeSolidEnabled(
                false,
                "Reacquire");

            negativeEnvelope.ClearEnvelope();
        }

        phase = EqualizerPhase.Reacquiring;

        EnterSynchronizedState();
    }


    public void BeginEmergencyVisualRecovery(float suggestedDuration = 0f)
    {
        ResolveReferences();
        CompleteContinuousNormalSupportHandoff("EmergencyVisualRecovery");
        ResetPostUpperDirectionalMemory();

        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        if (negativeEnvelope)
        {
            negativeEnvelope.SetUpperEnvelopeSolidEnabled(
                false,
                "EmergencyVisualRecovery");

            negativeEnvelope.ClearEnvelope();
        }

        Vector3 equalizerVelocity =
            ballVisualEqualizer.isKinematic
                ? ballVisual.velocity
                : ballVisualEqualizer.velocity;

        emergencyInitialRelativeOffset =
            ballVisualEqualizer.position -
            ballVisual.position;

        emergencyInitialRelativeVelocity =
            equalizerVelocity -
            ballVisual.velocity;

        emergencyInitialRotation =
            ballVisualEqualizer.rotation;

        emergencyVisualRecoveryInitialDistance =
            emergencyInitialRelativeOffset.magnitude;

        if (emergencyVisualRecoveryInitialDistance <=
            resumeSynchronizationSnapDistance)
        {
            EnterSynchronizedState();
            return;
        }

        float automaticDuration =
            emergencyVisualRecoveryInitialDistance /
            Mathf.Max(
                1f,
                emergencyReacquirePreferredRelativeSpeed);

        float requestedDuration =
            suggestedDuration > 0f
                ? suggestedDuration
                : automaticDuration;

        emergencyVisualRecoveryDuration =
            Mathf.Clamp(
                Mathf.Max(
                    emergencyReacquireMinimumDuration,
                    requestedDuration),
                Mathf.Max(
                    0.05f,
                    emergencyReacquireMinimumDuration),
                Mathf.Max(
                    emergencyReacquireMinimumDuration,
                    emergencyReacquireMaximumDuration));

        emergencyVisualRecoveryStartTime =
            Time.fixedTime;

        emergencyVisualRecoveryProgress01 = 0f;
        emergencyVisualRecoveryActive = true;

        synchronized = false;
        phase = EqualizerPhase.Reacquiring;

        // Emergency中は相対HermiteだけをEqualizerの位置権威にする。
        ballVisualEqualizer.useGravity = false;
        ballVisualEqualizer.detectCollisions = false;
        ballVisualEqualizer.isKinematic = true;

        rideAccelerationState = Vector3.zero;
        transportAccelerationState = Vector3.zero;
        goalPlanarVelocityState = Vector3.zero;
        ResetRideSupportKinematics();

    }


    private void UpdateEmergencyVisualRecovery()
    {
        if (!emergencyVisualRecoveryActive ||
            !ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        float duration =
            Mathf.Max(
                0.0001f,
                emergencyVisualRecoveryDuration);

        float elapsed =
            Mathf.Max(
                0f,
                Time.fixedTime -
                emergencyVisualRecoveryStartTime);

        float t =
            Mathf.Clamp01(
                elapsed / duration);

        emergencyVisualRecoveryProgress01 = t;

        // BallVisualを動く基準原点として、相対offsetだけを
        // Hermite(offset0, relativeV0 -> 0, 0)で消していく。
        // BallVisual自身がEmergency Hermite中でも、その軌道をそのまま追従できる。
        Vector3 relativeOffset =
            EvaluateHermiteVector(
                emergencyInitialRelativeOffset,
                emergencyInitialRelativeVelocity,
                Vector3.zero,
                Vector3.zero,
                duration,
                t);

        Vector3 targetPosition =
            ballVisual.position +
            relativeOffset;

        float rotationT =
            t * t * (3f - 2f * t);

        Quaternion targetRotation =
            Quaternion.Slerp(
                emergencyInitialRotation,
                ballVisual.rotation,
                rotationT);

        ballVisualEqualizer.MovePosition(
            targetPosition);

        ballVisualEqualizer.MoveRotation(
            targetRotation);

        if (t < 1f)
            return;

        // relative Hermiteの終点は厳密にoffset=0, relativeVelocity=0。
        // ここでのCopyBallVisualPoseは大距離snapではなく最終丸めだけになる。
        emergencyVisualRecoveryActive = false;

        EnterSynchronizedState();
    }


    private static Vector3 EvaluateHermiteVector(
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


    public void ResumeSynchronization()
    {
        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        if (negativeEnvelope)
        {
            negativeEnvelope.SetUpperEnvelopeSolidEnabled(
                false,
                "ResumeSynchronization");

            negativeEnvelope.ClearEnvelope();
        }

        phase = EqualizerPhase.Reacquiring;

        // TerminalRejoin完了はBallVisual側の完全同期点。
        // EqualizerだけEmergency/Hermite補間を継続すると、
        // その最中に次Incidentが始まりReleaseが競合する。
        // ここでは必ず同じFixed stepでBallVisualへ即時同期する。
        EnterSynchronizedState();
    }


    [System.Obsolete("Use ResumeSynchronization().")]
    public void ResumeSynchronization1() => ResumeSynchronization();

    [System.Obsolete("Use ResumeSynchronization().")]
    public void ResumeSynchronization2() => ResumeSynchronization();


    // ================================================================
    // Post Turn Hermite Bridge
    // ================================================================
    // BallVisualSlopeDriveが旋回直後の短いHermite区間を所有している間、
    // Equalizerは新しいRelease Energyを作らない。
    // すでにSynchronizedならBallVisual poseをそのまま追従し、
    // 非同期状態なら既存の相対Hermite Reacquireを同じ時間予算で開始する。

    public void BeginPostTurnHermiteBridge(float suggestedDuration = 0.10f)
    {
        ResolveReferences();

        postTurnHermiteBridgeActive = true;

        if (!ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        if (synchronized)
        {
            // FixedUpdate / Equalizeの通常CopyBallVisualPoseを継続する。
            // Energy / phase / Upper衝突イベントは新規生成しない。
            CopyBallVisualPose();

            return;
        }

        // 旋回入力がActive Equalizer中に来た例外ケース。
        // hard snapではなく、既存の相対HermiteでBallVisualへ回収する。
        BeginEmergencyVisualRecovery(
            Mathf.Max(0.05f, suggestedDuration));

    }


    public void EndPostTurnHermiteBridge()
    {
        postTurnHermiteBridgeActive = false;

        // synchronized=trueなら何もしない。
        // RelativeHermiteがちょうど完了していれば既にSynchronizedへ戻っている。
        // まだRecovery中なら、その連続軌道を壊さず完了まで任せる。
        if (synchronized)
            CopyBallVisualPose();

    }


    public void PrepareForVisualFrameTurnMapping()
    {
        ResolveReferences();
    }


    public void ApplyVisualFrameTurnDelta(
        Vector3 pivot,
        Quaternion deltaTurn)
    {
        ResolveReferences();

        if (synchronized)
            CopyBallVisualPose();

        gen3NormalAccelerationState = 0f;
    }

// ================================================================
// Collision ownership
// ================================================================

    public void RefreshVisualCollisionOwnership()
    {
        ResolveReferences();

        if (!ballVisual ||
            !ballVisualEqualizerCollider)
        {
            return;
        }

        SphereCollider ballVisualCollider =
            ballVisual.GetComponent<SphereCollider>();

        if (ballVisualCollider)
        {
            Physics.IgnoreCollision(
                ballVisualCollider,
                ballVisualEqualizerCollider,
                true);
        }

        if (negativeEnvelope)
        {
            negativeEnvelope.RefreshEqualizerBoundaryCollisionOwnership();
        }

        if (equalizerControlMode == EqualizerControlMode.Gen3VirtualLower)
            RefreshGen3CollisionOwnership();
    }

    private void OnDisable()
    {
        RestoreGen3CollisionOwnership();
    }

    private void OnDestroy()
    {
        RestoreGen3CollisionOwnership();
    }

}
