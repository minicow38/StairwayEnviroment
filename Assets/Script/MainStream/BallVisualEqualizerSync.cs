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

    [Header("Spatial Wave 24m/s Speed Normalization")]
    [Tooltip(
        "Spatial Waveの基準接線速度[m/s]。現在成功している16m/s帯を基準にし、\n" +
        "これより高速ではAcceleration/Jerk budgetだけを速度正規化します。Wave位置は変えません。")]
    [Min(1f)]
    [SerializeField]
    private float spatialReferenceTangentSpeed = 16f;

    [Tooltip("速度正規化倍率の上限。16->24m/sなら1.5。")] [Range(1f, 2f)] [SerializeField]
    private float maximumSpatialSpeedRatio = 1.5f;

    [Tooltip(
        "Jerkは理論上speedRatio^3ですが、最初から3.375倍まで許さず安全上限を掛けます。")]
    [Range(1f, 4f)]
    [SerializeField]
    private float maximumSpatialJerkScale = 2.5f;

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

    [Header("Upper Impact Observation")]
    [Tooltip(
        "Upper->Upper実測周期として採用する最小時間[s]。\n" +
        "0ならFixedDeltaTime*2を使用。")]
    [Min(0f)]
    [SerializeField]
    private float minimumObservedCycleSeconds = 0f;

    [Tooltip(
        "Upper衝突前後のStable-N速度から求めるEnergy retentionをEnvelopeへ反映するか。")]
    [SerializeField]
    private bool applyMeasuredImpactEnergyLoss = true;

    [Tooltip(
        "衝突後速度がSolver/接触ノイズで極端な場合のEnergy retention下限。")]
    [Range(0f, 1f)]
    [SerializeField]
    private float minimumImpactEnergyRetention01 = 0.05f;

    [Tooltip(
        "Upperを一度受理した後、4R-Hnのこの高さ率より下へ戻るまで次Upperを受理しません。\n" +
        "Mesh swap / 接触継続による1 FixedUpdate毎の誤カウントを防ぎます。")]
    [Range(0.05f, 0.75f)]
    [SerializeField]
    private float upperPeakRearmHeight01 = 0.35f;

    [Tooltip(
        "Upper impact energyは接触直後ではなく、この高さ率より下へ離れて下降した時に測定します。")]
    [Range(0.4f, 0.95f)]
    [SerializeField]
    private float impactMeasurementReleaseHeight01 = 0.85f;

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

    [SerializeField] private float current4RHnMeters;
    [SerializeField] private float current4RHnR;

    [Header("Upper Placement Runtime - Read Only")]
    [Tooltip("World-Y配置補正後の実Upper接触面までのStable-N中心距離[m]。")]
    [SerializeField] private float currentUpperNormalSpanMeters;

    [Tooltip("World-Y配置補正後の実Upper接触面までのStable-N中心距離[R]。")]
    [SerializeField] private float currentUpperNormalSpanR;

    [Tooltip("現在のEqualizer位置から実Upper接触中心へ向かう方向。診断/追跡参照用。Stable-N面法線は別Authorityとして維持します。")]
    [SerializeField] private Vector3 currentUpperTargetDirectionVisual = Vector3.up;

    [Tooltip("実Upper span / 元の4R-Hn。1未満ならColliderを下げたぶん到達Energy基準も減ります。")]
    [SerializeField] private float upperPlacementEnergyRatio = 1f;

    [Tooltip("元のLogical Launch Energyへ実Upper span/4R-Hnを掛けた配置対応Energy基準[J]。厳密な外力仕事量ではなく、初速過剰を防ぐEnergy scaleです。")]
    [SerializeField] private float upperPlacementRequiredEnergyJoule;

    [Tooltip("配置対応Energy比から得る初回上昇の速度Scale。Colliderを下げた時だけ1未満になり、上げてもEnergyは新規生成しません。")]
    [SerializeField] private float upperPlacementInitialSpeedScale = 1f;

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
    [SerializeField] private float rideAccelerationCommand;

    [SerializeField] private float transportLagMeters;
    [SerializeField] private float subjectTangentSpeed;
    [SerializeField] private float equalizerTangentSpeed;
    [SerializeField] private float requiredCatchUpSpeed;
    [SerializeField] private float catchUpAccelerationCommand;

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

    [Tooltip("現在接線速度 / spatialReferenceTangentSpeed。")] [SerializeField]
    private float spatialSpeedRatio = 1f;

    [SerializeField] private float spatialAccelerationBudgetScale = 1f;
    [SerializeField] private float spatialJerkBudgetScale = 1f;
    [SerializeField] private float spatialFeedForwardVelocity;
    [SerializeField] private float spatialFeedForwardAcceleration;
    [SerializeField, Range(0f, 1f)] private float terminalSpatialTransportAssist01;

    [Tooltip("Raw進捗が過去最大より後ろにあり、Monotonic Hold中ならTRUE。")] [SerializeField]
    private bool spatialMonotonicHoldActive;

    [Tooltip("Monotonic Holdへ入った瞬間にEnvelope Trigger Pulseを要求した回数。")] [SerializeField]
    private int spatialMonotonicTriggerPulseCount;

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
    [SerializeField] private bool upperPeakArmed = true;

    [Header("Logical / Physical Authority Runtime - Read Only")] [SerializeField]
    private NormalAuthorityZone normalAuthorityZone = NormalAuthorityZone.LogicalEnvelope;

    [SerializeField, Range(0f, 1f)] private float normalLogicalAuthority01 = 1f;
    [SerializeField] private float normalTargetAccelerationBeforeAuthority;
    [SerializeField] private float normalTargetAccelerationAfterAuthority;
    [SerializeField] private float normalActiveJerkLimit;
    [SerializeField] private bool physicalUpperSeenSinceLastLower;
    [SerializeField] private bool physicalLowerContactActive;
    [SerializeField] private int physicalLowerContactCount;
    [SerializeField] private int rejectedAscendingStairContactCount;
    [SerializeField] private string lastPhysicalLowerColliderName = "None";
    [SerializeField] private bool rideSupportFrameContinuous = true;
    [SerializeField] private int rideSupportFrameResetCount;
    [SerializeField] private float rideSupportLastCenterJumpMeters;
    [SerializeField] private float rideSupportLastMeasuredCenterSpeed;
    [SerializeField] private float lastPhysicalLowerIncomingNormalSpeed;
    [SerializeField] private float lastPhysicalLowerOutgoingNormalSpeed;
    [SerializeField] private float lastPhysicalLowerEnergyRetention01 = 1f;
    [SerializeField] private int physicalLowerEnergyResolveFrameCount;
    [SerializeField] private float physicalLowerBestObservedOutgoingNormalSpeed;
    [SerializeField] private string physicalLowerEnergyResolveReason = "None";

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
    [SerializeField, Range(0f, 1f)] private float naturalConnectControlAuthority01 = 1f;
    [SerializeField] private Vector3 naturalConnectAveragedVelocity;
    [SerializeField] private Vector3 naturalConnectAveragedSubjectVelocity;
    [SerializeField] private Vector3 naturalConnectAveragedNormal = Vector3.up;
    [SerializeField] private Vector3 naturalConnectAveragedTangent = Vector3.forward;
    [SerializeField] private float naturalConnectMeasuredRelativeNormalSpeed;
    [SerializeField] private float naturalConnectMeasuredEnergyJoule;

    [SerializeField] private float effectiveRideSpringDamper;


    [Header("Hybrid Damping Runtime - Read Only")]
    [SerializeField] private float activeTargetNormalDriveAcceleration;
    [SerializeField] private float totalPassiveNormalDampingAcceleration;
    [SerializeField] private float controllerPassiveDampingAcceleration;
    [SerializeField] private float rigidbodyPassiveDampingAcceleration;
    [SerializeField] private float rigidbodyPassiveDampingAppliedAcceleration;
    [SerializeField, Range(0f, 1f)] private float rigidbodyPassiveDampingShareApplied01;


    [SerializeField] private float positionErrorToBallVisual;
    [SerializeField] private float velocityErrorToBallVisual;
    [SerializeField] private float currentKineticEnergy;
    [SerializeField] private float subjectTransportGap;
    [SerializeField] private float subjectDistance;

    [SerializeField] private int physicsCollisionCount;
    [SerializeField] private int upperCollisionCount;

    [SerializeField] private float lastUpperIncomingNormalSpeed;
    [SerializeField] private float lastUpperOutgoingNormalSpeed;
    [SerializeField] private float lastImpactEnergyRetention01 = 1f;

// Compatibility diagnostics.
    [SerializeField] private float physicsCleanRate = 1f;
    [SerializeField] private float gameImpactSuccessRate = 1f;
    [SerializeField] private float releaseOverallSuccessRate = 1f;
    [SerializeField] private float dampingFeasibility01 = 1f;
    [SerializeField] private float subjectConvergenceFeasibility = 1f;
    [SerializeField] private float availableTimeToLimit;
    [SerializeField] private float averageGameImpactQuality = 1f;

// ================================================================
// Runtime state
// ================================================================

    private const float Epsilon = 0.000001f;

    private OscillationFrame oscillationFrame;
    private ReleaseFrame releaseFrame;

    private Vector3 rideAccelerationState;
    private Vector3 transportAccelerationState;
    private Vector3 goalPlanarVelocityState;

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

    private readonly HashSet<Collider> physicalLowerContacts =
        new HashSet<Collider>();

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

    public EqualizerPhase Phase => phase;

    public float EqualizerMass =>
        ballVisualEqualizer
            ? Mathf.Max(0.0001f, ballVisualEqualizer.mass)
            : 1f;

    public float PositionErrorToBallVisual => positionErrorToBallVisual;
    public float VelocityErrorToBallVisual => velocityErrorToBallVisual;
    public float CurrentKineticEnergy => currentKineticEnergy;
    public float SubjectTransportGap => subjectTransportGap;
    public float SubjectDistance => subjectDistance;

    public float CleanImpactRate => physicsCleanRate;
    public float PhysicsCleanImpactRate => physicsCleanRate;
    public float GameImpactSuccessRate => gameImpactSuccessRate;
    public float ReleaseOverallSuccessRate => releaseOverallSuccessRate;
    public float DampingFeasibility01 => dampingFeasibility01;
    public float SubjectConvergenceFeasibility => subjectConvergenceFeasibility;
    public float AvailableTimeToLimit => availableTimeToLimit;
    public float AverageGameImpactQuality => averageGameImpactQuality;

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
    public float SpatialSubjectTimeToGo => spatialSubjectTimeToGo;
    public float SpatialRequiredArrivalSpeed => spatialRequiredArrivalSpeed;
    public float SpatialArrivalFeasibility01 => spatialArrivalFeasibility01;

// ================================================================
// Unity
// ================================================================

    private void Start()
    {
        Debug.Log(
            "[EQUALIZER BUILD] NaturalConnect-Hybrid50-StableN-20260915-E",
            this);

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

        EnterSynchronizedState(
            "Start");
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
            UpdateEmergencyVisualRecovery();
            UpdateObserver();
            return;
        }

        if (naturalConnectSamplingActive)
        {
            UpdateNaturalConnectSampling();
            UpdateObserver();
            return;
        }

        if (synchronized)
        {
            CopyBallVisualPose();
            UpdateObserver();
            return;
        }

        UpdateNaturalConnectBlend();
        UpdateWaveTimingAuthority();

        // Upper impact remains a diagnostic in Hybrid Authority mode.
        ResolvePendingUpperImpactEnergyLoss();

        // Real StairWay impact is the physical dissipation authority.
        ResolvePendingPhysicalLowerImpactEnergyLoss();

        UpdateFloatingRideSpring();
        ApplyGoalVelocityCatchUp();
        UpdateObserver();
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


    private void EnterSynchronizedState(
        string reason)
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

        upperPeakArmed = true;
        physicalUpperSeenSinceLastLower = false;

        physicalLowerContacts.Clear();
        physicalLowerContactActive = false;
        physicalLowerContactCount = 0;
        rejectedAscendingStairContactCount = 0;
        rideSupportFrameContinuous = true;
        rideSupportFrameResetCount = 0;
        rideSupportLastCenterJumpMeters = 0f;
        rideSupportLastMeasuredCenterSpeed = 0f;
        lastPhysicalLowerColliderName = "None";
        pendingPhysicalLowerImpactEnergyMeasurement = false;
        pendingPhysicalLowerIncomingNormalSpeed = 0f;
        pendingPhysicalLowerEnergyFrames = 0;
        pendingPhysicalLowerBestOutgoingNormalSpeed = 0f;
        physicalLowerEnergyResolveFrameCount = 0;
        physicalLowerBestObservedOutgoingNormalSpeed = 0f;
        physicalLowerEnergyResolveReason = "None";
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
        pendingUpperIncomingNormalSpeed = 0f;

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

        Debug.Log(
            $"[EQUALIZER SYNC] reason={reason}",
            this);
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
            Debug.Log(
                "[EQUALIZER NATURAL CONNECT SUPPRESSED] Hermite bridge owns visual continuity.",
                this);
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

        // Sampling中はEqualizerをBallVisualへ完全同期。ここでは独立Forceを出さない。
        synchronized = true;
        phase = EqualizerPhase.Synchronized;
        CopyBallVisualPose();

        Debug.Log(
            $"[EQUALIZER NATURAL CONNECT BEGIN] " +
            $"time={Time.fixedTime:F4} " +
            $"H0={naturalConnectReferenceHeight:F4}m " +
            $"v={ballVisual.velocity:F4} " +
            $"N={naturalConnectPreferredNormal:F4} " +
            $"T={naturalConnectPreferredTangent:F4}",
            this);

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


    private void UpdateNaturalConnectSampling()
    {
        if (!naturalConnectSamplingActive ||
            !ballVisual ||
            !ballVisualEqualizer)
        {
            return;
        }

        CopyBallVisualPose();

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
            EnterSynchronizedState("NaturalConnectEnvelopeArmFailed");
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
        pendingUpperImpactEnergyMeasurement = false;
        pendingUpperIncomingNormalSpeed = 0f;

        physicalLowerContacts.Clear();
        upperPeakArmed = true;
        physicalUpperSeenSinceLastLower = false;
        physicalLowerContactActive = false;
        rejectedAscendingStairContactCount = 0;
        rideSupportFrameContinuous = true;
        rideSupportFrameResetCount = 0;
        rideSupportLastCenterJumpMeters = 0f;
        rideSupportLastMeasuredCenterSpeed = 0f;
        pendingPhysicalLowerImpactEnergyMeasurement = false;
        pendingPhysicalLowerIncomingNormalSpeed = 0f;
        pendingPhysicalLowerEnergyFrames = 0;
        pendingPhysicalLowerBestOutgoingNormalSpeed = 0f;
        physicalLowerEnergyResolveFrameCount = 0;
        physicalLowerBestObservedOutgoingNormalSpeed = 0f;
        physicalLowerEnergyResolveReason = "None";
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

        UpdateWaveTimingAuthority();

        negativeEnvelope.SetUpperEnvelopeSolidEnabled(
            true,
            "NaturalConnectRelease");

        negativeEnvelope.RefreshEqualizerBoundaryCollisionOwnership();
        phase = EqualizerPhase.FreeFlight;

        if (negativeEnvelope.TryGetFloatingRideFrame(
                out Vector3 releaseLowerCenter,
                out _,
                out Vector3 releaseUpperCenter,
                out _,
                out Vector3 releaseStableNormal,
                out float releaseEffectiveSpanMeters,
                out _,
                out _))
        {
            UpdateUpperPlacementDiagnostics(
                releaseLowerCenter,
                releaseUpperCenter,
                releaseStableNormal,
                releaseEffectiveSpanMeters);
        }

        Debug.Log(
            $"[EQUALIZER NATURAL CONNECT COMMIT] " +
            $"samples={naturalConnectSampleCount} " +
            $"sampleTime={naturalConnectSamplingElapsed:F4}s " +
            $"vAvg={measuredVelocity:F4} " +
            $"subjectVAvg={measuredSubjectVelocity:F4} " +
            $"vNrel={measuredRelativeNormalSpeed:F4}m/s " +
            $"measuredE={naturalConnectMeasuredEnergyJoule:F4}J " +
            $"envelopeArmE={envelopeEnergyJoule:F4}J " +
            $"H0={naturalConnectReferenceHeight:F4}m " +
            $"blend={naturalConnectBlendSeconds:F4}s " +
            $"hybridPassiveShare={rigidbodyPassiveDampingShare01:F2}",
            this);
    }


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

            Debug.Log(
                $"[EQUALIZER NATURAL CONNECT COMPLETE] " +
                $"elapsed={naturalConnectBlendElapsed:F4}s " +
                $"authority={naturalConnectControlAuthority01:F3}",
                this);
        }
    }


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
    }

// ================================================================
// Floating Ride Spring
// ================================================================

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

        UpdateUpperPlacementDiagnostics(
            lowerCenter,
            upperCenter,
            normal,
            spanMeters);

        // spanR is the effective shifted Upper span returned by the Envelope.
        // Keep it in the dedicated runtime field; current4RHnR remains the
        // unshifted 4R-Hn diagnostic from TryGetCurrentPresentationCenterTravel.
        currentUpperNormalSpanR = spanR;

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
            TryResolveSpatialWaveReference(
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

        float accelerationLimit =
            spatialMode
                ? Mathf.Max(
                    50f,
                    spatialWaveAccelerationBudget *
                    Mathf.Max(1f, spatialAccelerationBudgetScale))
                : Mathf.Max(
                    1f,
                    maximumRideAcceleration);


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
                ? Mathf.Max(
                    100f,
                    spatialWaveJerkBudget *
                    Mathf.Max(1f, spatialJerkBudgetScale))
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

        // Safety authority and Natural Connect authority are independent.
        // Connect smoothly opens the whole Hybrid controller without changing
        // the internal 50/50 passive damping split.
        authority01 *= Mathf.Clamp01(naturalConnectControlAuthority01);

        normalLogicalAuthority01 = authority01;
        normalActiveJerkLimit = activeJerkLimit;
        normalAuthorityZone = resolvedZone;
        normalTargetAccelerationAfterAuthority =
            desiredScalarAcceleration * authority01;

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

        rideAccelerationCommand =
            Vector3.Dot(rideAccelerationState, normal);

        if (normalAuthorityZone != previousNormalAuthorityZone)
        {
            Debug.Log(
                $"[EQUALIZER AUTHORITY] " +
                $"{previousNormalAuthorityZone}->{normalAuthorityZone} " +
                $"hN={rideActualHeight:F4}m " +
                $"vN={rideRelativeNormalVelocity:F4}m/s " +
                $"authority={normalLogicalAuthority01:F3} " +
                $"aTarget={normalTargetAccelerationAfterAuthority:F3}m/s2 " +
                $"passive={totalPassiveNormalDampingAcceleration:F3}m/s2 " +
                $"rbDamp={rigidbodyPassiveDampingAppliedAcceleration:F3}m/s2 " +
                $"rbShare={rigidbodyPassiveDampingShareApplied01:F2} " +
                $"jerk={normalActiveJerkLimit:F1}m/s3",
                this);

            previousNormalAuthorityZone = normalAuthorityZone;
        }



        phase =
            physicalLowerContactActive
                ? EqualizerPhase.LowerContact
                : rideRelativeNormalVelocity >= 0f
                    ? EqualizerPhase.HopperFlight
                    : EqualizerPhase.FreeFlight;
    }


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


    private void UpdateUpperPlacementDiagnostics(
        Vector3 lowerCenter,
        Vector3 upperCenter,
        Vector3 stableNormal,
        float effectiveSpanMeters)
    {
        // 4R-Hn is kept as the unshifted canonical geometry.
        // The moved Collider supplies a separate effective Stable-N span.
        if (negativeEnvelope &&
            negativeEnvelope.TryGetCurrentPresentationCenterTravel(
                out float raw4RHnMeters,
                out float raw4RHnR))
        {
            current4RHnMeters = raw4RHnMeters;
            current4RHnR = raw4RHnR;
        }

        currentUpperNormalSpanMeters =
            Mathf.Max(0f, effectiveSpanMeters);

        float radius =
            ResolveEqualizerWorldRadius();

        currentUpperNormalSpanR =
            radius > Epsilon
                ? currentUpperNormalSpanMeters / radius
                : 0f;

        Vector3 toUpper =
            upperCenter - ballVisualEqualizer.position;

        currentUpperTargetDirectionVisual =
            toUpper.sqrMagnitude > Epsilon
                ? toUpper.normalized
                : stableNormal;

        // Placement Energy law:
        //     E_upper = E_launch * D_effective / D_(4R-Hn)
        //
        // This does NOT rewrite 4R-Hn or create extra Energy. It only removes
        // the portion of the initial normal kinetic Energy that is no longer
        // needed when the whole Upper plane is moved downward.
        upperPlacementEnergyRatio =
            current4RHnMeters > Epsilon
                ? Mathf.Max(
                    0f,
                    currentUpperNormalSpanMeters / current4RHnMeters)
                : 1f;

        upperPlacementRequiredEnergyJoule =
            Mathf.Max(0f, logicalLaunchEnergyJoule) *
            upperPlacementEnergyRatio;

        // Raising Upper may require more Energy, but this class must never
        // manufacture it. Only downward placement can reduce entry-connect speed.
        upperPlacementInitialSpeedScale =
            Mathf.Sqrt(
                Mathf.Clamp01(
                    upperPlacementEnergyRatio));
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

        rideSupportLastCenterJumpMeters = centerJumpMeters;
        rideSupportLastMeasuredCenterSpeed = apparentCenterSpeed;

        if (projectionOrFrameJump)
        {
            rideSupportFrameResetCount++;

            previousRideSupportCenter = supportCenter;
            previousRideSupportVelocity = fallbackVelocity;
            previousRideSupportSampleTime = sampleTime;

            Debug.Log(
                $"[EQUALIZER SUPPORT FRAME RESET] " +
                $"jump={centerJumpMeters:F4}m " +
                $"apparentSpeed={apparentCenterSpeed:F2}m/s " +
                $"fallbackSpeed={fallbackVelocity.magnitude:F2}m/s " +
                $"maxJump={maximumCenterJump:F4}m " +
                $"count={rideSupportFrameResetCount}",
                this);

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


    private bool TryResolveSpatialWaveReference(
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
        // Monotonic spatial phase + one-shot Envelope Trigger pulse
        // ------------------------------------------------------------
        // Equalizer本体はCollider反発でSpline上を後退してよい。
        // Presentation位相だけは巻き戻さず、その「後退を初めて検出した瞬間」に
        // Envelope Solidを1 FixedUpdateだけTrigger化する実験を行う。
        float previousMonotonicProgress01 =
            spatialDomainProgress01;

        bool wasMonotonicHoldActive =
            spatialMonotonicHoldActive;

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

        // Holdへ「入った瞬間」だけ1回。Hold中の毎FixedUpdateでは再発火しない。
        if (spatialMonotonicHoldActive &&
            !wasMonotonicHoldActive)
        {
            bool triggerPulseAccepted =
                negativeEnvelope.PulseCurrentEnvelopeTriggerOneFixedStep(
                    "SpatialMonotonicBackstep");

            if (triggerPulseAccepted)
                spatialMonotonicTriggerPulseCount++;

            Debug.Log(
                $"[EQUALIZER MONOTONIC HOLD] " +
                $"raw={spatialRawDomainProgress01:F6} " +
                $"held={spatialDomainProgress01:F6} " +
                $"backstep={(spatialDomainProgress01 - spatialRawDomainProgress01):F6} " +
                $"triggerPulse={triggerPulseAccepted} " +
                $"count={spatialMonotonicTriggerPulseCount}",
                this);
        }

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

        float speedForFeasibility =
            Mathf.Max(
                0.25f,
                Mathf.Max(
                    Mathf.Abs(
                        equalizerForwardSpeed),
                    Mathf.Abs(
                        subjectForwardSpeed)));

        spatialSpeedRatio =
            Mathf.Clamp(
                speedForFeasibility /
                Mathf.Max(1f, spatialReferenceTangentSpeed),
                1f,
                Mathf.Max(1f, maximumSpatialSpeedRatio));

        spatialAccelerationBudgetScale =
            spatialSpeedRatio *
            spatialSpeedRatio;

        spatialJerkBudgetScale =
            Mathf.Min(
                spatialSpeedRatio *
                spatialSpeedRatio *
                spatialSpeedRatio,
                Mathf.Max(1f, maximumSpatialJerkScale));

        float spatialFrequency =
            waveCount *
            speedForFeasibility /
            activeArcLengthMeters;

        spatialReferencePeriodSeconds =
            1f /
            Mathf.Max(
                0.0001f,
                spatialFrequency);

        float desiredCarrierHeight =
            envelopeSpanMeters *
            Mathf.Clamp(
                spatialCarrierHeightFractionOfEnvelope,
                0.05f,
                0.8f);

        // h = H/2(1-cos wt)
        // |a|max = H/2 * w^2 = 2*pi^2*H*f^2
        float feasibleCarrierHeight =
            Mathf.Max(
                0f,
                spatialWaveAccelerationBudget *
                spatialAccelerationBudgetScale) /
            Mathf.Max(
                0.0001f,
                2f *
                Mathf.PI *
                Mathf.PI *
                spatialFrequency *
                spatialFrequency);

        spatialCarrierHeightBeforeFeasibility =
            Mathf.Min(
                envelopeSpanMeters * 0.95f,
                Mathf.Min(
                    desiredCarrierHeight,
                    feasibleCarrierHeight));

        spatialCarrierFeasibility01 =
            desiredCarrierHeight > Epsilon
                ? Mathf.Clamp01(
                    spatialCarrierHeightBeforeFeasibility /
                    desiredCarrierHeight)
                : 1f;

        // Carrier amplitude is now owned only by Envelope span + feasibility.
        // The former BallVisual momentum-bias experiment is intentionally removed.
        spatialCarrierHeightMeters =
            spatialCarrierHeightBeforeFeasibility;

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

// ================================================================
// Goal velocity catch-up
// ================================================================

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

        transportLagMeters =
            Vector3.Dot(
                planeGap,
                tangent);

        Vector3 subjectPlanarVelocity =
            Vector3.ProjectOnPlane(
                subjectVelocity,
                normal);

        subjectTangentSpeed =
            Vector3.Dot(
                subjectPlanarVelocity,
                tangent);

        Vector3 equalizerPlanarVelocity =
            Vector3.ProjectOnPlane(
                ballVisualEqualizer.velocity,
                normal);

        equalizerTangentSpeed =
            Vector3.Dot(
                equalizerPlanarVelocity,
                tangent);

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

        catchUpAccelerationCommand =
            Vector3.Dot(
                transportAccelerationState,
                tangent) *
            connectTransportAuthority;
    }


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

// ================================================================
// Upper collision -> measured T / measured energy loss
// ================================================================

    private void OnCollisionEnter(
        Collision collision)
    {
        if (synchronized ||
            collision == null ||
            collision.contactCount <= 0)
        {
            return;
        }

        physicsCollisionCount++;

        if (enableLogicalPhysicalHandoff &&
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
        upperCollisionCount++;
        phase = EqualizerPhase.UpperContact;

        // The smoothing authority owns ONLY release -> first real Upper.
        // From this exact contact onward, the normal 4R-Hn spatial carrier
        // resumes with no entry-connect scaling.

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

        if (waveTimingMode == WaveTimingMode.NaturalObserved)
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
                lastUpperIncomingNormalSpeed = incoming;
            }
        }

        // PhysX owns Upper collision response. In Hybrid mode its measured loss
        // is NOT multiplied into Envelope energy; Stair Lower owns that authority.
    }


    private void OnCollisionStay(Collision collision)
    {
        if (synchronized || collision == null || collision.contactCount <= 0)
            return;

        if (enableLogicalPhysicalHandoff &&
            negativeEnvelope &&
            negativeEnvelope.IsPhysicalLowerCandidateCollider(collision.collider))
        {
            RegisterPhysicalLowerContact(collision);
        }
    }


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
        {
            rejectedAscendingStairContactCount++;
            return;
        }

        bool newlyAdded = physicalLowerContacts.Add(collision.collider);
        physicalLowerContactActive = physicalLowerContacts.Count > 0;
        phase = EqualizerPhase.LowerContact;

        if (newlyAdded)
        {
            physicalLowerContactCount++;
            lastPhysicalLowerColliderName = collision.collider.name;
        }

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
        physicalLowerEnergyResolveFrameCount = 0;
        physicalLowerBestObservedOutgoingNormalSpeed = 0f;
        physicalLowerEnergyResolveReason = "Pending";

        // This real descending Stair impact closes the current physical wave.
        // Only now may the next Physical Upper be armed.
        physicalUpperSeenSinceLastLower = false;
        upperPeakArmed = true;

        Debug.Log(
            $"[EQUALIZER PHYSICAL LOWER] " +
            $"stair={lastPhysicalLowerColliderName} " +
            $"incoming={incoming:F4}m/s " +
            $"vN={relativeNormalVelocity:F4}m/s " +
            $"hN={rideActualHeight:F4}m " +
            $"wave={waveCycleIndex + 1}",
            this);
    }


    private void ResolvePendingUpperImpactEnergyLoss()
    {
        if (!pendingUpperImpactEnergyMeasurement ||
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

        float relativeNormalVelocity =
            Vector3.Dot(
                ballVisualEqualizer.velocity,
                normal) -
            supportNormalVelocity;

        // Do not sample while PhysX is still constraining the body at Upper.
        // Wait until the ball has clearly left Upper and is descending.
        float releaseHeight =
            currentUpperNormalSpanMeters *
            Mathf.Clamp01(
                impactMeasurementReleaseHeight01);

        if (currentUpperNormalSpanMeters > Epsilon &&
            (rideActualHeight > releaseHeight ||
             relativeNormalVelocity >= 0f))
        {
            return;
        }

        pendingUpperImpactEnergyMeasurement = false;

        float outgoing =
            Mathf.Abs(
                relativeNormalVelocity);

        lastUpperOutgoingNormalSpeed = outgoing;

        float incoming =
            Mathf.Max(
                Epsilon,
                pendingUpperIncomingNormalSpeed);

        float retention =
            Mathf.Clamp01(
                (outgoing * outgoing) /
                (incoming * incoming));

        retention =
            Mathf.Max(
                Mathf.Clamp01(
                    minimumImpactEnergyRetention01),
                retention);

        lastImpactEnergyRetention01 = retention;

        if (applyMeasuredImpactEnergyLoss &&
            !enableLogicalPhysicalHandoff)
        {
            float currentRatio =
                negativeEnvelope.CanonicalDampingEnergyRatio;

            negativeEnvelope.SetCanonicalEnergyRatio(
                currentRatio * retention);
        }

        pendingUpperIncomingNormalSpeed = 0f;
    }


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
        physicalLowerEnergyResolveFrameCount =
            pendingPhysicalLowerEnergyFrames;

        if (outgoing > pendingPhysicalLowerBestOutgoingNormalSpeed)
        {
            pendingPhysicalLowerBestOutgoingNormalSpeed = outgoing;
            physicalLowerBestObservedOutgoingNormalSpeed = outgoing;
        }

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

        physicalLowerEnergyResolveReason =
            resolveReason;

        if (applyPhysicalLowerImpactEnergyLoss)
        {
            negativeEnvelope.SubmitPhysicalLowerImpactEnergyRetention(
                retention,
                $"DescendingAfterUpperStairPhysX/{resolveReason}");
        }

        Debug.Log(
            $"[EQUALIZER PHYSICAL LOWER ENERGY] " +
            $"incoming={incoming:F4}m/s " +
            $"outgoing={resolvedOutgoing:F4}m/s " +
            $"retention={retention:F4} " +
            $"frames={pendingPhysicalLowerEnergyFrames} " +
            $"reason={resolveReason} " +
            $"canonical={negativeEnvelope.CanonicalDampingEnergyRatio:F4}",
            this);

        pendingPhysicalLowerIncomingNormalSpeed = 0f;
        pendingPhysicalLowerEnergyFrames = 0;
        pendingPhysicalLowerBestOutgoingNormalSpeed = 0f;
    }

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
// Observer
// ================================================================

    private void UpdateObserver()
    {
        if (!ballVisualEqualizer)
            return;

        if (ballVisual)
        {
            positionErrorToBallVisual =
                Vector3.Distance(
                    ballVisualEqualizer.position,
                    ballVisual.position);

            velocityErrorToBallVisual =
                Vector3.Distance(
                    ballVisualEqualizer.velocity,
                    ballVisual.velocity);
        }

        currentKineticEnergy =
            0.5f *
            EqualizerMass *
            ballVisualEqualizer.velocity.sqrMagnitude;

        Vector3 subjectPosition =
            ReadSubjectPositionVisual();

        subjectDistance =
            Vector3.Distance(
                ballVisualEqualizer.position,
                subjectPosition);

        if (oscillationFrame.valid)
        {
            subjectTransportGap =
                Vector3.ProjectOnPlane(
                    subjectPosition -
                    ballVisualEqualizer.position,
                    oscillationFrame.normal).magnitude;
        }
        else
        {
            subjectTransportGap =
                subjectDistance;
        }
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

        EnterSynchronizedState(
            "NextIncidentReacquired");
    }


    public void BeginEmergencyVisualRecovery(float suggestedDuration = 0f)
    {
        ResolveReferences();

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
            EnterSynchronizedState(
                "EmergencyAlreadyNear");
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

        Debug.Log(
            $"[EQUALIZER EMERGENCY BEGIN] " +
            $"time={Time.fixedTime:F4} " +
            $"distance={emergencyVisualRecoveryInitialDistance:F4} " +
            $"duration={emergencyVisualRecoveryDuration:F4}s",
            this);
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

        EnterSynchronizedState(
            "EmergencyVisualRecoveryComplete");
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
        EnterSynchronizedState(
            "ResumeRequested");
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

            Debug.Log(
                $"[EQUALIZER POST TURN BRIDGE BEGIN] " +
                $"mode=Synchronized duration={Mathf.Max(0f, suggestedDuration):F4}s",
                this);
            return;
        }

        // 旋回入力がActive Equalizer中に来た例外ケース。
        // hard snapではなく、既存の相対HermiteでBallVisualへ回収する。
        BeginEmergencyVisualRecovery(
            Mathf.Max(0.05f, suggestedDuration));

        Debug.Log(
            $"[EQUALIZER POST TURN BRIDGE BEGIN] " +
            $"mode=RelativeHermite duration={Mathf.Max(0f, suggestedDuration):F4}s",
            this);
    }


    public void EndPostTurnHermiteBridge()
    {
        postTurnHermiteBridgeActive = false;

        // synchronized=trueなら何もしない。
        // RelativeHermiteがちょうど完了していれば既にSynchronizedへ戻っている。
        // まだRecovery中なら、その連続軌道を壊さず完了まで任せる。
        if (synchronized)
            CopyBallVisualPose();

        Debug.Log(
            $"[EQUALIZER POST TURN BRIDGE END] " +
            $"synchronized={synchronized} " +
            $"recoveryActive={emergencyVisualRecoveryActive}",
            this);
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
    }

}

