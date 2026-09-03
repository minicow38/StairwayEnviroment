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
///     - Rigidbody Spring/Damper around the middle of 4R-Hn
///     - real PhysX Upper collision
///     - Virtual Lower is an authority handoff, NOT a stop point
///     - above Virtual Lower: jerk-limited logical normal acceleration
///     - below Virtual Lower: no logical normal force; StairWay/PhysX owns dissipation
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
/// 
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

    [Header("References")]
    [SerializeField] private Rigidbody ballVisual;
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
    "ThreeWavesPerStair: Release->TerminalのSpline区間へ空間的に3波を割り当てる。\n" +
    "速度が変化すると時間周期Tは自動的に変化します。")]
    [SerializeField]
    private WaveTimingMode waveTimingMode = WaveTimingMode.ThreeWavesPerStair;

    [Tooltip("ThreeWavesPerStair時の波数。24m/s試験基準は6。")]
    [Range(1, 8)]
    [SerializeField]
    private int spatialWavesPerStair = 6;

    [Tooltip(
    "4R-Hn Envelopeの何割を実Carrier波の希望Lower->Upper高さに使うか。\n" +
    "実際は加速度Budgetにより自動的に小さくなることがあります。")]
    [Range(0.05f, 0.8f)]
    [SerializeField]
    private float spatialCarrierHeightFractionOfEnvelope = 0.22f;

    // ================================================================
    // EXPERIMENT: BallVisual own-motion -> Equalizer carrier bias
    // ================================================================

    [Header("Spatial Wave BallVisual Momentum Bias - EXPERIMENT")]

    [Tooltip(
    "ONのとき、BallVisual自身がSubjectに対してStable-N方向へ既に大きく動いているほど、\n" +
    "Equalizerが追加する3波Carrier高さを弱めます。Rejoin/Subject/Transport速度は変更しません。")]
    [SerializeField]
    private bool spatialMomentumBiasEnabled = true;

    [Tooltip("Stable-N相対速度がこの値[m/s]を超えたらCarrier抑制を開始します。")]
    [Min(0f)]
    [SerializeField]
    private float spatialMomentumBiasStartNormalSpeed = 0.75f;

    [Tooltip("Stable-N相対速度がこの値[m/s]以上なら最大抑制になります。")]
    [Min(0.01f)]
    [SerializeField]
    private float spatialMomentumBiasFullNormalSpeed = 4.0f;

    [Tooltip("最大抑制時に残すCarrier高さの割合。反応確認用に0.35を初期値にしています。")]
    [Range(0.05f, 1f)]
    [SerializeField]
    private float spatialMomentumMinimumCarrierGain = 0.35f;

    [Tooltip(
    "Spatial Waveが使ってよいStable-N最大加速度[m/s^2]。\n" +
    "波数3を高速で見せるためNaturalモードより大きくできます。")]
    [Min(50f)]
    [SerializeField]
    private float spatialWaveAccelerationBudget = 1800f;

    [Tooltip("Spatial WaveのStable-N加速度Jerk上限[m/s^3]。")]
    [Min(100f)]
    [SerializeField]
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
    [SerializeField] private float spatialReferenceTangentSpeed = 16f;

    [Tooltip("速度正規化倍率の上限。16->24m/sなら1.5。")]
    [Range(1f, 2f)]
    [SerializeField] private float maximumSpatialSpeedRatio = 1.5f;

    [Tooltip(
    "Jerkは理論上speedRatio^3ですが、最初から3.375倍まで許さず安全上限を掛けます。")]
    [Range(1f, 4f)]
    [SerializeField] private float maximumSpatialJerkScale = 2.5f;

    [Tooltip(
    "Spatial進捗加速度dp/dtの微分項をFeedForwardへ含めます。\n" +
    "H''(p)*pDot^2 + H'(p)*pDDot の完全な空間FeedForwardになります。")]
    [SerializeField] private bool includeSpatialProgressAccelerationFeedForward = true;

    [Tooltip(
    "pDDot[1/s^2]の数値ノイズ上限。Spline射影の小さな揺れを巨大なNormal加速度へ変換しないためのGuardです。")]
    [Min(1f)]
    [SerializeField] private float maximumSpatialDomainAccelerationRate = 80f;

    [Tooltip(
    "3波モードでSubjectのTerminal到着時刻へ間に合わせるために許すmaxGroundSpeed超過率。\n" +
    "0.75なら最大1.75倍。SlopeStickCoreへは書き込みません。")]
    [Range(0f, 2f)]
    [SerializeField]
    private float spatialCatchUpSpeedHeadroom01 = 0.75f;

    [Tooltip("3波モードのCatch-up最大加速度[m/s^2]。")]
    [Min(1f)]
    [SerializeField]
    private float spatialMaximumCatchUpAcceleration = 180f;

    [Tooltip("3波モードでGoal速度自体を追従させる最大加速度[m/s^2]。")]
    [Min(1f)]
    [SerializeField]
    private float spatialGoalVelocityAcceleration = 160f;

    // ================================================================
    // Strong logical launch
    // ================================================================

    [Header("Strong Logical Launch")]

    [Tooltip(
    "ON: Release直後のStable-N速度を論理的に少し強め、Upperへ早く到達させます。\n" +
    "異常に大きいsource Normal速度には適用せず、既存Energy異常を増幅しません。")]
    [SerializeField] private bool enableStrongInitialLogicalLaunch = true;

    [Tooltip("基準速度帯の通常Releaseへ掛けるStable-N速度倍率。16m/s帯では1.20。")]
    [Range(1f, 1.8f)]
    [SerializeField] private float initialLogicalLaunchNormalSpeedMultiplier = 1.20f;

    [Tooltip("24m/s帯で許すStrong Launch最大倍率。Wave高さを変えずUpper到達を前倒しします。")]
    [Range(1f, 1.8f)]
    [SerializeField] private float maximumSpeedAwareLaunchNormalSpeedMultiplier = 1.40f;

    [Tooltip("Strong Launchを速度対応させる開始接線速度[m/s]。")]
    [Min(1f)]
    [SerializeField] private float launchBoostReferenceTangentSpeed = 16f;

    [Tooltip("Strong Launch最大倍率へ到達する接線速度[m/s]。")]
    [Min(1f)]
    [SerializeField] private float launchBoostFullTangentSpeed = 24f;

    [Tooltip(
    "このsource Normal速度[m/s]を超えるReleaseにはStrong Launch倍率を掛けません。\n" +
    "既存の高Energy外れ値をさらに増幅しないためのGuardです。")]
    [Min(0.5f)]
    [SerializeField] private float maximumSourceNormalSpeedForLaunchBoost = 8.0f;

    [Tooltip("Strong Launch後のStable-N速度の安全上限[m/s]。")]
    [Min(0.5f)]
    [SerializeField] private float maximumBoostedLogicalLaunchNormalSpeed = 8.0f;

    // ================================================================
    // Turn Guide handoff coordination
    // ================================================================

    [Header("Turn Guide Handoff Coordination")]

    [Tooltip(
    "ON: SlopeStickCoreが新しいTurn Guideを待っている間、またはVisual Frameが旋回中は、\n" +
    "Equalizer側の論理Spring/Damper・Transport Catch-upを一時停止します。\n" +
    "Rigidbody速度そのものは0にせず、PhysXの慣性を保持します。")]
    [SerializeField] private bool suspendLogicalEqualizerDuringTurnHandoff = true;

    [Tooltip(
    "Turn終了時にVirtual Lowerのsupport/projection履歴を捨て、\n" +
    "旋回前Visual座標との差分を物理support速度として誤解しないようにします。")]
    [SerializeField] private bool resetRideSupportAfterTurnHandoff = true;

    [Tooltip("Turn handoffの状態遷移をログ出力します。")]
    [SerializeField] private bool logTurnHandoffCoordination = true;

    // ================================================================
    // Floating Ride Spring / Damper
    // ================================================================

    [Header("Floating Ride - Rigidbody Spring/Damper")]

    [Tooltip(
    "WebのrideSpringStrength相当。Stable-N位置誤差[m]をAccelerationへ変換します。\n" +
    "ForceMode.Accelerationなので mass 非依存の s^-2 相当です。")]
    [Min(1f)]
    [SerializeField] private float rideSpringStrength = 420f;

    [Tooltip(
    "WebのrideSpringDamper相当。Stable-N相対速度[m/s]へ掛ける減衰係数[s^-1]。")]
    [Min(0f)]
    [SerializeField] private float rideSpringDamper = 18f;

    [Tooltip(
    "Stable-Nに掛かるUnity重力を何割相殺するか。\n" +
    "1でFloating Controllerと同様に重力を相殺してSpringが平衡点を所有します。")]
    [Range(0f, 1.5f)]
    [SerializeField] private float gravityCompensation = 1f;

    [Tooltip("Stable-N Spring/Damperの最大加速度[m/s^2]。")]
    [Min(1f)]
    [SerializeField] private float maximumRideAcceleration = 450f;

    [Tooltip("Stable-N加速度の最大変化率[m/s^3]。")]
    [Min(1f)]
    [SerializeField] private float maximumRideJerk = 3000f;

    [Tooltip(
    "4R-Hn内のSpring平衡点。0=Lower、0.5=中央、1=Upper。\n" +
    "通常は0.5。上下へ自然に振幅させるため中央を使います。")]
    [Range(0.05f, 0.95f)]
    [SerializeField] private float rideEquilibrium01 = 0.5f;

    // ================================================================
    // Logical -> Physical authority handoff around Virtual Lower
    // ================================================================

    [Header("Virtual Lower -> Physical Stair Authority Handoff")]

    [Tooltip(
    "ON: Virtual Lowerを停止点にせずAuthority切替点として使います。\n" +
    "Upper後の下降中だけLower直前で減衰を弱め、Lower通過後はNormal制御を切ってPhysX/StairWayへ任せます。")]
    [SerializeField] private bool enableLogicalPhysicalHandoff = true;

    [Tooltip(
    "Virtual Lowerの何R上を下降終盤Boundary Layerとして扱うか。\n" +
    "Upper経験後かつ下降中だけ有効で、この帯域ではEnvelope側の減衰比率を弱めます。")]
    [Range(0.10f, 3f)]
    [SerializeField] private float authorityHandoffBandR = 1.0f;

    [Tooltip(
    "Virtual Lowerを跨いだ後、Physical authorityへ渡す際の最大Jerk[m/s^3]。\n" +
    "Boundary Layer内ではLogical forceを維持し、Lowerを越えて初めて0へ渡します。")]
    [Min(1f)]
    [SerializeField] private float authorityHandoffMaxJerk = 900f;

    [Tooltip(
    "実StairWay衝突後、上向き相対Normal速度がこの値を超えたらPhysical Lower impactの\n" +
    "Energy retentionを確定します。[m/s]")]
    [Min(0f)]
    [SerializeField] private float physicalLowerMeasurementOutgoingSpeed = 0.05f;

    [Tooltip(
    "Physical Lowerとして受理する最低下降相対Normal速度[m/s]。\n" +
    "Accepted Upper後かつ vN < -この値 のStairWay接触だけをPhysical Lowerにします。")]
    [Min(0f)]
    [SerializeField] private float physicalLowerMinimumDescendingSpeed = 0.25f;

    [Tooltip(
    "Virtual Lower以下では残留Logical acceleration stateを0へリセットします。\n" +
    "Lower直前までは減衰を弱めたLogical forceを維持し、境界を越えて初めてPhysXへ完全移譲します。")]
    [SerializeField] private bool hardReleaseNormalForceBelowVirtualLower = true;

    [Header("Virtual Lower Support Frame Guard")]

    [Tooltip(
    "Virtual Lower射影中心が1 FixedUpdateでこのR数より大きく飛んだ場合、\n" +
    "Visual Frame回転/射影切替として扱い、その差分を速度へ変換しません。")]
    [Min(0.25f)]
    [SerializeField] private float supportKinematicsMaximumCenterJumpR = 2.5f;

    [Tooltip(
    "Virtual Lower中心差分から見た見かけ速度の上限[m/s]。\n" +
    "超過時はSupport速度をMapped Subject速度へ戻してFrame Sampleをリセットします。")]
    [Min(1f)]
    [SerializeField] private float supportKinematicsMaximumMeasuredSpeed = 80f;

    [Tooltip(
    "Mapped Subject速度から求めるSupport加速度の上限[m/s^2]。\n" +
    "Visual Frame回転を巨大なNormal feed-forwardへ変換しないためのGuardです。")]
    [Min(1f)]
    [SerializeField] private float supportKinematicsMaximumMeasuredAcceleration = 600f;

    // ================================================================
    // Goal velocity / catch-up
    // ================================================================

    [Header("Transport - Goal Velocity / Catch-up")]

    [Tooltip(
    "WebのgoalVelへMoveTowardsする加速度。Goal自体を急変させないための値[m/s^2]。")]
    [Min(1f)]
    [SerializeField] private float goalVelocityAcceleration = 80f;

    [Tooltip(
    "Subjectとの進行方向位置遅れ[m]を追加Goal速度へ戻す時間[s]。\n" +
    "階段/Upper衝突でタイムロスすると lag/time がGoal速度へ加算されます。")]
    [Min(0.03f)]
    [SerializeField] private float catchUpPositionTime = 0.25f;

    [Tooltip(
    "neededAccel=(goalVel-rb.velocity)/FixedDeltaTime の最大値[m/s^2]。")]
    [Min(1f)]
    [SerializeField] private float maximumCatchUpAcceleration = 90f;

    [Tooltip("Transport加速度の最大変化率[m/s^3]。")]
    [Min(1f)]
    [SerializeField] private float maximumTransportJerk = 1400f;

    [Tooltip(
    "maxGroundSpeedを超えてCatch-upする時の追加許容量。\n" +
    "例0.35なら一時的に1.35倍まで許可。SlopeStickCoreへは書き込みません。")]
    [Range(0f, 1f)]
    [SerializeField] private float catchUpSpeedHeadroom01 = 0.35f;

    [Tooltip(
    "Tangent以外のTransport面ズレを戻す弱いSpring[s^-2]。Stable-Nには作用しません。")]
    [Min(0f)]
    [SerializeField] private float lateralSpringStrength = 12f;

    [Tooltip("横方向相対速度のDamper[s^-1]。")]
    [Min(0f)]
    [SerializeField] private float lateralDamper = 5f;

    // ================================================================
    // Upper impact observation
    // ================================================================

    [Header("Upper Impact Observation")]

    [Tooltip(
    "Upper->Upper実測周期として採用する最小時間[s]。\n" +
    "0ならFixedDeltaTime*2を使用。")]
    [Min(0f)]
    [SerializeField] private float minimumObservedCycleSeconds = 0f;

    [Tooltip(
    "Upper衝突前後のStable-N速度から求めるEnergy retentionをEnvelopeへ反映するか。")]
    [SerializeField] private bool applyMeasuredImpactEnergyLoss = true;

    [Tooltip(
    "衝突後速度がSolver/接触ノイズで極端な場合のEnergy retention下限。")]
    [Range(0f, 1f)]
    [SerializeField] private float minimumImpactEnergyRetention01 = 0.05f;

    [Tooltip(
    "Upperを一度受理した後、4R-Hnのこの高さ率より下へ戻るまで次Upperを受理しません。\n" +
    "Mesh swap / 接触継続による1 FixedUpdate毎の誤カウントを防ぎます。")]
    [Range(0.05f, 0.75f)]
    [SerializeField] private float upperPeakRearmHeight01 = 0.35f;

    [Tooltip(
    "Upper impact energyは接触直後ではなく、この高さ率より下へ離れて下降した時に測定します。")]
    [Range(0.4f, 0.95f)]
    [SerializeField] private float impactMeasurementReleaseHeight01 = 0.85f;

    [Header("Physical Lower Impact Observation")]

    [Tooltip(
    "Authority Handoff中はUpper衝突をEnvelope Energyへ二重計上せず、\n" +
    "実StairWay Lower衝突の前後速度から次波Energy retentionを測ります。")]
    [SerializeField] private bool applyPhysicalLowerImpactEnergyLoss = true;

    [Tooltip("Stair衝突Energy retentionの測定下限。Solverノイズによる0化を防ぎます。")]
    [Range(0f, 1f)]
    [SerializeField] private float minimumPhysicalLowerEnergyRetention01 = 0.05f;

    [Tooltip(
    "Physical Lower後、OnCollisionExitを待たずに上向きStable-N速度を追跡する最大FixedUpdate数。\n" +
    "24m/sでは接触時間が短いため、既定6フレーム以内でEnergyを確定します。")]
    [Range(2, 12)]
    [SerializeField] private int physicalLowerEnergyResolveMaxFixedSteps = 6;

    [Header("Terminal Spatial Transport Assist")]

    [Tooltip(
    "Spatial Wave終盤だけTransport平面のCatch-upを増やします。Normal波形/Lower減衰には触れません。")]
    [SerializeField] private bool enableTerminalSpatialTransportAssist = true;

    [Tooltip("Spatial domainのこの進捗からTerminal transport assistを開始します。")]
    [Range(0.5f, 0.95f)]
    [SerializeField] private float terminalSpatialTransportAssistStartProgress01 = 0.80f;

    [Tooltip("Terminal付近でCatch-up acceleration/Jerkへ掛ける最大倍率。")]
    [Range(1f, 2f)]
    [SerializeField] private float terminalSpatialTransportAssistMultiplier = 1.50f;

    // ================================================================
    // Runtime diagnostics
    // ================================================================

    [Header("Runtime - Read Only")]
    [SerializeField] private bool synchronized = true;
    [SerializeField] private EqualizerPhase phase = EqualizerPhase.Synchronized;

    [SerializeField] private int waveCycleIndex;

    [SerializeField] private float current4RHnMeters;
    [SerializeField] private float current4RHnR;
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
    [SerializeField] private float spatialRawDomainProgress01;

    [Tooltip("Presentation用の単調増加進捗。ThreeWavesPerStair中は後退しません。")]
    [SerializeField] private float spatialDomainProgress01;

    [Tooltip("単調増加進捗の実増加率[1/s]。後退保持中は0です。")]
    [SerializeField] private float spatialDomainAdvanceRate01PerSecond;

    [Tooltip("単調進捗速度の時間微分[1/s^2]。H'(p)*pDDot feed-forwardに使います。")]
    [SerializeField] private float spatialDomainAccelerationRate01PerSecond2;

    [Tooltip("現在接線速度 / spatialReferenceTangentSpeed。")]
    [SerializeField] private float spatialSpeedRatio = 1f;
    [SerializeField] private float spatialAccelerationBudgetScale = 1f;
    [SerializeField] private float spatialJerkBudgetScale = 1f;
    [SerializeField] private float spatialFeedForwardVelocity;
    [SerializeField] private float spatialFeedForwardAcceleration;
    [SerializeField, Range(0f, 1f)] private float terminalSpatialTransportAssist01;

    [Tooltip("Raw進捗が過去最大より後ろにあり、Monotonic Hold中ならTRUE。")]
    [SerializeField] private bool spatialMonotonicHoldActive;

    [Tooltip("Monotonic Holdへ入った瞬間にEnvelope Trigger Pulseを要求した回数。")]
    [SerializeField] private int spatialMonotonicTriggerPulseCount;

    [SerializeField] private float spatialWavePhase01;
    [SerializeField] private float spatialReferencePeriodSeconds;
    [SerializeField] private float spatialCarrierHeightMeters;
    [SerializeField] private float spatialCarrierFeasibility01 = 1f;

    [Header("BallVisual Momentum Bias Runtime - Read Only")]
    [SerializeField] private float spatialBallVisualRelativeNormalSpeed;
    [SerializeField] private float spatialMomentumBias01;
    [SerializeField] private float spatialMomentumCarrierGain = 1f;
    [SerializeField] private float spatialCarrierHeightBeforeMomentumBias;
    [SerializeField] private float spatialSubjectTimeToGo;
    [SerializeField] private float spatialRequiredArrivalSpeed;
    [SerializeField] private float spatialArrivalFeasibility01 = 1f;
    [SerializeField] private bool upperPeakArmed = true;

    [Header("Logical / Physical Authority Runtime - Read Only")]
    [SerializeField] private NormalAuthorityZone normalAuthorityZone = NormalAuthorityZone.LogicalEnvelope;
    [SerializeField, Range(0f, 1f)] private float normalLogicalAuthority01 = 1f;
    [SerializeField] private float normalTargetAccelerationBeforeAuthority;
    [SerializeField] private float normalTargetAccelerationAfterAuthority;
    [SerializeField] private float normalActiveJerkLimit;
    [SerializeField] private float authorityHandoffBandMeters;
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

    [Header("Strong Launch / Lower Decay Runtime - Read Only")]
    [SerializeField] private float sourceCanonicalNormalSpeed;
    [SerializeField] private float logicalLaunchNormalSpeed;
    [SerializeField] private float logicalLaunchEnergyJoule;
    [SerializeField] private float logicalLaunchAppliedMultiplier = 1f;
    [SerializeField] private float logicalLaunchReferenceTangentSpeed;
    [SerializeField] private bool descendingLowerDecayActive;
    [SerializeField, Range(0f, 1f)] private float descendingLowerBoundaryNear01;
    [SerializeField, Range(0f, 1f)] private float descendingLowerHeightNear01;
    [SerializeField, Range(0f, 1f)] private float descendingLowerTimeNear01;
    [SerializeField] private float descendingLowerTimeToBoundarySeconds;
    [SerializeField, Range(0f, 1f)] private float descendingLowerDamperRatio01 = 1f;
    [SerializeField, Range(0f, 1f)] private float descendingLowerRestoringBrakeRatio01 = 1f;
    [SerializeField] private float effectiveRideSpringDamper;
    [SerializeField] private bool previousDescendingLowerDecayActive;

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

    private bool turnHandoffCoordinationActive;
    private bool previousTurnHandoffCoordinationActive;

    // ================================================================
    // Public compatibility
    // ================================================================

    public Rigidbody Body => ballVisualEqualizer;

    public int OscillationCycleIndex => waveCycleIndex;

    public float PlannedMaxGroundSpeedForCycle =>
    negativeEnvelope
    ? negativeEnvelope.PlannedMaxGroundSpeedForCycle
    : 0f;

    public bool IsSynchronized => synchronized;

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
        "[EQUALIZER BUILD] Spatial24-TurnSafeCoordination-20260902-E",
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

        UpdateTurnHandoffCoordination();

        if (synchronized)
        {
            CopyBallVisualPose();
            UpdateObserver();
            return;
        }

        // Turn handoff中は、旋回前/途中のSpline frameを使った論理力を加えない。
        // Rigidbody velocityは触らず、そのまま慣性・PhysXへ渡す。
        if (turnHandoffCoordinationActive &&
        suspendLogicalEqualizerDuringTurnHandoff)
        {
            ResolvePendingUpperImpactEnergyLoss();
            ResolvePendingPhysicalLowerImpactEnergyLoss();

            rideAccelerationState = Vector3.zero;
            transportAccelerationState = Vector3.zero;
            goalPlanarVelocityState = Vector3.zero;

            if (resetRideSupportAfterTurnHandoff)
            ResetRideSupportKinematics();

            UpdateObserver();
            return;
        }

        UpdateWaveTimingAuthority();

        // Upper impact remains a diagnostic in Hybrid Authority mode.
        ResolvePendingUpperImpactEnergyLoss();

        // Real StairWay impact is the physical dissipation authority.
        ResolvePendingPhysicalLowerImpactEnergyLoss();

        UpdateFloatingRideSpring();
        ApplyGoalVelocityCatchUp();
        UpdateObserver();
    }

    private void UpdateTurnHandoffCoordination()
    {
        bool coreWaiting =
        slopeCore &&
        slopeCore.IsWaitingForTurnGuide;

        bool visualTurning =
        correspondSubject &&
        correspondSubject.IsVisualFrameTurning;

        turnHandoffCoordinationActive =
        coreWaiting || visualTurning;

        if (turnHandoffCoordinationActive ==
        previousTurnHandoffCoordinationActive)
        {
            return;
        }

        if (turnHandoffCoordinationActive)
        {
            rideAccelerationState = Vector3.zero;
            transportAccelerationState = Vector3.zero;
            goalPlanarVelocityState = Vector3.zero;

            if (resetRideSupportAfterTurnHandoff)
            ResetRideSupportKinematics();

            negativeEnvelope?.NotifyTurnHandoffStarted();

            if (logTurnHandoffCoordination)
            {
                Debug.Log(
                $"[EQUALIZER TURN HANDOFF START] " +
                $"coreWaiting={coreWaiting} " +
                $"visualTurning={visualTurning} " +
                $"synchronized={synchronized}",
                this);
            }
        }
        else
        {
            if (resetRideSupportAfterTurnHandoff)
            ResetRideSupportKinematics();

            negativeEnvelope?.NotifyTurnHandoffCompleted();

            if (logTurnHandoffCoordination)
            {
                Debug.Log(
                "[EQUALIZER TURN HANDOFF END] support/projection history reset",
                this);
            }
        }

        previousTurnHandoffCoordinationActive =
        turnHandoffCoordinationActive;
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
        return correspondSubject.Mappedvelocity;

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
        authorityHandoffBandMeters = 0f;
        sourceCanonicalNormalSpeed = 0f;
        logicalLaunchNormalSpeed = 0f;
        logicalLaunchEnergyJoule = 0f;
        descendingLowerDecayActive = false;
        previousDescendingLowerDecayActive = false;
        descendingLowerBoundaryNear01 = 0f;
        descendingLowerDamperRatio01 = 1f;
        descendingLowerRestoringBrakeRatio01 = 1f;
        effectiveRideSpringDamper = rideSpringDamper;

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
        spatialCarrierHeightMeters = 0f;
        spatialCarrierFeasibility01 = 1f;
        spatialBallVisualRelativeNormalSpeed = 0f;
        spatialMomentumBias01 = 0f;
        spatialMomentumCarrierGain = 1f;
        spatialCarrierHeightBeforeMomentumBias = 0f;
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
    // Release
    // ================================================================

    public bool ReleaseToEnvelopeSimulation(
    Vector3 equalizerLaunchVelocity,
    float sourceEnergyJoule,
    float envelopeEntryHeight)
    {
        Vector3 inferredAxis =
        equalizerLaunchVelocity -
        ReadSubjectVelocityVisual();

        if (inferredAxis.sqrMagnitude <= Epsilon)
        inferredAxis = Vector3.up;

        return ReleaseToEnvelopeSimulation(
        equalizerLaunchVelocity,
        sourceEnergyJoule,
        envelopeEntryHeight,
        inferredAxis);
    }

    public bool ReleaseToEnvelopeSimulation(
    Vector3 equalizerLaunchVelocity,
    float sourceEnergyJoule,
    float envelopeEntryHeight,
    Vector3 sourceEnergyAxisVisual)
    {
        ResolveReferences();

        if (!ballVisual ||
        !ballVisualEqualizer ||
        !negativeEnvelope)
        {
            Debug.LogError(
            "[EQUALIZER] Release references are missing.",
            this);
            return false;
        }

        if (!synchronized)
        ReacquireForNextIncident();

        float safeEnergy =
        Mathf.Max(
        0f,
        sourceEnergyJoule);

        float safeHeight =
        Mathf.Max(
        0f,
        envelopeEntryHeight);

        if (safeEnergy <= Epsilon ||
        safeHeight <= Epsilon)
        {
            Debug.LogWarning(
            "[EQUALIZER] Release energy / height is invalid.",
            this);
            return false;
        }

        Vector3 tangent = Vector3.zero;
        Vector3 normal = sourceEnergyAxisVisual;

        if (negativeEnvelope.TryGetReleaseSurfaceFrameVisual(
        out Vector3 releaseTangent,
        out Vector3 releaseNormal))
        {
            tangent = releaseTangent;
            normal = releaseNormal;
        }

        if (normal.sqrMagnitude <= Epsilon)
        normal = Vector3.up;

        normal.Normalize();

        if (tangent.sqrMagnitude <= Epsilon)
        {
            tangent =
            Vector3.ProjectOnPlane(
            ReadSubjectVelocityVisual(),
            normal);
        }

        if (tangent.sqrMagnitude <= Epsilon)
        tangent = Vector3.forward;

        tangent =
        Vector3.ProjectOnPlane(
        tangent,
        normal).normalized;

        Vector3 lateral =
        Vector3.Cross(
        normal,
        tangent).normalized;

        Vector3 subjectVelocity =
        ReadSubjectVelocityVisual();

        Vector3 planarTransportVelocity =
        Vector3.ProjectOnPlane(
        subjectVelocity,
        normal);

        float canonicalNormalSpeed =
        Mathf.Sqrt(
        Mathf.Max(
        0f,
        2f *
        safeEnergy /
        EqualizerMass));

        sourceCanonicalNormalSpeed = canonicalNormalSpeed;

        float releaseTangentSpeed =
        Mathf.Abs(
        Vector3.Dot(
        planarTransportVelocity,
        tangent));

        logicalLaunchReferenceTangentSpeed =
        releaseTangentSpeed;

        float launchSpeed01 =
        Mathf.InverseLerp(
        Mathf.Max(1f, launchBoostReferenceTangentSpeed),
        Mathf.Max(
        launchBoostReferenceTangentSpeed + 0.01f,
        launchBoostFullTangentSpeed),
        releaseTangentSpeed);

        float speedAwareLaunchMultiplier =
        Mathf.Lerp(
        Mathf.Max(1f, initialLogicalLaunchNormalSpeedMultiplier),
        Mathf.Max(
        initialLogicalLaunchNormalSpeedMultiplier,
        maximumSpeedAwareLaunchNormalSpeedMultiplier),
        Mathf.SmoothStep(0f, 1f, launchSpeed01));

        float logicalNormalSpeed = canonicalNormalSpeed;
        bool launchBoostAllowed =
        enableStrongInitialLogicalLaunch &&
        canonicalNormalSpeed <=
        Mathf.Max(0.5f, maximumSourceNormalSpeedForLaunchBoost);

        logicalLaunchAppliedMultiplier =
        launchBoostAllowed
        ? speedAwareLaunchMultiplier
        : 1f;

        if (launchBoostAllowed)
        {
            logicalNormalSpeed =
            Mathf.Min(
            canonicalNormalSpeed *
            logicalLaunchAppliedMultiplier,
            Mathf.Max(
            canonicalNormalSpeed,
            maximumBoostedLogicalLaunchNormalSpeed));
        }

        logicalLaunchAppliedMultiplier =
        canonicalNormalSpeed > Epsilon
        ? logicalNormalSpeed / canonicalNormalSpeed
        : 1f;

        logicalLaunchNormalSpeed = logicalNormalSpeed;
        logicalLaunchEnergyJoule =
        0.5f *
        EqualizerMass *
        logicalNormalSpeed *
        logicalNormalSpeed;

        Vector3 canonicalLaunchVelocity =
        planarTransportVelocity +
        normal *
        logicalNormalSpeed;

        CopyBallVisualPose();

        Vector3 releasePosition =
        ballVisual.position;

        Quaternion releaseRotation =
        ballVisual.rotation;

        Vector3 releaseAngularVelocity =
        ballVisual.angularVelocity;

        bool envelopeReady =
        negativeEnvelope.ArmFromBallVisualEnergy(
        logicalLaunchEnergyJoule,
        safeHeight,
        canonicalLaunchVelocity,
        normal,
        1f);

        if (!envelopeReady)
        {
            Debug.LogWarning(
            "[EQUALIZER] Envelope arm failed.",
            this);

            EnterSynchronizedState(
            "EnvelopeArmFailed");

            return false;
        }

        if (negativeEnvelope.TryGetLatestOscillationFrameVisual(
        out Vector3 envelopeTangent,
        out Vector3 envelopeNormal))
        {
            if (envelopeNormal.sqrMagnitude > Epsilon)
            normal = envelopeNormal.normalized;

            if (envelopeTangent.sqrMagnitude > Epsilon)
            {
                tangent =
                Vector3.ProjectOnPlane(
                envelopeTangent,
                normal).normalized;
            }

            lateral =
            Vector3.Cross(
            normal,
            tangent).normalized;
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
            velocity = canonicalLaunchVelocity,
            angularVelocity = releaseAngularVelocity,
            sourceEnergy = logicalLaunchEnergyJoule,
            referenceHeight = safeHeight,
            sourceAxis = normal
        };

        synchronized = false;
        phase = EqualizerPhase.ReleaseArmed;

        ballVisualEqualizer.isKinematic = false;
        ballVisualEqualizer.detectCollisions = true;
        ballVisualEqualizer.useGravity = true;

        ballVisualEqualizer.position =
        releasePosition;

        ballVisualEqualizer.rotation =
        releaseRotation;

        ballVisualEqualizer.velocity =
        canonicalLaunchVelocity;

        ballVisualEqualizer.angularVelocity =
        releaseAngularVelocity;

        ballVisualEqualizer.collisionDetectionMode =
        CollisionDetectionMode.ContinuousDynamic;

        ballVisualEqualizer.solverIterations =
        Mathf.Max(
        ballVisualEqualizer.solverIterations,
        12);

        ballVisualEqualizer.solverVelocityIterations =
        Mathf.Max(
        ballVisualEqualizer.solverVelocityIterations,
        4);

        ballVisualEqualizer.WakeUp();

        rideAccelerationState = Vector3.zero;
        transportAccelerationState = Vector3.zero;
        goalPlanarVelocityState =
        planarTransportVelocity;
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
        normalLogicalAuthority01 = 1f;
        normalTargetAccelerationBeforeAuthority = 0f;
        normalTargetAccelerationAfterAuthority = 0f;
        normalActiveJerkLimit = 0f;
        descendingLowerDecayActive = false;
        previousDescendingLowerDecayActive = false;
        descendingLowerBoundaryNear01 = 0f;
        descendingLowerDamperRatio01 = 1f;
        descendingLowerRestoringBrakeRatio01 = 1f;
        effectiveRideSpringDamper = rideSpringDamper;

        UpdateWaveTimingAuthority();

        negativeEnvelope.SetUpperEnvelopeSolidEnabled(
        true,
        "FloatingRigidbodyRelease");

        negativeEnvelope.RefreshEqualizerBoundaryCollisionOwnership();

        phase = EqualizerPhase.FreeFlight;

        if (negativeEnvelope.TryGetCurrentPresentationCenterTravel(
        out current4RHnMeters,
        out current4RHnR))
        {
            // diagnostics updated
        }

        Debug.Log(
        $"[EQUALIZER FLOATING RELEASE] " +
        $"E0={logicalLaunchEnergyJoule:F4}J " +
        $"sourceE={safeEnergy:F4}J " +
        $"H0={safeHeight:F4}m " +
        $"vN0={logicalLaunchNormalSpeed:F4}m/s " +
        $"sourceVN={sourceCanonicalNormalSpeed:F4}m/s " +
        $"launchBoost={logicalLaunchAppliedMultiplier:F3}x " +
        $"launchVT={logicalLaunchReferenceTangentSpeed:F3}m/s " +
        $"span={current4RHnMeters:F4}m " +
        $"spanR={current4RHnR:F3}R " +
        $"mode={waveTimingMode} " +
        $"waves={(waveTimingMode == WaveTimingMode.ThreeWavesPerStair ? spatialWavesPerStair : 0)} " +
        $"masterT=None",
        this);

        return true;
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

        current4RHnMeters = spanMeters;
        current4RHnR = spanR;

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

        // Legacy modes may rearm from Virtual Lower height.
        // Hybrid authority requires REAL StairWay contact; Virtual Lower is only
        // an authority handoff and must never pretend to be a physical impact.
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
            spatialCarrierHeightMeters = 0f;
            spatialCarrierFeasibility01 = 1f;
            spatialBallVisualRelativeNormalSpeed = 0f;
            spatialMomentumBias01 = 0f;
            spatialMomentumCarrierGain = 1f;
            spatialCarrierHeightBeforeMomentumBias = 0f;
        }

        BallVisualNegativeEnvelopeCollider.DescendingLowerDecayProfile lowerDecayProfile;
        descendingLowerDecayActive =
        negativeEnvelope.TryResolveDescendingLowerDecayProfile(
        rideActualHeight,
        rideRelativeNormalVelocity,
        physicalUpperSeenSinceLastLower,
        authorityHandoffBandR,
        out lowerDecayProfile);

        descendingLowerBoundaryNear01 =
        descendingLowerDecayActive
        ? lowerDecayProfile.nearLower01
        : 0f;

        descendingLowerHeightNear01 =
        descendingLowerDecayActive
        ? lowerDecayProfile.heightNearLower01
        : 0f;

        descendingLowerTimeNear01 =
        descendingLowerDecayActive
        ? lowerDecayProfile.timeNearLower01
        : 0f;

        descendingLowerTimeToBoundarySeconds =
        descendingLowerDecayActive
        ? lowerDecayProfile.timeToVirtualLowerSeconds
        : float.PositiveInfinity;

        descendingLowerDamperRatio01 =
        descendingLowerDecayActive
        ? lowerDecayProfile.damperRatio01
        : 1f;

        descendingLowerRestoringBrakeRatio01 =
        descendingLowerDecayActive
        ? lowerDecayProfile.restoringBrakeRatio01
        : 1f;

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

        float normalVelocityError =
        targetNormalVelocity -
        rideRelativeNormalVelocity;

        effectiveRideSpringDamper =
        Mathf.Max(
        0f,
        rideSpringDamper) *
        descendingLowerDamperRatio01;

        damperAcceleration =
        normalVelocityError *
        effectiveRideSpringDamper;

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

        // Upper後の下降終盤だけ、Virtual Lowerへ近付くほど
        // 「下降速度を止める向き(+N)」の論理Accelerationを弱める。
        // 下向きAccelerationは維持するため、Lowerへ向かう勢いを人工的に殺さない。
        if (descendingLowerDecayActive &&
        rideRelativeNormalVelocity < 0f &&
        desiredScalarAcceleration > 0f)
        {
            desiredScalarAcceleration *=
            descendingLowerRestoringBrakeRatio01;
        }

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

        if (physicalOwnsNormal &&
        hardReleaseNormalForceBelowVirtualLower)
        {
            // Boundary LayerではLogical forceを維持して減衰だけを弱める。
            // Virtual Lowerを越えた瞬間に初めてcontroller forceを完全解放し、
            // 既存velocity / gravity / PhysXへそのまま渡す。
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

        if (!physicalOwnsNormal)
        {
            ballVisualEqualizer.AddForce(
            rideAccelerationState,
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
            $"jerk={normalActiveJerkLimit:F1}m/s3",
            this);

            previousNormalAuthorityZone = normalAuthorityZone;
        }

        if (descendingLowerDecayActive != previousDescendingLowerDecayActive)
        {
            Debug.Log(
            $"[EQUALIZER LOWER DECAY] " +
            $"active={descendingLowerDecayActive} " +
            $"hN={rideActualHeight:F4}m " +
            $"vN={rideRelativeNormalVelocity:F4}m/s " +
            $"near={descendingLowerBoundaryNear01:F3} " +
            $"heightNear={descendingLowerHeightNear01:F3} " +
            $"timeNear={descendingLowerTimeNear01:F3} " +
            $"tLower={(float.IsInfinity(descendingLowerTimeToBoundarySeconds) ? -1f : descendingLowerTimeToBoundarySeconds):F4}s " +
            $"damperRatio={descendingLowerDamperRatio01:F3} " +
            $"brakeRatio={descendingLowerRestoringBrakeRatio01:F3}",
            this);

            previousDescendingLowerDecayActive =
            descendingLowerDecayActive;
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
        float radius = ResolveEqualizerWorldRadius();
        authorityHandoffBandMeters =
        Mathf.Max(0.001f, radius * Mathf.Max(0.10f, authorityHandoffBandR));

        // A Visual-frame/projection discontinuity is not a physical crossing.
        // Never reinterpret it as PhysicalFree.
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

        bool descendingAfterUpper =
        physicalUpperSeenSinceLastLower &&
        rideRelativeNormalVelocity < 0f;

        // Release直後/上昇中のVirtual Lower近傍ではLogical authorityを一切抜かない。
        // Strong Launchと従来Envelopeをそのまま使ってUpperまで運ぶ。
        if (!descendingAfterUpper)
        {
            logicalAuthority01 = 1f;
            activeJerkLimit = Mathf.Max(1f, logicalJerkLimit);
            zone = NormalAuthorityZone.LogicalEnvelope;
            return;
        }

        // Upper後の下降でVirtual Lowerを越えた瞬間だけPhysXへ完全移譲する。
        if (signedHeightFromVirtualLower <= 0f)
        {
            logicalAuthority01 = 0f;
            activeJerkLimit = Mathf.Max(1f, authorityHandoffMaxJerk);
            zone = NormalAuthorityZone.PhysicalFree;
            return;
        }

        // Boundary LayerではLogical force自体は維持する。
        // 減衰/復元ブレーキの弱化はDescendingLowerDecayProfileが担当する。
        if (signedHeightFromVirtualLower < authorityHandoffBandMeters)
        {
            logicalAuthority01 = 1f;
            activeJerkLimit = Mathf.Lerp(
            Mathf.Max(1f, authorityHandoffMaxJerk),
            Mathf.Max(1f, logicalJerkLimit),
            Mathf.Clamp01(
            signedHeightFromVirtualLower /
            authorityHandoffBandMeters));
            zone = NormalAuthorityZone.HandoffToPhysics;
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

        // ------------------------------------------------------------
        // BallVisual own-motion bias (EXPERIMENT)
        // ------------------------------------------------------------
        // CarrierはStable-N方向の追加表現なので、BallVisualの全速度ではなく
        // Subjectとの差分のStable-N成分だけを抑制判定に使う。
        Vector3 ballVisualRelativeVelocity =
        ballVisual
        ? ballVisual.velocity - subjectVelocityVisual
        : Vector3.zero;

        spatialBallVisualRelativeNormalSpeed =
        Mathf.Abs(
        Vector3.Dot(
        ballVisualRelativeVelocity,
        normal));

        if (spatialMomentumBiasEnabled && ballVisual)
        {
            float biasStart =
            Mathf.Max(0f, spatialMomentumBiasStartNormalSpeed);

            float biasFull =
            Mathf.Max(
            biasStart + 0.01f,
            spatialMomentumBiasFullNormalSpeed);

            float rawBias01 =
            Mathf.InverseLerp(
            biasStart,
            biasFull,
            spatialBallVisualRelativeNormalSpeed);

            spatialMomentumBias01 =
            Mathf.SmoothStep(0f, 1f, rawBias01);

            spatialMomentumCarrierGain =
            Mathf.Lerp(
            1f,
            Mathf.Clamp(
            spatialMomentumMinimumCarrierGain,
            0.05f,
            1f),
            spatialMomentumBias01);
        }
        else
        {
            spatialMomentumBias01 = 0f;
            spatialMomentumCarrierGain = 1f;
        }

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

        // Original carrier height before the new bias.
        spatialCarrierHeightBeforeMomentumBias =
        Mathf.Min(
        envelopeSpanMeters * 0.95f,
        Mathf.Min(
        desiredCarrierHeight,
        feasibleCarrierHeight));

        spatialCarrierFeasibility01 =
        desiredCarrierHeight > Epsilon
        ? Mathf.Clamp01(
        spatialCarrierHeightBeforeMomentumBias /
        desiredCarrierHeight)
        : 1f;

        // Only the 3-wave presentation height is attenuated.
        // Rejoin / Subject / SlopeStickCore / transport are untouched.
        spatialCarrierHeightMeters =
        spatialCarrierHeightBeforeMomentumBias *
        spatialMomentumCarrierGain;

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

        // Spatial feed-forward:
        //   H(p) = A/2 * (1-cos(2*pi*N*p))
        //   vN   = H'(p) * pDot
        //   aN   = H''(p) * pDot^2 + H'(p) * pDDot
        // Presentation progressはMonotonicなので、Collider反発中に波を逆再生しません。
        float pDot =
        spatialDomainAdvanceRate01PerSecond;

        float pDDot =
        includeSpatialProgressAccelerationFeedForward
        ? spatialDomainAccelerationRate01PerSecond2
        : 0f;

        float dHeightDp =
        spatialCarrierHeightMeters *
        Mathf.PI *
        waveCount *
        Mathf.Sin(theta);

        float d2HeightDp2 =
        2f *
        spatialCarrierHeightMeters *
        Mathf.PI *
        Mathf.PI *
        waveCount *
        waveCount *
        Mathf.Cos(theta);

        targetNormalVelocity =
        dHeightDp *
        pDot;

        targetNormalAcceleration =
        d2HeightDp2 *
        pDot *
        pDot +
        dHeightDp *
        pDDot;

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

        ballVisualEqualizer.AddForceAtPosition(
        transportAccelerationState *
        EqualizerMass,
        ballVisualEqualizer.worldCenterOfMass,
        ForceMode.Force);

        catchUpAccelerationCommand =
        Vector3.Dot(
        transportAccelerationState,
        tangent);
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
        current4RHnMeters *
        Mathf.Clamp01(
        impactMeasurementReleaseHeight01);

        if (current4RHnMeters > Epsilon &&
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

        EnterSynchronizedState(
        "ResumeRequested");
    }

    public void ResumeSynchronization1()
    {
        ResumeSynchronization();
    }

    public void ResumeSynchronization2()
    {
        ResumeSynchronization();
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
