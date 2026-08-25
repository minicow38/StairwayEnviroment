using System.Collections;
using DG.Tweening;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class SlopeStickBall3D1 : MonoBehaviour
{
    [Header("ROLE 1 - PhysicsBall / subject")]
    [Tooltip("本物の物理玉。基本はこの Script が付いている subject を入れます。")]
    [SerializeField] Transform subjectRoot;
    [Tooltip("物理を受ける唯一の Rigidbody。subject 以外の子には Rigidbody を付けないでください。")]
    [SerializeField] Rigidbody physicsBodyRigidbody;
    [Tooltip("true の場合、CaptureNextStep / SlopeStopTarget は EffectGuide と BallVisual だけに使い、PhysicsBall を止めません。")]
    [SerializeField] bool effectGuideOnlyControlsBallVisual = true;
    [Tooltip("旧挙動用。true にすると EffectGuide のブレーキを PhysicsBall にも加えます。通常は false 推奨です。")]
    [SerializeField] bool applyEffectGuideBrakeToPhysicsBody = false;
    [Tooltip("子に Rigidbody が残っている場合に警告します。")]
    [SerializeField] bool warnIfChildHasRigidbody = true;
    [Header("Common References")]
    [SerializeField] Transform cameraTransform, headingTransform;
    [SerializeField, Min(1f)] float minFlickPixels = 10f;
    [SerializeField, Range(0f, 180f)] float maxTurnPerFlick = 90f;
    [SerializeField] bool useSideTurnExperiment = true;
    [SerializeField] bool snapHeadingOnSideTest = true;
    [SerializeField] Vector3 initialHeading = Vector3.right;
    [SerializeField, Min(1.1f)] float flickStrengthBandRatio = 1.8f;
    [SerializeField] float[] flickTurnAngleSteps =
    {
        12.5f, 20f, 30f, 45f
    };
    [SerializeField, Min(.01f)] float headingTurnDuration = .45f;
    [SerializeField, Min(.01f)] float minimumHeadingTurnDuration = .12f;
    [SerializeField] Ease headingTurnEase = Ease.InOutCubic;
    [SerializeField, Range(0f, 180f)] float maxPlayerTurnAngle = 45f;
    [SerializeField] Transform stageRoot, stagePivot;
    [SerializeField] bool stageTurnsOppositeToPlayer = true;
    [SerializeField] bool movePlayerAroundStagePivot = true;
    [Header("ROLE 2 - EffectGuide / BallVisual 指揮者")]
    [Tooltip("見えない指揮者。CaptureNextStep Point から Slope Stop Target までの進行率だけを担当します。")]
    [SerializeField] Transform effectGuideTransform;
    [Tooltip("true の場合、EffectGuide Transform を Capture 区間上に動かします。デバッグ用です。")]
    [SerializeField] bool moveEffectGuideTransform = true;
    [Tooltip("true の場合、BallVisual は subject の中心ではなく EffectGuide のワールド位置を基準に表示されます。")]
    [SerializeField] bool ballVisualFollowsEffectGuideWorld = true;
    [Tooltip("EffectGuide が Slope Stop Target に到達したあと、その場で一度止まる時間です。subject は止まりません。")]
    [SerializeField, Min(0f)] float effectGuideHoldAtTargetSeconds = .18f;
    [Tooltip("Target で止まった BallVisual / EffectGuide を subject へ戻す時間です。")]
    [SerializeField, Min(.01f)] float effectGuideReturnToSubjectSeconds = .35f;
    [Tooltip("true の場合、EffectGuide は Slope Stop Target で止まります。subject はそのまま進みます。")]
    [SerializeField] bool stopEffectGuideAtSlopeStopTarget = true;

    [Header("Stairway Visual Safety - 元機能を残した安全上限")]
    [Tooltip("OFF推奨。ONにすると旧版の大振幅・大回転をそのまま許可します。")]
    [SerializeField] bool useLegacyExtremeCaptureVisuals = false;
    [Tooltip("Capture区間でBallVisualがsubjectから離れてよい最大距離です。")]
    [SerializeField, Min(.01f)] float safeCaptureVisualSeparation = .18f;
    [Tooltip("Capture区間の上下波の安全上限です。")]
    [SerializeField, Min(.01f)] float safeCaptureWaveAmplitude = .14f;
    [Tooltip("EffectGuide復帰中の横カーブ量の安全上限です。")]
    [SerializeField, Min(0f)] float safeReturnCurveAmplitude = .12f;
    [Tooltip("通常・衝撃・WildSpinを含む見た目回転速度の安全上限です。")]
    [SerializeField, Min(0f)] float safeVisualSpinDegreesPerSecond = 1200f;
    [Tooltip("開発者指定の復帰縦回転量の安全上限です。")]
    [SerializeField, Min(0f)] float safeDeveloperReturnRollDegrees = 240f;
    [Tooltip("開発者指定の復帰横回転量の安全上限です。")]
    [SerializeField, Min(0f)] float safeDeveloperReturnSideSpinDegrees = 60f;

    [Header("Slope Stop Reached - 極限点の横回転トリック")]
    [Tooltip("Slope Stop Targetへ到達した瞬間だけ、BallVisualへ横回転を加えます。PhysicsBallには影響しません。")]
    [SerializeField] bool useSlopeStopReachedTrickSpin = true;
    [Tooltip("極限点で法線軸まわりに加える横回転速度です。単位は度/秒です。")]
    [SerializeField, Min(0f)] float slopeStopReachedSideSpinImpulse = 2200f;
    [Tooltip("横回転へ混ぜる進行軸まわりの傾き回転です。0なら純粋な横回転だけになります。")]
    [SerializeField, Min(0f)] float slopeStopReachedBankSpinImpulse = 520f;
    [Tooltip("極限点トリック回転が消えていく速さです。大きいほど短く鋭い回転になります。")]
    [SerializeField, Min(.1f)] float slopeStopReachedTrickSpinDamping = 7.5f;
    [Tooltip("極限点トリック中だけ許可する見た目回転速度の上限です。")]
    [SerializeField, Min(0f)] float slopeStopReachedTrickMaxDegreesPerSecond = 2000f;
    [Tooltip("最後にハンドルを切った方向へ横回転させます。")]
    [SerializeField] bool slopeStopReachedUsesLastHandleDirection = true;
    [Tooltip("上の項目がOFFのときに使う横回転方向です。+1/-1で反転します。")]
    [SerializeField, Range(-1f, 1f)] float slopeStopReachedSpinDirection = 1f;
    [Tooltip("極限点到達時に、それ以前のランダム回転をどれだけ残すかです。0でほぼ消し、1で全て残します。")]
    [SerializeField, Range(0f, 1f)] float slopeStopReachedPreviousSpinRemain = .30f;

    [Header("EffectGuide Return - 曲線復帰 / 縦揺れ禁止")]
    [Tooltip("true の場合、EffectGuide が subject へ戻るときに、ハンドル方向へ横カーブします。縦方向の跳ねは入れません。")]
    [SerializeField] bool useTrickyEffectGuideReturnCurve = true;
    [Tooltip("true の場合、復帰時間に captureNextStepWaveReturnSeconds を使います。")]
    [SerializeField] bool useCaptureNextStepWaveReturnSecondsForGuideReturn = true;
    [Tooltip("戻り中の横曲がり量。Y方向の上下ではなく、水平横方向だけです。")]
    [SerializeField, Min(0f)] float effectGuideReturnLateralCurveAmplitude = .65f;
    [Tooltip("戻り中の横曲がりの波数。1なら一度だけ横へふくらんで戻ります。")]
    [SerializeField, Min(.25f)] float effectGuideReturnLateralCurveFrequency = 1f;
    [Tooltip("戻り終盤で横曲がりを消す強さ。大きいほど早く subject へ吸い寄せます。")]
    [SerializeField, Range(.25f, 4f)] float effectGuideReturnLateralCurveFadePower = 1.25f;
    [Tooltip("true の場合、戻り中は上下波を完全に消します。過剰な縦揺れ防止用です。")]
    [SerializeField] bool suppressVerticalWaveWhileReturningToSubject = true;
    [Tooltip("戻り中だけ、見た目の横カーブに合わせた仮想トルク回転を入れます。PhysicsBall には AddTorque しません。")]
    [SerializeField] bool useVirtualTorqueWhileReturningToSubject = true;
    [Tooltip("戻り中の縦転がり回転の倍率。")]
    [SerializeField, Min(0f)] float returnVerticalRollMultiplier = 1.05f;
    [Tooltip("戻り中の横スピンの倍率。縦回転より小さめ推奨です。")]
    [SerializeField, Min(0f)] float returnHorizontalSideSpinMultiplier = .16f;
    [Header("Developer Return Motion - 戻り時間 / 三角関数 / 回転指定")]
    [Tooltip("true の場合、戻り処理は下の開発者指定パラメーターで制御します。PhysicsBall には力を加えません。")]
    [SerializeField] bool useDeveloperReturnMotion = true;
    [Tooltip("true なら戻り時間は captureNextStepWaveReturnSeconds を使います。false なら developerReturnSeconds を使います。")]
    [SerializeField] bool developerReturnUsesCaptureNextStepWaveReturnSeconds = true;
    [Tooltip("developerReturnUsesCaptureNextStepWaveReturnSeconds が false のときに使う戻り時間です。")]
    [SerializeField, Min(.01f)] float developerReturnSeconds = .24f;
    [Tooltip("true なら戻りの基本補間を SmoothStep にします。false なら等速 Lerp です。")]
    [SerializeField] bool developerReturnUseSmoothStep = true;
    [Tooltip("戻り中に横へふくらむ量。0 なら直線で subject に戻ります。")]
    [SerializeField, Min(0f)] float developerReturnCurveSideAmplitude = .55f;
    [Tooltip("戻り中の縦方向のふくらみ量。過剰な縦揺れを避けるなら 0 推奨です。")]
    [SerializeField, Min(0f)] float developerReturnCurveVerticalAmplitude = 0f;
    [Tooltip("戻り中の三角関数カーブの周期数。1 なら一度ふくらんで戻ります。")]
    [SerializeField, Min(.01f)] float developerReturnCurveCycles = 1f;
    [Tooltip("三角関数カーブの位相です。通常は 0 でよいです。")]
    [SerializeField, Range(-360f, 360f)] float developerReturnCurvePhaseDegrees = 0f;
    [Tooltip("開始・終了でカーブを 0 に戻す包絡線の強さです。大きいほど端で早く静まります。")]
    [SerializeField, Range(.05f, 8f)] float developerReturnCurveEnvelopePower = 1f;
    [Tooltip("true なら最後にハンドルを切った方向へ横カーブします。")]
    [SerializeField] bool developerReturnCurveUsesLastHandleDirection = true;
    [Tooltip("developerReturnCurveUsesLastHandleDirection が false のときの横カーブ方向です。")]
    [SerializeField, Range(-1f, 1f)] float developerReturnCurveDirection = 1f;
    [Tooltip("true の場合、戻り中の回転を下の角度指定で制御します。距離ベース回転は使いません。")]
    [SerializeField] bool useDeveloperReturnRotation = true;
    [Tooltip("戻り中の縦転がり回転量です。単位は度。")]
    [SerializeField, Min(0f)] float developerReturnRollDegrees = 720f;
    [Tooltip("戻り中に少し混ぜる横スピン量です。単位は度。")]
    [SerializeField, Min(0f)] float developerReturnSideSpinDegrees = 160f;
    [Tooltip("戻り中のひねり回転量です。不要なら 0。単位は度。")]
    [SerializeField, Min(0f)] float developerReturnTwistDegrees = 0f;
    [SerializeField, Range(-1f, 1f)] float developerReturnRollDirection = 1f;
    [SerializeField, Range(-1f, 1f)] float developerReturnSideSpinDirection = 1f;
    [SerializeField, Range(-1f, 1f)] float developerReturnTwistDirection = 1f;
    [Tooltip("戻り中の追加回転が開始・終了で 0 へ戻る強さです。")]
    [SerializeField, Range(.05f, 8f)] float developerReturnRotationEnvelopePower = 1f;
    [Header("ROLE 3 - BallVisual / 見た目玉")]
    [Tooltip("見た目だけの玉。Rigidbody / Collider は付けません。")]
    [SerializeField] Transform ballVisualTransform;
    [SerializeField, Min(0f)] float visualVelocityLag = .015f;
    [SerializeField, Min(0f)] float visualCatchUpSharpness = 24f;
    [SerializeField, Min(0f)] float visualImpactMinSpeed = 3f;
    [SerializeField, Min(0f)] float visualImpactPerSpeed = .012f, visualImpactMaxOffset = .16f;
    [SerializeField, Min(.1f)] float visualImpactFrequency = 14f;
    [SerializeField, Min(0f)] float visualImpactDamping = 9f;
    [SerializeField, Range(0f, 1f)] float visualImpactSideAmount = .25f, visualImpactSquash = .12f;
    [SerializeField, Min(0f)] float visualImpactCooldown = .06f;
    [SerializeField] bool useDotweenVisualBounce = true;
    [SerializeField] bool useCaptureNextStepBounce = true;
    [SerializeField, Min(0f)] float captureNextStepBounceMaxOffset = .32f;
    [SerializeField, Min(0f)] float captureNextStepBouncePerSpeed = .030f;
    [SerializeField, Range(0f, 1f)] float captureNextStepBounceSideAmount = .28f;
    [SerializeField, Range(0f, 1f)] float captureNextStepBounceSquash = .18f;
    [SerializeField, Min(.01f)] float captureNextStepBounceMinDuration = .08f;
    [SerializeField, Min(.01f)] float captureNextStepBounceMaxDuration = .72f;
    [SerializeField, Min(.1f)] float captureNextStepBounceFrequency = 10.5f;
    [SerializeField, Min(0f)] float captureNextStepBounceDamping = 2.6f;
    [SerializeField, Min(0f)] float captureNextStepBounceSpinImpulse = 1500f;
    [SerializeField] bool captureNextStepUseDistanceCosineWave = true;
    [SerializeField, Min(.05f)] float captureNextStepWaveLength = .90f;
    [SerializeField, Min(0f)] float captureNextStepWaveAmplitude = .65f;
    [SerializeField, Min(0f)] float captureNextStepWaveSideAmplitude = 0f;
    [SerializeField, Min(.01f)] float captureNextStepWaveFadeInDistance = .55f;
    [SerializeField, Min(.01f)] float captureNextStepWaveFadeDistance = 1.80f;
    [SerializeField, Range(.25f, 4f)] float captureNextStepWaveFadePower = 1.15f;
    [SerializeField] bool captureNextStepWaveFitEndpoint = true;
    [SerializeField, Min(0f)] float captureNextStepWaveReturnSeconds = .24f;
    [SerializeField] bool captureNextStepWaveIgnoreOtherVisualOffsets = true;
    [SerializeField] bool captureNextStepWaveUseSubjectLocalY = true;
    [SerializeField] bool captureNextStepWaveFreezeSpin = true;
    [SerializeField] bool captureNextStepWaveUseFixedCaptureNormal = true;
    [SerializeField] bool captureNextStepWaveUseDistanceBasedSpin = true;
    [SerializeField, Range(-2f, 2f)] float captureNextStepWaveSpinDirection = 1f;
    [SerializeField, Min(0f)] float captureNextStepWaveSpinMultiplier = 1f;
    [SerializeField, Min(0f)] float captureNextStepWaveVerticalRollMultiplier = 1.15f;
    [SerializeField, Min(0f)] float captureNextStepWaveHorizontalSideSpinMultiplier = .18f;
    [SerializeField, Range(-2f, 2f)] float captureNextStepWaveVerticalRollDirection = 1f;
    [SerializeField, Range(-2f, 2f)] float captureNextStepWaveHorizontalSideSpinDirection = 1f;
    [SerializeField, Min(.01f)] float captureNextStepWaveMaxStepDistance = .35f;
    [SerializeField, Min(.01f)] float visualSpinRadius = .5f;
    [SerializeField, Min(0f)] float visualSpinFollowSharpness = 6f;
    [SerializeField, Min(0f)] float visualSpinDamping = 1.6f;
    [SerializeField, Min(0f)] float visualImpactSpinImpulse = 1100f;
    [SerializeField, Min(0f)] float visualSpinChargeBuild = 3.5f;
    [SerializeField, Range(0f, 1f)] float visualSpinChargeRemain = .55f;
    [SerializeField, Min(0f)] float visualSpinMaxDegreesPerSecond = 3600f;
    [SerializeField] bool visualWildSpin = true;
    [SerializeField, Min(1f)] float visualWildSpinMultiplier = 2.8f;
    [SerializeField, Range(0f, 1f)] float visualWildSpinAxisJitter = .45f;
    [SerializeField, Min(0f)] float visualWildSpinPulseFrequency = 5f;
    [SerializeField, Range(0f, 1f)] float visualWildSpinPulseAmount = .35f;
    [SerializeField, Range(0f, 1f)] float visualWildSpinReleaseThreshold = .55f;
    [SerializeField, Min(0f)] float visualWildSpinReleaseImpulse = 1100f;
    [SerializeField, Min(0f)] float visualWildSpinReleaseCooldown = .18f;
    [SerializeField] Vector3 visualMeshLocalOffset;
    [SerializeField, Min(0f)] float visualMaxSeparation = .34f;
    [SerializeField, Min(0f)] float visualRecenteringSpinSpeed = 360f;
    [SerializeField, Min(0f)] float visualRecenteringAfterSeconds = .65f;
    [SerializeField, Min(.01f)] float visualTimedRecenteringSharpness = 18f;
    [SerializeField, Range(0f, 1f)] float visualRecenteringOffsetRemain = .15f;
    [SerializeField, Min(0f)] float visualTransitionMinDistance = .15f;
    [SerializeField, Min(0f)] float visualTransitionOffsetPerDistance = .08f;
    [SerializeField, Min(0f)] float visualTransitionMaxOffset = .18f;
    [SerializeField, Min(.01f)] float visualTransitionCatchUpSharpness = 7f;
    [SerializeField, Range(0f, 1f)] float visualTransitionDownAmount = .2f;
    [SerializeField, Min(0f)] float visualTransitionSpinPerDistance = 2200f;
    [SerializeField, Min(0f)] float visualTransitionCooldown = .08f;

    [Header("BallVisual - 流星型の平面終端滑落")]
    [Tooltip("平面終端から下り面へ移る瞬間だけ、BallVisualをsubject追従から一時的に外し、実速度と重力によるワールド軌道で滑落させます。PhysicsBallと停止計算には影響しません。")]
    [SerializeField] bool useVisualEdgeFloat = true;
    [Tooltip("流星型滑落に使う見た目の重力倍率です。1でPhysics.gravityと同じです。小さくすると前へ流れ、大きくすると鋭く落ちます。")]
    [SerializeField, Range(.2f, 1.5f)] float visualEdgeFloatGravityScale = .75f;
    [Tooltip("放物軌道を続ける最大時間です。接触予測がこれより早い場合は、予測時間を優先します。")]
    [SerializeField, Range(.05f, .30f)] float visualEdgeFloatMaximumDuration = .18f;
    [Tooltip("放物軌道からsubjectへ、斜面を滑るように合流する時間です。")]
    [SerializeField, Range(.01f, .15f)] float visualEdgeFloatHandoffSeconds = .05f;
    [SerializeField, HideInInspector] bool visualEdgeFloatReplacesPeakSlip = true;

    [SerializeField] bool visualPeakSlip = true;
    [SerializeField, Min(0f)] float visualPeakMinDistance = .20f;
    [SerializeField, Range(0f, 180f)] float visualPeakMinNormalAngle = 8f;
    [SerializeField, Min(0f)] float visualPeakMinDrop = .03f;
    [SerializeField, Range(0f, 89f)] float visualPeakMinSlopeAngle = 2f;
    [SerializeField, Min(0f)] float visualPeakOffsetPerDistance = .18f;
    [SerializeField, Min(0f)] float visualPeakMaxOffset = .20f;
    [SerializeField, Range(0f, 1f)] float visualPeakInertiaBackAmount = .90f;
    [SerializeField, Range(0f, 1f)] float visualPeakSlopePullAmount = .95f;
    [SerializeField, Range(0f, 1f)] float visualPeakDownAmount = .28f;
    [SerializeField, Min(.01f)] float visualPeakTakeSeconds = .13f;
    [SerializeField, Min(0f)] float visualPeakHoldSeconds = .32f;
    [SerializeField, Min(.01f)] float visualPeakReturnSharpness = 3.5f;
    [SerializeField, Min(0f)] float visualPeakSpinImpulse = 2600f;
    [SerializeField, Min(0f)] float visualPeakCrossSpinImpulse = 1600f;
    [SerializeField, Min(0f)] float visualPeakCooldown = .12f;
    [SerializeField] bool visualCurvatureContact = true;
    [SerializeField, Min(.01f)] float visualCurvatureMinRadius = .35f;
    [SerializeField, Min(.01f)] float visualCurvatureMaxRadius = 8f;
    [SerializeField, Range(0f, 1f)] float visualLowSupportStart = .70f;
    [SerializeField, Range(0f, 1f)] float visualNearDetachSupport = .15f;
    [SerializeField, Min(0f)] float visualCurvatureSlipMultiplier = 1.35f;
    [SerializeField] bool deterministicReplay = true;
    [SerializeField] int deterministicVisualSeed = 12345;
    [SerializeField] bool useFallRecovery = true;
    [Tooltip("ONならArcSlab0へ戻ります。OFFなら最後に記録した安全地点へ戻ります。")]
    [SerializeField] bool recoverToStartSlab = true;
    [SerializeField, Min(.1f)] float autoRecoverAfterAirTime = 2.5f;
    [SerializeField, Min(.1f)] float recoveryBelowSafePoint = 20f;
    [SerializeField, Min(0f)] float hardFallRecoverySpeed = 30f, hardFallRecoveryTime = .2f;
    [SerializeField, Min(0f)] float recoveryLift = .35f, safePointRecordInterval = .1f, maxSafeNormalSpeed = 4f;
    public float sphereRadius = .5f, groundProbeDistance = .25f;
    public LayerMask groundMask = ~0;
    [Range(0, 89)] public float maxSlopeAngle = 75;
    [SerializeField, Min(0f)] float maxExtraProbeDistance = 2f;
    public float maxGroundSpeed = 16f, maxGroundAcceleration = 65f, airAcceleration = 16f;
    [SerializeField, Range(2f, 75f)] float slopeForceTorqueMinAngle = 3f;
    [SerializeField, Range(0f, 2f)] float downhillGravityMultiplier = 1f;
    [SerializeField, Range(0f, 3f)] float slopeRollingResistance = .8f;
    [SerializeField] bool useSlopeProgressStop = true;
    [SerializeField, Range(.05f, .95f)] float slopeStopProgress = .30f;
    [SerializeField, Range(5f, 45f)] float slopeStopMaxBrakeAcceleration = 35f;
    [SerializeField, Range(.001f, .05f)] float slopeStopBoundaryEpsilon = .015f;
    [SerializeField, Range(.001f, .20f)] float slopeStopSpeedEpsilon = .035f;
    [SerializeField, Range(2f, 30f)] float slopeStopSmoothDeceleration = 12f;
    [SerializeField, Range(.80f, 1.10f)] float slopeStopCaptureRatio = .98f;
    [SerializeField, Range(5f, 200f)] float slopeStopBrakeJerk = 45f;
    [SerializeField] bool debugCaptureNextStep = true;
    [SerializeField] bool debugCaptureNextStepEveryFixedStep = true;
    [SerializeField, Range(.01f, .25f)] float debugCaptureNextStepLogInterval = .02f;
    [SerializeField, Range(0f, 1.5f)] float debugCaptureNextStepMinRatioToLog = .70f;
    [SerializeField] bool applyCaptureNextStepResultProfile = true;
    [SerializeField, Range(0f, 1f)] float captureNextStepInitialBrakeRatio = .55f;
    [SerializeField, Range(0f, 12f)] float captureNextStepInitialNetDeceleration = 2.5f;
    [SerializeField, Range(1f, 8f)] float captureNextStepBrakeJerkMultiplier = 3f;
    [SerializeField] bool captureNextStepUsePredictedDeceleration = true;
    [SerializeField] bool useCommittedPlanAxisForSlopeStop = true;
    [SerializeField] bool useCaptureNextStepTerminalVelocityLimit = true;
    [SerializeField, Range(1f, 8f)] float captureNextStepVelocityLimitResponseSteps = 2f;
    [SerializeField, Range(0f, .5f)] float captureNextStepTerminalSoftDistance = .08f;
    [SerializeField] bool showSlopeStopTargetMarker = true;
    [SerializeField] bool showCaptureNextStepMarker = true;
    [SerializeField] GameObject slopeStopMarkerPrefab;
    [SerializeField, Min(.01f)] float slopeStopMarkerScale = .28f;
    [SerializeField, Min(0f)] float slopeStopMarkerSurfaceOffset = .08f;
    [SerializeField] Color slopeStopTargetMarkerColor = Color.green;
    [SerializeField] Color captureNextStepMarkerColor = Color.cyan;

    [Header("Visual Edge Float Markers")]
    [Tooltip("平面終端から下り面へ入るときの BallisticDrop 開始点/終了点を表示します。PhysicsBall には影響しません。")]
    [SerializeField] bool showVisualEdgeFloatMarkers = true;
    [SerializeField] bool showVisualEdgeFloatStartMarker = true;
    [SerializeField] bool showVisualEdgeFloatEndMarker = true;
    [SerializeField] Color visualEdgeFloatStartMarkerColor = new Color(1f, .55f, 0f, 1f);
    [SerializeField] Color visualEdgeFloatEndMarkerColor = new Color(1f, .25f, 1f, 1f);
    [SerializeField, Min(.01f)] float visualEdgeFloatMarkerScaleMultiplier = .85f;

    [SerializeField] bool useSurfaceSnappedSlopeStopTarget = true;
    [SerializeField] bool followSlopeStopTargetWithCollider = true;
    [SerializeField] bool useSceneTargetDistanceForSlopeStop = true;
    [SerializeField, Min(.05f)] float slopeStopTargetProbeHeight = 3f;
    [SerializeField, Min(.1f)] float slopeStopTargetProbeDistance = 8f;
    [SerializeField, Min(.01f)] float slopeStopSceneMismatchWarning = .25f;
    [SerializeField] bool debugSlopeProgressStop = false;
    [SerializeField] bool drawSlopeProgressStop = true;

    [Header("Slope Stop Measurement Markers")]
    [Tooltip("停止計算を変えず、到達区域・現在位置・予測停止点だけを表示します。")]
    [SerializeField] bool showSlopeStopMeasurementMarkers = true;
    [SerializeField] bool showReachedZoneStartMarker = true;
    [SerializeField] bool showPassedSideReferenceMarker = true;
    [SerializeField] bool showCurrentProjectedPointMarker = true;
    [SerializeField] bool showPredictedStopPointMarker = true;
    [SerializeField] Color reachedZoneStartMarkerColor = Color.yellow;
    [SerializeField] Color passedSideReferenceMarkerColor = Color.red;
    [SerializeField] Color currentProjectedPointMarkerColor = Color.blue;
    [SerializeField] Color predictedStopPointMarkerColor = Color.magenta;
    [SerializeField, Min(.01f)] float measurementMarkerScaleMultiplier = .55f;

    [Header("Slope Stop Runtime Measurement (Play Mode)")]
    [SerializeField] float debugRemainingToTarget;
    [SerializeField] float debugForwardSpeed;
    [SerializeField] float debugPredictedStoppingDistance;
    [SerializeField] float debugFixedStepMargin;
    [SerializeField] bool debugReachedZoneEntered;
    [SerializeField] bool debugPassedExactTarget;

    [SerializeField, Range(-1.5f, 0f)] float targetNormalSpeed = -.25f;
    [SerializeField, Range(1f, 35f)] float normalSnapSharpness = 18f;
    [SerializeField, Range(0f, 12f)] float baseStickAcceleration = 4f;
    [SerializeField, Range(0f, 20f)] float extraStickAcceleration = 12f;
    public bool useAnalyticTrackAssist;
    public float derivativeStep = .05f, lookAheadDistance = 4f, analyticSharpness = 3f;
    public int takeoffSearchSegments = 20;
    public float analyticAmplitude = 2f, analyticFrequency = .5f, analyticYOffset;
    public bool hasSafePoint;
    public Vector3 lastSafePosition, lastSafeNormal = Vector3.up, lastSafeHeading = Vector3.right;
    public float airborneTime, hardFallTime;
    public GameObject slab;
    [SerializeField] Vector3 headingDir;
    Vector3 targetHeadingDir, groundNormal = Vector3.up, groundPoint;
    Vector2 flickStart;
    bool isGrounded, hasFlickStart, hasTakeoffPoint;
    Collider authoritativeGroundCollider;
    enum SlopeStopState
    {
        None,
        Armed,
        Committed,
        Braking,
        Reached
    }
    struct SlopeStopSegment
    {
        public SlopeStopState state;
        public Collider collider;
        public Vector3 entryPoint;
        public Vector3 direction;
        public float entryCoordinate;
        public float exitCoordinate;
        public float stopCoordinate;
        public float entryTime;
        public float entrySpeed;
        public float stopDistance;
        public int brakeReason;
        public bool captureNextStepTriggered;
        public float brakeDeceleration;
        public float brakeControl;
        public float brakeInitialControl;
        public bool hasLocalStopFrame;
        public Vector3 entryLocalPoint;
        public Vector3 stopLocalPoint;
        public Vector3 directionLocal;
        public Vector3 stopWorldPoint;
        public Vector3 stopWorldNormal;
    }
    enum EffectGuideState
    {
        None,
        MovingToTarget,
        HoldingAtTarget,
        ReturningToSubject
    }

    enum VisualEdgeDropState
    {
        None,
        BallisticDrop,
        LandingHandoff
    }
    SlopeStopSegment slopeStopSegment;
    bool hasQueuedTurn;
    float queuedTurnAngle;
    float lastPlayerTurnSign = 1f;
    float currentMargin, takeoffX, nextSafePointRecordTime;
    Tween headingTween, visualImpactTween, visualCaptureNextStepBounceTween, visualCaptureNextStepWaveReturnTween;
    Vector3 visualRestPosition, visualRestScale, visualLagOffset, visualCurrentOffset, visualTransitionOffset;
    Vector3 visualImpactTweenOffset, visualCaptureNextStepBounceOffset;
    Vector3 effectGuideCapturePoint, effectGuideTargetPoint;
    Vector3 effectGuideAxisWorld, effectGuideLocalUp, effectGuideLocalSide;
    Vector3 effectGuideFixedNormalWorld, effectGuideTangentWorld;
    Vector3 effectGuideSpinAxisLocal;
    Vector3 effectGuideRollAxisLocal, effectGuideSideSpinAxisLocal;
    Quaternion effectGuideSpinRotation = Quaternion.identity;
    float visualImpactTweenSquash, visualCaptureNextStepBounceSquash;
    float effectGuideDistance, effectGuideRuntimeAmplitude, effectGuidePreviousTraveled;
    int effectGuideHalfWaves = 1;
    bool effectGuideActive;
    EffectGuideState effectGuideState;
    Vector3 effectGuideCurrentWorldPoint;
    Vector3 effectGuideReturnStartWorldPoint;
    Vector3 effectGuideReturnPreviousWorldPoint;
    Quaternion effectGuideReturnStartRotation = Quaternion.identity;
    Quaternion effectGuideReturnVirtualSpinRotation = Quaternion.identity;
    float effectGuideStateStartTime;
    float effectGuideCurrentTraveled;
    Vector3 visualImpactUp, visualImpactSide, visualSpinVelocity, visualSpinKickVelocity;
    Vector3 slopeStopReachedTrickSpinVelocity;
    Vector3 visualGroundPoint, visualGroundNormal = Vector3.up;
    Vector3 visualPeakStartOffset, visualPeakEndOffset, visualPeakCurrentOffset;
    VisualEdgeDropState visualEdgeDropState;
    Vector3 visualEdgeDropStartWorldPoint;
    Vector3 visualEdgeDropLaunchVelocityWorld;
    Vector3 visualEdgeDropGravityWorld;
    Vector3 visualEdgeDropLaunchNormalWorld = Vector3.up;
    Vector3 visualEdgeDropLandingPlanePointWorld;
    Vector3 visualEdgeDropLandingPlaneNormalWorld = Vector3.up;
    Vector3 visualEdgeDropPreviousWorldPoint;
    Vector3 visualEdgeDropHandoffStartWorldPoint;
    Vector3 visualEdgeDropHandoffStartVelocityWorld;
    float visualEdgeDropAge;
    float visualEdgeDropContactTime;
    float visualEdgeDropHandoffAge;
    Quaternion visualRestRotation, visualSpinRotation = Quaternion.identity;
    Collider visualGroundCollider;
    bool hasVisualGroundReference;
    float visualImpactAge = -1f, visualImpactAmplitude, visualSpinCharge;
    float nextVisualImpactTime, nextVisualTransitionTime, visualStrongSpinTime, nextVisualWildSpinReleaseTime;
    float visualPeakAge = -1f, nextVisualPeakTime;
    float nextVisualEdgeFloatTime;
    float visualPeakSupport01 = 1f, visualPeakSupportLoss01, visualPeakCurvatureRadius;
    public float currentTangentSpeed;
    float nextCaptureNextStepDebugTime = -Mathf.Infinity;
    GameObject slopeStopTargetMarkerInstance;
    GameObject captureNextStepMarkerInstance;
    GameObject reachedZoneStartMarkerInstance;
    GameObject passedSideReferenceMarkerInstance;
    GameObject currentProjectedPointMarkerInstance;
    GameObject predictedStopPointMarkerInstance;
    GameObject visualEdgeFloatStartMarkerInstance;
    GameObject visualEdgeFloatEndMarkerInstance;
    struct PendingVisualPlaneTransition
    {
        public bool valid;
        public Vector3 previousPoint;
        public Vector3 previousNormal;
        public Vector3 nextPoint;
        public Vector3 nextNormal;
    }
    PendingVisualPlaneTransition pendingVisualPlaneTransition;
    bool simulationReady;
    float deterministicClock;
    int visualImpulseSequence;
    bool hasWarnedRoleSetup;
    float RuntimeClock => deterministicReplay ? deterministicClock : Time.time;
    public Vector3 HeadingDir => headingDir;
    public Vector3 FirstPlayerPos
    {
        get;
        private set;
    }
    void Reset()
    {
        ResolveRoleReferences();
    }
    void Awake()
    {
        ResolveRoleReferences();
        Vector3 startHeading = useSideTurnExperiment
        ? SnapToCardinalXZ(initialHeading)
        : Flat(headingTransform.forward, Vector3.forward);
        SetHeading(startHeading, false);
        CacheVisualRestPose();
    }
    void Start()
    {
        FindStageReferences();
        StartCoroutine(PlacePlayerOnStartSlab());
    }
    void Update()
    {
        ProbeVisualTransitionsAndDebug();
        ReadFlick();
    }
    void LateUpdate()
    {
        if (!deterministicReplay)
            StepVisualOnly(Time.deltaTime);
    }
    void OnDestroy()
    {
        headingTween?.Kill();
        visualImpactTween?.Kill();
        visualCaptureNextStepBounceTween?.Kill();
        visualCaptureNextStepWaveReturnTween?.Kill();
        ClearSlopeStopDebugMarkers();
    }
    void ResolveRoleReferences()
    {
        if (!subjectRoot)
            subjectRoot = transform;
        if (!physicsBodyRigidbody)
            physicsBodyRigidbody = GetComponent<Rigidbody>();
        if (!headingTransform)
            headingTransform = subjectRoot ? subjectRoot : transform;
        if (!ballVisualTransform)
        {
            Transform foundVisual = transform.Find("BallVisual");
            if (foundVisual)
                ballVisualTransform = foundVisual;
        }
        if (!effectGuideTransform)
        {
            Transform foundGuide = transform.Find("EffectGuide");
            if (foundGuide)
                effectGuideTransform = foundGuide;
        }
        WarnInvalidRoleSetupOnce();
    }
    void WarnInvalidRoleSetupOnce()
    {
        if (hasWarnedRoleSetup)
            return;
        hasWarnedRoleSetup = true;
        if (!physicsBodyRigidbody)
        {
            Debug.LogWarning(
            "[SlopeStickBall3D] PhysicsBall 用 Rigidbody がありません。SlopeStickBall3D は Rigidbody を持つ subject に付けてください。",
            this
            );
        }
        if (!ballVisualTransform)
        {
            Debug.LogWarning(
            "[SlopeStickBall3D] BallVisual が未設定です。見た目玉の Transform を ballVisualTransform に入れてください。",
            this
            );
        }
        if (ballVisualTransform &&
            ballVisualTransform.TryGetComponent(out Rigidbody visualRigidbody))
        {
            Debug.LogWarning(
            "[SlopeStickBall3D] BallVisual に Rigidbody が付いています。BallVisual は見た目だけなので Rigidbody は外してください。",
            visualRigidbody
            );
        }
        if (!warnIfChildHasRigidbody)
            return;
        Rigidbody[] childRigidbodies = GetComponentsInChildren<Rigidbody>(true);
        for (int i = 0; i < childRigidbodies.Length; i++)
        {
            Rigidbody childRigidbody = childRigidbodies[i];
            if (childRigidbody == physicsBodyRigidbody)
                continue;
            Debug.LogWarning(
            "[SlopeStickBall3D] 子 GameObject に別の Rigidbody があります。物理玉は1つだけにするのが安全です: " +
            childRigidbody.name,
            childRigidbody
            );
        }
    }
    Vector3 GetBallVisualAnchorWorld()
    {
        Vector3 localAnchor = visualRestPosition + visualMeshLocalOffset;
        if (ballVisualTransform && ballVisualTransform.parent)
            return ballVisualTransform.parent.TransformPoint(localAnchor);
        Transform root = subjectRoot ? subjectRoot : transform;
        return root.TransformPoint(localAnchor);
    }
    Vector3 ToBallVisualParentLocalPoint(Vector3 worldPoint)
    {
        if (ballVisualTransform && ballVisualTransform.parent)
            return ballVisualTransform.parent.InverseTransformPoint(worldPoint);
        Transform root = subjectRoot ? subjectRoot : transform;
        return root.InverseTransformPoint(worldPoint);
    }
    void SetEffectGuideState(EffectGuideState state)
    {
        effectGuideState = state;
        effectGuideStateStartTime = RuntimeClock;
    }
    void PlaceEffectGuideAtCapture(
    Vector3 captureWorldPoint,
    Vector3 targetWorldPoint,
    Vector3 normalWorld
    )
    {
        effectGuideCurrentWorldPoint = captureWorldPoint;
        SetEffectGuideState(EffectGuideState.MovingToTarget);
        ApplyEffectGuideTransform(captureWorldPoint, targetWorldPoint, normalWorld);
    }
    void ApplyEffectGuideTransform(
    Vector3 position,
    Vector3 lookTarget,
    Vector3 normalWorld
    )
    {
        if (!moveEffectGuideTransform || !effectGuideTransform)
            return;
        Vector3 forward = lookTarget - position;
        if (forward.sqrMagnitude <= 1e-6f)
            forward = effectGuideAxisWorld.sqrMagnitude > 1e-6f
            ? effectGuideAxisWorld
            : Vector3.forward;
        forward.Normalize();
        Vector3 up = normalWorld.sqrMagnitude > 1e-6f
        ? normalWorld.normalized
        : Vector3.up;
        effectGuideTransform.SetPositionAndRotation(
        position,
        Quaternion.LookRotation(forward, up)
        );
    }
    void UpdateEffectGuideTransformAtCurrentPoint()
    {
        Vector3 lookTarget = effectGuideCurrentWorldPoint +
        (effectGuideAxisWorld.sqrMagnitude > 1e-6f
        ? effectGuideAxisWorld.normalized
        : Vector3.forward);
        ApplyEffectGuideTransform(
        effectGuideCurrentWorldPoint,
        lookTarget,
        effectGuideFixedNormalWorld
        );
    }
    float GetEffectGuideReturnDuration()
    {
        float requestedDuration;
        if (useDeveloperReturnMotion)
        {
            requestedDuration = developerReturnUsesCaptureNextStepWaveReturnSeconds
            ? Mathf.Max(captureNextStepWaveReturnSeconds, .001f)
            : Mathf.Max(developerReturnSeconds, .001f);
        }
        else if (useCaptureNextStepWaveReturnSecondsForGuideReturn)
        {
            requestedDuration = Mathf.Max(captureNextStepWaveReturnSeconds, .001f);
        }
        else
        {
            requestedDuration = Mathf.Max(effectGuideReturnToSubjectSeconds, .001f);
        }

        // 本ゲームの映像ではTarget後の復帰が長く停止して見えないため、
        // 旧極端モード以外は短い受け渡しに制限します。
        return useLegacyExtremeCaptureVisuals
        ? requestedDuration
        : Mathf.Min(requestedDuration, .15f);
    }
    Vector3 GetFlatHeadingForReturn()
    {
        Vector3 forward = headingDir.sqrMagnitude > 1e-6f
        ? headingDir
        : subjectRoot
        ? subjectRoot.forward
        : transform.forward;
        forward = Vector3.ProjectOnPlane(forward, Vector3.up);
        if (forward.sqrMagnitude <= 1e-6f)
            return Vector3.forward;
        return forward.normalized;
    }
    Vector3 GetReturnCurveSideWorld()
    {
        Vector3 forward = GetFlatHeadingForReturn();
        Vector3 side = Side(forward);
        if (side.sqrMagnitude <= 1e-6f)
            side = Vector3.right;
        return side.normalized * lastPlayerTurnSign;
    }
    Vector3 GetEffectGuideReturnCurveOffset(float progress)
    {
        if (useDeveloperReturnMotion)
            return GetDeveloperReturnCurveOffset(progress);
        if (!useTrickyEffectGuideReturnCurve ||
            effectGuideReturnLateralCurveAmplitude <= .0001f)
        {
            return Vector3.zero;
        }
        progress = Mathf.Clamp01(progress);
        // 横方向だけにふくらませます。
        // Y方向の波は入れないので、戻り中に異空間へ縦跳ねしません。
        float wave = Mathf.Sin(
        progress * Mathf.PI * Mathf.Max(effectGuideReturnLateralCurveFrequency, .001f)
        );
        float fade = Mathf.Pow(
        1f - progress,
        Mathf.Max(effectGuideReturnLateralCurveFadePower, .001f)
        );
        float curveAmplitude = useLegacyExtremeCaptureVisuals
        ? effectGuideReturnLateralCurveAmplitude
        : Mathf.Min(effectGuideReturnLateralCurveAmplitude, safeReturnCurveAmplitude);
        return GetReturnCurveSideWorld() *
        (curveAmplitude * wave * fade);
    }
    float GetReturnBaseProgress(float progress)
    {
        progress = Mathf.Clamp01(progress);
        if (useDeveloperReturnMotion && !developerReturnUseSmoothStep)
            return progress;
        return progress * progress *(3f - 2f * progress);
    }
    float GetDeveloperReturnDirectionSign()
    {
        if (developerReturnCurveUsesLastHandleDirection)
            return Mathf.Abs(lastPlayerTurnSign) > .001f
        ? Mathf.Sign(lastPlayerTurnSign)
        : 1f;
        return Mathf.Abs(developerReturnCurveDirection) > .001f
        ? Mathf.Sign(developerReturnCurveDirection)
        : 1f;
    }
    Vector3 GetDeveloperReturnCurveOffset(float progress)
    {
        progress = Mathf.Clamp01(progress);
        float envelope = Mathf.Pow(
        Mathf.Sin(progress * Mathf.PI),
        Mathf.Max(developerReturnCurveEnvelopePower, .001f)
        );
        if (envelope <= .0001f)
            return Vector3.zero;
        float phase = developerReturnCurvePhaseDegrees * Mathf.Deg2Rad;
        float wave = Mathf.Sin(
        progress * Mathf.PI * 2f * Mathf.Max(developerReturnCurveCycles, .001f) + phase
        );
        float sign = GetDeveloperReturnDirectionSign();
        Vector3 side = GetReturnCurveSideWorld() * sign;
        float curveAmplitude = useLegacyExtremeCaptureVisuals
        ? developerReturnCurveSideAmplitude
        : Mathf.Min(developerReturnCurveSideAmplitude, safeReturnCurveAmplitude);
        Vector3 offset = side *
        (curveAmplitude * wave * envelope);
        // 基本は 0 推奨です。必要な場合だけ小さく入れます。
        if (developerReturnCurveVerticalAmplitude > .0001f)
        {
            offset += Vector3.up *
            (developerReturnCurveVerticalAmplitude * wave * envelope);
        }
        return offset;
    }
    Quaternion GetDeveloperReturnRotation(float progress)
    {
        progress = Mathf.Clamp01(progress);
        float baseProgress = GetReturnBaseProgress(progress);
        Quaternion baseRotation = Quaternion.Slerp(
        effectGuideReturnStartRotation,
        Quaternion.identity,
        baseProgress
        );
        float envelope = Mathf.Pow(
        Mathf.Sin(progress * Mathf.PI),
        Mathf.Max(developerReturnRotationEnvelopePower, .001f)
        );
        if (envelope <= .0001f)
            return baseRotation;
        Vector3 rollAxis = effectGuideRollAxisLocal.sqrMagnitude > 1e-6f
        ? effectGuideRollAxisLocal.normalized
        : Vector3.right;
        Vector3 sideSpinAxis = effectGuideSideSpinAxisLocal.sqrMagnitude > 1e-6f
        ? effectGuideSideSpinAxisLocal.normalized
        : Vector3.up;
        Vector3 twistAxis = effectGuideSpinAxisLocal.sqrMagnitude > 1e-6f
        ? effectGuideSpinAxisLocal.normalized
        : Vector3.forward;
        float handleSign = GetDeveloperReturnDirectionSign();
        float allowedRollDegrees = useLegacyExtremeCaptureVisuals
        ? developerReturnRollDegrees
        : Mathf.Min(developerReturnRollDegrees, safeDeveloperReturnRollDegrees);
        float allowedSideSpinDegrees = useLegacyExtremeCaptureVisuals
        ? developerReturnSideSpinDegrees
        : Mathf.Min(developerReturnSideSpinDegrees, safeDeveloperReturnSideSpinDegrees);
        float rollAngle =
        allowedRollDegrees *
        envelope *
        Mathf.Sign(developerReturnRollDirection == 0f ? 1f : developerReturnRollDirection);
        float sideAngle =
        allowedSideSpinDegrees *
        Mathf.Sin(progress * Mathf.PI * 2f) *
        envelope *
        handleSign *
        Mathf.Sign(developerReturnSideSpinDirection == 0f ? 1f : developerReturnSideSpinDirection);
        float twistAngle =
        developerReturnTwistDegrees *
        Mathf.Sin(progress * Mathf.PI * 2f) *
        envelope *
        handleSign *
        Mathf.Sign(developerReturnTwistDirection == 0f ? 1f : developerReturnTwistDirection);
        Quaternion extraRotation = Quaternion.identity;
        if (developerReturnRollDegrees > .0001f)
        {
            extraRotation = Quaternion.AngleAxis(
            rollAngle,
            rollAxis
            ) * extraRotation;
        }
        if (developerReturnSideSpinDegrees > .0001f)
        {
            extraRotation = Quaternion.AngleAxis(
            sideAngle,
            sideSpinAxis
            ) * extraRotation;
        }
        if (developerReturnTwistDegrees > .0001f)
        {
            extraRotation = Quaternion.AngleAxis(
            twistAngle,
            twistAxis
            ) * extraRotation;
        }
        return baseRotation * extraRotation;
    }
    void UpdateEffectGuideReturnVirtualTorque(
    Vector3 previousWorldPoint,
    Vector3 currentWorldPoint,
    float progress
    )
    {
        if (useDeveloperReturnMotion && useDeveloperReturnRotation)
        {
            effectGuideSpinRotation = GetDeveloperReturnRotation(progress);
            return;
        }
        if (!useVirtualTorqueWhileReturningToSubject)
        {
            effectGuideSpinRotation = Quaternion.Slerp(
            effectGuideReturnStartRotation,
            Quaternion.identity,
            progress
            );
            return;
        }
        Vector3 delta = currentWorldPoint - previousWorldPoint;
        float distance = delta.magnitude;
        if (distance <= .0001f)
        {
            effectGuideSpinRotation = Quaternion.Slerp(
            effectGuideReturnStartRotation * effectGuideReturnVirtualSpinRotation,
            Quaternion.identity,
            progress
            );
            return;
        }
        float radius = Mathf.Max(visualSpinRadius, sphereRadius, .001f);
        float baseDegrees = distance / radius * Mathf.Rad2Deg;
        Vector3 moveWorld = delta / distance;
        Vector3 rollAxisWorld = Vector3.Cross(Vector3.up, moveWorld);
        if (rollAxisWorld.sqrMagnitude <= 1e-6f)
            rollAxisWorld = GetReturnCurveSideWorld();
        rollAxisWorld.Normalize();
        Vector3 sideSpinAxisWorld = Vector3.up;
        Transform parent = ballVisualTransform ? ballVisualTransform.parent : null;
        Vector3 rollAxisLocal = parent
        ? parent.InverseTransformDirection(rollAxisWorld).normalized
        : rollAxisWorld;
        Vector3 sideSpinAxisLocal = parent
        ? parent.InverseTransformDirection(sideSpinAxisWorld).normalized
        : sideSpinAxisWorld;
        float middle = Mathf.Sin(progress * Mathf.PI);
        Quaternion deltaRotation = Quaternion.identity;
        if (rollAxisLocal.sqrMagnitude > 1e-6f)
        {
            deltaRotation = Quaternion.AngleAxis(
            baseDegrees * returnVerticalRollMultiplier,
            rollAxisLocal.normalized
            ) * deltaRotation;
        }
        if (sideSpinAxisLocal.sqrMagnitude > 1e-6f &&
            returnHorizontalSideSpinMultiplier > .0001f)
        {
            float sideDegrees =
            baseDegrees *
            returnHorizontalSideSpinMultiplier *
            middle *
            lastPlayerTurnSign;
            deltaRotation = Quaternion.AngleAxis(
            sideDegrees,
            sideSpinAxisLocal.normalized
            ) * deltaRotation;
        }
        effectGuideReturnVirtualSpinRotation =
        deltaRotation * effectGuideReturnVirtualSpinRotation;
        // 終点では必ず元の姿勢へ戻します。
        Quaternion spinning = effectGuideReturnStartRotation *
        effectGuideReturnVirtualSpinRotation;
        effectGuideSpinRotation = Quaternion.Slerp(
        spinning,
        Quaternion.identity,
        progress * progress
        );
    }
    void BeginEffectGuideReturnToSubject()
    {
        effectGuideReturnStartWorldPoint = effectGuideCurrentWorldPoint;
        effectGuideReturnPreviousWorldPoint = effectGuideCurrentWorldPoint;
        effectGuideReturnStartRotation = effectGuideSpinRotation;
        effectGuideReturnVirtualSpinRotation = Quaternion.identity;
        SetEffectGuideState(EffectGuideState.ReturningToSubject);
        // 復帰中は「元のメイン玉へ戻る」ことが目的なので、
        // 余弦波による過剰な縦揺れはここで止めます。
        visualCaptureNextStepBounceOffset = Vector3.zero;
        visualCaptureNextStepBounceSquash = 0f;
    }
    void CompleteEffectGuideReturnToSubject()
    {
        if (ballVisualTransform && ballVisualTransform != transform)
        {
            ballVisualTransform.localPosition = visualRestPosition + visualMeshLocalOffset;
            ballVisualTransform.localRotation = visualRestRotation * visualSpinRotation;
            ballVisualTransform.localScale = visualRestScale;
        }
        effectGuideReturnVirtualSpinRotation = Quaternion.identity;
        effectGuideReturnPreviousWorldPoint = Vector3.zero;
        ResetEffectGuideRuntime();
    }
    void ResetEffectGuideRuntime()
    {
        effectGuideActive = false;
        effectGuideState = EffectGuideState.None;
        effectGuideDistance = 0f;
        effectGuideRuntimeAmplitude = 0f;
        effectGuidePreviousTraveled = 0f;
        effectGuideCurrentTraveled = 0f;
        effectGuideCurrentWorldPoint = Vector3.zero;
        effectGuideReturnStartWorldPoint = Vector3.zero;
        effectGuideReturnPreviousWorldPoint = Vector3.zero;
        effectGuideSpinRotation = Quaternion.identity;
        effectGuideReturnStartRotation = Quaternion.identity;
        effectGuideReturnVirtualSpinRotation = Quaternion.identity;
    }
    void FindStageReferences()
    {
        if (!stagePivot)
        {
            GameObject center = GameObject.Find("Center1");
            if (center)
                stagePivot = center.transform;
        }
        if (!stageRoot)
            stageRoot = stagePivot;
    }
    IEnumerator PlacePlayerOnStartSlab()
    {
        yield return new WaitForSeconds(.8f);
        FindStageReferences();
        slab = GameObject.Find("ArcSlab0");
        if (!slab)
        {
            Debug.LogWarning("ArcSlab0 が見つかりません。");
            yield break;
        }
        yield return new WaitForSeconds(.1f);
        yield return new WaitForFixedUpdate();
        simulationReady = false;
        FirstPlayerPos = slab.transform.position + Vector3.up * 2f;
        physicsBodyRigidbody.position = FirstPlayerPos;
        physicsBodyRigidbody.velocity = Vector3.zero;
        physicsBodyRigidbody.angularVelocity = Vector3.zero;
        ResetVisualOnly();
        deterministicClock = 0f;
        visualImpulseSequence = 0;
        pendingVisualPlaneTransition = default;
        hasQueuedTurn = false;
        queuedTurnAngle = 0f;
        slopeStopSegment = default;
        ClearSlopeStopDebugMarkers();
        isGrounded = false;
        groundNormal = Vector3.up;
        groundPoint = physicsBodyRigidbody.position;
        airborneTime = hardFallTime = 0f;
        hasVisualGroundReference = false;
        visualGroundCollider = null;
        visualGroundNormal = Vector3.up;
        visualPeakAge = -1f;
        visualPeakSupport01 = 1f;
        visualPeakSupportLoss01 = visualPeakCurvatureRadius = 0f;
        visualPeakStartOffset = visualPeakEndOffset = visualPeakCurrentOffset = Vector3.zero;
        nextCaptureNextStepDebugTime = -Mathf.Infinity;
        Physics.SyncTransforms();
        SetRecoveryPoint(physicsBodyRigidbody.position, Vector3.up, headingDir);
        simulationReady = true;
    }
    void ProbeVisualTransitionsAndDebug()
    {
        if (!simulationReady || !physicsBodyRigidbody)
            return;

        // 物理接地判定とは分離した見た目専用Probeです。
        // 0度の平面も拾うため、平面→下り斜面の境界を検出できます。
        // この結果はisGroundedやSlopeStopSegmentへ書き込みません。
        if (!ProbeVisualGround(out RaycastHit hit))
            return;

        TrackVisualPlaneTransition(hit);
    }
    void ReadFlick()
    {
        if (UnityEngine.Input.GetMouseButtonDown(0))
        {
            flickStart = UnityEngine.Input.mousePosition;
            hasFlickStart = true;
        }
        Vector3 tangentVelocity = Vector3.ProjectOnPlane(
        physicsBodyRigidbody.velocity,
        groundNormal
        );
        currentTangentSpeed = tangentVelocity.magnitude;
        if (!UnityEngine.Input.GetMouseButtonUp(0) || !hasFlickStart)
            return;
        hasFlickStart = false;
        Vector2 flick = (Vector2)UnityEngine.Input.mousePosition - flickStart;
        float flickPixels = useSideTurnExperiment ? Mathf.Abs(flick.x) : flick.magnitude;
        if (flickPixels < minFlickPixels)
            return;
        Vector3 current = Flat(
        headingDir,
        headingTransform ? headingTransform.forward : Vector3.forward
        );
        Vector3 input;
        if (useSideTurnExperiment)
        {
            if (Mathf.Abs(flick.x) <= Mathf.Abs(flick.y))
                return;
            input = flick.x > 0f ? -Side(current) : Side(current);
        }
        else
        {
            GetCameraGroundBasis(out Vector3 forward, out Vector3 right);
            input = right * flick.x + forward * flick.y;
            if (input.sqrMagnitude <= 1e-6f)
                return;
            input.Normalize();
        }
        float rawAngle = Vector3.SignedAngle(current, input, Vector3.up);
        float turnAngle = GetLogarithmicTurnAngle(rawAngle, flickPixels);
        if (Mathf.Abs(turnAngle) > .001f)
            QueueTurnForNextPhysicsStep(turnAngle);
    }
    void QueueTurnForNextPhysicsStep(float turnAngle)
    {
        queuedTurnAngle = turnAngle;
        hasQueuedTurn = true;
        if (Mathf.Abs(turnAngle) > .001f)
            lastPlayerTurnSign = Mathf.Sign(turnAngle);
    }
    void ApplyQueuedTurnAtFixedBoundary()
    {
        if (!hasQueuedTurn)
            return;
        float turnAngle = queuedTurnAngle;
        hasQueuedTurn = false;
        queuedTurnAngle = 0f;
        Vector3 current = Flat(
        headingDir,
        headingTransform ? headingTransform.forward : Vector3.forward
        );
        SetHeading(Quaternion.AngleAxis(turnAngle, Vector3.up) * current);
    }
    float GetLogarithmicTurnAngle(float signedInputAngle, float flickPixels)
    {
        if (Mathf.Abs(signedInputAngle) <= .001f)
            return 0f;
        float maxAngle = Mathf.Min(maxTurnPerFlick, maxPlayerTurnAngle);
        if (maxAngle <= 0f)
            return 0f;
        if (flickTurnAngleSteps == null || flickTurnAngleSteps.Length == 0)
            return Mathf.Sign(signedInputAngle) * Mathf.Min(12.5f, maxAngle);
        float basePixels = Mathf.Max(minFlickPixels, 1f);
        float ratio = Mathf.Max(flickStrengthBandRatio, 1.01f);
        int index = Mathf.FloorToInt(
        Mathf.Log(Mathf.Max(flickPixels, basePixels) / basePixels, ratio)
        );
        index = Mathf.Clamp(index, 0, flickTurnAngleSteps.Length - 1);
        float angle = Mathf.Clamp(
        flickTurnAngleSteps[index],
        Mathf.Min(12.5f, maxAngle),
        maxAngle
        );
        return Mathf.Sign(signedInputAngle) * angle;
    }
    void SetHeading(Vector3 requestedDirection, bool rotate = true)
    {
        Vector3 requested = Flat(requestedDirection, headingDir);
        bool snapToCardinal = useSideTurnExperiment &&
        snapHeadingOnSideTest &&
        maxPlayerTurnAngle >= 90f - .001f;
        if (snapToCardinal)
            requested = SnapToCardinalXZ(requested);
        if (!rotate)
        {
            targetHeadingDir = requested;
            ApplyHeading(requested);
            SetHeadingRotation(headingTransform, Quaternion.LookRotation(headingDir, Vector3.up));
            return;
        }
        Vector3 start = Flat(headingDir, requested);
        float angle = Mathf.Clamp(
        Vector3.SignedAngle(start, requested, Vector3.up),
        -maxPlayerTurnAngle,
        maxPlayerTurnAngle
        );
        targetHeadingDir = Flat(Quaternion.AngleAxis(angle, Vector3.up) * start, start);
        RotateHeadingAndStage(start, angle);
    }
    void RotateHeadingAndStage(Vector3 startDir, float playerAngle)
    {
        FindStageReferences();
        Transform target = headingTransform ? headingTransform : transform;
        if (!target)
            return;
        headingTween?.Kill();
        Quaternion headingStart = target.rotation;
        Quaternion headingEnd = Quaternion.AngleAxis(playerAngle, Vector3.up) * headingStart;
        bool canRotateStage = stageRoot && stagePivot;
        bool playerIsStageChild = canRotateStage && transform.IsChildOf(stageRoot);
        Vector3 pivot = canRotateStage ? stagePivot.position : Vector3.zero;
        Vector3 stageStartPosition = canRotateStage ? stageRoot.position : Vector3.zero;
        Quaternion stageStartRotation = canRotateStage ? stageRoot.rotation : Quaternion.identity;
        Vector3 playerStartPosition = physicsBodyRigidbody ? physicsBodyRigidbody.position : transform.position;
        float stageMultiplier = stageTurnsOppositeToPlayer ? -1f : 1f;
        float duration = GetHeadingTurnDuration(playerAngle);
        void Apply(float angle)
        {
            Quaternion playerTurn = Quaternion.AngleAxis(angle, Vector3.up);
            Quaternion stageTurn = Quaternion.AngleAxis(angle * stageMultiplier, Vector3.up);
            ApplyHeading(playerTurn * startDir);
            SetHeadingRotation(target, playerTurn * headingStart);
            ApplyStageAndPlayerOrbit(
            stageTurn,
            canRotateStage,
            playerIsStageChild,
            pivot,
            stageStartPosition,
            stageStartRotation,
            playerStartPosition
            );
        }
        if (duration <= 0f || Mathf.Abs(playerAngle) <= .001f)
        {
            Apply(playerAngle);
            SetHeadingRotation(target, headingEnd);
            return;
        }
        headingTween = DOTween.To(() => 0f, Apply, playerAngle, duration)
        .SetEase(headingTurnEase)
        .SetUpdate(UpdateType.Fixed)
        .OnComplete(() =>
        {
            ApplyHeading(targetHeadingDir);
            SetHeadingRotation(target, headingEnd);
            ApplyStageAndPlayerOrbit(
            Quaternion.AngleAxis(playerAngle * stageMultiplier, Vector3.up),
            canRotateStage,
            playerIsStageChild,
            pivot,
            stageStartPosition,
            stageStartRotation,
            playerStartPosition
            );
        }
        );
    }
    void ApplyHeading(Vector3 direction) => headingDir = Flat(direction, targetHeadingDir);
    float GetHeadingTurnDuration(float angle)
    {
        float max = Mathf.Max(maxPlayerTurnAngle, .001f);
        float min = Mathf.Min(12.5f, max);
        float t = max <= min + .001f ? 1f : Mathf.InverseLerp(min, max, Mathf.Abs(angle));
        return Mathf.Lerp(
        Mathf.Min(minimumHeadingTurnDuration, headingTurnDuration),
        headingTurnDuration,
        t
        );
    }
    void ApplyStageAndPlayerOrbit(
    Quaternion stageTurn,
    bool canRotateStage,
    bool playerIsStageChild,
    Vector3 pivot,
    Vector3 stageStartPosition,
    Quaternion stageStartRotation,
    Vector3 playerStartPosition
    )
    {
        if (!canRotateStage)
            return;
        stageRoot.SetPositionAndRotation(
        pivot + stageTurn *(stageStartPosition - pivot),
        stageTurn * stageStartRotation
        );
        if (!movePlayerAroundStagePivot || playerIsStageChild)
            return;
        Vector3 position = pivot + stageTurn *(playerStartPosition - pivot);
        if (physicsBodyRigidbody && !physicsBodyRigidbody.isKinematic)
            physicsBodyRigidbody.MovePosition(position);
        else if (physicsBodyRigidbody)
            physicsBodyRigidbody.position = position;
        else
            transform.position = position;
    }
    void SetHeadingRotation(Transform target, Quaternion rotation)
    {
        if (target == transform && physicsBodyRigidbody && !physicsBodyRigidbody.isKinematic)
            physicsBodyRigidbody.MoveRotation(rotation);
        else if (target)
            target.rotation = rotation;
    }
    void GetCameraGroundBasis(out Vector3 forward, out Vector3 right)
    {
        forward = cameraTransform
        ? Vector3.ProjectOnPlane(cameraTransform.forward, Vector3.up)
        : Vector3.forward;
        forward = Flat(forward, Vector3.forward);
        right = Flat(Vector3.Cross(Vector3.up, forward), Vector3.right);
    }
    static Vector3 Flat(Vector3 direction, Vector3 fallback)
    {
        direction = Vector3.ProjectOnPlane(direction, Vector3.up);
        if (direction.sqrMagnitude > 1e-6f)
            return direction.normalized;
        fallback = Vector3.ProjectOnPlane(fallback, Vector3.up);
        return fallback.sqrMagnitude > 1e-6f ? fallback.normalized : Vector3.right;
    }
    public static Vector3 Side(Vector3 direction)
    {
        direction = Flat(direction, Vector3.right);
        return new Vector3(-direction.z, 0f, direction.x);
    }
    static Vector3 SnapToCardinalXZ(Vector3 direction)
    {
        direction = Flat(direction, Vector3.right);
        if (Mathf.Abs(direction.x) >= Mathf.Abs(direction.z))
            return direction.x >= 0f ? Vector3.right : Vector3.left;
        return direction.z >= 0f ? Vector3.forward : Vector3.back;
    }
    void CacheVisualRestPose()
    {
        if (!ballVisualTransform || ballVisualTransform == transform)
            return;
        visualRestPosition = ballVisualTransform.localPosition;
        visualRestRotation = ballVisualTransform.localRotation;
        visualRestScale = ballVisualTransform.localScale;
    }
    [ContextMenu("Snap BallVisual To Rest")]
    void SnapVisualToRest()
    {
        if (!ballVisualTransform || ballVisualTransform == transform)
            return;
        visualLagOffset = visualCurrentOffset = visualTransitionOffset = visualPeakCurrentOffset =
        visualPeakStartOffset = visualPeakEndOffset = visualSpinVelocity = visualSpinKickVelocity = Vector3.zero;
        slopeStopReachedTrickSpinVelocity = Vector3.zero;
        ResetVisualEdgeFloat(false);
        visualImpactAge = visualPeakAge = -1f;
        visualImpactTween?.Kill();
        visualCaptureNextStepBounceTween?.Kill();
        visualCaptureNextStepWaveReturnTween?.Kill();
        visualImpactTweenOffset = visualCaptureNextStepBounceOffset = Vector3.zero;
        visualImpactTweenSquash = visualCaptureNextStepBounceSquash = 0f;
        ResetEffectGuideRuntime();
        visualSpinCharge = visualStrongSpinTime = nextVisualWildSpinReleaseTime = 0f;
        nextVisualPeakTime = 0f;
        visualSpinRotation = Quaternion.identity;
        ballVisualTransform.localPosition = visualRestPosition + visualMeshLocalOffset;
        ballVisualTransform.localRotation = visualRestRotation;
        ballVisualTransform.localScale = visualRestScale;
    }
    void TryStartVisualImpact(float speed, Vector3 normal)
    {
        if (RuntimeClock < nextVisualImpactTime || speed < visualImpactMinSpeed)
            return;
        StartVisualImpact(speed, normal);
        nextVisualImpactTime = RuntimeClock + visualImpactCooldown;
    }
    void StartVisualImpact(float speed, Vector3 normal)
    {
        if (!ballVisualTransform || ballVisualTransform == transform)
            return;
        Transform parent = ballVisualTransform.parent;
        Vector3 localUp = parent
        ? parent.InverseTransformDirection(normal).normalized
        : normal.normalized;
        Vector3 tangent = Vector3.ProjectOnPlane(physicsBodyRigidbody ? physicsBodyRigidbody.velocity : headingDir, normal);
        Vector3 side = tangent.sqrMagnitude > 1e-6f
        ? Vector3.Cross(normal, tangent.normalized)
        : Vector3.Cross(normal, headingDir);
        visualImpactSide = parent
        ? parent.InverseTransformDirection(side).normalized
        : side.normalized;
        if (visualImpactSide.sqrMagnitude <= 1e-6f)
            visualImpactSide = Vector3.right;
        visualImpactUp = localUp;
        visualImpactAmplitude = Mathf.Min(
        (speed - visualImpactMinSpeed) * visualImpactPerSpeed,
        visualImpactMaxOffset
        );
        if (visualImpactAmplitude <= .0001f)
            return;
        float strength = visualImpactAmplitude / Mathf.Max(visualImpactMaxOffset, .001f);
        AddVisualSpinImpulse(visualImpactSide, visualImpactSpinImpulse * strength);
        visualSpinCharge *= visualSpinChargeRemain;
        Vector3 bounceDirection = (localUp + visualImpactSide * visualImpactSideAmount).normalized;
        PlayVisualImpactTween(
        bounceDirection,
        visualImpactAmplitude,
        visualImpactSquash * strength
        );
    }
    void PlayVisualImpactTween(
    Vector3 bounceDirection,
    float amplitude,
    float squashAmount
    )
    {
        if (!useDotweenVisualBounce)
        {
            visualImpactAge = 0f;
            return;
        }
        visualImpactTween?.Kill();
        visualImpactTweenOffset = Vector3.zero;
        visualImpactTweenSquash = 0f;
        visualImpactAge = -1f;
        float duration = Mathf.Clamp(
        5f / Mathf.Max(visualImpactDamping, .001f),
        .08f,
        .8f
        );
        float tweenAge = 0f;
        visualImpactTween = DOTween.To(
        () => tweenAge,
        age =>
        {
            tweenAge = age;
            float phase = age * visualImpactFrequency * Mathf.PI * 2f;
            float decay = Mathf.Exp(-visualImpactDamping * age);
            visualImpactTweenOffset =
            bounceDirection *
            (amplitude * Mathf.Sin(phase) * decay);
            visualImpactTweenSquash =
            squashAmount *
            decay *
            (.5f + .5f * Mathf.Cos(phase));
        }
        ,
        duration,
        duration
        )
        .SetEase(Ease.Linear)
        .SetUpdate(UpdateType.Fixed)
        .OnComplete(() =>
        {
            visualImpactTweenOffset = Vector3.zero;
            visualImpactTweenSquash = 0f;
            visualImpactTween = null;
        }
        );
    }
    void StartCaptureNextStepBounce(
    Vector3 captureWorldPoint,
    Vector3 targetWorldPoint,
    Vector3 surfaceNormal,
    Vector3 planAxis,
    float speed,
    float remainingDistance
    )
    {
        // CaptureNextStepが滑落中に始まった場合は瞬間的に切り替えず、
        // まず短いLandingHandoffへ移してからCapture演出へ渡します。
        if (IsVisualEdgeDropActive())
            BeginVisualEdgeDropHandoffImmediately();

        if (!useDotweenVisualBounce ||
            !useCaptureNextStepBounce ||
            !ballVisualTransform ||
            ballVisualTransform == transform)
        {
            return;
        }
        Transform parent = ballVisualTransform.parent;
        Vector3 worldAxis = targetWorldPoint - captureWorldPoint;
        float worldDistance = worldAxis.magnitude;
        if (worldDistance <= .001f)
        {
            worldAxis = planAxis.sqrMagnitude > 1e-6f
            ? planAxis.normalized * Mathf.Max(remainingDistance, .001f)
            : Vector3.forward * Mathf.Max(remainingDistance, .001f);
            worldDistance = worldAxis.magnitude;
            targetWorldPoint = captureWorldPoint + worldAxis;
        }
        if (captureNextStepUseDistanceCosineWave)
        {
            StartCaptureNextStepDistanceCosineWave(
            parent,
            captureWorldPoint,
            targetWorldPoint,
            worldAxis / Mathf.Max(worldDistance, .001f),
            worldDistance,
            surfaceNormal,
            planAxis,
            speed
            );
            return;
        }
        float safeSpeed = Mathf.Max(speed, .001f);
        float safeRemaining = Mathf.Max(remainingDistance, .001f);
        float duration = Mathf.Clamp(
        safeRemaining / safeSpeed * 2.25f,
        captureNextStepBounceMinDuration,
        captureNextStepBounceMaxDuration
        );
        float amplitude = Mathf.Min(
        Mathf.Max(safeSpeed * captureNextStepBouncePerSpeed, sphereRadius * .08f),
        captureNextStepBounceMaxOffset,
        GetVisualMaxOffset()
        );
        if (amplitude <= .0001f)
            return;
        Vector3 localNormal = ToVisualLocalDirection(parent, surfaceNormal);
        if (localNormal.sqrMagnitude <= 1e-6f)
            localNormal = Vector3.up;
        Vector3 localAxis = ToVisualLocalDirection(parent, planAxis);
        Vector3 localSide = Vector3.Cross(localNormal, localAxis);
        if (localSide.sqrMagnitude <= 1e-6f)
            localSide = visualImpactSide.sqrMagnitude > 1e-6f
            ? visualImpactSide
            : Vector3.right;
        else
            localSide.Normalize();
        Vector3 bounceDirection = (
        localNormal +
        localSide * captureNextStepBounceSideAmount
        ).normalized;
        PlayCaptureNextStepBounceTween(
        bounceDirection,
        localSide,
        amplitude,
        captureNextStepBounceSquash,
        duration
        );
    }
    void StartCaptureNextStepDistanceCosineWave(
    Transform parent,
    Vector3 captureWorldPoint,
    Vector3 targetWorldPoint,
    Vector3 axisWorld,
    float distance,
    Vector3 surfaceNormal,
    Vector3 planAxis,
    float speed
    )
    {
        visualCaptureNextStepBounceTween?.Kill();
        visualCaptureNextStepWaveReturnTween?.Kill();
        visualCaptureNextStepBounceOffset = Vector3.zero;
        visualCaptureNextStepBounceSquash = 0f;
        Vector3 visualGuideStartPoint = ballVisualFollowsEffectGuideWorld
        ? GetBallVisualAnchorWorld()
        : captureWorldPoint;
        effectGuideCapturePoint = visualGuideStartPoint;
        effectGuideTargetPoint = visualGuideStartPoint +
        axisWorld.normalized * Mathf.Max(distance, .001f);
        effectGuideAxisWorld = axisWorld.sqrMagnitude > 1e-6f
        ? axisWorld.normalized
        : planAxis.sqrMagnitude > 1e-6f
        ? planAxis.normalized
        : Vector3.forward;
        Vector3 selectedNormal = captureNextStepWaveUseFixedCaptureNormal
        ? surfaceNormal
        : groundNormal;
        effectGuideFixedNormalWorld = selectedNormal.sqrMagnitude > 1e-6f
        ? selectedNormal.normalized
        : groundNormal.sqrMagnitude > 1e-6f
        ? groundNormal.normalized
        : Vector3.up;
        Vector3 tangentWorld = Vector3.ProjectOnPlane(
        effectGuideAxisWorld,
        effectGuideFixedNormalWorld
        );
        if (tangentWorld.sqrMagnitude <= 1e-6f)
            tangentWorld = effectGuideAxisWorld;
        effectGuideTangentWorld = tangentWorld.sqrMagnitude > 1e-6f
        ? tangentWorld.normalized
        : Vector3.forward;
        effectGuideDistance = Mathf.Max(distance, .001f);
        effectGuideHalfWaves = GetCaptureNextStepWaveHalfWaveCount(
        effectGuideDistance
        );
        effectGuideLocalUp = captureNextStepWaveUseSubjectLocalY
        ? Vector3.up
        : ToVisualLocalDirection(parent, effectGuideFixedNormalWorld);
        if (effectGuideLocalUp.sqrMagnitude <= 1e-6f)
            effectGuideLocalUp = Vector3.up;
        else
            effectGuideLocalUp.Normalize();
        Vector3 localAxis = ToVisualLocalDirection(
        parent,
        effectGuideTangentWorld
        );
        effectGuideLocalSide = Vector3.Cross(
        effectGuideLocalUp,
        localAxis
        );
        if (effectGuideLocalSide.sqrMagnitude <= 1e-6f)
            effectGuideLocalSide = Vector3.right;
        else
            effectGuideLocalSide.Normalize();
        // 縦回転は「進行方向へ転がる回転」です。
        // localSide を主軸にすると、BallVisual が前へ転がるように見えます。
        effectGuideRollAxisLocal = effectGuideLocalSide;
        if (effectGuideRollAxisLocal.sqrMagnitude <= 1e-6f)
            effectGuideRollAxisLocal = Vector3.right;
        else
            effectGuideRollAxisLocal.Normalize();
        // 横回転は少量だけ混ぜる飾りです。
        // localUp 軸にすると、コマのような横スピンになります。
        effectGuideSideSpinAxisLocal = effectGuideLocalUp;
        if (effectGuideSideSpinAxisLocal.sqrMagnitude <= 1e-6f)
            effectGuideSideSpinAxisLocal = Vector3.up;
        else
            effectGuideSideSpinAxisLocal.Normalize();
        // 旧変数は縦回転側を指すように残します。
        effectGuideSpinAxisLocal = effectGuideRollAxisLocal;
        effectGuideSpinRotation = Quaternion.identity;
        effectGuidePreviousTraveled = 0f;
        float speedAmplitude = Mathf.Max(speed, .001f) * captureNextStepBouncePerSpeed;
        effectGuideRuntimeAmplitude = Mathf.Max(
        captureNextStepWaveAmplitude,
        speedAmplitude
        );
        if (!useLegacyExtremeCaptureVisuals)
        {
            effectGuideRuntimeAmplitude = Mathf.Min(
            effectGuideRuntimeAmplitude,
            safeCaptureWaveAmplitude
            );
        }
        effectGuideActive = true;
        PlaceEffectGuideAtCapture(
        effectGuideCapturePoint,
        effectGuideTargetPoint,
        effectGuideFixedNormalWorld
        );
        if (captureNextStepWaveIgnoreOtherVisualOffsets)
            ClearTemporaryVisualOffsetsForCaptureWave();
        if (captureNextStepWaveUseDistanceBasedSpin)
        {
            // Capture開始時に姿勢をidentityへ戻すと回転が途切れて見えます。
            // 速度だけ整理し、これまでの回転姿勢は維持します。
            visualSpinVelocity = Vector3.zero;
            visualSpinKickVelocity = Vector3.zero;
        }
        else
        {
            AddVisualSpinImpulse(
            effectGuideLocalSide,
            captureNextStepBounceSpinImpulse * Mathf.Clamp01(
            effectGuideRuntimeAmplitude / Mathf.Max(captureNextStepBounceMaxOffset, .001f)
            )
            );
        }
    }
    void ClearTemporaryVisualOffsetsForCaptureWave()
    {
        visualLagOffset = Vector3.zero;
        visualCurrentOffset = Vector3.zero;
        visualTransitionOffset = Vector3.zero;
        visualPeakCurrentOffset = Vector3.zero;
        visualPeakStartOffset = Vector3.zero;
        visualPeakEndOffset = Vector3.zero;
        if (IsVisualEdgeDropActive())
            BeginVisualEdgeDropHandoffImmediately();
        else
            ResetVisualEdgeFloat(true);
        visualImpactTweenOffset = Vector3.zero;
        visualImpactTweenSquash = 0f;
        visualCaptureNextStepBounceSquash = 0f;
        visualImpactAge = -1f;
        visualPeakAge = -1f;
        visualImpactTween?.Kill();
    }
    Vector3 GetCaptureNextStepDistanceWaveOffset()
    {
        if (!effectGuideActive)
            return visualCaptureNextStepBounceOffset;
        Vector3 subjectAnchorWorld = GetBallVisualAnchorWorld();
        float rawSubjectTraveled = Vector3.Dot(
        subjectAnchorWorld - effectGuideCapturePoint,
        effectGuideAxisWorld
        );
        rawSubjectTraveled = Mathf.Max(0f, rawSubjectTraveled);
        float guideTraveled = effectGuideCurrentTraveled;
        float remaining = Mathf.Max(0f, effectGuideDistance - guideTraveled);
        bool movingWave = effectGuideState == EffectGuideState.MovingToTarget;
        if (effectGuideState == EffectGuideState.MovingToTarget)
        {
            guideTraveled = Mathf.Clamp(
            rawSubjectTraveled,
            0f,
            Mathf.Max(effectGuideDistance, .001f)
            );
            effectGuideCurrentTraveled = guideTraveled;
            effectGuideCurrentWorldPoint =
            effectGuideCapturePoint + effectGuideAxisWorld * guideTraveled;
            if (rawSubjectTraveled >= effectGuideDistance &&
                stopEffectGuideAtSlopeStopTarget)
            {
                guideTraveled = effectGuideDistance;
                effectGuideCurrentTraveled = guideTraveled;
                effectGuideCurrentWorldPoint = effectGuideTargetPoint;
                SetEffectGuideState(EffectGuideState.HoldingAtTarget);
                movingWave = false;
                if (!useLegacyExtremeCaptureVisuals ||
                    effectGuideHoldAtTargetSeconds <= .001f)
                {
                    BeginEffectGuideReturnToSubject();
                }
            }
        }
        else if (effectGuideState == EffectGuideState.HoldingAtTarget)
        {
            guideTraveled = effectGuideDistance;
            effectGuideCurrentTraveled = guideTraveled;
            effectGuideCurrentWorldPoint = effectGuideTargetPoint;
            movingWave = false;
            float holdSeconds = useLegacyExtremeCaptureVisuals
            ? effectGuideHoldAtTargetSeconds
            : Mathf.Min(effectGuideHoldAtTargetSeconds, .02f);
            if (RuntimeClock - effectGuideStateStartTime >= holdSeconds)
                BeginEffectGuideReturnToSubject();
        }
        if (effectGuideState == EffectGuideState.ReturningToSubject)
        {
            float t = Mathf.Clamp01(
            (RuntimeClock - effectGuideStateStartTime) /
            GetEffectGuideReturnDuration()
            );
            float smooth = GetReturnBaseProgress(t);
            Vector3 baseReturnPoint = Vector3.Lerp(
            effectGuideReturnStartWorldPoint,
            subjectAnchorWorld,
            smooth
            );
            effectGuideCurrentWorldPoint =
            baseReturnPoint + GetEffectGuideReturnCurveOffset(t);
            UpdateEffectGuideReturnVirtualTorque(
            effectGuideReturnPreviousWorldPoint,
            effectGuideCurrentWorldPoint,
            t
            );
            effectGuideReturnPreviousWorldPoint = effectGuideCurrentWorldPoint;
            // captureNextStepWaveReturnSeconds の復帰区間では、
            // subject に戻ることが目的なので縦波は消します。
            // 横カーブと回転だけでトリッキーさを出します。
            if (suppressVerticalWaveWhileReturningToSubject)
                visualCaptureNextStepBounceOffset = Vector3.zero;
            visualCaptureNextStepBounceSquash = 0f;
            UpdateEffectGuideTransformAtCurrentPoint();
            if (t >= 1f)
                CompleteEffectGuideReturnToSubject();
            return Vector3.zero;
        }
        UpdateEffectGuideTransformAtCurrentPoint();
        if (!movingWave)
        {
            visualCaptureNextStepBounceOffset = Vector3.zero;
            visualCaptureNextStepBounceSquash = 0f;
            return Vector3.zero;
        }
        UpdateCaptureNextStepWaveDistanceSpin(guideTraveled);
        float progress = Mathf.Clamp01(guideTraveled / effectGuideDistance);
        remaining = Mathf.Max(0f, effectGuideDistance - guideTraveled);
        float phase = GetCaptureNextStepWavePhase(progress, guideTraveled);
        float cosine = Mathf.Cos(phase);
        float sideCosine = Mathf.Cos(phase + Mathf.PI * .5f);
        float amplitude = GetDistanceTaperedCaptureWaveAmplitude(
        guideTraveled,
        remaining
        );
        float remainingCap = remaining * .35f;
        if (remaining <= captureNextStepWaveFadeDistance)
            amplitude = Mathf.Min(amplitude, remainingCap);
        float sideAmplitude = captureNextStepWaveSideAmplitude *
        GetCaptureNextStepWaveStartEnvelope(guideTraveled) *
        GetCaptureNextStepWaveTargetEnvelope(remaining);
        visualCaptureNextStepBounceOffset =
        effectGuideLocalUp *(cosine * amplitude) +
        effectGuideLocalSide *(sideCosine * sideAmplitude);
        visualCaptureNextStepBounceSquash =
        captureNextStepBounceSquash * Mathf.Abs(cosine) *
        Mathf.Clamp01(amplitude / Mathf.Max(captureNextStepWaveAmplitude, .001f)) * .5f;
        return visualCaptureNextStepBounceOffset;
    }
    void UpdateCaptureNextStepWaveDistanceSpin(float traveled)
    {
        if (!captureNextStepWaveUseDistanceBasedSpin)
        {
            effectGuidePreviousTraveled = traveled;
            return;
        }
        float deltaDistance = traveled - effectGuidePreviousTraveled;
        effectGuidePreviousTraveled = traveled;
        if (deltaDistance <= 0f)
            return;
        deltaDistance = Mathf.Min(
        deltaDistance,
        Mathf.Max(captureNextStepWaveMaxStepDistance, .001f)
        );
        float radius = Mathf.Max(visualSpinRadius, sphereRadius, .001f);
        float baseDegrees =
        deltaDistance / radius * Mathf.Rad2Deg *
        captureNextStepWaveSpinMultiplier *
        captureNextStepWaveSpinDirection;
        float verticalRollDegrees =
        baseDegrees *
        captureNextStepWaveVerticalRollMultiplier *
        captureNextStepWaveVerticalRollDirection;
        float horizontalSideDegrees =
        baseDegrees *
        captureNextStepWaveHorizontalSideSpinMultiplier *
        captureNextStepWaveHorizontalSideSpinDirection;
        Quaternion deltaRotation = Quaternion.identity;
        if (effectGuideRollAxisLocal.sqrMagnitude > 1e-6f &&
            Mathf.Abs(verticalRollDegrees) > .0001f)
        {
            deltaRotation =
            Quaternion.AngleAxis(
            verticalRollDegrees,
            effectGuideRollAxisLocal.normalized
            ) * deltaRotation;
        }
        if (effectGuideSideSpinAxisLocal.sqrMagnitude > 1e-6f &&
            Mathf.Abs(horizontalSideDegrees) > .0001f)
        {
            deltaRotation =
            Quaternion.AngleAxis(
            horizontalSideDegrees,
            effectGuideSideSpinAxisLocal.normalized
            ) * deltaRotation;
        }
        effectGuideSpinRotation =
        deltaRotation * effectGuideSpinRotation;
    }
    int GetCaptureNextStepWaveHalfWaveCount(float distance)
    {
        if (!captureNextStepWaveFitEndpoint)
            return 0;
        float halfWaveLength = Mathf.Max(captureNextStepWaveLength * .5f, .001f);
        return Mathf.Max(1, Mathf.RoundToInt(distance / halfWaveLength));
    }
    float GetCaptureNextStepWavePhase(float progress, float traveled)
    {
        if (captureNextStepWaveFitEndpoint && effectGuideHalfWaves > 0)
            return progress * Mathf.PI * effectGuideHalfWaves - Mathf.PI * .5f;
        float waveLength = Mathf.Max(captureNextStepWaveLength, .001f);
        return traveled / waveLength * Mathf.PI * 2f - Mathf.PI * .5f;
    }
    float GetDistanceTaperedCaptureWaveAmplitude(
    float traveled,
    float remaining
    )
    {
        float startEnvelope = GetCaptureNextStepWaveStartEnvelope(traveled);
        float targetEnvelope = GetCaptureNextStepWaveTargetEnvelope(remaining);
        return effectGuideRuntimeAmplitude * startEnvelope * targetEnvelope;
    }
    float GetCaptureNextStepWaveStartEnvelope(float traveled)
    {
        float fadeInDistance = Mathf.Max(captureNextStepWaveFadeInDistance, .001f);
        float t = Mathf.Clamp01(traveled / fadeInDistance);
        return t * t *(3f - 2f * t);
    }
    float GetCaptureNextStepWaveTargetEnvelope(float remaining)
    {
        float fadeDistance = Mathf.Max(captureNextStepWaveFadeDistance, .001f);
        float t = Mathf.Clamp01(remaining / fadeDistance);
        float smooth = t * t *(3f - 2f * t);
        return Mathf.Pow(smooth, captureNextStepWaveFadePower);
    }
    bool HasCaptureNextStepWaveReachedTarget()
    {
        return effectGuideActive &&
        (effectGuideState == EffectGuideState.HoldingAtTarget ||
        effectGuideState == EffectGuideState.ReturningToSubject);
    }
    void PlayCaptureNextStepBounceTween(
    Vector3 bounceDirection,
    Vector3 spinAxis,
    float amplitude,
    float squashAmount,
    float duration
    )
    {
        visualCaptureNextStepBounceTween?.Kill();
        visualCaptureNextStepWaveReturnTween?.Kill();
        effectGuideActive = false;
        visualCaptureNextStepBounceOffset = Vector3.zero;
        visualCaptureNextStepBounceSquash = 0f;
        float spinStrength = amplitude / Mathf.Max(captureNextStepBounceMaxOffset, .001f);
        AddVisualSpinImpulse(spinAxis, captureNextStepBounceSpinImpulse * spinStrength);
        float tweenAge = 0f;
        visualCaptureNextStepBounceTween = DOTween.To(
        () => tweenAge,
        age =>
        {
            tweenAge = age;
            float progress = Mathf.Clamp01(age / Mathf.Max(duration, .001f));
            float toTargetFade = 1f - progress;
            float phase = age * captureNextStepBounceFrequency * Mathf.PI * 2f;
            float decay = Mathf.Exp(-captureNextStepBounceDamping * age);
            float envelope = Mathf.Pow(toTargetFade, 1.35f) * decay;
            visualCaptureNextStepBounceOffset =
            bounceDirection *
            (amplitude * Mathf.Sin(phase) * envelope);
            visualCaptureNextStepBounceSquash =
            squashAmount *
            envelope *
            (.5f + .5f * Mathf.Cos(phase));
        }
        ,
        duration,
        duration
        )
        .SetEase(Ease.Linear)
        .SetUpdate(UpdateType.Fixed)
        .OnComplete(() => StopCaptureNextStepBounce(false));
    }
    void StopCaptureNextStepBounce(bool complete)
    {
        visualCaptureNextStepBounceTween?.Kill();
        if (!complete && effectGuideActive)
        {
            if (effectGuideState != EffectGuideState.ReturningToSubject)
                BeginEffectGuideReturnToSubject();
            visualCaptureNextStepBounceTween = null;
            return;
        }
        effectGuideActive = false;
        if (complete || captureNextStepWaveReturnSeconds <= .001f)
        {
            visualCaptureNextStepWaveReturnTween?.Kill();
            visualCaptureNextStepBounceOffset = Vector3.zero;
            visualCaptureNextStepBounceSquash = 0f;
            ResetEffectGuideRuntime();
            visualCaptureNextStepBounceTween = null;
            visualCaptureNextStepWaveReturnTween = null;
            if (captureNextStepWaveIgnoreOtherVisualOffsets)
                ClearTemporaryVisualOffsetsForCaptureWave();
            if (captureNextStepWaveFreezeSpin)
            {
                visualSpinVelocity = Vector3.zero;
                visualSpinKickVelocity = Vector3.zero;
                visualSpinRotation = Quaternion.identity;
            }
            return;
        }
        Vector3 startOffset = visualCaptureNextStepBounceOffset;
        float startSquash = visualCaptureNextStepBounceSquash;
        float t = 0f;
        visualCaptureNextStepWaveReturnTween?.Kill();
        visualCaptureNextStepWaveReturnTween = DOTween.To(
        () => t,
        value =>
        {
            t = value;
            float remain = 1f - Mathf.Clamp01(value);
            visualCaptureNextStepBounceOffset = startOffset * remain;
            visualCaptureNextStepBounceSquash = startSquash * remain;
        }
        ,
        1f,
        captureNextStepWaveReturnSeconds
        )
        .SetEase(Ease.OutCubic)
        .SetUpdate(UpdateType.Fixed)
        .OnComplete(() =>
        {
            visualCaptureNextStepBounceOffset = Vector3.zero;
            visualCaptureNextStepBounceSquash = 0f;
            ResetEffectGuideRuntime();
            visualCaptureNextStepBounceTween = null;
            visualCaptureNextStepWaveReturnTween = null;
            if (captureNextStepWaveFreezeSpin)
            {
                visualSpinVelocity = Vector3.zero;
                visualSpinKickVelocity = Vector3.zero;
                visualSpinRotation = Quaternion.identity;
            }
            if (captureNextStepWaveIgnoreOtherVisualOffsets)
                ClearTemporaryVisualOffsetsForCaptureWave();
        }
        );
    }
    void OnCollisionEnter(Collision collision) => TryCollisionVisualImpact(collision);
    void OnCollisionStay(Collision collision) => TryCollisionVisualImpact(collision);
    void TryCollisionVisualImpact(Collision collision)
    {
        if (collision.contactCount == 0)
            return;
        ContactPoint contact = collision.GetContact(0);
        float speed = Mathf.Max(0f, -Vector3.Dot(collision.relativeVelocity, contact.normal));
        TryStartVisualImpact(speed, contact.normal);
    }
    void ResetVisualOnly()
    {
        visualImpactAge = visualPeakAge = -1f;
        visualImpactTween?.Kill();
        visualCaptureNextStepBounceTween?.Kill();
        visualCaptureNextStepWaveReturnTween?.Kill();
        visualImpactTweenOffset = visualCaptureNextStepBounceOffset = Vector3.zero;
        visualImpactTweenSquash = visualCaptureNextStepBounceSquash = 0f;
        ResetEffectGuideRuntime();
        visualLagOffset = visualCurrentOffset = visualTransitionOffset = visualPeakCurrentOffset =
        visualPeakStartOffset = visualPeakEndOffset = visualSpinVelocity = visualSpinKickVelocity = Vector3.zero;
        slopeStopReachedTrickSpinVelocity = Vector3.zero;
        ResetVisualEdgeFloat(false);
        visualSpinCharge = visualStrongSpinTime = nextVisualWildSpinReleaseTime = 0f;
        nextVisualImpactTime = nextVisualTransitionTime = nextVisualPeakTime = 0f;
        visualSpinRotation = Quaternion.identity;
        effectGuideSpinRotation = Quaternion.identity;
        effectGuidePreviousTraveled = 0f;
        hasVisualGroundReference = false;
        visualGroundCollider = null;
        visualGroundPoint = Vector3.zero;
        visualGroundNormal = Vector3.up;
        pendingVisualPlaneTransition = default;
        if (!ballVisualTransform || ballVisualTransform == transform)
            return;
        ballVisualTransform.localPosition = visualRestPosition + visualMeshLocalOffset;
        ballVisualTransform.localRotation = visualRestRotation;
        ballVisualTransform.localScale = visualRestScale;
    }
    void TrackVisualPlaneTransition(RaycastHit hit)
    {
        Collider nextCollider = hit.collider;

        if (hasVisualGroundReference)
        {
            bool colliderChanged = nextCollider != visualGroundCollider;
            float previousSlope = Vector3.Angle(visualGroundNormal, Vector3.up);
            float nextSlope = Vector3.Angle(hit.normal, Vector3.up);
            bool crossedFlatToSlope =
                previousSlope <= VisualEdgeFlatMaxAngle &&
                nextSlope >= VisualEdgeSlopeMinAngle;

            // Planeが別Colliderの場合に加え、同じMeshCollider内で法線だけが
            // 平面→斜面へ変化する構造でも一度だけ遷移候補にします。
            if (colliderChanged || crossedFlatToSlope)
            {
                QueueVisualPlaneTransition(
                visualGroundPoint,
                visualGroundNormal,
                hit.point,
                hit.normal
                );
            }
        }

        visualGroundCollider = nextCollider;
        visualGroundPoint = hit.point;
        visualGroundNormal = hit.normal;
        hasVisualGroundReference = true;
    }
    void QueueVisualPlaneTransition(
    Vector3 previousPoint,
    Vector3 previousNormal,
    Vector3 nextPoint,
    Vector3 nextNormal
    )
    {
        pendingVisualPlaneTransition = new PendingVisualPlaneTransition
        {
            valid = true,
            previousPoint = previousPoint,
            previousNormal = previousNormal,
            nextPoint = nextPoint,
            nextNormal = nextNormal
        };
    }
    void CommitPendingVisualPlaneTransition()
    {
        if (!pendingVisualPlaneTransition.valid)
            return;
        PendingVisualPlaneTransition transition = pendingVisualPlaneTransition;
        pendingVisualPlaneTransition = default;
        StartVisualPlaneTransition(
        transition.previousPoint,
        transition.nextPoint,
        transition.nextNormal
        );

        bool startedEdgeFloat = StartVisualEdgeFloat(
        transition.previousPoint,
        transition.previousNormal,
        transition.nextPoint,
        transition.nextNormal
        );

        if (!startedEdgeFloat || !visualEdgeFloatReplacesPeakSlip)
        {
            StartVisualPeakSlip(
            transition.previousPoint,
            transition.previousNormal,
            transition.nextPoint,
            transition.nextNormal
            );
        }
    }
    void StartVisualPlaneTransition(Vector3 previousPoint, Vector3 nextPoint, Vector3 normal)
    {
        if (!ballVisualTransform || ballVisualTransform == transform || RuntimeClock < nextVisualTransitionTime)
            return;
        Vector3 delta = nextPoint - previousPoint;
        float distance = delta.magnitude;
        if (distance < visualTransitionMinDistance)
            return;
        Vector3 movement = Vector3.ProjectOnPlane(delta, normal);
        if (movement.sqrMagnitude <= 1e-6f)
            movement = Vector3.ProjectOnPlane(physicsBodyRigidbody ? physicsBodyRigidbody.velocity : headingDir, normal);
        if (movement.sqrMagnitude <= 1e-6f)
            return;
        Transform parent = ballVisualTransform.parent;
        Vector3 localBack = parent
        ? parent.InverseTransformDirection(-movement.normalized)
        : -movement.normalized;
        Vector3 localNormal = parent
        ? parent.InverseTransformDirection(normal).normalized
        : normal.normalized;
        float amount = Mathf.Min(
        (distance - visualTransitionMinDistance) * visualTransitionOffsetPerDistance,
        visualTransitionMaxOffset
        );
        if (amount <= .001f)
            return;
        float maxOffset = GetVisualMaxOffset();
        visualTransitionOffset += localBack * amount - localNormal *(amount * visualTransitionDownAmount);
        visualTransitionOffset = Vector3.ClampMagnitude(visualTransitionOffset, maxOffset);
        Vector3 localSpinAxis = Vector3.Cross(localNormal, -localBack);
        if (localSpinAxis.sqrMagnitude > 1e-6f)
        {
            AddVisualSpinImpulse(
            localSpinAxis,
            visualTransitionSpinPerDistance * amount
            );
            visualSpinCharge *= Mathf.Sqrt(visualSpinChargeRemain);
        }
        nextVisualTransitionTime = RuntimeClock + visualTransitionCooldown;
    }
    const float VisualEdgeFlatMaxAngle = 5f;
    const float VisualEdgeSlopeMinAngle = 2f;
    const float VisualEdgeMinDrop = .005f;
    const float VisualEdgeMinDownhillAlignment = 0f;
    const float VisualEdgeCooldown = .08f;
    const float VisualEdgeMinimumContactTime = .035f;

    bool IsVisualEdgeDropActive()
    {
        return visualEdgeDropState != VisualEdgeDropState.None;
    }

    bool StartVisualEdgeFloat(
    Vector3 previousPoint,
    Vector3 previousNormal,
    Vector3 nextPoint,
    Vector3 nextNormal
    )
    {
        if (!useVisualEdgeFloat ||
            !ballVisualTransform ||
            ballVisualTransform == transform ||
            RuntimeClock < nextVisualEdgeFloatTime)
        {
            return false;
        }

        // CaptureNextStep以降の停止演出とは重ねません。
        // この判定は停止状態を読むだけで、PhysicsBallや停止計画は変更しません。
        if (effectGuideActive ||
            slopeStopSegment.captureNextStepTriggered ||
            slopeStopSegment.state == SlopeStopState.Braking ||
            slopeStopSegment.state == SlopeStopState.Reached)
        {
            return false;
        }

        float previousSlope = Vector3.Angle(previousNormal, Vector3.up);
        float nextSlope = Vector3.Angle(nextNormal, Vector3.up);
        if (previousSlope > VisualEdgeFlatMaxAngle ||
            nextSlope < VisualEdgeSlopeMinAngle)
        {
            return false;
        }

        Vector3 sourceVelocity = physicsBodyRigidbody
            ? physicsBodyRigidbody.velocity
            : headingDir * maxGroundSpeed;

        // 平面へ押し込む法線速度は見た目軌道へ持ち込まず、
        // 平面上を滑っていた接線速度だけを初速度として使います。
        Vector3 launchVelocity = Vector3.ProjectOnPlane(
            sourceVelocity,
            previousNormal
        );

        if (launchVelocity.sqrMagnitude <= 1e-6f)
        {
            Vector3 fallbackDirection = Vector3.ProjectOnPlane(
                headingDir,
                previousNormal
            );
            if (fallbackDirection.sqrMagnitude <= 1e-6f)
                return false;

            float fallbackSpeed = physicsBodyRigidbody
                ? Vector3.ProjectOnPlane(
                    physicsBodyRigidbody.velocity,
                    previousNormal
                ).magnitude
                : maxGroundSpeed;

            launchVelocity = fallbackDirection.normalized *
                Mathf.Max(fallbackSpeed, maxGroundSpeed * .35f);
        }

        Vector3 travelWorld = launchVelocity.normalized;
        Vector3 downhillWorld = Vector3.ProjectOnPlane(
            Physics.gravity,
            nextNormal
        );
        float downhillAlignment = downhillWorld.sqrMagnitude > 1e-6f
            ? Vector3.Dot(travelWorld, downhillWorld.normalized)
            : -1f;
        float drop = previousPoint.y - nextPoint.y;

        bool movesIntoDownhill =
            downhillAlignment >= VisualEdgeMinDownhillAlignment;
        bool hasVisibleDrop = drop >= VisualEdgeMinDrop;
        if (!movesIntoDownhill && !hasVisibleDrop)
            return false;

        visualEdgeDropStartWorldPoint = ballVisualTransform.position;
        visualEdgeDropLaunchVelocityWorld = launchVelocity;
        visualEdgeDropGravityWorld =
            Physics.gravity * Mathf.Max(visualEdgeFloatGravityScale, .01f);
        visualEdgeDropLaunchNormalWorld =
            previousNormal.sqrMagnitude > 1e-6f
                ? previousNormal.normalized
                : Vector3.up;
        visualEdgeDropLandingPlaneNormalWorld =
            nextNormal.sqrMagnitude > 1e-6f
                ? nextNormal.normalized
                : Vector3.up;

        // BallVisual中心が次の面へ接触する仮想面です。
        visualEdgeDropLandingPlanePointWorld =
            nextPoint +
            visualEdgeDropLandingPlaneNormalWorld * sphereRadius;

        float maximumDuration = Mathf.Max(
            visualEdgeFloatMaximumDuration,
            VisualEdgeMinimumContactTime
        );

        if (!TrySolveBallisticPlaneContactTime(
                visualEdgeDropStartWorldPoint,
                visualEdgeDropLaunchVelocityWorld,
                visualEdgeDropGravityWorld,
                visualEdgeDropLandingPlanePointWorld,
                visualEdgeDropLandingPlaneNormalWorld,
                out float contactTime))
        {
            float fallbackDrop = Mathf.Max(
                drop,
                sphereRadius * Mathf.Sin(nextSlope * Mathf.Deg2Rad) * .35f,
                .015f
            );
            float gravityMagnitude = Mathf.Max(
                visualEdgeDropGravityWorld.magnitude,
                .1f
            );
            contactTime = Mathf.Sqrt(
                2f * fallbackDrop / gravityMagnitude
            );
        }

        visualEdgeDropContactTime = Mathf.Clamp(
            contactTime,
            VisualEdgeMinimumContactTime,
            maximumDuration
        );
        visualEdgeDropAge = 0f;
        visualEdgeDropHandoffAge = 0f;
        visualEdgeDropPreviousWorldPoint = visualEdgeDropStartWorldPoint;
        visualEdgeDropHandoffStartWorldPoint = Vector3.zero;
        visualEdgeDropHandoffStartVelocityWorld = Vector3.zero;
        visualEdgeDropState = VisualEdgeDropState.BallisticDrop;
        RefreshVisualEdgeFloatStartMarker(
            visualEdgeDropStartWorldPoint,
            visualEdgeDropLaunchNormalWorld
        );
        nextVisualEdgeFloatTime = RuntimeClock + VisualEdgeCooldown;

        if (visualEdgeFloatReplacesPeakSlip)
        {
            visualPeakAge = -1f;
            visualPeakCurrentOffset = Vector3.zero;
            visualPeakStartOffset = Vector3.zero;
            visualPeakEndOffset = Vector3.zero;
        }

        return true;
    }

    void BeginVisualEdgeDropHandoffImmediately()
    {
        if (visualEdgeDropState == VisualEdgeDropState.None ||
            visualEdgeDropState == VisualEdgeDropState.LandingHandoff)
        {
            return;
        }

        float elapsed = Mathf.Min(
            visualEdgeDropAge,
            Mathf.Max(
                visualEdgeDropContactTime,
                VisualEdgeMinimumContactTime
            )
        );

        Vector3 currentWorldPoint = GetVisualEdgeBallisticPoint(elapsed);
        Vector3 currentVelocity =
            visualEdgeDropLaunchVelocityWorld +
            visualEdgeDropGravityWorld * elapsed;

        BeginVisualEdgeDropHandoff(
            currentWorldPoint,
            currentVelocity
        );
    }

    void BeginVisualEdgeDropHandoff(
    Vector3 currentWorldPoint,
    Vector3 currentVelocityWorld
    )
    {
        RefreshVisualEdgeFloatEndMarker(
            currentWorldPoint,
            visualEdgeDropLandingPlaneNormalWorld
        );
        visualEdgeDropState = VisualEdgeDropState.LandingHandoff;
        visualEdgeDropHandoffAge = 0f;
        visualEdgeDropHandoffStartWorldPoint = currentWorldPoint;

        Vector3 tangentVelocity = Vector3.ProjectOnPlane(
            currentVelocityWorld,
            visualEdgeDropLandingPlaneNormalWorld
        );
        if (tangentVelocity.sqrMagnitude <= 1e-6f)
        {
            tangentVelocity = Vector3.ProjectOnPlane(
                physicsBodyRigidbody
                    ? physicsBodyRigidbody.velocity
                    : headingDir * maxGroundSpeed,
                visualEdgeDropLandingPlaneNormalWorld
            );
        }

        visualEdgeDropHandoffStartVelocityWorld = tangentVelocity;
        visualEdgeDropPreviousWorldPoint = currentWorldPoint;
    }

    bool StepVisualEdgeDrop(float dt)
    {
        if (!IsVisualEdgeDropActive())
            return false;

        dt = Mathf.Max(dt, 1e-6f);

        if (visualEdgeDropState == VisualEdgeDropState.BallisticDrop)
        {
            float duration = Mathf.Max(
                visualEdgeDropContactTime,
                VisualEdgeMinimumContactTime
            );
            visualEdgeDropAge = Mathf.Min(
                visualEdgeDropAge + dt,
                duration
            );
            float elapsed = visualEdgeDropAge;
            float progress = Mathf.Clamp01(elapsed / duration);

            Vector3 worldPoint = GetVisualEdgeBallisticPoint(elapsed);
            worldPoint = ClampVisualEdgeDropSeparation(worldPoint);

            Vector3 blendedNormal = Vector3.Slerp(
                visualEdgeDropLaunchNormalWorld,
                visualEdgeDropLandingPlaneNormalWorld,
                progress
            );
            UpdateVisualEdgeDropRoll(
                visualEdgeDropPreviousWorldPoint,
                worldPoint,
                blendedNormal,
                dt
            );
            ApplyVisualEdgeDropPose(worldPoint);
            visualEdgeDropPreviousWorldPoint = worldPoint;

            if (visualEdgeDropAge >= duration)
            {
                Vector3 contactVelocity =
                    visualEdgeDropLaunchVelocityWorld +
                    visualEdgeDropGravityWorld * duration;
                BeginVisualEdgeDropHandoff(
                    worldPoint,
                    contactVelocity
                );
            }

            return true;
        }

        float handoffDuration = Mathf.Max(
            visualEdgeFloatHandoffSeconds,
            .001f
        );
        visualEdgeDropHandoffAge = Mathf.Min(
            visualEdgeDropHandoffAge + dt,
            handoffDuration
        );
        float handoff01 = Mathf.Clamp01(
            visualEdgeDropHandoffAge / handoffDuration
        );

        Vector3 subjectAnchorWorld = GetBallVisualAnchorWorld();
        Vector3 subjectVelocity = physicsBodyRigidbody
            ? Vector3.ProjectOnPlane(
                physicsBodyRigidbody.velocity,
                visualEdgeDropLandingPlaneNormalWorld
            )
            : Vector3.zero;

        // 接触時の斜面接線速度を始端接線に使うHermite補間です。
        // 位置だけをLerpするより、斜面へ滑り込む速度の連続性が残ります。
        Vector3 startTangent =
            visualEdgeDropHandoffStartVelocityWorld *
            handoffDuration * .55f;
        Vector3 endTangent =
            subjectVelocity * handoffDuration * .20f;
        Vector3 worldHandoffPoint = CubicHermite(
            visualEdgeDropHandoffStartWorldPoint,
            subjectAnchorWorld,
            startTangent,
            endTangent,
            handoff01
        );
        worldHandoffPoint = ClampVisualEdgeDropSeparation(
            worldHandoffPoint
        );

        UpdateVisualEdgeDropRoll(
            visualEdgeDropPreviousWorldPoint,
            worldHandoffPoint,
            visualEdgeDropLandingPlaneNormalWorld,
            dt
        );
        ApplyVisualEdgeDropPose(worldHandoffPoint);
        visualEdgeDropPreviousWorldPoint = worldHandoffPoint;

        if (visualEdgeDropHandoffAge >= handoffDuration)
            CompleteVisualEdgeDropToSubject();

        return true;
    }

    Vector3 GetVisualEdgeBallisticPoint(float elapsed)
    {
        return visualEdgeDropStartWorldPoint +
               visualEdgeDropLaunchVelocityWorld * elapsed +
               .5f * visualEdgeDropGravityWorld * elapsed * elapsed;
    }

    Vector3 ClampVisualEdgeDropSeparation(Vector3 worldPoint)
    {
        Vector3 subjectAnchorWorld = GetBallVisualAnchorWorld();
        Vector3 offset = worldPoint - subjectAnchorWorld;

        // Inspector項目を増やさず、既存の球半径とVisual上限から
        // 暴走時だけ働く安全距離を作ります。
        float safetyDistance = Mathf.Max(
            GetVisualMaxOffset(),
            sphereRadius * .65f
        );
        if (offset.sqrMagnitude <= safetyDistance * safetyDistance)
            return worldPoint;

        return subjectAnchorWorld +
            Vector3.ClampMagnitude(offset, safetyDistance);
    }

    void UpdateVisualEdgeDropRoll(
    Vector3 previousWorldPoint,
    Vector3 currentWorldPoint,
    Vector3 surfaceNormalWorld,
    float dt
    )
    {
        Vector3 delta = currentWorldPoint - previousWorldPoint;
        float distance = delta.magnitude;
        if (distance <= 1e-6f)
            return;

        Vector3 moveDirection = delta / distance;
        Vector3 normal = surfaceNormalWorld.sqrMagnitude > 1e-6f
            ? surfaceNormalWorld.normalized
            : Vector3.up;
        Vector3 rollAxisWorld = Vector3.Cross(
            normal,
            moveDirection
        );
        if (rollAxisWorld.sqrMagnitude <= 1e-6f)
            return;
        rollAxisWorld.Normalize();

        Transform parent = ballVisualTransform.parent;
        Vector3 rollAxisLocal = parent
            ? parent.InverseTransformDirection(rollAxisWorld)
            : rollAxisWorld;
        if (rollAxisLocal.sqrMagnitude <= 1e-6f)
            return;
        rollAxisLocal.Normalize();

        float degrees = distance /
            Mathf.Max(visualSpinRadius, sphereRadius, .001f) *
            Mathf.Rad2Deg;
        float spinLimit = useLegacyExtremeCaptureVisuals
            ? visualSpinMaxDegreesPerSecond
            : Mathf.Min(
                visualSpinMaxDegreesPerSecond,
                safeVisualSpinDegreesPerSecond
            );
        degrees = Mathf.Min(
            degrees,
            Mathf.Max(spinLimit, 0f) * dt
        );

        visualSpinRotation = Quaternion.AngleAxis(
            degrees,
            rollAxisLocal
        ) * visualSpinRotation;
    }

    void ApplyVisualEdgeDropPose(Vector3 worldPoint)
    {
        ballVisualTransform.position = worldPoint;
        ballVisualTransform.localRotation =
            visualRestRotation * visualSpinRotation;
        ballVisualTransform.localScale = visualRestScale;
    }

    void CompleteVisualEdgeDropToSubject()
    {
        if (ballVisualTransform && ballVisualTransform != transform)
        {
            ballVisualTransform.position = GetBallVisualAnchorWorld();
            ballVisualTransform.localRotation =
                visualRestRotation * visualSpinRotation;
            ballVisualTransform.localScale = visualRestScale;
        }

        // 次の通常追従フレームで以前のオフセットが復活しないよう、
        // 滑落中に競合し得る一時オフセットだけを整理します。
        visualLagOffset = Vector3.zero;
        visualCurrentOffset = Vector3.zero;
        visualTransitionOffset = Vector3.zero;
        visualPeakCurrentOffset = Vector3.zero;
        visualPeakStartOffset = Vector3.zero;
        visualPeakEndOffset = Vector3.zero;

        ResetVisualEdgeFloat(true);
    }

    void ResetVisualEdgeFloat(bool preserveCurrentRoll)
    {
        // 回転はvisualSpinRotationへ距離ベースで直接積算済みなので、
        // preserveCurrentRollの値にかかわらず姿勢を維持します。
        _ = preserveCurrentRoll;

        visualEdgeDropState = VisualEdgeDropState.None;
        visualEdgeDropStartWorldPoint = Vector3.zero;
        visualEdgeDropLaunchVelocityWorld = Vector3.zero;
        visualEdgeDropGravityWorld = Vector3.zero;
        visualEdgeDropLaunchNormalWorld = Vector3.up;
        visualEdgeDropLandingPlanePointWorld = Vector3.zero;
        visualEdgeDropLandingPlaneNormalWorld = Vector3.up;
        visualEdgeDropPreviousWorldPoint = Vector3.zero;
        visualEdgeDropHandoffStartWorldPoint = Vector3.zero;
        visualEdgeDropHandoffStartVelocityWorld = Vector3.zero;
        visualEdgeDropAge = 0f;
        visualEdgeDropContactTime = 0f;
        visualEdgeDropHandoffAge = 0f;
    }

    static Vector3 CubicHermite(
    Vector3 start,
    Vector3 end,
    Vector3 startTangent,
    Vector3 endTangent,
    float t
    )
    {
        t = Mathf.Clamp01(t);
        float t2 = t * t;
        float t3 = t2 * t;
        float h00 = 2f * t3 - 3f * t2 + 1f;
        float h10 = t3 - 2f * t2 + t;
        float h01 = -2f * t3 + 3f * t2;
        float h11 = t3 - t2;
        return h00 * start +
               h10 * startTangent +
               h01 * end +
               h11 * endTangent;
    }

    static bool TrySolveBallisticPlaneContactTime(
    Vector3 startPoint,
    Vector3 startVelocity,
    Vector3 gravity,
    Vector3 planePoint,
    Vector3 planeNormal,
    out float time
    )
    {
        time = 0f;

        Vector3 normal = planeNormal.sqrMagnitude > 1e-6f
            ? planeNormal.normalized
            : Vector3.up;

        float a = .5f * Vector3.Dot(gravity, normal);
        float b = Vector3.Dot(startVelocity, normal);
        float c = Vector3.Dot(startPoint - planePoint, normal);

        const float minimumTime = .02f;

        if (Mathf.Abs(a) <= 1e-6f)
        {
            if (Mathf.Abs(b) <= 1e-6f)
                return false;

            float linearTime = -c / b;
            if (linearTime <= minimumTime)
                return false;

            time = linearTime;
            return true;
        }

        float discriminant = b * b - 4f * a * c;
        if (discriminant < 0f)
            return false;

        float sqrt = Mathf.Sqrt(discriminant);
        float denominator = 2f * a;
        float t0 = (-b - sqrt) / denominator;
        float t1 = (-b + sqrt) / denominator;

        float best = float.PositiveInfinity;
        if (t0 > minimumTime)
            best = t0;
        if (t1 > minimumTime)
            best = Mathf.Min(best, t1);

        if (float.IsInfinity(best))
            return false;

        time = best;
        return true;
    }

    void StartVisualPeakSlip(
    Vector3 previousPoint,
    Vector3 previousNormal,
    Vector3 nextPoint,
    Vector3 nextNormal
    )
    {
        if (!visualPeakSlip ||
            !ballVisualTransform ||
            ballVisualTransform == transform ||
            RuntimeClock < nextVisualPeakTime)
            return;
        float distance = Vector3.Distance(previousPoint, nextPoint);
        float normalAngle = Vector3.Angle(previousNormal, nextNormal);
        float drop = previousPoint.y - nextPoint.y;
        float previousSlope = Vector3.Angle(previousNormal, Vector3.up);
        float nextSlope = Vector3.Angle(nextNormal, Vector3.up);
        bool containsSlope = Mathf.Max(previousSlope, nextSlope) >=
        visualPeakMinSlopeAngle;
        bool isFoldedPeak = normalAngle >= visualPeakMinNormalAngle;
        bool isDownwardTransition = drop >= visualPeakMinDrop;
        if (!containsSlope || (!isFoldedPeak && !isDownwardTransition))
            return;
        Vector3 inertiaWorld = Vector3.ProjectOnPlane(
        physicsBodyRigidbody ? physicsBodyRigidbody.velocity : headingDir,
        previousNormal
        );
        Vector3 pointDelta = nextPoint - previousPoint;
        if (inertiaWorld.sqrMagnitude <= 1e-6f)
            inertiaWorld = Vector3.ProjectOnPlane(pointDelta, previousNormal);
        if (inertiaWorld.sqrMagnitude <= 1e-6f)
            inertiaWorld = Vector3.ProjectOnPlane(headingDir, previousNormal);
        if (inertiaWorld.sqrMagnitude <= 1e-6f)
            return;
        Vector3 downhillWorld = Vector3.ProjectOnPlane(
        Physics.gravity,
        nextNormal
        );
        if (downhillWorld.sqrMagnitude <= 1e-6f)
            downhillWorld = Vector3.ProjectOnPlane(pointDelta, nextNormal);
        if (downhillWorld.sqrMagnitude <= 1e-6f)
            downhillWorld = Vector3.ProjectOnPlane(inertiaWorld, nextNormal);
        if (downhillWorld.sqrMagnitude <= 1e-6f)
            return;
        Transform parent = ballVisualTransform.parent;
        Vector3 localBack = ToVisualLocalDirection(parent, -inertiaWorld);
        Vector3 localDownhill = ToVisualLocalDirection(parent, downhillWorld);
        Vector3 localNextNormal = ToVisualLocalDirection(parent, nextNormal);
        float maxOffset = GetVisualMaxOffset();
        float amount = Mathf.Min(
        (distance - visualPeakMinDistance) * visualPeakOffsetPerDistance,
        visualPeakMaxOffset,
        maxOffset
        );
        if (amount <= .001f)
            return;
        visualPeakStartOffset = Vector3.ClampMagnitude(
        visualPeakCurrentOffset +
        localBack *(amount * visualPeakInertiaBackAmount),
        maxOffset
        );
        visualPeakEndOffset = Vector3.ClampMagnitude(
        localDownhill *(amount * visualPeakSlopePullAmount) -
        localNextNormal *(amount * visualPeakDownAmount),
        maxOffset
        );
        visualPeakAge = 0f;
        nextVisualPeakTime = RuntimeClock + visualPeakCooldown;
        EstimateVisualPeakSupport(
        previousPoint,
        nextPoint,
        previousNormal,
        inertiaWorld,
        normalAngle,
        out visualPeakCurvatureRadius,
        out visualPeakSupport01,
        out visualPeakSupportLoss01
        );
        float curvatureSlip = visualCurvatureContact
        ? Mathf.Lerp(1f, visualCurvatureSlipMultiplier, visualPeakSupportLoss01)
        : 1f;
        visualPeakStartOffset = Vector3.ClampMagnitude(
        visualPeakStartOffset * curvatureSlip,
        maxOffset
        );
        visualPeakEndOffset = Vector3.ClampMagnitude(
        visualPeakEndOffset * curvatureSlip,
        maxOffset
        );
        Vector3 worldMainAxis = Vector3.Cross(nextNormal, inertiaWorld);
        Vector3 worldCrossAxis = Vector3.Cross(inertiaWorld, downhillWorld);
        AddVisualSpinImpulse(
        ToVisualLocalDirection(parent, worldMainAxis),
        visualPeakSpinImpulse * amount / Mathf.Max(maxOffset, .001f)
        );
        AddVisualSpinImpulse(
        ToVisualLocalDirection(parent, worldCrossAxis),
        visualPeakCrossSpinImpulse * amount / Mathf.Max(maxOffset, .001f)
        );
    }
    void EstimateVisualPeakSupport(
    Vector3 previousPoint,
    Vector3 nextPoint,
    Vector3 previousNormal,
    Vector3 inertiaWorld,
    float normalAngle,
    out float radius,
    out float support01,
    out float supportLoss01
    )
    {
        float arcLength = Mathf.Max(Vector3.Distance(previousPoint, nextPoint), .001f);
        float radians = Mathf.Max(normalAngle * Mathf.Deg2Rad, .01f);
        radius = Mathf.Clamp(
        arcLength / radians,
        visualCurvatureMinRadius,
        visualCurvatureMaxRadius
        );
        float speed = Vector3.ProjectOnPlane(inertiaWorld, previousNormal).magnitude;
        float gravity = Mathf.Max(Physics.gravity.magnitude, .001f);
        float gravityIntoSurface = -Vector3.Dot(Physics.gravity, previousNormal);
        float normalLoadPerMass = gravityIntoSurface - speed * speed / radius;
        support01 = Mathf.Clamp01(normalLoadPerMass / gravity);
        supportLoss01 = Mathf.InverseLerp(
        visualLowSupportStart,
        visualNearDetachSupport,
        support01
        );
    }
    Vector3 UpdateVisualPeakSlip(float dt)
    {
        if (visualPeakAge < 0f)
            return Vector3.zero;
        float take = Mathf.Max(visualPeakTakeSeconds, .0001f);
        float holdEnd = take + Mathf.Max(visualPeakHoldSeconds, 0f);
        float age = visualPeakAge;
        Vector3 result;
        if (age < take)
        {
            float t = age / take;
            t = t * t *(3f - 2f * t);
            result = Vector3.Lerp(
            visualPeakStartOffset,
            visualPeakEndOffset,
            t
            );
        }
        else if (age < holdEnd)
        {
            result = visualPeakEndOffset;
        }
        else
        {
            float returnTime = age - holdEnd;
            float fade = Mathf.Exp(
            -visualPeakReturnSharpness * returnTime
            );
            result = visualPeakEndOffset * fade;
            if (result.sqrMagnitude <= 1e-6f)
            {
                visualPeakAge = -1f;
                visualPeakCurrentOffset = Vector3.zero;
                return Vector3.zero;
            }
        }
        visualPeakAge += dt;
        visualPeakCurrentOffset = result;
        return result;
    }
    float GetVisualMaxOffset()
    {
        return Mathf.Min(
        Mathf.Max(visualMaxSeparation, 0f),
        Mathf.Max(sphereRadius * .45f, 0f)
        );
    }
    static Vector3 ToVisualLocalDirection(Transform parent, Vector3 worldDirection)
    {
        if (worldDirection.sqrMagnitude <= 1e-6f)
            return Vector3.zero;
        Vector3 localDirection = parent
        ? parent.InverseTransformDirection(worldDirection)
        : worldDirection;
        return localDirection.sqrMagnitude > 1e-6f
        ? localDirection.normalized
        : Vector3.zero;
    }
    void StepCaptureNextStepWaveOnly(float dt)
    {
        dt = Mathf.Max(dt, 1e-6f);
        Vector3 waveOffset = GetCaptureNextStepDistanceWaveOffset();
        Vector3 anchor = visualRestPosition + visualMeshLocalOffset;
        Vector3 guideOffset = Vector3.zero;

        if (ballVisualFollowsEffectGuideWorld && effectGuideActive)
        {
            Vector3 guideLocalPoint =
            ToBallVisualParentLocalPoint(effectGuideCurrentWorldPoint);
            guideOffset = guideLocalPoint - anchor;
        }

        if (!useLegacyExtremeCaptureVisuals)
        {
            float safeSeparation = Mathf.Min(
            Mathf.Max(safeCaptureVisualSeparation, .01f),
            Mathf.Max(GetVisualMaxOffset(), .01f)
            );
            guideOffset = Vector3.ClampMagnitude(guideOffset, safeSeparation);
            waveOffset = Vector3.ClampMagnitude(
            waveOffset,
            Mathf.Min(safeCaptureWaveAmplitude, safeSeparation)
            );
            Vector3 combinedOffset = Vector3.ClampMagnitude(
            guideOffset + waveOffset,
            safeSeparation
            );
            ballVisualTransform.localPosition = anchor + combinedOffset;
        }
        else
        {
            ballVisualTransform.localPosition = anchor + guideOffset + waveOffset;
        }

        float visualDeformation = visualCaptureNextStepBounceSquash;
        ballVisualTransform.localScale = Vector3.Scale(
        visualRestScale,
        new Vector3(
        1f + visualDeformation * .5f,
        1f - visualDeformation,
        1f + visualDeformation * .5f
        )
        );
        Quaternion waveRotation = effectGuideActive &&
        captureNextStepWaveUseDistanceBasedSpin
        ? effectGuideSpinRotation
        : Quaternion.identity;

        // 通常走行からCapture区間へ入っても回転姿勢を連続させます。
        ballVisualTransform.localRotation =
        visualRestRotation * visualSpinRotation * waveRotation;
    }
    void StepVisualOnly(float dt)
    {
        if (!ballVisualTransform || ballVisualTransform == transform)
            return;

        dt = Mathf.Max(dt, 1e-6f);
        Transform parent = ballVisualTransform.parent;
        bool captureWaveOnly =
            captureNextStepUseDistanceCosineWave &&
            captureNextStepWaveIgnoreOtherVisualOffsets &&
            (effectGuideActive || visualCaptureNextStepWaveReturnTween != null);

        // 流星型滑落中はBallVisualのワールド位置を専用軌道が管理します。
        // Captureが始まった場合も、短い斜面接線Handoffを完了してから
        // Capture専用表示へ切り替えます。
        if (IsVisualEdgeDropActive())
        {
            if (captureWaveOnly)
                BeginVisualEdgeDropHandoffImmediately();

            StepVisualEdgeDrop(dt);
            return;
        }

        if (captureWaveOnly)
        {
            StepCaptureNextStepWaveOnly(dt);
            return;
        }

        Vector3 velocity = physicsBodyRigidbody
            ? physicsBodyRigidbody.velocity
            : Vector3.zero;
        Vector3 localVelocity = parent
            ? parent.InverseTransformDirection(velocity)
            : velocity;
        float maxOffset = GetVisualMaxOffset();
        Vector3 desiredLag = Vector3.ClampMagnitude(
            -localVelocity * visualVelocityLag,
            maxOffset
        );
        float lagFollow = 1f - Mathf.Exp(
            -visualCatchUpSharpness * dt
        );
        visualLagOffset = Vector3.Lerp(
            visualLagOffset,
            desiredLag,
            lagFollow
        );

        float totalSpinSpeed = UpdateVisualSpin(parent, dt);
        UpdateTimedRecentering(totalSpinSpeed, dt);
        Vector3 peakSlipOffset = UpdateVisualPeakSlip(dt);
        float peakPopStretch = 0f;
        Vector3 ballisticOffset = GetCaptureNextStepDistanceWaveOffset();

        if (!useLegacyExtremeCaptureVisuals)
        {
            ballisticOffset = Vector3.ClampMagnitude(
                ballisticOffset,
                Mathf.Min(
                    Mathf.Max(safeCaptureWaveAmplitude, .01f),
                    Mathf.Max(safeCaptureVisualSeparation, .01f)
                )
            );
        }

        float transitionFollow = 1f - Mathf.Exp(
            -visualTransitionCatchUpSharpness * dt
        );
        visualTransitionOffset = Vector3.Lerp(
            visualTransitionOffset,
            Vector3.zero,
            transitionFollow
        );

        if ((slopeStopSegment.state == SlopeStopState.Reached ||
            HasCaptureNextStepWaveReachedTarget()) &&
            (visualCaptureNextStepBounceTween != null || effectGuideActive))
        {
            StopCaptureNextStepBounce(false);
        }

        bool captureWaveBallistic =
            captureNextStepUseDistanceCosineWave &&
            (effectGuideActive || visualCaptureNextStepWaveReturnTween != null);
        bool captureWaveDominatesVisual =
            captureWaveBallistic &&
            captureNextStepWaveIgnoreOtherVisualOffsets;

        if (captureWaveDominatesVisual)
            ClearTemporaryVisualOffsetsForCaptureWave();

        Vector3 wobble = captureWaveBallistic
            ? Vector3.zero
            : visualImpactTweenOffset +
              visualCaptureNextStepBounceOffset;
        float squash = captureWaveBallistic
            ? visualCaptureNextStepBounceSquash
            : visualImpactTweenSquash +
              visualCaptureNextStepBounceSquash;

        if (!useDotweenVisualBounce && visualImpactAge >= 0f)
        {
            float phase = visualImpactAge *
                visualImpactFrequency * Mathf.PI * 2f;
            float decay = Mathf.Exp(
                -visualImpactDamping * visualImpactAge
            );
            wobble +=
                (visualImpactUp +
                 visualImpactSide * visualImpactSideAmount) *
                (visualImpactAmplitude * Mathf.Sin(phase) * decay);
            squash +=
                visualImpactSquash *
                (visualImpactAmplitude /
                 Mathf.Max(visualImpactMaxOffset, .001f)) *
                decay * (.5f + .5f * Mathf.Cos(phase));
            visualImpactAge += dt;
            if (decay < .01f)
                visualImpactAge = -1f;
        }

        Vector3 anchor = visualRestPosition + visualMeshLocalOffset;
        Vector3 observedOffset =
            ballVisualTransform.localPosition -
            anchor -
            ballisticOffset;

        if (!deterministicReplay &&
            (observedOffset - visualCurrentOffset).sqrMagnitude > 1e-6f)
        {
            visualCurrentOffset = Vector3.ClampMagnitude(
                observedOffset,
                maxOffset
            );
        }

        Vector3 desiredOffset = captureWaveDominatesVisual
            ? Vector3.zero
            : Vector3.ClampMagnitude(
                visualLagOffset +
                wobble +
                visualTransitionOffset +
                peakSlipOffset,
                maxOffset
            );

        bool timedReturn =
            visualStrongSpinTime >= visualRecenteringAfterSeconds;
        if (timedReturn)
            desiredOffset *= visualRecenteringOffsetRemain;

        float correction = timedReturn
            ? visualTimedRecenteringSharpness
            : visualCatchUpSharpness;
        float follow = 1f - Mathf.Exp(-correction * dt);
        visualCurrentOffset = captureWaveDominatesVisual
            ? Vector3.zero
            : Vector3.Lerp(
                visualCurrentOffset,
                desiredOffset,
                follow
            );

        Vector3 finalVisualOffset =
            visualCurrentOffset + ballisticOffset;
        if (!useLegacyExtremeCaptureVisuals)
        {
            finalVisualOffset = Vector3.ClampMagnitude(
                finalVisualOffset,
                Mathf.Max(safeCaptureVisualSeparation, .01f)
            );
        }

        ballVisualTransform.localPosition =
            anchor + finalVisualOffset;

        float visualDeformation = squash - peakPopStretch;
        ballVisualTransform.localScale = Vector3.Scale(
            visualRestScale,
            new Vector3(
                1f + visualDeformation * .5f,
                1f - visualDeformation,
                1f + visualDeformation * .5f
            )
        );
        ballVisualTransform.localRotation =
            captureWaveDominatesVisual &&
            captureNextStepWaveFreezeSpin
                ? visualRestRotation
                : visualRestRotation * visualSpinRotation;
    }
    void UpdateTimedRecentering(float spinSpeed, float dt)
    {
        if (spinSpeed >= visualRecenteringSpinSpeed)
            visualStrongSpinTime += dt;
        else
            visualStrongSpinTime = 0f;
    }
    void HandleSlopeStopReachedVisualTransition(SlopeStopState previousState)
    {
        if (previousState == SlopeStopState.Reached ||
            slopeStopSegment.state != SlopeStopState.Reached)
        {
            return;
        }

        TriggerSlopeStopReachedTrickSpin();
    }

    [ContextMenu("Test Slope Stop Reached Trick Spin")]
    void TriggerSlopeStopReachedTrickSpin()
    {
        if (!useSlopeStopReachedTrickSpin ||
            !ballVisualTransform ||
            ballVisualTransform == transform)
        {
            return;
        }

        Transform parent = ballVisualTransform.parent;

        Vector3 upWorld = groundNormal.sqrMagnitude > 1e-6f
            ? groundNormal.normalized
            : slopeStopSegment.stopWorldNormal.sqrMagnitude > 1e-6f
                ? slopeStopSegment.stopWorldNormal.normalized
                : Vector3.up;

        Vector3 forwardWorld = GetCurrentSlopeStopPlanAxis(
            headingDir.sqrMagnitude > 1e-6f
                ? headingDir
                : Vector3.forward
        );
        forwardWorld = Vector3.ProjectOnPlane(forwardWorld, upWorld);
        if (forwardWorld.sqrMagnitude <= 1e-6f)
            forwardWorld = Vector3.ProjectOnPlane(headingDir, upWorld);
        if (forwardWorld.sqrMagnitude <= 1e-6f)
            forwardWorld = Vector3.forward;
        forwardWorld.Normalize();

        Vector3 upLocal = parent
            ? parent.InverseTransformDirection(upWorld)
            : upWorld;
        Vector3 forwardLocal = parent
            ? parent.InverseTransformDirection(forwardWorld)
            : forwardWorld;

        if (upLocal.sqrMagnitude <= 1e-6f)
            upLocal = Vector3.up;
        else
            upLocal.Normalize();

        if (forwardLocal.sqrMagnitude <= 1e-6f)
            forwardLocal = Vector3.forward;
        else
            forwardLocal.Normalize();

        float directionSource = slopeStopReachedUsesLastHandleDirection
            ? lastPlayerTurnSign
            : slopeStopReachedSpinDirection;
        float directionSign = Mathf.Abs(directionSource) > .001f
            ? Mathf.Sign(directionSource)
            : 1f;

        // 到達前のWildSpinを少しだけ残し、極限点専用の横回転を主役にします。
        visualSpinVelocity *= slopeStopReachedPreviousSpinRemain;
        visualSpinKickVelocity *= slopeStopReachedPreviousSpinRemain;

        slopeStopReachedTrickSpinVelocity =
            upLocal * (slopeStopReachedSideSpinImpulse * directionSign) +
            forwardLocal * (slopeStopReachedBankSpinImpulse * -directionSign);
    }

    float UpdateVisualSpin(Transform parent, float dt)
    {
        Vector3 normal = isGrounded ? groundNormal : Vector3.up;
        Vector3 rollingVelocity = Vector3.ProjectOnPlane(physicsBodyRigidbody ? physicsBodyRigidbody.velocity : Vector3.zero, normal);
        float speed = rollingVelocity.magnitude;
        float radius = Mathf.Max(visualSpinRadius, .001f);
        Vector3 targetWorldSpin = speed > 1e-6f
        ? Vector3.Cross(normal, rollingVelocity.normalized) *(speed / radius * Mathf.Rad2Deg)
        : Vector3.zero;
        Vector3 targetLocalSpin = parent
        ? parent.InverseTransformDirection(targetWorldSpin)
        : targetWorldSpin;
        visualSpinCharge = Mathf.MoveTowards(
        visualSpinCharge,
        Mathf.Clamp01(speed / Mathf.Max(maxGroundSpeed, .001f)),
        visualSpinChargeBuild * dt
        );
        if (visualWildSpin && speed > .05f)
        {
            float pulse = 1f + Mathf.Sin(
            RuntimeClock * visualWildSpinPulseFrequency * Mathf.PI * 2f
            ) * visualWildSpinPulseAmount *(.25f + .75f * visualSpinCharge);
            float wildMultiplier = useLegacyExtremeCaptureVisuals
            ? visualWildSpinMultiplier
            : Mathf.Min(visualWildSpinMultiplier, 1.25f);
            float safePulse = useLegacyExtremeCaptureVisuals
            ? pulse
            : Mathf.Lerp(1f, pulse, .25f);
            targetLocalSpin *= wildMultiplier * safePulse;
        }
        float follow = 1f - Mathf.Exp(
        -(speed > .05f ? visualSpinFollowSharpness : visualSpinDamping) * dt
        );
        visualSpinVelocity = Vector3.Lerp(visualSpinVelocity, targetLocalSpin, follow);
        float kickFade = 1f - Mathf.Exp(-visualSpinDamping * dt);
        visualSpinKickVelocity = Vector3.Lerp(
        visualSpinKickVelocity,
        Vector3.zero,
        kickFade
        );

        float reachedTrickFade = 1f - Mathf.Exp(
            -Mathf.Max(slopeStopReachedTrickSpinDamping, .1f) * dt
        );
        slopeStopReachedTrickSpinVelocity = Vector3.Lerp(
            slopeStopReachedTrickSpinVelocity,
            Vector3.zero,
            reachedTrickFade
        );

        float spinLimit = useLegacyExtremeCaptureVisuals
        ? visualSpinMaxDegreesPerSecond
        : Mathf.Min(visualSpinMaxDegreesPerSecond, safeVisualSpinDegreesPerSecond);

        bool targetTrickActive =
            slopeStopReachedTrickSpinVelocity.sqrMagnitude > .01f;
        if (targetTrickActive)
        {
            spinLimit = Mathf.Max(
                spinLimit,
                slopeStopReachedTrickMaxDegreesPerSecond
            );
        }

        Vector3 totalSpin = Vector3.ClampMagnitude(
        visualSpinVelocity +
        visualSpinKickVelocity +
        slopeStopReachedTrickSpinVelocity,
        spinLimit
        );
        float degrees = totalSpin.magnitude * dt;
        if (degrees > 1e-5f)
            visualSpinRotation = Quaternion.AngleAxis(
            degrees,
            totalSpin.normalized
            ) * visualSpinRotation;
        return totalSpin.magnitude;
    }
    void AddVisualSpinImpulse(Vector3 axis, float impulse)
    {
        if (axis.sqrMagnitude <= 1e-6f || impulse <= 0f)
            return;
        int impulseId = visualImpulseSequence++;
        float charge = visualSpinCharge;
        Vector3 mainAxis = visualWildSpin
        ? GetWildSpinAxis(axis, impulseId, 0)
        : axis.normalized;
        float multiplier = visualWildSpin
        ? (useLegacyExtremeCaptureVisuals
        ? visualWildSpinMultiplier
        : Mathf.Min(visualWildSpinMultiplier, 1.25f))
        : 1f;
        visualSpinKickVelocity += mainAxis * impulse * multiplier *(.35f + charge);
        if (visualWildSpin &&
            charge >= visualWildSpinReleaseThreshold &&
            RuntimeClock >= nextVisualWildSpinReleaseTime)
        {
            Vector3 secondaryAxis = Vector3.Cross(mainAxis, Vector3.up);
            if (secondaryAxis.sqrMagnitude <= 1e-6f)
                secondaryAxis = Vector3.Cross(mainAxis, Vector3.forward);
            visualSpinKickVelocity += GetWildSpinAxis(secondaryAxis, impulseId, 1) *
            (visualWildSpinReleaseImpulse * charge);
            nextVisualWildSpinReleaseTime =
            RuntimeClock + visualWildSpinReleaseCooldown;
        }
        float spinLimit = useLegacyExtremeCaptureVisuals
        ? visualSpinMaxDegreesPerSecond
        : Mathf.Min(visualSpinMaxDegreesPerSecond, safeVisualSpinDegreesPerSecond);
        visualSpinKickVelocity = Vector3.ClampMagnitude(
        visualSpinKickVelocity,
        spinLimit
        );
    }
    Vector3 GetWildSpinAxis(Vector3 axis, int impulseId, int channel)
    {
        axis = axis.sqrMagnitude > 1e-6f ? axis.normalized : Vector3.right;
        if (!visualWildSpin || visualWildSpinAxisJitter <= 0f)
            return axis;
        Vector3 noise = new Vector3(
        Deterministic01(deterministicVisualSeed, impulseId, channel * 3 + 0) * 2f - 1f,
        Deterministic01(deterministicVisualSeed, impulseId, channel * 3 + 1) * 2f - 1f,
        Deterministic01(deterministicVisualSeed, impulseId, channel * 3 + 2) * 2f - 1f
        );
        float axisJitter = useLegacyExtremeCaptureVisuals
        ? visualWildSpinAxisJitter
        : Mathf.Min(visualWildSpinAxisJitter, .12f);
        Vector3 result = axis + noise * axisJitter;
        return result.sqrMagnitude > 1e-6f ? result.normalized : axis;
    }
    static float Deterministic01(int seed, int sequence, int channel)
    {
        unchecked
        {
            uint value = (uint)seed;
            value ^= (uint)sequence * 0x9E3779B9u;
            value ^= (uint)channel * 0x85EBCA6Bu;
            value ^= value >> 16;
            value *= 0x7FEB352Du;
            value ^= value >> 15;
            value *= 0x846CA68Bu;
            value ^= value >> 16;
            return(value & 0x00FFFFFFu) / 16777215f;
        }
    }
    void FixedUpdate()
    {
        if (!simulationReady || !physicsBodyRigidbody)
            return;
        if (deterministicReplay)
            deterministicClock += Time.fixedDeltaTime;
        ApplyQueuedTurnAtFixedBoundary();
        UpdateAuthoritativeGroundState();
        CommitPendingVisualPlaneTransition();
        if (ShouldRecoverFromFall())
        {
            RecoverFromFall();
            return;
        }
        SlopeStopState slopeStateBeforeSolve = slopeStopSegment.state;
        if (isGrounded)
            SolveGround(headingDir);
        else
            SolveAir(headingDir);
        HandleSlopeStopReachedVisualTransition(slopeStateBeforeSolve);
        if (deterministicReplay)
            StepVisualOnly(Time.fixedDeltaTime);
    }
    void UpdateAuthoritativeGroundState()
    {
        bool wasGrounded = isGrounded;
        Vector3 landingNormal = wasGrounded ? groundNormal : Vector3.up;
        float landingSpeed = Mathf.Max(
        0f,
        -Vector3.Dot(physicsBodyRigidbody.velocity, landingNormal)
        );
        isGrounded = ProbeGround(out RaycastHit hit);
        if (isGrounded)
        {
            groundNormal = hit.normal;
            groundPoint = hit.point;
            authoritativeGroundCollider = hit.collider;
            if (!wasGrounded)
                TryStartVisualImpact(landingSpeed, groundNormal);
            airborneTime = 0f;
            hardFallTime = 0f;
            TryRecordSafePoint();
        }
        else
        {
            authoritativeGroundCollider = null;
            slopeStopSegment = default;
            groundNormal = Vector3.up;
            groundPoint = physicsBodyRigidbody.position;
            float dt = Mathf.Max(Time.fixedDeltaTime, 1e-6f);
            airborneTime += dt;
            float fallSpeed = Mathf.Max(0f, Vector3.Dot(physicsBodyRigidbody.velocity, Vector3.down));
            hardFallTime = hardFallRecoverySpeed > 0f && fallSpeed >= hardFallRecoverySpeed
            ? hardFallTime + dt
            : 0f;
        }
        Vector3 tangentVelocity = Vector3.ProjectOnPlane(physicsBodyRigidbody.velocity, groundNormal);
        currentTangentSpeed = tangentVelocity.magnitude;
    }
    bool ProbeVisualGround(out RaycastHit hit)
    {
        Vector3 center = physicsBodyRigidbody
            ? physicsBodyRigidbody.position
            : transform.position;
        Vector3 origin = center + Vector3.up * .02f;
        float fallSpeed = physicsBodyRigidbody
            ? Mathf.Max(
                0f,
                Vector3.Dot(physicsBodyRigidbody.velocity, Vector3.down)
            )
            : 0f;
        float distance =
            sphereRadius +
            groundProbeDistance +
            Mathf.Min(
                fallSpeed * Time.fixedDeltaTime,
                maxExtraProbeDistance
            );

        if (!Physics.SphereCast(
            origin,
            sphereRadius * .95f,
            Vector3.down,
            out hit,
            distance,
            groundMask,
            QueryTriggerInteraction.Ignore
        ))
        {
            hit = default;
            return false;
        }

        // 見た目遷移では0度の平面も有効です。
        // authoritativeなProbeGroundの条件は変更しません。
        float angle = Vector3.Angle(hit.normal, Vector3.up);
        if (angle <= maxSlopeAngle)
            return true;

        hit = default;
        return false;
    }

    bool ProbeGround(out RaycastHit hit)
    {
        Vector3 center = physicsBodyRigidbody ? physicsBodyRigidbody.position : transform.position;
        Vector3 origin = center + Vector3.up * .02f;
        float fallSpeed = physicsBodyRigidbody
        ? Mathf.Max(0f, Vector3.Dot(physicsBodyRigidbody.velocity, Vector3.down))
        : 0f;
        float distance = sphereRadius + groundProbeDistance +
        Mathf.Min(fallSpeed * Time.fixedDeltaTime, maxExtraProbeDistance);
        Debug.DrawRay(origin, Vector3.down * distance, Color.red);
        if (Physics.SphereCast(
            origin,
            sphereRadius * .95f,
            Vector3.down,
            out hit,
            distance,
            groundMask,
            QueryTriggerInteraction.Ignore
            ))
        {
            if (Vector3.Angle(hit.normal, Vector3.up) <= maxSlopeAngle && Vector3.Angle(hit.normal, Vector3.up) > 2)
                return true;
        }
        hit = default;
        return false;
    }
    void SolveGround(Vector3 move)
    {
        float slopeAngle = Vector3.Angle(groundNormal, Vector3.up);
        if (slopeAngle < slopeForceTorqueMinAngle)
        {
            SolveFlatGround(move);
            return;
        }
        SolveSlopeGroundWithForceAndTorque(move);
    }
    void SolveFlatGround(Vector3 move)
    {
        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-6f);
        float stick = GetGroundStickAcceleration();
        float snapRate = 1f - Mathf.Exp(-normalSnapSharpness * dt);
        Vector3 velocity = physicsBodyRigidbody.velocity;
        Vector3 tangentVelocity = Vector3.ProjectOnPlane(velocity, groundNormal);
        Vector3 desiredVelocity = Vector3.ProjectOnPlane(move, groundNormal);
        float normalSpeed = Vector3.Dot(velocity, groundNormal);
        desiredVelocity = desiredVelocity.sqrMagnitude > 1e-6f
        ? desiredVelocity.normalized * maxGroundSpeed
        : Vector3.zero;
        physicsBodyRigidbody.AddForce(
        Vector3.ClampMagnitude(
        (desiredVelocity - tangentVelocity) / dt,
        maxGroundAcceleration
        ),
        ForceMode.Acceleration
        );
        ApplyGroundStick(normalSpeed, stick, snapRate, dt);
    }
    void SolveSlopeGroundWithForceAndTorque(Vector3 move)
    {
        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-6f);
        float stick = GetGroundStickAcceleration();
        float snapRate = 1f - Mathf.Exp(-normalSnapSharpness * dt);
        Vector3 velocity = physicsBodyRigidbody.velocity;
        Vector3 tangentVelocity = Vector3.ProjectOnPlane(velocity, groundNormal);
        float normalSpeed = Vector3.Dot(velocity, groundNormal);
        currentTangentSpeed = tangentVelocity.magnitude;
        Vector3 downhillGravity = Vector3.ProjectOnPlane(Physics.gravity, groundNormal);
        Vector3 downhillDirection = downhillGravity.sqrMagnitude > 1e-6f? downhillGravity.normalized: Vector3.zero;
        if (downhillGravity.sqrMagnitude > 1e-6f)
        {
            Vector3 nativeSlopeGravity = physicsBodyRigidbody.useGravity? downhillGravity: Vector3.zero;
            Vector3 desiredSlopeGravity =downhillGravity * downhillGravityMultiplier;
            Vector3 extraSlopeGravity =desiredSlopeGravity - nativeSlopeGravity;
            if (extraSlopeGravity.sqrMagnitude > 1e-6f)
            {
                physicsBodyRigidbody.AddForce(extraSlopeGravity, ForceMode.Acceleration);
            }
        }
        bool hasEffectGuidePlan = TryGetCommittedSlopePlan(tangentVelocity, downhillDirection, out Vector3 planDirection, out float planControlAcceleration);
        bool shouldApplyGuideBrakeToPhysics =
        hasEffectGuidePlan &&
        !effectGuideOnlyControlsBallVisual &&
        applyEffectGuideBrakeToPhysicsBody &&
        planDirection.sqrMagnitude > 1e-6f &&
        planControlAcceleration < 0f;
        if (shouldApplyGuideBrakeToPhysics)
        {
            physicsBodyRigidbody.AddForce(
            planDirection * planControlAcceleration,
            ForceMode.Acceleration
            );
        }
        if (tangentVelocity.sqrMagnitude > 1e-6f &&
            slopeRollingResistance > 0f)
        {
            physicsBodyRigidbody.AddForce(
            -tangentVelocity.normalized * slopeRollingResistance,
            ForceMode.Acceleration
            );
        }
        ApplyGroundStick(normalSpeed, stick, snapRate, dt);
    }
    string GetSlopeStopCaptureReasonName(int reason)
    {
        switch (reason)
        {
        case 1:
            return "NextStep";
        case 2:
            return "Now";
        case 3:
            return "CrossingBeforeNextSample";
        default:
            return "None";
        }
    }
    Vector3 GetCurrentSlopeStopEntryPoint()
    {
        if (followSlopeStopTargetWithCollider && slopeStopSegment.hasLocalStopFrame && slopeStopSegment.collider)
        {
            return slopeStopSegment.collider.transform.TransformPoint(slopeStopSegment.entryLocalPoint);
        }
        return slopeStopSegment.entryPoint;
    }
    Vector3 GetCurrentSlopeStopTargetPoint()
    {
        if (followSlopeStopTargetWithCollider && slopeStopSegment.hasLocalStopFrame && slopeStopSegment.collider)
        {
            return slopeStopSegment.collider.transform.TransformPoint(slopeStopSegment.stopLocalPoint);
        }
        if (slopeStopSegment.stopWorldPoint.sqrMagnitude > 1e-6f)
            return slopeStopSegment.stopWorldPoint;
        return GetSlopeStopWorldPoint(slopeStopSegment.stopCoordinate, slopeStopSegment.direction);
    }
    Vector3 GetCurrentSlopeStopPlanAxis(Vector3 fallbackAxis)
    {
        Vector3 axis = Vector3.zero;
        if (followSlopeStopTargetWithCollider && slopeStopSegment.hasLocalStopFrame && slopeStopSegment.collider)
        {
            axis = slopeStopSegment.collider.transform.TransformDirection(slopeStopSegment.directionLocal);
        }
        else if (slopeStopSegment.direction.sqrMagnitude > 1e-6f)
        {
            axis = slopeStopSegment.direction;
        }
        else
        {
            axis = fallbackAxis;
        }
        if (axis.sqrMagnitude <= 1e-6f)
            axis = fallbackAxis;
        if (axis.sqrMagnitude <= 1e-6f)
            axis = Vector3.right;
        axis.Normalize();
        if (fallbackAxis.sqrMagnitude > 1e-6f && Vector3.Dot(axis, fallbackAxis.normalized) < 0f)
        {
            axis = -axis;
        }
        return axis;
    }
    Vector3 GetSlopeStopWorldPoint(float coordinate, Vector3 planAxis)
    {
        if (slopeStopSegment.state != SlopeStopState.None && Mathf.Abs(coordinate - slopeStopSegment.stopCoordinate) <= .01f && (slopeStopSegment.hasLocalStopFrame || slopeStopSegment.stopWorldPoint.sqrMagnitude > 1e-6f))
        {
            return GetCurrentSlopeStopTargetPoint();
        }
        Vector3 axis = planAxis.sqrMagnitude > 1e-6f? planAxis.normalized: slopeStopSegment.direction.sqrMagnitude > 1e-6f? slopeStopSegment.direction.normalized: Vector3.right;
        return GetCurrentSlopeStopEntryPoint() +axis *(coordinate - slopeStopSegment.entryCoordinate);
    }
    bool TryResolveSlopeStopSurfacePoint(Collider preferredCollider, Vector3 approximatePoint, Vector3 referenceNormal, out Vector3 surfacePoint, out Vector3 surfaceNormal)
    {
        surfacePoint = approximatePoint;
        surfaceNormal = referenceNormal.sqrMagnitude > 1e-6f? referenceNormal.normalized: Vector3.up;
        Vector3 castNormal = surfaceNormal.sqrMagnitude > 1e-6f? surfaceNormal.normalized: Vector3.up;
        float castRadius = Mathf.Max(.03f, sphereRadius * .12f);
        Vector3 start = approximatePoint + castNormal * slopeStopTargetProbeHeight;
        float distance = slopeStopTargetProbeHeight + slopeStopTargetProbeDistance;
        if (Physics.SphereCast(start, castRadius, -castNormal, out RaycastHit sphereHit, distance, groundMask, QueryTriggerInteraction.Ignore))
        {
            if (!preferredCollider || sphereHit.collider == preferredCollider)
            {
                surfacePoint = sphereHit.point;
                surfaceNormal = sphereHit.normal;
                return true;
            }
        }
        if (Physics.Raycast(start, -castNormal, out RaycastHit rayHit, distance, groundMask, QueryTriggerInteraction.Ignore))
        {
            if (!preferredCollider || rayHit.collider == preferredCollider)
            {
                surfacePoint = rayHit.point;
                surfaceNormal = rayHit.normal;
                return true;
            }
        }
        if (preferredCollider)
        {
            Ray localRay = new Ray(start, -castNormal);
            if (preferredCollider.Raycast(localRay, out RaycastHit colliderHit, distance))
            {
                surfacePoint = colliderHit.point;
                surfaceNormal = colliderHit.normal;
                return true;
            }
            Vector3 closest = preferredCollider.ClosestPoint(approximatePoint);
            if ((closest - approximatePoint).sqrMagnitude <=Mathf.Max(.01f, slopeStopTargetProbeDistance * slopeStopTargetProbeDistance))
            {
                surfacePoint = closest;
                surfaceNormal = referenceNormal.sqrMagnitude > 1e-6f? referenceNormal.normalized: Vector3.up;
                return true;
            }
        }
        return false;
    }
    void StoreSlopeStopLocalFrame(Collider collider, Vector3 entryWorldPoint, Vector3 stopWorldPoint, Vector3 directionWorld, Vector3 stopNormal)
    {
        slopeStopSegment.stopWorldPoint = stopWorldPoint;
        slopeStopSegment.stopWorldNormal = stopNormal.sqrMagnitude > 1e-6f? stopNormal.normalized: groundNormal;
        slopeStopSegment.hasLocalStopFrame = false;
        if (!collider)
            return;
        Transform t = collider.transform;
        slopeStopSegment.entryLocalPoint = t.InverseTransformPoint(entryWorldPoint);
        slopeStopSegment.stopLocalPoint = t.InverseTransformPoint(stopWorldPoint);
        slopeStopSegment.directionLocal = t.InverseTransformDirection(directionWorld.normalized);
        slopeStopSegment.hasLocalStopFrame = true;
    }
    void RefreshSlopeStopTargetMarker(Vector3 planAxis, Vector3 surfaceNormal)
    {
        if (!showSlopeStopTargetMarker || slopeStopSegment.state == SlopeStopState.None || slopeStopSegment.direction.sqrMagnitude <= 1e-6f)
        {
            return;
        }
        Vector3 normal = surfaceNormal.sqrMagnitude > 1e-6f? surfaceNormal.normalized: Vector3.up;
        Vector3 targetWorldPoint = GetCurrentSlopeStopTargetPoint();
        SetSlopeStopDebugMarker(ref slopeStopTargetMarkerInstance, "Slope Stop Target", targetWorldPoint + normal * slopeStopMarkerSurfaceOffset, slopeStopTargetMarkerColor, slopeStopMarkerScale * 3f);
    }
    void RefreshCaptureNextStepMarker(Vector3 captureWorldPoint, Vector3 surfaceNormal)
    {
        if (!showCaptureNextStepMarker)
            return;
        Vector3 normal = surfaceNormal.sqrMagnitude > 1e-6f? surfaceNormal.normalized: Vector3.up;
        SetSlopeStopDebugMarker(ref captureNextStepMarkerInstance, "CaptureNextStep Point", captureWorldPoint + normal * slopeStopMarkerSurfaceOffset, captureNextStepMarkerColor, slopeStopMarkerScale * 3f);
    }

    void RefreshVisualEdgeFloatStartMarker(Vector3 worldPoint, Vector3 normalWorld)
    {
        if (!showVisualEdgeFloatMarkers || !showVisualEdgeFloatStartMarker)
            return;

        Vector3 normal = normalWorld.sqrMagnitude > 1e-6f
            ? normalWorld.normalized
            : Vector3.up;

        SetSlopeStopDebugMarker(
            ref visualEdgeFloatStartMarkerInstance,
            "Visual Edge Float Start",
            worldPoint + normal * slopeStopMarkerSurfaceOffset,
            visualEdgeFloatStartMarkerColor,
            slopeStopMarkerScale * Mathf.Max(visualEdgeFloatMarkerScaleMultiplier, .01f)
        );
    }

    void RefreshVisualEdgeFloatEndMarker(Vector3 worldPoint, Vector3 normalWorld)
    {
        if (!showVisualEdgeFloatMarkers || !showVisualEdgeFloatEndMarker)
            return;

        Vector3 normal = normalWorld.sqrMagnitude > 1e-6f
            ? normalWorld.normalized
            : Vector3.up;

        SetSlopeStopDebugMarker(
            ref visualEdgeFloatEndMarkerInstance,
            "Visual Edge Float End / Handoff Start",
            worldPoint + normal * slopeStopMarkerSurfaceOffset,
            visualEdgeFloatEndMarkerColor,
            slopeStopMarkerScale * Mathf.Max(visualEdgeFloatMarkerScaleMultiplier, .01f)
        );
    }

    Transform GetSlopeStopMarkerParent()
    {
        // 現在の停止計画を作った既存平面（Collider）を最優先にします。
        // マーカーはこのTransformの子になるため、ステージ回転中も
        // 平面上の同じローカル位置を保ちます。
        if (slopeStopSegment.collider)
            return slopeStopSegment.collider.transform;

        // 停止計画のColliderがまだ確定していない瞬間だけ、
        // 現在接地中の平面Colliderを予備として使います。
        if (authoritativeGroundCollider)
            return authoritativeGroundCollider.transform;

        return null;
    }

    void SetSlopeStopDebugMarker(
        ref GameObject instance,
        string markerName,
        Vector3 position,
        Color color,
        float scale
    )
    {
        if (!instance)
        {
            instance = slopeStopMarkerPrefab
                ? Instantiate(slopeStopMarkerPrefab)
                : GameObject.CreatePrimitive(PrimitiveType.Sphere);

            instance.name = markerName;

            foreach (Collider markerCollider
                     in instance.GetComponentsInChildren<Collider>())
            {
                markerCollider.enabled = false;
            }

            Renderer renderer = instance.GetComponentInChildren<Renderer>();
            if (renderer)
                renderer.material.color = color;
        }

        Transform planeParent = GetSlopeStopMarkerParent();

        // 既存平面の子へ入れます。
        // worldPositionStays=trueなので、親を変更した瞬間に
        // マーカーの見た目位置が飛ぶことはありません。
        if (planeParent && instance.transform.parent != planeParent)
            instance.transform.SetParent(planeParent, true);

        // positionはワールド座標で渡されています。
        // 親設定後に代入することで、Unityが対応するlocalPositionを保存します。
        instance.transform.position = position;

        SetMarkerWorldScale(
            instance.transform,
            Mathf.Max(.01f, scale)
        );
    }

    static void SetMarkerWorldScale(Transform marker, float worldScale)
    {
        if (!marker)
            return;

        Transform parent = marker.parent;
        if (!parent)
        {
            marker.localScale = Vector3.one * worldScale;
            return;
        }

        // 平面側にScaleが入っていても、マーカーの見かけの大きさが
        // 不必要に拡大・縮小されないように補正します。
        Vector3 parentScale = parent.lossyScale;

        float x = Mathf.Abs(parentScale.x) > .0001f
            ? worldScale / Mathf.Abs(parentScale.x)
            : worldScale;
        float y = Mathf.Abs(parentScale.y) > .0001f
            ? worldScale / Mathf.Abs(parentScale.y)
            : worldScale;
        float z = Mathf.Abs(parentScale.z) > .0001f
            ? worldScale / Mathf.Abs(parentScale.z)
            : worldScale;

        marker.localScale = new Vector3(x, y, z);
    }
    void RefreshSlopeStopMeasurementMarkers(
    Vector3 planAxis,
    Vector3 surfaceNormal,
    Vector3 currentWorldPoint,
    float predictedStoppingDistance,
    float fixedStepMargin,
    float remainingDistance
    )
    {
        debugRemainingToTarget = remainingDistance;
        debugForwardSpeed = Mathf.Max(
        0f,
        Vector3.Dot(
        Vector3.ProjectOnPlane(
        physicsBodyRigidbody ? physicsBodyRigidbody.velocity : Vector3.zero,
        surfaceNormal
        ),
        planAxis
        )
        );
        debugPredictedStoppingDistance = predictedStoppingDistance;
        debugFixedStepMargin = fixedStepMargin;
        debugReachedZoneEntered =
        remainingDistance <= slopeStopBoundaryEpsilon;
        debugPassedExactTarget = remainingDistance <= 0f;

        if (!showSlopeStopMeasurementMarkers ||
            slopeStopSegment.state == SlopeStopState.None)
        {
            return;
        }

        Vector3 axis = planAxis.sqrMagnitude > 1e-6f
        ? planAxis.normalized
        : slopeStopSegment.direction.normalized;
        Vector3 normal = surfaceNormal.sqrMagnitude > 1e-6f
        ? surfaceNormal.normalized
        : Vector3.up;
        Vector3 target = GetCurrentSlopeStopTargetPoint();
        float scale = slopeStopMarkerScale *
        Mathf.Max(measurementMarkerScaleMultiplier, .01f);
        Vector3 lift = normal * slopeStopMarkerSurfaceOffset;

        if (showReachedZoneStartMarker)
        {
            SetSlopeStopDebugMarker(
            ref reachedZoneStartMarkerInstance,
            "Reached Zone Start (-epsilon)",
            target - axis * slopeStopBoundaryEpsilon + lift,
            reachedZoneStartMarkerColor,
            scale
            );
        }

        if (showPassedSideReferenceMarker)
        {
            SetSlopeStopDebugMarker(
            ref passedSideReferenceMarkerInstance,
            "Passed Side Reference (+epsilon)",
            target + axis * slopeStopBoundaryEpsilon + lift,
            passedSideReferenceMarkerColor,
            scale
            );
        }

        if (showCurrentProjectedPointMarker)
        {
            SetSlopeStopDebugMarker(
            ref currentProjectedPointMarkerInstance,
            "Current Projected Point",
            currentWorldPoint + lift,
            currentProjectedPointMarkerColor,
            scale
            );
        }

        if (showPredictedStopPointMarker)
        {
            SetSlopeStopDebugMarker(
            ref predictedStopPointMarkerInstance,
            "Predicted Stop Point",
            currentWorldPoint +
            axis * Mathf.Max(0f, predictedStoppingDistance) +
            lift,
            predictedStopPointMarkerColor,
            scale
            );
        }
    }

    void ClearSlopeStopDebugMarkers()
    {
        DestroySlopeStopDebugMarker(ref slopeStopTargetMarkerInstance);
        DestroySlopeStopDebugMarker(ref captureNextStepMarkerInstance);
        DestroySlopeStopDebugMarker(ref reachedZoneStartMarkerInstance);
        DestroySlopeStopDebugMarker(ref passedSideReferenceMarkerInstance);
        DestroySlopeStopDebugMarker(ref currentProjectedPointMarkerInstance);
        DestroySlopeStopDebugMarker(ref predictedStopPointMarkerInstance);
        DestroySlopeStopDebugMarker(ref visualEdgeFloatStartMarkerInstance);
        DestroySlopeStopDebugMarker(ref visualEdgeFloatEndMarkerInstance);
    }
    void DestroySlopeStopDebugMarker(ref GameObject instance)
    {
        if (!instance)
            return;
        if (Application.isPlaying)
            Destroy(instance);
        else
            DestroyImmediate(instance);
        instance = null;
    }
    static string FormatVector3(Vector3 value)
    {
        return $"({value.x}, {value.y}, {value.z})";
    }
    bool TryGetCommittedSlopePlan(Vector3 tangentVelocity, Vector3 desiredDirection, out Vector3 planDirection, out float planControlAcceleration)
    {
        planDirection = Vector3.zero;
        planControlAcceleration = 0f;
        if (!useSlopeProgressStop ||
            !authoritativeGroundCollider ||
            desiredDirection.sqrMagnitude <= 1e-6f)
        {
            return false;
        }
        Vector3 probeDownhill = Vector3.ProjectOnPlane(Physics.gravity, groundNormal);
        if (probeDownhill.sqrMagnitude <= 1e-6f)
            return false;
        probeDownhill.Normalize();
        if (Vector3.Dot(desiredDirection, probeDownhill) <= .05f)
            return false;
        if (slopeStopSegment.state == SlopeStopState.Reached &&
            slopeStopSegment.collider == authoritativeGroundCollider)
        {
            return false;
        }
        if (slopeStopSegment.state == SlopeStopState.None ||
            slopeStopSegment.state < SlopeStopState.Committed)
        {
            if (!TryGetOrCreateSlopeStopSegment(authoritativeGroundCollider, probeDownhill))
                return false;
        }
        Vector3 planAxis = slopeStopSegment.state >= SlopeStopState.Committed &&
        useCommittedPlanAxisForSlopeStop
        ? GetCurrentSlopeStopPlanAxis(probeDownhill)
        : probeDownhill;
        float coordinate = Vector3.Dot(groundPoint, planAxis);
        float speed = Vector3.Dot(tangentVelocity, planAxis);
        if (slopeStopSegment.state < SlopeStopState.Committed)
        {
            if (speed <= .05f)
                return false;
            float armedLength = slopeStopSegment.exitCoordinate - coordinate;
            if (armedLength <= .05f)
                return false;
            float requestedDistance = Mathf.Max(
            armedLength * Mathf.Clamp01(slopeStopProgress),
            .001f
            );
            Vector3 targetWorldPoint = groundPoint + planAxis * requestedDistance;
            Vector3 targetWorldNormal = groundNormal;
            if (useSurfaceSnappedSlopeStopTarget)
            {
                TryResolveSlopeStopSurfacePoint(
                authoritativeGroundCollider,
                targetWorldPoint,
                groundNormal,
                out targetWorldPoint,
                out targetWorldNormal
                );
            }
            Vector3 entryToTarget = targetWorldPoint - groundPoint;
            if (entryToTarget.sqrMagnitude <= 1e-6f ||
                Vector3.Dot(entryToTarget.normalized, probeDownhill) <= .05f)
            {
                targetWorldPoint = groundPoint + probeDownhill * requestedDistance;
                targetWorldNormal = groundNormal;
                entryToTarget = targetWorldPoint - groundPoint;
            }
            Vector3 committedAxis = useSceneTargetDistanceForSlopeStop &&
            entryToTarget.sqrMagnitude > 1e-6f
            ? entryToTarget.normalized
            : planAxis;
            if (Vector3.Dot(committedAxis, probeDownhill) < 0f)
                committedAxis = -committedAxis;
            planAxis = committedAxis.normalized;
            coordinate = Vector3.Dot(groundPoint, planAxis);
            speed = Vector3.Dot(tangentVelocity, planAxis);
            if (speed <= .05f)
                return false;
            float stopCoordinate = Vector3.Dot(targetWorldPoint, planAxis);
            float sceneStopDistance = stopCoordinate - coordinate;
            if (sceneStopDistance <= .001f)
            {
                sceneStopDistance = requestedDistance;
                targetWorldPoint = groundPoint + planAxis * sceneStopDistance;
                stopCoordinate = coordinate + sceneStopDistance;
            }
            slopeStopSegment.entryPoint = groundPoint;
            slopeStopSegment.direction = planAxis;
            slopeStopSegment.entryCoordinate = coordinate;
            slopeStopSegment.stopCoordinate = stopCoordinate;
            slopeStopSegment.exitCoordinate =
            coordinate + Mathf.Max(armedLength, sceneStopDistance);
            slopeStopSegment.stopDistance = sceneStopDistance;
            StoreSlopeStopLocalFrame(
            authoritativeGroundCollider,
            groundPoint,
            targetWorldPoint,
            planAxis,
            targetWorldNormal
            );
            CommitSlopeStopPlan(speed, planAxis, tangentVelocity);
            StoreSlopeStopLocalFrame(
            authoritativeGroundCollider,
            groundPoint,
            targetWorldPoint,
            planAxis,
            targetWorldNormal
            );
            RefreshSlopeStopTargetMarker(planAxis, targetWorldNormal);
            coordinate = Vector3.Dot(groundPoint, planAxis);
            speed = Vector3.Dot(tangentVelocity, planAxis);
        }
        planAxis = GetCurrentSlopeStopPlanAxis(probeDownhill);
        planDirection = planAxis;
        coordinate = Vector3.Dot(groundPoint, planAxis);
        speed = Vector3.Dot(tangentVelocity, planAxis);
        Vector3 stopWorldPoint = GetCurrentSlopeStopTargetPoint();
        Vector3 toStopWorld = stopWorldPoint - groundPoint;
        float sceneWorldDistanceToTarget = toStopWorld.magnitude;
        float targetError = coordinate - slopeStopSegment.stopCoordinate;
        float remainingDistance = -targetError;
        if (useSceneTargetDistanceForSlopeStop &&
            (slopeStopSegment.hasLocalStopFrame ||
            slopeStopSegment.stopWorldPoint.sqrMagnitude > 1e-6f))
        {
            remainingDistance = Vector3.Dot(toStopWorld, planAxis);
            targetError = -remainingDistance;
            coordinate = slopeStopSegment.stopCoordinate + targetError;
        }
        float actualDistance = coordinate - slopeStopSegment.entryCoordinate;
        float elapsed = Mathf.Max(0f, RuntimeClock - slopeStopSegment.entryTime);
        GetPlannedSlopeKinematics(
        elapsed,
        out float plannedDistance,
        out float plannedSpeed,
        out _
        );
        bool crossedStopBoundary = targetError >= 0f;
        bool nearStopBoundary = Mathf.Abs(targetError) <= slopeStopBoundaryEpsilon;
        bool stillMovingDownhill = speed > slopeStopSpeedEpsilon;
        if (crossedStopBoundary && !stillMovingDownhill)
        {
            slopeStopSegment.state = SlopeStopState.Reached;
            slopeStopSegment.brakeControl = 0f;
            StopCaptureNextStepBounce(false);
            if (debugSlopeProgressStop || debugCaptureNextStep)
            {
                Debug.Log(
                $"[SlopeStop +0 Complete] " +
                $"axis=scene " +
                $"targetError={targetError:+0.0000;-0.0000;0.0000}m " +
                $"sceneRemaining={remainingDistance:F4}m " +
                $"worldDistance={sceneWorldDistanceToTarget:F4}m " +
                $"speed={speed:F4}m/s " +
                $"actualDistance={actualDistance:F4}m " +
                $"plannedDistance={plannedDistance:F4}m " +
                $"captureReason={GetSlopeStopCaptureReasonName(slopeStopSegment.brakeReason)}"
                );
            }
            return false;
        }
        if (crossedStopBoundary && stillMovingDownhill &&
            (debugSlopeProgressStop || debugCaptureNextStep))
        {
            Debug.Log(
            $"[SlopeStop +0 BrakingThrough] " +
            $"axis=scene " +
            $"targetError={targetError:+0.0000;-0.0000;0.0000}m " +
            $"sceneRemaining={remainingDistance:F4}m " +
            $"worldDistance={sceneWorldDistanceToTarget:F4}m " +
            $"speed={speed:F4}m/s " +
            $"continueBrake=True " +
            $"captureReason={GetSlopeStopCaptureReasonName(slopeStopSegment.brakeReason)}"
            );
        }
        if (speed <= 0f)
        {
            slopeStopSegment.state = SlopeStopState.Reached;
            slopeStopSegment.brakeControl = 0f;
            StopCaptureNextStepBounce(false);
            if (debugSlopeProgressStop || debugCaptureNextStep)
            {
                Debug.Log(
                $"[SlopeStop UpwardMotion] " +
                $"axis=scene " +
                $"targetError={targetError:+0.0000;-0.0000;0.0000}m " +
                $"sceneRemaining={remainingDistance:F4}m " +
                $"speed={speed:F4}m/s"
                );
            }
            return false;
        }
        if (nearStopBoundary && speed <= slopeStopSpeedEpsilon)
        {
            slopeStopSegment.state = SlopeStopState.Reached;
            slopeStopSegment.brakeControl = 0f;
            StopCaptureNextStepBounce(false);
            if (debugSlopeProgressStop || debugCaptureNextStep)
            {
                Debug.Log(
                $"[SlopeStop BoundaryComplete] " +
                $"axis=scene " +
                $"targetError={targetError:+0.0000;-0.0000;0.0000}m " +
                $"remaining={remainingDistance:F4}m " +
                $"worldDistance={sceneWorldDistanceToTarget:F4}m " +
                $"speed={speed:F4}m/s " +
                $"captureReason={GetSlopeStopCaptureReasonName(slopeStopSegment.brakeReason)}"
                );
            }
            return false;
        }
        float gravityAcceleration = Vector3.Dot(
        Vector3.ProjectOnPlane(Physics.gravity, groundNormal) *
        downhillGravityMultiplier,
        planAxis
        );
        float rollingAcceleration = GetRollingAccelerationAlongDirection(
        tangentVelocity,
        planAxis,
        slopeRollingResistance
        );
        float naturalAcceleration = gravityAcceleration + rollingAcceleration;
        float smoothDeceleration = Mathf.Clamp(
        slopeStopSmoothDeceleration,
        .001f,
        slopeStopMaxBrakeAcceleration
        );
        float safeRemaining = Mathf.Max(remainingDistance, .001f);
        float smoothStopDistance = speed * speed /(2f * smoothDeceleration);
        float captureRatio = Mathf.Clamp(slopeStopCaptureRatio, .80f, 1.10f);
        float captureThresholdDistance = safeRemaining * captureRatio;
        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-6f);
        float predictedSpeed = Mathf.Max(0f, speed + naturalAcceleration * dt);
        float predictedRemaining =
        remainingDistance - speed * dt - .5f * naturalAcceleration * dt * dt;
        float predictedSmoothStopDistance =
        predictedSpeed * predictedSpeed /(2f * smoothDeceleration);
        float predictedCaptureThreshold = Mathf.Max(0f, predictedRemaining) * captureRatio;
        float predictedRatio = predictedCaptureThreshold > .0001f
        ? predictedSmoothStopDistance / predictedCaptureThreshold
        : 999f;
        float predictedGap = predictedCaptureThreshold - predictedSmoothStopDistance;

        RefreshSlopeStopMeasurementMarkers(
        planAxis,
        groundNormal,
        groundPoint,
        smoothStopDistance,
        speed * dt,
        remainingDistance
        );

        bool captureNow = smoothStopDistance >= captureThresholdDistance;
        bool captureNextStep =
        predictedRemaining > slopeStopBoundaryEpsilon &&
        predictedSmoothStopDistance >= predictedCaptureThreshold;
        bool crossingBeforeNextSample = predictedRemaining <= slopeStopBoundaryEpsilon;

        if (captureNextStep)
        {
            Debug.Log("");
        }
        bool captureNextStepStartedThisFrame =
        captureNextStep && !slopeStopSegment.captureNextStepTriggered;
        if (captureNextStepStartedThisFrame)
        {
            slopeStopSegment.captureNextStepTriggered = true;
            Vector3 captureWorldPoint = groundPoint;
            float captureToTargetWorldDistance = Vector3.Distance(captureWorldPoint, stopWorldPoint);
            RefreshSlopeStopTargetMarker(planAxis, slopeStopSegment.stopWorldNormal);
            RefreshCaptureNextStepMarker(captureWorldPoint, groundNormal);
            StartCaptureNextStepBounce(
            captureWorldPoint,
            stopWorldPoint,
            groundNormal,
            planAxis,
            speed,
            Mathf.Max(remainingDistance, captureToTargetWorldDistance)
            );
            float sceneVsAxisGap = captureToTargetWorldDistance - Mathf.Abs(remainingDistance);
            string warning = Mathf.Abs(sceneVsAxisGap) >= slopeStopSceneMismatchWarning
            ? " <-- SCENE/AXIS GAP"
            : "";
            Debug.Log(
            $"[CaptureNextStep Measure] " +
            $"axis=scene " +
            $"t={elapsed:F3} " +
            $"targetError={targetError:+0.0000;-0.0000;0.0000}m " +
            $"remainingAlongAxis={remainingDistance:F4}m " +
            $"worldDistanceToTarget={captureToTargetWorldDistance:F4}m " +
            $"sceneAxisGap={sceneVsAxisGap:+0.0000;-0.0000;0.0000}m " +
            $"speed={speed:F3}m/s " +
            $"currentCoordinate={coordinate:F4} " +
            $"stopCoordinate={slopeStopSegment.stopCoordinate:F4} " +
            $"capturePoint={FormatVector3(captureWorldPoint)} " +
            $"targetPoint={FormatVector3(stopWorldPoint)}" +
            warning
            );
        }
        if (slopeStopSegment.state != SlopeStopState.Braking && debugCaptureNextStep)
        {
            bool shouldLogWatch =
            RuntimeClock >= nextCaptureNextStepDebugTime &&
            (
            debugCaptureNextStepEveryFixedStep ||
            predictedRatio >= debugCaptureNextStepMinRatioToLog ||
            captureNextStep
            );
            if (shouldLogWatch)
            {
                nextCaptureNextStepDebugTime =
                RuntimeClock + Mathf.Max(debugCaptureNextStepLogInterval, .001f);
                Debug.Log(
                $"[CaptureNextStep Watch] " +
                $"axis=scene " +
                $"t={elapsed:F3} " +
                $"targetError={targetError:+0.0000;-0.0000;0.0000}m " +
                $"remaining={remainingDistance:F3} " +
                $"worldDistance={sceneWorldDistanceToTarget:F3} " +
                $"speed={speed:F3} " +
                $"naturalA={naturalAcceleration:F3} " +
                $"predSpeed={predictedSpeed:F3} " +
                $"predRemaining={predictedRemaining:F3} " +
                $"predSmoothStop={predictedSmoothStopDistance:F3} " +
                $"predThreshold={predictedCaptureThreshold:F3} " +
                $"predGap={predictedGap:+0.000;-0.000;0.000} " +
                $"predRatio={predictedRatio:F3} " +
                $"captureNextStep={captureNextStep} " +
                $"captureNow={captureNow} " +
                $"crossing={crossingBeforeNextSample}"
                );
            }
            if (captureNextStepStartedThisFrame)
            {
                Debug.Log(
                $"[CaptureNextStep Trigger] " +
                $"axis=scene " +
                $"t={elapsed:F3} " +
                $"targetError={targetError:+0.0000;-0.0000;0.0000}m " +
                $"remaining={remainingDistance:F3} " +
                $"worldDistance={sceneWorldDistanceToTarget:F3} " +
                $"speed={speed:F3} " +
                $"predSpeed={predictedSpeed:F3} " +
                $"predRemaining={predictedRemaining:F3} " +
                $"predSmoothStop={predictedSmoothStopDistance:F3} " +
                $"predThreshold={predictedCaptureThreshold:F3} " +
                $"predGap={predictedGap:+0.000;-0.000;0.000} " +
                $"predRatio={predictedRatio:F3}"
                );
            }
        }
        if (slopeStopSegment.state != SlopeStopState.Braking)
        {
            int captureReason = 0;
            if (captureNextStep)
                captureReason = 1;
            else if (captureNow)
                captureReason = 2;
            else if (crossingBeforeNextSample)
                captureReason = 3;
            bool shouldCapture = captureNextStep;
            if (!shouldCapture)
                return false;
            float currentCaptureDeceleration =
            speed * speed /(2f * Mathf.Max(safeRemaining, .001f));
            float predictedCaptureDeceleration =
            predictedSpeed * predictedSpeed /
            (2f * Mathf.Max(predictedRemaining, .001f));
            float captureDeceleration = currentCaptureDeceleration;
            if (captureReason == 1 &&
                applyCaptureNextStepResultProfile &&
                captureNextStepUsePredictedDeceleration)
            {
                captureDeceleration = Mathf.Max(
                currentCaptureDeceleration,
                predictedCaptureDeceleration
                );
            }
            captureDeceleration = Mathf.Clamp(
            captureDeceleration,
            .001f,
            slopeStopMaxBrakeAcceleration
            );
            float captureControlAcceleration = Mathf.Clamp(
            -captureDeceleration - naturalAcceleration,
            -slopeStopMaxBrakeAcceleration,
            0f
            );
            float initialControlAcceleration = 0f;
            if (captureReason == 1 && applyCaptureNextStepResultProfile)
            {
                float ratioStartControl =
                captureControlAcceleration * Mathf.Clamp01(captureNextStepInitialBrakeRatio);
                float minimumUsefulControl =
                -Mathf.Max(0f, naturalAcceleration) -
                Mathf.Max(0f, captureNextStepInitialNetDeceleration);
                initialControlAcceleration = Mathf.Clamp(
                Mathf.Min(ratioStartControl, minimumUsefulControl),
                -slopeStopMaxBrakeAcceleration,
                0f
                );
            }
            slopeStopSegment.state = SlopeStopState.Braking;
            slopeStopSegment.brakeReason = captureReason;
            slopeStopSegment.brakeDeceleration = captureDeceleration;
            slopeStopSegment.brakeControl = initialControlAcceleration;
            slopeStopSegment.brakeInitialControl = initialControlAcceleration;
            if (debugSlopeProgressStop || debugCaptureNextStep)
            {
                Debug.Log(
                $"[SlopeStop SmoothCaptureStart -0] " +
                $"axis=scene " +
                $"reason={GetSlopeStopCaptureReasonName(captureReason)} " +
                $"t={elapsed:F3} " +
                $"targetError={targetError:+0.0000;-0.0000;0.0000}m " +
                $"remaining={remainingDistance:F3} " +
                $"worldDistance={sceneWorldDistanceToTarget:F3} " +
                $"speed={speed:F3} " +
                $"smoothStopDistance={smoothStopDistance:F3} " +
                $"captureThreshold={captureThresholdDistance:F3} " +
                $"predRemaining={predictedRemaining:F3} " +
                $"predSmoothStop={predictedSmoothStopDistance:F3} " +
                $"predThreshold={predictedCaptureThreshold:F3} " +
                $"predGap={predictedGap:+0.000;-0.000;0.000} " +
                $"predRatio={predictedRatio:F3} " +
                $"captureDecel={captureDeceleration:F3} " +
                $"predCaptureDecel={predictedCaptureDeceleration:F3} " +
                $"captureControlTarget={captureControlAcceleration:F3} " +
                $"initialControl={initialControlAcceleration:F3} " +
                $"controlGap={(captureControlAcceleration - initialControlAcceleration):F3}"
                );
            }
        }
        float capturedDeceleration = Mathf.Max(slopeStopSegment.brakeDeceleration, .001f);
        float terminalRemaining = crossedStopBoundary
        ? 0f
        : Mathf.Max(remainingDistance - captureNextStepTerminalSoftDistance, 0f);
        if (!crossedStopBoundary && terminalRemaining <= 1e-6f)
            terminalRemaining = Mathf.Max(remainingDistance, 0f);
        float capturedVLimit = Mathf.Sqrt(
        2f * capturedDeceleration * Mathf.Max(terminalRemaining, 0f)
        );
        float capturedNetAcceleration = -capturedDeceleration;
        float dynamicNetAcceleration =
        -speed * speed /(2f * Mathf.Max(safeRemaining, .001f));
        float velocityLimitNetAcceleration = capturedNetAcceleration;
        if (useCaptureNextStepTerminalVelocityLimit &&
            slopeStopSegment.brakeReason == 1)
        {
            float responseDt = Mathf.Max(
            dt * Mathf.Max(1f, captureNextStepVelocityLimitResponseSteps),
            dt
            );
            float speedError = speed - capturedVLimit;
            if (speedError > 0f)
                velocityLimitNetAcceleration = -speedError / responseDt;
        }
        float desiredNetAcceleration = Mathf.Min(
        capturedNetAcceleration,
        dynamicNetAcceleration,
        velocityLimitNetAcceleration
        );
        float targetControlAcceleration = Mathf.Clamp(
        desiredNetAcceleration - naturalAcceleration,
        -slopeStopMaxBrakeAcceleration,
        0f
        );
        float activeBrakeJerk = slopeStopBrakeJerk;
        if (slopeStopSegment.brakeReason == 1 && applyCaptureNextStepResultProfile)
            activeBrakeJerk *= Mathf.Max(1f, captureNextStepBrakeJerkMultiplier);
        slopeStopSegment.brakeControl = Mathf.MoveTowards(
        slopeStopSegment.brakeControl,
        targetControlAcceleration,
        activeBrakeJerk * dt
        );
        planControlAcceleration = slopeStopSegment.brakeControl;
        if (speed <= slopeStopSpeedEpsilon &&
            Mathf.Abs(targetError) <= slopeStopBoundaryEpsilon * 2f)
        {
            slopeStopSegment.state = SlopeStopState.Reached;
            slopeStopSegment.brakeControl = 0f;
            StopCaptureNextStepBounce(false);
            return false;
        }
        if (planControlAcceleration >= 0f)
            return false;
        slopeStopSegment.state = SlopeStopState.Braking;
        if (debugSlopeProgressStop || debugCaptureNextStep)
        {
            Debug.Log(
            $"[SlopePlan SmoothCapturedBrake] " +
            $"axis=scene " +
            $"reason={GetSlopeStopCaptureReasonName(slopeStopSegment.brakeReason)} " +
            $"t={elapsed:F3} " +
            $"targetError={targetError:+0.0000;-0.0000;0.0000}m " +
            $"remaining={remainingDistance:F3} " +
            $"worldDistance={sceneWorldDistanceToTarget:F3} " +
            $"actualX={actualDistance:F3} " +
            $"plannedX={plannedDistance:F3} " +
            $"speed={speed:F3} " +
            $"plannedV={plannedSpeed:F3} " +
            $"vLimit={capturedVLimit:F3} " +
            $"crossed={crossedStopBoundary} " +
            $"near={nearStopBoundary} " +
            $"stillMoving={stillMovingDownhill} " +
            $"captureDecel={capturedDeceleration:F3} " +
            $"desiredNetA={desiredNetAcceleration:F3} " +
            $"gravity={gravityAcceleration:F3} " +
            $"rolling={rollingAcceleration:F3} " +
            $"controlTarget={targetControlAcceleration:F3} " +
            $"controlNow={planControlAcceleration:F3} " +
            $"activeJerk={activeBrakeJerk:F3} " +
            $"initialControl={slopeStopSegment.brakeInitialControl:F3}"
            );
        }
        return true;
    }
    void CommitSlopeStopPlan(float actualEntrySpeed, Vector3 downhill, Vector3 entryTangentVelocity)
    {
        float requestedDistance = Mathf.Max(slopeStopSegment.stopCoordinate - slopeStopSegment.entryCoordinate, .001f);
        float entrySpeed = Mathf.Max(0f, actualEntrySpeed);
        float idealNetAcceleration = -entrySpeed * entrySpeed /(2f * requestedDistance);
        float gravityAcceleration = Vector3.Dot(Vector3.ProjectOnPlane(Physics.gravity, groundNormal) *downhillGravityMultiplier, downhill);
        float rollingAcceleration = GetRollingAccelerationAlongDirection(entryTangentVelocity, downhill, slopeRollingResistance);
        float naturalAcceleration = gravityAcceleration + rollingAcceleration;
        float idealControlAcceleration = idealNetAcceleration - naturalAcceleration;
        float clampedControlAcceleration = Mathf.Clamp(idealControlAcceleration, -slopeStopMaxBrakeAcceleration, 0f);
        float stopTime = entrySpeed > .001f && idealNetAcceleration < -.001f? -entrySpeed / idealNetAcceleration: 0f;
        slopeStopSegment.direction = downhill;
        slopeStopSegment.entrySpeed = entrySpeed;
        slopeStopSegment.stopDistance = requestedDistance;
        slopeStopSegment.entryTime = RuntimeClock;
        slopeStopSegment.state = SlopeStopState.Committed;
        slopeStopSegment.brakeReason = 0;
        slopeStopSegment.brakeDeceleration = 0f;
        slopeStopSegment.brakeControl = 0f;
        slopeStopSegment.brakeInitialControl = 0f;
        slopeStopSegment.captureNextStepTriggered = false;
        nextCaptureNextStepDebugTime = -Mathf.Infinity;
        if (debugSlopeProgressStop || debugCaptureNextStep)
        {
            Debug.Log(
            $"[SlopeStop Commit] " +
            $"v0={entrySpeed:F3}m/s " +
            $"distance={requestedDistance:F3}m " +
            $"aIdeal={idealNetAcceleration:F3}m/s² " +
            $"gravity={gravityAcceleration:F3}m/s² " +
            $"rolling={rollingAcceleration:F3}m/s² " +
            $"natural={naturalAcceleration:F3}m/s² " +
            $"controlIdeal={idealControlAcceleration:F3}m/s² " +
            $"controlClamped={clampedControlAcceleration:F3}m/s² " +
            $"smoothDecel={slopeStopSmoothDeceleration:F3}m/s² " +
            $"captureRatio={slopeStopCaptureRatio:F3} " +
            $"brakeJerk={slopeStopBrakeJerk:F3}m/s³ " +
            $"stopTime={stopTime:F3}s"
            );
        }
    }
    void GetPlannedSlopeKinematics(float elapsed, out float plannedDistance, out float plannedSpeed, out float plannedAcceleration)
    {
        float distance = Mathf.Max(slopeStopSegment.stopDistance, .001f);
        float entrySpeed = Mathf.Max(slopeStopSegment.entrySpeed, 0f);
        float netAcceleration = entrySpeed > .001f? -entrySpeed * entrySpeed /(2f * distance): 0f;
        float stopTime = entrySpeed > .001f && netAcceleration < -.001f? -entrySpeed / netAcceleration: 0f;
        float t = stopTime > 0f ? Mathf.Min(elapsed, stopTime) : 0f;
        plannedAcceleration = elapsed <= stopTime ? netAcceleration : 0f;
        plannedDistance = entrySpeed * t + .5f * netAcceleration * t * t;
        plannedDistance = Mathf.Clamp(plannedDistance, 0f, distance);
        plannedSpeed = entrySpeed + netAcceleration * t;
        if (elapsed >= stopTime)
            plannedSpeed = 0f;
        plannedSpeed = Mathf.Max(0f, plannedSpeed);
    }
    float GetRollingAccelerationAlongDirection(Vector3 tangentVelocity, Vector3 direction, float resistance)
    {
        if (resistance <= 0f || tangentVelocity.sqrMagnitude <= 1e-8f || direction.sqrMagnitude <= 1e-6f)
        {
            return 0f;
        }
        Vector3 rollingAcceleration =-tangentVelocity.normalized * resistance;
        return Vector3.Dot(rollingAcceleration, direction.normalized);
    }
    bool TryGetOrCreateSlopeStopSegment(Collider collider, Vector3 downhill)
    {
        bool needsNewSegment = slopeStopSegment.state == SlopeStopState.None || slopeStopSegment.collider != collider || Vector3.Dot(slopeStopSegment.direction, downhill) < .98f;
        if (!needsNewSegment)
            return true;
        if (!TryGetColliderCoordinateRange(collider, downhill, out float min, out float max))
            return false;
        float entry = Vector3.Dot(groundPoint, downhill);
        float exit = max;
        float length = exit - entry;
        if (length <= .05f)
            return false;
        float stopCoordinate = Mathf.Lerp(entry, exit, slopeStopProgress);
        slopeStopSegment = new SlopeStopSegment
        {
            state = SlopeStopState.Armed, collider = collider, entryPoint = groundPoint, direction = downhill, entryCoordinate = entry, exitCoordinate = exit, stopCoordinate = stopCoordinate, stopDistance = Mathf.Max(stopCoordinate - entry, .001f)
        };
        if (debugSlopeProgressStop)
        {
            Debug.Log($"[SlopeStop] armed length={length}m " + $"stop={slopeStopProgress} " + $"at={(slopeStopSegment.stopCoordinate - entry)}m");
        }
        return true;
    }
    static bool TryGetColliderCoordinateRange(Collider collider, Vector3 direction, out float min, out float max)
    {
        min = float.PositiveInfinity;
        max = float.NegativeInfinity;
        if (!collider || direction.sqrMagnitude <= 1e-6f)
            return false;
        Bounds b = collider.bounds;
        Vector3 c = b.center;
        Vector3 e = b.extents;
        for (int x = -1; x <= 1; x += 2)
            for (int y = -1; y <= 1; y += 2)
                for (int z = -1; z <= 1; z += 2)
                {
                    Vector3 corner = c + new Vector3(e.x * x, e.y * y, e.z * z);
                float coordinate = Vector3.Dot(corner, direction);
            min = Mathf.Min(min, coordinate);
            max = Mathf.Max(max, coordinate);
        }
        return max > min;
    }
    void OnDrawGizmosSelected()
    {
        if (!drawSlopeProgressStop || slopeStopSegment.state == SlopeStopState.None)
            return;
        Vector3 d = GetCurrentSlopeStopPlanAxis(slopeStopSegment.direction);
        Vector3 entryPoint = GetCurrentSlopeStopEntryPoint();
        Vector3 stopPoint = GetCurrentSlopeStopTargetPoint();
        Vector3 normal = groundNormal.sqrMagnitude > 1e-6f
        ? groundNormal.normalized
        : Vector3.up;
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(entryPoint, stopPoint);
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(stopPoint + normal * .04f, .08f);

        Vector3 zoneStart =
        stopPoint - d.normalized * slopeStopBoundaryEpsilon;
        Gizmos.color = reachedZoneStartMarkerColor;
        Gizmos.DrawSphere(zoneStart + normal * .04f, .055f);
        Gizmos.DrawLine(zoneStart + normal * .04f, stopPoint + normal * .04f);

        Gizmos.color = passedSideReferenceMarkerColor;
        Gizmos.DrawSphere(
        stopPoint + d.normalized * slopeStopBoundaryEpsilon + normal * .04f,
        .05f
        );
    }
    float GetGroundStickAcceleration()
    {
        float stick = baseStickAcceleration;
        if (!useAnalyticTrackAssist)
        {
            currentMargin = 999f;
            hasTakeoffPoint = false;
            return stick;
        }
        float speed = Mathf.Abs(Vector3.Dot(Vector3.ProjectOnPlane(physicsBodyRigidbody.velocity, groundNormal), Vector3.right));
        currentMargin = Margin(transform.position.x, speed, baseStickAcceleration, derivativeStep);
        stick += extraStickAcceleration *Mathf.Exp(-analyticSharpness * Mathf.Max(currentMargin, 0f));
        hasTakeoffPoint = FindTakeoffPointAhead(transform.position.x, speed, baseStickAcceleration, derivativeStep, out takeoffX);
        return stick;
    }
    void ApplyGroundStick(float normalSpeed, float stick, float snapRate, float dt)
    {
        float normalAcceleration = 0f;
        if (normalSpeed > targetNormalSpeed)
        {
            float snappedNormalSpeed = Mathf.Lerp(normalSpeed, targetNormalSpeed, snapRate);
            normalAcceleration = (snappedNormalSpeed - normalSpeed) / dt - stick;
        }
        physicsBodyRigidbody.AddForce(groundNormal * normalAcceleration, ForceMode.Acceleration);
    }
    void SolveAir(Vector3 move)
    {
        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-6f);
        Vector3 horizontalVelocity = Vector3.ProjectOnPlane(physicsBodyRigidbody.velocity, Vector3.up);
        Vector3 acceleration = (move * maxGroundSpeed - horizontalVelocity) / dt;
        physicsBodyRigidbody.AddForce(Vector3.ClampMagnitude(acceleration, airAcceleration), ForceMode.Acceleration);
        currentMargin = -1f;
        hasTakeoffPoint = false;
    }
    void TryRecordSafePoint()
    {
        if (!isGrounded || !physicsBodyRigidbody)
            return;
        bool unsafeLanding = Vector3.Dot(physicsBodyRigidbody.velocity, groundNormal) < -maxSafeNormalSpeed;
        if (unsafeLanding || RuntimeClock < nextSafePointRecordTime)
            return;
        SetRecoveryPoint(physicsBodyRigidbody.position, groundNormal, headingDir);
    }
    public void SetRecoveryPoint(Vector3 position, Vector3 normal, Vector3 direction)
    {
        lastSafePosition = position;
        lastSafeNormal = normal.sqrMagnitude > 1e-6f ? normal.normalized : Vector3.up;
        lastSafeHeading = useSideTurnExperiment? SnapToCardinalXZ(direction): Flat(direction, Vector3.right);
        hasSafePoint = true;
        nextSafePointRecordTime = RuntimeClock + safePointRecordInterval;
    }
    bool ShouldRecoverFromFall()
    {
        if (!useFallRecovery || !hasSafePoint || !physicsBodyRigidbody)
            return false;
        return !isGrounded && (airborneTime >= autoRecoverAfterAirTime || physicsBodyRigidbody.position.y < lastSafePosition.y - recoveryBelowSafePoint || (hardFallRecoverySpeed > 0f && hardFallTime >= hardFallRecoveryTime));
    }
    public void RequestFallRecovery() => RecoverFromFall();
    void RecoverFromFall()
    {
        if (!hasSafePoint || !physicsBodyRigidbody)
            return;
        headingTween?.Kill();
        ResetVisualOnly();
        pendingVisualPlaneTransition = default;
        hasQueuedTurn = false;
        queuedTurnAngle = 0f;
        visualImpulseSequence = 0;
        Vector3 position = lastSafePosition + lastSafeNormal * recoveryLift;
        if (recoverToStartSlab && slab)
            position = slab.transform.position + Vector3.up * 2f;
        physicsBodyRigidbody.position = position;
        physicsBodyRigidbody.velocity = Vector3.zero;
        physicsBodyRigidbody.angularVelocity = Vector3.zero;
        SetHeading(lastSafeHeading, false);
        SetHeadingRotation(headingTransform, Quaternion.LookRotation(headingDir, Vector3.up));
        groundNormal = lastSafeNormal;
        groundPoint = position - lastSafeNormal * sphereRadius;
        isGrounded = false;
        airborneTime = hardFallTime = 0f;
        nextSafePointRecordTime = RuntimeClock + safePointRecordInterval;
        Physics.SyncTransforms();
    }
    float H(float x) => analyticYOffset + analyticAmplitude * Mathf.Cos(x * analyticFrequency);
    float Margin(float x, float speed, float stick, float h)
    {
        h = Mathf.Max(1e-4f, h);
        float a = H(x + h);
        float b = H(x);
        float c = H(x - h);
        float d1 = (a - c) /(2f * h);
        float d2 = (a - 2f * b + c) /(h * h);
        float curvature = Mathf.Max(0f, -d2 / Mathf.Pow(1f + d1 * d1, 1.5f));
        return Mathf.Abs(Physics.gravity.y) /Mathf.Sqrt(1f + d1 * d1) +stick -speed * speed * curvature;
    }
    static bool Cross(float a, float b) =>(a > 0f && b <= 0f) || (a < 0f && b >= 0f) || Mathf.Approximately(b, 0f);
    bool FindTakeoffPointAhead(float x0, float speed, float stick, float h, out float x)
    {
        x = x0;
        if (lookAheadDistance <= 0f || takeoffSearchSegments <= 0)
            return false;
        float step = lookAheadDistance / takeoffSearchSegments;
        float leftX = x0;
        float leftMargin = Margin(leftX, speed, stick, h);
        for (int i = 1; i <= takeoffSearchSegments; i++)
        {
            float rightX = x0 + step * i;
            float rightMargin = Margin(rightX, speed, stick, h);
            if (!Cross(leftMargin, rightMargin))
            {
                leftX = rightX;
                leftMargin = rightMargin;
                continue;
            }
            float a = leftX;
            float b = rightX;
            float marginA = leftMargin;
            for (int j = 0; j < 24; j++)
            {
                float middle = (a + b) * .5f;
                float middleMargin = Margin(middle, speed, stick, h);
                if (Cross(marginA, middleMargin))
                    b = middle;
                else
                {
                    a = middle;
                    marginA = middleMargin;
                }
            }
            x = (a + b) * .5f;
            return true;
        }
        return false;
    }
}