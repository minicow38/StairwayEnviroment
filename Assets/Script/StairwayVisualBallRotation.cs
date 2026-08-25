using UnityEngine;

public class StairwayVisualBallRotation : MonoBehaviour
{
    [Header("References")]
    [SerializeField] Rigidbody rootRb;

    [Tooltip("回転させる見た目専用の球体。RigidbodyやColliderは付けない。")]
    [SerializeField] Transform visualBall;

    [Tooltip("親の進行方向。空なら rootRb.transform.forward を使う。")]
    [SerializeField] Transform headingTransform;

    [Header("Ground Probe")]
    [SerializeField] LayerMask groundMask = ~0;

    [Tooltip("下向きRaycastの開始位置")]
    [SerializeField] Vector3 groundProbeOffset = new Vector3(0f, 0.15f, 0f);

    [SerializeField] float groundProbeDistance = 1.2f;

    [Tooltip("階段判定に使うタグ")]
    [SerializeField] string stairsTag = "Stairs";

    [Header("Visual Roll")]
    [Tooltip("BallVisualの半径")]
    [SerializeField] float visualRadius = 0.5f;

    [Tooltip("通常回転の倍率")]
    [SerializeField] float normalSpinMultiplier = 1f;

    [Tooltip("実際の進行速度へ見た目速度が追従する速さ")]
    [SerializeField] float visualSpeedFollowSharpness = 8f;

    [Tooltip("回転角速度が目標値へ追従する速さ")]
    [SerializeField] float visualSpinFollowSharpness = 12f;

    [Tooltip("見た目上の回転を自然に減衰させる強さ")]
    [SerializeField] float visualSpinDamping = 5f;

    [Tooltip("見た目の回転速度上限。degree/sec")]
    [SerializeField] float visualSpinMaxDegreesPerSecond = 1500f;

    [Header("Visual Progress Lag")]
    [Tooltip("段差で見た目速度をどれだけ失速させるか")]
    [SerializeField, Range(0f, 1f)]
    float stairBrakeAmount = 0.35f;

    [Tooltip("段差直後に見た目だけ減速している時間")]
    [SerializeField] float stairBrakeSeconds = 0.12f;

    [Tooltip("見た目速度が実際の速度より遅れてよい最大差")]
    [SerializeField] float maxVisualSpeedLag = 4f;

    [Tooltip("階段中は見た目速度をさらにどれだけ落とすか")]
    [SerializeField, Range(0f, 1f)]
    float stairContinuousSlowAmount = 0.15f;

    [Header("Stair Rattle")]
    [Tooltip("段差を検知した瞬間の追加回転")]
    [SerializeField] float stairSpinImpulse = 180f;

    [Tooltip("段差で発生する横方向の乱れ")]
    [SerializeField] float stairSideSpinImpulse = 70f;

    [Tooltip("段差中の周期的な回転揺れ")]
    [SerializeField] float stairRattleSpinAmplitude = 35f;

    [SerializeField] float stairRattleFrequency = 14f;

    [Tooltip("段差中の見た目位置の上下揺れ")]
    [SerializeField] float stairVerticalRattleAmount = 0.04f;

    [Header("Visual Position")]
    [Tooltip("BallVisualの通常ローカル位置")]
    [SerializeField] Vector3 baseLocalPosition = Vector3.zero;

    [Tooltip("見た目が後ろへ引きずられる最大距離")]
    [SerializeField] float maxBackLagDistance = 0.22f;

    [Tooltip("後ろへの見た目遅れが追従する速さ")]
    [SerializeField] float visualPositionFollowSharpness = 12f;

    [Header("Debug")]
    [SerializeField] bool drawGizmos = true;

    Vector3 visualProgressVelocity;
    Vector3 visualSpinVelocity;
    Vector3 visualSpinKickVelocity;

    Quaternion visualSpinRotation = Quaternion.identity;

    bool isGrounded;
    bool isOnStairs;

    Vector3 groundNormal = Vector3.up;
    Collider currentGroundCollider;

    float stairBrakeUntil;
    float stairRattleTime;

    Vector3 currentVisualLocalPosition;

    void Awake()
    {
        if (!rootRb)
            rootRb = GetComponent<Rigidbody>();

        if (!visualBall && transform.childCount > 0)
            visualBall = transform.GetChild(0);

        if (visualBall)
        {
            baseLocalPosition = visualBall.localPosition;
            currentVisualLocalPosition = baseLocalPosition;
            visualSpinRotation = visualBall.localRotation;
        }
    }

    void OnEnable()
    {
        visualProgressVelocity = Vector3.zero;
        visualSpinVelocity = Vector3.zero;
        visualSpinKickVelocity = Vector3.zero;

        stairBrakeUntil = 0f;
        stairRattleTime = 0f;

        if (visualBall)
        {
            baseLocalPosition = visualBall.localPosition;
            currentVisualLocalPosition = baseLocalPosition;
            visualSpinRotation = visualBall.localRotation;
        }
    }

    void FixedUpdate()
    {
        ProbeGround();
    }

    void LateUpdate()
    {
        if (!rootRb || !visualBall)
            return;

        float dt = Time.deltaTime;
        if (dt <= 0f)
            return;

        UpdateVisualSpin(dt);
        UpdateVisualPosition(dt);

        visualBall.localRotation = visualSpinRotation;
        visualBall.localPosition = currentVisualLocalPosition;
    }

    void ProbeGround()
    {
        Vector3 origin = transform.TransformPoint(groundProbeOffset);

        bool hitGround = Physics.Raycast(
            origin,
            Vector3.down,
            out RaycastHit hit,
            groundProbeDistance,
            groundMask,
            QueryTriggerInteraction.Ignore
        );

        bool wasOnStairs = isOnStairs;

        isGrounded = hitGround;

        if (!hitGround)
        {
            groundNormal = Vector3.up;
            currentGroundCollider = null;
            isOnStairs = false;
            return;
        }

        groundNormal = hit.normal;
        currentGroundCollider = hit.collider;

        bool hasStairsTag =
            !string.IsNullOrEmpty(stairsTag) &&
            hit.collider.CompareTag(stairsTag);

        bool hasStairField =
            hit.collider.GetComponent<StairField>() != null;

        isOnStairs = hasStairsTag || hasStairField;

        if (!wasOnStairs && isOnStairs)
        {
            TriggerStairImpact(hit);
        }
    }

    void TriggerStairImpact(RaycastHit hit)
    {
        stairBrakeUntil = Mathf.Max(
            stairBrakeUntil,
            Time.time + stairBrakeSeconds
        );

        stairRattleTime = 0f;

        Transform heading = headingTransform
            ? headingTransform
            : rootRb.transform;

        Vector3 forward = heading.forward;
        Vector3 side = heading.right;

        AddVisualSpinImpulse(
            side,
            stairSpinImpulse
        );

        float randomSide =
            Random.Range(-1f, 1f) *
            stairSideSpinImpulse;

        AddVisualSpinImpulse(
            forward,
            randomSide
        );
    }

    public void AddVisualSpinImpulse(
        Vector3 worldAxis,
        float impulseDegreesPerSecond
    )
    {
        if (worldAxis.sqrMagnitude <= 0.000001f)
            return;

        if (Mathf.Abs(impulseDegreesPerSecond) <= 0.0001f)
            return;

        stairBrakeUntil = Mathf.Max(
            stairBrakeUntil,
            Time.time + stairBrakeSeconds
        );

        Transform parent = visualBall.parent;

        Vector3 localAxis = parent
            ? parent.InverseTransformDirection(worldAxis.normalized)
            : worldAxis.normalized;

        visualSpinKickVelocity +=
            localAxis *
            impulseDegreesPerSecond;
    }

    void UpdateVisualSpin(float dt)
    {
        Transform heading = headingTransform
            ? headingTransform
            : rootRb.transform;

        Vector3 forward = heading.forward;

        Vector3 realVelocity = rootRb.velocity;

        Vector3 realRollingVelocity =
            Vector3.ProjectOnPlane(
                realVelocity,
                groundNormal
            );

        bool visualBrakeActive =
            Time.time < stairBrakeUntil;

        Vector3 targetVisualVelocity =
            realRollingVelocity;

        if (isOnStairs)
        {
            targetVisualVelocity *=
                1f - stairContinuousSlowAmount;
        }

        if (visualBrakeActive)
        {
            targetVisualVelocity *=
                1f - stairBrakeAmount;
        }

        float progressFollow =
            1f - Mathf.Exp(
                -visualSpeedFollowSharpness * dt
            );

        visualProgressVelocity = Vector3.Lerp(
            visualProgressVelocity,
            targetVisualVelocity,
            progressFollow
        );

        Vector3 lag =
            realRollingVelocity -
            visualProgressVelocity;

        if (lag.magnitude > maxVisualSpeedLag)
        {
            visualProgressVelocity =
                realRollingVelocity -
                lag.normalized * maxVisualSpeedLag;
        }

        float visualSpeed =
            visualProgressVelocity.magnitude;

        Vector3 rollAxis =
            Vector3.Cross(
                groundNormal,
                visualProgressVelocity
            );

        Vector3 targetWorldSpin =
            rollAxis.sqrMagnitude > 0.000001f
                ? rollAxis.normalized *
                  (
                      visualSpeed /
                      Mathf.Max(visualRadius, 0.001f) *
                      Mathf.Rad2Deg *
                      normalSpinMultiplier
                  )
                : Vector3.zero;

        Transform parent = visualBall.parent;

        Vector3 targetLocalSpin = parent
            ? parent.InverseTransformDirection(targetWorldSpin)
            : targetWorldSpin;

        float spinFollow =
            1f - Mathf.Exp(
                -visualSpinFollowSharpness * dt
            );

        visualSpinVelocity = Vector3.Lerp(
            visualSpinVelocity,
            targetLocalSpin,
            spinFollow
        );

        if (isOnStairs)
        {
            stairRattleTime += dt;

            float rattle =
                Mathf.Sin(
                    stairRattleTime *
                    stairRattleFrequency *
                    Mathf.PI * 2f
                ) *
                stairRattleSpinAmplitude;

            Vector3 localForward = parent
                ? parent.InverseTransformDirection(forward)
                : forward;

            visualSpinKickVelocity +=
                localForward.normalized *
                rattle *
                dt;
        }

        float kickFade =
            1f - Mathf.Exp(
                -visualSpinDamping * dt
            );

        visualSpinKickVelocity = Vector3.Lerp(
            visualSpinKickVelocity,
            Vector3.zero,
            kickFade
        );

        Vector3 totalSpin =
            Vector3.ClampMagnitude(
                visualSpinVelocity +
                visualSpinKickVelocity,
                visualSpinMaxDegreesPerSecond
            );

        float degrees =
            totalSpin.magnitude * dt;

        if (degrees <= 0.00001f)
            return;

        visualSpinRotation =
            Quaternion.AngleAxis(
                degrees,
                totalSpin.normalized
            ) *
            visualSpinRotation;
    }

    void UpdateVisualPosition(float dt)
    {
        Transform heading = headingTransform
            ? headingTransform
            : rootRb.transform;

        float forwardSpeed =
            Vector3.Dot(
                rootRb.velocity,
                heading.forward
            );

        float visualForwardSpeed =
            Vector3.Dot(
                visualProgressVelocity,
                heading.forward
            );

        float speedDifference =
            Mathf.Max(
                0f,
                forwardSpeed - visualForwardSpeed
            );

        float backOffset = Mathf.Clamp(
            speedDifference * 0.03f,
            0f,
            maxBackLagDistance
        );

        Vector3 target =
            baseLocalPosition -
            Vector3.forward * backOffset;

        if (isOnStairs)
        {
            float vertical =
                Mathf.Sin(
                    stairRattleTime *
                    stairRattleFrequency *
                    Mathf.PI * 2f
                ) *
                stairVerticalRattleAmount;

            target.y += vertical;
        }

        float follow =
            1f - Mathf.Exp(
                -visualPositionFollowSharpness * dt
            );

        currentVisualLocalPosition = Vector3.Lerp(
            currentVisualLocalPosition,
            target,
            follow
        );
    }

    void OnDrawGizmosSelected()
    {
        if (!drawGizmos)
            return;

        Vector3 origin =
            transform.TransformPoint(groundProbeOffset);

        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(
            origin,
            origin + Vector3.down * groundProbeDistance
        );
    }
}