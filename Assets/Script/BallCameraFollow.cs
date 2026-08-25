using System.Collections;
using UnityEngine;

public class BallCameraFollow : MonoBehaviour
{
    [Header("References")]
    [SerializeField] Transform ball;
    
    bool wasTurning;
    
    
    [SerializeField] CorrespondSubject respondSubject;
    [Header("Camera Follow")]
    [SerializeField] Vector3 offset = new Vector3(8f, 2.64f, 12f);
    [SerializeField] Vector3 fixedEulerAngles =
        new Vector3(24.1f, 212f, 0f);

    [Header("Occlusion Detection")]
    [Tooltip(
        "螺旋板・坂・床だけを含むLayerにします。" +
        "subject と BallVisual は含めません。"
    )]
    [SerializeField] LayerMask occlusionMask = ~0;

    [Tooltip("プレイヤーのどの高さから遮蔽を調べるかです。")]
    [SerializeField] float focusHeight = .45f;

    [Tooltip("視線を太くして、板の端をすり抜けにくくします。")]
    [SerializeField, Min(.01f)] float occlusionRadius = .25f;

    [Tooltip("プレイヤー直近のColliderを遮蔽物として誤検知しない距離です。")]
    [SerializeField, Min(0f)] float minOcclusionDistance = .05f;

    [Header("Slope Angle Filter")]
    [Tooltip(
        "水平面を0°とした、遮蔽物として扱う最低坂角度です。" +
        "0なら水平床も遮蔽候補になります。"
    )]
    [SerializeField, Range(0f, 89f)]
    float minOccluderSlopeAngle = 10f;

    [Header("Side View When Occluded")]
    [Tooltip("ONなら通常カメラ位置から見て右側へ90°回り込みます。")]
    [SerializeField] bool useRightSideWhenOccluded = true;

    [Tooltip("遮蔽時に真横へ移る速さです。")]
    [SerializeField, Min(.01f)] float sideViewSmoothTime = .14f;

    [Tooltip("遮蔽解除後、通常位置へ戻る速さです。")]
    [SerializeField, Min(.01f)] float normalViewSmoothTime = .16f;

    [Tooltip("遮蔽中にプレイヤーを見る回転の追従速度です。")]
    [SerializeField, Min(0f)] float rotationSharpness = 14f;

    [Header("Debug")]
    [SerializeField] bool drawOcclusionLine = true;
    [SerializeField] bool logOcclusionState;

    public bool IsViewOccluded { get; private set; }

    public RaycastHit LastOcclusionHit { get; private set; }

    public Collider OccludingCollider =>
        IsViewOccluded
            ? LastOcclusionHit.collider
            : null;

    public float OccludingSlopeAngle { get; private set; }

    readonly RaycastHit[] occlusionHits = new RaycastHit[16];

    bool wasViewOccluded;
    bool hasInitializedCamera;

    Vector3 cameraPositionVelocity;
    Vector3 occlusionSidePosition;
    Quaternion occlusionSideRotation = Quaternion.identity;

    void Awake()
    {
        transform.rotation = Quaternion.Euler(fixedEulerAngles);
    }
    IEnumerator Start()
    {
        yield return new WaitForSeconds(.25f);

        if (!ball)
        {
            GameObject subject = GameObject.Find("subject");

            if (subject)
                ball = subject.transform;
        }

        if (!ball)
            yield break;
        if (!respondSubject)
            respondSubject = ball.GetComponent<CorrespondSubject>();
        
        Vector3 normalPosition = ball.position + offset;

        transform.position = normalPosition;
        transform.rotation = Quaternion.Euler(fixedEulerAngles);

        hasInitializedCamera = true;
    }

    void LateUpdate()
    {
        if (!ball)
            return;

        Vector3 focus =
            ball.position + Vector3.up * focusHeight;

        Vector3 normalCameraPosition =
            ball.position + offset;
        bool turning =
            respondSubject != null &&
            respondSubject.IsVisualFrameTurning;

        if (turning)
        {
            transform.position =
                normalCameraPosition;

            transform.rotation =
                Quaternion.Euler(fixedEulerAngles);

            cameraPositionVelocity =
                Vector3.zero;

            wasTurning = true;

            LogCameraFollow(
                turning: true,
                justEnded: false);

            return;
        }

// 旋回終了直後の最初のLateUpdate
        if (wasTurning)
        {
            transform.position =
                normalCameraPosition;

            transform.rotation =
                Quaternion.Euler(fixedEulerAngles);

            cameraPositionVelocity =
                Vector3.zero;

            wasTurning = false;

            LogCameraFollow(
                turning: false,
                justEnded: true);

            return;
        }
        UpdateOcclusionState(
            focus,
            normalCameraPosition
        );

        Vector3 desiredPosition = IsViewOccluded
            ? occlusionSidePosition
            : normalCameraPosition;

        Quaternion desiredRotation = IsViewOccluded
            ? occlusionSideRotation
            : Quaternion.Euler(fixedEulerAngles);

        if (!hasInitializedCamera)
        {
            transform.position = desiredPosition;
            transform.rotation = desiredRotation;
            hasInitializedCamera = true;
            return;
        }

        float smoothTime = IsViewOccluded
            ? sideViewSmoothTime
            : normalViewSmoothTime;

        transform.position = Vector3.SmoothDamp(
            transform.position,
            desiredPosition,
            ref cameraPositionVelocity,
            smoothTime
        );

        float rotationFollow = 1f - Mathf.Exp(
            -rotationSharpness * Time.deltaTime
        );

        transform.rotation = Quaternion.Slerp(
            transform.rotation,
            desiredRotation,
            rotationFollow
        );
        
    }

    void UpdateOcclusionState(
        Vector3 focus,
        Vector3 normalCameraPosition
    )
    {
        bool isBlocked = TryFindOcclusion(
            focus,
            normalCameraPosition,
            out RaycastHit hit,
            out float slopeAngle
        );

        IsViewOccluded = isBlocked;

        if (drawOcclusionLine)
        {
            Debug.DrawLine(
                focus,
                normalCameraPosition,
                isBlocked ? Color.red : Color.green
            );
        }

        if (isBlocked)
        {
            LastOcclusionHit = hit;
            OccludingSlopeAngle = slopeAngle;

            if (!wasViewOccluded)
                OnOcclusionStarted(hit, slopeAngle);

            OnOcclusionStay(hit, slopeAngle);
        }
        else if (wasViewOccluded)
        {
            LastOcclusionHit = default;
            OccludingSlopeAngle = 0f;

            OnOcclusionEnded();
        }

        wasViewOccluded = isBlocked;
    }

    bool TryFindOcclusion(
        Vector3 focus,
        Vector3 cameraPosition,
        out RaycastHit nearestHit,
        out float nearestSlopeAngle
    )
    {
        nearestHit = default;
        nearestSlopeAngle = 0f;

        Vector3 line = cameraPosition - focus;
        float distance = line.magnitude;

        if (distance <= 1e-6f)
            return false;

        Vector3 direction = line / distance;

        int hitCount = Physics.SphereCastNonAlloc(
            focus,
            occlusionRadius,
            direction,
            occlusionHits,
            distance,
            occlusionMask,
            QueryTriggerInteraction.Ignore
        );

        float nearestDistance = float.PositiveInfinity;
        bool found = false;

        for (int i = 0; i < hitCount; i++)
        {
            RaycastHit hit = occlusionHits[i];

            if (!hit.collider)
                continue;

            if (hit.distance < minOcclusionDistance)
                continue;

            if (IsBallHierarchy(hit.collider.transform))
                continue;

            float slopeAngle = Vector3.Angle(
                hit.normal,
                Vector3.up
            );

            if (slopeAngle < minOccluderSlopeAngle)
                continue;

            if (hit.distance >= nearestDistance)
                continue;

            nearestDistance = hit.distance;
            nearestHit = hit;
            nearestSlopeAngle = slopeAngle;
            found = true;
        }

        return found;
    }

    bool IsBallHierarchy(Transform hitTransform)
    {
        if (!ball || !hitTransform)
            return false;

        return hitTransform == ball ||
               hitTransform.IsChildOf(ball) ||
               ball.IsChildOf(hitTransform);
    }

    void OnOcclusionStarted(
        RaycastHit hit,
        float slopeAngle
    )
    {
        if (!logOcclusionState)
            return;

        Debug.Log(
            "遮蔽開始: " +
            hit.collider.name +
            " / 距離: " +
            hit.distance.ToString("F2") +
            " / 坂角度: " +
            slopeAngle.ToString("F1") +
            "°"
        );
    }

    void OnOcclusionStay(
        RaycastHit hit,
        float slopeAngle
    )
    {
        Vector3 focus =
            ball.position + Vector3.up * focusHeight;

        // 通常カメラ位置を、プレイヤー中心を軸に90°だけ回す。
        // offset.y は変わらないので、通常時と同じ高さを保ちます。
        float sideAngle =
            useRightSideWhenOccluded ? -90f : 90f;

        Vector3 sideOffset =
            Quaternion.AngleAxis(
                sideAngle,
                Vector3.up
            ) * offset;

        occlusionSidePosition =
            ball.position + sideOffset;

        Vector3 lookDirection =
            focus - occlusionSidePosition;

        if (lookDirection.sqrMagnitude <= 1e-6f)
            return;

        occlusionSideRotation = Quaternion.LookRotation(
            lookDirection.normalized,
            Vector3.up
        );
    }
    

    void OnOcclusionEnded()
    {
        if (logOcclusionState)
            Debug.Log("遮蔽終了");
    }
    void LogCameraFollow(
        bool turning,
        bool justEnded)
    {
        Vector3 target =
            ball.position + offset;

        Vector3 cameraError =
            transform.position - target;

        Debug.Log(
            $"[CAMERA FOLLOW] " +
            $"turning={turning} " +
            $"justEnded={justEnded} " +
            $"error={cameraError.magnitude:F4} " +
            $"camera={transform.position:F4} " +
            $"target={target:F4}"
        );
    }
}
