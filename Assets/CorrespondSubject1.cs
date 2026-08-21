using UnityEngine;
using System.Collections;

/// <summary>
/// PhysicsRoot上のInSubjectを、VisualPlayerRoot上の座標へ写してSubjectへ反映します。
/// PhysicsRootは回転しない物理計算系、VisualPlayerRootは回転する表示座標系です。
/// Subjectは代理体であり、独自の移動・接地・AddForce計算を行いません。
/// </summary>
[DefaultExecutionOrder(10000)]
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public sealed class CorrespondSubject1 : MonoBehaviour
{
    const float Epsilon = 0.000001f;

    [Header("Inertial Physics Frame")]
    [Tooltip("PhysicsRoot上で物理計算を行うInSubjectのRigidbodyです。")]
    [SerializeField] Rigidbody inSubjectBody;

    public SlopeStick3D inject;

    [Tooltip("回転させない物理座標系のRootです。")]
    [SerializeField] Transform physicsRoot;

    [Header("Rotating Visual Frame")]
    [Tooltip("画面上で回転する非慣性座標系のRootです。")]
    [SerializeField] Transform visualPlayerRoot;

    [Header("Mapped Subject")]
    [Tooltip("回転後座標を受け取るSubjectのRigidbodyです。Kinematicにしてください。")]
    [SerializeField] Rigidbody subjectBody;

    [Header("Synchronization")]
    [SerializeField] bool synchronizeInFixedUpdate = true;
    [SerializeField] bool logInvalidSetup = true;

    // Root参照を使用できない場合だけ使う互換用の手動回転です。
    Quaternion fallbackCoordinateRotation = Quaternion.identity;

    bool hasVelocitySample;
    float previousSampleTime;
    Vector3 previousMappedPosition;
    Quaternion previousMappedRotation = Quaternion.identity;
    public Vector3 mappedvelocity;
    public Vector3 mappedAngularVelocity;
    bool sameBodyErrorLogged;
    bool frameErrorLogged;

    public Rigidbody InSubjectBody => inSubjectBody;
    public Rigidbody SubjectBody => subjectBody;
    public Transform PhysicsRoot => physicsRoot;
    public Transform VisualPlayerRoot => visualPlayerRoot;
    public bool UsesRootFrames => physicsRoot && visualPlayerRoot;

    /// <summary>
    /// PhysicsRootからVisualPlayerRootへの現在の回転写像です。
    /// </summary>
    public Quaternion CoordinateRotation
    {
        get
        {
            if (UsesRootFrames)
            {
                return NormalizeSafe(
                    visualPlayerRoot.rotation *
                    Quaternion.Inverse(physicsRoot.rotation)
                );
            }

            return fallbackCoordinateRotation;
        }
    }

    void Start()
    {
        StartCoroutine(delayStart());
    }

    IEnumerator delayStart()
    {
        yield return new WaitForSeconds(0.8f);
        inject = inSubjectBody.transform.GetComponent<SlopeStick3D>();
    }

    /// <summary>
    /// InSubjectのPhysicsRoot内座標をVisualPlayerRoot内の同じ局所座標へ写した位置です。
    /// </summary>
    public Vector3 MappedPosition
    {
        get
        {
            if (!inSubjectBody)
                return subjectBody ? subjectBody.position : transform.position;

            if (UsesRootFrames)
            {
                if (inject != null)
                {
                    if (inject.groundKind == GroundKind.Flat)
                    {
                    }
                }

                Vector3 physicsLocalPosition = physicsRoot.InverseTransformPoint(inSubjectBody.position);

                return visualPlayerRoot.TransformPoint(physicsLocalPosition);
            }

            return fallbackCoordinateRotation * inSubjectBody.position;
        }
    }

    /// <summary>
    /// InSubjectの姿勢をVisualPlayerRoot側の姿勢へ写した結果です。
    /// InSubject自身の物理回転と、座標系の回転はここで初めて合成されます。
    /// </summary>
    void OnCollisionEnter(Collision collision)
    {
        // Debug.Log("");
    }

    public Quaternion MappedRotation =>
        inSubjectBody
        ? CoordinateRotation * inSubjectBody.rotation
        : subjectBody
        ? subjectBody.rotation
        : transform.rotation;

    /// <summary>
    /// VisualPlayerRootの回転による公転成分も含むSubject側の見かけの速度です。
    /// </summary>
    public Vector3 Mappedvelocity
    {
        get
        {
            if (hasVelocitySample)
            {
                return mappedvelocity;
            }

            if (inSubjectBody)
            {
                return MapDirection(inSubjectBody.velocity);
            }

            return Vector3.zero;
        }
    }

    public Vector3 MappedAngularVelocity =>
        hasVelocitySample
        ? mappedAngularVelocity
        : inSubjectBody
        ? MapDirection(inSubjectBody.angularVelocity)
        : Vector3.zero;

    void Awake()
    {
        if (!subjectBody)
            subjectBody = GetComponent<Rigidbody>();

        ConfigureSubjectBody();
        ValidateSetup();
    }

    void FixedUpdate()
    {
        if (synchronizeInFixedUpdate)
            SynchronizeNow();
    }

    /// <summary>
    /// SlopeStick3Dから物理本体と2つの座標系を結びます。
    /// </summary>
    public void Bind(
        Rigidbody sourceBody,
        Transform inertialPhysicsRoot,
        Transform rotatingVisualPlayerRoot)
    {
        if (sourceBody)
            inSubjectBody = sourceBody;

        if (inertialPhysicsRoot)
            physicsRoot = inertialPhysicsRoot;

        if (rotatingVisualPlayerRoot)
            visualPlayerRoot = rotatingVisualPlayerRoot;

        ConfigureSubjectBody();
        ValidateSetup();
    }

    /// <summary>
    /// 旧構成との互換用です。Root方式を使う場合は3引数Bindを使用してください。
    /// </summary>
    public void Bind(Rigidbody sourceBody, Transform unusedLegacyPivot)
    {
        if (sourceBody)
            inSubjectBody = sourceBody;

        ConfigureSubjectBody();
        ValidateSetup();
    }

    /// <summary>
    /// VisualPlayerRootをワールド空間のPivot回りに回します。
    /// InSubject、PhysicsRoot、InSubjectの速度・headingには触れません。
    /// </summary>
    public bool RotateVisualFrameAround(
        Vector3 pivot,
        Quaternion worldTurn,
        bool synchronizeImmediately = true)
    {
        if (!visualPlayerRoot)
        {
            if (logInvalidSetup && !frameErrorLogged)
            {
                frameErrorLogged = true;
                Debug.LogError(
                    "[CORRESPOND SUBJECT] VisualPlayerRootが未設定のため、表示座標系を回転できません。",
                    this
                );
            }

            return false;
        }

        Vector3 relativePosition = visualPlayerRoot.position - pivot;
        Vector3 rotatedPosition = pivot + worldTurn * relativePosition;
        Quaternion rotatedRotation = worldTurn * visualPlayerRoot.rotation;

        visualPlayerRoot.SetPositionAndRotation(
            rotatedPosition,
            NormalizeSafe(rotatedRotation)
        );

        if (synchronizeImmediately)
            SynchronizeNow();

        return true;
    }

    /// <summary>
    /// Root方式を使わない場合の互換APIです。
    /// Root方式ではCoordinateRotationはRoot同士の姿勢から自動導出されます。
    /// </summary>
    public void SetCoordinateRotation(
        Quaternion rotation,
        bool synchronizeImmediately = false)
    {
        fallbackCoordinateRotation = NormalizeSafe(rotation);

        if (synchronizeImmediately)
            SynchronizeNow();
    }

    public void ResetCoordinateRotation(bool synchronizeImmediately = true)
    {
        fallbackCoordinateRotation = Quaternion.identity;

        if (synchronizeImmediately)
            SynchronizeNow(true);
    }

    /// <summary>
    /// InSubjectの現在状態をSubjectへ反映します。
    /// resetDerivedVelocity=trueでは速度サンプルだけを現在値から取り直します。
    /// </summary>
    public void SynchronizeNow(bool resetDerivedVelocity = false)
    {
        if (!inSubjectBody || !subjectBody)
            return;

        if (subjectBody == inSubjectBody)
        {
            if (logInvalidSetup && !sameBodyErrorLogged)
            {
                sameBodyErrorLogged = true;
                Debug.LogError(
                    "[CORRESPOND SUBJECT] InSubject BodyとSubject Bodyへ同じRigidbodyを設定しないでください。",
                    this
                );
            }

            return;
        }

        ConfigureSubjectBody();

        Vector3 mappedPosition = MappedPosition;
        Quaternion mappedRotation = MappedRotation;

        UpdateMappedVelocities(
            mappedPosition,
            mappedRotation,
            resetDerivedVelocity
        );

        // InSubjectとSubjectは生成地点からそのまま開始します。
        // 初期配置待ち、基準取得、LateUpdate同期は使用しません。
        subjectBody.position = mappedPosition;
        subjectBody.rotation = mappedRotation;
    }

    public void ResetDerivedVelocitySample()
    {
        hasVelocitySample = false;
    }

    public Vector3 MapDirection(Vector3 inSubjectDirection)
    {
        return CoordinateRotation * inSubjectDirection;
    }

    public Vector3 InverseMapDirection(Vector3 subjectDirection)
    {
        return Quaternion.Inverse(CoordinateRotation) * subjectDirection;
    }

    public Vector3 MapPoint(Vector3 inSubjectPoint)
    {
        if (UsesRootFrames)
        {
            Vector3 physicsLocalPoint =
                physicsRoot.InverseTransformPoint(inSubjectPoint);

            return visualPlayerRoot.TransformPoint(physicsLocalPoint);
        }

        return fallbackCoordinateRotation * inSubjectPoint;
    }

    public Vector3 InverseMapPoint(Vector3 subjectPoint)
    {
        if (UsesRootFrames)
        {
            Vector3 visualLocalPoint =
                visualPlayerRoot.InverseTransformPoint(subjectPoint);

            return physicsRoot.TransformPoint(visualLocalPoint);
        }

        return Quaternion.Inverse(fallbackCoordinateRotation) * subjectPoint;
    }

    public Quaternion MapRotation(Quaternion inSubjectRotation)
    {
        return CoordinateRotation * inSubjectRotation;
    }

    public Quaternion InverseMapRotation(Quaternion subjectRotation)
    {
        return Quaternion.Inverse(CoordinateRotation) * subjectRotation;
    }

    void UpdateMappedVelocities(
        Vector3 currentPosition,
        Quaternion currentRotation,
        bool reset)
    {
        float sampleTime = Time.fixedTime;

        if (reset || !hasVelocitySample)
        {
            mappedvelocity = MapDirection(inSubjectBody.velocity);
            mappedAngularVelocity = MapDirection(inSubjectBody.angularVelocity);
            previousMappedPosition = currentPosition;
            previousMappedRotation = currentRotation;
            previousSampleTime = sampleTime;
            hasVelocitySample = true;
            return;
        }

        float dt = sampleTime - previousSampleTime;

        if (dt <= Epsilon)
            return;

        mappedvelocity =
            (currentPosition - previousMappedPosition) / dt;

        mappedAngularVelocity =
            AngularVelocityFromRotationDelta(
            previousMappedRotation,
            currentRotation,
            dt
        );

        previousMappedPosition = currentPosition;
        previousMappedRotation = currentRotation;
        previousSampleTime = sampleTime;
    }

    void ConfigureSubjectBody()
    {
        if (!subjectBody || subjectBody == inSubjectBody)
            return;

        subjectBody.useGravity = false;

        if (!subjectBody.isKinematic)
        {
            subjectBody.velocity = Vector3.zero;
            subjectBody.angularVelocity = Vector3.zero;
            // subjectBody.isKinematic = true;
        }
    }

    void ValidateSetup()
    {
        if (!logInvalidSetup)
            return;

        if (physicsRoot && visualPlayerRoot && physicsRoot == visualPlayerRoot)
        {
            Debug.LogError(
                "[CORRESPOND SUBJECT] PhysicsRootとVisualPlayerRootには別のTransformを設定してください。",
                this
            );
        }

        if (inSubjectBody && physicsRoot &&
            inSubjectBody.transform != physicsRoot &&
            !inSubjectBody.transform.IsChildOf(physicsRoot))
        {
            Debug.LogWarning(
                "[CORRESPOND SUBJECT] InSubjectはPhysicsRoot配下へ置くことを推奨します。",
                inSubjectBody
            );
        }

        if (subjectBody && subjectBody == inSubjectBody && !sameBodyErrorLogged)
        {
            sameBodyErrorLogged = true;
            Debug.LogError(
                "[CORRESPOND SUBJECT] InSubject BodyとSubject Bodyが同じです。",
                this
            );
        }
    }

    static Vector3 AngularVelocityFromRotationDelta(
        Quaternion previous,
        Quaternion current,
        float dt)
    {
        Quaternion delta =
            NormalizeSafe(current * Quaternion.Inverse(previous));

        if (delta.w < 0f)
        {
            delta.x = -delta.x;
            delta.y = -delta.y;
            delta.z = -delta.z;
            delta.w = -delta.w;
        }

        delta.ToAngleAxis(
            out float angleDegrees,
            out Vector3 axis
        );

        if (axis.sqrMagnitude <= Epsilon ||
            Mathf.Abs(angleDegrees) <= Epsilon)
        {
            return Vector3.zero;
        }

        if (angleDegrees > 180f)
            angleDegrees -= 360f;

        float angleRadians = angleDegrees * Mathf.Deg2Rad;

        return axis.normalized *
            (angleRadians / Mathf.Max(dt, Epsilon));
    }

    static Quaternion NormalizeSafe(Quaternion value)
    {
        float magnitude = Mathf.Sqrt(
            value.x * value.x +
            value.y * value.y +
            value.z * value.z +
            value.w * value.w
        );

        if (magnitude <= Epsilon)
            return Quaternion.identity;

        float inverse = 1f / magnitude;

        return new Quaternion(
            value.x * inverse,
            value.y * inverse,
            value.z * inverse,
            value.w * inverse
        );
    }
}
