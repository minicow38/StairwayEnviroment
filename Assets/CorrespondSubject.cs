using UnityEngine;
using DG.Tweening;

/// <summary>
/// PhysicsRoot上のInSubjectを、VisualPlayerRoot上の座標へ写してSubjectへ反映します。
/// PhysicsRootは回転しない物理計算系、VisualPlayerRootは回転する表示座標系です。
/// Subjectは代理体であり、独自の移動・接地・AddForce計算を行いません。
/// </summary>
[DefaultExecutionOrder(10000)]
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public sealed class CorrespondSubject : MonoBehaviour
{
    private bool initialChainLogged;
    const float Epsilon = 0.000001f;
  // public int CurrentPointToPlane = 0;

    [Header("Inertial Physics Frame")] [Tooltip("PhysicsRoot上で物理計算を行うInSubjectのRigidbodyです。")] [SerializeField]
    Rigidbody inSubjectBody;


    [Tooltip("回転させない物理座標系のRootです。")] [SerializeField]
    Transform physicsRoot;

    [Header("Rotating Visual Frame")] [Tooltip("画面上で回転する非慣性座標系のRootです。")] [SerializeField]
    Transform visualPlayerRoot;

    [Header("Turn Followers")] [SerializeField]
    Rigidbody ballVisualBody;

    [Header("Mapped Subject")] [Tooltip("回転後座標を受け取るSubjectのRigidbodyです。Kinematicにしてください。")] [SerializeField]
    Rigidbody subjectBody;

    [Header("Stage DOTween Turn")] [Tooltip("回転させるステージ全体のRootです。未設定ならStage Pivotを使用します。")] [SerializeField]
    Transform stageRoot;

    [Tooltip("ステージが回る中心です。未設定ならCenter1を検索します。")] [SerializeField]
    Transform stagePivot;

    [Tooltip("ONなら指定した旋回方向と逆方向へステージを回します。")] [SerializeField]
    bool stageTurnsOppositeToPlayer = true;

    [SerializeField, Range(0f, 180f)] float turnAngle = 45f;
    [SerializeField, Min(.01f)] float turnDuration = .45f;
    [SerializeField] Ease turnEase = Ease.InOutCubic;

    Tween turnTween;

    // Scene初期状態のVisual/Stage座標系を保存します。
    // VisualPlayerRootはworld Pose、StageRootは親との関係を保つためlocal Poseで保持します。
    bool hasInitialVisualPlayerRootPose;
    Vector3 initialVisualPlayerRootPosition;
    Quaternion initialVisualPlayerRootRotation = Quaternion.identity;

    bool hasInitialStageRootPose;
    Vector3 initialStageRootLocalPosition;
    Quaternion initialStageRootLocalRotation = Quaternion.identity;

    [Header("Synchronization")] [SerializeField]
    bool synchronizeInFixedUpdate = true;

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
    
    public bool IsVisualFrameTurning =>
        turnTween != null &&
        turnTween.IsActive() &&
        turnTween.IsPlaying();

    public Rigidbody InSubjectBody => inSubjectBody;
    public Rigidbody SubjectBody => subjectBody;
    public Transform PhysicsRoot => physicsRoot;
    public Transform VisualPlayerRoot => visualPlayerRoot;
    public bool UsesRootFrames => physicsRoot && visualPlayerRoot;

    /// <summary>
    /// PhysicsRootからVisualPlayerRootへの現在の回転写像です。
    /// </summary>
    ///
    void Start()
    {
       // PointToPlane=AndroidOneOnly.bestScore;
    }
    
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

    public Vector3 MappedPhysicalVelocity
    {
        get
        {
            if (!inSubjectBody)
                return Vector3.zero;

            return MapDirection(inSubjectBody.velocity);
        }
    }

    public Vector3 MappedPhysicalAngularVelocity
    {
        get
        {
            if (!inSubjectBody)
                return Vector3.zero;

            return MapDirection(inSubjectBody.angularVelocity);
        }
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
    private void DebugChainCheckpoint(string reason)
    {
        if (!inSubjectBody || !subjectBody)
            return;

        Vector3 mappedPos = MappedPosition;

        Vector3 mappedVelocityDirect =
            MapDirection(inSubjectBody.velocity);

        float subjectPositionError =
            Vector3.Distance(
                subjectBody.position,
                mappedPos);

        Vector3 backToPhysics =
            InverseMapPoint(mappedPos);

        float roundTripError =
            Vector3.Distance(
                inSubjectBody.position,
                backToPhysics);

        Debug.Log(
            $"[CHAIN MAP] " +
            $"reason={reason} " +
            $"time={Time.fixedTime:F3} " +
            $"turn={MainGameManager.PointToPlane} " +
            $"inPos={inSubjectBody.position:F3} " +
            $"mappedPos={mappedPos:F3} " +
            $"subjectPos={subjectBody.position:F3} " +
            $"subjectErr={subjectPositionError:F6} " +
            $"roundTripErr={roundTripError:F6} " +
            $"inVel={inSubjectBody.velocity:F3} " +
            $"mappedVel={mappedVelocityDirect:F3}"
        );
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
        if (!ballVisualBody)
        {
            GameObject ball =
                GameObject.Find("BallVisual");

            if (ball)
                ballVisualBody =
                    ball.GetComponent<Rigidbody>();
        }

        FindStageTurnReferences();
        CaptureInitialFramePose();
        ConfigureSubjectBody();
        ValidateSetup();
    }

    void FixedUpdate()
    {
        if (synchronizeInFixedUpdate)
            SynchronizeNow();
    }

    /// <summary>
    /// 物理コントローラからInSubject本体と2つの座標系を結びます。
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

        FindStageTurnReferences();
        CaptureInitialFramePose();
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
    /// Stage DOTween Turn用の参照を補完します。
    /// Stage Pivot未設定時はCenter1を検索し、Stage Root未設定時はStage Pivotを使用します。
    /// </summary>
    void FindStageTurnReferences()
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

    /// <summary>
    /// まだ一度も旋回していないScene初期状態を保存します。
    /// 未取得の参照だけ保存するため、Bind後に呼び直しても初期値を上書きしません。
    /// </summary>
    public void CaptureInitialFramePose()
    {
        FindStageTurnReferences();

        if (!hasInitialVisualPlayerRootPose && visualPlayerRoot)
        {
            initialVisualPlayerRootPosition =
                visualPlayerRoot.position;

            initialVisualPlayerRootRotation =
                visualPlayerRoot.rotation;

            hasInitialVisualPlayerRootPose = true;
        }

        if (!hasInitialStageRootPose &&
            stageRoot &&
            stageRoot != visualPlayerRoot)
        {
            initialStageRootLocalPosition =
                stageRoot.localPosition;

            initialStageRootLocalRotation =
                stageRoot.localRotation;

            hasInitialStageRootPose = true;
        }
    }

    /// <summary>
    /// 現在動いているStage/Visual旋回Tweenだけを停止します。
    /// 今後のRotateVisualFrameAround()/BeginStageTurn()は通常どおり再作成できます。
    /// </summary>
    public bool CancelVisualFrameTurn(
        bool synchronizeImmediately = false)
    {
        bool cancelled = false;

        if (turnTween != null)
        {
            if (turnTween.IsActive())
            {
                turnTween.Kill();
                cancelled = true;
            }

            turnTween = null;
        }

        if (synchronizeImmediately)
            SynchronizeNow(true);

        return cancelled;
    }

    /// <summary>
    /// VisualPlayerRootと、別Transformとして回転したStageRootをScene初期Poseへ戻します。
    /// 先にTweenを停止するため、復元直後に古いTweenから再上書きされません。
    /// </summary>
    public bool RestoreInitialFramePose(
        bool synchronizeImmediately = true)
    {
        FindStageTurnReferences();
        CaptureInitialFramePose();

        // 古いTweenだけを停止してからPoseを戻す。
        // Rigidbody.velocityやInSubjectのdirectionには触れない。
        CancelVisualFrameTurn(false);

        bool restored = false;

        if (hasInitialVisualPlayerRootPose && visualPlayerRoot)
        {
            visualPlayerRoot.SetPositionAndRotation(
                initialVisualPlayerRootPosition,
                initialVisualPlayerRootRotation);

            restored = true;
        }

        // StageRootはlocal Poseで戻すことで、VisualPlayerRootとの親子関係があっても破綻しません。
        if (hasInitialStageRootPose &&
            stageRoot &&
            stageRoot != visualPlayerRoot)
        {
            stageRoot.localPosition =
                initialStageRootLocalPosition;

            stageRoot.localRotation =
                initialStageRootLocalRotation;

            restored = true;
        }

        Physics.SyncTransforms();

        if (synchronizeImmediately)
            SynchronizeNow(true);

        return restored;
    }

    public void TurnStageLeft()
    {
        BeginStageTurn(-Mathf.Abs(turnAngle));
    }

    public void TurnStageRight()
    {
        BeginStageTurn(Mathf.Abs(turnAngle));
    }

    /// <summary>
    /// Stage RootをStage Pivot中心にDOTweenで回転させます。
    /// InSubject、PhysicsRoot、InSubjectの速度・angularVelocityには触れません。
    /// </summary>
    public void BeginStageTurn(float playerAngle)
    {
        FindStageTurnReferences();
        CaptureInitialFramePose();

        if (!stageRoot || !stagePivot)
        {
            if (logInvalidSetup)
            {
                Debug.LogWarning(
                    "[CORRESPOND SUBJECT] Stage RootまたはStage Pivotが未設定のため、ステージ回転を開始できません。",
                    this
                );
            }

            return;
        }

        Vector3 pivot = stagePivot.position;
        Vector3 stageStartPosition = stageRoot.position;
        Quaternion stageStartRotation = stageRoot.rotation;

        float stageMultiplier =
            stageTurnsOppositeToPlayer ? -1f : 1f;

        CancelVisualFrameTurn(false);

        void Apply(float angle)
        {
            Quaternion stageTurn =
                Quaternion.AngleAxis(
                    angle * stageMultiplier,
                    Vector3.up
                );

            ApplyStageRotation(
                stageTurn,
                pivot,
                stageStartPosition,
                stageStartRotation
            );
        }

        turnTween = DOTween.To(
                () => 0f,
                Apply,
                playerAngle,
                turnDuration
            )
            .SetEase(turnEase)
            .SetUpdate(UpdateType.Fixed)
            .OnComplete(() =>
            {
                //PointToPlane++;
                Apply(playerAngle);
                turnTween = null;
            });
    }

    /// <summary>
    /// Stage RootをPivot中心に公転させながら、そのRoot自身も同じ角度だけ回転させます。
    /// </summary>
    void ApplyStageRotation(
        Quaternion stageTurn,
        Vector3 pivot,
        Vector3 stageStartPosition,
        Quaternion stageStartRotation)
    {
        if (!stageRoot)
            return;

        stageRoot.SetPositionAndRotation(
            pivot +
            stageTurn *
            (stageStartPosition - pivot),

            NormalizeSafe(
                stageTurn *
                stageStartRotation
            )
        );
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
            return false;

        FindStageTurnReferences();
        CaptureInitialFramePose();
        CancelVisualFrameTurn(false);

        Vector3 visualStartPosition =
            visualPlayerRoot.position;

        Quaternion visualStartRotation =
            visualPlayerRoot.rotation;

        Vector3 visualRelative =
            visualStartPosition - pivot;
    

        // StageRootがVisualPlayerRootと別系統の場合の開始姿勢
        bool rotateStageSeparately =
            stageRoot &&
            stageRoot != visualPlayerRoot &&
            !stageRoot.IsChildOf(visualPlayerRoot) &&
            !visualPlayerRoot.IsChildOf(stageRoot);

        Vector3 stageStartPosition =
            stageRoot
                ? stageRoot.position
                : Vector3.zero;

        Quaternion stageStartRotation =
            stageRoot
                ? stageRoot.rotation
                : Quaternion.identity;

        Vector3 stageRelative =
            stageStartPosition - pivot;

        void Apply(float t)
        {
            Quaternion currentTurn =
                Quaternion.Slerp(
                    Quaternion.identity,
                    worldTurn,
                    t
                );

            // -------------------------
            // VisualPlayerRoot
            // -------------------------
            visualPlayerRoot.SetPositionAndRotation(
                pivot +
                currentTurn *
                visualRelative,

                NormalizeSafe(
                    currentTurn *
                    visualStartRotation
                )
            );

            if (logInvalidSetup)
            {
                Debug.Log(
                    $"[TURN FRAME] " +
                    $"pivot={pivot:F4} " +
                    $"currentTurn={currentTurn.eulerAngles:F2} " +
                    $"visualRelative={visualRelative:F4} " +
                    $"stageRelative={stageRelative:F4} " +
                    $"visualPlayerRoot={visualPlayerRoot.position:F4} " +
                    $"stageRoot={(stageRoot ? stageRoot.position.ToString("F4") : "N/A")} " +
                    $"mappedSubject={MappedPosition:F4}"
                );
            }

            // -------------------------
            // VisualStage
            // -------------------------
            // StageとVisualPlayerRootが兄弟の場合だけ、
            // 同じQuaternion・同じPivot・同じtで回す。
            if (rotateStageSeparately)
            {
                stageRoot.SetPositionAndRotation(
                    pivot +
                    currentTurn *
                    stageRelative,

                    NormalizeSafe(
                        currentTurn *
                        stageStartRotation
                    )
                );
            }

            // ★ここが今回最重要
            // Stage/VisualRootを動かした「同じTween tick」で
            // SubjectとBallVisualも移動させる。
            if (synchronizeImmediately)
                SynchronizeTurnFollowers();
        }

        turnTween = DOTween.To(
                () => 0f,
                Apply,
                1f,
                turnDuration
            )
            .SetEase(turnEase)
            .SetUpdate(UpdateType.Fixed)
            .OnComplete(() =>
            {
                //Time.timeScale = 0.125f;
                MainGameManager.PointToPlane++;
                Apply(1f);
                turnTween = null;
            });

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
        
        if (!initialChainLogged)
        {
            initialChainLogged = true;
            DebugChainCheckpoint("Initial");
        }

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

    void SynchronizeTurnFollowers()
    {
        // まずSubjectを現在のVisualPlayerRootへ写す
        SynchronizeNow(true);

        // BallVisualも「同じ瞬間」に写す
        if (ballVisualBody)
        {
            ballVisualBody.transform.localPosition =
                visualPlayerRoot.InverseTransformPoint(MappedPosition);

            ballVisualBody.transform.localRotation =
                Quaternion.Inverse(visualPlayerRoot.rotation) *
                MappedRotation;

            if (ballVisualBody.isKinematic)
            {
                ballVisualBody.velocity = Vector3.zero;
                ballVisualBody.angularVelocity = Vector3.zero;
            }
        }
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

    void OnDestroy()
    {
        CancelVisualFrameTurn(false);
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