using UnityEngine;

/// <summary>
/// 「Stairwayらしい動き」を、既存コードに依存せず組み直した試作です。
///
/// ねらい
/// 1. PhysicsBallは安定した前進と接地だけを担当する。
/// 2. BallVisualは平面終端で一時的に独立し、流星のような放物軌道を描く。
/// 3. 着地時は斜面の接線速度へつなぎ、LerpではなくHermite曲線で滑らかに合流する。
/// 4. 回転は実際に見た目玉が進んだ距離から計算する。
/// 5. フリック時は進行方向とステージを短時間で反対方向へ回す。
///
/// 推奨Hierarchy
/// subject  ← Rigidbody / Collider / このScript
/// └─ BallVisual  ← MeshRendererのみ。RigidbodyとColliderは付けない
///
/// StageRootとStagePivotは任意です。
/// </summary>
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public sealed class GptStaiway: MonoBehaviour
{
    enum VisualMotionState
    {
        FollowSubject,
        BallisticDrop,
        LandingHandoff
    }

    struct GroundHit
    {
        public bool grounded;
        public RaycastHit hit;
        public Vector3 point;
        public Vector3 normal;
        public float slopeAngle;
    }

    [Header("References")]
    [SerializeField] Rigidbody physicsBody;
    [SerializeField] Transform ballVisual;
    [SerializeField] Transform headingReference;
    [SerializeField] Transform stageRoot;
    [SerializeField] Transform stagePivot;

    [Header("PhysicsBall - Stable Slide")]
    [SerializeField, Min(.01f)] float sphereRadius = .5f;
    [SerializeField, Min(.01f)] float groundProbeDistance = .3f;
    [SerializeField] LayerMask groundMask = ~0;
    [SerializeField, Range(0f, 89f)] float maxGroundAngle = 75f;

    [SerializeField, Min(0f)] float targetSpeed = 10f;
    [SerializeField, Min(0f)] float groundAcceleration = 38f;
    [SerializeField, Min(0f)] float airAcceleration = 8f;
    [SerializeField, Min(0f)] float lateralDamping = 10f;
    [SerializeField, Min(0f)] float groundStickAcceleration = 8f;

    [Header("Heading / Stage Turn")]
    [SerializeField] Vector3 initialHeading = Vector3.right;
    [SerializeField, Min(1f)] float minimumFlickPixels = 20f;
    [SerializeField, Range(1f, 90f)] float turnAngle = 45f;
    [SerializeField, Min(.01f)] float turnDuration = .17f;
    [SerializeField] bool invertHorizontalInput;
    [SerializeField] bool rotateStage = true;
    [SerializeField] bool stageTurnsOppositeToHeading = true;

    [Header("BallVisual - Meteor Edge Drop")]
    [SerializeField] bool useMeteorEdgeDrop = true;

    [Tooltip("現在地点から前方の、次の面を探す距離です。")]
    [SerializeField, Min(.01f)] float edgeLookAhead = .55f;

    [Tooltip("前方Probeを開始する高さです。")]
    [SerializeField, Min(.01f)] float edgeProbeHeight = 1.2f;

    [Tooltip("前方Probeが下へ探す距離です。")]
    [SerializeField, Min(.1f)] float edgeProbeDepth = 4f;

    [Tooltip("これ以上の高低差または面法線差で、平面終端とみなします。")]
    [SerializeField, Min(0f)] float minimumDrop = .04f;

    [Tooltip("次の面との法線角度差です。")]
    [SerializeField, Range(0f, 90f)] float minimumNormalChange = 4f;

    [Tooltip("見た目用の重力倍率です。1でPhysics.gravityと同じです。")]
    [SerializeField, Range(.1f, 2f)] float meteorGravityScale = .65f;

    [Tooltip("BallVisualが独立軌道を進める最大時間です。")]
    [SerializeField, Range(.04f, .4f)] float maximumBallisticSeconds = .18f;

    [Tooltip("斜面到達後、subjectへ滑るように合流する時間です。")]
    [SerializeField, Range(.01f, .2f)] float landingHandoffSeconds = .055f;

    [Tooltip("同じ境界で何度も発動しないための待ち時間です。")]
    [SerializeField, Min(0f)] float edgeDropCooldown = .12f;

    [Header("BallVisual - Roll / Trick")]
    [SerializeField, Min(.01f)] float visualRollRadius = .5f;
    [SerializeField, Min(0f)] float visualRollMultiplier = 1f;
    [SerializeField, Min(0f)] float maximumRollDegreesPerSecond = 1800f;
    [SerializeField, Min(.1f)] float visualFollowSharpness = 32f;

    [Tooltip("着地時に加える横回転速度です。BallVisualだけに作用します。")]
    [SerializeField, Min(0f)] float landingSideSpin = 900f;

    [Tooltip("着地時に混ぜる進行軸回転です。")]
    [SerializeField, Min(0f)] float landingBankSpin = 220f;

    [SerializeField, Min(.1f)] float landingSpinDamping = 8f;

    [Header("Debug")]
    [SerializeField] bool drawDebug = true;
    [SerializeField] Color edgeProbeColor = Color.yellow;
    [SerializeField] Color predictedLandingColor = Color.magenta;

    GroundHit ground;

    Vector3 headingDirection;

    Vector2 pointerStart;
    bool pointerTracking;

    bool turnActive;
    float turnAge;
    float activeTurnAngle;
    Vector3 turnStartHeading;
    Vector3 stageStartPosition;
    Quaternion stageStartRotation;

    Vector3 visualRestLocalPosition;
    Quaternion visualRestLocalRotation;
    Vector3 visualRestLocalScale;

    VisualMotionState visualState = VisualMotionState.FollowSubject;

    Vector3 ballisticStartWorldPoint;
    Vector3 ballisticLaunchVelocity;
    Vector3 ballisticGravity;
    Vector3 ballisticPreviousWorldPoint;
    float ballisticAge;
    float ballisticContactTime;

    Vector3 landingPlanePoint;
    Vector3 landingPlaneNormal = Vector3.up;

    Vector3 handoffStartWorldPoint;
    Vector3 handoffStartVelocity;
    float handoffAge;

    Quaternion distanceRollRotation = Quaternion.identity;
    Quaternion trickSpinRotation = Quaternion.identity;
    Vector3 trickSpinVelocityLocal;

    Vector3 previousVisualWorldPoint;
    bool hasPreviousVisualWorldPoint;

    float nextEdgeDropTime;

    Vector3 lastPredictedLandingPoint;
    bool hasPredictedLandingPoint;

    void Reset()
    {
        physicsBody = GetComponent<Rigidbody>();
        headingReference = transform;

        Transform foundVisual = transform.Find("BallVisual");
        if (foundVisual)
            ballVisual = foundVisual;
    }

    void Awake()
    {
        ResolveReferences();

        headingDirection = FlatDirection(
            initialHeading,
            transform.forward
        );

        if (ballVisual)
        {
            visualRestLocalPosition = ballVisual.localPosition;
            visualRestLocalRotation = ballVisual.localRotation;
            visualRestLocalScale = ballVisual.localScale;

            previousVisualWorldPoint = ballVisual.position;
            hasPreviousVisualWorldPoint = true;
        }
    }

    void Update()
    {
        ReadPointerInput();
    }

    void FixedUpdate()
    {
        StepTurn(Time.fixedDeltaTime);
        ProbeGround();
        ApplyPhysicsMotion();

        if (useMeteorEdgeDrop)
            TryBeginMeteorDrop();
    }

    void LateUpdate()
    {
        StepBallVisual(Time.deltaTime);
    }

    void ResolveReferences()
    {
        if (!physicsBody)
            physicsBody = GetComponent<Rigidbody>();

        if (!headingReference)
            headingReference = transform;

        if (!ballVisual)
        {
            Transform foundVisual = transform.Find("BallVisual");
            if (foundVisual)
                ballVisual = foundVisual;
        }
    }

    void ProbeGround()
    {
        ground = default;
        ground.normal = Vector3.up;

        Vector3 origin =
            physicsBody.position + Vector3.up * .08f;

        float castRadius =
            Mathf.Max(.01f, sphereRadius * .92f);

        float castDistance =
            sphereRadius + groundProbeDistance;

        bool hitGround = Physics.SphereCast(
            origin,
            castRadius,
            Vector3.down,
            out RaycastHit hit,
            castDistance,
            groundMask,
            QueryTriggerInteraction.Ignore
        );

        if (!hitGround)
            return;

        float angle =
            Vector3.Angle(hit.normal, Vector3.up);

        if (angle > maxGroundAngle)
            return;

        ground.grounded = true;
        ground.hit = hit;
        ground.point = hit.point;
        ground.normal = hit.normal.normalized;
        ground.slopeAngle = angle;
    }

    void ApplyPhysicsMotion()
    {
        if (!physicsBody)
            return;

        if (!ground.grounded)
        {
            ApplyAirMotion();
            return;
        }

        Vector3 forward =
            Vector3.ProjectOnPlane(
                headingDirection,
                ground.normal
            );

        if (forward.sqrMagnitude <= 1e-6f)
            return;

        forward.Normalize();

        Vector3 velocity = physicsBody.velocity;
        Vector3 tangentVelocity =
            Vector3.ProjectOnPlane(
                velocity,
                ground.normal
            );

        float forwardSpeed =
            Vector3.Dot(tangentVelocity, forward);

        Vector3 lateralVelocity =
            tangentVelocity -
            forward * forwardSpeed;

        float speedError =
            targetSpeed - forwardSpeed;

        float forwardAcceleration =
            Mathf.Clamp(
                speedError /
                Mathf.Max(Time.fixedDeltaTime, .001f),
                -groundAcceleration,
                groundAcceleration
            );

        Vector3 lateralAcceleration =
            -lateralVelocity * lateralDamping;

        lateralAcceleration =
            Vector3.ClampMagnitude(
                lateralAcceleration,
                groundAcceleration
            );

        physicsBody.AddForce(
            forward * forwardAcceleration +
            lateralAcceleration,
            ForceMode.Acceleration
        );

        physicsBody.AddForce(
            -ground.normal * groundStickAcceleration,
            ForceMode.Acceleration
        );
    }

    void ApplyAirMotion()
    {
        Vector3 flatHeading =
            FlatDirection(
                headingDirection,
                transform.forward
            );

        Vector3 horizontalVelocity =
            Vector3.ProjectOnPlane(
                physicsBody.velocity,
                Vector3.up
            );

        float currentForwardSpeed =
            Vector3.Dot(
                horizontalVelocity,
                flatHeading
            );

        float speedError =
            targetSpeed - currentForwardSpeed;

        float acceleration =
            Mathf.Clamp(
                speedError,
                -airAcceleration,
                airAcceleration
            );

        physicsBody.AddForce(
            flatHeading * acceleration,
            ForceMode.Acceleration
        );
    }

    void ReadPointerInput()
    {
        if (Input.touchCount > 0)
        {
            Touch touch = Input.GetTouch(0);

            if (touch.phase == TouchPhase.Began)
            {
                pointerStart = touch.position;
                pointerTracking = true;
            }
            else if (
                pointerTracking &&
                (touch.phase == TouchPhase.Ended ||
                 touch.phase == TouchPhase.Canceled))
            {
                HandleFlick(
                    touch.position - pointerStart
                );

                pointerTracking = false;
            }

            return;
        }

        if (Input.GetMouseButtonDown(0))
        {
            pointerStart = Input.mousePosition;
            pointerTracking = true;
        }
        else if (
            Input.GetMouseButtonUp(0) &&
            pointerTracking)
        {
            Vector2 end = Input.mousePosition;
            HandleFlick(end - pointerStart);
            pointerTracking = false;
        }
    }

    void HandleFlick(Vector2 delta)
    {
        if (turnActive)
            return;

        if (delta.magnitude < minimumFlickPixels)
            return;

        if (Mathf.Abs(delta.x) <= Mathf.Abs(delta.y))
            return;

        float sign = Mathf.Sign(delta.x);

        if (invertHorizontalInput)
            sign *= -1f;

        BeginTurn(sign * turnAngle);
    }

    void BeginTurn(float angle)
    {
        turnActive = true;
        turnAge = 0f;
        activeTurnAngle = angle;
        turnStartHeading = headingDirection;

        if (stageRoot)
        {
            stageStartPosition = stageRoot.position;
            stageStartRotation = stageRoot.rotation;
        }
    }

    void StepTurn(float deltaTime)
    {
        if (!turnActive)
            return;

        turnAge += deltaTime;

        float t = Mathf.Clamp01(
            turnAge /
            Mathf.Max(turnDuration, .001f)
        );

        float eased =
            t * t * (3f - 2f * t);

        Quaternion headingDelta =
            Quaternion.AngleAxis(
                activeTurnAngle * eased,
                Vector3.up
            );

        headingDirection =
            FlatDirection(
                headingDelta * turnStartHeading,
                turnStartHeading
            );

        if (headingReference &&
            headingReference != transform)
        {
            headingReference.rotation =
                Quaternion.LookRotation(
                    headingDirection,
                    Vector3.up
                );
        }

        if (rotateStage &&
            stageRoot &&
            stagePivot)
        {
            float stageSign =
                stageTurnsOppositeToHeading
                    ? -1f
                    : 1f;

            Quaternion stageDelta =
                Quaternion.AngleAxis(
                    activeTurnAngle *
                    stageSign *
                    eased,
                    Vector3.up
                );

            Vector3 offset =
                stageStartPosition -
                stagePivot.position;

            stageRoot.SetPositionAndRotation(
                stagePivot.position +
                stageDelta * offset,
                stageDelta * stageStartRotation
            );
        }

        if (t >= 1f)
        {
            headingDirection =
                FlatDirection(
                    Quaternion.AngleAxis(
                        activeTurnAngle,
                        Vector3.up
                    ) * turnStartHeading,
                    turnStartHeading
                );

            turnActive = false;
        }
    }

    void TryBeginMeteorDrop()
    {
        if (!ground.grounded)
            return;

        if (visualState !=
            VisualMotionState.FollowSubject)
        {
            return;
        }

        if (Time.time < nextEdgeDropTime)
            return;

        Vector3 forward =
            Vector3.ProjectOnPlane(
                headingDirection,
                ground.normal
            );

        if (forward.sqrMagnitude <= 1e-6f)
            return;

        forward.Normalize();

        Vector3 probeCenter =
            physicsBody.position +
            forward * edgeLookAhead;

        Vector3 probeOrigin =
            probeCenter +
            Vector3.up * edgeProbeHeight;

        bool foundNextSurface = Physics.Raycast(
            probeOrigin,
            Vector3.down,
            out RaycastHit nextHit,
            edgeProbeHeight + edgeProbeDepth,
            groundMask,
            QueryTriggerInteraction.Ignore
        );

        if (!foundNextSurface)
            return;

        float verticalDrop =
            ground.point.y - nextHit.point.y;

        float normalChange =
            Vector3.Angle(
                ground.normal,
                nextHit.normal
            );

        bool differentSurface =
            nextHit.collider != ground.hit.collider;

        bool significantTransition =
            verticalDrop >= minimumDrop ||
            normalChange >= minimumNormalChange;

        if (!differentSurface &&
            !significantTransition)
        {
            return;
        }

        BeginMeteorDrop(nextHit);
    }

    void BeginMeteorDrop(RaycastHit nextSurface)
    {
        if (!ballVisual)
            return;

        ballisticStartWorldPoint =
            ballVisual.position;

        Vector3 forward =
            FlatDirection(
                headingDirection,
                physicsBody.velocity
            );

        Vector3 launchVelocity =
            physicsBody.velocity;

        float forwardSpeed =
            Vector3.Dot(
                launchVelocity,
                forward
            );

        float minimumForwardSpeed =
            targetSpeed * .75f;

        if (forwardSpeed < minimumForwardSpeed)
        {
            launchVelocity +=
                forward *
                (minimumForwardSpeed -
                 forwardSpeed);
        }

        ballisticLaunchVelocity =
            launchVelocity;

        ballisticGravity =
            Physics.gravity *
            meteorGravityScale;

        landingPlaneNormal =
            nextSurface.normal.sqrMagnitude >
            1e-6f
                ? nextSurface.normal.normalized
                : Vector3.up;

        landingPlanePoint =
            nextSurface.point +
            landingPlaneNormal *
            sphereRadius;

        bool solved =
            TrySolvePlaneContactTime(
                ballisticStartWorldPoint,
                ballisticLaunchVelocity,
                ballisticGravity,
                landingPlanePoint,
                landingPlaneNormal,
                out float contactTime
            );

        ballisticContactTime =
            solved
                ? Mathf.Clamp(
                    contactTime,
                    .04f,
                    maximumBallisticSeconds
                )
                : maximumBallisticSeconds;

        ballisticAge = 0f;
        ballisticPreviousWorldPoint =
            ballisticStartWorldPoint;

        visualState =
            VisualMotionState.BallisticDrop;

        nextEdgeDropTime =
            Time.time + edgeDropCooldown;

        lastPredictedLandingPoint =
            EvaluateBallisticPoint(
                ballisticContactTime
            );

        hasPredictedLandingPoint = true;
    }

    void StepBallVisual(float deltaTime)
    {
        if (!ballVisual ||
            deltaTime <= 0f)
        {
            return;
        }

        switch (visualState)
        {
            case VisualMotionState.FollowSubject:
                StepVisualFollow(deltaTime);
                break;

            case VisualMotionState.BallisticDrop:
                StepVisualBallistic(deltaTime);
                break;

            case VisualMotionState.LandingHandoff:
                StepVisualHandoff(deltaTime);
                break;
        }

        StepTrickSpin(deltaTime);

        ballVisual.localRotation =
            visualRestLocalRotation *
            distanceRollRotation *
            trickSpinRotation;

        ballVisual.localScale =
            visualRestLocalScale;
    }

    void StepVisualFollow(float deltaTime)
    {
        float blend =
            1f -
            Mathf.Exp(
                -visualFollowSharpness *
                deltaTime
            );

        ballVisual.localPosition =
            Vector3.Lerp(
                ballVisual.localPosition,
                visualRestLocalPosition,
                blend
            );

        UpdateDistanceRoll(
            ballVisual.position,
            ground.grounded
                ? ground.normal
                : Vector3.up,
            deltaTime
        );
    }

    void StepVisualBallistic(float deltaTime)
    {
        ballisticAge += deltaTime;

        float evaluationTime =
            Mathf.Min(
                ballisticAge,
                ballisticContactTime
            );

        Vector3 worldPoint =
            EvaluateBallisticPoint(
                evaluationTime
            );

        ballVisual.position = worldPoint;

        UpdateDistanceRoll(
            worldPoint,
            Vector3.up,
            deltaTime
        );

        ballisticPreviousWorldPoint =
            worldPoint;

        if (ballisticAge >=
            ballisticContactTime)
        {
            BeginLandingHandoff();
        }
    }

    Vector3 EvaluateBallisticPoint(float time)
    {
        return
            ballisticStartWorldPoint +
            ballisticLaunchVelocity * time +
            .5f * ballisticGravity *
            time * time;
    }

    Vector3 EvaluateBallisticVelocity(float time)
    {
        return
            ballisticLaunchVelocity +
            ballisticGravity * time;
    }

    void BeginLandingHandoff()
    {
        handoffStartWorldPoint =
            EvaluateBallisticPoint(
                ballisticContactTime
            );

        Vector3 impactVelocity =
            EvaluateBallisticVelocity(
                ballisticContactTime
            );

        handoffStartVelocity =
            Vector3.ProjectOnPlane(
                impactVelocity,
                landingPlaneNormal
            );

        if (handoffStartVelocity.sqrMagnitude <=
            1e-6f)
        {
            handoffStartVelocity =
                Vector3.ProjectOnPlane(
                    headingDirection,
                    landingPlaneNormal
                ) * targetSpeed;
        }

        handoffAge = 0f;

        visualState =
            VisualMotionState.LandingHandoff;
    }

    void StepVisualHandoff(float deltaTime)
    {
        handoffAge += deltaTime;

        float duration =
            Mathf.Max(
                landingHandoffSeconds,
                .001f
            );

        float t =
            Mathf.Clamp01(
                handoffAge / duration
            );

        Vector3 subjectAnchor =
            GetVisualAnchorWorld();

        Vector3 endVelocity =
            physicsBody
                ? Vector3.ProjectOnPlane(
                    physicsBody.velocity,
                    landingPlaneNormal
                )
                : headingDirection * targetSpeed;

        Vector3 startTangent =
            handoffStartVelocity * duration;

        Vector3 endTangent =
            endVelocity * duration * .35f;

        Vector3 worldPoint =
            CubicHermite(
                handoffStartWorldPoint,
                subjectAnchor,
                startTangent,
                endTangent,
                t
            );

        ballVisual.position = worldPoint;

        UpdateDistanceRoll(
            worldPoint,
            landingPlaneNormal,
            deltaTime
        );

        if (t >= 1f)
        {
            ballVisual.localPosition =
                visualRestLocalPosition;

            visualState =
                VisualMotionState.FollowSubject;

            TriggerLandingTrickSpin();
        }
    }

    Vector3 GetVisualAnchorWorld()
    {
        if (ballVisual.parent)
        {
            return ballVisual.parent.TransformPoint(
                visualRestLocalPosition
            );
        }

        return transform.TransformPoint(
            visualRestLocalPosition
        );
    }

    void UpdateDistanceRoll(
        Vector3 currentWorldPoint,
        Vector3 surfaceNormal,
        float deltaTime
    )
    {
        if (!hasPreviousVisualWorldPoint)
        {
            previousVisualWorldPoint =
                currentWorldPoint;

            hasPreviousVisualWorldPoint = true;
            return;
        }

        Vector3 displacement =
            currentWorldPoint -
            previousVisualWorldPoint;

        previousVisualWorldPoint =
            currentWorldPoint;

        Vector3 tangentDisplacement =
            Vector3.ProjectOnPlane(
                displacement,
                surfaceNormal
            );

        float distance =
            tangentDisplacement.magnitude;

        if (distance <= 1e-6f)
            return;

        Vector3 moveDirection =
            tangentDisplacement / distance;

        Vector3 rollAxisWorld =
            Vector3.Cross(
                surfaceNormal,
                moveDirection
            );

        if (rollAxisWorld.sqrMagnitude <=
            1e-6f)
        {
            return;
        }

        rollAxisWorld.Normalize();

        float degrees =
            distance /
            Mathf.Max(
                visualRollRadius,
                .001f
            ) *
            Mathf.Rad2Deg *
            visualRollMultiplier;

        float maxDegrees =
            maximumRollDegreesPerSecond *
            deltaTime;

        degrees =
            Mathf.Clamp(
                degrees,
                -maxDegrees,
                maxDegrees
            );

        Transform parent = ballVisual.parent;

        Vector3 localAxis =
            parent
                ? parent.InverseTransformDirection(
                    rollAxisWorld
                )
                : rollAxisWorld;

        if (localAxis.sqrMagnitude <=
            1e-6f)
        {
            return;
        }

        localAxis.Normalize();

        distanceRollRotation =
            Quaternion.AngleAxis(
                degrees,
                localAxis
            ) *
            distanceRollRotation;
    }

    void TriggerLandingTrickSpin()
    {
        Transform parent = ballVisual.parent;

        Vector3 upWorld =
            landingPlaneNormal.sqrMagnitude >
            1e-6f
                ? landingPlaneNormal
                : Vector3.up;

        Vector3 forwardWorld =
            Vector3.ProjectOnPlane(
                headingDirection,
                upWorld
            );

        if (forwardWorld.sqrMagnitude <=
            1e-6f)
        {
            forwardWorld = headingDirection;
        }

        upWorld.Normalize();
        forwardWorld.Normalize();

        Vector3 upLocal =
            parent
                ? parent.InverseTransformDirection(
                    upWorld
                )
                : upWorld;

        Vector3 forwardLocal =
            parent
                ? parent.InverseTransformDirection(
                    forwardWorld
                )
                : forwardWorld;

        float directionSign =
            Mathf.Sign(activeTurnAngle);

        if (Mathf.Abs(directionSign) < .5f)
            directionSign = 1f;

        trickSpinVelocityLocal +=
            upLocal.normalized *
            landingSideSpin *
            directionSign;

        trickSpinVelocityLocal +=
            forwardLocal.normalized *
            landingBankSpin *
            -directionSign;
    }

    void StepTrickSpin(float deltaTime)
    {
        float speed =
            trickSpinVelocityLocal.magnitude;

        if (speed > .001f)
        {
            Vector3 axis =
                trickSpinVelocityLocal /
                speed;

            trickSpinRotation =
                Quaternion.AngleAxis(
                    speed * deltaTime,
                    axis
                ) *
                trickSpinRotation;
        }

        trickSpinVelocityLocal *=
            Mathf.Exp(
                -landingSpinDamping *
                deltaTime
            );
    }

    static bool TrySolvePlaneContactTime(
        Vector3 startPoint,
        Vector3 startVelocity,
        Vector3 gravity,
        Vector3 planePoint,
        Vector3 planeNormal,
        out float time
    )
    {
        planeNormal =
            planeNormal.sqrMagnitude >
            1e-6f
                ? planeNormal.normalized
                : Vector3.up;

        float a =
            .5f *
            Vector3.Dot(
                planeNormal,
                gravity
            );

        float b =
            Vector3.Dot(
                planeNormal,
                startVelocity
            );

        float c =
            Vector3.Dot(
                planeNormal,
                startPoint - planePoint
            );

        const float epsilon = 1e-6f;

        if (Mathf.Abs(a) <= epsilon)
        {
            if (Mathf.Abs(b) <= epsilon)
            {
                time = 0f;
                return false;
            }

            float linearTime = -c / b;

            if (linearTime > .001f)
            {
                time = linearTime;
                return true;
            }

            time = 0f;
            return false;
        }

        float discriminant =
            b * b - 4f * a * c;

        if (discriminant < 0f)
        {
            time = 0f;
            return false;
        }

        float sqrt =
            Mathf.Sqrt(discriminant);

        float denominator =
            2f * a;

        float t0 =
            (-b - sqrt) / denominator;

        float t1 =
            (-b + sqrt) / denominator;

        float best =
            float.PositiveInfinity;

        if (t0 > .001f)
            best = t0;

        if (t1 > .001f &&
            t1 < best)
        {
            best = t1;
        }

        if (float.IsInfinity(best))
        {
            time = 0f;
            return false;
        }

        time = best;
        return true;
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

        float h00 =
            2f * t3 -
            3f * t2 +
            1f;

        float h10 =
            t3 -
            2f * t2 +
            t;

        float h01 =
            -2f * t3 +
            3f * t2;

        float h11 =
            t3 -
            t2;

        return
            h00 * start +
            h10 * startTangent +
            h01 * end +
            h11 * endTangent;
    }

    static Vector3 FlatDirection(
        Vector3 direction,
        Vector3 fallback
    )
    {
        direction =
            Vector3.ProjectOnPlane(
                direction,
                Vector3.up
            );

        if (direction.sqrMagnitude <=
            1e-6f)
        {
            direction =
                Vector3.ProjectOnPlane(
                    fallback,
                    Vector3.up
                );
        }

        if (direction.sqrMagnitude <=
            1e-6f)
        {
            direction = Vector3.forward;
        }

        return direction.normalized;
    }

    void OnDrawGizmosSelected()
    {
        if (!drawDebug)
            return;

        Rigidbody body =
            physicsBody
                ? physicsBody
                : GetComponent<Rigidbody>();

        if (!body)
            return;

        Vector3 normal =
            ground.grounded
                ? ground.normal
                : Vector3.up;

        Vector3 forward =
            Vector3.ProjectOnPlane(
                headingDirection.sqrMagnitude >
                1e-6f
                    ? headingDirection
                    : initialHeading,
                normal
            );

        if (forward.sqrMagnitude <= 1e-6f)
            forward = Vector3.forward;

        forward.Normalize();

        Vector3 probeCenter =
            body.position +
            forward * edgeLookAhead;

        Vector3 probeOrigin =
            probeCenter +
            Vector3.up * edgeProbeHeight;

        Gizmos.color = edgeProbeColor;
        Gizmos.DrawLine(
            probeOrigin,
            probeOrigin +
            Vector3.down *
            (edgeProbeHeight + edgeProbeDepth)
        );

        Gizmos.DrawWireSphere(
            probeCenter,
            .06f
        );

        if (hasPredictedLandingPoint)
        {
            Gizmos.color =
                predictedLandingColor;

            Gizmos.DrawSphere(
                lastPredictedLandingPoint,
                .08f
            );
        }
    }
}