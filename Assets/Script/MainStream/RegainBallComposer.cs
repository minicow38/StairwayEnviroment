using UnityEngine;


[DefaultExecutionOrder(20000)]
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public sealed class RegainBallComposer : MonoBehaviour
{
    const float Epsilon =
        0.000001f;


    // ================================================================
    // References
    // ================================================================

    [Header("References")]

    [Tooltip(
        "InSubject -> VisualPlayerRoot の正式な座標写像。")]
    [SerializeField]
    private CorrespondSubject correspondSubject;


    [Tooltip(
        "大域軌道を所有するBallVisual。")]
    [SerializeField]
    private Rigidbody ballVisual;


    [Tooltip(
        "局所Stable-N振動を所有するEqualizer。")]
    [SerializeField]
    private BallVisualEqualizerSync equalizer;


    [Tooltip(
        "最終合成結果となるRegainBall。")]
    [SerializeField]
    private Rigidbody regainBall;


    // ================================================================
    // Runtime - Read Only
    // ================================================================

    [Header("Runtime - Read Only")]

    [SerializeField]
    private Vector3 basePosition;

    [SerializeField]
    private Vector3 globalTrajectoryOffset;

    [SerializeField]
    private Vector3 localOscillationOffset;

    [SerializeField]
    private Vector3 composedPosition;


    [SerializeField]
    private Vector3 baseVelocity;

    [SerializeField]
    private Vector3 globalTrajectoryVelocity;

    [SerializeField]
    private Vector3 localOscillationVelocity;

    [SerializeField]
    private Vector3 composedVelocity;


    [SerializeField]
    private Vector3 equalizerCarrierPosition;

    [SerializeField]
    private Vector3 oscillationNormal =
        Vector3.up;

    [SerializeField]
    private bool localOscillationActive;


    // ================================================================
    // Public Read Only
    // ================================================================

    public Vector3 CompositePosition =>
        composedPosition;

    public Vector3 CompositeVelocity =>
        composedVelocity;

    public Vector3 BasePosition =>
        basePosition;

    public Vector3 GlobalTrajectoryOffset =>
        globalTrajectoryOffset;

    public Vector3 LocalOscillationOffset =>
        localOscillationOffset;

    public bool LocalOscillationActive =>
        localOscillationActive;


    // ================================================================
    // Unity
    // ================================================================

    private void Awake()
    {
        if (!regainBall)
        {
            regainBall =
                GetComponent<Rigidbody>();
        }

        ResolveReferences();
        ConfigureRegainBall();
    }


    private void Start()
    {
        Physics.IgnoreCollision(
            transform.GetComponent<SphereCollider>(), 
            GameObject.Find("InSubject").GetComponent<SphereCollider>(),
            true);
        Physics.IgnoreCollision(
            transform.GetComponent<SphereCollider>(), 
            GameObject.Find("subject").GetComponent<SphereCollider>(),
            true);
        Physics.IgnoreCollision(
            transform.GetComponent<SphereCollider>(), 
            GameObject.Find("BallVisual").GetComponent<SphereCollider>(),
            true);
        Physics.IgnoreCollision(
            transform.GetComponent<SphereCollider>(), 
            GameObject.Find("BallVisualEqualizer").GetComponent<SphereCollider>(),
            true);
        ResolveReferences();
        ComposeImmediately();
    }


    private void FixedUpdate()
    {
        if (!ReferencesReady())
        {
            ResolveReferences();

            if (!ReferencesReady())
                return;
        }

        ComposeMotion();
        ApplyMotion();
    }


    // ================================================================
    // Composition
    // ================================================================

    private void ComposeMotion()
    {
        // ------------------------------------------------------------
        // 1. Base motion
        //
        // InSubject
        //     ↓ CorrespondSubject mapping
        // Subject
        //
        // x_base = x_S
        // ------------------------------------------------------------

        basePosition =
            correspondSubject.MappedPosition;

        baseVelocity =
            correspondSubject.MappedPhysicalVelocity;


        // ------------------------------------------------------------
        // 2. Global BallVisual mode
        //
        // delta_B = x_B - x_S
        //
        // BallVisualがSubjectと同期している場合は0。
        // Incident / Missile / Terminal中だけ大域偏差になる。
        // ------------------------------------------------------------

        globalTrajectoryOffset =
            ballVisual.position -
            basePosition;

        globalTrajectoryVelocity =
            ballVisual.velocity -
            baseVelocity;


        // ------------------------------------------------------------
        // 3. Local Equalizer mode
        //
        // Stable-N成分だけを取り出す。
        //
        // T / L はすでにSubject transportなので、
        // Regainへもう一度足さない。
        // ------------------------------------------------------------

        localOscillationOffset =
            Vector3.zero;

        localOscillationVelocity =
            Vector3.zero;

        localOscillationActive =
            false;

        equalizerCarrierPosition =
            ballVisual.position;


        if (equalizer &&
            !equalizer.IsSynchronized)
        {
            Rigidbody equalizerBody =
                equalizer.Body;

            if (equalizerBody &&
                equalizer.TryGetRegainOscillationFrame(
                    out Vector3 equalizerReleasePosition,
                    out Vector3 subjectReleasePosition,
                    out Vector3 stableNormal))
            {
                if (stableNormal.sqrMagnitude >
                    Epsilon)
                {
                    stableNormal.Normalize();

                    oscillationNormal =
                        stableNormal;


                    // ------------------------------------------------
                    // Moving carrier
                    //
                    // C(t)
                    // =
                    // EqualizerReleasePosition
                    // +
                    // [ Subject(t) - Subject(t0) ]
                    //
                    // Release時:
                    //
                    // C(t0) = EqualizerReleasePosition
                    //
                    // EqualizerはRelease直前にBallVisualへ完全同期して
                    // いるため、初期局所位置偏差は0になる。
                    // ------------------------------------------------

                    Vector3 subjectTravel =
                        basePosition -
                        subjectReleasePosition;

                    equalizerCarrierPosition =
                        equalizerReleasePosition +
                        subjectTravel;


                    // ------------------------------------------------
                    // Equalizerの全位置差からStable-N成分だけを抽出。
                    //
                    // r = x_E - C
                    //
                    // delta_N =
                    // N * dot(r, N)
                    // ------------------------------------------------

                    Vector3 relativePosition =
                        equalizerBody.position -
                        equalizerCarrierPosition;

                    float normalDistance =
                        Vector3.Dot(
                            relativePosition,
                            stableNormal);

                    localOscillationOffset =
                        stableNormal *
                        normalDistance;


                    // ------------------------------------------------
                    // 速度も同じ空間分解を行う。
                    //
                    // Equalizer carrier velocity = Subject velocity
                    //
                    // u_N =
                    // N * dot(v_E - v_S, N)
                    // ------------------------------------------------

                    Vector3 relativeVelocity =
                        equalizerBody.velocity -
                        baseVelocity;

                    float normalSpeed =
                        Vector3.Dot(
                            relativeVelocity,
                            stableNormal);

                    localOscillationVelocity =
                        stableNormal *
                        normalSpeed;


                    localOscillationActive =
                        true;
                }
            }
        }


        // ------------------------------------------------------------
        // Final Regain composition
        //
        // x_R
        // =
        // x_S
        // +
        // (x_B - x_S)
        // +
        // delta_N
        //
        // =
        // x_B + delta_N
        // ------------------------------------------------------------

        composedPosition =
            basePosition +
            globalTrajectoryOffset +
            localOscillationOffset;


        // ------------------------------------------------------------
        // Same decomposition for velocity
        // ------------------------------------------------------------

        composedVelocity =
            baseVelocity +
            globalTrajectoryVelocity +
            localOscillationVelocity;
    }


    // ================================================================
    // Output
    // ================================================================

    private void ApplyMotion()
    {
        if (!regainBall)
            return;


        // RegainBall自身は物理系をもう一つ増やさない。
        // 3つの既存モードの最終出力だけを担当する。

        regainBall.position =
            composedPosition;


        // 回転については現在BallVisualを大域所有者とする。
        // Equalizerの局所振動は位置Nモードだけを合成する。

        regainBall.rotation =
            ballVisual.rotation;
    }


    private void ComposeImmediately()
    {
        if (!ReferencesReady())
            return;

        ComposeMotion();

        regainBall.position =
            composedPosition;

        regainBall.rotation =
            ballVisual.rotation;
    }


    // ================================================================
    // Setup
    // ================================================================

    private void ConfigureRegainBall()
    {
        if (!regainBall)
            return;


        // RegainBallは出力体。
        //
        // AddForce / Gravity / Collisionによって
        // 4つ目の独立運動を発生させない。

        regainBall.useGravity =
            false;

        regainBall.detectCollisions =
            false;

        regainBall.isKinematic =
            true;

        regainBall.interpolation =
            RigidbodyInterpolation.Interpolate;
    }


    private void ResolveReferences()
    {
        if (!regainBall)
        {
            regainBall =
                GetComponent<Rigidbody>();
        }


        if (!correspondSubject)
        {
            correspondSubject =
                FindFirstObjectByType<
                    CorrespondSubject>();
        }


        if (!ballVisual)
        {
            GameObject ballVisualObject =
                GameObject.Find(
                    "BallVisual");

            if (ballVisualObject)
            {
                ballVisual =
                    ballVisualObject.GetComponent<
                        Rigidbody>();
            }
        }


        if (!equalizer)
        {
            equalizer =
                FindFirstObjectByType<
                    BallVisualEqualizerSync>();
        }
    }


    private bool ReferencesReady()
    {
        return
            regainBall &&
            correspondSubject &&
            ballVisual;
    }
}