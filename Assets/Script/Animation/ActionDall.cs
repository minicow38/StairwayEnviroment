using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text.RegularExpressions;

public class ActionDall : MonoBehaviour
{
    public Rigidbody rb;
    public string ContactNum = "";
    public Collider[] AroundStairway;
    public Collider[] ExpendScopePlayer;
    public float[] resouceY;
    public Transform DollAttachGrond;
    public List<GameObject> AroundStairwayPhysics;
    public Vector3 angle;
    [Header("KnockBack")]
    public float knockBackPower = 60f;
    public float knockUpPower = 25f;
    public float knockTorque = 3f;

    private float totalRotation;
    public Quaternion previousRotation;
    public bool StimulateSeekingAnimation = false;
    public bool NonComeBackPhysicX = false;
    private bool knockedBack = false;
    public Animator anime;

    // ================================================================
    // FunCharacter diagnostics (read-only; no forces/pose changes)
    // ================================================================
    [Header("FunCharacter Diagnostics")]
    [SerializeField] bool debugFunCharacter = true;
    [SerializeField] float debugDriftThreshold = 0.15f;
    [SerializeField] float debugRiseSpeed = 3f;
    [SerializeField] float debugInterval = 0.25f;
    Transform diagnosticVisualArc;
    Transform diagnosticPhysicsArc;
    Transform diagnosticVisualRoot;
    Transform diagnosticPhysicsRoot;
    Vector3 diagnosticAnchor;
    int diagnosticId;
    bool diagnosticReady;
    bool diagnosticHasPrevious;
    bool diagnosticWasDrifting;
    bool diagnosticWasRising;
    float diagnosticNextMotionLog;
    float diagnosticNextStayLog;
    float diagnosticLastRootMotion = -1f;
    float diagnosticLastVisualTurn;
    float diagnosticLastPhysicsTurn;
    Vector3 diagnosticPreviousLocal;
    Vector3 diagnosticPreviousWorld;
    Quaternion diagnosticPreviousVisualRoot;
    Quaternion diagnosticPreviousPhysicsRoot;
    Vector3 diagnosticPreviousVisualRootPosition;
    Vector3 diagnosticPreviousPhysicsRootPosition;
    Vector3 diagnosticPreviousVelocity;
    string diagnosticPreviousPhase;
    Collider[] diagnosticVisualColliders;
    Collider[] diagnosticPhysicsColliders;
    Collider[] diagnosticDollColliders;
    CapsuleCollider diagnosticCapsule;

    public void BindFunDiagnostic(GameObject character, GameObject physicsArc,
        GameObject visualArc, bool enable)
    {
        debugFunCharacter = enable;
        if (!enable) return;
        diagnosticId = character.GetInstanceID();
        diagnosticVisualArc = visualArc.transform;
        diagnosticPhysicsArc = physicsArc.transform;
        diagnosticVisualRoot = diagnosticVisualArc.root;
        diagnosticPhysicsRoot = diagnosticPhysicsArc.root;
        diagnosticAnchor = diagnosticVisualArc.InverseTransformPoint(transform.position);
        diagnosticReady = true;
        // The arc hierarchy includes the newly spawned doll; exclude it below.
        diagnosticVisualColliders = visualArc.GetComponentsInChildren<Collider>(true);
        diagnosticPhysicsColliders = physicsArc.GetComponentsInChildren<Collider>(true);
        diagnosticDollColliders = GetComponentsInChildren<Collider>(true);
        diagnosticCapsule = GetComponent<CapsuleCollider>();
    }

    bool DebugBeforeKnock => debugFunCharacter && !knockedBack;

    static string DebugObject(Transform t)
    {
        if (!t) return "null";
        return $"{t.name}#{t.gameObject.GetInstanceID()} root={t.root.name} layer={LayerMask.LayerToName(t.gameObject.layer)} tag={t.tag}";
    }

    static string DebugCollider(Collider c)
    {
        if (!c) return "null";
        Rigidbody body = c.attachedRigidbody;
        return $"{DebugObject(c.transform)} collider={c.GetType().Name}#{c.GetInstanceID()} " +
               $"enabled={c.enabled} active={c.gameObject.activeInHierarchy} trigger={c.isTrigger} " +
               $"body={(body ? body.name : "none")}";
    }

    string DebugPairRole(Collider c)
    {
        if (!c) return "none";
        if (c.transform == transform || c.transform.IsChildOf(transform)) return "self";
        if (diagnosticVisualArc && (c.transform == diagnosticVisualArc || c.transform.IsChildOf(diagnosticVisualArc))) return "visualArcHierarchy";
        if (diagnosticPhysicsArc && (c.transform == diagnosticPhysicsArc || c.transform.IsChildOf(diagnosticPhysicsArc))) return "physicsArcHierarchy";
        if (diagnosticVisualRoot && c.transform.root == diagnosticVisualRoot) return "otherVisualRoot";
        if (diagnosticPhysicsRoot && c.transform.root == diagnosticPhysicsRoot) return "otherPhysicsRoot";
        return "other";
    }

    string DebugState()
    {
        Vector3 local = diagnosticVisualArc ? diagnosticVisualArc.InverseTransformPoint(transform.position) : transform.localPosition;
        Vector3 expected = diagnosticVisualArc ? diagnosticVisualArc.TransformPoint(diagnosticAnchor) : transform.position;
        Vector3 physicsExpected = diagnosticPhysicsArc ? diagnosticPhysicsArc.TransformPoint(diagnosticAnchor) : expected;
        return $"world={transform.position:F3} arcLocal={local:F3} anchor={diagnosticAnchor:F3} " +
               $"localDrift={Vector3.Distance(local, diagnosticAnchor):F3} worldDrift={Vector3.Distance(transform.position, expected):F3} " +
               $"pairGap={Vector3.Distance(expected, physicsExpected):F3} rot={transform.eulerAngles:F3} " +
               $"parent={DebugObject(transform.parent)} ground={DebugObject(DollAttachGrond)} " +
               $"groundLocked={NonComeBackPhysicX} seeking={StimulateSeekingAnimation} knocked={knockedBack} " +
               (rb ? $"rbPos={rb.position:F3} v={rb.velocity:F3} w={rb.angularVelocity:F3} " +
               $"gravity={rb.useGravity} kinematic={rb.isKinematic} constraints={rb.constraints} " : "rb=null ") +
               $"capsuleTrigger={(diagnosticCapsule ? diagnosticCapsule.isTrigger.ToString() : "n/a")} " +
               $"rootMotion={(anime ? anime.applyRootMotion.ToString() : "n/a")}";
    }

    void DebugEvent(string kind, string extra = "")
    {
        if (!debugFunCharacter) return;
        Debug.Log($"[FUN {kind}] id={diagnosticId} doll={name} component={GetInstanceID()} " +
                  $"frame={Time.frameCount} fixed={Time.fixedTime:F3} time={Time.time:F3} " + extra + " " + DebugState(), this);
    }

    void DebugColliders(string label, Collider[] colliders, bool excludeDoll)
    {
        if (colliders == null) return;
        foreach (Collider c in colliders)
        {
            if (!c || (excludeDoll && c.GetComponentInParent<ActionDall>())) continue;
            DebugEvent("COLLIDER STATE", $"side={label} {DebugCollider(c)}");
        }
    }

    void DebugCollision(string kind, Collision col)
    {
        Collider other = col.collider;
        string contact = "none";
        if (col.contactCount > 0)
        {
            ContactPoint cp = col.GetContact(0);
            contact = $"point={cp.point:F3} normal={cp.normal:F3} separation={cp.separation:F4} selfCollider={DebugCollider(cp.thisCollider)}";
        }
        DebugEvent(kind, $"other={DebugCollider(other)} role={DebugPairRole(other)} " +
                   $"relativeVelocity={col.relativeVelocity:F3} impulse={col.impulse:F3} contacts={col.contactCount} {contact}");
    }

    // A downward query is only a snapshot, not a physical contact test.
    void DebugGroundProbe()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;
        RaycastHit[] hits = Physics.RaycastAll(origin, Vector3.down, 3f,
            Physics.AllLayers, QueryTriggerInteraction.Ignore);
        Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));
        int count = 0;
        foreach (RaycastHit hit in hits)
        {
            if (!hit.collider || hit.collider.transform == transform || hit.collider.transform.IsChildOf(transform)) continue;
            DebugEvent("GROUND PROBE", $"hit={DebugCollider(hit.collider)} role={DebugPairRole(hit.collider)} distance={hit.distance:F3} point={hit.point:F3} normal={hit.normal:F3}");
            if (++count >= 4) break;
        }
        if (count == 0) DebugEvent("GROUND PROBE", "noNonTriggerHitWithin3m=true");
    }

    void DebugObserveMotion(string phase)
    {
        if (!DebugBeforeKnock || !diagnosticReady || !diagnosticVisualArc || !rb) return;
        Vector3 local = diagnosticVisualArc.InverseTransformPoint(transform.position);
        Quaternion vr = diagnosticVisualRoot ? diagnosticVisualRoot.rotation : Quaternion.identity;
        Quaternion pr = diagnosticPhysicsRoot ? diagnosticPhysicsRoot.rotation : Quaternion.identity;
        Vector3 vp = diagnosticVisualRoot ? diagnosticVisualRoot.position : Vector3.zero;
        Vector3 pp = diagnosticPhysicsRoot ? diagnosticPhysicsRoot.position : Vector3.zero;
        float drift = Vector3.Distance(local, diagnosticAnchor);
        float localStep = diagnosticHasPrevious ? Vector3.Distance(local, diagnosticPreviousLocal) : 0f;
        float visualTurn = diagnosticHasPrevious ? Quaternion.Angle(vr, diagnosticPreviousVisualRoot) : 0f;
        float physicsTurn = diagnosticHasPrevious ? Quaternion.Angle(pr, diagnosticPreviousPhysicsRoot) : 0f;
        float visualMove = diagnosticHasPrevious ? Vector3.Distance(vp, diagnosticPreviousVisualRootPosition) : 0f;
        float physicsMove = diagnosticHasPrevious ? Vector3.Distance(pp, diagnosticPreviousPhysicsRootPosition) : 0f;
        if (visualTurn > 0.1f || physicsTurn > 0.1f || visualMove > 0.03f || physicsMove > 0.03f)
        {
            diagnosticLastRootMotion = Time.unscaledTime;
            diagnosticLastVisualTurn = visualTurn;
            diagnosticLastPhysicsTurn = physicsTurn;
        }
        bool drifting = drift >= Mathf.Max(0.01f, debugDriftThreshold);
        bool rising = rb.velocity.y > debugRiseSpeed;
        bool newAnomaly = (drifting && !diagnosticWasDrifting) || (rising && !diagnosticWasRising);
        bool motionWithRoot = localStep > 0.03f && (visualTurn > 0.1f || physicsTurn > 0.1f || visualMove > 0.03f || physicsMove > 0.03f);
        if (newAnomaly || ((drifting || rising || motionWithRoot) && Time.unscaledTime >= diagnosticNextMotionLog))
        {
            diagnosticNextMotionLog = Time.unscaledTime + Mathf.Max(0.05f, debugInterval);
            DebugEvent(newAnomaly ? "MOTION BEGIN" : "MOTION", $"phase={phase} prevPhase={diagnosticPreviousPhase} " +
                       $"drifting={drifting} rising={rising} localStep={localStep:F3} " +
                       $"previousLocal={diagnosticPreviousLocal:F3} previousWorld={diagnosticPreviousWorld:F3} " +
                       $"previousV={diagnosticPreviousVelocity:F3} deltaV={(rb.velocity-diagnosticPreviousVelocity):F3} " +
                       $"visualTurnStep={visualTurn:F3} physicsTurnStep={physicsTurn:F3} " +
                       $"visualRootMove={visualMove:F3} physicsRootMove={physicsMove:F3} " +
                       $"visualRootRot={vr.eulerAngles:F3} physicsRootRot={pr.eulerAngles:F3} " +
                       $"lastRootMotionAgo={(diagnosticLastRootMotion < 0f ? -1f : Time.unscaledTime-diagnosticLastRootMotion):F3} " +
                       $"lastVisualTurn={diagnosticLastVisualTurn:F3} lastPhysicsTurn={diagnosticLastPhysicsTurn:F3}");
            if (newAnomaly)
            {
                DebugColliders("expectedVisual", diagnosticVisualColliders, true);
                DebugColliders("expectedPhysics", diagnosticPhysicsColliders, false);
                DebugColliders("doll", diagnosticDollColliders, false);
                DebugGroundProbe();
            }
        }
        diagnosticWasDrifting = drifting;
        diagnosticWasRising = rising;
        diagnosticPreviousLocal = local;
        diagnosticPreviousWorld = transform.position;
        diagnosticPreviousVisualRoot = vr;
        diagnosticPreviousPhysicsRoot = pr;
        diagnosticPreviousVisualRootPosition = vp;
        diagnosticPreviousPhysicsRootPosition = pp;
        diagnosticPreviousVelocity = rb.velocity;
        diagnosticPreviousPhase = phase;
        diagnosticHasPrevious = true;
    }

    void Start()
    {
        previousRotation = transform.rotation;
        resouceY = new float[3];
        anime = transform.GetComponent<Animator>();
        rb = GetComponent<Rigidbody>();

        if (debugFunCharacter)
        {
            // Also usable when the prefab is tested outside the generator.
            if (!diagnosticReady)
            {
                diagnosticId = gameObject.GetInstanceID();
                diagnosticVisualArc = transform.parent;
                diagnosticVisualRoot = transform.root;
                diagnosticAnchor = diagnosticVisualArc ? diagnosticVisualArc.InverseTransformPoint(transform.position) : transform.localPosition;
                diagnosticCapsule = GetComponent<CapsuleCollider>();
                diagnosticDollColliders = GetComponentsInChildren<Collider>(true);
                diagnosticReady = true;
            }
            DebugEvent("START", $"expectedVisual={DebugObject(diagnosticVisualArc)} expectedPhysics={DebugObject(diagnosticPhysicsArc)}");
            DebugColliders("expectedVisual", diagnosticVisualColliders, true);
            DebugColliders("expectedPhysics", diagnosticPhysicsColliders, false);
            DebugColliders("doll", diagnosticDollColliders, false);
            DebugObserveMotion("Start");
        }
    }

    void FixedUpdate()
    {
        DebugObserveMotion("FixedUpdate.begin");
        float delta = Mathf.DeltaAngle(previousRotation.eulerAngles.y, transform.eulerAngles.y);
        totalRotation += Mathf.Abs(delta);
        bool isRotating = totalRotation > 210f;
        if (rb.velocity.y > 3f)
        {
            Debug.Log("累積回転が210度を超えました");
        }

        if (AroundStairway.Length == 0)
        {
            AroundStairway = Physics.OverlapSphere(transform.position, 6f);
            for (int i = 0; i < AroundStairway.Length; i++)
            {
                if (Regex.Match(AroundStairway[i].name, @"([a-zA-Z]+)(\d*)_(\d*)_(Render)").Success)
                {
                    Match match = Regex.Match(AroundStairway[i].name, @"([a-zA-Z]+)(\d*)_(\d*)_(Render)");
                    AroundStairwayPhysics.Add(AroundStairway[i].transform.gameObject);
                    var turn_name = "" + match.Groups[1] + match.Groups[2] + "_" + match.Groups[3] + "_" + "Physics";
                    var found = GameObject.Find("" + turn_name);
                    if (DebugBeforeKnock)
                        DebugEvent("GROUND SEARCH", $"added={DebugCollider(AroundStairway[i])} lookup={turn_name} found={DebugObject(found ? found.transform : null)} count={AroundStairwayPhysics.Count}");
                    Debug.Log("");
                }
            }
            for (int incentBall = 0; incentBall < MainGameManager.VisualPlayerChildCollider.Length; incentBall++)
            {
                if (DebugBeforeKnock)
                    DebugEvent("IGNORE PAIR", $"self={DebugCollider(transform.GetComponent<CapsuleCollider>())} other={DebugCollider(MainGameManager.VisualPlayerChildCollider[incentBall].GetComponent<SphereCollider>())}");
                Physics.IgnoreCollision(transform.GetComponent<CapsuleCollider>(),
                    MainGameManager.VisualPlayerChildCollider[incentBall].GetComponent<SphereCollider>());
            }
            if (DebugBeforeKnock)
                DebugEvent("SEARCH DONE", $"near={AroundStairway.Length} matched={AroundStairwayPhysics.Count}");
        }
        else
        {
            ExpendScopePlayer = Physics.OverlapSphere(transform.position, 12f);
            foreach (Collider AnotherNascent in ExpendScopePlayer)
            {
                if (AnotherNascent.transform.CompareTag("SubjectVisual"))
                {
                    if (!StimulateSeekingAnimation)
                    {
                        if (DebugBeforeKnock)
                            DebugEvent("MESH ENABLE BEFORE", $"near={DebugCollider(AnotherNascent)} groundMesh={DebugCollider(DollAttachGrond ? DollAttachGrond.GetComponent<MeshCollider>() : null)}");
                        MeshCollider groundMesh = DollAttachGrond
                            ? DollAttachGrond.GetComponent<MeshCollider>()
                            : null;
                        if (!groundMesh)
                            continue; // Wait for a valid ground reference; do not mark seeking complete.

                        groundMesh.enabled = true;
                        StimulateSeekingAnimation = true;
                        if (DebugBeforeKnock)
                            DebugEvent("MESH ENABLE AFTER", $"groundMesh={DebugCollider(groundMesh)}");
                    }
                }
            }
        }
    }

    void LateUpdate()
    {
        DebugObserveMotion("LateUpdate");
    }

    void OnTriggerEnter(Collider other)
    {
        if (DebugBeforeKnock)
            DebugEvent("TRIGGER ENTER", $"other={DebugCollider(other)} role={DebugPairRole(other)} matchesSubject={other.transform.name == "subject"}");
        if (other.transform.name == "subject")
        {
            if (knockedBack) return;
            knockedBack = true;
            anime.SetBool("KnockSwitch", true);
            Rigidbody ballRb = other.attachedRigidbody;
            Vector3 knockDirection;
            if (ballRb != null && ballRb.velocity.sqrMagnitude > 0.01f)
            {
                knockDirection = ballRb.velocity.normalized;
            }
            else
            {
                knockDirection = (transform.position - other.transform.position).normalized;
            }
            Vector3 impulse = knockDirection * knockBackPower + Vector3.up * knockUpPower;
            DebugEvent("KNOCK BEGIN", $"other={DebugCollider(other)} requestedImpulse={impulse:F3}");
            rb.AddForce(impulse, ForceMode.Impulse);
            rb.AddTorque(transform.right * knockTorque, ForceMode.Impulse);
            MainGameManager.Coin += 10;
            StartCoroutine(DelayDestory());
        }
    }

    void OnCollisionEnter(Collision col)
    {
        if (DebugBeforeKnock) DebugCollision("COLLISION ENTER", col);
        if (col.transform.CompareTag("plane") && !NonComeBackPhysicX && AroundStairwayPhysics.Count > 0)
        {
            Match match;
            int currrentArcHit = 0;
            try
            {
                match = Regex.Match(AroundStairwayPhysics[0].transform.name, @"^([a-zA-Z]+)(\d*)_(\d*)_(Physics)");
            }
            catch (Exception e)
            {
                Debug.Log("");
            }
            for (int arcHit = 0; arcHit < AroundStairwayPhysics.Count; arcHit++)
            {
                if (Regex.Match(AroundStairwayPhysics[arcHit].name, @"(ArcSlab)+(\d*)").Success)
                {
                    currrentArcHit = arcHit;
                    break;
                }
            }
            var localAngle = col.transform.localEulerAngles;
            if (DebugBeforeKnock)
                DebugEvent("GROUND LOCK BEFORE", $"chosen={DebugCollider(col.collider)} role={DebugPairRole(col.collider)} targetWorldEuler={new Vector3(localAngle.x, localAngle.y+180, localAngle.z):F3}");
            transform.rotation = Quaternion.Euler(localAngle.x, localAngle.y + 180, localAngle.z);
            transform.GetComponent<CapsuleCollider>().isTrigger = true;
            DollAttachGrond = col.transform;
            NonComeBackPhysicX = true;
            int fit = 0;
            rb.constraints = RigidbodyConstraints.FreezePositionY;
            if (DebugBeforeKnock)
                DebugEvent("GROUND LOCK AFTER", $"chosen={DebugCollider(col.collider)} role={DebugPairRole(col.collider)}");
        }
    }

    void OnCollisionStay(Collision col)
    {
        if (!DebugBeforeKnock || Time.unscaledTime < diagnosticNextStayLog) return;
        if (col.impulse.magnitude < 1f) return;
        diagnosticNextStayLog = Time.unscaledTime + Mathf.Max(0.05f, debugInterval);
        DebugCollision("COLLISION STAY IMPULSE", col);
    }

    void OnCollisionExit(Collision col)
    {
        if (DebugBeforeKnock) DebugCollision("COLLISION EXIT", col);
    }

    void OnTransformParentChanged()
    {
        if (DebugBeforeKnock && diagnosticReady) DebugEvent("PARENT CHANGED");
    }

    IEnumerator DelayDestory()
    {
        yield return new WaitForSeconds(1.2f);
        Destroy(transform.gameObject);
    }
}
