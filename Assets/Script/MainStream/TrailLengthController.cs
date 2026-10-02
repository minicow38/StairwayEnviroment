using UnityEngine;

[DisallowMultipleComponent]
public sealed class TrailLengthController : MonoBehaviour, IVisualProxyBindable
{
    [SerializeField] private TrailRenderer trail;
    [SerializeField] private Rigidbody targetBody;

    [SerializeField] private float targetLength = 0.8f;
    [SerializeField] private float minTime = 0.03f;
    [SerializeField] private float maxTime = 0.20f;

    private TrailRenderer proxyTrail;
    private bool holdTime;
    private float heldTime;

    void Awake()
    {
        if (!trail)
            trail = GetComponent<TrailRenderer>();

        if (!targetBody)
            targetBody = GetComponent<Rigidbody>();
    }

    public void BindVisualProxy(Transform proxy)
    {
        proxyTrail = proxy ? proxy.GetComponent<TrailRenderer>() : null;

        if (trail && proxyTrail)
            proxyTrail.time = trail.time;
    }

    public void UnbindVisualProxy()
    {
        proxyTrail = null;
    }

    public void HoldTrailTime(float minimumTime)
    {
        holdTime = true;
        heldTime = Mathf.Max(CurrentTime(), minimumTime);
        ApplyTime(heldTime);
    }

    public void ReleaseTrailTime()
    {
        holdTime = false;
        UpdateTrailTime();
    }

    void Update()
    {
        if (holdTime)
        {
            ApplyTime(heldTime);
            return;
        }

        UpdateTrailTime();
    }

    void UpdateTrailTime()
    {
        if (!targetBody)
            return;

        float speed = targetBody.velocity.magnitude;
        float time = speed < 0.01f
            ? maxTime
            : Mathf.Clamp(targetLength / speed, minTime, maxTime);

        ApplyTime(time);
    }

    float CurrentTime()
    {
        if (trail)
            return trail.time;

        if (proxyTrail)
            return proxyTrail.time;

        return maxTime;
    }

    void ApplyTime(float time)
    {
        if (trail)
            trail.time = time;

        if (proxyTrail)
            proxyTrail.time = time;
    }
}