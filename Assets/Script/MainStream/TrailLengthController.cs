using UnityEngine;

[DisallowMultipleComponent]
public sealed class TrailLengthController : MonoBehaviour
{
    [SerializeField] private Rigidbody targetBody;

    [SerializeField] private float targetLength = 0.8f;
    [SerializeField] private float minTime = 0.03f;
    [SerializeField] private float maxTime = 0.20f;

    private TrailRenderer trail;
    private bool holdTime;
    private float heldTime;

    void Awake()
    {
        if (!targetBody)
            targetBody = GetComponent<Rigidbody>();
    }

    public void SetTrail(TrailRenderer targetTrail)
    {
        trail = targetTrail;
    }

    public void HoldTrailTime(float minimumTime)
    {
        if (!trail)
            return;

        holdTime = true;
        heldTime = Mathf.Max(trail.time, minimumTime);
        trail.time = heldTime;
    }

    public void ReleaseTrailTime()
    {
        holdTime = false;
        UpdateTrailTime();
    }

    void Update()
    {
        if (!trail)
            return;

        if (holdTime)
        {
            trail.time = heldTime;
            return;
        }

        UpdateTrailTime();
    }

    void UpdateTrailTime()
    {
        if (!trail || !targetBody)
            return;

        float speed = targetBody.velocity.magnitude;

        trail.time = speed < 0.01f
            ? maxTime
            : Mathf.Clamp(
                targetLength / speed,
                minTime,
                maxTime);
    }
}