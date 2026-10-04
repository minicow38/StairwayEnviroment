using UnityEngine;

[DisallowMultipleComponent]
public sealed class BallVisualTrailTurnReset : MonoBehaviour
{
    [SerializeField] private TrailRenderer trailRenderer;
    [SerializeField] private CorrespondSubject correspondSubject;
    [SerializeField, Min(0.2f)] private float turnHoldTime = 2f;

    private TrailRenderer proxyTrailRenderer;
    private TrailLengthController trailLengthController;
    private bool wasTurning;
    private float sourceTimeBeforeTurn;
    private float proxyTimeBeforeTurn;

    void Awake()
    {
        if (!trailRenderer)
            trailRenderer = GetComponent<TrailRenderer>();

        if (!correspondSubject)
            correspondSubject = FindFirstObjectByType<CorrespondSubject>();

        trailLengthController = GetComponent<TrailLengthController>();
    }

    public void BindVisualProxy(Transform proxy)
    {
        proxyTrailRenderer = proxy ? proxy.GetComponent<TrailRenderer>() : null;
    }

    public void UnbindVisualProxy()
    {
        proxyTrailRenderer = null;
    }

    void LateUpdate()
    {
        if (!correspondSubject)
            return;

        bool turning = correspondSubject.IsVisualFrameTurning;

        if (turning && !wasTurning)
            BeginTurn();
        else if (!turning && wasTurning)
            EndTurn();

        wasTurning = turning;
    }

    void BeginTurn()
    {
        if (trailLengthController)
        {
            trailLengthController.HoldTrailTime(turnHoldTime);
        }
        else
        {
            sourceTimeBeforeTurn = trailRenderer ? trailRenderer.time : 0f;
            proxyTimeBeforeTurn = proxyTrailRenderer ? proxyTrailRenderer.time : 0f;
            SetMinimumTime(turnHoldTime);
        }

        SetEmitting(false);
    }

    void EndTurn()
    {
        ClearTrails();
        SetEmitting(true);

        if (trailLengthController)
        {
            trailLengthController.ReleaseTrailTime();
        }
        else
        {
            if (trailRenderer)
                trailRenderer.time = sourceTimeBeforeTurn;

            if (proxyTrailRenderer)
                proxyTrailRenderer.time = proxyTimeBeforeTurn;
        }
    }

    void SetMinimumTime(float minimumTime)
    {
        if (trailRenderer)
            trailRenderer.time = Mathf.Max(trailRenderer.time, minimumTime);

        if (proxyTrailRenderer)
            proxyTrailRenderer.time = Mathf.Max(proxyTrailRenderer.time, minimumTime);
    }

    void SetEmitting(bool value)
    {
        if (trailRenderer)
            trailRenderer.emitting = value;

        if (proxyTrailRenderer)
            proxyTrailRenderer.emitting = value;
    }

    void ClearTrails()
    {
        if (trailRenderer)
            trailRenderer.Clear();

        if (proxyTrailRenderer)
            proxyTrailRenderer.Clear();
    }
}
