using UnityEngine;

public sealed class BallVisualTrailTurnReset : MonoBehaviour
{
    [SerializeField]
    TrailRenderer trailRenderer;

    [SerializeField]
    CorrespondSubject correspondSubject;

    bool wasTurning;

    void Awake()
    {
        if (!trailRenderer)
            trailRenderer = GetComponent<TrailRenderer>();

        if (!correspondSubject)
            correspondSubject =
                FindFirstObjectByType<CorrespondSubject>();
    }

    void LateUpdate()
    {
        if (!trailRenderer || !correspondSubject)
            return;

        bool turning =
            correspondSubject.IsVisualFrameTurning;

        // 旋回開始
        if (turning && !wasTurning)
        {
            trailRenderer.emitting = false;
        }

        // 旋回完了
        if (!turning && wasTurning)
        {
            trailRenderer.Clear();
            trailRenderer.emitting = true;
        }

        wasTurning = turning;
    }
}