using UnityEngine;

[DisallowMultipleComponent]
public sealed class BallVisualTrailTurnReset : MonoBehaviour
{
    [SerializeField] private CorrespondSubject correspondSubject;

    private TrailRenderer trail;
    private TrailLengthController lengthController;
    private bool wasTurning;

    void Awake()
    {
        if (!correspondSubject)
            correspondSubject =
                FindFirstObjectByType<CorrespondSubject>();

        lengthController =
            GetComponent<TrailLengthController>();
    }

    public void SetTrail(TrailRenderer targetTrail)
    {
        trail = targetTrail;
    }

    void LateUpdate()
    {
        if (!trail || !correspondSubject)
            return;

        bool turning =
            correspondSubject.IsVisualFrameTurning;

        if (turning && !wasTurning)
        {
            trail.emitting = false;
        }
        else if (!turning && wasTurning)
        {
            trail.Clear();
            trail.emitting = true;
        }

        wasTurning = turning;
    }
}