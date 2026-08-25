using UnityEngine;

public class TrailLengthController : MonoBehaviour
{
    [SerializeField] private TrailRenderer trail;
    [SerializeField] private Rigidbody targetBody;

    [SerializeField] private float targetLength = 0.8f;

    [SerializeField] private float minTime = 0.03f;
    [SerializeField] private float maxTime = 0.20f;

    void Update()
    {
        float speed = targetBody.velocity.magnitude;

        if (speed < 0.01f)
        {
            trail.time = maxTime;
            return;
        }

        float time =
            targetLength / speed;

        trail.time =
            Mathf.Clamp(
                time,
                minTime,
                maxTime);
    }
}