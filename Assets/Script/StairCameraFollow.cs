using UnityEngine;

public class StairCameraFollow : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private Vector3 offset = new Vector3(0f, 6f, -7f);
    [SerializeField] private bool rotateWithTarget = true;
    [SerializeField] private float positionLerp = 8f;
    [SerializeField] private float rotationLerp = 8f;
    [SerializeField] private Vector3 lookAtOffset = new Vector3(0f, 1f, 0f);

    public void SetTarget(Transform newTarget)
    {
        target = newTarget;
    }

    private void LateUpdate()
    {
        if (target == null)
            return;

        Quaternion heading = rotateWithTarget ? Quaternion.Euler(0f, target.eulerAngles.y, 0f) : Quaternion.identity;
        Vector3 desiredPosition = target.position + heading * offset;
        transform.position = Vector3.Lerp(transform.position, desiredPosition, positionLerp * Time.deltaTime);

        Vector3 lookTarget = target.position + lookAtOffset;
        Quaternion desiredRotation = Quaternion.LookRotation(lookTarget - transform.position, Vector3.up);
        transform.rotation = Quaternion.Slerp(transform.rotation, desiredRotation, rotationLerp * Time.deltaTime);
    }
}
