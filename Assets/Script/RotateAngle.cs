using UnityEngine;

public class RotateAngle : MonoBehaviour
{
    private Vector3 direction = Vector3.right;

    void Start()
    {
        Vector3 normal = Quaternion.AngleAxis(45f, Vector3.forward) * Vector3.up;

        // このメッシュの表向きが +Y だと仮定
        transform.rotation = Quaternion.FromToRotation(Vector3.up, normal);
    }
}