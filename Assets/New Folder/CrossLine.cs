using UnityEngine;
using System.Collections;

public class CrossLine : MonoBehaviour
{
    public Camera targetCamera;
    public GameObject LineParent;

    public float distanceFromCamera = 8f;
    public float crossHalfSize = 0.5f;
    public float lineWidth = 0.05f;

    private LineRenderer verticalLine;
    private LineRenderer horizontalLine;

    void Start()
    {
        if (targetCamera == null)
            targetCamera = Camera.main;

        if (LineParent == null)
            LineParent = new GameObject("LineParent");

        StartCoroutine(DrawLine());
    }

    IEnumerator DrawLine()
    {
        yield return new WaitForSeconds(3f);

        Vector3 center = targetCamera.ViewportToWorldPoint(
            new Vector3(0.5f, 0.5f, distanceFromCamera)
        );

        Vector3 up = targetCamera.transform.up * crossHalfSize;
        Vector3 right = targetCamera.transform.right * crossHalfSize;

        verticalLine = CreateLineRenderer("VerticalLine");
        horizontalLine = CreateLineRenderer("HorizontalLine");

        verticalLine.positionCount = 2;
        verticalLine.SetPosition(0, center - up);
        verticalLine.SetPosition(1, center + up);

        horizontalLine.positionCount = 2;
        horizontalLine.SetPosition(0, center - right);
        horizontalLine.SetPosition(1, center + right);
    }

    LineRenderer CreateLineRenderer(string name)
    {
        GameObject obj = new GameObject(name);
        obj.transform.SetParent(LineParent.transform, true);

        LineRenderer lr = obj.AddComponent<LineRenderer>();
        lr.useWorldSpace = true;
        lr.material = new Material(Shader.Find("Sprites/Default"));
        lr.startWidth = lr.endWidth = lineWidth;
        lr.startColor = lr.endColor = Color.white;
        return lr;
    }
}