using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Splines;

public class PowTwin2 : MonoBehaviour
{
    [Header("Transform")]
    [SerializeField] public Transform PickUpSpot;
    [SerializeField] public Transform NeedlePoint;

    public bool SeekingLineAcross = true;
    public bool ChangingAcrossMainSeek = true;
    public bool OnEllipse = true;

    [SerializeField] public float radius = 1f;
    public float multi;
    [SerializeField] public float angleA = 0f;
    [SerializeField] public float RotateAnglePos = 0f;

    public SplineContainer CircleEmbled;
    public List<Spline> planeNeighbor;

    public GameObject Player;
    public MeshCollider meshCollider;

    [SerializeField] public float ellipseX = 0.25f;
    [SerializeField] public float ellipseZ = 0.5f;

    public float activeAngle;
    public bool ShowUpObj = false;

    public GameObject SimulateName;

    private GameObject[] points;
    public readonly LineRenderer[] renderers = new LineRenderer[6];

    private readonly int numSplinePts = 16;
    private readonly int[] counts = { 100, 2, 2, 2, 2, 3 };

    private enum LineType
    {
        Circle,
        Tangent,
        Secant,
        PA,
        PB,
        PAB
    }

    [Header("Ellipse Params")]
    public float tDeg = 0f;
    public float semiMinorA => ellipseX * radius;
    public float semiMinorB => ellipseZ * radius;

    public Vector3 _ellipseCenter;
    public Vector3 worldOffset;

    public bool WorldBoundsReady = false;
    public bool UsePositiveOffset = false;

    public float CurrentActiveAngle = 0;
    public float CurrentTDeg = 0;

    private const float InitialOffsetY = 0.015f;
    private const float CircleYOffset = 0.01f;
    private const float MarkerYOffset = 0.075f;

    private Mesh runtimeMesh;
    private Transform markerRoot;

    private Quaternion EllipseRotation => Quaternion.AngleAxis(RotateAnglePos, Vector3.up);

    IEnumerator Start()
    {
        Color[] colors =
        {
            Color.white,
            Color.green,
            Color.red,
            Color.blue,
            Color.cyan,
            Color.grey
        };

        points = new GameObject[3];
   
        for (int i = 0; i < renderers.Length; i++)
        {
            renderers[i] = InitLineRenderer(
                ((LineType)i).ToString(),
                colors[i],
                0.02f,
                counts[i]
            );
        }

        for (int i = 0; i < 3; i++)
        {
            points[i] = CreateMarker("Point" + i, colors[i]);
        }

        yield return new WaitForSeconds(0.5f);
      
        

        Player = GameObject.Find("subject");
        PickUpSpot = GameObject.Find("subject").transform;
        NeedlePoint = GameObject.Find("ConnerSecond").transform;
        // 元コード互換: 未設定なら subject を使う
        Vector3 up = Vector3.up * InitialOffsetY;
        NeedlePoint.position += up;
        PickUpSpot.position += up;

        if (meshCollider == null)
        {
            meshCollider = Player.GetComponent<MeshCollider>();
        }

        if (meshCollider == null)
        {
            meshCollider = GetComponent<MeshCollider>();
        }

        TryUpdateRadiusFromPlaneNeighbor();

        Transform groupColorBall = EnsureChild(Player.transform, "InTheCyrcle");
        SimulateName = new GameObject("Simulate" + groupColorBall.childCount);
        SimulateName.transform.SetParent(groupColorBall, false);
        markerRoot = SimulateName.transform;

        for (int i = 0; i < points.Length; i++)
        {
            points[i].transform.SetParent(markerRoot, false);
        }

        if (!ChangingAcrossMainSeek)
        {
            _ellipseCenter = PickUpSpot.position;
        }
        else
        {
            _ellipseCenter = ComputeEllipseCenter(
                PickUpSpot.position,
                RotateAnglePos,
                tDeg,
                -1f
            );
        }

        renderers[(int)LineType.Circle].startWidth = 0.01f;
        renderers[(int)LineType.Circle].endWidth = 0.01f;

        runtimeMesh = new Mesh
        {
            name = $"{nameof(PowTwin1)}_RuntimeMesh"
        };
        runtimeMesh.MarkDynamic();

        ShowUpObj = true;
        CurrentActiveAngle = activeAngle;
        CurrentTDeg = tDeg;
    }

    void Update()
    {
        if (!ShowUpObj || PickUpSpot == null || NeedlePoint == null)
        {
            return;
        }

        Vector3 pickupPos = PickUpSpot.position;

        if (ChangingAcrossMainSeek && SeekingLineAcross && !Mathf.Approximately(tDeg, CurrentTDeg))
        {
            // 元コードの意図を残しつつ 1 フレーム遅れを消す
            _ellipseCenter = pickupPos;

            PickUpSpot.position = ComputeEllipseCenter(
                pickupPos,
                RotateAnglePos,
                tDeg,
                UsePositiveOffset ? 1f : -1f
            );

            bool flip = WorldBoundsReady && !UsePositiveOffset;
            if (flip)
            {
                UsePositiveOffset = true;
            }

            WorldBoundsReady = true;
        }
        else if (ChangingAcrossMainSeek || SeekingLineAcross)
        {
            _ellipseCenter = ComputeEllipseCenter(pickupPos, RotateAnglePos, tDeg, -1f);
        }
        else
        {
            _ellipseCenter = PickUpSpot.position;
        }

        CurrentActiveAngle = activeAngle;

        DrawCircle();
        DrawGeometries();

        CurrentTDeg = tDeg;
    }

    // sign: -1 => pickup - offset, +1 => pickup + offset
    private Vector3 ComputeEllipseCenter(Vector3 pickup, float rotateDeg, float tDegValue, float sign)
    {
        float t = tDegValue * Mathf.Deg2Rad;

        worldOffset =
            Quaternion.AngleAxis(rotateDeg, Vector3.up)
            * new Vector3(semiMinorA * Mathf.Cos(t), 0f, semiMinorB * Mathf.Sin(t));

        return pickup + sign * worldOffset;
    }

    private void TryUpdateRadiusFromPlaneNeighbor()
    {
        if (planeNeighbor == null || planeNeighbor.Count < 3)
        {
            return;
        }

        if (planeNeighbor[1] == null || planeNeighbor[2] == null)
        {
            return;
        }

        var knot1 = planeNeighbor[1].Knots.FirstOrDefault();
        var knot2 = planeNeighbor[2].Knots.FirstOrDefault();

        radius = Vector3.Distance((Vector3)knot1.Position, (Vector3)knot2.Position) * 0.75f;
    }

    private Transform EnsureChild(Transform parent, string childName)
    {
        Transform child = parent.Find(childName);
        if (child != null)
        {
            return child;
        }

        GameObject go = new GameObject(childName);
        go.transform.SetParent(parent, false);
        return go.transform;
    }

    private Vector3 GetActiveCenter()
    {
        return (OnEllipse || SeekingLineAcross) ? _ellipseCenter : PickUpSpot.position;
    }

    private Vector3 CircleLocalToWorld(Vector3 center, Vector3 circleLocal)
    {
        Vector3 ellipseLocal = new Vector3(
            circleLocal.x * ellipseX,
            circleLocal.y,
            circleLocal.z * ellipseZ
        );

        return center + EllipseRotation * ellipseLocal;
    }

    private Vector3 WorldToCircleLocal(Vector3 center, Vector3 world)
    {
        Vector3 rotated = Quaternion.Inverse(EllipseRotation) * (world - center);

        float x = Mathf.Approximately(ellipseX, 0f) ? 0f : rotated.x / ellipseX;
        float z = Mathf.Approximately(ellipseZ, 0f) ? 0f : rotated.z / ellipseZ;

        return new Vector3(x, rotated.y, z);
    }

    void DrawCircle()
    {
        Vector3 centerWorld = GetActiveCenter();
        Vector3 lineUp = Vector3.up * CircleYOffset;

        LineRenderer lr = renderers[(int)LineType.Circle];
        lr.loop = true;
        lr.positionCount = counts[0];

        Vector3[] linePts = new Vector3[counts[0]];

        for (int i = 0; i < counts[0]; i++)
        {
            float t = (2f * Mathf.PI * i) / counts[0];
            Vector3 circleLocal = new Vector3(
                Mathf.Cos(t) * radius,
                0f,
                Mathf.Sin(t) * radius
            );

            linePts[i] = CircleLocalToWorld(centerWorld, circleLocal) + lineUp;
        }

        lr.SetPositions(linePts);

        if (CircleEmbled == null || CircleEmbled.Splines.Count == 0)
        {
            return;
        }

        // Spline 側を使う場合も、LineRenderer と同じ座標を使う
        for (int i = 0; i < numSplinePts; i++)
        {
            float t = (2f * Mathf.PI * i) / numSplinePts;

            Vector3 circleLocal = new Vector3(
                Mathf.Cos(t) * radius,
                0f,
                Mathf.Sin(t) * radius
            );

            Vector3 circleTangentLocal = new Vector3(
                -Mathf.Sin(t) * radius,
                0f,
                Mathf.Cos(t) * radius
            );

            Vector3 worldPos = CircleLocalToWorld(centerWorld, circleLocal) + lineUp;

            Vector3 ellipseTangentLocal = new Vector3(
                circleTangentLocal.x * ellipseX,
                0f,
                circleTangentLocal.z * ellipseZ
            );

            Vector3 worldTangent = EllipseRotation * ellipseTangentLocal;
            Vector3 worldSin = worldTangent.normalized * (0.2f * radius);

            Vector3 localPos = CircleEmbled.transform.InverseTransformPoint(worldPos);
            Vector3 localSin = CircleEmbled.transform.InverseTransformVector(worldSin);

            // 必要なら spline.Add(...) をここで再開
            _ = localPos;
            _ = localSin;
        }
    }

    void DrawGeometries()
    {
        Vector3 centerWorld = GetActiveCenter();
        Vector3 needleLocal = WorldToCircleLocal(centerWorld, NeedlePoint.position);
        Vector3 needleLocalFlat = new Vector3(needleLocal.x, 0f, needleLocal.z);

        float d = needleLocalFlat.magnitude;
        if (d <= radius)
        {
            ClearAllLines();
            ClearMarkers();
            ClearMesh();
            return;
        }

        float radA = Mathf.Deg2Rad * angleA;
        Vector3 ALocal = new Vector3(Mathf.Cos(radA), 0f, Mathf.Sin(radA)) * radius;

        float theta = Mathf.Acos(Mathf.Clamp(radius / d, -1f, 1f)) * Mathf.Rad2Deg;
        Vector3 dirToNeedle = needleLocalFlat.normalized;

        Vector3 TLocal =
            Quaternion.AngleAxis(theta + activeAngle, Vector3.up) * dirToNeedle * radius;

        float pt2 = (needleLocalFlat - TLocal).sqrMagnitude;
        float paLength = Vector3.Distance(needleLocalFlat, ALocal);

        float pbLength = paLength > Mathf.Epsilon ? pt2 / paLength : 0f;
        Vector3 BLocal = needleLocalFlat + (ALocal - needleLocalFlat).normalized * pbLength;

        Vector3 centerLocal = (ALocal + BLocal + TLocal) / 3f;
        Vector3 topLocal = centerLocal + Vector3.up * 2.5f;

        Vector3 A = CircleLocalToWorld(centerWorld, ALocal);
        Vector3 B = CircleLocalToWorld(centerWorld, BLocal);
        Vector3 T = CircleLocalToWorld(centerWorld, TLocal);
        Vector3 top = CircleLocalToWorld(centerWorld, topLocal);

        SetMarkerPosition(0, T);
        SetMarkerPosition(1, A);
        SetMarkerPosition(2, B);

        UpdateMesh(A, B, T, top);
    }

    private void SetMarkerPosition(int index, Vector3 worldPoint)
    {
        if (points == null || index < 0 || index >= points.Length || points[index] == null)
        {
            return;
        }

        points[index].transform.position = worldPoint + Vector3.up * MarkerYOffset;
    }

    private void UpdateMesh(Vector3 a, Vector3 b, Vector3 t, Vector3 top)
    {
        if (meshCollider == null)
        {
            return;
        }

        if (runtimeMesh == null)
        {
            runtimeMesh = new Mesh
            {
                name = $"{nameof(PowTwin1)}_RuntimeMesh"
            };
            runtimeMesh.MarkDynamic();
        }

        runtimeMesh.Clear();

        runtimeMesh.vertices = new[]
        {
            transform.InverseTransformPoint(a),
            transform.InverseTransformPoint(b),
            transform.InverseTransformPoint(t),
            transform.InverseTransformPoint(top)
        };

        runtimeMesh.triangles = new[]
        {
            0, 1, 2,
            0, 1, 3,
            1, 2, 3,
            2, 0, 3
        };

        runtimeMesh.RecalculateNormals();
        runtimeMesh.RecalculateBounds();

        meshCollider.sharedMesh = null;
        meshCollider.sharedMesh = runtimeMesh;
    }

    void ClearAllLines()
    {
        for (int i = 1; i < renderers.Length; i++)
        {
            renderers[i].positionCount = 0;
        }
    }

    void ClearMarkers()
    {
        if (points == null)
        {
            return;
        }

        for (int i = 0; i < points.Length; i++)
        {
            if (points[i] != null)
            {
                points[i].SetActive(false);
            }
        }
    }

    void ClearMesh()
    {
        if (runtimeMesh != null)
        {
            runtimeMesh.Clear();
        }

        if (meshCollider != null)
        {
            meshCollider.sharedMesh = null;
        }
    }

    GameObject CreateMarker(string markerName, Color color)
    {
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.name = markerName;
        sphere.transform.localScale = Vector3.one * 0.125f;
        sphere.transform.position = transform.position;

        Collider col = sphere.GetComponent<Collider>();
        if (col != null)
        {
            Destroy(col);
        }

        Renderer renderer = sphere.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material = new Material(Shader.Find("Sprites/Default"));
            renderer.material.color = color;
        }

        return sphere;
    }

    LineRenderer InitLineRenderer(string lineName, Color color, float width, int count)
    {
        GameObject obj = new GameObject(lineName);
        obj.transform.SetParent(transform, false);

        LineRenderer lr = obj.AddComponent<LineRenderer>();
        lr.material = new Material(Shader.Find("Sprites/Default"));
        lr.startWidth = width;
        lr.endWidth = width;
        lr.positionCount = count;
        lr.startColor = color;
        lr.endColor = color;
        lr.useWorldSpace = true;

        return lr;
    }

    private void OnTriggerEnter(Collider other)
    {
        /*if (other.transform.CompareTag("Clasher"))
        {
            Debug.Log("Collision Pattern");
        }

        if (other.transform.CompareTag("Player"))
        {
            Debug.Log("Collision Pattern2");
        }*/
    }

    bool TryGetTangentPoint(
        Vector3 N,
        Vector3 P,
        float r,
        Vector3 u,
        out Vector3 Tsel,
        out Vector3 Tother
    )
    {
        Vector3 v = N - P;
        float d = v.magnitude;

        if (d <= r)
        {
            Tsel = Tother = Vector3.zero;
            return false;
        }

        Vector3 vHat = v / d;
        Vector3 nHat = Vector3.Cross(Vector3.up, vHat).normalized;

        float c = r / d;
        float s = Mathf.Sqrt(Mathf.Max(0f, 1f - c * c));

        Vector3 Tplus = P + r * (c * vHat + s * nHat);
        Vector3 Tminus = P + r * (c * vHat - s * nHat);

        if (Vector3.Dot(nHat, u.normalized) >= 0f)
        {
            Tsel = Tplus;
            Tother = Tminus;
        }
        else
        {
            Tsel = Tminus;
            Tother = Tplus;
        }

        return true;
    }
}