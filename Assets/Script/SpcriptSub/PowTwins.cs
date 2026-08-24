using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEngine;
using System.Collections;
using System;
using UnityEngine.Splines;

public class PowTwin1 : MonoBehaviour
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
    //public HandwritingManager StereoBasicUI;

    [SerializeField] public float ellipseX = 0.25f;
    [SerializeField] public float ellipseZ = 0.5f;

    public float activeAngle;

    public bool ShowUpObj = false;

    public GameObject SimulateName;

    private GameObject[] points;
    public readonly LineRenderer[] renderers = new LineRenderer[6];

    private int numSplinePts = 16;
    private int[] counts = { 100, 2, 2, 2, 2, 3 };

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

    IEnumerator Start()
    {
        //StereoBasicUI = GameObject.Find("BasicUI").GetComponent<HandwritingManager>();
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
        for (int i = 0; i < renderers.Length; i++)
        {
           

            if (i >= 2 && i < renderers.Length - 1)
            {
                try
                {
                    points[i - 2] = CreateMarker("Point" + (i - 2), colors[i - 2]);
                }
                catch (Exception e)
                {
                    Debug.Log("");
                }
            }
        }
        yield return new WaitForSeconds(0.5f);
        
        
        Player = GameObject.Find("subject").gameObject;
        PickUpSpot = Player.transform;

        var up = Vector3.up * 0.015f;
        NeedlePoint.position += up;
        PickUpSpot.position += up;

        meshCollider = Player.GetComponent<MeshCollider>();
        
       

        var k1 = planeNeighbor[1].Knots.ToList()[0];
        var k2 = planeNeighbor[2].Knots.ToList()[0];
        radius = Vector3.Distance((Vector3)k1.Position, (Vector3)k2.Position) * 0.75f;
        var playerSubCollect = Player.transform;
        bool minNority = playerSubCollect.Cast<Transform>().Any(t =>
            Regex.Match(t.name, @"InTheCyrcle").Success
        );

        GameObject groupColorBall = GameObject.Find("InTheCyrcle");

        if (!minNority && groupColorBall == null)
        {
            groupColorBall = new GameObject("InTheCyrcle");
            groupColorBall.transform.SetParent(Player.transform);
        }

        SimulateName = new GameObject("Simulate" + groupColorBall.transform.childCount);
        
       

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

        SimulateName.transform.SetParent(groupColorBall.transform);
        for (int i = 0; i < 3; i++)
        {
            points[i].transform.SetParent(Player.transform.Find("InTheCyrcle").Find("Simulate0"));
        }
        renderers[(int)LineType.Circle].startWidth =
            renderers[(int)LineType.Circle].endWidth = 0.01f;

        //CircleEmbled = StereoBasicUI.CornnerApproch.AddComponent<SplineContainer>();
        ShowUpObj = true;
        CurrentActiveAngle = activeAngle;
        CurrentTDeg = tDeg;
    }

    // sign: -1 => pickup - offset（旧Center1）, +1 => pickup + offset（旧Center2）
    private Vector3 ComputeEllipseCenter(Vector3 pickup, float rotateDeg, float tDeg, float sign)
    {
        float t = tDeg * Mathf.Deg2Rad;

        worldOffset =
            Quaternion.AngleAxis(rotateDeg, Vector3.up)
            * new Vector3(semiMinorA * Mathf.Cos(t), 0f, semiMinorB * Mathf.Sin(t));

        return pickup + sign * worldOffset;
    }

    void Update()
    {
        
        Vector3 pickupPos = PickUpSpot.position;

        if (ChangingAcrossMainSeek && SeekingLineAcross && tDeg != CurrentTDeg)
        {
            PickUpSpot.position = ComputeEllipseCenter(
                pickupPos,
                RotateAnglePos,
                tDeg,
                UsePositiveOffset ? 1f : -1f
            );

            bool flip = WorldBoundsReady && !UsePositiveOffset;
            /*if (StereoBasicUI.FirstApproch || flip)
            {
                _ellipseCenter = pickupPos;
            }*/

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

        CurrentActiveAngle = activeAngle;

       /* if (!StereoBasicUI.FirstAct)
        {
            DrawCircle();
            DrawGeometries();
        }*/
       if (ShowUpObj)
       {
           DrawCircle();
           DrawGeometries();
       }

       CurrentTDeg = tDeg;
    }

    void DrawCircle()
    { 

        Quaternion q = Quaternion.AngleAxis(RotateAnglePos, Vector3.up);

        Vector3 centerWorld = (OnEllipse || SeekingLineAcross)
            ? _ellipseCenter
            : PickUpSpot.position;

        const float yOffset = 0.01f;
        Vector3 up = Vector3.up * yOffset;

        var lr = renderers[(int)LineType.Circle];
        lr.loop = true;
        lr.positionCount = counts[0];

        var spline = CircleEmbled.Splines[0];
        //spline.Clear();

        Vector3[] linePts = new Vector3[counts[0]];

        for (int i = 0; i < counts[0]; i++)
        {
            float t = (2f * Mathf.PI * i) / counts[0];

            Vector3 rot = q * new Vector3(semiMinorA * Mathf.Cos(t),0f,semiMinorB * Mathf.Sin(t));

            linePts[i] = centerWorld + rot + up;
        }

        lr.SetPositions(linePts);

        for (int i = 0; i < numSplinePts; i++)
        {
            float t = (2f * Mathf.PI * i) / numSplinePts;

            Vector3 rot = q * new Vector3(semiMinorA * Mathf.Cos(t),0f,semiMinorB * Mathf.Sin(t));

            // 元コード同様 centerWorld を足さない
            Vector3 worldPos = rot + up;

            Vector3 dRot = q * new Vector3(-semiMinorA * Mathf.Sin(t),0f,semiMinorB * Mathf.Cos(t));

            Vector3 worldSin = dRot.normalized * (0.2f * radius);

            Vector3 localPos = CircleEmbled.transform.InverseTransformPoint(worldPos);
            Vector3 localSin = CircleEmbled.transform.InverseTransformVector(worldSin);
            /*if(spline.Count<17)
            spline.Add(new BezierKnot(localPos)
            {
                TangentIn = -localSin,
                TangentOut = localSin
            });*/
        }

        //spline.Closed = true;
    }

    void DrawGeometries()
    {
        Vector3 N = NeedlePoint.position;
        Vector3 P = OnEllipse ? _ellipseCenter : PickUpSpot.position;
        float d = (N - P).magnitude;
        if (d <= radius)
        {
            ClearAllLines();
            return;
        }

        // _A
        float radA = Mathf.Deg2Rad * angleA;
        Vector3 A = P + new Vector3(Mathf.Cos(radA), 0, Mathf.Sin(radA)) * radius;
        points[1].transform.position = new Vector3(A.x, 0.075f, A.z);

        // ===== ここは「残す」指定ブロック（そのまま）=====
        float theta = Mathf.Acos(radius / d) * Mathf.Rad2Deg;
        Vector3 dirToP = (N - P).normalized;

        Vector3 T = P
            + Quaternion.AngleAxis(theta + activeAngle, Vector3.up) * dirToP * radius;
        points[0].transform.position = new Vector3(T.x, 0.075f, T.z);

        float pt2 = (N - T).sqrMagnitude;

        float paLength = Vector3.Distance(N, A);
        float pbLength = pt2 / paLength;

        Vector3 B = N + (A - N).normalized * pbLength;
        points[2].transform.position = new Vector3(B.x, 0.075f, B.z);

        Vector3 center = (A + B + T) / 3f;
        Vector3 t = center + Vector3.up * 2.5f;
        // ===== ここまで =====

        float spotAngle = RotateAnglePos * Mathf.Deg2Rad;

        // ★短縮：ローカル関数 + タプル代入で4回の回転をまとめる
        Vector3 R(Vector3 v) => RotatePointAroundPivot(v, P, spotAngle);
        (A, B, T, t) = (R(A), R(B), R(T), R(t));

        var roots = Player.transform.Find("InTheCyrcle").GetChild(0);
        for (int i = 0; i < 3; i++)
        {
            var p = roots.Find("Point" + i);
            p.position = R(p.position);
        }

        // ★短縮：InverseTransformもまとめる
        Vector3 Inv(Vector3 v) => transform.InverseTransformPoint(v);
        var vertices = new[] { Inv(A), Inv(B), Inv(T), Inv(t) };

        var mesh = new Mesh
        {
            vertices = vertices,
            triangles = new[] { 0, 1, 2, 0, 1, 3, 1, 2, 3, 2, 0, 3 }
        };

        mesh.RecalculateNormals();
        meshCollider.sharedMesh = mesh;
    }

    Vector3 RotatePointAroundPivot(Vector3 point, Vector3 pivot, float angle)
    {
        Vector3 v = point - pivot;

        float c = Mathf.Cos(angle);
        float s = Mathf.Sin(angle);

        float lx = c * v.x - s * v.z;
        float lz = s * v.x + c * v.z;

        lx *= ellipseX;
        lz *= ellipseZ;

        float wx = c * lx + s * lz;
        float wz = -s * lx + c * lz;

        return pivot + new Vector3(wx, v.y, wz);
    }

    void ClearAllLines()
    {
        for (int i = 1; i < renderers.Length; i++)
        {
            renderers[i].positionCount = 0;
        }
    }

    GameObject CreateMarker(string name, Color color)
    {
        var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.localScale = Vector3.one * 0.125f;
        sphere.transform.position = transform.localPosition;
        GameObject obj = Instantiate(sphere);
        obj.name = name;

        Material material = obj.GetComponent<Renderer>().material;
        material.color = color;

        return obj;
    }

    LineRenderer InitLineRenderer(string name, Color color, float width, int count)
    {
        GameObject obj = new GameObject(name);
        obj.transform.parent = this.transform;

        var lr = obj.AddComponent<LineRenderer>();
        lr.material = new Material(Shader.Find("Sprites/Default"));
        lr.startWidth = lr.endWidth = width;
        lr.positionCount = count;
        lr.startColor = lr.endColor = color;
        lr.useWorldSpace = true;

        return lr;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.transform.CompareTag("Clasher"))
        {
            Debug.Log("Collision Pattern");
        }

        if (other.transform.CompareTag("Player"))
        {
            Debug.Log("Collision Pattern2");
        }
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