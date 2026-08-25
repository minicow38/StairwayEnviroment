using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;

public class InsertSplinePath : MonoBehaviour
{
    public List<BezierKnot> splineActive;
    public List<Spline> reversedPlaneNeighbor;

    public SplineContainer DuplicatePlaneNeighbor;
    public SplineContainer CircleEmbled;

    public GameObject Player;

    public bool VerticalChanger = false;
    GameObject CornnerFirst;
    private LineRenderer lr;
    Vector3[][] positions = new Vector3[2][];

    public BezierKnot[] SevenLine;

    public class CoreSpline
    {
        public Vector3 startPoint;
        public Vector3 endPoint;
        public Vector3 breakpoint;
        public Vector3 bentDir;
        public Vector3 DirLine;
    }

    [SerializeField] public float justifyWidth = 0f;

    public List<Spline> planeNeighbor;
    List<CoreSpline> EntrySpline = new List<CoreSpline>();

    public GameObject mainStream;

    public string chunkWord;

    public float[] LinapLine1;
    public float[] LinapLine2;

    public ChangeSpline[] TotalLines;

    public SplineContainer splineContainer;
    public float OneSquareLength = 0f;
    public float OneSquareWidth = 0f;
    private float[] planeWidth;
    private float thirdProcess = 0f;
    public int numberOfKnots = 8;

    [SerializeField] private int bendStartIndex = 5;
    [SerializeField] private float bendAngle = 45f;

    [Header("Center Edge")]
    [SerializeField] private float centerEdgeStepOverride = 0f;

    [Header("Debug")]
    [SerializeField] private bool drawDebugGizmos = true;

    Vector3 startPoint;
    Vector3 endPoint;

    LineRenderer CreateLineRender(GameObject obj,Vector3[]pos)
    {
       // pos = new Vector3[2]; 
       //var lr=CreateLineRender(sphere1,new Vector3[]{sphere1.transform.position,sphere2.transform.position});

        if (obj.transform.GetComponent<LineRenderer>() == null)
        {
            obj.AddComponent<LineRenderer>();
        }

        var lr=obj.transform.GetComponent<LineRenderer>();
       Vector3 [][]positions =new Vector3[][]
        {
            pos,
            new Vector3[]{}
        };


        lr.positionCount = 2;
        lr.startWidth = lr.endWidth = 0.125f;
        lr.startColor = Color.white;
        lr.material = new Material(Shader.Find("Sprites/Default"));
        lr.SetPositions(positions[0]);
        return lr;
    }
    
    private struct LaneParameter
    {
        public int breakIndex;
        public Vector3 altStartPoint;
        public Vector3 altEndPoint;
        public Vector3 breakPoint;
        public Vector3 baseDir;
        public Vector3 bentDir;
        public Vector3 bentEndPoint;
        public float firstLength;
        public float secondLength;
    }

    private void Start()
    {
        Debug.Log("");
        SevenLine = new BezierKnot[3];

        InitializeFields();
        ResolveSceneReferences();

        if (splineContainer == null)
        {
            Debug.LogError("SplineContainer が見つかりません。");
            return;
        }
       // lr=CreateLineRender(out centerObj);

        ExtendSplineWithKnots();

        Player = GameObject.Find("Subject");
        var centerObj = Instantiate(GameObject.CreatePrimitive(PrimitiveType.Sphere), Vector3.zero, Quaternion.identity);
        centerObj.transform.SetParent(GameObject.Find("StairwaySimple").transform.Find("Plane (1)").transform);
        centerObj.transform.localScale = Vector3.one * 0.5f;
        
        centerObj.transform.name = "Plane1Center";
        centerObj.transform.localPosition = Vector3.zero;
        centerObj.transform.GetComponent<MeshRenderer>().material.color = Color.green;

        if (Player == null)
        {
            Player = GameObject.Find("subject");
        }

        mainStream = GameObject.Find("MainStream");

        StartCoroutine(Delay(0.8f));

        Debug.Log("");
    }

    private void InitializeFields()
    {
        if (LinapLine1 == null || LinapLine1.Length != 3)
        {
            LinapLine1 = new float[3];
        }

        if (LinapLine2 == null || LinapLine2.Length != 3)
        {
            LinapLine2 = new float[3];
        }

        if (planeWidth == null || planeWidth.Length != 4)
        {
            planeWidth = new float[4];
        }

        if (splineActive == null)
        {
            splineActive = new List<BezierKnot>();
        }

        if (reversedPlaneNeighbor == null)
        {
            reversedPlaneNeighbor = new List<Spline>();
        }

        if (planeNeighbor == null)
        {
            planeNeighbor = new List<Spline>();
        }

        if (EntrySpline == null)
        {
            EntrySpline = new List<CoreSpline>();
        }

        while (EntrySpline.Count < 3)
        {
            EntrySpline.Add(new CoreSpline());
        }
    }

    private void ResolveSceneReferences()
    {
        Transform stairPlane = GameObject.Find("StairwaySimple")?.transform.Find("Plane");
        if (stairPlane == null)
        {
            Debug.LogError("StairwaySimple/Plane が見つかりません。");
            return;
        }

        MeshCollider meshCollider = stairPlane.GetComponent<MeshCollider>();
        if (meshCollider == null)
        {
            Debug.LogError("Plane に MeshCollider がありません。");
            return;
        }

        OneSquareLength = meshCollider.bounds.size.z;
        OneSquareWidth = meshCollider.bounds.size.x;

        for (int i = 0; i < 4; i++)
        {
            planeWidth[i] = i * OneSquareLength / 4f;
        }

        thirdProcess = OneSquareLength / 4f;

        startPoint = new Vector3(
            transform.position.x + OneSquareWidth / 2f,
            0f,
            0f
        );

        endPoint = new Vector3(
            transform.position.x - OneSquareWidth / 2f,
            2f,
            0f
        );

        splineContainer = GetComponent<SplineContainer>();
        TotalLines = FindObjectsOfType<ChangeSpline>();
    }

    private void ExtendSplineWithKnots()
    {
        if (numberOfKnots < 2)
        {
            Debug.LogError("numberOfKnots は 2 以上にしてください。");
            return;
        }

        EnsureSplineCount(3);
        ClearAllSplines();

        planeNeighbor.Clear();
        splineActive.Clear();

        List<Spline> splines = new List<Spline>();

        for (int m = 0; m < 3; m++)
        {
            Spline newSpline = splineContainer.Splines[m];
            newSpline.Clear();
            newSpline.Closed = false;

            Vector3 altStartPoint = GetLaneStartPoint(m);
            Vector3 altEndPoint = GetLaneBaseEndPoint(m);

            LaneParameter lane = BuildLaneParameter(m, altStartPoint, altEndPoint);

            VerticalChanger = false;

            BuildFrontHalf(newSpline, lane);
            Vector3 tailEnd = BuildBackHalf(newSpline);

            EntrySpline[m].startPoint = lane.altStartPoint;
            EntrySpline[m].endPoint = tailEnd;
            EntrySpline[m].breakpoint = lane.breakPoint;
            EntrySpline[m].bentDir = lane.bentDir;
            EntrySpline[m].DirLine = (tailEnd - lane.bentEndPoint).normalized;

            float thirdLength = Vector3.Distance(lane.bentEndPoint, tailEnd);
            LinapLine1[m] = lane.firstLength + lane.secondLength + thirdLength;
            LinapLine2[m] = 0f;

            splineActive = newSpline.Knots.ToList();
           SevenLine[m] = splineActive[7];
            splines.Add(newSpline);
            var list = newSpline.Knots.ToList();
            Debug.Log("");
        }

        //var two = math.distance(SevenLine[1].Position, SevenLine[0].Position);
        //Vector3 p2World = transform.TransformPoint((Vector3)SevenLine[0].Position);
        float3 p1World = transform.TransformPoint(splines[0].Knots.ToList()[7].Position);
        float3 p2World = transform.TransformPoint(splines[1].Knots.ToList()[7].Position);
        float3 p3World = transform.TransformPoint(splines[2].Knots.ToList()[7].Position);
        
        var dis1 = math.distance(p2World ,p1World);
        var dis2 = math.distance(p3World ,p2World);
        
        var dir1=math.normalize(p1World - p2World);
        var dir2=math.normalize(p2World - p3World);
        var dir3=math.normalize(p1World - p2World);
        
        var sphere0=Instantiate(GameObject.CreatePrimitive(PrimitiveType.Sphere),p1World-dir1*dis1, Quaternion.identity);
        var sphere1=Instantiate(GameObject.CreatePrimitive(PrimitiveType.Sphere),p2World+dir2*dis1, Quaternion.identity);
       var sphere2=Instantiate(GameObject.CreatePrimitive(PrimitiveType.Sphere),p2World-dir2*dis2, Quaternion.identity);
       
       sphere0.transform.localScale = Vector3.one;
       sphere0.transform.name = "BorderSquare0";
       sphere0.transform.GetComponent<MeshRenderer>().material.color = Color.red;
       sphere0.transform.SetParent(GameObject.Find("StairwaySimple").transform.Find("Plane (1)").transform);

       sphere1.transform.localScale = Vector3.one*0.5f;
       sphere1.transform.name = "BorderSquare1";
       sphere1.transform.SetParent(GameObject.Find("StairwaySimple").transform.Find("Plane (1)").transform);
       
       sphere2.transform.localScale = Vector3.one*0.5f;
       sphere2.transform.name = "BorderSquare2";
       sphere2.transform.SetParent(GameObject.Find("StairwaySimple").transform.Find("Plane (1)").transform);

       
       //var lr=CreateLineRender(sphere1,new Vector3[]{sphere1.transform.parent.TransformPoint(sphere1.transform.position),sphere2.transform.parent.TransformPoint(sphere2.transform.position)});
       CreateLineRender(sphere1,new Vector3[]{sphere1.transform.position,sphere2.transform.position});
       //CreateLineRender(sphere1, new Vector3[] { sphere2.transform.position, sphere1.transform.position });
        planeNeighbor = splines;

        
        if (TotalLines != null)
        {
            for (int i = 0; i < TotalLines.Length; i++)
            {
                if (TotalLines[i] != null)
                {
                    TotalLines[i].LinenapLine = LinapLine1;
                }
            }
        }

        GameObject connerSecond = GameObject.Find("ConnerSecond");
        if (connerSecond != null)
        {
            PowTwin2 powTwin2 = connerSecond.GetComponent<PowTwin2>();
            if (powTwin2 != null)
            {
                powTwin2.planeNeighbor = planeNeighbor;
            }
        }
    }

    private Vector3 GetLaneStartPoint(int m)
    {
        return new Vector3(
            -startPoint.x,
            0.075f,
            startPoint.z - planeWidth[m] + thirdProcess
        );
    }

    private Vector3 GetLaneBaseEndPoint(int m)
    {
        return new Vector3(
            -endPoint.x + justifyWidth + 1f,
            0.075f,
            endPoint.z - planeWidth[m] + thirdProcess
        );
        Debug.Log("");
    }

    private LaneParameter BuildLaneParameter(int laneIndex, Vector3 altStartPoint, Vector3 altEndPoint)
    {
        LaneParameter lane = new LaneParameter();

        lane.altStartPoint = altStartPoint;
        lane.altEndPoint = altEndPoint;
        lane.breakIndex = Mathf.Clamp(bendStartIndex, 1, numberOfKnots - 2);

        float breakT = lane.breakIndex / (float)(numberOfKnots - 1);

        lane.breakPoint = Vector3.Lerp(lane.altStartPoint, lane.altEndPoint, breakT);
        lane.baseDir = (lane.altEndPoint - lane.altStartPoint).normalized;
        lane.bentDir = (Quaternion.AngleAxis(-bendAngle, Vector3.forward) * lane.baseDir).normalized;

        float step = centerEdgeStepOverride > 0f
            ? centerEdgeStepOverride
            : (justifyWidth > 0f ? justifyWidth : OneSquareWidth / 4f);
        
        // 中央Spline1を基準
        float centerX = GetLaneBaseEndPoint(1).x;

        float targetX;
        if (laneIndex == 0)      targetX = centerX - step; // 右
        else if (laneIndex == 1) targetX = centerX;        // 中央
        else                     targetX = centerX+step; // 左

        lane.bentEndPoint = new Vector3(
            targetX,
            lane.breakPoint.y + lane.bentDir.y * Mathf.Abs((targetX - lane.breakPoint.x) / Mathf.Max(0.0001f, lane.bentDir.x)),
            lane.altEndPoint.z
        );

        lane.firstLength = Vector3.Distance(lane.altStartPoint, lane.breakPoint);
        lane.secondLength = Vector3.Distance(lane.breakPoint, lane.bentEndPoint);

        return lane;
    }

    private float GetTargetCenterEdgeX(int laneIndex, float baseX)
    {
        float step = centerEdgeStepOverride;
        if (step <= 0f)
        {
            step = justifyWidth > 0f ? justifyWidth : OneSquareWidth / 4f;
        }

        // laneIndex:
        // 0 = 右
        // 1 = 中央
        // 2 = 左
        float offsetIndex = laneIndex-1f;

        return baseX + step * offsetIndex;
    }

    private void BuildFrontHalf(Spline spline, LaneParameter lane)
    {
        for (int i = 0; i < numberOfKnots; i++)
        {
            Vector3 posFirst;

            if (i <= lane.breakIndex)
            {
                float localT = lane.breakIndex == 0 ? 0f : i / (float)lane.breakIndex;
                posFirst = Vector3.Lerp(lane.altStartPoint, lane.breakPoint, localT);
                VerticalChanger = true;
            }
            else
            {
                float denom = (numberOfKnots - 1) - lane.breakIndex;
                float localT = denom <= 0f ? 1f : (i - lane.breakIndex) / denom;

                // numberOfKnots = 8 のとき、i == 7 が knot[7]
                posFirst = Vector3.Lerp(lane.breakPoint, lane.bentEndPoint, localT);
                VerticalChanger = false;
            }

            spline.Add(new BezierKnot(posFirst));
        }
    }

    private Vector3 BuildBackHalf(Spline spline)
    {
        float onePieceDis = (Vector3.Distance(endPoint, startPoint) / numberOfKnots) / 2f;

        Vector3 p0 = (Vector3)spline[0].Position;
        Vector3 p1 = (Vector3)spline[1].Position;
        Vector3 direction = (p1 - p0).normalized;

        // 元コードどおり、後半は最初の進行方向を Y 軸で -90 度回した方向
        Vector3 rotate = (Quaternion.AngleAxis(-90f, Vector3.up) * direction).normalized;

        Vector3 firstTailPoint = (Vector3)spline[numberOfKnots - 1].Position + rotate * onePieceDis;
        spline.Add(new BezierKnot(firstTailPoint));

        Vector3 tailEnd = firstTailPoint + rotate * (onePieceDis * (numberOfKnots - 1));

        for (int j = 1; j < numberOfKnots; j++)
        {
            float subT = j / (float)(numberOfKnots - 1);
            Vector3 pos2 = Vector3.Lerp(firstTailPoint, tailEnd, subT);
            spline.Add(new BezierKnot(pos2));
        }

        return tailEnd;
    }

    private void EnsureSplineCount(int requiredCount)
    {
        if (splineContainer == null)
        {
            return;
        }

        if (splineContainer.Splines.Count == 0)
        {
            SplineUtility.AddSpline(splineContainer);
        }

        while (splineContainer.Splines.Count < requiredCount)
        {
            SplineUtility.AddSpline(splineContainer);
        }
    }

    private void ClearAllSplines()
    {
        if (splineContainer == null)
        {
            return;
        }

        for (int i = 0; i < splineContainer.Splines.Count; i++)
        {
            splineContainer.Splines[i].Clear();
            splineContainer.Splines[i].Closed = false;
        }
    }

    private void LateUpdate()
    {
        ReflectRayPlane(Input.mousePosition);
    }

    private void OnEnable()
    {
    }

    private float ReflectRayPlane(Vector3 reflectPoint)
    {
        float angle = 0f;

        if (Camera.main == null)
        {
            return angle;
        }

        Ray ray = Camera.main.ScreenPointToRay(reflectPoint);
        Plane underPlane = new Plane(Vector3.up, Vector3.zero);

        if (underPlane.Raycast(ray, out float enter))
        {
            Vector3 hitPoint = ray.GetPoint(enter);
            float planeAngle = Vector3.Angle(underPlane.normal, Vector3.up);
        }

        return angle;
    }

    public void CreateReversedSplineClones()
    {
        DuplicatePlaneNeighbor = GetComponent<SplineContainer>();
        if (DuplicatePlaneNeighbor == null)
        {
            return;
        }

        reversedPlaneNeighbor = new List<Spline>();
        EnsureDuplicateSplineCount(planeNeighbor.Count);

        int secCount = 0;

        foreach (var orig in planeNeighbor)
        {
            Spline clone = DuplicatePlaneNeighbor.Splines[secCount];

            clone.Clear();
            clone.Closed = orig.Closed;

            var origKnots = orig.Knots.ToList();
            for (int i = origKnots.Count - 1; i >= 0; i--)
            {
                var k = origKnots[i];
                var newKnot = new BezierKnot(k.Position, k.TangentIn, k.TangentOut, k.Rotation);
                clone.Add(newKnot);
            }

            reversedPlaneNeighbor.Add(clone);
            secCount++;
        }
    }

    private void EnsureDuplicateSplineCount(int requiredCount)
    {
        if (DuplicatePlaneNeighbor == null)
        {
            return;
        }

        if (DuplicatePlaneNeighbor.Splines.Count == 0)
        {
            SplineUtility.AddSpline(DuplicatePlaneNeighbor);
        }

        while (DuplicatePlaneNeighbor.Splines.Count < requiredCount)
        {
            SplineUtility.AddSpline(DuplicatePlaneNeighbor);
        }
    }

    private IEnumerator Delay(float time)
    {
        yield return new WaitForSeconds(time);
        var gat=GameObject.Find("Plane (1)").transform.GetComponent<MeshCollider>().bounds.size.x;
        var height = GameObject.Find("Plane (1)").transform.GetComponent<MeshCollider>().bounds.size.z;

        GameObject subject = GameObject.Find("subject");
        if (subject == null)
        {
            subject = GameObject.Find("Subject");
        }

        if (subject == null || splineContainer == null)
        {
            yield break;
        }

        SplineAnimate anim = subject.GetComponent<SplineAnimate>();
        if (anim == null)
        {
            anim = subject.AddComponent<SplineAnimate>();
        }

        anim.PlayOnAwake = false;
        anim.Container = splineContainer;
        anim.Duration = 64f;
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawDebugGizmos || splineContainer == null)
        {
            return;
        }

        Gizmos.color = Color.red;
        for (int i = 0; i < splineContainer.Splines.Count; i++)
        {
            Spline s = splineContainer.Splines[i];
            if (s == null || s.Count <= 7)
            {
                continue;
            }

            Vector3 k7 = transform.TransformPoint((Vector3)s[7].Position);
            Gizmos.DrawSphere(k7, 0.08f);
        }
    }
}