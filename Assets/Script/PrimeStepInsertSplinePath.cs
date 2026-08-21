using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Splines;

public class PrimeStepInsertSplinePath : MonoBehaviour
{
    [Header("External References")]
    public SplineContainer DuplicatePlaneNeighbor;
    public SplineContainer CircleEmbled;
    public GameObject Player;
    public GameObject mainStream;
    public Transform stairPlane;

    [Header("Runtime Data")]
    public List<BezierKnot> splineActive;
    public List<Spline> planeNeighbor;
    public List<Spline> reversedPlaneNeighbor;
    public BezierKnot[] SevenLine;
    public SplineContainer splineContainer;

    [Header("Lengths")]
    public float[] LinapLine1;
    public float[] LinapLine2;
    public float OneSquareLength = 0f;
    public float OneSquareWidth = 0f;

    [Header("Lane Layout")]
    [SerializeField] public float justifyWidth = 3f;
    [SerializeField] private float straightEndPadding = 1f;
    [SerializeField] private float centerEdgeStepOverride = 0f;
    float duringBentEndStepMultiplier = 2.0f;

    [Header("Spline Shape")]
    [SerializeField] private float bendAngle = 45f;
    [SerializeField] private float knotStepDistance = 2.0f;
    [SerializeField] private float topPlaneCurveAngle = 45f;
    [SerializeField] private float topPlaneCenterRadius = 6.0f;
    [SerializeField] private bool curveToPositiveZ = true;

    [Header("Debug")]
    [SerializeField] private bool drawDebugGizmos = true;
    [SerializeField] private bool spawnDebugObjects = false;

    [Header("Compatibility")]
    public bool VerticalChanger = false;
    public bool jurge = false;
    public string chunkWord;
    public float HalfStep = 0f;
    public float DuringHalfStep = 0f;

    private float[] planeWidth;
    private float thirdProcess = 0f;
    private readonly List<CoreSpline> EntrySpline = new List<CoreSpline>();

    // private Vector3 initAngle = Vector3.right;

    private Vector3 currentRotate = new Vector3(0f, 1f, 1f);
    private readonly Vector2 toggle = new Vector2(0f, 1f);
    private SquareFrame initialFrame;
    private SquareFrame currentFrame;
    private bool hasAnchorFrame = false;
    private bool includeStart = false;

    public Vector3 primeStart;
    public Vector3 primeEnd;

    private struct SquareFrame
    {
        public Vector3 startBase;
        public Vector3 endBase;
    }

    private struct LaneParameter
    {
        public Vector3 altStartPoint;
        public Vector3 altEndPoint;
        // public Vector3 breakPoint;
        public Vector3 baseDir;
        public Vector3 bentDir;
        public Vector3 DuringBentEndPoint;
        public float firstLength;
        public float secondLength;
    }

    public class CoreSpline
    {
        public Vector3 startPoint;
        public Vector3 endPoint;
        public Vector3 breakpoint;
        public Vector3 bentDir;
        public Vector3 DirLine;
    }

    Vector3 toggled()
    {
        Debug.Log(currentRotate);
        currentRotate = new Vector3(
            currentRotate.x * toggle.x + currentRotate.z * toggle.y,
            0,
            currentRotate.x * toggle.y + currentRotate.z * toggle.x
        );
        return currentRotate;
    }

    private void Start()
    {
        InitializeFields();
        ResolveSceneReferences();
        ExtendSplineWithKnots();
        // ResolveRuntimeReferences();
        // Vector3 refVec=Mathf.Abs(Vector3.Dot(T,))
        StartCoroutine(Delay(0.8f));
    }

    private void InitializeFields()
    {
        if (LinapLine1 == null || LinapLine1.Length != 3) LinapLine1 = new float[3];
        if (LinapLine2 == null || LinapLine2.Length != 3) LinapLine2 = new float[3];
        if (planeWidth == null || planeWidth.Length != 4) planeWidth = new float[4];
        if (splineActive == null) splineActive = new List<BezierKnot>();
        if (planeNeighbor == null) planeNeighbor = new List<Spline>();
        if (reversedPlaneNeighbor == null) reversedPlaneNeighbor = new List<Spline>();
        if (SevenLine == null || SevenLine.Length != 3) SevenLine = new BezierKnot[3];
        while (EntrySpline.Count < 3) EntrySpline.Add(new CoreSpline());
    }

    private void ResolveSceneReferences()
    {
        if (stairPlane == null)
        {
            stairPlane = GameObject.Find("StairwaySimple")?.transform.Find("Plane");
            MeshCollider meshCollider = stairPlane.GetComponent<MeshCollider>();
            OneSquareLength = meshCollider.bounds.size.z;
            OneSquareWidth = meshCollider.bounds.size.x;
        }

        for (int i = 0; i < 4; i++) planeWidth[i] = i * OneSquareLength / 4f;

        Vector3 initialStartBase = Vector3.zero;
        thirdProcess = OneSquareLength / 4f;
        splineContainer = GetComponent<SplineContainer>();

        initialStartBase = new Vector3(-(initialStartBase.x + OneSquareWidth / 2f), 0.075f, 0f);
        Vector3 initialEndBase = initialStartBase + Vector3.right * OneSquareWidth;

        initialFrame = new SquareFrame { startBase = initialStartBase, endBase = initialEndBase };
        currentFrame = initialFrame;
        hasAnchorFrame = false;
    }

    private void ResolveRuntimeReferences()
    {
        if (Player == null) Player = GameObject.Find("Subject");
        if (Player == null) Player = GameObject.Find("subject");
        if (mainStream == null) mainStream = GameObject.Find("MainStream");
    }

    private void SetNextSquareAnchor(Vector3 anchor)
    {
        currentFrame = new SquareFrame { startBase = anchor, endBase = anchor + Vector3.right * OneSquareWidth };
        hasAnchorFrame = true;
    }

    private float GetLaneStep()
    {
        float step = centerEdgeStepOverride > 0f
            ? centerEdgeStepOverride
            : (justifyWidth > 0f ? justifyWidth : OneSquareWidth / 4f);

        HalfStep = step;
        DuringHalfStep = step * duringBentEndStepMultiplier;

        // 結局3しか返ってこない
        if (step != 3)
        {
            Debug.Log("");
        }
        return step;
    }

    private float GetLaneSign(int laneIndex)
    {
        if (laneIndex == 0) return -1f;
        if (laneIndex == 1) return 0f;
        return 1f;
    }

    public Vector3 GetLaneStartPoint(int laneIndex, float height)
    {
        Vector3 shiftUp = currentRotate * (-planeWidth[laneIndex] + thirdProcess);
        Vector3 curveLine = new Vector3(
            currentFrame.startBase.x + shiftUp.x,
            height,
            currentFrame.startBase.z + shiftUp.z
        );
        return curveLine;
    }

    public Vector3 GetLaneBaseEndPoint(int laneIndex, float height)
    {
        Vector3 shiftUp = currentRotate * (-planeWidth[laneIndex] + thirdProcess);
        Vector3 curveLine = new Vector3(
            currentFrame.endBase.x + shiftUp.x,
            height,
            currentFrame.endBase.z + shiftUp.z
        );
        return curveLine;
    }

    LaneParameter BuildLaneParameter(int laneIndex, LaneParameter lane, List<Vector3> points)
    {
        lane.baseDir = Vector3.right;
        lane.bentDir = (Quaternion.AngleAxis(-bendAngle, Vector3.forward) * lane.baseDir).normalized;

        float step = GetLaneStep(); // 結局3しか返ってこない
        float center = GetLaneBaseEndPoint(1, lane.DuringBentEndPoint.y).x;
        float target = 0;
        int laneType = laneIndex;
        float forwardDistance = OneSquareLength / 2;
        float rightDistance = OneSquareWidth / 2;

        if (!includeStart)
        {
            target = center + step + step + straightEndPadding;
        }
        else
        {
            if (laneIndex % 3 == 0) forwardDistance = OneSquareWidth / 6;
            else if (laneIndex % 3 == 1) forwardDistance = OneSquareWidth * 0.5f;
            else forwardDistance = OneSquareWidth * 5 / 6;

            float safeStep = Mathf.Max(0.01f, step);
           
            Vector3 startPoint = lane.DuringBentEndPoint;
            Vector3 cornerPoint = currentFrame.startBase = new Vector3(
                lane.DuringBentEndPoint.x + forwardDistance,
                lane.DuringBentEndPoint.y,
                lane.DuringBentEndPoint.z
            );
            Vector3 endPoint = currentFrame.endBase = new Vector3(
                currentFrame.startBase.x,
                currentFrame.startBase.y,
                currentFrame.startBase.z + rightDistance
            );
            

            float horizontalLength = Vector3.Distance(startPoint, cornerPoint);
            float verticalLength = Vector3.Distance(cornerPoint, endPoint);
            int horizontalDiv = Mathf.Max(1, Mathf.CeilToInt(horizontalLength / safeStep));
            int verticalDiv = Mathf.Max(1, Mathf.CeilToInt(verticalLength / safeStep));

            // BuildLaneParameter
            if (laneIndex == 2)
            {
                Debug.Log("");
            }
            ConstructorPrime(laneIndex, lane.DuringBentEndPoint.y);

            int startIndex = includeStart ? 0 : 1;
            for (int i = startIndex+1; i <= horizontalDiv; i++)
            {
                float t = i / (float)horizontalDiv;
                points.Add(Vector3.Lerp(startPoint, cornerPoint, t));
            }

            int frontBend = Mathf.CeilToInt(verticalDiv / 2);
            int getToHalfCount = 2;

            for (getToHalfCount = 1; getToHalfCount < verticalDiv; getToHalfCount++)
            {
                float t = getToHalfCount / (float)verticalDiv;
                points.Add(Vector3.Lerp(cornerPoint, endPoint, t));
                if (laneIndex == 0)
                {
                }
                else if (laneIndex == 1)
                {
                    Debug.Log("");
                }
                if (laneIndex == 2)
                {
                    var cornerZ = new Vector3(cornerPoint.x, cornerPoint.y, cornerPoint.z + OneSquareWidth/2);
                    var DuringBentEndPoint = new Vector3(endPoint.x, endPoint.y, endPoint.z+OneSquareWidth/2);
                    points.Add(Vector3.Lerp(cornerZ, DuringBentEndPoint, t));
                }
            }


            Vector3 downAngle = new Vector3(0.5f, 0, 0);

           

            Vector3 dir = cornerPoint - endPoint;
            lane.altEndPoint = cornerPoint;

            // lane.baseDir = dir.sqrMagnitude > 0.00001f ? dir.normalized : Vector3.forward;
            lane.baseDir = Vector3.down;
            lane.bentDir = (Quaternion.AngleAxis(45, Vector3.forward) * lane.baseDir).normalized;
            var temporary = BuildBentTarget2(lane.bentDir, endPoint.x, endPoint.z);
            Vector3 halfPoint = Vector3.Lerp(cornerPoint, endPoint, 0.5f);
            //float totalDistance = Vector3.Distance(halfPoint, lane.DuringBentEndPoint);

            Vector3 dir2 = (temporary - halfPoint).normalized;
            
            Vector3 wHalfPoint = halfPoint + dir2 * 1.991f;
            Vector3 w2HalfPoint = halfPoint + dir2 * 1.991f * 2;
            //Vector3 w2HalfPoint=Vector3.Lerp()
           for (int endPointIndex = getToHalfCount; endPointIndex <= verticalDiv; endPointIndex++)
            {
                float t = endPointIndex / (float)verticalDiv;
                /*if (laneIndex == 2)
                {
                    var cornerZ = new Vector3(cornerPoint.x, cornerPoint.y, cornerPoint.z + OneSquareWidth);
                    var DuringBentEndPoint = new Vector3(lane.DuringBentEndPoint.x, lane.DuringBentEndPoint.y, lane.DuringBentEndPoint.z+OneSquareWidth);
                    points.Add(Vector3.Lerp(cornerZ, DuringBentEndPoint, t));
                }*/

                if (laneIndex == 0)
                {
                    
                    points.Add(wHalfPoint);
                    var recive = Vector3.Lerp(endPoint, lane.DuringBentEndPoint, t);
                    points.Add(w2HalfPoint);
                    //points.

                } else if (laneIndex == 1) {
                    points.Add(endPoint);
                    
                    var DuringBentEndPoint = new Vector3(points[points.Count - 1].x, points[points.Count - 1].y, points[points.Count - 1].z+ OneSquareWidth/4);
                    lane.DuringBentEndPoint = BuildBentTarget2(lane.bentDir, DuringBentEndPoint.x, DuringBentEndPoint.z);
                    var inDir = ( lane.DuringBentEndPoint-endPoint).normalized;
                    halfPoint = endPoint + inDir * 1.991f;
                    wHalfPoint = endPoint + inDir * 1.991f * 2;
                    
                    points.Add(halfPoint);
                    points.Add(wHalfPoint);

                    // lane.DuringBentEndPoint = BuildBentTarget2(lane.bentDir, DuringBentEndPoint.x, DuringBentEndPoint.z);
                    //points.Add(lane.DuringBentEndPoint);
                    // points.Add(lane.DuringBentEndPoint);
                    /*var norm = points[points.Count - 1] - points[points.Count - 2];
                    var dis = (Vector3.Distance(points[points.Count - 1], points[points.Count - 2])) / 2;
                    points.Add(lane.DuringBentEndPoint+dis*norm);*/
                }
                else
                {
                    
                    points.Add(new Vector3(wHalfPoint.x, wHalfPoint.y, wHalfPoint.z + rightDistance));
                    var DuringBentEndPoint = new Vector3(lane.DuringBentEndPoint.x, lane.DuringBentEndPoint.y, lane.DuringBentEndPoint.z + rightDistance);
                    points.Add(new Vector3(w2HalfPoint.x, w2HalfPoint.y, w2HalfPoint.z + rightDistance));

                   // points.Add(DuringBentEndPoint); 
                }
            }
           
            Debug.Log("");
        }

        lane.DuringBentEndPoint = BuildBentTarget(lane.bentDir, target, primeEnd.z);

        // ストレート行くと下だけ1/6足らなくなるけど、曲がると真ん中だけたるようになる
        // primeEnd = GetLaneBaseEndPoint(laneIndex);

        /*
        lane.firstLength = Vector3.Distance(lane.altStartPoint, lane.breakPoint);
        lane.secondLength = Vector3.Distance(lane.breakPoint, lane.DuringBentEndPoint);
        */

        return lane;
    }

    private Vector3 BuildBentTarget(Vector3 bentDir, float targetX, float targetZ)
    {
        float safeX = Mathf.Abs(bentDir.x) < 0.0001f ? 0.0001f : bentDir.x;
        float travel = Mathf.Abs((targetX - primeEnd.x) / safeX);
        return new Vector3(targetX, primeEnd.y + bentDir.y * travel, targetZ);
    }
    Vector3 ShortenSegment(Vector3 start, Vector3 end, float rate)
    {
        return Vector3.Lerp(start, end, Mathf.Clamp01(rate));
    }
    private Vector3 BuildBentTarget2
        (Vector3 bentDir, float targetX, float targetZ)
    {
        float safeZ = Mathf.Abs(bentDir.x) < 0.0001f ? 0.0001f : bentDir.x;
        float travel = Mathf.Abs((targetZ - primeEnd.z) / safeZ);
        return new Vector3(targetX, primeEnd.y + bentDir.y * travel, targetZ);
    }

    public void ExtendSplineWithKnots()
    {
        EnsureSplineCount(3);
        ClearAllSplines();

        LaneParameter[] lanes = new LaneParameter[3];

        if (lanes[1].altEndPoint == Vector3.zero)
        {
            planeNeighbor.Clear();
            splineActive.Clear();
        }

        List<Vector3> points = new List<Vector3>();
        for (int laneIndex = 0; laneIndex < 3; laneIndex++)
        {
            ConstructorPrime(laneIndex, lanes[laneIndex].DuringBentEndPoint.y);
            lanes[laneIndex] = BuildLaneParameter(laneIndex, lanes[laneIndex], points);
        }

        Vector3 upperDir = lanes[1].DuringBentEndPoint - primeEnd;
        upperDir = Vector3.ProjectOnPlane(upperDir, Vector3.up);

        if (upperDir.sqrMagnitude < 0.00001f) upperDir = Vector3.right;
        else upperDir.Normalize();

        Vector3 side = Vector3.Cross(Vector3.up, upperDir).normalized;
        if (!curveToPositiveZ) side = -side;

        Vector3 commonCurveCenter = lanes[1].DuringBentEndPoint + side * topPlaneCenterRadius;
        Vector3? nextSquareAnchor = null;

        for (int laneIndex = 0; laneIndex < 3; laneIndex++)
        {
            Spline spline = splineContainer.Splines[laneIndex];
            spline.Clear();
            spline.Closed = false;

            points = BuildLanePoints(lanes[laneIndex], commonCurveCenter, laneIndex);
            for (int i = 0; i < points.Count; i++) spline.Add(new BezierKnot(points[i]));

            if (laneIndex == 1 && points.Count > 0) nextSquareAnchor = points[points.Count - 1];

            // UpdateEntrySpline(laneIndex, lanes[laneIndex], points);
            planeNeighbor.Add(spline);

            if (spline.Count > 7) SevenLine[laneIndex] = spline[7];
        }

        if (!jurge && nextSquareAnchor.HasValue) SetNextSquareAnchor(nextSquareAnchor.Value);
    }

    void ConstructorPrime(int index, float height)
    {
        primeStart = GetLaneStartPoint(index, height);
        primeEnd = GetLaneBaseEndPoint(index, height);
    }

    private List<Vector3> BuildLanePoints(LaneParameter lane, Vector3 commonCurveCenter, int index)
    {
        List<Vector3> points = new List<Vector3>();
        float step = Mathf.Max(0.01f, knotStepDistance);
        ConstructorPrime(index, 0.075f);

        // 上を動的なDuringBentEndPointに変えてしまうとエラーになる。しかもコメントアウトする機能しなくなる。バグになりやすい部分です。

        // 1平面目
        AddStraightByDistance(points, primeStart, primeEnd, step, true);

        // 2平面目
        AddStraightByDistance(points, primeEnd, lane.DuringBentEndPoint, step, false);

        // 3平面目と
        // 下を通らなくてもDuringBentEndPointは正しい境界にあります
        includeStart = true;
        BuildLaneParameter(index, lane, points);
        includeStart = false;
        BuildLaneParameter(index, lane, points);


        /*(Vector3 value1, Vector3 value2) = AddArcByDistance(points, commonCurveCenter, lane.DuringBentEndPoint, topPlaneCurveAngle, step, false, index, lane);

        index++;
        LaneParameter[] lanes = new LaneParameter[3];
        currentFrame.startBase = value1;
        currentFrame.endBase = value2;
        toggled();

        Vector3 start = GetLaneStartPoint(index);
        Vector3 end = GetLaneBaseEndPoint(index);*/

        ResolveSceneReferences();

        // AddStraightByDistance(points, lane.breakPoint, lane.DuringBentEndPoint, step, false);
        return points;
    }

    /*
    public Vector3 OnTheBendOneSquare(Vector3 start)
    {

    }
    */

    private void AddStraightByDistance(List<Vector3> points, Vector3 from, Vector3 to, float step, bool includeStart)
    {
        float length = Vector3.Distance(from, to);
        if (length <= 0.00001f) return;

        int divisionCount = Mathf.Max(1, Mathf.CeilToInt(length / step));
        int startIndex = includeStart ? 0 : 1;

        for (int i = startIndex; i <= divisionCount; i++)
        {
            float t = i / (float)divisionCount;
            points.Add(Vector3.Lerp(from, to, t));
        }
    }

    (Vector3, Vector3) AddArcByDistance(
        List<Vector3> points,
        Vector3 center,
        Vector3 startPoint,
        float angleDeg,
        float step,
        bool includeStart,
        int index,
        LaneParameter lane
    )
    {
        float safeStep = Mathf.Max(0.01f, step);
        int laneType = index;

        Debug.Log("");

        float rightDistance;
        float forwardDistance;

        switch (laneType)
        {
            case 0:
                forwardDistance = OneSquareWidth / 6;
                rightDistance = OneSquareLength / 2;
                break;
            case 1:
                forwardDistance = OneSquareWidth * 0.5f;
                rightDistance = OneSquareLength * 0.5f;
                break;
            default:
                forwardDistance = OneSquareWidth * 5 / 6;
                rightDistance = OneSquareLength * 1 / 2;
                break;
        }

        Vector3 cornerPoint = new Vector3(startPoint.x + forwardDistance, startPoint.y, startPoint.z);
        Vector3 endPoint = new Vector3(cornerPoint.x, startPoint.y, startPoint.z + rightDistance);

        float horizontalLength = Vector3.Distance(startPoint, cornerPoint);
        float verticalLength = Vector3.Distance(cornerPoint, endPoint);

        int horizontalDiv = Mathf.Max(1, Mathf.CeilToInt(horizontalLength / safeStep));
        int verticalDiv = Mathf.Max(1, Mathf.CeilToInt(verticalLength / safeStep));

        int startIndex = includeStart ? 0 : 1;

        for (int i = startIndex; i <= horizontalDiv; i++)
        {
            float t = i / (float)horizontalDiv;
            points.Add(Vector3.Lerp(startPoint, cornerPoint, t));
        }

        int frontBend = Mathf.CeilToInt(verticalDiv / 2);
        int getToHalfCount = 2;

        for (getToHalfCount = 1; getToHalfCount <= verticalDiv; getToHalfCount++)
        {
            float t = getToHalfCount / (float)verticalDiv;

            if (getToHalfCount > frontBend)
            {
                Vector3 playback = points[points.Count - 1];
                Vector3 delicateRecipe = Vector3.Lerp(cornerPoint, endPoint, t);
                float dis = Vector3.Distance(playback, delicateRecipe) / 2;
                Vector3 norm = (delicateRecipe - playback).normalized;

                Vector3 knotIn = playback + dis * norm;
                points.Add(knotIn);
                break;
            }

            points.Add(Vector3.Lerp(cornerPoint, endPoint, t));
        }

        Vector3 downAngle = new Vector3(0.5f, 0, 0);

        for (int endPointIndex = getToHalfCount; endPointIndex <= verticalDiv; endPointIndex++)
        {
            float t = endPointIndex / (float)verticalDiv;
            points.Add(Vector3.Lerp(cornerPoint, endPoint, t));
        }

        Vector3 dir = cornerPoint - endPoint;
        // lane.altEndPoint = cornerPoint;

        // lane.baseDir = dir.sqrMagnitude > 0.00001f ? dir.normalized : Vector3.forward;
        lane.baseDir = Vector3.down;
        lane.bentDir = (Quaternion.AngleAxis(45, Vector3.forward) * lane.baseDir).normalized;
        lane.DuringBentEndPoint = BuildBentTarget2(lane.bentDir, endPoint.x, endPoint.z);

        return (cornerPoint, endPoint);
    }

    private void UpdateEntrySpline(int laneIndex, LaneParameter lane, List<Vector3> points)
    {
        EntrySpline[laneIndex].startPoint = lane.altStartPoint;
        EntrySpline[laneIndex].endPoint = points[points.Count - 1];
        EntrySpline[laneIndex].breakpoint = lane.altEndPoint;
        EntrySpline[laneIndex].bentDir = lane.bentDir;

        if (points.Count >= 2) EntrySpline[laneIndex].DirLine = (points[points.Count - 1] - points[points.Count - 2]).normalized;
        else EntrySpline[laneIndex].DirLine = Vector3.zero;

        float total = 0f;
        for (int i = 1; i < points.Count; i++) total += Vector3.Distance(points[i - 1], points[i]);

        LinapLine1[laneIndex] = total;
        LinapLine2[laneIndex] = 0f;
    }

    private void EnsureSplineCount(int requiredCount)
    {
        if (splineContainer.Splines.Count == 0) SplineUtility.AddSpline(splineContainer);
        while (splineContainer.Splines.Count < requiredCount) SplineUtility.AddSpline(splineContainer);
    }

    private void ClearAllSplines()
    {
        for (int i = 0; i < splineContainer.Splines.Count; i++)
        {
            splineContainer.Splines[i].Clear();
            splineContainer.Splines[i].Closed = false;
        }
    }

    private IEnumerator Delay(float time)
    {
        yield return new WaitForSeconds(time);

        GameObject subject = GameObject.Find("subject");
        if (subject == null) subject = GameObject.Find("Subject");
        if (subject == null) yield break;

        SplineAnimate anim = subject.GetComponent<SplineAnimate>();
        if (anim == null) anim = subject.AddComponent<SplineAnimate>();

        anim.PlayOnAwake = false;
        anim.Container = splineContainer;
        anim.Duration = 64f;
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawDebugGizmos || splineContainer == null) return;

        Gizmos.color = Color.red;

        for (int i = 0; i < splineContainer.Splines.Count; i++)
        {
            Spline s = splineContainer.Splines[i];
            if (s == null || s.Count <= 7) continue;

            Vector3 k7 = transform.TransformPoint((Vector3)s[7].Position);
            Gizmos.DrawSphere(k7, 0.08f);
        }
    }
}