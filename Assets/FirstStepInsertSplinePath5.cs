using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;

public class CoreStepInsertSplinePathNatural5 : MonoBehaviour
{
    [SerializeField] Transform stairPlane;
    [SerializeField] SplineContainer splineBox;
    [SerializeField] float pathWidth = 3f, endPadding = 1f, edgeStepOverride, knotStep = 2f;
    [SerializeField, Range(1f, 89f)] float bendDegrees = 45f;
    [SerializeField] float laneGapOverride, miterLimit = 4f, bendZOffset, falseFlatScale = 1f;
    [SerializeField] bool turnToPositiveZ = true, regularizeTurn = true, verticalFalsePair = true;
    [SerializeField] int extraPointSearch = 2;

    readonly bool[] startPattern = { false, true, true, true, false, false, true, false, true, true, true };

    public Spline accumulatedSpline;
    public Vector3 stepHandlePoint;
    public bool firstSpiderContract, missleadRightCorner;

    const int LaneCount = 3, CenterLane = 1;
    const float LaneHeight = .075f, Epsilon = .00001f;

    bool lastWasAppend;
    Vector3 currentDir = Vector3.right;
    Vector2 planeSize;

    float PlaneWidth => planeSize.x;
    float PlaneLength => planeSize.y;
    float StepLength => edgeStepOverride > 0 ? edgeStepOverride : pathWidth > 0 ? pathWidth : PlaneWidth * .25f;
    float LaneGap => laneGapOverride > 0 ? laneGapOverride : PlaneLength * .25f;
    float FalseFlat => Mathf.Max(Epsilon, PlaneWidth * Mathf.Max(0, falseFlatScale));

    void Start() => RebuildSpline();

    [ContextMenu("RebuildSpline")]
    public void RebuildSpline()
    {
        if (!Prepare()) return;
        ClearSplines();

        for (int stepIndex = 0; stepIndex < startPattern.Length; stepIndex++)
        {
            bool isFalse = !startPattern[stepIndex];
            bool hasFalsePair = isFalse && ((stepIndex > 0 && !startPattern[stepIndex - 1]) || (stepIndex + 1 < startPattern.Length && !startPattern[stepIndex + 1]));
            if (!(stepIndex > 0 && isFalse && !hasFalsePair)) BuildStep(startPattern[stepIndex], verticalFalsePair && hasFalsePair);
        }
    }

    [ContextMenu("ExtendSplineWithKnots")]
    public void ExtendSplineWithKnots()
    {
        if (!Prepare()) return;
        if (accumulatedSpline == null || !HasAnyKnot()) RebuildSpline(); else BuildStep(lastWasAppend, false);
    }

    bool Prepare()
    {
        if (!stairPlane)
        {
            var stairway = GameObject.Find("StairwaySimple");
            stairPlane = stairway ? stairway.transform.Find("Plane") : null;
        }
        if (!splineBox) splineBox = GetComponent<SplineContainer>();
        if (!stairPlane || !splineBox) return false;

        Bounds planeBounds;
        if (stairPlane.TryGetComponent(out MeshCollider mesh)) planeBounds = mesh.bounds;
        else if (stairPlane.TryGetComponent(out Renderer render)) planeBounds = render.bounds;
        else return false;

        planeSize = new Vector2(planeBounds.size.x, planeBounds.size.z);
        while (splineBox.Splines.Count < LaneCount) SplineUtility.AddSpline(splineBox);
        return true;
    }

    void ClearSplines()
    {
        lastWasAppend = firstSpiderContract = false;
        currentDir = Vector3.right;
        accumulatedSpline = null;
        foreach (var spline in splineBox.Splines) { spline.Clear(); spline.Closed = false; }
    }

    bool HasAnyKnot()
    {
        foreach (var spline in splineBox.Splines) if (spline.Count > 0) return true;
        return false;
    }

    void BuildStep(bool appendStep, bool verticalOnly)
    {
        lastWasAppend = appendStep;
        var centerSpline = splineBox.Splines[CenterLane];
        EmitLaneKnots(MakeCenterLine(centerSpline, appendStep, verticalOnly), Mathf.Max(.01f, knotStep), centerSpline.Count > 0);
        accumulatedSpline = centerSpline;
    }

    List<Vector3> MakeCenterLine(Spline centerSpline, bool appendStep, bool verticalOnly)
    {
        bool hasCenter = centerSpline.Count > 0, canAppend = appendStep && hasCenter;
        Vector3 forwardDir = UnitXZ(currentDir, Vector3.right);
        Vector3 startPoint = hasCenter ? LastPoint(centerSpline) : Vector3.up * LaneHeight - forwardDir * PlaneWidth * .5f;
        float runLength = StepLength * 2f + endPadding;
        Vector3 dropStep = Vector3.down * Mathf.Tan(bendDegrees * Mathf.Deg2Rad) * runLength;
        Vector3 edgePoint = startPoint + forwardDir * (verticalOnly ? FalseFlat : canAppend ? PlaneWidth * .5f : PlaneWidth);
        stepHandlePoint = edgePoint;

        var centerLine = new List<Vector3> { startPoint, edgePoint };
        if (verticalOnly)
        {
            currentDir = forwardDir;
            centerLine.Add(edgePoint + forwardDir * runLength + dropStep);
            return centerLine;
        }

        Vector3 turnBase = edgePoint;
        currentDir = SideDir(forwardDir);
        if (!canAppend)
        {
            centerLine.Add(edgePoint + forwardDir * runLength + dropStep);
            centerLine.Add(centerLine[centerLine.Count - 1] + forwardDir * PlaneWidth * .5f);
            turnBase = centerLine[centerLine.Count - 1];
        }
        centerLine.Add(TurnPoint(turnBase, currentDir, canAppend));
        centerLine.Add(centerLine[centerLine.Count - 1] + currentDir * runLength + dropStep);
        return centerLine;
    }

    Vector3 TurnPoint(Vector3 point, Vector3 axisDir, bool shortTurn)
    {
        axisDir = UnitXZ(axisDir, Vector3.forward);
        return !regularizeTurn || shortTurn
            ? point + axisDir * PlaneWidth * .5f
            : point + axisDir * (PlaneLength * .5f + bendZOffset - Vector3.Dot(point, axisDir));
    }

    void EmitLaneKnots(IReadOnlyList<Vector3> centerLine, float step, bool skipFirst)
    {
        if (centerLine == null || centerLine.Count < 2) return;
        var laneLines = new List<Vector3>[LaneCount];

        for (int laneIndex = 0; laneIndex < LaneCount; laneIndex++)
        {
            laneLines[laneIndex] = new List<Vector3>(centerLine.Count);
            float laneOffset = (CenterLane - laneIndex) * LaneGap;
            for (int pointIndex = 0; pointIndex < centerLine.Count; pointIndex++)
                laneLines[laneIndex].Add(LaneCorner(centerLine, pointIndex, laneOffset));
        }

        for (int segmentIndex = 0; segmentIndex < centerLine.Count - 1; segmentIndex++)
        {
            float segmentLength = Vector3.Distance(centerLine[segmentIndex], centerLine[segmentIndex + 1]);
            if (segmentLength <= Epsilon) continue;
            if (segmentIndex == 0 && !skipFirst) AddLanePoint(laneLines, segmentIndex);
            foreach (float ratio in PointRatios(segmentLength, step)) AddLanePoint(laneLines, segmentIndex, ratio);
            AddLanePoint(laneLines, segmentIndex + 1);
        }
    }

    Vector3 LaneCorner(IReadOnlyList<Vector3> line, int pointIndex, float laneOffset)
    {
        Vector3 centerPoint = line[pointIndex];
        if (Mathf.Abs(laneOffset) <= Epsilon) return centerPoint;

        bool hasPrev = TryLineDir(line, pointIndex, -1, out Vector3 prevDir);
        bool hasNext = TryLineDir(line, pointIndex, 1, out Vector3 nextDir);
        if (!hasPrev && !hasNext) return centerPoint;
        if (!hasPrev || !hasNext) return centerPoint + SideDir(hasPrev ? prevDir : nextDir, laneOffset);

        Vector3 prevSide = SideDir(prevDir, laneOffset), nextSide = SideDir(nextDir, laneOffset);
        if (TryLineHit(centerPoint + prevSide, prevDir, centerPoint + nextSide, nextDir, centerPoint.y, out Vector3 hitPoint))
        {
            float miterMax = Mathf.Max(LaneGap, Mathf.Abs(laneOffset)) * Mathf.Max(1, miterLimit);
            if ((hitPoint - centerPoint).sqrMagnitude <= miterMax * miterMax) return hitPoint;
        }
        return centerPoint + UnitXZ(prevSide + nextSide, nextSide) * Mathf.Abs(laneOffset);
    }

    bool TryLineDir(IReadOnlyList<Vector3> line, int pointIndex, int searchDir, out Vector3 lineDir)
    {
        for (int segmentIndex = searchDir < 0 ? pointIndex - 1 : pointIndex; segmentIndex >= 0 && segmentIndex < line.Count - 1; segmentIndex += searchDir < 0 ? -1 : 1)
            if (((lineDir = line[segmentIndex + 1] - line[segmentIndex]).x * lineDir.x + lineDir.z * lineDir.z) > Epsilon * Epsilon)
            {
                lineDir = UnitXZ(lineDir, Vector3.right);
                return true;
            }

        lineDir = Vector3.right;
        return false;
    }

    bool TryLineHit(Vector3 pointA, Vector3 dirA, Vector3 pointB, Vector3 dirB, float y, out Vector3 hitPoint)
    {
        dirA = UnitXZ(dirA, Vector3.right);
        dirB = UnitXZ(dirB, Vector3.forward);
        float cross = dirA.x * dirB.z - dirA.z * dirB.x;
        if (Mathf.Abs(cross) <= Epsilon) { hitPoint = pointA; return false; }
        hitPoint = pointA + dirA * (((pointB.x - pointA.x) * dirB.z - (pointB.z - pointA.z) * dirB.x) / cross);
        hitPoint.y = y;
        return true;
    }

    void AddLanePoint(IReadOnlyList<Vector3>[] laneLines, int pointIndex, float ratio = -1)
    {
        for (int laneIndex = 0; laneIndex < LaneCount; laneIndex++)
        {
            Vector3 knotPoint = ratio < 0 ? laneLines[laneIndex][pointIndex] : Vector3.Lerp(laneLines[laneIndex][pointIndex], laneLines[laneIndex][pointIndex + 1], ratio);
            var laneSpline = splineBox.Splines[laneIndex];
            if (laneSpline.Count == 0 || (knotPoint - LastPoint(laneSpline)).sqrMagnitude > Epsilon * Epsilon)
                laneSpline.Add(new BezierKnot(new float3(knotPoint.x, knotPoint.y, knotPoint.z)));
        }
    }

    IEnumerable<float> PointRatios(float segmentLength, float step)
    {
        int bestCount = 0, searchMax = Mathf.Max(1, Mathf.CeilToInt(segmentLength / step) + Mathf.Max(0, extraPointSearch));
        float bestGap = 0, bestCost = (segmentLength - step) * (segmentLength - step);

        for (int count = 1; count <= searchMax; count++)
        {
            float gap = (segmentLength - (count - 1) * step) * .5f;
            float cost = 2f * (gap - step) * (gap - step);
            if (gap > 0 && (cost < bestCost - Epsilon || Mathf.Abs(cost - bestCost) <= Epsilon && count > bestCount))
            {
                bestCount = count;
                bestGap = gap;
                bestCost = cost;
            }
        }

        for (int pointIndex = 0; pointIndex < bestCount; pointIndex++)
        {
            float distance = bestGap + step * pointIndex;
            if (distance > Epsilon && distance < segmentLength - Epsilon) yield return distance / segmentLength;
        }
    }

    Vector3 SideDir(Vector3 dir, float scale = 1)
    {
        dir = UnitXZ(new Vector3(-dir.z, 0, dir.x), Vector3.forward) * scale;
        return turnToPositiveZ ? dir : -dir;
    }

    Vector3 UnitXZ(Vector3 dir, Vector3 fallback)
    {
        float sqrLength = dir.x * dir.x + dir.z * dir.z;
        if (sqrLength <= Epsilon * Epsilon)
        {
            dir = fallback;
            sqrLength = dir.x * dir.x + dir.z * dir.z;
        }
        return new Vector3(dir.x, 0, dir.z) / Mathf.Sqrt(sqrLength);
    }

    Vector3 LastPoint(Spline spline)
    {
        float3 point = spline[spline.Count - 1].Position;
        return new Vector3(point.x, point.y, point.z);
    }
}
