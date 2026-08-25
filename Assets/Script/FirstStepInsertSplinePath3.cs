using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;

public class CoreStepInsertSplinePathNatural3 : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Transform stairPlane;
    [SerializeField] private SplineContainer splineContainer;

    [Header("Lane Layout")]
    [SerializeField] private float justifyWidth = 3f;
    [SerializeField] private float straightEndPadding = 1f;
    [SerializeField] private float centerEdgeStepOverride = 0f;

    [Header("Spline Shape")]
    [SerializeField, Range(1f, 89f)] private float bendAngle = 45f;
    [SerializeField] private float knotStepDistance = 2f;
    [SerializeField] private bool curveToPositiveZ = true;

    [Tooltip("現在の調整では、p4 -> p5 は p1 -> p2 と同じ距離にそろえるため、この値は使いません。")]
    [SerializeField] private float terminalDiagonalLength = 0f;

    [SerializeField] private bool useRegularizedTurnLayout = true;
    [SerializeField] private float bendStartZOffset = 0f;
    [SerializeField] private bool preserveOriginalLaneForm = true;
    [SerializeField] private int searchExtraInteriorCount = 2;

    public Spline AccumerateInBounding;

    private const int LaneCount = 3;
    private const float LaneHeight = 0.075f;
    private const float Epsilon = 0.00001f;

    private Vector3 currentRotate = new Vector3(1f, 0f, 0f);
    public Vector3 StepHandleRotate;

    private readonly Vector2 toggle = new Vector2(0f, 1f);

    private static readonly float[] ForwardRatios =
    {
        1f / 6f,
        0.5f,
        5f / 6f
    };

    private static readonly bool[] HalfPointPatterns =
    {
        true,
        false,
        true
    };

    private float oneSquareLength;
    private float oneSquareWidth;
    private readonly float[] laneBaseOffsets = new float[LaneCount];

    private void Start()
    {
        ExtendSplineWithKnots();
        ExtendSplineWithKnots();
    }

    [ContextMenu("ExtendSplineWithKnots")]
    public void ExtendSplineWithKnots()
    {
        if (!ResolveSceneReferences())
        {
            return;
        }

        EnsureSplineCount(LaneCount);

        if (AccumerateInBounding == null)
        {
            ClearAllSplines();
        }
        else
        {
            Debug.Log("");
        }

        float step = Mathf.Max(0.01f, knotStepDistance);
        Spline spline = null;

        for (int laneIndex = 0; laneIndex < LaneCount; laneIndex++)
        {
            spline = splineContainer.Splines[laneIndex];

            bool isContinuation;

            List<Vector3> corners = BuildLaneCorners(
                laneIndex,
                spline,
                out isContinuation
            );

            List<Vector3> points = BuildPointsFromPolyline(
                corners,
                step,
                isContinuation
            );

            foreach (Vector3 point in points)
            {
                spline.Add(new BezierKnot(point));
            }
        }

        AccumerateInBounding = spline;
        Debug.Log("");
    }

    private bool ResolveSceneReferences()
    {
        if (stairPlane == null)
        {
            GameObject stairway = GameObject.Find("StairwaySimple");
            stairPlane = stairway != null ? stairway.transform.Find("Plane") : null;
        }

        if (splineContainer == null)
        {
            splineContainer = GetComponent<SplineContainer>();
        }

        if (stairPlane == null || splineContainer == null)
        {
            return false;
        }

        if (!TryGetPlaneSize(out oneSquareWidth, out oneSquareLength))
        {
            return false;
        }

        for (int i = 0; i < LaneCount; i++)
        {
            laneBaseOffsets[i] = i * oneSquareLength * 0.25f;
        }

        return true;
    }

    private bool TryGetPlaneSize(out float width, out float length)
    {
        MeshCollider meshCollider = stairPlane.GetComponent<MeshCollider>();

        if (meshCollider != null)
        {
            width = meshCollider.bounds.size.x;
            length = meshCollider.bounds.size.z;
            return true;
        }

        Renderer renderer = stairPlane.GetComponent<Renderer>();

        if (renderer != null)
        {
            width = renderer.bounds.size.x;
            length = renderer.bounds.size.z;
            return true;
        }

        width = 0f;
        length = 0f;
        return false;
    }

    private Vector3 GetLanePoint(int laneIndex, Vector3 lastKnots, float x)
    {
        if (lastKnots != Vector3.zero)
        {
            return new Vector3(
                lastKnots.x,
                lastKnots.y,
                oneSquareLength * 0.25f - lastKnots.z
            );
        }

        return new Vector3(
            x,
            LaneHeight,
            oneSquareLength * 0.25f - laneBaseOffsets[laneIndex]
        );
    }

    private float GetLaneStep()
    {
        if (centerEdgeStepOverride > 0f)
        {
            return centerEdgeStepOverride;
        }

        if (justifyWidth > 0f)
        {
            return justifyWidth;
        }

        return oneSquareWidth * 0.25f;
    }

    private Vector3 GetZAxis()
    {
        return curveToPositiveZ ? Vector3.forward : Vector3.back;
    }

    private float GetCommonBendStartZ()
    {
        float sign = curveToPositiveZ ? 1f : -1f;
        return sign * (oneSquareLength * 0.5f + bendStartZOffset);
    }

    private Vector3 GetDiagonalOrigin(Vector3 turnExit, int laneIndex)
    {
        if (useRegularizedTurnLayout)
        {
            return new Vector3(
                turnExit.x,
                turnExit.y,
                GetCommonBendStartZ()
            );
        }

        Vector3 bendBase = turnExit + GetZAxis() * (oneSquareWidth * 0.5f);

        if (preserveOriginalLaneForm && HalfPointPatterns[laneIndex])
        {
            return Vector3.Lerp(turnExit, bendBase, 0.5f);
        }

        return bendBase;
    }

    private List<Vector3> BuildLaneCorners(
        int laneIndex,
        Spline inSpline,
        out bool isContinuation
    )
    {
        List<BezierKnot> list = inSpline.Knots.ToList();
        isContinuation = list.Count > 0;

        Vector3 start;

        if (!isContinuation)
        {
            start = GetLanePoint(laneIndex, Vector3.zero, -oneSquareWidth * 0.5f);
            StepHandleRotate = currentRotate * oneSquareWidth;
        }
        else
        {
            if (laneIndex == 0)
            {
                toggled();
            }

            start = list[list.Count - 1].Position;

            // 以前はここで start から半マス進めていました。
            // StepHandleRotate = start
            //     + currentRotate.normalized * oneSquareWidth / 2f
            //     + Vector3.down * oneSquareWidth / 2f;
            //
            // その start -> p1 が Knot19〜22 のような余分な継ぎ目になっていたため、
            // 継続時は p1 を start と同じ位置にします。
            StepHandleRotate = start;

            Debug.Log("");
        }

        float angleRad = bendAngle * Mathf.Deg2Rad;

        // p1 -> p2 の斜め距離。
        // justifyWidth = 3, straightEndPadding = 1, bendAngle = 45 の場合:
        // bendRunX = 7
        // bendDropY = 7
        // 距離 = sqrt(7^2 + 7^2) = 約 9.899
        float bendRunX = GetLaneStep() * 2f + straightEndPadding;
        float bendDropY = Mathf.Tan(angleRad) * bendRunX;

        float forwardDistance;

        if (!isContinuation)
        {
            forwardDistance = oneSquareWidth * ForwardRatios[laneIndex];
            Debug.Log("");
        }
        else
        {
            forwardDistance = oneSquareWidth / 2f * ForwardRatios[laneIndex];
            Debug.Log("");
        }

        Vector3 zAxis = GetZAxis();

        Vector3 p1;

        if (!isContinuation)
        {
            p1 = start + currentRotate * oneSquareWidth;
        }
        else
        {
            // ここが今回の調整点。
            // 継続時の start -> p1 区間を作らない。
            p1 = start;
        }

        // 前半の45度斜め下降。
        // Knot5 -> Knot10 相当。
        Vector3 p2 = p1 + new Vector3(
            bendRunX,
            -bendDropY,
            0f
        );

        // 折れ曲がり前の短い直線。
        Vector3 p3 = p2 + Vector3.right * forwardDistance;

        // 横45度へ入るための開始点。
        // p5を正しく横方向へ沿わせるため、このp4は残します。
        Vector3 p4 = GetDiagonalOrigin(p3, laneIndex);

        // 後半の45度斜め下降。
        // p5は残す。
        // ただし旧コードの terminalAxis / terminalDropY ではなく、
        // p1 -> p2 と同じ bendRunX / bendDropY を使います。
        Vector3 p5 = p4
            + zAxis * bendRunX
            + Vector3.down * bendDropY;

        return new List<Vector3>
        {
            start,
            p1,
            p2,
            p3,
            p4,
            p5
        };
    }

    private IEnumerable<float> ChooseBestSymmetricPlan(float length, float targetStep)
    {
        if (length <= Epsilon)
        {
            yield break;
        }

        int bestInteriorCount = 0;
        float bestBoundaryMargin = 0f;
        float bestCost = (length - targetStep) * (length - targetStep);

        int maxCandidate = Mathf.Max(
            1,
            Mathf.CeilToInt(length / targetStep) + Mathf.Max(0, searchExtraInteriorCount)
        );

        for (int n = 1; n <= maxCandidate; n++)
        {
            float margin = (length - (n - 1) * targetStep) * 0.5f;

            if (margin <= 0f)
            {
                continue;
            }

            float cost = 2f * (margin - targetStep) * (margin - targetStep);

            bool isBetter =
                cost < bestCost - 0.000001f ||
                Mathf.Abs(cost - bestCost) <= 0.000001f && n > bestInteriorCount;

            if (!isBetter)
            {
                continue;
            }

            bestInteriorCount = n;
            bestBoundaryMargin = margin;
            bestCost = cost;
        }

        for (int k = 0; k < bestInteriorCount; k++)
        {
            float dist = bestBoundaryMargin + targetStep * k;

            if (dist > Epsilon && dist < length - Epsilon)
            {
                yield return dist;
            }
        }
    }

    private List<Vector3> BuildPointsFromPolyline(
        IReadOnlyList<Vector3> corners,
        float targetStep,
        bool skipFirstPoint
    )
    {
        List<Vector3> points = new List<Vector3>();

        if (corners == null || corners.Count == 0)
        {
            return points;
        }

        // 継続時は、corners[0] が前回最後の knot と同じです。
        // ここで追加すると、同じ場所の knot が重なります。
        if (!skipFirstPoint)
        {
            points.Add(corners[0]);
        }

        for (int i = 0; i < corners.Count - 1; i++)
        {
            Vector3 start = corners[i];
            Vector3 end = corners[i + 1];

            float length = Vector3.Distance(start, end);

            if (length <= Epsilon)
            {
                continue;
            }

            Vector3 dir = (end - start).normalized;

            foreach (float dist in ChooseBestSymmetricPlan(length, targetStep))
            {
                points.Add(start + dir * dist);
            }

            points.Add(end);
        }

        return points;
    }

    private Vector3 toggled()
    {
        Debug.Log(currentRotate);

        currentRotate = new Vector3(
            currentRotate.x * toggle.x + currentRotate.z * toggle.y,
            0f,
            currentRotate.x * toggle.y + currentRotate.z * toggle.x
        );

        return currentRotate;
    }

    private void EnsureSplineCount(int requiredCount)
    {
        while (splineContainer.Splines.Count < requiredCount)
        {
            SplineUtility.AddSpline(splineContainer);
        }
    }

    private void ClearAllSplines()
    {
        for (int i = 0; i < splineContainer.Splines.Count; i++)
        {
            splineContainer.Splines[i].Clear();
            splineContainer.Splines[i].Closed = false;
        }
    }
}