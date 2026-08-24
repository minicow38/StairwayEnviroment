using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

[DisallowMultipleComponent]
public sealed class NearestKnotDetector : MonoBehaviour
{
    const float Eps = 0.000001f, MinSlopeAngle = 2f, MinCurvature = 0.0005f, MaxCurvature = 1f;

    [SerializeField] SplineContainer splineContainer;
    [Header("Search")]
    [Min(1)] [SerializeField] int localSearchWindow = 4;
    [Min(.1f)] [SerializeField] float fullSearchDistance = 3f, teleportDistance = 5f;

    public int NearestSplineIndex { get; private set; } = -1;
    public int NearestKnotIndex { get; private set; } = -1;
    public Vector3 NearestKnotPosition { get; private set; }
    public float NearestKnotDistance { get; private set; } = float.PositiveInfinity;
    public GuideFrame CurrentGuide { get; private set; }

    [System.Serializable]
    public struct GuideFrame
    {
        public bool valid, isSlope, nextIsSlope;
        public int splineIndex, segmentIndex, sectionIndex;
        public Vector3 point, tangent, normal, nextTangent, nextNormal;
        public float distanceToGuide, segmentT, slopeAngle;
        public float curvature, entryCurvature, exitCurvature;
        public float sectionProgress01, sectionLength, distanceFromSectionStart, distanceToSectionEnd;
        public float distanceToNextSlope, nextSlopeAngle, nextCurvature;
    }

    // Same-section read-only sampling for SlopeStickCore / BallVisual.
    // This uses the same cached polyline geometry as Evaluate(), so
    // physics control and BallVisual planning share one canonical path model.
    [System.Serializable]
    public struct GuideSample
    {
        public bool valid, isSlope;
        public int splineIndex, segmentIndex, sectionIndex;
        public Vector3 point, tangent, normal;
        public float segmentT, slopeAngle;
        public float curvature, entryCurvature, exitCurvature;
        public float sectionProgress01, sectionLength, distanceFromSectionStart, distanceToSectionEnd;
    }

    sealed class SegmentInfo
    {
        public int splineIndex, segmentIndex, startKnotIndex, endKnotIndex, sectionIndex = -1;
        public Vector3 start, end, tangent, normal;
        public float length, slopeAngle, entryCurvature, exitCurvature, curvature;
        public float sectionLength, distanceFromSectionStart;
        public bool isSlope;
    }

    readonly List<SegmentInfo> segments = new();
    int currentSegment = -1, cachedSplineCount = -1, nextSectionIndex;
    Vector3 lastPosition;
    bool hasLastPosition;

    void Start() => RebuildCache();

    // RuntimeでSplineを書き換えた場合は、生成完了後に一度呼ぶ。
    public void RebuildCache()
    {
        segments.Clear(); currentSegment = -1; nextSectionIndex = 0; hasLastPosition = false;
        if (!splineContainer || splineContainer.Splines.Count == 0) { cachedSplineCount = 0; CurrentGuide = default; return; }
        cachedSplineCount = splineContainer.Splines.Count;

        for (int s = 0; s < splineContainer.Splines.Count; s++)
        {
            Spline spline = splineContainer.Splines[s];
            if (spline == null || spline.Count < 2) continue;
            int first = segments.Count;

            for (int k = 0; k < spline.Count - 1; k++)
            {
                Vector3 a = splineContainer.transform.TransformPoint((Vector3)spline[k].Position);
                Vector3 b = splineContainer.transform.TransformPoint((Vector3)spline[k + 1].Position);
                Vector3 delta = b - a; float length = delta.magnitude;
                if (length <= Eps) continue;
                Vector3 tangent = delta / length, normal = BuildUnbankedNormal(tangent);
                float angle = Vector3.Angle(normal, Vector3.up);
                segments.Add(new SegmentInfo {
                    splineIndex = s, segmentIndex = k, startKnotIndex = k, endKnotIndex = k + 1,
                    start = a, end = b, tangent = tangent, normal = normal, length = length,
                    slopeAngle = angle, isSlope = angle >= MinSlopeAngle
                });
            }

            int last = segments.Count - 1;
            if (last < first) continue;
            PopulateCurvature(first, last);
            BuildSlopeSections(first, last);
        }
    }

    // Physics側のFixedUpdateから1回だけ呼ぶ。
    public GuideFrame Evaluate(Vector3 position)
    {
        if (!splineContainer) return CurrentGuide = default;
        if (segments.Count == 0 || cachedSplineCount != splineContainer.Splines.Count) RebuildCache();
        if (segments.Count == 0) return CurrentGuide = default;

        bool teleported = hasLastPosition && Vector3.Distance(position, lastPosition) > teleportDistance;
        int index = teleported || currentSegment < 0
            ? FindGlobal(position, out float t, out Vector3 point, out float sqr)
            : FindLocal(position, out t, out point, out sqr);
        if (index < 0) return CurrentGuide = default;
        if (!teleported && sqr > fullSearchDistance * fullSearchDistance)
            index = FindGlobal(position, out t, out point, out sqr);

        currentSegment = index; lastPosition = position; hasLastPosition = true;
        SegmentInfo seg = segments[index];
        float sectionDistance = seg.distanceFromSectionStart + t * seg.length;
        float progress = seg.isSlope && seg.sectionLength > Eps ? Mathf.Clamp01(sectionDistance / seg.sectionLength) : 0f;

        UpdateNearestKnot(seg, t, position);
        ReadNextSlope(index, t, out bool nextSlope, out float nextDistance,
            out Vector3 nextTangent, out Vector3 nextNormal, out float nextAngle, out float nextCurvature);

        return CurrentGuide = new GuideFrame {
            valid = true, splineIndex = seg.splineIndex, segmentIndex = seg.segmentIndex, sectionIndex = seg.sectionIndex,
            point = point, tangent = seg.tangent, normal = seg.normal, distanceToGuide = Mathf.Sqrt(sqr), segmentT = t,
            isSlope = seg.isSlope, slopeAngle = seg.slopeAngle, curvature = seg.curvature,
            entryCurvature = seg.entryCurvature, exitCurvature = seg.exitCurvature,
            sectionProgress01 = progress, sectionLength = seg.sectionLength, distanceFromSectionStart = sectionDistance,
            distanceToSectionEnd = seg.isSlope ? Mathf.Max(0f, seg.sectionLength - sectionDistance) : 0f,
            nextIsSlope = nextSlope, distanceToNextSlope = nextDistance, nextTangent = nextTangent,
            nextNormal = nextNormal, nextSlopeAngle = nextAngle, nextCurvature = nextCurvature
        };
    }

    // The anchor identifies the logical slope section. This method never
    // changes CurrentGuide/currentSegment; it is a pure read-only look-ahead.
    public bool TryEvaluateSameSection(
        GuideFrame anchor,
        float progress01,
        out GuideSample sample)
    {
        sample = default;

        if (!anchor.valid ||
            !anchor.isSlope ||
            anchor.sectionIndex < 0 ||
            anchor.splineIndex < 0)
        {
            return false;
        }

        if (!splineContainer)
            return false;

        if (segments.Count == 0 ||
            cachedSplineCount != splineContainer.Splines.Count)
        {
            RebuildCache();
        }

        if (segments.Count == 0)
            return false;

        float sectionLength = Mathf.Max(Eps, anchor.sectionLength);
        float clampedProgress = Mathf.Clamp01(progress01);
        float targetDistance = clampedProgress * sectionLength;

        SegmentInfo selected = null;

        for (int i = 0; i < segments.Count; i++)
        {
            SegmentInfo s = segments[i];

            if (s.splineIndex != anchor.splineIndex ||
                s.sectionIndex != anchor.sectionIndex ||
                !s.isSlope)
            {
                continue;
            }

            float segmentStart = s.distanceFromSectionStart;
            float segmentEnd = segmentStart + s.length;

            // At progress=1 choose the final segment in the section.
            if (targetDistance <= segmentEnd + Eps)
            {
                selected = s;
                break;
            }
        }

        if (selected == null)
            return false;

        float localDistance =
            targetDistance -
            selected.distanceFromSectionStart;

        float segmentT =
            selected.length > Eps
                ? Mathf.Clamp01(localDistance / selected.length)
                : 0f;

        Vector3 point =
            Vector3.Lerp(
                selected.start,
                selected.end,
                segmentT);

        sample = new GuideSample
        {
            valid = true,
            isSlope = true,
            splineIndex = selected.splineIndex,
            segmentIndex = selected.segmentIndex,
            sectionIndex = selected.sectionIndex,
            point = point,
            tangent = selected.tangent,
            normal = selected.normal,
            segmentT = segmentT,
            slopeAngle = selected.slopeAngle,
            curvature = selected.curvature,
            entryCurvature = selected.entryCurvature,
            exitCurvature = selected.exitCurvature,
            sectionProgress01 = clampedProgress,
            sectionLength = sectionLength,
            distanceFromSectionStart = targetDistance,
            distanceToSectionEnd =
                Mathf.Max(
                    0f,
                    sectionLength - targetDistance)
        };

        return true;
    }

    public bool IsSameSection(
        GuideFrame a,
        GuideFrame b)
    {
        return
            a.valid &&
            b.valid &&
            a.isSlope &&
            b.isSlope &&
            a.sectionIndex >= 0 &&
            a.splineIndex == b.splineIndex &&
            a.sectionIndex == b.sectionIndex;
    }

    // sectionProgress01 is arc-length normalized in this detector's cached
    // piecewise-linear section, so this is the exact cached-path distance.
    public bool TryGetDistanceAlongSameSection(
        GuideFrame anchor,
        float fromProgress01,
        float toProgress01,
        out float distance)
    {
        distance = 0f;

        if (!anchor.valid ||
            !anchor.isSlope ||
            anchor.sectionIndex < 0 ||
            anchor.sectionLength <= Eps)
        {
            return false;
        }

        distance =
            Mathf.Abs(
                Mathf.Clamp01(toProgress01) -
                Mathf.Clamp01(fromProgress01)) *
            anchor.sectionLength;

        return true;
    }

    int FindLocal(Vector3 p, out float t, out Vector3 point, out float sqr)
    {
        SegmentInfo current = segments[currentSegment];
        int from = Mathf.Max(0, currentSegment - localSearchWindow), to = Mathf.Min(segments.Count - 1, currentSegment + localSearchWindow);
        int best = -1; t = 0f; point = default; sqr = float.PositiveInfinity;
        for (int i = from; i <= to; i++)
            if (segments[i].splineIndex == current.splineIndex) Test(i, p, ref best, ref t, ref point, ref sqr);
        return best;
    }

    int FindGlobal(Vector3 p, out float t, out Vector3 point, out float sqr)
    {
        int best = -1; t = 0f; point = default; sqr = float.PositiveInfinity;
        for (int i = 0; i < segments.Count; i++) Test(i, p, ref best, ref t, ref point, ref sqr);
        return best;
    }

    void Test(int index, Vector3 p, ref int best, ref float bestT, ref Vector3 bestPoint, ref float bestSqr)
    {
        SegmentInfo s = segments[index]; Project(p, s.start, s.end, out float t, out Vector3 point, out float sqr);
        if (sqr >= bestSqr) return;
        best = index; bestT = t; bestPoint = point; bestSqr = sqr;
    }

    static void Project(Vector3 p, Vector3 a, Vector3 b, out float t, out Vector3 point, out float sqr)
    {
        Vector3 ab = b - a; float d = ab.sqrMagnitude;
        t = d > Eps ? Mathf.Clamp01(Vector3.Dot(p - a, ab) / d) : 0f;
        point = a + ab * t; sqr = (p - point).sqrMagnitude;
    }

    void PopulateCurvature(int first, int last)
    {
        for (int i = first; i <= last; i++)
        {
            SegmentInfo s = segments[i];
            s.entryCurvature = i > first ? BoundaryCurvature(segments[i - 1], s) : 0f;
            s.exitCurvature = i < last ? BoundaryCurvature(s, segments[i + 1]) : 0f;
            s.curvature = Mathf.Clamp(Mathf.Max(MinCurvature, Mathf.Max(s.entryCurvature, s.exitCurvature)), MinCurvature, MaxCurvature);
        }
    }

    static float BoundaryCurvature(SegmentInfo a, SegmentInfo b)
    {
        float angle = Vector3.Angle(a.tangent, b.tangent) * Mathf.Deg2Rad;
        return Mathf.Clamp(angle / Mathf.Max(Eps, (a.length + b.length) * .5f), 0f, MaxCurvature);
    }

    void BuildSlopeSections(int first, int last)
    {
        for (int i = first; i <= last;)
        {
            if (!segments[i].isSlope) { i++; continue; }
            int start = i; float length = 0f;
            while (i <= last && segments[i].isSlope) { length += segments[i].length; i++; }
            int section = nextSectionIndex++; float distance = 0f;
            for (int j = start; j < i; j++) {
                segments[j].sectionIndex = section; segments[j].sectionLength = length;
                segments[j].distanceFromSectionStart = distance; distance += segments[j].length;
            }
        }
    }

    void ReadNextSlope(int index, float t, out bool found, out float distance, out Vector3 tangent,
        out Vector3 normal, out float angle, out float curvature)
    {
        found = false; distance = float.PositiveInfinity; tangent = default; normal = Vector3.up; angle = 0f; curvature = MinCurvature;
        SegmentInfo current = segments[index]; if (current.isSlope) return;
        float d = current.length * (1f - t);
        for (int i = index + 1; i < segments.Count && segments[i].splineIndex == current.splineIndex; i++) {
            SegmentInfo s = segments[i];
            if (s.isSlope) { found = true; distance = d; tangent = s.tangent; normal = s.normal; angle = s.slopeAngle; curvature = s.curvature; return; }
            d += s.length;
        }
    }

    void UpdateNearestKnot(SegmentInfo s, float t, Vector3 p)
    {
        bool start = t <= .5f;
        NearestSplineIndex = s.splineIndex; NearestKnotIndex = start ? s.startKnotIndex : s.endKnotIndex;
        NearestKnotPosition = start ? s.start : s.end; NearestKnotDistance = Vector3.Distance(p, NearestKnotPosition);
    }

    static Vector3 BuildUnbankedNormal(Vector3 tangent)
    {
        Vector3 flat = Vector3.ProjectOnPlane(tangent, Vector3.up); if (flat.sqrMagnitude <= Eps) return Vector3.up;
        Vector3 side = Vector3.Cross(Vector3.up, flat.normalized).normalized;
        Vector3 normal = Vector3.Cross(tangent.normalized, side).normalized;
        return Vector3.Dot(normal, Vector3.up) < 0f ? -normal : normal;
    }
}
