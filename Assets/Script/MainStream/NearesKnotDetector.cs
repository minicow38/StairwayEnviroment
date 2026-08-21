using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

[DisallowMultipleComponent]
public sealed class NearestKnotDetector : MonoBehaviour
{
    const float Eps = .000001f, MinSlopeY = .035f, MaxCurvature = 1f;

    [SerializeField] SplineContainer splineContainer;
    [Header("Search")]
    [Min(1)] [SerializeField] int localSearchWindow = 4;
    [Min(.1f)] [SerializeField] float fullSearchDistance = 3f, teleportDistance = 5f;

    public GuideFrame CurrentGuide { get; private set; }

    [System.Serializable]
    public struct GuideFrame
    {
        public bool valid, isSlope, nextIsSlope;
        public Vector3 tangent, normal;
        public float distanceToGuide, curvature, entryCurvature, sectionProgress01, sectionLength;
    }

    sealed class Segment
    {
        public int spline;
        public Vector3 start, tangent, normal;
        public float length, entryK, sectionLength, sectionStart;
        public bool slope, nextSlope;
    }

    readonly List<Segment> segments = new();
    int current = -1, cachedSplineCount = -1;
    Vector3 lastPosition;
    bool hasLastPosition;

    void Start() => RebuildCache();

    public void RebuildCache()
    {
        segments.Clear(); current = -1; hasLastPosition = false;
        if (!splineContainer || splineContainer.Splines.Count == 0) {
            cachedSplineCount = 0; CurrentGuide = default; return;
        }

        cachedSplineCount = splineContainer.Splines.Count;
        Transform root = splineContainer.transform;

        for (int s = 0; s < cachedSplineCount; s++)
        {
            Spline spline = splineContainer.Splines[s];
            if (spline == null || spline.Count < 2) continue;
            int first = segments.Count;

            for (int k = 0; k < spline.Count - 1; k++)
            {
                Vector3 a = root.TransformPoint((Vector3)spline[k].Position);
                Vector3 d = root.TransformPoint((Vector3)spline[k + 1].Position) - a;
                float length = d.magnitude;
                if (length <= Eps) continue;

                Vector3 tangent = d / length;
                Vector3 normal = Vector3.ProjectOnPlane(Vector3.up, tangent);
                normal = normal.sqrMagnitude > Eps ? normal.normalized : Vector3.up;

                segments.Add(new Segment {
                    spline = s, start = a, tangent = tangent, normal = normal, length = length,
                    slope = Mathf.Abs(tangent.y) >= MinSlopeY
                });
            }

            int last = segments.Count - 1;
            if (last >= first) BuildMetadata(first, last);
        }
    }

    public GuideFrame Evaluate(Vector3 p)
    {
        if (!splineContainer) return CurrentGuide = default;
        if (segments.Count == 0 || cachedSplineCount != splineContainer.Splines.Count) RebuildCache();
        if (segments.Count == 0) return CurrentGuide = default;

        bool global = current < 0 || hasLastPosition &&
            (p - lastPosition).sqrMagnitude > teleportDistance * teleportDistance;

        int from = global ? 0 : Mathf.Max(0, current - localSearchWindow);
        int to = global ? segments.Count - 1 : Mathf.Min(segments.Count - 1, current + localSearchWindow);
        int filter = global ? -1 : segments[current].spline;

        int index = FindBest(p, from, to, filter, out float t, out float sqr);
        if (index < 0) return CurrentGuide = default;

        if (!global && sqr > fullSearchDistance * fullSearchDistance)
            index = FindBest(p, 0, segments.Count - 1, -1, out t, out sqr);
        if (index < 0) return CurrentGuide = default;

        current = index; lastPosition = p; hasLastPosition = true;
        Segment seg = segments[index];
        float progress = seg.slope && seg.sectionLength > Eps
            ? Mathf.Clamp01((seg.sectionStart + t * seg.length) / seg.sectionLength) : 0f;

        return CurrentGuide = new GuideFrame {
            valid = true, isSlope = seg.slope, nextIsSlope = seg.nextSlope,
            tangent = seg.tangent, normal = seg.normal, distanceToGuide = Mathf.Sqrt(sqr),
            curvature = seg.entryK, entryCurvature = seg.entryK,
            sectionProgress01 = progress, sectionLength = seg.sectionLength
        };
    }

    int FindBest(Vector3 p, int from, int to, int filter, out float bestT, out float bestSqr)
    {
        int best = -1; bestT = 0f; bestSqr = float.PositiveInfinity;
        for (int i = from; i <= to; i++)
        {
            Segment s = segments[i];
            if (filter >= 0 && s.spline != filter) continue;
            float along = Mathf.Clamp(Vector3.Dot(p - s.start, s.tangent), 0f, s.length);
            float sqr = (p - (s.start + s.tangent * along)).sqrMagnitude;
            if (sqr >= bestSqr) continue;
            best = i; bestT = along / s.length; bestSqr = sqr;
        }
        return best;
    }

    void BuildMetadata(int first, int last)
    {
        for (int i = first + 1; i <= last; i++) {
            Segment a = segments[i - 1], b = segments[i];
            float angle = Vector3.Angle(a.tangent, b.tangent) * Mathf.Deg2Rad;
            b.entryK = Mathf.Clamp(angle / Mathf.Max(Eps, (a.length + b.length) * .5f), 0f, MaxCurvature);
        }

        for (int i = first; i <= last;)
        {
            if (!segments[i].slope) { i++; continue; }
            int start = i; float length = 0f;
            while (i <= last && segments[i].slope) length += segments[i++].length;

            float distance = 0f;
            for (int j = start; j < i; j++) {
                segments[j].sectionLength = length;
                segments[j].sectionStart = distance;
                distance += segments[j].length;
            }
        }

        bool futureSlope = false;
        for (int i = last; i >= first; i--) {
            segments[i].nextSlope = !segments[i].slope && futureSlope;
            if (segments[i].slope) futureSlope = true;
        }
    }
}
