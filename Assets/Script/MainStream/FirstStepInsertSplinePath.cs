using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;
using System.Collections;
using Object = UnityEngine.Object;

//using System;

[DisallowMultipleComponent]
public class CoreStepInsertSplinePathNatural : MonoBehaviour
{
    sealed class BoardPair
    {
        public Transform Physics;
        public Transform Visual;
    }

    [Header("Coin Local Placement")] [SerializeField]
    float coinSpacing = 1.5f;

    [SerializeField] float coinLocalHeight = 0f;
    [SerializeField] NearestKnotDetector knotDetector;

    public MainGameManager ActiveSlopeReciver;
    [Header("Source")] [SerializeField] Transform stairPlane;
    [SerializeField] SplineContainer splineBox;
    public GameObject PrimitivePlane;
    public GameObject StairwayPrefab;

    [Header("Output Roots")] [Tooltip("PhysicsRoot/CollisionStageRootを設定します。")] [SerializeField]
    Transform collisionStageRoot;

    [Tooltip("VisualPlayerRoot/RenderStageRootを設定します。")] [SerializeField]
    Transform renderStageRoot;

    public int ModifyOverrap;

    public int ContinuousPattern = 0;
    private Vector3 ActivePlane;

    [SerializeField] string collisionStageRootPath = "/PhysicsRoot/CollisionStageRoot";
    [SerializeField] string renderStageRootPath = "/VisualPlayerRoot/RenderStageRoot";
    [SerializeField] string generatedPhysicsName = "__GeneratedPhysics";
    [SerializeField] string generatedVisualPlayerName = "__GeneratedVisualPlayer";
    [SerializeField] string legacyGeneratedBoardRootName = "__GeneratedSplineBoards";
    [SerializeField] bool removeLegacyGeneratedBoardRoot = true;

    [Header("Path")] [SerializeField] float pathWidth = 3f;
    [SerializeField] float endPadding = 1f;
    [SerializeField] float edgeStepOverride;
    [SerializeField] float knotStep = 2f;
    [SerializeField, Range(1f, 89f)] float bendDegrees = 45f;
    [SerializeField] float laneGapOverride;
    [SerializeField] float miterLimit = 4f;
    [SerializeField] float bendZOffset;
    [SerializeField] float falseFlatScale = 1f;
    [SerializeField] bool turnToPositiveZ = true;
    [SerializeField] bool regularizeTurn = true;
    [SerializeField] bool verticalFalsePair = true;

    [Header("Generated Representations")] [SerializeField]
    bool hidePhysicsRenderers = true;

    [SerializeField] bool disableVisualColliders = true;
    [SerializeField] bool removeGeneratedRigidbodies = true;

    public bool ReverseSpline;
    public bool FirstVertical;
    public bool StraightStumble;
    public bool PlaneTime;
    public bool cased;
    public bool twistReturn;

    public Dictionary<String, List<GameObject>> RogicalEntity;
    public List<GameObject> StackStairway1;
    public List<GameObject> StackStairway2;

    public GameObject PylonPrefab;
    public GameObject CoinPrefab;

    public int FirstShift = 0;

    public Vector3 RootStartpoint = new Vector3(-8.535f, 31.8f, -0.1f);
    public Vector3 stepHandlePoint;
    Vector3 previousFlatPosition;
    Vector3 previousFlatDirection;
    bool hasPreviousFlat;

    [SerializeField] float flatShiftThreshold = 12.15f;
    public List<Vector3> PrevInclined;
    public Spline accumulatedSpline;
    public List<int> outcount;
    public GameObject RootInSpiral;
    public (Vector3, Vector3) playBackDownWard;
    public int ActiveTurnPoint;

    List<int> startPattern = new List<int>
    {
        -1, 0, -1, 0, -5, 0, -3, 0, -5, 0, -1, 0, -1, 0, -1, 0, -1
    };

    private Vector3[] ShiftCoint =
    {
        new Vector3(-3f, 0, 0),
        new Vector3(0, 0, 0),
        new Vector3(3f, 0, 0)
    };

    static readonly Vector3[] Dirs =
    {
        Vector3.right, Vector3.forward, Vector3.left, Vector3.back
    };

    const int Lanes = 3;
    const int Center = 1;
    const int Max = 6;
    const float E = 0.00001f;

    readonly Spline[] lanes = new Spline[Lanes];
    readonly Vector3[] left = new Vector3[Max];
    readonly Vector3[] prev = new Vector3[Max];
    readonly Vector3[] next = new Vector3[Max];
    readonly Vector3[] last = new Vector3[Lanes];
    readonly bool[] hasLast = new bool[Lanes];

    Vector3[] points;
    int[] counts;
    int[] scales;

    Transform generatedPhysicsRoot;
    Transform generatedVisualRoot;
    Material[] materials;
    Matrix4x4 toSpline;

    Vector3 up;
    Vector3 fall;
    float width;
    float halfWidth;
    float halfLength;
    float gap;
    float flat;
    float run;
    float knot;
    float e2;
    float sideGap;
    float miter2;
    int arcSlabCount;
    int stairwayCount;

    void FixedUpdate()
    {
        if (ActiveSlopeReciver.OpenChunkStage)
        {
            ActiveSlopeReciver.OpenChunkStage = false;
            Start();
        }

    }

    [ContextMenu("RebuildSpline")]
    void Start()
    {
        if (RogicalEntity == null || RogicalEntity.Count == 0)
        {
            ActiveSlopeReciver = GameObject.Find("GameManager").transform.GetComponent<MainGameManager>();
            RogicalEntity = new Dictionary<String, List<GameObject>>();
            StackStairway1 = new List<GameObject>();
            StackStairway2 = new List<GameObject>();
            RogicalEntity["Physics"] = StackStairway1;
            RogicalEntity["Renderer"] = StackStairway2;
        }

        // Time.timeScale = 0.5f;
        if (!RootInSpiral)
            RootInSpiral = GameObject.Find("StairwaySimple");

        if (!Prepare() || !EnsureOutputRoots())
            return;
        if (outcount.Count == 0)
        {
            hasPreviousFlat = false;
            previousFlatPosition = Vector3.zero;
            previousFlatDirection = Vector3.zero;
            ClearGeneratedStage();
            ClearLegacyGeneratedStage();

            ClearSplines();


            CacheTransforms();
        }
        else
        {
            ContinuousPattern = startPattern.Count;
            startPattern.AddRange(new int[]
            {
                -1, 0, -1, 0, -1, 0, -1, 0, -1
            });
            Debug.Log("");
        }

        EnsureWorkingBuffers();


        ActivePlane = RootStartpoint;

        StraightStumble = false;
        FirstVertical = false;
        ReverseSpline = false;
        ActiveTurnPoint = 0;
        arcSlabCount = 0;
        stairwayCount = 0;

        if (outcount.Count == 0)
            outcount.Clear();

        Build(0, 0, RootStartpoint);

        for (int i = ContinuousPattern; i < startPattern.Count; i++)
            Emit(i, i > 0);

        int finalOffset = (startPattern.Count - 1) * Max;
        int finalCount = counts[startPattern.Count - 1];

        if (PrevInclined == null)
            PrevInclined = new List<Vector3>(Max);
        else
            PrevInclined.Clear();

        for (int i = 0; i < finalCount; i++)
            PrevInclined.Add(points[finalOffset + i]);

        StartCoroutine(DelayStandOnObject());
        accumulatedSpline = lanes[Center];
        if (knotDetector)
            knotDetector.RebuildCache();
    }

    void EnsureWorkingBuffers()
    {
        int count = startPattern.Count;

        if (points == null || points.Length < count * Max)
            points = new Vector3[count * Max];
        if (counts == null || counts.Length < count)
            counts = new int[count];
        if (scales == null || scales.Length < count)
            scales = new int[count];
    }

    void Build(int index, int enter, Vector3 start)
    {
        int step = startPattern[index];
        int abs = Mathf.Abs(step);
        bool zero = abs <= E;
        bool vertical = zero && index > 0;
        bool append = !zero && index > 0;
        bool pair = vertical && verticalFalsePair && ((index > 0 && Mathf.Abs(startPattern[index - 1]) <= E) ||
                                                      (index + 1 < startPattern.Count &&
                                                       Mathf.Abs(startPattern[index + 1]) <= E));

        int turnStep = zero ? 0 : step > 0f ? -1 : 1;
        if (!turnToPositiveZ)
            turnStep = -turnStep;

        outcount.Add(turnStep);

        int exit = vertical ? enter : (enter + turnStep) & 3;
        int scale = zero ? 1 : abs;
        float slopeRun = run * scale;
        Vector3 forward = Dirs[enter & 3];
        Vector3 direction = Dirs[exit];
        Vector3 drop = fall * slopeRun;

        Vector3 edge = start + forward * (pair ? flat : append ? halfWidth : width);
        Vector3 slopeEnd = edge + forward * slopeRun + drop;
        Vector3 basePoint = !vertical && !append ? slopeEnd + forward * halfWidth : edge;
        Vector3 seedTurn = vertical ? slopeEnd : Turn(basePoint, direction, append);
        Vector3 seedEnd = vertical ? slopeEnd : seedTurn + direction * slopeRun + drop;
        bool hasChild = index + 1 < startPattern.Count;

        if (hasChild)
            Build(index + 1, exit, seedEnd);

        Vector3 turn = seedTurn;
        Vector3 end = seedEnd;

        if (hasChild && !vertical)
        {
            end = points[(index + 1) * Max];
            turn = end - direction * slopeRun - drop;
            ActiveTurnPoint++;
        }

        int offset = index * Max;
        int pointCount = 0;
        points[offset + pointCount++] = start;
        points[offset + pointCount++] = edge;

        if (vertical)
        {
            points[offset + pointCount++] = slopeEnd;
        }
        else
        {
            if (!append)
            {
                points[offset + pointCount++] = slopeEnd;
                points[offset + pointCount++] = basePoint;
            }

            points[offset + pointCount++] = turn;
            points[offset + pointCount++] = end;
        }

        counts[index] = pointCount;
        scales[index] = scale;
    }

    void Emit(int plan, bool skipFirstKnot)
    {
        int offset = plan * Max;
        int pointCount = counts[plan];
        CacheCorners(offset, pointCount);

        stepHandlePoint = points[offset + 1];
        playBackDownWard = (points[offset + pointCount - 2], points[offset + pointCount - 1]);

        for (int i = 0; i < pointCount - 1; i++)
        {
            Vector3 start = points[offset + i];
            Vector3 direction = points[offset + i + 1] - start;
            float sqr = direction.sqrMagnitude;

            if (sqr <= e2)
                continue;

            float length = Mathf.Sqrt(sqr);

            if (i == 0 && !skipFirstKnot)
                Knot(offset, i, -1f);

            int inside = Mathf.CeilToInt(length / knot) - 1;

            for (int k = 1; k <= inside; k++)
                Knot(offset, i, k / (float)(inside + 1));

            Knot(offset, i + 1, -1f);

            float horizontalLength = Mathf.Sqrt(direction.x * direction.x + direction.z * direction.z);

            float slopeAngle = horizontalLength > E
                ? Mathf.Atan2(Mathf.Abs(direction.y), horizontalLength) * Mathf.Rad2Deg
                : 0f;

            bool slope = horizontalLength > E && slopeAngle >= 1f;

            if (slope)
                PlaneTime = false;

            Board(start, direction, slope, slope ? scales[plan] : 1, plan, i);
        }
    }

    void CacheCorners(int offset, int pointCount)
    {
        Vector3 direction = Vector3.zero;

        for (int i = 0; i < pointCount; i++)
        {
            prev[i] = direction;
            if (i + 1 < pointCount)
                direction = Flat(points[offset + i + 1] - points[offset + i], direction);
        }

        direction = Vector3.zero;

        for (int i = pointCount - 1; i >= 0; i--)
        {
            next[i] = direction;
            if (i > 0)
                direction = Flat(points[offset + i] - points[offset + i - 1], direction);
        }

        for (int i = 0; i < pointCount; i++)
            left[i] = Corner(points[offset + i], prev[i], next[i]);
    }

    Vector3 Flat(Vector3 value, Vector3 fallback)
    {
        float horizontalSqr = value.x * value.x + value.z * value.z;
        return horizontalSqr <= e2 ? fallback : new Vector3(value.x, 0f, value.z) / Mathf.Sqrt(horizontalSqr);
    }

    Vector3 Corner(Vector3 point, Vector3 previousDirection, Vector3 nextDirection)
    {
        bool noPrevious = previousDirection == Vector3.zero;
        bool noNext = nextDirection == Vector3.zero;

        if (noPrevious)
            return noNext ? point : point + Side(nextDirection);
        if (noNext)
            return point + Side(previousDirection);

        Vector3 previousSide = Side(previousDirection);
        Vector3 nextSide = Side(nextDirection);

        return Hit(point + previousSide, previousDirection, point + nextSide, nextDirection, point.y,
            out Vector3 hit) && (hit - point).sqrMagnitude <= miter2
            ? hit
            : point + Flat(previousSide + nextSide, nextSide) * gap;
    }

    Vector3 Side(Vector3 direction) => new Vector3(-direction.z * sideGap, 0f, direction.x * sideGap);

    void Knot(int offset, int pointIndex, float progress)
    {
        Vector3 center = progress < 0f
            ? points[offset + pointIndex]
            : Vector3.Lerp(points[offset + pointIndex], points[offset + pointIndex + 1], progress);

        Vector3 outer = progress < 0f
            ? left[pointIndex]
            : Vector3.Lerp(left[pointIndex], left[pointIndex + 1], progress);

        for (int laneIndex = 0; laneIndex < Lanes; laneIndex++)
        {
            Vector3 world = laneIndex == 0 ? outer : laneIndex == Center ? center : center * 2f - outer;

            if (hasLast[laneIndex] && (world - last[laneIndex]).sqrMagnitude <= e2)
                continue;

            hasLast[laneIndex] = true;
            last[laneIndex] = world;

            Vector3 local = toSpline.MultiplyPoint3x4(world);
            Spline spline = lanes[laneIndex];
            int knotIndex = spline.Count;
            spline.Add(new BezierKnot(new float3(local.x, local.y, local.z)));
            spline.SetTangentMode(knotIndex, TangentMode.Linear);
        }
    }

    void Board(
        Vector3 worldStart,
        Vector3 worldDirection,
        bool slope,
        int scale,
        int plan,
        int segmentIndex)
    {
        if (PlaneTime)
            return;

        // ============================================================
        // World -> Local
        // ============================================================

        Vector3 localStart =
            collisionStageRoot.InverseTransformPoint(worldStart);

        Vector3 localDirection =
            collisionStageRoot.InverseTransformVector(worldDirection);

        float directionSqr =
            localDirection.sqrMagnitude;

        if (directionSqr <= e2)
            return;

        Vector3 forward =
            localDirection / Mathf.Sqrt(directionSqr);

        Vector3 right =
            Vector3.Cross(up, forward);

        float rightSqr =
            right.sqrMagnitude;

        if (rightSqr <= e2)
            return;

        right /= Mathf.Sqrt(rightSqr);

        // ============================================================
        // Position / Rotation / Scale
        // ============================================================

        Vector3 localPosition;

        if (scale == 1)
        {
            // PrimitivePlane の中心を区間中央へ置く。
            localPosition =
                localStart +
                localDirection * 0.5f;
        }
        else
        {
            localPosition = localStart;
        }

        Quaternion localRotation =
            Quaternion.LookRotation(
                forward,
                Vector3.Cross(forward, right));

        Vector3 localScale =
            slope
                ? new Vector3(1f, 1f, scale)
                : Vector3.one;

        // ============================================================
        // Flat
        // ============================================================

        if (!slope)
        {
            Vector3 currentFlatPosition =
                worldStart;

            Vector3 currentFlatDirection =
                worldDirection.sqrMagnitude > e2
                    ? worldDirection.normalized
                    : Vector3.zero;

            bool shiftHalf = false;

            float currentProjection = 0f;
            float previousProjection = 0f;
            float directDistance = 0f;
            float robustDistance = 0f;
            float directionAngle = 0f;

            // ========================================================
            // 前回の Flat と比較
            // ========================================================

            if (hasPreviousFlat)
            {
                Vector3 delta =
                    currentFlatPosition -
                    previousFlatPosition;

                directDistance =
                    delta.magnitude;

                // 現在の Flat 方向への射影。
                // Abs を使うため、方向が反転しても距離は正値になる。
                if (currentFlatDirection.sqrMagnitude > e2)
                {
                    currentProjection =
                        Mathf.Abs(
                            Vector3.Dot(
                                delta,
                                currentFlatDirection));
                }

                // 前回の Flat 方向への射影。
                // 90度曲がった場合も距離を拾えるようにする。
                if (previousFlatDirection.sqrMagnitude > e2)
                {
                    previousProjection =
                        Mathf.Abs(
                            Vector3.Dot(
                                delta,
                                previousFlatDirection));
                }

                robustDistance =
                    Mathf.Max(
                        currentProjection,
                        previousProjection);

                if (currentFlatDirection.sqrMagnitude > e2 &&
                    previousFlatDirection.sqrMagnitude > e2)
                {
                    directionAngle =
                        Vector3.Angle(
                            previousFlatDirection,
                            currentFlatDirection);
                }

                // ====================================================
                // shiftHalf 判定
                // ====================================================

                bool validPlan =
                    plan >= 0 &&
                    plan < startPattern.Count;

                bool currentNonZero =
                    validPlan &&
                    startPattern[plan] != 0;

                bool previousNonZero =
                    plan > 0 &&
                    plan - 1 < startPattern.Count &&
                    startPattern[plan - 1] != 0;

                // Build() の点構造に対応した補正対象。
                //
                // plan == 0 の非0パターン:
                //   segment 0 : start -> edge       Flat
                //   segment 1 : edge -> slopeEnd    Slope
                //   segment 2 : slopeEnd -> base    Flat  <- 補正候補
                //
                // plan > 0 の非0パターン (append):
                //   segment 0 : start -> edge       Flat  <- 補正候補
                //   後続 Flat は PlaneTime により重複生成されない。
                bool correctionSegment =
                    (plan == 0 && segmentIndex == 2) ||
                    (plan > 0 && segmentIndex == 0);

                // 従来の距離判定。
                bool actualGap =
                    robustDistance >= flatShiftThreshold;

                // -1,-1,-1... のように非0が連続した場合は、
                // 距離だけでは識別しにくいため plan 構造でも許可する。
                bool consecutiveNonZero =
                    previousNonZero &&
                    currentNonZero &&
                    segmentIndex == 0;

                shiftHalf =
                    currentNonZero &&
                    correctionSegment &&
                    (actualGap || consecutiveNonZero);
            }

            // ========================================================
            // Plane 生成
            // ========================================================

            BoardPair[] arcSlab =
                TakeBoard(
                    false,
                    LayerMask.NameToLayer("Slope"),
                    $"ArcSlab{ContinuousPattern + arcSlabCount++}",
                    scale);

            ApplyBoardPose(
                arcSlab,
                localPosition,
                localDirection,
                localRotation,
                localScale,
                shiftHalf);

            // ========================================================
            // Debug
            // ========================================================

            int patternValue =
                plan >= 0 && plan < startPattern.Count
                    ? startPattern[plan]
                    : int.MinValue;

            Debug.Log(
                $"Flat " +
                $"plan={plan}, " +
                $"pattern={patternValue}, " +
                $"segment={segmentIndex}, " +
                $"shift={shiftHalf}, " +
                $"distance={directDistance:F3}, " +
                $"currentProjection={currentProjection:F3}, " +
                $"previousProjection={previousProjection:F3}, " +
                $"robustDistance={robustDistance:F3}, " +
                $"angle={directionAngle:F1}, " +
                $"direction={currentFlatDirection}");

            // 次の Flat 判定用に保存する。
            previousFlatPosition =
                currentFlatPosition;

            previousFlatDirection =
                currentFlatDirection;

            hasPreviousFlat = true;
        }
        // ============================================================
        // Slope
        // ============================================================
        else
        {
            BoardPair[] stairway =
                TakeBoard(
                    true,
                    LayerMask.NameToLayer("Slope"),
                    $"StairWay{ContinuousPattern + stairwayCount++}",
                    scale);

            Debug.Log(
                $"StairWay{ContinuousPattern + stairwayCount} " +
                $"plan={plan}, " +
                $"segment={segmentIndex}, " +
                $"scale={scale}, " +
                $"start={localStart}, " +
                $"direction={localDirection}, " +
                $"length={localDirection.magnitude}");

            // Slope 側には Flat 用の半区間補正を掛けない。
            ApplyBoardPose(
                stairway,
                localPosition,
                localDirection,
                localRotation,
                localScale,
                false);
        }

        if (!slope)
            PlaneTime = true;

        ActivePlane = worldStart;
    }

    BoardPair[] TakeBoard(bool slope, int physicsLayer, string boardName, int mulPlane)
    {
        GameObject sourcePrefab;

        BoardPair[] boardPairs = new BoardPair[mulPlane];

        for (int i = 0; i < mulPlane; i++)
        {
            GameObject visualObject;

            GameObject physicsObject = Instantiate(PrimitivePlane, generatedPhysicsRoot, false);
            if (slope)
            {

                visualObject = Instantiate(StairwayPrefab, generatedVisualRoot, false);
                StackStairway1.Add(physicsObject);
                StackStairway2.Add(visualObject);
            }
            else
            {
                visualObject = Instantiate(PrimitivePlane, generatedVisualRoot, false);
            }

            physicsObject.name = $"{boardName}_{i}_Physics";
            visualObject.name = $"{boardName}_{i}_Render";

            ConfigurePhysicsRepresentation(physicsObject, physicsLayer);
            ConfigureVisualRepresentation(visualObject, slope);

            boardPairs[i] = new BoardPair
            {
                Physics = physicsObject.transform,
                Visual = visualObject.transform
            };
        }

        return boardPairs;
    }

    void ConfigurePhysicsRepresentation(GameObject physicsObject, int physicsLayer)
    {
        if (physicsLayer >= 0)
            SetLayerRecursively(physicsObject, physicsLayer);
        else
            Debug.LogError("SlopeまたはStairway Layerが見つかりません。", this);

        if (hidePhysicsRenderers)
        {
            foreach (Renderer renderer in physicsObject.GetComponentsInChildren<Renderer>(true))
                renderer.enabled = false;
        }

        if (removeGeneratedRigidbodies)
            RemoveJointsAndRigidbodies(physicsObject, true);

        if (physicsObject.GetComponentsInChildren<Collider>(true).Length == 0)
            Debug.LogWarning($"{physicsObject.name}にColliderがありません。", physicsObject);
    }

    void ConfigureVisualRepresentation(GameObject visualObject, bool InSlope)
    {
        if (!InSlope)
        {
            foreach (Collider collider in visualObject.GetComponentsInChildren<Collider>(true))
                collider.enabled = false;
        }

        if (removeGeneratedRigidbodies)
            RemoveJointsAndRigidbodies(visualObject, false);

        if (materials == null || materials.Length == 0)
            return;

        foreach (MeshRenderer renderer in visualObject.GetComponentsInChildren<MeshRenderer>(true))
            renderer.sharedMaterials = materials;
    }

    void ApplyBoardPose(
        BoardPair[] pairs,
        Vector3 localPosition,
        Vector3 localDirection,
        Quaternion localRotation,
        Vector3 localScale,
        bool shiftHalf)
    {
        if (pairs == null || pairs.Length == 0)
            return;


        // ============================================================
        // 複数枚
        // ============================================================

        if (pairs.Length > 1)
        {
            Vector3 totalVec =
                Vector3.zero;

            for (int i = 0; i < pairs.Length; i++)
            {
                BoardPair pair =
                    pairs[i];

                if (pair == null)
                    continue;

                totalVec +=
                    localDirection.normalized *
                    (i == 0 ? 5f : 10f);

                ApplyLocalPose(
                    pair.Physics,
                    localPosition + totalVec,
                    localRotation,
                    localScale);

                ApplyLocalPose(
                    pair.Visual,
                    localPosition + totalVec,
                    localRotation,
                    localScale);
            }

            return;
        }


        // ============================================================
        // 1枚
        // ============================================================

        BoardPair singlePair =
            pairs[0];

        if (singlePair == null)
            return;


        Vector3 position =
            localPosition;


        if (shiftHalf)
        {
            position +=
                localDirection * 0.5f;
        }


        ApplyLocalPose(
            singlePair.Physics,
            position,
            localRotation,
            localScale);

        ApplyLocalPose(
            singlePair.Visual,
            position,
            localRotation,
            localScale);
    }

    void ApplyLocalPose(Transform target, Vector3 localPosition, Quaternion localRotation, Vector3 localScale)
    {
        if (!target)
            return;
        target.localPosition = localPosition;
        target.localRotation = localRotation;

        target.localScale = Vector3.one;
        target.gameObject.SetActive(true);
    }

    Vector3 Turn(Vector3 point, Vector3 direction, bool append) =>
        !regularizeTurn || append
            ? point + direction * halfWidth
            : point + direction * (halfLength + bendZOffset - Vector3.Dot(point, direction));

    static bool Hit(Vector3 firstPoint, Vector3 firstDirection, Vector3 secondPoint, Vector3 secondDirection, float y,
        out Vector3 hit)
    {
        float cross = firstDirection.x * secondDirection.z - firstDirection.z * secondDirection.x;

        if (Mathf.Abs(cross) <= E)
        {
            hit = firstPoint;
            return false;
        }

        float t = ((secondPoint.x - firstPoint.x) * secondDirection.z -
                   (secondPoint.z - firstPoint.z) * secondDirection.x) / cross;

        hit = firstPoint + firstDirection * t;
        hit.y = y;
        return true;
    }

    bool Prepare()
    {
        if (!RootInSpiral)
            RootInSpiral = GameObject.Find("StairwaySimple");

        if (!stairPlane && RootInSpiral)
            stairPlane = RootInSpiral.transform.Find("Plane");

        if (!splineBox)
            splineBox = GetComponent<SplineContainer>();

        if (!stairPlane || !splineBox || !PrimitivePlane || !StairwayPrefab)
        {
            Debug.LogError("stairPlane、SplineContainer、PrimitivePlane、StairwayPrefabを確認してください。", this);
            return false;
        }

        float length;

        if (stairPlane.TryGetComponent(out MeshFilter filter) && filter.sharedMesh)
        {
            Vector3 size = filter.sharedMesh.bounds.size;
            Vector3 sourceScale = stairPlane.lossyScale;
            width = Mathf.Abs(size.x * sourceScale.x);
            length = Mathf.Abs(size.z * sourceScale.z);
        }
        else if (stairPlane.TryGetComponent(out Renderer renderer))
        {
            width = renderer.bounds.size.x;
            length = renderer.bounds.size.z;
        }
        else
        {
            Debug.LogError("stairPlaneにMeshFilterまたはRendererがありません。", stairPlane);
            return false;
        }

        width = Mathf.Max(E, width);
        halfWidth = width * 0.5f;
        halfLength = Mathf.Max(E, length) * 0.5f;
        gap = laneGapOverride > E ? laneGapOverride : halfLength * 0.5f;
        flat = Mathf.Max(E, width * Mathf.Max(0f, falseFlatScale));
        run = (edgeStepOverride > E ? edgeStepOverride : pathWidth > E ? pathWidth : width * 0.25f) * 2f + endPadding;
        knot = Mathf.Max(0.01f, knotStep);
        e2 = E * E;
        fall = Vector3.down * Mathf.Tan(bendDegrees * Mathf.Deg2Rad);
        sideGap = (turnToPositiveZ ? 1f : -1f) * gap;
        float miter = gap * Mathf.Max(1f, miterLimit);
        miter2 = miter * miter;

        if (stairPlane.TryGetComponent(out MeshRenderer sourceRenderer))
            materials = sourceRenderer.sharedMaterials;

        while (splineBox.Splines.Count < Lanes)
            SplineUtility.AddSpline(splineBox);

        for (int i = 0; i < Lanes; i++)
            lanes[i] = splineBox.Splines[i];

        return true;
    }

    bool EnsureOutputRoots()
    {
        if (!collisionStageRoot)
            collisionStageRoot = FindTransformByPath(collisionStageRootPath);
        if (!collisionStageRoot)
            collisionStageRoot = FindTransformByPath("/PhysicsRoot/CollisionStageRoot");
        if (!renderStageRoot)
            renderStageRoot = FindTransformByPath(renderStageRootPath);

        if (!collisionStageRoot || !renderStageRoot)
        {
            Debug.LogError("CollisionStageRootまたはRenderStageRootを取得できません。Inspectorで設定してください。", this);
            return false;
        }

        if (collisionStageRoot == renderStageRoot || collisionStageRoot.IsChildOf(renderStageRoot) ||
            renderStageRoot.IsChildOf(collisionStageRoot))
        {
            Debug.LogError("CollisionStageRootとRenderStageRootは独立させてください。", this);
            return false;
        }

        generatedPhysicsRoot = GetOrCreateChild(collisionStageRoot, generatedPhysicsName);
        generatedVisualRoot = GetOrCreateChild(renderStageRoot, generatedVisualPlayerName);
        ResetGeneratedRootTransform(generatedPhysicsRoot);
        ResetGeneratedRootTransform(generatedVisualRoot);
        ValidateRootScales();
        return true;
    }

    void CacheTransforms()
    {
        toSpline = splineBox.transform.worldToLocalMatrix;
        up = collisionStageRoot.InverseTransformDirection(Vector3.up);
        up = up.sqrMagnitude <= e2 ? Vector3.up : up.normalized;
    }

    void ClearSplines()
    {
        PlaneTime = false;

        for (int i = 0; i < splineBox.Splines.Count; i++)
        {
            Spline spline = splineBox.Splines[i];
            spline.Clear();
            spline.Closed = false;
        }

        for (int i = 0; i < Lanes; i++)
            hasLast[i] = false;
    }

    void ClearGeneratedStage()
    {
        DestroyChildren(generatedPhysicsRoot);
        DestroyChildren(generatedVisualRoot);
    }

    void ClearLegacyGeneratedStage()
    {
        if (!removeLegacyGeneratedBoardRoot || !splineBox)
            return;

        DestroyNamedChild(splineBox.transform, legacyGeneratedBoardRootName);

        // 旧Sceneでアンダースコアが1個だった場合にも対応する。
        if (legacyGeneratedBoardRootName != "_GeneratedSplineBoards")
            DestroyNamedChild(splineBox.transform, "_GeneratedSplineBoards");
    }

    static void DestroyNamedChild(Transform parent, string childName)
    {
        if (!parent || string.IsNullOrWhiteSpace(childName))
            return;

        Transform child = parent.Find(childName);
        if (!child)
            return;

        child.gameObject.SetActive(false);
        DestroyObjectSafely(child.gameObject);
    }

    static Transform FindTransformByPath(string hierarchyPath)
    {
        if (string.IsNullOrWhiteSpace(hierarchyPath))
            return null;

        GameObject found = GameObject.Find(hierarchyPath);
        return found ? found.transform : null;
    }

    static Transform GetOrCreateChild(Transform parent, string childName)
    {
        Transform child = parent.Find(childName);
        if (child)
            return child;

        child = new GameObject(childName).transform;
        child.SetParent(parent, false);
        return child;
    }

    static void ResetGeneratedRootTransform(Transform generatedRoot)
    {
        if (!generatedRoot)
            return;

        generatedRoot.localPosition = Vector3.zero;
        generatedRoot.localRotation = Quaternion.identity;
        generatedRoot.localScale = Vector3.one;
    }

    void ValidateRootScales()
    {
        if ((collisionStageRoot.lossyScale - renderStageRoot.lossyScale).sqrMagnitude <= 0.000001f)
            return;

        Debug.LogWarning("CollisionStageRootとRenderStageRootのScaleが異なります。両方を同じScaleにしてください。", this);
    }

    static void SetLayerRecursively(GameObject root, int layer)
    {
        if (!root || layer < 0)
            return;

        foreach (Transform child in root.GetComponentsInChildren<Transform>(true))
            child.gameObject.layer = layer;
    }

    static void RemoveJointsAndRigidbodies(GameObject root, bool keepColliderInteraction)
    {
        if (!root)
            return;

        foreach (Joint joint in root.GetComponentsInChildren<Joint>(true))
            DestroyComponentSafely(joint);

        /* foreach (Rigidbody body in root.GetComponentsInChildren<Rigidbody>(true))
        {
            body.useGravity = false;
            body.isKinematic = true;
            body.detectCollisions = keepColliderInteraction;
            body.velocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            DestroyComponentSafely(body);
        }*/
    }

    static void DestroyChildren(Transform root)
    {
        if (!root)
            return;

        for (int i = root.childCount - 1; i >= 0; i--)
        {
            GameObject child = root.GetChild(i).gameObject;
            child.SetActive(false);
            DestroyObjectSafely(child);
        }
    }

    static void DestroyComponentSafely(Component component)
    {
        if (!component)
            return;

        if (Application.isPlaying)
            Object.Destroy(component);
        else
            Object.DestroyImmediate(component);
    }

    static void DestroyObjectSafely(Object target)
    {
        if (!target)
            return;

        if (Application.isPlaying)
            Object.Destroy(target);
        else
            Object.DestroyImmediate(target);
    }

    IEnumerator DelayStandOnObject()
    {
        yield return new WaitForSeconds(0.45f);
        for (int i = ActiveSlopeReciver.LimitTouchingphase + FirstShift;
             i < ActiveSlopeReciver.LimitTouchingphase + 8 + FirstShift;
             i++)
        {
            bool DontSeqItem = false;

            GameObject ActiveStairway1 = StackStairway1[i];
            GameObject ActiveStairway2 = StackStairway2[i];

            float angleY = StackStairway1[i].transform.localEulerAngles.y;


            Debug.Log(
                $"[STAIR] " +
                $"index={i}, " +
                $"name={ActiveStairway1.name}, " +
                $"angleY={angleY:F2}°, " +
                $"rotationType={GetRotationType(angleY)}, " +
                $"PhysicsPosition={ActiveStairway1.transform.position}, " +
                $"PhysicsLocalPosition={ActiveStairway1.transform.localPosition}, " +
                $"PhysicsEuler={ActiveStairway1.transform.eulerAngles}, " +
                $"PhysicsLocalEuler={ActiveStairway1.transform.localEulerAngles}, " +
                $"PhysicsForward={ActiveStairway1.transform.forward}, " +
                $"VisualPosition={ActiveStairway2.transform.position}, " +
                $"VisualEuler={ActiveStairway2.transform.eulerAngles}, " +
                $"VisualForward={ActiveStairway2.transform.forward}",
                ActiveStairway1);

            DontSeqItem =
                GeneratePylon(
                    angleY,
                    ActiveStairway1,
                    ActiveStairway2);

            if (!DontSeqItem)
            {
                GenerateCoin(
                    angleY,
                    ActiveStairway1,
                    ActiveStairway2);
            }

        }

        if (ActiveSlopeReciver.LimitTouchingphase < 8)
        {
            FirstShift = 2;
            ActiveSlopeReciver.LimitTouchingphase = 8;
        }

        Debug.Log("");
    }

    static string GetRotationType(float angleY)
    {
        float y =
            Mathf.Repeat(angleY, 360f);

        if (Mathf.Abs(Mathf.DeltaAngle(y, 0f)) < 1f)
            return "Y=0°";

        if (Mathf.Abs(Mathf.DeltaAngle(y, 90f)) < 1f)
            return "Y=90°";

        if (Mathf.Abs(Mathf.DeltaAngle(y, 180f)) < 1f)
            return "Y=180°";

        if (Mathf.Abs(Mathf.DeltaAngle(y, 270f)) < 1f)
            return "Y=270°";

        return $"Y={y:F2}°";
    }

    bool GeneratePylon(
        float angle,
        GameObject ActiveStairway1,
        GameObject ActiveStairway2)
    {
        bool OnPylon = false;

        // 10%で生成
        int value =
            UnityEngine.Random.Range(0, 10);

        if (value < 1)
        {
            OnPylon = true;
        }

        if (!OnPylon)
            return false;

        if (!PylonPrefab ||
            !ActiveStairway1 ||
            !ActiveStairway2)
        {
            return false;
        }

        // =====================================================
        // 階段ローカル座標
        // =====================================================
        //
        // X : 階段の左右
        // Y : 階段面からの高さ
        // Z : 階段の前後
        //
        // 階段中心を必ず (0, 0, 0) とする。
        // =====================================================

        Vector3 localPylonPosition =
            new Vector3(
                -3.3f,
                0.5f,
                0);


        // =====================================================
        // Physics
        // =====================================================

        GameObject StageOnPylon =
            Instantiate(PylonPrefab, ActiveStairway1.transform);

        StageOnPylon.transform.localPosition =
            localPylonPosition;

        StageOnPylon.transform.localRotation =
            Quaternion.Euler(-135f, 0f, 0f);

        StageOnPylon.transform.localScale =
            Vector3.one * 1.5f;


        // =====================================================
        // Visual
        // =====================================================

        GameObject VisualStageOnPylon =
            Instantiate(PylonPrefab, ActiveStairway2.transform);

        VisualStageOnPylon.transform.localPosition =
            localPylonPosition;

        VisualStageOnPylon.transform.localRotation =
            Quaternion.Euler(-135f, 0f, 0f);

        VisualStageOnPylon.transform.localScale =
            Vector3.one * 1.5f;


        // =====================================================
        // Name
        // =====================================================

        StageOnPylon.name =
            $"Thorn_Physics_{StageOnPylon.GetInstanceID()}";

        VisualStageOnPylon.name =
            $"Thorn_Visual_{VisualStageOnPylon.GetInstanceID()}";


        // =====================================================
        // Debug
        // =====================================================

        Debug.Log(
            $"[PYLON LOCAL] " +
            $"stair={ActiveStairway1.name}, " +
            $"angleY={angle:F2}, " +
            $"local={localPylonPosition}, " +
            $"physicsWorld={StageOnPylon.transform.position}, " +
            $"visualWorld={VisualStageOnPylon.transform.position}",
            StageOnPylon);


        return true;
    }

    bool GenerateCoin(
        float angle,
        GameObject ActiveStairway1,
        GameObject ActiveStairway2)
    {
        // 20%で生成
        bool onCoin =
            UnityEngine.Random.Range(0, 10) < 2;

        if (!onCoin)
            return false;

        if (!CoinPrefab ||
            !ActiveStairway1 ||
            !ActiveStairway2)
        {
            return false;
        }

        // =====================================================
        // 横方向
        // =====================================================

        // ShiftCoint は
        //
        // -3
        //  0
        // +3
        //
        // の3レーンなので 0～2 を選ぶ。
        int widthCoin =
            UnityEngine.Random.Range(
                0,
                ShiftCoint.Length);

        float localX =
            ShiftCoint[widthCoin].x;


        // =====================================================
        // Coin数
        // =====================================================

        // 1～4個
        int coinCount =
            UnityEngine.Random.Range(1, 5);


        // =====================================================
        // Coin生成
        // =====================================================

        for (int i = 0; i < coinCount; i++)
        {
            // -------------------------------------------------
            // Coin列全体を階段中心 z=0 の周囲に配置
            // -------------------------------------------------

            float totalLength =
                (coinCount - 1) *
                coinSpacing;

            float startZ =
                -totalLength * 0.5f;

            float localZ =
                startZ +
                coinSpacing * i;


            // =================================================
            // 階段ローカル座標
            // =================================================

            Vector3 localCoinPosition =
                new Vector3(
                    localX,
                    coinLocalHeight + 1.5f,
                    localZ);


            // =================================================
            // Physics
            // =================================================

            GameObject physicsCoin =
                Instantiate(
                    CoinPrefab,
                    ActiveStairway1.transform);

            physicsCoin.transform.localPosition =
                localCoinPosition;

            physicsCoin.transform.localRotation =
                Quaternion.identity;


            // =================================================
            // Visual
            // =================================================

            GameObject visualCoin =
                Instantiate(
                    CoinPrefab,
                    ActiveStairway2.transform);

            visualCoin.transform.localPosition =
                localCoinPosition;

            visualCoin.transform.localRotation =
                Quaternion.identity;


            // =================================================
            // Debug
            // =================================================

            Debug.Log(
                $"[COIN LOCAL] " +
                $"stair={ActiveStairway1.name}, " +
                $"angleY={angle:F2}, " +
                $"coin={i}/{coinCount}, " +
                $"local={localCoinPosition}, " +
                $"physicsWorld={physicsCoin.transform.position}, " +
                $"visualWorld={visualCoin.transform.position}");


            // =================================================
            // Name
            // =================================================

            physicsCoin.name =
                $"Coin_{i}_Physics_{physicsCoin.GetInstanceID()}";

            visualCoin.name =
                $"Coin_{i}_Visual_{visualCoin.GetInstanceID()}";


            // =================================================
            // Physics側は非表示
            // =================================================

            foreach (Renderer renderer in
                     physicsCoin.GetComponentsInChildren<Renderer>(true))
            {
                renderer.enabled = false;
            }


            // =================================================
            // Visual側はCollider無効
            // =================================================

            foreach (Collider collider in
                     visualCoin.GetComponentsInChildren<Collider>(true))
            {
                collider.enabled = false;
            }
        }

        return true;
    }



}
