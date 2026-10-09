using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using TMPro;
using UnityEngine.Splines;
using System.Collections;
using Object = UnityEngine.Object;

//using System;

[DisallowMultipleComponent]
public class CoreStepInsertSplinePathNatural : MonoBehaviour
{
    // Board生成中だけ使う一時ペア。
    // classではなくstructにして、各BoardごとのManaged Object生成を避ける。
    struct BoardPair
    {
        public Transform Physics;
        public Transform Visual;
    }

    [Header("Coin Local Placement")] [SerializeField]
    float coinSpacing = 1.5f;

    [SerializeField] float coinLocalHeight = 0f;
    [SerializeField] NearestKnotDetector knotDetector;
    public SlopeStickCore resumeOnly;

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

    [SerializeField]
    string visualStairwayLayerName = "VisualStairway";

    public bool ReverseSpline;
    public bool FirstVertical;
    public bool StraightStumble;
    public bool PlaneTime;
    public bool cased;
    public bool twistReturn;

    public Dictionary<String, List<GameObject>> RogicalEntity;

    public List<GameObject> ArcSlab1;
    public List<GameObject> ArcSlab2;
    public List<GameObject> StackStairway1;
    public List<GameObject> StackStairway2;

    public GameObject CoinPrefab;
    public GameObject EnemyPrefab;
    //public GameObject PresonerPrefab;

    public GameObject PylonPrefab;

    public int FirstShift = 0;
    public CorrespondSubject reSubject;

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

    static readonly int[] InitialStartPattern =
    {
        -1, 0, -5, 0, -5, 0, -5, 0, -5,
        0, -5, 0, -5, 0, -5, 0,-5
    };

    public int startDashDot = 0;

    List<int> startPattern =
        new List<int>(InitialStartPattern);

    private Vector3[] ShiftObj =
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

    // ============================================================
    // Rolling stage generation
    // ============================================================
    // 初期ステージは InitialStartPattern から自然に生成する。
    // 現在のパターンでは ArcSlab / StairWay とも 0～17 が生成される。
    // プレイヤーが 8, 16, 24... に到達するたび、次チャンクを先行生成する。
    const int ChunkTriggerStep = 8;

    // 名前は startPattern.Count ではなく、実際に生成した次番号を保持して連番にする。
    int nextArcSlabIndex;
    int nextStairwayIndex;

    // ============================================================
    // Item decoration rolling window
    // ============================================================
    // アイテム配置は必ず 8 Stairway 単位の半開区間で進める。
    // [0,8) -> [8,16) -> [16,24) -> ...
    // 前回の終点が、そのまま次回の始点になる。
    const int ItemChunkSize = 8;

    int nextDecorationStartIndex;

    // 同じStairwayでCoin/Pylon抽選を二度行わないための保険。
    // アイテムが「抽選で何も出なかった」場合も処理済みとして記録する。
    readonly HashSet<int> decoratedStairwayInstanceIds =
        new HashSet<int>();

    // FunCharacter は ArcSlab 単位で独立して管理する。
    // Random.Range(0,6) によって Stairway / ArcSlab の生成数が変動しても、
    // 追加チャンクで新しく生成された ArcSlab を漏らさない。
    readonly HashSet<int> decoratedArcSlabInstanceIds =
        new HashSet<int>();

    bool rebuilding = false;

    // 再生成中に旧DelayStandOnObjectが残らないように管理する。
    Coroutine delayStandRoutine;
    Coroutine deathRestartRoutine;

    // ============================================================
    // Runtime caches / reusable buffers
    // ============================================================
    // FixedUpdateごとのFind/GetComponent/文字列再生成を避ける。
    TextMeshProUGUI currentCoinText;
    TextMeshProUGUI currentScoreText;
    TextMeshProUGUI bestScoreText;
    int shownCoin = int.MinValue;
    int shownScore = int.MinValue;
    int shownBest = int.MinValue;

    // GetComponentsInChildren<T>() が返す配列を毎回生成しないための再利用List。
    // Coin/Pylonなど既存の生成処理でも使うため、Renderer/Collider用は残す。
    readonly List<Renderer> rendererBuffer = new List<Renderer>(32);
    readonly List<Collider> colliderBuffer = new List<Collider>(32);

    // Stage Boardの初期化では、Renderer/Collider/Joint/Transformを型別に何度も
    // GetComponentsInChildrenする代わりに、一度だけComponent全体を走査する。
    readonly List<Component> generatedComponentBuffer = new List<Component>(96);

    // TakeBoard()のたびに BoardPair[] と BoardPair class を生成しないための再利用バッファ。
    BoardPair[] boardPairBuffer = new BoardPair[Max];

    // LayerMask.NameToLayer をBoard生成ごとに繰り返さない。
    int slopeLayer = int.MinValue;
    int visualStairwayLayer = int.MinValue;

    void FixedUpdate()
    {

        if (rebuilding)
            return;

        // MainGameManager.Start() の実行順に左右されないよう、
        // 通常プレイ時の最初の先行生成ラインを最低8に保つ。

        if (!MainGameManager.OnDead &&
            MainGameManager.LimitTouchingphase < ChunkTriggerStep)
        {
            MainGameManager.LimitTouchingphase = ChunkTriggerStep;
        }

        // Endless-stage の一般的な先行生成。
        // 0～17を保持した状態で8へ到達したら18以降を生成し、
        // 以後も16,24,32...で次チャンクを足していく。
        if (!MainGameManager.OnDead &&
            !MainGameManager.OpenChunkStage &&
            MainGameManager.lastTouch >= MainGameManager.LimitTouchingphase)
        {
           /*if (!resumeOnly.Earliest)
            return;*/
            MainGameManager.OpenChunkStage = true;
            MainGameManager.LimitTouchingphase += ChunkTriggerStep;
        }
        UpdateUiIfChanged();

        if (MainGameManager.OpenChunkStage)
        {

            MainGameManager.OpenChunkStage = false;
            rebuilding = true;


            Start();

            rebuilding = false;
        }

        if (MainGameManager.OnDead && deathRestartRoutine == null)
        {
            MainGameManager.reSubject.PointToPlane = 0;

            rebuilding = true;
            Start();
            rebuilding = false;
        }
    }

    void UpdateUiIfChanged()
    {
        EnsureUiReferences();

        int coin = MainGameManager.Coin;
        int score = AndroidOneOnly.currentScore;
        int best = AndroidOneOnly.bestScore;

        if (currentCoinText && shownCoin != coin)
        {
            shownCoin = coin;
            currentCoinText.text = coin.ToString();
        }

        if (currentScoreText && shownScore != score)
        {
            shownScore = score;
            currentScoreText.text = score.ToString();
        }

        if (bestScoreText && shownBest != best)
        {
            shownBest = best;
            bestScoreText.text = best.ToString();
        }
    }

    void EnsureUiReferences()
    {
        if (!currentCoinText && MainGameManager.PreviewIconRoot)
        {
            Transform currentCoin =
                MainGameManager.PreviewIconRoot.transform.Find("CurrentCoin");

            if (currentCoin)
            {
                currentCoinText = currentCoin.GetComponent<TextMeshProUGUI>();
                shownCoin = int.MinValue;
            }
        }

        if (!currentScoreText && MainGameManager.TopLiteral)
        {
            Transform score =
                MainGameManager.TopLiteral.transform.Find("Score");

            if (score && score.childCount > 0)
            {
                currentScoreText = score.GetChild(0).GetComponent<TextMeshProUGUI>();
                shownScore = int.MinValue;
            }
        }

        if (!bestScoreText && MainGameManager.TopLiteral)
        {
            Transform best =
                MainGameManager.TopLiteral.transform.Find("Best");

            if (best && best.childCount > 0)
            {
                bestScoreText = best.GetChild(0).GetComponent<TextMeshProUGUI>();
                shownBest = int.MinValue;
            }
        }
    }

    [ContextMenu("RebuildSpline")]
    void Start()
    {
        for (int i = 1; i < InitialStartPattern.Length; i++)
        {
            if (InitialStartPattern[i] != 0)
            {
                startDashDot = i;
                break;
            }

        }
        bool restartingFromDeath =
            MainGameManager.OnDead;

        // ============================================================
        // 死亡再生成時は、旧ステージを参照する遅延処理を先に停止する。
        // ============================================================
        if (restartingFromDeath)
        {
            if (delayStandRoutine != null)
            {
                StopCoroutine(delayStandRoutine);
                delayStandRoutine = null;
            }

            if (deathRestartRoutine != null)
            {
                StopCoroutine(deathRestartRoutine);
                deathRestartRoutine = null;
            }
        }

        // ============================================================
        // 初回 / 死亡再生成時の管理データ初期化
        // ============================================================
        if (RogicalEntity == null ||
            RogicalEntity.Count == 0 ||
            restartingFromDeath)
        {
            RogicalEntity =
                new Dictionary<String, List<GameObject>>();

            if (StackStairway1 == null)
            {
                StackStairway1 = new List<GameObject>();
                ArcSlab1= new List<GameObject>();
            }
            else
            {
                StackStairway1.Clear();
                ArcSlab1.Clear();
            }

            if (StackStairway2 == null)
            {
                StackStairway2 = new List<GameObject>();
                ArcSlab2= new List<GameObject>();

            }
            else
            {
                StackStairway2.Clear();
                ArcSlab2.Clear();
            }



            RogicalEntity["Physics"] = StackStairway1;
            RogicalEntity["Renderer"] = StackStairway2;

            points = null;

            if (outcount == null)
                outcount = new List<int>();
            else
                outcount.Clear();

            startPattern.Clear();
            startPattern.AddRange(InitialStartPattern);

            ContinuousPattern = 0;
            FirstShift = 0;
            nextDecorationStartIndex = 0;
            decoratedStairwayInstanceIds.Clear();
            decoratedArcSlabInstanceIds.Clear();

            GameObject inSubjectObject =
                GameObject.Find("InSubject");

            if (inSubjectObject)
            {
                resumeOnly =
                    inSubjectObject.GetComponent<SlopeStickCore>();
            }
        }

        // ============================================================
        // 死亡後の再構築前にVisual座標系を初期Poseへ戻す。
        // ============================================================
        if (restartingFromDeath)
        {
            if (resumeOnly)
            {
                resumeOnly.PrepareForStageRebuild();
            }
            else
            {
                Debug.LogError(
                    "[STAGE REBUILD] InSubjectのSlopeStickCoreを取得できないため、Visual座標系を復元できません。",
                    this);
            }
        }

        if (!RootInSpiral)
            RootInSpiral = GameObject.Find("StairwaySimple");

        if (!Prepare() || !EnsureOutputRoots())
            return;

        CacheLayers();

        // ============================================================
        // 死亡時：新しいStageを作る「前」に古いStageを消す。
        // ============================================================
        bool addingChunk = false;

        if (restartingFromDeath)
        {
            // プレイヤーもステージも0から再開するため、進行側も同じ基準へ戻す。
            // OnDead中に古いlastTouchがOpenChunkStageを誤発火させることを防ぐ。
            MainGameManager.OpenChunkStage = false;
            MainGameManager.lastTouch = 0;
            MainGameManager.LimitTouchingphase = ChunkTriggerStep;

            FirstShift = 0;
            nextDecorationStartIndex = 0;
            decoratedStairwayInstanceIds.Clear();
            decoratedArcSlabInstanceIds.Clear();

            // 新しいステージの番号も0から振り直す。
            nextArcSlabIndex = 0;
            nextStairwayIndex = 0;

            hasPreviousFlat = false;
            previousFlatPosition = Vector3.zero;
            previousFlatDirection = Vector3.zero;

            ClearGeneratedStage();
            ClearLegacyGeneratedStage();
            ClearSplines();

            CacheTransforms();

            ContinuousPattern = 0;
        }
        // ============================================================
        // 初回生成
        // ============================================================
        else if (outcount.Count == 0)
        {
            hasPreviousFlat = false;
            previousFlatPosition = Vector3.zero;
            previousFlatDirection = Vector3.zero;

            ClearGeneratedStage();
            ClearLegacyGeneratedStage();
            ClearSplines();

            CacheTransforms();

            ContinuousPattern = 0;
            nextArcSlabIndex = 0;
            nextStairwayIndex = 0;
            nextDecorationStartIndex = 0;
            decoratedStairwayInstanceIds.Clear();
            decoratedArcSlabInstanceIds.Clear();

            if (MainGameManager.LimitTouchingphase < ChunkTriggerStep)
                MainGameManager.LimitTouchingphase = ChunkTriggerStep;
        }
        // ============================================================
        // 通常の追加チャンク生成
        // ============================================================
        else
        {
            addingChunk = true;
            ContinuousPattern = startPattern.Count;

            // 追加チャンクは従来どおり9回Random.Range(0, 6)を呼ぶ。
            // 0～5の結果により実際に生成される Stairway / ArcSlab 数が
            // チャンクごとに変わるため、装飾対象は後段で「実際に増えたList範囲」から決める。
            for (int i = 0; i < 9; i++)
            {
                int multiplier = UnityEngine.Random.Range(0, 6);
                startPattern.Add(-multiplier);
            }
        }

        // Emit前の実オブジェクト数を記録する。
        // 追加チャンクではこの位置から後ろだけが今回新しく生成された範囲になる。
        int generatedStairwayStartIndex =
            Mathf.Min(
                StackStairway1 != null ? StackStairway1.Count : 0,
                StackStairway2 != null ? StackStairway2.Count : 0);

        int generatedArcStartIndex =
            Mathf.Min(
                ArcSlab1 != null ? ArcSlab1.Count : 0,
                ArcSlab2 != null ? ArcSlab2.Count : 0);

        EnsureWorkingBuffers();

        ActivePlane = RootStartpoint;

        StraightStumble = false;
        FirstVertical = false;
        ReverseSpline = false;
        ActiveTurnPoint = 0;
        arcSlabCount = 0;
        stairwayCount = 0;

        // ============================================================
        // Spline構造計算 → Stage生成
        // ============================================================
        // Build()はstartPattern全体についてoutcount.Add()するため、
        // List内部配列の段階的な拡張を避けて必要量を先に確保する。
        if (outcount != null)
        {
            int requiredOutcountCapacity =
                outcount.Count + startPattern.Count;

            if (outcount.Capacity < requiredOutcountCapacity)
            {
                outcount.Capacity =
                    Mathf.NextPowerOfTwo(requiredOutcountCapacity);
            }
        }

        Build(0, 0, RootStartpoint);

        for (int i = ContinuousPattern;
             i < startPattern.Count;
             i++)
        {
            Emit(i, i > 0);
        }

        int finalOffset =
            (startPattern.Count - 1) * Max;

        int finalCount =
            counts[startPattern.Count - 1];

        if (PrevInclined == null)
            PrevInclined = new List<Vector3>(Max);
        else
            PrevInclined.Clear();

        for (int i = 0; i < finalCount; i++)
        {
            PrevInclined.Add(
                points[finalOffset + i]);
        }

        accumulatedSpline = lanes[Center];

        // 新しいSplineをNearestKnotDetectorへ確定する。
        if (knotDetector)
            knotDetector.RebuildCache();

        // ============================================================
        // 死亡後：新しく生成済みのArcSlab2_0_PhysicsへInSubjectを戻す。
        // ============================================================
        if (restartingFromDeath)
        {
            deathRestartRoutine =
                StartCoroutine(FinishDeathRestart());
        }

        // ============================================================
        // Coin / Pylon / FunCharacter の装飾範囲を確定する。
        // ============================================================
        // Random.Range(0, 6) × 9 では、1パターンから生成される
        // Stairway / ArcSlab の実数が一定ではない。
        // そのため追加チャンクでは「8個固定」のカーソルではなく、
        // Emit前後のList.Count差分を今回の装飾対象とする。
        int availableStairwayCount =
            Mathf.Min(
                StackStairway1 != null ? StackStairway1.Count : 0,
                StackStairway2 != null ? StackStairway2.Count : 0);

        int availableArcCount =
            Mathf.Min(
                ArcSlab1 != null ? ArcSlab1.Count : 0,
                ArcSlab2 != null ? ArcSlab2.Count : 0);

        int stairwayDecorationStart;
        int stairwayDecorationEndExclusive;
        int arcDecorationStart;
        int arcDecorationEndExclusive;

        if (addingChunk)
        {
            // 追加チャンク: 実際に今回増えたものをすべて対象にする。
            stairwayDecorationStart =
                Mathf.Clamp(generatedStairwayStartIndex, 0, availableStairwayCount);
            stairwayDecorationEndExclusive =
                availableStairwayCount;

            arcDecorationStart =
                Mathf.Clamp(generatedArcStartIndex, 0, availableArcCount);
            arcDecorationEndExclusive =
                availableArcCount;

            // 互換用カーソルも実際の末尾へ追従させる。
            nextDecorationStartIndex =
                stairwayDecorationEndExclusive;
        }
        else
        {
            // 初回 / 死亡再生成は従来仕様を維持する。
            // Coin / Pylon は最初の5 Stairwayを避け、その後最大8個を抽選する。
            stairwayDecorationStart =
                Mathf.Clamp(
                    nextDecorationStartIndex + 5,
                    0,
                    availableStairwayCount);

            stairwayDecorationEndExclusive =
                Mathf.Min(
                    stairwayDecorationStart + ItemChunkSize,
                    availableStairwayCount);

            if (stairwayDecorationStart < stairwayDecorationEndExclusive)
            {
                nextDecorationStartIndex =
                    stairwayDecorationEndExclusive;
            }

            // FunCharacter は従来と同様、初回に生成済みArcSlab全体を抽選する。
            arcDecorationStart = 0;
            arcDecorationEndExclusive = availableArcCount;
        }

        if (stairwayDecorationStart < stairwayDecorationEndExclusive ||
            arcDecorationStart < arcDecorationEndExclusive)
        {
            delayStandRoutine =
                StartCoroutine(
                    DelayDecorateGeneratedRanges(
                        stairwayDecorationStart,
                        stairwayDecorationEndExclusive,
                        arcDecorationStart,
                        arcDecorationEndExclusive));
        }
    }

    IEnumerator FinishDeathRestart()
    {
        // この時点では新しい __GeneratedPhysics / ArcSlab2_0_Physics が
        // すでに生成済みであることが前提。
        if (resumeOnly)
        {
            yield return StartCoroutine(
                resumeOnly.delayStart());
        }
        else
        {
            yield return null;
        }

        MainGameManager.OnDead = false;
        deathRestartRoutine = null;

        Debug.Log(
            "[STAGE REBUILD COMPLETE] 死亡後のステージ再生成が完了しました。",
            this);
    }

    void EnsureWorkingBuffers()
    {
        int requiredCount = startPattern.Count;
        int currentCapacity = counts != null ? counts.Length : 0;

        if (currentCapacity >= requiredCount &&
            points != null && points.Length >= requiredCount * Max &&
            scales != null && scales.Length >= requiredCount)
        {
            return;
        }

        int newCapacity =
            Mathf.NextPowerOfTwo(Mathf.Max(16, requiredCount));

        Array.Resize(ref points, newCapacity * Max);
        Array.Resize(ref counts, newCapacity);
        Array.Resize(ref scales, newCapacity);
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
        Vector3 worldStart, Vector3 worldDirection, 
        bool slope, int scale, int plan, int segmentIndex)
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
            float robustDistance = 0f;

            // ========================================================
            // 前回の Flat と比較
            // ========================================================

            if (hasPreviousFlat)
            {
                Vector3 delta =
                    currentFlatPosition -
                    previousFlatPosition;

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

            int generatedArcSlabIndex =
                nextArcSlabIndex++;

            arcSlabCount++;

            int arcSlabCountForPose =
                TakeBoard(
                    false,
                    slopeLayer,
                    $"ArcSlab{generatedArcSlabIndex}",
                    scale);

            ApplyBoardPose(
                boardPairBuffer,
                arcSlabCountForPose,
                localPosition,
                localDirection,
                localRotation,
                localScale,
                shiftHalf);

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
            int generatedStairwayIndex =
                nextStairwayIndex++;

            stairwayCount++;

            int stairwayCountForPose =
                TakeBoard(
                    true,
                    slopeLayer,
                    $"StairWay{generatedStairwayIndex}",
                    scale);

            // Slope 側には Flat 用の半区間補正を掛けない。
            ApplyBoardPose(
                boardPairBuffer,
                stairwayCountForPose,
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

    int TakeBoard(bool slope, int physicsLayer, string boardName, int mulPlane)
    {
        EnsureBoardPairBuffer(mulPlane);

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
                ArcSlab1.Add(physicsObject);
                ArcSlab2.Add(visualObject);
            }

            physicsObject.name = $"{boardName}_{i}_Physics";
            visualObject.name = $"{boardName}_{i}_Render";

            ConfigurePhysicsRepresentation(physicsObject, physicsLayer);
            ConfigureVisualRepresentation(visualObject, slope);

            boardPairBuffer[i].Physics = physicsObject.transform;
            boardPairBuffer[i].Visual = visualObject.transform;
        }

        return mulPlane;
    }

    void EnsureBoardPairBuffer(int requiredCount)
    {
        if (boardPairBuffer.Length >= requiredCount)
            return;

        int newCapacity =
            Mathf.NextPowerOfTwo(Mathf.Max(Max, requiredCount));

        Array.Resize(ref boardPairBuffer, newCapacity);
    }

    void ConfigurePhysicsRepresentation(GameObject physicsObject, int physicsLayer)
    {
        if (!physicsObject)
            return;

        if (physicsLayer < 0)
            Debug.LogError("SlopeまたはStairway Layerが見つかりません。", this);

        bool hasCollider = false;

        // 以前は Transform / Renderer / Joint / Collider を別々に検索していた。
        // ここではHierarchyを一度だけ走査し、最終状態を同じまま適用する。
        generatedComponentBuffer.Clear();
        physicsObject.GetComponentsInChildren(true, generatedComponentBuffer);

        for (int i = 0; i < generatedComponentBuffer.Count; i++)
        {
            Component component = generatedComponentBuffer[i];
            if (!component)
                continue;

            if (physicsLayer >= 0 && component is Transform targetTransform)
                targetTransform.gameObject.layer = physicsLayer;

            if (hidePhysicsRenderers && component is Renderer renderer)
                renderer.enabled = false;

            if (component is Collider)
                hasCollider = true;

            if (removeGeneratedRigidbodies && component is Joint joint)
                DestroyComponentSafely(joint);
        }

        if (!hasCollider)
            Debug.LogWarning($"{physicsObject.name}にColliderがありません。", physicsObject);
    }
    void ConfigureVisualRepresentation(
        GameObject visualObject,
        bool inSlope)
    {
        if (!visualObject)
            return;

        // 元コードと同じく、Visual Stairway Layerが無い場合は
        // Joint除去やMaterial適用まで進まず、その場で終了する。
        if (inSlope && visualStairwayLayer < 0)
        {
            Debug.LogError(
                $"Visual Stairway Layer '{visualStairwayLayerName}' が存在しません。",
                visualObject);

            return;
        }

        bool applyMaterials =
            materials != null && materials.Length > 0;

        // Collider / Transform / Joint / MeshRenderer の探索を1回に統合する。
        generatedComponentBuffer.Clear();
        visualObject.GetComponentsInChildren(true, generatedComponentBuffer);

        for (int i = 0; i < generatedComponentBuffer.Count; i++)
        {
            Component component = generatedComponentBuffer[i];
            if (!component)
                continue;

            if (!inSlope && component is Collider collider)
                collider.enabled = false;

            if (inSlope && component is Transform targetTransform)
                targetTransform.gameObject.layer = visualStairwayLayer;

            if (removeGeneratedRigidbodies && component is Joint joint)
                DestroyComponentSafely(joint);

            if (applyMaterials && component is MeshRenderer meshRenderer)
                meshRenderer.sharedMaterials = materials;
        }
    }

    void ApplyBoardPose(
        BoardPair[] pairs,
        int pairCount,
        Vector3 localPosition,
        Vector3 localDirection,
        Quaternion localRotation,
        Vector3 localScale,
        bool shiftHalf)
    {
        if (pairs == null || pairCount <= 0)
            return;

        // ============================================================
        // 複数枚
        // ============================================================

        if (pairCount > 1)
        {
            Vector3 totalVec =
                Vector3.zero;

            // 同じlocalDirectionをループ内で何度もnormalizeしない。
            Vector3 normalizedLocalDirection =
                localDirection.normalized;

            for (int i = 0; i < pairCount; i++)
            {
                BoardPair pair =
                    pairs[i];

                totalVec +=
                    normalizedLocalDirection *
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
       // if (!RootInSpiral)
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

    void CacheLayers()
    {
        slopeLayer = LayerMask.NameToLayer("Slope");
        visualStairwayLayer = LayerMask.NameToLayer(visualStairwayLayerName);
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

    IEnumerator DelayDecorateGeneratedRanges(
        int stairwayStartIndex,
        int stairwayEndIndexExclusive,
        int arcStartIndex,
        int arcEndIndexExclusive)
    {
        yield return new WaitForSeconds(0.1f);

        int stairwayAvailableCount =
            Mathf.Min(
                StackStairway1 != null ? StackStairway1.Count : 0,
                StackStairway2 != null ? StackStairway2.Count : 0);

        int safeStairwayStart =
            Mathf.Clamp(
                stairwayStartIndex,
                0,
                stairwayAvailableCount);

        int safeStairwayEndExclusive =
            Mathf.Clamp(
                stairwayEndIndexExclusive,
                safeStairwayStart,
                stairwayAvailableCount);

        bool arcRangeProcessed = false;

        for (int i = safeStairwayStart;
             i < safeStairwayEndExclusive;
             i++)
        {
            GameObject activeStairway1 =
                StackStairway1[i];

            GameObject activeStairway2 =
                StackStairway2[i];

            if (!activeStairway1 || !activeStairway2)
                continue;

            int stairwayInstanceId =
                activeStairway1.GetInstanceID();

            if (!decoratedStairwayInstanceIds.Add(stairwayInstanceId))
                continue;

            bool firstCorner =
                i > startDashDot;

            float angleY =
                activeStairway1.transform.localEulerAngles.y;

            bool dontSeqItem =
                GeneratePylon(
                    angleY,
                    activeStairway1,
                    activeStairway2,
                    firstCorner);

            // 正常動作時のRandom呼び出し順に近づけるため、
            // 最初の有効StairwayのPylon抽選直後にFunCharacterを処理する。
            // Stairwayが0個のチャンクでも、ループ後に必ずArc側を処理する。
            if (!arcRangeProcessed)
            {
                DecorateArcRange(
                    arcStartIndex,
                    arcEndIndexExclusive);

                arcRangeProcessed = true;
            }

            if (!dontSeqItem)
            {
                GenerateCoin(
                    angleY,
                    activeStairway1,
                    activeStairway2,
                    firstCorner);
            }
        }

        if (!arcRangeProcessed)
        {
            DecorateArcRange(
                arcStartIndex,
                arcEndIndexExclusive);
        }

        delayStandRoutine = null;
    }

    void DecorateArcRange(
        int startIndex,
        int endIndexExclusive)
    {
        int availableCount =
            Mathf.Min(
                ArcSlab1 != null ? ArcSlab1.Count : 0,
                ArcSlab2 != null ? ArcSlab2.Count : 0);

        int safeStart =
            Mathf.Clamp(
                startIndex,
                0,
                availableCount);

        int safeEndExclusive =
            Mathf.Clamp(
                endIndexExclusive,
                safeStart,
                availableCount);

        // 従来のGeneratePylon内と同じく、新しい側から逆順に抽選する。
        for (int i = safeEndExclusive - 1;
             i >= safeStart;
             i--)
        {
            GameObject physicsArc =
                ArcSlab1[i];

            GameObject visualArc =
                ArcSlab2[i];

            if (!physicsArc || !visualArc)
                continue;

            int arcInstanceId =
                physicsArc.GetInstanceID();

            if (!decoratedArcSlabInstanceIds.Add(arcInstanceId))
                continue;

            GenerateFunCharacter(
                physicsArc.transform.localEulerAngles.y,
                physicsArc,
                visualArc,
                i > startDashDot);
        }
    }

    IEnumerator DelayStandSlopeOnObject(
        int startIndex,
        int endIndexExclusive)
    {
        yield return new WaitForSeconds(0.1f);

        int physicsCount =
            StackStairway1 != null
                ? StackStairway1.Count
                : 0;

        int visualCount =
            StackStairway2 != null
                ? StackStairway2.Count
                : 0;

        int availableCount =
            Mathf.Min(
                physicsCount,
                visualCount);

        // Coroutine開始後に死亡/再構築が入ってListが短くなっても
        // 範囲外へ出ないよう、現在のCountで終端をもう一度丸める。
        int safeStart =
            Mathf.Clamp(
                startIndex,
                0,
                availableCount);

        int safeEndExclusive =
            Mathf.Clamp(
                endIndexExclusive,
                safeStart,
                availableCount);

        if (safeStart >= safeEndExclusive)
        {
            delayStandRoutine = null;
            yield break;
        }

        for (int i = safeStart;
             i < safeEndExclusive;
             i++)
        {
            GameObject ActiveStairway1 =
                StackStairway1[i];

            GameObject ActiveStairway2 =
                StackStairway2[i];

            if (!ActiveStairway1 ||
                !ActiveStairway2)
            {
                continue;
            }

            // ========================================================
            // Idempotency:
            // 同じPhysics Stairwayに対するアイテム抽選は一度だけ。
            //
            // GenerateCoin/GeneratePylonが「今回は生成なし」を返した場合も
            // このStairwayは処理済みにする。再抽選すると、Stage更新のたびに
            // 後からCoin/Pylonが増えて重複の原因になるため。
            // ========================================================
            int stairwayInstanceId =
                ActiveStairway1.GetInstanceID();

            if (!decoratedStairwayInstanceIds.Add(
                    stairwayInstanceId))
            {
                Debug.LogWarning(
                    $"[ITEM CHUNK SKIP] " +
                    $"already processed index={i}, " +
                    $"stairway={ActiveStairway1.name}",
                    ActiveStairway1);

                continue;
            }

            bool FirstCorner=i > startDashDot;

            float angleY =
                ActiveStairway1
                    .transform
                    .localEulerAngles
                    .y;

            bool DontSeqItem =
                GeneratePylon(
                    angleY,
                    ActiveStairway1,
                    ActiveStairway2,FirstCorner);

            if (!DontSeqItem)
            {
                GenerateCoin(
                    angleY,
                    ActiveStairway1,
                    ActiveStairway2,FirstCorner);
            }
        }

        delayStandRoutine = null;
    }
    IEnumerator DelayStandPlaneOnObject(
        int startIndex,
        int endIndexExclusive)
    {
        yield return new WaitForSeconds(0.1f);

        int physicsCount =
            StackStairway1 != null
                ? StackStairway1.Count
                : 0;

        int visualCount =
            StackStairway2 != null
                ? StackStairway2.Count
                : 0;

        int availableCount =
            Mathf.Min(
                physicsCount,
                visualCount);

        // Coroutine開始後に死亡/再構築が入ってListが短くなっても
        // 範囲外へ出ないよう、現在のCountで終端をもう一度丸める。
        int safeStart =
            Mathf.Clamp(
                startIndex,
                0,
                availableCount);

        int safeEndExclusive =
            Mathf.Clamp(
                endIndexExclusive,
                safeStart,
                availableCount);

        if (safeStart >= safeEndExclusive)
        {
            delayStandRoutine = null;
            yield break;
        }

        for (int i = safeStart;
             i < safeEndExclusive;
             i++)
        {
            GameObject ActiveStairway1 =
                StackStairway1[i];

            GameObject ActiveStairway2 =
                StackStairway2[i];

            if (!ActiveStairway1 ||
                !ActiveStairway2)
            {
                continue;
            }

            // ========================================================
            // Idempotency:
            // 同じPhysics Stairwayに対するアイテム抽選は一度だけ。
            //
            // GenerateCoin/GeneratePylonが「今回は生成なし」を返した場合も
            // このStairwayは処理済みにする。再抽選すると、Stage更新のたびに
            // 後からCoin/Pylonが増えて重複の原因になるため。
            // ========================================================
            int stairwayInstanceId =
                ActiveStairway1.GetInstanceID();

            if (!decoratedStairwayInstanceIds.Add(
                    stairwayInstanceId))
            {
                Debug.LogWarning(
                    $"[ITEM CHUNK SKIP] " +
                    $"already processed index={i}, " +
                    $"stairway={ActiveStairway1.name}",
                    ActiveStairway1);

                continue;
            }

            bool FirstCorner=i > startDashDot;

            float angleY =
                ActiveStairway1
                    .transform
                    .localEulerAngles
                    .y;

            bool DontSeqItem =
                GeneratePylon(
                    angleY,
                    ActiveStairway1,
                    ActiveStairway2,FirstCorner);

            if (!DontSeqItem)
            {
                GenerateCoin(
                    angleY,
                    ActiveStairway1,
                    ActiveStairway2,FirstCorner);
            }
        }

        delayStandRoutine = null;
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

    // Coin / Pylon / FunCharacter の共通となる「1体分」の生成・姿勢設定。
    // localScale == null はPrefab由来のScaleを維持する（Coin用）。
    GameObject SpawnOne(
        GameObject prefab,
        GameObject parent,
        Vector3 localPosition,
        Quaternion localRotation,
        Vector3? localScale,
        string namePrefix,
        bool disableColliders)
    {
        GameObject instance = Instantiate(prefab, parent.transform);
        instance.transform.localPosition = localPosition;
        instance.transform.localRotation = localRotation;

        if (localScale.HasValue)
            instance.transform.localScale = localScale.Value;

        instance.name = $"{namePrefix}_{instance.GetInstanceID()}";

        if (namePrefix.IndexOf("FunChr", StringComparison.Ordinal) < 0 &&
            disableColliders)
        {
            colliderBuffer.Clear();
            instance.GetComponentsInChildren(true, colliderBuffer);

            for (int i = 0; i < colliderBuffer.Count; i++)
                colliderBuffer[i].enabled = false;
        }


        return instance;
    }

    // Coin / Pylon は従来どおりPhysicsとVisualの両方を生成する。
    (GameObject physics, GameObject visual) SpawnPair(
        GameObject prefab,
        GameObject physicsParent,
        GameObject visualParent,
        Vector3 localPosition,
        Quaternion localRotation,
        Vector3? localScale,
        string namePrefix,
        bool separateVisualPhysics)
    {
        GameObject physics = SpawnOne(
            prefab, physicsParent, localPosition, localRotation, localScale,
            namePrefix + "_Physics", false);

        GameObject visual = SpawnOne(
            prefab, visualParent, localPosition, localRotation, localScale,
            namePrefix + "_Visual", separateVisualPhysics);

        if (separateVisualPhysics)
        {
            rendererBuffer.Clear();
            physics.GetComponentsInChildren(true, rendererBuffer);

            for (int i = 0; i < rendererBuffer.Count; i++)
                rendererBuffer[i].enabled = false;
        }

        return (physics, visual);
    }

    // FunCharacter専用: Render側だけに生成し、Colliderを無効にする。
    GameObject SpawnVisual(
        GameObject prefab,
        GameObject visualParent,
        Vector3 localPosition,
        Quaternion localRotation,
        Vector3? localScale,
        string namePrefix)
    {
        return SpawnOne(
            prefab, visualParent, localPosition, localRotation, localScale,
            namePrefix + "_Visual", true);
    }

    bool GeneratePylon(
        float angle,
        GameObject ActiveStairway1,
        GameObject ActiveStairway2,
        bool FirstCorner)
    {
        // Pylon 10%。従来の抽選順序・戻り値を維持する。
        int value = UnityEngine.Random.Range(0, 10);
        int widthObj = FirstCorner
            ? UnityEngine.Random.Range(0, ShiftObj.Length)
            : (UnityEngine.Random.Range(0, 2) == 0 ? 0 : 2);
        float localX = ShiftObj[widthObj].x;
        bool onPylon = value < 1;
        bool generated = false;

        if (onPylon && PylonPrefab && ActiveStairway1 && ActiveStairway2)
        {
            Vector3 localPylonPosition = new Vector3(localX, 0.5f, 0f);
            var pair = SpawnPair(
                PylonPrefab,
                ActiveStairway1,
                ActiveStairway2,
                localPylonPosition,
                Quaternion.Euler(-135f, 0f, 0f),
                Vector3.one * 1.5f,
                "Thorn",
                false);


            // Physics側のRendererだけ無効化
            rendererBuffer.Clear();
            pair.physics.GetComponentsInChildren(true, rendererBuffer);

            for (int i = 0; i < rendererBuffer.Count; i++)
                rendererBuffer[i].enabled = false;

            generated = true;
        }

        // 以前の正常系と同じ入口を残す。
        // Pylon抽選の直後に、末尾へ追加された未処理ArcSlabだけを新しい側から処理する。
        // decoratedArcSlabInstanceIds により、DelayDecorateGeneratedRanges() 側の
        // DecorateArcRange() と二重抽選になることはない。
        DecoratePendingArcSlabs();

        return generated;
    }

    void DecoratePendingArcSlabs()
    {
        int availableArcCount =
            Mathf.Min(
                ArcSlab1 != null ? ArcSlab1.Count : 0,
                ArcSlab2 != null ? ArcSlab2.Count : 0);

        // ArcSlabは末尾へ追加されるため、新しい側から戻り、
        // 最初の処理済みArcSlabに到達した時点で終了する。
        for (int i = availableArcCount - 1; i >= 0; i--)
        {
            GameObject physicsArc = ArcSlab1[i];
            GameObject visualArc = ArcSlab2[i];

            if (!physicsArc || !visualArc)
                continue;

            int arcInstanceId = physicsArc.GetInstanceID();

            if (!decoratedArcSlabInstanceIds.Add(arcInstanceId))
                break;

            GenerateFunCharacter(
                physicsArc.transform.localEulerAngles.y,
                physicsArc,
                visualArc,
                i > startDashDot);
        }
    }


    bool GenerateCoin(float angle, GameObject ActiveStairway1, GameObject ActiveStairway2, bool FirstConner)
    {
        // Pylonが配置されなかったStairwayに対し20%で抽選。
        bool onCoin = UnityEngine.Random.Range(0, 10) < 2;
        if (!onCoin)
            return false;

        if (!CoinPrefab || !ActiveStairway1 || !ActiveStairway2)
            return false;

        int widthObj;
        if (FirstConner)
            widthObj = UnityEngine.Random.Range(0, ShiftObj.Length);
        else
            widthObj = UnityEngine.Random.Range(0, 2) == 0 ? 0 : 2;
        float localX = ShiftObj[widthObj].x;

        int coinCount = UnityEngine.Random.Range(1, 5);
        for (int i = 0; i < coinCount; i++)
        {
            float totalLength = (coinCount - 1) * coinSpacing;
            float startZ = -totalLength * 0.5f;
            float localZ = startZ + coinSpacing * i;
            Vector3 localCoinPosition = new Vector3(localX, coinLocalHeight + 1.5f, localZ);

            SpawnPair(
                CoinPrefab,
                ActiveStairway1,
                ActiveStairway2,
                localCoinPosition,
                Quaternion.identity,
                null, // Coinは従来どおりPrefab自身のScaleを使う。
                $"Coin_{i}",
                true);
        }

        return true;
    }
    bool GenerateFunCharacter(
        float angle,
        GameObject physicsArc,
        GameObject visualArc,
        bool FirstCorner)
    {
        // ArcSlabごとに40%で抽選。Pylon/Coinの成否には依存しない。
        if (UnityEngine.Random.Range(0, 10) >= 4)
            return false;

        if (!EnemyPrefab || !physicsArc || !visualArc)
            return false;

        int widthObj = FirstCorner
            ? UnityEngine.Random.Range(0, ShiftObj.Length)
            : (UnityEngine.Random.Range(0, 2) == 0 ? 0 : 2);
        Vector3 localPosition = new Vector3(ShiftObj[widthObj].x, 0.125f, 0f);

        GameObject visualCharacter = SpawnVisual(
            EnemyPrefab,
            visualArc,
            localPosition,
            // 正常に表示されていた従来姿勢へ戻す。
            Quaternion.Euler(-135f, 0f, 0f),
            Vector3.one * 1.5f,
            "FunChr");

        return true;
    }

}
