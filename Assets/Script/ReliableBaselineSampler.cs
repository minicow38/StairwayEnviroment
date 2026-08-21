using UnityEngine;
using System.IO;

/// <summary>
/// 3つの意味の異なる局面をパズルピースとして回収します。
///
/// 1. InitialGroundedRest : 最初に地面へ着いて静止した状態（複数サンプルの平均）
/// 2. AbruptNormalChange  : 法線急変を検出した瞬間
/// 3. BeforeDOTweenTurn   : DOTween回転開始直前
///
/// 保存は1つのJSONファイルに対して2段階で行います。
/// A. InitialGroundedRest完成時に、初回静止だけを含む仮JSONを新規生成します。
/// B. 残り2ピースがそろい、完成サイクルの検証に合格した時に、同じJSONを完成版で上書きします。
///
/// 起動時点ですでに保存ファイルが存在した場合、その実行中はファイルを追加・上書き・退避しません。
/// 既存ファイルを作り直す場合は、ContextMenuの Delete Saved Baseline File And Resample を明示的に実行します。
/// Rigidbodyへ力・速度・位置を加える処理は行いません。
/// </summary>
[DisallowMultipleComponent]
public sealed class ReliableBaselineSampler : MonoBehaviour
{
    [Header("Initial Grounded Rest")]
    [Tooltip("初回接地後、着地衝撃を逃がすために待つFixedUpdate数です。")]
    [SerializeField, Range(1, 30)] int settleFrames = 4;

    [Tooltip("同じCollider・安定法線が連続して必要なフレーム数です。")]
    [SerializeField, Range(1, 30)] int stableFramesRequired = 3;

    [Tooltip("前回法線との差がこの角度以内なら安定と判定します。")]
    [SerializeField, Range(.05f, 10f)] float normalToleranceDegrees = 1f;

    [Tooltip("初回静止ピースを作るために採用するサンプル数です。")]
    [SerializeField, Range(5, 60)] int requiredRestSampleCount = 30;

    [Tooltip("初回静止サンプルとして採用する法線速度の絶対値上限です。")]
    [SerializeField, Min(.01f)] float restNormalSpeedLimit = .5f;

    [Header("Initial Rest Robust Average")]
    [Tooltip("初回静止値の上下から除外する割合です。")]
    [SerializeField, Range(0f, .4f)] float trimRatio = .2f;

    [SerializeField, Min(.001f)] float restForwardStdLimit = .35f;
    [SerializeField, Min(.001f)] float restNormalStdLimit = .2f;
    [SerializeField, Min(.001f)] float restRollingStdLimit = .75f;

    [Header("Completed Cycle Validation")]
    [Tooltip("3ピースのベクトルにNaN/Infinityがあればサイクルを破棄します。")]
    [SerializeField] bool rejectNonFiniteValues = true;

    [Tooltip("法線急変とDOTween回転直前の保存時刻の最大差です。0以下なら検査しません。")]
    [SerializeField, Min(0f)] float maxNormalToTurnSeconds = 30f;

    [Header("Persistent Baseline File")]
    [Tooltip("Application.persistentDataPath内に生成するJSONファイル名です。初回静止の仮保存と完成版で同じファイルを使用します。")]
    [SerializeField] string savedFileName = "ReliableBaselineCycle.json";

    [Tooltip("保存ファイルの絶対パスと、起動時の書き込みロック状態をConsoleへ表示します。")]
    [SerializeField] bool logSavedFilePath = true;

    [Header("Debug")]
    [SerializeField] bool debugSampling = true;

    public enum SamplingPhase
    {
        WaitingForFirstLanding,
        Settling,
        SamplingInitialRest,
        CollectingCyclePieces
    }

    public enum CyclePieceType
    {
        InitialGroundedRest,
        AbruptNormalChange,
        BeforeDOTweenTurn
    }

    public enum SavedFileStatus
    {
        Missing = 0,
        Empty = 1,
        Valid = 2,
        InvalidJson = 3,
        InvalidSchema = 4,
        InvalidCycle = 5,
        IoError = 6,
        InitialRestOnly = 7
    }

    public enum SavedDataStage
    {
        None = 0,
        InitialRestOnly = 1,
        CompletedCycle = 2
    }

    [System.Serializable]
    public struct BaselineResult
    {
        public bool valid;
        public Vector3 velocity;
        public Vector3 angularVelocity;
        public Vector3 normal;
        public Vector3 heading;
        public Vector3 side;
        public float forwardSpeed;
        public float lateralSpeed;
        public float normalSpeed;
        public float rollingAngularSpeed;
        public float yawAngularSpeed;
        public float headingAngularSpeed;
        public float forwardStd;
        public float normalStd;
        public float rollingStd;
        public int sampleCount;
        public Collider sourceCollider;
        public float savedTime;
    }

    [System.Serializable]
    public struct CompletedCycleResult
    {
        public bool valid;
        public int cycleNumber;
        public BaselineResult initialGroundedRest;
        public BaselineResult abruptNormalChange;
        public BaselineResult beforeDOTweenTurn;
        public float completedTime;
    }

    // Collider参照はJSONへ保存できないため、ファイル用DTOでは除外する。
    [System.Serializable]
    struct SavedBaselineResult
    {
        public bool valid;
        public Vector3 velocity;
        public Vector3 angularVelocity;
        public Vector3 normal;
        public Vector3 heading;
        public Vector3 side;
        public float forwardSpeed;
        public float lateralSpeed;
        public float normalSpeed;
        public float rollingAngularSpeed;
        public float yawAngularSpeed;
        public float headingAngularSpeed;
        public float forwardStd;
        public float normalStd;
        public float rollingStd;
        public int sampleCount;
        public float savedTime;
    }

    [System.Serializable]
    struct SavedCycleFile
    {
        public int formatVersion;
        public bool valid;
        public SavedDataStage dataStage;
        public int trustedCycleCount;
        public float createdRealtime;
        public float updatedRealtime;
        public SavedBaselineResult initialGroundedRest;
        public SavedBaselineResult abruptNormalChange;
        public SavedBaselineResult beforeDOTweenTurn;
        public float completedTime;
    }

    struct RestSample
    {
        public Vector3 normal;
        public Vector3 heading;
        public float forwardSpeed;
        public float lateralSpeed;
        public float normalSpeed;
        public float rollingAngularSpeed;
        public float yawAngularSpeed;
        public float headingAngularSpeed;
    }

    readonly System.Collections.Generic.List<RestSample> restSamples = new System.Collections.Generic.List<RestSample>(60);

    SamplingPhase phase = SamplingPhase.WaitingForFirstLanding;

    BaselineResult permanentInitialRest;
    bool hasPermanentInitialRest;

    BaselineResult candidateAbruptNormal;
    BaselineResult candidateBeforeTurn;
    bool hasCandidateAbruptNormal;
    bool hasCandidateBeforeTurn;

    CompletedCycleResult trustedCycle;
    int trustedCycleCount;

    bool attemptedInitialFileLoad;
    bool loadedFromFile;
    bool fileExistedAtStartup;
    bool fileWriteLocked;
    bool initialRestFileCreatedThisRun;
    float initialRestFileCreatedRealtime;
    SavedFileStatus lastSavedFileStatus = SavedFileStatus.Missing;

    int settleFramesRemaining;
    int stableFrameCount;
    Vector3 previousNormal = Vector3.up;
    Collider sampledCollider;

    public SamplingPhase Phase => phase;

    // SlopeStickBall3D側から初回静止サンプリング数を確認するための公開値。
    public int InitialRestSampleCount => restSamples.Count;
    public int RequiredInitialRestSampleCount => requiredRestSampleCount;

    // 必要数へ初めて到達したことを一度だけ記録するフラグ。
    public bool FirstSamplingInCollect { get; set; }

    public bool HasInitialGroundedRest => hasPermanentInitialRest;
    public bool HasPermanentInitialRest => hasPermanentInitialRest;
    public bool HasTrustedCycle => trustedCycle.valid;
    public int TrustedCycleCount => trustedCycleCount;
    public CompletedCycleResult TrustedCycle => trustedCycle;

    public bool HasAbruptNormalPiece => hasCandidateAbruptNormal;
    public bool HasBeforeTurnPiece => hasCandidateBeforeTurn;

    public string SavedFilePath => Path.Combine(Application.persistentDataPath, SanitizeFileName(savedFileName));

    public bool SavedFileExists => File.Exists(SavedFilePath);
    public bool LoadedFromFile => loadedFromFile;
    public bool FileExistedAtStartup => fileExistedAtStartup;
    public bool FileWriteLocked => fileWriteLocked;
    public bool InitialRestFileCreatedThisRun => initialRestFileCreatedThisRun;
    public SavedFileStatus LastSavedFileStatus => lastSavedFileStatus;
    public bool HasUsableSavedFile => loadedFromFile && trustedCycle.valid && lastSavedFileStatus == SavedFileStatus.Valid;

    void Awake()
    {
        EnsureSavedFileLoaded(true);

        if (logSavedFilePath)
        {
            Debug.Log(
                $"[Baseline File Path] {SavedFilePath} " +
                $"existedAtStartup={fileExistedAtStartup} writeLocked={fileWriteLocked} " +
                $"status={lastSavedFileStatus}",
                this);
        }
    }

    void OnValidate()
    {
        settleFrames = Mathf.Clamp(settleFrames, 1, 30);
        stableFramesRequired = Mathf.Clamp(stableFramesRequired, 1, 30);
        normalToleranceDegrees = Mathf.Clamp(normalToleranceDegrees, .05f, 10f);
        requiredRestSampleCount = Mathf.Clamp(requiredRestSampleCount, 5, 60);
        restNormalSpeedLimit = Mathf.Max(.01f, restNormalSpeedLimit);
        trimRatio = Mathf.Clamp(trimRatio, 0f, .4f);
        restForwardStdLimit = Mathf.Max(.001f, restForwardStdLimit);
        restNormalStdLimit = Mathf.Max(.001f, restNormalStdLimit);
        restRollingStdLimit = Mathf.Max(.001f, restRollingStdLimit);
        maxNormalToTurnSeconds = Mathf.Max(0f, maxNormalToTurnSeconds);

        if (string.IsNullOrWhiteSpace(savedFileName))
            savedFileName = "ReliableBaselineCycle.json";

        if (!savedFileName.EndsWith(".json", System.StringComparison.OrdinalIgnoreCase))
            savedFileName += ".json";
    }

    /// <summary>
    /// 初回静止ピースが完成するまでSlopeStickBall3D.FixedUpdateから呼びます。
    /// trueの間は初期サンプリングを続行します。
    /// 初回静止ピース完成後はfalseを返し、以後の2ピースはCaptureCyclePieceで回収します。
    /// </summary>
    public bool Tick(Rigidbody rb, bool isGrounded, RaycastHit hit, Vector3 requestedHeading, Collider currentCollider, out BaselineResult completedResult)
    {
        return Tick(
            rb,
            isGrounded,
            hit.collider ? hit.normal : Vector3.up,
            requestedHeading,
            currentCollider ? currentCollider : hit.collider,
            out completedResult);
    }

    public bool Tick(
        Rigidbody rb,
        bool isGrounded,
        Vector3 surfaceNormal,
        Vector3 requestedHeading,
        Collider currentCollider,
        out BaselineResult completedResult)
    {
        EnsureSavedFileLoaded(false);
        completedResult = permanentInitialRest;

        // 完成済みファイルを読み込んだ場合は新しい測定を行わない。
        if (loadedFromFile && trustedCycle.valid)
            return false;

        if (hasPermanentInitialRest)
            return false;

        if (!rb)
        {
            ResetInitialRestSampling();
            return true;
        }

        if (!isGrounded || !currentCollider)
        {
            ResetInitialRestSampling();
            return true;
        }

        Vector3 currentNormal = NormalizeOrFallback(surfaceNormal, Vector3.up);

        if (phase == SamplingPhase.WaitingForFirstLanding)
        {
            phase = SamplingPhase.Settling;
            settleFramesRemaining = settleFrames;
            stableFrameCount = 1;
            previousNormal = currentNormal;
            sampledCollider = currentCollider;
            restSamples.Clear();

            Log($"[Baseline Cycle Initial Rest Landing] collider={currentCollider.name} " +
                $"settle={settleFrames} samples={requiredRestSampleCount}");
            return true;
        }

        bool sameCollider = sampledCollider == currentCollider;
        float normalDelta = Vector3.Angle(previousNormal, currentNormal);
        bool stableNormal = normalDelta <= normalToleranceDegrees;

        if (!sameCollider || !stableNormal)
        {
            RestartInitialRestOnCurrentSurface(currentCollider, currentNormal);
            Log($"[Baseline Cycle Initial Rest Reset] collider={currentCollider.name} normalDelta={normalDelta:F4}");
            return true;
        }

        previousNormal = currentNormal;
        stableFrameCount++;

        if (settleFramesRemaining > 0)
        {
            settleFramesRemaining--;
            return true;
        }

        if (stableFrameCount < stableFramesRequired)
            return true;

        phase = SamplingPhase.SamplingInitialRest;

        Vector3 heading = ProjectHeadingOnSurface(requestedHeading, currentNormal);
        RestSample sample = CaptureRestSample(rb, currentNormal, heading);

        if (Mathf.Abs(sample.normalSpeed) <= restNormalSpeedLimit)
        {
            restSamples.Add(sample);
            Log($"[Baseline Cycle Initial Rest Sample] {restSamples.Count}/{requiredRestSampleCount} " +
                $"forward={sample.forwardSpeed:F5} normal={sample.normalSpeed:F5} " +
                $"roll={sample.rollingAngularSpeed:F5}");
        }
        else
        {
            Log($"[Baseline Cycle Initial Rest Rejected] normalSpeed={sample.normalSpeed:F5} " +
                $"limit={restNormalSpeedLimit:F5}");
        }

        if (restSamples.Count < requiredRestSampleCount)
            return true;

        if (!TryBuildInitialRest(currentCollider, out BaselineResult built))
        {
            int removeCount = Mathf.Max(1, restSamples.Count / 2);
            restSamples.RemoveRange(0, removeCount);
            Log($"[Baseline Cycle Initial Rest Variance Retry] " +
                $"forwardStd={built.forwardStd:F5} normalStd={built.normalStd:F5} " +
                $"rollingStd={built.rollingStd:F5}");
            return true;
        }

        permanentInitialRest = built;
        hasPermanentInitialRest = true;
        phase = SamplingPhase.CollectingCyclePieces;
        completedResult = permanentInitialRest;

        Log($"[Baseline Cycle Initial Rest Completed] samples={built.sampleCount} " +
            $"velocity={built.velocity:F5} angular={built.angularVelocity:F5}");

        // 第1段階保存：初回静止平均がメモリへ確定した直後に、初回静止だけのJSONを新規生成する。
        // 起動時にファイルが存在した場合、または現在すでに同名ファイルが存在する場合は書き込まない。
        if (!TryCreateInitialRestFileOnce())
        {
            Log($"[Baseline Initial Rest File Not Created] path={SavedFilePath} " +
                $"writeLocked={fileWriteLocked} status={lastSavedFileStatus}");
        }

        // 完了フレームは初期制御を停止し、次のFixedUpdateから通常制御へ進む。
        return true;
    }

    /// <summary>
    /// 現在サイクルの1ピースを回収します。
    /// 正しい順序以外のピース、同一サイクルで重複したピースは採用しません。
    /// BeforeDOTweenTurnが採用された時点で3ピースを検証し、
    /// 合格した完成サイクルだけを信用済み結果へ反映します。
    /// </summary>
    public bool CaptureCyclePiece(CyclePieceType pieceType, Rigidbody rb, bool isGrounded, Vector3 surfaceNormal, Vector3 heading, Collider sourceCollider)
    {
        EnsureSavedFileLoaded(false);

        // 完成済みファイルを読み込んだ場合、またはこの実行中に完成版へ上書き済みの場合は回収しない。
        // InitialRestOnlyファイルを読み込んだ場合は、2ピースのメモリ収集自体は可能だが、起動時ロックによりファイルは上書きしない。
        if (trustedCycle.valid)
            return false;

        if (!hasPermanentInitialRest || !rb)
            return false;

        if (pieceType == CyclePieceType.InitialGroundedRest)
            return false;

        if (pieceType == CyclePieceType.AbruptNormalChange)
        {
            if (hasCandidateAbruptNormal)
                return false;

            candidateAbruptNormal = CaptureInstantResult(rb, isGrounded, surfaceNormal, heading, sourceCollider);
            hasCandidateAbruptNormal = true;

            LogPiece(pieceType, candidateAbruptNormal);
            return true;
        }


        if (pieceType == CyclePieceType.BeforeDOTweenTurn)
        {
            if (!hasCandidateAbruptNormal || hasCandidateBeforeTurn)
            {
                return false;
            }

            candidateBeforeTurn = CaptureInstantResult(rb, isGrounded, surfaceNormal, heading, sourceCollider);
            hasCandidateBeforeTurn = true;

            LogPiece(pieceType, candidateBeforeTurn);
            TryCompleteCurrentCycle();
            return true;
        }

        return false;
    }

    /// <summary>
    /// 起動時に一度だけ読み込んだ信用済みサイクルから、対応するピースを返します。
    /// このメソッドは呼び出しごとにFile.ReadAllTextを実行しません。
    /// </summary>
    public bool TryGetTrustedPiece(CyclePieceType pieceType, out BaselineResult trustedPiece)
    {
        trustedPiece = default;

        // Awakeより先に呼ばれる特殊なケースにも対応する。
        // attemptedInitialFileLoadにより、ディスク確認は実行中に一度だけです。
        EnsureSavedFileLoaded(false);

        if (!HasUsableSavedFile)
            return false;

        return TryGetPieceFromLoadedCycle(pieceType, out trustedPiece);
    }

    /// <summary>
    /// 既存コードとの互換用です。現在はディスクを再読込せず、
    /// TryGetTrustedPieceと同じメモリキャッシュを使用します。
    /// </summary>
    public bool TryReadSavedPiece(CyclePieceType pieceType, out BaselineResult trustedPiece)
    {
        return TryGetTrustedPiece(pieceType, out trustedPiece);
    }

    bool TryGetPieceFromLoadedCycle(CyclePieceType pieceType, out BaselineResult trustedPiece)
    {
        trustedPiece = default;

        if (!trustedCycle.valid)
            return false;

        switch (pieceType)
        {
            case CyclePieceType.InitialGroundedRest: trustedPiece = trustedCycle.initialGroundedRest;
                break;
            case CyclePieceType.AbruptNormalChange: trustedPiece = trustedCycle.abruptNormalChange;
                break;
            case CyclePieceType.BeforeDOTweenTurn: trustedPiece = trustedCycle.beforeDOTweenTurn;
                break;
            default: return false;
        }

        return trustedPiece.valid;
    }

    /// <summary>
    /// 実行中の候補状態だけを初期化します。
    /// 既に生成済みのJSONファイルは削除しません。
    /// </summary>
    [ContextMenu("Reload Saved Baseline File")]
    public void ClearCompletedResult()
    {
        FirstSamplingInCollect = false;
        ClearCandidateCycle();
        restSamples.Clear();

        attemptedInitialFileLoad = false;
        loadedFromFile = false;
        fileExistedAtStartup = false;
        fileWriteLocked = false;
        initialRestFileCreatedThisRun = false;
        initialRestFileCreatedRealtime = 0f;
        lastSavedFileStatus = SavedFileStatus.Missing;
        trustedCycle = default;
        trustedCycleCount = 0;
        permanentInitialRest = default;
        hasPermanentInitialRest = false;

        // 明示的な再読込でも、現時点でファイルが存在すれば以後の自動書き込みをロックする。
        EnsureSavedFileLoaded(true);

        if (!loadedFromFile)
        {
            phase = SamplingPhase.WaitingForFirstLanding;
            ResetInitialRestSampling();
        }
    }

    /// <summary>
    /// 新しい測定ファイルを作り直す必要がある場合だけ明示的に実行します。
    /// 通常起動時には呼びません。
    /// </summary>
    [ContextMenu("Delete Saved Baseline File And Resample")]
    public void DeleteSavedBaselineFileAndResample()
    {
        FirstSamplingInCollect = false;
        string path = SavedFilePath;
        string temporaryPath = path + ".tmp";

        try
        {
            if (File.Exists(path))
                File.Delete(path);

            if (File.Exists(temporaryPath))
                File.Delete(temporaryPath);
        }
        catch (System.Exception exception)
        {
            Debug.LogError($"[Baseline File Delete Failed] path={path} error={exception.Message}", this);
            return;
        }

        attemptedInitialFileLoad = true;
        loadedFromFile = false;
        fileExistedAtStartup = false;
        fileWriteLocked = false;
        initialRestFileCreatedThisRun = false;
        initialRestFileCreatedRealtime = 0f;
        lastSavedFileStatus = SavedFileStatus.Missing;
        trustedCycle = default;
        trustedCycleCount = 0;
        permanentInitialRest = default;
        hasPermanentInitialRest = false;
        ClearCandidateCycle();
        phase = SamplingPhase.WaitingForFirstLanding;
        ResetInitialRestSampling();

        Log($"[Baseline File Deleted And Write Unlocked] path={path}");
    }

    [ContextMenu("Discard Current Candidate Cycle")]
    public void DiscardCurrentCandidateCycle()
    {
        ClearCandidateCycle();
        Log("[Baseline Cycle Candidate Discarded]");
    }

    void TryCompleteCurrentCycle()
    {
        if (!hasPermanentInitialRest ||
            !hasCandidateAbruptNormal ||
            !hasCandidateBeforeTurn)
        {
            return;
        }

        CompletedCycleResult completed = new CompletedCycleResult
        {
            valid = true,
            cycleNumber = 1,
            initialGroundedRest = permanentInitialRest,
            abruptNormalChange = candidateAbruptNormal,
            beforeDOTweenTurn = candidateBeforeTurn,
            completedTime = Time.fixedTime
        };

        if (!ValidateCompletedCycle(completed, out string rejection))
        {
            Log($"[Baseline Cycle Rejected] reason={rejection}");
            ClearCandidateCycle();
            return;
        }

        // 現仕様は最初の完成サイクルをそのまま確定値とする。
        // 2つの瞬間ピース同士の平均や、複数サイクル間の平均は行わない。
        trustedCycle = completed;
        trustedCycleCount = 1;

        Log($"[Baseline Cycle Completed] trustedCycles={trustedCycleCount} " +
            $"normalForward={trustedCycle.abruptNormalChange.forwardSpeed:F5} " +
            $"preTurnForward={trustedCycle.beforeDOTweenTurn.forwardSpeed:F5}");

        // 第2段階保存：この実行中に作成したInitialRestOnlyファイルだけを、完成版で上書きする。
        if (TryOverwriteInitialRestFileWithCompletedCycle())
        {
            loadedFromFile = true;
            Log($"[Baseline File Completed And Overwritten] path={SavedFilePath} status={lastSavedFileStatus}");
        }
        else
        {
            Log($"[Baseline Completed In Memory Only] path={SavedFilePath} " +
                $"writeLocked={fileWriteLocked} initialFileCreatedThisRun={initialRestFileCreatedThisRun} " +
                $"status={lastSavedFileStatus}");
        }

        ClearCandidateCycle();
    }

    bool ValidateInitialRest(BaselineResult initialRest, out string rejection)
    {
        rejection = string.Empty;

        if (!initialRest.valid)
        {
            rejection = "invalid_initial_rest";
            return false;
        }

        if (initialRest.sampleCount < 1)
        {
            rejection = "initial_rest_sample_count";
            return false;
        }

        if (rejectNonFiniteValues && !IsFinite(initialRest))
        {
            rejection = "initial_rest_non_finite_value";
            return false;
        }

        return true;
    }

    bool ValidateCompletedCycle(CompletedCycleResult completed, out string rejection)
    {
        rejection = string.Empty;

        if (!completed.initialGroundedRest.valid ||
            !completed.abruptNormalChange.valid ||
            !completed.beforeDOTweenTurn.valid)
        {
            rejection = "missing_or_invalid_piece";
            return false;
        }

        if (rejectNonFiniteValues &&
            (!IsFinite(completed.initialGroundedRest) ||
             !IsFinite(completed.abruptNormalChange) ||
             !IsFinite(completed.beforeDOTweenTurn)))
        {
            rejection = "non_finite_value";
            return false;
        }

        float normalToTurn =
            completed.beforeDOTweenTurn.savedTime -
            completed.abruptNormalChange.savedTime;

        if (normalToTurn < 0f ||
            (maxNormalToTurnSeconds > 0f &&
             normalToTurn > maxNormalToTurnSeconds))
        {
            rejection = $"normal_to_turn_time={normalToTurn:F4}";
            return false;
        }

        return true;
    }

    static BaselineResult CaptureInstantResult(Rigidbody rb, bool isGrounded, Vector3 normal, Vector3 heading, Collider collider)
    {
        BuildBasis(normal, heading, out normal, out heading, out Vector3 side);

        Vector3 velocity = rb.velocity;
        Vector3 angularVelocity = rb.angularVelocity;

        return new BaselineResult
        {
            valid = true,
            velocity = velocity,
            angularVelocity = angularVelocity,
            normal = normal,
            heading = heading,
            side = side,
            forwardSpeed = Vector3.Dot(velocity, heading),
            lateralSpeed = Vector3.Dot(velocity, side),
            normalSpeed = Vector3.Dot(velocity, normal),
            rollingAngularSpeed = Vector3.Dot(angularVelocity, side),
            yawAngularSpeed = Vector3.Dot(angularVelocity, normal),
            headingAngularSpeed = Vector3.Dot(angularVelocity, heading),
            forwardStd = 0f,
            normalStd = 0f,
            rollingStd = 0f,
            sampleCount = 1,
            sourceCollider = collider,
            savedTime = Time.fixedTime
        };
    }


    bool TryCreateInitialRestFileOnce()
    {
        if (!hasPermanentInitialRest || !permanentInitialRest.valid)
            return false;

        if (fileWriteLocked)
            return false;

        if (initialRestFileCreatedThisRun)
            return true;

        string path = SavedFilePath;

        // 初回静止の計算より前から、または作成直前に同名ファイルが存在する場合は触らない。
        if (File.Exists(path))
        {
            fileWriteLocked = true;
            lastSavedFileStatus = InspectSavedFile(out _, out _, true);
            Debug.LogWarning(
                $"[Baseline Initial Rest File Creation Blocked] Existing file will not be modified. " +
                $"path={path} status={lastSavedFileStatus}",
                this);
            return false;
        }

        initialRestFileCreatedRealtime = Time.realtimeSinceStartup;
        SavedCycleFile fileData = ToInitialRestSavedFile(permanentInitialRest, initialRestFileCreatedRealtime);

        if (!TryWriteNewFile(fileData, SavedDataStage.InitialRestOnly))
            return false;

        SavedFileStatus finalStatus = InspectSavedFile(out CompletedCycleResult loaded, out int loadedCount, false);

        if (finalStatus != SavedFileStatus.InitialRestOnly || !loaded.initialGroundedRest.valid || loadedCount != 0)
        {
            lastSavedFileStatus = finalStatus;
            Debug.LogError($"[Baseline Initial Rest File Final Validation Failed] path={path} status={finalStatus}", this);
            return false;
        }

        initialRestFileCreatedThisRun = true;
        lastSavedFileStatus = SavedFileStatus.InitialRestOnly;

        Log($"[Baseline Initial Rest File Created] path={path} stage={SavedDataStage.InitialRestOnly}");
        return true;
    }

    bool TryOverwriteInitialRestFileWithCompletedCycle()
    {
        if (!trustedCycle.valid)
            return false;

        if (fileWriteLocked)
            return false;

        // 起動後、このインスタンス自身が初回静止ファイルを作った場合だけ上書きを許可する。
        if (!initialRestFileCreatedThisRun)
            return false;

        string path = SavedFilePath;

        if (!File.Exists(path))
        {
            lastSavedFileStatus = SavedFileStatus.Missing;
            Debug.LogError($"[Baseline Completed Overwrite Failed] Initial-rest file is missing. path={path}", this);
            return false;
        }

        SavedFileStatus existingStatus = InspectSavedFile(out CompletedCycleResult existingCycle, out int existingCount, false);

        // 同じ実行中に生成したInitialRestOnlyファイル以外は上書きしない。
        if (existingStatus != SavedFileStatus.InitialRestOnly ||
            !existingCycle.initialGroundedRest.valid ||
            existingCount != 0 ||
            !MatchesInitialRestIdentity(existingCycle.initialGroundedRest, permanentInitialRest))
        {
            fileWriteLocked = true;
            lastSavedFileStatus = existingStatus;
            Debug.LogWarning(
                $"[Baseline Completed Overwrite Blocked] Existing file is not the expected InitialRestOnly file. " +
                $"path={path} status={existingStatus}",
                this);
            return false;
        }

        float createdRealtime = initialRestFileCreatedRealtime > 0f
            ? initialRestFileCreatedRealtime
            : Time.realtimeSinceStartup;

        SavedCycleFile completedFile = ToCompletedSavedFile(trustedCycle, createdRealtime);

        if (!TryOverwriteExistingFile(completedFile, SavedDataStage.CompletedCycle))
            return false;

        SavedFileStatus finalStatus = InspectSavedFile(out CompletedCycleResult loadedCycle, out int loadedCount, false);

        if (finalStatus != SavedFileStatus.Valid)
        {
            lastSavedFileStatus = finalStatus;
            Debug.LogError($"[Baseline File Final Validation Failed] path={path} status={finalStatus}", this);
            return false;
        }

        ApplyLoadedCycle(loadedCycle, loadedCount);
        lastSavedFileStatus = SavedFileStatus.Valid;
        return true;
    }

    bool TryWriteNewFile(SavedCycleFile fileData, SavedDataStage expectedStage)
    {
        string path = SavedFilePath;
        string temporaryPath = path + ".tmp";

        try
        {
            string directory = Path.GetDirectoryName(path);

            if (!string.IsNullOrEmpty(directory))
                Directory.CreateDirectory(directory);

            if (File.Exists(path))
            {
                fileWriteLocked = true;
                Debug.LogWarning($"[Baseline New File Write Blocked] path={path}", this);
                return false;
            }

            if (File.Exists(temporaryPath))
                File.Delete(temporaryPath);

            string json = JsonUtility.ToJson(fileData, true);
            File.WriteAllText(temporaryPath, json);

            if (!VerifyTemporarySavedFile(temporaryPath, expectedStage))
            {
                File.Delete(temporaryPath);
                return false;
            }

            // 新規生成なので、検証済み一時ファイルを正式名へ昇格する。
            File.Move(temporaryPath, path);
            return true;
        }
        catch (System.Exception exception)
        {
            lastSavedFileStatus = SavedFileStatus.IoError;
            Debug.LogError($"[Baseline New File Save Failed] path={path} error={exception.Message}", this);
            return false;
        }
    }

    bool TryOverwriteExistingFile(SavedCycleFile fileData, SavedDataStage expectedStage)
    {
        string path = SavedFilePath;
        string temporaryPath = path + ".tmp";

        try
        {
            if (!File.Exists(path))
            {
                lastSavedFileStatus = SavedFileStatus.Missing;
                return false;
            }

            if (File.Exists(temporaryPath))
                File.Delete(temporaryPath);

            string json = JsonUtility.ToJson(fileData, true);
            File.WriteAllText(temporaryPath, json);

            if (!VerifyTemporarySavedFile(temporaryPath, expectedStage))
            {
                File.Delete(temporaryPath);
                return false;
            }

            // 同じ保存先を上書きする。起動時から存在したファイルには、このメソッドへ到達しない。
            File.Copy(temporaryPath, path, true);
            File.Delete(temporaryPath);
            return true;
        }
        catch (System.Exception exception)
        {
            lastSavedFileStatus = SavedFileStatus.IoError;
            Debug.LogError($"[Baseline File Overwrite Failed] path={path} error={exception.Message}", this);
            return false;
        }
    }

    static bool MatchesInitialRestIdentity(BaselineResult fileValue, BaselineResult memoryValue)
    {
        const float scalarTolerance = 0.0001f;
        const float vectorSqrTolerance = 0.0000001f;

        return fileValue.valid &&
               memoryValue.valid &&
               fileValue.sampleCount == memoryValue.sampleCount &&
               Mathf.Abs(fileValue.savedTime - memoryValue.savedTime) <= scalarTolerance &&
               (fileValue.velocity - memoryValue.velocity).sqrMagnitude <= vectorSqrTolerance &&
               (fileValue.angularVelocity - memoryValue.angularVelocity).sqrMagnitude <= vectorSqrTolerance &&
               (fileValue.normal - memoryValue.normal).sqrMagnitude <= vectorSqrTolerance &&
               (fileValue.heading - memoryValue.heading).sqrMagnitude <= vectorSqrTolerance;
    }

    bool VerifyTemporarySavedFile(string temporaryPath, SavedDataStage expectedStage)
    {
        try
        {
            string temporaryJson = File.ReadAllText(temporaryPath);
            SavedCycleFile verification = JsonUtility.FromJson<SavedCycleFile>(temporaryJson);

            if (!verification.valid ||
                verification.formatVersion != 3 ||
                verification.dataStage != expectedStage)
            {
                Debug.LogError(
                    $"[Baseline File Temporary Validation Failed] path={temporaryPath} " +
                    $"version={verification.formatVersion} stage={verification.dataStage}",
                    this);
                return false;
            }

            if (expectedStage == SavedDataStage.InitialRestOnly)
            {
                BaselineResult initialRest = FromSavedResult(verification.initialGroundedRest);
                return ValidateInitialRest(initialRest, out _);
            }

            CompletedCycleResult completed = FromSavedCycleFile(verification);
            return ValidateCompletedCycle(completed, out _);
        }
        catch (System.Exception exception)
        {
            Debug.LogError($"[Baseline File Temporary Read Failed] path={temporaryPath} error={exception.Message}", this);
            return false;
        }
    }

    void EnsureSavedFileLoaded(bool forceReload)
    {
        if (attemptedInitialFileLoad && !forceReload)
            return;

        bool firstLoadAttempt = !attemptedInitialFileLoad;
        attemptedInitialFileLoad = true;

        // Awakeより先に他コンポーネントから呼ばれた場合も、最初のディスク確認を起動時確認として扱う。
        // この瞬間にファイルがあれば、その実行中の自動書き込みを完全にロックする。
        bool existsNow = File.Exists(SavedFilePath);

        if (firstLoadAttempt || forceReload)
        {
            fileExistedAtStartup = existsNow;
            fileWriteLocked = existsNow;
            initialRestFileCreatedThisRun = false;
            initialRestFileCreatedRealtime = 0f;
        }

        SavedFileStatus status = InspectSavedFile(out CompletedCycleResult loadedCycle, out int loadedCount, true);
        lastSavedFileStatus = status;

        if (status == SavedFileStatus.Valid)
        {
            ApplyLoadedCycle(loadedCycle, loadedCount);
            return;
        }

        if (status == SavedFileStatus.InitialRestOnly && loadedCycle.initialGroundedRest.valid)
        {
            ApplyLoadedInitialRest(loadedCycle.initialGroundedRest);
            return;
        }

        loadedFromFile = false;
        trustedCycle = default;
        trustedCycleCount = 0;

        if (!hasPermanentInitialRest)
            phase = SamplingPhase.WaitingForFirstLanding;
        else
            phase = SamplingPhase.CollectingCyclePieces;
    }

    bool TryLoadSavedCycleFromDisk(out CompletedCycleResult loadedCycle, out int loadedCount)
    {
        return InspectSavedFile(out loadedCycle, out loadedCount, true) == SavedFileStatus.Valid;
    }

    SavedFileStatus InspectSavedFile(out CompletedCycleResult loadedCycle, out int loadedCount, bool logFailure)
    {
        loadedCycle = default;
        loadedCount = 0;

        string path = SavedFilePath;

        if (!File.Exists(path))
            return SavedFileStatus.Missing;

        try
        {
            FileInfo info = new FileInfo(path);

            if (info.Length <= 0)
            {
                if (logFailure)
                    Debug.LogWarning($"[Baseline File Empty] path={path}", this);

                return SavedFileStatus.Empty;
            }

            string json = File.ReadAllText(path);

            if (string.IsNullOrWhiteSpace(json))
            {
                if (logFailure)
                    Debug.LogWarning($"[Baseline File Empty] path={path}", this);

                return SavedFileStatus.Empty;
            }

            SavedCycleFile fileData;

            try
            {
                fileData = JsonUtility.FromJson<SavedCycleFile>(json);
            }
            catch (System.Exception)
            {
                if (logFailure)
                    Debug.LogWarning($"[Baseline File Invalid JSON] path={path}", this);

                return SavedFileStatus.InvalidJson;
            }

            // formatVersion=3だけを3点サイクルとして受け入れる。

            if (fileData.formatVersion != 3 || !fileData.valid)
            {
                if (logFailure)
                    Debug.LogWarning($"[Baseline File Invalid Schema] path={path}", this);

                return SavedFileStatus.InvalidSchema;
            }

            if (fileData.dataStage == SavedDataStage.InitialRestOnly)
            {
                BaselineResult initialRest = FromSavedResult(fileData.initialGroundedRest);

                if (!ValidateInitialRest(initialRest, out string initialRejection))
                {
                    if (logFailure)
                        Debug.LogWarning($"[Baseline File Invalid Initial Rest] path={path} reason={initialRejection}", this);

                    return SavedFileStatus.InvalidCycle;
                }

                loadedCycle = new CompletedCycleResult
                {
                    valid = false,
                    cycleNumber = 0,
                    initialGroundedRest = initialRest,
                    completedTime = 0f
                };
                loadedCount = 0;
                return SavedFileStatus.InitialRestOnly;
            }

            if (fileData.dataStage != SavedDataStage.CompletedCycle)
            {
                if (logFailure)
                    Debug.LogWarning($"[Baseline File Invalid Stage] path={path} stage={fileData.dataStage}", this);

                return SavedFileStatus.InvalidSchema;
            }

            loadedCycle = FromSavedCycleFile(fileData);
            loadedCount = Mathf.Max(1, fileData.trustedCycleCount);

            if (!ValidateCompletedCycle(loadedCycle, out string rejection))
            {
                if (logFailure)
                    Debug.LogWarning($"[Baseline File Invalid Cycle] path={path} reason={rejection}", this);

                loadedCycle = default;
                loadedCount = 0;
                return SavedFileStatus.InvalidCycle;
            }

            return SavedFileStatus.Valid;
        }
        catch (System.Exception exception)
        {
            if (logFailure)
                Debug.LogError($"[Baseline File Read Failed] path={path} error={exception.Message}", this);

            return SavedFileStatus.IoError;
        }
    }


    void ApplyLoadedCycle(CompletedCycleResult loadedCycle, int loadedCount)
    {
        trustedCycle = loadedCycle;
        trustedCycleCount = Mathf.Max(1, loadedCount);
        permanentInitialRest = loadedCycle.initialGroundedRest;
        hasPermanentInitialRest = permanentInitialRest.valid;
        phase = SamplingPhase.CollectingCyclePieces;
        loadedFromFile = true;
        ClearCandidateCycle();
    }

    void ApplyLoadedInitialRest(BaselineResult loadedInitialRest)
    {
        permanentInitialRest = loadedInitialRest;
        hasPermanentInitialRest = loadedInitialRest.valid;
        trustedCycle = default;
        trustedCycleCount = 0;
        phase = SamplingPhase.CollectingCyclePieces;
        loadedFromFile = true;
        ClearCandidateCycle();
    }

    static SavedCycleFile ToInitialRestSavedFile(BaselineResult initialRest, float createdRealtime)
    {
        return new SavedCycleFile
        {
            formatVersion = 3,
            valid = initialRest.valid,
            dataStage = SavedDataStage.InitialRestOnly,
            trustedCycleCount = 0,
            createdRealtime = createdRealtime,
            updatedRealtime = createdRealtime,
            initialGroundedRest = ToSavedResult(initialRest),
            abruptNormalChange = default,
            beforeDOTweenTurn = default,
            completedTime = 0f
        };
    }

    static SavedCycleFile ToCompletedSavedFile(CompletedCycleResult cycle, float createdRealtime)
    {
        return new SavedCycleFile
        {
            formatVersion = 3,
            valid = cycle.valid,
            dataStage = SavedDataStage.CompletedCycle,
            trustedCycleCount = 1,
            createdRealtime = createdRealtime,
            updatedRealtime = Time.realtimeSinceStartup,
            initialGroundedRest = ToSavedResult(cycle.initialGroundedRest),
            abruptNormalChange = ToSavedResult(cycle.abruptNormalChange),
            beforeDOTweenTurn = ToSavedResult(cycle.beforeDOTweenTurn),
            completedTime = cycle.completedTime
        };
    }

    static CompletedCycleResult FromSavedCycleFile(SavedCycleFile fileData)
    {
        return new CompletedCycleResult
        {
            valid = fileData.valid,
            cycleNumber = Mathf.Max(1, fileData.trustedCycleCount),
            initialGroundedRest = FromSavedResult(fileData.initialGroundedRest),
            abruptNormalChange = FromSavedResult(fileData.abruptNormalChange),
            beforeDOTweenTurn = FromSavedResult(fileData.beforeDOTweenTurn),
            completedTime = fileData.completedTime
        };
    }

    static SavedBaselineResult ToSavedResult(BaselineResult result)
    {
        return new SavedBaselineResult
        {
            valid = result.valid,
            velocity = result.velocity,
            angularVelocity = result.angularVelocity,
            normal = result.normal,
            heading = result.heading,
            side = result.side,
            forwardSpeed = result.forwardSpeed,
            lateralSpeed = result.lateralSpeed,
            normalSpeed = result.normalSpeed,
            rollingAngularSpeed = result.rollingAngularSpeed,
            yawAngularSpeed = result.yawAngularSpeed,
            headingAngularSpeed = result.headingAngularSpeed,
            forwardStd = result.forwardStd,
            normalStd = result.normalStd,
            rollingStd = result.rollingStd,
            sampleCount = result.sampleCount,
            savedTime = result.savedTime
        };
    }

    static BaselineResult FromSavedResult(SavedBaselineResult result)
    {
        return new BaselineResult
        {
            valid = result.valid,
            velocity = result.velocity,
            angularVelocity = result.angularVelocity,
            normal = result.normal,
            heading = result.heading,
            side = result.side,
            forwardSpeed = result.forwardSpeed,
            lateralSpeed = result.lateralSpeed,
            normalSpeed = result.normalSpeed,
            rollingAngularSpeed = result.rollingAngularSpeed,
            yawAngularSpeed = result.yawAngularSpeed,
            headingAngularSpeed = result.headingAngularSpeed,
            forwardStd = result.forwardStd,
            normalStd = result.normalStd,
            rollingStd = result.rollingStd,
            sampleCount = result.sampleCount,
            sourceCollider = null,
            savedTime = result.savedTime
        };
    }

    static string SanitizeFileName(string fileName)
    {
        if (string.IsNullOrWhiteSpace(fileName))
            return "ReliableBaselineCycle.json";

        foreach (char invalid in Path.GetInvalidFileNameChars())
            fileName = fileName.Replace(invalid, '_');

        return fileName.EndsWith(".json", System.StringComparison.OrdinalIgnoreCase) ? fileName : fileName + ".json";
    }

    void ClearCandidateCycle()
    {
        candidateAbruptNormal = default;
        candidateBeforeTurn = default;
        hasCandidateAbruptNormal = false;
        hasCandidateBeforeTurn = false;
    }

    void ResetInitialRestSampling()
    {
        if (hasPermanentInitialRest)
            return;

        phase = SamplingPhase.WaitingForFirstLanding;
        settleFramesRemaining = settleFrames;
        stableFrameCount = 0;
        previousNormal = Vector3.up;
        sampledCollider = null;
        restSamples.Clear();
    }

    void RestartInitialRestOnCurrentSurface(Collider collider, Vector3 normal)
    {
        phase = SamplingPhase.Settling;
        settleFramesRemaining = settleFrames;
        stableFrameCount = 1;
        previousNormal = normal;
        sampledCollider = collider;
        restSamples.Clear();
    }

    static RestSample CaptureRestSample(Rigidbody rb, Vector3 normal, Vector3 heading)
    {
        BuildBasis(normal, heading, out normal, out heading, out Vector3 side);

        Vector3 velocity = rb.velocity;
        Vector3 angularVelocity = rb.angularVelocity;

        return new RestSample
        {
            normal = normal,
            heading = heading,
            forwardSpeed = Vector3.Dot(velocity, heading),
            lateralSpeed = Vector3.Dot(velocity, side),
            normalSpeed = Vector3.Dot(velocity, normal),
            rollingAngularSpeed = Vector3.Dot(angularVelocity, side),
            yawAngularSpeed = Vector3.Dot(angularVelocity, normal),
            headingAngularSpeed = Vector3.Dot(angularVelocity, heading)
        };
    }

    bool TryBuildInitialRest(Collider collider, out BaselineResult built)
    {
        built = default;
        built.forwardStd = built.normalStd = built.rollingStd = float.PositiveInfinity;

        if (restSamples.Count < 5)
            return false;

        Vector3 normalSum = Vector3.zero;
        Vector3 headingSum = Vector3.zero;
        Vector3 referenceHeading = restSamples[0].heading;

        var forward = new System.Collections.Generic.List<float>(restSamples.Count);
        var lateral = new System.Collections.Generic.List<float>(restSamples.Count);
        var normalSpeed = new System.Collections.Generic.List<float>(restSamples.Count);
        var rolling = new System.Collections.Generic.List<float>(restSamples.Count);
        var yaw = new System.Collections.Generic.List<float>(restSamples.Count);
        var headingSpin = new System.Collections.Generic.List<float>(restSamples.Count);

        foreach (RestSample sample in restSamples)
        {
            normalSum += sample.normal;
            headingSum += Vector3.Dot(sample.heading, referenceHeading) < 0f ? -sample.heading : sample.heading;

            forward.Add(sample.forwardSpeed);
            lateral.Add(sample.lateralSpeed);
            normalSpeed.Add(sample.normalSpeed);
            rolling.Add(sample.rollingAngularSpeed);
            yaw.Add(sample.yawAngularSpeed);
            headingSpin.Add(sample.headingAngularSpeed);
        }

        Vector3 averageNormal = NormalizeOrFallback(normalSum, restSamples[0].normal);
        Vector3 averageHeading = ProjectHeadingOnSurface(headingSum, averageNormal);
        BuildBasis(averageNormal, averageHeading, out averageNormal, out averageHeading, out Vector3 averageSide);

        float meanForward = TrimmedMean(forward);
        float meanLateral = TrimmedMean(lateral);
        float meanNormal = Mathf.Min(TrimmedMean(normalSpeed), 0f);
        float meanRolling = TrimmedMean(rolling);
        float meanYaw = TrimmedMean(yaw);
        float meanHeadingSpin = TrimmedMean(headingSpin);

        float forwardStd = StandardDeviation(forward, meanForward);
        float normalStd = StandardDeviation(normalSpeed, Mean(normalSpeed));
        float rollingStd = StandardDeviation(rolling, meanRolling);

        built.forwardStd = forwardStd;
        built.normalStd = normalStd;
        built.rollingStd = rollingStd;

        if (forwardStd > restForwardStdLimit || normalStd > restNormalStdLimit || rollingStd > restRollingStdLimit)
        {
            return false;
        }

        built = new BaselineResult
        {
            valid = true,
            velocity = averageHeading * meanForward + averageSide * meanLateral + averageNormal * meanNormal,
            angularVelocity = averageSide * meanRolling + averageNormal * meanYaw + averageHeading * meanHeadingSpin,
            normal = averageNormal,
            heading = averageHeading,
            side = averageSide,
            forwardSpeed = meanForward,
            lateralSpeed = meanLateral,
            normalSpeed = meanNormal,
            rollingAngularSpeed = meanRolling,
            yawAngularSpeed = meanYaw,
            headingAngularSpeed = meanHeadingSpin,
            forwardStd = forwardStd,
            normalStd = normalStd,
            rollingStd = rollingStd,
            sampleCount = restSamples.Count,
            sourceCollider = collider,
            savedTime = Time.fixedTime
        };

        return true;
    }

    float TrimmedMean(System.Collections.Generic.List<float> values)
    {
        var sorted = new System.Collections.Generic.List<float>(values);
        sorted.Sort();

        int trim = Mathf.FloorToInt(sorted.Count * trimRatio);
        int start = trim;
        int end = sorted.Count - trim;

        if (end <= start)
            return sorted[sorted.Count / 2];

        float sum = 0f;

        for (int i = start; i < end; i++)
            sum += sorted[i];

        return sum / (end - start);
    }

    static float Mean(System.Collections.Generic.List<float> values)
    {
        float sum = 0f;

        for (int i = 0; i < values.Count; i++)
            sum += values[i];

        return values.Count > 0 ? sum / values.Count : 0f;
    }

    static float StandardDeviation(System.Collections.Generic.List<float> values, float mean)
    {
        if (values.Count < 2)
            return 0f;

        float sum = 0f;

        for (int i = 0; i < values.Count; i++)
        {
            float difference = values[i] - mean;
            sum += difference * difference;
        }

        return Mathf.Sqrt(sum / values.Count);
    }

    static bool IsFinite(BaselineResult value)
    {
        return IsFinite(value.velocity) && IsFinite(value.angularVelocity) && IsFinite(value.normal) && IsFinite(value.heading) && IsFinite(value.side) && IsFinite(value.forwardSpeed) && IsFinite(value.lateralSpeed) && IsFinite(value.normalSpeed) && IsFinite(value.rollingAngularSpeed) && IsFinite(value.yawAngularSpeed) && IsFinite(value.headingAngularSpeed);
    }

    static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    static Vector3 AlignHemisphere(Vector3 value, Vector3 reference)
    {
        return Vector3.Dot(value, reference) < 0f ? -value : value;
    }

    static void BuildBasis(Vector3 normalInput, Vector3 headingInput, out Vector3 normal, out Vector3 heading, out Vector3 side)
    {
        normal = NormalizeOrFallback(normalInput, Vector3.up);
        heading = ProjectHeadingOnSurface(headingInput, normal);

        side = Vector3.Cross(normal, heading);
        side = side.sqrMagnitude > 1e-8f ? side.normalized : ProjectHeadingOnSurface(Vector3.right, normal);
    }

    static Vector3 NormalizeOrFallback(Vector3 value, Vector3 fallback)
    {
        if (value.sqrMagnitude > 1e-8f)
            return value.normalized;

        return fallback.sqrMagnitude > 1e-8f ? fallback.normalized : Vector3.up;
    }

    static Vector3 ProjectHeadingOnSurface(Vector3 heading, Vector3 normal)
    {
        normal = NormalizeOrFallback(normal, Vector3.up);

        Vector3 projected = Vector3.ProjectOnPlane(heading, normal);

        if (projected.sqrMagnitude > 1e-8f)
            return projected.normalized;

        projected = Vector3.ProjectOnPlane(Vector3.forward, normal);

        if (projected.sqrMagnitude > 1e-8f)
            return projected.normalized;

        return Vector3.ProjectOnPlane(Vector3.right, normal).normalized;
    }

    void LogPiece(CyclePieceType pieceType, BaselineResult piece)
    {
        Log($"[Baseline Cycle Piece] type={pieceType} time={piece.savedTime:F4} " + $"velocity={piece.velocity:F5} angular={piece.angularVelocity:F5} " + $"forward={piece.forwardSpeed:F5} normal={piece.normalSpeed:F5} " + $"roll={piece.rollingAngularSpeed:F5}");
    }

    void Log(string message)
    {
        if (debugSampling)
            Debug.Log(message, this);
    }

}
