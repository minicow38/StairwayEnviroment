using System.Collections.Generic;
using System.Reflection;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;
[DefaultExecutionOrder(12100)] [DisallowMultipleComponent] public sealed class EqualizerFutureSpline:MonoBehaviour
{
    const float Eps = 0.000001f;
    const string StairPrefix = "StairWay";
    const string PhysicsSuffix = "_Physics";
    const BindingFlags FieldFlags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
    [Header("Read Only Source")] [SerializeField] ExampleBallVisualEqualizer source;
    [SerializeField] NearestKnotDetector knotDetector;
    [SerializeField] CorrespondSubject correspondSubject;
    [Header("Generated Roots")] [SerializeField] Transform generatedPhysicsRoot;
    [SerializeField] Transform generatedVisualPlayer;
    [Header("Flat -> Stair Candidate")] [SerializeField,Min(.5f)] float lookAheadDistance = 8f;
    [SerializeField,Range(0f,1f)] float minimumDirectionAlignment =.80f;
    [SerializeField,Min(.1f)] float minimumPredictionSpeed = 1f;
    [Header("Spline")] [SerializeField,Range(17,161)] int totalKnotTarget = 65;
    [SerializeField,Range(1,8)] int retainGroupCount = 2;
    [SerializeField] bool includeInitialCY = true;
    [SerializeField,Min(.01f)] float startMarkerScale = .12f;
    [Header("Observed Spline")]
    [Tooltip("BallVisualEqualizerの実表示位置を1本のSplineとして後追い記録する。") ]
    [SerializeField,Min(.001f)] float observedMinimumSampleDistance = .01f;
    [SerializeField,Min(.01f)] float observedTipCubeScale = .14f;
    [Header("Prediction")]
    [Tooltip("平面上で測ったBall中心の支持高さを追従する速さ。")]
    [SerializeField,Range(.5f,20f)] float supportHeightFollowHz = 6f;
    [Tooltip("支持高さの瞬間値が推定値からこれ以上外れたら外れ値として無視する。")]
    [SerializeField,Min(.05f)] float supportHeightOutlierTolerance = .40f;
    [Tooltip("Probe simulation safety limit.")] [SerializeField,Range(16,128)] int maximumEntrySimulationSteps = 96;
    [Tooltip("How quickly the predicted body direction settles from flat entry to stair tangent.")] [SerializeField,Range(.25f,4f)] float carrierSettleDistance = 1.5f;
    [Tooltip("Planar acceleration estimate used only before stair entry.")] [SerializeField,Range(0f,60f)] float maxPredictedPlanarAcceleration = 35f;
    [Header("Debug")] [SerializeField] bool enableLog = true;
    [Header("Runtime - Read Only")] [SerializeField] string currentCandidate = "";
    [SerializeField] string lastGeneratedGroup = "";
    [SerializeField] int predictionRevision;
    [SerializeField] int generatedSplineCount;
    [SerializeField] int generatedKnotCount;
    [SerializeField] int predictedMultiplier;
    [SerializeField] int predictedWaveCount;
    [SerializeField] float predictedSectionLength;
    [SerializeField] float predictedEntryTime;
    [SerializeField] float predictedEntrySlopeDegrees;
    [SerializeField] float predictedEntryRoughness;
    [SerializeField] float predictedEntryNormalSpeed;
    [SerializeField] float predictedA1;
    [SerializeField] float predictedA2;
    [SerializeField] float predictedA3;
    [SerializeField] float candidateForwardDistance;
    [SerializeField] float candidateLateralDistance;
    [SerializeField] float candidateVerticalError;
    [SerializeField] float candidateAlignment;
    [SerializeField] float tangentNormalDot;
    [SerializeField] Vector3 mappedFirstPoint;
    [SerializeField] Vector3 mappedFirstBallPoint;
    [SerializeField] float stableSupportHeight;
    [SerializeField] bool hasStableSupportHeight;
    [SerializeField] int observedKnotCount;
    [SerializeField] float observedPathLength;
    [SerializeField] Vector3 observedLatestPoint;
    [SerializeField] string observedCurrentStairGroup = "";
    [SerializeField] string observedFutureComparedSpline = "";
    [SerializeField] Vector3 observedFutureAnchorWorld;
    [SerializeField] Vector3 observedFutureRawDelta;
    [SerializeField] bool observedFutureAlignmentApplied;
    [SerializeField] Vector3 observedFutureConfirmedTangent;
    [SerializeField] Vector3 observedFutureConfirmedNormal;
    [SerializeField] float observedFutureEntryFrameRotationDegrees;
    [SerializeField] float observedFutureTangentErrorDegrees;
    [SerializeField] float observedFutureMaxTangentCorrection;
    [SerializeField] float observedFutureMaxNormalCorrection;
    [SerializeField] float observedFutureAnchorError;
    [SerializeField] string observedWaveEntryGroup = "";
    [SerializeField] Vector3 observedWaveEntryPoint;
    [SerializeField] Vector3 observedWaveEntryTangent;
    [SerializeField] Vector3 observedWaveEntryNormal;
    [SerializeField] Vector3 observedWaveEntryVelocity;
    [SerializeField] float observedWaveEntryForwardSpeed;
    [SerializeField] int observedWaveEntryRevision;
    [SerializeField] bool timingCandidateActive;
    long timingCandidateSectionKey = long.MinValue;
    readonly HashSet<string> generatedGroups = new HashSet<string>();
    readonly List<GeneratedGroup> history = new List<GeneratedGroup>();
    Rigidbody body;
    Vector3 previousVelocity;
    Vector3 filteredPlanarAcceleration;
    bool hasPreviousVelocity;
    SplineContainer observedSpline;
    Transform observedTipCube;
    Transform observedDisplayTarget;
    FieldInfo observedDisplayTargetField;
    Vector3 observedLastPoint;
    bool hasObservedLastPoint;
    bool courseGuideStateInitialized;
    bool previousGuideWasSlope;
    bool courseRecognitionArmed;
    bool courseSawVisualTurn;
    bool previousSourceActive;
    bool waveEntryPointCapturePending;
    bool hasObservedWaveEntry;
    Vector3 observedWaveEntryBinormal;
    readonly List<ObservedStairNode> observedStairNodes = new List<ObservedStairNode>();
    struct ObservedStairNode
    {
        public Transform physics;
        public string group;
        public int index;
    }
    sealed class GeneratedPart
    {
        public SplineContainer future;
        public SplineContainer ball;

        // Generation-time normal decomposition in the visual-parent local frame.
        // Keeping this local makes it survive later VisualPlayerRoot turns.
        public Vector3[] predictedNormalLocal;
        public Vector3[] carrierTangentLocal;
        public float[] normalDistance;
        public float[] traveledDistance;
    }
    sealed class GeneratedGroup
    {
        public string name;
        public bool alignmentApplied;
        public Vector3 alignmentDeltaWorld;
        public float predictedEntryForwardSpeed;
        public readonly List<SplineContainer> splines = new List<SplineContainer>();
        public readonly List<GeneratedPart> parts = new List<GeneratedPart>();
    }
    struct StairPair
    {
        public int index;
        public Transform physics;
        public Transform visual;
    }
    struct EntryPrediction
    {
        public Vector3 position;
        public Vector3 velocity;
        public Vector3 heading;
        public Vector3 tangent;
        public Vector3 normal;
        public float time;
        public float distanceToBoundary;
        public float slopeDegrees;
        public float roughness;
        public float curvature;
        public float gradient;
        public float supportHeight;
    }
    struct WavePlan
    {
        public int multiplier;
        public int repeat23;
        public int waveCount;
        public int motionPattern;
        public float baseSectionLength;
        public float totalLength;
        public float fBoundary;
        public float a1;
        public float a2;
        public float a3;
        public float gravityN;
        public float initialWaveSpeed;
        public float initialCY;
        public float lowerRatio;
        public float firstApex;
        public float laterApex;
        public float dampingPerMeter;
        public float restitution;
        public float rough1;
        public float rough2;
        public float rough3;
        public float roughGain;
        public float roughReference;
        public float entryRoughness;
        public float roughFollowHz;
        public float patternVariation;
        public float maximumResidual;
        public float entryBlendSeconds;
        public float referencePlanarSpeed;
        public float probeSpacing;
    }
    public int PredictionRevision => predictionRevision;
    public int GeneratedSplineCount => generatedSplineCount;
    public int GeneratedKnotCount => generatedKnotCount;
    void Awake()
    {
        ResolveReferences();
    }
    void FixedUpdate()
    {
        if (!ResolveReferences()) return;

        UpdateAccelerationEstimate();
        UpdateObservedWaveEntryState();

        if (!TryConfirmFlatToStairTiming(
                out NearestKnotDetector.GuideFrame flatGuide,
                out NearestKnotDetector.GuideSample slopeEntry))
        {
            return;
        }

        if (!TryResolvePhysicsZeroFromSlopeEntry(
                slopeEntry,
                out Transform physicsZero,
                out string groupName))
        {
            return;
        }

        currentCandidate = physicsZero.name;
        candidateForwardDistance = flatGuide.distanceToNextSlope;

        if (generatedGroups.Contains(groupName))
            return;

        if (!TryCollectWholeGroup(
                physicsZero,
                groupName,
                out List<StairPair> group))
        {
            return;
        }

        if (group.Count == 0 || group[0].index != 0)
            return;

        Vector3 stairT =
            ResolveStairTangent(
                group[0].physics,
                body.velocity);

        Vector3 stairN =
            ResolveStairNormal(
                group[0].physics,
                stairT);

        if (!PredictExampleEntryState(
                physicsZero,
                stairT,
                stairN,
                out EntryPrediction entry))
        {
            return;
        }

        // Timing is already authoritative from NearestKnotDetector.
        // Do not use StairWayN_0.transform.position as the spatial entry E:
        // in a multi-piece stair N_0 is placed inside the section, not at
        // the section boundary. Anchor E to the canonical future slope sample.
        AnchorPredictionToSlopeEntry(
            flatGuide,
            slopeEntry,
            ref entry);

        BuildPrediction(
            groupName,
            group,
            stairT,
            stairN,
            entry);
    }

    void LateUpdate()
    {
        // ExampleBallVisualEqualizer は DefaultExecutionOrder(12000)、
        // このクラスは 12100。source の LateUpdate が実表示位置を書いた後に読む。
        if (!Application.isPlaying || !source) return;
        if (!generatedVisualPlayer)
        {
            GameObject root = GameObject.Find("__GeneratedVisualPlayer");
            if (root) generatedVisualPlayer = root.transform;
        }
        if (!generatedVisualPlayer) return;

        if (!TryGetObservedWorldPoint(out Vector3 point)) return;
        EnsureObservedSpline();
        if (!observedSpline) return;

        FinalizeObservedWaveEntryPoint(point);
        UpdateObservedTipCube(point);
        UpdateObservedStairRetention(point);
        AppendObservedPoint(point);
    }

    bool ResolveReferences()
    {
        if (!source) source = GetComponent<ExampleBallVisualEqualizer>();
        if (!source) source = FindObjectOfType<ExampleBallVisualEqualizer>();
        if (!source) return false;
        body = source.InSubject;
        if (!body) return false;
        if (!knotDetector) knotDetector = body.GetComponent<NearestKnotDetector>();
        if (!knotDetector) knotDetector = FindObjectOfType<NearestKnotDetector>();
        if (!knotDetector) return false;
        if (!correspondSubject) correspondSubject = FindObjectOfType<CorrespondSubject>();
        if (!generatedPhysicsRoot)
        {
            GameObject root = GameObject.Find("__GeneratedPhysics");
            if (root) generatedPhysicsRoot = root.transform;
        }
        if (!generatedVisualPlayer)
        {
            GameObject root = GameObject.Find("__GeneratedVisualPlayer");
            if (root) generatedVisualPlayer = root.transform;
        }
return generatedPhysicsRoot && generatedVisualPlayer;
    }
    bool TryConfirmFlatToStairTiming(
        out NearestKnotDetector.GuideFrame guide,
        out NearestKnotDetector.GuideSample slopeEntry)
    {
        guide = knotDetector.CurrentGuide;
        slopeEntry = default;

        UpdateStableFlatSupportHeight(guide);

        // FutureSpline は「未来予測」なので、Flat上で nextIsSlope が
        // 有効になった最初の瞬間に生成する。
        // 以前の 2回観測 / 0.08m 確認待ちは、生成を1 FixedUpdate以上
        // 遅らせるだけだったため、予測生成のゲートから外した。
        if (!guide.valid ||
            guide.isSlope ||
            !guide.nextIsSlope ||
            !IsFinite(guide.distanceToNextSlope) ||
            guide.distanceToNextSlope < 0f ||
            guide.distanceToNextSlope > lookAheadDistance)
        {
            ResetTimingCandidate();
            return false;
        }

        if (!knotDetector.TryEvaluateForwardSlopeSection(
                guide,
                0f,
                out slopeEntry) ||
            !slopeEntry.valid ||
            !slopeEntry.isSlope)
        {
            ResetTimingCandidate();
            return false;
        }

        long sectionKey =
            MakeSectionKey(
                slopeEntry.splineIndex,
                slopeEntry.sectionIndex);

        bool newCandidate =
            !timingCandidateActive ||
            sectionKey != timingCandidateSectionKey;

        timingCandidateActive = true;
        timingCandidateSectionKey = sectionKey;

        if (enableLog && newCandidate)
        {
            Debug.Log(
                $"[EqualizerFutureSpline][ENTRY_TIMING_PREDICTED] " +
                $"spline={slopeEntry.splineIndex} " +
                $"section={slopeEntry.sectionIndex} " +
                $"nextSlope={guide.distanceToNextSlope:F3}m " +
                $"immediate=True",
                this);
        }

        return true;
    }

    void UpdateStableFlatSupportHeight(
        NearestKnotDetector.GuideFrame guide)
    {
        if (!guide.valid || guide.isSlope || !body) return;

        // Flat上の支持高さはWorld Upで測る。
        // guide.normal は入口接近時に傾き始める場合があるため、ここでは使わない。
        float measured = body.position.y - guide.point.y;
        if (!IsFinite(measured) || measured <= .01f || measured > 3f)
            return;

        if (!hasStableSupportHeight)
        {
            stableSupportHeight = measured;
            hasStableSupportHeight = true;
            return;
        }

        if (Mathf.Abs(measured - stableSupportHeight) >
            supportHeightOutlierTolerance)
        {
            return;
        }

        float alpha = FollowAlpha(
            supportHeightFollowHz,
            Mathf.Max(.001f,Time.fixedDeltaTime));

        stableSupportHeight = Mathf.Lerp(
            stableSupportHeight,
            measured,
            alpha);
    }

    void ResetTimingCandidate()
    {
        timingCandidateActive = false;
        timingCandidateSectionKey = long.MinValue;
    }

    bool TryResolvePhysicsZeroFromSlopeEntry(
        NearestKnotDetector.GuideSample slopeEntry,
        out Transform physicsZero,
        out string groupName)
    {
        physicsZero = null;
        groupName = null;

        Transform[] nodes =
            generatedPhysicsRoot.GetComponentsInChildren<Transform>(
                true);

        float bestScore =
            float.PositiveInfinity;

        Vector3 entryT =
            NormalizeSafe(
                slopeEntry.tangent,
                body.velocity);

        for (int i = 0;
             i < nodes.Length;
             i++)
        {
            Transform candidate =
                nodes[i];

            if (!candidate ||
                !TryParsePhysicsStair(
                    candidate.name,
                    out string candidateGroup,
                    out int childIndex) ||
                childIndex != 0)
            {
                continue;
            }

            Vector3 candidateT =
                NormalizeSafe(
                    candidate.forward,
                    entryT);

            if (candidateT.y > 0.001f)
            {
                candidateT = -candidateT;
            }

            if (entryT.y > 0.001f)
            {
                entryT = -entryT;
            }

            float alignment =
                Vector3.Dot(
                    candidateT,
                    entryT);

            if (alignment <
                minimumDirectionAlignment)
            {
                continue;
            }

            float distance =
                Vector3.Distance(
                    candidate.position,
                    slopeEntry.point);

            float score =
                distance +
                (1f - alignment) *
                2f;

            if (score >=
                bestScore)
            {
                continue;
            }

            bestScore = score;
            physicsZero = candidate;
            groupName = candidateGroup;
            candidateAlignment = alignment;
        }

        if (!physicsZero)
            return false;

        Vector3 toEntry =
            slopeEntry.point -
            body.position;

        Vector3 planarVelocity =
            Vector3.ProjectOnPlane(
                body.velocity,
                Vector3.up);

        Vector3 moveDirection =
            NormalizeSafe(
                planarVelocity,
                slopeEntry.tangent);

        candidateLateralDistance =
            Vector3.ProjectOnPlane(
                toEntry -
                moveDirection *
                Vector3.Dot(
                    toEntry,
                    moveDirection),
                Vector3.up)
            .magnitude;

        candidateVerticalError =
            Mathf.Abs(
                toEntry.y);

        return true;
    }

    static long MakeSectionKey(
        int splineIndex,
        int sectionIndex)
    {
        return
            ((long)splineIndex << 32) ^
            (uint)sectionIndex;
    }

    static bool IsFinite(
        float value)
    {
        return
            !float.IsNaN(value) &&
            !float.IsInfinity(value);
    }

    static bool IsFinite(
        Vector3 value)
    {
        return
            IsFinite(value.x) &&
            IsFinite(value.y) &&
            IsFinite(value.z);
    }

    void AnchorPredictionToSlopeEntry(
        NearestKnotDetector.GuideFrame flatGuide,
        NearestKnotDetector.GuideSample slopeEntry,
        ref EntryPrediction prediction)
    {
        Vector3 flatT = NormalizeSafe(
            Vector3.ProjectOnPlane(flatGuide.tangent,Vector3.up),
            Vector3.ProjectOnPlane(body.velocity,Vector3.up));

        if (Vector3.Dot(flatT,body.velocity) < 0f)
            flatT = -flatT;

        Vector3 flatB = NormalizeSafe(
            Vector3.Cross(Vector3.up,flatT),
            Vector3.right);

        // 入口の向きは完成した45度法線へ即スナップせず、
        // PredictExampleEntryState が未来シミュレーションしたT/Nを使う。
        Vector3 predictedT = NormalizeSafe(
            prediction.tangent,
            slopeEntry.tangent);

        Vector3 predictedN = NormalizeSafe(
            Vector3.ProjectOnPlane(prediction.normal,predictedT),
            slopeEntry.normal);

        if (Vector3.Dot(predictedN,Vector3.up) < 0f)
            predictedN = -predictedN;

        Vector3 predictedB = NormalizeSafe(
            Vector3.Cross(predictedN,predictedT),
            flatB);

        if (Vector3.Dot(predictedB,flatB) < 0f)
            predictedB = -predictedB;

        predictedT = NormalizeSafe(
            Vector3.Cross(predictedB,predictedN),
            predictedT);

        float supportHeight = hasStableSupportHeight
            ? stableSupportHeight
            : Mathf.Max(.01f,body.position.y - flatGuide.point.y);

        // 1本目はPhysicsSplineRootの中心線 + 予測法線方向の支持高さ。
        // 横laneはここへ混ぜない。
        prediction.position =
            slopeEntry.point +
            predictedN * supportHeight;

        prediction.tangent = predictedT;
        prediction.normal = predictedN;
        prediction.heading = flatT;
        prediction.distanceToBoundary = 0f;
        prediction.supportHeight = supportHeight;

        if (enableLog)
        {
            Debug.Log(
                $"[EqualizerFutureSpline][ENTRY_ANCHOR] " +
                $"guidePoint={flatGuide.point:F4} " +
                $"slopePoint={slopeEntry.point:F4} " +
                $"supportHeight={supportHeight:F4} " +
                $"anchoredPos={prediction.position:F4} " +
                $"predictedT={predictedT:F4} " +
                $"predictedN={predictedN:F4} " +
                $"predictedB={predictedB:F4}",
                this);
        }
    }

    void UpdateAccelerationEstimate()
    {
        Vector3 velocity = body.velocity;
        if (!hasPreviousVelocity)
        {
            previousVelocity = velocity;
            hasPreviousVelocity = true;
            return;
        }
        float dt = Mathf.Max(.001f,Time.fixedDeltaTime);
        Vector3 rawAcceleration = (velocity - previousVelocity) / dt;
        rawAcceleration = Vector3.ProjectOnPlane(rawAcceleration,Vector3.up);
        rawAcceleration = Vector3.ClampMagnitude(rawAcceleration,maxPredictedPlanarAcceleration);
        float alpha = 1f - Mathf.Exp(-2f * Mathf.PI * 4f * dt);
        filteredPlanarAcceleration = Vector3.Lerp(filteredPlanarAcceleration,rawAcceleration,alpha);
        previousVelocity = velocity;
    }
    bool TryCollectWholeGroup(Transform physicsZero,string groupName,out List<StairPair> group)
    {
        group = new List<StairPair>();
        Transform root = physicsZero.parent;
        if (!root) return false;
        List<Transform> physicsParts = new List<Transform>();
        for (int i = 0;i < root.childCount;i++)
        {
            Transform child = root.GetChild(i);
            if (!TryParsePhysicsStair(child.name,out string candidateGroup,out _))
            {
                continue;
            }
            if (candidateGroup == groupName)
            {
                physicsParts.Add(child);
            }
        }
        physicsParts.Sort((a,b) =>
        {
            TryParsePhysicsStair(a.name,out _,out int ai);TryParsePhysicsStair(b.name,out _,out int bi);return ai.CompareTo(bi);
        }
        );
        for (int i = 0;i < physicsParts.Count;i++)
        {
            Transform physics = physicsParts[i];
            TryParsePhysicsStair(physics.name,out _,out int index);
            if (index != i) return false;
            Transform visual = FindVisualPart(groupName,index);
            if (!visual) return false;
            group.Add(new StairPair
            {
                index = index,physics = physics,visual = visual
            }
            );
        }
        return group.Count > 0;
    }
    Transform FindVisualPart(string groupName,int index)
    {
        Transform found = FindDescendant(generatedVisualPlayer,$"{groupName}_{index}_Render");
        if (found) return found;
        return FindDescendant(generatedVisualPlayer,$"{groupName}_{index}_Renderer");
    }
    Vector3 ResolveStairTangent(Transform physicsZero,Vector3 velocity)
    {
        Vector3 stairT = NormalizeSafe(physicsZero.forward,velocity);

        // StairWay*_Physics.forward is generated as the downhill authority.
        // Never flip the whole 3D tangent from XZ/velocity alignment because
        // that also flips Y and turns a descending stair into an ascending one.
        if (Mathf.Abs(stairT.y) > 0.001f)
        {
            if (stairT.y > 0f)
            {
                stairT = -stairT;
            }

            return stairT.normalized;
        }

        // Only a nearly-flat fallback may use planar velocity to choose sign.
        Vector3 planarVelocity = Vector3.ProjectOnPlane(velocity,Vector3.up);
        Vector3 planarT = Vector3.ProjectOnPlane(stairT,Vector3.up);

        if (planarVelocity.sqrMagnitude > Eps &&
            planarT.sqrMagnitude > Eps &&
            Vector3.Dot(planarVelocity,planarT) < 0f)
        {
            stairT = -stairT;
        }

        return stairT.normalized;
    }
    static Vector3 ResolveStairNormal(Transform physicsZero,Vector3 stairT)
    {
        Vector3 stairN = Vector3.ProjectOnPlane(physicsZero.up,stairT);
        stairN = NormalizeSafe(stairN,Vector3.up);
        if (Vector3.Dot(stairN,Vector3.up) < 0f)
        {
            stairN = -stairN;
        }
        return stairN;
    }
    bool PredictExampleEntryState(
        Transform physicsZero,
        Vector3 stairT,
        Vector3 stairN,
        out EntryPrediction prediction)
    {
        prediction = default;
        Vector3 planarVelocity = Vector3.ProjectOnPlane(body.velocity,Vector3.up);
        float startSpeed = planarVelocity.magnitude;
        if (startSpeed < minimumPredictionSpeed)
        {
            return false;
        }
        Vector3 heading = planarVelocity.normalized;
        Vector3 stairHorizontal = Vector3.ProjectOnPlane(stairT,Vector3.up);
        if (stairHorizontal.sqrMagnitude < Eps)
        {
            return false;
        }
        stairHorizontal.Normalize();
        if (Vector3.Dot(stairHorizontal,heading) < 0f)
        {
            stairHorizontal = -stairHorizontal;
        }
        float headingToStair = Mathf.Max(.05f,Vector3.Dot(heading,stairHorizontal));
        float stairHorizontalLength = Mathf.Max(Eps,Vector3.ProjectOnPlane(stairT,Vector3.up).magnitude);
        float grade = Mathf.Abs(stairT.y) / stairHorizontalLength;
        grade = Mathf.Max(.01f,grade);
        float probeSpacing = Mathf.Max(.05f,ReadFloat("probeSpacing",.45f));
        float entrySlopeDegrees = Mathf.Clamp(ReadFloat("entrySlopeDegrees",12f),1f,60f);
        float frameFollowHz = Mathf.Max(.01f,ReadFloat("frameFollowHz",10f));
        float roughFollowHz = Mathf.Max(.01f,ReadFloat("roughFollowHz",12f));
        float referenceSpeed = Mathf.Max(1f,ReadFloat("referencePlanarSpeed",18f));
        float maxSpeed = referenceSpeed * 1.35f;
        float dt = Mathf.Max(.005f,Time.fixedDeltaTime);
        // T/NはObserved snapshotから受け取らない。
        // 現在のExampleBVE安定フレームを開始点にして未来側で発展させる。
        Vector3 simT = NormalizeSafe(source.StableT,heading);
        Vector3 simN = NormalizeSafe(source.StableN,Vector3.up);
        float filteredRoughness = Mathf.Max(0f,source.Roughness);
        float filteredCurvature = source.SignedCurvature;
        float distance = 0f;
        float time = 0f;
        Vector3 boundaryPoint = physicsZero.position;
        // 未来予測の原点は常にPhysics body。Observed表示位置は使わない。
        Vector3 simulationOrigin = body.position;
        float accelerationAlongHeading = Vector3.Dot(filteredPlanarAcceleration,heading);
        accelerationAlongHeading = Mathf.Clamp(accelerationAlongHeading,-maxPredictedPlanarAcceleration,maxPredictedPlanarAcceleration);
        for (int step = 0;step < maximumEntrySimulationSteps;step++)
        {
            float predictedSpeed = Mathf.Clamp(startSpeed + accelerationAlongHeading * time,minimumPredictionSpeed,Mathf.Max(startSpeed,maxSpeed));
            distance += predictedSpeed * dt;
            time += dt;
            Vector3 simulatedPosition = simulationOrigin + heading * distance;
            simulatedPosition.y = simulationOrigin.y;
            float bodyX = Vector3.Dot(simulatedPosition - boundaryPoint,stairHorizontal);
            float projectedProbeStep = probeSpacing * headingToStair;
            float h0 = IdealSurfaceHeight(bodyX - 1.5f * projectedProbeStep,grade);
            float h1 = IdealSurfaceHeight(bodyX -.5f * projectedProbeStep,grade);
            float h2 = IdealSurfaceHeight(bodyX +.5f * projectedProbeStep,grade);
            float h3 = IdealSurfaceHeight(bodyX + 1.5f * projectedProbeStep,grade);
            float effectiveD = Mathf.Max(.05f,probeSpacing);
            float slope0 = Mathf.Atan2(h1 - h0,effectiveD);
            float slope1 = Mathf.Atan2(h2 - h1,effectiveD);
            float slope2 = Mathf.Atan2(h3 - h2,effectiveD);
            float gradient = (h3 - h0) / (3f * effectiveD);
            float curvature = slope1 -.5f * (slope0 + slope2);
            float roughness = Mathf.Abs(curvature) +.5f * Mathf.Abs(slope2 - slope0);
            float roughAlpha = FollowAlpha(roughFollowHz,dt);
            filteredCurvature = Mathf.Lerp(filteredCurvature,curvature,roughAlpha);
            filteredRoughness = Mathf.Lerp(filteredRoughness,roughness,roughAlpha);
            Vector3 desiredT = (heading + Vector3.up * gradient).normalized;
            if (desiredT.sqrMagnitude < Eps)
            {
                desiredT = simT;
            }
            Vector3 transportedN = Quaternion.FromToRotation(simT,desiredT) * simN;
            Vector3 rawN = Vector3.ProjectOnPlane(Vector3.up,desiredT);
            rawN = NormalizeSafe(rawN,transportedN);
            if (Vector3.Dot(rawN,transportedN) < 0f)
            {
                rawN = -rawN;
            }
            simT = desiredT;
            simN = Vector3.Slerp(transportedN,rawN,FollowAlpha(frameFollowHz,dt)).normalized;
            simN = NormalizeSafe(Vector3.ProjectOnPlane(simN,simT),stairN);
            float slopeDegrees = -Mathf.Atan(gradient) * Mathf.Rad2Deg;
            if (slopeDegrees >= entrySlopeDegrees)
            {
                Vector3 predictedVelocity = heading * predictedSpeed;
                prediction.position = simulatedPosition;
                prediction.velocity = predictedVelocity;
                prediction.heading = heading;
                prediction.tangent = simT;
                prediction.normal = simN;
                prediction.time = time;
                prediction.slopeDegrees = slopeDegrees;
                prediction.roughness = filteredRoughness;
                prediction.curvature = filteredCurvature;
                prediction.gradient = gradient;
                prediction.distanceToBoundary = Mathf.Max(0f,-bodyX);
                return true;
            }
            if (distance > lookAheadDistance + 2f)
            {
                break;
            }
        }
        float thresholdGradient = Mathf.Tan(entrySlopeDegrees * Mathf.Deg2Rad);
        float triggerX = (3f * probeSpacing * thresholdGradient / grade) - 1.5f * probeSpacing * headingToStair;
        float currentX = Vector3.Dot(simulationOrigin - boundaryPoint,stairHorizontal);
        Vector3 predictedPlanarVelocity = heading * startSpeed;
        float closingSpeed = Mathf.Max(.1f,Vector3.Dot(predictedPlanarVelocity,stairHorizontal));
        float remainingX = triggerX - currentX;
        float fallbackTime = Mathf.Max(0f,remainingX / closingSpeed);
        Vector3 fallbackPosition = simulationOrigin + heading * startSpeed * fallbackTime;
        fallbackPosition.y = simulationOrigin.y;
        Vector3 fallbackT = NormalizeSafe(heading + Vector3.up * -thresholdGradient,stairT);
        Vector3 fallbackN = NormalizeSafe(Vector3.ProjectOnPlane(Vector3.up,fallbackT),stairN);
        prediction.position = fallbackPosition;
        prediction.velocity = heading * startSpeed;
        prediction.heading = heading;
        prediction.tangent = fallbackT;
        prediction.normal = fallbackN;
        prediction.time = fallbackTime;
        prediction.slopeDegrees = entrySlopeDegrees;
        prediction.roughness = source.Roughness;
        prediction.curvature = source.SignedCurvature;
        prediction.gradient = -thresholdGradient;
        prediction.distanceToBoundary = Mathf.Max(0f,-triggerX);
        return true;
    }
    static float IdealSurfaceHeight(float xAfterBoundary,float grade)
    {
        if (xAfterBoundary <= 0f) return 0f;
        return -grade * xAfterBoundary;
    }
    void BuildPrediction(string groupName,List<StairPair> group,Vector3 stairT,Vector3 stairN,EntryPrediction entry)
    {
        tangentNormalDot = Vector3.Dot(stairT,stairN);
        WavePlan plan = CreateWavePlan(group.Count,entry);
        GeneratedGroup generated = new GeneratedGroup
        {
            name = groupName
        };

        generatedSplineCount = 0;
        generatedKnotCount = 0;

        int knotPerPart = Mathf.Max(
            6,
            Mathf.CeilToInt((totalKnotTarget - 1) / (float)group.Count) + 1);

        // FutureSplineだけを物理・波計算から作る。
        // FutureBallは後からObservedの波開始フレーム T0/N0 へ合わせるため、
        // 各Knotの予測法線・carrier接線・(支持高さ+波高)・進行距離を保存する。
        Vector3[][] basePoints = new Vector3[group.Count][];
        Vector3[][] predictedNormalsLocal = new Vector3[group.Count][];
        Vector3[][] carrierTangentsLocal = new Vector3[group.Count][];
        float[][] normalDistances = new float[group.Count][];
        float[][] traveledDistances = new float[group.Count][];

        for (int i = 0;i < group.Count;i++)
        {
            basePoints[i] = BuildBasePartPoints(
                group[i],
                plan,
                stairT,
                stairN,
                entry,
                knotPerPart,
                out predictedNormalsLocal[i],
                out carrierTangentsLocal[i],
                out normalDistances[i],
                out traveledDistances[i]);
        }

        // FutureBallSplineは最初はFutureSplineと完全同形・同位置。
        // rawDeltaはFlat上の最寄り判定では確定しない。
        // 実際に次のSlopeへ入り、必要なVisualFrame旋回が完了した後だけ、
        // まずObservedの波開始フレームへFutureBall入口を再構築し、
        // その再構築後AnchorとObservedEntryPointのWorld差分だけを1回適用する。
        for (int i = 0;i < group.Count;i++)
        {
            StairPair pair = group[i];
            Vector3 fallbackT = PhysicsDirectionToVisual(pair,stairT);

            SplineContainer baseSpline =
                CreateSpline(pair.visual,groupName,pair.index,false);

            BuildSplineFromPoints(
                baseSpline,
                basePoints[i],
                fallbackT);

            SplineContainer ballSpline =
                CreateSpline(pair.visual,groupName,pair.index,true);

            BuildSplineFromPoints(
                ballSpline,
                basePoints[i],
                fallbackT);

            generated.parts.Add(
                new GeneratedPart
                {
                    future = baseSpline,
                    ball = ballSpline,
                    predictedNormalLocal = predictedNormalsLocal[i],
                    carrierTangentLocal = carrierTangentsLocal[i],
                    normalDistance = normalDistances[i],
                    traveledDistance = traveledDistances[i]
                });

            if (pair.index == 0)
            {
                mappedFirstPoint = basePoints[i][0];
                mappedFirstBallPoint = basePoints[i][0];

                CreateStartMarker(
                    ballSpline,
                    mappedFirstBallPoint,
                    "BallStartKnot");
            }

            generated.splines.Add(baseSpline);
            generated.splines.Add(ballSpline);
            generatedSplineCount += 2;
            generatedKnotCount +=
                baseSpline.Spline.Count + ballSpline.Spline.Count;
        }

        generated.predictedEntryForwardSpeed = Mathf.Max(
            0f,
            Vector3.Dot(entry.velocity,entry.tangent));

        generatedGroups.Add(groupName);
        history.Add(generated);
        TrimHistory();

        predictionRevision++;
        lastGeneratedGroup = groupName;
        predictedMultiplier = plan.multiplier;
        predictedWaveCount = plan.waveCount;
        predictedSectionLength = plan.totalLength;
        predictedEntryTime = entry.time;
        predictedEntrySlopeDegrees = entry.slopeDegrees;
        predictedEntryRoughness = entry.roughness;
        predictedEntryNormalSpeed = plan.initialWaveSpeed;
        predictedA1 = plan.a1;
        predictedA2 = plan.a2;
        predictedA3 = plan.a3;

        if (enableLog)
        {
            Debug.Log(
                $"[EqualizerFutureSpline][FUTURE_BALL_INITIAL] " +
                $"group={groupName} parts={group.Count} " +
                $"mode=IdentityCopy alignmentPending=True " +
                $"baseFirst={mappedFirstPoint:F4} " +
                $"ballFirst={mappedFirstBallPoint:F4} " +
                $"splines={generatedSplineCount} " +
                $"knots={generatedKnotCount}",
                this);
        }
    }

    WavePlan CreateWavePlan(int multiplier,EntryPrediction entry)
    {
        WavePlan p = new WavePlan();
        p.multiplier = Mathf.Max(1,multiplier);
        p.repeat23 = p.multiplier - 1;
        p.waveCount = 3 + p.repeat23 * 2;
        p.baseSectionLength = Mathf.Max(.1f,ReadFloat("sectionLength",9.9f));
        p.totalLength = p.baseSectionLength * p.multiplier;
        p.fBoundary = Mathf.Clamp(source.TargetSlopeProgressPerecent,.05f,.95f);
        float initialHeight = Mathf.Max(.01f,ReadFloat("initialHeight",.45f));
        float fallbackNormalSpeed = Mathf.Max(0f,ReadFloat("fallbackInitialNormalSpeed",3.1367f));
        float energyBlend = Mathf.Clamp01(ReadFloat("energyBlend",.35f));
        p.restitution = Mathf.Clamp(ReadFloat("restitution",.86f),.05f,.99f);
        p.lowerRatio = Mathf.Clamp(ReadFloat("lowerRatio",.75f),.05f,1f);
        p.firstApex = ReadFloat("firstApexFraction",.82f);
        p.laterApex = ReadFloat("laterApexFraction",.50f);
        p.dampingPerMeter = Mathf.Max(0f,ReadFloat("dampingPerMeter",.02f));
        p.patternVariation = Mathf.Clamp(ReadFloat("patternVariation",.08f),0f,.15f);
        p.initialCY = ReadFloat("initialCY",0f);
        p.rough1 = ReadFloat("firstRoughness",.08f);
        p.rough2 = ReadFloat("secondRoughness",.22f);
        p.rough3 = ReadFloat("thirdRoughness",.12f);
        p.roughGain = ReadFloat("measuredRoughnessGain",.10f);
        p.roughReference = Mathf.Max(.01f,ReadFloat("roughnessReference",.40f));
        p.roughFollowHz = Mathf.Max(.01f,ReadFloat("roughFollowHz",12f));
        p.maximumResidual = Mathf.Max(0f,ReadFloat("maximumResidual",.20f));
        p.entryBlendSeconds = Mathf.Max(.01f,ReadFloat("entryBlendSeconds",.10f));
        p.referencePlanarSpeed = Mathf.Max(1f,ReadFloat("referencePlanarSpeed",18f));
        p.probeSpacing = Mathf.Max(.05f,ReadFloat("probeSpacing",.45f));
        p.entryRoughness = Mathf.Max(0f,entry.roughness);
        p.gravityN = Mathf.Max(.1f,-Vector3.Dot(Physics.gravity,entry.normal));
        float measuredNormalSpeed = Mathf.Max(0f,Vector3.Dot(entry.velocity,entry.normal));
        p.initialWaveSpeed = measuredNormalSpeed >.1f ? measuredNormalSpeed:fallbackNormalSpeed;
        float ballisticRise = p.initialWaveSpeed * p.initialWaveSpeed / (2f * p.gravityN);
        float a1 = Mathf.Max(.02f,Mathf.Lerp(initialHeight,ballisticRise,energyBlend));
        float minimumEntrySpeed = Mathf.Max(0f,ReadFloat("minimumEntryPlanarSpeed",2f));
        float referenceSpeed = p.referencePlanarSpeed;
        float entrySlope = Mathf.Clamp(ReadFloat("entrySlopeDegrees",12f),1f,60f);
        float heightInfluence = Mathf.Clamp(ReadFloat("motionHeightInfluence",.06f),0f,.15f);
        Vector3 planarVelocity = Vector3.ProjectOnPlane(entry.velocity,Vector3.up);
        float speed01 = Mathf.InverseLerp(minimumEntrySpeed,referenceSpeed * 1.35f,planarVelocity.magnitude);
        float slope01 = Mathf.InverseLerp(entrySlope,45f,Mathf.Clamp(entry.slopeDegrees,entrySlope,45f));
        float rough01 = Mathf.Clamp01(p.entryRoughness / p.roughReference);
        Vector3 planarTangent = Vector3.ProjectOnPlane(entry.tangent,Vector3.up);
        float alignment01 = 1f;
        if (planarVelocity.sqrMagnitude >.01f && planarTangent.sqrMagnitude >.01f)
        {
            alignment01 = Mathf.Clamp01(Vector3.Dot(planarVelocity.normalized,planarTangent.normalized));
        }
        float motionCharacter = Mathf.Clamp01(.50f * speed01 +.25f * slope01 +.15f * rough01 +.10f * alignment01);
        float characterSigned = (motionCharacter -.5f) * 2f;
        a1 *= 1f + characterSigned * heightInfluence;
        p.motionPattern = motionCharacter <.34f ? 0:motionCharacter <.67f ? 1:2;
        float baseWaveLength = p.baseSectionLength / 3f;
        float retention = p.restitution * p.restitution * Mathf.Exp(-p.dampingPerMeter * baseWaveLength);
        float motionSign = Mathf.Clamp(p.motionPattern - 1f,-1f,1f);
        p.a1 = a1;
        p.a2 = p.a1 * retention * (1f + motionSign * p.patternVariation);
        p.a3 = p.a2 * retention * (1f - motionSign * p.patternVariation *.55f);
        p.a2 = Mathf.Min(p.a2,p.a1 *.96f);
        p.a3 = Mathf.Min(p.a3,p.a2 *.96f);
        return p;
    }
    Vector3[] BuildBasePartPoints(
        StairPair pair,
        WavePlan plan,
        Vector3 stairT,
        Vector3 stairN,
        EntryPrediction entry,
        int knotCount,
        out Vector3[] predictedNormalLocal,
        out Vector3[] carrierTangentLocal,
        out float[] normalDistance,
        out float[] traveledDistance)
    {
        Vector3[] points = new Vector3[knotCount];
        predictedNormalLocal = new Vector3[knotCount];
        carrierTangentLocal = new Vector3[knotCount];
        normalDistance = new float[knotCount];
        traveledDistance = new float[knotCount];
        float startProgress = pair.index / (float)plan.multiplier;
        float endProgress = (pair.index + 1) / (float)plan.multiplier;

        for (int i = 0;i < knotCount;i++)
        {
            float local01 = i / (float)(knotCount - 1);
            float progress = Mathf.Lerp(startProgress,endProgress,local01);
            float traveled = progress * plan.totalLength;

            Vector3 carrier =
                EvaluateCarrier(entry,stairT,traveled,out Vector3 carrierT);

            Vector3 waveN =
                EvaluateFutureNormal(entry,stairN,carrierT,traveled);

            float wave =
                EvaluateWaveOffset(plan,progress,traveled,entry);

            Vector3 physicsPoint = carrier + waveN * wave;

            if (includeInitialCY)
            {
                physicsPoint += Vector3.up * plan.initialCY;
            }

            points[i] = PhysicsToVisual(pair,physicsPoint);

            Vector3 visualNormal = NormalizeSafe(
                PhysicsDirectionToVisual(pair,waveN),
                pair.visual.up);

            Vector3 visualTangent = NormalizeSafe(
                PhysicsDirectionToVisual(pair,carrierT),
                pair.visual.forward);

            predictedNormalLocal[i] =
                pair.visual.InverseTransformDirection(visualNormal);

            carrierTangentLocal[i] =
                pair.visual.InverseTransformDirection(visualTangent);

            // supportHeight is the existing ball-center/support offset.
            // Rotate it together with the wave; do not add another radius.
            normalDistance[i] =
                entry.supportHeight + wave;

            traveledDistance[i] = traveled;
        }

        return points;
    }

    static void BuildSplineFromPoints(
        SplineContainer container,
        Vector3[] points,
        Vector3 fallbackT)
    {
        Spline spline = container.Spline;
        spline.Clear();
        spline.Closed = false;

        for (int i = 0;i < points.Length;i++)
        {
            Vector3 point = points[i];
            Vector3 previous = i > 0 ? points[i - 1]:point;
            Vector3 next = i < points.Length - 1 ? points[i + 1]:point;
            Vector3 tangent = NormalizeSafe(next - previous,fallbackT);
            float inLength =
                i > 0 ? Vector3.Distance(previous,point) / 3f:0f;
            float outLength =
                i < points.Length - 1 ? Vector3.Distance(point,next) / 3f:0f;

            AddKnot(
                container,
                spline,
                point,
                -tangent * inLength,
                tangent * outLength);
        }
    }

    Vector3 EvaluateCarrier(EntryPrediction entry,Vector3 stairT,float traveled,out Vector3 tangent)
    {
        float flatLead = Mathf.Clamp(entry.distanceToBoundary,0f,1.5f);
        if (traveled <= flatLead)
        {
            tangent = entry.heading;
            return entry.position + entry.heading * traveled;
        }
        float transitionDistance = Mathf.Max(.25f,carrierSettleDistance);
        float transitionS = traveled - flatLead;
        Vector3 boundaryPoint = entry.position + entry.heading * flatLead;
        if (transitionS < transitionDistance)
        {
            float u = Mathf.Clamp01(transitionS / transitionDistance);
            Vector3 p0 = boundaryPoint;
            Vector3 p1 = boundaryPoint + stairT * transitionDistance;
            Vector3 m0 = entry.heading * transitionDistance;
            Vector3 m1 = stairT * transitionDistance;
            Vector3 point = HermiteVector(p0,p1,m0,m1,u);
            Vector3 derivative = HermiteVectorDerivative(p0,p1,m0,m1,u);
            tangent = NormalizeSafe(derivative,stairT);
            return point;
        }
        tangent = stairT;
        return boundaryPoint + stairT * traveled - stairT * flatLead;
    }
    Vector3 EvaluateFutureNormal(EntryPrediction entry,Vector3 stairN,Vector3 carrierT,float traveled)
    {
        float transitionEnd = Mathf.Max(.25f,entry.distanceToBoundary + carrierSettleDistance);
        float u = Mathf.Clamp01(traveled / transitionEnd);
        Vector3 blended = Vector3.Slerp(entry.normal,stairN,Mathf.SmoothStep(0f,1f,u));
        return NormalizeSafe(Vector3.ProjectOnPlane(blended,carrierT),stairN);
    }
    float EvaluateWaveOffset(WavePlan p,float progress,float traveled,EntryPrediction entry)
    {
        ResolveWave(p,progress,out int waveIndex,out int waveType,out int priorType,out float u,out float waveLength);
        float fade = Mathf.Exp(-(p.repeat23 > 0 ? 0f:p.dampingPerMeter * traveled));
        float current = AmplitudeAt(p,waveType) * fade;
        float prior = AmplitudeAt(p,priorType) * fade;
        float start = waveIndex == 0 ? 0f:-prior * p.lowerRatio;
        float end = -current * p.lowerRatio;
        float apex = PatternApex(p,waveType);
        float speed = PredictSlopeSpeed(p,entry,traveled);
        float elapsed = traveled / Mathf.Max(.1f,.5f * (entry.velocity.magnitude + speed));
        float probePersistence = 2f * p.probeSpacing / Mathf.Max(.1f,entry.velocity.magnitude);
        float roughDecayTime = Mathf.Max(0f,elapsed - probePersistence);
        float predictedRoughness = p.entryRoughness * Mathf.Exp(-2f * Mathf.PI * p.roughFollowHz * roughDecayTime);
        float rough01 = Mathf.Clamp01(predictedRoughness / p.roughReference);
        float rough = (waveType == 0 ? p.rough1:waveType == 1 ? p.rough2:p.rough3) + p.roughGain * rough01;
        float priorImpact = Mathf.Sqrt(2f * p.gravityN * prior * (1f + p.lowerRatio));
        float impact = Mathf.Sqrt(2f * p.gravityN * current * (1f + p.lowerRatio));
        float riseSpeed = waveIndex == 0 ? p.initialWaveSpeed:p.restitution * priorImpact;
        float period = Mathf.Max(.001f,waveLength) / Mathf.Max(.1f,speed);
        float y = EvaluateHermiteWave(u,start,current,end,apex,riseSpeed,impact,period,rough);
        float lowerBase = -Mathf.Lerp(waveIndex == 0 ? current:prior,current,Mathf.SmoothStep(0f,1f,u)) * p.lowerRatio;
        float upper = current * (1f + Mathf.Abs(rough)) + p.maximumResidual;
        float lower = lowerBase - p.maximumResidual;
        y = Mathf.Clamp(y,lower,upper);
        float entryBlend = Mathf.SmoothStep(0f,1f,elapsed / p.entryBlendSeconds);
        return y * entryBlend;
    }
    static float PredictSlopeSpeed(WavePlan p,EntryPrediction entry,float traveled)
    {
        float entrySpeed = Mathf.Max(.1f,entry.velocity.magnitude);
        float targetSpeed = Mathf.Max(entrySpeed,p.referencePlanarSpeed * 1.35f);
        float settle = 1f - Mathf.Exp(-.35f * Mathf.Max(0f,traveled));
        return Mathf.Lerp(entrySpeed,targetSpeed,settle);
    }
    static void ResolveWave(WavePlan p,float progress,out int waveIndex,out int waveType,out int priorType,out float u,out float waveLength)
    {
        progress = Mathf.Clamp01(progress);
        if (p.repeat23 == 0)
        {
            ResolveUniform(progress,3,0,0,0,out waveIndex,out waveType,out priorType,out u);
            waveLength = p.totalLength / 3f;
            return;
        }
        if (progress < p.fBoundary)
        {
            ResolveUniform(progress / p.fBoundary,3,0,0,0,out waveIndex,out waveType,out priorType,out u);
            waveLength = p.totalLength * p.fBoundary / 3f;
            return;
        }
        float fg = Mathf.Clamp01((progress - p.fBoundary) / (1f - p.fBoundary));
        float scaled = fg * p.repeat23;
        int chunk = fg >= 1f ? p.repeat23 - 1:Mathf.Clamp(Mathf.FloorToInt(scaled),0,p.repeat23 - 1);
        float chunk01 = fg >= 1f ? 1f:scaled - chunk;
        ResolveUniform(chunk01,2,1,3 + chunk * 2,2,out waveIndex,out waveType,out priorType,out u);
        waveLength = p.totalLength * (1f - p.fBoundary) / (p.repeat23 * 2f);
    }
    static void ResolveUniform(float local01,int phaseCount,int firstType,int baseIndex,int firstPrior,out int index,out int type,out int priorType,out float u)
    {
        local01 = Mathf.Clamp01(local01);
        int localIndex;
        if (local01 >= 1f)
        {
            localIndex = phaseCount - 1;
            u = 1f;
        }
        else
        {
            float scaled = local01 * phaseCount;
            localIndex = Mathf.Clamp(Mathf.FloorToInt(scaled),0,phaseCount - 1);
            u = scaled - localIndex;
        }
        index = baseIndex + localIndex;
        type = firstType + localIndex;
        priorType = localIndex == 0 ? firstPrior:type - 1;
    }
    static float AmplitudeAt(WavePlan p,int type)
    {
        return type <= 0 ? p.a1:type == 1 ? p.a2:p.a3;
    }
    static float PatternApex(WavePlan p,int type)
    {
        if (type <= 0) return p.firstApex;
        float motionSign = Mathf.Clamp(p.motionPattern - 1f,-1f,1f);
        float waveSign = type == 1 ? -1f:1f;
        return Mathf.Clamp(p.laterApex + motionSign * p.patternVariation *.45f * waveSign,.35f,.65f);
    }
    static float EvaluateHermiteWave(float u,float start,float peak,float end,float apex,float riseSpeed,float impactSpeed,float period,float rough)
    {
        float risingTangent = Mathf.Min(Mathf.Max(0f,riseSpeed * period * apex),3f * Mathf.Max(0f,peak - start));
        float fallingTangent = Mathf.Min(Mathf.Max(0f,impactSpeed * period * (1f - apex)),3f * Mathf.Max(0f,peak - end));
        float y = u < apex ? Hermite(start,peak,risingTangent,0f,u / apex):Hermite(peak,end,0f,-fallingTangent,(u - apex) / (1f - apex));
        float window = Mathf.Pow(Mathf.Sin(Mathf.PI * u) * Mathf.Sin(Mathf.PI * (u - apex)),2f);
        return y + peak * rough * window * Mathf.Sin(2f * Mathf.PI * u);
    }
    static float Hermite(float a,float b,float da,float db,float u)
    {
        float u2 = u * u;
        float u3 = u2 * u;
        return (2f * u3 - 3f * u2 + 1f) * a + (u3 - 2f * u2 + u) * da + (3f * u2 - 2f * u3) * b + (u3 - u2) * db;
    }
    static Vector3 HermiteVector(Vector3 p0,Vector3 p1,Vector3 m0,Vector3 m1,float u)
    {
        float u2 = u * u;
        float u3 = u2 * u;
        return (2f * u3 - 3f * u2 + 1f) * p0 + (u3 - 2f * u2 + u) * m0 + (-2f * u3 + 3f * u2) * p1 + (u3 - u2) * m1;
    }
    static Vector3 HermiteVectorDerivative(Vector3 p0,Vector3 p1,Vector3 m0,Vector3 m1,float u)
    {
        float u2 = u * u;
        return (6f * u2 - 6f * u) * p0 + (3f * u2 - 4f * u + 1f) * m0 + (-6f * u2 + 6f * u) * p1 + (3f * u2 - 2f * u) * m1;
    }
    bool TryGetObservedWorldPoint(out Vector3 point)
    {
        point = Vector3.zero;

        if (!observedDisplayTarget)
        {
            if (observedDisplayTargetField == null)
            {
                observedDisplayTargetField = source.GetType().GetField(
                    "actualDisplayTarget",
                    FieldFlags);
            }

            if (observedDisplayTargetField == null) return false;
            observedDisplayTarget =
                observedDisplayTargetField.GetValue(source) as Transform;
        }

        if (!observedDisplayTarget) return false;

        point = observedDisplayTarget.position;
        return IsFinite(point);
    }

    void EnsureObservedSpline()
    {
        if (observedSpline) return;

        Transform existing = FindDescendant(
            generatedVisualPlayer,
            "EqualizerObservedSpline");

        if (existing)
        {
            observedSpline = existing.GetComponent<SplineContainer>();
        }

        if (!observedSpline)
        {
            GameObject obj = new GameObject("EqualizerObservedSpline");
            obj.transform.SetParent(generatedVisualPlayer,false);
            obj.transform.localPosition = Vector3.zero;
            obj.transform.localRotation = Quaternion.identity;
            obj.transform.localScale = Vector3.one;
            observedSpline = obj.AddComponent<SplineContainer>();
            observedSpline.Spline = new Spline();
        }

        if (observedSpline.Spline == null)
            observedSpline.Spline = new Spline();

        Transform cube = FindDescendant(
            observedSpline.transform,
            "ObservedTipCube");

        if (cube)
        {
            observedTipCube = cube;
        }
        else
        {
            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Cube);
            marker.name = "ObservedTipCube";
            marker.transform.SetParent(observedSpline.transform,true);
            SetWorldUniformScale(marker.transform,observedTipCubeScale);
            Collider markerCollider = marker.GetComponent<Collider>();
            if (markerCollider)
            {
                markerCollider.enabled = false;
                Destroy(markerCollider);
            }
            observedTipCube = marker.transform;
        }
    }

    void UpdateObservedWaveEntryState()
    {
        bool active = source && source.IsActive;

        if (active && !previousSourceActive)
        {
            Vector3 fallbackVelocity =
                body ? body.velocity : Vector3.forward;

            Vector3 entryT = NormalizeSafe(
                source.StableT,
                fallbackVelocity);

            Vector3 entryN = NormalizeSafe(
                Vector3.ProjectOnPlane(source.StableN,entryT),
                Vector3.up);

            if (Vector3.Dot(entryN,Vector3.up) < 0f)
                entryN = -entryN;

            Vector3 entryB = NormalizeSafe(
                Vector3.Cross(entryN,entryT),
                source.StableB);

            if (source.StableB.sqrMagnitude > Eps &&
                Vector3.Dot(entryB,source.StableB) < 0f)
            {
                entryB = -entryB;
            }

            entryT = NormalizeSafe(
                Vector3.Cross(entryB,entryN),
                entryT);

            Vector3 entryVelocity =
                body ? body.velocity : Vector3.zero;

            string entryGroup = "";
            if (TryGetClosestObservedStair(
                    out string closestGroup,
                    out int closestIndex) &&
                closestIndex == 0)
            {
                entryGroup = closestGroup;
            }
            else if (!string.IsNullOrEmpty(lastGeneratedGroup))
            {
                entryGroup = lastGeneratedGroup;
            }

            observedWaveEntryGroup = entryGroup;
            observedWaveEntryTangent = entryT;
            observedWaveEntryNormal = entryN;
            observedWaveEntryBinormal = entryB;
            observedWaveEntryVelocity = entryVelocity;
            observedWaveEntryForwardSpeed =
                Vector3.Dot(entryVelocity,entryT);
            observedWaveEntryRevision++;

            // actualDisplayTarget は ExampleBVE の LateUpdate 後に確定する。
            // FixedUpdate では T/N/velocity だけをラッチし、同じフレームの
            // 表示位置 P0 はこのクラスの LateUpdate で受け取る。
            waveEntryPointCapturePending = true;
            hasObservedWaveEntry = false;
        }

        previousSourceActive = active;
    }

    void FinalizeObservedWaveEntryPoint(Vector3 point)
    {
        if (!waveEntryPointCapturePending)
            return;

        observedWaveEntryPoint = point;

        if (string.IsNullOrEmpty(observedWaveEntryGroup))
        {
            if (TryGetClosestObservedStair(
                    out string closestGroup,
                    out int closestIndex) &&
                closestIndex == 0)
            {
                observedWaveEntryGroup = closestGroup;
            }
            else if (!string.IsNullOrEmpty(lastGeneratedGroup))
            {
                observedWaveEntryGroup = lastGeneratedGroup;
            }
        }

        waveEntryPointCapturePending = false;
        hasObservedWaveEntry = true;

        if (enableLog)
        {
            Debug.Log(
                $"[EqualizerFutureSpline][WAVE_ENTRY_CAPTURE] " +
                $"revision={observedWaveEntryRevision} " +
                $"group={observedWaveEntryGroup} " +
                $"point={observedWaveEntryPoint:F4} " +
                $"T={observedWaveEntryTangent:F4} " +
                $"N={observedWaveEntryNormal:F4} " +
                $"B={observedWaveEntryBinormal:F4} " +
                $"velocity={observedWaveEntryVelocity:F4} " +
                $"forwardSpeed={observedWaveEntryForwardSpeed:F3}",
                this);
        }
    }

    void UpdateObservedStairRetention(Vector3 point)
    {
        if (!knotDetector) return;

        NearestKnotDetector.GuideFrame guide =
            knotDetector.CurrentGuide;

        if (!guide.valid)
            return;

        bool isSlope = guide.isSlope;
        bool turning = IsVisualFrameTurning();

        if (!courseGuideStateInitialized)
        {
            courseGuideStateInitialized = true;
            previousGuideWasSlope = isSlope;

            // 初回も含め、次に実際のSlopeへ入った時点を認識点にする。
            courseRecognitionArmed = true;
        }

        // 角度やforwardから「曲がるはず」と予測しない。
        // 前回のコース確定後にVisualFrame旋回が実際に起きたかだけを記録する。
        if (turning)
            courseSawVisualTurn = true;

        // Stair -> Flat に出たら、次コースの認識を必ず待ち状態へ戻す。
        // Flat上では最寄りStairWayを使ってrawDeltaを確定しない。
        if (previousGuideWasSlope && !isSlope)
            courseRecognitionArmed = true;

        // rawDeltaを取るのは「実際にSlope上」「旋回は終了済み」の両方を満たす時だけ。
        // 直進コースではturningが一度もtrueにならないので、そのままここへ入る。
        // 曲がるコースではturning中は待ち、falseへ戻った後にここへ入る。
        if (courseRecognitionArmed &&
            isSlope &&
            !turning)
        {
            if (!TryGetClosestObservedStair(
                    out string groupName,
                    out int childIndex))
            {
                previousGuideWasSlope = isSlope;
                return;
            }

            // 実際に次のStairWayの先頭区間へ入った時だけ確定する。
            if (childIndex != 0)
            {
                previousGuideWasSlope = isSlope;
                return;
            }

            // Futureがまだ生成されていないなら、この認識は消費しない。
            if (!TryGetGeneratedGroup(
                    groupName,
                    out GeneratedGroup generated) ||
                generated.parts.Count == 0)
            {
                previousGuideWasSlope = isSlope;
                return;
            }

            string previousGroup =
                observedCurrentStairGroup;

            bool groupChanged =
                string.IsNullOrEmpty(previousGroup) ||
                groupName != previousGroup;

            if (groupChanged)
            {
                observedCurrentStairGroup = groupName;

                if (!string.IsNullOrEmpty(previousGroup))
                    ResetObservedSpline(point);
            }

            // FutureBall は「コース確定時の現在フレーム」ではなく、
            // ExampleBVE が実際に波を開始した瞬間の P0/T0/N0 を使う。
            // これにより旋回完了待ちで初期条件が上書きされるのを防ぐ。
            if (!hasObservedWaveEntry)
            {
                previousGuideWasSlope = isSlope;
                return;
            }

            if (string.IsNullOrEmpty(observedWaveEntryGroup))
                observedWaveEntryGroup = groupName;

            if (observedWaveEntryGroup != groupName)
            {
                previousGuideWasSlope = isSlope;
                return;
            }

            string courseMode =
                courseSawVisualTurn
                    ? "TurnCompleted"
                    : "StraightNoTurn";

            Vector3 confirmedObservedTangent = NormalizeSafe(
                observedWaveEntryTangent,
                source ? source.StableT : guide.tangent);

            Vector3 confirmedObservedNormal = NormalizeSafe(
                Vector3.ProjectOnPlane(
                    observedWaveEntryNormal,
                    confirmedObservedTangent),
                guide.normal);

            if (Vector3.Dot(confirmedObservedNormal,Vector3.up) < 0f)
                confirmedObservedNormal = -confirmedObservedNormal;

            ApplyObservedAlignmentToFutureBall(
                groupName,
                observedWaveEntryPoint,
                confirmedObservedTangent,
                confirmedObservedNormal,
                observedWaveEntryForwardSpeed);

            if (!generated.alignmentApplied)
            {
                previousGuideWasSlope = isSlope;
                return;
            }

            if (enableLog)
            {
                Debug.Log(
                    $"[EqualizerFutureSpline][COURSE_CONFIRMED] " +
                    $"group={groupName} " +
                    $"mode={courseMode} " +
                    $"turnSeen={courseSawVisualTurn} " +
                    $"turning={turning} " +
                    $"point={point:F4} " +
                    $"entryPoint={observedWaveEntryPoint:F4} " +
                    $"confirmedT={confirmedObservedTangent:F4} " +
                    $"confirmedN={confirmedObservedNormal:F4} " +
                    $"entryForwardSpeed={observedWaveEntryForwardSpeed:F3}",
                    this);

                if (groupChanged &&
                    !string.IsNullOrEmpty(previousGroup))
                {
                    Debug.Log(
                        $"[EqualizerFutureSpline][OBSERVED_NEXT_STAIR_ZERO] " +
                        $"from={previousGroup} to={groupName} " +
                        $"point={point:F4} " +
                        $"course={courseMode}",
                        this);
                }
            }

            courseRecognitionArmed = false;
            courseSawVisualTurn = false;
            hasObservedWaveEntry = false;
        }

        previousGuideWasSlope = isSlope;
    }

    bool IsVisualFrameTurning()
    {
        return
            correspondSubject != null &&
            correspondSubject.IsVisualFrameTurning;
    }

    void ApplyObservedAlignmentToFutureBall(
        string groupName,
        Vector3 observedAnchorWorld,
        Vector3 observedTangentWorld,
        Vector3 observedNormalWorld,
        float observedForwardSpeed)
    {
        if (string.IsNullOrEmpty(groupName))
            return;

        if (!TryGetGeneratedGroup(groupName,out GeneratedGroup generated) ||
            generated.parts.Count == 0)
        {
            if (enableLog)
            {
                Debug.Log(
                    $"[EqualizerFutureSpline][OBSERVED_FUTURE_DELTA] " +
                    $"group={groupName} found=False observedAnchor={observedAnchorWorld:F4}",
                    this);
            }
            return;
        }

        if (generated.alignmentApplied)
            return;

        GeneratedPart anchorPart = generated.parts[0];
        if (!TryGetFirstKnotWorldPoint(
                anchorPart.future,
                out Vector3 futureAnchorWorld))
        {
            return;
        }

        observedTangentWorld = NormalizeSafe(
            observedTangentWorld,
            Vector3.forward);

        observedNormalWorld = NormalizeSafe(
            Vector3.ProjectOnPlane(
                observedNormalWorld,
                observedTangentWorld),
            Vector3.up);

        if (Vector3.Dot(observedNormalWorld,Vector3.up) < 0f)
            observedNormalWorld = -observedNormalWorld;

        // Nだけを後から回す方式は、Tが違う区間でProjectOnPlaneにより
        // 補正がほぼ消える場合があった。今回は波開始時の実測 T0/N0 を
        // 1つの直交フレームとして扱い、FutureBallの入口側だけを補正する。
        bool rebuiltEntryFrame =
            RebuildFutureBallEntryFrame(
                generated,
                observedTangentWorld,
                observedNormalWorld,
                out Vector3 predictedAnchorTangentWorld,
                out Vector3 predictedAnchorNormalWorld,
                out float tangentErrorDegrees,
                out float normalErrorDegrees,
                out float entryFrameRotationDegrees,
                out float maxTangentCorrection,
                out float maxNormalCorrection);

        // T/N再構築後のFutureBall先頭点を位置合わせの基準にする。
        // P0は最後にrawDeltaだけで一致させ、フレーム補正量は消さない。
        Vector3 correctedBallAnchorBeforeWorld =
            futureAnchorWorld;

        if (!TryGetFirstKnotWorldPoint(
                anchorPart.ball,
                out correctedBallAnchorBeforeWorld))
        {
            correctedBallAnchorBeforeWorld =
                futureAnchorWorld;
        }

        Vector3 rawDelta =
            observedAnchorWorld - correctedBallAnchorBeforeWorld;

        for (int i = 0;i < generated.parts.Count;i++)
        {
            GeneratedPart part = generated.parts[i];
            if (!part.ball)
                continue;

            part.ball.transform.position += rawDelta;
        }

        generated.alignmentApplied = true;
        generated.alignmentDeltaWorld = rawDelta;

        TryGetFirstKnotWorldPoint(
            anchorPart.ball,
            out Vector3 ballAnchorWorld);

        float reconstructionError =
            Vector3.Distance(
                ballAnchorWorld,
                observedAnchorWorld);

        Transform parent =
            anchorPart.future && anchorPart.future.transform.parent
                ? anchorPart.future.transform.parent
                : null;

        Vector3 expectedLocalShift =
            parent
                ? parent.InverseTransformVector(rawDelta)
                : rawDelta;

        Vector3 actualLocalShift =
            anchorPart.ball && anchorPart.future
                ? anchorPart.ball.transform.localPosition -
                  anchorPart.future.transform.localPosition
                : Vector3.zero;

        float supportAtAnchor =
            anchorPart.normalDistance != null &&
            anchorPart.normalDistance.Length > 0
                ? anchorPart.normalDistance[0]
                : 0f;

        float predictedForwardSpeed =
            generated.predictedEntryForwardSpeed;

        float forwardSpeedError =
            observedForwardSpeed - predictedForwardSpeed;

        observedFutureComparedSpline =
            anchorPart.future ? anchorPart.future.name : "";
        observedFutureAnchorWorld = futureAnchorWorld;
        observedFutureRawDelta = rawDelta;
        observedFutureAlignmentApplied = true;
        observedFutureConfirmedTangent = observedTangentWorld;
        observedFutureConfirmedNormal = observedNormalWorld;
        observedFutureEntryFrameRotationDegrees = entryFrameRotationDegrees;
        observedFutureTangentErrorDegrees = tangentErrorDegrees;
        observedFutureMaxTangentCorrection = maxTangentCorrection;
        observedFutureMaxNormalCorrection = maxNormalCorrection;
        observedFutureAnchorError = reconstructionError;
        mappedFirstBallPoint = ballAnchorWorld;

        if (enableLog)
        {
            string parentName =
                parent ? parent.name : "<none>";

            Debug.Log(
                $"[EqualizerFutureSpline][ENTRY_FRAME_ALIGNMENT] " +
                $"group={groupName} " +
                $"predictedT={predictedAnchorTangentWorld:F4} " +
                $"observedT={observedTangentWorld:F4} " +
                $"tangentErrorBefore={tangentErrorDegrees:F3} " +
                $"predictedN={predictedAnchorNormalWorld:F4} " +
                $"observedN={observedNormalWorld:F4} " +
                $"normalErrorBefore={normalErrorDegrees:F3} " +
                $"entryFrameRotationDeg={entryFrameRotationDegrees:F3} " +
                $"predictedForwardSpeed={predictedForwardSpeed:F3} " +
                $"observedForwardSpeed={observedForwardSpeed:F3} " +
                $"forwardSpeedError={forwardSpeedError:F3} " +
                $"maxTangentCorrection={maxTangentCorrection:F4} " +
                $"maxNormalCorrection={maxNormalCorrection:F4} " +
                $"entryFrameRebuilt={rebuiltEntryFrame}",
                this);

            Debug.Log(
                $"[EqualizerFutureSpline][OBSERVED_FUTURE_DELTA] " +
                $"group={groupName} " +
                $"future={anchorPart.future.name} " +
                $"ball={anchorPart.ball.name} " +
                $"parent={parentName} " +
                $"futureAnchor={futureAnchorWorld:F4} " +
                $"correctedBallAnchorBefore={correctedBallAnchorBeforeWorld:F4} " +
                $"observedAnchor={observedAnchorWorld:F4} " +
                $"rawDelta={rawDelta:F4} " +
                $"supportAtAnchor={supportAtAnchor:F4} " +
                $"ballAnchorAfter={ballAnchorWorld:F4} " +
                $"reconstructionError={reconstructionError:F6} " +
                $"expectedLocalShift={expectedLocalShift:F4} " +
                $"actualLocalShift={actualLocalShift:F4} " +
                $"appliedToBall=True",
                this);
        }
    }

    bool RebuildFutureBallEntryFrame(
        GeneratedGroup generated,
        Vector3 observedTangentWorld,
        Vector3 observedNormalWorld,
        out Vector3 predictedAnchorTangentWorld,
        out Vector3 predictedAnchorNormalWorld,
        out float tangentErrorDegrees,
        out float normalErrorDegrees,
        out float entryFrameRotationDegrees,
        out float maxTangentCorrection,
        out float maxNormalCorrection)
    {
        predictedAnchorTangentWorld = Vector3.forward;
        predictedAnchorNormalWorld = Vector3.up;
        tangentErrorDegrees = 0f;
        normalErrorDegrees = 0f;
        entryFrameRotationDegrees = 0f;
        maxTangentCorrection = 0f;
        maxNormalCorrection = 0f;

        if (generated == null || generated.parts.Count == 0)
            return false;

        GeneratedPart anchorPart = generated.parts[0];
        if (!HasEntryFrameData(anchorPart))
            return false;

        predictedAnchorTangentWorld = NormalizeSafe(
            anchorPart.future.transform.TransformDirection(
                anchorPart.carrierTangentLocal[0]),
            Vector3.forward);

        predictedAnchorNormalWorld = NormalizeSafe(
            Vector3.ProjectOnPlane(
                anchorPart.future.transform.TransformDirection(
                    anchorPart.predictedNormalLocal[0]),
                predictedAnchorTangentWorld),
            Vector3.up);

        if (Vector3.Dot(predictedAnchorNormalWorld,Vector3.up) < 0f)
            predictedAnchorNormalWorld = -predictedAnchorNormalWorld;

        observedTangentWorld = NormalizeSafe(
            observedTangentWorld,
            predictedAnchorTangentWorld);

        observedNormalWorld = NormalizeSafe(
            Vector3.ProjectOnPlane(
                observedNormalWorld,
                observedTangentWorld),
            predictedAnchorNormalWorld);

        if (Vector3.Dot(observedNormalWorld,Vector3.up) < 0f)
            observedNormalWorld = -observedNormalWorld;

        tangentErrorDegrees = Vector3.Angle(
            predictedAnchorTangentWorld,
            observedTangentWorld);

        normalErrorDegrees = Vector3.Angle(
            predictedAnchorNormalWorld,
            observedNormalWorld);

        Quaternion predictedFrame =
            Quaternion.LookRotation(
                predictedAnchorTangentWorld,
                predictedAnchorNormalWorld);

        Quaternion observedFrame =
            Quaternion.LookRotation(
                observedTangentWorld,
                observedNormalWorld);

        Quaternion entryFrameDelta =
            observedFrame * Quaternion.Inverse(predictedFrame);

        entryFrameRotationDegrees =
            Quaternion.Angle(
                Quaternion.identity,
                entryFrameDelta);

        float settleDistance =
            Mathf.Max(.25f,carrierSettleDistance);

        for (int p = 0;p < generated.parts.Count;p++)
        {
            GeneratedPart part = generated.parts[p];
            if (!HasEntryFrameData(part))
                continue;

            int count = Mathf.Min(
                part.future.Spline.Count,
                Mathf.Min(
                    part.predictedNormalLocal.Length,
                    Mathf.Min(
                        part.carrierTangentLocal.Length,
                        Mathf.Min(
                            part.normalDistance.Length,
                            part.traveledDistance.Length))));

            if (count < 2)
                continue;

            Vector3[] correctedWorldPoints =
                new Vector3[count];

            for (int k = 0;k < count;k++)
            {
                BezierKnot futureKnot =
                    part.future.Spline[k];

                Vector3 futureWorld =
                    part.future.transform.TransformPoint(
                        ToVector3(futureKnot.Position));

                Vector3 predictedT = NormalizeSafe(
                    part.future.transform.TransformDirection(
                        part.carrierTangentLocal[k]),
                    predictedAnchorTangentWorld);

                Vector3 predictedN = NormalizeSafe(
                    Vector3.ProjectOnPlane(
                        part.future.transform.TransformDirection(
                            part.predictedNormalLocal[k]),
                        predictedT),
                    predictedAnchorNormalWorld);

                if (Vector3.Dot(predictedN,Vector3.up) < 0f)
                    predictedN = -predictedN;

                float traveled =
                    Mathf.Max(0f,part.traveledDistance[k]);

                float u = Mathf.Clamp01(
                    traveled / settleDistance);

                float entryWeight =
                    1f - Mathf.SmoothStep(0f,1f,u);

                Quaternion localFrameDelta =
                    Quaternion.Slerp(
                        Quaternion.identity,
                        entryFrameDelta,
                        entryWeight);

                Vector3 rotatedT =
                    localFrameDelta * predictedT;

                Vector3 rotatedN =
                    localFrameDelta * predictedN;

                Vector3 correctedT = NormalizeSafe(
                    rotatedT,
                    predictedT);

                Vector3 correctedN = NormalizeSafe(
                    Vector3.ProjectOnPlane(
                        rotatedN,
                        correctedT),
                    rotatedN);

                if (Vector3.Dot(correctedN,rotatedN) < 0f)
                    correctedN = -correctedN;

                // T補正は入口からの進行距離に対してのみ作用する。
                // entryWeightが0になると元Futureへ滑らかに復帰するので、
                // 後半の階段形状や別partを丸ごと回転させない。
                Vector3 tangentCorrection =
                    (correctedT - predictedT) * traveled;

                Vector3 normalCorrection =
                    (correctedN - predictedN) *
                    part.normalDistance[k];

                correctedWorldPoints[k] =
                    futureWorld +
                    tangentCorrection +
                    normalCorrection;

                maxTangentCorrection = Mathf.Max(
                    maxTangentCorrection,
                    tangentCorrection.magnitude);

                maxNormalCorrection = Mathf.Max(
                    maxNormalCorrection,
                    normalCorrection.magnitude);
            }

            part.ball.transform.localPosition =
                part.future.transform.localPosition;
            part.ball.transform.localRotation =
                part.future.transform.localRotation;
            part.ball.transform.localScale =
                part.future.transform.localScale;

            Vector3 fallbackT = NormalizeSafe(
                correctedWorldPoints[1] - correctedWorldPoints[0],
                observedTangentWorld);

            BuildSplineFromPoints(
                part.ball,
                correctedWorldPoints,
                fallbackT);

            Transform startMarker =
                part.ball.transform.Find("BallStartKnot");

            if (startMarker && p == 0)
                startMarker.position = correctedWorldPoints[0];
        }

        return true;
    }

    static bool HasEntryFrameData(
        GeneratedPart part)
    {
        return
            part != null &&
            part.future &&
            part.ball &&
            part.future.Spline != null &&
            part.predictedNormalLocal != null &&
            part.carrierTangentLocal != null &&
            part.normalDistance != null &&
            part.traveledDistance != null &&
            part.predictedNormalLocal.Length > 0 &&
            part.carrierTangentLocal.Length > 0 &&
            part.normalDistance.Length > 0 &&
            part.traveledDistance.Length > 0;
    }

    bool TryGetGeneratedGroup(
        string groupName,
        out GeneratedGroup generated)
    {
        for (int i = history.Count - 1;i >= 0;i--)
        {
            GeneratedGroup candidate = history[i];
            if (candidate != null && candidate.name == groupName)
            {
                generated = candidate;
                return true;
            }
        }

        generated = null;
        return false;
    }

    static bool TryGetFirstKnotWorldPoint(
        SplineContainer container,
        out Vector3 worldPoint)
    {
        worldPoint = Vector3.zero;

        if (!container ||
            container.Spline == null ||
            container.Spline.Count == 0)
        {
            return false;
        }

        BezierKnot knot =
            container.Spline[0];

        worldPoint =
            container.transform.TransformPoint(
                ToVector3(knot.Position));

        return true;
    }

    bool TryGetClosestObservedStair(out string groupName,out int childIndex)
    {
        groupName = null;
        childIndex = -1;

        if (!generatedPhysicsRoot || !body) return false;
        EnsureObservedStairCache();
        if (observedStairNodes.Count == 0) return false;

        Vector3 position = body.position;
        float bestDistanceSq = float.PositiveInfinity;
        ObservedStairNode best = default;
        bool found = false;

        for (int i = 0;i < observedStairNodes.Count;i++)
        {
            ObservedStairNode node = observedStairNodes[i];
            if (!node.physics) continue;

            float distanceSq =
                (node.physics.position - position).sqrMagnitude;

            if (distanceSq >= bestDistanceSq) continue;
            bestDistanceSq = distanceSq;
            best = node;
            found = true;
        }

        if (!found) return false;
        groupName = best.group;
        childIndex = best.index;
        return true;
    }

    void EnsureObservedStairCache()
    {
        if (observedStairNodes.Count > 0) return;

        Transform[] nodes =
            generatedPhysicsRoot.GetComponentsInChildren<Transform>(true);

        for (int i = 0;i < nodes.Length;i++)
        {
            Transform node = nodes[i];
            if (!node ||
                !TryParsePhysicsStair(
                    node.name,
                    out string groupName,
                    out int childIndex))
            {
                continue;
            }

            observedStairNodes.Add(
                new ObservedStairNode
                {
                    physics = node,
                    group = groupName,
                    index = childIndex
                });
        }
    }

    void UpdateObservedTipCube(Vector3 point)
    {
        if (!observedTipCube) return;
        observedTipCube.position = point;
        SetWorldUniformScale(observedTipCube,observedTipCubeScale);
    }

    void AppendObservedPoint(Vector3 point)
    {
        if (!observedSpline) return;

        if (!hasObservedLastPoint)
        {
            ResetObservedSpline(point);
            return;
        }

        float step = Vector3.Distance(observedLastPoint,point);

        // 観測Splineは常にBallVisualEqualizerを追従する。
        // 旋回や座標フレーム更新で一時的に距離が大きくなっても、
        // 追記を停止しない。Splineを消すのは次の StairWayN_0 だけ。
        observedLatestPoint = point;
        if (step < observedMinimumSampleDistance) return;

        AddObservedKnot(point);
        observedPathLength += step;
        observedLastPoint = point;
    }

    void ResetObservedSpline(Vector3 point)
    {
        Spline spline = observedSpline.Spline;
        spline.Clear();
        spline.Closed = false;
        observedKnotCount = 0;
        observedPathLength = 0f;
        observedLastPoint = point;
        observedLatestPoint = point;
        hasObservedLastPoint = true;
        AddObservedKnot(point);

        if (enableLog)
        {
            Debug.Log(
                $"[EqualizerFutureSpline][OBSERVED_BEGIN] point={point:F4}",
                this);
        }
    }

    void AddObservedKnot(Vector3 worldPoint)
    {
        Transform frame = observedSpline.transform;
        Vector3 localPoint = frame.InverseTransformPoint(worldPoint);
        float3 zero = new float3(0f,0f,0f);

        observedSpline.Spline.Add(
            new BezierKnot(ToFloat3(localPoint),zero,zero),
            TangentMode.Broken);

        observedKnotCount = observedSpline.Spline.Count;
    }

    static Vector3 PhysicsToVisual(StairPair pair,Vector3 physicsWorldPoint)
    {
        Vector3 stairLocal = pair.physics.InverseTransformPoint(physicsWorldPoint);
        return pair.visual.TransformPoint(stairLocal);
    }

    static Vector3 PhysicsDirectionToVisual(StairPair pair,Vector3 physicsWorldDirection)
    {
        Vector3 stairLocal = pair.physics.InverseTransformDirection(physicsWorldDirection);
        return pair.visual.TransformDirection(stairLocal);
    }

    static SplineContainer CreateSpline(
        Transform visualParent,
        string groupName,
        int index,
        bool ballLane)
    {
        string prefix =
            ballLane ? "EqualizerFutureBallSpline":"EqualizerFutureSpline";

        GameObject obj =
            new GameObject($"{prefix}_{groupName}_{index}");

        obj.transform.SetParent(visualParent,false);
        obj.transform.localPosition = Vector3.zero;
        obj.transform.localRotation = Quaternion.identity;
        obj.transform.localScale = Vector3.one;

        SplineContainer spline = obj.AddComponent<SplineContainer>();
        spline.Spline = new Spline();
        return spline;
    }

    void CreateStartMarker(
        SplineContainer container,
        Vector3 worldPoint,
        string markerName)
    {
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.name = markerName;
        marker.transform.SetParent(container.transform,true);
        marker.transform.position = worldPoint;
        SetWorldUniformScale(marker.transform,startMarkerScale);

        Collider markerCollider = marker.GetComponent<Collider>();
        if (markerCollider)
        {
            markerCollider.enabled = false;
            Destroy(markerCollider);
        }
    }

    static void SetWorldUniformScale(Transform target,float scale)
    {
        Vector3 parentScale =
            target.parent ? target.parent.lossyScale:Vector3.one;

        target.localScale = new Vector3(
            scale / Mathf.Max(Eps,Mathf.Abs(parentScale.x)),
            scale / Mathf.Max(Eps,Mathf.Abs(parentScale.y)),
            scale / Mathf.Max(Eps,Mathf.Abs(parentScale.z)));
    }
    static void AddKnot(SplineContainer container,Spline spline,Vector3 visualWorldPoint,Vector3 tangentInWorld,Vector3 tangentOutWorld)
    {
        Transform frame = container.transform;
        spline.Add(new BezierKnot(ToFloat3(frame.InverseTransformPoint(visualWorldPoint)),ToFloat3(frame.InverseTransformVector(tangentInWorld)),ToFloat3(frame.InverseTransformVector(tangentOutWorld))),TangentMode.Broken);
    }
    void TrimHistory()
    {
        int keep = Mathf.Clamp(retainGroupCount,1,8);
        while (history.Count > keep)
        {
            GeneratedGroup old = history[0];
            history.RemoveAt(0);
            for (int i = 0;i < old.splines.Count;i++)
            {
                SplineContainer spline = old.splines[i];
                if (spline)
                {
                    Destroy(spline.gameObject);
                }
            }
        }
    }
    float ReadFloat(string fieldName,float fallback)
    {
        FieldInfo field = source.GetType().GetField(fieldName,FieldFlags);
        if (field == null || field.FieldType != typeof(float))
        {
            return fallback;
        }
        return (float)field.GetValue(source);
    }
    static bool TryParsePhysicsStair(string objectName,out string groupName,out int childIndex)
    {
        groupName = null;
        childIndex = -1;
        if (string.IsNullOrEmpty(objectName) || !objectName.EndsWith(PhysicsSuffix,System.StringComparison.Ordinal))
        {
            return false;
        }
        string core = objectName.Substring(0,objectName.Length - PhysicsSuffix.Length);
        int split = core.LastIndexOf('_');
        if (split <= 0 || !int.TryParse(core.Substring(split + 1),out childIndex))
        {
            return false;
        }
        groupName = core.Substring(0,split);
        return groupName.StartsWith(StairPrefix,System.StringComparison.Ordinal);
    }
    static Transform FindDescendant(Transform root,string exactName)
    {
        if (!root) return null;
        if (root.name == exactName)
        {
            return root;
        }
        for (int i = 0;i < root.childCount;i++)
        {
            Transform found = FindDescendant(root.GetChild(i),exactName);
            if (found) return found;
        }
        return null;
    }
    static float FollowAlpha(float hz,float dt)
    {
        return 1f - Mathf.Exp(-2f * Mathf.PI * Mathf.Max(.0001f,hz) * Mathf.Max(0f,dt));
    }
    static Vector3 NormalizeSafe(Vector3 value,Vector3 fallback)
    {
        if (value.sqrMagnitude > Eps)
        {
            return value.normalized;
        }
        if (fallback.sqrMagnitude > Eps)
        {
            return fallback.normalized;
        }
        return Vector3.forward;
    }
    static Vector3 ToVector3(float3 v)
    {
        return new Vector3(v.x,v.y,v.z);
    }

    static float3 ToFloat3(Vector3 v)
    {
        return new float3(v.x,v.y,v.z);
    }
}
