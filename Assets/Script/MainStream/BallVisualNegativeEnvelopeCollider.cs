using System.Collections.Generic;
using UnityEngine;

public enum EnvelopeFirstContactMode
{
    Soft,
    Balanced,
    Dynamic,
    AutoTime,
    AutoHeight,
    AutoSpeed
}

/// <summary>
/// BallVisualEqualizer用の -0側 Upper Envelope Collider。
///
/// SlopeStick3Dは完全READ ONLY。
///
/// Release時の未来軌道からFirst Contactを選び、
/// Selected First Contact -> Exact Limit に向かって
///
///     A(t) = A0 * exp(-Gamma * t)
///
/// で上側Colliderを狭めます。
///
/// 下包絡線は生成しません。
/// 下側境界は実階段Colliderが担当します。
///
/// BallVisualEqualizerがColliderに当たっても、
/// このクラスから特殊なThrow / AddForceは行いません。
/// 通常のUnity Collisionとして扱います。
/// </summary>
[DisallowMultipleComponent]
public sealed class BallVisualNegativeEnvelopeCollider : MonoBehaviour
{
    // ================================================================
    // References
    // ================================================================

    [Header("References")] [Tooltip("READ ONLYで参照します。このComponentからSlopeStick3Dを書き換えません。")] [SerializeField]
    private SlopeStick3D slopeStick;

    [Tooltip("PhysicsRoot座標 -> VisualPlayerRoot座標への変換に使用します。")] [SerializeField]
    private CorrespondSubject correspondSubject;

    [Tooltip("包絡線内を実際に跳ねるBallVisualEqualizerのRigidbodyです。")] [SerializeField]
    private Rigidbody ballVisualEqualizer;

    [Tooltip("BallVisualEqualizerのSphereColliderです。")] [SerializeField]
    private SphereCollider ballVisualEqualizerCollider;

    [SerializeField] private Collider inSubjectCollider;
    // ================================================================
    // Geometry only
    // 物理挙動を調整する値ではなく、Collider近似精度です。
    // ================================================================

    [Header("Envelope Geometry")] [Tooltip("指数Envelope Meshを進行方向に何分割してサンプリングするか。")] [Range(8, 64)] [SerializeField]
    private int segmentCount = 32;

    [Tooltip("進行方向と直交するEnvelopeの全幅[m]。中心から左右へ envelopeWidth / 2 ずつ広がります。")] [Min(0.1f)] [SerializeField]
    private float envelopeWidth = 10.0f;

    
    [Header("Envelope Runtime Tuning")]

    [Tooltip("Envelope全体を斜面法線方向へ上下させる距離[m]。Use Curved OffsetがOFFなら全頂点へ同じ量を適用します。")]
    [SerializeField]
    private float normalOffset = 0f;

    [Tooltip("OFF: Envelope全体を同じ量だけ上下。ON: 始点/終点では0、中央だけNormal Offsetを強くする湾曲モード。")]
    [SerializeField]
    private bool useCurvedOffset = false;

    [Tooltip("湾曲モードでNormal Offsetを両端から何%の範囲で0→100%へ馴染ませるか。")]
    [Range(0.01f, 0.49f)]
    [SerializeField]
    private float edgeBlendWidth = 0.15f;

    [Tooltip("A(t)=A0*exp(-Gamma*t) の振幅倍率。1=元の大きさ、2=2倍、0.5=半分。")]
    [Min(0f)]
    [SerializeField]
    private float amplitudeScale = 1f;

    [Tooltip("ONにするとWorld Y Correction RatioをEnvelope形状へ適用します。OFFなら従来どおり無効です。")]
    [SerializeField]
    private bool useWorldYCorrection = false;

    [Tooltip("指数振幅に比例するWorld +Y補正率。Use World Y CorrectionがONの時だけ使用します。")]
    [Range(0f, 30f)]
    [SerializeField]
    private float worldYCorrectionRatio = 0.15f;


    // ================================================================
    // First contact selection
    // ================================================================

    [Header("First Contact Selection")]
    [Tooltip(
        "次に生成するEnvelopeの初回接触方式。" +
        "Soft/Balanced/Dynamicは到達可能候補の相対的な法線強度から選びます。" +
        "固定目標角は使わず、実際のRelease軌道に合わせて接触表現を変えます。" +
        "AutoTime/AutoHeight/AutoSpeedも到達可能候補だけを比較します。" +
        "変更は次のEnvelope生成から反映されます。")]
    [SerializeField]
    private EnvelopeFirstContactMode firstContactMode =
        EnvelopeFirstContactMode.AutoSpeed;


    [Header("Envelope Generation History")]

    [Tooltip(
        "OFF: 平面→斜面などで次のEnvelopeを生成する時、以前に生成したEnvelopeを全て削除して最新1個だけ残します。\n" +
        "ON: 次のEnvelopeを生成しても以前のEnvelopeを削除せず、Slope区間ごとにドンドン追加して残します。\n" +
        "この設定はInspectorのリアルタイム形状調整とは別です。")]
    [SerializeField]
    private bool accumulateEnvelopesAcrossSlopes = false;


    // ================================================================
    // Runtime
    // ================================================================

    [Header("Runtime - Read Only")] [SerializeField]
    private bool armed;

    [SerializeField] private bool envelopeBuilt;

    [SerializeField] private float sourceEnergyJoule;

    [SerializeField] private float equalizerLaunchSpeed;

    [SerializeField] private float equalizerVerticalLaunchSpeed;

    [SerializeField] private float entryApexHeight;

    [SerializeField] private float gamma;

    [SerializeField] private float minimumFreeAmplitude;

    [SerializeField] private Vector3 capturedEntryPhysics;

    [SerializeField] private Vector3 capturedLimitPhysics;

    [SerializeField] private float capturedTargetProgress01;

    [Header("First Contact Runtime - Read Only")]
    [SerializeField] private bool firstContactSelected;
    [SerializeField] private float selectedFirstContactTime;
    [SerializeField] private float selectedFirstContactPathProgress01;
    [SerializeField] private float selectedFirstContactAngleDeg;
    [SerializeField] private float selectedFirstContactSeverity01;
    [SerializeField] private float selectedFirstContactNormalSpeed;
    [SerializeField] private float selectedFirstContactAmplitude;
    [SerializeField] private float firstContactMaximumAmplitude;
    [SerializeField] private Vector3 selectedFirstContactPositionVisual;
    [SerializeField] private Vector3 selectedFirstContactSurfacePhysics;
    [SerializeField] private Vector3 capturedEqualizerReleasePositionVisual;
    [SerializeField] private Vector3 capturedEqualizerLaunchVelocityVisual;


    // First Contactの表現値は固定角ではなく、到達可能候補の分布から決める。
    // 以下は数値探索・安全性だけの固定値。
    private const float FirstContactPredictionMaximumSeconds = 1.50f;
    private const int FirstContactPredictionSamples = 96;
    private const int FirstContactSolveIterations = 6;
    private const float FirstContactPlaneTolerance = 0.035f;
    private const float FirstContactMinimumRemainingLength = 0.10f;

    private struct FirstContactCandidate
    {
        public bool valid;
        public float time;
        public float pathProgress01;
        public float incidenceAngleDeg;
        public float normalSpeed;
        public float severity01;
        public float solvedA0;
        public float contactPlaneError;
        public Vector3 predictedPositionVisual;
        public Vector3 predictedVelocityVisual;
        public Vector3 surfacePhysics;
        public Vector3 envelopeNormalVisual;
    }


    private Transform generatedRoot;
    private Mesh generatedMesh;

    // ------------------------------------------------------------
    // Slopeごとに生成されたEnvelope Rootの履歴。
    //
    // accumulateEnvelopesAcrossSlopes = ON の時は、
    // 過去RootをDestroyせずこのListに保持したまま次を生成する。
    // OFF の時は次の生成前に全て削除する。
    // ------------------------------------------------------------
    private readonly List<Transform> generatedEnvelopeRoots =
        new List<Transform>();

    // 現在「最後に生成された」Envelopeだけをリアルタイム更新するための参照。
    private Transform generatedMeshTransform;
    private MeshFilter generatedMeshFilter;
    private MeshCollider generatedMeshCollider;

    // 最新Envelopeを作った時点の固定幾何。
    // Inspector調整ではSlopeStick3Dから取り直さず、この区間だけを再生成する。
    private bool latestEnvelopeGeometryCached;
    private Vector3 cachedEntrySurfacePhysics;
    private Vector3 cachedLimitSurfacePhysics;
    private Vector3 cachedAxisPhysics;
    private Vector3 cachedSlopeNormalPhysics;
    private float cachedA0;
    private float cachedGamma;
    private float cachedEqualizerRadius;

    // Inspector変更検出用。
    private bool liveSettingsSnapshotValid;
    private int lastSegmentCount;
    private float lastEnvelopeWidth;
    private float lastNormalOffset;
    private bool lastUseCurvedOffset;
    private float lastEdgeBlendWidth;
    private float lastAmplitudeScale;
    private bool lastUseWorldYCorrection;
    private float lastWorldYCorrectionRatio;


    // ================================================================
    // Unity
    // ================================================================

    private void Awake()
    {
        ResolveReferences();
        CaptureLiveSettingsSnapshot();
    }


    private void FixedUpdate()
    {
        // 通常の生成フローは従来どおり。
        TryBuildEnvelopeIfReady();
    }


    private void Update()
    {
        // ------------------------------------------------------------
        // Play中のInspector変更を毎描画フレーム監視する。
        //
        // 重要:
        // BuildNegativeEnvelope()をやり直すのではなく、
        // 「最後に生成されたEnvelope」の固定幾何を使って
        // Meshアセットだけを差し替える。
        // ------------------------------------------------------------

        if (!Application.isPlaying)
            return;

        if (!envelopeBuilt ||
            !latestEnvelopeGeometryCached ||
            !generatedMeshTransform ||
            !generatedMeshFilter ||
            !generatedMeshCollider)
        {
            CaptureLiveSettingsSnapshot();
            return;
        }

        if (!LiveSettingsChanged())
            return;

        RebuildLatestGeneratedMesh();
    }



    // ================================================================
    // Arm
    // ================================================================

    /// <summary>
    /// BallVisualから得たEnergyと、
    /// BallVisualEqualizerへ与える予定の投射方向から
    /// -0側包絡線を準備します。
    ///
    /// E = 1/2 m v^2
    ///
    /// v = sqrt(2E/m)
    ///
    /// その上向き成分から
    ///
    /// hA = vy^2 / 2g
    ///
    /// を求め、Entry側の自然な最大振幅にします。
    /// </summary>
    public void ArmFromBallVisualEnergy(
        float energyJoule,
        Vector3 equalizerLaunchDirectionVisual)
    {
        ResolveReferences();

        if (!ballVisualEqualizer)
            return;

        float mass =
            Mathf.Max(
                0.0001f,
                ballVisualEqualizer.mass);

        float safeEnergy =
            Mathf.Max(
                0f,
                energyJoule);

        if (safeEnergy <= 0.000001f)
            return;

        Vector3 direction =
            equalizerLaunchDirectionVisual;

        if (direction.sqrMagnitude <= 0.000001f)
            return;

        direction.Normalize();


        // ------------------------------------------------------------
        // E = 1/2 m v²
        //
        // v = sqrt(2E/m)
        // ------------------------------------------------------------

        float speed =
            Mathf.Sqrt(
                2f * safeEnergy / mass);


        Vector3 launchVelocity =
            direction * speed;


        ArmFromEqualizerLaunchVelocity(
            launchVelocity,
            safeEnergy);
    }


    /// <summary>
    /// Equalizer側ですでに初速を計算済みなら、
    /// こちらを直接使用できます。
    /// </summary>
    public void ArmFromEqualizerLaunchVelocity(
        Vector3 launchVelocityVisual)
    {
        float mass =
            ballVisualEqualizer
                ? Mathf.Max(
                    0.0001f,
                    ballVisualEqualizer.mass)
                : 1f;

        float energy =
            0.5f *
            mass *
            launchVelocityVisual.sqrMagnitude;

        ArmFromEqualizerLaunchVelocity(
            launchVelocityVisual,
            energy);
    }


    private void ArmFromEqualizerLaunchVelocity(
        Vector3 launchVelocityVisual,
        float energyJoule)
    {
        float g =
            Mathf.Abs(
                Physics.gravity.y);

        if (g <= 0.0001f)
            return;


        capturedEqualizerLaunchVelocityVisual =
            launchVelocityVisual;

        capturedEqualizerReleasePositionVisual =
            ballVisualEqualizer
                ? ballVisualEqualizer.position
                : transform.position;


        // Equalizerの初速のうち、
        // Apex高さを作る上向き成分だけを取り出す。
        float verticalSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    launchVelocityVisual,
                    Vector3.up));


        // ------------------------------------------------------------
        // 1/2 m vy² = mgh
        //
        // h = vy² / 2g
        // ------------------------------------------------------------

        float apexHeight =
            verticalSpeed *
            verticalSpeed /
            (2f * g);


        if (apexHeight <= 0.0001f)
            return;


        PrepareForNextEnvelopeGeneration();

        Debug.Log(
            $"[ENVELOPE NEXT GENERATION] " +
            $"mode={(accumulateEnvelopesAcrossSlopes ? "APPEND" : "REPLACE")} " +
            $"survivingRoots={generatedEnvelopeRoots.Count}",
            this);


        sourceEnergyJoule =
            energyJoule;

        equalizerLaunchSpeed =
            launchVelocityVisual.magnitude;

        equalizerVerticalLaunchSpeed =
            verticalSpeed;

        entryApexHeight =
            apexHeight;

        armed =
            true;

        envelopeBuilt =
            false;
    }


    // ================================================================
    // Build
    // ================================================================

    private void BuildNegativeEnvelope()
    {
        Vector3 axisPhysics =
            slopeStick.BallVisualSlopeTangent;

        if (axisPhysics.sqrMagnitude <= 0.000001f)
            return;

        axisPhysics.Normalize();


        float sectionLength =
            slopeStick.BallVisualSlopeSectionLength;

        if (sectionLength <= 0.0001f)
            return;


        float targetProgress01 =
            Mathf.Clamp01(
                slopeStick.TargetSlopeProgressPercent *
                0.01f);

        if (targetProgress01 <= 0.0001f)
            return;


        // SlopeStick3DがREAD ONLYで公開しているExact Limit中心位置。
        Vector3 limitCenterPhysics =
            slopeStick.BallVisualTargetProgressCenterPhysics;


        // Limit = Entry + axis * (L * targetProgress)
        // からSlope Entryを逆算する。
        Vector3 entryCenterPhysics =
            limitCenterPhysics -
            axisPhysics *
            (sectionLength * targetProgress01);


        capturedEntryPhysics =
            entryCenterPhysics;

        capturedLimitPhysics =
            limitCenterPhysics;

        capturedTargetProgress01 =
            targetProgress01;


        float equalizerRadius =
            ResolveEqualizerWorldRadius();


        float subjectRadius =
            ResolveSlopeStickWorldRadius();


        // ------------------------------------------------------------
        // Envelope用Slope Normal
        //
        // BallVisualSurfaceNormalは、その瞬間のCollisionContact Normalを
        // 含む場合があり、Slope全体のaxisPhysicsと直交しないことがある。
        //
        // Envelopeの4R+Aは「Slope全体に対する法線距離」なので、
        // axisPhysicsに対するWorld UpのGram-Schmidt射影から
        // 必ず直交する上向きNormalを構成する。
        //
        //     N = normalize(Up - T * dot(Up, T))
        //
        // よって
        //
        //     dot(T, N) = 0
        //
        // を幾何学的に保証する。
        // ------------------------------------------------------------

        Vector3 observedSurfaceNormal =
            slopeStick.BallVisualSurfaceNormal;

        Vector3 slopeNormalPhysics =
            Vector3.ProjectOnPlane(
                Vector3.up,
                axisPhysics);

        // axisがほぼ垂直など、World UpからNormalを作れない場合だけ
        // SlopeStickの観測Normalをaxisに直交化してFallbackする。
        if (slopeNormalPhysics.sqrMagnitude <= 0.000001f)
        {
            slopeNormalPhysics =
                Vector3.ProjectOnPlane(
                    observedSurfaceNormal,
                    axisPhysics);
        }

        // 最終Fallback。
        if (slopeNormalPhysics.sqrMagnitude <= 0.000001f)
        {
            Vector3 fallbackAxis =
                Mathf.Abs(
                    Vector3.Dot(
                        axisPhysics,
                        Vector3.right)) < 0.95f
                    ? Vector3.right
                    : Vector3.forward;

            slopeNormalPhysics =
                Vector3.Cross(
                    axisPhysics,
                    Vector3.Cross(
                        fallbackAxis,
                        axisPhysics));
        }

        slopeNormalPhysics.Normalize();

        // EnvelopeはUpper側なので常に上向きNormalを採用する。
        if (Vector3.Dot(
                slopeNormalPhysics,
                Vector3.up) < 0f)
        {
            slopeNormalPhysics =
                -slopeNormalPhysics;
        }

        float tangentNormalDot =
            Vector3.Dot(
                axisPhysics,
                slopeNormalPhysics);

        Debug.Log(
            $"[ENVELOPE FRAME] " +
            $"axis={axisPhysics:F5} " +
            $"normal={slopeNormalPhysics:F5} " +
            $"observedNormal={observedSurfaceNormal:F5} " +
            $"dotTN={tangentNormalDot:F6} " +
            $"entryCenter={entryCenterPhysics:F5} " +
            $"limitCenter={limitCenterPhysics:F5}",
            this);

        // SlopeStickのTargetはInSubject中心なので、
        // 半径分だけ面側へ戻して仮想Slope surface lineを作る。
        Vector3 entrySurfacePhysics =
            entryCenterPhysics -
            slopeNormalPhysics *
            subjectRadius;


        Vector3 limitSurfacePhysics =
            limitCenterPhysics -
            slopeNormalPhysics *
            subjectRadius;


        float contactOffset =
            ballVisualEqualizerCollider
                ? Mathf.Max(
                    ballVisualEqualizerCollider.contactOffset,
                    Physics.defaultContactOffset)
                : Physics.defaultContactOffset;


        minimumFreeAmplitude =
            Mathf.Max(
                0.001f,
                contactOffset * 2f);


        // ------------------------------------------------------------
        // BallVisual POP高さはWorld-Yの高さ。
        // EnvelopeのA0はSlope Normal方向の振幅なので、
        // N・Upで法線振幅上限へ変換する。
        // ------------------------------------------------------------
        float normalUp =
            Mathf.Abs(
                Vector3.Dot(
                    slopeNormalPhysics,
                    Vector3.up));

        normalUp =
            Mathf.Max(
                0.10f,
                normalUp);


        float maximumA0 =
            Mathf.Max(
                minimumFreeAmplitude,
                entryApexHeight / normalUp);

        firstContactMaximumAmplitude =
            maximumA0;


        if (!useCurvedOffset &&
            normalOffset > 0.50f)
        {
            Debug.LogWarning(
                $"[ENVELOPE FIRST CONTACT] " +
                $"Normal Offset is large ({normalOffset:F3}m). " +
                $"The automatic contact solver includes this offset, " +
                $"so a reachable candidate may not exist. " +
                $"For the first validation, Normal Offset = 0 is recommended.",
                this);
        }


        FirstContactCandidate selected;

        bool selectedValid =
            TrySelectFirstContact(
                entrySurfacePhysics,
                limitSurfacePhysics,
                slopeNormalPhysics,
                equalizerRadius,
                maximumA0,
                out selected);


        Vector3 envelopeStartSurfacePhysics;
        float A0;

        if (selectedValid)
        {
            envelopeStartSurfacePhysics =
                selected.surfacePhysics;

            A0 =
                Mathf.Max(
                    selected.solvedA0,
                    minimumFreeAmplitude);

            firstContactSelected = true;
            selectedFirstContactTime = selected.time;
            selectedFirstContactPathProgress01 = selected.pathProgress01;
            selectedFirstContactAngleDeg = selected.incidenceAngleDeg;
            selectedFirstContactSeverity01 = selected.severity01;
            selectedFirstContactNormalSpeed = selected.normalSpeed;
            selectedFirstContactAmplitude = A0;
            selectedFirstContactPositionVisual =
                selected.predictedPositionVisual;
            selectedFirstContactSurfacePhysics =
                selected.surfacePhysics;

            Debug.Log(
                $"[ENVELOPE FIRST CONTACT SELECTED] " +
                $"mode={firstContactMode} " +
                $"time={selected.time:F4}s " +
                $"pathProgress={selected.pathProgress01 * 100f:F2}% " +
                $"angleObserved={selected.incidenceAngleDeg:F3}deg " +
                $"severity={selected.severity01:F4} " +
                $"normalSpeed={selected.normalSpeed:F4}m/s " +
                $"A0={A0:F4}m " +
                $"maxA0={maximumA0:F4}m " +
                $"planeError={selected.contactPlaneError:F4}m " +
                $"predictedPos={selected.predictedPositionVisual:F4}",
                this);
        }
        else
        {
            // 候補が1つも成立しない場合だけ、旧Entry方式へ戻す。
            // EqualizerをCollider無しでReleaseしないための安全Fallback。
            envelopeStartSurfacePhysics =
                entrySurfacePhysics;

            A0 =
                Mathf.Max(
                    entryApexHeight,
                    minimumFreeAmplitude);

            firstContactSelected = false;
            selectedFirstContactTime = -1f;
            selectedFirstContactPathProgress01 = 0f;
            selectedFirstContactAngleDeg = 0f;
            selectedFirstContactSeverity01 = 0f;
            selectedFirstContactNormalSpeed = 0f;
            selectedFirstContactAmplitude = A0;
            selectedFirstContactPositionVisual = Vector3.zero;
            selectedFirstContactSurfacePhysics =
                entrySurfacePhysics;

            Debug.LogWarning(
                $"[ENVELOPE FIRST CONTACT FALLBACK] " +
                $"mode={firstContactMode} " +
                $"No physically reachable candidate was found. " +
                $"Legacy Entry envelope is used. " +
                $"normalOffset={normalOffset:F3} " +
                $"amplitudeScale={amplitudeScale:F3} " +
                $"maxA0={maximumA0:F4}",
                this);
        }


        float AL =
            Mathf.Min(
                A0,
                minimumFreeAmplitude);


        gamma =
            A0 > AL + 0.000001f
                ? Mathf.Log(A0 / AL)
                : 0f;


        CacheLatestEnvelopeGeometry(
            envelopeStartSurfacePhysics,
            limitSurfacePhysics,
            axisPhysics,
            slopeNormalPhysics,
            A0,
            gamma,
            equalizerRadius);


        CreateRoot();


        // 選択されたFirst Contact -> Exact Limitを
        // 1枚の連続MeshColliderとして生成する。
        envelopeBuilt =
            CreateEnvelopeMesh(
                envelopeStartSurfacePhysics,
                limitSurfacePhysics,
                axisPhysics,
                slopeNormalPhysics,
                A0,
                gamma,
                equalizerRadius);
    }


    // ================================================================
    // First contact prediction / selection
    // ================================================================

    private bool TrySelectFirstContact(
        Vector3 entrySurfacePhysics,
        Vector3 limitSurfacePhysics,
        Vector3 slopeNormalPhysics,
        float equalizerRadius,
        float maximumA0,
        out FirstContactCandidate selected)
    {
        selected = default;

        if (!correspondSubject ||
            capturedEqualizerLaunchVelocityVisual.sqrMagnitude <= 0.000001f)
        {
            return false;
        }

        Vector3 entrySurfaceVisual =
            correspondSubject.MapPoint(
                entrySurfacePhysics);

        Vector3 limitSurfaceVisual =
            correspondSubject.MapPoint(
                limitSurfacePhysics);

        Vector3 pathVisual =
            limitSurfaceVisual -
            entrySurfaceVisual;

        float pathLength =
            pathVisual.magnitude;

        if (pathLength <= FirstContactMinimumRemainingLength)
            return false;

        Vector3 tangentVisual =
            pathVisual / pathLength;

        Vector3 normalVisual =
            correspondSubject.MapDirection(
                slopeNormalPhysics);

        if (normalVisual.sqrMagnitude <= 0.000001f)
            return false;

        normalVisual.Normalize();

        Vector3 widthAxisVisual =
            Vector3.Cross(
                normalVisual,
                tangentVisual);

        if (widthAxisVisual.sqrMagnitude <= 0.000001f)
            return false;

        widthAxisVisual.Normalize();

        Vector3 mappedUpVisual =
            correspondSubject.MapDirection(
                Vector3.up);

        if (mappedUpVisual.sqrMagnitude <= 0.000001f)
            mappedUpVisual = Vector3.up;
        else
            mappedUpVisual.Normalize();

        Vector3 releasePosition =
            capturedEqualizerReleasePositionVisual;

        Vector3 releaseVelocity =
            capturedEqualizerLaunchVelocityVisual;

        Vector3 gravity =
            Physics.gravity;

        List<FirstContactCandidate> candidates =
            new List<FirstContactCandidate>(
                FirstContactPredictionSamples);

        for (int i = 1;
             i <= FirstContactPredictionSamples;
             i++)
        {
            float t =
                FirstContactPredictionMaximumSeconds *
                i /
                FirstContactPredictionSamples;

            Vector3 predictedVelocity =
                releaseVelocity +
                gravity * t;

            if (predictedVelocity.sqrMagnitude <= 0.000001f)
                continue;

            Vector3 predictedPosition =
                releasePosition +
                releaseVelocity * t +
                0.5f * gravity * t * t;

            float along =
                Vector3.Dot(
                    predictedPosition - entrySurfaceVisual,
                    tangentVisual);

            float progress01 =
                along / pathLength;

            if (progress01 < 0f ||
                progress01 >= 1f)
            {
                continue;
            }

            float remainingLength =
                pathLength *
                (1f - progress01);

            if (remainingLength <= FirstContactMinimumRemainingLength)
                continue;

            Vector3 surfacePhysics =
                Vector3.Lerp(
                    entrySurfacePhysics,
                    limitSurfacePhysics,
                    progress01);

            Vector3 surfaceVisual =
                correspondSubject.MapPoint(
                    surfacePhysics);

            FirstContactCandidate candidate;

            if (!TrySolveFirstContactCandidate(
                    t,
                    progress01,
                    predictedPosition,
                    predictedVelocity,
                    surfacePhysics,
                    surfaceVisual,
                    tangentVisual,
                    normalVisual,
                    widthAxisVisual,
                    mappedUpVisual,
                    remainingLength,
                    equalizerRadius,
                    maximumA0,
                    out candidate))
            {
                continue;
            }

            candidates.Add(
                candidate);
        }

        if (candidates.Count == 0)
            return false;

        selected =
            SelectFirstContactFromReachableSet(
                candidates);

        return
            selected.valid;
    }


    private FirstContactCandidate SelectFirstContactFromReachableSet(
        List<FirstContactCandidate> candidates)
    {
        if (candidates == null ||
            candidates.Count == 0)
        {
            return default;
        }

        if (firstContactMode ==
            EnvelopeFirstContactMode.AutoTime)
        {
            FirstContactCandidate best =
                candidates[0];

            for (int i = 1;
                 i < candidates.Count;
                 i++)
            {
                if (candidates[i].time < best.time)
                    best = candidates[i];
            }

            return best;
        }

        if (firstContactMode ==
            EnvelopeFirstContactMode.AutoHeight)
        {
            FirstContactCandidate best =
                candidates[0];

            for (int i = 1;
                 i < candidates.Count;
                 i++)
            {
                FirstContactCandidate candidate =
                    candidates[i];

                if (candidate.solvedA0 > best.solvedA0 + 0.0001f ||
                    (Mathf.Abs(candidate.solvedA0 - best.solvedA0) <= 0.0001f &&
                     candidate.time < best.time))
                {
                    best = candidate;
                }
            }

            return best;
        }

        if (firstContactMode ==
            EnvelopeFirstContactMode.AutoSpeed)
        {
            // Data-derived target: RMS normal speed of all reachable
            // contacts.  No absolute m/s or angle target is imposed.
            float meanSquare =
                0f;

            for (int i = 0;
                 i < candidates.Count;
                 i++)
            {
                float speed =
                    candidates[i].normalSpeed;

                meanSquare +=
                    speed * speed;
            }

            float rmsSpeed =
                Mathf.Sqrt(
                    meanSquare /
                    candidates.Count);

            FirstContactCandidate best =
                candidates[0];

            float bestError =
                Mathf.Abs(
                    best.normalSpeed -
                    rmsSpeed);

            for (int i = 1;
                 i < candidates.Count;
                 i++)
            {
                FirstContactCandidate candidate =
                    candidates[i];

                float error =
                    Mathf.Abs(
                        candidate.normalSpeed -
                        rmsSpeed);

                if (error < bestError - 0.0001f ||
                    (Mathf.Abs(error - bestError) <= 0.0001f &&
                     candidate.time < best.time))
                {
                    best = candidate;
                    bestError = error;
                }
            }

            return best;
        }

        candidates.Sort(
            (a, b) =>
                a.severity01.CompareTo(
                    b.severity01));

        float quantile01;

        switch (firstContactMode)
        {
            case EnvelopeFirstContactMode.Soft:
                quantile01 = 0.25f;
                break;

            case EnvelopeFirstContactMode.Dynamic:
                quantile01 = 0.75f;
                break;

            case EnvelopeFirstContactMode.Balanced:
            default:
                quantile01 = 0.50f;
                break;
        }

        int index =
            Mathf.Clamp(
                Mathf.RoundToInt(
                    (candidates.Count - 1) *
                    quantile01),
                0,
                candidates.Count - 1);

        // Same relative strength can have several physically reachable
        // candidates.  Prefer the earlier one so more stair path remains.
        FirstContactCandidate selected =
            candidates[index];

        float severityWindow =
            candidates.Count > 1
                ? 1f /
                  (candidates.Count - 1f)
                : 1f;

        for (int i = 0;
             i < candidates.Count;
             i++)
        {
            FirstContactCandidate candidate =
                candidates[i];

            if (Mathf.Abs(
                    candidate.severity01 -
                    selected.severity01) <=
                severityWindow * 0.5f &&
                candidate.time < selected.time)
            {
                selected = candidate;
            }
        }

        return selected;
    }


    private bool TrySolveFirstContactCandidate(
        float time,
        float progress01,
        Vector3 predictedPositionVisual,
        Vector3 predictedVelocityVisual,
        Vector3 surfacePhysics,
        Vector3 surfaceVisual,
        Vector3 tangentVisual,
        Vector3 normalVisual,
        Vector3 widthAxisVisual,
        Vector3 mappedUpVisual,
        float remainingLength,
        float equalizerRadius,
        float maximumA0,
        out FirstContactCandidate candidate)
    {
        candidate = default;

        float safeAmplitudeScale =
            Mathf.Max(
                0.0001f,
                amplitudeScale);

        float entryNormalOffset =
            useCurvedOffset
                ? 0f
                : normalOffset;

        float normalUp =
            Mathf.Max(
                0.01f,
                Vector3.Dot(
                    mappedUpVisual,
                    normalVisual));

        float worldYFactor =
            useWorldYCorrection
                ? 1f +
                  worldYCorrectionRatio *
                  normalUp
                : 1f;

        worldYFactor =
            Mathf.Max(
                0.01f,
                worldYFactor);


        // ------------------------------------------------------------
        // まずSlope Normal距離からA0の初期推定を作る。
        //
        // Upper Envelope面:
        //
        //     H = 4R + A
        //
        // Equalizerは半径Rの球なので、下側から接触するときの
        // 球中心のSlope Normal距離は
        //
        //     Hcenter = (4R + A) - R
        //             = 3R + A
        //
        // よって
        //
        //     A = centerNormalDistance - 3R
        //
        // で逆算する。
        // ------------------------------------------------------------

        float centerNormalDistance =
            Vector3.Dot(
                predictedPositionVisual - surfaceVisual,
                normalVisual);

        float desiredFreeAmplitude =
            centerNormalDistance -
            equalizerRadius * 3f;

        float solvedA0 =
            (desiredFreeAmplitude - entryNormalOffset) /
            (safeAmplitudeScale * worldYFactor);

        solvedA0 =
            Mathf.Clamp(
                solvedA0,
                minimumFreeAmplitude,
                maximumA0);


        Vector3 localEnvelopeNormal =
            normalVisual;

        float planeError =
            float.PositiveInfinity;


        // EnvelopeはA0によってEntry局所角も変わるため、
        // 接触Plane誤差を数回補正してA0を収束させる。
        for (int iteration = 0;
             iteration < FirstContactSolveIterations;
             iteration++)
        {
            float AL =
                Mathf.Min(
                    solvedA0,
                    minimumFreeAmplitude);

            float candidateGamma =
                solvedA0 > AL + 0.000001f
                    ? Mathf.Log(solvedA0 / AL)
                    : 0f;


            float renderedAmplitude =
                solvedA0 *
                safeAmplitudeScale;

            float worldYCorrection =
                useWorldYCorrection
                    ? renderedAmplitude *
                      worldYCorrectionRatio
                    : 0f;


            Vector3 envelopePointVisual =
                surfaceVisual +
                normalVisual *
                (equalizerRadius * 4f +
                 renderedAmplitude +
                 entryNormalOffset) +
                mappedUpVisual *
                worldYCorrection;


            float amplitudeDerivative =
                remainingLength > 0.0001f
                    ? -candidateGamma *
                      renderedAmplitude /
                      remainingLength
                    : 0f;

            float worldYDerivative =
                useWorldYCorrection
                    ? amplitudeDerivative *
                      worldYCorrectionRatio
                    : 0f;


            Vector3 envelopeTangent =
                tangentVisual +
                normalVisual *
                amplitudeDerivative +
                mappedUpVisual *
                worldYDerivative;

            if (envelopeTangent.sqrMagnitude <= 0.000001f)
                return false;

            envelopeTangent.Normalize();


            localEnvelopeNormal =
                Vector3.Cross(
                    envelopeTangent,
                    widthAxisVisual);

            if (localEnvelopeNormal.sqrMagnitude <= 0.000001f)
                return false;

            localEnvelopeNormal.Normalize();

            if (Vector3.Dot(
                    localEnvelopeNormal,
                    normalVisual) < 0f)
            {
                localEnvelopeNormal =
                    -localEnvelopeNormal;
            }


            // Equalizer中心はUpper Envelopeの下側なので、
            // PlaneまでのSigned Distanceが -radius なら接触。
            planeError =
                Vector3.Dot(
                    predictedPositionVisual -
                    envelopePointVisual,
                    localEnvelopeNormal) +
                equalizerRadius;


            float response =
                safeAmplitudeScale *
                worldYFactor *
                Mathf.Max(
                    0.05f,
                    Vector3.Dot(
                        normalVisual,
                        localEnvelopeNormal));

            solvedA0 +=
                planeError /
                response;

            solvedA0 =
                Mathf.Clamp(
                    solvedA0,
                    minimumFreeAmplitude,
                    maximumA0);
        }


        // 最終状態でもう一度局所Normalと誤差を評価する。
        float finalAL =
            Mathf.Min(
                solvedA0,
                minimumFreeAmplitude);

        float finalGamma =
            solvedA0 > finalAL + 0.000001f
                ? Mathf.Log(solvedA0 / finalAL)
                : 0f;

        float finalRenderedAmplitude =
            solvedA0 *
            safeAmplitudeScale;

        float finalWorldYCorrection =
            useWorldYCorrection
                ? finalRenderedAmplitude *
                  worldYCorrectionRatio
                : 0f;

        Vector3 finalEnvelopePointVisual =
            surfaceVisual +
            normalVisual *
            (equalizerRadius * 4f +
             finalRenderedAmplitude +
             entryNormalOffset) +
            mappedUpVisual *
            finalWorldYCorrection;

        float finalAmplitudeDerivative =
            remainingLength > 0.0001f
                ? -finalGamma *
                  finalRenderedAmplitude /
                  remainingLength
                : 0f;

        float finalWorldYDerivative =
            useWorldYCorrection
                ? finalAmplitudeDerivative *
                  worldYCorrectionRatio
                : 0f;

        Vector3 finalEnvelopeTangent =
            tangentVisual +
            normalVisual *
            finalAmplitudeDerivative +
            mappedUpVisual *
            finalWorldYDerivative;

        if (finalEnvelopeTangent.sqrMagnitude <= 0.000001f)
            return false;

        finalEnvelopeTangent.Normalize();

        localEnvelopeNormal =
            Vector3.Cross(
                finalEnvelopeTangent,
                widthAxisVisual);

        if (localEnvelopeNormal.sqrMagnitude <= 0.000001f)
            return false;

        localEnvelopeNormal.Normalize();

        if (Vector3.Dot(
                localEnvelopeNormal,
                normalVisual) < 0f)
        {
            localEnvelopeNormal =
                -localEnvelopeNormal;
        }

        planeError =
            Vector3.Dot(
                predictedPositionVisual -
                finalEnvelopePointVisual,
                localEnvelopeNormal) +
            equalizerRadius;


        if (Mathf.Abs(planeError) > FirstContactPlaneTolerance)
            return false;


        float normalSpeed =
            Vector3.Dot(
                predictedVelocityVisual,
                localEnvelopeNormal);

        // Upper Envelopeへ向かっている時だけ候補。
        if (normalSpeed <= 0.0001f)
            return false;


        float speed =
            predictedVelocityVisual.magnitude;

        if (speed <= 0.0001f)
            return false;


        float incidenceAngleDeg =
            Mathf.Asin(
                Mathf.Clamp01(
                    normalSpeed / speed)) *
            Mathf.Rad2Deg;


        candidate.valid = true;
        candidate.time = time;
        candidate.pathProgress01 = progress01;
        candidate.incidenceAngleDeg = incidenceAngleDeg;
        candidate.normalSpeed = normalSpeed;
        candidate.severity01 =
            Mathf.Clamp01(
                normalSpeed /
                speed);
        candidate.solvedA0 = solvedA0;
        candidate.contactPlaneError = planeError;
        candidate.predictedPositionVisual = predictedPositionVisual;
        candidate.predictedVelocityVisual = predictedVelocityVisual;
        candidate.surfacePhysics = surfacePhysics;
        candidate.envelopeNormalVisual = localEnvelopeNormal;

        return true;
    }


    // ================================================================
    // Envelope mesh
    // ================================================================

    private bool CreateEnvelopeMesh(
        Vector3 entrySurfacePhysics,
        Vector3 limitSurfacePhysics,
        Vector3 axisPhysics,
        Vector3 slopeNormalPhysics,
        float A0,
        float gammaValue,
        float equalizerRadius)
    {
        if (!generatedRoot)
            return false;

        GameObject meshObject =
            new GameObject(
                "NegativeEnvelopeMesh");

        meshObject.layer =
            gameObject.layer;

        generatedMeshTransform =
            meshObject.transform;

        generatedMeshTransform.SetParent(
            generatedRoot,
            false);

        generatedMeshTransform.localPosition =
            Vector3.zero;

        generatedMeshTransform.localRotation =
            Quaternion.identity;

        generatedMeshTransform.localScale =
            Vector3.one;


        Mesh mesh =
            BuildEnvelopeMeshAsset(
                generatedMeshTransform,
                entrySurfacePhysics,
                limitSurfacePhysics,
                axisPhysics,
                slopeNormalPhysics,
                A0,
                gammaValue,
                equalizerRadius);

        if (!mesh)
        {
            Destroy(
                meshObject);

            generatedMeshTransform =
                null;

            return false;
        }


        generatedMeshFilter =
            meshObject.AddComponent<MeshFilter>();

        generatedMeshFilter.sharedMesh =
            mesh;


        generatedMeshCollider =
            meshObject.AddComponent<MeshCollider>();

        generatedMeshCollider.sharedMesh =
            mesh;

        generatedMeshCollider.convex =
            false;


        // InSubjectだけ除外
        if (inSubjectCollider)
        {
            Physics.IgnoreCollision(
                inSubjectCollider,
                generatedMeshCollider,
                true);
        }


        meshObject.AddComponent<
            BallVisualEnvelopeSurfaceMarker>();


        generatedMesh =
            mesh;

        CaptureLiveSettingsSnapshot();


        Debug.Log(
            $"[ENVELOPE MESH CREATED] " +
            $"vertices={mesh.vertexCount} " +
            $"width={envelopeWidth:F2} " +
            $"normalOffset={normalOffset:F3} " +
            $"curve={useCurvedOffset} " +
            $"amplitudeScale={amplitudeScale:F3} " +
            $"accumulateAcrossSlopes={accumulateEnvelopesAcrossSlopes} " +
            $"rootCount={generatedEnvelopeRoots.Count}",
            this);


        return true;
    }


    private Mesh BuildEnvelopeMeshAsset(
        Transform meshTransform,
        Vector3 entrySurfacePhysics,
        Vector3 limitSurfacePhysics,
        Vector3 axisPhysics,
        Vector3 slopeNormalPhysics,
        float A0,
        float gammaValue,
        float equalizerRadius)
    {
        if (!meshTransform ||
            !correspondSubject)
        {
            return null;
        }


        int sampleCount =
            Mathf.Max(
                2,
                segmentCount + 1);

        float halfWidth =
            Mathf.Max(
                0.05f,
                envelopeWidth * 0.5f);


        Vector3 tangentPhysics =
            axisPhysics.normalized;


        // 進行方向に直交する横方向
        Vector3 widthAxisPhysics =
            Vector3.Cross(
                slopeNormalPhysics,
                tangentPhysics);

        if (widthAxisPhysics.sqrMagnitude <=
            0.000001f)
        {
            widthAxisPhysics =
                Vector3.Cross(
                    Vector3.up,
                    tangentPhysics);
        }

        if (widthAxisPhysics.sqrMagnitude <=
            0.000001f)
        {
            widthAxisPhysics =
                Vector3.right;
        }

        widthAxisPhysics.Normalize();


        // 1 sample = 左右2点
        Vector3[] vertices =
            new Vector3[
                sampleCount * 2];

        int[] triangles =
            new int[
                (sampleCount - 1) * 6];


        float safeEdgeBlendWidth =
            Mathf.Clamp(
                edgeBlendWidth,
                0.01f,
                0.49f);

        float safeAmplitudeScale =
            Mathf.Max(
                0f,
                amplitudeScale);


        // --------------------------------------------------------
        // Vertices
        // --------------------------------------------------------

        for (int i = 0;
             i < sampleCount;
             i++)
        {
            float t =
                i /
                (float)(sampleCount - 1);


            // A(t) = A0 * exp(-Gamma*t)
            // Amplitude Scaleは指数形状の倍率。
            float amplitude =
                A0 *
                Mathf.Exp(
                    -gammaValue * t) *
                safeAmplitudeScale;


            Vector3 surfacePhysics =
                Vector3.Lerp(
                    entrySurfacePhysics,
                    limitSurfacePhysics,
                    t);


            // 斜面法線方向の基本高さ
            float clearance =
                equalizerRadius * 4f +
                amplitude;


            // ----------------------------------------------------
            // Normal Offset
            //
            // OFF:
            //   全頂点へ同じnormalOffset。
            //   -> Envelope全体がそのまま上下する。
            //
            // ON:
            //   始点/終点は0、中央はnormalOffset。
            //   -> 従来の「中央だけ湾曲して上げる」モード。
            // ----------------------------------------------------

            float offsetWeight =
                1f;

            if (useCurvedOffset)
            {
                float entryBlend =
                    Mathf.SmoothStep(
                        0f,
                        1f,
                        Mathf.Clamp01(
                            t /
                            safeEdgeBlendWidth));

                float limitBlend =
                    Mathf.SmoothStep(
                        0f,
                        1f,
                        Mathf.Clamp01(
                            (1f - t) /
                            safeEdgeBlendWidth));

                offsetWeight =
                    entryBlend *
                    limitBlend;
            }


            float appliedNormalOffset =
                normalOffset *
                offsetWeight;


            // 従来のWorld +Y補正。
            float worldYCorrection =
                useWorldYCorrection
                    ? amplitude *
                      worldYCorrectionRatio
                    : 0f;


            Vector3 centerPhysics =
                surfacePhysics
                + slopeNormalPhysics *
                  (clearance +
                   appliedNormalOffset)
                + Vector3.up *
                  worldYCorrection;


            Vector3 leftPhysics =
                centerPhysics -
                widthAxisPhysics *
                halfWidth;

            Vector3 rightPhysics =
                centerPhysics +
                widthAxisPhysics *
                halfWidth;


            Vector3 leftWorld =
                correspondSubject.MapPoint(
                    leftPhysics);

            Vector3 rightWorld =
                correspondSubject.MapPoint(
                    rightPhysics);


            vertices[i * 2 + 0] =
                meshTransform.InverseTransformPoint(
                    leftWorld);

            vertices[i * 2 + 1] =
                meshTransform.InverseTransformPoint(
                    rightWorld);
        }


        // --------------------------------------------------------
        // Triangles
        // --------------------------------------------------------

        int ti =
            0;

        for (int i = 0;
             i < sampleCount - 1;
             i++)
        {
            int left0 =
                i * 2;

            int right0 =
                left0 + 1;

            int left1 =
                (i + 1) * 2;

            int right1 =
                left1 + 1;


            // BallVisualEqualizerが下側から当たる面。
            triangles[ti++] =
                left0;

            triangles[ti++] =
                right0;

            triangles[ti++] =
                right1;


            triangles[ti++] =
                left0;

            triangles[ti++] =
                right1;

            triangles[ti++] =
                left1;
        }


        Mesh mesh =
            new Mesh();

        mesh.name =
            "NegativeEnvelope_ExponentialMesh_Live";

        mesh.vertices =
            vertices;

        mesh.triangles =
            triangles;

        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        return mesh;
    }


    private void CacheLatestEnvelopeGeometry(
        Vector3 entrySurfacePhysics,
        Vector3 limitSurfacePhysics,
        Vector3 axisPhysics,
        Vector3 slopeNormalPhysics,
        float A0,
        float gammaValue,
        float equalizerRadius)
    {
        cachedEntrySurfacePhysics =
            entrySurfacePhysics;

        cachedLimitSurfacePhysics =
            limitSurfacePhysics;

        cachedAxisPhysics =
            axisPhysics;

        cachedSlopeNormalPhysics =
            slopeNormalPhysics;

        cachedA0 =
            A0;

        cachedGamma =
            gammaValue;

        cachedEqualizerRadius =
            equalizerRadius;

        latestEnvelopeGeometryCached =
            true;
    }


    private bool LiveSettingsChanged()
    {
        if (!liveSettingsSnapshotValid)
            return true;

        if (segmentCount !=
            lastSegmentCount)
        {
            return true;
        }

        if (Mathf.Abs(
                envelopeWidth -
                lastEnvelopeWidth) >
            0.00001f)
        {
            return true;
        }

        if (Mathf.Abs(
                normalOffset -
                lastNormalOffset) >
            0.00001f)
        {
            return true;
        }

        if (useCurvedOffset !=
            lastUseCurvedOffset)
        {
            return true;
        }

        if (Mathf.Abs(
                edgeBlendWidth -
                lastEdgeBlendWidth) >
            0.00001f)
        {
            return true;
        }

        if (Mathf.Abs(
                amplitudeScale -
                lastAmplitudeScale) >
            0.00001f)
        {
            return true;
        }

        if (useWorldYCorrection !=
            lastUseWorldYCorrection)
        {
            return true;
        }

        if (Mathf.Abs(
                worldYCorrectionRatio -
                lastWorldYCorrectionRatio) >
            0.00001f)
        {
            return true;
        }

        return false;
    }


    private void CaptureLiveSettingsSnapshot()
    {
        lastSegmentCount =
            segmentCount;

        lastEnvelopeWidth =
            envelopeWidth;

        lastNormalOffset =
            normalOffset;

        lastUseCurvedOffset =
            useCurvedOffset;

        lastEdgeBlendWidth =
            edgeBlendWidth;

        lastAmplitudeScale =
            amplitudeScale;

        lastUseWorldYCorrection =
            useWorldYCorrection;

        lastWorldYCorrectionRatio =
            worldYCorrectionRatio;

        liveSettingsSnapshotValid =
            true;
    }


    private void RebuildLatestGeneratedMesh()
    {
        if (!latestEnvelopeGeometryCached ||
            !generatedMeshTransform ||
            !generatedMeshFilter ||
            !generatedMeshCollider)
        {
            CaptureLiveSettingsSnapshot();
            return;
        }


        // ------------------------------------------------------------
        // 新しいMeshを先に作る。
        // 成功するまで現在のMeshColliderはそのまま残す。
        // ------------------------------------------------------------

        Mesh newMesh =
            BuildEnvelopeMeshAsset(
                generatedMeshTransform,
                cachedEntrySurfacePhysics,
                cachedLimitSurfacePhysics,
                cachedAxisPhysics,
                cachedSlopeNormalPhysics,
                cachedA0,
                cachedGamma,
                cachedEqualizerRadius);

        if (!newMesh)
        {
            Debug.LogWarning(
                "[ENVELOPE LIVE UPDATE] New mesh build failed.",
                this);

            CaptureLiveSettingsSnapshot();
            return;
        }


        Mesh oldMesh =
            generatedMesh;


        // ------------------------------------------------------------
        // MeshFilter / MeshColliderを新Meshへ一気に差し替える。
        //
        // MeshColliderはsharedMeshを一度nullにすることで
        // 新しい形状の再Cookを明示的に発生させる。
        // ------------------------------------------------------------

        generatedMeshFilter.sharedMesh =
            newMesh;

        bool colliderWasEnabled =
            generatedMeshCollider.enabled;

        generatedMeshCollider.enabled =
            false;

        generatedMeshCollider.sharedMesh =
            null;

        generatedMeshCollider.sharedMesh =
            newMesh;

        generatedMeshCollider.enabled =
            colliderWasEnabled;


        generatedMesh =
            newMesh;


        // 新Meshの割り当てが終わってから旧Meshを削除。
        if (oldMesh &&
            oldMesh != newMesh)
        {
            Destroy(
                oldMesh);
        }


        Physics.SyncTransforms();

        CaptureLiveSettingsSnapshot();


        Debug.Log(
            $"[ENVELOPE LIVE UPDATED] " +
            $"normalOffset={normalOffset:F4} " +
            $"curve={useCurvedOffset} " +
            $"edgeBlend={edgeBlendWidth:F3} " +
            $"amplitudeScale={amplitudeScale:F3} " +
            $"useWorldYCorrection={useWorldYCorrection} " +
            $"worldYCorrection={worldYCorrectionRatio:F3} " +
            $"width={envelopeWidth:F3} " +
            $"segments={segmentCount}",
            this);
    }



    // ================================================================
    // Root
    // ================================================================

    private void CreateRoot()
    {
        // ------------------------------------------------------------
        // 通常はArm時のPrepareForNextEnvelopeGeneration()で
        // 既に次の生成準備が終わっている。
        //
        // ただし直接Buildされた場合にも安全に動くよう、
        // current Rootが残っていれば生成モードに従って処理する。
        // ------------------------------------------------------------

        if (generatedRoot)
        {
            if (accumulateEnvelopesAcrossSlopes)
            {
                ReleaseCurrentEnvelopeReferencesWithoutDestroying();
            }
            else
            {
                ClearAllGeneratedEnvelopeRoots();
            }
        }


        GameObject root =
            new GameObject(
                "BallVisualEqualizer_NegativeEnvelope");

        root.layer =
            gameObject.layer;

        generatedRoot =
            root.transform;

        generatedRoot.SetParent(
            transform,
            true);


        generatedEnvelopeRoots.Add(
            generatedRoot);


        // MeshCollider版ではRigidbodyを付けない。
        // Envelopeは固定された環境Collisionとして扱う。
    }


    // ================================================================
    // Public clear / generation history
    // ================================================================

    /// <summary>
    /// 明示的な完全Clear。
    /// Generation Historyモードに関係なく、
    /// 現在 + 過去に蓄積した全Envelopeを削除します。
    /// </summary>
    public void ClearEnvelope()
    {
        ClearAllGeneratedEnvelopeRoots();

        latestEnvelopeGeometryCached =
            false;

        liveSettingsSnapshotValid =
            false;

        armed =
            false;

        envelopeBuilt =
            false;

        sourceEnergyJoule =
            0f;

        equalizerLaunchSpeed =
            0f;

        equalizerVerticalLaunchSpeed =
            0f;

        entryApexHeight =
            0f;

        gamma =
            0f;

        ResetFirstContactRuntime();
    }


    /// <summary>
    /// 平面→斜面などで「次のEnvelopeを生成する直前」に使用する。
    ///
    /// OFF:
    ///   過去Envelopeを全部削除してから次を生成。
    ///
    /// ON:
    ///   過去Envelopeを残したまま、
    ///   current参照だけ手放して次のEnvelopeを生成。
    /// </summary>
    private void PrepareForNextEnvelopeGeneration()
    {
        ResetFirstContactRuntime();

        if (accumulateEnvelopesAcrossSlopes)
        {
            ReleaseCurrentEnvelopeReferencesWithoutDestroying();
        }
        else
        {
            ClearAllGeneratedEnvelopeRoots();
        }


        // 新しいSlopeの固定幾何はBuild時に取り直す。
        latestEnvelopeGeometryCached =
            false;

        // 新しいMeshが出来るまでは未Build。
        envelopeBuilt =
            false;

        // Inspector差分監視は次のMesh生成後に再Snapshotする。
        liveSettingsSnapshotValid =
            false;
    }


    private void ResetFirstContactRuntime()
    {
        firstContactSelected = false;
        selectedFirstContactTime = -1f;
        selectedFirstContactPathProgress01 = 0f;
        selectedFirstContactAngleDeg = 0f;
        selectedFirstContactSeverity01 = 0f;
        selectedFirstContactNormalSpeed = 0f;
        selectedFirstContactAmplitude = 0f;
        firstContactMaximumAmplitude = 0f;
        selectedFirstContactPositionVisual = Vector3.zero;
        selectedFirstContactSurfacePhysics = Vector3.zero;
    }


    /// <summary>
    /// 過去Envelope GameObjectをDestroyせず、
    /// 「最新Envelope」用参照だけnullへ戻す。
    ///
    /// Appendモードの核心。
    /// </summary>
    private void ReleaseCurrentEnvelopeReferencesWithoutDestroying()
    {
        generatedRoot =
            null;

        generatedMesh =
            null;

        generatedMeshTransform =
            null;

        generatedMeshFilter =
            null;

        generatedMeshCollider =
            null;
    }


    /// <summary>
    /// 現在まで生成したEnvelope Rootを全て削除する。
    /// Replaceモードで次Slopeへ入る時と、
    /// 明示的Clear時に使用する。
    /// </summary>
    private void ClearAllGeneratedEnvelopeRoots()
    {
        // List外にcurrent Rootがある異常ケースも拾えるよう、
        // currentを先にListへ補完する。
        if (generatedRoot &&
            !generatedEnvelopeRoots.Contains(
                generatedRoot))
        {
            generatedEnvelopeRoots.Add(
                generatedRoot);
        }


        for (int i =
                 generatedEnvelopeRoots.Count - 1;
             i >= 0;
             i--)
        {
            Transform root =
                generatedEnvelopeRoots[i];

            if (!root)
                continue;


            // Root配下にある生成Meshアセットも明示的に破棄。
            MeshFilter[] filters =
                root.GetComponentsInChildren<MeshFilter>(
                    true);

            for (int j = 0;
                 j < filters.Length;
                 j++)
            {
                MeshFilter filter =
                    filters[j];

                if (!filter)
                    continue;

                Mesh mesh =
                    filter.sharedMesh;

                filter.sharedMesh =
                    null;

                MeshCollider collider =
                    filter.GetComponent<MeshCollider>();

                if (collider)
                {
                    collider.enabled =
                        false;

                    collider.sharedMesh =
                        null;
                }

                if (mesh)
                {
                    Destroy(
                        mesh);
                }
            }


            root.gameObject.SetActive(
                false);

            Destroy(
                root.gameObject);
        }


        generatedEnvelopeRoots.Clear();

        generatedRoot =
            null;

        generatedMesh =
            null;

        generatedMeshTransform =
            null;

        generatedMeshFilter =
            null;

        generatedMeshCollider =
            null;
    }



    // ================================================================
    // References / radius
    // ================================================================

    private void ResolveReferences()
    {
        if (!slopeStick)
        {
            slopeStick =
                FindFirstObjectByType<SlopeStick3D>();
        }

        if (!correspondSubject)
        {
            correspondSubject =
                FindFirstObjectByType<CorrespondSubject>();
        }

        if (!ballVisualEqualizer &&
            ballVisualEqualizerCollider)
        {
            ballVisualEqualizer =
                ballVisualEqualizerCollider.attachedRigidbody;
        }

        if (!ballVisualEqualizerCollider &&
            ballVisualEqualizer)
        {
            ballVisualEqualizerCollider =
                ballVisualEqualizer.GetComponent<
                    SphereCollider>();
        }
    }


    private bool ReferencesValid()
    {
        return
            slopeStick &&
            correspondSubject &&
            ballVisualEqualizer &&
            ballVisualEqualizerCollider;
    }


    private float ResolveEqualizerWorldRadius()
    {
        if (!ballVisualEqualizerCollider)
            return 0.5f;

        Vector3 scale =
            ballVisualEqualizerCollider.transform.lossyScale;

        float maximumScale =
            Mathf.Max(
                Mathf.Abs(scale.x),
                Mathf.Abs(scale.y),
                Mathf.Abs(scale.z));

        return
            ballVisualEqualizerCollider.radius *
            maximumScale;
    }


    private float ResolveSlopeStickWorldRadius()
    {
        if (!slopeStick)
            return 0.5f;

        SphereCollider sphere =
            slopeStick.GetComponent<SphereCollider>();

        if (!sphere)
            return 0.5f;

        Vector3 scale =
            sphere.transform.lossyScale;

        float maximumScale =
            Mathf.Max(
                Mathf.Abs(scale.x),
                Mathf.Abs(scale.y),
                Mathf.Abs(scale.z));

        return
            sphere.radius *
            maximumScale;
    }


    // ================================================================
    // Stable oscillation frame for BallVisualEqualizerSync
    // ================================================================

    /// <summary>
    /// 最新Envelopeを生成したSlope全体のTangent/NormalをVisual座標で返します。
    /// 局所Mesh三角形のNormalではなく、Build時にGram-Schmidtで確定した
    /// 連続なSlope frameです。Equalizerの減衰エネルギー基底に使用します。
    /// </summary>
    public bool TryGetLatestOscillationFrameVisual(
        out Vector3 tangentVisual,
        out Vector3 normalVisual)
    {
        tangentVisual =
            Vector3.zero;

        normalVisual =
            Vector3.zero;

        if (!latestEnvelopeGeometryCached ||
            !correspondSubject)
        {
            return false;
        }

        tangentVisual =
            correspondSubject.MapDirection(
                cachedAxisPhysics);

        normalVisual =
            correspondSubject.MapDirection(
                cachedSlopeNormalPhysics);

        if (tangentVisual.sqrMagnitude <= 0.000001f ||
            normalVisual.sqrMagnitude <= 0.000001f)
        {
            tangentVisual = Vector3.zero;
            normalVisual = Vector3.zero;
            return false;
        }

        tangentVisual.Normalize();

        normalVisual =
            Vector3.ProjectOnPlane(
                normalVisual,
                tangentVisual);

        if (normalVisual.sqrMagnitude <= 0.000001f)
        {
            tangentVisual = Vector3.zero;
            normalVisual = Vector3.zero;
            return false;
        }

        normalVisual.Normalize();

        Vector3 mappedUp =
            correspondSubject.MapDirection(
                Vector3.up);

        if (mappedUp.sqrMagnitude > 0.000001f &&
            Vector3.Dot(
                normalVisual,
                mappedUp) < 0f)
        {
            normalVisual =
                -normalVisual;
        }

        return true;
    }


    /// <summary>
    /// 最新EnvelopeのEntry/Limit中心と、Releaseから予測First Contactまでの
    /// 時間をVisual座標で返します。形状やFirstContactModeを変更するAPIではなく、
    /// BallVisualEqualizerSyncが残距離・残時間・減衰可能周期数を評価するための
    /// read-only geometry accessorです。
    /// </summary>
    public bool TryGetLatestFeasibilityGeometryVisual(
        out Vector3 entryCenterVisual,
        out Vector3 limitCenterVisual,
        out float firstContactTime)
    {
        entryCenterVisual =
            Vector3.zero;

        limitCenterVisual =
            Vector3.zero;

        firstContactTime =
            -1f;

        if (!latestEnvelopeGeometryCached ||
            !correspondSubject)
        {
            return false;
        }

        entryCenterVisual =
            correspondSubject.MapPoint(
                capturedEntryPhysics);

        limitCenterVisual =
            correspondSubject.MapPoint(
                capturedLimitPhysics);

        if (firstContactSelected &&
            selectedFirstContactTime >= 0f)
        {
            firstContactTime =
                selectedFirstContactTime;
        }

        return true;
    }


    // ================================================================
    // Debug
    // ================================================================

    private void OnDrawGizmosSelected()
    {
        if (!envelopeBuilt)
            return;

        Gizmos.DrawSphere(
            correspondSubject.MapPoint(
                capturedEntryPhysics),
            0.08f);

        Gizmos.DrawSphere(
            correspondSubject.MapPoint(
                capturedLimitPhysics),
            0.08f);

        if (firstContactSelected)
        {
            Gizmos.DrawWireSphere(
                selectedFirstContactPositionVisual,
                0.12f);
        }
    }

    public bool ArmFromBallVisualEnergy(
        float energyJoule,
        float ballVisualPopHeight,
        Vector3 equalizerLaunchVelocityVisual)
    {
        ResolveReferences();


        if (!ReferencesValid())
        {
            Debug.LogWarning(
                "[ENVELOPE] References are not valid.",
                this);

            return false;
        }


        float safeEnergy =
            Mathf.Max(
                0f,
                energyJoule);


        float safePopHeight =
            Mathf.Max(
                0f,
                ballVisualPopHeight);


        if (safeEnergy <= 0.000001f)
        {
            Debug.LogWarning(
                "[ENVELOPE] Source Energy is zero.",
                this);

            return false;
        }


        if (safePopHeight <= 0.000001f)
        {
            Debug.LogWarning(
                "[ENVELOPE] POP height is zero.",
                this);

            return false;
        }


        if (equalizerLaunchVelocityVisual.sqrMagnitude <=
            0.000001f)
        {
            Debug.LogWarning(
                "[ENVELOPE] Equalizer launch velocity is zero.",
                this);

            return false;
        }


        // ReleaseToEnvelopeSimulation()は、この呼び出し直前に
        // EqualizerをBallVisualへ一致させている。
        // その瞬間の位置と、これから与える初速をFirst Contact予測へ固定保存する。
        capturedEqualizerReleasePositionVisual =
            ballVisualEqualizer.position;

        capturedEqualizerLaunchVelocityVisual =
            equalizerLaunchVelocityVisual;


        // 前区間のEnvelopeを削除。
        PrepareForNextEnvelopeGeneration();

        Debug.Log(
            $"[ENVELOPE NEXT GENERATION] " +
            $"mode={(accumulateEnvelopesAcrossSlopes ? "APPEND" : "REPLACE")} " +
            $"survivingRoots={generatedEnvelopeRoots.Count}",
            this);


        sourceEnergyJoule =
            safeEnergy;


        equalizerLaunchSpeed =
            equalizerLaunchVelocityVisual.magnitude;


        equalizerVerticalLaunchSpeed =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    equalizerLaunchVelocityVisual,
                    Vector3.up));


        // ------------------------------------------------------------
        // BallVisualSlopeDriveが確定したPOP高さを保存する。
        // Build時にSlope Normal方向の最大A0へ変換し、
        // First Contact候補の中から実際に使うA0を選択する。
        // したがってBallVisualのPOP高さは「Envelope表現の上限」。
        // ------------------------------------------------------------

        entryApexHeight =
            safePopHeight;


        armed =
            true;

        envelopeBuilt =
            false;


        // ------------------------------------------------------------
        // BeginIncidentMethod()の時点では
        // ActiveSlopeFrameが既に成立しているため、
        // 可能ならこの場でColliderを生成する。
        //
        // EqualizerをReleaseする前にColliderを完成させる。
        // ------------------------------------------------------------

        return TryBuildEnvelopeIfReady();
    }

    private bool TryBuildEnvelopeIfReady()
    {
        if (!armed)
            return false;


        if (envelopeBuilt)
            return true;


        if (!ReferencesValid())
            return false;


        if (!slopeStick.BallVisualHasActiveSlopeFrame)
            return false;


        float targetProgress01 =
            Mathf.Clamp01(
                slopeStick.TargetSlopeProgressPercent *
                0.01f);


        if (targetProgress01 <= 0.0001f)
            return false;


        // -0側だけを生成する。
        if (slopeStick.BallVisualSlopeProgress01 >=
            targetProgress01)
        {
            return false;
        }


        BuildNegativeEnvelope();


        return envelopeBuilt;
    }
}


/// <summary>
/// BallVisualEqualizer側のOnCollisionEnterで
///
/// collision.collider.GetComponent<BallVisualEnvelopeSurfaceMarker>()
///
/// を調べればEnvelope衝突か判定できます。
///
/// 力は一切加えません。
/// </summary>
public sealed class BallVisualEnvelopeSurfaceMarker
    : MonoBehaviour
{
    
   /* */
    

}
