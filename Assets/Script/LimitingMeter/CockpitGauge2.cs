using UnityEngine;
using UnityEngine.UI;

// Attach to the gauge parent, not to the Needle child.
// Unity 2022.3 / CockpitGauge v3 compatible.
[ExecuteAlways]
[DisallowMultipleComponent]
public sealed class CockpitGauge2 : MonoBehaviour
{
    [Header("Reading / 数値")]
    [SerializeField, Range(4f, 40f)] float value = 24f;
    [SerializeField] bool dead;

    [Header("Needle Z Calibration / 針のZ角度")]
    [SerializeField] float angleAt4Z = 120f;
    [SerializeField] float angleAt40Z = -96f;
    [SerializeField] float angleAtDeadZ = -120f;
    [SerializeField] float needleArtOffsetZ;

    [Header("Needle Movement / 追従")]
    [SerializeField, Min(0f)] float smoothSpeed = 12f;

    [Header("References / 子オブジェクト")]
    [SerializeField] RectTransform needle;
    [SerializeField] Text digitalText;

    [Header("SlopeStickCore / 数値連動")]
    [Tooltip("Value(4～40)をMaxGroundSpeedに、10～100%へ換算してTargetSlopeProgressPercentに渡します。")]
    [SerializeField] bool syncTargetSlopeProgress = true;
    [Tooltip("InSubject の SlopeStickCore。未設定なら MainGameManager.core を使用します。")]
    [SerializeField] SlopeStickCore targetCore;
    [Tooltip("左側の CockpitGauge。登録すると、左側ゲージのValueも同時更新してCoreとの競合を防ぎます。未設定ならCorrespondSubjectから自動取得します。")]
    [SerializeField] CockpitGauge targetProgressGauge;
    [SerializeField] bool debugSync;

    float displayedAngleZ;
    bool initialized;
    bool syncedOnce;
    float lastSyncedValue = float.NaN;
    SlopeStickCore lastSyncedCore;
    CockpitGauge lastSyncedGauge;
    bool warnedMissingCore;
    bool lastSyncEnabled;

    // Speed=4..40 -> Progress=10..100% in the same ten steps as CockpitGauge.
    // Example: speed=4 -> 10%; speed=24 -> 60%; speed=40 -> 100%.
    public float TargetSlopeProgressPercent =>
        Mathf.Clamp(Mathf.Round(value / 4f) * 10f, 10f, 100f);

    public float Value
    {
        get => value;
        set => SetValue(value);
    }

    public bool Dead
    {
        get => dead;
        set
        {
            dead = value;
            if (!Application.isPlaying) ApplyImmediately();
        }
    }

    public void Step(int amount) => SetValue(value + amount);
    public void RightTap() => Step(+8);
    public void LeftTap() => Step(-8);

    void SetValue(float requested)
    {
        value = Mathf.Clamp(requested, 4f, 40f);
        if (Application.isPlaying)
            SynchronizeToCore();
        else
            ApplyImmediately();
    }

    public float TargetNeedleAngleZ => GetTargetAngleZ();
    public float CurrentNeedleAngleZ => displayedAngleZ;
    public float AnglePerNumberStepZ => (angleAt40Z - angleAt4Z) / 9f;
    public bool AnglesHaveCorrectOrder =>
        angleAt4Z > angleAt40Z && angleAt40Z > angleAtDeadZ;
    public RectTransform Needle => needle;

    public void SetReferences(RectTransform needleTransform, Text readout)
    {
        needle = needleTransform;
        digitalText = readout;
        ApplyImmediately();
    }

    void OnEnable()
    {
        FindReferencesIfMissing();
        ApplyImmediately();
        syncedOnce = false;
        if (Application.isPlaying)
            SynchronizeToCore();
    }

    // After every scene object has received OnEnable, reapply the initial
    // value. This prevents Gauge1's own initialization from overwriting it.
    void Start()
    {
        if (Application.isPlaying) SynchronizeToCore();
    }

    void OnDisable()
    {
        initialized = false;
        syncedOnce = false;
    }

    void OnValidate()
    {
        value = Mathf.Clamp(value, 4f, 40f);
        smoothSpeed = Mathf.Max(0f, smoothSpeed);
#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            UnityEditor.EditorApplication.delayCall -= RefreshInEditor;
            UnityEditor.EditorApplication.delayCall += RefreshInEditor;
        }
#endif
    }

#if UNITY_EDITOR
    void RefreshInEditor()
    {
        if (!this || Application.isPlaying) return;
        FindReferencesIfMissing();
        ApplyImmediately();
        UnityEditor.SceneView.RepaintAll();
    }
#endif

    void Update()
    {
        if (!needle) FindReferencesIfMissing();
        if (Application.isPlaying)
        {
            // Inspector edits and delayed GameManager/Core initialization.
            // Only transmit on changes, not on every Update: CockpitGauge
            // also drives the same progress field and must not be fought.
            SlopeStickCore core = targetCore ? targetCore : MainGameManager.core;
            if (!syncedOnce || !Mathf.Approximately(lastSyncedValue, value) ||
                (core && core != lastSyncedCore) ||
                syncTargetSlopeProgress != lastSyncEnabled ||
                (targetProgressGauge && targetProgressGauge != lastSyncedGauge) ||
                (syncTargetSlopeProgress && !targetProgressGauge &&
                    Time.frameCount % 30 == 0))
                SynchronizeToCore();
        }

        if (!needle) return;

        float target = GetTargetAngleZ();
        if (!Application.IsPlaying(gameObject) || !initialized || smoothSpeed <= 0f)
            displayedAngleZ = target;
        else
            displayedAngleZ = Mathf.Lerp(displayedAngleZ, target,
                1f - Mathf.Exp(-smoothSpeed * Time.deltaTime));

        needle.localRotation = Quaternion.Euler(0f, 0f, displayedAngleZ);
        UpdateDigitalReadout();
        initialized = true;
    }

    // Writes MaxGroundSpeed and the corresponding extreme-point position.
    // Also updates the left-side progress meter so its Update() does not
    // immediately overwrite the value we just sent to SlopeStickCore.
    void SynchronizeToCore()
    {
        if (!Application.isPlaying) return;

        SlopeStickCore core = targetCore ? targetCore : MainGameManager.core;
        CockpitGauge gauge = ResolveProgressGauge();
        if (!core)
        {
            if (debugSync && !warnedMissingCore)
            {
                Debug.LogWarning("[GAUGE2] SlopeStickCore is missing; assign Target Core.", this);
                warnedMissingCore = true;
            }
            syncedOnce = false; // retry when game controller appears
            return;
        }

        warnedMissingCore = false;
        core.MaxGroundSpeed = value;
        float progress = TargetSlopeProgressPercent;
        if (syncTargetSlopeProgress)
        {
            // This setter invokes CockpitGauge.ApplyToGame() and updates its
            // Inspector value; direct Core assignment is retained to guarantee
            // the right target even when Gauge1 has no explicit Core reference.
            if (gauge && !Mathf.Approximately(gauge.Value, progress))
                gauge.Value = progress;
            core.targetSlopeProgressPercent = progress;
        }

        lastSyncedValue = value;
        lastSyncedCore = core;
        lastSyncedGauge = gauge;
        lastSyncEnabled = syncTargetSlopeProgress;
        syncedOnce = true;

        if (debugSync)
            Debug.Log($"[GAUGE2] Value={value:F1} speed={core.MaxGroundSpeed:F1} " +
                $"targetProgress={core.targetSlopeProgressPercent:F1}%", this);
    }

    CockpitGauge ResolveProgressGauge()
    {
        if (targetProgressGauge) return targetProgressGauge;

        // Use the two gauge references already wired in CorrespondSubject.
        // Do not pick an arbitrary gauge from another character/canvas.
        foreach (CorrespondSubject mapping in FindObjectsOfType<CorrespondSubject>(true))
        {
            if (mapping.RightDragOnMeter != this || !mapping.LeftOneTapMeter)
                continue;
            targetProgressGauge = mapping.LeftOneTapMeter;
            break;
        }
        return targetProgressGauge;
    }

    float GetTargetAngleZ() => dead
        ? angleAtDeadZ + needleArtOffsetZ
        : Mathf.Lerp(angleAt4Z, angleAt40Z, (value - 4f) / 36f) + needleArtOffsetZ;

    void ApplyImmediately()
    {
        FindReferencesIfMissing();
        displayedAngleZ = GetTargetAngleZ();
        initialized = false;
        if (needle) needle.localRotation = Quaternion.Euler(0f, 0f, displayedAngleZ);
        UpdateDigitalReadout();
    }

    void UpdateDigitalReadout()
    {
        if (!digitalText) return;
        float fraction = Mathf.InverseLerp(angleAt4Z, angleAt40Z,
            displayedAngleZ - needleArtOffsetZ);
        float shown = Mathf.Lerp(4f, 40f, fraction);
        string desired = dead ? "DEAD" : shown.ToString("0.0");
        if (digitalText.text != desired) digitalText.text = desired;
        Color c = dead ? new Color32(255, 138, 83, 255)
                       : new Color32(234, 246, 255, 255);
        if (digitalText.color != c) digitalText.color = c;
    }

    void FindReferencesIfMissing()
    {
        if (!needle)
        {
            Transform child = transform.Find("Needle");
            if (child) needle = child.GetComponent<RectTransform>();
        }
        if (!digitalText)
        {
            Transform child = transform.Find("Digital Reading");
            if (child) digitalText = child.GetComponent<Text>();
        }
    }
}
