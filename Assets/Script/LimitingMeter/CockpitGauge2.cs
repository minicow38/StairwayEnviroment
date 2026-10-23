using UnityEngine;
using UnityEngine.UI;

// Place this on the gauge's parent object, not on its Needle child.
// Compatible with the CockpitGauge v3 UI hierarchy and Inspector.
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
    [SerializeField] float needleArtOffsetZ = 0f;

    [Header("Needle Movement / 追従")]
    [SerializeField, Min(0f)] float smoothSpeed = 12f;

    [Header("References / 子オブジェクト")]
    [SerializeField] RectTransform needle;
    [SerializeField] Text digitalText;

    float displayedAngleZ;
    bool initialized;

    public float Value
    {
        get => value;
        set
        {
            this.value = Mathf.Clamp(value, 4f, 40f);
            if (!Application.isPlaying) ApplyImmediately();
        }
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

    // Right tap: +8, left tap: -8.
    // At 40, another right tap enters DEAD; left from DEAD returns to 40.
    public void Step(int amount)
    {
        MainGameManager.core.MaxGroundSpeed = value + amount;
        value = Mathf.Max(4f, value + amount);

        if (!Application.isPlaying) ApplyImmediately();
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
    }

    void OnDisable() => initialized = false;

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
        // Reflect needle's displayed position while it moves, not only its destination.
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

    public void RightTap()
    {
        if (value < 44)
        {
            Step(+8);
        }
    }
    public void LeftTap()
    {
        if (value > 8)
        {
            Step(-8);
        }

    }
}
