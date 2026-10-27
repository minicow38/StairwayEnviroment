using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

/// <summary>
/// 10-segment vertical gauge.
/// Normal: dual color. Holding: visible charge. After long-press:
/// animated glow and bright cursor; drag up/down to change the selected step.
/// Attach to the PARENT UI Image, not to the individual segments.
/// </summary>
[ExecuteAlways]
[DisallowMultipleComponent]
[RequireComponent(typeof(RectTransform))]
public sealed class CockpitGauge : MonoBehaviour,
    IPointerDownHandler, IInitializePotentialDragHandler, IDragHandler, IPointerUpHandler
{
    const int SegmentCount = 10;
    const float MinimumValue = 10f;
    const float MaximumValue = 100f;
    const float ValuePerSegment = 10f;

    [Header("Reading / 数値")]
    [SerializeField, Range(10f, 100f)] float value = 60f;
    [SerializeField] bool dead;
    // 旧版（4～40 / 4刻み）からのInspector設定を一度だけ10～100へ換算する。
    [SerializeField, HideInInspector] bool upgradedToHundred;

    [Header("Long Press & Swipe / 長押し・スワイプ")]
    [SerializeField, Min(0.2f)] float holdSeconds = 0.48f;
    [SerializeField, Min(3f)] float pixelsPerStep = 50f;
    [SerializeField, Min(1)] int valuePerSwipe = 10;
    [Tooltip("ON: 画面上のメーターの小ささに合わせ、スワイプ感度を自動補正")]
    [SerializeField] bool autoSwipeSensitivity = true;

    [Header("Layout / 見た目")]
    [SerializeField, Min(0f)] float leftPadding = 12f;
    [SerializeField, Min(0f)] float topBottomPadding = 12f;
    [SerializeField, Min(0f)] float barGap = 7f;
    [SerializeField, Min(1f)] float bottomBarWidth = 30f;
    [SerializeField, Min(1f)] float topBarWidth = 130f;

    [Header("Active Position / 操作中の目盛り")]
    [SerializeField] bool showActiveValue = true;
    [SerializeField, Min(0f)] float activeLabelGap = 8f;
    [SerializeField, Min(1f)] float labelFollowSpeed = 18f;
    [SerializeField, Min(0f)] float glowPulseSpeed = 2.5f;
    [Tooltip("長押し中に光が点灯済みの目盛りを往復する速度 (段/秒)")]
    [SerializeField, Min(0f)] float glowScanSpeed = 3f;
    [SerializeField] bool debugInput = false;

    [Header("Skybox Soft Palette / 空になじむ色（Inspectorで調整可）")]
    [SerializeField] Color lowerSegmentColor = new Color(.66f, .84f, .85f, 1f);
    [SerializeField] Color middleSegmentColor = new Color(.52f, .74f, .79f, 1f);
    [SerializeField] Color upperSegmentColor = new Color(.81f, .87f, .84f, 1f);
    [SerializeField] Color lowerGlowColor = new Color(.79f, .92f, .93f, 1f);
    [SerializeField] Color upperGlowColor = new Color(.89f, .97f, .94f, 1f);
    [SerializeField] Color cursorColor = new Color(.87f, .98f, .96f, .87f);
    [SerializeField] Color badgeNormalColor = new Color(.27f, .50f, .55f, .58f);
    [SerializeField] Color badgeGlowColor = new Color(.40f, .64f, .66f, .74f);
    [SerializeField] Color badgeLabelColor = new Color(.94f, .98f, .97f, 1f);

    [Header("References / 参照 (未設定なら自動生成)")]
    [SerializeField] Slider slider;
    [SerializeField] Text digitalText;

    readonly GaugeSegmentGraphic[] segments = new GaugeSegmentGraphic[SegmentCount];
    RectTransform rectTransform;
    RectTransform badgeRect;
    Image badgeBackground;
    Text badgeText;
    RectTransform cursorRect;
    Image cursorImage;
    bool pressed;
    bool glowMode;
    int pressedPointerId;
    float pressedAt;
    float lastPointerScreenY;
    float floatingIndex;
    float badgeY;
    float nextVisualRefresh;
    float observedValue = float.NaN;
    bool observedDead;

    // The gauge owns touch-start attribution, independent of press/glow visuals.
    // Once a gesture begins on this UI, its finger ID stays excluded from turns
    // until Ended/Canceled. No post-release delay is necessary.
    readonly HashSet<int> turnExcludedFingerIds = new HashSet<int>();
    readonly List<RaycastResult> turnHitResults = new List<RaycastResult>(8);
    bool turnExcludedMouseGesture;

    public float Value
    {
        get => value;
        set => SetMeterValue(value, true);
    }
    public bool Dead
    {
        get => dead;
        set { dead = value; RefreshVisuals(); }
    }
    // Unlike an auto-property, this always reflects the existing glowMode field.
    // Keep the setter for compatibility with callers using IsGlowMode = true/false.
    public bool IsGlowMode
    {
        get => glowMode;
        set => glowMode = value;
    }
    public bool IsPressed => pressed;
    public float ActiveSegmentPosition => floatingIndex;

    /// <summary>
    /// Called from SlopeStickCore through CorrespondSubject for EVERY touch.
    /// Only the Began position determines whether that touch belongs to this UI.
    /// The same finger remains excluded even after it leaves the gauge's bounds.
    /// Ended/Canceled are excluded on their own frame, then the ID is forgotten.
    /// </summary>
    public bool ShouldIgnoreTurnTouch(Touch touch)
    {
        int id = touch.fingerId;
        if (touch.phase == TouchPhase.Began)
        {
            if (IsGaugeHitAt(touch.position))
                turnExcludedFingerIds.Add(id);
            else
                turnExcludedFingerIds.Remove(id);
        }

        bool excluded = turnExcludedFingerIds.Contains(id);
        if (touch.phase == TouchPhase.Ended || touch.phase == TouchPhase.Canceled)
            turnExcludedFingerIds.Remove(id);

        return excluded;
    }

    /// <summary>
    /// Unity Editor's mouse path uses the same start-position rule.
    /// Does not use IsGlowMode/pressed because those end before input cleanup.
    /// </summary>
    public bool ShouldIgnoreTurnMouse(Vector2 screenPosition, bool mouseDown, bool mouseUp)
    {
        if (mouseDown)
            turnExcludedMouseGesture = IsGaugeHitAt(screenPosition);

        bool excluded = turnExcludedMouseGesture;
        if (mouseUp)
            turnExcludedMouseGesture = false;

        return excluded;
    }

    public void ResetTurnGestureTracking()
    {
        turnExcludedFingerIds.Clear();
        turnExcludedMouseGesture = false;
    }

    bool IsGaugeHitAt(Vector2 screenPosition)
    {
        if (!isActiveAndEnabled || !gameObject.activeInHierarchy)
            return false;

        EventSystem eventSystem = EventSystem.current;
        if (eventSystem == null)
            return false;

        // Raycast on the Began frame itself. This works regardless of whether
        // the EventSystem or SlopeStickCore Update happens first in that frame.
        PointerEventData pointer = new PointerEventData(eventSystem)
        {
            position = screenPosition
        };
        turnHitResults.Clear();
        eventSystem.RaycastAll(pointer, turnHitResults);

        for (int i = 0; i < turnHitResults.Count; i++)
        {
            GameObject hitObject = turnHitResults[i].gameObject;
            if (!hitObject)
                continue;

            Transform hit = hitObject.transform;
            if (hit == transform || hit.IsChildOf(transform))
                return true;
        }

        return false;
    }

    void OnEnable()
    {
        UpgradePreviousScale();
        EnsureHierarchy();
        if (slider != null)
        {
            slider.onValueChanged.RemoveListener(OnSliderValueChanged);
            slider.onValueChanged.AddListener(OnSliderValueChanged);
        }
        floatingIndex = GetValueIndex();
        RefreshVisuals();
    }

    void OnDisable()
    {
        if (slider != null) slider.onValueChanged.RemoveListener(OnSliderValueChanged);
        pressed = false;
        glowMode = false;
        ResetTurnGestureTracking();
        if (badgeRect != null) badgeRect.gameObject.SetActive(false);
        if (cursorRect != null) cursorRect.gameObject.SetActive(false);
    }

    void OnApplicationPause(bool paused)
    {
        if (paused) ResetTurnGestureTracking();
    }

    void OnApplicationFocus(bool hasFocus)
    {
        if (!hasFocus) ResetTurnGestureTracking();
    }

    void OnValidate()
    {
        UpgradePreviousScale();
        value = Quantize(value);
        holdSeconds = Mathf.Max(0.2f, holdSeconds);
        pixelsPerStep = Mathf.Max(3f, pixelsPerStep);
        // 10刻みに補正。以前の4刻みが残っても不正な段数にしない。
        valuePerSwipe = Mathf.Clamp(Mathf.RoundToInt(valuePerSwipe / ValuePerSegment), 1, SegmentCount) * (int)ValuePerSegment;
        bottomBarWidth = Mathf.Max(1f, bottomBarWidth);
        topBarWidth = Mathf.Max(bottomBarWidth, topBarWidth);
        labelFollowSpeed = Mathf.Max(1f, labelFollowSpeed);
        glowPulseSpeed = Mathf.Max(0f, glowPulseSpeed);
        glowScanSpeed = Mathf.Max(0f, glowScanSpeed);
#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            UnityEditor.EditorApplication.delayCall -= EditorRefresh;
            UnityEditor.EditorApplication.delayCall += EditorRefresh;
        }
#endif
    }

#if UNITY_EDITOR
    void EditorRefresh()
    {
        if (this == null || Application.isPlaying) return;
        EnsureHierarchy();
        floatingIndex = GetValueIndex();
        RefreshVisuals();
    }
#endif

    void Update()
    {
        if (!Application.isPlaying) return;

        float q = Quantize(value);
        if (!Mathf.Approximately(observedValue, q) || observedDead != dead)
        {
            value = q;
            if (!glowMode) floatingIndex = GetValueIndex();
            RefreshVisuals();
        }

        if (!pressed) return;
        if (!glowMode && Time.unscaledTime - pressedAt >= holdSeconds)
            BeginGlow();

        // Visual feedback runs while the finger is held, even without any drag event.
        if (Time.unscaledTime >= nextVisualRefresh)
        {
            nextVisualRefresh = Time.unscaledTime + 1f / 30f;
            RefreshVisuals();
        }
        MoveActiveBadge(false);
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        if (!Application.isPlaying || pressed) return;
        pressed = true;
        pressedPointerId = eventData.pointerId;
        pressedAt = Time.unscaledTime;
        lastPointerScreenY = eventData.position.y;
        floatingIndex = GetValueIndex();
        nextVisualRefresh = 0f;
        RefreshVisuals(); // pointer-down feedback is immediate
        if (debugInput) Debug.Log("[GAUGE] DOWN index=" + floatingIndex);
    }

    public void OnInitializePotentialDrag(PointerEventData eventData)
    {
        // Small World Space UI: don't wait for EventSystem drag threshold.
        eventData.useDragThreshold = false;
    }

    void BeginGlow()
    {
        if (glowMode || !pressed) return;
        glowMode = true;
        floatingIndex = GetValueIndex();
        nextVisualRefresh = 0f;
        RefreshVisuals();
        MoveActiveBadge(true);
        if (debugInput) Debug.Log("[GAUGE] GLOW ON; effectivePixelsPerStep=" + EffectivePixelsPerStep().ToString("F1"));
    }

    public void OnDrag(PointerEventData eventData)
    {
        if (!Application.isPlaying || !pressed || eventData.pointerId != pressedPointerId) return;

        float dy = eventData.position.y - lastPointerScreenY;
        lastPointerScreenY = eventData.position.y;
        if (!glowMode)
        {
            if (Time.unscaledTime - pressedAt < holdSeconds) return;
            BeginGlow();
        }

        float stepPixels = EffectivePixelsPerStep();
        floatingIndex = Mathf.Clamp(floatingIndex + dy / stepPixels * (valuePerSwipe / ValuePerSegment),
            0f, SegmentCount - 1f);
        int index = Mathf.Clamp(Mathf.RoundToInt(floatingIndex), 0, SegmentCount - 1);
        float newValue = MinimumValue + index * ValuePerSegment;

        if (!Mathf.Approximately(value, newValue) || dead)
        {
            SetMeterValue(newValue, true);
            if (debugInput) Debug.Log("[GAUGE] DRAG value=" + value + " index=" + floatingIndex.ToString("F2"));
        }
        else RefreshVisuals();
        MoveActiveBadge(false);
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        if (!pressed || eventData.pointerId != pressedPointerId) return;
        EndPress();
    }

    void EndPress()
    {
        pressed = false;
        glowMode = false;
        floatingIndex = GetValueIndex();
        RefreshVisuals();
        if (debugInput)
        {
            Debug.Log("[GAUGE] UP value=" + value);
        }
    }

    public void Step(int amount) => SetMeterValue(value + amount, true);
    public void RightTap() => Step(+10);
    public void LeftTap() => Step(-10);

    void SetMeterValue(float requested, bool applyToGame)
    {
        float previous = value;
        value = Quantize(requested);
        dead = false;
        if (!glowMode) floatingIndex = GetValueIndex();
        RefreshVisuals();
        if (applyToGame && Application.isPlaying && !Mathf.Approximately(previous, value)) ApplyToGame();
    }

    void OnSliderValueChanged(float index) =>
        SetMeterValue(MinimumValue + Mathf.Round(index) * ValuePerSegment, true);

    static float Quantize(float raw)
    {
        int index = Mathf.Clamp(Mathf.RoundToInt((raw - MinimumValue) / ValuePerSegment), 0, SegmentCount - 1);
        return Mathf.Clamp(MinimumValue + index * ValuePerSegment, MinimumValue, MaximumValue);
    }

    void UpgradePreviousScale()
    {
        if (upgradedToHundred) return;
        // 古いScene/Prefabで設定された値の「段番号」を保持する。
        // 例: Value=24, ValuePerSwipe=4 -> Value=60, ValuePerSwipe=10
        //     Value=40, ValuePerSwipe=8 -> Value=100, ValuePerSwipe=20
        if (valuePerSwipe > 0 && valuePerSwipe < 10 && value >= 4f && value <= 40f)
        {
            int oldIndex = Mathf.Clamp(Mathf.RoundToInt((value - 4f) / 4f), 0, SegmentCount - 1);
            value = MinimumValue + oldIndex * ValuePerSegment;
            int previousSteps = Mathf.Max(1, Mathf.RoundToInt(valuePerSwipe / 4f));
            valuePerSwipe = Mathf.Clamp(previousSteps, 1, SegmentCount) * (int)ValuePerSegment;
        }
        upgradedToHundred = true;
        value = Quantize(value);
    }

    int GetValueIndex() => Mathf.Clamp(Mathf.RoundToInt((value - MinimumValue) / ValuePerSegment), 0, SegmentCount - 1);

    void ApplyToGame()
    {
        if (MainGameManager.core != null)
            MainGameManager.core.targetSlopeProgressPercent = value;
    }

    float EffectivePixelsPerStep()
    {
        if (!autoSwipeSensitivity || segments[0] == null || segments[SegmentCount - 1] == null)
            return Mathf.Max(3f, pixelsPerStep);

        Canvas canvas = GetComponentInParent<Canvas>();
        Camera cam = null;
        if (canvas != null && canvas.rootCanvas.renderMode != RenderMode.ScreenSpaceOverlay)
            cam = canvas.rootCanvas.worldCamera != null ? canvas.rootCanvas.worldCamera : Camera.main;

        var low = (RectTransform)segments[0].transform;
        var high = (RectTransform)segments[SegmentCount - 1].transform;
        // WorldToScreenPoint also works for overlay UI with a null camera through RectTransformUtility.
        Vector3 lowWorld = low.TransformPoint(low.rect.center);
        Vector3 highWorld = high.TransformPoint(high.rect.center);
        Vector2 lowScreen = RectTransformUtility.WorldToScreenPoint(cam, lowWorld);
        Vector2 highScreen = RectTransformUtility.WorldToScreenPoint(cam, highWorld);
        float perVisibleStep = Mathf.Abs(highScreen.y - lowScreen.y) / (SegmentCount - 1f);
        if (perVisibleStep < 0.01f) return Mathf.Max(3f, pixelsPerStep);
        return Mathf.Max(4f, Mathf.Min(pixelsPerStep, perVisibleStep));
    }

    void EnsureHierarchy()
    {
        rectTransform = GetComponent<RectTransform>();
        Image hitArea = GetComponent<Image>();
        if (hitArea == null) hitArea = gameObject.AddComponent<Image>();
        hitArea.color = Color.clear;
        hitArea.raycastTarget = true;

        if (slider == null)
        {
            Transform existing = transform.Find("__ValueSlider");
            if (existing != null) slider = existing.GetComponent<Slider>();
            if (slider == null)
            {
                GameObject obj = new GameObject("__ValueSlider", typeof(RectTransform), typeof(Slider));
                obj.transform.SetParent(transform, false);
                slider = obj.GetComponent<Slider>();
                RectTransform sr = obj.GetComponent<RectTransform>();
                sr.anchorMin = sr.anchorMax = Vector2.zero;
                sr.pivot = Vector2.zero;
                sr.sizeDelta = Vector2.zero;
            }
        }
        slider.minValue = 0f;
        slider.maxValue = 9f;
        slider.wholeNumbers = true;
        slider.interactable = false;
        slider.direction = Slider.Direction.BottomToTop;
        slider.SetValueWithoutNotify(GetValueIndex());

        for (int i = 0; i < SegmentCount; i++)
        {
            if (segments[i] != null) continue;
            string name = "Segment_" + (i + 1).ToString("00");
            Transform t = transform.Find(name);
            if (t != null) segments[i] = t.GetComponent<GaugeSegmentGraphic>();
            if (segments[i] == null)
            {
                GameObject obj = new GameObject(name, typeof(RectTransform), typeof(CanvasRenderer), typeof(GaugeSegmentGraphic));
                obj.transform.SetParent(transform, false);
                segments[i] = obj.GetComponent<GaugeSegmentGraphic>();
            }
            segments[i].raycastTarget = false;
        }
        LayoutSegments();
        EnsureCursor();
        EnsureBadge();
    }

    void EnsureCursor()
    {
        Transform child = transform.Find("__ActiveCursor");
        if (child == null)
        {
            GameObject go = new GameObject("__ActiveCursor", typeof(RectTransform), typeof(Image));
            go.transform.SetParent(transform, false);
            child = go.transform;
        }
        cursorRect = child.GetComponent<RectTransform>();
        cursorImage = child.GetComponent<Image>();
        cursorImage.raycastTarget = false;
        cursorRect.anchorMin = cursorRect.anchorMax = Vector2.zero;
        cursorRect.pivot = new Vector2(0.5f, 0.5f);
        cursorRect.sizeDelta = new Vector2(8f, 28f);
        cursorRect.SetAsLastSibling();
        cursorRect.gameObject.SetActive(false);
    }

    void EnsureBadge()
    {
        Transform root = transform.Find("__ActiveValueBadge");
        if (root == null)
        {
            GameObject go = new GameObject("__ActiveValueBadge", typeof(RectTransform), typeof(Image));
            go.transform.SetParent(transform, false);
            root = go.transform;
        }
        badgeRect = root.GetComponent<RectTransform>();
        badgeBackground = root.GetComponent<Image>();
        if (badgeBackground == null) badgeBackground = root.gameObject.AddComponent<Image>();
        badgeBackground.color = badgeNormalColor;
        badgeBackground.raycastTarget = false;
        badgeRect.anchorMin = badgeRect.anchorMax = Vector2.zero;
        badgeRect.pivot = new Vector2(0f, 0.5f);
        badgeRect.sizeDelta = new Vector2(54f, 30f);
        badgeRect.SetAsLastSibling();

        Transform textChild = root.Find("Number");
        if (textChild == null)
        {
            GameObject go = new GameObject("Number", typeof(RectTransform), typeof(Text));
            go.transform.SetParent(root, false);
            textChild = go.transform;
        }
        badgeText = textChild.GetComponent<Text>();
        if (badgeText == null) badgeText = textChild.gameObject.AddComponent<Text>();
        RectTransform tr = (RectTransform)badgeText.transform;
        tr.anchorMin = Vector2.zero;
        tr.anchorMax = Vector2.one;
        tr.offsetMin = tr.offsetMax = Vector2.zero;
        badgeText.raycastTarget = false;
        badgeText.alignment = TextAnchor.MiddleCenter;
        badgeText.fontStyle = FontStyle.Bold;
        badgeText.fontSize = 21;
        badgeText.color = badgeLabelColor;
        if (badgeText.font == null)
            badgeText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        badgeRect.gameObject.SetActive(false);
    }

    void OnRectTransformDimensionsChange()
    {
        if (rectTransform != null && segments[0] != null) LayoutSegments();
    }

    void LayoutSegments()
    {
        if (rectTransform == null) return;
        float availableHeight = Mathf.Max(10f, rectTransform.rect.height - 2f * topBottomPadding);
        float actualGap = Mathf.Min(barGap, availableHeight / (SegmentCount * 2f));
        float height = Mathf.Max(1f, (availableHeight - actualGap * (SegmentCount - 1)) / SegmentCount);
        float parentWidth = Mathf.Max(1f, rectTransform.rect.width);
        float referenceWidth = Mathf.Max(1f, topBarWidth + 2f * leftPadding);
        float scaledPadding = leftPadding * parentWidth / referenceWidth;
        float widest = Mathf.Max(0.1f, parentWidth - 2f * scaledPadding);
        float narrowest = widest * Mathf.Clamp01(bottomBarWidth / topBarWidth);

        for (int i = 0; i < SegmentCount; i++)
        {
            if (segments[i] == null) continue;
            RectTransform r = (RectTransform)segments[i].transform;
            r.anchorMin = r.anchorMax = Vector2.zero;
            r.pivot = Vector2.zero;
            r.sizeDelta = new Vector2(Mathf.Lerp(narrowest, widest, i / 9f), height);
            r.anchoredPosition = new Vector2(scaledPadding, topBottomPadding + i * (height + actualGap));
        }
    }

    void MoveActiveBadge(bool immediate)
    {
        if (segments[0] == null || badgeRect == null) return;
        int nearest = Mathf.Clamp(Mathf.RoundToInt(floatingIndex), 0, SegmentCount - 1);
        RectTransform bar = (RectTransform)segments[nearest].transform;
        float y0 = ((RectTransform)segments[0].transform).anchoredPosition.y + bar.rect.height * .5f;
        float y9 = ((RectTransform)segments[9].transform).anchoredPosition.y + bar.rect.height * .5f;
        float targetY = Mathf.Lerp(y0, y9, floatingIndex / 9f);
        badgeY = immediate ? targetY : Mathf.Lerp(badgeY, targetY,
            1f - Mathf.Exp(-labelFollowSpeed * Time.unscaledDeltaTime));

        // The bar itself always has a bright white cursor while held.
        if (cursorRect != null)
        {
            bool show = pressed;
            if (cursorRect.gameObject.activeSelf != show) cursorRect.gameObject.SetActive(show);
            float x = bar.anchoredPosition.x + Mathf.Min(9f, bar.rect.width * .25f);
            cursorRect.sizeDelta = new Vector2(8f, Mathf.Max(10f, bar.rect.height + 5f));
            cursorRect.anchoredPosition = new Vector2(x, badgeY);
            if (cursorImage != null)
            {
                float phase = Time.unscaledTime * glowPulseSpeed * Mathf.PI * 2f;
                float opacity = glowMode ? .92f + .08f * Mathf.Sin(phase) : .45f + .25f * HoldProgress();
                cursorImage.color = new Color(
                    cursorColor.r, cursorColor.g, cursorColor.b,
                    cursorColor.a * opacity);
            }
        }

        bool badgeVisible = pressed && showActiveValue;
        if (badgeRect.gameObject.activeSelf != badgeVisible) badgeRect.gameObject.SetActive(badgeVisible);
        if (!badgeVisible) return;

        float xBadge = bar.anchoredPosition.x + bar.rect.width + activeLabelGap;
        badgeRect.anchoredPosition = new Vector2(xBadge, badgeY);
        if (badgeText != null) badgeText.text = value.ToString("0");

        // In tiny World Space canvases, scale the badge so its screen glyphs remain readable.
        float pixelsPerUi = GetScreenPixelsPerUiUnit();
        float labelScale = Mathf.Clamp(1f / Mathf.Max(.04f, pixelsPerUi), 1f, 22f);
        float pulse = (Mathf.Sin(Time.unscaledTime * glowPulseSpeed * Mathf.PI * 2f) + 1f) * .5f;
        badgeRect.localScale = Vector3.one * labelScale * (1f + (glowMode ? .035f * pulse : 0f));
        if (badgeBackground != null)
            badgeBackground.color = glowMode ? badgeGlowColor : badgeNormalColor;
    }

    float GetScreenPixelsPerUiUnit()
    {
        if (segments[0] == null || segments[9] == null) return 1f;
        Canvas canvas = GetComponentInParent<Canvas>();
        Camera cam = canvas != null && canvas.rootCanvas.renderMode != RenderMode.ScreenSpaceOverlay
            ? (canvas.rootCanvas.worldCamera != null ? canvas.rootCanvas.worldCamera : Camera.main) : null;
        RectTransform a = (RectTransform)segments[0].transform;
        RectTransform b = (RectTransform)segments[9].transform;
        Vector2 screenA = RectTransformUtility.WorldToScreenPoint(cam, a.TransformPoint(a.rect.center));
        Vector2 screenB = RectTransformUtility.WorldToScreenPoint(cam, b.TransformPoint(b.rect.center));
        float units = Mathf.Abs(b.anchoredPosition.y - a.anchoredPosition.y);
        if (units < .001f) return 1f;
        return Mathf.Max(.01f, Mathf.Abs(screenB.y - screenA.y) / units);
    }

    float HoldProgress() => Mathf.Clamp01((Time.unscaledTime - pressedAt) / Mathf.Max(.2f, holdSeconds));

    void RefreshVisuals()
    {
        if (slider != null) slider.SetValueWithoutNotify(GetValueIndex());
        int litIndex = GetValueIndex();
        float pulse = .5f + .5f * Mathf.Sin(Time.unscaledTime * glowPulseSpeed * Mathf.PI * 2f);
        float scanIndex = glowMode
            ? Mathf.PingPong(Time.unscaledTime * glowScanSpeed, Mathf.Max(.05f, floatingIndex))
            : -100f;
        float charge = pressed && !glowMode ? HoldProgress() : 0f;

        for (int i = 0; i < SegmentCount; i++)
        {
            if (segments[i] == null) continue;
            bool lit = !dead && i <= litIndex;
            float focus = pressed ? Mathf.Clamp01(1f - Mathf.Abs(i - floatingIndex)) : 0f;
            float scan = glowMode ? Mathf.Clamp01(1f - Mathf.Abs(i - scanIndex) / 1.3f) : 0f;
            segments[i].Configure(GetDualColor(i), GetGlowColor(i), lit, glowMode, focus, pulse, charge, scan);
        }
        if (pressed) MoveActiveBadge(false);
        else
        {
            if (badgeRect != null && badgeRect.gameObject.activeSelf) badgeRect.gameObject.SetActive(false);
            if (cursorRect != null && cursorRect.gameObject.activeSelf) cursorRect.gameObject.SetActive(false);
        }
        if (digitalText != null) digitalText.text = dead ? "DEAD" : value.ToString("0");
        observedValue = value;
        
        observedDead = dead;
    }

    Color GetDualColor(int i)
    {
        float t = Mathf.Clamp01(i / 9f);
        // Slight three-stop sky gradient, without green/yellow/red traffic-light colours.
        return t <= .5f
            ? Color.Lerp(lowerSegmentColor, middleSegmentColor, t * 2f)
            : Color.Lerp(middleSegmentColor, upperSegmentColor, (t - .5f) * 2f);
    }

    Color GetGlowColor(int i) =>
        Color.Lerp(lowerGlowColor, upperGlowColor, Mathf.Clamp01(i / 9f));
}
