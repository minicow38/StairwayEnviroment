using UnityEngine;
using UnityEngine.UI;

// Place on a separate Screen Space Overlay Canvas, NOT under the dial.
// Tracks an independently rendered needle + digital display to the dial's
// apparent screen center, even when the dial has a different world-space depth.
[ExecuteAlways]
[DefaultExecutionOrder(3000)]
[DisallowMultipleComponent]
public sealed class CockpitGaugeScreenRig : MonoBehaviour
{
    [Header("Scene References / シーン上の時計")]
    [SerializeField] CockpitGauge2 gauge;
    [SerializeField] RectTransform backgroundFace;
    [SerializeField] Camera projectionCameraOverride;

    [Header("Front Overlay / 最前面")]
    [SerializeField] Canvas overlayCanvas;
    [SerializeField] RectTransform foregroundFace;
    [SerializeField] RectTransform touchSurface;
    [SerializeField] RectTransform frontNeedle;
    [SerializeField] Text frontDigitalReading;

    [Header("Projection")]
    [Tooltip("ON: 時計の表示角度も投影。針は時計の回転角+数値の回転角になります。")]
    [SerializeField] bool followDialRotation = true;

    public CockpitGauge2 Gauge => gauge;

    public void Setup(CockpitGauge2 owner, RectTransform source, Canvas canvas,
        RectTransform foreground, RectTransform hitSurface,
        RectTransform needle, Text reading)
    {
        gauge = owner;
        backgroundFace = source;
        overlayCanvas = canvas;
        foregroundFace = foreground;
        touchSurface = hitSurface;
        frontNeedle = needle;
        frontDigitalReading = reading;
        // The original gauge now controls components OUTSIDE its own hierarchy.
        // No local Needle child is necessary.
        if (gauge) gauge.SetReferences(frontNeedle, frontDigitalReading);
        Project();
    }

    void OnEnable() => Project();
    void LateUpdate() => Project();

    void Project()
    {
        if (!backgroundFace || !overlayCanvas || !foregroundFace || !touchSurface) return;
        RectTransform canvasRect = overlayCanvas.GetComponent<RectTransform>();
        if (!canvasRect) return;

        Canvas sourceCanvas = backgroundFace.GetComponentInParent<Canvas>();
        bool sourceIsOverlay = sourceCanvas &&
            sourceCanvas.rootCanvas.renderMode == RenderMode.ScreenSpaceOverlay;

        Camera cameraToUse = null;
        if (!sourceIsOverlay)
        {
            cameraToUse = projectionCameraOverride;
            if (!cameraToUse && sourceCanvas) cameraToUse = sourceCanvas.worldCamera;
            if (!cameraToUse) cameraToUse = Camera.main;
        }
        if (!sourceIsOverlay && !cameraToUse)
        {
            Show(false);
            return;
        }

        Rect r = backgroundFace.rect;
        Vector3 center = backgroundFace.TransformPoint(new Vector3(r.center.x, r.center.y, 0f));
        Vector3 left = backgroundFace.TransformPoint(new Vector3(r.xMin, r.center.y, 0f));
        Vector3 right = backgroundFace.TransformPoint(new Vector3(r.xMax, r.center.y, 0f));
        Vector3 top = backgroundFace.TransformPoint(new Vector3(r.center.x, r.yMax, 0f));
        Vector3 bottom = backgroundFace.TransformPoint(new Vector3(r.center.x, r.yMin, 0f));

        Vector2 screenCenter, screenLeft, screenRight, screenTop, screenBottom;
        if (!TryProject(center, cameraToUse, sourceIsOverlay, out screenCenter) ||
            !TryProject(left, cameraToUse, sourceIsOverlay, out screenLeft) ||
            !TryProject(right, cameraToUse, sourceIsOverlay, out screenRight) ||
            !TryProject(top, cameraToUse, sourceIsOverlay, out screenTop) ||
            !TryProject(bottom, cameraToUse, sourceIsOverlay, out screenBottom))
        {
            Show(false);
            return;
        }

        Vector2 localCenter, localLeft, localRight, localTop, localBottom;
        if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(canvasRect, screenCenter, null, out localCenter) ||
            !RectTransformUtility.ScreenPointToLocalPointInRectangle(canvasRect, screenLeft, null, out localLeft) ||
            !RectTransformUtility.ScreenPointToLocalPointInRectangle(canvasRect, screenRight, null, out localRight) ||
            !RectTransformUtility.ScreenPointToLocalPointInRectangle(canvasRect, screenTop, null, out localTop) ||
            !RectTransformUtility.ScreenPointToLocalPointInRectangle(canvasRect, screenBottom, null, out localBottom))
        {
            Show(false);
            return;
        }

        float width = Vector2.Distance(localLeft, localRight);
        float height = Vector2.Distance(localBottom, localTop);
        if (width < 1f || height < 1f)
        {
            Show(false);
            return;
        }

        Show(true);

        // Independent front elements are projected to the background clock's
        // on-screen center. Their world Z values are irrelevant to alignment.
        SetSurface(foregroundFace, localCenter, new Vector2(width, height));
        Vector2 up = localTop - localBottom;
        float faceZ = Mathf.Atan2(-up.x, up.y) * Mathf.Rad2Deg;
        foregroundFace.localRotation = Quaternion.Euler(0f, 0f, followDialRotation ? faceZ : 0f);

        // Hit regions remain SCREEN-LEFT / SCREEN-RIGHT even when the clock turns.
        SetSurface(touchSurface, localCenter, new Vector2(width, height));
        touchSurface.localRotation = Quaternion.identity;
    }

    static bool TryProject(Vector3 point, Camera cameraToUse, bool isOverlay, out Vector2 xy)
    {
        Vector3 screen = isOverlay
            ? RectTransformUtility.WorldToScreenPoint(null, point)
            : cameraToUse.WorldToScreenPoint(point);
        xy = new Vector2(screen.x, screen.y);
        return isOverlay || screen.z > 0f;
    }

    static void SetSurface(RectTransform surface, Vector2 pos, Vector2 size)
    {
        surface.anchorMin = surface.anchorMax = new Vector2(.5f, .5f);
        surface.pivot = new Vector2(.5f, .5f);
        surface.anchoredPosition = pos;
        surface.sizeDelta = size;
    }

    void Show(bool show)
    {
        if (foregroundFace && foregroundFace.gameObject.activeSelf != show)
            foregroundFace.gameObject.SetActive(show);
        if (touchSurface && touchSurface.gameObject.activeSelf != show)
            touchSurface.gameObject.SetActive(show);
    }
}
