using UnityEngine;
using UnityEngine.EventSystems;

// Attach to an invisible UI Image (Raycast Target = ON) on the overlay Canvas.
[DisallowMultipleComponent]
public sealed class CockpitGaugeScreenTap : MonoBehaviour,
    IPointerClickHandler, ICanvasRaycastFilter
{
    [SerializeField] CockpitGauge gauge;
    [SerializeField] RectTransform wholeTouchSurface;
    [SerializeField] bool increase;
    [SerializeField, Range(.5f, 1f)] float circleRadiusFraction = .98f;

    public void Setup(CockpitGauge owner, RectTransform all, bool isIncrease)
    {
        gauge = owner;
        wholeTouchSurface = all;
        increase = isIncrease;
    }

    public void OnPointerClick(PointerEventData eventData)
    {
        if (!gauge || !gauge.isActiveAndEnabled) return;
        if (eventData.button != PointerEventData.InputButton.Left) return;

        // Works with both v3 and v4 CockpitGauge scripts (uses public Value/Dead).
        if (increase)
        {
            if (gauge.Dead) return;
            if (gauge.Value >= 39.999f) gauge.Dead = true;
            else gauge.Value = Mathf.Min(40f, gauge.Value + 8f);
        }
        else
        {
            if (gauge.Dead)
            {
                gauge.Dead = false;
                gauge.Value = 40f;
            }
            else gauge.Value = Mathf.Max(4f, gauge.Value - 8f);
        }
    }

    // Even though the hit images are rectangular, only the actual circular
    // face accepts touches. No physics raycasts or world Z collision needed.
    public bool IsRaycastLocationValid(Vector2 screenPoint, Camera eventCamera)
    {
        if (!gauge || !gauge.isActiveAndEnabled || !wholeTouchSurface) return false;
        if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(
            wholeTouchSurface, screenPoint, eventCamera, out Vector2 local)) return false;

        Rect r = wholeTouchSurface.rect;
        if (r.width <= .001f || r.height <= .001f) return false;
        float x = (local.x - r.center.x) / (r.width * .5f);
        float y = (local.y - r.center.y) / (r.height * .5f);
        return (x * x + y * y) <= circleRadiusFraction * circleRadiusFraction;
    }
}
