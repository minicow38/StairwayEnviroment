using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Creates left-aligned trapezoids directly on Canvas (no sprites needed).
/// Active row: bright white/yellow focus; while held, an amber scan travels
/// across the lit rows. Shows a charge animation before long-press completes.
/// </summary>
[DisallowMultipleComponent]
public sealed class GaugeSegmentGraphic : MaskableGraphic
{
    Color dualColor = Color.green;
    Color glowColor = Color.yellow;
    bool lit;
    bool glowMode;
    float focus;
    float pulse;
    float charge;
    float scan;

    public void Configure(Color normal, Color glowingColor, bool illuminated, bool glowing,
        float focus01 = 0f, float pulse01 = 0f, float charge01 = 0f, float scan01 = 0f)
    {
        dualColor = normal;
        glowColor = glowingColor;
        lit = illuminated;
        glowMode = glowing;
        focus = Mathf.Clamp01(focus01);
        pulse = Mathf.Clamp01(pulse01);
        charge = Mathf.Clamp01(charge01);
        scan = Mathf.Clamp01(scan01);
        SetVerticesDirty();
    }

    protected override void OnPopulateMesh(VertexHelper vh)
    {
        vh.Clear();
        Rect r = rectTransform.rect;
        if (r.width < 1f || r.height < 1f) return;
        float slant = Mathf.Min(5f, r.width * 0.13f);

        float strength = Mathf.Clamp01(focus * (.75f + .25f * pulse) + scan * .32f);
        if (glowMode && lit)
        {
            for (int n = 3; n >= 1; n--)
            {
                float spread = n * (3.5f + 2.8f * strength);
                Color halo = glowColor;
                halo.a = (n == 3 ? .05f : n == 2 ? .10f : .19f) + strength * (n == 3 ? .10f : .26f);
                AddTrapezoid(vh, r.xMin - spread, r.yMin - spread,
                    r.yMax + spread, r.xMax + spread, r.xMax - slant + spread,
                    halo, halo);
            }
        }

        Color top, bottom;
        if (!lit)
        {
            top = new Color(.30f,.33f,.36f,.95f);
            bottom = new Color(.20f,.23f,.26f,.95f);
        }
        else if (glowMode)
        {
            // Active bar turns visibly white; non-active bars remain amber-orange.
            top = Color.Lerp(glowColor, Color.white, .10f + .83f * strength);
            bottom = Color.Lerp(glowColor, Color.white, .05f + .58f * strength);
        }
        else
        {
            top = Color.Lerp(dualColor, Color.white, .08f + charge * focus * .55f);
            bottom = Color.Lerp(dualColor, Color.white, charge * focus * .28f);
        }

        if (glowMode && lit)
        {
            Color rim = Color.Lerp(new Color(1f,.77f,.32f,1f), Color.white, strength);
            float thickness = 1.8f + strength * 2.7f;
            AddTrapezoid(vh, r.xMin - thickness, r.yMin - thickness,
                r.yMax + thickness, r.xMax + thickness,
                r.xMax - slant + thickness, rim, rim);
        }
        else if (lit && charge > .01f && focus > .01f)
        {
            Color pending = new Color(1f, .96f, .65f, charge * focus * .95f);
            AddTrapezoid(vh, r.xMin - 2f, r.yMin - 2f,
                r.yMax + 2f, r.xMax + 2f, r.xMax - slant + 2f,
                pending, pending);
        }

        AddTrapezoid(vh, r.xMin, r.yMin, r.yMax, r.xMax, r.xMax - slant, top, bottom);

        if (glowMode && lit)
        {
            // Small moving white glint on each active row, stronger on the selected row.
            float u = Mathf.PingPong(Time.unscaledTime * .7f, 1f);
            float stripeX = Mathf.Lerp(r.xMin + 3f, r.xMax - 4f, u);
            Color sheen = new Color(1f,1f,1f, .09f + .65f * strength);
            AddTrapezoid(vh, stripeX - 3f, r.yMin + 2f, r.yMax - 2f,
                stripeX + 3f, stripeX + 3f, sheen, sheen);
        }
        float h = Mathf.Min(2f, r.height * .16f);
        Color highlight = lit ? new Color(1f,1f,1f, .13f + (glowMode ? .22f + strength * .55f : charge * focus * .2f))
                              : new Color(1f,1f,1f,.07f);
        AddTrapezoid(vh, r.xMin + 1f, r.yMax - h, r.yMax,
            r.xMax - 1f, r.xMax - slant - 1f, highlight, highlight);
    }

    static void AddTrapezoid(VertexHelper vh, float x0, float y0, float y1,
        float rightTop, float rightBottom, Color upper, Color lower)
    {
        int k = vh.currentVertCount;
        AddVertex(vh, new Vector2(x0,y0), lower);
        AddVertex(vh, new Vector2(x0,y1), upper);
        AddVertex(vh, new Vector2(rightTop,y1), upper);
        AddVertex(vh, new Vector2(rightBottom,y0), lower);
        vh.AddTriangle(k,k+1,k+2);
        vh.AddTriangle(k,k+2,k+3);
    }

    static void AddVertex(VertexHelper vh, Vector2 p, Color c)
    {
        UIVertex v = UIVertex.simpleVert;
        v.position = p;
        v.color = c;
        vh.AddVert(v);
    }
}
