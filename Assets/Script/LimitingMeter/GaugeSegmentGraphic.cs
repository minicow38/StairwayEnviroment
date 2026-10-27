using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Soft skybox-coloured ten-step gauge bar. No texture, material, or shader required.
/// The original left-aligned trapezoid is retained, but its corners and highlights
/// are softened. This class replaces the previous GaugeSegmentGraphic IN PLACE.
/// Keep the existing GaugeSegmentGraphic.cs.meta (Unity GUID) when replacing it.
/// </summary>
[DisallowMultipleComponent]
public sealed class GaugeSegmentGraphic : MaskableGraphic
{
    Color dualColor = new Color(.65f, .83f, .83f, 1f);
    Color glowColor = new Color(.86f, .95f, .94f, 1f);
    bool lit;
    bool glowMode;
    float focus;
    float pulse;
    float charge;
    float scan;

    [Header("Skybox Soft Style / 空になじむ描画")]
    [SerializeField, Range(0f, 1f)] float illuminatedOpacity = .70f;
    [SerializeField, Range(0f, 1f)] float inactiveOpacity = .26f;
    [SerializeField, Range(0f, 1f)] float outlineOpacity = .10f;
    [SerializeField, Range(0f, 1f)] float haloOpacity = .085f;
    [SerializeField, Range(0f, 16f)] float cornerRadius = 6f;
    [SerializeField] Color inactiveTint = new Color(.54f, .68f, .70f, 1f);

    // Compatibility with original scripts that used the four-argument Configure.
    public void Configure(Color normal, Color glow, bool illuminated, bool glowing)
    {
        Configure(normal, glow, illuminated, glowing, 0f, 0f, 0f, 0f);
    }

    // Compatible with the current CockpitGauge which passes focus/pulse/charge/scan.
    public void Configure(Color normal, Color glow, bool illuminated, bool glowing,
        float focused, float pulsing, float charging, float scanning)
    {
        dualColor = normal;
        glowColor = glow;
        lit = illuminated;
        glowMode = glowing;
        focus = Mathf.Clamp01(focused);
        pulse = Mathf.Clamp01(pulsing);
        charge = Mathf.Clamp01(charging);
        scan = Mathf.Clamp01(scanning);
        SetVerticesDirty();
    }

    protected override void OnPopulateMesh(VertexHelper vh)
    {
        vh.Clear();
        Rect r = rectTransform.rect;
        if (r.width < 1f || r.height < 1f) return;

        float slant = Mathf.Min(5f, r.width * .13f);
        float radius = Mathf.Min(cornerRadius, Mathf.Min(r.height * .42f, r.width * .15f));
        float alpha = lit ? illuminatedOpacity : inactiveOpacity;
        Color tint = lit ? dualColor : inactiveTint;

        if (lit && glowMode)
        {
            tint = Color.Lerp(tint, glowColor, .30f + .20f * scan);
            alpha += .07f * pulse + .10f * scan;
        }
        else if (lit)
        {
            tint = Color.Lerp(tint, glowColor, .12f * focus + .10f * charge);
            alpha += .09f * focus + .05f * charge;
        }
        alpha = Mathf.Clamp01(alpha);

        // A very faint external halo only while actively glowing.
        // No orange/yellow hard ring; the surrounding sky can show through.
        if (lit && glowMode && haloOpacity > 0f)
        {
            Color outer = glowColor;
            outer.a = haloOpacity * (.24f + .26f * scan);
            AddShape(vh, r, slant, radius + 6f, 6f, outer, outer);

            Color inner = glowColor;
            inner.a = haloOpacity * (.55f + .30f * scan);
            AddShape(vh, r, slant, radius + 3f, 3f, inner, inner);
        }

        // Muted pale rim. It adds definition against both light and dark skies.
        if (outlineOpacity > 0f)
        {
            Color edge = Color.Lerp(inactiveTint, glowColor, lit ? .50f : .15f);
            edge.a = outlineOpacity * (lit ? 1f : .72f);
            AddShape(vh, r, slant, radius + 1.1f, 1.1f, edge, edge);
        }

        Color top = Color.Lerp(tint, Color.white, lit ? .085f : .025f);
        Color bottom = Color.Lerp(tint, new Color(.33f, .49f, .55f, 1f), lit ? .055f : .035f);
        top.a = alpha;
        bottom.a = alpha * .94f;
        AddShape(vh, r, slant, radius, 0f, top, bottom);

        // Small translucent reflection rather than a high-contrast highlight.
        float stripeHeight = Mathf.Min(2f, r.height * .12f);
        Color reflection = Color.Lerp(tint, Color.white, .32f);
        reflection.a = lit ? (glowMode ? .12f : .075f) : .028f;
        Rect stripe = new Rect(r.xMin + 2f, r.yMax - stripeHeight - 1f,
            Mathf.Max(1f, r.width - 4f), stripeHeight);
        AddShape(vh, stripe, 0f, Mathf.Min(radius * .25f, stripeHeight * .5f), 0f,
            reflection, reflection);
    }

    // Rounded trapezoid tessellation: corners are quadratic curves, no textures.
    // Winding is the same as the old AddTrapezoid helper.
    static void AddShape(VertexHelper vh, Rect r, float slant,
        float radius, float outset, Color top, Color bottom)
    {
        float left = r.xMin - outset;
        float right = r.xMax + outset;
        float low = r.yMin - outset;
        float high = r.yMax + outset;

        Vector2[] vertices =
        {
            new Vector2(left, low),
            new Vector2(left, high),
            new Vector2(right, high),
            new Vector2(right - slant, low)
        };

        const int samples = 4;
        int start = vh.currentVertCount;
        Vector2 center = (vertices[0] + vertices[1] + vertices[2] + vertices[3]) * .25f;
        AddVertex(vh, center, Color.Lerp(bottom, top, .5f));

        for (int i = 0; i < 4; i++)
        {
            Vector2 prev = vertices[(i + 3) % 4];
            Vector2 corner = vertices[i];
            Vector2 next = vertices[(i + 1) % 4];
            float trim = Mathf.Min(radius, Mathf.Min(
                Vector2.Distance(corner, prev) * .36f,
                Vector2.Distance(corner, next) * .36f));

            Vector2 p0 = Vector2.MoveTowards(corner, prev, trim);
            Vector2 p2 = Vector2.MoveTowards(corner, next, trim);
            for (int j = 0; j <= samples; j++)
            {
                float t = j / (float)samples;
                Vector2 p = (1f - t) * (1f - t) * p0 +
                    2f * (1f - t) * t * corner + t * t * p2;
                float v = Mathf.Clamp01((p.y - low) / Mathf.Max(1f, high - low));
                AddVertex(vh, p, Color.Lerp(bottom, top, v));
            }
        }

        const int perimeter = 4 * (samples + 1);
        for (int i = 0; i < perimeter; i++)
        {
            vh.AddTriangle(start, start + 1 + i, start + 1 + (i + 1) % perimeter);
        }
    }

    static void AddVertex(VertexHelper vh, Vector2 point, Color tint)
    {
        UIVertex v = UIVertex.simpleVert;
        v.position = point;
        v.color = tint;
        vh.AddVert(v);
    }
}
