using UnityEngine;

/// <summary>
/// Small, mobile-friendly approximation of the current skybox/fog tone.
/// Does not sample on-screen pixels. The particle material must honor particle vertex color.
/// Put this on the ParticleSystem object; it changes Start Color only.
/// </summary>
[DisallowMultipleComponent]
[RequireComponent(typeof(ParticleSystem))]
public sealed class SmokeSkyboxHarmony : MonoBehaviour
{
    [Header("Environment matching / 背景とのなじみ")]
    [SerializeField, Range(0f, 1f)] float skyInfluence = .65f;
    [SerializeField, Range(0f, 1f)] float smokeOpacity = .78f;
    [SerializeField, Range(0f, 1f)] float skySaturation = .7f;

    [Tooltip("Fallback tone for a teal/blue-gray stairway sky; use when the skybox has no readable color.")]
    [SerializeField] Color fallbackSkyColor = new Color(.30f, .55f, .58f, 1f);

    [Tooltip("Prefer the same color as scene fog when fog is enabled.")]
    [SerializeField] bool preferFog = true;
    [Tooltip("Read common color fields from camera/global skybox material.")]
    [SerializeField] bool useSkyboxMaterialColor = true;
    [Tooltip("Use ambient light if it has a noticeable tint and skybox has no readable color.")]
    [SerializeField] bool useAmbientColor = true;

    [Header("Optional auto update / 背景が変化する場合")]
    [SerializeField] bool refreshWhilePlaying = false;
    [SerializeField, Min(.2f)] float refreshInterval = 1f;

    ParticleSystem cachedSystem;
    float nextRefresh;

    void Awake() { Cache(); ApplyEnvironmentTone(); }
    void OnEnable() { Cache(); ApplyEnvironmentTone(); }

    void LateUpdate()
    {
        if (!refreshWhilePlaying || Time.unscaledTime < nextRefresh) return;
        nextRefresh = Time.unscaledTime + Mathf.Max(.2f, refreshInterval);
        ApplyEnvironmentTone();
    }

    void Cache()
    {
        if (!cachedSystem) cachedSystem = GetComponent<ParticleSystem>();
    }

    [ContextMenu("Apply environment tone now / 背景色を反映")]
    public void ApplyEnvironmentTone()
    {
        Cache();
        if (!cachedSystem) return;

        Color sky = GetEnvironmentColor();
        // Some skyboxes are vivid: retain a little neutral smoke character.
        float gray = sky.r * .2126f + sky.g * .7152f + sky.b * .0722f;
        Color grayColor = new Color(gray, gray, gray, 1f);
        Color naturalSky = Color.Lerp(grayColor, sky, Mathf.Clamp01(skySaturation));

        // Multiply the prefab's existing Color over Lifetime gradient; do not rebuild
        // curves, counts, velocity or shape. White = original color unchanged.
        Color tint = Color.Lerp(Color.white, naturalSky, Mathf.Clamp01(skyInfluence));
        tint.a = Mathf.Clamp01(smokeOpacity);
        var main = cachedSystem.main;
        main.startColor = tint;
    }

    Color GetEnvironmentColor()
    {
        if (preferFog && RenderSettings.fog)
            return RenderSettings.fogColor;

        if (useSkyboxMaterialColor)
        {
            Material sky = null;
            Camera cam = Camera.main;
            if (cam)
            {
                Skybox component = cam.GetComponent<Skybox>();
                if (component && component.enabled && component.material)
                    sky = component.material;
            }
            if (!sky) sky = RenderSettings.skybox;
            if (sky)
            {
                Color sampled;
                if (TryReadSkyColor(sky, out sampled))
                    return sampled;
            }
        }

        if (useAmbientColor)
        {
            Color ambient = Color.Lerp(RenderSettings.ambientEquatorColor,
                RenderSettings.ambientSkyColor, .25f);
            if (IsColorful(ambient)) return ambient;
        }

        return fallbackSkyColor;
    }

    // Property names depend on the skybox Shader. These are common conventions.
    // A procedural skybox Sky Tint is not the actual rendered sky pixel color.
    static bool TryReadSkyColor(Material sky, out Color result)
    {
        string[] names = {
            "_HorizonColor", "_MidColor", "_SkyColor", "_TopColor",
            "_SkyTint", "_Tint", "_Color"
        };
        for (int i = 0; i < names.Length; i++)
        {
            if (!sky.HasProperty(names[i])) continue;
            Color color = sky.GetColor(names[i]);
            if (IsColorful(color))
            {
                result = color;
                return true;
            }
        }
        result = Color.white;
        return false;
    }

    static bool IsColorful(Color c)
    {
        float largest = Mathf.Max(c.r, Mathf.Max(c.g, c.b));
        float smallest = Mathf.Min(c.r, Mathf.Min(c.g, c.b));
        return largest - smallest > .07f && largest > .1f;
    }
}
