using System;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

// Works with an EXISTING gauge background under Main Camera/UICamera/GameUI.
// No Needle child is required. It creates independent frontmost UI elements.
public static class CockpitGaugeSplitFrontInstaller
{
    const string Menu = "Tools/Cockpit Gauge/Setup Separate Front Needle + Digital (Selected Background)";

    [MenuItem(Menu)]
    public static void Install()
    {
        GameObject selection = Selection.activeGameObject;
        if (!selection)
        {
            EditorUtility.DisplayDialog("Cockpit Gauge",
                "Select the background CockpitGauge under GameUI first.", "OK");
            return;
        }

        CockpitGauge gauge = selection.GetComponent<CockpitGauge>();
        if (!gauge) gauge = selection.GetComponentInParent<CockpitGauge>();
        if (!gauge)
        {
            // Supports GameUI > CockpitGauge which is currently only a dial Image.
            gauge = Undo.AddComponent<CockpitGauge>(selection);
        }

        RectTransform face = null;
        Transform dialChild = gauge.transform.Find("Dial");
        if (dialChild) face = dialChild.GetComponent<RectTransform>();
        if (!face) face = gauge.GetComponent<RectTransform>();
        if (!face)
        {
            EditorUtility.DisplayDialog("Cockpit Gauge",
                "The selected background needs a RectTransform (UI Image).", "OK");
            return;
        }

        Sprite needleSprite = LoadSprite("GaugeNeedle.png");
        if (!needleSprite) needleSprite = LoadSprite("FrontNeedle_Default.png");
        Sprite panelSprite = LoadSprite("GaugeDigitalPanel.png");
        if (!panelSprite) panelSprite = LoadSprite("FrontDigitalPanel_Default.png");
        if (!needleSprite)
        {
            EditorUtility.DisplayDialog("Cockpit Gauge",
                "Needle sprite not found. Import the whole CockpitGauge_SplitFront folder, then try again.", "OK");
            return;
        }

        // If the previous v4 overlay is present for this gauge, disable it
        // to prevent two pointers and two overlapping sets of invisible touches.
        foreach (Canvas existingCanvas in UnityEngine.Object.FindObjectsOfType<Canvas>(true))
        {
            if (!existingCanvas || !existingCanvas.gameObject.activeSelf) continue;
            string objectName = existingCanvas.gameObject.name;
            if (objectName == gauge.name + "_FrontmostLayer" ||
                objectName == gauge.name + "_SplitFront")
            {
                Undo.RecordObject(existingCanvas.gameObject, "Disable previous gauge overlay");
                existingCanvas.gameObject.SetActive(false);
            }
        }

        GameObject canvasObject = new GameObject(
            gauge.name + "_SplitFront", typeof(RectTransform), typeof(Canvas),
            typeof(CanvasScaler), typeof(GraphicRaycaster), typeof(CockpitGaugeScreenRig));
        Undo.RegisterCreatedObjectUndo(canvasObject, "Create gauge split-front overlay");
        Canvas canvas = canvasObject.GetComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvas.overrideSorting = true;
        canvas.sortingOrder = 32700;
        canvas.pixelPerfect = false;
        canvasObject.GetComponent<CanvasScaler>().uiScaleMode =
            CanvasScaler.ScaleMode.ConstantPixelSize;
        GraphicRaycaster raycaster = canvasObject.GetComponent<GraphicRaycaster>();
        raycaster.blockingObjects = GraphicRaycaster.BlockingObjects.None;

        RectTransform frontFace = MakeRect("VisualCenter_FollowsClock", canvasObject.transform);
        frontFace.anchorMin = frontFace.anchorMax = new Vector2(.5f, .5f);
        frontFace.sizeDelta = new Vector2(360f, 360f);

        RectTransform needle = MakeFullImage("Needle_Front", frontFace, needleSprite);
        Image panel = MakeFullImage("DigitalPanel_Front", frontFace, panelSprite).GetComponent<Image>();
        if (!panelSprite)
        {
            // Fallback if panel texture was deleted from the project.
            RectTransform p = panel.rectTransform;
            p.anchorMin = new Vector2(.38f, .32f);
            p.anchorMax = new Vector2(.62f, .41f);
            p.offsetMin = p.offsetMax = Vector2.zero;
            panel.color = new Color(.025f, .055f, .085f, .94f);
        }

        GameObject textObject = new GameObject("DigitalReading_Front",
            typeof(RectTransform), typeof(Text));
        textObject.transform.SetParent(frontFace, false);
        RectTransform textRect = textObject.GetComponent<RectTransform>();
        // Readout at y = 653 in a 1024px-wide face (lower than needle pivot).
        textRect.anchorMin = new Vector2(.38f, .326f);
        textRect.anchorMax = new Vector2(.62f, .414f);
        textRect.offsetMin = textRect.offsetMax = Vector2.zero;
        Text reading = textObject.GetComponent<Text>();
        reading.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
        reading.fontSize = 30;
        reading.fontStyle = FontStyle.Bold;
        reading.alignment = TextAnchor.MiddleCenter;
        reading.resizeTextForBestFit = true;
        reading.resizeTextMinSize = 8;
        reading.resizeTextMaxSize = 42;
        reading.raycastTarget = false;

        // The touch area intentionally stays unrotated, so left/right are
        // SCREEN left/right, even when the physical clock background rotates.
        RectTransform touchSurface = MakeRect("TapSurface_AlwaysFront", canvasObject.transform);
        touchSurface.anchorMin = touchSurface.anchorMax = new Vector2(.5f, .5f);
        touchSurface.sizeDelta = new Vector2(360f, 360f);
        MakeTapHalf("LeftTap_-8", touchSurface, gauge, false);
        MakeTapHalf("RightTap_+8", touchSurface, gauge, true);

        // Old world-space visuals are not necessary once the overlay is present.
        // The background itself remains in GameUI and can still be occluded.
        Transform priorNeedle = gauge.transform.Find("Needle");
        if (priorNeedle)
        {
            Image old = priorNeedle.GetComponent<Image>();
            if (old)
            {
                Undo.RecordObject(old, "Hide world needle");
                old.enabled = false;
            }
        }
        Transform priorPanel = gauge.transform.Find("Digital Panel");
        if (priorPanel)
        {
            Image old = priorPanel.GetComponent<Image>();
            if (old)
            {
                Undo.RecordObject(old, "Hide world digital panel");
                old.enabled = false;
            }
        }
        Transform priorReading = gauge.transform.Find("Digital Reading");
        if (priorReading)
        {
            Text old = priorReading.GetComponent<Text>();
            if (old)
            {
                Undo.RecordObject(old, "Hide world digital reading");
                old.enabled = false;
            }
        }

        canvasObject.GetComponent<CockpitGaugeScreenRig>().Setup(
            gauge, face, canvas, frontFace, touchSurface, needle, reading);
        EnsureEventSystem();

        Selection.activeGameObject = canvasObject;
        EditorGUIUtility.PingObject(canvasObject);
        EditorUtility.DisplayDialog("Cockpit Gauge",
            "Done. The background is unchanged in GameUI. The independent pointer, " +
            "digital panel, reading, and +/-8 touch zones are now on a " +
            "frontmost Screen Space Overlay Canvas. No child Needle is required.", "OK");
    }

    static RectTransform MakeRect(string name, Transform parent)
    {
        GameObject go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        RectTransform rect = go.GetComponent<RectTransform>();
        rect.pivot = new Vector2(.5f, .5f);
        return rect;
    }

    static RectTransform MakeFullImage(string name, Transform parent, Sprite sprite)
    {
        GameObject go = new GameObject(name, typeof(RectTransform), typeof(Image));
        go.transform.SetParent(parent, false);
        RectTransform rect = go.GetComponent<RectTransform>();
        rect.anchorMin = Vector2.zero;
        rect.anchorMax = Vector2.one;
        rect.offsetMin = rect.offsetMax = Vector2.zero;
        rect.pivot = new Vector2(.5f, .5f);
        Image image = go.GetComponent<Image>();
        image.sprite = sprite;
        image.preserveAspect = true;
        image.raycastTarget = false;
        return rect;
    }

    static void MakeTapHalf(string name, RectTransform frame, CockpitGauge gauge, bool increase)
    {
        GameObject go = new GameObject(name, typeof(RectTransform),
            typeof(Image), typeof(CockpitGaugeScreenTap));
        go.transform.SetParent(frame, false);
        RectTransform rect = go.GetComponent<RectTransform>();
        rect.anchorMin = increase ? new Vector2(.5f, 0f) : Vector2.zero;
        rect.anchorMax = increase ? Vector2.one : new Vector2(.5f, 1f);
        rect.offsetMin = rect.offsetMax = Vector2.zero;
        Image image = go.GetComponent<Image>();
        image.color = Color.clear;
        image.raycastTarget = true;
        go.GetComponent<CockpitGaugeScreenTap>().Setup(gauge, frame, increase);
    }

    static Sprite LoadSprite(string filename)
    {
        string name = System.IO.Path.GetFileNameWithoutExtension(filename);
        string[] guids = AssetDatabase.FindAssets(name);
        foreach (string guid in guids)
        {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            if (!path.EndsWith("/" + filename, StringComparison.OrdinalIgnoreCase)) continue;
            TextureImporter importer = AssetImporter.GetAtPath(path) as TextureImporter;
            if (importer && importer.textureType != TextureImporterType.Sprite)
            {
                importer.textureType = TextureImporterType.Sprite;
                importer.spriteImportMode = SpriteImportMode.Single;
                importer.alphaIsTransparency = true;
                importer.mipmapEnabled = false;
                importer.SaveAndReimport();
            }
            return AssetDatabase.LoadAssetAtPath<Sprite>(path);
        }
        return null;
    }

    static void EnsureEventSystem()
    {
        if (UnityEngine.Object.FindObjectOfType<EventSystem>()) return;
        GameObject go = new GameObject("EventSystem", typeof(EventSystem));
#if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER
        Type input = Type.GetType("UnityEngine.InputSystem.UI.InputSystemUIInputModule, Unity.InputSystem");
        if (input != null) go.AddComponent(input);
        else go.AddComponent<StandaloneInputModule>();
#else
        go.AddComponent<StandaloneInputModule>();
#endif
        Undo.RegisterCreatedObjectUndo(go, "Create EventSystem");
    }
}
