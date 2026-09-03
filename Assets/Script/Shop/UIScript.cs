using UnityEngine;

public class UIScript : MonoBehaviour
{
    private const int LineCount = 8;

    // [縦/横の番号, 0=縦 1=横]
    private LineRenderer[,] lines;

    // [縦線番号, 横線番号]
    private GameObject[,] crossObjects;

    [Header("Parent")]
    [SerializeField]
    private GameObject CrossLine;

    [Header("Grid")]
    [SerializeField]
    private int sep2 = 8;

    [SerializeField]
    private float offsetX = 0f;

    [SerializeField]
    private float offsetY = 0f;

    [Header("Depth")]
    [SerializeField]
    private float distanceFromCamera = 8f;

    [Header("Line")]
    [SerializeField]
    private float lineWidth = 0.05f;

    [Header("Cross Prefab")]
    [SerializeField]
    private GameObject crossPrefab;

    [SerializeField]
    private Vector3 prefabScale = Vector3.one;

    private Camera mainCamera;
    private Material lineMaterial;


    // =========================================================
    // Start
    // =========================================================

    void Start()
    {
        mainCamera = Camera.main;

        if (mainCamera == null)
        {
            Debug.LogError("MainCamera が見つかりません。");
            return;
        }

        if (CrossLine == null)
        {
            Debug.LogError("CrossLine が設定されていません。");
            return;
        }

        if (sep2 <= 0)
        {
            sep2 = 8;
        }

        lines =
            new LineRenderer[LineCount, 2];

        crossObjects =
            new GameObject[LineCount, LineCount];

        Shader shader =
            Shader.Find(
                "Universal Render Pipeline/Unlit");

        if (shader == null)
        {
            Debug.LogError(
                "URP Unlit Shader が見つかりません。");

            return;
        }

        lineMaterial =
            new Material(shader);

        CreateLineRenderers();

        CreateCrossObjects();

        UpdateEverything();
    }


    // =========================================================
    // LateUpdate
    // =========================================================

    void LateUpdate()
    {
        UpdateEverything();
    }


    // =========================================================
    // 全体更新
    // =========================================================

    void UpdateEverything()
    {
        if (mainCamera == null)
            return;

        if (sep2 <= 0)
            return;

        UpdateLineRenderers();

        UpdateCrossObjects();
    }


    // =========================================================
    // LineRenderer生成
    // =========================================================

    void CreateLineRenderers()
    {
        for (int index = 0;
             index < LineCount;
             index++)
        {
            // -----------------------------------------
            // 縦線
            // -----------------------------------------

            GameObject verticalObject =
                new GameObject(
                    $"VerticalLine_{index}");

            verticalObject.transform.SetParent(
                CrossLine.transform,
                false);

            LineRenderer vertical =
                verticalObject.AddComponent<LineRenderer>();

            SetupLineRenderer(vertical);

            lines[index, 0] =
                vertical;


            // -----------------------------------------
            // 横線
            // -----------------------------------------

            GameObject horizontalObject =
                new GameObject(
                    $"HorizontalLine_{index}");

            horizontalObject.transform.SetParent(
                CrossLine.transform,
                false);

            LineRenderer horizontal =
                horizontalObject.AddComponent<LineRenderer>();

            SetupLineRenderer(horizontal);

            lines[index, 1] =
                horizontal;
        }
    }


    // =========================================================
    // LineRenderer共通設定
    // =========================================================

    void SetupLineRenderer(
        LineRenderer line)
    {
        line.positionCount = 2;

        line.startWidth =
            lineWidth;

        line.endWidth =
            lineWidth;

        line.startColor =
            Color.white;

        line.endColor =
            Color.white;

        line.material =
            lineMaterial;

        // 重要
        //
        // ScreenToWorldPoint() が返す値は
        // World座標なので true にする。
        line.useWorldSpace = true;
    }


    // =========================================================
    // Prefab生成
    // =========================================================

    void CreateCrossObjects()
    {
        if (crossPrefab == null)
        {
            Debug.LogWarning(
                "crossPrefab が設定されていません。");

            return;
        }

        for (int vertical = 0;
             vertical < LineCount;
             vertical++)
        {
            for (int horizontal = 0;
                 horizontal < LineCount;
                 horizontal++)
            {
                GameObject obj =
                    Instantiate(
                        crossPrefab);

                obj.name =
                    $"Cross_{vertical}_{horizontal}";

                // ★重要
                //
                // LineRendererはWorld座標なので、
                // PrefabもまずWorld座標オブジェクトとして扱う。
                //
                // CrossLineの子にはしない。
                obj.transform.SetParent(
                    null,
                    true);

                obj.transform.localScale =
                    prefabScale;

                crossObjects[
                    vertical,
                    horizontal] =
                    obj;
            }
        }
    }


    // =========================================================
    // LineRenderer位置更新
    // =========================================================

    void UpdateLineRenderers()
    {
        float screenWidth =
            Screen.width;

        float screenHeight =
            Screen.height;

        float depth =
            mainCamera.nearClipPlane +
            distanceFromCamera;


        for (int index = 0;
             index < LineCount;
             index++)
        {
            float rate =
                (index + 1f) /
                sep2;


            // =================================================
            // 縦線
            // =================================================

            float verticalScreenX =
                screenWidth *
                rate +
                offsetX;


            Vector3 verticalStartScreen =
                new Vector3(
                    verticalScreenX,
                    offsetY,
                    depth);


            Vector3 verticalEndScreen =
                new Vector3(
                    verticalScreenX,
                    screenHeight +
                    offsetY,
                    depth);


            Vector3 verticalStartWorld =
                mainCamera.ScreenToWorldPoint(
                    verticalStartScreen);


            Vector3 verticalEndWorld =
                mainCamera.ScreenToWorldPoint(
                    verticalEndScreen);


            lines[index, 0]
                .SetPosition(
                    0,
                    verticalStartWorld);


            lines[index, 0]
                .SetPosition(
                    1,
                    verticalEndWorld);



            // =================================================
            // 横線
            // =================================================

            float horizontalScreenY =
                screenHeight *
                rate +
                offsetY;


            Vector3 horizontalStartScreen =
                new Vector3(
                    offsetX,
                    horizontalScreenY,
                    depth);


            Vector3 horizontalEndScreen =
                new Vector3(
                    screenWidth +
                    offsetX,
                    horizontalScreenY,
                    depth);


            Vector3 horizontalStartWorld =
                mainCamera.ScreenToWorldPoint(
                    horizontalStartScreen);


            Vector3 horizontalEndWorld =
                mainCamera.ScreenToWorldPoint(
                    horizontalEndScreen);


            lines[index, 1]
                .SetPosition(
                    0,
                    horizontalStartWorld);


            lines[index, 1]
                .SetPosition(
                    1,
                    horizontalEndWorld);
        }
    }


    // =========================================================
    // 交点Prefab位置更新
    // =========================================================

    void UpdateCrossObjects()
    {
        if (crossPrefab == null)
            return;

        float screenWidth =
            Screen.width;

        float screenHeight =
            Screen.height;

        float depth =
            mainCamera.nearClipPlane +
            distanceFromCamera;


        // =====================================================
        // 縦線
        // =====================================================

        for (int vertical = 0;
             vertical < LineCount;
             vertical++)
        {
            float rateX =
                (vertical + 1f) /
                sep2;


            float screenX =
                screenWidth *
                rateX +
                offsetX;


            // =================================================
            // 横線
            // =================================================

            for (int horizontal = 0;
                 horizontal < LineCount;
                 horizontal++)
            {
                float rateY =
                    (horizontal + 1f) /
                    sep2;


                float screenY =
                    screenHeight *
                    rateY +
                    offsetY;


                // =============================================
                // ★このScreen座標は
                //
                // 縦線:
                // X = screenX
                //
                // 横線:
                // Y = screenY
                //
                // なので、この点が必ず交点。
                // =============================================

                Vector3 intersectionScreen =
                    new Vector3(
                        screenX,
                        screenY,
                        depth);


                Vector3 intersectionWorld =
                    mainCamera.ScreenToWorldPoint(
                        intersectionScreen);


                GameObject obj =
                    crossObjects[
                        vertical,
                        horizontal];


                if (obj == null)
                    continue;


                // LineRendererと同じWorld座標
                obj.transform.position =
                    intersectionWorld;


                // カメラと同じ向きにしたい場合
                obj.transform.rotation =
                    mainCamera.transform.rotation;
            }
        }
    }


    // =========================================================
    // Destroy
    // =========================================================

    void OnDestroy()
    {
        if (lineMaterial != null)
        {
            Destroy(
                lineMaterial);
        }
    }
}