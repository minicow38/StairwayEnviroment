using System.Collections;
using UnityEngine;
using TMPro;

public class SpherePreviewManager : MonoBehaviour
{
    [Header("撮影用")]
    public Camera previewCamera;

    [Header("撮影するSphere")]
    public Renderer previewSphereRenderer;

    [Header("撮影先")]
    public RenderTexture renderTexture;

    [Header("BlockInBallが入っているContent")]
    public Transform content;

    public static bool ConvertedCoin = false;
    public int commodityNumber = 0;
    
    private IEnumerator Start()
    {
        commodityNumber = 0;
        // UIなどの初期化を1フレーム待つ
        yield return null;
        GameObject.Find("Main Camera/UICamera/CoinLimit").transform.GetChild(0).GetComponent<TextMeshProUGUI>().text =
            AndroidOneOnly.pharseCoin.ToString();

        GenerateAllThumbnails();
    }

    void FixedUpdate()
    {
        if (ConvertedCoin)
        {
            GameObject.Find("Main Camera/UICamera/CoinLimit").transform.GetChild(0)
                    .GetComponent<TextMeshProUGUI>().text =
                AndroidOneOnly.pharseCoin.ToString();
            ConvertedCoin = false;
        }
    }
    [ContextMenu("Generate All Thumbnails")]
    public void GenerateAllThumbnails()
    {
        if (previewCamera == null)
        {
            Debug.LogError("previewCamera が設定されていません。");
            return;
        }

        if (previewSphereRenderer == null)
        {
            Debug.LogError("previewSphereRenderer が設定されていません。");
            return;
        }

        if (renderTexture == null)
        {
            Debug.LogError("renderTexture が設定されていません。");
            return;
        }

        if (content == null)
        {
            Debug.LogError("content が設定されていません。");
            return;
        }

        PersonalMaterial[] items =
            content.GetComponentsInChildren<PersonalMaterial>(true);

        RenderTexture previousActive = RenderTexture.active;

        previewCamera.targetTexture = renderTexture;

        foreach (PersonalMaterial item in items)
        {
            if (item == null)
                continue;

            if (item.BallMaterial == null)
                continue;

            
            previewSphereRenderer.sharedMaterial =
                item.BallMaterial;

           
            previewCamera.Render();
            
            RenderTexture.active = renderTexture;

            Texture2D thumbnail =
                new Texture2D(
                    renderTexture.width,
                    renderTexture.height,
                    TextureFormat.RGBA32,
                    false
                );

            thumbnail.ReadPixels(
                new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);

            thumbnail.Apply();

            thumbnail.name =
                item.gameObject.name + "_Thumbnail";

            // ------------------------------------------------
            // ④ このBlockInBallだけに画像を渡す
            // ------------------------------------------------
            item.SetThumbnail(thumbnail);
          

            commodityNumber++;

        }

        RenderTexture.active = previousActive;
    }
}