using UnityEngine;
using UnityEngine.UI;

public class PersonalMaterial : MonoBehaviour
{
    [Header("このマスで表示するボールMaterial")]
    public Material BallMaterial;

    [Header("自動取得されるので通常は空でもOK")]
    public RawImage rawImage;

    [HideInInspector]
    public Texture2D generatedTexture;

    private void Awake()
    {
        FindRawImage();
    }

    private void OnValidate()
    {
        FindRawImage();
    }

    private void FindRawImage()
    {
        if (rawImage == null)
            rawImage = GetComponentInChildren<RawImage>(true);
    }

    public void SetThumbnail(Texture2D texture)
    {
        if (generatedTexture != null)
        {
            if (Application.isPlaying)
                Destroy(generatedTexture);
            else
                DestroyImmediate(generatedTexture);
        }

        generatedTexture = texture;

        if (rawImage != null)
            rawImage.texture = generatedTexture;
    }
}