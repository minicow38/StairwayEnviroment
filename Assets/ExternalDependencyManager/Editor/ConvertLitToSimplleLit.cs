using UnityEditor;
using UnityEngine;

public static class ConvertAllToBakedLit
{
    [MenuItem("Tools/Materials/Convert All To Baked Lit")]
    private static void Convert()
    {
        Shader bakedLit =
            Shader.Find("Universal Render Pipeline/Lit");

        if (bakedLit == null)
        {
            Debug.LogError("Baked Lit Shader が見つかりません。");
            return;
        }

        string[] guids =
            AssetDatabase.FindAssets("t:Material");

        int count = 0;

        foreach (string guid in guids)
        {
            string path =
                AssetDatabase.GUIDToAssetPath(guid);

            Material material =
                AssetDatabase.LoadAssetAtPath<Material>(path);

            if (material == null)
                continue;

            // すでにBaked Litなら変更しない
            if (material.shader == bakedLit)
                continue;

            Undo.RecordObject(
                material,
                "Convert All To Baked Lit");

            material.shader = bakedLit;

            EditorUtility.SetDirty(material);
            count++;
        }

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();

        Debug.Log(
            $"Baked Litへ変換完了: {count} Materials");
    }
}