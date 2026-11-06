using UnityEditor;
using UnityEngine;

public static class AssetDependencyChecker
{
    [MenuItem("Tools/Assets/Show Dependencies")]
    static void ShowDependencies()
    {
        Object selected = Selection.activeObject;
        string path = AssetDatabase.GetAssetPath(selected);

        if (string.IsNullOrEmpty(path))
        {
            Debug.LogWarning("Projectからファイルを選択してください。");
            return;
        }

        string[] dependencies =
            AssetDatabase.GetDependencies(path, true);

        Debug.Log(
            "対象: " + path +
            "\n参照ファイル数: " + dependencies.Length +
            "\n" + string.Join("\n", dependencies));
        Debug.Log("");
    }
}