// UnusedAssetManager.cs  -  Unity 2022.3 / UnityEditor only
// Put this file under Assets/Editor/ . This tool never deletes automatically.
// A "candidate" is NOT proof that an asset is unused at runtime.
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using UnityEditor;
using UnityEngine;

public sealed class UnusedAssetManager : EditorWindow
{
    private sealed class Entry
    {
        public string path;
        public string category;
        public string status;
        public long bytes;
        public bool candidate;
        public bool selected;
    }

    private const int PageSize = 70;
    private readonly List<Entry> items = new List<Entry>();
    private string[] allPaths = new string[0];
    private Vector2 scroll;
    private string query = "";
    private string additionalKeepFolders = "";
    private int showMode; // 0: candidates, 1: other scenes, 2: all
    private int page;
    private bool addressablesInspectionIncomplete;
    private string scanSummary = "まだ調査していません。";

    private static readonly HashSet<string> CandidateExtensions = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
    {
        ".unitypackage", ".png", ".jpg", ".jpeg", ".tga", ".psd", ".psb", ".tif", ".tiff",
        ".bmp", ".gif", ".exr", ".hdr", ".cubemap", ".spriteatlas", ".rendertexture",
        ".fbx", ".obj", ".blend", ".dae", ".3ds", ".prefab", ".mat",
        ".wav", ".mp3", ".ogg", ".aif", ".aiff", ".mp4", ".mov", ".webm",
        ".anim", ".controller", ".overridecontroller", ".mask", ".playable",
        ".ttf", ".otf", ".asset", ".unity", ".uxml", ".uss",
        ".shadergraph", ".shadersubgraph", ".vfx", ".physicmaterial", ".physicsmaterial2d",
        ".csv", ".tsv", ".txt", ".log"
    };

    [MenuItem("Tools/Asset Cleanup/未使用素材マネージャー")]
    private static void Open()
    {
        var window = GetWindow<UnusedAssetManager>("未使用素材マネージャー");
        window.minSize = new Vector2(850, 510);
        window.Show();
    }

    private void OnGUI()
    {
        EditorGUILayout.Space(7);
        EditorGUILayout.LabelField("未使用素材マネージャー  /  UI・画像・FBX・音声・Prefab・Materialなど", EditorStyles.boldLabel);
        EditorGUILayout.HelpBox(
            "『未使用候補』はビルドSceneからの静的な参照が見つからない素材です。Resources.Load、Addressables、コードからの読み込み、" +
            "実行時生成などは完全には判定できません。削除前にバックアップを作成します。", MessageType.Warning);

        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Assets全体を調査", GUILayout.Height(28))) Scan();
        if (GUILayout.Button("CSV出力", GUILayout.Height(28), GUILayout.Width(100))) ExportCsv();
        if (GUILayout.Button("選択解除", GUILayout.Height(28), GUILayout.Width(100)))
        {
            foreach (var e in items) e.selected = false;
        }
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.LabelField("削除対象から除外するフォルダー（任意 / 1行につき1パス / サブフォルダーも保護）");
        additionalKeepFolders = EditorGUILayout.TextArea(additionalKeepFolders, GUILayout.MinHeight(39));
        EditorGUILayout.LabelField("例: Assets/MyDynamicPrefabs  ※変更したら再調査", EditorStyles.miniLabel);
        EditorGUILayout.Space(4);

        EditorGUILayout.BeginHorizontal();
        showMode = GUILayout.Toolbar(showMode, new[] { "シーン未参照候補", "他Sceneだけで使用", "全素材（使用中も表示）" }, GUILayout.Width(570));
        GUILayout.FlexibleSpace();
        GUILayout.Label("検索:", GUILayout.Width(39));
        string nextQuery = GUILayout.TextField(query, GUILayout.MinWidth(150));
        if (nextQuery != query) { query = nextQuery; page = 0; }
        EditorGUILayout.EndHorizontal();

        if (addressablesInspectionIncomplete)
        {
            EditorGUILayout.HelpBox("Addressablesの参照一覧を完全に取得できませんでした。Addressables用素材がある場合、" +
                                    "上欄にそのフォルダーを登録して再調査してください。", MessageType.Error);
        }
        EditorGUILayout.LabelField(scanSummary, EditorStyles.miniLabel);

        var visible = Filtered();
        int pages = Mathf.Max(1, (visible.Count + PageSize - 1) / PageSize);
        page = Mathf.Clamp(page, 0, pages - 1);
        int first = page * PageSize;
        int last = Mathf.Min(first + PageSize, visible.Count);
        int selectedCount = items.Count(x => x.selected);
        long selectedBytes = items.Where(x => x.selected).Sum(x => x.bytes);

        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("このページを全選択", GUILayout.Width(136)))
            for (int i = first; i < last; ++i) if (visible[i].candidate) visible[i].selected = true;
        if (GUILayout.Button("表示中の候補をすべて選択", GUILayout.Width(178)))
        {
            if (EditorUtility.DisplayDialog("一括選択の確認",
                "参照がないと判定されても実行時に使用されている可能性があります。表示中の候補を選択しますか？", "選択する", "キャンセル"))
                foreach (var entry in visible) if (entry.candidate) entry.selected = true;
        }
        GUILayout.FlexibleSpace();
        GUI.enabled = page > 0;
        if (GUILayout.Button("◀", GUILayout.Width(35))) { --page; scroll = Vector2.zero; }
        GUI.enabled = true;
        GUILayout.Label((page + 1) + " / " + pages + "  (" + visible.Count + " 件)", GUILayout.Width(125));
        GUI.enabled = page + 1 < pages;
        if (GUILayout.Button("▶", GUILayout.Width(35))) { ++page; scroll = Vector2.zero; }
        GUI.enabled = true;
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal(EditorStyles.toolbar);
        GUILayout.Label("選択", GUILayout.Width(40));
        GUILayout.Label("容量", GUILayout.Width(78));
        GUILayout.Label("判定", GUILayout.Width(122));
        GUILayout.Label("種類", GUILayout.Width(100));
        GUILayout.Label("ファイル名（クリックで位置表示）", GUILayout.Width(290));
        GUILayout.Label("パス", GUILayout.MinWidth(230));
        EditorGUILayout.EndHorizontal();

        scroll = EditorGUILayout.BeginScrollView(scroll);
        for (int i = first; i < last; ++i)
        {
            var e = visible[i];
            EditorGUILayout.BeginHorizontal();
            using (new EditorGUI.DisabledScope(!e.candidate))
                e.selected = GUILayout.Toggle(e.selected, "", GUILayout.Width(40));
            GUILayout.Label(SizeText(e.bytes), GUILayout.Width(78));
            GUILayout.Label(e.status, GUILayout.Width(122));
            GUILayout.Label(e.category, GUILayout.Width(100));
            if (GUILayout.Button(Path.GetFileName(e.path), EditorStyles.linkLabel, GUILayout.Width(290)))
            {
                var obj = AssetDatabase.LoadMainAssetAtPath(e.path);
                if (obj != null) { Selection.activeObject = obj; EditorGUIUtility.PingObject(obj); }
            }
            GUILayout.Label(e.path, EditorStyles.miniLabel, GUILayout.MinWidth(230));
            EditorGUILayout.EndHorizontal();
        }
        EditorGUILayout.EndScrollView();

        EditorGUILayout.BeginHorizontal(EditorStyles.helpBox);
        GUILayout.Label("選択: " + selectedCount + " 件  /  元ファイル合計: " + SizeText(selectedBytes), EditorStyles.boldLabel);
        GUILayout.FlexibleSpace();
        using (new EditorGUI.DisabledScope(selectedCount == 0))
        {
            if (GUILayout.Button("選択をバックアップしてごみ箱へ", GUILayout.Width(230), GUILayout.Height(30)))
                BackupAndMoveSelectedToTrash();
        }
        EditorGUILayout.EndHorizontal();
        EditorGUILayout.LabelField("※バックアップはAssets外へ保存。ごみ箱に移らないバージョン管理環境もあるため、バックアップを必須にしています。", EditorStyles.miniLabel);
    }

    private List<Entry> Filtered()
    {
        IEnumerable<Entry> selected = items;
        if (showMode == 0) selected = selected.Where(x => x.status == "Scene参照なし");
        else if (showMode == 1) selected = selected.Where(x => x.status == "他Sceneのみ");
        if (!string.IsNullOrWhiteSpace(query))
            selected = selected.Where(x => x.path.IndexOf(query.Trim(), StringComparison.OrdinalIgnoreCase) >= 0
                                        || x.category.IndexOf(query.Trim(), StringComparison.OrdinalIgnoreCase) >= 0);
        return selected.OrderByDescending(x => x.bytes).ThenBy(x => x.path, StringComparer.OrdinalIgnoreCase).ToList();
    }

    private void Scan()
    {
        var enabledScenes = EditorBuildSettings.scenes
            .Where(x => x.enabled && !string.IsNullOrEmpty(x.path) && x.path.StartsWith("Assets/", StringComparison.Ordinal) && File.Exists(AbsolutePath(x.path)))
            .Select(x => x.path.Replace('\\', '/')).Distinct().ToArray();
        if (enabledScenes.Length == 0)
        {
            EditorUtility.DisplayDialog("調査を中止しました", "Build SettingsのScenes In Buildに使用Sceneが1つもありません。" +
                "正しいSceneを追加・有効にしてから実行してください。", "OK");
            return;
        }

        try
        {
            EditorUtility.DisplayProgressBar("未使用素材を調査", "AssetDatabaseから一覧を取得中...", 0.04f);
            allPaths = AssetDatabase.GetAllAssetPaths()
                .Where(p => p.StartsWith("Assets/", StringComparison.Ordinal) && !AssetDatabase.IsValidFolder(p))
                .Distinct(StringComparer.OrdinalIgnoreCase).ToArray();
            string[] otherScenes = allPaths.Where(p => p.EndsWith(".unity", StringComparison.OrdinalIgnoreCase)).ToArray();

            EditorUtility.DisplayProgressBar("未使用素材を調査", "ビルドSceneの参照を確認中...", 0.15f);
            var usedBuild = new HashSet<string>(AssetDatabase.GetDependencies(enabledScenes, true), StringComparer.OrdinalIgnoreCase);
            EditorUtility.DisplayProgressBar("未使用素材を調査", "全Sceneの参照を確認中...", 0.31f);
            var allSceneUse = new HashSet<string>(AssetDatabase.GetDependencies(otherScenes, true), StringComparer.OrdinalIgnoreCase);
            EditorUtility.DisplayProgressBar("未使用素材を調査", "動的読み込み系の保護を準備中...", 0.48f);
            var keepRoots = CollectKeepRoots(allPaths);
            var usedProtected = keepRoots.Count == 0
                ? new HashSet<string>(StringComparer.OrdinalIgnoreCase)
                : new HashSet<string>(AssetDatabase.GetDependencies(keepRoots.ToArray(), true), StringComparer.OrdinalIgnoreCase);

            var results = new List<Entry>();
            int count = 0;
            foreach (string p in allPaths)
            {
                if (!CandidateExtensions.Contains(Path.GetExtension(p))) continue;
                if ((++count % 180) == 0)
                    EditorUtility.DisplayProgressBar("未使用素材を調査", "容量と使用状況を照合中...", 0.55f + Mathf.Min(0.43f, (float)count / allPaths.Length * 0.43f));
                bool isBuild = usedBuild.Contains(p);
                bool isProtected = usedProtected.Contains(p) || IsProtectedPath(p);
                bool inOtherScene = allSceneUse.Contains(p);
                string status = isBuild ? "ビルドScene使用" : isProtected ? "保護対象" : inOtherScene ? "他Sceneのみ" : "Scene参照なし";
                string abs = AbsolutePath(p);
                long bytes = File.Exists(abs) ? new FileInfo(abs).Length : 0;
                results.Add(new Entry
                {
                    path = p,
                    category = Category(p),
                    status = status,
                    candidate = !isBuild && !isProtected,
                    bytes = bytes
                });
            }
            items.Clear();
            items.AddRange(results);
            page = 0;
            scroll = Vector2.zero;
            scanSummary = "調査完了: " + items.Count + " 件 / Scene未参照 " + items.Count(x => x.status == "Scene参照なし") +
                          " 件 / 他Sceneのみ " + items.Count(x => x.status == "他Sceneのみ") +
                          " 件 / 保護・使用中 " + items.Count(x => !x.candidate) + " 件";
            Repaint();
        }
        catch (Exception ex)
        {
            Debug.LogException(ex);
            EditorUtility.DisplayDialog("調査エラー", "Consoleを確認してください。\n" + ex.Message, "OK");
        }
        finally { EditorUtility.ClearProgressBar(); }
    }

    private HashSet<string> CollectKeepRoots(IEnumerable<string> paths)
    {
        var roots = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        foreach (string p in paths)
            if (IsProtectedPath(p)) roots.Add(p);

        // Directly assigned AssetBundle assets: preserve these and their dependencies.
        foreach (string bundleName in AssetDatabase.GetAllAssetBundleNames())
            foreach (string p in AssetDatabase.GetAssetPathsFromAssetBundle(bundleName))
                AddRootOrFolderContents(p, roots);

        addressablesInspectionIncomplete = false;
        TryAddAddressables(roots);
        return roots;
    }

    private void TryAddAddressables(HashSet<string> roots)
    {
        Type defaultType = null;
        foreach (Assembly asm in AppDomain.CurrentDomain.GetAssemblies())
        {
            defaultType = asm.GetType("UnityEditor.AddressableAssets.Settings.AddressableAssetSettingsDefaultObject", false);
            if (defaultType != null) break;
        }
        if (defaultType == null) return; // Addressables package is absent.
        try
        {
            object settings = ReadProperty(defaultType, null, "Settings");
            if (settings == null) { addressablesInspectionIncomplete = true; return; }
            IEnumerable groups = ReadProperty(settings.GetType(), settings, "groups") as IEnumerable;
            if (groups == null) { addressablesInspectionIncomplete = true; return; }
            foreach (object group in groups)
            {
                if (group == null) continue;
                IEnumerable entries = ReadProperty(group.GetType(), group, "entries") as IEnumerable;
                if (entries == null) { addressablesInspectionIncomplete = true; continue; }
                foreach (object entry in entries)
                {
                    if (entry == null) continue;
                    string guid = ReadProperty(entry.GetType(), entry, "guid") as string;
                    if (string.IsNullOrEmpty(guid)) { addressablesInspectionIncomplete = true; continue; }
                    AddRootOrFolderContents(AssetDatabase.GUIDToAssetPath(guid), roots);
                }
            }
        }
        catch (Exception ex)
        {
            addressablesInspectionIncomplete = true;
            Debug.LogWarning("[UnusedAssetManager] Addressables照合に失敗しました: " + ex.Message);
        }
    }

    private static object ReadProperty(Type type, object instance, string name)
    {
        var flags = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static;
        var prop = type.GetProperty(name, flags);
        if (prop != null) return prop.GetValue(instance, null);
        var field = type.GetField(name, flags);
        return field == null ? null : field.GetValue(instance);
    }

    private static void AddRootOrFolderContents(string path, HashSet<string> roots)
    {
        if (string.IsNullOrEmpty(path) || !path.StartsWith("Assets/", StringComparison.Ordinal)) return;
        if (AssetDatabase.IsValidFolder(path))
        {
            foreach (var guid in AssetDatabase.FindAssets("", new[] { path }))
            {
                string child = AssetDatabase.GUIDToAssetPath(guid);
                if (!AssetDatabase.IsValidFolder(child)) roots.Add(child);
            }
        }
        else roots.Add(path);
    }

    private bool IsProtectedPath(string path)
    {
        string[] segments = path.Split('/');
        // Common runtime/Editor dynamic-load paths. Code/config assets are never in the candidate extensions list.
        foreach (var segment in segments)
            if (segment.Equals("Resources", StringComparison.OrdinalIgnoreCase) ||
                segment.Equals("StreamingAssets", StringComparison.OrdinalIgnoreCase) ||
                segment.Equals("Editor", StringComparison.OrdinalIgnoreCase) ||
                segment.Equals("Editor Default Resources", StringComparison.OrdinalIgnoreCase) ||
                segment.Equals("Plugins", StringComparison.OrdinalIgnoreCase) ||
                segment.Equals("Gizmos", StringComparison.OrdinalIgnoreCase) ||
                segment.Equals("AddressableAssetsData", StringComparison.OrdinalIgnoreCase)) return true;
        foreach (string line in additionalKeepFolders.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries))
        {
            string folder = line.Trim().TrimEnd('/').Replace('\\', '/');
            if (folder.StartsWith("Assets/", StringComparison.OrdinalIgnoreCase) &&
                (path.Equals(folder, StringComparison.OrdinalIgnoreCase) ||
                 path.StartsWith(folder + "/", StringComparison.OrdinalIgnoreCase))) return true;
        }
        return false;
    }

    private static string Category(string path)
    {
        switch (Path.GetExtension(path).ToLowerInvariant())
        {
            case ".unitypackage": return "導入パッケージ";
            case ".png": case ".jpg": case ".jpeg": case ".tga": case ".psd": case ".psb":
            case ".tif": case ".tiff": case ".bmp": case ".gif": case ".exr": case ".hdr":
            case ".cubemap": case ".spriteatlas": case ".rendertexture": return "画像/テクスチャ";
            case ".fbx": case ".obj": case ".blend": case ".dae": case ".3ds": return "3Dモデル";
            case ".prefab": return "Prefab";
            case ".mat": case ".physicmaterial": case ".physicsmaterial2d": return "Material";
            case ".wav": case ".mp3": case ".ogg": case ".aif": case ".aiff": return "音声";
            case ".mp4": case ".mov": case ".webm": return "動画";
            case ".anim": case ".controller": case ".overridecontroller": case ".mask":
            case ".playable": return "アニメーション";
            case ".ttf": case ".otf": return "フォント";
            case ".unity": return "Scene";
            case ".asset": return "Asset/設定";
            case ".csv": case ".tsv": case ".txt": case ".log": return "データ/ログ";
            default: return "その他";
        }
    }

    private void ExportCsv()
    {
        if (items.Count == 0) { EditorUtility.DisplayDialog("未調査", "先にAssets全体を調査してください。", "OK"); return; }
        string file = EditorUtility.SaveFilePanel("CSV保存", Environment.GetFolderPath(Environment.SpecialFolder.Desktop),
                                                  "UnusedAssets.csv", "csv");
        if (string.IsNullOrEmpty(file)) return;
        try
        {
            using (var writer = new StreamWriter(file, false, new UTF8Encoding(true)))
            {
                writer.WriteLine("選択,判定,種類,サイズMB,パス");
                foreach (var e in items.OrderByDescending(x => x.bytes))
                    writer.WriteLine((e.selected ? "Yes" : "No") + "," + Csv(e.status) + "," +
                                     Csv(e.category) + "," + (e.bytes / 1048576.0).ToString("F3", System.Globalization.CultureInfo.InvariantCulture) +
                                     "," + Csv(e.path));
            }
            EditorUtility.RevealInFinder(file);
        }
        catch (Exception ex) { EditorUtility.DisplayDialog("CSVエラー", ex.Message, "OK"); }
    }


    private static bool CheckDependenciesFromRemainingAssets(List<Entry> selected)
    {
        var selectedSet = new HashSet<string>(selected.Select(e => e.path), StringComparer.OrdinalIgnoreCase);
        try
        {
            EditorUtility.DisplayProgressBar("参照の最終確認", "削除しないPrefabやシーンからの参照を調査中...", 0.1f);
            // One batched dependency call, rather than a GetDependencies call for every texture.
            var remaining = AssetDatabase.GetAllAssetPaths()
                .Where(p => p.StartsWith("Assets/", StringComparison.Ordinal) &&
                            !AssetDatabase.IsValidFolder(p) && !selectedSet.Contains(p) &&
                            CanReferenceOtherAssets(p)).ToArray();
            var dependencies = remaining.Length == 0 ? new HashSet<string>(StringComparer.OrdinalIgnoreCase)
                : new HashSet<string>(AssetDatabase.GetDependencies(remaining, true), StringComparer.OrdinalIgnoreCase);
            var stillReferenced = selected.Where(e => dependencies.Contains(e.path)).Select(e => e.path).ToArray();
            if (stillReferenced.Length == 0) return true;
            EditorUtility.DisplayDialog("まだ参照しているファイルがあります",
                "選択した素材を、削除対象に含まれないPrefab・Scene・Assetなどが参照しています。" +
                "参照元を残して素材だけ削除すると破損するため、中止しました。\n\n" +
                string.Join("\n", stillReferenced.Take(12).ToArray()) +
                (stillReferenced.Length > 12 ? "\n...ほか " + (stillReferenced.Length - 12) + " 件" : "") +
                "\n\n参照元も不要なら、そちらも選択してから削除してください。", "OK");
            return false;
        }
        catch (Exception ex)
        {
            EditorUtility.DisplayDialog("安全確認に失敗", "削除は実行していません。\n" + ex.Message, "OK");
            return false;
        }
        finally { EditorUtility.ClearProgressBar(); }
    }

    private static bool CanReferenceOtherAssets(string path)
    {
        string ext = Path.GetExtension(path).ToLowerInvariant();
        // Code-based loads cannot be resolved by GetDependencies and require manual protection.
        switch (ext)
        {
            case ".unity": case ".prefab": case ".asset": case ".mat":
            case ".anim": case ".controller": case ".overridecontroller": case ".playable":
            case ".uxml": case ".uss": case ".spriteatlas": case ".rendertexture":
            case ".shadergraph": case ".shadersubgraph": case ".vfx":
            case ".fbx": case ".obj": case ".blend": case ".psd": case ".psb":
            case ".physicmaterial": case ".physicsmaterial2d": case ".asmdef":
                return true;
            default:
                return false;
        }
    }

    private void BackupAndMoveSelectedToTrash()
    {
        var selected = items.Where(x => x.selected && x.candidate).ToList();
        if (selected.Count == 0) return;
        // Also protect serialized references from assets that the user is NOT deleting.
        // This prevents deleting a texture while leaving its referring prefab behind.
        if (!CheckDependenciesFromRemainingAssets(selected)) return;
        if (!EditorUtility.DisplayDialog("削除前の確認（1/2）",
             selected.Count + " 件（元ファイル " + SizeText(selected.Sum(x => x.bytes)) + "）を対象にします。\n" +
             "Scene未参照でも動的読み込みがある場合は使用中です。必ず選択内容を確認してください。\n" +
             "まず全ファイルと.metaをAssets外へバックアップし、成功した場合のみ削除します。",
             "バックアップ先を選ぶ", "キャンセル")) return;

        string backupBase = EditorUtility.OpenFolderPanel("バックアップ保存先（Assetsの外）",
            Environment.GetFolderPath(Environment.SpecialFolder.Desktop), "");
        if (string.IsNullOrEmpty(backupBase)) return;
        string projectRoot = Path.GetFullPath(Directory.GetParent(Application.dataPath).FullName);
        string backupFull = Path.GetFullPath(backupBase);
        if (backupFull.TrimEnd(Path.DirectorySeparatorChar).Equals(projectRoot.TrimEnd(Path.DirectorySeparatorChar), StringComparison.OrdinalIgnoreCase) ||
            backupFull.StartsWith(projectRoot.TrimEnd(Path.DirectorySeparatorChar) + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase))
        {
            EditorUtility.DisplayDialog("場所を変更してください", "バックアップはUnityプロジェクトの外へ作成してください。", "OK");
            return;
        }
        if (!EditorUtility.DisplayDialog("削除前の確認（2/2）",
              "バックアップ後に " + selected.Count + " 件をごみ箱へ移動します。\n" +
              "他Scene専用素材を消した場合、そのSceneやPrefabの参照は壊れます。\n\n" +
              "本当に続行しますか？", "バックアップして削除", "やめる")) return;

        string destRoot = Path.Combine(backupFull, "UnityUnusedAssets_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + "_" + Guid.NewGuid().ToString("N").Substring(0, 6));
        int copied = 0;
        try
        {
            Directory.CreateDirectory(destRoot);
            foreach (var entry in selected)
            {
                if (EditorUtility.DisplayCancelableProgressBar("バックアップ作成", entry.path, (float)copied / selected.Count))
                    throw new OperationCanceledException("バックアップが中断されました。素材は削除していません。");
                string src = AbsolutePath(entry.path);
                if (!File.Exists(src)) throw new IOException("ファイルが見つかりません: " + entry.path);
                string dest = Path.Combine(destRoot, entry.path.Replace('/', Path.DirectorySeparatorChar));
                Directory.CreateDirectory(Path.GetDirectoryName(dest));
                File.Copy(src, dest, true);
                if (File.Exists(src + ".meta")) File.Copy(src + ".meta", dest + ".meta", true);
                ++copied;
            }
        }
        catch (Exception ex)
        {
            EditorUtility.ClearProgressBar();
            EditorUtility.DisplayDialog("削除中止", ex.Message + "\n既存アセットはそのままです。\nバックアップ場所: " + destRoot, "OK");
            return;
        }
        finally { EditorUtility.ClearProgressBar(); }

        // Re-check the protected roots right before deletion, in case the project has changed since scanning.
        try
        {
            string[] enabledScenes = EditorBuildSettings.scenes.Where(s => s.enabled && !string.IsNullOrEmpty(s.path) && s.path.StartsWith("Assets/", StringComparison.Ordinal) && File.Exists(AbsolutePath(s.path))).Select(s => s.path).ToArray();
            if (enabledScenes.Length == 0) throw new InvalidOperationException("Scenes In Buildが空になりました。削除を中止します。");
            var used = new HashSet<string>(AssetDatabase.GetDependencies(enabledScenes, true), StringComparer.OrdinalIgnoreCase);
            var protectedRoots = CollectKeepRoots(AssetDatabase.GetAllAssetPaths().Where(p => p.StartsWith("Assets/", StringComparison.Ordinal) && !AssetDatabase.IsValidFolder(p)));
            var protectedDeps = protectedRoots.Count == 0 ? new HashSet<string>(StringComparer.OrdinalIgnoreCase)
                : new HashSet<string>(AssetDatabase.GetDependencies(protectedRoots.ToArray(), true), StringComparer.OrdinalIgnoreCase);
            var newlyUsed = selected.Where(e => used.Contains(e.path) || protectedDeps.Contains(e.path) || IsProtectedPath(e.path))
                                    .Select(e => e.path).ToArray();
            if (newlyUsed.Length > 0) throw new InvalidOperationException(
                "使用中・保護対象になったファイルがあります。削除を中止します。\n" + string.Join("\n", newlyUsed.Take(8).ToArray()));
            if (addressablesInspectionIncomplete) throw new InvalidOperationException(
                "Addressablesの保護対象を完全に確認できないため、安全のため一括削除を中止します。" +
                "対象フォルダーを追加保護して再調査してください。");

            var failures = new List<string>();
            bool ok = AssetDatabase.MoveAssetsToTrash(selected.Select(e => e.path).ToArray(), failures);
            AssetDatabase.Refresh();
            int succeeded = selected.Count - failures.Count;
            EditorUtility.DisplayDialog("削除結果",
                "成功: " + succeeded + " 件 / 失敗: " + failures.Count + " 件\n" +
                "バックアップ: " + destRoot + (ok ? "" : "\n失敗ファイルはConsoleも確認してください。"), "OK");
            if (!ok) Debug.LogWarning("[UnusedAssetManager] 削除できなかったパス:\n" + string.Join("\n", failures.ToArray()));
            Scan();
        }
        catch (Exception ex)
        {
            EditorUtility.DisplayDialog("削除中止", ex.Message + "\nバックアップ: " + destRoot, "OK");
        }
    }

    private static string AbsolutePath(string assetPath)
    {
        string root = Directory.GetParent(Application.dataPath).FullName;
        return Path.Combine(root, assetPath.Replace('/', Path.DirectorySeparatorChar));
    }

    private static string Csv(string value) { return "\"" + value.Replace("\"", "\"\"") + "\""; }
    private static string SizeText(long bytes)
    {
        if (bytes >= 1073741824L) return (bytes / 1073741824.0).ToString("F2") + " GB";
        if (bytes >= 1048576L) return (bytes / 1048576.0).ToString("F2") + " MB";
        return (bytes / 1024.0).ToString("F1") + " KB";
    }
}
