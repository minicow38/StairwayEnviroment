#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using System.Collections.Generic;

public static class HShieldCharacterPrefabGeneratorV2
{
    const string Root = "Assets/Generated/HShieldCharacterV2";
    const string MeshFolder = Root + "/Meshes";
    const string MatFolder = Root + "/Materials";
    const string PrefabPath = Root + "/HShieldCharacter_V2.prefab";

    [MenuItem("Tools/GPT Prefabs/Create H Shield Character V2")]
    public static void Create()
    {
        Ensure("Assets", "Generated");
        Ensure("Assets/Generated", "HShieldCharacterV2");
        Ensure(Root, "Meshes");
        Ensure(Root, "Materials");

        Material skin   = Mat("Skin",       new Color(0.68f, 0.56f, 0.45f));
        Material hair   = Mat("Hair",       new Color(0.76f, 0.60f, 0.12f));
        Material beard  = Mat("Beard",      new Color(0.68f, 0.50f, 0.07f));
        Material green  = Mat("Green",      new Color(0.27f, 0.34f, 0.10f));
        Material boot   = Mat("Boot",       new Color(0.24f, 0.29f, 0.08f));
        Material blue   = Mat("Blue",       new Color(0.07f, 0.32f, 0.62f));
        Material grey   = Mat("Grey",       new Color(0.42f, 0.42f, 0.40f));
        Material dark   = Mat("Dark",       new Color(0.17f, 0.17f, 0.16f));
        Material gold   = Mat("Gold",       new Color(0.73f, 0.55f, 0.06f));
        Material eyeMat = Mat("Eye",        new Color(0.05f, 0.05f, 0.04f));
        Material baseM  = Mat("Base",       new Color(0.09f, 0.10f, 0.09f));

        var existing = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (existing != null)
            AssetDatabase.DeleteAsset(PrefabPath);

        GameObject root = new GameObject("HShieldCharacter_V2");

        // --- Base
        var baseObj = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        baseObj.name = "Base";
        baseObj.transform.SetParent(root.transform, false);
        baseObj.transform.localPosition = new Vector3(0f, 0.08f, 0.05f);
        baseObj.transform.localScale = new Vector3(1.55f, 0.07f, 1.15f);
        baseObj.GetComponent<Renderer>().sharedMaterial = baseM;

        // --- Legs / boots: short, wide, slightly outward
        AddLowPolyCapsule(root.transform, "Leg_L",
            new Vector3(-0.34f, 0.92f, 0.05f), new Vector3(0.25f, 0.70f, 0.25f), skin, 6);
        AddLowPolyCapsule(root.transform, "Leg_R",
            new Vector3( 0.34f, 0.92f, 0.05f), new Vector3(0.25f, 0.70f, 0.25f), skin, 6);

        AddPrism(root.transform, "Boot_L",
            new Vector2[] {
                new Vector2(-0.34f,-0.30f), new Vector2(0.24f,-0.30f),
                new Vector2(0.36f,-0.06f), new Vector2(0.22f,0.30f),
                new Vector2(-0.22f,0.30f)
            }, 0.42f, new Vector3(-0.34f, 0.48f, -0.03f), boot);

        AddPrism(root.transform, "Boot_R",
            new Vector2[] {
                new Vector2(-0.24f,-0.30f), new Vector2(0.34f,-0.30f),
                new Vector2(0.22f,0.30f), new Vector2(-0.22f,0.30f),
                new Vector2(-0.36f,-0.06f)
            }, 0.42f, new Vector3(0.34f, 0.48f, -0.03f), boot);

        // --- Ragged green shorts
        AddPrism(root.transform, "Shorts",
            new Vector2[] {
                new Vector2(-0.55f, 0.30f), new Vector2(0.55f,0.30f),
                new Vector2(0.48f,-0.20f), new Vector2(0.25f,-0.12f),
                new Vector2(0.02f,-0.28f), new Vector2(-0.22f,-0.10f),
                new Vector2(-0.48f,-0.22f)
            }, 0.62f, new Vector3(0f, 1.38f, 0f), green);

        // --- Bare torso: narrow waist, broader shoulders, leaning forward
        var torso = AddPrism(root.transform, "Torso",
            new Vector2[] {
                new Vector2(-0.50f,-0.58f), new Vector2(0.50f,-0.58f),
                new Vector2(0.60f,0.42f), new Vector2(0.36f,0.64f),
                new Vector2(-0.36f,0.64f), new Vector2(-0.60f,0.42f)
            }, 0.58f, new Vector3(-0.04f, 2.18f, 0.08f), skin);
        torso.transform.localRotation = Quaternion.Euler(8f, 0f, 0f);

        // --- Head: larger and angular
        var head = AddLowPolySphere(root.transform, "Head",
            new Vector3(-0.10f, 3.18f, -0.02f), new Vector3(0.52f, 0.65f, 0.49f),
            skin, 8, 5);

        // Long angular nose
        AddPrism(root.transform, "Nose",
            new Vector2[] {
                new Vector2(-0.11f,0.19f), new Vector2(0.11f,0.19f),
                new Vector2(0.17f,-0.02f), new Vector2(0.02f,-0.27f),
                new Vector2(-0.17f,-0.02f)
            }, 0.22f, new Vector3(-0.10f, 3.16f, -0.48f), skin);

        // Eyes beneath hair
        AddLowPolySphere(root.transform, "Eye_L",
            new Vector3(-0.28f, 3.32f, -0.46f), Vector3.one * 0.045f, eyeMat, 6, 3);
        AddLowPolySphere(root.transform, "Eye_R",
            new Vector3(0.04f, 3.32f, -0.46f), Vector3.one * 0.045f, eyeMat, 6, 3);

        // --- Hair: hood-like silhouette wrapping face
        AddLowPolySphere(root.transform, "Hair_Crown",
            new Vector3(-0.08f, 3.64f, 0.02f), new Vector3(0.65f, 0.48f, 0.55f),
            hair, 8, 4);

        AddPrism(root.transform, "Hair_Left",
            new Vector2[] {
                new Vector2(-0.34f,0.52f), new Vector2(0.08f,0.44f),
                new Vector2(0.20f,-0.35f), new Vector2(-0.05f,-0.70f),
                new Vector2(-0.38f,-0.55f), new Vector2(-0.48f,0.20f)
            }, 0.42f, new Vector3(-0.54f, 3.25f, 0.02f), hair);

        AddPrism(root.transform, "Hair_Right",
            new Vector2[] {
                new Vector2(-0.08f,0.44f), new Vector2(0.34f,0.52f),
                new Vector2(0.48f,0.20f), new Vector2(0.38f,-0.55f),
                new Vector2(0.05f,-0.70f), new Vector2(-0.20f,-0.35f)
            }, 0.42f, new Vector3(0.38f, 3.25f, 0.02f), hair);

        // Hair tips
        AddPrism(root.transform, "HairTip_L",
            new Vector2[] { new Vector2(-0.22f,0.18f), new Vector2(0.18f,0.18f), new Vector2(-0.10f,-0.38f) },
            0.28f, new Vector3(-0.78f, 2.93f, 0.06f), hair);
        AddPrism(root.transform, "HairTip_R",
            new Vector2[] { new Vector2(-0.18f,0.18f), new Vector2(0.22f,0.18f), new Vector2(0.10f,-0.38f) },
            0.28f, new Vector3(0.62f, 2.93f, 0.06f), hair);

        // --- Beard: pointed and dominant
        AddPrism(root.transform, "Beard",
            new Vector2[] {
                new Vector2(-0.36f,0.40f), new Vector2(0.36f,0.40f),
                new Vector2(0.28f,-0.18f), new Vector2(0.04f,-0.72f),
                new Vector2(-0.22f,-0.26f)
            }, 0.30f, new Vector3(-0.05f, 2.82f, -0.38f), beard);

        AddPrism(root.transform, "Moustache_L",
            new Vector2[] {
                new Vector2(-0.30f,0.08f), new Vector2(0.05f,0.16f),
                new Vector2(0.18f,0.00f), new Vector2(-0.20f,-0.14f)
            }, 0.18f, new Vector3(-0.26f, 3.02f, -0.49f), beard);
        AddPrism(root.transform, "Moustache_R",
            new Vector2[] {
                new Vector2(-0.05f,0.16f), new Vector2(0.30f,0.08f),
                new Vector2(0.20f,-0.14f), new Vector2(-0.18f,0.00f)
            }, 0.18f, new Vector3(0.06f, 3.02f, -0.49f), beard);

        // --- Blue hanging banner behind the shield
        AddPrism(root.transform, "BlueBanner",
            new Vector2[] {
                new Vector2(-0.48f,0.92f), new Vector2(0.46f,0.92f),
                new Vector2(0.40f,-0.62f), new Vector2(0.18f,-0.88f),
                new Vector2(-0.08f,-0.68f), new Vector2(-0.38f,-0.88f)
            }, 0.11f, new Vector3(0.40f, 2.12f, -0.68f), blue);

        // --- Shield: dominant stepped silhouette
        Transform shield = new GameObject("Shield").transform;
        shield.SetParent(root.transform, false);
        shield.localPosition = new Vector3(0.48f, 2.04f, -0.96f);
        shield.localRotation = Quaternion.Euler(1f, -2f, -1f);

        Vector2[] shieldOuter = {
            new Vector2(-0.48f, 0.86f),
            new Vector2( 0.48f, 0.86f),
            new Vector2( 0.70f, 0.66f),
            new Vector2( 0.70f, 0.36f),
            new Vector2( 0.82f, 0.36f),
            new Vector2( 0.82f,-0.34f),
            new Vector2( 0.64f,-0.34f),
            new Vector2( 0.64f,-0.58f),
            new Vector2( 0.36f,-0.78f),
            new Vector2(-0.36f,-0.78f),
            new Vector2(-0.64f,-0.58f),
            new Vector2(-0.64f,-0.34f),
            new Vector2(-0.82f,-0.34f),
            new Vector2(-0.82f, 0.36f),
            new Vector2(-0.70f, 0.36f),
            new Vector2(-0.70f, 0.66f)
        };
        AddPrism(shield, "ShieldOuter", shieldOuter, 0.24f, Vector3.zero, grey);

        Vector2[] shieldInner = {
            new Vector2(-0.46f,0.55f), new Vector2(0.46f,0.55f),
            new Vector2(0.58f,0.42f), new Vector2(0.58f,-0.34f),
            new Vector2(0.38f,-0.52f), new Vector2(-0.38f,-0.52f),
            new Vector2(-0.58f,-0.34f), new Vector2(-0.58f,0.42f)
        };
        AddPrism(shield, "ShieldInner", shieldInner, 0.07f, new Vector3(0,0,-0.155f), dark);

        // H emblem
        AddBox(shield, "H_Left",  new Vector3(-0.28f,0f,-0.225f), new Vector3(0.13f,0.78f,0.06f), gold);
        AddBox(shield, "H_Right", new Vector3( 0.28f,0f,-0.225f), new Vector3(0.13f,0.78f,0.06f), gold);
        AddBox(shield, "H_Mid",   new Vector3( 0f,0.02f,-0.225f), new Vector3(0.55f,0.13f,0.06f), gold);
        AddBox(shield, "H_TL", new Vector3(-0.28f,0.34f,-0.225f), new Vector3(0.26f,0.13f,0.06f), gold);
        AddBox(shield, "H_TR", new Vector3( 0.28f,0.34f,-0.225f), new Vector3(0.26f,0.13f,0.06f), gold);
        AddBox(shield, "H_BL", new Vector3(-0.28f,-0.34f,-0.225f), new Vector3(0.26f,0.13f,0.06f), gold);
        AddBox(shield, "H_BR", new Vector3( 0.28f,-0.34f,-0.225f), new Vector3(0.26f,0.13f,0.06f), gold);

        // --- Arms laid across top of shield, heavy hands
        AddCylinderBetween(root.transform, "UpperArm_L",
            new Vector3(-0.48f,2.63f,-0.35f), new Vector3(-0.18f,2.48f,-0.75f),
            0.18f, skin, 6);
        AddCylinderBetween(root.transform, "ForeArm_L",
            new Vector3(-0.18f,2.48f,-0.75f), new Vector3(0.18f,2.50f,-1.02f),
            0.17f, skin, 6);

        AddCylinderBetween(root.transform, "UpperArm_R",
            new Vector3(0.42f,2.62f,-0.30f), new Vector3(0.68f,2.52f,-0.72f),
            0.18f, skin, 6);
        AddCylinderBetween(root.transform, "ForeArm_R",
            new Vector3(0.68f,2.52f,-0.72f), new Vector3(0.88f,2.48f,-1.03f),
            0.17f, skin, 6);

        AddLowPolySphere(root.transform, "Hand_L",
            new Vector3(0.18f,2.50f,-1.03f), new Vector3(0.24f,0.21f,0.22f), skin, 6, 3);
        AddLowPolySphere(root.transform, "Hand_R",
            new Vector3(0.88f,2.48f,-1.04f), new Vector3(0.24f,0.21f,0.22f), skin, 6, 3);

        // Remove temporary colliders from Unity primitives.
        foreach (var c in root.GetComponentsInChildren<Collider>())
            Object.DestroyImmediate(c);

        PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
        Object.DestroyImmediate(root);

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();

        var p = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        Selection.activeObject = p;
        EditorGUIUtility.PingObject(p);
        Debug.Log("Created: " + PrefabPath);
    }

    static GameObject AddBox(Transform parent, string name, Vector3 pos, Vector3 scale, Material mat)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = name;
        go.transform.SetParent(parent, false);
        go.transform.localPosition = pos;
        go.transform.localScale = scale;
        go.GetComponent<Renderer>().sharedMaterial = mat;
        return go;
    }

    static GameObject AddPrism(Transform parent, string name, Vector2[] poly, float depth, Vector3 pos, Material mat)
    {
        Mesh mesh = BuildPrism(poly, depth);
        SaveMesh(mesh, name);

        var go = new GameObject(name);
        go.transform.SetParent(parent, false);
        go.transform.localPosition = pos;
        var mf = go.AddComponent<MeshFilter>();
        var mr = go.AddComponent<MeshRenderer>();
        mf.sharedMesh = mesh;
        mr.sharedMaterial = mat;
        return go;
    }

    static Mesh BuildPrism(Vector2[] p, float depth)
    {
        int n = p.Length;
        float z0 = -depth * 0.5f;
        float z1 =  depth * 0.5f;
        List<Vector3> v = new List<Vector3>(n * 2);
        for (int i=0;i<n;i++) v.Add(new Vector3(p[i].x, p[i].y, z0));
        for (int i=0;i<n;i++) v.Add(new Vector3(p[i].x, p[i].y, z1));

        List<int> t = new List<int>();

        for (int i=1;i<n-1;i++) {
            t.Add(0); t.Add(i+1); t.Add(i);
            t.Add(n); t.Add(n+i); t.Add(n+i+1);
        }

        for (int i=0;i<n;i++) {
            int j = (i+1)%n;
            t.Add(i); t.Add(j); t.Add(n+j);
            t.Add(i); t.Add(n+j); t.Add(n+i);
        }

        Mesh m = new Mesh();
        m.name = "Prism";
        m.SetVertices(v);
        m.SetTriangles(t, 0);
        m.RecalculateNormals();
        m.RecalculateBounds();
        return m;
    }

    static GameObject AddLowPolySphere(Transform parent, string name, Vector3 pos, Vector3 scale, Material mat, int lon, int lat)
    {
        Mesh mesh = BuildLowPolySphere(lon, lat);
        SaveMesh(mesh, name);

        var go = new GameObject(name);
        go.transform.SetParent(parent, false);
        go.transform.localPosition = pos;
        go.transform.localScale = scale;
        go.AddComponent<MeshFilter>().sharedMesh = mesh;
        go.AddComponent<MeshRenderer>().sharedMaterial = mat;
        return go;
    }

    static Mesh BuildLowPolySphere(int lon, int lat)
    {
        List<Vector3> v = new List<Vector3>();
        List<int> t = new List<int>();

        v.Add(Vector3.up);

        for (int y=1; y<lat; y++) {
            float a = Mathf.PI * y / lat;
            float sy = Mathf.Cos(a);
            float r = Mathf.Sin(a);
            for (int x=0; x<lon; x++) {
                float b = Mathf.PI * 2f * x / lon;
                v.Add(new Vector3(Mathf.Sin(b)*r, sy, Mathf.Cos(b)*r));
            }
        }

        int bottom = v.Count;
        v.Add(Vector3.down);

        for (int x=0; x<lon; x++) {
            int a = 1 + x;
            int b = 1 + ((x+1)%lon);
            t.Add(0); t.Add(b); t.Add(a);
        }

        for (int y=0; y<lat-2; y++) {
            int row0 = 1 + y*lon;
            int row1 = row0 + lon;
            for (int x=0; x<lon; x++) {
                int x1 = (x+1)%lon;
                int a = row0+x;
                int b = row0+x1;
                int c = row1+x;
                int d = row1+x1;
                t.Add(a); t.Add(b); t.Add(d);
                t.Add(a); t.Add(d); t.Add(c);
            }
        }

        int lastRow = 1 + (lat-2)*lon;
        for (int x=0; x<lon; x++) {
            int a = lastRow+x;
            int b = lastRow+((x+1)%lon);
            t.Add(a); t.Add(b); t.Add(bottom);
        }

        Mesh m = new Mesh();
        m.name = "LowPolySphere";
        m.SetVertices(v);
        m.SetTriangles(t, 0);
        m.RecalculateNormals();
        m.RecalculateBounds();
        return m;
    }

    static GameObject AddLowPolyCapsule(Transform parent, string name, Vector3 pos, Vector3 scale, Material mat, int sides)
    {
        // A compact low-poly cylinder with tapered ends.
        Mesh mesh = BuildTaperedCylinder(sides);
        SaveMesh(mesh, name);

        var go = new GameObject(name);
        go.transform.SetParent(parent, false);
        go.transform.localPosition = pos;
        go.transform.localScale = scale;
        go.AddComponent<MeshFilter>().sharedMesh = mesh;
        go.AddComponent<MeshRenderer>().sharedMaterial = mat;
        return go;
    }

    static Mesh BuildTaperedCylinder(int sides)
    {
        float[] ys = {-1f, -0.75f, 0.75f, 1f};
        float[] rs = {0.35f, 1f, 1f, 0.35f};
        List<Vector3> v = new List<Vector3>();
        List<int> t = new List<int>();

        for (int r=0;r<ys.Length;r++) {
            for (int i=0;i<sides;i++) {
                float a = Mathf.PI*2f*i/sides;
                v.Add(new Vector3(Mathf.Sin(a)*rs[r], ys[r], Mathf.Cos(a)*rs[r]));
            }
        }

        for (int r=0;r<ys.Length-1;r++) {
            int a0 = r*sides;
            int b0 = (r+1)*sides;
            for (int i=0;i<sides;i++) {
                int j=(i+1)%sides;
                t.Add(a0+i); t.Add(a0+j); t.Add(b0+j);
                t.Add(a0+i); t.Add(b0+j); t.Add(b0+i);
            }
        }

        Mesh m = new Mesh();
        m.name = "LowPolyCapsule";
        m.SetVertices(v);
        m.SetTriangles(t,0);
        m.RecalculateNormals();
        m.RecalculateBounds();
        return m;
    }

    static GameObject AddCylinderBetween(Transform parent, string name, Vector3 a, Vector3 b, float radius, Material mat, int sides)
    {
        Mesh mesh = BuildCylinder(sides);
        SaveMesh(mesh, name);

        Vector3 mid = (a+b)*0.5f;
        Vector3 dir = b-a;
        float len = dir.magnitude;

        var go = new GameObject(name);
        go.transform.SetParent(parent, false);
        go.transform.localPosition = mid;
        go.transform.localRotation = Quaternion.FromToRotation(Vector3.up, dir.normalized);
        go.transform.localScale = new Vector3(radius, len*0.5f, radius);
        go.AddComponent<MeshFilter>().sharedMesh = mesh;
        go.AddComponent<MeshRenderer>().sharedMaterial = mat;
        return go;
    }

    static Mesh BuildCylinder(int sides)
    {
        List<Vector3> v = new List<Vector3>();
        List<int> t = new List<int>();

        for (int y=0;y<2;y++) {
            float yy = y==0 ? -1f : 1f;
            for (int i=0;i<sides;i++) {
                float a = Mathf.PI*2f*i/sides;
                v.Add(new Vector3(Mathf.Sin(a), yy, Mathf.Cos(a)));
            }
        }

        for (int i=0;i<sides;i++) {
            int j=(i+1)%sides;
            t.Add(i); t.Add(j); t.Add(sides+j);
            t.Add(i); t.Add(sides+j); t.Add(sides+i);
        }

        Mesh m = new Mesh();
        m.name = "LowPolyCylinder";
        m.SetVertices(v);
        m.SetTriangles(t,0);
        m.RecalculateNormals();
        m.RecalculateBounds();
        return m;
    }

    static void SaveMesh(Mesh mesh, string baseName)
    {
        string path = MeshFolder + "/" + baseName + ".asset";
        var old = AssetDatabase.LoadAssetAtPath<Mesh>(path);
        if (old != null) {
            EditorUtility.CopySerialized(mesh, old);
            Object.DestroyImmediate(mesh);
        } else {
            AssetDatabase.CreateAsset(mesh, path);
        }
    }

    static Material Mat(string name, Color c)
    {
        string path = MatFolder + "/" + name + ".mat";
        Material m = AssetDatabase.LoadAssetAtPath<Material>(path);

        Shader s = Shader.Find("Universal Render Pipeline/Lit");
        if (s == null) s = Shader.Find("Standard");

        if (m == null) {
            m = new Material(s);
            m.name = name;
            AssetDatabase.CreateAsset(m, path);
        } else {
            m.shader = s;
        }

        m.color = c;
        if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", 0.03f);
        if (m.HasProperty("_Metallic")) m.SetFloat("_Metallic", 0f);
        EditorUtility.SetDirty(m);
        return m;
    }

    static void Ensure(string parent, string child)
    {
        string path = parent + "/" + child;
        if (!AssetDatabase.IsValidFolder(path))
            AssetDatabase.CreateFolder(parent, child);
    }
}
#endif
