using UnityEditor;
using UnityEngine;

// Inspector controls for rapid needle/label alignment. Editor-only.
[CustomEditor(typeof(CockpitGauge))]
public sealed class CockpitGaugeInspector : Editor
{
    SerializedProperty valueProp;
    SerializedProperty deadProp;

    void OnEnable()
    {
        valueProp = serializedObject.FindProperty("value");
        deadProp = serializedObject.FindProperty("dead");
    }

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        var gauge = (CockpitGauge)target;

        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("目盛り合わせ / Calibration Test", EditorStyles.boldLabel);
        EditorGUILayout.HelpBox(
            "まず Value=24 で針を真上(0°)に合わせます。" +
            "ずれていたら Needle Art Offset Z を調整。" +
            "次に4/40/deadを押して端の角度を確認してください。", MessageType.Info);

        using (new EditorGUILayout.HorizontalScope())
        {
            if (GUILayout.Button("4 (左下)")) SetPreview(4f, false);
            if (GUILayout.Button("24 (真上)")) SetPreview(24f, false);
            if (GUILayout.Button("40 (右下)")) SetPreview(40f, false);
            if (GUILayout.Button("dead")) SetPreview(40f, true);
        }

        EditorGUILayout.LabelField("4→8→…→40 の1目盛り", gauge.AnglePerNumberStepZ.ToString("F2") + "°");
        EditorGUILayout.LabelField("目標 Z", gauge.TargetNeedleAngleZ.ToString("F2") + "°");
        EditorGUILayout.LabelField("表示 Z", gauge.CurrentNeedleAngleZ.ToString("F2") + "°");

        if (!gauge.AnglesHaveCorrectOrder)
            EditorGUILayout.HelpBox(
                "付属の文字盤は 4の角度 > 40の角度 > deadの角度 です。" +
                "この順に設定しないと、針の進行順序が合いません。", MessageType.Warning);

        if (!gauge.Needle)
            EditorGUILayout.HelpBox(
                "Needleが未設定です。子オブジェクト Needle を References の Needle に割り当ててください。", MessageType.Warning);
        else if (Vector2.Distance(gauge.Needle.pivot, new Vector2(.5f, .5f)) > .001f)
            EditorGUILayout.HelpBox(
                "Needle の RectTransform Pivot は (0.5, 0.5) にしてください。", MessageType.Warning);
    }

    void SetPreview(float reading, bool isDead)
    {
        serializedObject.Update();
        valueProp.floatValue = reading;
        deadProp.boolValue = isDead;
        serializedObject.ApplyModifiedProperties();
        EditorUtility.SetDirty(target);
        SceneView.RepaintAll();
    }
}
