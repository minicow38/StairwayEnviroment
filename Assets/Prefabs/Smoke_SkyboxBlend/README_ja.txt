Smoke Skybox Blend — Unity 2022.3 / URP

使い方：
1. ZIP内の Assets/Smoke_SkyboxBlend/ フォルダを自分のUnityプロジェクトの Assets/ にコピーする。
2. 現在の煙に置き換えて Smoke_SkyboxMatched.prefab を使用する。
3. Inspector > Smoke Skybox Harmonyで skyInfluence (0～1) と smokeOpacity (0～1) を調整。
4. 既存の eff_smoke1_alpha マテリアルが必要です。元と同じ参照GUIDを使っています。

重要：
- 既存の煙の形、寿命、発生数、速度、Looping設定、マテリアル参照はそのままです。
- Smoke_CoolSky_Static.prefab はスクリプト不要、色だけ変更した比較版です。
- スクリプトは、FogがONならFog Colorを優先し、なければSkybox Materialの代表的な色プロパティ（_HorizonColorなど）を試します。
  取得できない時はAmbientの色または動画で見えた青灰色に近いfallbackSkyColorを使用。
- Skybox MaterialのTintは画面に描画されたピクセルの色とは異なります。完全な背景ピクセルへの溶け込みにはURP Shader GraphでScene Colorをサンプルする必要があります。
- 元マテリアルのシェーダーがAdditiveやParticle Color未対応の場合、背景色と一致させる効果は限定的です。
- 煙の見た目を自然にするならマテリアルはURP/Particles/Unlit（Surface Transparent/Alpha）または URP/Particles/Simple Litで調整してください。
- Soft ParticlesにはURP Camera/AssetのDepth Texture有効化が必要です。
- 静的な背景ならスクリプトは有効化時だけ反映する設定。動的な背景ならrefreshWhilePlayingをONにすると定期更新（新しい粒子に反映）します。
- 実際のUnity画面上のレンダリングは未テスト。シェーダー/マテリアルによって見た目が変わります。
