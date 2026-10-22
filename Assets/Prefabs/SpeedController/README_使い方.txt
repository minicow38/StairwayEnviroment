CockpitGauge - 航空計器風メーター（4〜40 / dead）
Unity 2022.3 / URP / UGUI (Canvas) / 無料オリジナル素材

【導入】
1. ZIPを展開して CockpitGauge フォルダーごと Unity プロジェクトの Assets に入れます。
2. Unityのコンパイルが終わったら、上部メニューで
   GameObject > UI > Cockpit Gauge (4-40, DEAD)
   をクリックします。
3. Hierarchyに CockpitGauge が作成されます。
   既存のCanvasがなければ専用Canvasも自動作成します。
4. CockpitGauge を選択し、Inspector > Cockpit Gauge (Script) の
   Value (4〜40) スライダーを調整すると、再生前でも針が動きます。
5. Dead をONにすると、最後の「dead」へ針が移動します。
   Playモードでは Smooth Speed で針の滑らかさを調節できます。

【ゲームコードから制御】
  public CockpitGauge gauge;
  void Update() { gauge.Value = Mathf.Clamp(speed, 4f, 40f); }
  // 死亡時など
  gauge.Dead = true;

【目盛りの角度】
4    = 左下   (Unity Z=+120°)
24   = 真上   (Unity Z=   0°)
40   = 右下手前(Unity Z= -96°)
dead = 右下端 (Unity Z=-120°)

【画像】
Art/GaugeDial.png   : 数字と目盛りが描かれた盤面（1024x1024 RGBA）
Art/GaugeNeedle.png : 針だけの透明画像（1024x1024 RGBA）
Art/*.svg           : 元のベクター素材。Unityへの取り込みはPNGを使います。

【注意】
- 盤面の数字は画像に焼き込まれています。値の読み取り部分はUI.Textで更新します。
- 針画像は中心を回転軸としているため、RectTransformのPivotは(0.5,0.5)のままにしてください。
- UIスケールは親RectTransformのSize Deltaで変更できます。
- 完全にオリジナルの素材として作成しています。追加アセット不要です。
