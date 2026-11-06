using UnityEngine;
using System;
using System.IO;

public class GameScreenshot : MonoBehaviour
{
    private const KeyCode ShutterKey = KeyCode.F12;

    void Update()
    {
        // シャッター入力待ち
        if (!Input.GetKeyDown(ShutterKey))
            return;

        TakeScreenshot();
    }

    private void TakeScreenshot()
    {
        string folder = Path.Combine(
            Application.dataPath, "..", "Screenshots");

        Directory.CreateDirectory(folder);

        string filename = "PGuiltyStairway_" +
                          DateTime.Now.ToString("yyyyMMdd_HHmmss_fff") +
                          ".png";

        string path = Path.GetFullPath(
            Path.Combine(folder, filename));

        // 現在のゲーム画面を撮影
        ScreenCapture.CaptureScreenshot(path);

        Debug.Log("[SCREENSHOT] " + path);
    }
}