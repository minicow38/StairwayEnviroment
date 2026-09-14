using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class TenjinExampleScript : MonoBehaviour
{
    [Header("TenjinSDKキーを入力してください")]
    [SerializeField] string TenjinSDKKey; //TenjinSDK キー    
     bool isInitialized = false; // 再実行防止フラグ
    public bool CopeWithAnalytics = false;
    void Start()
    {
        //TenjinConnect();
        StartCoroutine(InitFlow());
        Debug.Log("");
    }

    IEnumerator InitFlow()
    {
        yield return null;
        TenjinConnect(); // Tenjin初期化
       // yield return new WaitForSeconds(1.5f); // 少し待ってからシーン遷移(安定化)

        //StartCoroutine(ChangeGameScene());
        //ChangeGameScene();
    }

    public void TenjinConnect()
    {
        if (isInitialized) { return; } // 二重実行防止
        isInitialized = true;

        try
        {
            if (string.IsNullOrEmpty(TenjinSDKKey)) { return; }
            BaseTenjin instance = Tenjin.getInstance(TenjinSDKKey);
            if (instance == null) { return; }

            instance.SetAppStoreType(AppStoreType.googleplay);
            // Sends install/open event to Tenjin
            instance.Connect();
            Debug.Log("TenjinSDK接続");
        }
        catch (System.Exception e)
        {
            Debug.LogError(" Tenjin例外発生: " + e);
        }
    }

    void LateUpdate()
    {
        
        //ChangeGameScene();
        
    }

    public IEnumerator ChangeGameScene()
    {
        Debug.Log("[BOOT_FLOW] Tenjin ChangeGameScene START frame=" + Time.frameCount + " time=" + Time.realtimeSinceStartup);

        //ゲーム画面を呼び出す処理を記入する
        //Firebase.Analytics.FirebaseAnalytics.LogEvent("test_event_fuck");
        yield return new WaitForSeconds(1);
        Debug.Log("[BOOT_FLOW] Before LoadScene Level_01 frame=" + Time.frameCount + " time=" + Time.realtimeSinceStartup);

        //記入例
        SceneManager.LoadScene("Level_01");

        //SceneManager.LoadScene("Level_01");
    }
}