using DG.Tweening;
using System;
using System.Collections;
using System.Collections.Generic;


using UnityEngine.SceneManagement;
using System.Collections.Generic;
using UnityEngine;

public class FireBaseAdConversion : MonoBehaviour
{
    static public FireBaseAdConversion instance;
   
    public Firebase.FirebaseApp app;

    [Header("マウスオーバーで各種変数名のツールチップを参照してください")]
    [Header("シングルトンでアクセス可能です　例：FireBaseAdConversion.instance.InterstitialsAd();")]
    [Header("使用方法：各種ユニットIDを記載してください。使用しない広告は空欄にしてください。")]
    [Tooltip("表示：FireBaseAdConversion.instance.InterstitialsAd();")] [SerializeField] public string adInterUnitId; //インターステイシャル用ユニットID
    int retryAttempt;

    [Tooltip("表示：FireBaseAdConversion.instance.BannerAd(); 非表示：FireBaseAdConversion.instance.BannerHideAd();")] [SerializeField] private string bannerAdUnitId;   //バナー用ユニットID
    [Tooltip("表示：FireBaseAdConversion.instance.MrecAd(); 非表示：FireBaseAdConversion.instance.MrecHideAd();")] [SerializeField] private string mrecAdUnitId;  //MREC広告用ユニットID 

    [Tooltip("表示：FireBaseAdConversion.instance.RewardedAd(); ")] [SerializeField] private string adRewardUnitId; //リワード広告ユニットID  
    int RewardRetryAttempt;

    [Header("AppLovinMAX SDKキー")]
    [SerializeField] private string SdkKey;
    [Header("テストモードを使用する場合はテストデバイスIDを入力してください、空欄の場合はテストモードオフ")]
    [SerializeField] private string TestDeviceId;

    [Header("AutoBannerOnにチェックを入れると自動で開始3秒後にバナー広告を表示します。")]
    [SerializeField] private bool AutoBannerOn;

    [Header("AdStopOnにチェックを入れるとインターステイシャル及びリワード広告再生時にゲームを一時停止します。")]
    [SerializeField] private bool AdStopOn;
    bool enableByteBrew = false;
    public void Awake()
    {
        if (instance == null)
        {
            instance = this;
        }
        else
        {
            Destroy(gameObject);
        }
    }
    IEnumerator Start()
    {
        
        
        float timeout = 5f;
        float timer = 0f;

       
        InitMax();
        yield break;
    }

    // Start is called before the first frame update
    void InitMax()
    {
        //Firebase初期化コード
        Firebase.FirebaseApp.CheckAndFixDependenciesAsync().ContinueWith(task =>
        {
            var dependencyStatus = task.Result;

            if (dependencyStatus == Firebase.DependencyStatus.Available)
            {
                app = Firebase.FirebaseApp.DefaultInstance;

                Debug.Log("FireBase初期化!");

                StartCoroutine(ChangeGameScene());
            }
            else
            {
                UnityEngine.Debug.LogError(System.String.Format(
                    "Could not resolve all Firebase dependencies: {0}", dependencyStatus));
            }

        }, System.Threading.Tasks.TaskScheduler.FromCurrentSynchronizationContext());
    }

  
    public IEnumerator ChangeGameScene()
    {

        yield return new WaitForSeconds(1);


        SceneManager.LoadScene(1);
    }
    private void OnApplicationFocus(bool focus)
    {
        Debug.Log("[APP_STATE] Focus: " + focus + " frame=" + Time.frameCount);
        Debug.Log("");
    }

    private void OnApplicationPause(bool pause)
    {
        Debug.Log("[APP_STATE] Pause: " + pause + " frame=" + Time.frameCount);
        Debug.Log("");
    }
}

