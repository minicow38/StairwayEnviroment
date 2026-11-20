using DG.Tweening;
using System;
using System.Collections;
using UnityEngine.SceneManagement;

using System.Collections.Generic;
using UnityEngine;

public class FireBaseAdConversion : MonoBehaviour
{
    static public FireBaseAdConversion instance;

    [Header("マウスオーバーで各種変数名のツールチップを参照してください")]
    [Header("シングルトンでアクセス可能です　例：FireBaseAdConversion.instance.InterstitialsAd();")]
    [Header("使用方法：各種ユニットIDを記載してください。使用しない広告は空欄にしてください。")]
    [Tooltip("表示：FireBaseAdConversion.instance.InterstitialsAd();")] [SerializeField] private string adInterUnitId; //インターステイシャル用ユニットID
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

    private void Awake()
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


    // Start is called before the first frame update
    IEnumerator Start()
    {
        
        
        float timeout = 5f;
        float timer = 0f;

       
        InitMax();
        yield break;
    }
    void InitMax()
    {
          Firebase.FirebaseApp.CheckAndFixDependenciesAsync().ContinueWith(task =>
        {
            var dependencyStatus = task.Result;
            if (dependencyStatus == Firebase.DependencyStatus.Available)
            {
                // Create and hold a reference to your FirebaseApp,
                // where app is a Firebase.FirebaseApp property of your application class.
                var app = Firebase.FirebaseApp.DefaultInstance;
                // Set a flag here to indicate whether Firebase is ready to use by your app.
                //Firebase.Analytics.FirebaseAnalytics.LogEvent(Firebase.Analytics.FirebaseAnalytics.EventAppOpen);
                Debug.Log("FireBase初期化!");
            }
            else
            {
                UnityEngine.Debug.LogError(System.String.Format(
                  "Could not resolve all Firebase dependencies: {0}", dependencyStatus));
                // Firebase Unity SDK is not safe to use here.
            }
        });


        //SDK初期化
        MaxSdkCallbacks.OnSdkInitializedEvent += (MaxSdkBase.SdkConfiguration sdkConfiguration) =>
        {
            // AppLovin SDK is initialized, start loading ads
        };

        MaxSdk.SetSdkKey(this.SdkKey);
        MaxSdk.SetUserId("USER_ID");
       
        if (!string.IsNullOrEmpty(TestDeviceId)) MaxSdk.SetTestDeviceAdvertisingIdentifiers(new string[] { this.TestDeviceId });  //空欄の場合はテストモードオフ
        MaxSdk.InitializeSdk();
        if (!string.IsNullOrEmpty(adInterUnitId)) InitializeInterstitialAds(); //インタースティシャル初期化ロード
        if (!string.IsNullOrEmpty(adRewardUnitId)) InitializeRewardedAds();     //リワード広告初期化ロード

        if (!string.IsNullOrEmpty(bannerAdUnitId))
        {

            InitializeBannerAds();       //バナー初期化ロード
        }
        else
        {
            AutoBannerOn = false;
        }
        

        if (!string.IsNullOrEmpty(mrecAdUnitId)) InitializeMRecAds();         //MREC初期化ロード


        if (AutoBannerOn) Invoke("BannerAd", 3);  //ロードのためにバナーは3秒後に開始

        DOVirtual.DelayedCall(0.5f, () =>
        {
            //TenjinSDKSCのステージロードを起動
            StartCoroutine(ChangeGameScene());
        });
    }
    public IEnumerator ChangeGameScene()
    {

        yield return new WaitForSeconds(1);


        SceneManager.LoadScene(1);
    }

    public void InitializeMRecAds()
    {
        // MRECs are sized to 300x250 on phones and tablets
        MaxSdk.CreateMRec(mrecAdUnitId, MaxSdkBase.AdViewPosition.BottomCenter);
    }

    public void InitializeBannerAds()
    {
        // Banners are automatically sized to 320?50 on phones and 728?90 on tablets
        // You may call the utility method MaxSdkUtils.isTablet() to help with view sizing adjustments
        MaxSdk.CreateBanner(bannerAdUnitId, MaxSdkBase.BannerPosition.BottomCenter);

        // Set background or background color for banners to be fully functional
        Color color = new Color(255, 255, 255, 0);
        MaxSdk.SetBannerBackgroundColor(bannerAdUnitId, color);
    }


    public void InitializeInterstitialAds()
    {
        // Attach callback
        MaxSdkCallbacks.Interstitial.OnAdLoadedEvent += OnInterstitialLoadedEvent;
        MaxSdkCallbacks.Interstitial.OnAdLoadFailedEvent += OnInterstitialLoadFailedEvent;
        MaxSdkCallbacks.Interstitial.OnAdDisplayedEvent += OnInterstitialDisplayedEvent;
        MaxSdkCallbacks.Interstitial.OnAdClickedEvent += OnInterstitialClickedEvent;
        MaxSdkCallbacks.Interstitial.OnAdHiddenEvent += OnInterstitialHiddenEvent;
        MaxSdkCallbacks.Interstitial.OnAdDisplayFailedEvent += OnInterstitialAdFailedToDisplayEvent;

        // Load the first interstitial
        LoadInterstitial();
    }

    private void LoadInterstitial()
    {
        MaxSdk.LoadInterstitial(adInterUnitId);
    }

    private void OnInterstitialLoadedEvent(string adUnitId, MaxSdkBase.AdInfo adInfo)
    {
        // Interstitial ad is ready for you to show. MaxSdk.IsInterstitialReady(adUnitId) now returns 'true'

        // Reset retry attempt
        retryAttempt = 0;
    }

    private void OnInterstitialLoadFailedEvent(string adUnitId, MaxSdkBase.ErrorInfo errorInfo)
    {
        // Interstitial ad failed to load 
        // AppLovin recommends that you retry with exponentially higher delays, up to a maximum delay (in this case 64 seconds)

        retryAttempt++;
        double retryDelay = Math.Pow(2, Math.Min(6, retryAttempt));

        Invoke("LoadInterstitial", (float)retryDelay);
    }

    private void OnInterstitialDisplayedEvent(string adUnitId, MaxSdkBase.AdInfo adInfo)
    {
        //カスタムイベント
        //gameDirectorSC.CustomInterstitialsAd_Start();

        if (AdStopOn) Time.timeScale = 0;

    }

    private void OnInterstitialAdFailedToDisplayEvent(string adUnitId, MaxSdkBase.ErrorInfo errorInfo, MaxSdkBase.AdInfo adInfo)
    {
        // Interstitial ad failed to display. AppLovin recommends that you load the next ad.
        LoadInterstitial();
    }

    private void OnInterstitialClickedEvent(string adUnitId, MaxSdkBase.AdInfo adInfo) { }

    private void OnInterstitialHiddenEvent(string adUnitId, MaxSdkBase.AdInfo adInfo)
    {
        // Interstitial ad is hidden. Pre-load the next ad.
        LoadInterstitial();
        if (AdStopOn)
        {
            DOTween.PlayAll();
            Time.timeScale = 1;
        }
    }



    public void InitializeRewardedAds()
    {
        // Attach callback
        MaxSdkCallbacks.Rewarded.OnAdLoadedEvent += OnRewardedAdLoadedEvent;
        MaxSdkCallbacks.Rewarded.OnAdLoadFailedEvent += OnRewardedAdLoadFailedEvent;
        MaxSdkCallbacks.Rewarded.OnAdDisplayedEvent += OnRewardedAdDisplayedEvent;
        MaxSdkCallbacks.Rewarded.OnAdClickedEvent += OnRewardedAdClickedEvent;
        MaxSdkCallbacks.Rewarded.OnAdRevenuePaidEvent += OnRewardedAdRevenuePaidEvent;
        MaxSdkCallbacks.Rewarded.OnAdHiddenEvent += OnRewardedAdHiddenEvent;
        MaxSdkCallbacks.Rewarded.OnAdDisplayFailedEvent += OnRewardedAdFailedToDisplayEvent;
        MaxSdkCallbacks.Rewarded.OnAdReceivedRewardEvent += OnRewardedAdReceivedRewardEvent;

        // Load the first rewarded ad
        LoadRewardedAd();
        Debug.Log("リワード広告ロード");
    }

    private void LoadRewardedAd()
    {
        MaxSdk.LoadRewardedAd(adRewardUnitId);
    }

    private void OnRewardedAdLoadedEvent(string adUnitId, MaxSdkBase.AdInfo adInfo)
    {
        // Rewarded ad is ready for you to show. MaxSdk.IsRewardedAdReady(adUnitId) now returns 'true'.

        // Reset retry attempt
        RewardRetryAttempt = 0;
    }

    private void OnRewardedAdLoadFailedEvent(string adUnitId, MaxSdkBase.ErrorInfo errorInfo)
    {
        // Rewarded ad failed to load 
        // AppLovin recommends that you retry with exponentially higher delays, up to a maximum delay (in this case 64 seconds).

        RewardRetryAttempt++;
        double retryDelay = Math.Pow(2, Math.Min(6, RewardRetryAttempt));

        Invoke("LoadRewardedAd", (float)retryDelay);
    }

    private void OnRewardedAdDisplayedEvent(string adUnitId, MaxSdkBase.AdInfo adInfo)
    {

        if (AdStopOn)
        {
            Time.timeScale = 0;
        }
    }

    private void OnRewardedAdFailedToDisplayEvent(string adUnitId, MaxSdkBase.ErrorInfo errorInfo, MaxSdkBase.AdInfo adInfo)
    {
        // Rewarded ad failed to display. AppLovin recommends that you load the next ad.
        LoadRewardedAd();
    }

    private void OnRewardedAdClickedEvent(string adUnitId, MaxSdkBase.AdInfo adInfo) { }

    private void OnRewardedAdHiddenEvent(string adUnitId, MaxSdkBase.AdInfo adInfo)
    {
        // Rewarded ad is hidden. Pre-load the next ad
        LoadRewardedAd();
        if (AdStopOn)
        {
            DOTween.PlayAll();
            Time.timeScale = 1;
        }

    }

    //リワード広告視聴完了
    //リワード報酬処理をここに記入
    private void OnRewardedAdReceivedRewardEvent(string adUnitId, MaxSdk.Reward reward, MaxSdkBase.AdInfo adInfo)
    {
        // The rewarded ad displayed and the user should receive the reward.




    }

    private void OnRewardedAdRevenuePaidEvent(string adUnitId, MaxSdkBase.AdInfo adInfo)
    {
        // Ad revenue paid. Use this callback to track user revenue.
    }



    //バナー広告表示メソッド
    public void BannerAd()
    {
        MaxSdk.ShowBanner(bannerAdUnitId);
    }

    //バナー広告非表示メソッド
    public void BannerHideAd()
    {
        MaxSdk.HideBanner(bannerAdUnitId);
    }

    //インターステイシャル広告表示メソッド
    public void InterstitialsAd()
    {
        if (MaxSdk.IsInterstitialReady(adInterUnitId))
        {
            if (AdStopOn)
            {
                DOTween.PauseAll();
            }
            MaxSdk.ShowInterstitial(adInterUnitId);
        }
    }

    //MREC広告表示メソッド
    public void MrecAd()
    {
        MaxSdk.ShowMRec(mrecAdUnitId);
    }

    //MREC広告非表示メソッド
    public void MrecHideAd()
    {
        MaxSdk.HideMRec(mrecAdUnitId);
    }

    //リワード広告表示メソッド
    public void RewardedAd()
    {
        if (MaxSdk.IsRewardedAdReady(adRewardUnitId))
        {
            if (AdStopOn)
            {
                DOTween.PauseAll();
            }
            MaxSdk.ShowRewardedAd(adRewardUnitId);
        }
    }

    //広告の表示状態を確認メソッド
    public bool RewardAdLoadCheck()
    {
        return MaxSdk.IsRewardedAdReady(adRewardUnitId);
    }

}
