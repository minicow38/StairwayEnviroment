using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Text.RegularExpressions;
using TMPro;
using GoogleMobileAds.Api;
using Firebase.Analytics;
using System.Collections;

public class AdManager : MonoBehaviour
{
    public static AdManager instance;

  
    


    private string RewardedAdType;

    void Awake()
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

    void Start()
    {
        //FirebaseAnalytics.LogEvent("firebase_check");

    }

   


    //インターステイシャル表示判定
    //インターステイシャル表示判定
    public void InterstitialsAdStart()
    {
        
        //インタースティシャル広告広告表示
        FireBaseAdConversion.instance.InterstitialsAd();
    }


    //リワード広告表示
    public void RewardedAdStart(string type)
    {
        //リワード広告のタイプを保存
        RewardedAdType = type;
        Firebase.Analytics.FirebaseAnalytics.LogEvent("InterstitialsStart");

        //リワード広告の表示
        FireBaseAdConversion.instance.RewardedAd();
        //RewardedAdEnd();
    }

}

