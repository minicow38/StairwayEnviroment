using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Implementation : MonoBehaviour
{
    public static Implementation Instance { set; get; }

    /// <summary>
    /// Initialize the ads
    /// </summary>
    void Awake()
    {
        if (Instance == null)
            Instance = this;
        else
            Destroy(gameObject);

        DontDestroyOnLoad(this.gameObject);

        Advertisements.Instance.Initialize();
    }


    void Start()
    {
        ShowBanner();
    }

    /// <summary>
    /// Show banner, assigned from inspector
    /// </summary>
    public void ShowBanner()
    {
        Advertisements.Instance.ShowBanner(BannerPosition.BOTTOM);
    }

    public void HideBanner()
    {
        Advertisements.Instance.HideBanner();
    }


    /// <summary>
    /// Show Interstitial, assigned from inspector
    /// </summary>
    public void ShowInterstitial()
    {
        Advertisements.Instance.ShowInterstitial();
    }

    /// <summary>
    /// Show rewarded video, assigned from inspector
    /// </summary>
    public void ShowRewardedVideo()
    {
        Advertisements.Instance.ShowRewardedVideo(CompleteMethod);
    }


    /// <summary>
    /// This is for testing purpose
    /// </summary>
    void Update()
    {
        
    }

    private void CompleteMethod(bool completed)
    {
        if (completed)
        {
           
        }

        
    }
}
