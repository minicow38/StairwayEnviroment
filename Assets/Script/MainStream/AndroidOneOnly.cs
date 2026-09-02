using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AndroidOneOnly : MonoBehaviour
{
    public static readonly string isFreshInstallLaunch = "isFreshInstallLaunch";
    public static readonly string CallForCurrrentScore = "CallForCurrrentScore";
    public static readonly string CallForCurrrentCoin = "CallForCurrrentCoin";

    public static readonly string CallForBestScore = "CallForBestScore";

    public MainGameManager mainGameManager;

    public CorrespondSubject mainDrive;
    public static int pharseCoin = 0;
    public static int currentScore = 0;
    public static int bestScore = 0;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void InitOncePerLaunch()
    {
        Debug.Log("[InitOncePerLaunch]");

        currentScore = PlayerPrefs.GetInt(CallForCurrrentScore);
        bestScore=PlayerPrefs.GetInt(CallForBestScore);
        pharseCoin = PlayerPrefs.GetInt(CallForCurrrentCoin);
        

        if (PlayerPrefs.GetInt(isFreshInstallLaunch) == 0)
        {
        }
    }

    void Start()
    {
        StartCoroutine(GameManagerStandBySystem());
    }

    IEnumerator GameManagerStandBySystem()
    {
        yield return new WaitForSeconds(1f);
        mainDrive = GameObject.Find("subject").transform.GetComponent<CorrespondSubject>();
        mainGameManager= GameObject.Find("GameManager").transform.GetComponent<MainGameManager>();

    }
    
    void OnApplicationQuit()
    {
       
        PlayerPrefs.SetInt(CallForCurrrentScore, mainDrive.PointToPlane);
        PlayerPrefs.SetInt(CallForCurrrentCoin, MainGameManager.Coin);


        if (mainDrive.PointToPlane >bestScore)
        {
            PlayerPrefs.SetInt(CallForBestScore, mainDrive.PointToPlane);
        }

        PlayerPrefs.Save();

        Debug.Log(
            $"Pause時に保存: PointToPlane={mainDrive.PointToPlane}"
        );
    }
}