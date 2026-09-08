using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AndroidOneOnly : MonoBehaviour
{
    public static readonly string isFreshInstallLaunch = "isFreshInstallLaunch";
    public static readonly string CallForCurrrentScore = "CallForCurrrentScore";
    public static readonly string CallForCurrrentCoin = "CallForCurrrentCoin";
    public static readonly string itemList = "itemList";

    public static readonly string CallForBestScore = "CallForBestScore";

    public MainGameManager mainGameManager;

    public CorrespondSubject mainDrive;
    public static int pharseCoin = 0;
    public static int currentScore = 0;
    public static int bestScore = 0;
    public static string MiddleItemList;
    public static List<int> LinenapItemList;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void InitOncePerLaunch()
    {

        LinenapItemList = new List<int>();
        Debug.Log("[InitOncePerLaunch]");
        currentScore = PlayerPrefs.GetInt(CallForCurrrentScore);
        bestScore=PlayerPrefs.GetInt(CallForBestScore);
        pharseCoin = PlayerPrefs.GetInt(CallForCurrrentCoin);
        MiddleItemList = PlayerPrefs.GetString(itemList);
        if (MiddleItemList == "")
        {
            for (int i = 0; i < 30; i++)
            {
                int num = 0;
                LinenapItemList.Add(0);
                
            }

            Debug.Log("");
        }
        else
        {
            for (int i = 0; i < MiddleItemList.Length; i++)
            {
                LinenapItemList.Add(MiddleItemList[i] - '0');
            }

            Debug.Log("");
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

        string stringPlus = "";
        for (int i = 0; i < LinenapItemList.Count; i++)
        {
            stringPlus += LinenapItemList[i].ToString();
        }
        PlayerPrefs.SetString(itemList,stringPlus);

        PlayerPrefs.Save();

        Debug.Log(
            $"Pause時に保存: PointToPlane={mainDrive.PointToPlane}"
        );
    }
}