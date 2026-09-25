using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AndroidOneOnly : MonoBehaviour
{
    public static readonly string isFreshInstallLaunch = "isFreshInstallLaunch";
    public static readonly string CallForCurrrentScore = "CallForCurrrentScore";
    public static readonly string CallForCurrrentCoin = "CallForCurrrentCoin";
    public static readonly string itemList = "itemList";

    //public static readonly string BallNumber = "ActiveUselessBall";
    public static readonly string ArchiveBallItem = "SettingBallItem";

    public static readonly string CallForBestScore = "CallForBestScore";
    

    public MainGameManager mainGameManager;

    public CorrespondSubject mainDrive;
    
    public GameObject[] BackGrounds;
    public static int pharseCoin = 0;
    public static int currentScore = 0;
    public static int UsingBallNow=0;
    public static int bestScore = 0;
    public static string MiddleItemList;
    public static string activeBallMaterial;
    public static List<int> LinenapItemList;
    
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void InitOncePerLaunch()
    {

        LinenapItemList = new List<int>();
        Debug.Log("[InitOncePerLaunch]");
        currentScore = PlayerPrefs.GetInt(CallForCurrrentScore,0);
        bestScore=PlayerPrefs.GetInt(CallForBestScore,0);
        pharseCoin = PlayerPrefs.GetInt(CallForCurrrentCoin,0);
        //UsingBallNow = PlayerPrefs.GetInt(BallNumber,0);
        activeBallMaterial = PlayerPrefs.GetString(ArchiveBallItem, "Airman");
        
        MiddleItemList = PlayerPrefs.GetString(itemList,"");
        
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

        Debug.Log("");

    }

    void Start()
    {
       // StartCoroutine(GameManagerStandBySystem());
    }

   /* IEnumerator GameManagerStandBySystem()
    {
        yield return new WaitForSeconds(1f);
        if(GameObject.Find("subject")!=null)
        mainDrive = GameObject.Find("subject").transform.GetComponent<CorrespondSubject>();
        if(GameObject.Find("GameManager")!=null)
        mainGameManager= GameObject.Find("GameManager").transform.GetComponent<MainGameManager>();

    }*/
    
    void OnApplicationFocus(bool hasFocus)
    {
        if (!hasFocus)
        {
            PlayerPrefs.SetInt(CallForCurrrentScore, MainGameManager.PointToPlane);
            PlayerPrefs.SetInt(CallForCurrrentCoin, MainGameManager.Coin);


            if (MainGameManager.PointToPlane > bestScore)
            {
                PlayerPrefs.SetInt(CallForBestScore, MainGameManager.PointToPlane);
            }

            string stringPlus = "";
            for (int i = 0; i < LinenapItemList.Count; i++)
            {
                stringPlus += LinenapItemList[i].ToString();
            }

            PlayerPrefs.SetString(itemList, stringPlus);
            PlayerPrefs.SetString(ArchiveBallItem, activeBallMaterial);
            PlayerPrefs.SetInt(CallForCurrrentCoin, pharseCoin);
            //PlayerPrefs.SetInt(BallNumber,UsingBallNow);



            PlayerPrefs.Save();

            Debug.Log(
                $"Pause時に保存: PointToPlane={MainGameManager.PointToPlane}"
            );
        }
    }
}