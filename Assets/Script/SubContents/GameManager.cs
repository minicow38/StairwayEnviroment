using System.Collections;
using System.Linq;
using System.Text.RegularExpressions;
using NUnit.Framework.Constraints;
using UnityEngine;
using TMPro;
public class MainGameManager : MonoBehaviour
{
    public int CheckingStairwaySegment = 0;

    public CoreStepInsertSplinePathNatural PiercingSpiral;

    public static CorrespondSubject reSubject;
    public TextMeshProUGUI displayScore;
    public TextMeshProUGUI displayCoin;
    
    public static CockpitGauge RightDragOnMeter;
    public static CockpitGauge2 LeftOneTapMeter;


    public static GameObject Userbility;
    public static GameObject TopLiteral;
    public static GameObject PlayButton;
    public static GameObject TopTitle;
    public static GameObject DropOut;
    public static Collider[] VisualPlayerChildCollider;
    
    
    public static int PointToPlane;
    
    public static GameObject PreviewIconRoot;
    public static int CurrentPointToPlane = 0;
    public static int CurrentCoin = 0;
    public static int lastTouch = 0;
    public static int Coin = 0;
    public static GameObject HopingSwipeTurn;

    public static bool OnInifnite = false;
    public static bool OnDead = false;
    public static bool OpenChunkStage = false;
    public static SlopeStickCore core;
    public bool initRotation = false;
    public Vector3 VisualRoatationPlayer;


    public static int LimitTouchingphase = 4;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        Coin = AndroidOneOnly.pharseCoin;
        var width = Screen.width;
        LimitTouchingphase = 0;
        var fat = GameObject.FindObjectsOfType<SphereSpikes>();
        PointToPlane = CurrentPointToPlane;
        core = GameObject.Find("InSubject").transform.GetComponent<SlopeStickCore>();
        var array=GameObject.Find("VisualPlayerRoot").transform.GetComponentsInChildren<SphereCollider>();
        VisualPlayerChildCollider = array
             .Where(x => x != null && x.transform.name != "subject")
             .ToArray();
        reSubject= GameObject.Find("VisualPlayerRoot/subject").transform.GetComponent<CorrespondSubject>();
        PiercingSpiral=GameObject.Find("StairwaySimple/MainStream").GetComponent<CoreStepInsertSplinePathNatural>();

        HopingSwipeTurn = GameObject.Find("GameUI/RightTurn").transform.gameObject;
        TopTitle= GameObject.Find("GameUI/Title").transform.gameObject;
       PreviewIconRoot= GameObject.Find("GameUI/PreviewIconRoot").transform.gameObject;
        TopLiteral = GameObject.Find("GameUI/TopLiteral").transform.gameObject;
        PlayButton= GameObject.Find("GameUI/PlayButton").transform.gameObject;
        DropOut=GameObject.Find("GameUI/DropOut").transform.gameObject;
       // PreviewIconRoot.transform.Find("CurrentCoin").transform.GetComponent<TextMeshProUGUI>().text = Coin.ToString();
        TopLiteral.transform.Find("Score").transform.GetChild(0).GetComponent<TextMeshProUGUI>().text = AndroidOneOnly.currentScore.ToString("");
        
        //RightDragOnMeter = GameObject.Find("SegmentLVInGauge").transform.GetComponent<CockpitGauge>();
        LeftOneTapMeter = GameObject.Find("GameUI/StairwayUserbility/CockpitGauge ").transform.GetComponent<CockpitGauge2>();
        
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 60;
        HopingSwipeTurn.SetActive(false);
        DropOut.SetActive(false);

       // TopLiteral.transform.Find("Best").transform.GetChild(0).GetComponent<TextMeshProUGUI>().text = AndroidOneOnly.bestScore.ToString("");
        displayScore = GameObject.Find("StairwayUserbility/Score/").transform.GetComponent<TextMeshProUGUI>();
        displayCoin = GameObject.Find("StairwayUserbility/Coin/").transform.GetComponent<TextMeshProUGUI>();
        Userbility = GameObject.Find("GameUI/StairwayUserbility").transform.gameObject;
        
       

        Userbility.transform.gameObject.SetActive(false);
        displayScore.text = AndroidOneOnly.currentScore.ToString("");
        Material activeMaterial =
            Resources.Load<Material>(
                "BallCollections/" + AndroidOneOnly.activeBallMaterial
            );
        
        GameObject.Find("VisualPlayerRoot/BallVisualEqualizer")
            .GetComponent<MeshRenderer>()
            .material = new Material(activeMaterial);

       
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (Coin != CurrentCoin)
        {
            displayCoin.text =Coin.ToString("");
            CurrentCoin = Coin;

        }
        if (LimitTouchingphase <lastTouch)
        {
            var del=lastTouch-LimitTouchingphase-1;
            OpenChunkStage = true;
            
            LimitTouchingphase = LimitTouchingphase + 8 + del;
            PiercingSpiral.ModifyOverrap = del;
        }
        if (CurrentPointToPlane != reSubject.PointToPlane)
        {
            
            displayScore.text = reSubject.PointToPlane.ToString("");
            CurrentPointToPlane=reSubject.PointToPlane;
            
        }
    }

    
    
}
