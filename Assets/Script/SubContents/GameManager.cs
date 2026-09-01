using UnityEngine;
using TMPro;
public class MainGameManager : MonoBehaviour
{
    public int CheckingStairwaySegment = 0;

    public CoreStepInsertSplinePathNatural PiercingSpiral;

    public CorrespondSubject mainDrive;
    public TextMeshProUGUI displayScore;
    public TextMeshProUGUI displayCoin;

    public GameObject TopLiteral;
    public GameObject PlayButton;
    public GameObject TopTitle;
    public GameObject PreviewIconRoot;
    public int CurrentPointToPlane = 0;
    public int lastTouch = 0;
    public int Coin = 0;
    public bool OpenChunkStage = false;

    public int LimitTouchingphase = 4;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        Coin = AndroidOneOnly.pharseCoin;
        var width = Screen.width;
        LimitTouchingphase = 0;
        CurrentPointToPlane = CurrentPointToPlane;
        mainDrive= GameObject.Find("VisualPlayerRoot/subject").transform.GetComponent<CorrespondSubject>();
        PiercingSpiral=GameObject.Find("StairwaySimple/MainStream").GetComponent<CoreStepInsertSplinePathNatural>();
        
        TopTitle= GameObject.Find("GameUI/Title").transform.gameObject;
        PreviewIconRoot= GameObject.Find("GameUI/PreviewIconRoot").transform.gameObject;
        TopLiteral = GameObject.Find("GameUI/TopLiteral").transform.gameObject;
        PlayButton= GameObject.Find("GameUI/PlayButton").transform.gameObject;
        
        TopLiteral.transform.Find("Score").transform.GetChild(0).GetComponent<TextMeshProUGUI>().text =
            AndroidOneOnly.currentScore.ToString("");

        TopLiteral.transform.Find("Best").transform.GetChild(0).GetComponent<TextMeshProUGUI>().text =
            AndroidOneOnly.bestScore.ToString("");
        displayScore = GameObject.Find("StairwayUserbility/Score/").transform.GetComponent<TextMeshProUGUI>();
        displayCoin = GameObject.Find("StairwayUserbility/Coin/").transform.GetComponent<TextMeshProUGUI>();
        Debug.Log("");
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (LimitTouchingphase <lastTouch)
        {
            var del=lastTouch-LimitTouchingphase-1;
            OpenChunkStage = true;

            LimitTouchingphase = LimitTouchingphase + 8 + del;
            PiercingSpiral.ModifyOverrap = del;
        }
        if (CurrentPointToPlane != mainDrive.PointToPlane)
        {
            displayScore.text = mainDrive.PointToPlane.ToString("");
            displayCoin.text = transform.GetComponent<MainGameManager>().Coin.ToString("");
        }
    }
}
