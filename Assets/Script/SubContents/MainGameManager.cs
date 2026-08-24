using UnityEngine;
using TMPro;
public class MainGameManager : MonoBehaviour
{
    public int CheckingStairwaySegment = 0;

    public CoreStepInsertSplinePathNatural PiercingSpiral;

    public CorrespondSubject mainDrive;
    public TextMeshProUGUI uGUI;
    public int CurrentPointToPlane = 0;
    public int lastTouch = 0;
    public bool OpenChunkStage = false;

    public int LimitTouchingphase = 4;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        var width = Screen.width;
        CurrentPointToPlane = CurrentPointToPlane;
        mainDrive= GameObject.Find("VisualPlayerRoot/subject").transform.GetComponent<CorrespondSubject>();
        PiercingSpiral=GameObject.Find("StairwaySimple/MainStream").GetComponent<CoreStepInsertSplinePathNatural>();
        Debug.Log("");
        uGUI = GameObject.Find("StairwayUserbility/Score/").transform.GetChild(0).transform.GetComponent<TextMeshProUGUI>();
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
            uGUI.text = mainDrive.PointToPlane.ToString("");
        }
    }
}
