using UnityEngine;
using TMPro;
public class MainGameManager : MonoBehaviour
{
    public int CheckingStairwaySegment = 0;

    public CorrespondSubject mainDrive;
    public TextMeshProUGUI uGUI;
    public int CurrentPointToPlane = 0;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        var width = Screen.width;
        CurrentPointToPlane = CurrentPointToPlane;
        mainDrive= GameObject.Find("VisualPlayerRoot/subject").transform.GetComponent<CorrespondSubject>();
        Debug.Log("");
        uGUI = GameObject.Find("StairwayUserbility/Score/").transform.GetChild(0).transform.GetComponent<TextMeshProUGUI>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (CurrentPointToPlane != mainDrive.PointToPlane)
        {
            uGUI.text = mainDrive.PointToPlane.ToString("");
        }
    }
}
