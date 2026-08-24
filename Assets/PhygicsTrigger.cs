using System.Collections;
using System.Collections.Generic;
using UnityEditor.Rendering;
using System.Text.RegularExpressions;

using UnityEngine;

public class PhygicsTrigger : MonoBehaviour
{
    public GameObject BallOnVisual;

    public LayerMask mask;

    [SerializeField]
    public float length;

    public bool jurge = false;

    public int lastTouch = 0;
    public int i = 0;
    private string currentStairName = "";
    private Vector3 stairFirstContactPoint;

    public MainGameManager BringActiveSlope;

    private const int StairCount = 6;
    private const float StairSectionLength = 9.9f;

    private bool[] touchedSteps = new bool[StairCount];
    private int touchedStepCount = 0;

    // Start is called before the first frame update
    void Start()
    {
        // transform.GetComponent<MeshCollider>().convex = true;
     
        BallOnVisual = GameObject.Find("BallVisual");
        BringActiveSlope = GameObject.Find("GameManager").GetComponent<MainGameManager>();


    }

    // Update is called once per frame
    void Update()
    {
       
    }

    void OnTriggerEnter(Collider other)
    {
        

        if (other.transform.CompareTag("SubjectVisual") &&
            BallOnVisual.transform.GetComponent<SphereCollider>().isTrigger == true)
        {
            var match = Regex.Match(transform.name, @"\d+");

            if (match.Success && BringActiveSlope)
            {
                BringActiveSlope.lastTouch = int.Parse(match.Value);
            }
            transform.GetComponent<MeshCollider>().isTrigger = false;
           
            BallOnVisual.transform.GetComponent<SphereCollider>().isTrigger = false;
            i++;
        }
    }

    void OnCollisionEnter(Collision col)
    {


        if (transform.CompareTag("stairway"))
        {
           // Debug.Log("Superstairway"+transform.name);
        }

        if (col.transform.CompareTag("SubjectVisual"))
        {
            // transform.GetComponent<MeshCollider>().isTrigger = true;
            // col.transform.GetComponent<SphereCollider>().isTrigger = true;
        }
    }

    void OnCollisionExit(Collision col)
    {
    }
    void OnCollisionStay(Collision col)
    {
        string objectName = transform.gameObject.name;

        // 現在のログでは Render に実際のCollisionが出ているため
        // StairWay～Render を対象にする
        if (!objectName.StartsWith("StairWay") ||
            !objectName.EndsWith("Render"))
        {
            return;
        }

        if (col.contactCount == 0)
            return;

        ContactPoint contact = col.GetContact(0);

        // 新しいStairWay区間へ入った
        if (currentStairName != objectName)
        {
            currentStairName = objectName;
            stairFirstContactPoint = contact.point;

            touchedSteps = new bool[StairCount];
            touchedStepCount = 0;

            Debug.Log(
                $"[STAIR START] {objectName} 0/{StairCount}"
            );
        }

        // 入口から現在の接触点までの距離
        float distance =
            Vector3.Distance(
                stairFirstContactPoint,
                contact.point
            );

        float oneStepLength =
            StairSectionLength / StairCount;

        int stepIndex =
            Mathf.FloorToInt(
                distance / oneStepLength
            );

        stepIndex =
            Mathf.Clamp(
                stepIndex,
                0,
                StairCount - 1
            );

        // 同じ段は二重カウントしない
        if (!touchedSteps[stepIndex])
        {
            touchedSteps[stepIndex] = true;
            touchedStepCount++;

            Debug.Log(
                $"[STAIR STEP] " +
                $"{objectName} " +
                $"段={stepIndex + 1} " +
                $"接触={touchedStepCount}/{StairCount} " +
                $"point={contact.point:F3}"
            );
        }
    }
}