using System.Collections;
using System.Collections.Generic;
using UnityEngine.Splines;
using Unity.Mathematics;
using UnityEngine;

public class ChangeSpline : MonoBehaviour
{


    Vector3 currentPos;
    float closestT;
    Vector3 targetPos;
    public int laneMove = 0;
    public GameObject Player;
    public float[] LinenapLine;
    public GameObject CornnerGroup;
    public Vector3 MemorizePlayerPos;
    [SerializeField] float speed = 8;
    [SerializeField] private SplineAnimate follower;
    [SerializeField] private SplineContainer innerSpline;
    [SerializeField] public InsertSplinePath DisSpline;

    // Start is called before the first frame update
    IEnumerator Start()
    {
        //var all=GameObject.FindObjectsOfType<SplineAnimate>();
        closestT = 1;
        //FixHandwrite = GameObject.Find("BasicUI").transform.GetComponent<HandwritingManager>();
        Player = transform.gameObject;
        // Vector3 currentPos = transform.position;
        yield return new WaitForEndOfFrame();
        innerSpline = CornnerGroup.transform.GetComponent<SplineContainer>();

       // DisSpline = CornnerGroup.transform.GetComponent<InsertSplinePath>();

        Debug.Log("");
        if (transform.name == "MainStream")
        {
           // var pat = DisSpline.LinapLine1;
            currentPos = transform.position;
            var fit = innerSpline.Splines[laneMove % 3];
            MemorizePlayerPos = Player.transform.position;

            float t = FindNearestT(fit, currentPos);

            float3 nerestPoint;
            var targetPos = SplineUtility.EvaluatePosition(innerSpline.Splines[(laneMove + 1) % 3], t);
            /*if (!FixHandwrite.FirstLine)
            {
                currentPos = new Vector3(targetPos.x, targetPos.y, targetPos.z);
            }
            if (!FixHandwrite.FirstLine)
            {
                currentPos = new Vector3(targetPos.x, targetPos.y, targetPos.z);
            }
            FixHandwrite.FirstLine = true;*/

            StartCoroutine(SmoothSwitch(currentPos, targetPos, innerSpline, 0));
        }
     
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Z))
        {
            //var pat = DisSpline.LinapLine1;
            currentPos = transform.position;
            var fit = innerSpline.Splines[laneMove % 3];
            MemorizePlayerPos = Player.transform.position;

            float t = FindNearestT(fit, currentPos);

            float3 nerestPoint;
            //SplineUtility.GetNearestPoint(fit, currentPos,out nerestPoint,t);
            var targetPos = SplineUtility.EvaluatePosition(innerSpline.Splines[(laneMove + 1) % 3], t);
            currentPos = new Vector3(targetPos.x, targetPos.y, targetPos.z);


            StartCoroutine(SmoothSwitch(currentPos, targetPos, innerSpline, t));
        }

        if (Input.GetKeyDown(KeyCode.X))
        {
            currentPos = transform.position;


        }
        if (Input.GetKeyDown(KeyCode.F))
        {
            transform.GetComponent<SplineAnimate>().Play();
        }
        if (Input.GetKey(KeyCode.G))
        {
            transform.GetComponent<SplineAnimate>().Pause();
        }
    }
    public float FindNearestT(Spline spline, Vector3 worldPos)
    {
        float closestT = 0f;
        int resolution = 100;
        Vector3 PinPoint = Vector3.zero;
        ///float t = 0f;
        float minDistance = float.MaxValue;
        Vector3 localPos = innerSpline.transform.InverseTransformPoint(worldPos);
        for (int i = 0; i <= resolution; i++)
        {
            float t = i / (float)resolution;
            transform.Find("pintman ");
            //Vector3 pos = SplineUtility.EvaluatePosition(innerSpline, t);

            Vector3 point = (Vector3)SplineUtility.EvaluatePosition(spline, t);




            float dist = (point - localPos).sqrMagnitude;

            if (dist < minDistance)
            {
                //��ԋ߂��ꏊ�Ƀq�b�g��������Ηǂ��̂Ń��C��������Ă��K���Ȃ̂�������
                PinPoint = point;
                minDistance = dist;
                closestT = t;
            }
        }

        return closestT;
    }
    IEnumerator SmoothSwitch(Vector3 from, Vector3 to, SplineContainer newSpline, float newT)
    {
        float time = 0f;


        yield return null;


        follower = transform.GetComponent<SplineAnimate>();
        var Duration = transform.GetComponent<SplineAnimate>().Duration;
        var v1 = (float)(LinenapLine[0] + LinenapLine[1] + LinenapLine[2]) / Duration;
        //(laneM  v1;
        float totalRate = 0;
        float nextRate = 0;
        int i = 0;

        for (i = 0; i < (laneMove + 1) % 3; i++)
        {
            totalRate += LinenapLine[i];
        }
        nextRate = LinenapLine[(i) % 3] * newT;

        var stack_rest = totalRate;

        var fest = ((stack_rest + nextRate) / Duration) / v1;
        /*if (fest<0.95f && fest<1)
        {
            fest = 0;
        }*/
        
        follower.NormalizedTime = fest;

        laneMove++;

    }
}
