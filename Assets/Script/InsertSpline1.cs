using UnityEngine;
using System;
using System.Collections;
using UnityEngine.Splines;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine.UIElements;
using Unity.VisualScripting;
using UnityEditor.Rendering;

public class InsertSplinePath1 : MonoBehaviour
{
    public List<BezierKnot> splineActive;
    public List<Spline> reversedPlaneNeighbor;

    public SplineContainer DuplicatePlaneNeighbor;
    public SplineContainer CircleEmbled;

    public GameObject Player;

    GameObject CornnerFirst;

    public class CoreSpline
    {
        public Vector3 startPoint;
        public Vector3 endPoint;
        public List<float3> DirLine;
    };

    [SerializeField] public float justifyWidth = 0;

    public List<Spline> planeNeighbor;
    public List<CoreSpline> EntrySpline;

    public GameObject mainStream;

    public CoreSpline SplineHandle;
    public string chunkWord;

    public float[] LinapLine;

    public ChangeSpline[] TotalLines;

    public SplineContainer splineContainer;
    public float OneSquareWidth = 0;
    private float[] planeWidth;
    private float thirdProcess = 0;
    public int numberOfKnots;

    Vector3 startPoint;
    Vector3 endPoint;

    void Start()
    {
        LinapLine = new float[3];
        planeWidth = new float[4];
        
        OneSquareWidth=GameObject.Find("StairwaySimple").transform.Find("Plane").transform.GetComponent<MeshCollider>().bounds.size.z;
        for (int i = 0; i < 4; i++)
        {
            planeWidth[i] = i*OneSquareWidth / 4;
        }

        thirdProcess = OneSquareWidth / 4;
        
        //ConsumeExpand = GameObject.Find("BasicUI").transform.GetComponent<HandwritingManager>();

        startPoint = new Vector3(0, 0, 0);
        endPoint = new Vector3(-30, 0, 0);

        splineContainer = transform.GetComponent<SplineContainer>();
        TotalLines = GameObject.FindObjectsOfType<ChangeSpline>();

        ExtendSplineWithKnots();

        //StartCoroutine(AttachAnime());
        Player = GameObject.Find("Subject");
            
        var mainStream = GameObject.Find("MainStream");

        StartCoroutine(Delay(0.1f));
        
        
       // CreateReversedSplineClones();

        /*var aim2 = mainStream.transform.AddComponent<SplineAnimate>();
        mainStream.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
        mainStream.transform.position = Vector3.zero;
        aim2.PlayOnAwake = false;
        aim2.Container = DuplicatePlaneNeighbor;
        aim2.Duration = 256;
        mainStream.transform.SetParent(transform);
        
        
        SplineAnimate anim3 = GameObject.Find("player(Clone)").AddComponent<SplineAnimate>();
        anim3.PlayOnAwake = false;
        anim3.Container = transform.GetComponents<InsertSplinePath>()[2].splineContainer;
        anim3.Duration = 64;*/

        

       

        Debug.Log("");
        //ExtendCircle(10f);

    }

    void ExtendSplineWithKnots()
    {
        float[] justCount;
        justCount = new float[3];

        Spline spline = splineContainer.Spline;
        List<Spline> splines = new List<Spline>();
        Spline newSpline;
        spline.Clear();

        //ConsumeExpand.splineActive = new List<List<BezierKnot>>();

        float temp = 0;
        var primeSize = (endPoint.x - 2 * justifyWidth + 1) - startPoint.x;

        for (int m = 0; m < 3; m++)
        {
            if (m == 0)
            {
                newSpline = splineContainer.Splines[0];
            }
            else
            {
                newSpline = SplineUtility.AddSpline(splineContainer);
                Debug.Log("");
            }
            newSpline.Closed = false;
            var overSize = (endPoint.x - m * justifyWidth + 1) - startPoint.x;
            var HitPoint = -primeSize + overSize;
            justCount[m] = HitPoint / 4f;

            /*Vector3 AltStartPoint = new Vector3(-startPoint.x, startPoint.y+0.5f, startPoint.z - m * justifyWidth + 1+3)*/
            
            Vector3 AltStartPoint = new Vector3(-startPoint.x, startPoint.y+0.5f, (startPoint.z) - planeWidth[m]+thirdProcess);
            Vector3 AltEndPoint = new Vector3(-endPoint.x + m * justifyWidth + 1, endPoint.y, (endPoint.z)- planeWidth[m]+thirdProcess);
          

            for (int i = 0; i < numberOfKnots; i++)
            {
                float t = i / (float)(numberOfKnots - 1);
                Vector3 pos = Vector3.Lerp(AltStartPoint, AltEndPoint, t);

                BezierKnot knot = new BezierKnot(pos);
                newSpline.Add(knot);

                var radian = Mathf.Deg2Rad * 90;

                // ここから下はオマケ。折角だから進行方向を曲げてみました
                if (numberOfKnots - 1 == i)
                {
                    var OneStep = Vector3.Distance(newSpline[3].Position, newSpline[0].Position);
                    var FirstDir = math.normalize(newSpline[0].Position - newSpline[3].Position);
                    var FirtDistance = OneStep * FirstDir;
                    var OnePieceDis = (Vector3.Distance(endPoint, startPoint) / numberOfKnots) / 2;
                    var direction = (newSpline[i-1].Position - newSpline[i].Position);
                    Vector3 rotate = (Quaternion.AngleAxis(90, Vector3.up) * direction).normalized;

                    float3 newPosition = newSpline[i].Position + (float3)(rotate * OnePieceDis);

                    BezierKnot newKnot = new BezierKnot(newPosition);
                    newSpline.Add(newKnot);

                    newKnot.Position = new float3(
                        newSpline[newSpline.Count - 1].Position.x,
                        newSpline[newSpline.Count - 1].Position.y,
                        newSpline[newSpline.Count - 1].Position.z
                    );
                    newSpline[newSpline.Count - 1] = newKnot;

                    Vector3 AllInOneConner = new Vector3(newKnot.Position.x, newKnot.Position.y, newKnot.Position.z);

                    var LastPoint = rotate.normalized * (OnePieceDis * (numberOfKnots - 1));

                    for (int j = 1; j < numberOfKnots; j++)
                    {
                        float subT = j / (float)(numberOfKnots - 1);
                        var pit = AllInOneConner + rotate * (OnePieceDis) * (numberOfKnots - 1);
                        Vector3 pos2 = Vector3.Lerp(AllInOneConner, AllInOneConner + LastPoint, subT);
                        BezierKnot newKnot2 = new BezierKnot(pos2);

                        newSpline.Add(newKnot2);
                    }

                    var TwoStep = Vector3.Distance(newSpline[7].Position, newSpline[3].Position);
                    var SecondDir = math.normalize(newSpline[3].Position - newSpline[7].Position);
                    var ThreeStep = Vector3.Distance(newSpline[15].Position, newSpline[7].Position);
                    var ThreeDir = math.normalize(newSpline[7].Position - newSpline[15].Position);

                    var SecondDistance = TwoStep * SecondDir;

                    LinapLine[m] = OneStep + TwoStep + ThreeStep;
                }

                temp = t;
            }

            splines.Add(newSpline);
            var list = newSpline.Knots.ToList();

            
        }
        
        for (int i = 0; i < TotalLines.Length; i++)
        {
            TotalLines[i].LinenapLine = LinapLine;
        }
        
        planeNeighbor = splines;
        GameObject.Find("ConnerSecond").GetComponent<PowTwin2>().planeNeighbor = planeNeighbor;

    }

    void LateUpdate()
    {
        ReflectRayPlane(Input.mousePosition);

    }

    void OnEnable()
    {
        
    }

    
    
    
    float ReflectRayPlane(Vector3 reflectPoint)
    {
        float angle = 0;
        Ray ray = Camera.main.ScreenPointToRay(reflectPoint);
        Plane UnderPlane = new Plane(Vector3.up,Vector3.zero);

        if(UnderPlane.Raycast(ray,out float enter))
        {
            Vector3 hitPoint = ray.GetPoint(enter);

            float planeAngle = Vector3.Angle(UnderPlane.normal, Vector3.up);
        }
        return angle;
    }
    
   
    void CreateReversedSplineClones()
    {
        DuplicatePlaneNeighbor = transform.GetComponent<SplineContainer>();

        // 新しいリストを用意
        reversedPlaneNeighbor = new List<Spline>();
        Spline clone;

        int secCount = 0;

        // 元のスプラインたちをひとつずつ処理
        foreach (var orig in planeNeighbor)
        {
            if (secCount == 0)
            {
                clone = DuplicatePlaneNeighbor.Splines[0];
            }
            else
            {
                clone = SplineUtility.AddSpline(DuplicatePlaneNeighbor);
            }

            // 念のため初期状態をきれいに
            clone.Clear();

            // 閉じてる/閉じてない状態はコピー
            clone.Closed = orig.Closed;

            // 元スプラインの Knot をリスト化しておく
            var origKnots = orig.Knots.ToList();

            // 逆順で Knot を複製して追加
            for (int i = origKnots.Count - 1; i >= 0; i--)
            {
                var k = origKnots[i];

                // 完全クローン: 位置・イン/アウトタングent・回転をコピー
                // （BezierKnotはstructっぽい扱いだけど、念のため new して別インスタンスにする）
                var newKnot = new BezierKnot(k.Position, k.TangentIn, k.TangentOut, k.Rotation);

                clone.Add(newKnot);
            }

            // 作った逆向きスプラインを控えておく
            reversedPlaneNeighbor.Add(clone);
            secCount++;
        }
    }
    IEnumerator Delay(float time)
    {
        yield return new WaitForSeconds(time);
        
        
        SplineAnimate anim = GameObject.Find("subject").AddComponent<SplineAnimate>();
        anim.PlayOnAwake = false;
        anim.Container = splineContainer;
        anim.Duration = 64;
    }
    
}
