using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UIScript1 : MonoBehaviour
{
    private GameObject []lineObj;
    public GameObject CrossLine;
    [SerializeField]public int sep1;
    [SerializeField]public int sep2;
    [SerializeField]public int offsetX;
    [SerializeField]public int offsetY;
    // Start is called before the first frame update
    void Start()
    {
         lineObj = new GameObject[10];
         CreateLineRenderer(10);

    }

    // Update is called once per frame
    void Update()
    {

    }
    void CreateLineRenderer(int count)
        {
            LineRenderer[] line;
            line = new LineRenderer[2];
    
            var x = Screen.width;
            var y = Screen.height;
            var z = Camera.main.nearClipPlane + 8;
    
            /*var start1 = Camera.main.ScreenToWorldPoint(new Vector3(x * sep1/sep2, 0, z));
            var end1 = Camera.main.ScreenToWorldPoint(new Vector3(x *sep1/sep2, y, z));
    
            var start2 = Camera.main.ScreenToWorldPoint(new Vector3(0, y / 2, z));
            var end2 = Camera.main.ScreenToWorldPoint(new Vector3(x, y / 2, z));*/






            for (int col = 0; col < 6; col++)
            {
                for (int i = 0; i < 2; i++)
                {
                    lineObj[i] = new GameObject("LineObj" + i);
                }

                for (int i = 0; i < line.Length; i++)
                {
                    var start1 = Camera.main.ScreenToWorldPoint(new Vector3(x * (col + 1) / sep2+offsetX, offsetY, z));
                    var end1 = Camera.main.ScreenToWorldPoint(new Vector3(x * (col + 1) / sep2+offsetX, y+offsetY, z));

                    var start2 = Camera.main.ScreenToWorldPoint(new Vector3(offsetX, y * (col + 1) / sep2+offsetY, z));
                    var end2 = Camera.main.ScreenToWorldPoint(new Vector3(x+offsetX, y * (col + 1) / sep2+offsetY, z));

                    var points = new Vector3[][]
                    {
                        new Vector3[] { start1, end1 },
                        new Vector3[] { start2, end2 },
                    };
                    var activeLine = lineObj[i].AddComponent<LineRenderer>();

                    lineObj[i].transform.SetParent(CrossLine.transform);

                    activeLine.positionCount = 2;
                    activeLine.startWidth = activeLine.endWidth = 0.05f;

                    if (i != 2)
                    {
                        activeLine.startColor = activeLine.endColor = Color.white;
                    }

                    activeLine.material = new Material(Shader.Find("Sprites/Default"));
                    transform.SetParent(lineObj[i].transform);
                    activeLine.SetPositions(points[i]);
                }
            }



            /*
            var centerMark = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            centerMark.transform.position = Camera.main.ScreenToWorldPoint(new Vector3(x / 2, y / 2, z));
            centerMark.transform.localScale = new Vector3(0.25f, 0.25f, 0.25f);
            */
    
            ;
        }
}
