using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CenterRotate : MonoBehaviour
{
    private int count = 0;
    public GameObject Player;
    public Quaternion rot;
    public Vector3 IntervalFlickDir;
    public Vector3 ArchInclineFlickDir;
    public GameObject center;
    private bool FirstClick = false;

    public bool AttachementInRotate = false;
    // Start is called before the first frame update
    
    void Start()
    {
        //IntervalFlickDir=(GameObject.Find("Plane").transform.position - GameObject.Find("Plane (14)").transform.position).normalized;
        Debug.Log("");
        StartCoroutine(temporary());
    }

    IEnumerator temporary()
    {
        yield return new WaitForSeconds(0.25f);
        center=GameObject.Find("Center1");
        Player = GameObject.Find("subject");
    }
    
    // Update is called once per frame
    void Update()
    {
        if (IntervalFlickDir != ArchInclineFlickDir)
        {
            if (FirstClick == true)
            {
                if (IntervalFlickDir.x <=0)
                {
                    rot = Quaternion.AngleAxis(90, Vector3.up);
                    transform.rotation = rot * transform.rotation;
                    Quaternion playerRotationBefore = Player.transform.rotation;

                    MeshRenderer mr = Player.GetComponent<MeshRenderer>();
                    if (mr != null)
                    {
                        mr.material.color = Color.blue;
                    }
                    Debug.Log("Right-turn");
                    Vector3 offset = Player.transform.position - center.transform.position;
                    Vector3 rotatedOffset = rot*offset;
                    Player.transform.position = center.transform.position + rotatedOffset;

                    count++;
                }
                else
                {
                    rot = Quaternion.AngleAxis(-90, Vector3.up);
                    Quaternion playerRotationBefore = Player.transform.rotation;

                    transform.rotation = rot * transform.rotation;
                    Debug.Log("Left-turn");

                    MeshRenderer mr = Player.GetComponent<MeshRenderer>();
                    if (mr != null)
                    {
                        mr.material.color = Color.blue;
                    }

                    Vector3 offset = Player.transform.position - center.transform.position;
                    Vector3 rotatedOffset = rot*offset;
                    Player.transform.position = center.transform.position + rotatedOffset;
                    //playerRotationBefore=Quaternion.Euler(-90,0,0);

                    count--;
                }
            }

            AttachementInRotate = true;
            FirstClick = true;
            ArchInclineFlickDir = IntervalFlickDir;
        }
       
    }
}
