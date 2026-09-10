using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text.RegularExpressions;
using Unity.VisualScripting;
using UnityEditor.Rendering;


public class DirectionDall : MonoBehaviour
{
    public Rigidbody rb;
    public string ContactNum = "";
    public  Collider[] AroundStairway;
    public List<GameObject> AroundStairwayPhysics;
   
    // Start is called before the first frame update
    void Start()
    {
        
    }

     void FixedUpdate()
    {
        if (AroundStairway.Length == 0)
        {
            AroundStairway = Physics.OverlapSphere(transform.position, 6f);
            for (int i = 0; i < AroundStairway.Length; i++)
            {
                if (Regex.Match(AroundStairway[i].name, @"([a-zA-Z]+)(\d*)_(\d*)_(Physics)").Success)
                {
                    Match match = Regex.Match(AroundStairway[i].name, @"([a-zA-Z]+)(\d*)_(\d*)_(Physics)");
                    AroundStairwayPhysics.Add(AroundStairway[i].transform.gameObject);
                    
                }
            }

           /* if (AroundStairwayRenderer.Count== 1)
            {
                Physics.IgnoreCollision(transform.GetComponent<CapsuleCollider>(),
                    AroundStairwayRenderer[0].transform.GetComponent<MeshCollider>());
            }
            else
            {
                Physics.IgnoreCollision(transform.GetComponent<CapsuleCollider>(),
                    AroundStairwayRenderer[1].transform.GetComponent<MeshCollider>());
            }*/
            
            Physics.IgnoreCollision(transform.GetComponent<CapsuleCollider>(), 
                GameObject.Find("InSubject").transform.GetComponent<SphereCollider>());
            for (int incentBall = 0;  incentBall<MainGameManager.VisualPlayerChildCollider.Length; incentBall++)
            {
                Physics.IgnoreCollision(transform.GetComponent<CapsuleCollider>(),
                    MainGameManager.VisualPlayerChildCollider[incentBall].GetComponent<SphereCollider>());
            }
            /*rb=PhyOnMob.transform.AddComponent<Rigidbody>();
            rb.isKinematic = true;
            PhyOnMob.transform.SetParent(AroundStairwayPhysics[1].transform);*/

            Debug.Log("");
        }
    }

   
    // Update is called once per frame
    void OnCollisionEnter(Collision col)
    {
        Match match;
        int currrentArcHit = 0;
       
        match = Regex.Match(AroundStairwayPhysics[1].transform.name, @"^([a-zA-Z]+)(\d*)_(\d*)_(Physics)");

        for (int arcHit = 0; arcHit<AroundStairwayPhysics.Count; arcHit++)
        {
            if (Regex.Match(AroundStairwayPhysics[arcHit].name, @"ArchSlab.*").Success)
            {
                currrentArcHit = arcHit;
                break;
            }
        }

        Vector3 direction;
        if (currrentArcHit == 0)
        {
            direction = (AroundStairwayPhysics[1].transform.position - AroundStairwayPhysics[0].transform.position);
        }
        else
        {
            direction = (AroundStairwayPhysics[0].transform.position - AroundStairwayPhysics[1].transform.position);
        }

        if (direction.x == 0)
        {
            transform.rotation = Quaternion.Euler(0, -180, 0);
        }

        Debug.Log("");
        if(direction.z==0 && direction.x>0)
        {
            transform.rotation = Quaternion.Euler(0, 90, 0);

        }else
        {
            transform.rotation = Quaternion.Euler(0, -90, 0);

        }
      


        Debug.Log("");
    }
    
}
