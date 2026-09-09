using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text.RegularExpressions;
using UnityEditor.Rendering;


public class DirectionDall : MonoBehaviour
{
    public string ContactNum = "";
    public  Collider[] AroundStairway;
    public List<GameObject> AroundStairwayRenderer;
   
    // Start is called before the first frame update
    void Start()
    {
        
    }

     void FixedUpdate()
    {
        if (AroundStairway.Length == 0)
        {
            AroundStairway = Physics.OverlapSphere(transform.position, 5f);
            for (int i = 0; i < AroundStairway.Length; i++)
            {
                if (Regex.Match(AroundStairway[i].name, @"^(\w*)(\d*)_(\d*)_(Physics)").Success)
                {
                    Match match = Regex.Match(AroundStairway[i].name, @"^(\w*)(\d*)_(\d*)_(Physics)");

                    AroundStairwayRenderer.Add(AroundStairway[i].transform.gameObject);
                    
                }
            }

            Debug.Log("");
        }
    }

   
    // Update is called once per frame
    void OnCollisionEnter(Collision col)
    {
        Match match = Regex.Match(col.transform.name, @"^(\w*)(\d*)_(\d*)_(Physics)");
        var AroundStairway = Physics.OverlapSphere(col.transform.position, 5f);
        for (int i = 0; i < AroundStairway.Length; i++)
        {
            if (AroundStairway[i].transform.name == col.transform.name)
            {
               var dir= AroundStairway[i].transform.position - AroundStairway[i - 1].transform.position;
               var objName=match.Groups[1].Value + "_" + match.Groups[3].Value+"_"+ ("Render");

               Debug.Log("");
            }
        }
        Debug.Log("");
        if (match.Success)
        {
            var objName=match.Groups[1].Value + "_" + match.Groups[3].Value+"_"+ ("Render");
            
           var ExCahngeObj=GameObject.Find(objName);
            int step = 0;
        }
        Debug.Log("");
    }
    
}
