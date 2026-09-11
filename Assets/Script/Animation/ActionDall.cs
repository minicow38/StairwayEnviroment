using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text.RegularExpressions;
using Unity.VisualScripting;
using UnityEditor.Rendering;


public class ActionDall : MonoBehaviour
{
    public Rigidbody rb;
    public string ContactNum = "";
    public  Collider[] AroundStairway;
    public float[] resouceY;
    public List<GameObject> AroundStairwayPhysics;
    
    [Header("KnockBack")]
    public float knockBackPower = 60f;
    public float knockUpPower = 25f;
    public float knockTorque = 3f;

    private bool knockedBack = false;
    
    
    public Animator anime;
   
    // Start is called before the first frame update
    void Start()
    {
        resouceY = new float[3];
        anime = transform.GetComponent<Animator>();
        rb = GetComponent<Rigidbody>();
       
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
            
            /*Physics.IgnoreCollision(transform.GetComponent<CapsuleCollider>(), 
                GameObject.Find("InSubject").transform.GetComponent<SphereCollider>());*/
            for (int incentBall = 0;  incentBall<MainGameManager.VisualPlayerChildCollider.Length; incentBall++)
            {
                Physics.IgnoreCollision(transform.GetComponent<CapsuleCollider>(),
                    MainGameManager.VisualPlayerChildCollider[incentBall].GetComponent<SphereCollider>());
            }
            /*rb=PhyOnMob.transform.AddComponent<Rigidbody>();
            rb.isKinematic = true;
            PhyOnMob.transform.SetParent(AroundStairwayPhysics[1].transform);*/

            Debug.Log(""); 
            //transform.GetComponent<CapsuleCollider>().isTrigger = true;
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.transform.name == "InSubject")
        {
            if (knockedBack)
                return;

            knockedBack = true;

            // KnockBackアニメーション
            anime.SetBool("KnockSwitch", true);

            Rigidbody ballRb = other.attachedRigidbody;

            Vector3 knockDirection;

            if (ballRb != null && ballRb.velocity.sqrMagnitude > 0.01f)
            {
                // ボールが飛んできた方向へ、そのまま押し出す
                knockDirection = ballRb.velocity.normalized;
            }
            else
            {
                // velocityが取れない場合の保険
                knockDirection =
                    (transform.position - other.transform.position).normalized;
            }

            // 少し上方向にも吹き飛ばす
            Vector3 impulse =
                knockDirection * knockBackPower
                + Vector3.up * knockUpPower;

            rb.AddForce(
                impulse,
                ForceMode.Impulse
            );

            // 少し回転も加えると、吹き飛ばされた感じが出る
            rb.AddTorque(
                transform.right * knockTorque,
                ForceMode.Impulse
            );
        }
    }
    // Update is called once per frame
    void OnCollisionEnter(Collision col)
    {
        if (col.transform.name != "InSubject")
        {
            Match match;
            int currrentArcHit = 0;

            match = Regex.Match(AroundStairwayPhysics[1].transform.name, @"^([a-zA-Z]+)(\d*)_(\d*)_(Physics)");


            for (int arcHit = 0; arcHit < AroundStairwayPhysics.Count; arcHit++)
            {
                if (Regex.Match(AroundStairwayPhysics[arcHit].name, @"(ArcSlab)+(\d*)").Success)
                {
                    currrentArcHit = arcHit;
                    break;
                }
            }

            var localAngle = col.transform.localEulerAngles;
            transform.rotation = Quaternion.Euler(localAngle.x, localAngle.y + 180, localAngle.z);
            transform.GetComponent<CapsuleCollider>().isTrigger = true;
            rb.constraints = RigidbodyConstraints.FreezePositionY;


            int fit = 0;
        }

        Debug.Log("");
    }
    
}
