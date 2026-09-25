using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text.RegularExpressions;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEditor.Rendering;


public class ActionDall : MonoBehaviour
{
    public Rigidbody rb;
    public string ContactNum = "";
    public Collider[] AroundStairway;
    public Collider[] ExpendScopePlayer;
    public float[] resouceY;
    public Transform DollAttachGrond;
    public List<GameObject> AroundStairwayPhysics;
    public Vector3 angle;
    [Header("KnockBack")]
    public float knockBackPower = 60f;
    public float knockUpPower = 25f;
    public float knockTorque = 3f;

    public bool StimulateSeekingAnimation = false;
    public bool NonComeBackPhysicX = false;
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
                if (Regex.Match(AroundStairway[i].name, @"([a-zA-Z]+)(\d*)_(\d*)_(Render)").Success)
                {
                    Match match = Regex.Match(AroundStairway[i].name, @"([a-zA-Z]+)(\d*)_(\d*)_(Render)");
                    AroundStairwayPhysics.Add(AroundStairway[i].transform.gameObject);
                    var turn_name=""+ match.Groups[1] + match.Groups[2] + "_" + match.Groups[3] + "_" + "Physics";
                    var obj=GameObject.Find("" + turn_name).transform.gameObject;
                   angle=obj.transform.localEulerAngles;
                   transform.rotation = Quaternion.Euler(0, angle.y, 0);
                   //rb.constraints = RigidbodyConstraints.FreezePositionY;
                    Debug.Log("");

                }
                
            }
            
            for (int incentBall = 0;  incentBall<MainGameManager.VisualPlayerChildCollider.Length; incentBall++)
            {
                Physics.IgnoreCollision(transform.GetComponent<CapsuleCollider>(),
                    MainGameManager.VisualPlayerChildCollider[incentBall].GetComponent<SphereCollider>());
            }
        }
        else
        {
            ExpendScopePlayer = Physics.OverlapSphere(transform.position, 12f);

            foreach (Collider AnotherNascent in ExpendScopePlayer)
            {
                if (AnotherNascent.transform.CompareTag("SubjectVisual"))
                {
                    transform.rotation = Quaternion.Euler(0, angle.y, 0);
                    if (!StimulateSeekingAnimation)
                    {

                        DollAttachGrond.transform.GetComponent<MeshCollider>().enabled = true;
                        
                        StimulateSeekingAnimation = true;
                    }

                }
            }

        }
    }
     
   

    
    void OnTriggerEnter(Collider other)
    {
        if (other.transform.name == "subject")
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
    }void OnCollisionEnter(Collision col)
    {
        if (col.transform.CompareTag("plane")&&!NonComeBackPhysicX)
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
            transform.GetComponent<CapsuleCollider>().isTrigger = true;
            var localAngle = col.transform.localEulerAngles;
            transform.rotation = Quaternion.Euler(localAngle.x, localAngle.y + 180, localAngle.z);
            DollAttachGrond = col.transform;

            NonComeBackPhysicX = true;


            int fit = 0;
        }
        rb.constraints = RigidbodyConstraints.FreezePositionY;
    }
    // Update is called once per frame
    
    
}
