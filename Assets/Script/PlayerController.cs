using System.Collections.Generic;
using UnityEngine;
using System.Collections;

public class PlayerController : MonoBehaviour
{
    private Rigidbody rb;

    public float multi_pow = 0;

    public float IndicateNumber = 2;

    public float DecisionNumber = 4;

    public Vector3 FirstPlayerPos;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        var fit=GameObject.FindObjectsOfType<CrossLine>();
        var fit2=GameObject.FindObjectsOfType<InsertSplinePath>();
        multi_pow *= -1;
        rb = GetComponent<Rigidbody>();
        StartCoroutine(OnPlayer());
    }

    IEnumerator OnPlayer()
    {
        yield return new WaitForSeconds(0.8f);
        FirstPlayerPos=GameObject.Find("ArcSlab0").transform.position;
        FirstPlayerPos = new Vector3(FirstPlayerPos.x, FirstPlayerPos.y + 2, FirstPlayerPos.z);
        yield return new WaitForSeconds(0.1f);
        transform.position = FirstPlayerPos;

    }
    // Update is called once per frame
    void Update()
    { 
        

        if (Input.GetKey(KeyCode.Space))
        { 
             Delta(IndicateNumber, DecisionNumber);
             if (rb.velocity.y > 8f)
            {
                Debug.Log("");
            }
        }
        
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            rb.AddForce(Vector3.left*multi_pow, ForceMode.Acceleration);
        }

        if (Input.GetKey(KeyCode.RightArrow))
        {
            rb.AddForce(Vector3.right*multi_pow, ForceMode.Acceleration);

        }

        if (Input.GetKey(KeyCode.UpArrow))
        {
            rb.AddForce(Vector3.forward*multi_pow,ForceMode.Acceleration);
        }

        if (Input.GetKey(KeyCode.DownArrow))
        {
            rb.AddForce(Vector3.back*multi_pow, ForceMode.Acceleration);
        }
    }

    float Delta(float y,float height)
    {
        float subtle=(func(y + height) - func(y)) / height;
        return subtle;
    }

    float func(float height)
    {
        return height * height;
    }
    
}
