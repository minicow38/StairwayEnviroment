using UnityEngine;
using System;
using System.Collections;
public class TrackingCamera : MonoBehaviour
{
    private GameObject Player;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
       
        StartCoroutine(temporary());
        Debug.Log("");
    }

    // Update is called once per frame
    IEnumerator temporary()
    {
        yield return new WaitForSeconds(0.25f);
        Player = GameObject.Find("subject");
        //transform.position = Player.transform.position;
        //transform.SetParent(Player.transform);
        
    }
    
    void Update()
    {
        if (Player != null)
        {
            Camera.main.transform.position = new Vector3(Player.transform.position.x, Player.transform.position.y+5, Player.transform.position.z+8);
        }

    }
}
