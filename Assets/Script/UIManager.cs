using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UIManager : MonoBehaviour
{
    public static int Coin = 0;
    

    // Start is called before the first frame update
    void Start()
    {
        Coin = AndroidOneOnly.pharseCoin;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
