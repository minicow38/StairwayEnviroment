using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class ShopEntranceMethod : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("");
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void OnClick()
    {
        int stageNumber = SceneManager.GetActiveScene().buildIndex;
        if(stageNumber==1)
        SceneManager.LoadScene(2);
        else if(stageNumber==2)
        {
            SceneManager.LoadScene(1);
        }
        else
        {
            SceneManager.LoadScene(0);
        }
    }
}
