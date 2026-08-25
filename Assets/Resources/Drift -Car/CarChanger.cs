using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarChanger : MonoBehaviour
{

    int count = 0;
    int childCount;
    private void Start()
    {
        childCount = transform.childCount;
    }
    // Update is called once per frame
    void Update()
    {
        if(Input.GetKeyDown(KeyCode.RightArrow))
        {
            count++;
            SetCar();
        }
        if(Input.GetKeyDown(KeyCode.LeftArrow))
        {
            count--;
            SetCar();
        }
        

    }

    void SetCar()
    {
        if (count < 0)
        {
            count = childCount - 1;
        }
        if (count > childCount - 1)
        {
            count = 0;
        }

        for (int i=0;i<childCount;i++)
        {
            if(i==count)
            {
                transform.GetChild(i).gameObject.SetActive(true);
            }
            else
            {
                transform.GetChild(i).gameObject.SetActive(false);
            }
        }
    }

}
