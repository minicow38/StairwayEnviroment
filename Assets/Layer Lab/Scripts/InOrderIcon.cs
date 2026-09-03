using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;
public class InOrderIcon : MonoBehaviour
{
    public RectTransform rectTransform;

    public Image[] BallCollector;
    // Start is called before the first frame update
    void Start()
    {
        BallCollector = transform.GetComponentsInChildren<Image>();
        for (int i = 0;i< BallCollector.Length; i++)
        {
            BallCollector[i].rectTransform.anchoredPosition = Vector2.zero;
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
