using System.Collections;
using System;
using System.Collections.Generic;
using UnityEngine;
using DG.Tweening;

public class SwipeHand : MonoBehaviour
{
    public bool isSwing = false;

    public bool finish = false;
    // Start is called before the first frame update
    void Start()
    {
        transform.position=transform.TransformPoint(transform.parent.position);
        
        Debug.Log("");

    }

    // Update is called once per frame
    void Update()
    {
        if (!isSwing&& !finish)
        {
            transform.DOLocalMoveX(-120f, 1f)
                .SetRelative()
                .SetEase(Ease.Linear)
                .SetLoops(-1, LoopType.Restart).OnComplete(() =>
            {
                transform.position = new Vector3(0, transform.position.y, transform.position.z);
                finish = false;
                isSwing = false;
                
            });
            finish = true;
            isSwing = true;
        }
    }

   /* IEnumerator DelaySwipe()
    {
        yield return new WaitForSeconds(5f);


    }*/
}
