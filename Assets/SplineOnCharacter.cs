using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;


public class SplineOnCharacter : MonoBehaviour
{
    public SplineAnimate anime;
    // Start is called before the first frame update
    void Start()
    {
       /* anime=transform.GetComponent<SplineAnimate>();
        anime.Pause();*/
        StartCoroutine(delayFirstAnime());

    }

    IEnumerator delayFirstAnime()
    {
        yield return new WaitForSeconds(3f);
        //anime.Play();
      ;
    }
    // Update is called once per frame
    void Update()
    {
        
    }
}
