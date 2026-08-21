using UnityEngine;

public class Test : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        var x = transform.GetComponent<Renderer>().bounds.size.x;
        Debug.Log("");
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
