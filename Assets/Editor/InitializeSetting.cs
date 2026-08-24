using UnityEngine;

public class InitializeSetting : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }
#if UNITY_EDITOR
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void AndroidInit()
    {
        Debug.Log("");
    }
    // Update is called once per frame
    void Update()
    {
        
    }
    #endif
}
