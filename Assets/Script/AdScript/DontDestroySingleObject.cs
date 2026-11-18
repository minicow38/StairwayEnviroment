using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class DontDestroySingleObject : MonoBehaviour
{
    public static DontDestroySingleObject Instance
    {
        get; private set;
    }

    void Awake()
    {
        if (Instance != null)
        {
            Destroy(gameObject);
            return;
        }
        Instance = this;
        DontDestroyOnLoad(gameObject);
    }

    IEnumerator dealyScene()
    {
        yield return new WaitForSeconds(0.5f);
    }
}