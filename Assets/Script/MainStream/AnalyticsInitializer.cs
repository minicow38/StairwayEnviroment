using UnityEngine;
using ByteBrewSDK;
using System.Collections;

[DefaultExecutionOrder(-1000)]

public class AnalyticsInitializer : MonoBehaviour
{
    private static AnalyticsInitializer instance;
    public static bool isInitialized = false;

    void Awake()
    {
        if (instance != null)
        {
            Destroy(gameObject);
            return;
        }

        instance = this;
        DontDestroyOnLoad(gameObject);

        ByteBrew.InitializeByteBrew();

        StartCoroutine(WaitInit());
    }

    IEnumerator WaitInit()
    {
        float timeout = 5f;
        float timer = 0f;

        while (!ByteBrew.IsByteBrewInitialized() && timer < timeout)
        {
            timer += Time.deltaTime;
            yield return null;
        }

        isInitialized = ByteBrew.IsByteBrewInitialized();
        Debug.Log("ByteBrew Initialized: " + isInitialized);
    }
}