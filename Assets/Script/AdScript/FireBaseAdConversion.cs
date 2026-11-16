using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FireBaseAdConversion : MonoBehaviour
{
    static public FireBaseAdConversion instance;

    private void Awake()
    {
       /* if (instance == null)
        {
            instance = this;
        }
        else
        {
            Destroy(gameObject);
        }*/
    }


    // Start is called before the first frame update
    void Start()
    {

        //Firebase�������R�[�h
        Firebase.FirebaseApp.CheckAndFixDependenciesAsync().ContinueWith(task =>
        {
            var dependencyStatus = task.Result;
            if (dependencyStatus == Firebase.DependencyStatus.Available)
            {
                // Create and hold a reference to your FirebaseApp,
                // where app is a Firebase.FirebaseApp property of your application class.
                var app = Firebase.FirebaseApp.DefaultInstance;
                // Set a flag here to indicate whether Firebase is ready to use by your app.
                //Firebase.Analytics.FirebaseAnalytics.LogEvent(Firebase.Analytics.FirebaseAnalytics.EventAppOpen);
                Debug.Log("FireBase������");
            }
            else
            {
                UnityEngine.Debug.LogError(System.String.Format(
                  "Could not resolve all Firebase dependencies: {0}", dependencyStatus));
                // Firebase Unity SDK is not safe to use here.
            }
        });
    }
}