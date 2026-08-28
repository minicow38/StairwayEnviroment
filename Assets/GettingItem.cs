using UnityEngine;
using System.Text.RegularExpressions;
using UnityEditor.Rendering;
using System.Collections;
using TMPro;

public class GettingItem : MonoBehaviour
{
    public GameObject PhysicsMul;
    public GameObject RendererMul;
    public MainGameManager mainGameManger;
   // public TextMeshUGUI textMesh
    

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        PhysicsMul = GameObject.Find("__GeneratedPhysics");
        RendererMul = GameObject.Find("__GeneratedVisualPlayer");
       // GameObject.Find("StairwayUserbility/Bounus").transform.GetComponent<TextMeshUGUI>();
        StartCoroutine(delayStart());
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    IEnumerator delayStart()
    {
        yield return new WaitForSeconds(0.8f);
        mainGameManger = GameObject.Find("GameManager").transform.GetComponent<MainGameManager>();
        Debug.Log("");

    }
    
    void OnTriggerEnter(Collider other)
    {
        string hit1 = "";
        Match fit = Regex.Match(transform.parent.name, @"(.*)_Physics");
        if (fit.Success)
        {
            hit1 = fit.Groups[1].Value;
            Destroy(transform.gameObject);
        }

        Match subReg = Regex.Match(transform.name, @"(.*)_Physics");
        string subChr=subReg.Groups[1].Value;

        var pat = hit1 + "_Render";
        var subline=GameObject.Find("" + pat);
        Debug.Log("");
        //string pattern = $@"^{Regex.Escape(hit1)}_Render$";
        foreach (Transform PhysicsMul in subline.transform)
        {
            if (Regex.Match(PhysicsMul.name, @".*" + subChr).Success)
            {
                mainGameManger.Coin++;
                Destroy(PhysicsMul.transform.gameObject);
            }
            /*if (Regex.Match(PhysicsMul.name,pattern).Success)
            {
                Debug.Log("");
            }*/
        }

        int x = 0;
    }
}
