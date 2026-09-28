using UnityEngine;
using System.Text.RegularExpressions;
using UnityEditor.Rendering;
using System.Collections;
using TMPro;

public class GettingTraffic_Corn : MonoBehaviour
{
    public GameObject PhysicsMul;
    public GameObject RendererMul;

    public MainGameManager mainGameManager;
   // public TextMeshUGUI textMesh
    

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        StartCoroutine(delayStart());
        // GameObject.Find("StairwayUserbility/Bounus").transform.GetComponent<TextMeshUGUI>();
        //StartCoroutine(delayStart());
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    IEnumerator delayStart()
    {
        yield return new WaitForSeconds(0.1f);
        PhysicsMul = GameObject.Find("__GeneratedPhysics");
        RendererMul = GameObject.Find("__GeneratedVisualPlayer");
        mainGameManager = GameObject.Find("GameManager").GetComponent<MainGameManager>();
    }

    
    
    void OnTriggerEnter(Collider other)
    {
        string hit1 = "";
        Match fit = Regex.Match(transform.parent.name, @"(.*)_Physics");
        
        if (fit.Success)
        {
            hit1 = fit.Groups[1].Value;
           // Destroy(transform.gameObject);
        }
        else
        {
            return;
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
               MainGameManager.core.BeginCommandOnTouch = false;
               if (!MainGameManager.OnDead)
               {
                   MainGameManager.core.transform.GetComponent<Rigidbody>().isKinematic = true;
                   MainGameManager.PointToPlane = 0;
                   StartCoroutine(delayResume());
               }

              
            }
            /*if (Regex.Match(PhysicsMul.name,pattern).Success)
            {
                Debug.Log("");
            }*/
        }
    }

    IEnumerator delayResume()
    {
        Debug.Log("A : Coroutine開始");
        yield return new WaitForSeconds(0.5f);

        MainGameManager.DropOut.SetActive(true);

        Debug.Log("B : 待機開始 " + Time.timeScale);
        
        
        yield return new WaitForSeconds(2f);
        
        MainGameManager.DropOut.SetActive(false);

        MainGameManager.OnDead = true;


        Debug.Log("C : 待機終了");

        MainGameManager.OpenChunkStage = true;

        Debug.Log("D : ステージ解放");

        MainGameManager.TopTitle.SetActive(true);
        MainGameManager.PreviewIconRoot.SetActive(true);
        MainGameManager.TopLiteral.SetActive(true);
        MainGameManager.PlayButton.SetActive(true);
        MainGameManager.Userbility.SetActive(false);

        Debug.Log("E : 全処理完了");
    }
    void OnDisable()
    {
        Debug.Log("GettingTraffic_Corn : OnDisable " + name);
    }

    void OnDestroy()
    {
        Debug.Log("GettingTraffic_Corn : OnDestroy " + name);
    }
}
