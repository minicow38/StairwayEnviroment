using UnityEngine;
using System.Collections;
using System.Text.RegularExpressions;
using TMPro;

// Trigger detection stays separate from SlopeStickCore ground probes.
// Only the REAL InSubject rigidbody can trigger Pylon game-over; a visual
// Subject or other object tagged Player is ignored.
public class GettingTraffic_Corn : MonoBehaviour
{
    public GameObject PhysicsMul;
    public GameObject RendererMul;
    public GameObject ReSubject;
    public bool OneOnly = false;
    public static readonly string CallForBestScore = "CallForBestScore";

    public MainGameManager mainGameManager;

    void Start()
    {
        StartCoroutine(delayStart());
    }

    void Update() { }

    IEnumerator delayStart()
    {
        yield return new WaitForSeconds(0.1f);
        PhysicsMul = GameObject.Find("__GeneratedPhysics");
        RendererMul = GameObject.Find("__GeneratedVisualPlayer");
        GameObject manager = GameObject.Find("GameManager");
        if (manager)
            mainGameManager = manager.GetComponent<MainGameManager>();
    }

    void OnTriggerEnter(Collider other)
    {
        if (!other || OneOnly || !other.CompareTag("Player"))
            return;

        SlopeStickCore core = MainGameManager.core;
        Rigidbody physicalPlayerBody = core
            ? core.GetComponent<Rigidbody>() : null;

        Debug.LogWarning($"[THORN TRIGGER] " +
            $"hazard={gameObject.name} other={other.name} " +
            $"otherLayer={LayerMask.LayerToName(other.gameObject.layer)} " +
            $"otherRoot={other.transform.root.name} " +
            $"rigidbody={(other.attachedRigidbody ? other.attachedRigidbody.name : "NONE")} " +
            $"expectedBody={(physicalPlayerBody ? physicalPlayerBody.name : "NONE")} " +
            $"hazardPos={transform.position:F3} " +
            $"otherPos={other.transform.position:F3} " +
            $"time={Time.fixedTime:F3}", this);

        // OnTriggerEnter is a real overlap callback, NOT the result of a
        // ground Raycast. Avoid accidental death caused by the visual proxy.
        if (!physicalPlayerBody || other.attachedRigidbody != physicalPlayerBody)
        {
            Debug.LogWarning($"[THORN IGNORE NON-PHYSICAL PLAYER] " +
                $"hazard={gameObject.name} other={other.name} " +
                $"actualBody={(other.attachedRigidbody ? other.attachedRigidbody.name : "NONE")}", this);
            return;
        }

        if (MainGameManager.OnDead || !transform.parent)
            return;

        Match parentMatch = Regex.Match(transform.parent.name, @"(.*)_Physics");
        if (!parentMatch.Success)
            return;
        Match childMatch = Regex.Match(transform.name, @"(.*)_Physics");
        if (!childMatch.Success)
            return;

        string visualParentName = parentMatch.Groups[1].Value + "_Render";
        GameObject visualParent = GameObject.Find(visualParentName);
        if (!visualParent)
        {
            Debug.LogWarning($"[THORN VISUAL NOT FOUND] {visualParentName}", this);
            return;
        }

        // Keep the original matching requirement: only the corresponding
        // generated visual Pylon/Thorn object authorizes the game-over.
        string visualChildPattern = @".*" + Regex.Escape(childMatch.Groups[1].Value);
        foreach (Transform child in visualParent.transform)
        {
            if (!Regex.IsMatch(child.name, visualChildPattern))
                continue;

            core.BeginCommandOnTouch = false;
            core.enabled = false;
            MainGameManager.ClashBurst.gameObject.SetActive(true);
            MainGameManager.PointToPlane = 0;
            StartCoroutine(delayResume());
            return;
        }
    }

    IEnumerator delayResume()
    {
        Debug.Log("A : Coroutine開始");
        yield return new WaitForSeconds(0.125f);

        MainGameManager.DropOut.SetActive(true);
        MainGameManager.core.AcceleFirstTurn = false;
        Debug.Log("B : 待機開始 " + Time.timeScale);
        AndroidOneOnly.pharseCoin = MainGameManager.Coin;
        AndroidOneOnly.currentScore = MainGameManager.CurrentPointToPlane;
        if (MainGameManager.CurrentPointToPlane > AndroidOneOnly.bestScore)
        {
            PlayerPrefs.SetInt(CallForBestScore, MainGameManager.PointToPlane);
            AndroidOneOnly.bestScore = MainGameManager.CurrentPointToPlane;
        }

        yield return new WaitForSeconds(1f);

        MainGameManager.DropOut.SetActive(false);
        MainGameManager.core.enabled = true;

        MainGameManager.OnDead = true;
        MainGameManager.reSubject.PointToPlane = 0;
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
