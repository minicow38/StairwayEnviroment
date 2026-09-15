using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text.RegularExpressions;
using Unity.Mathematics;

public class OnClickChooseBall : MonoBehaviour
{
    // Start is called before the first frame update
    public int CommodityNumber = 0;
    readonly string CallForCurrrentCoin = "CallForCurrrentCoin";
    readonly string itemList = "itemList";
    private readonly string UsingBallNow = "ActiveUselessBall";
    
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void OnClick()
    {
        if (ScrollViewState.IsDragging)
            return;

        if (AndroidOneOnly.pharseCoin < 100)
            return;

        Match match = Regex.Match(
            transform.name,
            @"^(BlockInBall)(?:\s*\((\d+)\))?$"
        );

        if (!match.Success)
            return;
        var obj=transform.Find("OnLock");
        
            bool WBooking = false;
            int commodityNumber = 0;

            if (match.Groups[2].Success)
            {
                commodityNumber = int.Parse(match.Groups[2].Value);
                
                WBooking=AndroidOneOnly.LinenapItemList[commodityNumber] == 1 ? true : false;
                
                if(!WBooking)
                AndroidOneOnly.LinenapItemList[commodityNumber] = 1;
                Debug.Log("");
            }
            else
            {
                // 名前がちょうど "BlockInBall"
                AndroidOneOnly.LinenapItemList[0] = 1;

            }

            if (obj.transform.gameObject.activeSelf)
            {
                AndroidOneOnly.pharseCoin -= 100;

                string stringPlus = "";

                for (int i = 0; i < AndroidOneOnly.LinenapItemList.Count; i++)
                {
                    stringPlus += AndroidOneOnly.LinenapItemList[i].ToString();
                }

                PlayerPrefs.SetInt(
                    CallForCurrrentCoin,
                    AndroidOneOnly.pharseCoin
                );

                PlayerPrefs.SetString(
                    itemList,
                    stringPlus
                );
                obj.transform.gameObject.SetActive(false);
                SpherePreviewManager.ConvertedCoin = true;

            }
            else
            {
                AndroidOneOnly.UsingBallNow = commodityNumber;
                PlayerPrefs.SetInt(UsingBallNow, commodityNumber);

            }
            PlayerPrefs.Save();

    }
   
}
