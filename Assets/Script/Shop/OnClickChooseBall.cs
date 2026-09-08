using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text.RegularExpressions;
using Unity.Mathematics;

public class OnClickChooseBall : MonoBehaviour
{
    // Start is called before the first frame update
    public int CommodityNumber = 0;
    public static readonly string CallForCurrrentCoin = "CallForCurrrentCoin";
    public static readonly string itemList = "itemList";
    
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

        

        if (match.Groups.Count==3)
        {
            // BlockInBall (数字)
            int commodityNumber = int.Parse(match.Groups[2].Value);

            AndroidOneOnly.LinenapItemList[commodityNumber] = 1;

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

            PlayerPrefs.Save();

            SpherePreviewManager.ConvertedCoin = true;
        }
        else if(match.Groups.Count==1)
        {
            // 名前がちょうど "BlockInBall"
            Debug.Log("BlockInBall 本体です");
        }
    }
}
