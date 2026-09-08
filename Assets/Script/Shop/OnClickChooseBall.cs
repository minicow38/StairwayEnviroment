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
    public static  bool[] itemList = new bool[30];
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void OnClick()
    {
        if (!ScrollViewState.IsDragging)
        {
            if (AndroidOneOnly.pharseCoin > 100)
            {
                Match match = Regex.Match(
                    transform.name,
                    @"BlockInBall\s*\((\d+)\)"
                );
                if (match.Success)
                {
                    int commodityNumber = int.Parse(match.Groups[1].Value);
                    PlayerPrefs.SetInt(CallForCurrrentCoin,AndroidOneOnly.pharseCoin);
                    //PlayerPrefs.SetInt(itemList[commodityNumber],true);

                   // AndroidOneOnly.itemList[commodityNumber] = true;

                }
                AndroidOneOnly.pharseCoin = AndroidOneOnly.pharseCoin - 100;
                SpherePreviewManager.ConvertedCoin = true;
               // AndroidOneOnly.itemList[CommodityNumber] = true;
            }
        }
      
    }
}
