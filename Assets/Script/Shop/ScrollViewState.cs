using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class ScrollViewState : MonoBehaviour,
    IBeginDragHandler,
    IEndDragHandler
{
    public static bool IsDragging { get; private set; }

    void Start()
    {
       
    }

    
    public  void OnBeginDrag(PointerEventData eventData)
    {
        IsDragging = true;
    }

    public void OnEndDrag(PointerEventData eventData)
    {
        IsDragging = false;
    }
}