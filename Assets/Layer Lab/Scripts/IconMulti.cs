using UnityEngine;
using UnityEngine.UI;

public class IconMulti : MonoBehaviour
{
    [SerializeField] private Image ballPrefab;
    [SerializeField] private RectTransform parent;

    [SerializeField] private int count = 8;

    private Image[,] BallCollector;

    void Start()
    {
        BallCollector =
            new Image[count, count];

        Rect rect = parent.rect;

        for (int x = 0; x < count; x++)
        {
            for (int y = 0; y < count; y++)
            {
                BallCollector[x, y] =
                    Instantiate(
                        ballPrefab,
                        parent
                    );

                float rateX =
                    (x + 1f) / count;

                float rateY =
                    (y + 1f) / count;

                float posX =
                    rect.xMin +
                    rect.width * rateX;

                float posY =
                    rect.yMin +
                    rect.height * rateY;

                BallCollector[x, y]
                        .rectTransform
                        .anchoredPosition =
                    new Vector2(
                        posX,
                        posY
                    );
            }
        }
    }
}