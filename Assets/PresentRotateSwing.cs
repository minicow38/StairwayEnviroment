using UnityEngine;
using DG.Tweening;

public class PresentRotateSwing : MonoBehaviour
{
    [SerializeField] private float swingAngle = 30f;
    [SerializeField] private float halfCycleDuration = 0.9f;

    private Tween swingTween;

    private void Start()
    {
        // Start from the left side so the motion is perfectly symmetric around 0 degrees.
        transform.localRotation = Quaternion.Euler(0f, 0f, -swingAngle);

        swingTween = transform
            .DOLocalRotate(
                new Vector3(0f, 0f, swingAngle),
                halfCycleDuration,
                RotateMode.Fast)
            .SetEase(Ease.InOutSine)
            .SetLoops(-1, LoopType.Yoyo);
    }

    private void OnDestroy()
    {
        if (swingTween != null && swingTween.IsActive())
        {
            swingTween.Kill();
        }
    }
}
