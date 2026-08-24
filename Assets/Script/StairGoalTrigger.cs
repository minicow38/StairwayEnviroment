using UnityEngine;

public class StairGoalTrigger : MonoBehaviour
{
    private bool fired;

    private void OnTriggerEnter(Collider other)
    {
        if (fired)
            return;

        if (other.GetComponentInParent<StairPlayerController>() == null)
            return;

        fired = true;

        if (StairGameManager.Instance != null)
        {
            StairGameManager.Instance.NotifyGoalReached();
        }
    }
}
