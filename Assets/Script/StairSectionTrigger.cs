using UnityEngine;

public class StairSectionTrigger : MonoBehaviour
{
    private ProceduralStairway owner;
    private int sectionIndex;
    private bool fired;

    public void Initialize(ProceduralStairway owner, int sectionIndex)
    {
        this.owner = owner;
        this.sectionIndex = sectionIndex;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (fired || owner == null)
            return;

        if (other.GetComponentInParent<StairPlayerController>() == null)
            return;

        fired = true;
        owner.NotifySectionPassed(sectionIndex);
        gameObject.SetActive(false);
    }
}