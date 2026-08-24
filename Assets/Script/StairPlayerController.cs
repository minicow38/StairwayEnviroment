using UnityEngine;

[RequireComponent(typeof(CharacterController))]
public class StairPlayerController : MonoBehaviour
{
    [Header("Movement")]
    [SerializeField] private float forwardSpeed = 5.5f;
    [SerializeField] private float gravity = 24f;
    [SerializeField] private float groundedPull = 6f;
    [SerializeField] private float turnAngle = 90f;

    [Header("Ground Probe")]
    [SerializeField] private LayerMask groundMask = ~0;
    [SerializeField] private float probeStartOffset = 0.2f;
    [SerializeField] private float probeDistance = 1.2f;

    [Header("Fail State")]
    [SerializeField] private float failAfterAirborneSeconds = 0.8f;

    private CharacterController controller;
    private float verticalVelocity;
    private float airborneTimer;

    private void Reset()
    {
        CharacterController cc = GetComponent<CharacterController>();
        cc.radius = 0.35f;
        cc.height = 1.8f;
        cc.center = new Vector3(0f, 0.9f, 0f);
        cc.stepOffset = 0.35f;
        cc.slopeLimit = 89f;
        cc.skinWidth = 0.04f;
        cc.minMoveDistance = 0f;
    }
    public void Teleport(Vector3 worldPosition, Quaternion worldRotation)
    {
        if (controller == null)
        {
            controller = GetComponent<CharacterController>();
        }

        controller.enabled = false;
        transform.SetPositionAndRotation(worldPosition, worldRotation);
        controller.enabled = true;

        verticalVelocity = 0f;
        airborneTimer = 0f;
    }

    private void Awake()
    {
        controller = GetComponent<CharacterController>();
    }

    private void Update()
    {
        /*StairGameManager manager = StairGameManager.Instance;
        if (manager == null || !manager.IsRunning)
            return;*/

        HandleTurnInput();
        HandleMovement();
    }

    private void HandleTurnInput()
    {
        if (Input.GetKeyDown(KeyCode.A) || Input.GetKeyDown(KeyCode.LeftArrow))
            transform.Rotate(0f, -turnAngle, 0f, Space.World);

        if (Input.GetKeyDown(KeyCode.D) || Input.GetKeyDown(KeyCode.RightArrow) ||
            Input.GetKeyDown(KeyCode.Space) || Input.GetMouseButtonDown(0))
            transform.Rotate(0f, turnAngle, 0f, Space.World);
    }

    private void HandleMovement()
    {
        Vector3 probeOrigin = transform.position + Vector3.up * probeStartOffset;
        float probeRadius = Mathf.Max(0.05f, controller.radius * 0.9f);

        bool hasGround = Physics.SphereCast(
            probeOrigin,
            probeRadius,
            Vector3.down,
            out RaycastHit hit,
            controller.height * 0.5f + probeDistance,
            groundMask,
            QueryTriggerInteraction.Ignore
        );

        Vector3 moveDir = transform.forward;

        if (hasGround && Vector3.Angle(hit.normal, Vector3.up) <= controller.slopeLimit)
        {
            airborneTimer = 0f;
            verticalVelocity = -groundedPull;

            Vector3 projected = Vector3.ProjectOnPlane(transform.forward, hit.normal);
            if (projected.sqrMagnitude > 0.0001f)
                moveDir = projected.normalized;
        }
        else
        {
            verticalVelocity -= gravity * Time.deltaTime;
            airborneTimer += Time.deltaTime;
        }

        Vector3 motion = moveDir * forwardSpeed + Vector3.up * verticalVelocity;
        CollisionFlags flags = controller.Move(motion * Time.deltaTime);

        if ((flags & CollisionFlags.Below) != 0)
        {
            airborneTimer = 0f;
            if (verticalVelocity < 0f)
                verticalVelocity = -groundedPull;
        }

        if (airborneTimer >= failAfterAirborneSeconds && StairGameManager.Instance != null)
        {
            StairGameManager.Instance.NotifyPlayerFell();
        }
    }
}