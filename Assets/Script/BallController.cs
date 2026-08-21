using UnityEngine;
using System.Collections;
public class BallController2 : MonoBehaviour
{
    [Header("Move")]
    [SerializeField] float moveSpeed = 2f;

    [Header("Ground")]
    [SerializeField] float fallSpeed = 5f;
    [SerializeField] float dropSpeed = 15f;
    [SerializeField] float ballRadius = 0.5f;

    [Header("Bounce")]
    [SerializeField] float bounceHeight = 0.15f;
    [SerializeField] float bounceSpeed = 5f;

    [Header("Visual")]
    [SerializeField] Transform ballVisual;

    public Vector3 restart;

    Vector3 moveDirection = Vector3.forward;

    bool isGrounded = true;
    bool isBouncing = false;

    float baseY;
    float bounceOffset;
    void Start()
    {
        //baseY = transform.position.y;
        StartCoroutine(DelayStart());
    }
    void Update()
    {
        CheckInput();
        Move();

        if (isGrounded)
        {
            FollowGround();
        }
        else
        {
            Fall();
        }

        UpdateBounce();
    }

    void CheckInput()
    {
        if (Input.GetMouseButtonDown(0))
        {
            moveDirection =
                Quaternion.Euler(0f, -90f, 0f) * moveDirection;
        }
    }

    void Move()
    {
        float distance = moveSpeed * Time.deltaTime;

        transform.position +=
            moveDirection * distance;

        RotateBall(distance);
    }

    void FollowGround()
    {
        Vector3 rayOrigin =
            transform.position + Vector3.up;

        if (Physics.Raycast(
            rayOrigin,
            Vector3.down,
            out RaycastHit hit,
            5f))
        {
            float targetY =
                hit.point.y + ballRadius;

            // ���̒i�ֈڂ����u��
            if (targetY < baseY - 0.01f && !isBouncing)
            {
                StartBounce();
            }

            baseY =
                Mathf.MoveTowards(
                    baseY,
                    targetY,
                    fallSpeed * Time.deltaTime);

            Vector3 pos = transform.position;
            pos.y = baseY + bounceOffset;
            transform.position = pos;
        }
        else
        {
            isGrounded = false;
        }
    }

    void StartBounce()
    {
        isBouncing = true;
    }

    void UpdateBounce()
    {
        if (!isBouncing)
            return;

        bounceOffset += bounceSpeed * Time.deltaTime;

        if (bounceOffset >= bounceHeight)
        {
            bounceOffset = bounceHeight;
            bounceSpeed *= -1f;
        }

        if (bounceOffset <= 0f)
        {
            bounceOffset = 0f;
            bounceSpeed = Mathf.Abs(bounceSpeed);
            isBouncing = false;
        }
    }

    void Fall()
    {
        transform.position +=
            Vector3.down * dropSpeed * Time.deltaTime;
    }

    void RotateBall(float distance)
    {
        if (ballVisual == null)
            return;

        float circumference =
            2f * Mathf.PI * ballRadius;

        float rotation =
            distance / circumference * 360f;

        Vector3 rotationAxis =
            Vector3.Cross(Vector3.up, moveDirection);

        ballVisual.Rotate(
            rotationAxis,
            rotation,
            Space.World);
    }
     IEnumerator DelayStart()
    {
        yield return new WaitForSeconds(.8f);
        Time.timeScale = 0.75f;

        GameObject startSlab = GameObject.Find("CollisionStageRoot/__GeneratedPhysics/ArcSlab2_0_Physics");
        Vector3 synchronizedPosition = new Vector3(restart.x, restart.y + 2f, restart.z);
        transform.position = synchronizedPosition;

    }
}