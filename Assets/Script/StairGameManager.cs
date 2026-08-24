using UnityEngine;
using UnityEngine.SceneManagement;

public class StairGameManager : MonoBehaviour
{
    public enum GameState
    {
        Ready,
        Running,
        Cleared,
        Failed
    }

    public static StairGameManager Instance { get; private set; }

    [Header("Scene References")]
    [SerializeField] private ProceduralStairway stairway;
    [SerializeField] private StairPlayerController player;
    [SerializeField] private StairCameraFollow followCamera;

    [Header("Texts")]
    [SerializeField] private string gameTitle = "Stairway Prototype";

    private GameState state = GameState.Ready;
    private float elapsedTime;

    public bool IsRunning => state == GameState.Running;
    public GameState State => state;

    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }

        Instance = this;
        Time.timeScale = 1f;
    }
    

    private void Start()
    {
        if (stairway == null)
        {
            stairway = FindObjectOfType<ProceduralStairway>();
        }

        if (player == null)
        {
            player = FindObjectOfType<StairPlayerController>();
        }

        if (followCamera == null)
        {
            followCamera = FindObjectOfType<StairCameraFollow>();
        }

        ResetRun();
    }

    private void Update()
    {
        if (state == GameState.Ready)
        {
            if (Input.GetKeyDown(KeyCode.Return))
            {
                BeginGame();
            }
        }
        else if (state == GameState.Running)
        {
            elapsedTime += Time.deltaTime;
        }
        else
        {
            if (Input.GetKeyDown(KeyCode.R))
            {
                RestartScene();
            }
        }
    }

    public void BeginGame()
    {
        if (state != GameState.Ready)
            return;

        state = GameState.Running;
    }

    public void NotifyPlayerFell()
    {
        if (state != GameState.Running)
            return;

        state = GameState.Failed;
    }

    public void NotifyGoalReached()
    {
        if (state != GameState.Running)
            return;

        state = GameState.Cleared;
    }

    public void RestartScene()
    {
        Time.timeScale = 1f;
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }

    private void ResetRun()
    {
        elapsedTime = 0f;
        state = GameState.Ready;

        if (stairway != null)
        {
            stairway.RebuildCourse();
        }

        if (player != null && stairway != null)
        {
            player.Teleport(stairway.RecommendedSpawnPosition, stairway.RecommendedSpawnRotation);
        }

        if (followCamera != null && player != null)
        {
            followCamera.SetTarget(player.transform);
        }
    }

    private void OnGUI()
    {
        GUIStyle labelStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 22,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.UpperLeft
        };

        GUIStyle smallStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 16
        };

        GUI.Label(new Rect(18, 18, 360, 30), gameTitle, labelStyle);

        string hudText = $"Section {Mathf.Min(stairway != null ? stairway.GeneratedSectionCount : 0, stairway != null ? stairway.TotalSections : 0)} / {(stairway != null ? stairway.TotalSections : 0)}\nTime {elapsedTime:0.0}s";
        GUI.Label(new Rect(18, 52, 240, 50), hudText, smallStyle);

        if (state == GameState.Ready)
        {
            DrawCenteredPanel(
                $"{gameTitle}\n\nEnter で開始\nA / ← : 左へ90°\nD / → / Space / Click : 右へ90°\n落ちたら失敗、ゴールゲートでクリア",
                "Start",
                BeginGame);
        }
        else if (state == GameState.Cleared)
        {
            DrawCenteredPanel(
                $"GOAL!\n\nクリアタイム {elapsedTime:0.00} 秒",
                "Restart",
                RestartScene);
        }
        else if (state == GameState.Failed)
        {
            DrawCenteredPanel(
                "FALL DOWN\n\n階段から落ちました",
                "Restart",
                RestartScene);
        }
    }

    private void DrawCenteredPanel(string message, string buttonText, System.Action onButtonPressed)
    {
        Rect area = new Rect(Screen.width * 0.5f - 180f, Screen.height * 0.5f - 110f, 360f, 220f);
        GUI.Box(area, string.Empty);

        GUIStyle centered = new GUIStyle(GUI.skin.label)
        {
            fontSize = 18,
            alignment = TextAnchor.MiddleCenter,
            wordWrap = true
        };

        GUI.Label(new Rect(area.x + 18f, area.y + 18f, area.width - 36f, 120f), message, centered);

        if (GUI.Button(new Rect(area.x + 110f, area.y + 150f, 140f, 36f), buttonText))
        {
            onButtonPressed?.Invoke();
        }
    }
}
