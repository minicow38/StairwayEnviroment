using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class VelocityCheak : MonoBehaviour
{
    public Rigidbody rb;

    public TextMeshProUGUI textMesh;
    // Start is called before the first frame update
    void Start()
    {
        rb = transform.GetComponent<Rigidbody>();
        
    }

    // Update is called once per frame
    void Update()
    {
        textMesh.text = (rb.velocity.y).ToString("");
    }
}
