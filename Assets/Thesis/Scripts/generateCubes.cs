using UnityEngine;
using System.Collections;

public class generateCubes : MonoBehaviour {

    public Rigidbody cubePrefab;
    private float nextActionTime = 0.0f;
    public float period = 2f;

    // Use this for initialization
    void Start () {
	
	}
	
	// Update is called once per frame
	void FixedUpdate () {
        if (Input.GetKey(KeyCode.P) || Time.time > nextActionTime)
        {
            nextActionTime = Time.time + period;
            Rigidbody cubeInstance;
            Vector3 position_player = GameObject.FindGameObjectWithTag("Player").transform.position;
            Quaternion rotation = GameObject.FindGameObjectWithTag("Player").transform.rotation;
            Vector3 position = new Vector3(position_player.x + Random.Range(-10.0F, 10.0F), position_player.y, position_player.z + 10);
            cubeInstance = Instantiate(cubePrefab, position, rotation) as Rigidbody;
        }
	}
}
