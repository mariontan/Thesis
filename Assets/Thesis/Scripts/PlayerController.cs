using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System;

public class PlayerController : MonoBehaviour {

    public Rigidbody rb;
    public float speed = 1;
    public float timeSec = 60f;
    public float rotationspeed = 1;
    private int score;
    private float moveHorizontal = 0;
    private float moveVertical = 0;
    public int yaw = 0;
    public int roll = 1;
    public Text scoreText, timeText;


    void Start()
    {
		//StartCoroutine(WaitToStartSerial(120));
		rb = GetComponent<Rigidbody>();
        score = 0;
    }

    void FixedUpdate()
    {
        timeSec -= Time.deltaTime;
        timeText.text = "Time: " + String.Format("{0:0.#}", timeSec);

        if(timeSec < 0)
        {
            string sceneName = SceneManager.GetActiveScene().name;
            SceneManager.UnloadScene(sceneName);
            SceneManager.LoadScene("Open Ocean", LoadSceneMode.Single);
        }

        if (Input.GetKey(KeyCode.I))
        {
            string sceneName = SceneManager.GetActiveScene().name;
            SceneManager.UnloadScene(sceneName);
            SceneManager.LoadScene("Open Ocean", LoadSceneMode.Single);
        }
        else if (Input.GetKey(KeyCode.O))
        {
            string sceneName = SceneManager.GetActiveScene().name;
            SceneManager.UnloadScene(sceneName);
            SceneManager.LoadScene("Quest 1", LoadSceneMode.Single);
        }
        else if (Input.GetKey(KeyCode.P))
        {
            string sceneName = SceneManager.GetActiveScene().name;
            SceneManager.UnloadScene(sceneName);
            SceneManager.LoadScene("Quest 2", LoadSceneMode.Single);
        }
    }

    void LateUpdate()
    {
        Vector3 movement = new Vector3(moveHorizontal, 0.0f, moveVertical);
        transform.Rotate(0, 0, -moveHorizontal * rotationspeed);

        rb.AddForce(movement * speed);
    }

    

    void OnTriggerEnter(Collider other)
    {
        if(other.gameObject.CompareTag("Pickup"))
        {
            other.gameObject.SetActive(false);
            score++;
            scoreText.text = "Score: " + score.ToString();
        }
    }

    void OnSerialValues(string[] values)
    {
        if (yaw < values.Length)
        {
            //GetComponent<GUIText>().text = "Last value [" + yaw + "]: " + values[yaw];
            moveHorizontal = (float) Convert.ToDouble(values[yaw]);

        }
        if (roll < values.Length)
        {
            moveVertical = (float)Convert.ToDouble(values[roll]);
        }
    }
}