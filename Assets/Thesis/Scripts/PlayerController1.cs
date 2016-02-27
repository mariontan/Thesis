using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using System;
using UnityEngine.SceneManagement;

public class PlayerController1 : MonoBehaviour
{

    public Rigidbody rb;
    public float speed = 1;
    public float timeSec = 10f;
    public float rotationspeed = 1;
    private int life; 
    private float moveHorizontal = 0;
    private float moveVertical = 0;
    public int yaw = 0;
    public int roll = 1;
    public Text lifeText, timeText;


    void Start()
    {
        rb = GetComponent<Rigidbody>();
        life = 10;
    }

    void FixedUpdate()
    {
        timeSec -= Time.deltaTime;
        timeText.text = "Time: " + String.Format("{0:0.#}", timeSec);
        if (timeSec < 0 || life == 0)
        {
            string sceneName = SceneManager.GetActiveScene().name;
            SceneManager.UnloadScene(sceneName);
            SceneManager.LoadScene("Open Ocean", LoadSceneMode.Single);
        }
    }

    void LateUpdate()
    {
        Vector3 movement = new Vector3(moveHorizontal, 0.0f, 1);
        transform.Rotate(0, 0, -moveHorizontal * rotationspeed);

        rb.AddForce(movement * speed);
    }



    void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Pickup"))
        {
            other.gameObject.SetActive(false);
            life--;
            lifeText.text = "Life: " + life.ToString();
        }
    }

    void OnSerialValues(string[] values)
    {
        if (yaw < values.Length)
        {
            //GetComponent<GUIText>().text = "Last value [" + yaw + "]: " + values[yaw];
            moveHorizontal = (float)Convert.ToDouble(values[yaw]);
        }
        if (roll < values.Length)
        {
            moveVertical = (float)Convert.ToDouble(values[roll]);
        }
    }
}