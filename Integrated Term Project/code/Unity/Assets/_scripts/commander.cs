using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class commander : MonoBehaviour
{
    public BluetoothTest blt;
    public AudioSource audioSource;
    public falllDetector fd;

    public Text messageToShow;
    public RawImage background;

    public udpStateSender uss;

    public int startFlag = 0;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        string message = "";

        int flag_sum = blt.co2_flag + blt.temp_flag + blt.button_flag + fd.fallFlag;
        if (flag_sum != 0)
        {
            playAlarm();
            change_format_warning();

            if (blt.co2_flag == 1)
            {
                message = "CO2 Warning ";
            }
            if (blt.temp_flag == 1)
            {
                message += "Temperature Warning ";
            }
            if (blt.button_flag == 1)
            {
                message += "Manual Alarm ";
            }
            if (fd.fallFlag == 1)
            {
                message += "Fall Detected ";
            }
        }
        else if (flag_sum == 0)
        {
            message = "Everything IS All RIGHT";
            change_format_clear();
            pauseAlarm();
        }
        messageToShow.text = message;
        StartCoroutine(send_message(message));
        
    }

    IEnumerator send_message(string message)
    {
        uss.sendString(message);
        yield return new WaitForSeconds(0.5f);
    }

    public void playAlarm()
    {
        if (!audioSource.isPlaying)
        {
            audioSource.Play();
        }
    }

    public void pauseAlarm()
    {
        if (audioSource.isPlaying)
        {
            audioSource.Pause();
        }
    }

    void change_format_warning()
    {
        messageToShow.color = Color.black;
        background.color = Color.red;
    }

    void change_format_clear()
    {
        messageToShow.color = Color.white;
        background.color = Color.green;
    }
}

