using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

public class udpStateRecv : MonoBehaviour
{
    UdpClient client;
    // Start is called before the first frame update
    public int port;
    public Text statusText;
    public AudioSource audioSource;

    string previousMessage;

    Thread receiveThread;

    void Start()
    {
        port = 11211;
        //receiveThread = new Thread(
        //    new ThreadStart(ReceiveData));
        //receiveThread.IsBackground = true;
        //receiveThread.Start();
        client = new UdpClient(port);
        client.Client.ReceiveTimeout = 100;
    }

    // Update is called once per frame
    void Update()
    {
        try
        {
            // Bytes empfangen.
            IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, port);
            byte[] data = client.Receive(ref anyIP);

            // Bytes mit der UTF8-Kodierung in das Textformat kodieren.
            string text = Encoding.UTF8.GetString(data);
            if (text == previousMessage)
                return;
            Debug.Log(text);
            statusText.text = text;

            if (text.Contains("RIGHT"))
            {
                statusText.color = Color.green;
                pauseAlarm();
            }
            else
            {
                statusText.color = Color.red;
                playAlarm();
            }
            previousMessage = text;

        }
        catch (Exception err)
        {
            Debug.Log(err.ToString());
        }
    }
    private void ReceiveData()
    {
        Debug.Log("Startin the thread");
        
        while (true)
        {

            try
            {
                // Bytes empfangen.
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, port);
                byte[] data = client.Receive(ref anyIP);

                // Bytes mit der UTF8-Kodierung in das Textformat kodieren.
                string text = Encoding.UTF8.GetString(data);
                if (text == previousMessage)
                    return;
                Debug.Log(text);
                statusText.text = text;
               
                if (text.Contains("RIGHT"))
                    statusText.color = Color.green;
                else
                    statusText.color = Color.red;
                previousMessage = text;

            }
            catch (Exception err)
            {
                Debug.Log(err.ToString());
            }
        }
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
}
