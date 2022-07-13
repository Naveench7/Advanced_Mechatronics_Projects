using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine.UI;
using System;

public class udpRecvAdv : MonoBehaviour
{

    UdpClient client;

    public int port;


    string previousMessage;
    Thread receiveThread;

    public Text enemy_text;
    public Text friendly_text;
    public Text intersection_text;

    public Button start_button;
    public Text start_button_text;


    public int enemy_count = 0;
    public int friendly_count = 0;
    public int intersection_count = 0;

    // Start is called before the first frame update
    void Start()
    {
        port = 14687;
        client = new UdpClient(port);
        client.Client.ReceiveTimeout = 100;
    }

    // Update is called once per frame
    void Update()
    {
       // while (true)
       // {
        try
        {
            IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, port);
            byte[] data = client.Receive(ref anyIP);

            // Bytes mit der UTF8-Kodierung in das Textformat kodieren.
            string text = Encoding.UTF8.GetString(data);

            Debug.Log(text);

            if (text.Contains("Enemy"))
            {
                enemy_count++;
                enemy_text.text = "Enemy count: " + enemy_count.ToString();
            }

            if (text.Contains("Friend"))
            {
                friendly_count++;
                friendly_text.text = "Friendly count: " + friendly_count.ToString();
            }

            if (text.Contains("Intersection"))
            {
                intersection_count++;
                intersection_text.text = "Intersection count: " + intersection_count.ToString();
            }

            if (text.Contains("Start"))
            {
                start_button.image.color = Color.green;
                start_button_text.color = Color.black;
            }

            previousMessage = text;

        }
        catch (Exception err)
        {
            Debug.Log(err.ToString());
        }
       // }
    }

    /*public void startBot(string s)
    {
        try
        {
            byte[] data = Encoding.UTF8.GetBytes(s);
            sendClient.Send(data, data.Length, remoteEndPoint);
        }
        catch (Exception e)
        {
            Debug.Log(e.ToString());
        }
    }*/
}
