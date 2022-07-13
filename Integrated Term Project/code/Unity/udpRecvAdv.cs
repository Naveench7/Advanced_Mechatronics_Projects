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
    // UDP Client class
    UdpClient client;
    // Port for UDP
    public int port;


    string previousMessage;
    Thread receiveThread;
    
    // Text fields we will modify
    public Text enemy_text;
    public Text friendly_text;
    public Text intersection_text;
    
    // Button manupulation
    public Button start_button;
    public Text start_button_text;

    // Counter for the messages
    public int enemy_count = 0;
    public int friendly_count = 0;
    public int intersection_count = 0;

    // Start is called before the first frame update
    void Start()
    {
        // Initialize UDP
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
            // UDP Recv Endpoint
            IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, port);
            // Get the data from UDP
            byte[] data = client.Receive(ref anyIP);

            // Convert from byte array to string
            string text = Encoding.UTF8.GetString(data);

            Debug.Log(text);
            
            // Check for enemy
            if (text.Contains("Enemy"))
            {
                enemy_count++;
                enemy_text.text = "Enemy count: " + enemy_count.ToString();
            }
            
            // Check for friend
            if (text.Contains("Friend"))
            {
                friendly_count++;
                friendly_text.text = "Friendly count: " + friendly_count.ToString();
            }
            
            // Check for intersection
            if (text.Contains("Intersection"))
            {
                intersection_count++;
                intersection_text.text = "Intersection count: " + intersection_count.ToString();
            }
            
            // Check for start message
            if (text.Contains("Start"))
            {
                start_button.image.color = Color.green;
                start_button_text.color = Color.black;
            }

            previousMessage = text;

        }
        catch (Exception err)
        {
            // Exception during timeout
            Debug.Log(err.ToString());
        }
       // }
    }


}
