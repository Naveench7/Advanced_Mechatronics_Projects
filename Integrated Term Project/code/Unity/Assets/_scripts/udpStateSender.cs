using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class udpStateSender : MonoBehaviour
{
    // Start is called before the first frame update
    IPEndPoint remoteEndPoint;
    UdpClient client;

    public string IP;
    public int port;

    string prevSend;
    int cntr = 0;

    void Start()
    {
        IP = "192.168.73.176";
        port = 15689;

        remoteEndPoint = new IPEndPoint(IPAddress.Parse(IP), port);
        client = new UdpClient();
    }

    public void sendString(string s)
    {
        try
        {
            byte[] data = Encoding.UTF8.GetBytes(s);
            client.Send(data, data.Length, remoteEndPoint);
        }
        catch (Exception e)
        {
            Debug.Log(e.ToString());
        }
        prevSend = s;
    }


}
