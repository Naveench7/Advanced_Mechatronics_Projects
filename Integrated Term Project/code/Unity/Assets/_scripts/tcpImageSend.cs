using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using UnityEngine;

public class tcpImageSend : MonoBehaviour
{
    // Start is called before the first frame update
    public RenderTexture _webcamBuffer;
    WebCamTexture _webcamRaw;
    private Texture2D sendImage2DMain;

    byte[] data = new byte[0];
    int sent;
    IPEndPoint ipep = new IPEndPoint(IPAddress.Parse("192.168.196.207"), 1337);
    Socket server = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

    Vector2Int _resolution = new Vector2Int(640, 480);

    void Start()
    {
        _webcamRaw = new WebCamTexture(_resolution.x, _resolution.y, 60);
        _webcamRaw.Play();

        _resolution.x = _webcamRaw.width;
        _resolution.y = _webcamRaw.height;

        _webcamBuffer = new RenderTexture(_resolution.x, _resolution.y, 0);
        try
        {
            server.Connect(ipep);
        }
        catch (SocketException e)
        {
            Debug.Log(e.ToString());
        }
    }
    public void sendImage()
    {
        RenderTexture.active = _webcamBuffer;
        sendImage2DMain.ReadPixels(new Rect(0, 0, _resolution.x, _resolution.y), 0, 0);
        sendImage2DMain.Apply();
        sent = SendVarData(server, sendImage2DMain.EncodeToPNG());
    }

    private static int SendVarData(Socket s, byte[] data)
    {
        int total = 0;
        int size = data.Length;
        int dataleft = size;
        int sent;

        byte[] datasize = new byte[0];
        datasize = BitConverter.GetBytes(size);
        sent = s.Send(datasize);

        while (total < size)
        {
            sent = s.Send(data, total, dataleft, SocketFlags.None);
            total += sent;
            dataleft -= sent;
        }

        return total;
    }
}
