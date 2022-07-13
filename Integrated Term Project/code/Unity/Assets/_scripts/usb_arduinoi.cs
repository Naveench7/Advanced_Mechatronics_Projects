using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.IO.Ports;
using UnityEngine.UI;

public class usb_arduinoi : MonoBehaviour
{
    // Start is called before the first frame update
    static private SerialPort serialPort = new SerialPort("COM6", 9600);
    public Text debugText;

    void Start()
    {
        Debug.Log("Connecting ...");
        showPort();
        OpenConnection();
    }

    void showPort()
    {
        // Get a list of serial port names. 
        string[] ports = SerialPort.GetPortNames();

        Debug.Log("The following serial ports were found:");
        

        // Display each port name to the console
        foreach (string port in ports)
        {
            Debug.Log(port);
            debugText.text = port;
        }
    }

    void OpenConnection()
    {
        if (serialPort != null)
        {
            if (!serialPort.IsOpen)
            {
                serialPort.Open(); // ouverture de la connection
                serialPort.ReadTimeout = 1;
                Debug.Log("Port ouvert");
                debugText.text = "Opened Port";
            }
        }

        else
        {
            Debug.Log("Port == null");
            debugText.text = "Port = null";
        }
    }

    void SendIntMessage(int pin, int value)
    {

        Debug.Log("Not implemeted here ");
        //serialPort.Write(message, 0, 6);
        //Debug.Log("send");
    }
}
