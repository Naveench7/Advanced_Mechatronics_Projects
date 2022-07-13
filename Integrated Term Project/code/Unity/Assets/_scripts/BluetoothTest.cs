using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.UI;

public class BluetoothTest : MonoBehaviour
{
    public Text connectionStatus;
    //public Text recievedMessages;
    public Text debugMessage;
    private bool IsConnected;
    public static string dataRecived = "";
    //public RawImage background;

    public int co2_flag = 0;
    public int temp_flag = 0;
    public int button_flag = 0;
    public int alarm_flag = 0;

    public udpStateSender uss;

    //public april_detector detector;
    // Start is called before the first frame update
    void Start()
    {
        IsConnected = false;
        BluetoothService.CreateBluetoothObject();
        //detector = GetComponent<april_detector>();
    }

    // Update is called once per frame
    void Update()
    {
        if (IsConnected) {
            //try
            //{
            //    if (detector.updated)
            //    {
            //        SendCommnad(detector.lin_vel, detector.ang_vel);
            //        detector.updated = false;
            //    }
            //}
            //catch(Exception e)
            //{
            //    connectionStatus.text = "Error in sending data ???? ";
            //}
            try
            {
               string datain =  BluetoothService.ReadFromBluetooth();
                if (datain.Length > 1)
                {
                    dataRecived = datain;
                    print(dataRecived);
                    if (dataRecived.Contains("CO2"))
                        co2_flag = 1;
                    if (dataRecived.Contains("Temp"))
                        temp_flag = 1;
                    if (dataRecived.Contains("Button"))
                        button_flag = 1;
                    if (dataRecived.Contains("CO2") || dataRecived.Contains("Temp"))
                    {
                        //recievedMessages.color = Color.black;
                        //recievedMessages.text = dataRecived;
                        //background.color = Color.red;
                        //co2_flag = 1;
                        //temp_flag = 1;
                        alarm_flag = 1;
                    }
                    else
                    {
                        //recievedMessages.color = Color.white;
                        //recievedMessages.text = dataRecived;
                        //background.color = Color.green;
                        alarm_flag = 0;
                        co2_flag = 0;
                        temp_flag = 0;
                    }
                    uss.SendMessage(dataRecived);
                    
                }
            }
            catch (Exception e)
            {
                debugMessage.text = "Got an exception somewhere";
            }
        }
        
    }

    public void resetButton()
    {
        button_flag = 0;
    }


    public void StartButton()
    {
        connectionStatus.text = "Trying to connect";
        if (!IsConnected)
        {
            try
            {
                IsConnected = BluetoothService.StartBluetoothConnection("mecha6");
            }
            catch
            {
                connectionStatus.text = "Connection Error";
                connectionStatus.color = Color.red;
            }
            connectionStatus.text = "Connected";
            connectionStatus.color = Color.green;
            IsConnected = true;
        }
    }

    //public void SendButton()
    //{
    //    if (IsConnected && (dataToSend.ToString() != "" || dataToSend.ToString() != null))
    //    {
    //        BluetoothService.WritetoBluetooth(dataToSend.text.ToString());
    //    }
    //}

    public byte[] IntArrayToByteArray(int[] intArray)
    {
        int totalBytes = (intArray.Length * 4) + 4;  //Integers are 4 bytes long, plus we're going to add another integer at the beginning with the total array length
        byte[] serialized = new byte[totalBytes];  //Byte array we are going to return

        List<byte[]> listOfBytes = new List<byte[]>();  //A temporary list of byte arrays of converted integers
        foreach (int i in intArray)
        {
            byte[] converted = BitConverter.GetBytes(i);  //convert an integer into a 4 length byte array
            listOfBytes.Add(converted);
        }

        //Now lets build the final byte array
        int location = 0;  //track our current location within the byte array we are going to return with this

        Array.Copy(BitConverter.GetBytes(intArray.Length), 0, serialized, location, 4);  //include the length of the integer array as a header in front of the actual data
        location += 4;
        foreach (byte[] ba in listOfBytes)  //now add the contents of the list to the byte array
        {
            Array.Copy(ba, 0, serialized, location, 4);
            location += 4;
        }

        return serialized;
    }

    public void SendCommnad(int v, int w)
    {
        string start = "G";

        if (IsConnected)
        {
            BluetoothService.WritetoBluetooth(start);
            BluetoothService.WritetoBluetooth("\n");
            BluetoothService.WritetoBluetooth(v.ToString());
            BluetoothService.WritetoBluetooth("\n");
            BluetoothService.WritetoBluetooth(w.ToString());
            BluetoothService.WritetoBluetooth("\n");
        }
        else
        {
            connectionStatus.text = "Issue with connection";
            connectionStatus.color = Color.red;
        }
    }

    public void StopButton()
    {
        if (IsConnected)
        {
            BluetoothService.StopBluetoothConnection();
        }
        Application.Quit();
    }
}
