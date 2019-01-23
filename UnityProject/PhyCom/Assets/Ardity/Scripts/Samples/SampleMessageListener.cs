/**
 * Ardity (Serial Communication for Arduino + Unity)
 * Author: Daniel Wilches <dwilches@gmail.com>
 *
 * This work is released under the Creative Commons Attributions license.
 * https://creativecommons.org/licenses/by/2.0/
 */

using UnityEngine;
using System.Collections;
using System;

/**
 * When creating your message listeners you need to implement these two methods:
 *  - OnMessageArrived
 *  - OnConnectionEvent
 */
public class SampleMessageListener : MonoBehaviour
{
    private char lineSeperater = '\n';
    private char fieldSeperater = ';';

    public KalmanFilter kalmanFilter;

    // Invoked when a line of data is received from the serial device.
    void OnMessageArrived(string msg)
    {
        string data = msg;
        // Split string on spaces (this will separate all the words).
        string[] words = data.Split(';');

        int x = 0;

        if (words.Length > 1)
        {
            Int32.TryParse(words[1], out x);

            x = kalmanFilter.FlexSensorToRad(x);
            words[1] = x.ToString();

            string tmpMsg = "";
            for (int i = 0; i < words.Length; i++)
            {
                if (i < words.Length - 1)
                {
                    tmpMsg = tmpMsg + words[i] + ";";
                }
                else
                {
                    tmpMsg = tmpMsg + words[i];
                }
            }

            msg = tmpMsg;
        }
        Debug.Log(msg);
        System.IO.File.AppendAllText(Application.dataPath + "/SavedData.csv", msg + lineSeperater);
    }

    // Invoked when a connect/disconnect event occurs. The parameter 'success'
    // will be 'true' upon connection, and 'false' upon disconnection or
    // failure to connect.
    void OnConnectionEvent(bool success)
    {
        if (success)
            Debug.Log("Connection established");
        else
            Debug.Log("Connection attempt failed or disconnection detected");
    }
}
