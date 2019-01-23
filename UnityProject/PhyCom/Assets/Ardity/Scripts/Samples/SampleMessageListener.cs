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
    private int n = 0;
    public KalmanFilter kFilter;

    private void Start()
    {
        kFilter = gameObject.AddComponent<KalmanFilter>();
    }
    // Invoked when a line of data is received from the serial device.
    void OnMessageArrived(string msg)
    {

        string[] words = msg.Split(';');
        kFilter.UseKalmanFilterLive(n, words);
        n++;
        //Debug.Log(msg);
        //System.IO.File.AppendAllText(Application.dataPath + "/SavedData.csv", msg + lineSeperater);
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
