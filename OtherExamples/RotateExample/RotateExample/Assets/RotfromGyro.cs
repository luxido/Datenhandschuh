using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotfromGyro : MonoBehaviour {
    public SerialController serialController;
    public Vector3 rotCalibration;
    public bool doInit;

    // Use this for initialization
    void Start()
    {
        serialController = GameObject.Find("SerialController").GetComponent<SerialController>();
        doInit = true;
    }

    void processMessage(string message)
    {

        if (message.StartsWith("D.ypr:"))
        {
            message = message.Substring(6);
            string[] ypr = message.Split(',');
            float yaw = float.Parse(ypr[0]);
            float pitch = float.Parse(ypr[1]);
            float roll = float.Parse(ypr[2]);
            Vector3 yprvec = new Vector3(-pitch, yaw, -roll);
            if (doInit)
            {
                rotCalibration = yprvec;
                doInit = false;
            }

            transform.eulerAngles = yprvec - rotCalibration;
        }

        if (message.StartsWith("D.gr.txyz:"))
        {
            Debug.Log(message);
            message = message.Substring(10);
            string[] txyz = message.Split(',');
            long timestamp = long.Parse(txyz[0]);
            float grx = float.Parse(txyz[2]);
            float gry = -float.Parse(txyz[3]);
            float grz = -float.Parse(txyz[1]);

            if (doInit)
            {
                doInit = false;
            }

            Vector3 rotxyz = transform.eulerAngles;
            rotxyz.x += grx * Time.deltaTime;
            rotxyz.y += gry * Time.deltaTime;
            rotxyz.z += grz * Time.deltaTime;

            transform.eulerAngles = rotxyz;
        }



        if (message.StartsWith("xD.a.txyz:"))
        {
            Debug.Log(message);
            message = message.Substring(9);
            string[] txyz = message.Split(',');
            long timestamp = long.Parse(txyz[0]);
            float ax = float.Parse(txyz[2]);
            float ay = float.Parse(txyz[3]);
            float az = float.Parse(txyz[1]);

            if (doInit)
            {
                doInit = false;
            }

            Vector3 rotxyz = transform.eulerAngles;
            rotxyz.x = Mathf.Atan2(az, ay) * 180 / Mathf.PI;
            rotxyz.y = 0;
            rotxyz.z = 90 - Mathf.Atan2(ay, ax) * 180 / Mathf.PI;

            transform.eulerAngles = rotxyz;
        }
    }

    void Update()
    {

        string message;

        while (true)
        {
            message = serialController.ReadSerialMessage();
            if (message == null)
                break;
            if (ReferenceEquals(message, SerialController.SERIAL_DEVICE_CONNECTED))
                continue;
            else if (ReferenceEquals(message, SerialController.SERIAL_DEVICE_DISCONNECTED))
                continue;
            processMessage(message);
        }
    }
}
