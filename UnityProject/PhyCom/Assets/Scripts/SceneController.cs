using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SceneController : MonoBehaviour
{
    public enum Mode {Live, CSV};
    public Mode selectMode;
    public GameObject SerialController;
    public GameObject MessageListener;
    public KalmanFilter KalmanFilterScript;
    public ReadCSV ReadCSVScript;
    //example inputFileName "drehung1_daten.csv";
    public string inputFileName;
    public string outputFileName = "FilteredData.csv";

    private int n = 0;

    private void Awake()
    {
        SerialController = GameObject.Find("SerialController");
        MessageListener = GameObject.Find("MessageListener");
        KalmanFilterScript = gameObject.GetComponent<KalmanFilter>();
        ReadCSVScript = gameObject.GetComponent<ReadCSV>();
        KalmanFilterScript.init(Application.dataPath + "/" + outputFileName);
    }

    // Start is called before the first frame update
    void Start()
    {
        if (selectMode == Mode.Live)
        {
            MessageListener.GetComponent<SampleMessageListener>().init(gameObject.GetComponent<SceneController>());
        }
        else if (selectMode == Mode.CSV)
        {
            SerialController.SetActive(false);
            MessageListener.SetActive(false);
            ReadCSVScript.init(Application.dataPath + "/" + inputFileName);
            StartCoroutine(KalmanFilterScript.UseKFWithCSV(ReadCSVScript));
            float[] gyroYArrFirst, gyroYArrSecond, flex6ArrSecond, flex6First;
            gyroYArrFirst = ReadCSVScript.GetValuesToArray("GyroY", 1, 221);
            gyroYArrSecond = ReadCSVScript.GetValuesToArray("GyroY", 526, 807);
            flex6First = ReadCSVScript.GetValuesToArray("angle", 1, 221);
            flex6ArrSecond = ReadCSVScript.GetValuesToArray("angle", 526, 807);
            float meanGyro = (KalmanFilterScript.calculateMean(gyroYArrFirst) +
                KalmanFilterScript.calculateMean(gyroYArrSecond)) / 2;
            float stdGyro = (KalmanFilterScript.calculateStd(gyroYArrFirst) +
                KalmanFilterScript.calculateStd(gyroYArrSecond)) / 2;
            float varianceGyro = (KalmanFilterScript.calculateVariance(gyroYArrFirst) +
                KalmanFilterScript.calculateVariance(gyroYArrSecond)) / 2;
            float varianceFlex6 = (KalmanFilterScript.calculateVariance(flex6First) +
                KalmanFilterScript.calculateVariance(flex6ArrSecond)) / 2;
            Debug.Log("stdGyro: "+ stdGyro);
            Debug.Log("varianceGyro: " + varianceGyro);
            Debug.Log("varianceFlex6: " + varianceFlex6);
        }
    }

    public void UseKalmanFilterLive(string msg)
    {
        KalmanFilterScript.StartKalmanFilter(n, msg);
        n++;
    }
}
