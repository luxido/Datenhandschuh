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
            SerialController.GetComponent<SampleMessageListener>().init(gameObject.GetComponent<SceneController>());
        }
        else if (selectMode == Mode.CSV)
        {
            SerialController.SetActive(false);
            MessageListener.SetActive(false);
            ReadCSVScript.init(Application.dataPath + "/" + inputFileName);
            StartCoroutine(KalmanFilterScript.UseKFWithCSV(ReadCSVScript));
        }
    }

    public void UseKalmanFilterLive(string msg)
    {
        KalmanFilterScript.StartKalmanFilter(n, msg);
        n++;
    }
}
