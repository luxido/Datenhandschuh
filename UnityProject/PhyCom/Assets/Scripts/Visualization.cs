using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Visualization : MonoBehaviour
{
    public Vector3[] GyroXValues;
    public Vector3[] GyroXCumsumValues;
    public Vector3[] FlexValues;

    public LineRenderer GyroXLine;
    public LineRenderer GyroXCumsumLine;
    public LineRenderer FlexLine;

    void Start()
    {
        GyroXLine = transform.Find(GyroXLine.ToString()).GetComponent<LineRenderer>();
        GyroXCumsumLine = transform.Find(GyroXCumsumLine.ToString()).GetComponent<LineRenderer>();
        FlexLine = transform.Find(FlexLine.ToString()).GetComponent<LineRenderer>();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
