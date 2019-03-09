using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KalmanFilter : MonoBehaviour {

    //Deklarierung der Gameobjects zur Darstellung der Werte des Biegesensors, des Gyrosensors und der mit dem Kalman-Filter gefilterten Fusionierung beider Sensorenwerte
    public GameObject unfilteredBiege;
    public GameObject unfilteredGyro;
    public GameObject filtered;
    //Ts -> TaktRate
    
    //Initialiesierung der Taktrate, sowie der Sigma-Werte vom Biegesensor, Gyrosensor und dessen Offset
    public float Ts = 0.033f, sigma_biege = 3f, sigma_gyro = 0.5f, sigma_offset = 3f;

    private MatrixCalc mCalc;
    private List<float[,]>
        x = new List<float[,]>(),
        P = new List<float[,]>(),
        K = new List<float[,]>();
    private float xGyroNew = 0;
    private List<float> xGyro = new List<float>();

    //float[,] x = {Winkel(Biege & ausGyro errechnet), Winkelgeschwindigkeit(Gyro), Offset(Gyro) }  
    
    //Deklarierung aller für den Kalman-Filter notwendigen Vektoren und Matrizen sowie des Einheitsvektors
    private float[,] x0, P0, A, Ad, C, Gd, Q, R, Identity, Bd;

    private char lineSeperater = '\n';
    private char fieldSeperater = ';';
    private string outputFilenamePath;


    void Start () {
        mCalc = gameObject.AddComponent<MatrixCalc>();
    }

    public void init(string filenamePath)
    {
        outputFilenamePath = filenamePath;
    }

    //Instanzieren aller für den Kalman-Filter notwendigen deklarierten Variabeln
    private void setVariables()
    {
        Identity = new float[,] { 
            { 1, 0 , 0}, 
            { 0, 1 , 0}, 
            { 0, 0 , 1}
        };

        //Erster x-Eintrag
        x0 = new float[,] { 
            { 0 }, 
            { 0 }, 
            { 0 }
        }; 

        P0 = new float[,]{ 
            { 100, 0 , 0},
            { 0, 100 , 0},
            { 0, 0 , 100}
        };

        A = new float[,]{ 
            { 0, Ts , -1},
            { 0, 0 , 0},
            { 0, 0 , 0}
        };

        Ad = mCalc.AddMatrix(A, Identity);

        C = new float[,]{ 
            { 1, 0 , 0},
            { 0, 1 , 1}
        };

        Gd = new float[,]{ 
            { pow(0.5f*Ts, 2), -Ts},
            { Ts, 0},
            { 0, Ts}
        };

        Q = new float[,]{ 
            { pow(sigma_biege,2), 0},
            { 0, pow(sigma_gyro,2)}
        };

        R = new float[,]{ 
            { 12, 0},
            { 0, 29}
        };
    }

    private void setVariablesExample()
    {
        Ts = 0.033f;
        sigma_biege = 3f;
        sigma_gyro = 0.5f;
        sigma_offset = -7f;

        Identity = new float[,] {
            { 1, 0 },
            { 0, 1 }
        };

        //Erster x-Eintrag
        x0 = new float[,] {
            { 0 },
            { 0 }
        };

        P0 = new float[,]{
            { 100, 0 },
            { 0, 100 }
        };

        A = new float[,]{
            { 0, -1 },
            { 0, 0 },
        };

        Ad = mCalc.AddMatrix(A, Identity);

        Bd = new float[,]{
            {Ts},
            {0},
        };

        C = new float[,]{
            { 1, 0 },
            { 0, 0 }
        };

        Gd = new float[,]{
            { Ts, -Ts },
            { 0, Ts }
        };

        Q = new float[,]{
            { pow(sigma_gyro,2), 0},
            { 0, pow(sigma_offset,2)}
        };

        R = new float[,]{
            { 702, 0},
            { 0, 1000}
        };
    }

    //Auslesen von Werten einer Zeiler aus einer CSV Datei und anschließender Anwendung des Kalman-Filters
    public IEnumerator UseKFWithCSV(ReadCSV rCSV)
    {
        //-2 not -1 because of the header
        int N = rCSV.CountLines() - 2;
        for (int n = 0; n < N; n++)
        {
            StartKalmanFilter(n, rCSV.GetLine(n));
            yield return new WaitForSeconds(Ts);
        }
    }

    //Methode zur Ausführung eines Kalman-Filter Schrittes und gleichzeitiges schreiben der Filterung in eine CSV
    public void StartKalmanFilter(int n, string msg)
    {
        string newMsg = msg;
        if (n == 0)
        {
            newMsg += fieldSeperater + "angle" + fieldSeperater + "filteredAngle" + fieldSeperater + "Offset" + fieldSeperater + "P";
        }
        else if (n>0)
        {
            newMsg += fieldSeperater +  FilterStep(n, msg);
        }
        System.IO.File.AppendAllText(outputFilenamePath, newMsg + lineSeperater);

    }

    //Kalman-Filter Schritt Definition
    private string FilterStep(int n, string msg)
    {
        setVariables();
        string[] values = msg.Split(fieldSeperater);
        float[,] xlast, x_priori, Plast, P_priori, S, Kn, x_post, yn, P_post;

        //Bei erstmaligem anwenden des Filters werden Startwerte genutzt 
        if (n == 1)
        {
            xlast = x0; Plast = P0;
        }
        //Nutzen der gefilterten Werte aus dem letzten Schritt
        else
        {
            xlast = x[n - 2]; Plast = P[n - 2];
        }

        float angle = FlexSensorToRad(float.Parse(values[1], System.Globalization.CultureInfo.InvariantCulture));
        yn = new float[,]{
            {angle},
            {float.Parse(values[2], System.Globalization.CultureInfo.InvariantCulture)}
        };

        //Ausführung der Formeln für den Kalman-Filter

        //x_priori = Ad * xlast; //+ Bd * yn[1]
        x_priori = mCalc.MultiplyMatrix(Ad, xlast);
        //P_priori = Ad * Plast * Ad.T + Gd * Q * Gd.T
        P_priori = mCalc.AddMatrix(
            mCalc.MultiplyMatrix(mCalc.MultiplyMatrix(Ad, Plast), mCalc.Transpose(Ad)),
            mCalc.MultiplyMatrix(mCalc.MultiplyMatrix(Gd, Q), mCalc.Transpose(Gd))
        );
        //S = C * P_priori * C.T + R
        S = mCalc.AddMatrix(
            mCalc.MultiplyMatrix(mCalc.MultiplyMatrix(C, P_priori), mCalc.Transpose(C)),
            R
        );
        //Kn = P_priori * C.T * linalg.pinv(S)
        Kn = mCalc.MultiplyMatrix(
            mCalc.MultiplyMatrix(P_priori, mCalc.Transpose(C)), mCalc.InvertMatrix(S)
        );

        //x_post = x_priori + Kn * (yn - C * x_priori         // - D * y[n, 1])
        float[,] C_mul_xpriori = mCalc.MultiplyMatrix(C, x_priori);
        x_post = mCalc.AddMatrix(
            x_priori,
            mCalc.MultiplyMatrix(Kn,
                (mCalc.SubMatrix(yn, C_mul_xpriori))
            )
        );

        //P_post = (np.eye(2) - Kn * C) * P_priori
        P_post = mCalc.MultiplyMatrix(mCalc.SubMatrix(Identity, mCalc.MultiplyMatrix(Kn, C)), P_priori);

        x.Add(x_post);
        P.Add(P_post);
        K.Add(Kn);
        xGyro.Add(yn[1, 0]); // ZU EDELER !!! RAUSCHEN NACH VORNE ZU GROß
    
        //Setzen der Winkel der Gameobjects zur Darstellung der eben gefilterten Werte
        unfilteredBiege.transform.eulerAngles = new Vector3(angle, 0, 0);
        unfilteredGyro.transform.eulerAngles = new Vector3(cumSum(xGyro)*Ts, 0, 0);
        filtered.transform.eulerAngles = new Vector3(x_post[0, 0], 0, 0);

        //Debug.Log(angle);
        //Debug.Log(x_post[0, 0]);
        //Debug.Log("yn");
        //mCalc.printMatrix(yn);
        //Debug.Log("Xpost");
        //mCalc.printMatrix(x_post);
        //mCalc.printMatrix(P_post);

        return angle.ToString("0.00", System.Globalization.CultureInfo.InvariantCulture) 
            + fieldSeperater 
            + x_post[0, 0].ToString("0.00", System.Globalization.CultureInfo.InvariantCulture)
            + fieldSeperater
            + x_post[2, 0].ToString("0.00", System.Globalization.CultureInfo.InvariantCulture)
            + fieldSeperater
            + P_post[0, 0].ToString("0.00", System.Globalization.CultureInfo.InvariantCulture);
    }

    //Methode zur Umrechnung der Flexsensor-Werte in einen Zahlenbereich zwischen ~ 0-90 
    public float FlexSensorToRad(float flexSensorValue)
    {
        //flexOffsetValue kann noch berechnet werden
        int flexOffsetValue = 650;
        float radValue = (flexSensorValue - flexOffsetValue) * 9/10; 
        return radValue;
    }

    private static float pow(float f, int p)
    {
        float result = (float)Math.Pow(f, p);
        return result;
    }

    //Methode zur Berechnung des Mittelwerts
    public float calculateMean(float[] data)
    {
        float result = 0;

        for(int i = 0; i < data.Length; i++)
        {
            result += data[i];
        }

        result /= data.Length;

        return result;
    }

    //Methode zur Berechnung der Standardabweichung
    public float calculateStd(float[] data)
    {
        float result = 0;
        result = (float)Math.Sqrt(calculateVariance(data));
        return result;
    }

    //Methode zur Berechnung der Variance
    public float calculateVariance(float[] data)
    {
        float result = 0;
        float mean = calculateMean(data);

        for (int i = 0; i < data.Length; i++)
        {
            result += pow(data[i] - mean, 2);
        }

        result /= data.Length;

        return result;
    }

    //Methode zur Berechnung der kummulierten Summe von Werten einer Liste
    private float cumSum(List<float> list)
    {
        float cumSum = 0f;
        foreach (float value in list)
        {
            cumSum += value;
        }
        return cumSum;
    }
}
