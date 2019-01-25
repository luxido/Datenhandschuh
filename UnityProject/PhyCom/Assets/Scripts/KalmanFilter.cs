using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KalmanFilter : MonoBehaviour {

    public GameObject unfiltered;
    public GameObject filtered;
    //Ts -> TaktRate
    public float Ts = 0.033f, sigma_biege = 3f, sigma_gyro = 0.5f, sigma_offset = -7f;

    private MatrixCalc mCalc;
    private List<float[,]>
        x = new List<float[,]>(),
        P = new List<float[,]>(),
        K = new List<float[,]>();

    //float[,] x = {Winkel(Biege & ausGyro errechnet), Winkelgeschwindigkeit(Gyro), Offset(Gyro) }   
    private float[,] x0, P0, A, Ad, C, Gd, Q, R, Identity, Bd;
    private char lineSeperater = '\n';
    private char fieldSeperater = ';';
    private string outputFilenamePath;


    void Start () {
        mCalc = gameObject.AddComponent<MatrixCalc>();
        filtered = GameObject.Find("filtered");
        unfiltered = GameObject.Find("unfiltered");
    }

    public void init(string filenamePath)
    {
        outputFilenamePath = filenamePath;
    }

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
            { 702, 0},
            { 0, 702}
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

    public IEnumerator UseKFWithCSV(ReadCSV rCSV)
    {
        //-2 not -1 because of the header
        int N = rCSV.CountLines() - 2;
        for (int n = 0; n < N; n++)
        {
            StartKalmanFilter(n, rCSV.GetLine(n));
            yield return new WaitForSeconds(0.05f);
        }
    }

    /*
    void UseKalmanFilterEdeler()
    {
        setVariablesExample();
        int N,
            gyroX_index = rCSV.GetIndexOf("GyroX"),
            accY_index = rCSV.GetIndexOf("AccY"),
            accZ_index = rCSV.GetIndexOf("AccZ");
        //-2 not -1 because of the header
        N = rCSV.CountLines() - 2;
        float[,] xlast, x_priori, Plast, P_priori, S, Kn, x_post, yn, P_post;
        float[,] D = new float[,]
            {
                { 0},
                { 1}
            }; ;
        float[,] Bd = new float[,]
            {
                { Ts},
                { 0}
            }; ;

        List<float[,]>
            x = new List<float[,]>(),
            P = new List<float[,]>(),
            K = new List<float[,]>();

        for (int n = 0; n < N; n++)
        {
            if (n == 0)
            {
                xlast = x0; Plast = P0;
            }
            else
            {
                xlast = x[n - 1]; Plast = P[n - 1];
            }

            //+1 because of the header
            string[] values = rCSV.GetLineValues(n + 1);
            float accY = float.Parse(values[accY_index], System.Globalization.CultureInfo.InvariantCulture);
            float accZ = float.Parse(values[accZ_index], System.Globalization.CultureInfo.InvariantCulture);
            yn = new float[,]{
                {mCalc.calcAngle(accY, accZ)},
                {float.Parse(values[gyroX_index], System.Globalization.CultureInfo.InvariantCulture)}
            };

            //x_priori = Ad * xlast; //+ Bd * yn[1]
            x_priori = mCalc.AddMatrix(
                mCalc.MultiplyMatrix(Ad, xlast), mCalc.MultiplyMatrixWithScalar(Bd, yn[1, 0])
            );
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
            float u = yn[1, 0];
            float[,] D_mul_u = mCalc.MultiplyMatrixWithScalar(D, u);
            float[,] C_mul_xpriori = mCalc.MultiplyMatrix(C, x_priori);
            x_post = mCalc.AddMatrix(
                x_priori,
                mCalc.MultiplyMatrix(Kn,
                    mCalc.SubMatrix((mCalc.SubMatrix(yn, C_mul_xpriori)), D_mul_u)
                )
            );

            //P_post = (np.eye(2) - Kn * C) * P_priori
            P_post = mCalc.MultiplyMatrix(mCalc.SubMatrix(Identity, mCalc.MultiplyMatrix(Kn, C)), P_priori);

            x.Add(x_post);
            P.Add(P_post);
            K.Add(Kn);
        }
        mCalc.printMatrix(x[480]);
        mCalc.printMatrix(P[480]);
    }
    */

    public void StartKalmanFilter(int n, string msg)
    {
        string newMsg = msg;
        if (n == 0)
        {
            newMsg += fieldSeperater + "angle" + fieldSeperater + "filteredAngle";
        }
        else if (n>0)
        {
            newMsg += fieldSeperater +  FilterStep(n, msg);
        }
        System.IO.File.AppendAllText(outputFilenamePath, newMsg + lineSeperater);

    }

    private string FilterStep(int n, string msg)
    {
        setVariables();
        string[] values = msg.Split(fieldSeperater);
        float[,] xlast, x_priori, Plast, P_priori, S, Kn, x_post, yn, P_post;
        if (n == 1)
        {
            xlast = x0; Plast = P0;
        }
        else
        {
            xlast = x[n - 2]; Plast = P[n - 2];
        }

        float angle = (float.Parse(values[1], System.Globalization.CultureInfo.InvariantCulture));
        yn = new float[,]{
            {angle},
            {float.Parse(values[2], System.Globalization.CultureInfo.InvariantCulture)}
        };

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

        unfiltered.transform.eulerAngles = new Vector3(angle, 0, 0);
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
            + x_post[0, 0].ToString("0.00", System.Globalization.CultureInfo.InvariantCulture);
    }

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
}
