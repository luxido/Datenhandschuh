using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KalmanFilter : MonoBehaviour {

    MatrixCalc matrixCalc = new MatrixCalc();

    float[,] Identity = { { 1, 0 , 0},
                          { 0, 1 , 0},
                          { 0, 0 , 1}};

    public static float Ts = 0.033f;   //TaktRate

    //float[,] x = {Winkel(Biege & ausGyro errechnet), Winkelgeschwindigkeit(Gyro), Offset(Gyro) }   

    float[,] x0 = { { 0 }, { 0 }, { 0 } }; //Erster x-Eintrag
    float[,] P0 = { { 100, 0 , 0},
                    { 0, 100 , 0},
                    { 0, 0 , 100}};

    float[,] A = { { 0, Ts , -1}, 
                   { 0, 0 , 0},
                   { 0, 0 , 0}};

    float[,] Ad;

    float[,] C = { { 1, 0 , 0},
                   { 0, 1 , 0} };
                   
    float[,] Gd = { { Ts,pow(0.5f*Ts, 2) , Ts},
                   { 0, Ts , 0},
                   { 0, 0 , Ts}};

    public static float sigma_biege = 3f;
    public static float sigma_gyro = 0.5f;

    public static float sigma_offset = -7f;

    float[,] Q = { { pow(sigma_biege,2), 0 , 0},
                   { 0, pow(sigma_gyro,2) , 0},
                   { 0, 0 , pow(sigma_offset,2)}};

    float[,] R = { { 702, 0 , 0},
                   { 0, 702 , 0},
                   { 0, 0 , 0}};


    // Use this for initialization
    void Start () {
        Ad = matrixCalc.Add3x3Matrices(A, Identity);
        Debug.Log("");
        //float[,] test = matrixCalc.Add2x2Matrices(A, Identity);
        //Debug.Log(test[0, 0] + ", " + test[0, 1]);
        //Debug.Log(test[1, 0] + ", " + test[1, 1]);
        //test = matrixCalc.Sub3x3Matrices(C, Identity);
        //Debug.Log(test[0, 0] + ", " + test[0, 1]);
        //Debug.Log(test[1, 0] + ", " + test[1, 1]);
        //test = matrixCalc.Mult3x3Matrices(rnd, rnd2);
        //Debug.Log(test[0, 0]);
        //Debug.Log(test[1, 0]);
        //Debug.Log(test[2, 0]);


    }

    // Update is called once per frame
    void Update () {
		
	}

    void UseKalmanFilter()
    {
        int N = 10;
        float[,] xlast;
        float[,] x_priori;
        float[,] Plast;
        List<float[,]> x = new List<float[,]>();
        List<float[,]> P = new List<float[,]>();
        List<float[,]> K = new List<float[,]>();

        for (int n = 0; n < N; n++)
        {
            if (n == 0)
            {
                xlast = x0;
                Plast = P0;
            }
            else
            {
                xlast = x[n - 1];
                Plast = P[n - 1];
            }

            //x_priori = Ad * xlast;                               //+ Bd * yn[1]
            //P_priori = Ad * Plast * Ad.T + Gd * Q * Gd.T

            //S = C * P_priori * C.T + R
            //Kn = P_priori * C.T * linalg.pinv(S)
            //x_post = x_priori + Kn * (yn - C * x_priori         // - D * y[n, 1])
            //P_post = (np.eye(2) - Kn * C) * P_priori


            //x.append(x_post)
            //P.append(P_post)
            //K.append(Kn)
        }


    }

    private static float pow(float f, int p)
    {
        float result = (float)Math.Pow(f, p);
        return result;
    }
}
