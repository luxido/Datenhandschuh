using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KalmanFilter : MonoBehaviour {

    private MatrixCalc mCalc;
    //Ts -> TaktRate
    public float Ts = 0.033f, sigma_biege = 3f, sigma_gyro = 0.5f, sigma_offset = -7f;

    //float[,] x = {Winkel(Biege & ausGyro errechnet), Winkelgeschwindigkeit(Gyro), Offset(Gyro) }   
    private float[,] x0, P0, A, Ad, C, Gd, Q, R, Identity;

    void Start () {
        mCalc = gameObject.AddComponent<MatrixCalc>();
        setVariables();
        UseKalmanFilter();
    }

    private void setVariables()
    {
        Ts = 0.033f;
        sigma_biege = 3f;
        sigma_gyro = 0.5f;
        sigma_offset = -7f;

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
            { 0, 1 , 0}
        };

        Gd = new float[,]{ 
            { Ts,pow(0.5f*Ts, 2) , Ts},
            { 0, Ts , 0},
            { 0, 0 , Ts}
        };

        Q = new float[,]{ 
            { pow(sigma_biege,2), 0 , 0},
            { 0, pow(sigma_gyro,2) , 0},
            { 0, 0 , pow(sigma_offset,2)}
        };

        R = new float[,]{ 
            { 702, 0},
            { 0, 702}
        };
    }

    void UseKalmanFilter()
    {
        int N = 1;
        float[,] xlast, x_priori, Plast, P_priori, S, Kn, test;
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

            //x_priori = Ad * xlast;
            x_priori = mCalc.MultiplyMatrix(Ad, xlast);                          //+ Bd * yn[1]

            //P_priori = Ad * Plast * Ad.T + Gd * Q * Gd.T
            P_priori = mCalc.AddMatrix(
                mCalc.MultiplyMatrix(mCalc.MultiplyMatrix(Ad, Plast), mCalc.Transpose(Ad)),
                mCalc.MultiplyMatrix(mCalc.MultiplyMatrix(Gd, Q), mCalc.Transpose(Gd))
            );

            //S = C * P_priori * C.T + R
            S = mCalc.AddMatrix(
                mCalc.MultiplyMatrix(mCalc.MultiplyMatrix(C, P_priori),mCalc.Transpose(C)),
                R
            );

            //Kn = P_priori * C.T * linalg.pinv(S)
            //Kn = mCalc.MultiplyMatrix(mCalc.MultiplyMatrix(P_priori,mCalc.Transpose(C),);
            //x_post = x_priori + Kn * (yn - C * x_priori         // - D * y[n, 1])
            //P_post = (np.eye(2) - Kn * C) * P_priori


            //x.append(x_post)
            //P.append(P_post)
            //K.append(Kn)
            //Gd = ;
            //mCalc.printMatrix(Gd);
            //mCalc.printMatrix(R);
            mCalc.printMatrix(S);
            mCalc.printMatrix(mCalc.InvertMatrix(S));
            mCalc.printMatrix(mCalc.MultiplyMatrix(S, mCalc.InvertMatrix(S)));
            
        }


    }

    private static float pow(float f, int p)
    {
        float result = (float)Math.Pow(f, p);
        return result;
    }
}
