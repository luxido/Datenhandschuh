using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KalmanFilter : MonoBehaviour {

    MatrixCalc matrixCalc = new MatrixCalc();

    public float Ts = 33 * 10^-3;   //TaktRate

    float[,] A = { { 0, 0 }, 
                   { 0, 0 } };

    float[,] Ad;

    float[,] C = { { 0, 0 },
                   { 0, 0 } };

    float[,] Cd;

    float[,] G = { { 0, 0 },
                   { 0, 0 } };

    float[,] Q = { { 0, 0 },
                   { 0, 0 } };

    float[,] R = { { 0, 0 },
                   { 0, 0 } };

    float[,] Identity = { { 1, 0 },
                          { 0, 1 } };

    float[,] rnd = { { 3, 4 },
                          { 1, 2 } };


    float[,] rnd2 = { { 1 },
                          { 3} };


    // Use this for initialization
    void Start () {
        float[,] test = matrixCalc.Add2x2Matrices(A, Identity);
        Debug.Log(test[0, 0] + ", " + test[0, 1]);
        Debug.Log(test[1, 0] + ", " + test[1, 1]);
        test = matrixCalc.Sub3x3Matrices(C, Identity);
        Debug.Log(test[0, 0] + ", " + test[0, 1]);
        Debug.Log(test[1, 0] + ", " + test[1, 1]);
        test = matrixCalc.Mult3x3Matrices(rnd, rnd2);
        Debug.Log(test[0, 0]);
        Debug.Log(test[1, 0]);
        Debug.Log(test[2, 0]);


    }

    // Update is called once per frame
    void Update () {
		
	}

    void UseKalmanFilter()
    {
        //x_priori = Ad * xlast                               //+ Bd * yn[1]
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
