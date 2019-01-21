using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MatrixCalc : MonoBehaviour {

    public void printMatrix(float[,] arr)
    {
        int rowLength = arr.GetLength(0);
        int colLength = arr.GetLength(1);
        string line = "";

        for (int i = 0; i < rowLength; i++)
        {
            for (int j = 0; j < colLength; j++)
            {
                line +=  arr[i, j] + " ";
            }
            line += System.Environment.NewLine;
        }
        Debug.Log(line);
    }

    public double[,] MultiplyMatrix(double[,] A, double[,] B)
    {
        int rA = A.GetLength(0);
        int cA = A.GetLength(1);
        int rB = B.GetLength(0);
        int cB = B.GetLength(1);
        double temp = 0;
        double[,] kHasil = new double[rA, cB];
        if (cA != rB)
        {
            Debug.Log("matrix can't be multiplied !!");
        }
        else
        {
            for (int i = 0; i < rA; i++)
            {
                for (int j = 0; j < cB; j++)
                {
                    temp = 0;
                    for (int k = 0; k < cA; k++)
                    {
                        temp += A[i, k] * B[k, j];
                    }
                    kHasil[i, j] = temp;
                }
            }
            return kHasil;
        }
        return kHasil;
    }

    public float[,] Mult2x2Matrices(float[,] firstMatrix, float[,] secondMatrix)
    {
        
        float[,] result = { { 0, 0  }, {  0, 0 } };

        int rowLength = firstMatrix.Rank;
        int columnLength = secondMatrix.Length / secondMatrix.Rank;

        if(columnLength == 1)
        {
            result = new float[2, 1];
        }

        for(int i = 0; i < rowLength; i++)
        {
            for(int j = 0; j < columnLength; j++)
            {
                result[i, j] = firstMatrix[i, 0] * secondMatrix[0, j] + firstMatrix[i, 1] * secondMatrix[1, j]; 
            }
        }

        return result;
    }

    public float[,] Mult3x3Matrices(float[,] firstMatrix, float[,] secondMatrix)
    {
        float[,] result = { { 0, 0, 0 }, { 0, 0 , 0 }, { 0, 0, 0 } };

        int rowLength = firstMatrix.Rank;
        int columnLength = secondMatrix.Length / secondMatrix.Rank;

        if (columnLength == 1)
        {
            result = new float[3, 1];
        }

        for (int i = 0; i < rowLength; i++)
        {
            for (int j = 0; j < columnLength; j++)
            {
                result[i, j] = firstMatrix[i, 0] * secondMatrix[0, j] + firstMatrix[i, 1] * secondMatrix[1, j];
            }
        }

        return result;
    }

    public float[,] Add2x2Matrices(float[,] firstMatrix, float[,] secondMatrix)
    {
        float[,] result = { { 0, 0 }, { 0, 0 } };
        int i, j, n;
        n = 2;
        for (i = 0; i < n; i++)
            for (j = 0; j < n; j++)
                result[i, j] = firstMatrix[i, j] + secondMatrix[i, j];
        return result;
    }

    public float[,] Add3x3Matrices(float[,] firstMatrix, float[,] secondMatrix)
    {
        float[,] result = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

        int i, j, n;
        n = 3;
        for (i = 0; i < n; i++)
            for (j = 0; j < n; j++)
                result[i, j] = firstMatrix[i, j] + secondMatrix[i, j];
        return result;
    }

    public float[,] Sub2x2Matrices(float[,] firstMatrix, float[,] secondMatrix)
    {
        float[,] result = { { 0, 0 }, { 0, 0 } };

        int i, j, n;
        n = 2;
        for (i = 0; i < n; i++)
            for (j = 0; j < n; j++)
                result[i, j] = firstMatrix[i, j] - secondMatrix[i, j];
        return result;
    }

    public float[,] Sub3x3Matrices(float[,] firstMatrix, float[,] secondMatrix)
    {
        float[,] result = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

        int i, j, n;
        n = 3;
        for (i = 0; i < n; i++)
            for (j = 0; j < n; j++)
                result[i, j] = firstMatrix[i, j] - secondMatrix[i, j];
        return result;
    }
}
