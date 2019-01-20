using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MatrixCalc : MonoBehaviour {

    // Use this for initialization
    
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
