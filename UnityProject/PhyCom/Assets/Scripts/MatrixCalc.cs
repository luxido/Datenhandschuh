using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Drawing;
using System;
using System.Globalization;

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
                //line += decimal.Parse((arr[i, j]).ToString(), NumberStyles.Float) + " ";
                line += arr[i, j] + " ";
            }
            line += System.Environment.NewLine;
        }
        Debug.Log(line);
    }

    //modified from  https://stackoverflow.com/questions/6311309/how-can-i-multiply-two-matrices-in-c
    public float[,] MultiplyMatrix(float[,] A, float[,] B)
    {
        int rA = A.GetLength(0);
        int cA = A.GetLength(1);
        int rB = B.GetLength(0);
        int cB = B.GetLength(1);
        float temp = 0;
        float[,] kHasil = new float[rA, cB];
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

    public float[,] MultiplyMatrixWithScalar(float[,] A, float B)
    {
        int rA = A.GetLength(0);
        int cA = A.GetLength(1);
        float[,] kHasil = new float[rA, cA];
        for (int i = 0; i < rA; i++)
        {
            for (int k = 0; k < cA; k++)
            {
                kHasil[i, k] += A[i, k] * B;
            }
        }
        return kHasil;
    }

    //modified from  https://stackoverflow.com/questions/29483660/how-to-transpose-matrix
    public float[,] Transpose(float[,] matrix)
    {
        int w = matrix.GetLength(0);
        int h = matrix.GetLength(1);

        float[,] result = new float[h, w];

        for (int i = 0; i < w; i++)
        {
            for (int j = 0; j < h; j++)
            {
                result[j, i] = matrix[i, j];
            }
        }

        return result;
    }

    //modified from  http://programmertech.com/program/csharp/csharp-program-add-two-matrix
    public float[,] AddMatrix(float[,] matrix1, float[,] matrix2)
    {
        float[,] addition = null;
        if (matrix1 != null && matrix2 != null && matrix1.GetLength(0) == matrix2.GetLength(0) && matrix1.GetLength(1) == matrix2.GetLength(1))
        {
            addition = new float[matrix1.GetLength(0), matrix1.GetLength(1)];
            for (int row = 0; row < matrix1.GetLength(0); row++)
            {
                for (int col = 0; col < matrix1.GetLength(1); col++)
                {
                    addition[row, col] = matrix1[row, col] + matrix2[row, col];
                }
            }
        }
        else
        {
            Debug.Log("addition not Possible required rows and columns of both matrix equals");
        }
        return addition;
    }

    public float[,] SubMatrix(float[,] matrix1, float[,] matrix2)
    {
        float[,] addition = null;
        if (matrix1 != null && matrix2 != null && matrix1.GetLength(0) == matrix2.GetLength(0) && matrix1.GetLength(1) == matrix2.GetLength(1))
        {
            addition = new float[matrix1.GetLength(0), matrix1.GetLength(1)];
            for (int row = 0; row < matrix1.GetLength(0); row++)
            {
                for (int col = 0; col < matrix1.GetLength(1); col++)
                {
                    addition[row, col] = matrix1[row, col] - matrix2[row, col];
                }
            }
        }
        else
        {
            Debug.Log("addition not Possible required rows and columns of both matrix equals");
        }
        return addition;
    }

    public float calcAngle(float a, float b)
    {
        double radians, angle;
        radians = Math.Atan2(a,b);
        angle = radians / Math.PI * 180;
        return (float)angle;
    }

    //modified from http://www.rkinteractive.com/blogs/SoftwareDevelopment/post/2013/05/21/Algorithms-In-C-Finding-The-Inverse-Of-A-Matrix.aspx
    public float[,] InvertMatrix(float[,] A)
    {
        float[,] Stemp = A.Clone() as float[,];
        int n = Stemp.GetLength(0) ;
        //e will represent each column in the identity matrix
        float[] e;
        //x will hold the inverse matrix to be returned
        float[,] x = new float[n,n];

        /*
        * solve will contain the vector solution for the LUP decomposition as we solve
        * for each vector of x.  We will combine the solutions into the float[,] array x.
        * */
        float[] solve;

        //Get the LU matrix and P matrix (as an array)
        Tuple<float[,], int[]> results = LUPDecomposition(Stemp);
        float[,] LU = results.Item1;
        int[] P = results.Item2;

        /*
        * Solve AX = e for each column ei of the identity matrix using LUP decomposition
        * */
        for (int i = 0; i < n; i++)
        {
            e = new float[n];
            e[i] = 1;
            solve = LUPSolve(LU, P, e);
            for (int j = 0; j < solve.Length; j++)
            {
                x[j,i] = solve[j];
            }
        }
        return x;
    }

    private Tuple<float[,], int[]> LUPDecomposition(float[,] A)
    {
        /*
        * Perform LUP decomposition on a matrix A.
        * Return L and U as a single matrix(float[,]) and P as an array of ints.
        * We implement the code to compute LU "in place" in the matrix A.
        * In order to make some of the calculations more straight forward and to 
        * match Cormen's et al. pseudocode the matrix A should have its first row and first columns
        * to be all 0.
        * */
        int n = A.GetLength(0)-1;
        /*
        * pi represents the permutation matrix.  We implement it as an array
        * whose value indicates which column the 1 would appear.  We use it to avoid 
        * dividing by zero or small numbers.
        * */
        int[] pi = new int[n + 1];
        float p = 0;
        int kp = 0;
        int pik = 0;
        int pikp = 0;
        float aki = 0;
        float akpi = 0;

        //Initialize the permutation matrix, will be the identity matrix
        for (int j = 0; j <= n; j++)
        {
            pi[j] = j;
        }

        for (int k = 0; k <= n; k++)
        {
            /*
            * In finding the permutation matrix p that avoids dividing by zero
            * we take a slightly different approach.  For numerical stability
            * We find the element with the largest 
            * absolute value of those in the current first column (column k).  If all elements in
            * the current first column are zero then the matrix is singluar and throw an
            * error.
            * */
            p = 0;
            for (int i = k; i <= n; i++)
            {
                if (Math.Abs(A[i,k]) > p)
                {
                    p = Math.Abs(A[i,k]);
                    kp = i;
                }
            }
            /*
            if (p == 0)
            {
                throw new Exception("singular matrix");
            }*/
            /*
            * These lines update the pivot array (which represents the pivot matrix)
            * by exchanging pi[k] and pi[kp].
            * */
            pik = pi[k];
            pikp = pi[kp];
            pi[k] = pikp;
            pi[kp] = pik;

            /*
            * Exchange rows k and kpi as determined by the pivot
            * */
            for (int i = 0; i <= n; i++)
            {
                aki = A[k,i];
                akpi = A[kp,i];
                A[k,i] = akpi;
                A[kp,i] = aki;
            }

            /*
                * Compute the Schur complement
                * */
            for (int i = k + 1; i <= n; i++)
            {
                A[i,k] = A[i,k] / A[k,k];
                for (int j = k + 1; j <= n; j++)
                {
                    A[i,j] = A[i,j] - (A[i,k] * A[k,j]);
                }
            }
        }
        return Tuple.Create(A, pi);
    }

    private float[] LUPSolve(float[,] LU, int[] pi, float[] b)
    {
        /*
        * Given L,U,P and b solve for x.
        * Input the L and U matrices as a single matrix LU.
        * Return the solution as a float[].
        * LU will be a n+1xm+1 matrix where the first row and columns are zero.
        * This is for ease of computation and consistency with Cormen et al.
        * pseudocode.
        * The pi array represents the permutation matrix.
        * */
        int n = LU.GetLength(0)-1;
        float[] x = new float[n + 1];
        float[] y = new float[n + 1];
        float suml = 0;
        float sumu = 0;
        float lij = 0;

        /*
        * Solve for y using formward substitution
        * */
        for (int i = 0; i <= n; i++)
        {
            suml = 0;
            for (int j = 0; j <= i - 1; j++)
            {
                /*
                * Since we've taken L and U as a singular matrix as an input
                * the value for L at index i and j will be 1 when i equals j, not LU[i,j], since
                * the diagonal values are all 1 for L.
                * */
                if (i == j)
                {
                    lij = 1;
                }
                else
                {
                    lij = LU[i,j];
                }
                suml = suml + (lij * y[j]);
            }
            y[i] = b[pi[i]] - suml;
        }
        //Solve for x by using back substitution
        for (int i = n; i >= 0; i--)
        {
            sumu = 0;
            for (int j = i + 1; j <= n; j++)
            {
                sumu = sumu + (LU[i,j] * x[j]);
            }
            x[i] = (y[i] - sumu) / LU[i,i];
        }
        return x;
    }
}
