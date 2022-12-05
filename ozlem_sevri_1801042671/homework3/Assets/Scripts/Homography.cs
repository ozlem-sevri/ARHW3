using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Numerics.LinearAlgebra.Double;

public class Homography : MonoBehaviour
{
    
    public static double[,] HomographyMatrix(double[,] s, double[,] d)
    {
        double [] x = CalcHomography(s, d);
        double[,] hm = new double[3, 3];

        int row = 0;
        for(int i = 0; i < x.Length; i++)
        {
            if (i % 3 == 0 && i != 0)
                ++row;
            hm[row, i % 3] = x[i];
        }
        return hm;
    }

    private static double[] CalcHomography(double[,] s, double[,] d)
    {
        int N = s.GetLength(0);
        int M = 9;
        double[,] A = new double[2 * N, M];

        int p = 0;
        for(int i = 0; i < 2 * N; i++)
        {
            if(i % 2 == 0)
            {
                A[i, 0] = -1 * s[p, 0];
                A[i, 1] = -1 * s[p, 1];
                A[i, 2] = -1;
                A[i, 3] = 0;
                A[i, 4] = 0;
                A[i, 5] = 0;
                A[i, 6] = s[p, 0] * d[p, 0];
                A[i, 7] = s[p, 1] * d[p, 0];
                A[i, 8] = d[p, 0];
            }
            else
            {
                A[i, 0] = 0;
                A[i, 1] = 0;
                A[i, 2] = 0;
                A[i, 3] = -1 * s[p, 0];
                A[i, 4] = -1 * s[p, 1];
                A[i, 5] = -1;
                A[i, 6] = s[p, 0] * d[p, 1];
                A[i, 7] = d[p, 1] * s[p, 1];
                A[i, 8] = d[p, 1];
                ++p;
            }
        }
        var mat = DenseMatrix.OfArray(A);
        var svd = mat.Svd(true);

        return svd.VT.Row(svd.VT.RowCount - 1).ToArray();

    }
    
    public static double[,] Projection(double[,] hm, double[,] xy,bool flag)
    {
        double[,] match = ApplyHomography(hm, xy);
        if (flag)
        {
            Debug.Log("Applying Projection");
            Debug.Log("(x,y) : " + xy[0, 0] + " , " + xy[1, 0] + " , " + xy[2, 0]);
            Debug.Log("(u,v) : " + match[0, 0] + " , " + match[1, 0] + " , " + match[2, 0]);

        }
        return match;
    }
    
    private static double [,] ApplyHomography(double[,]hm, double[,] a)
    {
        double[,] temp = MatrixMult(hm, a);
        double[,] res = new double[,] { { temp[0, 0] / temp[2, 0] }, { temp[1, 0] / temp[2, 0] }, { 1 } };
        return res;
    }
   
    private static double[,] MatrixMult(double[,] a, double[,] b)
    {
        int m = a.GetLength(0);
        int n = b.GetLength(1);

        double[,] res = new double[m,n];
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                res[i, j] = 0;
                for (int k = 0; k < a.GetLength(1); k++)
                {
                    res[i, j] += a[i, k] * b[k, j];
                }
            }
        }
        return res;
    }

    public static double[,] InverseProjection(double[,] hm, double[,] uv, bool flag)
    {
        double[,] match = ApplyInverseHomography(hm, uv);
        if (flag)
        {
            Debug.Log("Applying Inverse Projection");
            Debug.Log("(u,v) : " + uv[0, 0] + " , " + uv[1, 0] + " , " + uv[2, 0]);
            Debug.Log("(x,y) : " + match[0, 0] + " , " + match[1, 0] + " , " + match[2, 0]);
        }
        return match;
    }
    
    private static double[,] ApplyInverseHomography(double[,] hm, double[,] a)
    {
        var m = DenseMatrix.OfArray(hm).Inverse();
        return ApplyHomography(m.ToArray(), a);
    }
}
