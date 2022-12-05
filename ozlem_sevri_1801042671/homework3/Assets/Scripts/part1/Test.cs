using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Test : MonoBehaviour
{
    public double[,] s = new double[4, 2] { { 1, 2 }, { 5, 3 }, { 4, 10 }, { 1, 6 } };
    public double[,] d = new double[4, 2] { { 1, 2 }, { 5, 3 }, { 4, 10 }, { 1, 6 } };

    public double[,] s2 = new double[5, 2] { { 100, 100 }, { 300, 200 }, { 100, 400 }, { 200, 500 }, { 200, 300 } };

    // image 1
    public double[,] d1 = new double[5, 2] { { 755, 706 }, { 1221, 933 }, { 767, 1393 }, { 997, 1624 }, { 994, 1166 } };
    // image 2
    public double[,] d2 = new double[5, 2] { { 740, 508 }, { 1208, 732 }, { 724, 1208 }, { 960, 1448 }, { 968, 968 } };
    // image 3
    public double[,] d3 = new double[5, 2] { { 876, 748 }, { 1376, 980 }, { 924, 1460 }, { 1168, 1664 }, { 1148, 1220 } };
    // Start is called before the first frame update
    void Start()
    {
        double[,] hm = Homography.HomographyMatrix(s, d);

        string str = "";

        for(int i = 0; i < hm.GetLength(0); i++)
        {
            for (int j = 0; j < hm.GetLength(1); j++)
                str += hm[i, j] + "     ";
            str += "\n";
        }
       
        //1.1
        Debug.Log("Homography Matrix : \n\n" + str);
       
        //1.3
        double[,] xy = new double[3, 1] { { s[2, 0] }, { s[2, 1] }, { 1 } };
        Homography.Projection(hm, xy, true);
       
        //1.4
        double[,] uv = new double[3, 1] { { d[1, 0] }, { d[1, 1] }, { 1 } };
        Homography.InverseProjection(hm, uv, true);

        //1.5
        List<double[,]> P = new List<double[,]>();  // points (x, y)
        List<double[,]> A = new List<double[,]>(); // actual point correspondences (u, v)
        double[,] hm1 = Homography.HomographyMatrix(s2, d1);
        double[,] hm2 = Homography.HomographyMatrix(s2, d2);
        double[,] hm3 = Homography.HomographyMatrix(s2, d3);

        P.Add(new double[3, 1] { { 900 }, { 100 }, { 1 } });
        P.Add(new double[3, 1] { { 800 }, { 300 }, { 1 } });
        P.Add(new double[3, 1] { { 700 }, { 400 }, { 1 } });

        Debug.Log("Projecting Image 1");
        A.Add(new double[3, 1] { { 2612 }, { 681 }, { 1 } });
        A.Add(new double[3, 1] { { 2379 }, { 1151 }, { 1 } });
        A.Add(new double[3, 1] { { 2145 }, { 1381 }, { 1 } });

        MatchAndErrorCalc(hm1, s, d1, P, A);

        Debug.Log("Projecting Image 2");
        A = new List<double[,]>(); // actual point correspondences (u, v)

        A.Add(new double[3, 1] { { 2612 }, { 476 }, { 1 } });
        A.Add(new double[3, 1] { { 2396 }, { 948 }, { 1 } });
        A.Add(new double[3, 1] { { 2160 }, { 988 }, { 1 } });

        MatchAndErrorCalc(hm2, s, d2, P, A);
 
        Debug.Log("Projecting Image 3");
        A = new List<double[,]>(); // actual point correspondences (u, v)

        A.Add(new double[3, 1] { { 2664 }, { 712 }, { 1 } });
        A.Add(new double[3, 1] { { 2444 }, { 1152 }, { 1 } });
        A.Add(new double[3, 1] { { 2240 }, { 1376 }, { 1 } });

        MatchAndErrorCalc(hm3, s, d3, P, A);

        //1.6
        double[,] p1 = new double[3, 1] { { 7.5f }, { 5.5f }, { 1 } };
        double[,] p2 = new double[3, 1] { { 6.3f }, { 3.3f }, { 1 } };
        double[,] p3 = new double[3, 1] { { 0.1f }, { 0.1f }, { 1 } };

        Debug.Log("Projection For Image 1");
        Homography.Projection(hm1, p1, true);
        Homography.Projection(hm1, p2, true);
        Homography.Projection(hm1, p3, true);
        Debug.Log("Projection For Image 2");
        Homography.Projection(hm2, p1, true);
        Homography.Projection(hm2, p2, true);
        Homography.Projection(hm2, p3, true);
        Debug.Log("Projection For Image 3");
        Homography.Projection(hm3, p1, true);
        Homography.Projection(hm3, p2, true);
        Homography.Projection(hm3, p3, true);

        //1.7
        double[,] i1 = new double[3, 1] { { 500 }, { 400 }, { 1 } };
        double[,] i2 = new double[3, 1] { { 86 }, { 167 }, { 1 } };
        double[,] i3 = new double[3, 1] { { 10 }, { 10 }, { 1 } };

        Debug.Log("Inverse Projection For Image 1");
        Homography.InverseProjection(hm1, i1, true);
        Homography.InverseProjection(hm1, i2, true);
        Homography.InverseProjection(hm1, i3, true);

        Debug.Log("Inverse Projection For Image 2");
        Homography.InverseProjection(hm2, i1, true);
        Homography.InverseProjection(hm2, i2, true);
        Homography.InverseProjection(hm2, i3, true);

        Debug.Log("Inverse Projection For Image 3");
        Homography.InverseProjection(hm3, i1, true);
        Homography.InverseProjection(hm3, i2, true);
        Homography.InverseProjection(hm3, i3, true);
    }

    private void MatchAndErrorCalc(double[,] hm, double[,] s, double[,] d, List<double[,]> P, List<double[,]> A)
    {
        string str = "";
        for (int i = 0; i < hm.GetLength(0); i++)
        {
            for (int j = 0; j < hm.GetLength(1); j++)
                str += hm[i, j] + "   ";
            str += "\n";
            
        }
        Debug.Log("Calculated Homography Matrix : \n\n" + str);
        Debug.Log("Calculating Projection of Points");

        int k = 0;
        foreach (double[,] xy in P)
        {
            double[,] uv = A[k++];
            var res = Homography.Projection(hm, xy, true);
            Debug.Log("Error : %" + ProjectionError(res, uv));
        }
    }

    public float ProjectionError(double[,] res, double[,] real)
    {
        var x1 = res[0, 0];
        var x2 = real[0, 0];
        var y1 = res[1, 0];
        var y2 = real[1, 0];

        float resDif = Mathf.Sqrt(Mathf.Pow((float)(x1), 2) + Mathf.Pow((float)(y1), 2));
        float realDif = Mathf.Sqrt(Mathf.Pow((float)(x2), 2) + Mathf.Pow((float)(y2), 2));

        return Mathf.Abs((realDif - resDif) / realDif * 100);
    }
}
