#include "allan_variance.h"

int allan_variance(Eigen::MatrixXf gyro_values, Eigen::MatrixXf &allan_var)
{
    Eigen::MatrixXf gyro_proms;
    Eigen::Vector3f gyro_aux, temp;
    int m, k, l;
    int s;
    int i, j, N;
    N = gyro_values.rows();
    Eigen::MatrixXf A(N,3),B(N,3);

    for(k=1; k <= N/2; k++)                         // k=N/2 means minimum of 2 subgroups
    {
        m = N / k;                                  // number of subgroups
        for(s = 0; s < m; s++)                      // average subgroups
        {
            gyro_aux.setZero();
            for(l = s * k; l < s * k + k; l++)      // short term variance, use adjacent groups
            {
                gyro_aux += gyro_values.row(l);
            }
            A.row(s) = gyro_aux / k;
        }
        gyro_aux.setZero();
        m --;
        for(s = 0; s < m; s++)
        {
            gyro_aux(0) += (A(s+1,0) - A(s,0)) * (A(s+1,0) - A(s,0));
            gyro_aux(1) += (A(s+1,1) - A(s,1)) * (A(s+1,1) - A(s,1));
            gyro_aux(2) += (A(s+1,2) - A(s,2)) * (A(s+1,2) - A(s,2));
        }
        B.row(k-1) = gyro_aux / (2 * m);
    }
    
    B.conservativeResize(k-1,3);
    allan_var = B;
    return 0;
}