///**********************************************************
// * File: mexWindTunnel.cpp
// * Purpose: Mex S-Functions to create wind tunnel graphs in MATLAB
// * Author(s): Weston Scott
// * Date Created: 11/2/2020
// *********************************************************/
//
//#include "mex.h"
//#include "global_consts.h"
//
//
//void LiftDrag(double q, mwSize n, mwSize m, double *alpha, double *del_e, double *Va, double *output1, double* output2, double* output3)
//{
//    for (int i = 0; i < m; i++)
//    {
//        for (int j = 0; j < n; j++)
//        {
//            double C_D = C_D_p + pow(C_L_0 + C_L_alpha * alpha[i + m * j], 2) / (M_PI * e * AR);
//            double Sigma = (1.0f + exp(-M * (alpha[i + m * j] - alpha0)) + exp(M * (alpha[i + m * j] + alpha0))) / ((1.0f + exp(-M * (alpha[i + m * j] - alpha0))) * (1.0f + exp(M * (alpha[i + m * j] + alpha0))));
//            double C_L = (1.0f - Sigma) * (C_L_0 + C_L_alpha * alpha[i + m * j]) + Sigma * (2.0f * copysign(1.0f, alpha[i + m * j]) * pow(sin(alpha[i + m * j]), 2) * cos(alpha[i + m * j]));
//
//            // Calculate Drag and Lift Forces
//            double Flift = 0.5f * rho * pow(Va[i + m * j], 2) * S_wing * (C_L + C_L_q * c * q / 2.0f / Va[i + m * j] + C_L_delta_e * del_e[i + m * j]);
//            double Fdrag = 0.5f * rho * pow(Va[i + m * j], 2) * S_wing * (C_D + C_D_q * c * q / 2.0f / Va[i + m * j] + C_D_delta_e * del_e[i + m * j]);
//
//            // Convert Drag and Lift Forces to body frame
//            double fxLD = ((cos(alpha[i + m * j]) * -Fdrag) + (-sin(alpha[i + m * j]) * -Flift));
//            double fzLD = ((sin(alpha[i + m * j]) * -Fdrag) + (cos(alpha[i + m * j]) * -Flift));
//            double TyLD = (0.5f * rho * pow(Va[i + m * j], 2) * S_wing * c * (C_m_0 + C_m_alpha * alpha[i + m * j] + C_m_q * c * q / 2.0f / Va[i + m * j] + C_m_delta_e * del_e[i + m * j]));
//            
//            output1[i + m * j] = fxLD;
//            output2[i + m * j] = fzLD;
//            output3[i + m * j] = TyLD;
//        }
//    }
//}
//
//void LateralDynamics(double beta, double p, double r, mwSize n, mwSize m, double *del_a, double *del_r, double *Va, double *output1, double *output2, double *output3)
//{
//    for (int i = 0; i < m; i++)
//    {
//        for (int j = 0; j < n; j++)
//        {
//            double fyLat = (0.5f * rho * pow(Va[i + m * j], 2) * S_wing * (C_Y_0 + C_Y_beta * beta + C_Y_p * b * p / 2.0f / Va[i + m * j] + C_Y_r * b * r / 2.0f / Va[i + m * j] + C_Y_delta_a * del_a[i + m * j] + C_Y_delta_r * del_r[i + m * j]));
//            double TxLat = (0.5f * rho * pow(Va[i + m * j], 2) * S_wing * b * (C_ell_0 + C_ell_beta * beta + C_ell_p * b * p / 2.0f / Va[i + m * j] + C_ell_r * b * r / 2.0f / Va[i + m * j] + C_ell_delta_a * del_a[i + m * j] + C_ell_delta_r * del_r[i + m * j]));
//            double TzLat = (0.5f * rho * pow(Va[i + m * j], 2) * S_wing * b * (C_n_0 + C_n_beta * beta + C_n_p * b * p / 2.0f / Va[i + m * j] + C_n_r * b * r / 2.0f / Va[i + m * j] + C_n_delta_a * del_a[i + m * j] + C_n_delta_r * del_r[i + m * j]));
//            
//            output1[i + m * j] = fyLat;
//            output2[i + m * j] = TxLat;
//            output3[i + m * j] = TzLat;
//        }
//    }
//}
//
///* The gateway function */
//void mexFunction(int nlhs, mxArray* plhs[],
//    int nrhs, const mxArray* prhs[])
//{
//    /* variable declarations here */
//    double test_type;
//    double* output1;      /* output matrix */
//    double* output2;      /* output matrix */
//    double* output3;      /* output matrix */
//    
//    // LiftDrag(0) or LaternalDynamics(1) Wind Tunnel Test
//    test_type = mxGetScalar(prhs[0]);
//    if (int(test_type) == 0) {
//        // mexWindTunnel(test_type, q, alpha, delta_e, Va) --> call method
//
//        double q;                // (rad / s) initial body frame pitch rate
//        double* alpha;           // Angle of attack
//        double* delta_e;         // elevator angle (delta_e)
//        double* Va;              // airspeed
//
//        /* get the value of the scalar input  */
//        q = mxGetScalar(prhs[1]);
//
//        /* create a pointer to the real data in the input matrices  */
//        alpha = (double*)mxGetData(prhs[2]);
//        delta_e = (double*)mxGetData(prhs[3]);
//        Va = (double*)mxGetData(prhs[4]);
//
//        // matrix row sizes
//        mwSize m1, m2, m3;
//
//        // matrix column sizes
//        mwSize n1, n2, n3;
//
//        /* get dimensions of the input matrices */
//        m1 = mxGetM(prhs[2]); n1 = mxGetN(prhs[2]);
//        m2 = mxGetM(prhs[3]); n2 = mxGetN(prhs[3]);
//        m3 = mxGetM(prhs[4]); n3 = mxGetN(prhs[4]);
//
//        // error checking
//        //if ((n1 != n2 != n3) || (m1 != m2 != m3)
//        //{
//        //    mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs",
//        //        "Input arrays must be of equal sizes.");
//        //}
//        
//        /* create the output matrix */
//        plhs[0] = mxCreateDoubleMatrix(m1, n1, mxREAL);
//        plhs[1] = mxCreateDoubleMatrix(m1, n1, mxREAL);
//        plhs[2] = mxCreateDoubleMatrix(m1, n1, mxREAL);
//
//        /* get a pointer to the real data in the output matrix */
//        output1 = mxGetDoubles(plhs[0]);
//        output2 = mxGetDoubles(plhs[1]);
//        output3 = mxGetDoubles(plhs[2]);
//
//        /* call the computational routine */
//
//        LiftDrag(q, n1, m1, alpha, delta_e, Va, output1, output2, output3);
//    }
//    else if (int(test_type) == 1) {
//        // mexWindTunnel(test_type, beta, p, r, delta_r, delta_a, Va) --> call method
//
//        double beta;                // (rad / s) initial body frame pitch rate
//        double p;
//        double r;
//        double* delta_r;           // Angle of rudder
//        double* delta_a;           // aileron angle
//        double* Va;                // airspeed
//
//        /* get the value of the scalar input  */
//        beta = mxGetScalar(prhs[1]);
//        p = mxGetScalar(prhs[2]);
//        r = mxGetScalar(prhs[3]);
//
//        /* create a pointer to the real data in the input matrices  */
//        delta_r = (double*)mxGetData(prhs[4]);
//        delta_a = (double*)mxGetData(prhs[5]);
//        Va = (double*)mxGetData(prhs[6]);
//
//        // matrix row sizes
//        mwSize m1, m2, m3;
//
//        // matrix column sizes
//        mwSize n1, n2, n3;
//
//        /* get dimensions of the input matrices */
//        m1 = mxGetM(prhs[4]); n1 = mxGetN(prhs[4]);
//        m2 = mxGetM(prhs[5]); n2 = mxGetN(prhs[5]);
//        m3 = mxGetM(prhs[6]); n3 = mxGetN(prhs[6]);
//
//        // error checking
//        //if ((n1 != n2 != n3) || (m1 != m2 != m3)
//        //{
//        //    mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs",
//        //        "Input arrays must be of equal sizes.");
//        //}
//
//        /* create the output matrix */
//        plhs[0] = mxCreateDoubleMatrix(m1, n1, mxREAL);
//        plhs[1] = mxCreateDoubleMatrix(m1, n1, mxREAL);
//        plhs[2] = mxCreateDoubleMatrix(m1, n1, mxREAL);
//
//        /* get a pointer to the real data in the output matrix */
//        output1 = mxGetDoubles(plhs[0]);
//        output2 = mxGetDoubles(plhs[1]);
//        output3 = mxGetDoubles(plhs[2]);
//
//        /* call the computational routine */
//
//        LateralDynamics(beta, p, r, n1, m1, delta_a, delta_r, Va, output1, output2, output3);
//    }
//}
//
//
//
