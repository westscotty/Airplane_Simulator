///**********************************************************
// * File: mexWindTunnel.cpp
// * Purpose: Mex S-Functions to verify equations of motion with figure in matlab
// * Author(s): Weston Scott
// * Date Created: 11/2/2020
// *********************************************************/
//#pragma once
//
//#include "mex.h"
//#include <cmath>
//#include <math.h>
//#include "global_consts.h"
//#include "plane_dynamics.cpp"
//#include "euler_quat.h"
//#include "plane_mechanics.h"
//
//void run_mex_sim(double dt, double eLeft, double eRight, double throttle, double* state, double* eul, double* output1, double* output2)
//{
//    // Create airplane object
//    Plane_Mechanics airplane;
//
//    // update airplane states
//    airplane.plane.pn = (float)state[0]; airplane.plane.pe = (float)state[1]; airplane.plane.pd = (float)state[2];
//    airplane.plane.u = (float)state[3]; airplane.plane.v = (float)state[4]; airplane.plane.w = (float)state[5];
//    airplane.plane.quat.e0 = (float)state[6]; airplane.plane.quat.e1 = (float)state[7];
//    airplane.plane.quat.e2 = (float)state[8]; airplane.plane.quat.e3 = (float)state[9];
//    airplane.plane.p = (float)state[10]; airplane.plane.q = (float)state[11]; airplane.plane.r = (float)state[12];
//    airplane.plane.eul = quaternion2Euler(Quaternion((float)state[7], (float)state[8], (float)state[9], (float)state[10]));
//
//    // Update elevons
//    airplane.control_vals.el_left = (float)eLeft;
//    airplane.control_vals.el_right = (float)eRight;
//    airplane.control_vals.del_t = (float)throttle;
//    airplane.convert_elevons();
//
//    airplane.plane.quat = euler2Quaternion(airplane.plane.eul);
//    airplane.update_va_alpha_beta();
//
//    Matrix x =
//    {
//        { airplane.plane.pn },
//        { airplane.plane.pe },
//        { airplane.plane.pd },
//        { airplane.plane.u },
//        { airplane.plane.v },
//        { airplane.plane.w },
//        { airplane.plane.quat.e0 },
//        { airplane.plane.quat.e1 },
//        { airplane.plane.quat.e2 },
//        { airplane.plane.quat.e3 },
//        { airplane.plane.p },
//        { airplane.plane.q },
//        { airplane.plane.r }
//    };
//
//    // Calculate Forces and Torques
//    airplane.update_comb_forces();
//
//    // Runge Kutta 4th Order Method --------------
//    Matrix k1 = airplane_dynamics(x, airplane);
//    Matrix k1x = matrix_add(x, k1, (float)dt, 0);
//    Matrix k2 = airplane_dynamics(k1x, airplane);
//    Matrix k2x = matrix_add(x, k2, (float)dt, 0);
//    Matrix k3 = airplane_dynamics(k2x, airplane);
//    Matrix k3x = matrix_add(x, k3, (float)dt, 1);
//    Matrix k4 = airplane_dynamics(k3x, airplane);
//
//    // Update x state
//    for (int m = 0; m < 13; m++)
//    {
//        x[m][0] += (k1[m][0] + 2 * k2[m][0] + 2 * k3[m][0] + k4[m][0]) * (float)dt / 6.0f;
//        output1[m] = (double)x[m][0];
//    }
//
//    output2[0] = (double)airplane.plane.eul.phi;
//    output2[1] = (double)airplane.plane.eul.theta;
//    output2[2] = (double)airplane.plane.eul.psi;
//}
//
//void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
//{
//    // Initialize variable declarations
//    double dt;      // time step
//    double eLeft;   // left elevon value
//    double eRight;  // right elevon value
//    double throttle;  // throttle value
//    double* state;  // read current state estimates
//    double* eul;
//    mwSize n1;  // number of columns in state variable
//    mwSize n2;
//    // import values from MATLAB
//    dt = mxGetScalar(prhs[0]);
//    eLeft = mxGetScalar(prhs[1]);
//    eRight = mxGetScalar(prhs[2]);
//    throttle = mxGetScalar(prhs[3]);
//    state = mxGetDoubles(prhs[4]);
//    eul = mxGetDoubles(prhs[5]);
//    n1 = mxGetN(prhs[4]);
//    n2 = mxGetN(prhs[5]);
//
//    // error checking
//    if (n1 != 13)
//    {
//        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs",
//            "Input state array not correct size.");
//    }
//
//    // Create output state matrix
//    double* output1;
//    double* output2;
//    plhs[0] = mxCreateDoubleMatrix(1, n1, mxREAL);
//    plhs[1] = mxCreateDoubleMatrix(1, n2, mxREAL);
//    output1 = mxGetDoubles(plhs[0]);
//    output2 = mxGetDoubles(plhs[1]);
//
//    run_mex_sim(dt, eLeft, eRight, throttle, state, eul, output1, output2);
//}