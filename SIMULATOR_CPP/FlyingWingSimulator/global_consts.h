
/**********************************************************
 * File: global_consts.h
 * Purpose: Establish static constant variables for use in entire simulation
 * Author(s): Weston Scott
 * Date Created: 9/25/2020
 * Date Modifed: 9/27/2020
 *********************************************************/

#pragma once

// Declare Matrix Data type --> 2D float vector
#include <vector>
#include <cmath>
#include <math.h>
typedef std::vector<std::vector<float>> Matrix;

/////////////////////////////////////////////
//Define Physical Airframe constants
static const float gravity = 9.8072f;       // m/s^2 gravity
static const float mass = 0.9f;           // kg airplane mass
static const float Jx = 0.1147f;             // (kg m2) airplane x moment of inertia
static const float Jy = 0.0576f;             // (kg m2) airplane y moment of inertia
static const float Jz = 0.1712f;             // (kg m2) airplane z moment of inertia
static const float Jxz = 0.0015f;            // (kg m2) airplane xz gyro moment

//////////////////////////////////////////////
//Define Gamma parameters from UAV book page 36
static const float Gamma = Jx* Jz - pow(Jxz, 2);
static const float Gamma1 = (Jxz* (Jx - Jy + Jz)) / Gamma;
static const float Gamma2 = (Jz* (Jz - Jy) + pow(Jxz, 2)) / Gamma;
static const float Gamma3 = Jz / Gamma;
static const float Gamma4 = Jxz / Gamma;
static const float Gamma5 = (Jz - Jx) / Jy;
static const float Gamma6 = Jxz / Jy;
static const float Gamma7 = ((Jx - Jy)* Jx + pow(Jxz, 2)) / Gamma;
static const float Gamma8 = Jx / Gamma;

/////////////////////////////////////////////////
//Aerodynmaic Coefficients
static const float S_wing = 0.2589f;          // (m^2) - wing area
static const float b = 1.4224f;               // (m) wingspan
static const float c = 0.3302f;               // (m) wing width
static const float S_prop = 0.0314f;
static const float rho = 1.2682f;           // (kg/m^3) air density
static const float k_motor = 20.0f;
static const float k_T_P = 0.0f;
static const float k_Omega = 0.0f;
static const float e = 0.9f;                // oswald efficiency factor
static const float AR = pow(b, 2) / S_wing; // Wing aspect ratio
static const float C_L_0 = 0.09167f;
static const float C_L_alpha = 3.5016f;       //  Lift coefficient
static const float C_L_q = 2.8932f;           //  Lift coefficient
static const float C_L_delta_e = 0.2724f;     //  Lift coefficient
static const float C_D_0 = 0.016131f;          //  drag coefficient
static const float C_D_alpha = 0.2108f;      //  drag coefficient
static const float C_D_p = 0.0254f;            //  drag coefficient
static const float C_D_q = 0.0f;            //  drag coefficient
static const float C_D_delta_e = 0.3045f;   //  drag coefficient
static const float C_m_0 = -0.02338f;         //  drag / lift moment coefficient 
static const float C_m_alpha = -0.5675f;      //  drag / lift moment coefficient 
static const float C_m_q = -1.399f;         //  drag / lift moment coefficient 
static const float C_m_delta_e = -0.3254f;    //  drag / lift moment coefficient 
static const float C_Y_0 = 0.0f;            //  Lateral force coefficient
static const float C_Y_beta = -0.07359f;       //  Lateral force coefficient
static const float C_Y_p = 0.0f;            //  Lateral force coefficient
static const float C_Y_r = 0.0f;            //  Lateral force coefficient
static const float C_Y_delta_a = 0.0f;    //  Lateral force coefficient
static const float C_Y_delta_r = 0.0f;     //  Lateral force coefficient                  ----------------? confirm
static const float C_ell_0 = 0.0f;          //  Lateral roll torque coefficient
static const float C_ell_beta = -0.02854f;     //  Lateral roll torque coefficient
static const float C_ell_p = -0.3209f;        //  Lateral roll torque coefficient
static const float C_ell_r = 0.03066f;        //  Lateral roll torque coefficient
static const float C_ell_delta_a = 0.1682f;   //  Lateral roll torque coefficient
static const float C_ell_delta_r = 0.0f;//  Lateral roll torque coefficient               ----------------? confirm
static const float C_n_0 = 0.0f;            //  Lateral yaw torque coefficient
static const float C_n_beta = -0.0004f;       //  Lateral yaw torque coefficient
static const float C_n_p = -0.01297f;         //  Lateral yaw torque coefficient
static const float C_n_r = -0.00434f;         //  Lateral yaw torque coefficient
static const float C_n_delta_a = -0.00328f;   //  Lateral yaw torque coefficient 
static const float C_n_delta_r = 0.0f;   // Lateral yaw torque coefficient               ----------------? confirm
static const float C_prop = 1.0f;
static const float M = 50.0f;               // blend function parameter
static const float epsilon = 0.1529f;
static const float alpha0 = 0.4712f;          // (rad)blend function parameter

////////////////////////////////////////////////////////
//Parameters for propulsion thrust and torque models
//Prop parameters
static const float D_prop = 9.0f * (0.0254f);   // (m) prop diameter

//Motor parameters
static const float K_V = 1180.0f;            // from datasheet RPM/V 
static const float M_PI = 3.14159265358979323846f;   // pi
static const float KQ = (1.0f / K_V) * 60.0f / (2.0f * M_PI);   // KQ in N - m / A, V - s / rad 
static const float R_motor = 0.042f;       // ohms                                          ----------------? confirm
static const float i0 = 0.5f;              // no - load(zero - torque) current(A)           ----------------? confirm

//Inputs
static const float ncells = 3.0f;         // Number of cells in the battery 
static const float V_max = 3.7f * ncells;  // max voltage for specified number of battery cells 

//Coeffiecients from prop_data fit
static const float C_Q2 = -0.01664f; // Propeller torque fitting parameter* /
static const float C_Q1 = 0.004970f; // Propeller torque fitting parameter* /
static const float C_Q0 = 0.005230f; // Propeller torque fitting parameter* /

static const float C_T2 = -0.1079f; // Propeller thrust fitting parameter* /
static const float C_T1 = -0.06044f; // Propeller thrust fitting parameter* /
static const float C_T0 = 0.09357f; // Propeller thrust fitting parameter* /