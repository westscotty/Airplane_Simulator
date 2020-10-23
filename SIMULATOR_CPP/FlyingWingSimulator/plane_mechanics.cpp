
/**********************************************************
 * File: plane_mechanics.cpp
 * Purpose: Initilize forces_torques objects
 * Functions: 1) gravity_forces
 *			  2) lift_drag
 *			  3) lateral_dynamics
 *			  4) propeller_thrust_torque
 *            5) update_comb_forces
 *            6) update_vel_alpha_beta
 * Author(s): Weston Scott
 * Date Created: 9/25/2020
 * Date Modifed: 9/28/2020
 *********************************************************/
#pragma once

#include <iostream>
#include <iomanip>
#include <cmath>
#include <math.h>
#include "plane_mechanics.h"
#include "global_consts.h"
#include "euler_quat.h"

using namespace std;

// Force and Torque Component Functions
void Plane_Mechanics::gravity_forces()
{
	grav_comps.fxg = (mass * gravity * 2.0f * (plane.quat.e1 * plane.quat.e3 - plane.quat.e2 * plane.quat.e0));
	grav_comps.fyg = (mass * gravity * 2.0f * (plane.quat.e2 * plane.quat.e3 + plane.quat.e1 * plane.quat.e0));
	grav_comps.fzg = (mass * gravity * (pow(plane.quat.e3 , 2) + pow(plane.quat.e0, 2) - pow(plane.quat.e1, 2) - pow(plane.quat.e2, 2)));
}

void Plane_Mechanics::lift_drag()
{
	// Calculate Drag and Lift coefficients
	float C_D = C_D_p + pow(C_L_0 + C_L_alpha * plane.alpha,2) / (M_PI * e * AR);
	float Sigma = (1.0f + exp(-M * (plane.alpha - alpha0)) + exp(M * (plane.alpha + alpha0))) / ((1.0f + exp(-M * (plane.alpha - alpha0))) * (1.0f + exp(M * (plane.alpha + alpha0))));
	float C_L = (1.0f - Sigma) * (C_L_0 + C_L_alpha * plane.alpha) + Sigma * (2.0f * copysign(1.0f, plane.alpha) * pow(sin(plane.alpha),2) * cos(plane.alpha));

	// Calculate Drag and Lift Forces
	float Flift = 0.5f * rho * pow(plane.Va,2) * S_wing * (C_L + C_L_q * c * plane.q / 2.0f / plane.Va + C_L_delta_e * control_vals.del_e);
	float Fdrag = 0.5f * rho * pow(plane.Va,2) * S_wing * (C_D + C_D_q * c * plane.q / 2.0f / plane.Va + C_D_delta_e * control_vals.del_e);

	// Convert Drag and Lift Forces to body frame
	ld_comps.fxLD = ((cos(plane.alpha) * -Fdrag) + (-sin(plane.alpha) * -Flift));
	ld_comps.fzLD = ((sin(plane.alpha) * -Fdrag) + (cos(plane.alpha) * -Flift));
	ld_comps.TyLD = (0.5f * rho * pow(plane.Va,2) * S_wing * c * (C_m_0 + C_m_alpha * plane.alpha + C_m_q * c * plane.q / 2.0f / plane.Va + C_m_delta_e * control_vals.del_e));
}

void Plane_Mechanics::lateral_dynamics()
{
	lat_comps.fyLat = (0.5f * rho * pow(plane.Va, 2) * S_wing * (C_Y_0 + C_Y_beta * plane.beta + C_Y_p * b * plane.p / 2.0f / plane.Va + C_Y_r * b * plane.r / 2.0f / plane.Va + C_Y_delta_a * control_vals.del_a + C_Y_delta_r * control_vals.del_r));
	lat_comps.TxLat = (0.5f * rho * pow(plane.Va, 2) * S_wing * b * (C_ell_0 + C_ell_beta * plane.beta + C_ell_p * b * plane.p / 2.0f / plane.Va + C_ell_r * b * plane.r / 2.0f / plane.Va + C_ell_delta_a * control_vals.del_a + C_ell_delta_r * control_vals.del_r));
	lat_comps.TzLat = (0.5f * rho * pow(plane.Va, 2) * S_wing * b * (C_n_0 + C_n_beta * plane.beta + C_n_p * b * plane.p / 2.0f / plane.Va + C_n_r * b * plane.r / 2.0f / plane.Va + C_n_delta_a * control_vals.del_a + C_n_delta_r * control_vals.del_r));
}

void Plane_Mechanics::propeller_thrust_torques()
{
	float Vin = V_max * control_vals.del_t;  //Voltage into the motor

	// The following are aa, bb, and cc coefficients for quadratic equation --> ax ^ 2 + bx + c = 0
	float aa = C_Q0 * rho * pow(D_prop, 5) / pow((2.0f * M_PI), 2);
	float bb = C_Q1 * plane.Va * rho * pow(D_prop, 4) / (2.0f * M_PI) + pow(KQ, 2) / R_motor;
	float cc = rho * pow(D_prop, 3) * C_Q2 * pow(plane.Va, 2) - KQ / R_motor * Vin + KQ * i0;

	float OPspeed = (-bb + sqrt(pow(bb, 2) - 4.0f * aa * cc)) / (2.0f * aa);  // (rad / s) Operating speed.Solved for using quadratic formula
	float n = OPspeed / (2.0f * M_PI);  // Propeller speed(rev / sec)
	float J_op = 2.0f * M_PI * plane.Va / (OPspeed * D_prop);  // unitless advance ratio
	float C_T = C_T2 * pow(J_op, 2) + C_T1 * J_op + C_T0;  // Experimental curve fit stuff
	float C_Q = C_Q2 * pow(J_op, 2) + C_Q1 * J_op + C_Q0;

	prop_comps.fxp = (rho * pow(n, 2) * pow(D_prop, 4) * C_T);  // Thrust force provided by propeller
	prop_comps.Txp = (rho * pow(n, 2) * pow(D_prop, 5) * C_Q);  // (N*m) Torque along propeller axis
}

void Plane_Mechanics::update_comb_forces()
{
	gravity_forces();
	lift_drag();
	lateral_dynamics();
	propeller_thrust_torques();

	comb_forces.fx = grav_comps.fxg + ld_comps.fxLD + prop_comps.fxp;
	comb_forces.fy = grav_comps.fyg + lat_comps.fyLat;
	comb_forces.fz = grav_comps.fzg + ld_comps.fzLD;
	comb_forces.Tx = lat_comps.TxLat + prop_comps.Txp;
	comb_forces.Ty = ld_comps.TyLD;
	comb_forces.Tz = lat_comps.TzLat;
}

void Plane_Mechanics::update_va_beta_alpha()
{
	plane.Va = sqrt(pow(plane.u, 2) + pow(plane.v, 2) + pow(plane.w, 2));
	plane.alpha = atan(plane.w / plane.u);
	plane.beta = asin(plane.v / plane.Va);
}

void Plane_Mechanics::convert_elevons()
{
	control_vals.del_e = 0.5f * (control_vals.el_right + control_vals.el_left);
	control_vals.del_a = 0.5f * (-control_vals.el_right + control_vals.el_left);
}