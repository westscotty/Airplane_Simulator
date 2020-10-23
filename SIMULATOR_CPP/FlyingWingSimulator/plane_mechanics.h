
/**********************************************************
 * File: plane_mechanics.h
 * Purpose: Establish Plane Mechanics Object struct (and further structs containing data members)
 * Author(s): Weston Scott
 * Date Created: 9/25/2020
 * Date Modifed: 9/28/2020
 *********************************************************/
#pragma once

#include <iostream>
#include <iomanip>
#include <cmath>
#include <math.h>
#include "global_consts.h"
#include "euler_quat.h"

struct Plane_Mechanics
{
	struct Contoller_Values
	{
		// Values from the controller
		float del_e = 0.0f;    // (rad) elevator    -(readVoltage(a, 'A0')-2.5)*45/2.5*pi/180;
		float del_r = 0.0f;    // (rad) rudder      -(readVoltage(a, 'A1')-2.3)*30/2.5*pi/180;
		float del_t = 0.001f;    // throttle (0 - 1)   (readVoltage(a, 'A2'))/5;
		float del_a = 0.0f;    // (rad) aileron      (readVoltage(a, 'A3')-2.5709)*20/1.67*pi/180;
		float el_left = 0.01f;  // (rad) left elevon
		float el_right = 0.01f; // (rad) right elevon
	} control_vals;

	struct Airplane_State
	{
		// Initial Condition Parameters
		float pn = 0.0f;             // (m)initial North position
		float pe = 0.0f;             // (m)initial East position
		float pd = -10.0f;          // (m)initial Down position(negative altitude)
		float u = 17.0f;              // (m / s) initial velocity along body x - axis
		float v = 0.00001f;              // (m / s) initial velocity along body y - axis
		float w = 0.00001f;              // (m / s) initial velocity along body z - axis
		float p = 0.00001f;              // (rad/s) initial body frame roll drate
		float q = 0.00001f;              // (rad/s) initial body frame pitch rate
		float r = 0.00001f;              // (rad/s) initial body frame yaw rate
		float Va = 0.00001f;             // (m/s) airspeed
		float alpha = 0.000001f;          // (rad) angle of attack
		float beta = 0.000001f;           // (rad) side slip angle
		Euler eul;
		Quaternion quat;
	} plane;

	struct Gravity_Components
	{
		// GRAVITY FORCES (N)
		float fxg = 0.0f;
		float fyg = 0.0f;
		float fzg = 0.0f;
	}  grav_comps;

	struct Lift_Drag_Components
	{
		// LIFT AND DRAG FORCES (N) AND TORQUES (N*m)
		// -- NOTE: THE FUNCTION lift_drag SETS fxLD = -fxDrag BECAUSE A POSITIVE DRAG FORCE IS A NEGATIVE BODY X-FORCE
		float fxLD = 0.0f;
		float fzLD = 0.0f;
		float TyLD = 0.0f;
	} ld_comps;

	struct Lateral_Components
	{
		// LATERAL FORCES (N) AND TORQUES (N*m)
		float fyLat = 0.0f;
		float TxLat = 0.0f;
		float TzLat = 0.0f;
	} lat_comps;

	struct Prop_Components
	{
		// PROPELLER FORCES (N) AND TORQUES (N*m)
		float fxp = 0.0f;
		float Txp = 0.0f;
	} prop_comps;
	
	struct Combined_Forces
	{
		// Combined Forces & Torques struct object
		float fx = 0.0f;
		float fy = 0.0f;
		float fz = 0.0f;
		float Tx = 0.0f;
		float Ty = 0.0f;
		float Tz = 0.0f;
	} comb_forces;

	// Fuctions to calculate force components
	void gravity_forces();
	void lift_drag();
	void lateral_dynamics();
	void propeller_thrust_torques();

	// Update Vals
	void update_comb_forces();
	void update_va_alpha_beta();
	void convert_elevons();
};