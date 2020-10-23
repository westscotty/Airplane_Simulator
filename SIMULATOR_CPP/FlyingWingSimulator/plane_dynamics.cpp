
/**********************************************************
 * File: plane_dynamics.cpp
 * Purpose: Initilize airplane.airairplane objects
 * Functions: 1) airairplane_dynamics
 *			  4) matrix_mult
 *			  5) matrix_add
 * Author(s): Weston Scott
 * Date Created: 9/26/2020
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

//using namespace std;

// Add Matrices together
inline Matrix matrix_add(const Matrix& aa, const Matrix& bb, const float dt, const int index)
{
	Matrix cc(aa.size(), std::vector<float>(1, 0.0)); // Preallocate a vector of n rows (size) of 1 column with a default value of 0.0 (13:1)
	if (index == 0)
	{
		for (int i = 0; i < aa.size(); i++)
		{	
			cc[i][0] = aa[i][0] + bb[i][0] * dt / 2.0f;
		}
	}
	else if (index == 1)
	{
		for (int j = 0; j < aa.size(); j++)
		{
			cc[j][0] = aa[j][0] + bb[j][0] * dt;
		}
	}
	return cc;
}

// Multiply Matrices together
inline Matrix matrix_mult(const Matrix& aa, const Matrix& bb)
{
	Matrix cc(bb.size(), std::vector<float>(1, 0.0f)); // Preallocate a vector of n rows (size) of 1 column with a default value of 0.0 (3:1)
	for (int i = 0; i < aa.size(); ++i)
	{
		float sum = 0.0f;
		for (int j = 0; j < aa.size(); ++j)
		{
			sum += aa[i][j] * bb[j][0];
		}
		cc[i][0] = sum;
	}
	return cc;
}

// Calculates the various derivative components of flight 
inline Matrix airplane_dynamics(Matrix x, const Plane_Mechanics& airplane)
{
	// Avoid the possibility of dividing by zero
	float norm_e = sqrt(pow(x[6][0], 2) + pow(x[7][0], 2) + pow(x[8][0], 2) + pow(x[9][0], 2));
	if (norm_e < 0.0001)
	{
		x[6][0] = 1.0;
		x[7][0] = 0.0;
		x[8][0] = 0.0;
		x[9][0] = 0.0;
	}
	else
	{
		x[6][0] = (x[6][0] / norm_e);
		x[7][0] = (x[7][0] / norm_e);
		x[8][0] = (x[8][0] / norm_e);
		x[9][0] = (x[9][0] / norm_e);
	}

	// Calculate the Euler angles (rad)
	Euler eul1 = quaternion2Euler(Quaternion(x[6][0], x[7][0], x[8][0], x[9][0]));

	//Caclulate the State Derivatives (dx)
	//Establish pDots matrices!
	Matrix pDots_0 =
	{
		{ cos(eul1.theta) * cos(eul1.psi), sin(eul1.phi) * sin(eul1.theta) * cos(eul1.psi) - cos(eul1.phi) * sin(eul1.psi), cos(eul1.phi) * sin(eul1.theta) * cos(eul1.psi) + sin(eul1.phi) * sin(eul1.psi) },
		{ cos(eul1.theta) * sin(eul1.psi), sin(eul1.phi) * sin(eul1.theta) * sin(eul1.psi) + cos(eul1.phi) * cos(eul1.psi), cos(eul1.phi) * sin(eul1.theta) * sin(eul1.psi) - sin(eul1.phi) * cos(eul1.psi) },
		{ -sin(eul1.theta), sin(eul1.phi) * cos(eul1.theta), cos(eul1.phi) * cos(eul1.theta) }
	};
	Matrix pDots_1 =
	{
		{ x[3][0] },
		{ x[4][0] },
		{ x[5][0] }
	};
	Matrix pDots = matrix_mult(pDots_0, pDots_1);

	// Establish uvwDots matrices!
	Matrix uvwDots =
	{
		{ x[12][0] * x[4][0] - x[11][0] * x[5][0] + airplane.comb_forces.fx / mass },
		{ x[10][0] * x[5][0] - x[12][0] * x[3][0] + airplane.comb_forces.fy / mass },
		{ x[11][0] * x[3][0] - x[10][0] * x[4][0] + airplane.comb_forces.fz / mass }
	};

	// Matrix multiplication to calculate e0dot, e1dot, e2dot, and e3dot.
	Matrix eDots_0 =
	{
		{ 0, -x[10][0] / 2, -x[11][0] / 2, -x[12][0] / 2 },
		{ x[10][0] / 2, 0, x[12][0] / 2, -x[11][0] / 2 },
		{ x[11][0] / 2, -x[12][0] / 2, 0, x[10][0] / 2 },
		{ x[12][0] / 2, x[11][0] / 2, -x[10][0] / 2, 0 }
	};
	Matrix eDots_1 = { { x[6][0] }, { x[7][0] }, { x[8][0] }, { x[9][0] } };
	Matrix eDots = matrix_mult(eDots_0, eDots_1);

	// matrix addition to calculate pDot, qDot, and rDot
	Matrix pqrDots =
	{
		{ Gamma1 * x[10][0] * x[11][0] - Gamma2 * x[11][0] * x[12][0] + Gamma3 * airplane.comb_forces.Tx + Gamma4 * airplane.comb_forces.Tz },
		{ Gamma5 * x[10][0] * x[12][0] - Gamma6 * (pow(x[10][0], 2) - pow(x[12][0], 2)) + airplane.comb_forces.Ty / Jy },
		{ Gamma7 * x[10][0] * x[11][0] - Gamma1 * x[11][0] * x[12][0] + Gamma4 * airplane.comb_forces.Tx + Gamma8 * airplane.comb_forces.Tz }
	};

	//	Place all of the state derivatives back into one big matrix for outputting
	Matrix dx = 
	{ 
		{ pDots[0][0] },  
		{ pDots[1][0] }, 
		{ pDots[2][0] }, 
		{ uvwDots[0][0] }, 
		{ uvwDots[1][0] },
		{ uvwDots[2][0] }, 
		{ eDots[0][0] },
		{ eDots[1][0] }, 
		{ eDots[2][0] }, 
		{ eDots[3][0] },
		{ pqrDots[0][0] }, 
		{ pqrDots[1][0] }, 
		{ pqrDots[2][0] } 
	};

	// Return state derivative matrix
	return dx;
}