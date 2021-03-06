
/**********************************************************
 * File: simulator.cpp
 * Purpose: Initilize airplane.plane objects
 * Functions: 1) main.cpp --> driver function
 * Author(s): Weston Scott
 * Date Created: 9/25/2020
 * Date Modifed: 10/13/2020
 *********************************************************/
#pragma once

// Import necessary libraries and files
#include <iostream>
#include <iomanip>
#include <cmath>
#include <math.h>
#include "global_consts.h"
#include "plane_dynamics.cpp"
#include "euler_quat.h"
#include "plane_mechanics.h"
#include "serialport.h"
#include <sstream>
#include <string>
#include <windows.h>
#include <fstream>
#include "simulator.h"

using namespace std;

#define MAX_DATA_LENGTH 255
char incomingData[MAX_DATA_LENGTH];
const char* portName = "\\\\.\\COM5";

//Gets data serially,unpacks it, and updates it based on how much time has passed
void Simulator::ReceiveData(float& j_x, float& j_y, float& pot, int& once)
{
	int readResult = arduino->readSerialPort(incomingData, MAX_DATA_LENGTH);

	//Initialize variables and constants
	std::string str = incomingData;
	static LARGE_INTEGER timeFreq, timeNewj_x, timeOldj_x, timeNewj_y, timeOldj_y, timeNewpot, timeOldpot;
	float tau = 0.1f / 4.0f;
	int err = -9999;

	// Find intial time  -- This should only run once
	if (once == 0) {
		QueryPerformanceCounter(&timeOldj_x);
		QueryPerformanceCounter(&timeOldj_y);
		QueryPerformanceCounter(&timeOldpot);
		// Find frequency in order to time change in time
		QueryPerformanceFrequency(&timeFreq);
		once = 1;
	}

	//Separates one set of data from the entire string
	int i = 0, j = 0;
	for (i = 0; i < str.length(); i++) {
		if (str[i] == '<') {
			for (j = 0; j < str.length() - i; j++) {
				if (str[i + j] == '>') {

					break;
				}
			}
			break;
		}
	}
	str = str.substr(i + 1, j - 1);

	//Seperates data into a vector
	std::vector<float> vect1;
	int k = 0;
	j = 0;
	for (i = 0; i <= str.length(); i++) {
		if (str[i] == ',') {
			vect1.push_back(stof(str.substr(j, k)));
			j = i + 1;
			k = -1;
		}
		k = k + 1;
	}

	//Sets any missing data to -9999 to show that there is an error
	vect1.push_back(err);
	vect1.push_back(err);
	vect1.push_back(err);

	//Updates each position if there was not an error and updates time
	if (vect1[2] != err) {
		QueryPerformanceCounter(&timeNewpot);
		float dtx2 = (float)((timeNewpot.QuadPart - timeOldpot.QuadPart) / (double)timeFreq.QuadPart);
		timeOldpot = timeNewpot;
		pot = pot * exp(-dtx2 / tau) + vect1[2] * (1 - exp(-dtx2 / tau));
	}
	if (vect1[1] != err) {
		QueryPerformanceCounter(&timeNewj_y);
		float dty1 = (float)((timeNewj_y.QuadPart - timeOldj_y.QuadPart) / (double)timeFreq.QuadPart);
		timeOldj_y = timeNewj_y;
		j_y = j_y * exp(-dty1 / tau) + vect1[1] * (1 - exp(-dty1 / tau));
	}
	if (vect1[0] != err) {
		QueryPerformanceCounter(&timeNewj_x);
		float dtx1 = (float)((timeNewj_x.QuadPart - timeOldj_x.QuadPart) / (float)timeFreq.QuadPart);
		timeOldj_x = timeNewj_x;
		j_x = j_x * exp(-dtx1 / tau) + vect1[0] * (1 - exp(-dtx1 / tau));
	}

	Sleep(50);  //Must sleep for about 30ms or program will crash
}

//Connects C++ code to arduino
void Simulator::autoConnect()
{
	//works better than recusion
	//avoid stack overflows
	while (1) {
		// ui - searching
		std::cout << "Searching in progress";
		// wait connection
		while (!arduino->isConnected()) {
			Sleep(100);
			std::cout << ".";
			arduino = new SerialPort(portName);
		}

		//Checking if arduino is connected or not
		if (arduino->isConnected()) {
			std::cout << std::endl << "Connection established at port " << portName << std::endl;
			return;
		}
	}
}

// rk4 method
void Simulator::rk4_method()
{
	// Read Values from Arduino Contoller
	ReceiveData(j_x, j_y, pot, once);

	// Set values from controller
	airplane.control_vals.el_left = 0.0; // j_y;
	airplane.control_vals.el_right = 0.0; // j_x;
	airplane.control_vals.del_t = 0.5; // pot;
	airplane.convert_elevons();

	// Calculate Forces and Torques
	airplane.update_comb_forces();

	// Runge Kutta 4th Order Method --------------
	Matrix k1 = airplane_dynamics(x, airplane);
	Matrix k1x = matrix_add(x, k1, dt, 0);
	Matrix k2 = airplane_dynamics(k1x, airplane);
	Matrix k2x = matrix_add(x, k2, dt, 0);
	Matrix k3 = airplane_dynamics(k2x, airplane);
	Matrix k3x = matrix_add(x, k3, dt, 1);
	Matrix k4 = airplane_dynamics(k3x, airplane);

	// Update x state
	for (int m = 0; m < 13; m++)
	{
		x[m][0] += (k1[m][0] + 2 * k2[m][0] + 2 * k3[m][0] + k4[m][0]) * dt / 6.0f;
	}

	// Update new values for plane state ---------
	// Update position
	airplane.plane.pn = x[0][0]; airplane.plane.pe = x[1][0]; airplane.plane.pd = x[2][0];

	// Update directonal velocities
	airplane.plane.u = x[3][0]; airplane.plane.v = x[4][0]; airplane.plane.w = x[5][0];

	// Update quaternion angles
	airplane.plane.quat.e0 = x[6][0]; airplane.plane.quat.e1 = x[7][0];
	airplane.plane.quat.e2 = x[8][0]; airplane.plane.quat.e3 = x[9][0];

	// Update roll, pitch, and yaw angle rates
	airplane.plane.p = x[10][0]; airplane.plane.q = x[11][0]; airplane.plane.r = x[12][0];

	// Update Euler Angles
	airplane.plane.eul = quaternion2Euler(Quaternion(x[6][0], x[7][0], x[8][0], x[9][0]));

	// Update Velocity, Alpha, and Beta
	airplane.update_va_alpha_beta();

	// Display data to screen
	//for (int n = 0; n < 13; n++)
	//{
	//	std::cout << x[n][0] << ", ";
	//}
	std::cout << airplane.plane.eul.phi * 180 / M_PI << ", " << airplane.plane.eul.psi * 180 / M_PI << ", " << airplane.plane.eul.theta * 180 / M_PI << "\n";


	//std::cout << "\n";
	//std::cout << pot << ", " << j_x << ", " << j_y << "\n";
}

// Driver function for the entire simulation
void Simulator::run_simulator()
{
	// Set up serial connnection to arduino controller !!!!!
	// Connect C++ to arduino
	arduino = new SerialPort(portName);
	autoConnect();

	// Initialize text file for storing data and open file
	//ofstream myFile;
	//myFile.open(path);
	//myFile << "pn, pe, pd, u, v, w, throttle, el_Left, el_Right \n";
	
	// Initialize Quaternion Angles, Alpha, Beta, Velocity
	airplane.plane.quat = euler2Quaternion(airplane.plane.eul);
	airplane.update_va_alpha_beta();

	// Declare state variable x
	//x = 
	//{ 
	//	{ airplane.plane.pn }, 
	//	{ airplane.plane.pe }, 
	//	{ airplane.plane.pd },
	//	{ airplane.plane.u },
	//	{ airplane.plane.v },
	//	{ airplane.plane.w },
	//	{ airplane.plane.quat.e0 },
	//	{ airplane.plane.quat.e1 },
	//	{ airplane.plane.quat.e2 },
	//	{ airplane.plane.quat.e3 },
	//	{ airplane.plane.p },
	//	{ airplane.plane.q },
	//	{ airplane.plane.r }
	//};

	// Enter Simulation RK4 Loop
	while (arduino->isConnected()) {
		rk4_method();

		// Write txt file for saved data
		//for (int n = 0; n < 6; n++)
		//{
			//myFile << x[n][0] << ", ";
		//}
		//myFile << pot << ", " << j_x << ", " << j_y << "\n";
	}

	// Close txt file
	//myFile.close();
}

//pawn_constructor {
//	arduino = new SerialPort(portName);
//	autoConnect();
//	airplane.plane.quat = euler2Quaternion(airplane.plane.eul);
//	airplane.update_va_alpha_beta();
//}
