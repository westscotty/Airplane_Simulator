
/**********************************************************
 * File: simulator.h
 * Purpose: Simulator object object definition
 * Author(s): Weston Scott
 * Date Created: 10/25/2020
 * Date Modifed: 
 *********************************************************/
#pragma once

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

//#define MAX_DATA_LENGTH 255
//char incomingData[MAX_DATA_LENGTH];
//const char* portName = "\\\\.\\COM6";

struct Simulator
{
	// Initialize simulator objects
	Plane_Mechanics airplane;
	Matrix x = 
		{
			{ airplane.plane.pn },
			{ airplane.plane.pe },
			{ airplane.plane.pd },
			{ airplane.plane.u },
			{ airplane.plane.v },
			{ airplane.plane.w },
			{ airplane.plane.quat.e0 },
			{ airplane.plane.quat.e1 },
			{ airplane.plane.quat.e2 },
			{ airplane.plane.quat.e3 },
			{ airplane.plane.p },
			{ airplane.plane.q },
			{ airplane.plane.r }
		};
	float dt = 0.005f;
	SerialPort* arduino;
	//const char*path = "C:\\Users\\Wes\\Documents\\wsl_share\\Airplane_Simulator\\SIMULATOR_CPP\\FlyingWingSimulator\\file_write.txt";
	int once = 0;
	float j_x = 0.0f;
	float j_y = 0.0f;
	float pot = 0.0f;

	// Simulator methods
	void ReceiveData(float& j_x, float& j_y, float& pot, int& once);
	void autoConnect();
	void run_simulator();
	void rk4_method();
};