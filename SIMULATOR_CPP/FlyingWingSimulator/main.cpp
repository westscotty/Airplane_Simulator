
/**********************************************************
 * File: main.cpp
 * Purpose: Creates simulator object, and runs tick function
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
#include "simulator.h"

int main()
{
	Simulator simulator;
	simulator.run_simulator();
	return 0;
}