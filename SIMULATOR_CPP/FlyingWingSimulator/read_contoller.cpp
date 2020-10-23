/**********************************************************
 * File: read_contoller.cpp
 * Purpose: header file for establishing serial port
 *          connection for Arduino Microcontroller
 * Author(s): Manash Kumar Mandal
 * Modified by: William Keller, Weston Scott
 * Date Modifed: 10/13/2020
 ********************************************************

#pragma once
#include <iostream>
#include "serialport.h"
#include <sstream>
#include <string>
#include <windows.h>
#include "global_consts.h"

using namespace std;

const char* portName = "\\\\.\\COM6";
#define MAX_DATA_LENGTH 255
char incomingData[MAX_DATA_LENGTH];

//Arduino SerialPort object
SerialPort* arduino;

//Gets data serially,unpacks it, and updates it based on how much time has passed
void ReceiveData(float& j_x, float& j_y, float& pot, int& once)
{
	int readResult = arduino->readSerialPort(incomingData, MAX_DATA_LENGTH);

	//Initialize variables and constants
	string str = incomingData;
	static LARGE_INTEGER timeFreq, timeNewj_x, timeOldj_x, timeNewj_y, timeOldj_y, timeNewpot, timeOldpot;
	float tau = 0.1 / 4.0;
	int err = -9999;

	// Find intial time  -- This should only run once
	if (once == 0) {
		QueryPerformanceCounter(&timeOldj_x);
		QueryPerformanceCounter(&timeOldj_y);
		QueryPerformanceCounter(&timeOldpot);
		//QueryPerformanceCounter(&timeOldy2);
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
	vector<float> vect1;
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
void autoConnect()
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
}*/