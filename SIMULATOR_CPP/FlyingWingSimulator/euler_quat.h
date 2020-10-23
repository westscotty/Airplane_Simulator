
/**********************************************************
 * File: euler_quat.h
 * Purpose: Calculate angles between euler and quaternions
 * Author(s): Weston Scott
 * Date Created: 9/27/2020
 * Date Modifed: 
 *********************************************************/

#pragma once
#include <iostream>
#include <iomanip>
#include <cmath>
#include <math.h>
#include <minmax.h>

struct Euler 
{
    // Euler Angles struct
    float phi;
    float theta;
    float psi;
    Euler(float phi_ = 0.0f, float theta_ = 0.0f, float psi_ = 0.0f) :
        phi(phi_), theta(theta_), psi(psi_) {};
};

struct Quaternion
{
    // Quaternion Angles Struct
    float e0;
    float e1;
    float e2;
    float e3;
    Quaternion(float e0_ = 1.0f, float e1_ = 0.0f, float e2_ = 0.0f, float e3_ = 0.0f) :
        e0(e0_), e1(e1_), e2(e2_), e3(e3_) {};
};

/************************************************************
* This function converts Quaternions to Euler angles using
* the equations given in Doctors Beard& McLain's pdf summary
* of their text book on airplane dynamics and the like.
*************************************************************/

static Euler quaternion2Euler(const Quaternion& q)
{
    //double theta_fix = 2.0f * (q.e0 * q.e2 - q.e1 * q.e3);
    //theta_fix = max(-1.0f, min(theta_fix, 1.0f));
    // Quaternion Angles conversion to Euler Angles
    return Euler((atan2(2.0f * (q.e0 * q.e1 + q.e2 * q.e3), (pow(q.e0, 2) + pow(q.e3, 2) - pow(q.e1, 2) - pow(q.e2, 2)))),
                 (asin(2.0f * (q.e0 * q.e2 - q.e1 * q.e3))),
                 (atan2(2.0f * (q.e0 * q.e3 + q.e1 * q.e2), (pow(q.e0, 2) + pow(q.e1, 2) - pow(q.e2, 2) - pow(q.e3, 2)))));
}

static Quaternion euler2Quaternion(const Euler& e)
{
    // Euler Angles conversion to Quaternion Angles
    return Quaternion((cos(e.psi / 2) * cos(e.theta / 2) * cos(e.phi / 2) + sin(e.psi / 2) * sin(e.theta / 2) * sin(e.phi / 2)),
                      (cos(e.psi / 2) * cos(e.theta / 2) * sin(e.phi / 2) - sin(e.psi / 2) * sin(e.theta / 2) * cos(e.phi / 2)),
                      (cos(e.psi / 2) * sin(e.theta / 2) * cos(e.phi / 2) + sin(e.psi / 2) * cos(e.theta / 2) * sin(e.phi / 2)),
                      (sin(e.psi / 2) * cos(e.theta / 2) * cos(e.phi / 2) - cos(e.psi / 2) * sin(e.theta / 2) * sin(e.phi / 2)));
}