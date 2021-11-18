#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include "CAN_Access.h"
#include <cstdio>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
#include <numbers>

using namespace std;
using std::cout;
using std::cin;
using std::endl;
using std::this_thread::sleep_for;
class CAN
{
private:
	long id1;
	long id2;
	int len;
	CAN_HANDLE h;
	double encoder1;
	double rpm1;
	double encoder2;
	double rpm2;
public:
	unsigned char sdata1[8] = { 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	unsigned char sdata2[8] = { 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	
	double getomega(double& before_Angle, double& Angle, double delta_t);
	void encoder2angle(double encod1, double& ang1, double encod2, double& ang2);
	void rpm2omega(double rp1, double& omega1, double rp2, double& omega2);
	CAN();
	vector<double> readData();
	void open();
	void hardware_control(int32_t& torque1, int32_t& torque2, double& angle1, double& before_angle1, double& w1, double& angle2, double& before_angle2, double& w2, int count, double& d_t);
	void close();
};

