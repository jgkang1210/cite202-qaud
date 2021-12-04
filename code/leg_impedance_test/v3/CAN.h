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
#include "motor.h"
#include <thread>
using std::thread;
using namespace std;
using std::cout;
using std::cin;
using std::endl;
using std::this_thread::sleep_for;
class CAN
{
private:
	int len;
	CAN_HANDLE h;
public:
	vector<motor> motorback;
	double getomega(double& before_Angle, double& Angle, double delta_t);
	CAN();
	CAN(vector<motor> m);
	//vector<double> readData();
	//void readData();
	CAN_HANDLE open();
	void hardware_control(int count, double& d_t);
	void close();
};

