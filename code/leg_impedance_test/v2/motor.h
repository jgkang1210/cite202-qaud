#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;
class motor
{
private:
	
	int encoder_origin;
public:
	long id;
	int len = 8;
	int readID = 0;
	double original_encoder = 0;
	double encoder = 0;
	double dps = 0;
	double rpm = 0;
	double angle = 0;
	double before_angle = 0;
	double w = 0;
	double angle_moving_average[3] = {0,0,0};
	double w_moving_average[3] = {0,0,0};
	int32_t torque = 0; //not real torque, -2000~2000 data
	unsigned char sdata[8] = { 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	motor();
	motor(long id, int readid, int encoder_offset);
	void torqueSave();
	void encoder_calibration();
	void encoder2angle();
	void dps2rpm();
	void dps2omega();
	void moving_average();
};

