#include "motor.h"
motor::motor()
{
	id = 0x141;
}
motor::motor(long id_, int readid, int motor_offset)
{
	id = id_;
	readID = readid;
	encoder_origin = motor_offset;
}
void motor::torqueSave()
{
	sdata[4] = torque & 0xFF;
	sdata[5] = (torque >> 8) & 0xFF;
}
void motor::encoder_calibration()
{
	encoder = original_encoder - encoder_origin;//411
	if (encoder < 0)
		encoder += 16383;
}
void motor::encoder2angle()
{
	double angledata = (encoder * (double)((360. / 16383.)))* (M_PI / 180.0);
	if (angledata > M_PI)
	{
		angle = -(2*M_PI-angledata);
	}
	else
	{
		angle = angledata;
	}
}
void motor::dps2rpm()
{
	rpm = dps * 0.16666666666667;
}
void motor::dps2omega()
{
	w = dps * 0.16666666666667 * (double)((2. * M_PI) / 60.);
}
void motor::moving_average()
{
	angle_moving_average[0] = angle_moving_average[1];
	angle_moving_average[1] = angle_moving_average[2];
	angle_moving_average[2] = angle;

	double sub01 = abs((angle_moving_average[0]) - abs(angle_moving_average[1]));
	double sub12 = abs((angle_moving_average[1]) - abs(angle_moving_average[2]));
	double sub02 = abs((angle_moving_average[0]) - abs(angle_moving_average[2]));
	vector<double> array = { sub01, sub12, sub02 };
	int index = min_element(array.begin(), array.end()) - array.begin();
	if (index == 0)
		angle_average = (angle_moving_average[0] + angle_moving_average[1]) / 2.;
	else if (index == 1)
		angle_average = (angle_moving_average[1] + angle_moving_average[2]) / 2.;
	else if (index == 2)
		angle_average = (angle_moving_average[2] + angle_moving_average[0]) / 2.;

	w_moving_average[0] = w_moving_average[1];
	w_moving_average[1] = w_moving_average[2];
	w_moving_average[2] = w_moving_average[3];
	w_moving_average[3] = w_moving_average[4];
	w_moving_average[4] = w_moving_average[5];
	w_moving_average[5] = w_moving_average[6];
	w_moving_average[6] = w_moving_average[7];
	w_moving_average[7] = w_moving_average[8];
	w_moving_average[8] = w_moving_average[9];
	w_moving_average[9] = w_moving_average[10];
	w_moving_average[10] = w_moving_average[11];
	w_moving_average[11] = w_moving_average[12];
	w_moving_average[12] = w_moving_average[13];
	w_moving_average[13] = w_moving_average[14];
	w_moving_average[14] = w;
	double add = 0;
	for (int i = 0; i < 15; i++)
	{
		add += w_moving_average[i];
	}
	w_average = add / 15.0;
}