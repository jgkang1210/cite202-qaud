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
	angle = (encoder * (double)((360. / 16383.)))* (M_PI / 180.0);
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
		angle = (angle_moving_average[0] + angle_moving_average[1]) / 2.;
	else if (index == 1)
		angle = (angle_moving_average[1] + angle_moving_average[2]) / 2.;
	else if (index == 2)
		angle = (angle_moving_average[2] + angle_moving_average[0]) / 2.;

	w_moving_average[0] = w_moving_average[1];
	w_moving_average[1] = w_moving_average[2];
	w_moving_average[2] = w;

	double wsub01 = abs((w_moving_average[0]) - abs(w_moving_average[1]));
	double wsub12 = abs((w_moving_average[1]) - abs(w_moving_average[2]));
	double wsub02 = abs((w_moving_average[0]) - abs(w_moving_average[2]));
	vector<double> warray = { wsub01, wsub12, wsub02 };
	int windex = min_element(warray.begin(), warray.end()) - warray.begin();
	if (windex == 0)
		w = (w_moving_average[0] + w_moving_average[1]) / 2.;
	else if (windex == 1)
		w = (w_moving_average[1] + w_moving_average[2]) / 2.;
	else if (windex == 2)
		w = (w_moving_average[2] + w_moving_average[0]) / 2.;

}