#include "CAN.h"
CAN::CAN()
{
	len = 8;
}
CAN::CAN(vector<motor> m)
{
	motorback = m;
	len = 8;
}
CAN_HANDLE CAN::open()
{
	h = CAN_OpenUsb("NT5YYJCT");
	CAN_SetTransferMode(h, 1);
	sleep_for(std::chrono::milliseconds(1000));
	return h;
}
//void CAN::readData()
//{
//	int rx_count = CAN_CountRxQueue(h);
//	if (rx_count > 0)
//	{
//		char rdata[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
//		long rid;
//		int rlen, ext, rtr;
//		int ret = CAN_Recv(h, &rid, &rlen, rdata, &ext, &rtr);
//		if (ret)
//		{
//			if ((uint8_t)(unsigned char)rdata[0] == 0xA1)
//			{
//				double encoder;
//				double rpm;
//				int16_t prerpm;
//				prerpm = (int16_t)((uint16_t)rdata[4] | ((uint16_t)rdata[5] << 8));
//				rpm = (double)prerpm;
//				encoder = (double)((uint8_t)(int)(unsigned char)rdata[6] + ((uint8_t)(int)(unsigned char)rdata[7]) * 256);
//				result = { 1, (double)rid, rpm, encoder };
//			}
//		}
//		else
//			result = { -1, 0, 0, 0 };
//	}
//	else
//		result = { -1, 0, 0, 0 };
//}
//vector<double> CAN::readData()
//{
//	char rdata[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
//	long rid;
//	int rlen, ext, rtr;
//	int ret = CAN_Recv(h, &rid, &rlen, rdata, &ext, &rtr);
//	if (ret)
//	{
//		if ((uint8_t)(unsigned char)rdata[0] == 0xA1)
//		{
//			double encoder;
//			double rpm;
//			int16_t prerpm;
//			prerpm = (int16_t)((uint16_t)rdata[4] | ((uint16_t)rdata[5] << 8));
//			rpm = (double)prerpm;
//			encoder = (double)((uint8_t)(int)(unsigned char)rdata[6] + ((uint8_t)(int)(unsigned char)rdata[7]) * 256);
//			vector<double> result = { 1, (double)rid, rpm, encoder };
//			return result;
//		}
//	}
//	else
//		return { -1, 0, 0, 0 };
//}
void CAN::hardware_control(int count, double &d_t)//TODO
{
	
	std::chrono::system_clock::time_point StartTime = std::chrono::system_clock::now();
	vector<double> readdata(4);
	for (int i = 0; i < motorback.size(); i++)
	{
		motorback[i].torqueSave();
		CAN_Send(h, motorback[i].id, len, (char*)motorback[i].sdata, 0, 0);
	}
	for (int i = 0; i < motorback.size(); i++)
	{
		if (motorback[i].result[0] == 1)
		{
			motorback[i].original_encoder = motorback[i].result[3];
			motorback[i].dps = motorback[i].result[2];
		}
	}
	for (int i = 0; i < motorback.size(); i++)
	{
		motorback[i].encoder_calibration();
		motorback[i].encoder2angle();
		motorback[i].dps2omega();
		//motorback[i].moving_average();
	}
	
	//w1 = getomega(before_angle1, angle1, d_t);
	//w2 = getomega(before_angle2, angle2, d_t);

	//sleep_for(std::chrono::microseconds(10));
	std::chrono::system_clock::time_point EndTime = std::chrono::system_clock::now();
	std::chrono::microseconds micro = std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime);
	d_t = (double)micro.count() * (1.0/1000000.0);
}
void CAN::close()
{
	CAN_Close(h);
}

double CAN::getomega(double& before_Angle, double& Angle, double delta_t)
{
	if (Angle - before_Angle >= M_PI)
		Angle = Angle - 2 * M_PI;
	else if (Angle - before_Angle <= -M_PI)
		before_Angle = before_Angle - 2 * M_PI;
	double Omega = (Angle - before_Angle) / delta_t;
	before_Angle = Angle;
	
	return Omega;
}





/*int rx_count = CAN_CountRxQueue(h);
if (rx_count > 0)
	readdata = readData();*/;
	//readdata = result; //TODO
	//if (readdata[0] == 1) // for torque receive
	//{
	//	for (int i = 0; i < motorback.size(); i++)
	//	{
	//		if ((int)(unsigned char)readdata[1] == motorback[i].readID)
	//		{
	//			motorback[i].original_encoder = readdata[3];
	//			motorback[i].dps = readdata[2];
	//		}
	//	}
	//}