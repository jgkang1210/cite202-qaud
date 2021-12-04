#include "CAN.h"
#include "controller.h"
#include "motor.h"
#include <iostream>
#include <thread>
#include <cstdio>

#define GRAVITY //모터 중력보정 게인조정시
using std::thread;

motor m1(0x142, 66, 1511);
motor m2(0x141, 65, 3331);
vector<motor> motorback = {m1, m2};
double dt = 0;
//vector<vector<double>> resultblock;
//vector<double> re1 = {0,0,0,0};
//vector<double> re2 = {0,0,0,0};
enum {ANGLE, ENCODER, ORIGINAL_ENCODER, OMEGA, TORQUE};
void printdata(const vector<motor> motorback, int show)
{
	if (show == ANGLE)
	{
		for(int i = 0; i < motorback.size(); i++)
			printf("angle%d : %4.4lf, ", i, motorback[i].angle);
		printf("\n");
	}
	else if (show == ENCODER)
	{
		for (int i = 0; i < motorback.size(); i++)
			printf("encoder%d : %4.4lf, ", i, motorback[i].encoder);
		printf("\n");
	}
	else if (show == ORIGINAL_ENCODER)
	{
		for (int i = 0; i < motorback.size(); i++)
			printf("original_encoder%d : %4.4lf, ", i, motorback[i].original_encoder);
		printf("\n");
	}
	else if (show == OMEGA)
	{
		for (int i = 0; i < motorback.size(); i++)
			printf("w%d : %4.4lf, ", i, motorback[i].w);
		printf("\n");
	}
	else if (show == TORQUE)
	{
		for (int i = 0; i < motorback.size(); i++)
			printf("torque%d : %5d, ", i, motorback[i].torque);
		printf("\n");
	}
}
void printdata(const motor motorback, int show)
{
	if (show == ANGLE)
	{
		printf("  angle : %4.4lf, ", motorback.angle);
		printf("\n");
	}
	else if (show == ENCODER)
	{
		printf("  encoder : %4.4lf, ", motorback.encoder);
		printf("\n");
	}
	else if (show == ORIGINAL_ENCODER)
	{
		printf("  original_encoder : %4.4lf, ", motorback.original_encoder);
		printf("\n");
	}
	else if (show == OMEGA)
	{
		printf("  w : %4.4lf, ", motorback.w);
		printf("\n");
	}
	else if (show == TORQUE)
	{
		printf("  torque% : %5d, ", motorback.torque);
		printf("\n");
	}
}
double torque2input(double tq)
{
	return tq * (4 / 3) * (2000 / 3.2)*0.1;
}
void readData(CAN_HANDLE h)
{
	while (true)
	{
		int rx_count = CAN_CountRxQueue(h);
		if (rx_count > 0)
		{
			char rdata[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			long rid;
			int rlen, ext, rtr;
			int ret = CAN_Recv(h, &rid, &rlen, rdata, &ext, &rtr);
			if (ret)
			{
				if ((uint8_t)(unsigned char)rdata[0] == 0xA1)
				{
					double encoder;
					double rpm;
					unsigned char buff[2];
					int16_t prerpm;
					uint16_t preencoder;
					
					buff[0] = rdata[4];
					buff[1] = rdata[5];

					memcpy(&prerpm, buff, sizeof(int16_t));
					
					rpm = (double)prerpm;

					buff[0] = rdata[6];
					buff[1] = rdata[7];

					memcpy(&preencoder, buff, sizeof(int16_t));

					encoder = (double)(preencoder);

					for (int i = 0; i < motorback.size(); i++)
					{
						if ((int)(unsigned char)(double)rid == motorback[i].readID)
						{
							motorback[i].result = { 1, (double)rid, rpm, encoder };
							break;
						}
					}
					/*if ((int)(unsigned char)(double)rid == m1.readID)
						motorback[0].result = { 1, (double)rid, rpm, encoder };
					else if ((unsigned char)(double)rid == m2.readID)
						motorback[1].result = { 1, (double)rid, rpm, encoder };*/
				}
			}
			else
			{
				for (int i = 0; i < motorback.size(); i++)
				{
					if ((int)(unsigned char)(double)rid == motorback[i].readID)
					{
						motorback[i].result = { -1, 0, 0, 0 };
						break;
					}
				}
			}
		}
	}
}

int main()
{
	Vector2d tau(0, 0);
	Controller controller;
	CAN can;
	CAN_HANDLE h = can.open();
	// can.motorback = { m1, m2 };
	int count = 1;
	double d_t = 0;
	thread t1(readData,h);
	while (true) //loop for test
	{	
		/*can.motorback[0].result = re1;
		can.motorback[1].result = re2;*/
		can.motorback = motorback;
		count++;
		can.hardware_control(count, d_t);
		printdata(can.motorback, ORIGINAL_ENCODER); //각도출력
		if (count > 300)
			break;
	}


	int choosenum = 0;
	cout << endl << endl << endl;
	cout << "choose mode : \n1. one-motor Gravity Compensation test \n2. one-leg ompedance control\n : ";
	cin >> choosenum;


	if (choosenum == 1)
	{
		double gravity_compensation = 0;
		double control_input = 0;
		double kp = 2.652;// 2.7;
		double kd = 1.145;// 1.1;
		double ki = 0.0005;
		double desired_angle = M_PI / 5.;//M_PI/2.;
		double desired_omega = 0;
		double cur_angle = 0;
		double cur_omega = 0;
		double p_term = 0;
		double d_term = 0;
		double error = 0;
		double i_term = 0;

		desired_angle = (double)((int)(desired_angle * 1000)) / 1000.;


		//double dt = 0;
		while (true) // start
		{
			std::chrono::system_clock::time_point StartTime = std::chrono::system_clock::now();
			/*can.motorback[0].result = re1;
			can.motorback[1].result = re2;*/
			//can.motorback = motorback;
			can.motorback[0].result = motorback[0].result;
			can.motorback[1].result = motorback[1].result;
			count++;
			can.hardware_control(count, d_t);
			//torque limit
			double limit_torque = 10;
			if (limit_torque < tau.coeff(0, 0))
				tau(0, 0) = limit_torque;
			else if (-limit_torque > tau.coeff(0, 0))
				tau(0, 0) = -limit_torque;
			if (limit_torque < tau.coeff(1, 0))
				tau(1, 0) = limit_torque;
			else if (-limit_torque > tau.coeff(1, 0))
				tau(1, 0) = -limit_torque;

			cur_angle = can.motorback[1].angle;
			cur_angle = (double)((int)(cur_angle * 1000)) / 1000.;

			cout << cur_angle << endl;

			can.motorback[0].w = (can.motorback[0].angle - can.motorback[0].before_angle) / dt;
			//can.motorback[1].w = (can.motorback[1].angle - can.motorback[1].before_angle) / dt;
			can.motorback[0].before_angle = can.motorback[0].angle;
			can.motorback[1].before_angle = can.motorback[1].angle;

			
			cur_omega = can.motorback[1].w;
			cur_omega = (double)((int)(cur_omega * 1000)) / 1000.;


			gravity_compensation = 2.048700192 * sin(cur_angle) * 0.47;
			/*if (abs(desired_angle - cur_angle) < M_PI / 18.)
			{
				kp = 10;
				kd = 1;
			}
			else if (abs(desired_angle - cur_angle) >= M_PI / 18.)
			{
				kp = 400;
				kd = 40;
			}*/
			p_term = kp * (desired_angle - cur_angle);
			
			d_term = kd * (desired_omega - cur_omega);
			error = error + (desired_angle - cur_angle);
			//cout << "p: " << p_term << ",      d: " << d_term << endl;
			i_term = ki * error;

			double limit_iterm = 0.067;
			if (limit_iterm < i_term)
				i_term = limit_iterm;
			else if (-limit_iterm > i_term)
				i_term = -limit_iterm;
			/*if (limit_iterm < tau.coeff(1, 0))
				tau(1, 0) = limit_iterm;
			else if (-limit_iterm > tau.coeff(1, 0))
				tau(1, 0) = -limit_iterm;*/

			control_input = gravity_compensation + p_term + d_term + i_term;// + d_term;

			can.motorback[0].torque = 0;
			can.motorback[1].torque = torque2input(control_input);// torque2input(control_input);

			//printdata(can.motorback[1], TORQUE);
			//printdata(can.motorback[1], OMEGA);
			std::chrono::system_clock::time_point EndTime = std::chrono::system_clock::now();
			std::chrono::microseconds micro = std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime);
			dt = (double)micro.count() * (1.0 / 1000000.0);
		}
	}
	else if (choosenum == 2)
	{
		while (true) // start
		{
			std::chrono::system_clock::time_point StartTime = std::chrono::system_clock::now();
			/*can.motorback[0].result = re1;
			can.motorback[1].result = re2;*/
			//can.motorback = motorback;
			can.motorback[0].result = motorback[0].result;
			can.motorback[1].result = motorback[1].result;
			count++;
			can.hardware_control(count, d_t);
			for (int i = 0; i < can.motorback.size(); i++)
			{
				//can.motorback[i].w = (can.motorback[i].angle - can.motorback[i].before_angle) / dt;
				can.motorback[i].w = (double)((int)(can.motorback[i].w * 1000)) / 1000.;
				can.motorback[i].angle = (double)((int)(can.motorback[i].angle * 1000)) / 1000.;
				//can.motorback[i].before_angle = can.motorback[i].angle;
			}

			tau = controller.computedtorquemethod(can.motorback[0].angle, can.motorback[1].angle, can.motorback[0].w, can.motorback[1].w);
			//cout << "angle1: " << can.motorback[0].encoder << ", angle2: " << can.motorback[1].encoder << endl;
			//torque limit
			double limit_torque = 10;
			if (limit_torque < tau.coeff(0, 0))
				tau(0, 0) = limit_torque;
			else if (-limit_torque > tau.coeff(0, 0))
				tau(0, 0) = -limit_torque;
			if (limit_torque < tau.coeff(1, 0))
				tau(1, 0) = limit_torque;
			else if (-limit_torque > tau.coeff(1, 0))
				tau(1, 0) = -limit_torque;

			can.motorback[0].torque = torque2input(tau(0, 0)); // torque2input(tau(0, 0));
			can.motorback[1].torque = torque2input(tau(1, 0)); // torque2input(tau(1, 0));

			// printdata(can.motorback, TORQUE);
			
			std::chrono::system_clock::time_point EndTime = std::chrono::system_clock::now();
			std::chrono::microseconds micro = std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime);
			dt = (double)micro.count() * (1.0 / 1000000.0);
		}
	}
	t1.detach();
	return 0;
}



































//double torque2input(double tq)
//{
//	return tq * (4 / 3) * (2000 / 3.2);
//}
//vector<double> readData(CAN_HANDLE h)
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
//			rpm = (double)((uint8_t)(int)(unsigned char)rdata[4] + ((uint8_t)(int)(unsigned char)rdata[5]) * 256);
//			encoder = (double)((uint8_t)(int)(unsigned char)rdata[6] + ((uint8_t)(int)(unsigned char)rdata[7]) * 256);
//			int encod1 = encoder - 411;//411
//			if (encod1 < 0)
//				encod1 += 16383;
//			double angle = (encod1 * (double)((360. / 16383.)));
//			vector<double> result = { 1, (double)rid, rpm, angle };
//			return result;
//		}
//	}
//	else
//	return { -1, 0, 0, 0 };
//}
//int main()
//{
//	long id1 = 0x142;
//	long id2 = 0x141;
//	int len = 8;
//	unsigned char sdata[8] = { 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//	int32_t torque = -torque2input(0.01);
//	sdata[4] = torque & 0xFF;
//	sdata[5] = (torque >> 8) & 0xFF;
//	CAN_HANDLE h = CAN_OpenUsb("NT5YYJCT");
//	CAN_SetTransferMode(h, 1);
//
//	sleep_for(std::chrono::milliseconds(1000));
//	cout << "start" << endl;
//	while (true)
//	{
//		vector<double> readdata(4);
//		CAN_Send(h, id1, len, (char*)sdata, 0, 0);
//		int rx_count = CAN_CountRxQueue(h);
//		if (rx_count > 0)
//		{
//			readdata = readData(h);
//		}
//		if (readdata[0] == 1) // for torque receive
//		{
//			printf("id: %d", (int)(unsigned char)readdata[1]);
//			cout << ", encoder:" << readdata[3] << ", rpm: " << readdata[2] << endl;
//		}
//		sleep_for(std::chrono::milliseconds(1));
//	}
//	CAN_Close(h);
//	return 0;
//
//
//}
//int main()
//{
//	long id1 = 0x142;
//	long id2 = 0x141;
//	int len = 8;
//	unsigned char sdata1[8] = { 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//	unsigned char sdata2[8] = { 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//	int32_t torque1 = 0;
//	int32_t torque2 = 1500;
//	sdata1[4] = torque1 & 0xFF;
//	sdata1[5] = (torque1 >> 8) & 0xFF;
//	sdata2[4] = torque2 & 0xFF;
//	sdata2[5] = (torque2 >> 8) & 0xFF;
//	CAN_HANDLE h = CAN_OpenUsb("NT5YYJCT");
//	CAN_SetTransferMode(h, 1);
//
//	sleep_for(std::chrono::milliseconds(1000));
//	cout << "start" << endl;
//	thread t1(readData, h);
//	while (true)
//	{
//		CAN_Send(h, id1, len, (char*)sdata1, 0, 0);
//		CAN_Send(h, id2, len, (char*)sdata2, 0, 0);
//		int rx_count = CAN_CountRxQueue(h);
//		vector<vector<double>> motorback = {re1,re2};
//		for (int i = 0; i < 2; i++)
//		{
//			if (motorback[i][0] == 1)
//			{
//				cout << "id: "<< motorback[i][1] << ", encoder:" << motorback[i][3] << ", rpm: " << motorback[i][2] << endl;
//			}
//		}
//		sleep_for(std::chrono::milliseconds(1));
//	}
//	t1.detach();
//	CAN_Close(h);
//
//	return 0;
//
//
//}



