#include "CAN.h"
#include "controller.h"
double torque2input(double tq)
{
	return tq * (4 / 3) * (2000 / 3.2)*0.1;
}
int main()
{
	int32_t torque1 = 0;
	int32_t torque2 = 0;
	double angle1, angle2;
	double before_angle1, before_angle2;
	double w1, w2;
	double d_t;
	controller Controller;
	CAN can;

	can.open();
	int count = 0;
	while (true)
	{
		count++;
		can.hardware_control(torque1, torque2, angle1, before_angle1, w1, angle2, before_angle2, w2, count, d_t);
		cout << "angle1: " << angle1 << ", w1: " << w1 << ", angle2: " << angle2 << ", w2: " << w2 <<", count:"<< count << endl;
		if (count > 300)
			break;
	}
	vec2 tau (0,0);
	while (true)
	{
		count++;
		can.hardware_control(torque1,torque2, angle1, before_angle1, w1, angle2, before_angle2, w2, count, d_t);
		cout << angle1 << "," << angle2 << endl;
		//cout << "torque1: " << torque1 << ", torque2: " << torque2 << endl;
		//cout << "torque1: " << tau[0] << ", torque2: " << tau[1] << ", ";
		//cout << "angle1: " << angle1 << ", w1: " << w1 << ", angle2: " << angle2 << ", w2: " << w2 << ", count:" << count << endl;
		tau = Controller.leg2spring(angle1, angle2, w1, w2);
		
		//torque มฆวั
		double limit_torque = 0.3;
		if (limit_torque < tau[0])
			tau[0] = limit_torque;
		else if (-limit_torque > tau[0])
			tau[0] = -limit_torque;
		if (limit_torque < tau[1])
			tau[1] = limit_torque;
		else if (-limit_torque > tau[1])
			tau[1] = -limit_torque;
		
		torque1 = torque2input(tau[0]);
		torque2 = torque2input(tau[1]);
	}
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



