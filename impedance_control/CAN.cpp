#include "CAN.h"
CAN::CAN()
{
	id1 = 0x142;
	id2 = 0x141;
	len = 8;
}
void CAN::open()
{
	h = CAN_OpenUsb("NT5YYJCT");
	CAN_SetTransferMode(h, 1);
	sleep_for(std::chrono::milliseconds(1000));
}
vector<double> CAN::readData()
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
			int16_t prerpm;
			prerpm = (int16_t)((uint16_t)rdata[4] | ((uint16_t)rdata[5] << 8));
			rpm = (double)prerpm;
			encoder = (double)((uint8_t)(int)(unsigned char)rdata[6] + ((uint8_t)(int)(unsigned char)rdata[7]) * 256);
			vector<double> result = { 1, (double)rid, rpm, encoder };
			return result;
		}
	}
	else
		return { -1, 0, 0, 0 };
}
void CAN::hardware_control(int32_t& torque1, int32_t& torque2, double& angle1, double& before_angle1, double& w1, double& angle2, double& before_angle2, double& w2, int count, double &d_t)
{
	std::chrono::system_clock::time_point StartTime = std::chrono::system_clock::now();
	vector<double> readdata(4);
	sdata1[4] = torque1 & 0xFF;
	sdata1[5] = (torque1 >> 8) & 0xFF;
	sdata2[4] = torque2 & 0xFF;
	sdata2[5] = (torque2 >> 8) & 0xFF;
	if(count%2 == 0)
		CAN_Send(h, id1, len, (char*)sdata1, 0, 0);
	else
		CAN_Send(h, id2, len, (char*)sdata2, 0, 0);
	int rx_count = CAN_CountRxQueue(h);
	if (rx_count > 0)
	{
		readdata = readData();
	}
	if (readdata[0] == 1) // for torque receive
	{
		if ((int)(unsigned char)readdata[1] == 66)
		{
			encoder1 = readdata[3];
			rpm1 = readdata[2];
		}
		else if ((int)(unsigned char)readdata[1] == 65)
		{
			encoder2 = readdata[3];
			rpm2 = readdata[2];
		}
	}
	int encod1;
	int encod2;
	encod1 = encoder1 - 411;//411
	if (encod1 < 0)
		encod1 += 16383;
	encod2 = encoder2 - 9905;//9905
	if (encod2 < 0)
		encod2 += 16383;
	angle1 = (encod1 * (double)((360. / 16383.))) * (M_PI / 180.0);
	angle2 = (encod2 * (double)((360. / 16383.))) * (M_PI / 180.0);
	//cout << "encoder1: "<<angle1 << ", encoder2: " << angle2 << endl; //test
	w1 = rpm1 * 0.16666666666667 * (double)((2. * 3.141592) / 60.); //수정필요
	w2 = rpm2 * 0.16666666666667 * (double)((2. * 3.141592) / 60.);
	//cout<<"w2 : " <<w2<<endl;
	//w1 = getomega(before_angle1, angle1, d_t);
	//w2 = getomega(before_angle2, angle2, d_t);
	//w1 = rpm1;
	//w2 = rpm2;
	sleep_for(std::chrono::microseconds(25));
	std::chrono::system_clock::time_point EndTime = std::chrono::system_clock::now();
	std::chrono::microseconds micro = std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime);
	d_t = (double)micro.count() * (1.0/1000000.0);
}
void CAN::close()
{
	CAN_Close(h);
}
void CAN::encoder2angle(double encod1, double& ang1, double encod2, double& ang2) //0도일때 encoder값이 16383/2가 되도록
{
	
	ang1 = (encod1 * (double)((360. / 16383.)) - 150.) * (M_PI / 180.0);
	ang2 = (encod2 * (double)((360. / 16383.)) - 180.) * (M_PI / 180.0);

}
void CAN::rpm2omega(double rp1, double& omega1, double rp2, double& omega2)
{
	omega1 = rp1 * ((2 * M_PI) / 60); //수정필요
	omega2 = rp2 * ((2 * M_PI) / 60);
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