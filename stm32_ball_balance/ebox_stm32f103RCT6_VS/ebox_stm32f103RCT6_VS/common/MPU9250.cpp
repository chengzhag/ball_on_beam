/**
  ******************************************************************************
  * @file    mpu6050.cpp
  * @author  shentq
  * @version V1.2
  * @date    2016/08/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  * No part of this software may be used for any commercial activities by any form 
  * or means, without the prior written consent of shentq. This specification is 
  * preliminary and is subject to change at any time without notice. shentq assumes
  * no responsibility for any errors contained herein.
  * <h2><center>&copy; Copyright 2015 shentq. All Rights Reserved.</center></h2>
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#

void Mpu9250::begin(uint32_t speed)
{
    this->speed = speed;
    i2c->take_i2c_right(this->speed);
    i2c->begin(this->speed);
	i2c->write_byte(SLAVEADDRESS, PWR_MGMT_1, 0x80);    //复位
	i2c->write_byte(SLAVEADDRESS, PWR_MGMT_1, 0x00);   //唤醒
    i2c->write_byte(SLAVEADDRESS, PWR_MGMT_1, 0x01);
    i2c->write_byte(SLAVEADDRESS, SMPLRT_DIV, sample); //166hz采样
    i2c->write_byte(SLAVEADDRESS, CONFIG, 0x07);    //10hz滤波
    i2c->write_byte(SLAVEADDRESS, GYRO_CONFIG, 0x18);
    i2c->write_byte(SLAVEADDRESS, ACCEL_CONFIG, 0x00);//2g
    i2c->write_byte(SLAVEADDRESS, PWR_MGMT_1, 0x01); 
    i2c->write_byte(SLAVEADDRESS, SMPLRT_DIV, 0x09);
    i2c->write_byte(SLAVEADDRESS, CONFIG, 0x06);
    i2c->write_byte(SLAVEADDRESS, GYRO_CONFIG, 0x18);
    i2c->write_byte(SLAVEADDRESS, ACCEL_CONFIG, 0x01);
    i2c->release_i2c_right();
}
void Mpu9250::get_id(uint8_t *id)
{
	u8 adress;
	if (salve_adress == SLAVEADDRESS)
		adress = WHO_AM_I;
	if (salve_adress == RA_MAG_ADDRESS)
		adress = 0x00;
    i2c->take_i2c_right(speed);
    i2c->read_byte(salve_adress, adress, id, 1);
    i2c->release_i2c_right();
};

int16_t  Mpu9250::get_data_2byte(uint8_t reg_address)
{
    uint8_t tmp[2];
    i2c->take_i2c_right(speed);
    i2c->read_byte(salve_adress, reg_address, tmp, 2);
    i2c->release_i2c_right();
    if(salve_adress== SLAVEADDRESS)
        return ((tmp[0] << 8) + tmp[1]);
	if (salve_adress == RA_MAG_ADDRESS)
		return ((tmp[1] << 8) + tmp[0]);
}
int8_t  Mpu9250::get_data_1byte(uint8_t reg_address)
{
	uint8_t tmp[1];
	i2c->take_i2c_right(speed);
	i2c->read_byte(salve_adress, reg_address, tmp, 1);
	i2c->release_i2c_right();
	return (tmp[0]);

}
int8_t  Mpu9250::get_data(uint8_t reg_address, int16_t *buf, uint8_t num_to_read)
{
    uint8_t i, readnum;
    uint8_t tmpbuf[20];

    i2c->take_i2c_right(speed);
    readnum = i2c->read_byte(salve_adress, reg_address, tmpbuf, num_to_read * 2);
    i2c->release_i2c_right();

    for(i = 0; i < readnum / 2; i++)
    {
		if (salve_adress == SLAVEADDRESS)
            *buf++ = (tmpbuf[i * 2 + 0] << 8) + (tmpbuf[i * 2 + 1]);
		if (salve_adress == RA_MAG_ADDRESS)
            *buf++ = (tmpbuf[i * 2 + 1] << 8) + (tmpbuf[i * 2 + 0]);
	}
    return readnum / 2;
}


void  Mpu9250::write_data(u8 reg_address, u8 data)
{
	i2c->take_i2c_right(speed);
	i2c->write_byte(salve_adress, reg_address, data);
	i2c->release_i2c_right();
}

u8 Mpu9250::get_data_MPU(uint8_t reg_address)
{
	uint8_t tmp[1];
	i2c->take_i2c_right(speed);
	i2c->read_byte(SLAVEADDRESS, reg_address, tmp, 1);
	i2c->release_i2c_right();
	return (tmp[0]);
}
void Mpu9250::mode(u8 mode)
{
	if (mode)
	{
		salve_flag = 1;
		i2c->take_i2c_right(this->speed);
		i2c->write_byte(SLAVEADDRESS, INT_PIN_CFG, 0x02);
		i2c->release_i2c_right();
		salve_adress = RA_MAG_ADDRESS;
	}

	if (!mode)
	{
		salve_flag = 0;
		i2c->take_i2c_right(this->speed);
		i2c->write_byte(SLAVEADDRESS, INT_PIN_CFG, 0x00);
		i2c->release_i2c_right();
		salve_adress = SLAVEADDRESS;
	}

	
}

//将参数传给基类
Mpu9250_Ahrs::Mpu9250_Ahrs(I2c *i2c):Mpu9250(i2c)
{
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	Pitch_off = 0.00f;
	Roll_off = 0.00f;
	Yaw_off = 0.00f;
	exInt = 0;
	eyInt = 0;
	ezInt = 0;

}

int Mpu9250_Ahrs::Get_MPU9250_Data(void)
{
	u8 id;
	u8 AK_id;
	int16_t temp[7];
	int16_t AK_temp[3];
	//数据读出
	this->mode(MPU6500);
	this->get_id(&id);
	if (id == 0x73)
		this->get_data(ACCEL_XOUT_H, temp, 7);
	else
		return -1;
	delay_us(100);
	this->mode(AK8963);
	delay_ms(1);
	this->get_id(&AK_id);
	if (AK_id == 0x48)
		this->write_data(MAG_CNTL1, 0x11);
	else
		return -2;
	this->write_data(MAG_TEST1, 0x08);
	AK_temp[0] = this->get_data_2byte(RA_MAG_XOUT_L);
	AK_temp[1] = this->get_data_2byte(RA_MAG_YOUT_L);
	AK_temp[2] = this->get_data_2byte(RA_MAG_ZOUT_L);
	this->get_data_2byte(RA_MAG_ST2);

    //数据传入
	//加速度
	this->Accbuf.X = temp[0];
	this->Accbuf.Y = temp[1];
	this->Accbuf.Z = temp[2];
	//陀螺仪
	this->Gyrobuf.X = temp[4];
	this->Gyrobuf.Y = temp[5];
	this->Gyrobuf.Z = temp[6];
	//磁力计
	this->Magbuf.X = AK_temp[0];
	this->Magbuf.Y = AK_temp[1];
	this->Magbuf.Z = AK_temp[2];
	
	return 0;
}

void Mpu9250_Ahrs::get_data_buf(int16_t *mpu, int16_t *AK)
{
	*mpu++ = this->Accbuf.X;
	*mpu++ = this->Accbuf.Y;
	*mpu++ = this->Accbuf.Z;

	*mpu++ = this->Gyrobuf.X;
	*mpu++ = this->Gyrobuf.Y;
	*mpu   = this->Gyrobuf.Z;

	*AK++ = this->Magbuf.X;
	*AK++ = this->Magbuf.Y;
	*AK   = this->Magbuf.Z;
	
}

void Mpu9250_Ahrs::AHRS_Dataprepare()
{
	this->Get_MPU9250_Data();//先获取数据
	 //16.4 = 2^16/4000 lsb °/s     1/16.4=0.061     0.0174 = 3.14/180
	 //陀螺仪数据从ADC转化为弧度每秒(这里需要减去偏移值)
	GyroFinal.X = (Gyrobuf.X - Gyrooffset.X)*0.061*0.0174;
	GyroFinal.Y = (Gyrobuf.Y - Gyrooffset.Y)*0.061*0.0174;
	GyroFinal.Z = (Gyrobuf.Z - Gyrooffset.Z)*0.061*0.0174;		//读出值减去基准值乘以单位，计算陀螺仪角速度

    //+-2g,2^16/4=16384lsb/g--0.061mg/lsb
	//此处0.0098是：(9.8m/s^2)/1000,乘以mg得m/s^2
	AccFinal.X = (float)((Accbuf.X - Accoffset.X)*0.061)*0.0098;
	AccFinal.Y = (float)((Accbuf.Y - Accoffset.Y)*0.061)*0.0098;
	AccFinal.Z = (float)((Accbuf.Z - Accoffset.Z)*0.061)*0.0098;

	//±4800uT 2^16/9600 = 6.83lsb/uT     1/6.83 = 0.1465
	//地磁强度为 5-6 x 10^(-5) T = 50 - 60 uT
	MagFinal.X = (float)(Magbuf.X - Magoffset.X)*0.1465;
	MagFinal.Y = (float)(Magbuf.Y - Magoffset.Y)*0.1465;
	MagFinal.Z = (float)(Magbuf.Z - Magoffset.Z)*0.1465;
}

void Mpu9250_Ahrs::Acc_Correct()
{
	u8 i = 0;
	u8 numAcc = 200;//取100次累计量

	int Angleaccx = 0;  //加速度计校正中间变量
	int Angleaccy = 0;
	int Angleaccz = 0;

	for (i = 0; i<numAcc; i++)
	{
		Get_MPU9250_Data();
		Angleaccx += Accbuf.X;
		Angleaccy += Accbuf.Y;
		Angleaccz += Accbuf.Z;
		delay_ms(2);
	}
	Accoffset.X = Angleaccx / numAcc;
	Accoffset.Y = Angleaccy / numAcc;
	Accoffset.Z = Angleaccy / numAcc;				   //得到加速度计基准
}

void Mpu9250_Ahrs::Gyro_Correct()
{
	unsigned char i = 0;
	unsigned char numGyro = 200;

	int Gyrox = 0;
	int Gyroy = 0;
	int Gyroz = 0;							  //陀螺仪校正中间变量

	for (i = 0; i<numGyro; i++)
	{
		Get_MPU9250_Data();
		Gyrox += Gyrobuf.X;
		Gyroy += Gyrobuf.Y;
		Gyroz += Gyrobuf.Z;
		delay_ms(2);
	}

	Gyrooffset.X = Gyrox / numGyro;
	Gyrooffset.Y = Gyroy / numGyro;
	Gyrooffset.Z = Gyroz / numGyro;
}

void Mpu9250_Ahrs::Mag_Correct()
{
	unsigned char i = 0;
	unsigned char numMag = 100;
	int Magx = 0;
	int Magy = 0;
	int Magz = 0;							  //磁力计校正中间变量

	for (i = 0; i<numMag; i++)
	{
		Get_MPU9250_Data();
		Magx += Magbuf.X;
		Magy += Magbuf.Y;
		Magz += Magbuf.Z;
		delay_ms(2);
	}

	Magoffset.X = Magx / numMag;
	Magoffset.Y = Magy / numMag;
	Magoffset.Z = Magz / numMag;
}

void Mpu9250_Ahrs::get_data_adc(float *mpu, float *AK)
{
	*mpu++ = this->AccFinal.X;
	*mpu++ = this->AccFinal.Y;
	*mpu++ = this->AccFinal.Z;
	*mpu++ = this->GyroFinal.X;
	*mpu++ = this->GyroFinal.Y;
	*mpu =   this->GyroFinal.Z;

	*AK++ = this->MagFinal.X;
	*AK++ = this->MagFinal.Y;
	*AK =   this->MagFinal.Z;


}

float Mpu9250_Ahrs::invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void Mpu9250_Ahrs::AHRSupdate(void)
{
	float ax = this->AccFinal.X;
	float ay = this->AccFinal.Y;
	float az = this->AccFinal.Z;

	float gx = this->GyroFinal.X;
	float gy = this->GyroFinal.Y;
	float gz = this->GyroFinal.Z;

	float mx = this->MagFinal.X;
	float my = this->MagFinal.Y;
	float mz = this->MagFinal.Z;

	 
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, hz,bx,bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float qa, qb, qc;
	float integralFBx, integralFBy, integralFBz;
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement
		//正常化的加速度测量值
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		//正常化的磁力计测量值
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		//预先进行四元数数据运算，以避免重复运算带来的效率问题。
			// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		hz= 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
		bx = sqrt(hx * hx + hy * hy);
		bz = hz;

		vx = q1q3 - q0q2;
		vy = q0q1 + q2q3;
		vz = q0q0 -0.5f + q3q3;
		wx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		wy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		wz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		//使用叉积来计算重力和地磁误差。
			// Error is sum of cross product between estimated direction and measured direction of field vectors
		ex = (ay * vz - az * vy) + (my * wz - mz * wy)/1;
		ey = (az * vx - ax * vz) + (mz * wx - mx * wz)/1;
		ez = (ax * vy - ay * vx) + (mx * wy - my * wx)/1;
		
		//对误差进行积分
		exInt += Ki * ex * (1.0f / sampleFreq); // integral error scaled by Ki
		eyInt += Ki * ey * (1.0f / sampleFreq);
		ezInt += Ki * ez * (1.0f / sampleFreq);

		//将真实的加速度测量值以一定比例作用于陀螺仪，0就是完全信任陀螺仪，1就是完全信任加速度，大于1？
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;

       /*
		qa = q0;
		qb = q1;
		qc = q2;
		q0 += (-qb * gx - qc * gy - q3 * gz);
		q1 += (qa * gx + qc * gz - q3 * gy);
		q2 += (qa * gy - qb * gz + q3 * gx);
		q3 += (qa * gz + qb * gy - qc * gx);
		*/
		///*
		qa = q0;
		qb = q1;
		qc = q2;
		q0 += (-qb * gx - qc * gy - q3 * gz)*halfT;
		q1 += (qa * gx + qc * gz - q3 * gy)*halfT;
		q2 += (qa * gy - qb * gz + q3 * gx)*halfT;
		q3 += (qa * gz + qb * gy - qc * gx)*halfT;
		//*/
		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;

		
		//四元数转换成欧拉角
		/*
		Pitch = asin(2 * q0*q2 - 2 * q1*q3) / 3.14 * 180;
		Roll = atan2(2 * q0*q1 + 2 * q2*q3, 1 - 2 * q1*q1 - 2 * q2*q2) / 3.14 * 180;
		Yaw = atan2(2 * q0*q3 + 2 * q1*q2, 1 - 2 * q2*q2 - 2 * q3*q3) / 3.14 * 180;
		*/
	//	/*
		Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2); //俯仰角，绕y轴转动         
		Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1); //滚动角，绕x轴转动
																				//0.9和0.1是修正系数，其中5.73=0.1*57.3，乘以57.3是为了将弧度转化为角度
		Yaw = -(0.85 * (-Yaw + gz * 2 * halfT) + 0.15*57.3 * atan2(mx*cos(Roll) + my*sin(Roll)*sin(Pitch) + mz*sin(Roll)*cos(Pitch), my*cos(Pitch) - mz*sin(Pitch)));
		Pitch = Pitch * 57.3;
		Roll = Roll * 57.3;
	//	*/
	}

}
void Mpu9250_Ahrs::set_parameter(float m_kp, float m_ki, float m_sample)
{
	Kp = m_kp;;
	Ki = m_ki;
	sampleFreq = m_sample;
	sample = int(1000/m_sample-1);
	halfT = 1 / (2 * m_sample);
}

void Mpu9250_Ahrs::get_parameter(float *m_kp, float *m_ki, float *m_samplefreq, uint8_t *sampleH, float *m_halfT)
{
	*m_kp = this->Kp;
	*m_ki = this->Ki;
	*m_samplefreq = this->sampleFreq;
	*sampleH = this->sample;
	*m_halfT = halfT;

}
void Mpu9250_Ahrs::get_data_ahrs(float *m_Pitch, float *m_Roll, float *m_Yaw)
{
	*m_Pitch = Pitch-Pitch_off;
	*m_Roll = Roll-Roll_off;
	*m_Yaw = Yaw-Yaw_off;
}

void Mpu9250_Ahrs::get_data_q(float *q)
{
	*q++ = q0;
	*q++ = q1;
	*q++ = q2;
	*q = q3;
}

void Mpu9250_Ahrs::update_data(void)
{
	//this->Pitch_off = Pitch;
	//this->Roll_off = Roll;
	this->Yaw_off = Yaw;
}