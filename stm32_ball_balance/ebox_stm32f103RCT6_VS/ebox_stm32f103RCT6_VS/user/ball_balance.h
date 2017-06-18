#ifndef __BALL_BALANCE_H
#define __BALL_BALANCE_H

#include "ebox.h"
#include "uart_num.h"
//#include "servo.h"
#include "PID.hpp"
#include "my_math.h"
#include "uart_vcan.h"
#include "signal_stream.h"
#include "drv8825.h"
#include "MPU6050.h"
#include "tb6612fng.h"

#define __BALL_BALANCE_DEBUG
#define __FILTER_WINDOW_SIZE 3

class Motor9250
{
	Mpu9250_Ahrs mpu;
	DRV8825 motor;
	greg::PID pid;
	float angle[3];

public:
	Motor9250(Gpio *motorPinDir, 
		Gpio *motorPinPwm, SoftI2c *i2c, float refreshInterval=0.01) :
		motor(motorPinPwm, motorPinDir),
		mpu(i2c)
	{
		pid.setRefreshInterval(refreshInterval);
		pid.setWeights(0.1, 0, 0.01);
		pid.setOutputLowerLimit(-INF_FLOAT);
		pid.setOutputUpperLimit(INF_FLOAT);
		pid.setDesiredPoint(0);
	}

	void begin(const float &Kp, const float &Ki, const float &Kd)
	{
		pid.setWeights(Kp, Ki, Kd);
		begin();
	}

	void begin()
	{
		mpu.set_parameter(10.3, 0.008, 125);
		mpu.begin(400000);

		//mpu.Acc_Correct();
		//mpu.Gyro_Correct();

		motor.begin();
		motor.setPercent(0);
	}

	void refresh()
	{
		mpu.ahrs_dataprepare();
		mpu.ahrs_update();
		mpu.get_data_ahrs(angle, angle + 1, angle + 2);

		float motorOut = 0;
		motorOut += pid.refresh(angle[1]);
		limit<float>(motorOut, -100, 100);
		motor.setPercent(motorOut);
	}

	void setTarget(float angle)
	{
		limit<float>(angle, -90, 90);
		pid.setDesiredPoint(angle);
	}

	float getAngle()
	{
		return angle[1];
	}
};

class BallBalance
{
	UartNum<float, 1> uartPos;
	DRV8825 motor;
	sky::PID pid;
	
	class AverageFilter :public SignalStream<float, __FILTER_WINDOW_SIZE>
	{
	public:
		float getFilterOut(float newNum)
		{
			push(newNum);
			float temp = 0;
			for (int i = 0; i < __FILTER_WINDOW_SIZE; i++)
			{
				temp += operator[](i);
			}
			temp /= __FILTER_WINDOW_SIZE;
			return temp;
		}
	}filter;

#ifdef __BALL_BALANCE_DEBUG
	UartVscan uartOut;
	FpsCounter timer;
#endif

	void refresh(UartNum<float, 1>* uartPosIn)
	{
		float pos = 0;
		if (uartPosIn->getLength() == 1)
		{
			pos = *(uartPosIn->getNum());
			pos = filter.getFilterOut(pos);
		}
		float pct = pid.refresh(pos);
		limit<float>(pct, -100, 100);
#ifdef __BALL_BALANCE_DEBUG
		float outData[] = { pos ,pct ,timer.getFps() };
		uartOut.sendOscilloscope(outData, 3);
#endif
		motor.setPercent(pct);
	}

public:
	BallBalance(Uart* uartPosIn, Gpio *motorPinPwm, Gpio *motorPinDir
		/*,SoftI2c *i2c,*//* float refreshInterval = 0.01*/
#ifdef __BALL_BALANCE_DEBUG
		, Uart* uartDebug = &uart1
#endif
	) :
		uartPos(uartPosIn),
		motor(motorPinPwm, motorPinDir)
#ifdef __BALL_BALANCE_DEBUG
		, uartOut(uartDebug)
#endif
	{

	}

	void begin()
	{
		//初始化PID
		pid.setRefreshRate(30);
		pid.setWeights(0.01, 0, 0);
		pid.setOutputLowerLimit(-INF_FLOAT);
		pid.setOutputUpperLimit(INF_FLOAT);
		pid.setISeperateThres(60);
		pid.setDesiredPoint(0);

		//初始化电机
		motor.begin();
		motor.setPercent(0);
		
		//初始化数据传入串口
		uartPos.begin(115200);
		uartPos.attach(this, &BallBalance::refresh);

#ifdef __BALL_BALANCE_DEBUG
		uartOut.begin(115200);
		timer.begin();
#endif
	}


	void setTarget(float pos)
	{
		limit<float>(pos, -100, 100);
		pid.setDesiredPoint(pos);
	}
	
};

#endif

