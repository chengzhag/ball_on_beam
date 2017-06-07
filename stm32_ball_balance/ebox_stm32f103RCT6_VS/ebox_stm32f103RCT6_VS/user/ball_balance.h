#ifndef __BALL_BALANCE_H
#define __BALL_BALANCE_H

#include "ebox.h"
#include "uart_num.h"
#include "servo.h"
#include "PID.hpp"
#include "my_math.h"
#include "uart_vcan.h"
#include "signal_stream.h"

#define __BALL_BALANCE_DEBUG
#define __FILTER_WINDOW_SIZE 2

class BallBalance
{
	UartNum<float, 1> uartPos;
	Servo servo;
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
#endif
public:
	BallBalance(Uart* uartPosIn, Gpio* pinServo
#ifdef __BALL_BALANCE_DEBUG
		, Uart* uartDebug = &uart1
#endif
	) :
		uartPos(uartPosIn),
		servo(pinServo, 100, 1.2, 2.13)
#ifdef __BALL_BALANCE_DEBUG
		, uartOut(uartDebug)
#endif
	{

	}

	void begin()
	{
		//初始化PID
		pid.setRefreshRate(30);
		pid.setWeights(0.25 / 5, 0.2 / 5, 0.16 / 4.5);
		pid.setOutputLowerLimit(-INF_FLOAT);
		pid.setOutputUpperLimit(INF_FLOAT);
		pid.setISeperateThres(50);
		pid.setDesiredPoint(0);

		//初始化舵机
		servo.begin();
		servo.setPct(50);
		
		//初始化数据传入串口
		uartPos.begin(115200);
		uartPos.attach(this, &BallBalance::refresh);

#ifdef __BALL_BALANCE_DEBUG
		uartOut.begin(115200);
#endif
	}
	void refresh(UartNum<float, 1>* uartPosIn)
	{
		float pos = 0;
		if (uartPosIn->getLength() == 1)
		{
			pos = *(uartPosIn->getNum());
			pos = filter.getFilterOut(pos);
		}
		float pct = pid.refresh(pos) + 50;
		limit<float>(pct, 0, 100);
		servo.setPct(pct);
#ifdef __BALL_BALANCE_DEBUG
		float outData[] = { pos ,pct };
		uartOut.sendOscilloscope(outData, 2);
#endif

		//static float i = 0, increase = 0.5;
		//servo.setPct(i);
		//i += increase;
		//if (i > 100 || i < 0)
		//{
		//	increase = -increase;
		//}
		//uartOut.sendOscilloscope(&i, 1);

		//servo.setPct(50);
	}
};

#endif

