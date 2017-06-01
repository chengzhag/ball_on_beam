#ifndef __BALL_BALANCE_H
#define __BALL_BALANCE_H

#include "ebox.h"
#include "uart_num.h"
#include "servo.h"
#include "PID.hpp"
#include "my_math.h"
#include "uart_vcan.h"

#define __BALL_BALANCE_DEBUG

class BallBalance
{
	UartNum<float, 1> uartPos;
	Servo servo;
	sky::PID pid;

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
		servo(pinServo, 100, 1.2, 2.0)
#ifdef __BALL_BALANCE_DEBUG
		, uartOut(uartDebug)
#endif
	{

	}

	void begin()
	{
		//初始化PID
		pid.setRefreshRate(30);
		pid.setWeights(0.25, 0.2, 0.16);
		pid.setOutputLowerLimit(-INF_FLOAT);
		pid.setOutputUpperLimit(INF_FLOAT);
		pid.setISeperateThres(50);
		pid.setDesiredPoint(0);

		//初始化舵机
		servo.begin();
		servo.setPct(51);
		
		//初始化数据传入串口
		uartPos.begin(115200);
		uartPos.attach(this, &BallBalance::refresh);

#ifdef __BALL_BALANCE_DEBUG
		uartOut.begin(115200);
#endif
	}

	void refresh(UartNum<float, 1>* uartPosIn)
	{
		float pos;
		if (uartPosIn->getLength() == 1)
		{
			pos = *(uartPosIn->getNum());
		}
		float pct = -pid.refresh(pos) + 51;
		limit<float>(pct, 0, 100);
		servo.setPct(pct);
#ifdef __BALL_BALANCE_DEBUG
		float outData[] = { pos ,pct };
		uartOut.sendOscilloscope(outData, 2);
#endif
	}
};

#endif

