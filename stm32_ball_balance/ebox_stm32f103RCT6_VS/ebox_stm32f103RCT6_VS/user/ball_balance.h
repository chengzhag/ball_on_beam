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
		servo(pinServo, 100, 1.2, 1.7)
#ifdef __BALL_BALANCE_DEBUG
		, uartOut(uartDebug)
#endif
	{

	}

	void begin()
	{
		//初始化PID
		pid.setRefreshInterval(30);
		pid.setWeights(1.5, 0.2, 1.6);
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
	}

	void refresh(UartNum<float, 1>* uartPosIn)
	{
		float pos;
		if (uartPosIn->getLength() == 1)
		{
			pos = *(uartPosIn->getNum());
		}
		float pct = -pid.refresh(pos) + 50;
		limit<float>(pct, 0, 100);
		servo.setPct(pct);
	}
};

#endif

