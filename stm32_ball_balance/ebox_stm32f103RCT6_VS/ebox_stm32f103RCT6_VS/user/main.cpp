/**
  ******************************************************************************
  * @file   : *.cpp
  * @author : shentq
  * @version: V1.2
  * @date   : 2016/08/14

  * @brief   ebox application example .
  *
  * Copyright 2016 shentq. All Rights Reserved.         
  ******************************************************************************
 */

#include "ebox.h"
#include "ball_balance.h"
#include "uart_vcan.h"
#include "my_math.h"
#include "tb6612fng.h"
#include "PID.hpp"

UartVscan uartvscan(&uart1);
FpsCounter timer;
//Mpu9250_Ahrs mpu(&si2c2);
//Motor9250 motor(&PB14, &PB15, &PA6, &si2c2, 0.01);
BallBalance ballBalance(&uart2, &PB14, &PB15, &PA6, &si2c2, 0.01);

void setup()
{
    ebox_init();
    uart1.begin(115200);


	//mpu.calibrate();
	ballBalance.begin();
	//motor.begin();

	//mpu.set_parameter(2, 0.005, 100);
	//mpu.begin(400000);

	timer.begin();
}
int main(void)
{
    setup();
	float data[4];
	//float increase = 0.2, target = 0;
    while(1)
    {
		//target += increase;
		//if (target > 45 || target < -45)
		//{
		//	increase = -increase;
		//}
		//motor.setTarget(target);
		//motor.refresh();
		ballBalance.motorRefresh();

		//mpu.AHRS_Dataprepare();
		//mpu.AHRSupdate();
		//mpu.get_data_ahrs(data, data + 1, data + 2);

		

		delay_ms(2);
		//data[0] = ballBalance.getAngle();
		//data[1]=timer.getFps();
		//uartvscan.sendOscilloscope(data, 2);

	}

}


