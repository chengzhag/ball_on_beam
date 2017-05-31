#ifndef __SERVO_H
#define __SERVO_H
#include "ebox.h"
#include "stm32f10x_tim.h"
#include "accurate_pwm.h"
#include "my_math.h"

//�������
//֧�ֶ˿ڣ�    
//TIM1 PA8  PA9  PA10 PA11 
//TIM2 PA0  PA1  PA2  PA3
//TIM3 PA6  PA7  PA0  PA1
//TIM4 PB6  PB7  PB8  PB9
class Servo
{
	AcurratePwm pwm;
	uint32_t frq;
	uint16_t limLow,limHigh;
	float pct;

public:

	//�����������Ĭ��Ƶ��50Hz
	//��׼50Hzʱ��ռ�ձȷ�Χ3%~13%
	//��׼����ߵ�ƽ��Χ0.5ms~2.5ms
	//�˴�ȡlimLowMs=0.6ms~limHighMs=2.4ms����һЩ����
	Servo(Gpio* pin, uint32_t frequency = 50, float limLowMs = 0.6, float limHighMs = 2.4);

	//��ʼ��pwm
	void begin();

	//���ðٷֱ�
	void setPct(float percent);

	//��ȡ�ٷֱ�
	float getPct();
};



#endif