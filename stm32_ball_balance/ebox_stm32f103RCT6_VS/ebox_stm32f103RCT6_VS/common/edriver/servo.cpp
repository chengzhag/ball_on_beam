#include "servo.h"

Servo::Servo(Gpio* pin, uint32_t frequency /*= 50*/, float limLowMs /*= 0.6*/, float limHighMs /*= 2.4*/) :
	pwm(pin), pct(50)
{
	//Ϊ��֤һ��������������2.4ms�ĸߵ�ƽʱ�䣬������Ϊ����2.5ms����Ƶ�����400Hz
	limit(frequency, (uint32_t)50, (uint32_t)400);
	frq = frequency;
	float T = 1.0 / frequency * 1000;//���ڣ���msΪ��λ
	limLow = limLowMs / T * 1000;
	limHigh = limHighMs / T * 1000;
}

void Servo::begin()
{
	pwm.begin(frq, 0);
	setPct(50);
}

void Servo::setPct(float percent)
{
	limit(percent, 0.f, 100.f);
	pct = percent;
	pwm.set_duty(percent*(limHigh - limLow) / 100 + limLow);
}

float Servo::getPct()
{
	return pct;
}
