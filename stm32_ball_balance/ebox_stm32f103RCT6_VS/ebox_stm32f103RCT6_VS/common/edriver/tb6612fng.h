#ifndef  __TB6612FNG
#define  __TB6612FNG

#include "ebox.h"
#include <math.h>

//motorPinPwm��Ҫ֧��PWM�����ע����ȷ��timer�Ƿ�ռ��
class TB6612FNG
{
	Gpio *pinA;
	Gpio *pinB;
	Gpio *pinP;
	Pwm pwm;
	uint32_t frq;
	float pct;
public:
	TB6612FNG(Gpio *motorPinA, Gpio *motorPinB,
		Gpio *motorPinPwm, uint32_t pwmFrequency = 15000);

	//����PWM�Ϳ��ƽ����
	void begin();

	//��������ٷֱ�
	void setPercent(float p);

	//��ȡ����ٷֱ�
	float getPercent();
};




#endif