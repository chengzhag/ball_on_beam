#ifndef  __DRV8825_H
#define  __DRV8825_H
#include "ebox.h"


class DRV8825
{
	Pwm pwm;
	Gpio* pinDir;
	float pct;
	int maxFre;

	//�������Ƶ�ʣ�������������maxFre��Χ��
	void setFrequency(int frequency);

public:
	//DRV8825�����������
	//pinStep�������STEP�ţ�����Ϊpwm��
	//pinDir�������DIR��
	DRV8825(Gpio* pinStep,Gpio* pinDir, int maxFre=25000);

	//��ʼ��pwm�����dir���
	void begin();

	//��������ٷֱ�
	void setPercent(float percent);
	
	//��ȡ�ٷֱ�
	float getPercent();

	//��ȡƵ��
	int getFrequency();

	//��ȡ���Ƶ��
	uint32_t getMaxFrequency();
};


#endif
