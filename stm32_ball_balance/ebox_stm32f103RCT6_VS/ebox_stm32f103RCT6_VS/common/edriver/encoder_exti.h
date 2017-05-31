#ifndef  __ENCODER_EXTI
#define  __ENCODER_EXTI

#include "ebox.h"

class EncoderExti
{
	long pos;
	long posOld;
	long diff;
	Gpio *pinA;
	Gpio *pinB;
	Exti extiA;
	Exti extiB;
	void eventA();
	void eventB();
public:

	//��ʼ��������������Apin��Bpin�ֱ�Ϊ��������A��B��
	EncoderExti(Gpio *encoderPinA, Gpio *encoderPinB);

	//��ʼ����eboxѡ��
	void begin();

	//��ȡλ��
	long getPos();

	//����λ��Ϊ0
	void resetPos();

	//�������ϴ�λ�õĲ�ֵ
	void refreshDiff();

	//��ȡ����Ĳ�ֵ
	long getDiff();
};




#endif