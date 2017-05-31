#ifndef __MY_MATH_H
#define __MY_MATH_H

#include "ebox.h"

//����ĳ�������½�
template<typename T>
void limitLow(T &num, T limL)
{
	if (num < limL)
	{
		num = limL;
	}
}

//����ĳ�������Ͻ�
template<typename T>
void limitHigh(T &num, T limH)
{
	if (num > limH)
	{
		num = limH;
	}
}


//����ĳ���������½�
template<typename T>
void limit(T &num, T limL, T limH)
{
	limitLow(num, limL);
	limitHigh(num, limH);
}

//��������ģ��matlab��tictoc�࣬��λms
class TicToc
{
	unsigned long ticTime;
public:
	TicToc();
	//��ʼ��ʱ
	void tic();

	//���ش��ϴο�ʼ��ʱ�����ڵ�ʱ���
	unsigned long toc();

};


//֡�ʼ�����
class FpsCounter:private TicToc
{
public:
	FpsCounter();

	//��ʼ��ʱ
	void begin();

	//����֡��
	float getFps();
};






#endif