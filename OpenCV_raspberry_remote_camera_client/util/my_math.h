#ifndef __MY_MATH_H
#define __MY_MATH_H

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



#endif