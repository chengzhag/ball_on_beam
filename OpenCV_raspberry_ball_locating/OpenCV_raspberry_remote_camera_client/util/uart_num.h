#ifndef __UART_NUM
#define __UART_NUM

#include "ebox.h"

//#define __UART_NUM_DEBUG

template<typename NumType = float, int numBufSize = 3>
class UartNum
{
	Uart *uart;
	FunctionPointerArg1<void, UartNum*> numReceivedEvent;

	//���ڴ洢��������λ��ת��Ϊ����
	union Char2num
	{
		NumType num;
		unsigned char c[sizeof(NumType)];
	}char2num;

	NumType numBuf[numBufSize];//���ڴ洢���յ�����

	int recievedLength;

	int charBufIndex;//��������λ��index
	int numBufIndex;//�������ֵ�index
	bool isTrans;//��־��һ���Ƿ���ת���ַ�
	
	bool isBegin;//��־�Ƿ���֡ͷ

	void rxEvent()
	{
		uint8_t c;
		c = uart->read();
		if (isBegin)
		{
			recievedLength = 0;
			charBufIndex = 0;
			numBufIndex = 0;
			isTrans = false;
			isBegin = false;
		}
		//���ڷ�ת��״̬
		if (!isTrans)
		{
			if (c == '\\')
			{
				isTrans = true;
				return;
			}
			else if (c == '\n')
			{
				//��δ���뷴ת��ʱ������'\n'��������֡�Ľ���
				recievedLength = numBufIndex;
				if (recievedLength >numBufSize)
				{
#ifdef __UART_NUM_DEBUG
					printf("���ֻ������!!!recieved %d nums\r\n", recievedLength);
#endif
					recievedLength = numBufSize;
				}
				isBegin = true;
				numReceivedEvent.call(this);

#ifdef __UART_NUM_DEBUG
				printf("ת�����%d����", recievedLength);
				for (int j = 0; j < recievedLength; j++)
				{
					printf("%f ", numBuf[j]);
				}
				printf("\r\n");
#endif
				return;
			}
			else
			{
				//��ǰλ������λ
				char2num.c[charBufIndex] = c;
				charBufIndex++;
			}
		}
		//�ڷ�ת��״̬
		else
		{
			//��ǰλ������λ
			char2num.c[charBufIndex] = c;
			charBufIndex++;
			isTrans = false;
		}

		//�ж��ѽ�������λ����������������ݽ�����ϣ��洢����
		if (charBufIndex >= sizeof(NumType))
		{
			if (numBufIndex < numBufSize)
			{
				numBuf[numBufIndex] = char2num.num;
			}
			numBufIndex++;
			charBufIndex = 0;
		}
	}

public:

	//��������uartX��UartNum��
	UartNum(Uart *uartX) :
		uart(uartX),
		isTrans(false),
		charBufIndex(0),
		numBufIndex(0),
		recievedLength(0),
		isBegin(true)
	{

	}

	//��ʼ��uart�����жϺ���
	void begin(uint32_t baud_rate, uint8_t data_bit, uint8_t parity, float stop_bit, uint8_t _use_dma)
	{
		uart->begin(baud_rate, data_bit, parity, stop_bit, _use_dma);
		uart->attach(this, &UartNum::rxEvent, RxIrq);
	}

	//��ʼ��uart�����жϺ���
	void begin(uint32_t baud_rate, uint8_t _use_dma = 1)
	{
		uart->begin(baud_rate, _use_dma);
		uart->attach(this, &UartNum::rxEvent, RxIrq);
	}

	void printf(const char *fmt, ...)
	{
		uart->printf(fmt);
	}

	void sendNum(NumType* num, int length)
	{
		union Num2Char {
			NumType num;
			unsigned char c[sizeof(NumType)];
		}num2char;
		int i = 0, j = 0, k = 0;
		for (i = 0; i < length; i++)
		{
			num2char.num = num[i];
			for (j = 0; j < sizeof(NumType); j++)
			{
				if (num2char.c[j] == '\n' || num2char.c[j] == '\\')
				{
					uart->wait_busy();
					uart->write('\\');
					k++;
				}
				uart->wait_busy();
				uart->write(num2char.c[j]);
				k++;
			}
		}
		uart->wait_busy();
		uart->write('\n');
	}

	//�Ƿ�ʼ����
	bool busy()
	{
		return !isBegin;
	}

	void waitBusy()
	{
		while (busy());
	}

	NumType* getNum()
	{
		return numBuf;
	}

	int getLength()
	{
		return recievedLength;
	}

	//�����ݴ�����
	void attach(void(*numReceivedEvent)(UartNum<NumType, numBufSize>*))
	{
		this->numReceivedEvent.attach(numReceivedEvent);
	}

	//�����ݴ����Ա����
	template<typename T>
	void attach(T *pObj, void (T::*numReceivedEvent)(UartNum<NumType,numBufSize>*))
	{
		this->numReceivedEvent.attach(pObj, numReceivedEvent);
	}
};

#endif