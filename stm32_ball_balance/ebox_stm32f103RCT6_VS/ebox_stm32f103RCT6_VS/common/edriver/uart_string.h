#ifndef __UART_STRING
#define __UART_STRING

#include "ebox.h"

#define UART_STRING_BUFFER_SIZE 100
class UartString
{
	Uart *uart;
	uint16_t bufferIndex;
	char buffer[UART_STRING_BUFFER_SIZE];

	//�����յ��ֽ��жϴ�����
	void rxEvent();
	//�ַ���������ɴ�����
	FunctionPointerArg1<void,char*> stringEvent;
public:
	//��������uartX��UartString��
	UartString(Uart *uartX);
	//��ʼ��uart�����жϺ���
	void begin(uint32_t baud_rate, uint8_t _use_dma = 1);
	//��ʼ��uart�����жϺ���
	void begin(uint32_t baud_rate, uint8_t data_bit, uint8_t parity, float stop_bit, uint8_t _use_dma);

	void printf(const char *fmt, ...);
	
	//���ַ���������
	void attach(void(*stringEvent)(char *str));
	//���ַ��������Ա����
	template<typename T>
	void attach(T *pObj, void (T::*classStringEvent)(char *str));
};

template<typename T>
void UartString::attach(T *pObj, void (T::*classStringEvent)(char *str))
{
	this->stringEvent.attach(pObj, classStringEvent);
}


#endif