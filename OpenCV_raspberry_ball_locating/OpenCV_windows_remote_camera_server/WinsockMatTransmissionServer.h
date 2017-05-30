/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  ����OpenCV��Winsock��ͼ���䣨���գ�
//
//  By ���� , at CUST, 2016.08.06
//
//  website: www.pengz0807.com  email: pengz0807@163.com
//
//M*/

#ifndef __WINSOCKMATTRANSMISSIONSEVER_H__  
#define __WINSOCKMATTRANSMISSIONSEVER_H__  

#include "opencv2/opencv.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/core/core.hpp"  
#include <stdio.h>  
#include <Winsock2.h>  

#pragma comment(lib,"WS2_32.lib")  



class WinsockMatTransmissionServer
{
	std::vector<uchar> buf;
	SOCKET sockConn;

	// ��socket����  
	// params : port    ����˿�  
	// return : -1      ����ʧ��  
	//          1       ���ӳɹ�  
	int socketConnect(int port);

public:

	WinsockMatTransmissionServer(void);
	~WinsockMatTransmissionServer(void);

	//��ʼ����
	//params��port	�����˿�
	//return��-1		����ʧ��
	//			1		���ӳɹ�
	int begin(int port=6666);


	// ����ͼ��  
	// params : image   ������ͼ��  
	// return : -1      ����ʧ��  
	//          1       ���ճɹ�  
	int receive(cv::Mat& image, int flag = cv::IMREAD_GRAYSCALE);


	// �Ͽ�socket����  
	void disconnect(void);
};

#endif  