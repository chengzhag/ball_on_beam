#include <ctime>
#include <iostream>
#include <raspicam_cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "SocketMatTransmissionClient.h"  

#include "my_opencv.h"
#include "my_math.h"
#include <algorithm>
#include "uart_num.h"


using namespace cv;
using namespace std;

//#define STDIO_DEBUG
//#define SOCKET_SEND_IMAGE

int main(int argc, char **argv)
{
	///��������
	//���ݲɼ�
	raspicam::RaspiCam_Cv cam;
	Mat rawIm, railIm;
	cam.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, cam.get(CV_CAP_PROP_FRAME_WIDTH) * 0.5);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, cam.get(CV_CAP_PROP_FRAME_HEIGHT) * 0.5);
	const int rawImHeight = cam.get(CV_CAP_PROP_FRAME_HEIGHT),
		rawImWitdh = cam.get(CV_CAP_PROP_FRAME_WIDTH);
	
	//�㷨���
	//���е���λ��ͼ��
	float railRegionHeight = 0.005, railRegionShift = -0.06;
	Rect railRegion(0,
		int(rawImHeight*(0.5 - railRegionHeight / 2 + railRegionShift)),
		rawImWitdh,
		rawImHeight*railRegionHeight);
	//Ԥ����
	const int structElementSize = 3;
	Mat element = getStructuringElement(MORPH_ELLIPSE,  
		Size(2*structElementSize + 1, 2*structElementSize + 1),  
		Point(structElementSize, structElementSize));
	//���Ҷ�ͶӰ��ˮƽ����
	vector<unsigned long> verticalVector;
	//С��λ�ü��㣬��λmm
	const float railLength=300;
	const float camCenterShift = 0;
	
	//��ʼ������
	if (!cam.open())
		return 1;

#ifdef SOCKET_SEND_IMAGE
	cout << "socket connecting..." << endl;
	SocketMatTransmissionClient socketMat;
	socketMat.begin("192.168.2.100");
#endif // SOCKET_SEND_IMAGE
	
	
	//��ʼ������
	UartNum<float> uart;
	uart.begin();

	double timeStart = 0, timeEnd = 0;
	while (1)
	{
		cam.grab();
		cam.retrieve(rawIm);

		if (rawIm.empty())
			return 0;

		/// С��λ�㷨��ʼ

		//���е���λ��ͼ��
		railIm = rawIm(railRegion);
		
		//Ԥ����
		morphologyEx(railIm, railIm, CV_MOP_ERODE, element);
		GaussianBlur(railIm, railIm, Size(int(0.03*rawImWitdh) * 2 + 1, 1), int(0.02*rawImWitdh) * 2 + 1, 0);//��С��뾶������Ϊ���ڳ���
//		medianBlur(railIm, railIm, 9);
//		morphologyEx(railIm, railIm, CV_MOP_DILATE, element);
//		medianBlur(railIm, railIm, 3);
//		equalizeHist(railIm, railIm);
//		threshold(railIm, railIm, 0, 255, CV_THRESH_OTSU);

		//���Ҷ�ͶӰ��ˮƽ����
		verticalProject(railIm, verticalVector);

		//������������ͼ
//		Mat plotBrightness;
//		plotSimple(verticalVector, plotBrightness);
		
		//ͨ�����������ҵ�С��
		vector<unsigned long>::iterator minBrightnessIt = min_element(verticalVector.begin(), verticalVector.end());
		int minBrightnessPos = distance(verticalVector.begin(), minBrightnessIt);
		
		//ֱ���Լ�СֵΪ����
		float pos = (float(minBrightnessPos) - verticalVector.size() / 2)*railLength / verticalVector.size() - camCenterShift;
		
//		//�Է���Χ+-0.1*rawImWitdh��������ġ����ã���������
//		int peakStart = minBrightnessPos - 0.1*rawImWitdh, peakEnd;
//		limitLow<int>(peakStart, 0);
//		peakEnd = 2*minBrightnessPos - peakStart;
//		unsigned long peakArea = 0, indexWeighted = 0;
//		for (int i = peakStart; i < peakEnd; i++)
//		{
//			peakArea += verticalVector[i];
//			indexWeighted += verticalVector[i]*i;
//		}
//		float peakCenter = indexWeighted / peakArea;
//		float pos = (peakCenter - verticalVector.size() / 2)*railLength / verticalVector.size() + camCenterShift;
		
		
#ifdef STDIO_DEBUG
		//����֡��
		cout << "fps: " << 1.0 / (timeEnd - timeStart)*(double)getTickFrequency() << '\t' << pos << endl;
		timeStart = timeEnd;
		timeEnd = (double)getTickCount();
#endif // STDIO_DEBUG
		
		uart.sendNum(&pos, 1);

		///С��λ�㷨����
		
#ifdef SOCKET_SEND_IMAGE
		//����ͼ�����ڲ���
		socketMat.transmit(railIm, 90);
#endif // SOCKET_SEND_IMAGE
		
		
	}
#ifdef SOCKET_SEND_IMAGE
	socketMat.disconnect();
#endif // SOCKET_SEND_IMAGE
	cam.release();
}




