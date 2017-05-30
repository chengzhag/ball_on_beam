#include <ctime>
#include <iostream>
#include <raspicam_cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "time.h"

#include "SocketMatTransmissionClient.h"  

#include "my_math.h"
#include <algorithm>
#include "wiringSerial.h"

using namespace cv;
using namespace std;

#define SOCKET_SEND_IMAGE

void verticalProject(const Mat& src, vector<unsigned long>& dst)
{
	CV_Assert(src.depth() != sizeof(unsigned char));
	int i, j;
	const unsigned char* p;
	dst.resize(src.cols);
	for (j = 0; j < src.cols; j++) {
		dst[j] = 0;
		for (i = 0; i < src.rows; i++) {
			p = src.ptr<unsigned char>(i);
			dst[j] += p[j];
		}
	}
}

template<typename T>
	void plotSimple(const vector<T> src, Mat& dst, unsigned int plotHeight = 256)
{
	dst = Mat(Size(src.size(), plotHeight), CV_8UC1, Scalar(0));
	T maxVal = *max_element(src.begin(), src.end());
	for (int i = 0; i < src.size(); i++)
	{
		unsigned int y;
		y = plotHeight - 1 - (float)src[i] * (plotHeight - 1) / maxVal;
		for (int j = y; j < plotHeight; j++)
		{
			dst.at<uchar>(j, i) = 255;
		}
	}
}

int main(int argc, char **argv)
{
	///声明变量
	//数据采集
	raspicam::RaspiCam_Cv cam;
	Mat rawIm, railIm;
	cam.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, cam.get(CV_CAP_PROP_FRAME_WIDTH) * 0.5);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, cam.get(CV_CAP_PROP_FRAME_HEIGHT) * 0.5);
	int rawImHeight = cam.get(CV_CAP_PROP_FRAME_HEIGHT),
		rawImWitdh = cam.get(CV_CAP_PROP_FRAME_WIDTH);
	
	//算法相关
	//剪切导轨位置图像
	float railRegionHeight = 0.008, railRegionShift = -0.06;
	Rect railRegion(0,
		int(rawImHeight*(0.5 - railRegionHeight / 2 + railRegionShift)),
		rawImWitdh,
		rawImHeight*railRegionHeight);
	//预处理
	int structElementSize = 3;
	Mat element = getStructuringElement(MORPH_ELLIPSE,  
		Size(2*structElementSize + 1, 2*structElementSize + 1),  
		Point(structElementSize, structElementSize));
	//将灰度投影到水平方向
	vector<unsigned long> verticalVector;
	//小球位置计算，单位mm
	const float railLength=300;
	const float camCenterShift = -3.6;
	
	//初始化连接
	if (!cam.open())
		return 1;

#ifdef SOCKET_SEND_IMAGE
	cout << "socket connecting..." << endl;
	SocketMatTransmissionClient socketMat;
	socketMat.begin("192.168.2.100");
#endif // SOCKET_SEND_IMAGE

	double startTime, endTime;
	while (1)
	{
		startTime = clock();
		waitKey(30);
		cam.grab();
		cam.retrieve(rawIm);

		if (rawIm.empty())
			return 0;

		/// 小球定位算法开始

		//剪切导轨位置图像
		railIm = rawIm(railRegion);
		
		//预处理
//		medianBlur(railIm, railIm, 3);
		erode(railIm, railIm, element);
//		equalizeHist(railIm, railIm);
//		threshold(railIm, railIm, 0, 255, CV_THRESH_OTSU);

		//将灰度投影到水平方向
		verticalProject(railIm, verticalVector);

//		//绘制亮度曲线图
//		Mat plotBrightness;
//		plotSimple(verticalVector, plotBrightness);
		
		//通过亮度曲线找到小球
		vector<unsigned long>::iterator minBrightnessIt = min_element(verticalVector.begin(), verticalVector.end());
		int minBrightnessPos = distance(verticalVector.begin(), minBrightnessIt);
		float pos = (float(minBrightnessPos) - verticalVector.size() / 2)*railLength / verticalVector.size() + camCenterShift;
		cout << pos << endl;

		///小球定位算法结束
		
#ifdef SOCKET_SEND_IMAGE
		//发送图像，用于测试
		socketMat.transmit(railIm, 90);
#endif // SOCKET_SEND_IMAGE

		endTime = clock();
		cout << "fps: " << CLOCKS_PER_SEC / (endTime - startTime) << endl;
	}
#ifdef SOCKET_SEND_IMAGE
	socketMat.disconnect();
#endif // SOCKET_SEND_IMAGE
	cam.release();
}




