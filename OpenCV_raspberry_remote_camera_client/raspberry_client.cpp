#include <ctime>
#include <iostream>
#include <raspicam_cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "time.h"

#include "SocketMatTransmissionClient.h"  

#include "my_math.h"
#include <algorithm>

using namespace cv;
using namespace std;


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
	raspicam::RaspiCam_Cv cam;
	Mat rawIm, railIm;
	cam.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, cam.get(CV_CAP_PROP_FRAME_WIDTH) * 0.2);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, cam.get(CV_CAP_PROP_FRAME_HEIGHT) * 0.2);
	int rawImHeight = cam.get(CV_CAP_PROP_FRAME_HEIGHT),
		rawImWitdh = cam.get(CV_CAP_PROP_FRAME_WIDTH);

	if (!cam.open())
		return 1;

	cout << "socket connecting..." << endl;
	SocketMatTransmissionClient socketMat;
	socketMat.begin("192.168.2.100");

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
		float railRegionHeight = 0.05;
		Rect railRegion(0,
			int(rawImHeight*(0.5 - railRegionHeight / 2)),
			rawImWitdh,
			rawImHeight*railRegionHeight);
		railIm = rawIm(railRegion);
		
		//预处理
		medianBlur(railIm, railIm, 5);
		int structElementSize = 3;
		Mat element = getStructuringElement(MORPH_ELLIPSE,  
			Size(2*structElementSize + 1, 2*structElementSize + 1),  
			Point(structElementSize, structElementSize));
		erode(railIm, railIm, element);
//		equalizeHist(railIm, railIm);
//		threshold(railIm, railIm, 0, 255, CV_THRESH_OTSU);

		//将灰度投影到水平方向
		vector<unsigned long> verticalVector;
		verticalProject(railIm, verticalVector);

//		//绘制亮度曲线图
//		Mat plotBrightness;
//		plotSimple(verticalVector, plotBrightness);
		
		//通过亮度曲线找到小球
		vector<unsigned long>::iterator minBrightnessIt = min_element(verticalVector.begin(), verticalVector.end());
		int minBrightnessPos = distance(verticalVector.begin(), minBrightnessIt);
		float pos = (float(minBrightnessPos) - verticalVector.size() / 2) / verticalVector.size();
		cout << pos << endl;

		///小球定位算法结束
		
		//发送图像，用于测试
		socketMat.transmit(railIm, 90);

		endTime = clock();
		cout << "fps: " << CLOCKS_PER_SEC / (endTime - startTime) << endl;
	}
	socketMat.disconnect();
	cam.release();
}




