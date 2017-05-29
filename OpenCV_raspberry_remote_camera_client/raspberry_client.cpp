#include <ctime>
#include <iostream>
#include <raspicam_cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "time.h"

#include "SocketMatTransmissionClient.h"  

using namespace cv;
using namespace std;
 
int main(int argc, char **argv)
{
	raspicam::RaspiCam_Cv cam;
	Mat image;
	cam.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, cam.get(CV_CAP_PROP_FRAME_WIDTH)*0.5);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, cam.get(CV_CAP_PROP_FRAME_HEIGHT)*0.5);
//	cam.set(CV_CAP_PROP_FRAME_WIDTH, 640);
//	cam.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	if (!cam.open())
		return 1;
	
	cout << "socket connecting..." << endl;
	SocketMatTransmissionClient socketMat;  
	socketMat.begin("192.168.2.100");
    
	double startT, endT; 
	for (;;)
	{
		startT = clock();
		waitKey(30);
		cam.grab();
		cam.retrieve(image);
		
		if (image.empty())  
			return 0;  
		
		socketMat.transmit(image,30);
		
		endT = clock();
		cout <<"fps: "<< CLOCKS_PER_SEC / (endT - startT) << endl;
	}
	socketMat.disconnect();  
	cam.release();
}




