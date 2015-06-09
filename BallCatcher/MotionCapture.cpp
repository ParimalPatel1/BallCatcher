#include "opencv2/opencv.hpp"
#include <Windows.h>

using namespace cv;

int main(int, char**)
{
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		return -1;
	Mat edges;
	namedWindow("edges", 1);
	Mat prevframe;
	Mat nextframe;
	for (;;)
	{
		cap >> prevframe; // get a new frame from camera
		cap >> nextframe;
		cvtColor(prevframe, edges, CV_BGR2GRAY);
		GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
		Canny(edges, edges, 0, 30, 3);
		imshow("edges", edges);
		//imshow("Prev", prevframe);
		imshow("Next", nextframe);
		if (waitKey(30) >= 0) break;
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}