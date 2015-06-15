#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <iostream>
#include <ctime>



using namespace std;
using namespace cv;


int main(int, char**)
{
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		return -1;
	// set camera parameters
	//cap.set(CV_CAP_PROP_FPS, 30);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	//Mat edges;
	//namedWindow("edges", 1);
	Mat prevframe;
	Mat nextframe;
	Mat current_frame;
	int framecounter = 0;

	Point textcenter1(100, 50); // text variables start
	Point textcenter2(100, 100);
	Point textcenter3(100, 150);
	int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontScale = 1;
	int thickness = 2; // text variables end
	int current_time = 0;
	unsigned int start = clock(); // timer variable
	for (;;)
	{

	//	cap >> current_frame; // get a new frame from camera
		cap.grab();
		cap.retrieve(current_frame);

	//	cap >> nextframe; // this is taking the next frame, so 2 frames per loop. 
	//	cvtColor(prevframe, edges, CV_BGR2GRAY);
	//	GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
	//	Canny(edges, edges, 0, 30, 3);

		//Time and Frame tracker and display -> should be exracted to seperate functions
		current_time = 1 + ((clock() - start) / 1000);
		framecounter++;
		//	putText(edges, "Frame count: " + to_string(framecounter++), textcenter1, fontFace, fontScale,Scalar::all(255), thickness, 5);
		putText(current_frame, "Seconds past: " + to_string(cap.get(CV_CAP_PROP_FPS)), textcenter2, fontFace, fontScale, Scalar::all(255), thickness, 5);
		putText(current_frame, "FPS: " + to_string(framecounter/current_time), textcenter3, fontFace, fontScale, Scalar::all(255), thickness, 5);
		cout << to_string(framecounter / current_time) + "\n";

		imshow("Webcam", current_frame);
		//imshow("Prev", prevframe);
		//imshow("Next", nextframe);
		if (waitKey(30) >= 0) break;

	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}