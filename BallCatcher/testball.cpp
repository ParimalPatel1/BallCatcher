//objectTrackingTutorial.cpp

//Written by  Kyle Hounslow 2013

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.

#pragma once
#include  <opencv2\opencv.hpp>
#include <sstream>
#include <string>
#include <iostream>
#include <opencv\highgui.h>
#include <opencv\cv.h>

#include "motiontracking.hpp"
using namespace cv;
using namespace std;
//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 30;
int H_MAX = 120;
int S_MIN = 54;
int S_MAX = 135;
int V_MIN = 135;
int V_MAX = 240;

int VV_MAX = 256;
int SS_MAX = 256;
int HH_MAX = 256;

int const max_lowThreshold = 100;
int lowThreshold = 75;
int ratio = 3;
int kernel_size = 3;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 1000; // 50
//minimum and maximum object area
const int MIN_OBJECT_AREA = 100;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName3 = "Trackbars";
void on_trackbar(int, void*)
{//This function gets called whenever a
 // trackbar position is changed
}
string intToString(int number) {
	std::stringstream ss;
	ss << number;
	return ss.str();
}
void createTrackbars() {
	//create window for trackbars
	namedWindow(trackbarWindowName3, 0);
	//create memory to store trackbar name on window
	char TrackbarName[60];
	sprintf(TrackbarName, "H_MIN", H_MIN);
	sprintf(TrackbarName, "H_MAX", H_MAX);
	sprintf(TrackbarName, "S_MIN", S_MIN);
	sprintf(TrackbarName, "S_MAX", S_MAX);
	sprintf(TrackbarName, "V_MIN", V_MIN);
	sprintf(TrackbarName, "V_MAX", V_MAX);

	//sprintf(TrackbarName, "thresh", lowThreshold);
//	sprintf(TrackbarName, "ratio", ratio);
//	sprintf(TrackbarName, "kernel", kernel_size);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar("H_MIN", trackbarWindowName3, &H_MIN, HH_MAX, on_trackbar);
	createTrackbar("H_MAX", trackbarWindowName3, &H_MAX, HH_MAX, on_trackbar);
	createTrackbar("S_MIN", trackbarWindowName3, &S_MIN, SS_MAX, on_trackbar);
	createTrackbar("S_MAX", trackbarWindowName3, &S_MAX, SS_MAX, on_trackbar);
	createTrackbar("V_MIN", trackbarWindowName3, &V_MIN, VV_MAX, on_trackbar);
	createTrackbar("V_MAX", trackbarWindowName3, &V_MAX, VV_MAX, on_trackbar);

	//createTrackbar("thresh", trackbarWindowName3, &lowThreshold, max_lowThreshold, on_trackbar);
	//createTrackbar("ratio", trackbarWindowName3, &ratio, 10, on_trackbar);
//	createTrackbar("kernel", trackbarWindowName3, &kernel_size, 10, on_trackbar);


}
bool largerradius(ballLoc a, ballLoc b)
{
	return (a.radius < b.radius);
}
void drawObject(int x, int y, int r, Mat &frame) {

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
	circle(frame, Point(x, y), r, Scalar(255, 0, 0), 1);
}
void morphOps(Mat &thresh) {

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);


	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);



}
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed) {

	Mat temp;
	threshold.copyTo(temp);
	vector<ballLoc> ball_canidates;
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours, CHANGES THRESHOLD IMAGE
	
	for (int idx = 0; idx > 0; idx = hierarchy[idx][0])
	{
		Scalar color(rand() & 255, rand() & 255, rand() & 255);
		drawContours(cameraFeed, contours, idx, color, CV_FILLED, 8, hierarchy);
	}

	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	cv::Point2f center;
	float radius;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS) {
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we save a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea) {
				//	x = moment.m10 / area;
				//	y = moment.m01 / area;
					ballLoc ball_canidate;
					cv::minEnclosingCircle(contours[index], ball_canidate.center, ball_canidate.radius);
					ball_canidates.push_back(ball_canidate);
					objectFound = true;
					refArea = area;
				}
				//else objectFound = false;


			}
			//let user know you found an object
			if (objectFound == true) {
				putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
				//draw object location on screen
				sort(ball_canidates.begin(), ball_canidates.end(), largerradius); // sort by diameter
				int i = ball_canidates.size() - 1;
				drawObject(ball_canidates[i].center.x, ball_canidates[i].center.y, ball_canidates[i].radius, cameraFeed);
			}

		}
		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	}
}
//#define ORIGINAL
int main(int argc, char* argv[])
{
	//some boolean variables for different functionality within this
	//program
	bool trackObjects = true;
	bool useMorphOps = false;
	//Matrix to store each frame of the webcam feed
	Mat currcameraFeed;
	Mat prevcameraFeed;
	//matrix storage for HSV image
	Mat HSV;

	Mat edges;
	Mat CurrGraySource;
	Mat PrevGraySource;
	//matrix storage for binary threshold image
	Mat threshold;
	Mat HSVthreshold;
	//x and y values for the location of the object
	int x = 0, y = 0;
	//create slider bars for HSV filtering
	createTrackbars();
	//video capture object to acquire webcam feed
	VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(1);
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	unsigned int last_time = clock();
	unsigned int framecounter = 0;
	float fps;
	Mat differenceImage; // grayscale
	Mat thresholdImage; // binary
	Mat blurImage;		// grayscale
	capture.read(prevcameraFeed);
	cv::cvtColor(prevcameraFeed, PrevGraySource, COLOR_BGR2GRAY);
	while (1) {
		//store image to matrix
		fps = 1000.0 / (float)(1 + clock() - last_time); // time stuff
		last_time = clock();
		cout << "FPS: " << fps << endl; // faster than draw??

		capture.read(currcameraFeed);
		//convert frame from BGR to HSV colorspace
#ifdef ORIGINAL
		cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
		inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		if (useMorphOps)
			morphOps(threshold);
		if (trackObjects)
			trackFilteredObject(x, y, threshold, cameraFeed);
#endif
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
#ifndef ORIGNIAL  
	//	int fillvalue = 120;

	//	bitwise_or(DrawImage, Scalar(fillvalue), DrawImage, ThresholdImage);
		cvtColor(currcameraFeed, HSV, COLOR_BGR2HSV);
		inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), HSVthreshold);
		morphOps(HSVthreshold);

		cv::cvtColor(currcameraFeed, CurrGraySource, COLOR_BGR2GRAY);
		absdiff(PrevGraySource, CurrGraySource, differenceImage); // order matters here
		 //	imshow("diff", differenceImage);
		 //threshold intensity image at a given sensitivity value
		//no prev threshold to filter out ball, or set backround image when ball found
		cv::threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
		//blur the image to get rid of the noise. This will output an intensity image
		blur(thresholdImage, blurImage, Size(BLUR_SIZE, BLUR_SIZE));
		//threshold again to obtain binary image from blur output
		cv::threshold(blurImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);

	//	cv::Canny(grey, edges, lowThreshold, lowThreshold*ratio, kernel_size);
		bitwise_and(HSVthreshold, thresholdImage, threshold);
		trackFilteredObject(x, y, threshold, currcameraFeed);

		CurrGraySource.copyTo(PrevGraySource);
#endif

		//show frames 
	//	imshow(windowName2, edges);
		imshow(windowName2, threshold);
		imshow(windowName, currcameraFeed);
		imshow(windowName1, HSVthreshold);


		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(30);
	}






	return 0;
}
