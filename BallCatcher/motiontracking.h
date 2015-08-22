#pragma once
#include <opencv\cv.h>
#include  <opencv2\opencv.hpp>
#include <Windows.h>
#include <ctime>
#include "ProjectionMethods.h"

using namespace cv;
using namespace ipp;
using namespace std;
#define RESOLUTION Size(640,480)
const static int SENSITIVITY_VALUE = 20;
//size of blur used to smooth the intensity image output from absdiff() function
const static int BLUR_SIZE = 10;

struct ballLoc
{
	Point center;
	int radius;
};
class objectTracker
{
public:
	int trackedObjects;
	int ballcount;
	LARGE_INTEGER Frequency;
	objectTracker()
	{
		trackedObjects = 0;
		ballcount = 0;
	}
	struct timestamp_t
	{
		LARGE_INTEGER currentTime;
		LARGE_INTEGER ElapsedMicroseconds;  // in microseconds
	};
	struct ballTrack
	{
		ballLoc ball_location;
		timestamp_t timestamp;
	};
	ballTrack trackerList[2][100]; 
	void track(int objnum) // adds an object to be timed. assumes its the same object for now.
	{
		QueryPerformanceCounter(&trackerList[objnum][ballcount].timestamp.currentTime);
		if (ballcount == 0)
		{
			QueryPerformanceFrequency(&Frequency); // get frequency only once
			trackerList[trackedObjects][ballcount].timestamp.ElapsedMicroseconds.QuadPart = 1;
		}
		else
		{
			trackerList[trackedObjects][ballcount].timestamp.ElapsedMicroseconds.QuadPart = trackerList[trackedObjects][ballcount].timestamp.currentTime.QuadPart - trackerList[trackedObjects][ballcount - 1].timestamp.currentTime.QuadPart;
			trackerList[trackedObjects][ballcount].timestamp.ElapsedMicroseconds.QuadPart *= 1000000;
			trackerList[trackedObjects][ballcount].timestamp.ElapsedMicroseconds.QuadPart /= Frequency.QuadPart;
		}
		ballcount++;
	}
};

template <class type>
class CircularBuffer
{
public:
	CircularBuffer() {}; // default constructor
	CircularBuffer(int buffersize1, string name, int pixelformat) {
		if ((buffersize1 < 1) || (buffersize1 > 100000))// check parameters
		{
			cout << "Error with buffersize of " << name;
			Sleep(300000); // how to return from constructor to stop bad init?
		}
		// initialize class variables
		buffersize = buffersize1;
		bufferend = buffersize;
		current_frame = bufferstart;
		previous_frame = bufferend - 1;
		next_frame = bufferstart + 1;
		buffer = new type[buffersize];//(RESOLUTION, CV_8UC3);
		switch (pixelformat) {
		case 0:
			for (int i = 0; i < buffersize; i++)
				buffer[i] = Mat(RESOLUTION, CV_8UC3);
			break;
		case 1:
			for (int i = 0; i < buffersize; i++)
				buffer[i] = Mat(RESOLUTION, CV_8UC1);
			break;
		case 2:
			for (int i = 0; i < buffersize; i++)
				buffer[i] = Mat(RESOLUTION, CV_8UC1);
				//buffer[i] = Mat(Size(100,100), CV_8UC1);
			break;
		}
		buffername = name;
	};
	~CircularBuffer() {
		delete buffer;
	};

	type current()
	{
		current_ptr = &buffer[current_frame];
		return *current_ptr;
	}
	type previous()
	{
		previous_ptr = &buffer[previous_frame];
		return *previous_ptr;
	}
	type next()
	{
		next_ptr = &buffer[next_frame];
		return *next_ptr;
	}
	type store()
	{
		circulate(previous_frame);
		circulate(current_frame);
		circulate(next_frame);
		return current(); 
	}
	type backspace()
	{
		decirculate(previous_frame);
		decirculate(current_frame);
		decirculate(next_frame);
		return current();
	}
	void set(type &source)
	{
		buffer[current_frame] = source;
	}
	void copythis(type &copy)
	{
		buffer[current_frame].copyTo(copy);
	}
	void diffthis(type &diff)
	{
		absdiff(buffer[current_frame], buffer[previous_frame], diff);
	}

protected:
	type* buffer;
	type* current_ptr;
	type* previous_ptr;
	type* next_ptr;
private:
	int buffersize = 1;
	int previous_frame = 0;
	int current_frame = 0;
	int next_frame = 0;
	int bufferstart = 0;
	int bufferend = buffersize;
	int source = 0;
	string buffername = "Beef";
	void circulate(int &a)
	{
		a = a + 1 == bufferend ? bufferstart : a + 1;
	}
	void decirculate(int &a)
	{
		a = a - 1 == bufferstart - 1? bufferend - 1: a - 1;
	}

};

class ImageSource
{
public:
	CircularBuffer<Mat>* RGB_Buffer;
	CircularBuffer<Mat>* Gray_Buffer;
	CircularBuffer<Mat>* Disp_Buffer;
	VideoCapture cam;
	ImageSource() {}; // default constructor
	ImageSource(int capid, string config)
	{
		RGB_Buffer = new CircularBuffer<Mat>(50, "RGB_Buffer", 0); // 50 = buffersize, 0 = RGB
		Gray_Buffer = new CircularBuffer<Mat>(50, "Gray_Buffer", 1); // 50 = buffersize, 1 = grayscale
		Disp_Buffer = new CircularBuffer<Mat>(50, "Disparity_Buffer", 2); // 50 = buffersize, 2 = resized grayscale
		cam.open(capid);
		if (!cam.isOpened()) {
			cout << "ERROR ACQUIRING VIDEO FEED for id " + to_string(capid) + " \n";
			getchar();
			//error; 
		}
		cam.set(CV_CAP_PROP_FRAME_WIDTH, RESOLUTION.width); // 1280 for intergrated webcam, 1080 for external webcam
		cam.set(CV_CAP_PROP_FRAME_HEIGHT, RESOLUTION.height);
		cam.set(CV_CAP_PROP_FPS, 30); // changes property but not camera fps 
									  ////capture.set(CV_CAP_PROP_GAIN, 5);
									  ////capture.set(CV_CAP_PROP_EXPOSURE, 5);
									  //	//capture.set(CV_CAP_PROP_EXPOSURE, 99); // does nothing
									  //capture.set(CV_CAP_PROP_GAIN, 5);
									  //capture.set(CV_CAP_PROP_EXPOSURE, -1);
	}
	ImageSource(string filepath, string config)
	{
		RGB_Buffer = new CircularBuffer<Mat>(50, "RGB_Buffer", 0); // 50 = buffersize, 0 = RGB
		Gray_Buffer = new CircularBuffer<Mat>(50, "Gray_Buffer", 1); // 50 = buffersize, 1 = grayscale
		cam.open(filepath);
		if (!cam.isOpened()) {
			cout << "ERROR ACQUIRING VIDEO FEED for id " + filepath + " \n";
			getchar();
			//error; 
		}
		cam.set(CV_CAP_PROP_FRAME_WIDTH, RESOLUTION.width); // 1280 for intergrated webcam, 1080 for external webcam
		cam.set(CV_CAP_PROP_FRAME_HEIGHT, RESOLUTION.height);
		cam.set(CV_CAP_PROP_FPS, 30); // changes property but not camera fps 
									  ////capture.set(CV_CAP_PROP_GAIN, 5);
									  ////capture.set(CV_CAP_PROP_EXPOSURE, 5);
									  //	//capture.set(CV_CAP_PROP_EXPOSURE, 99); // does nothing
									  //capture.set(CV_CAP_PROP_GAIN, 5);
									  //capture.set(CV_CAP_PROP_EXPOSURE, -1);
	}

	~ImageSource()
	{
		RGB_Buffer->~CircularBuffer();
	}
	Mat background_image;

};
class ImagePipeline
{
public:
	int(*detect)(Mat, Mat ,ballLoc&);
	int(*track)(Mat&, Mat &, ballLoc&)
	{
	};
	int(*project)(Mat, Mat, Mat&)
	{
	};

	objectTracker objTracker;
	ImageSource* sources[5]; // camera limit?
	int numofcams;
	//camera constructor, stereo included
	ImagePipeline(int numcams, int capid[] = NULL, string* config = NULL, int(*detection)(Mat, Mat, ballLoc&) = NULL, int(*tracking)(Mat &SearchImage, Mat &movingobjects, ballLoc &ballspotted) = NULL, int(*projection)(Mat, Mat, Mat&) = NULL)
	{   // parameter check
		if (numofcams < 1 || capid == NULL || config == NULL)
		{
			//error
		}
		numofcams = numcams;
		for (int a = 0; a < numofcams; ++a)
		{
			sources[a] = new ImageSource(capid[a], "");
		}
		detect = detection; // set object detection function
		track = tracking; // set object tracking function
		project = projection;
	};
	ImagePipeline(int numcams, string* filepaths, string* config = NULL, int(*detection)(Mat, Mat, ballLoc&) = NULL, int(*tracking)(Mat &SearchImage, Mat &movingobjects, ballLoc &ballspotted) = NULL, int(*projection)(Mat, Mat, Mat&) = NULL)
	{   // parameter check
		if (numofcams < 1 || filepaths == NULL || config == NULL)
		{
			//error
		}
		numofcams = numcams;
		for (int a = 0; a < numofcams; ++a)
		{
			sources[a] = new ImageSource(*filepaths, "");
		}
		detect = detection; // set object detection function
		track = tracking; // set object tracking function
		project = projection;
	};
	~ImagePipeline()
	{
		for (int a = 0; a < numofcams; a++)
			sources[a]->~ImageSource();
	};
	void RGBtoGray()
	{
		for (int a = 0; a < numofcams; a++)
		{
			//Mat blurImage;
			//cvtColor(sources[a]->RGB_Buffer->current(), blurImage, COLOR_BGR2GRAY);
			//blur(blurImage, sources[a]->Gray_Buffer->store(), Size(BLUR_SIZE, BLUR_SIZE));
			cvtColor(sources[a]->RGB_Buffer->current(), sources[a]->Gray_Buffer->store(), COLOR_BGR2GRAY);
		}
	}

	int cap()
	{	//  minimal software sychronization
		for (int a = 0; a < numofcams; a++)
		{ // capture frame first
			sources[a]->cam.grab();
		}
		for (int a = 0; a < numofcams; a++)
		{ // decode frame second
			sources[a]->cam.retrieve(sources[a]->RGB_Buffer->store());
		}
		return 0;
	}

	int detection()
	{
		int ballfoundcount = 0;
		for (int a = 0; a < numofcams; a++)
		{
			ballfoundcount += detect(sources[a]->Gray_Buffer->current(), sources[a]->background_image, objTracker.trackerList[a][objTracker.ballcount].ball_location); // check left, if no ball found
		}
		if (ballfoundcount >= numofcams)
		{
			tracking();
			return 1; //ball found in every camera, start tracking
		}
		else
			return 0; // can't find ball in both cameras
	}
	int tracking()
	{
		objTracker.track(0); // assume the cameras share the same timestamps for now
		return 0;
	}


protected:

private:

};

void on_mouse(int e, int x, int y, int d, void *ptr)
{
	if (e != CV_EVENT_LBUTTONDOWN)
		return;
	Point*p = (Point*)ptr;
	p->x = x;
	p->y = y;
}

Mat Window(Point testpoint, Mat &testimage, Size windowsize)
{ // assumes same pixel format of input as output
  //Range check
  /*	int rowwindow = windowsize.height / 2;
  int colwindow = windowsize.width / 2;
  int lowerboundrow = testpoint.x - rowwindow < 0 ? 1 : testpoint.x - rowwindow;
  int upperboundrow = testpoint.x + rowwindow > testimage.row ? (testimage.rows) : testpoint.x + rowwindow; // WEIRD REVERSED LOGIC FIX
  int lowerboundcolumn = testpoint.y - colwindow < 0 ? 1 : testpoint.y - colwindow;
  int upperboundcolumn = testpoint.y + colwindow > testimage.cols ? (testimage.cols - 1) : testpoint.y + colwindow;// WEIRD REVERSED LOGIC FIX
  Mat window = testimage(Range(lowerboundrow, upperboundrow), Range(lowerboundcolumn, upperboundcolumn));
  return window;
  */
	Mat nothing;
	return nothing;
}

void drawObject(int x, int y, Mat &frame) {
	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25<frame.rows)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, frame.rows), Scalar(0, 255, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25<frame.cols)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(frame.cols, y), Scalar(0, 255, 0), 2);

	putText(frame, to_string(x) + "," + to_string(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
}
Mat Window(Rect rect, Mat &testimage)
{
	if (rect.width + rect.x > testimage.cols)
		rect.width = testimage.cols - rect.x;
	if (rect.height + rect.y > testimage.rows)
		rect.height = testimage.rows - rect.y;
	Mat window = testimage(rect);
	return window;
}

int dp = 1; /*The inverse ratio of resolution*/	const int dp_MAX = 10;
int min_dist = 20; /*Minimum distance between detected centers*/const int min_dist_MAX = 200;
int param_1 = 200; /*Upper threshold for the internal Canny edge detector */const int param_1_MAX = 400;
int param_2 = 100; /* Threshold for center detection.*/	const int param_2_MAX = 200;
int min_radius = 0; /* Minimum radio to be detected.If unknown, put zero as default. */	const int min_radius_MAX = 40;
int max_radius = 0; /*Maximum radius to be detected.If unknown, put zero as default */	const int max_radius_MAX = 100;

int minThreshold = 133;
int maxThreshold = 230;
int filterByArea = true;
int minArea = 10;
int maxArea = 50000;
int filterByCircularity = true;
int minCircularity = 1;
int maxCircularity = 10;
int filterByConvexity = true;
int minConvexity = 7;
int maxConvexity = 10;
int filterByInertia = true;
int minInertiaRatio = 1;
int maxInertiaRatio = 100;
const string trackbarWindowName2 = "BlobTrackbars";
Ptr<SimpleBlobDetector> detector;

void Blob_update()
{
	//
	SimpleBlobDetector::Params params;
	int itemp = 0;
	float ftemp = 0.0;
	bool btemp = true;
	// Change thresholds
	params.minThreshold = getTrackbarPos("minThreshold", trackbarWindowName2);
	params.maxThreshold = getTrackbarPos("maxThreshold", trackbarWindowName2);

	// Filter by Area.
	btemp = getTrackbarPos("filterByArea", trackbarWindowName2) == 1 ? true : false;
	params.filterByArea = btemp;
	params.minArea = getTrackbarPos("minArea", trackbarWindowName2);
	params.maxArea = getTrackbarPos("maxArea", trackbarWindowName2);

	// Filter by Circularity
	btemp = getTrackbarPos("filterByCircularity", trackbarWindowName2) == 1 ? true : false;
	params.filterByCircularity = btemp;
	ftemp = (float)getTrackbarPos("minCircularity", trackbarWindowName2) / 10.0;
	params.minCircularity = ftemp;
	ftemp = (float)getTrackbarPos("maxCircularity", trackbarWindowName2) / 10.0;
	params.maxCircularity = ftemp;

	// Filter by Convexity
	btemp = getTrackbarPos("filterByConvexity", trackbarWindowName2) == 1 ? true : false;
	params.filterByConvexity = btemp;
	ftemp = (float)getTrackbarPos("minConvexity", trackbarWindowName2) / 10.0;
	params.minConvexity = ftemp;
	ftemp = (float)getTrackbarPos("maxConvexity", trackbarWindowName2) / 10.0;
	params.maxConvexity = ftemp;

	// Filter by Inertia
	btemp = getTrackbarPos("filterByInertia", trackbarWindowName2) == 1 ? true : false;
	params.filterByInertia = btemp;
	ftemp = (float)getTrackbarPos("minInertiaRatio", trackbarWindowName2) / 100.0;
	params.minInertiaRatio = ftemp;
	params.filterByColor = false;

	detector = SimpleBlobDetector::create(params);
}


const int minThreshold_MAX = 254;
const int maxThreshold_MAX = 255;
const int filterByArea_MAX = 1;
const int minArea_MAX = 300;
const int maxArea_MAX = 80000;
const int filterByCircularity_MAX = 1;
const int minCircularity_MAX = 9;
const int maxCircularity_MAX = 10;
const int filterByConvexity_MAX = 1;
const int minConvexity_MAX = 9;
const int maxConvexity_MAX = 10;
const int filterByInertia_MAX = 1;
const int minInertiaRatio_MAX = 99;


void on_trackbar2(int, void*)
{
	Blob_update();
};//This function gets called whenever a trackbar position is changed
void createTrackbars_Blob() { //Create window for trackbars
	char TrackbarName[200];

	// Create TrackBars Window
	namedWindow("BlobTrackbars", 0);

	// Create memory to store Trackbar name on window
	sprintf(TrackbarName, "minThreshold");
	sprintf(TrackbarName, "maxThreshold");
	sprintf(TrackbarName, "filterByArea");
	sprintf(TrackbarName, "minArea");
	sprintf(TrackbarName, "maxArea");
	sprintf(TrackbarName, "filterByCircularity");
	sprintf(TrackbarName, "minCircularity");
	sprintf(TrackbarName, "maxCircularity");
	sprintf(TrackbarName, "filterByConvexity");
	sprintf(TrackbarName, "minConvexity");
	sprintf(TrackbarName, "maxConvexity");
	sprintf(TrackbarName, "filterByInertia");
	sprintf(TrackbarName, "minInertiaRatio");

	////Create Trackbars and insert them into window
	createTrackbar("minThreshold", trackbarWindowName2, &minThreshold, minThreshold_MAX, on_trackbar2);
	createTrackbar("maxThreshold", trackbarWindowName2, &maxThreshold, maxThreshold_MAX, on_trackbar2);
	createTrackbar("filterByArea", trackbarWindowName2, &filterByArea, filterByArea_MAX, on_trackbar2);
	createTrackbar("minArea", trackbarWindowName2, &minArea, minArea_MAX, on_trackbar2);
	createTrackbar("maxArea", trackbarWindowName2, &maxArea, maxArea_MAX, on_trackbar2);
	createTrackbar("filterByCircularity", trackbarWindowName2, &filterByCircularity, filterByCircularity_MAX, on_trackbar2);
	createTrackbar("minCircularity", trackbarWindowName2, &minCircularity, minCircularity_MAX, on_trackbar2);
	createTrackbar("maxCircularity", trackbarWindowName2, &maxCircularity, maxCircularity_MAX, on_trackbar2);
	createTrackbar("filterByConvexity", trackbarWindowName2, &filterByConvexity, filterByConvexity_MAX, on_trackbar2);
	createTrackbar("minConvexity", trackbarWindowName2, &minConvexity, minConvexity_MAX, on_trackbar2);
	createTrackbar("maxConvexity", trackbarWindowName2, &maxConvexity, maxConvexity_MAX, on_trackbar2);
	createTrackbar("filterByInertia", trackbarWindowName2, &filterByInertia, filterByInertia_MAX, on_trackbar2);
	createTrackbar("minInertiaRatio", trackbarWindowName2, &minInertiaRatio, minInertiaRatio, on_trackbar2);
}
