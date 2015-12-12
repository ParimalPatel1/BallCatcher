#pragma once
#include  <opencv2\opencv.hpp>
#include <Windows.h>
#include <ctime>
#include "ProjectionMethods.hpp"
#include "Circle.hpp"
#include "circledetect.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include <process.h>
#include "MotionCapture.h"


//using namespace cv;
//using namespace ipp; // how to use?
//using namespace std;
#define RESOLUTION cv::Size(640,480)
const static int SENSITIVITY_VALUE = 30;
//size of blur used to smooth the intensity image output from absdiff() function
const static int BLUR_SIZE = 10;

cv::Point textcenter1(100, 50); // text variables start
cv::Point textcenter2(100, 100);
cv::Point textcenter3(100, 150);
int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
double fontScale = 1;
int thickness = 2; // text variables end


void drawObject(int x, int y, int r, cv::Mat &frame) {
	//circle(frame, cv::Point(x, y), 20, cv::Scalar(0, 255, 0), 2);
	//if (y - 25>0)
	//	line(frame, cv::Point(x, y), cv::Point(x, y - 25), cv::Scalar(0, 255, 0), 2);
	//else line(frame, cv::Point(x, y), cv::Point(x, 0), cv::Scalar(0, 255, 0), 2);
	//if (y + 25<RESOLUTION.height)
	//	line(frame, cv::Point(x, y), cv::Point(x, y + 25), cv::Scalar(0, 255, 0), 2);
	//else line(frame, cv::Point(x, y), cv::Point(x, RESOLUTION.height), cv::Scalar(0, 255, 0), 2);
	//if (x - 25>0)
	//	line(frame, cv::Point(x, y), cv::Point(x - 25, y), cv::Scalar(0, 255, 0), 2);
	//else line(frame, cv::Point(x, y), cv::Point(0, y), cv::Scalar(0, 255, 0), 2);
	//if (x + 25<RESOLUTION.width)
	//	line(frame, cv::Point(x, y), cv::Point(x + 25, y), cv::Scalar(0, 255, 0), 2);
	//else line(frame, cv::Point(x, y), cv::Point(RESOLUTION.width, y), cv::Scalar(0, 255, 0), 2);

	//putText(frame, std::to_string(x) + "," + std::to_string(y), cv::Point(x, y + 30), 1, 1, cv::Scalar(0, 255, 0), 2);
	circle(frame, cv::Point(x, y), r, cv::Scalar(255, 0, 0), 1);
}
struct ballLoc
{
	cv::Point2f center;
	float radius;
};
class objectTracker
{
public:
	//int trackedObjects; single tracking for now
//	int ballcount;
	LARGE_INTEGER Frequency;
	objectTracker()
	{
		//trackedObjects = 0;
		//ballcount = 0;
	}
	struct timestamp_t
	{
		LARGE_INTEGER currentTime;
		LARGE_INTEGER ElapsedMicroseconds;  // in microseconds
	};
	struct ballTrack
	{
		ballLoc ball_location[50];
		timestamp_t timestamp[50];
		int ballcount = 0;
	};
	ballTrack trackerList[2]; 
	void track(int camera, ballLoc ballcoords) // adds an object to be timed. assumes its the same object for now.
	{
		int i = trackerList[camera].ballcount; // shitty implementation for now
		if (i >= 0)
		{
			QueryPerformanceCounter(&trackerList[camera].timestamp[i].currentTime);
			if (i == 0)
			{
				QueryPerformanceFrequency(&Frequency); // get frequency only once
				trackerList[camera].timestamp[i].ElapsedMicroseconds.QuadPart = 1;
			}
			else if (i > 0)
			{
				trackerList[camera].timestamp[i].ElapsedMicroseconds.QuadPart = trackerList[camera].timestamp[i].currentTime.QuadPart - trackerList[camera].timestamp[i-1].currentTime.QuadPart;
				trackerList[camera].timestamp[i].ElapsedMicroseconds.QuadPart *= 1000000;
				trackerList[camera].timestamp[i].ElapsedMicroseconds.QuadPart /= Frequency.QuadPart;
			}
			trackerList[camera].ball_location[i] = ballcoords;
			trackerList[camera].ballcount++;
		}
		else // ballcount < 0
			trackerList[camera].ballcount = 0; // reset


	}
	
	void fillBalls(std::vector<cv::Point2f> &balls, int leftorright)
	{

		for (int i = 0; i < trackerList[leftorright].ballcount; i++)
		{
			cv::Point2f temppoint(trackerList[leftorright].ball_location[i].center.x, trackerList[leftorright].ball_location[i].center.y);
			balls.push_back(temppoint);
		}
			
	}

	void listballs()
	{
		std::cout << "Left balls " << std::endl;
		for (int i = 0; i < trackerList[0].ballcount; i++)
		{
			std::cout << "Pos: " << trackerList[0].ball_location[i].center.x << " " << trackerList[0].ball_location[i].center.y;
			std::cout << " " << trackerList[0].ball_location[i].radius << "Time: " << trackerList[0].timestamp << std::endl;
		}
		std::cout << "right balls " << std::endl;
		for (int i = 0; i < trackerList[1].ballcount; i++)
		{
			std::cout << "Pos: " << trackerList[1].ball_location[i].center.x << " " << trackerList[1].ball_location[i].center.y;
			std::cout << " " << trackerList[1].ball_location[i].radius << "Time: " << trackerList[1].timestamp << std::endl;
		}

	}
};

template <class type>
class CircularBuffer
{
public:
	CircularBuffer() {}; // default constructor
	CircularBuffer(int buffersize1, std::string name, int pixelformat) {
		if ((buffersize1 < 1) || (buffersize1 > 100000))// check parameters
		{
			cout << "Error with buffersize of " << name;
			Sleep(300000); // how to return from constructor to stop bad init?
		}
		// initialize class variables
		buffersize = buffersize1;
		bufferend = buffersize;
		Rcurrent_frame = bufferstart;
		Rprevious_frame = bufferend - 1;
		Wcurrent_frame = bufferstart;
		Wprevious_frame = bufferend - 1;
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
			break;
		}
		buffername = name;
	};
	~CircularBuffer() {
		delete buffer;
	};


	type read()
	{
		read_ptr = &buffer[readspot];
		circulate(Rprevious_frame);
		circulate(Rcurrent_frame);
		circulate(readspot);
		return *read_ptr;
	}
	type write() // always increment, add write protection
	{
		write_ptr = &buffer[storespot];
		circulate(Wprevious_frame);
		circulate(Wcurrent_frame);
		circulate(storespot);
		return *write_ptr;
	}
	type Wprevious()
	{
		Wprevious_ptr = &buffer[Wprevious_frame];
		return *Wprevious_ptr;
	}
	type Wcurrent()
	{
		Wcurrent_ptr = &buffer[Wcurrent_frame];
		return *Wcurrent_ptr;
	}
	type Rprevious()
	{
		Rprevious_ptr = &buffer[Rprevious_frame];
		return *Rprevious_ptr;
	}
	type Rcurrent()
	{
		Rcurrent_ptr = &buffer[Rcurrent_frame];
		return *Rcurrent_ptr;
	}

	int writespots(void)
	{
		return (readspot - storespot) <= 0 ? (readspot - storespot + buffersize - 1) : (readspot - storespot);
	}
	int readspots(void)
	{
		return (storespot - readspot) >= 0 ? (storespot - readspot) : (storespot - readspot + buffersize - 1);
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

private:
	type* buffer;
	type* Wcurrent_ptr;
	type* Wprevious_ptr;
	type* Rcurrent_ptr;
	type* Rprevious_ptr;
	type* write_ptr;
	type* read_ptr;
	int buffersize = 1;
	int Wprevious_frame = 0;
	int Wcurrent_frame = 0;
	int Rprevious_frame = 0;
	int Rcurrent_frame = 0;
	int bufferstart = 0;
	int bufferend = buffersize;
	int source = 0;
	std::string buffername = "Beef";
	volatile int readspot= 0;
	volatile int storespot = 0;

	void circulate(volatile int &a)
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
	CircularBuffer<cv::Mat>* RGB_Buffer;
	CircularBuffer<cv::Mat>* Gray_Buffer;
	CircularBuffer<cv::Mat>* HSV_Buffer;
	//CircularBuffer<cv::Mat>* Disp_Buffer;
	cv::VideoCapture cam;
	ImageSource() {}; // default constructor
	ImageSource(int capid, std::string config)
	{
		RGB_Buffer = new CircularBuffer<cv::Mat>(30, "RGB_Buffer", 0); // 50 = buffersize, 0 = RGB
		Gray_Buffer = new CircularBuffer<cv::Mat>(30, "Gray_Buffer", 1); // 50 = buffersize, 1 = grayscale
		//Disp_Buffer = new CircularBuffer<cv::Mat>(30, "Disparity_Buffer", 2); // 50 = buffersize, 2 = resized grayscale
		HSV_Buffer = new CircularBuffer<cv::Mat>(30, "HSV_Buffer", 0); // 50 = buffersize, 2 = resized grayscale
		cam.open(capid);
		if (!cam.isOpened()) {
			std::cout << "ERROR ACQUIRING VIDEO FEED for id " + std::to_string(capid) + " \n";
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
	ImageSource(std::string filepath, std::string config)
	{
		RGB_Buffer = new CircularBuffer<cv::Mat>(50, "RGB_Buffer", 0); // 50 = buffersize, 0 = RGB
		Gray_Buffer = new CircularBuffer<cv::Mat>(50, "Gray_Buffer", 1); // 50 = buffersize, 1 = grayscale
		cam.open(filepath);
		if (!cam.isOpened()) {
			std::cout << "ERROR ACQUIRING VIDEO FEED for id " + filepath + " \n";
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
	cv::Mat background_image;
};
class ImagePipeline
{
public:
	// function pointers
	ballLoc(*detect)(cv::Mat&, cv::Mat&, cv::Mat&);//int(*detect)(cv::Mat, cv::Mat, ballLoc&);
	ballLoc(*track)(cv::Mat&, cv::Mat&, cv::Mat&);
	int(*project)(void);//(cv::Mat, cv::Mat, cv::Mat&);

	objectTracker objTracker;
	ImageSource* sources[5]; // camera limit?
	int numofcams;
	Stereotime stereoset;// intergrate with pipeline

	//camera constructor, stereo included
	ImagePipeline(int numcams, int capid[] = NULL, std::string* config = NULL, ballLoc(*detection)(cv::Mat&, cv::Mat&, cv::Mat&) = NULL, ballLoc(*tracking)(cv::Mat&, cv::Mat&, cv::Mat&) = NULL, int(*projection)(void) = NULL)
	{   // parameter check
		stereoset.Stereo_Init(RESOLUTION);
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
	ImagePipeline(int numcams, std::string* filepaths, std::string* config = NULL, ballLoc(*detection)(cv::Mat&, cv::Mat&, cv::Mat&) = NULL, ballLoc(*tracking)(cv::Mat&, cv::Mat&, cv::Mat&) = NULL, int(*projection)(void) = NULL)
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
			cvtColor(sources[a]->RGB_Buffer->Wcurrent(), sources[a]->Gray_Buffer->write(), cv::COLOR_BGR2GRAY);
		}
	}

	void RGBtoHSV()
	{
		for (int a = 0; a < numofcams; a++)
		{
			cvtColor(sources[a]->RGB_Buffer->Wcurrent(), sources[a]->HSV_Buffer->write(), cv::COLOR_BGR2HSV);
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
			sources[a]->cam.retrieve(sources[a]->RGB_Buffer->write());
		}
		return 0;
	}

	int detection()
	{
		ballLoc ballfound;
		int ballcount = 0;
		for (int a = 0; a < numofcams; a++)
		{
			ballfound = detect(sources[a]->HSV_Buffer->read(), sources[a]->Gray_Buffer->Rprevious(), sources[a]->Gray_Buffer->read());
			if (ballfound.radius > 0) // ball was found, add it to the list
			{
				ballcount++;
				//drawObject(ballfound.center.x, ballfound.center.y, ballfound.radius, sources[a]->RGB_Buffer->current());
			}
			if (ballcount == 2) // only track if ball detected in both frames
				objTracker.track(a, ballfound);
			//sources[a]->Gray_Buffer->current(), sources[a]->background_image, objTracker.trackerList[a][objTracker.ballcount].ball_location); // check left, if no ball found
		}
		if (objTracker.trackerList[0].ballcount > 2 && objTracker.trackerList[1].ballcount > 2)
		{
			// woohoo! time to track the ball!
			return 1;
		}
		else
		return 0;
	}
	int tracking()
	{
		//objTracker.track(0); // assume the cameras share the same timestamps for now
		return 0;
	}
	int projectBall()
	{
		// convert ballLoc to vector point2f
		std::vector<cv::Point2f> leftpoints;
		std::vector<cv::Point2f> rightpoints;
		objTracker.fillBalls(leftpoints, 0);
		objTracker.fillBalls(rightpoints, 1);

		std::vector<cv::Point3f> disparitypoints =  stereoset.calcDisparityPoints(leftpoints, rightpoints);
		std::vector<cv::Point3f> inches = stereoset.projectTheBall(disparitypoints);

		// convert to 3D now
		Point3D point1(inches[0].x, inches[0].y, inches[0].z, (double)objTracker.trackerList[0].timestamp[0].currentTime.QuadPart);
		Point3D point2(inches[1].x, inches[1].y, inches[1].z, (double)objTracker.trackerList[0].timestamp[1].currentTime.QuadPart);

		std::vector<Point3D> points;
		Point3D BLS = calculateObjectPosition(points, point1, point2); // ball landing spot
		std::cout << "X: " << BLS.x << " " << "Y: " << BLS.y << " " << "Z: " << BLS.z << std::endl;
		return 1;
	}

protected:

private:

};

void on_mouse(int e, int x, int y, int d, void *ptr)
{
	if (e != CV_EVENT_LBUTTONDOWN)
		return;
	cv::Point*p = (cv::Point*)ptr;
	p->x = x;
	p->y = y;
}

cv::Mat Window(cv::Point testpoint, cv::Mat &testimage, cv::Size windowsize)
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
	cv::Mat nothing;
	return nothing;
}

cv::Mat Window(cv::Rect rect, cv::Mat &testimage)
{
	if (rect.width + rect.x > testimage.cols)
		rect.width = testimage.cols - rect.x;
	if (rect.height + rect.y > testimage.rows)
		rect.height = testimage.rows - rect.y;
	cv::Mat window = testimage(rect);
	return window;
}
