//motionTracking.cpp

//Written by  Kyle Hounslow, December 2013

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.

#include <opencv\cv.h>
#include <opencv\highgui.h>
#include  <opencv2\opencv.hpp>
//#include  <opencv2\videoio.hpp>
#include <Windows.h>
#include <ctime>


using namespace cv;
using namespace std;
//#define SECONDCAMERA
#define OUTPUTCAP
// START GLOBALS
//our sensitivity value to be used in the absdiff() function
const static int SENSITIVITY_VALUE = 20;
//size of blur used to smooth the intensity image output from absdiff() function
const static int BLUR_SIZE = 10;
//we'll have just one object to search for
//and keep track of its position.
int trackedObject[2] = { 0,0 };
//bounding rectangle of the object, we will use the center of this as its position.

vector<int> compression_params;
int imagecount = 0;
int imagecount2 = 0;
int framecount = 0;
#define RESOLUTION Size(640,480)
#define BufferSize 2
Point pointlist[10000];
Point pointlist2[10000];
int airtime_clock = 0;
int airtime_clock2 = 0;
bool trackingEnabled = false;

Mat frame; //current frame
Mat fgMaskMOG2; //fg mask fg mask generated by MOG2 method
Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
int keyboard; //input from keyboard
// END GLOBALS
template <class type>
class CircularBuffer
{
public:
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
		}


		buffername = name;
	};
	~CircularBuffer() {
		delete buffer;
	};

	type* buffer;
	type* current_ptr;
	type* previous_ptr;
	type* next_ptr;
/*	int initialize(type source) // initialize the 1st element of the buffer to get things rolling
	{
		buffer[bufferstart] = source;
		if (buffersize > 1)
		{
			buffer
		}
	} */

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
		return current(); //current is next, so store there. PROBLEM
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
		absdiff(buffer[current_frame], buffer[previous_frame],diff);
	}

protected:
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

};

static inline Point calcPoint(Point2f center, double R, double angle)
{
	return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
}
template <class type>
class ImagePipeline
{
public:
	CircularBuffer<type> a;
protected:

private:

};

Mat Window(Point testpoint, Mat &testimage, Size windowsize)
{ // assumes same pixel format of input as output
	//Range check
	int rowwindow = windowsize.height / 2;
	int colwindow = windowsize.width / 2;
	int lowerboundrow = testpoint.x - rowwindow < 0 ? 0 : testpoint.x - rowwindow;
	int upperboundrow = testpoint.x + rowwindow > testimage.rows ? testimage.rows : testpoint.x + rowwindow;
	int lowerboundcolumn = testpoint.y - colwindow < 0 ? 0 : testpoint.y - colwindow;
	int upperboundcolumn = testpoint.y + colwindow < testimage.cols ? testimage.cols : testpoint.y + colwindow;
	Mat window = testimage(Range(lowerboundrow, upperboundrow), Range(lowerboundcolumn, upperboundcolumn));
	return window;
}
void on_mouse(int e, int x, int y, int d, void *ptr)
{
	if (e != CV_EVENT_LBUTTONDOWN)
		return;
	Point*p = (Point*)ptr;
	p->x = x;
	p->y = y;
}
//int to string helper function
string intToString(int number) {

	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}
 // finds moving objecs in an image
int searchForMovement(Mat &thresholdImage, Mat &movingobjects, int camera = 1) {
	bool objectDetected = false;
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	//imwrite("thresh" + intToString(framecount) + ".png", thresholdImage);
	findContours(thresholdImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours, CHANGES THRESHOLD IMAGE
//	HoughCircles()

	//if contours vector is not empty, we have found some objects
	if (contours.size() > 0)
	{
	//	imwrite("contours" + intToString(framecount) + ".png", thresholdImage);
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVec;
		float circularratio;
		for (int i = contours.size() - 1; i == 0; i--)
		{ 
			//	circlecheck(Point(contours.at(i).at(i).x, contours.at(i).at(i).y));
			//largestContourVec.push_back(contours.at(contours.size() - 1));
			Mat rects;
			thresholdImage.copyTo(rects);
			objectBoundingRectangle = boundingRect(contours.at(i));//largestContourVec.at(0));
			rectangle(rects, objectBoundingRectangle, Scalar(255, 255, 255));
		//	imshow("rect", rects);
			if (objectBoundingRectangle.area() > 20 && objectBoundingRectangle.area() < 500000) // first size check
			{
				rectangle(movingobjects, objectBoundingRectangle, Scalar(255, 255, 255));
			//	imwrite("rectangle" + intToString(framecount) + ".png", movingobjects);
				circularratio = (float)objectBoundingRectangle.width / objectBoundingRectangle.height;

				float threshold = 3; 
			//	if (circularratio > (float)1/threshold || circularratio < (float)1*threshold)// (circularratio > 1 - threshold || circularratio < 1 + threshold)  //then roundness check
			//	{ // we think this a moving circle now
				int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
				int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;
				circle(movingobjects, Point(xpos, ypos), 20, Scalar(255, 255, 255), 2);
				if (camera == 1)
				{
					pointlist[imagecount] = Point(xpos, ypos); 
					if (imagecount > 0)
					{
						line(movingobjects, pointlist[imagecount - 1], pointlist[imagecount], Scalar(255, 255, 255), 2);
					}
					imagecount++;
				}
				if (camera == 2)
				{
					pointlist2[imagecount2] = Point(xpos, ypos);
					if (imagecount > 0)
					{
						line(movingobjects, pointlist2[imagecount2 - 1], pointlist2[imagecount2], Scalar(255, 255, 255), 2);
					}
					imagecount2++;
				}
			//	}
			}

		}
	//	largestContourVec.push_back(contours.at(contours.size() - 1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
	//	objectBoundingRectangle = boundingRect(largestContourVec.at(0));
		//if (objectBoundingRectangle.area() > 50 && objectBoundingRectangle.area() < 1000) // RESOLUTION DEPENDANT
		//{
		//	rectangle(movingobjects, objectBoundingRectangle, Scalar(255, 255, 0));
		//}

		//int circularratio = objectBoundingRectangle.width / objectBoundingRectangle.height;
		//if (objectBoundingRectangle.area() > 30) // filter out more noise between frames // RESOLUTION DEPENDANT 
		//{
		//	int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
		//	int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;
		//	
		//	if (camera == 1)
		//		{
		//			pointlist[imagecount] = Point(xpos, ypos); //.at(imagecount,imagecount);
		//			circle(movingobjects, pointlist[imagecount], 20, Scalar(255, 255, 255), 2);
		//		/*	if (imagecount == 0)
		//			{
		//				airtime_clock = clock(); // start timer for first point
		//				putText(movingobjects, "Time(ms): 0", pointlist[imagecount], 1, 1, Scalar(255, 0, 0), 2);
		//			}
		//			else
		//			{
		//				int timelaspe = clock() - airtime_clock;
		//				putText(movingobjects, "Time(ms): " + to_string(timelaspe), pointlist[imagecount], 1, 1, Scalar(255, 0, 0), 2);
		//			}
		//			*/
		//			//putText(TrackerImage, "Tracking object at (" + intToString(xpos) + "," + intToString(ypos) + ")", pointlist[imagecount], 1, 1, Scalar(255, 0, 0), 2);
		//			if (imagecount > 0)
		//			{
		//				line(movingobjects, pointlist[imagecount - 1], pointlist[imagecount], Scalar(255, 255, 255), 2);
		//			}
		//			imagecount++;
		//		}
		//		if (camera == 2)
		//		{
		//			pointlist2[imagecount2] = Point(xpos, ypos); //.at(imagecount,imagecount);
		//			circle(movingobjects, pointlist2[imagecount2], 20, Scalar(0, 255, 0), 2);
		//		/*	if (imagecount2 == 0)
		//			{
		//				airtime_clock2 = clock(); // start timer for first point
		//				putText(movingobjects, "Time(ms): 0", pointlist2[imagecount2], 1, 1, Scalar(255, 0, 0), 2);
		//			}
		//			else
		//			{
		//				int timelaspe = clock() - airtime_clock2;
		//				putText(movingobjects, "Time(ms): " + to_string(timelaspe), pointlist2[imagecount2], 1, 1, Scalar(255, 0, 0), 2);
		//			}
		//			*/
		//			if (imagecount2 > 0)
		//			{
		//				line(movingobjects, pointlist2[imagecount2 - 1], pointlist2[imagecount2], Scalar(0, 255, 0), 2);
		//			}
		//			imagecount2++;
			}
			return 1;
}

int searchForCircles(Mat &thresholdImage, Mat &movingobjects, int camera = 1) {
	bool objectDetected = false;
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	findContours(thresholdImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours																					//if contours vector is not empty, we have found some objects
	if (contours.size() > 0)
	{
		/// Find the rotated rectangles and ellipses for each contour
		vector<RotatedRect> minRect(contours.size());
		vector<RotatedRect> minEllipse(contours.size());
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size() - 1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		objectBoundingRectangle = boundingRect(largestContourVec.at(0));

		for (int i = 0; i < contours.size(); i++) /////// MOVE ELSEWHERE
		{
			if (contours[i].size() > 20 && contours[i].size() < 300)
			{
				minEllipse[i] = fitEllipse(Mat(contours[i]));
				if ((((2 * minEllipse[i].size.height + 2 * minEllipse[i].size.width)* (2 * minEllipse[i].size.height + 2 * minEllipse[i].size.width))/(minEllipse[i].size.height * minEllipse[i].size.width)) > 0.25)
					ellipse(movingobjects, minEllipse[i], Scalar(255, 255, 0));
				//drawContours(movingobjects, contours, i, Scalar(40,100,55));
			}
		}
	}
	return 0;
}


//update the objects positions by changing the 'theObject' array values
/*	theObject[0] = xpos, theObject[1] = ypos;
int x = theObject[0];
int y = theObject[1];
//draw some crosshairs around the object

line(TrackerImage, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
line(TrackerImage, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
line(TrackerImage, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
line(TrackerImage, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
*/
//write the position of the object to the screen	
//imshow("TrackerImage", TrackerImage);
//imwrite("trackingH"+intToString(imagecount)+".png", cameraFeed, compression_params);

// INPUT: Grayimage, OUTPUT: Thresholdimage
// for detecting moving objects
int Algorithim1(Mat GraySource1, Mat GraySource2, Mat &thresholdImage) {
	Mat differenceImage;
	//perform frame differencing with the sequential images. 
	absdiff(GraySource1, GraySource2, differenceImage);
	//imshow("diff", differenceImage);
	//threshold intensity image at a given sensitivity value
	threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
	//blur the image to get rid of the noise. This will output an intensity image
	blur(thresholdImage, thresholdImage, Size(BLUR_SIZE, BLUR_SIZE));
	//threshold again to obtain binary image from blur output
	threshold(thresholdImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
	return 0;
}
//
int Algorithim2(Mat GraySource1, Mat &thresholdImage) {
	int thresh = 50;
	
	GaussianBlur(GraySource1, GraySource1, Size(9, 9), 2, 2);
	Canny(GraySource1, thresholdImage, thresh, thresh * 2, 3);
	return 0;
}

void MyCallbackForContrast(int iValueForContrast, void *userData)
{
	Mat dst;
	int iValueForBrightness = *(static_cast<int*>(userData));

	//Calculating brightness and contrast value
	int iBrightness = iValueForBrightness - 50;
	double dContrast = iValueForContrast / 50.0;

	//Calculated contrast and brightness value
	cout << "MyCallbackForContrast : Contrast=" << dContrast << ", Brightness=" << iBrightness << endl;

	//adjust the brightness and contrast
	//src.convertTo(dst, -1, dContrast, iBrightness);

	//show the brightness and contrast adjusted image
	imshow("My Window", dst);
}

int main() {
	// START DECLARATIONS
	//some boolean variables for added functionality
	bool objectDetected = false;
	//these two can be toggled by pressing 'd' or 't'
	bool debugMode = false;
	//pause and resume code
	bool pause = false;

	Mat differenceImage;
	Mat thresholdImage =  Mat(RESOLUTION, CV_8UC1);
	Mat thresholdImage2 =  Mat(RESOLUTION, CV_8UC1);
	Mat movingobjects = Mat(RESOLUTION, CV_8UC3);
	Mat movingobjects2 = Mat(RESOLUTION, CV_8UC3);
	Mat* contour_drawing; //=  new Mat Mat::create(RESOLUTION, CV_8UC3);

	VideoCapture capture;
	VideoCapture capture2;
	
	Point textcenter1(100, 50); // text variables start
	Point textcenter2(100, 100);
	Point textcenter3(100, 150);
	int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontScale = 1;
	int thickness = 2; // text variables end

	unsigned int current_time = 1;
	float start_time = clock(); // timer variable
	unsigned int last_time = clock();
	unsigned int framecounter = 0;
	int frame_start = 0;
	float fps;
	const string filepath = "test.avi"; //C:\Users\Benjamin\Desktop
	VideoWriter output_cap;
	const string videopath = "ballthrow2.avi";

	CircularBuffer<Mat> RGB_Buffer(50, "RGB_Buffer",0); // only 50 right now
	CircularBuffer<Mat> RGB_Buffer2(50, "RGB_Buffer2",0);
	CircularBuffer<Mat> Gray_Buffer(50, "Gray_Buffer",1);
	CircularBuffer<Mat> Gray_Buffer2(50, "Gray_Buffer2",1);

	// KALMANN
	Mat img(RESOLUTION, CV_8UC3);
	KalmanFilter KF(5, 1, 0);
	Mat state(5, 1, CV_32F); /* (phi, delta_phi) */
	Mat processNoise(5, 1, CV_32F);
	Mat measurement = Mat::zeros(1, 1, CV_32F);

//END DECLARATIONS

// START INIT
	// BLOB
	// Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 0;//10
	params.maxThreshold = 255;//20

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 10;
	params.maxArea = 50000;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.1;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.7;

	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.01;
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	// KALMANN
	randn(state, Scalar::all(0), Scalar::all(0.1));
	KF.transitionMatrix = (Mat_<float>(5, 5) << 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1);
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, Scalar::all(1));
	randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
	//
	//create Background Subtractor objects
	pMOG2 = createBackgroundSubtractorMOG2 (255, 16.0, false);//MOG2 approach
	//pMOG2 = createBackgroundSubtractorKNN(500, 400.0, false);
	//namedWindow("Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);
	capture.open(701); // 700 -> directshow + index, index 0 for laptop webcam, 1 for usb webcam usually
					   //	Sleep(500);
	//capture.open(videopath);
	if (!capture.isOpened()) {
		cout << "ERROR ACQUIRING VIDEO FEED\n";
		getchar();
		return -1;
	}
	//capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G')); // doesn't set
	capture.set(CV_CAP_PROP_FRAME_WIDTH, RESOLUTION.width); // 1280 for intergrated webcam, 1080 for external webcam
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, RESOLUTION.height);
	capture.set(CV_CAP_PROP_FPS, 30); // changes property but not camera fps 
	////capture.set(CV_CAP_PROP_GAIN, 5);
	////capture.set(CV_CAP_PROP_EXPOSURE, 5);
	//	//capture.set(CV_CAP_PROP_EXPOSURE, 99); // does nothing
	//capture.set(CV_CAP_PROP_GAIN, 5);
	//capture.set(CV_CAP_PROP_EXPOSURE, -1);

#ifdef SECONDCAMERA
	capture2.open(702); //"http://umd:umd@192.168.0.101/video.cgi?.mjpg"
	if (!capture2.isOpened()) {
		cout << "error acquiring video feed\n";
		getchar();
		return -1;
	}
	//capture2.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G')); // doesn't set
	capture2.set(CV_CAP_PROP_FRAME_WIDTH, RESOLUTION.width); // 1280 for intergrated webcam, 1080 for external webcam
	capture2.set(CV_CAP_PROP_FRAME_HEIGHT, RESOLUTION.height);
	capture2.set(CV_CAP_PROP_FPS, 30); // changes property but not camera fps 

#endif

	// First initialize to get at least one previous frame
	while (!capture.read(RGB_Buffer.current())); // preload buffer so we have a previous frame  on 1st loop
	cvtColor(RGB_Buffer.current(), Gray_Buffer.current(), COLOR_BGR2GRAY);
#ifdef SECONDCAMERA
		while (!capture2.read(RGB_Buffer2.current()));	cvtColor(RGB_Buffer2.current(), Gray_Buffer2.current(), COLOR_BGR2GRAY);
#endif

#ifdef OUTPUTCAP
	int debug = output_cap.open(filepath,
		CV_FOURCC_MACRO('M','J','P','G'), //YUYV
	capture.get(CV_CAP_PROP_FPS),
	Size(capture.get(CV_CAP_PROP_FRAME_WIDTH),
		capture.get(CV_CAP_PROP_FRAME_HEIGHT)),true);
	 Sleep(100);
	 if (!output_cap.isOpened())
	 {
		 cout << "!!! Output video could not be opened" << debug << "\n" <<
			 capture.get(CV_CAP_PROP_FPS) << "\n" <<
			 capture.get(CV_CAP_PROP_FRAME_HEIGHT) << "\n" <<
			 capture.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
		 return -1;
	 }
#endif
	 Mat kernel = (Mat_<int>(5, 5) <<
		 -1, -1, -1, -1, 0,
		 -1, -1, -1, 0, 1,
		 -1, -1, 0, 1, 1,
		 -1, 0, 1, 1, 1,
		 0, 1, 1, 1, 1);

	 // TRACKBARS
	 // Change thresholds
	 params.minThreshold = 0;//10
	 params.maxThreshold = 255;//20

							   // Filter by Area.
	 params.filterByArea = true;
	 params.minArea = 10;
	 params.maxArea = 50000;

	 // Filter by Circularity
	 params.filterByCircularity = true;
	 params.minCircularity = 0.4;

	 // Filter by Convexity
	 params.filterByConvexity = true;
	 params.minConvexity = 0.7;

	 // Filter by Inertia
	 params.filterByInertia = true;
	 params.minInertiaRatio = 0.01;

	 //Create trackbar to change contrast
	 //int minThreshold = 0;
	 //createTrackbar("minThreshold", "My Window", &minThreshold, 255);
	 //Point p = Point(0,0);
	 namedWindow("RGB");
	 //setMouseCallback("RGB", on_mouse, &p);
 // END INIT

	 // MAIN LOOP
	while (1) { // capture and process loop
		fps = 1000 / (1 + clock() - last_time); // time stuff
		last_time = clock();
		cout << "FPS: " << fps << endl; // faster than draw??
		framecount++; 

		capture.grab(); // two grabs right next to each other to maximize synchronization of frames
#ifdef SECONDCAMERA
		capture2.grab();
#endif
		capture.retrieve(RGB_Buffer.store()); //decode image and store in  RGB circular buffer
#ifdef SECONDCAMERA
		capture2.retrieve(RGB_Buffer2.store());
#endif

		imshow("RGB", RGB_Buffer.current()); //show our captured frame
	

	//	Mat Widow = Window(p, RGB_Buffer.current(), Size(200,200));
	//	imshow("window", Widow);

#ifdef SECONDCAMERA
		imshow("RGB2", RGB_Buffer2.current()); 
#endif
		cvtColor(RGB_Buffer.current(), Gray_Buffer.store(), COLOR_BGR2GRAY); // convert RGB image to grayscale
		//	imshow("Gray1", Gray_Buffer.current())

		cvtColor(RGB_Buffer2.current(), Gray_Buffer2.store(), COLOR_BGR2GRAY);
		//	imshow("Gray2", Gray_Buffer2.current()); 
		
	Algorithim1(Gray_Buffer.current(), Gray_Buffer.previous(), thresholdImage);
	imshow("Thresh1", thresholdImage);
#ifdef SECONDCAMERA
	Algorithim1(Gray_Buffer2.current(), Gray_Buffer2.previous(), thresholdImage2);
#endif

		if (trackingEnabled)
		{
			searchForMovement(thresholdImage, movingobjects, 1);
			imshow("TrackerImage", movingobjects);
			
			searchForMovement(thresholdImage2, movingobjects2, 2);
			imshow("TrackerImage2", movingobjects2);
		}

///////// TEST CODE START
		// KALMANN
		//cvtColor(RGB_Buffer.current(), Gray_Buffer.store(), COLOR_BGR2GRAY);
		//Algorithim1(Gray_Buffer.current(), Gray_Buffer.previous(), thresholdImage);
		//searchForMovement(thresholdImage, movingobjects);
		//		img = RGB_Buffer.current();
		//		Point2f center(trackedObject[0], trackedObject[1]);//center(img.cols*0.5f, img.rows*0.5f);
		//		float R = img.cols / 3.f;
		//		double stateAngle = state.at<float>(0);
		//		Point statePt = pointlist[imagecount];//calcPoint(center, R, stateAngle);
		//
		//		Mat prediction = KF.predict();
		//		double predictAngle = prediction.at<float>(0);
		//		Point predictPt = calcPoint(center, R, predictAngle);
		//
		//		randn(measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));
		//
		//		// generate measurement
		//		measurement += KF.measurementMatrix * state;
		//
		//		double measAngle = measurement.at<float>(0);
		//		Point measPt = calcPoint(center, R, measAngle);
		//
		//		// plot points
		//#define drawCross( center, color, d )                                        \
		//                line( img, Point( center.x - d, center.y - d ),                          \
//                             Point( center.x + d, center.y + d ), color, 1, LINE_AA, 0); \
//                line( img, Point( center.x + d, center.y - d ),                          \
//                             Point( center.x - d, center.y + d ), color, 1, LINE_AA, 0 )
//
//		//img = Scalar::all(0);
//		drawCross(statePt, Scalar(255, 255, 255), 3);
//		drawCross(measPt, Scalar(0, 0, 255), 3);
//		drawCross(predictPt, Scalar(0, 255, 0), 3);
//		line(img, statePt, measPt, Scalar(0, 0, 255), 3, LINE_AA, 0);
//		line(img, statePt, predictPt, Scalar(0, 255, 255), 3, LINE_AA, 0);
//
//		if (theRNG().uniform(0, 4) != 0)
//			KF.correct(measurement);
//
//		randn(processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
//		state = KF.transitionMatrix*state + processNoise;
//
//		imshow("Kalman", img);
// KALMANN END

//	pMOG2->apply(Gray_Buffer.current(), thresholdImage);


		// detect!
//		vector<cv::KeyPoint> keypoints;
		//detector->detect(thresholdImage, keypoints);
	//	Mat img_keypoints_1;
	//	drawKeypoints(thresholdImage, keypoints, img_keypoints_1, Scalar(255, 255, 255), DrawMatchesFlags::DEFAULT);
		//imshow("Keypoints 1", img_keypoints_1);
		//	imshow("Thresh2", thresholdImage2); 
	//	Mat dst;
	//	Point anchor = Point(-1, -1);
	//	int delta = 0;
	//	filter2D(RGB_Buffer.current(), dst, -1, kernel, anchor, delta, BORDER_DEFAULT);
		//imshow("emboss", dst);
	//	Algorithim2(Gray_Buffer.current(), thresholdImage);
	//	Algorithim2(Gray_Buffer2.current(), thresholdImage2);
	
	//	searchForCircles(thresholdImage, movingobjects);
		//putText(*grayImage1, "FPS: " + to_string(fps), textcenter3, fontFace, fontScale, Scalar::all(255), thickness, 5); // DONT DRAW ON GRAY OR RGB IMAGES BECAUSE THEY ARE STORED FOR NEXT CYCLE
//////////// TEST CODE END /////////////////////////////

#ifdef OUTPUTCAP		
	output_cap.write(RGB_Buffer.current());
#endif
		// start keyboard interface
		char key = waitKey(1);
		switch (key) {
		case 'q': //'esc' key has been pressed, exit program.
			output_cap.release();
			capture.release();
			return 0;
		case 't': //'t' has been pressed. this will toggle tracking
			trackingEnabled = !trackingEnabled;
			if (trackingEnabled == false) cout << "Tracking disabled." << endl;
			else cout << "Tracking enabled." << endl;
			break;
		case 'd': //'d' has been pressed. this will debug mode
			debugMode = !debugMode;
			if (debugMode == false) cout << "Debug mode disabled." << endl;
			else cout << "Debug mode enabled." << endl;
			break;
		case 'p': //'p' has been pressed. this will pause/resume the code.
			pause = !pause;
			if (pause == true) {
				cout << "Code paused, press 'p' again to resume" << endl;
				while (pause == true) {
					//stay in this loop until 
					switch (waitKey()) {
					case 'p':
						//change pause back to false
						pause = false;
						cout << "Code Resumed" << endl;
						break;
					}
				}
			}
		case 'r' :  // refresh the drawn points
			imagecount = 0;
			imagecount2 = 0;
			RGB_Buffer.current().copyTo(movingobjects);
			RGB_Buffer2.current().copyTo(movingobjects2);
			break;
		}
		// end keyboad interface
	}
	return 0;
}