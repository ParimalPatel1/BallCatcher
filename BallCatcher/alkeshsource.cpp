
#include <opencv\cv.h>
#include  <opencv2\opencv.hpp>
#include <Windows.h>
#include <ctime>


using namespace cv;
using namespace std;
// prototypes for main
void createTrackbars();
void initblobs();
void Hough_update();
void Blob_update();
void on_trackbar1(int, void*)
{
	Hough_update();
};//This function gets called whenever a trackbar position is changed
void on_trackbar2(int, void*)
{
	Blob_update();
};//This function gets called whenever a trackbar position is changed
// Hough circle params
int dp = 1; /*The inverse ratio of resolution*/	const int dp_MAX = 10;
int min_dist = 20; /*Minimum distance between detected centers*/const int min_dist_MAX = 200;
int param_1 = 200; /*Upper threshold for the internal Canny edge detector */const int param_1_MAX = 400;
int param_2 = 100; /* Threshold for center detection.*/	const int param_2_MAX = 200;
int min_radius = 0; /* Minimum radio to be detected.If unknown, put zero as default. */	const int min_radius_MAX = 40;
int max_radius = 0; /*Maximum radius to be detected.If unknown, put zero as default */	const int max_radius_MAX = 100;

int minThreshold = 0;
int maxThreshold = 255;
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

const int minThreshold_MAX = 254;
const int maxThreshold_MAX = 255;
const int filterByArea_MAX = 1;
const int minArea_MAX = 1500;
const int maxArea_MAX = 80000;
const int filterByCircularity_MAX = 1;
const int minCircularity_MAX = 9;
const int maxCircularity_MAX = 10;
const int filterByConvexity_MAX = 1;
const int minConvexity_MAX = 9;
const int maxConvexity_MAX = 10;
const int filterByInertia_MAX = 1;
const int minInertiaRatio_MAX = 99;

const string trackbarWindowName1 = "HoughTrackbars";
const string trackbarWindowName2 = "BlobTrackbars";

Ptr<SimpleBlobDetector> detector;
#define numofimages 50
// Search Image should be grayscale, because Houghcircles needs it to be in this format.
int searchForBalls(Mat &SearchImage, Mat &movingobjects, int method) {
	//	Mat img = Mat(Size(SearchImage.cols, SearchImage.rows), CV_8UC1);
	int circlesfound = 0;
	//these two vectors needed for output of findContours
	//vector<Vec3f> circles;
	//HoughCircles(SearchImage, circles, CV_HOUGH_GRADIENT,
	//	dp, min_dist, param_1, param_2, min_radius, max_radius); // going to be dependant on resolution and windowed image
	//circlesfound = circles.size();
	//if (circlesfound > 0)
	//{
	//	for (size_t i = 0; i < circlesfound; i++)
	//	{
	//		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	//		int radius = cvRound(circles[i][2]);
	//		// draw the circle center
	//		circle(SearchImage, center, 3, Scalar(0, 255, 0), -1, 8, 0);
	//		// draw the circle outline
	//		circle(SearchImage, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	//	}
	//}
	//if (circlesfound == 0)
	//{
	// detect!
	vector<cv::KeyPoint> keypoints;
	detector->detect(SearchImage, keypoints);
	drawKeypoints(SearchImage, keypoints, movingobjects, Scalar(0, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//	}
	return circlesfound;
}
int main()
{
	int imageid = 1;
	Mat testimage; // loaded image
	Mat drawimage; // image we can draw on and refresh
	Mat movingobjects;
	testimage = imread("coin" + to_string(imageid) + ".png", CV_LOAD_IMAGE_COLOR);   // load the first image
	if (!testimage.data)// Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}
	cvtColor(testimage, testimage, COLOR_BGR2GRAY);
	testimage.copyTo(drawimage);
	testimage.copyTo(movingobjects);
	initblobs();
	createTrackbars();
	Mat RGB_image;
	VideoCapture cam(700); // could be 701, or 702 as well. 
	if (!cam.isOpened()) {
		cout << "ERROR ACQUIRING VIDEO FEED for id \n";
		getchar();
		//error; 
	}
	cam.read(RGB_image);
	//cvtColor(RGB_image, testimage, COLOR_BGR2GRAY);
	//testimage.copyTo(drawimage);
	//testimage.copyTo(movingobjects);

	namedWindow("Circle window", WINDOW_AUTOSIZE);// Create a window for display.
	namedWindow("Blob window", WINDOW_AUTOSIZE);// Create a window for display.
	while (imageid > 0 && imageid < numofimages)
	{

		searchForBalls(drawimage, movingobjects, 0);
		imshow("Circle window", drawimage);
		imshow("Blob window", movingobjects);

		// keyboard interface
		char key = waitKey(20);
		switch (key) {
		case 'n': // load the next image
			imageid++;
			testimage = imread("coin" + to_string(imageid) + ".png", CV_LOAD_IMAGE_COLOR);   // load the next image
			if (!testimage.data)                              // Check for invalid input
			{
				cout << "Could not open or find the image" << std::endl;
				return -1;
			}
			cvtColor(testimage, testimage, COLOR_BGR2GRAY);
			testimage.copyTo(movingobjects);
			testimage.copyTo(drawimage);
			break;
		case 'p': // load the previous image
			imageid--;
			testimage = imread("coin" + to_string(imageid) + ".png", CV_LOAD_IMAGE_COLOR);   // load the next image
			if (!testimage.data)                              // Check for invalid input
			{
				cout << "Could not open or find the image" << std::endl;
				return -1;
			}
			cvtColor(testimage, testimage, COLOR_BGR2GRAY);
			testimage.copyTo(movingobjects);
			testimage.copyTo(drawimage);
			break;
		case 'r': // refrehs the draw image
			testimage.copyTo(drawimage);
			testimage.copyTo(movingobjects);
			break;
		case 'c': // capture an image from the webcam
			cam.read(RGB_image);
			cvtColor(RGB_image, testimage, COLOR_BGR2GRAY);
			testimage.copyTo(drawimage);
			testimage.copyTo(movingobjects);
			break;
		}
	}
	return 0;
}

void initblobs()
{
//	 Setup SimpleBlobDetector parameters.
	Ptr<SimpleBlobDetector> detector;
	SimpleBlobDetector::Params params;
	params.minThreshold = 150;
	params.maxThreshold = 230;
	params.filterByArea = true;
	params.minArea = 50;
	params.maxArea = 50000;
	params.filterByConvexity = true;
	params.minConvexity = 0.9;
	params.maxConvexity = 1.0;

	params.filterByCircularity = false;
	params.filterByInertia = false;
	params.filterByColor = false;
	detector = SimpleBlobDetector::create(params);
}
	void Hough_update()
	{
		//dp, min_dist, param_1, param_2, min_radius, max_radius
		int temp;

		temp = getTrackbarPos("dp", trackbarWindowName1);
		dp = temp;

		temp = getTrackbarPos("min_dist", trackbarWindowName1);
		min_dist = temp;

		temp = getTrackbarPos("param_1", trackbarWindowName1);
		param_1 = temp;

		temp = getTrackbarPos("param_2", trackbarWindowName1);
		param_2 = temp;

		temp = getTrackbarPos("min_radius", trackbarWindowName1);
		min_radius = temp;

		temp = getTrackbarPos("max_radius", trackbarWindowName1);
		max_radius = temp;
		// example of adjusting trackbar value
	//	temp = getTrackbarPos("SADWindowSize", trackbarWindowName)*2.5 + 5;
	//	if (temp % 2 == 1 && temp >= 5 && temp <= 255 && temp <= RESOLUTION.height) {
	//		bm->setSpeckleWindowSize(temp);
	//		cout << getTrackbarPos("SADWindowSize", trackbarWindowName) << "\t" << temp << endl;
	//	}
	}
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
		btemp = getTrackbarPos("filterByArea", trackbarWindowName2) == 1 ? true: false;
		params.filterByArea = btemp;
		params.minArea = getTrackbarPos("minArea", trackbarWindowName2);
		params.maxArea = getTrackbarPos("maxArea", trackbarWindowName2);

		// Filter by Circularity
		btemp = getTrackbarPos("filterByCircularity", trackbarWindowName2) == 1 ? true : false;
		params.filterByCircularity = btemp;
		ftemp = (float)getTrackbarPos("minCircularity", trackbarWindowName2)/10.0;
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
	void createTrackbars() { //Create window for trackbars
		char TrackbarName[200];

		// Create TrackBars Window
		namedWindow(trackbarWindowName1, 0);

		sprintf(TrackbarName, "dp");
		sprintf(TrackbarName, "min_dist");
		sprintf(TrackbarName, "param_1");
		sprintf(TrackbarName, "param_2");
		sprintf(TrackbarName, "min_radius");
		sprintf(TrackbarName, "max_radius");

		//Create Trackbars and insert them into window
		createTrackbar("dp", trackbarWindowName1, &dp, dp_MAX, on_trackbar1);
		createTrackbar("min_dist", trackbarWindowName1, &min_dist, min_dist_MAX, on_trackbar1);
		createTrackbar("param_1", trackbarWindowName1, &param_1, param_1_MAX, on_trackbar1);
		createTrackbar("param_2", trackbarWindowName1, &param_2, param_2_MAX, on_trackbar1);
		createTrackbar("min_radius", trackbarWindowName1, &min_radius, min_radius_MAX, on_trackbar1);
		createTrackbar("tmax_radius", trackbarWindowName1, &max_radius, max_radius_MAX, on_trackbar1);
		
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