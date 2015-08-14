//motionTracking.cpp

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.



#include "motiontracking.h"

//#define OUTPUTCAP
// START GLOBALS
//our sensitivity value to be used in the absdiff() function
const static int SENSITIVITY_VALUE = 20;
//size of blur used to smooth the intensity image output from absdiff() function
const static int BLUR_SIZE = 10;

int imagecount = 0;
int framecount = 0;
int calibcount = 0;
int movingcount = 0;

int airtime_clock = 0;
bool trackingEnabled = false;

Mat frame; //current frame
int keyboard; //input from keyboard
// END GLOBALS
Ptr<SimpleBlobDetector> detector;
void initblobs()
{
	//	 Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = minThreshold;//10
	params.maxThreshold = 255;//20

							  // Filter by Area.
	params.filterByArea = true;
	params.minArea = 10;
	params.maxArea = 50000;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.1;
	params.maxCircularity = 1.0;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.7;
	params.maxConvexity = 1.0;

	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.01;

	params.filterByColor = false;

	detector = SimpleBlobDetector::create(params);
}
// Search Image should be grayscale, because Houghcircles needs it to be in this format.
int searchForBalls(Mat &SearchImage, Mat &movingobjects, ballLoc &ballspotted) {
	//	Mat img = Mat(Size(SearchImage.cols, SearchImage.rows), CV_8UC1);
	int circlesfound = 0;
	int blobsfound = 0;
	//these two vectors needed for output of findContours
	vector<Vec3f> circles;
	HoughCircles(SearchImage, circles, CV_HOUGH_GRADIENT,
		dp, min_dist, param_1, param_2, min_radius, max_radius); // going to be dependant on resolution and windowed image
	circlesfound = circles.size();
	if (circlesfound > 0)
	{
		int largestradius = 0;
		for (int i = 0; i < circlesfound; i++)
		{
			ballspotted.radius = cvRound(circles[i][2]);
			if (ballspotted.radius > largestradius)
			{
				largestradius = ballspotted.radius; // new largest keypoint
				ballspotted.center = Point(cvRound(circles[i][0]), cvRound(circles[i][1]));
			}
			// draw the circle center
			//	circle(movingobjects, ballspotted.center, 3, Scalar(0, 255, 0), -1, 8, 0);
			// draw the circle outline
			//	circle(movingobjects, ballspotted.center, ballspotted.radius, Scalar(0, 0, 255), 3, 8, 0);
		}
		return circlesfound;
	}
	if (circlesfound == 0)
	{
		// detect!
		vector<cv::KeyPoint> keypoints;
		detector->detect(SearchImage, keypoints);
		blobsfound = keypoints.size();
		if (blobsfound > 0)
		{
			//	drawKeypoints(SearchImage, keypoints, movingobjects, Scalar(0, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

			int largestdiameter = 0;
			for (int i = 0; i < blobsfound; i++)
			{//assume the keypoint with the largest size is the ball
				if (keypoints[i].size > largestdiameter)
				{
					largestdiameter = keypoints[i].size; // new largest keypoint
					ballspotted.center = Point(cvRound(keypoints[i].pt.x), cvRound(keypoints[i].pt.y));
					ballspotted.radius = cvRound(keypoints[i].size / 2); // KeyPoint::size is a diameter
				}
			}
		}
		return blobsfound;
	}
	return 0; // nothing found;
}
 // finds moving objecs in an image
int searchForMovement(Mat CurrGraySource, Mat PrevGraySource, ballLoc &ballspotted) {
	Mat differenceImage; // grayscale
	Mat thresholdImage; // binary
	Mat blurImage;		// grayscale
	//perform frame differencing with the sequential images. 
	absdiff(PrevGraySource, CurrGraySource, differenceImage); // order matters here
	//imshow("diff", differenceImage);
	//threshold intensity image at a given sensitivity value
	threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
	//blur the image to get rid of the noise. This will output an intensity image
	blur(thresholdImage, blurImage, Size(BLUR_SIZE, BLUR_SIZE));
	//threshold again to obtain binary image from blur output
	threshold(blurImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
//	imshow("threshold", thresholdImage);
	bool objectDetected = false;
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	findContours(thresholdImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours, CHANGES THRESHOLD IMAGE
	//if contours vector is not empty, we have found some  moving objects
	if (contours.size() > 0)
	{
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVec;
		//	circlecheck(Point(contours.at(i).at(i).x, contours.at(i).at(i).y));
		//largestContourVec.push_back(contours.at(contours.size() - 1));
		float circularratio;
		double refArea = 0;
		bool objectFound = false;
		for (int i = contours.size() - 1; i >= 0; i--) // start at the largest contour, work way down.
		{
			Moments moment = moments((cv::Mat)contours[i]);
			double area = moment.m00;
			int MIN_OBJECT_AREA = 300; // resolution dependant or independat?
			int MAX_OBJECT_AREA = thresholdImage.rows*thresholdImage.cols / 1.5; // 3/2 area of image
			//iteration and compare it to the area in the next iteration.
			if (area > MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea) {
				
			//	ballspotted.center = Point(moment.m10 / area, moment.m01 / area);
			//	ballspotted.radius = 3;
				objectFound = true;
				refArea = area;
				Mat tempnothing;
				if (searchForBalls(CurrGraySource, tempnothing, ballspotted) == 1)
					return 1;
			}
			else objectFound = false;
		}
		return 0; // no balls found
			//let user know you found an object
		//	if (objectFound == true) {
			//	putText(movingobjects, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
				//draw object location on screen
			//	drawObject(x, y, movingobjects);
		//	}
			//else 
			//	putText(movingobjects, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	//	}
			//objectBoundingRectangle = boundingRect(contours.at(i));//largestContourVec.at(0));
			//if (objectBoundingRectangle.area() > 0 && objectBoundingRectangle.area() < 50000) // SIZE CHECK DEPENDANT ON RESOLUTION
			//{
			//	int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
			//	int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;
			//	int foundcircle = 0;
			//	objectBoundingRectangle.width = objectBoundingRectangle.width + 20;
			//	objectBoundingRectangle.height = objectBoundingRectangle.height + 20;
			//	movingobjects = Window(objectBoundingRectangle, CurrGraySource);
			//	movingcount++;
			//	imwrite("testimage" + to_string(movingcount) + ".png", movingobjects);
			//	//movingobjects = Window(Point(xpos, ypos), CurrGraySource, Size(objectBoundingRectangle.width, objectBoundingRectangle.height));
			//	foundcircle = searchForCircles(movingobjects, movingobjects);
			////	imshow("moving objects", movingobjects);
			//	if (foundcircle > 0)
			//	{ // at least one circle found
			//	// moving objects has one ball, start running tracking
			//		imwrite("circles" + to_string(framecount) + ".png", movingobjects);
			//	//	break; // stop looking?
			//	}
			//	//	rectangle(rects, objectBoundingRectangle, Scalar(255, 255, 255));
			//	imshow("rect", rects);
			//	//rectangle(movingobjects, objectBoundingRectang le, Scalar(255, 255, 255));
			////	imwrite("rectangle" + intToString(framecount) + ".png", movingobjects);
			//	circularratio = (float)objectBoundingRectangle.width / objectBoundingRectangle.height;
			//	float threshold = 1; 
			//	if (circularratio > (float)1/threshold || circularratio < (float)1*threshold)// (circularratio > 1 - threshold || circularratio < 1 + threshold)  //then roundness check
			//	{ // we think this a moving circle now
			//	circle(movingobjects, Point(xpos, ypos), 20, Scalar(255, 255, 255), 2);
			//	 // make these generic structs to be passed to tracking method
			//	pointlist[imagecount] = Point(xpos, ypos); 
			//	if (imagecount > 0)
			//	{
			//		line(movingobjects, pointlist[imagecount - 1], pointlist[imagecount], Scalar(255, 255, 255), 2);
			//	}
			//	imagecount++;
			//	}
			//}
			//else
			//	break;

		//}
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
	}
	return 0;
}

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
	cvUseOptimized(1);
	bool objectDetected = false;
	//pause and resume code
	bool pause = false;

	Mat differenceImage;
	Mat thresholdImage =  Mat(RESOLUTION, CV_8UC1);
	Mat movingobjects = Mat(RESOLUTION, CV_8UC3);
	Mat* contour_drawing; 
	
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

//END DECLARATIONS

// START INIT

	int capids[] = { 701, 702};
	string config = videopath;
	//string config = "L";

	Stereotime stereoset;// Stereotime::Stereo_update
	stereoset.Stereo_Init();
	initblobs();
	createTrackbars();
	//void(*on_trackbar)(int, void*) = (void(*))Stereotime::Stereo_update;//(int, void*)
	ImagePipeline Pipeline(2, capids, &config, searchForMovement, searchForBalls); // init with webcams source
//	ImagePipeline Pipeline(1, &config, &config, searchForMovement, searchForBalls); // init with video source
	int trackmode = 0;
#ifdef OUTPUTCAP
	int debug = output_cap.open(filepath,
		CV_FOURCC_MACRO('M','J','P','G'), //YUYV
	30, RESOLUTION, true);
	 Sleep(100);
	 if (!output_cap.isOpened())
	 {
		 cout << "!!! Output video could not be opened" << debug << "\n" 
			 //<<
			 //capture.get(CV_CAP_PROP_FPS) << "\n" <<
			// capture.get(CV_CAP_PROP_FRAME_HEIGHT) << "\n" <<
			// capture.get(CV_CAP_PROP_FRAME_WIDTH) 
			 << endl;
		 return -1;
	 }
#endif

	 //Create trackbar to change contrast
	 //int minThreshold = 0;
	 //createTrackbar("minThreshold", "My Window", &minThreshold, 255);
	 //Point p = Point(0,0);
	// namedWindow("RGB");
//	 namedWindow("circles", 1);
	 //setMouseCallback("RGB", on_mouse, &p);
	 Mat smalldisp;
	 Mat bigdisp;								/// WAS 1
	 Mat ShiftedImage;// = Mat(RESOLUTION, CV_8UC1);
	 Point2f srcTri[3];
	 Point2f dstTri[3];
	 srcTri[0] = Point2f(1);
	 srcTri[1] = Point2f(1);
	 srcTri[2] = Point2f(1);
	 dstTri[0] = Point2f(1);
	 dstTri[1] = Point2f(1);
	 dstTri[2] = Point2f(1);
	 Mat affinetrans = getAffineTransform(srcTri, dstTri);
	 Mat TranslationMat = Mat(Size(3, 2), affinetrans.type());
	 TranslationMat(Rect(Point(0, 0), Size(1, 1))) = 1;
	 TranslationMat(Rect(Point(1, 0), Size(1, 1))) = 0;
	 TranslationMat(Rect(Point(2, 0), Size(1, 1))) = 30;
	 TranslationMat(Rect(Point(0, 1), Size(1, 1))) = 0;
	 TranslationMat(Rect(Point(1, 1), Size(1, 1))) = 1;
	 TranslationMat(Rect(Point(2, 1), Size(1, 1))) = 0;
 // END INIT
	 Pipeline.cap(); // initial frame for each buffer so previous frame is available on first loop
	 Pipeline.RGBtoGray();
	////////// MAIN LOOP START
	while (1) { // capture and process loop
		fps = 1000 / (1 + clock() - last_time); // time stuff
		last_time = clock();
		cout << "FPS: " << fps << endl; // faster than draw??
		framecount++; 

		Pipeline.cap();
		Pipeline.RGBtoGray();
		imshow("L_RGB", Pipeline.sources[0]->RGB_Buffer->current());
		//imshow("L_GRAY", Pipeline.sources[0]->Gray_Buffer->current());
		imshow("R_RGB", Pipeline.sources[1]->RGB_Buffer->current());
	//	imshow("R_GRAY", Pipeline.sources[1]->Gray_Buffer->current());

	//	warpAffine(Pipeline.sources[0]->Gray_Buffer->current(), ShiftedImage, TranslationMat, RESOLUTION);
		//imshow("shift", ShiftedImage);

	//	stereoset.compute_Stereo(Pipeline.sources[0]->Gray_Buffer->current(), ShiftedImage, bigdisp);
	//	Mat Windowdisp = Window(Rect(Point(50,50), Size(50,50)), bigdisp);
	//	imshow("bdisp", Windowdisp);

	//	double errorL2 = norm(smalldisp, Windowdisp, CV_L2);
		// Convert to a reasonable scale, since L2 error is summed across all pixels of the image.
	//	double similarity = errorL2 / (double)(smalldisp.rows * smalldisp.cols);
	//	cout << "similarity:" << similarity << endl;
		// try abs diff next?
	//	Pipeline.detect(Pipeline.sources[0]->Gray_Buffer->current(), Pipeline.sources[0]->Gray_Buffer->previous(), Pipeline.objTracker.trackerList[0][0].ball_location);
	//	Pipeline.detect(Pipeline.sources[1]->Gray_Buffer->current(), Pipeline.sources[1]->Gray_Buffer->previous(), Pipeline.objTracker.trackerList[1][0].ball_location);
	// PIPELINE START
		//if (trackmode == 0)
		//{
		//	if (Pipeline.detection() == 1)
		//		trackmode = 1; // we found the ball, now track it!WWW
		//}
		//else if (trackmode == 1)
		//{
		//	int q;
		//	if (Pipeline.detection() == 1)
		//	{
		//		Mat WindowL = Window(Rect(Pipeline.objTracker.trackerList[0][Pipeline.objTracker.ballcount].ball_location.center, Size(100, 100)), Pipeline.sources[0]->Gray_Buffer->current());
		//		Mat WindowR = Window(Rect(Pipeline.objTracker.trackerList[1][Pipeline.objTracker.ballcount].ball_location.center, Size(100, 100)), Pipeline.sources[1]->Gray_Buffer->current());
		//		stereoset.compute_Stereo(WindowL, WindowR, Pipeline.sources[0]->Disp_Buffer->store());
		//		imshow("stereo", Pipeline.sources[0]->Disp_Buffer->store());
		//	}
		//	else
		//		trackmode = 2;//we've lost the ball
		//	//else
		//	//	Pipeline.tracking();
		//	//if (Pipeline.objTracker.ballcount > 6)
		//	//{
		//	//	trackmode = 2;	// we've decided this is the ball, now project
		//	//}
		//}
		//else if (trackmode == 2) // post calculation.
		//{ // calculate trajectory using stereo and predict where ball is going to land.
		//	for (int i = 0; i < Pipeline.objTracker.ballcount; i++)
		//	{
		//		Mat WindowL = Window(Rect(Pipeline.objTracker.trackerList[0][i].ball_location.center, Size(120, 120)), Pipeline.sources[0]->Gray_Buffer->current());
		//		Mat WindowR = Window(Rect(Pipeline.objTracker.trackerList[1][i].ball_location.center, Size(120, 120)), Pipeline.sources[1]->Gray_Buffer->current());
		//		stereoset.compute_Stereo(WindowL, WindowR, smalldisp);
		//		imshow("sdisp", smalldisp);
		//	}
		//}
///////////////	PIPELINE END
	//	searchForCircles(thresholdImage, movingobjects);
	// DONT DRAW ON GRAY OR RGB IMAGES BECAUSE THEY ARE SOURCE IMAGES AND DRAWING PERMANENTLY MODIFIES THE IMAGE
	// useful code
	// copy an smaller image into a bigger one
	//Mat BlackMatR = Mat(RESOLUTION, CV_8UC1);
	//Mat WindowR = Window(Rect(1, 1, 51, 51), Pipeline.sources[1]->Gray_Buffer->current());
	//cv::Rect roiR(cv::Point(1, 1), WindowR.size());
	//WindowR.copyTo(BlackMatR(roiR));
#ifdef OUTPUTCAP		
	output_cap.write(Pipeline.sources[0]->RGB_Buffer->current());
#endif
		// start keyboard interface
		char key = waitKey(1);
		switch (key) {
		case 'q': //'esc' key has been pressed, exit program.
			output_cap.release();
			// add all deconstructors here?
			return 0;
		case 't': //'t' has been pressed. this will toggle tracking
			trackingEnabled = !trackingEnabled;
			if (trackingEnabled == false) cout << "Tracking disabled." << endl;
			else cout << "Tracking enabled." << endl;
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
		case 'c':
			calibcount++;
			imwrite("left" + to_string(calibcount) + ".jpg", Pipeline.sources[0]->RGB_Buffer->current()); // left
			imwrite("right" + to_string(calibcount) + ".jpg", Pipeline.sources[1]->RGB_Buffer->current()); // right
			break;
		}
		// end keyboad interface
	}
	return 0;
}

