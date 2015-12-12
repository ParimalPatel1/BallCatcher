//motionTracking.cpp

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.



#include "motiontracking.hpp"
using namespace cv;
using namespace std;

//#define OUTPUTCAP
// START GLOBALS

int imagecount = 0;
int framecount = 0;
int calibcount = 0;
int movingcount = 0;

int H_MIN = 30;
int H_MAX = 120;
int S_MIN = 54;
int S_MAX = 135;
int V_MIN = 135;
int V_MAX = 240;

const int MIN_OBJECT_AREA = 100;
const int MAX_OBJECT_AREA = RESOLUTION.height * RESOLUTION.width / 1.5;
#define DEBUG
int keyboard; //input from keyboard
// END GLOBALS


bool largerradius(ballLoc a, ballLoc b)
{
	return (a.radius < b.radius);
}
ballLoc searchForBalls(Mat& HSVImage, Mat& CurrGraySource, Mat& PrevGraySource) {
	Mat differenceImage; // grayscale
	Mat BGthreshold; // backround threshold
	Mat blurImage;		// grayscale
	Mat HSVthreshold;
	Mat thrownballthreshold;
	// Part 1
	//perform frame differencing with the sequential images. 
	absdiff(PrevGraySource, CurrGraySource, differenceImage); // order matters here
	threshold(differenceImage, BGthreshold,  SENSITIVITY_VALUE, 255, THRESH_BINARY);
	//blur the image to get rid of the noise. This will output an intensity image
	blur(BGthreshold, blurImage, Size(BLUR_SIZE, BLUR_SIZE));
	//threshold again to obtain binary image from blur output
	threshold(blurImage, BGthreshold, SENSITIVITY_VALUE, 255, THRESH_BINARY);
	// Part 2
	inRange(HSVImage, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), HSVthreshold);
	//Mat erodeElement = getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, Size(3, 3));
	////dilate with larger element so make sure object is nicely visible
	//Mat dilateElement = getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, Size(8, 8));
	//erode(HSVthreshold, HSVthreshold, erodeElement);
	//erode(HSVthreshold, HSVthreshold, erodeElement);
	//dilate(HSVthreshold, HSVthreshold, dilateElement);
	//dilate(HSVthreshold, HSVthreshold, dilateElement);
	// Part 3 find all moving balls
	bitwise_and(HSVthreshold, BGthreshold, thrownballthreshold);

	vector<ballLoc> ball_canidates;
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	findContours(thrownballthreshold, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours, CHANGES THRESHOLD IMAGE
	double refArea = 0;
	bool objectFound = false;
	int numObjects = hierarchy.size();
	if (numObjects > 0) {
		for (int index = 0; index >= 0; index = hierarchy[index][0]) 
		{
			Moments moment = moments((cv::Mat)contours[index]);
			double area = moment.m00;
			//if the area is less than 20 px by 20px then it is probably just noise
			//if the area is the same as the 3/2 of the image size, probably just a bad filter
			//we only want the object with the largest area so we save a reference area each
			//iteration and compare it to the area in the next iteration.
			if (area > MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea) {
				//	x = moment.m10 / area;
				//	y = moment.m01 / area;
				ballLoc ball_canidate;
				cv::minEnclosingCircle(contours[index], ball_canidate.center, ball_canidate.radius);
				ball_canidates.push_back(ball_canidate);
				objectFound = true;
				refArea = area;
			}
			//else objectFound = false; // ??
		}
	}
	// Part 4 return the best ball if one is found
	//let user know you found an object
	if (objectFound == true) {
		//putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
		//draw object location on screen
		sort(ball_canidates.begin(), ball_canidates.end(), largerradius); // sort by diameter
		int i = ball_canidates.size() - 1;
		return ball_canidates[i];
		//	drawObject(ball_canidates[i].center.x, ball_canidates[i].center.y, ball_canidates[i].radius, cameraFeed);
	}
	else // nothing found
	{
		ballLoc empty;
		empty.radius = -1;
		return empty;
	}
}

bool morePoints(vector<Point> a, vector<Point> b)
{
	return (a.size() < b.size());
}
 // finds moving objecs in an image
int const max_lowThreshold = 100;
int lowThreshold = 50;
int ratio = 3;
int kernel_size = 3;
int searchForMovement(Mat CurrGraySource, Mat PrevGraySource, ballLoc &ballspotted) {
	Mat differenceImage; // grayscale
	Mat thresholdImage; // binary
	Mat blurImage;		// grayscale
	Mat edges;			//binary

	Mat drawimage;
	CurrGraySource.copyTo(drawimage);
	cvtColor(CurrGraySource, drawimage, CV_GRAY2RGB);

	//perform frame differencing with the sequential images. 
	absdiff(PrevGraySource, CurrGraySource, differenceImage); // order matters here
//	imshow("diff", differenceImage);
	//threshold intensity image at a given sensitivity value
	threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
	//blur the image to get rid of the noise. This will output an intensity image
	blur(thresholdImage, blurImage, Size(BLUR_SIZE, BLUR_SIZE));
	//threshold again to obtain binary image from blur output
	threshold(blurImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
	//
//	cv::Canny(CurrGraySource, edges, lowThreshold, lowThreshold*ratio, kernel_size);
	//imwrite("backround2.png", thresholdImage);
	//imwrite("edges2.png", edges);
	//vector<Vec2f> lines;
	//HoughLines(edges, lines, 1, CV_PI / 180, 100, 0, 0);

	//for (size_t i = 0; i < lines.size(); i++)
	//{
	//	float rho = lines[i][0], theta = lines[i][1];
	//	Point pt1, pt2;
	//	double a = cos(theta), b = sin(theta);
	//	double x0 = a*rho, y0 = b*rho;
	//	pt1.x = cvRound(x0 + 1000 * (-b));
	//	pt1.y = cvRound(y0 + 1000 * (a));
	//	pt2.x = cvRound(x0 - 1000 * (-b));
	//	pt2.y = cvRound(y0 - 1000 * (a));
	//	line(drawimage, pt1, pt2, Scalar(0, 0, 255), 1, CV_AA);
	//}
	//cv::imshow("edges", edges);
	//cv::imshow("lines", drawimage);
	//imshow("threshold", thresholdImage);
	//waitKey(10);

//	Mat ContourImage;
//	edges.copyTo(ContourImage);
	//thresholdImage.copyTo(ContourImage);
	//bool objectDetected = false;
	//these two vectors needed for output of findContours
	//vector< vector<Point> > contours;
	//vector<Vec4i> hierarchy;
	////Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
	////findContours(ContourImage,contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);// retrieves all contours
	//findContours(ContourImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours, CHANGES THRESHOLD IMAGE

	////if contours vector is not empty, we have found some  moving objects
	//if (contours.size() > 0)
	//{
	//	vector<Point2f>center(contours.size());
	//	vector<float>radius(contours.size());
	//	//double refArea = 0;
	//	//bool objectFound = false;

	//	int MIN_OBJECT_AREA = 100; // resolution dependant or independent?
	//	int MAX_OBJECT_AREA = 30000;//thresholdImage.rows*thresholdImage.cols / 1.5; // 3/2 area of image

	//	sort(contours.begin(), contours.end(), morePoints); // sort contours by pixel size
	//	for (int i = contours.size() - 1; i >= 0; i--) // start at the largest contour, work way down.
	//	{
	//		if (contours[i].size() > 7)
	//		{
	//			Moments mo = moments((Mat)contours[i]);
	//			double area = mo.m00;
	//			if (area > MIN_OBJECT_AREA)// && area<MAX_OBJECT_AREA && area>refArea)
	//			{ // three methods for calculating eccentricity
	//				RotatedRect tempellipse;
	//				tempellipse.size.height = 0;
	//				findTheBall(contours[i], tempellipse, true);
	//				/*	vector<Point> tc;
	//					tc.push_back(Point(78,60));
	//					tc.push_back(Point(76,65));
	//					tc.push_back(Point(73, 69));
	//					tc.push_back(Point(69, 73));
	//					tc.push_back(Point(65, 76));
	//					tc.push_back(Point(60, 78));
	//					tc.push_back(Point(55, 79));
	//					tc.push_back(Point(50, 80));
	//					tc.push_back(Point(45, 79));
	//					findTheBall(tc, tempellipse);
	//					tempellipse = cv::fitEllipse(tc);*/
	//				if (tempellipse.size.height > 0)
	//				{
	//					ellipse(drawimage, tempellipse, Scalar(255), 2);
	//					/*	for (int j = 0; j < tc.size() - 1; j++)
	//						{
	//							circle(drawimage, tc[j], 1, Scalar(240, 230, 211));
	//						}*/
	//				}
	//			}
	//		}
	//	}
	//	return 0; // no balls found
	//}
	return 0;
}
cv::Point3f quadequ(vector<Point2f> ayy)
{
	int hue = ayy.size();
	cv::Mat A = cv::Mat::ones(hue, 3, CV_32F);

	for (int i = 0; i < hue; i++)
	{ 	//// initialize A
		A.at<float>(i, 0) = std::pow(ayy[i].x, 2);
		A.at<float>(i, 1) = ayy[i].x;
	}
	cv::Mat B = cv::Mat::ones(hue, 1, CV_32F);
	for (int j = 0; j < hue; j++) // initalize B
	{
		B.at<float>(j, 0) = ayy[j].y;
	}
	////declare a vecotor for results
	Mat coeff(3, 1, CV_32F);

	//// solve the linear system
	solve(A, B, coeff, cv::DECOMP_NORMAL);
	cv::Point3f coe;
	coe.x = coeff.at<float>(0, 0);
	coe.y = coeff.at<float>(1, 0);
	coe.z= coeff.at<float>(2, 0);
	return coe;
}
HANDLE frameMutex;
//int framecount = 0;
const int maxframedifference = 20; // can be at most 20 frames ahead
DWORD WINAPI capthread(__in LPVOID lpParameter)
{
	char escapekey;
	ImagePipeline* Pipeline = (ImagePipeline*)lpParameter;

	DWORD dwCount = 0, dwWaitResult;

	while (waitKey(1) != 'q')
	{
		if (Pipeline->sources[0]->RGB_Buffer->writespots() > 1)
		{
			printf("cap");
			Pipeline->cap();
			Pipeline->stereoset.remap_RGB(Pipeline->sources[0]->RGB_Buffer->Wcurrent(), Pipeline->sources[1]->RGB_Buffer->Wcurrent());
			Pipeline->RGBtoGray();
			Pipeline->RGBtoHSV();
		}
	}
	return 0;
}

DWORD WINAPI calcthread(__in LPVOID lpParameter)
{
	ImagePipeline* Pipeline = (ImagePipeline*)lpParameter;
	char escapekey;
	while (waitKey(1) != 'q')
	{
		if (Pipeline->sources[1]->HSV_Buffer->readspots() > 1)
		{
			printf("read");
			Pipeline->detection();
			if (Pipeline->detection() == 1) // balls found, project!
			{
				Pipeline->projectBall();
			//	//printf("ball detect!");
			//	Pipeline->objTracker.listballs();
			//	while (1);
			}
			imshow("left_RGB", Pipeline->sources[0]->RGB_Buffer->read());
			imshow("right_RGB", Pipeline->sources[1]->RGB_Buffer->read());
		}
	}
	return 0;
}

int main() {
	// START DECLARATIONS
	//pause and resume code
	bool pause = false;
	float start_time = clock(); // timer variable
	unsigned int last_time = clock();
	unsigned int framecounter = 0;
	float fps;
	const string filepath = "test.avi"; //C:\Users\Benjamin\Desktop
	VideoWriter output_cap;
	const string videopath = "ballthrow2.avi";
//END DECLARATIONS

// START INIT

	bool trackingEnabled = false;
	int capids[] = { 2,1 };
	string config = "does nothing"; 	// make config string meaningful
	string videosource = videopath;
	Stereotime stereoset;// intergrate with pipeline
	stereoset.Stereo_Init(RESOLUTION);
	//createTrackbars();
//	createTrackbars_Blob();
	ImagePipeline Pipeline(2, capids, &config, searchForBalls, searchForBalls); // init with webcams source
	//ImagePipeline PipelineR(1, capidsR, &config, searchForMovement, searchForBalls); // init with webcams source
//	ImagePipeline Pipeline(1, &videosource, &config, searchForMovement, searchForBalls); // init with video source
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
 // END INIT
	 ballLoc ballspotted;
	 //Ptr<BackgroundSubtractor> BGS = createBackgroundSubtractorMOG2();
	 //Mat FG;// foreground
	 //Mat FGT;
	 //Mat FGB;
	 //Mat BG; // background
	 //for (int i = 0; i < 100; i++)
	 //{
		// Pipeline.cap(); // initial frame for each buffer so previous frame is available on first loop
		// Pipeline.RGBtoGray();
		// BGS->apply(Pipeline.sources[0]->Gray_Buffer->current(), FG, 0.8);
	 //}
	 //BGS->getBackgroundImage(Pipeline.sources[0]->background_image);

	// stereoset.remap_Grey(Pipeline.sources[0]->Gray_Buffer->current(), Pipeline.sources[1]->Gray_Buffer->current());
	 //Pipeline.sources[0]->Gray_Buffer->current().copyTo(Pipeline.sources[0]->background_image);
	 //Pipeline.sources[1]->Gray_Buffer->current().copyTo(Pipeline.sources[1]->background_image);
#ifdef DEBUG
	 namedWindow("Debug Window", WINDOW_AUTOSIZE);// Create a window for display.
#endif
	////////// MAIN LOOP START
	 HANDLE caphandle, calchandle;
	 caphandle = CreateThread(0, 0, capthread, &Pipeline, 0, 0);
	 calchandle = CreateThread(0, 0, calcthread, &Pipeline, 0, 0);

	while (1) { // capture and process loop
		//fps = 1000.0/(float)(1 + clock() - last_time); // time stuff
		//last_time = clock();
		//cout << "FPS: " << fps << endl; // faster than draw??
		//framecount++; 

		//Pipeline.cap();
		//Pipeline.RGBtoGray();
		//Pipeline.RGBtoHSV();
		//stereoset.remap_Grey(Pipeline.sources[0]->Gray_Buffer->current(), Pipeline.sources[1]->Gray_Buffer->current());
		//stereoset.remap_HSV(Pipeline.sources[0]->HSV_Buffer->current(), Pipeline.sources[1]->HSV_Buffer->current());


		//balllocationleft = searchForBalls(Pipeline.sources[0]->HSV_Buffer->current(), Pipeline.sources[0]->Gray_Buffer->previous(), Pipeline.sources[0]->Gray_Buffer->current());
		//balllocationright = searchForBalls(Pipeline.sources[1]->HSV_Buffer->current(), Pipeline.sources[1]->Gray_Buffer->previous(), Pipeline.sources[1]->Gray_Buffer->current());

		//Pipeline.sources[0]->HSV_Buffer->current().copyTo(debugimageleft);
		//Pipeline.sources[1]->HSV_Buffer->current().copyTo(debugimageright);
		//if (balllocationleft.radius >= 0)
		//	drawObject(balllocationleft.center.x, balllocationleft.center.y, balllocationleft.radius, debugimageleft);
		//if (balllocationright.radius >= 0)
		//	drawObject(balllocationright.center.x, balllocationright.center.y, balllocationright.radius, debugimageright);
		//imshow("Debug Window left", debugimageleft);
		//imshow("Debug Window right", debugimageright);

	//	searchForMovement(Pipeline.sources[0]->Gray_Buffer->current(), Pipeline.sources[0]->background_image, ballspotted);

	//	imshow("background", BG);
	//	blur(FG, FGB, Size(5, 5));
		//threshold(FGB, FGT, 50.0, 255, cv::ThresholdTypes::THRESH_BINARY);
	//	imshow("cmon", FGT);

	//	imshow("blobs", drawimage);
	//	stereoset.remap_Grey(Pipeline.sources[0]->Gray_Buffer->current(), Pipeline.sources[1]->Gray_Buffer->current());

	//	imshow("L_RGB", Pipeline.sources[0]->RGB_Buffer->current());
	//	imshow("R_RGB", Pipeline.sources[1]->RGB_Buffer->current());
	//	imshow("L_GRAY", Pipeline.sources[0]->Gray_Buffer->current());
	//	imshow("R_GRAY", Pipeline.sources[1]->Gray_Buffer->current());

		//Mat WindowL = Window(Rect(Point(100,100), Size(100, 100)), Pipeline.sources[0]->Gray_Buffer->current());
		//Mat WindowR = Window(Rect(Point(100,100), Size(100, 100)), Pipeline.sources[1]->Gray_Buffer->current());
		//	Mat dispare = stereoset.compute_Stereo(WindowL, WindowR);
	//	Mat dispare = stereoset.compute_Stereo(Pipeline.sources[0]->Gray_Buffer->current(), Pipeline.sources[1]->Gray_Buffer->current());
		//normalize(dispare, Pipeline.sources[1]->Disp_Buffer->store(), 0, 255, CV_MINMAX, CV_8U);
	//	imshow("NormDisparity", Pipeline.sources[1]->Disp_Buffer->current());

	// PIPELINE START
		//if (trackmode == 0)
		//{
		//	if ((Pipeline.detection() == 1)&&(Pipeline.objTracker.ballcount > 5))
		//		trackmode = 1; // we found the ball, now track it!
		//}
		//else if (trackmode == 1)
		//{
		//	for (int i = 0; i < Pipeline.objTracker.ballcount; i++)
		//	{
		//		Mat WindowL = Window(Rect(Pipeline.objTracker.trackerList[0][Pipeline.objTracker.ballcount].ball_location.center, Size(100, 100)), Pipeline.sources[0]->Gray_Buffer->current());
		//		Mat WindowR = Window(Rect(Pipeline.objTracker.trackerList[1][Pipeline.objTracker.ballcount].ball_location.center, Size(100, 100)), Pipeline.sources[1]->Gray_Buffer->current());
		//		Mat dispare = stereoset.compute_Stereo(WindowL, WindowR);
		//		normalize(dispare, Pipeline.sources[1]->Disp_Buffer->store(), 0, 255, CV_MINMAX, CV_8U);
		//		imwrite("NormDisparity" + to_string(i) + ".png", Pipeline.sources[1]->Disp_Buffer->current());
		//	}
		//	trackmode = 2;
		//}
		//else if (trackmode == 2) // post calculation.
		//{ // calculate trajectory using stereo and predict where ball is going to land.
		//	int a = 0;
		//}
///////////////	PIPELINE END
	// DONT DRAW ON GRAY OR RGB IMAGES BECAUSE THEY ARE SOURCE IMAGES AND DRAWING PERMANENTLY MODIFIES THE IMAGE


#ifdef OUTPUTCAP		
	output_cap.write(Pipeline.sources[0]->RGB_Buffer->current());
#endif
		// start keyboard interface
		char key = waitKey(30);
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
			imwrite("left" + to_string(calibcount) + ".png", Pipeline.sources[0]->Gray_Buffer->Rcurrent()); // left
			imwrite("right" + to_string(calibcount) + ".png", Pipeline.sources[1]->Gray_Buffer->Rcurrent()); // right
			break;
		case 's':
			imwrite("gray2.png", Pipeline.sources[0]->Gray_Buffer->Rcurrent());
			break;
		}
		// end keyboad interface
	}
	return 0;
}

