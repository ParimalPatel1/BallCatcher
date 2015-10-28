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

int keyboard; //input from keyboard
// END GLOBALS

void initblobs()
{
	//	 Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = minThreshold;
	params.maxThreshold = maxThreshold;

							  // Filter by Area.
	params.filterByArea = true;
	params.minArea = 200;
	params.maxArea = 50000;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.3;
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
	int blobsfound = 0;
	// detect!
	vector<cv::KeyPoint> keypoints;
	detector->detect(SearchImage, keypoints);
	blobsfound = keypoints.size();
	if (blobsfound > 0)
	{
		drawKeypoints(SearchImage, keypoints, movingobjects, Scalar(0, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		//imwrite("blobs" + to_string(framecount) + ".png", movingobjects);
		int largestdiameter = 0;
		for (int i = 0; i < blobsfound; i++)
		{//assume the keypoint with the largest size is the ball
			drawKeypoints(SearchImage, keypoints, movingobjects, Scalar(0, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			if (keypoints[i].size > largestdiameter)
			{
				largestdiameter = keypoints[i].size; // new largest keypoint
				ballspotted.center = Point(cvRound(keypoints[i].pt.x), cvRound(keypoints[i].pt.y));
				ballspotted.radius = cvRound(keypoints[i].size / 2); // KeyPoint::size is a diameter
				//keypoints[i].
			}
		}
		return blobsfound;
	}
	return 0; // nothing found;
}

int RoundParts(vector<Point> contours, int samplesize, int method, Mat drawimage)
{	 // amount of points at a time
	bool opencurve = false;
	if (contours.size() < samplesize) // forloop condition already provides this logic?
		return 0;
	vector<Point>;
	if (method == 1)
	{
		for (int j = 1; j < contours.size() - samplesize; j++)
		{
			vector<Point>::const_iterator first = contours.begin() + samplesize;
			//contours.
			vector<Point>::const_iterator last = contours.begin() + samplesize + j ;
			vector<Point> subsetin(first, last);
			vector<Point> subsetout;
			approxPolyDP(subsetin, subsetout, 3, opencurve);
			Moments mo = moments((Mat)subsetout);
			double area = mo.m00;
			int arcL = arcLength(subsetout, opencurve); // using contour smoothness
			double eccentricity1 = (4.0 * CV_PI * area) / (arcL * arcL);
			double eccentricity2 = ((mo.mu20 - mo.mu02)*(mo.mu20 - mo.mu02) - (4 * mo.mu11)) / ((mo.mu20 + mo.mu02)*(mo.mu20 + mo.mu02));
			double bigSqrt = sqrt((mo.m20 - mo.m02) *  (mo.m20 - mo.m02) + 4 * mo.m11 * mo.m11);
			double eccentricity3 = (double)(mo.m20 + mo.m02 + bigSqrt) / (mo.m20 + mo.m02 - bigSqrt);
		}
	}
	else if (method == 2)
	{
		//Vec4f line;
		//fitLine((cv::Mat)contours, line, CV_DIST_L2, 0, 0.01, 0.01);
		//int x0 = line[2];
		//int y0 = line[3];
		//int x1 = x0 - 200 * line[0];
		//int y1 = y0 - 200 * line[1];
		//cv::borderInterpolate()
		//int arcL = arcLength(subsetout, opencurve); // using contour smoothness
	//	vector<RotatedRect>;
		RotatedRect tempellipse;
		for (int j = 1; j < contours.size() - samplesize - 1; j += samplesize)
		{
		//	vector<Point>::const_iterator first = contours.begin() + j;
		//	vector<Point>::const_iterator last = contours.begin() + samplesize + j;
		//	vector<Point> subsetin(first, last);
			//vector<Point> subsetout;
			//tempellipse = fitEllipse(subsetin);
			//ellipse(drawimage, tempellipse, Scalar(255, 255, 0));
			Circle a(contours[j], contours[j+1], contours[j+2]);
			circle(drawimage, a.GetCenter(), a.GetRadius(), Scalar(255, 255, 0));

			//cv::solve(subsetin, subsetout, , );
			//polyfit();
			//polylines(drawimage, subsetout, opencurve, Scalar(j * 10, j * 10, j * 10));
			//atan((y2 - y1) / (x2 - x1)) // 1st derative between two points x1,y1, x2,y2.
			//approxPolyDP(subsetin, subsetout, 3, opencurve);
		//	(contours[j+1].y - contours[j].y)/(contours[j + 1].x - contours[j].x);

			//Mat G, dG, ddG, dX, dY;
			//int width = 3;
			//int sigma = 2;
			//transpose(getGaussianKernel(width, sigma, CV_64FC1), G);
			////filter2D(X, Xsmooth, X.depth(), G); // only for visualization 
			////filter2D(Y, Ysmooth, Y.depth(), G);
	
			//// 1st and 2nd deratives
			//Sobel(G, dG, G.depth(), 1, 0);
			//Sobel(G, ddG, G.depth(), 2, 0);
			//flip(dG, dG, 0);
			//flip(ddG, ddG, 0);
			//Point anchor(dG.cols - 1, dG.rows – 1);
			//anchor.x
			//
			//filter2D(X, dX, X.depth(), dG, anchor);
			//filter2D(Y, dY, Y.depth(), dG, anchor);
			//filter2D(X, ddX, X.depth(), ddG, anchor);
			//filter2D(Y, ddY, Y.depth(), ddG, anchor);

		}
	}
	return 0;
}
 // finds moving objecs in an image
int searchForMovement(Mat CurrGraySource, Mat PrevGraySource, ballLoc &ballspotted) {
	Mat differenceImage; // grayscale
	Mat thresholdImage; // binary
	Mat blurImage;		// grayscale
	//perform frame differencing with the sequential images. 
	absdiff(PrevGraySource, CurrGraySource, differenceImage); // order matters here
//	imshow("diff", differenceImage);
	//threshold intensity image at a given sensitivity value
	threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
	//blur the image to get rid of the noise. This will output an intensity image
	blur(thresholdImage, blurImage, Size(BLUR_SIZE, BLUR_SIZE));
	//threshold again to obtain binary image from blur output
	threshold(blurImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
	imshow("threshold", thresholdImage);

	//int erosion_size = 2;
	//Mat element = getStructuringElement(MORPH_ELLIPSE,
	//	Size(2 * erosion_size + 1, 2 * erosion_size + 1),
	//	Point(erosion_size, erosion_size));
	//Mat erosion;
	//Mat dilation;
	//erode(thresholdImage, erosion, element);
	//dilate(erosion, dilation, element);
	//imshow("opening", dilation);

	//dilate(thresholdImage, dilation, element);
	//erode(dilation, erosion, element);
	//imshow("closing", erosion);

	Mat ContourImage;
	thresholdImage.copyTo(ContourImage);
	//thresholdImage.copyTo(ContourImage);
	//dilation.copyTo(ContourImage);
	bool objectDetected = false;
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
	//findContours(ContourImage,contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);// retrieves all contours
	findContours(ContourImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours, CHANGES THRESHOLD IMAGE

	//if contours vector is not empty, we have found some  moving objects
	if (contours.size() > 0)
	{
		//vector<vector<Vec4i>> Defects(contours.size()); 
		//vector<vector<Point>> Hull(contours.size());
		vector<vector<Point> > contours_poly(contours.size());
		vector<Point2f>center(contours.size());
		vector<float>radius(contours.size());


		double refArea = 0;
		bool objectFound = false;
		Mat drawimage;
		//CurrGraySource.copyTo(drawimage);
		cvtColor(CurrGraySource, drawimage, CV_GRAY2RGB);
		RNG rng(12345);
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		int MIN_OBJECT_AREA = 200; // resolution dependant or independat?
		int MAX_OBJECT_AREA = 30000;//thresholdImage.rows*thresholdImage.cols / 1.5; // 3/2 area of image

		//vector<Vec4i> lines;
		//HoughLinesP(thresholdImage, lines, 1, CV_PI / 180, 80, 30, 10);
		//for (size_t i = 0; i < lines.size(); i++)
		//{
		//	float rho = lines[i][0];
		//	float theta = lines[i][1];
		//	if 

		//	line(drawimage, Point(lines[i][0], lines[i][1]),
		//		Point(lines[i][2], lines[i][3]), Scalar(255), 3, 8);
		//}
		for (int i = contours.size() - 1; i >= 0; i--) // start at the largest contour, work way down.
		{
		//	vector<Vec4f> partialLines;
			//fitLine(contours[i], partialLines, cv::DistanceTypes::DIST_L1, 0, 0.01, 0.01);
		//	polylines(drawimage, contours[i], true, Scalar(255));
			Moments mo = moments((Mat)contours[i]);
			double area = mo.m00;
			if (area > MIN_OBJECT_AREA)// && area<MAX_OBJECT_AREA && area>refArea)
			{ // three methods for calculating eccentricity

				//RoundParts(contours[i], 3, 2, drawimage);
				RotatedRect tempellipse;
				calcEccentricity(contours[i], tempellipse);
				ellipse(drawimage, tempellipse, Scalar(255), 2);
				waitKey(10);


			//	approxPolyDP((Mat)contours[i], contours_poly[i], 3, true);
			//	minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
			//	ballspotted.center = Point(cvRound(center[i].x), cvRound(center[i].y));
			//	ballspotted.radius = cvRound((int)radius[i]); // KeyPoint::size is a diameter
				//circle(drawimage, ballspotted.center, ballspotted.radius, Scalar(255));
				//	putText(drawimage, "R: " + to_string(ballspotted.radius), Point(20, 40), FONT_HERSHEY_SCRIPT_SIMPLEX, 1, Scalar(255), 2);
				//drawContours(drawimage, contours, i, Scalar(255), 2, 8, hierarchy, 0, Point());
			}
		}
		imshow("drawimage", drawimage);
		waitKey(20);
		return 0; // no balls found
	}
}

//	//	drawContours(drawimage, Hull, i, CV_RGB(255, 255, 255), 2, 8, hierarchy, 0, Point());
//	//	objectFound = true;
//		//refArea = area;
//		//Mat SearchWindow = Window(Rect(Point2f(moment.m10 / moment.m00, moment.m01 / moment.m00), Size(200, 200)), CurrGraySource);
//		//if (searchForBalls(CurrGraySource, drawimage, ballspotted) >= 1) // SearchWindow
//		//{
//		//	imshow("Draw", drawimage);
//		//	return 1;
//		//}

//	}
//	else objectFound = false;
//}
////RotatedRect fittedEllipse = fitEllipse(contours[i]);
////		convexHull((Mat)contours[i], Hull[i], false, false);
//	//	convexityDefects((Mat)contours[i], Hull[i], Defects[i]);

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
//	cvUseOptimized(1); // how to check we are using IPP calls?
	//pause and resume code
	bool pause = false;
	
	Point textcenter1(100, 50); // text variables start
	Point textcenter2(100, 100);
	Point textcenter3(100, 150);
	int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontScale = 1;
	int thickness = 2; // text variables end

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
	int capids[] = { 1 };//, 702 };
	string config = "does nothing"; 	// make config string meaningful
	string videosource = videopath;
	Stereotime stereoset;// intergrate with pipeline
	stereoset.Stereo_Init(RESOLUTION);
	initblobs();
	createTrackbars();
	createTrackbars_Blob();
	ImagePipeline Pipeline(1, capids, &config, searchForMovement, searchForBalls); // init with webcams source
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
	 Ptr<BackgroundSubtractor> BGS = createBackgroundSubtractorMOG2();
	 Mat FG;// foreground
	 Mat FGT;
	 Mat FGB;
	 Mat BG; // background
	 for (int i = 0; i < 100; i++)
	 {
		 Pipeline.cap(); // initial frame for each buffer so previous frame is available on first loop
		 Pipeline.RGBtoGray();
		 BGS->apply(Pipeline.sources[0]->Gray_Buffer->current(), FG, 0.8);
	 }
	 BGS->getBackgroundImage(Pipeline.sources[0]->background_image);
	// stereoset.remap_Grey(Pipeline.sources[0]->Gray_Buffer->current(), Pipeline.sources[1]->Gray_Buffer->current());
	// Pipeline.sources[0]->Gray_Buffer->current().copyTo(Pipeline.sources[0]->background_image);
	// Pipeline.sources[1]->Gray_Buffer->current().copyTo(Pipeline.sources[1]->background_image);

	////////// MAIN LOOP START
	while (1) { // capture and process loop
		fps = 1000.0/(float)(1 + clock() - last_time); // time stuff
		last_time = clock();
		cout << "FPS: " << fps << endl; // faster than draw??
		framecount++; 

		Pipeline.cap();
		Pipeline.RGBtoGray();

	//	imshow("background", BG);
	//	blur(FG, FGB, Size(5, 5));
		//threshold(FGB, FGT, 50.0, 255, cv::ThresholdTypes::THRESH_BINARY);
	//	imshow("cmon", FGT);
	//	searchForMovement(Pipeline.sources[0]->Gray_Buffer->current(), Pipeline.sources[0]->background_image, ballspotted);
	//	searchForBalls(Pipeline.sources[0]->Gray_Buffer->current(), drawimage, ayy);
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


	//int kernel_size = 31;
	//double sig = 1, th = 0, lm = 1.0, gm = 0.02, ps = 0;

	//Mat StartImage = imread("TestImage.jpg", CV_LOAD_IMAGE_COLOR);
	//Mat InputImage;
	//Mat GaborImage;
	//StartImage.convertTo(InputImage, CV_32F);

	//cv::Mat kernel = cv::getGaborKernel(cv::Size(kernel_size, kernel_size), sig, th, lm, gm, ps);
	//cv::filter2D(InputImage, GaborImage, CV_32F, kernel);

	//// Load Image
	//Mat StartImage;
	//Mat GaborImage;
	//Mat InputImage;
	//Mat DisplayImage;
	//// Convert Image type
	//Pipeline.sources[0]->RGB_Buffer->current().convertTo(InputImage, CV_32F);

	//// Create the kernel
	//int kernel_size = 31;
	//double sig = 1, th = 0, lm = 1.0, gm = 0.02, ps = 0;
	//cv::Mat kernel = cv::getGaborKernel(cv::Size(kernel_size, kernel_size), sig, th, lm, gm, ps);
	//// Apply to the Image
	//cv::filter2D(Pipeline.sources[0]->RGB_Buffer->current(), GaborImage, CV_32F, kernel);
	//
	//// convert resulting image back so it can be viewed
	//GaborImage.convertTo(DisplayImage, CV_8U, 1.0 / 255.0);
	//imshow("GaborImage", DisplayImage);
	//waitKey(50);
#ifdef OUTPUTCAP		
	output_cap.write(Pipeline.sources[0]->RGB_Buffer->current());
#endif
		// start keyboard interface
		char key = waitKey(20);
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
			imwrite("left" + to_string(calibcount) + ".png", Pipeline.sources[0]->Gray_Buffer->current()); // left
			imwrite("right" + to_string(calibcount) + ".png", Pipeline.sources[1]->Gray_Buffer->current()); // right
			break;
		}
		// end keyboad interface
	}
	return 0;
}

