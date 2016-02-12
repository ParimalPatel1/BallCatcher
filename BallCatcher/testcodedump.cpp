// int erosion_size = 2;
// Mat element = getStructuringElement(MORPH_ELLIPSE,
//	Size(2 * erosion_size + 1, 2 * erosion_size + 1),
//	Point(erosion_size, erosion_size));
// Mat erosion;
// Mat dilation;
// erode(thresholdImage, erosion, element);
// dilate(erosion, dilation, element);
// imshow("opening", dilation);

// dilate(thresholdImage, dilation, element);
// erode(dilation, erosion, element);
// imshow("closing", erosion);

// int kernel_size = 31;
// double sig = 1, th = 0, lm = 1.0, gm = 0.02, ps = 0;

// Mat StartImage = imread("TestImage.jpg", CV_LOAD_IMAGE_COLOR);
// Mat InputImage;
// Mat GaborImage;
// StartImage.convertTo(InputImage, CV_32F);

// cv::Mat kernel = cv::getGaborKernel(cv::Size(kernel_size, kernel_size), sig, th, lm, gm, ps);
// cv::filter2D(InputImage, GaborImage, CV_32F, kernel);

//// Load Image
// Mat StartImage;
// Mat GaborImage;
// Mat InputImage;
// Mat DisplayImage;
//// Convert Image type
// Pipeline.sources[0]->RGB_Buffer->current().convertTo(InputImage, CV_32F);

//// Create the kernel
// int kernel_size = 31;
// double sig = 1, th = 0, lm = 1.0, gm = 0.02, ps = 0;
// cv::Mat kernel = cv::getGaborKernel(cv::Size(kernel_size, kernel_size), sig, th, lm, gm, ps);
//// Apply to the Image
// cv::filter2D(Pipeline.sources[0]->RGB_Buffer->current(), GaborImage, CV_32F, kernel);
//
//// convert resulting image back so it can be viewed
// GaborImage.convertTo(DisplayImage, CV_8U, 1.0 / 255.0);
// imshow("GaborImage", DisplayImage);
// waitKey(50);

// void MyCallbackForContrast(int iValueForContrast, void *userData)
//{
//	Mat dst;
//	int iValueForBrightness = *(static_cast<int*>(userData));
//	//Calculating brightness and contrast value
//	int iBrightness = iValueForBrightness - 50;
//	double dContrast = iValueForContrast / 50.0;
//	//Calculated contrast and brightness value
//	cout << "MyCallbackForContrast : Contrast=" << dContrast << ", Brightness=" << iBrightness << endl;
//	//adjust the brightness and contrast
//	//src.convertTo(dst, -1, dContrast, iBrightness);
//	//show the brightness and contrast adjusted image
//	imshow("My Window", dst);
//}

//	//	drawContours(drawimage, Hull, i, CV_RGB(255, 255, 255), 2, 8, hierarchy, 0, Point());
//	//	objectFound = true;
//		//refArea = area;
//		//Mat SearchWindow = Window(Rect(Point2f(moment.m10 / moment.m00, moment.m01 / moment.m00), Size(200,
//200)), CurrGraySource);
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

// objectBoundingRectangle = boundingRect(contours.at(i));//largestContourVec.at(0));
// if (objectBoundingRectangle.area() > 0 && objectBoundingRectangle.area() < 50000) // SIZE CHECK DEPENDANT ON
// RESOLUTION
//{
//	int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
//	int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;
//	int foundcircle = 0;
//	objectBoundingRectangle.width = objectBoundingRectangle.width + 20;
//	objectBoundingRectangle.height = objectBoundingRectangle.height + 20;
//	movingobjects = Window(objectBoundingRectangle, CurrGraySource);
//	movingcount++;
//	imwrite("testimage" + to_string(movingcount) + ".png", movingobjects);
//	//movingobjects = Window(Point(xpos, ypos), CurrGraySource, Size(objectBoundingRectangle.width,
//objectBoundingRectangle.height));
//	foundcircle = searchForCircles(movingobjects, movingobjects);

//	approxPolyDP((Mat)contours[i], contours_poly[i], 3, true);
//	minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
//	ballspotted.center = Point(cvRound(center[i].x), cvRound(center[i].y));
//	ballspotted.radius = cvRound((int)radius[i]); // KeyPoint::size is a diameter
// circle(drawimage, ballspotted.center, ballspotted.radius, Scalar(255));
//	putText(drawimage, "R: " + to_string(ballspotted.radius), Point(20, 40), FONT_HERSHEY_SCRIPT_SIMPLEX, 1,
//Scalar(255), 2);
// drawContours(drawimage, contours, i, Scalar(255), 2, 8, hierarchy, 0, Point());

// vector<Vec4i> lines;
// HoughLinesP(thresholdImage, lines, 1, CV_PI / 180, 80, 30, 10);
// for (size_t i = 0; i < lines.size(); i++)
//{
//	float rho = lines[i][0];
//	float theta = lines[i][1];
//	if

//	line(drawimage, Point(lines[i][0], lines[i][1]),
//		Point(lines[i][2], lines[i][3]), Scalar(255), 3, 8);
//}

//	vector<Vec4f> partialLines;
// fitLine(contours[i], partialLines, cv::DistanceTypes::DIST_L1, 0, 0.01, 0.01);
//	polylines(drawimage, contours[i], true, Scalar(255));

// RoundParts(contours[i], 3, 2, drawimage);

// vector<vector<Vec4i>> Defects(contours.size());
// vector<vector<Point>> Hull(contours.size());
// vector<vector<Point> > contours_poly(contours.size());

// RNG rng(12345);
// Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

// int RoundParts(vector<Point> contours, int samplesize, int method, Mat drawimage)
//{	 // amount of points at a time
//	bool opencurve = false;
//	if (contours.size() < samplesize) // forloop condition already provides this logic?
//		return 0;
//	vector<Point>;
//	if (method == 1)
//	{
//		for (int j = 1; j < contours.size() - samplesize; j++)
//		{
//			vector<Point>::const_iterator first = contours.begin() + samplesize;
//			//contours.
//			vector<Point>::const_iterator last = contours.begin() + samplesize + j;
//			vector<Point> subsetin(first, last);
//			vector<Point> subsetout;
//			approxPolyDP(subsetin, subsetout, 3, opencurve);
//			Moments mo = moments((Mat)subsetout);
//			double area = mo.m00;
//			int arcL = arcLength(subsetout, opencurve); // using contour smoothness
//			double eccentricity1 = (4.0 * CV_PI * area) / (arcL * arcL);
//			double eccentricity2 = ((mo.mu20 - mo.mu02)*(mo.mu20 - mo.mu02) - (4 * mo.mu11)) / ((mo.mu20 +
//mo.mu02)*(mo.mu20 + mo.mu02));
//			double bigSqrt = sqrt((mo.m20 - mo.m02) *  (mo.m20 - mo.m02) + 4 * mo.m11 * mo.m11);
//			double eccentricity3 = (double)(mo.m20 + mo.m02 + bigSqrt) / (mo.m20 + mo.m02 - bigSqrt);
//		}
//	}
//	else if (method == 2)
//	{
// Vec4f line;
// fitLine((cv::Mat)contours, line, CV_DIST_L2, 0, 0.01, 0.01);
// int x0 = line[2];
// int y0 = line[3];
// int x1 = x0 - 200 * line[0];
// int y1 = y0 - 200 * line[1];
// cv::borderInterpolate()
// int arcL = arcLength(subsetout, opencurve); // using contour smoothness
//	vector<RotatedRect>;
// RotatedRect tempellipse;
// for (int j = 1; j < contours.size() - samplesize - 1; j += samplesize)
//{
//	vector<Point>::const_iterator first = contours.begin() + j;
//	vector<Point>::const_iterator last = contours.begin() + samplesize + j;
//	vector<Point> subsetin(first, last);
// vector<Point> subsetout;
// tempellipse = fitEllipse(subsetin);
// ellipse(drawimage, tempellipse, Scalar(255, 255, 0));
// Circle a(contours[j], contours[j + 1], contours[j + 2]);
// circle(drawimage, a.GetCenter(), a.GetRadius(), Scalar(255, 255, 0));

// cv::solve(subsetin, subsetout, , );
// polyfit();
// polylines(drawimage, subsetout, opencurve, Scalar(j * 10, j * 10, j * 10));
// atan((y2 - y1) / (x2 - x1)) // 1st derative between two points x1,y1, x2,y2.
// approxPolyDP(subsetin, subsetout, 3, opencurve);
//	(contours[j+1].y - contours[j].y)/(contours[j + 1].x - contours[j].x);

// Mat G, dG, ddG, dX, dY;
// int width = 3;
// int sigma = 2;
// transpose(getGaussianKernel(width, sigma, CV_64FC1), G);
////filter2D(X, Xsmooth, X.depth(), G); // only for visualization
////filter2D(Y, Ysmooth, Y.depth(), G);

//// 1st and 2nd deratives
// Sobel(G, dG, G.depth(), 1, 0);
// Sobel(G, ddG, G.depth(), 2, 0);
// flip(dG, dG, 0);
// flip(ddG, ddG, 0);
// Point anchor(dG.cols - 1, dG.rows – 1);
// anchor.x
//
// filter2D(X, dX, X.depth(), dG, anchor);
// filter2D(Y, dY, Y.depth(), dG, anchor);
// filter2D(X, ddX, X.depth(), ddG, anchor);
// filter2D(Y, ddY, Y.depth(), ddG, anchor);
//
//		}
//	}
//	return 0;
//}

// bool largerdiameter(KeyPoint a, KeyPoint b)
//{
//	return (a.size < b.size);
//}
//// Search Image should be grayscale, because Houghcircles needs it to be in this format.
// ballLoc searchForBalls(Mat &SearchImage) {
//	// detect!
//	ballLoc largest_ball;
//	vector<cv::KeyPoint> ball_canidates;
//	Mat tempimage;
//	SearchImage.copyTo(tempimage);
//	cvtColor(SearchImage, tempimage, cv::COLOR_BGR2GRAY);
//	detector->detect(tempimage, ball_canidates);
//
//	if (ball_canidates.size() > 0)
//	{
//		int i = ball_canidates.size() - 1;
//		cv::putText(SearchImage, std::to_string(i), textcenter1, fontFace, fontScale, cv::Scalar(0, 0, 0),
//thickness);
//
//		drawKeypoints(tempimage, ball_canidates, SearchImage, cv::Scalar(0, 0, 0),
//DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//		sort(ball_canidates.begin(), ball_canidates.end(), largerdiameter); // sort by diameter
//		largest_ball.center = Point(cvRound(ball_canidates[i].pt.x), cvRound(ball_canidates[i].pt.y));
//		largest_ball.radius = cvRound(ball_canidates[i].size / 2); // KeyPoint::size is a diameter
//		return largest_ball;
//	}
//	largest_ball.center = Point(-1, -1);
//	largest_ball.radius = -1;
//	return largest_ball; // nothing found;
//}
//
//
// bool morePoints(vector<Point> a, vector<Point> b)
//{
//	return (a.size() < b.size());
//}
//// finds moving objecs in an image
// int const max_lowThreshold = 100;
// int lowThreshold = 50;
// int ratio = 3;
// int kernel_size = 3;
// int searchForMovement(Mat CurrGraySource, Mat PrevGraySource, ballLoc &ballspotted) {
//	Mat differenceImage; // grayscale
//	Mat thresholdImage; // binary
//	Mat blurImage;		// grayscale
//	Mat edges;			//binary
//
//	Mat drawimage;
//	CurrGraySource.copyTo(drawimage);
//	cvtColor(CurrGraySource, drawimage, CV_GRAY2RGB);
//
//	//perform frame differencing with the sequential images.
//	absdiff(PrevGraySource, CurrGraySource, differenceImage); // order matters here
//															  //	imshow("diff",
//differenceImage);
//															  //threshold intensity
//image at a given sensitivity value
//	threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
//	//blur the image to get rid of the noise. This will output an intensity image
//	blur(thresholdImage, blurImage, Size(BLUR_SIZE, BLUR_SIZE));
//	//threshold again to obtain binary image from blur output
//	threshold(blurImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);

//
//	cv::Canny(CurrGraySource, edges, lowThreshold, lowThreshold*ratio, kernel_size);
// imwrite("backround2.png", thresholdImage);
// imwrite("edges2.png", edges);
// vector<Vec2f> lines;
// HoughLines(edges, lines, 1, CV_PI / 180, 100, 0, 0);

// for (size_t i = 0; i < lines.size(); i++)
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
// cv::imshow("edges", edges);
// cv::imshow("lines", drawimage);
// imshow("threshold", thresholdImage);
// waitKey(10);

//	Mat ContourImage;
//	edges.copyTo(ContourImage);
// thresholdImage.copyTo(ContourImage);
// bool objectDetected = false;
// these two vectors needed for output of findContours
// vector< vector<Point> > contours;
// vector<Vec4i> hierarchy;
////Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
////findContours(ContourImage,contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);// retrieves all contours
// findContours(ContourImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external
// contours, CHANGES THRESHOLD IMAGE

////if contours vector is not empty, we have found some  moving objects
// if (contours.size() > 0)
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
//	return 0;
//}