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
#include <Windows.h>
#include <ctime>
//#include <opencv2/photo.hpp>
//#include "opencv2/imgcodecs.hpp"
//#include <opencv2/highgui.hpp>
//#include  <opencv2\highgui\highgui_c.h>
//#include  <opencv2\core\core.hpp>

//#include <string>


using namespace cv;
using namespace std;

//our sensitivity value to be used in the absdiff() function
const static int SENSITIVITY_VALUE = 20;
//size of blur used to smooth the intensity image output from absdiff() function
const static int BLUR_SIZE = 10;
//we'll have just one object to search for
//and keep track of its position.
int theObject[2] = { 0,0 };
Mat TrackerImage;
//bounding rectangle of the object, we will use the center of this as its position.
Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
vector<int> compression_params;
int imagecount = 0;
//int to string helper function
string intToString(int number) {

	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void searchForMovement(Mat &thresholdImage, Mat &cameraFeed) {
	//notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
	//to take the values passed into the function and manipulate them, rather than just working with a copy.
	//eg. we draw to the cameraFeed to be displayed in the main() function.
	bool objectDetected = false;
	//Mat temp;
	//thresholdImage.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	findContours(thresholdImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours

																					  //if contours vector is not empty, we have found some objects
	if (contours.size() > 0)objectDetected = true;
	else objectDetected = false;

	if (objectDetected) {
		imagecount++;
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size() - 1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		objectBoundingRectangle = boundingRect(largestContourVec.at(0));
		int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
		int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;

		//update the objects positions by changing the 'theObject' array values
		theObject[0] = xpos, theObject[1] = ypos;

		int x = theObject[0];
		int y = theObject[1];
		vector<Vec3f> circles;

	/*	HoughCircles(cameraFeed, circles, CV_HOUGH_GRADIENT, 2, 170, 200, 100, 5, 200);
		/// Draw the circles detected
		for (size_t i = 0; i < circles.size(); i++)
		{
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			// circle center
			circle(cameraFeed, center, 3, Scalar(0, 255, 0), -1, 8, 0);
			// circle outline
			circle(cameraFeed, center, radius, Scalar(0, 0, 255), 3, 8, 0);
		}
		*/
		//draw some crosshairs around the object
		circle(TrackerImage, Point(x, y), 20, Scalar(0, 255, 0), 2);
		line(TrackerImage, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
		line(TrackerImage, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
		line(TrackerImage, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
		line(TrackerImage, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);

		//write the position of the object to the screen
		putText(TrackerImage, "Tracking object at (" + intToString(x) + "," + intToString(y) + ")", Point(x, y), 1, 1, Scalar(255, 0, 0), 2);
		
		imshow("TrackerImage", TrackerImage);
			imwrite("trackingH"+intToString(imagecount)+".png", cameraFeed, compression_params);
	}
	//make some temp x and y variables so we dont have to type out so much
}
int main() {

	//some boolean variables for added functionality
	bool objectDetected = false;
	//these two can be toggled by pressing 'd' or 't'
	bool debugMode = false;
	bool trackingEnabled = false;
	//pause and resume code
	bool pause = false;
	//set up the matrices that we will need
	//the two frames we will be comparing
	//Mat frame1, frame2;
	Mat* frame1; // current frame, consider renaming?
	Mat* frame2; // previous frame
	//their grayscale images (needed for absdiff() function)
	Mat* grayImage1;
	Mat* grayImage2;
	//resulting difference image
	Mat differenceImage;
	//thresholded difference image (for use in findContours() function)
	Mat* thresholdImage = &Mat();
	//video capture object.
	VideoCapture capture;
	
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

	Mat image_array[2];
	Mat gray_array[2];

	VideoWriter outputVideo;

	const string filepath = "\test.avi"; //C:\Users\Benjamin\Desktop
	VideoWriter output_cap;



	namedWindow("Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);
	capture.open(701); // 700 -> directx + index
//	Sleep(500);
	if (!capture.isOpened()) {
		cout << "ERROR ACQUIRING VIDEO FEED\n";
		getchar();
		return -1;
	}
	

	capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 640); // 1280 for intergrated webcam, 1080 for external webcam
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	capture.set(CV_CAP_PROP_FPS, 30);
	capture.set(CV_CAP_PROP_EXPOSURE, 99);
	
	int toggle = 0;
	while(!capture.read(image_array[toggle]));
	while(!capture.read(image_array[toggle]));
	cvtColor(image_array[toggle], gray_array[toggle], COLOR_BGR2GRAY);
	image_array[toggle].copyTo(TrackerImage);
//	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//	compression_params.push_back(9);
	//int ex = static_cast<int>(capture.get(CV_CAP_PROP_FOURCC));
	//char EXT[] = { ex & 0XFF , (ex & 0XFF00) >> 8,(ex & 0XFF0000) >> 16,(ex & 0XFF000000) >> 24, 0 };
/*	 int debug = output_cap.open(filepath,
		 -1, //YUYV
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
	 */
	
	


//#pragma loop(hint_parallel(8))
	while (1) {
		if (toggle == 0) // pointer manipulation for circular buffer
		{
			frame2 = &image_array[0];
			frame1 = &image_array[1];
			grayImage2 = &gray_array[0];
			grayImage1 = &gray_array[1];
			toggle = 1;
		}
		else
		{
			frame2 = &image_array[1];
			frame1 = &image_array[0];
			grayImage2 = &gray_array[1];
			grayImage1 = &gray_array[0];
			toggle = 0;
		}
		capture.read(image_array[toggle]);
		
		cvtColor(image_array[toggle], gray_array[toggle], COLOR_BGR2GRAY);
		GaussianBlur(*grayImage1, *grayImage1, Size(9, 9), 2, 2);
		//perform frame differencing with the sequential images. 
		absdiff(*grayImage1, *grayImage2, differenceImage);
		//threshold intensity image at a given sensitivity value
		threshold(differenceImage, *thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
		//blur the image to get rid of the noise. This will output an intensity image
		blur(*thresholdImage, *thresholdImage, Size(BLUR_SIZE, BLUR_SIZE));
	//	blur(differenceImage, *thresholdImage, Size(BLUR_SIZE, BLUR_SIZE));
		//threshold again to obtain binary image from blur output
		threshold(*thresholdImage, *thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);

		searchForMovement(*thresholdImage, *grayImage1);




		/// Show your results


			
		//if tracking enabled, search for contours in our thresholded image
	//	if (trackingEnabled) {
			//output_cap.write(*frame1);

		fps = 1000 / (1 + clock() - last_time);
		cout << "FPS: " << fps << endl; // faster than draw??

	
		//putText(*grayImage1, "FPS: " + to_string(fps), textcenter3, fontFace, fontScale, Scalar::all(255), thickness, 5); // DONT DRAW ON GRAY OR RGB IMAGES BECAUSE THEY ARE STORED FOR NEXT CYCLE
		imshow("ThresholdImage", *thresholdImage);
		//show our captured frame
		imshow("Frame1", *frame1);
		
		//check to see if a button has been pressed.
		//this 10ms delay is necessary for proper operation of this program
		//if removed, frames will not have enough time to referesh and a blank 
		//image will appear.
		char key = waitKey(1);
		switch (key) {

		case 'q': //'esc' key has been pressed, exit program.
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
						//a switch statement inside a switch statement? Mind blown.
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
			image_array[toggle].copyTo(TrackerImage);
			break;
		}
		last_time = clock();
	}
	//release the capture before re-opening and looping again.
//	capture.release();
//	output_cap.release();

	return 0;

}