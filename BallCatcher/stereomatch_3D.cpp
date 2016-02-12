// StereoMatch_3D
// Nicolas Rosa, April 2015.

#include "stereomatch_3D_lib.h"

//#define OPENCV_OLD
#define OPENCV_NEW

//#define RESOLUTION_320x240
#define RESOLUTION_640x480
//#define RESOLUTION_1280x720
#define CALIBRATION_ON

using namespace cv;
using namespace std;

// Global Variables
const string trackbarWindowName = "Trackbars";
// Gaussian Filter, Erosion and Dilation to take out spurious noise
Mat element_erode = getStructuringElement(MORPH_RECT,
                                          Size(2 * EROSION_SIZE + 1, 2 * EROSION_SIZE + 1),
                                          Point(EROSION_SIZE, EROSION_SIZE));
Mat element_dilate =
    getStructuringElement(MORPH_RECT, Size(2 * DILATE_SIZE + 1, 2 * DILATE_SIZE + 1), Point(DILATE_SIZE, DILATE_SIZE));
Size element_blur = Size(2 * BLUR_SIZE + 1, 2 * BLUR_SIZE + 1);

// Initial value and MAX StereoBM Parameters. These will be changed using trackbars
#ifndef RESOLUTION_640x480
// Default
int preFilterSize = 0;
const int preFilterSize_MAX = 100;
int preFilterCap = 0;
const int preFilterCap_MAX = 100;
int SADWindowSize = 0;
const int SADWindowSize_MAX = 100;
int minDisparity = 50;
const int minDisparity_MAX = 100;
int numberOfDisparities = 1;
const int numberOfDisparities_MAX = 16;
int textureThreshold = 0;
const int textureThreshold_MAX = 100;
int uniquenessRatio = 0;
const int uniquenessRatio_MAX = 100;
int speckleWindowSize = 0;
const int speckleWindowSize_MAX = 100;
int speckleRange = 0;
const int speckleRange_MAX = 100;
int disp12MaxDiff = 1;
const int disp12MaxDiff_MAX = 1;
#endif

#ifdef RESOLUTION_640x480
#ifndef CALIBRATION_ON
// 640x480 	- 50 50 21 50 2 77 0  0   0 1
int preFilterSize = 50;
const int preFilterSize_MAX = 100;
int preFilterCap = 50;
const int preFilterCap_MAX = 100;
int SADWindowSize = 21;
const int SADWindowSize_MAX = 100;
int minDisparity = 50;
const int minDisparity_MAX = 100;
int numberOfDisparities = 2;
const int numberOfDisparities_MAX = 16;
int textureThreshold = 77;
const int textureThreshold_MAX = 100;
int uniquenessRatio = 0;
const int uniquenessRatio_MAX = 100;
int speckleWindowSize = 0;
const int speckleWindowSize_MAX = 100;
int speckleRange = 0;
const int speckleRange_MAX = 100;
int disp12MaxDiff = 1;
const int disp12MaxDiff_MAX = 1;
#endif
#ifdef CALIBRATION_ON
// 640x480 	- 50 50 21 52 4 0 0  0   0 1
int preFilterSize = 50;
const int preFilterSize_MAX = 100;
int preFilterCap = 46;
const int preFilterCap_MAX = 100;
int SADWindowSize = 17;
const int SADWindowSize_MAX = 100;
int minDisparity = 53;
const int minDisparity_MAX = 100;
int numberOfDisparities = 5;
const int numberOfDisparities_MAX = 16;
int textureThreshold = 20;
const int textureThreshold_MAX = 100;
int uniquenessRatio = 0;
const int uniquenessRatio_MAX = 100;
int speckleWindowSize = 21;
const int speckleWindowSize_MAX = 100;
int speckleRange = 100;
const int speckleRange_MAX = 100;
int disp12MaxDiff = 1;
const int disp12MaxDiff_MAX = 1;
#endif
#endif

// initial min and max HSV filter values.
// these will be changed using trackbars
int H_MIN = 41;
int H_MAX = 189;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 76;
int V_MAX = 193;
// default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
// max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
// minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

// Scope
void create_windows();
void createTrackbars_sv();
void on_trackbar(int, void*)
{
} // This function gets called whenever a trackbar position is changed
void resize_frame(Mat* frame1, Mat* frame2);
// void change_resolution(VideoCapture* capL,VideoCapture* capR);
void apply_Sobel(Mat& input, Mat& output);
void apply_Tracking_Object(Mat& input, Mat& camerafeed);
void morphOps(Mat& thresh);
void drawObject(int x, int y, Mat& frame);
void trackFilteredObject(int& x, int& y, Mat threshold, Mat& cameraFeed);
void readCalibFiles(Mat& M1, Mat& D1, Mat& M2, Mat& D2, Mat& R, Mat& T);
void contrast_and_brightness(Mat& left, Mat& right, float alpha, float beta);

string intToString(int number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

int main(int, char**)
{
    // Matrix to store each left and right frame of the video_l.avi and video_r.avi
    Mat imageL, imageR, imageL_grey, imageR_grey;
    Mat res, res_bgr;
    Rect roi1, roi2;
    Mat Q;
    Mat sobel;
    Mat segmented;
    int temp[10];
    int frame_counter = 0;
    int input_num = 0;
    float scale = 1.f;
    char key = 0;
    int show_input_image = 1, show_stereo_parameters = 0;
    const char* imageL_filename = 0;
    const char* imageR_filename = 0;

#ifdef OPENCV_OLD
    StereoBM bm;
#endif

#ifdef OPENCV_NEW
    Ptr<StereoBM> bm = StereoBM::create(16, 9);
#endif

    // Create an object that decodes the input video stream.
    printf("Video Number(1,2,3,4,5): ");
    scanf("%d", &input_num);
    switch(input_num) {
    case 1:
        imageL_filename = "../data/left/video2_denoised.avi";
        imageR_filename = "../data/right/video2_denoised.avi";
        cout << "video2_denoised.avi" << endl;
        break;
    case 2:
        imageL_filename = "../data/left/video0.avi";
        imageR_filename = "../data/right/video0.avi";
        cout << "video0.avi" << endl;
        break;
    case 3:
        imageL_filename = "../data/left/video1.avi";
        imageR_filename = "../data/right/video1.avi";
        cout << "video1.avi" << endl;
        break;
    case 4:
        imageL_filename = "../data/left/video2_noised.avi";
        imageR_filename = "../data/right/video2_noised.avi";
        cout << "video2_noised.avi" << endl;
        break;
    case 5:
        imageL_filename = "../data/left/20004.avi";
        imageR_filename = "../data/right/30004.avi";
        break;
    }

    VideoCapture capL(imageL_filename);
    VideoCapture capR(imageR_filename);

    if(!capL.isOpened() || !capR.isOpened()) { // Check if we succeeded
        cout << "Could not open or find the input videos!" << std::endl;
        return -1;
    }

    // change_resolution(&capL,&capR);
    cout << "Input 1 Resolution: " << capR.get(CV_CAP_PROP_FRAME_WIDTH) << "x" << capR.get(CV_CAP_PROP_FRAME_HEIGHT)
         << endl;
    cout << "Input 2 Resolution: " << capL.get(CV_CAP_PROP_FRAME_WIDTH) << "x" << capL.get(CV_CAP_PROP_FRAME_HEIGHT)
         << endl;

#ifndef CALIBRATION_ON
    cout << "Calibration: OFF" << endl;
#endif
#ifdef CALIBRATION_ON
    cout << "Calibration: ON" << endl;
    Mat M1, D1, M2, D2;
    Mat R, T, R1, P1, R2, P2;
    readCalibFiles(M1, D1, M2, D2, R, T);
#endif

    // PreSetup StereoBM Parameters
    numberOfDisparities =
        numberOfDisparities > 0 ? numberOfDisparities : ((int)(capR.get(CV_CAP_PROP_FRAME_WIDTH) / 8) + 15) / 4 & -16;

#ifdef OPENCV_NEW
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);
#endif

#ifdef OPENCV_OLD
    bm.state->preFilterCap = 31;
    bm.state->SADWindowSize = 9;
    bm.state->minDisparity = 0;
    bm.state->numberOfDisparities = numberOfDisparities;
    bm.state->textureThreshold = 10;
    bm.state->uniquenessRatio = 15;
    bm.state->speckleWindowSize = 100;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1;
#endif

    create_windows();
    // Create slider bars for Stereo Matching Parameters and for the HSV Filtering
    createTrackbars_sv();

    while(1) {
        capL >> imageL;
        capR >> imageR;

        // contrast_and_brightness(imageL,imageR,1.2,0);

        resize_frame(&imageL, &imageR);

// GaussianBlur(imageL,imageL,element_blur,0,0); //imshow("Blur",res);
// GaussianBlur(imageR,imageR,element_blur,0,0); //imshow("Blur",res);

// Contrast and Brightness. Do the operation: new_image(i,j) = alpha*image(i,j) + beta
/*for( int y = 0; y < imageL.rows; y++ ){
        for( int x = 0; x < imageL.cols; x++ ){
                for( int c = 0; c < 3; c++ ){
                        imageL.at<Vec3b>(y,x)[c] = saturate_cast<uchar>( ALPHA*( imageL.at<Vec3b>(y,x)[c] ) + BETA );
                        imageR.at<Vec3b>(y,x)[c] = saturate_cast<uchar>( ALPHA*( imageR.at<Vec3b>(y,x)[c] ) + BETA );
                }
        }
}*/

#ifdef CALIBRATION_ON
        Size img_size = imageL.size();
        stereoRectify(M1,
                      D1,
                      M2,
                      D2,
                      img_size,
                      R,
                      T,
                      R1,
                      R2,
                      P1,
                      P2,
                      Q,
                      CALIB_ZERO_DISPARITY,
                      -1,
                      img_size,
                      &roi1,
                      &roi2); // Mat map11, map12, map21, map22;
        Mat rmap[2][2];
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, rmap[0][0], rmap[0][1]);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, rmap[1][0], rmap[1][1]);
        Mat imageLr, imageRr;
        remap(imageL, imageLr, rmap[0][0], rmap[0][1], INTER_LINEAR);
        remap(imageR, imageRr, rmap[1][0], rmap[1][1], INTER_LINEAR);

        imageL = imageLr;
        imageR = imageRr;
#endif

        /*bm.state->preFilterSize = 5;
        bm.state->preFilterCap = 18;
        bm.state->SADWindowSize = 31;
        bm.state->minDisparity = 0;
        bm.state->numberOfDisparities = 64;
        bm.state->textureThreshold = 0;
        bm.state->uniquenessRatio = 0;
        bm.state->speckleWindowSize = 0;
        bm.state->speckleRange = 0;
        bm.state->disp12MaxDiff = 1;*/

        // numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

        /*bm->setROI1(roi1);
        bm->setROI2(roi2);
        bm->setPreFilterCap(31);
        bm->setBlockSize(9);
        bm->setMinDisparity(0);
        bm->setNumDisparities(64);
        bm->setTextureThreshold(10);
        bm->setUniquenessRatio(15);
        bm->setSpeckleWindowSize(100);
        bm->setSpeckleRange(32);
        bm->setDisp12MaxDiff(1);*/

        // Setting StereoBM Parameters
        temp[0] = getTrackbarPos("preFilterSize", trackbarWindowName) * 2.5 + 5;
        temp[1] = getTrackbarPos("preFilterCap", trackbarWindowName) * 0.625 + 1;
        temp[2] = getTrackbarPos("SADWindowSize", trackbarWindowName) * 2.5 + 5;
        temp[3] = getTrackbarPos("minDisparity", trackbarWindowName) * 2.0 - 100;
        temp[4] = getTrackbarPos("numberOfDisparities", trackbarWindowName) * 16;
        temp[5] = getTrackbarPos("textureThreshold", trackbarWindowName) * 320;
        temp[6] = getTrackbarPos("uniquenessRatio", trackbarWindowName) * 2.555;
        temp[7] = getTrackbarPos("speckleWindowSize", trackbarWindowName) * 1.0;
        temp[8] = getTrackbarPos("speckleRange", trackbarWindowName) * 1.0;
        temp[9] = getTrackbarPos("disp12MaxDiff", trackbarWindowName) * 1.0;

#ifdef OPENCV_OLD
        bm.state->roi1 = roi1;
        bm.state->roi2 = roi1;
#endif
#ifdef OPENCV_NEW
        bm->setROI1(roi1);
        bm->setROI2(roi2);
#endif

        if(temp[0] % 2 == 1 && temp[0] >= 5 && temp[0] <= 255) {
            // bm.state->preFilterSize = temp[0];
            bm->setPreFilterSize(temp[0]);
        }

        if(temp[1] >= 1 && temp[1] <= 63) {
            // bm.state->preFilterCap = temp[1];
            bm->setPreFilterCap(temp[1]);
        }

        if(temp[2] % 2 == 1 && temp[2] >= 5 && temp[2] <= 255 && temp[2] <= imageL.rows) {
            // bm.state->SADWindowSize = temp[2];
            bm->setBlockSize(temp[2]);
        }

        if(temp[3] >= -100 && temp[3] <= 100) {
            // bm.state->minDisparity = temp[3];
            bm->setMinDisparity(temp[3]);
        }

        if(temp[4] % 16 == 0 && temp[4] >= 16 && temp[4] <= 256) {
            // bm.state->numberOfDisparities = temp[4];
            bm->setNumDisparities(temp[4]);
        }

        if(temp[5] >= 0 && temp[5] <= 32000) {
            // bm.state->textureThreshold = temp[5];
            bm->setTextureThreshold(temp[5]);
        }

        if(temp[6] >= 0 && temp[6] <= 255) {
            // bm.state->uniquenessRatio = temp[6];
            bm->setUniquenessRatio(temp[6]);
        }

        if(temp[7] >= 0 && temp[7] <= 100) {
            // bm.state->speckleWindowSize = temp[7];
            bm->setSpeckleWindowSize(temp[7]);
        }

        if(temp[8] >= 0 && temp[8] <= 100) {
            // bm.state->speckleRange = temp[8];
            bm->setSpeckleRange(temp[8]);
        }

        if(temp[9] >= 0 && temp[9] <= 100) {
            // bm.state->disp12MaxDiff = temp[9];
            bm->setDisp12MaxDiff(temp[9]);
        }

        if(show_stereo_parameters) {
            cout << getTrackbarPos("preFilterSize", trackbarWindowName) << "\t" << temp[0] << endl;
            cout << getTrackbarPos("preFilterCap", trackbarWindowName) << "\t" << temp[1] << endl;
            cout << getTrackbarPos("SADWindowSize", trackbarWindowName) << "\t" << temp[2] << endl;
            cout << getTrackbarPos("minDisparity", trackbarWindowName) << "\t" << temp[3] << endl;
            cout << getTrackbarPos("numberOfDisparities", trackbarWindowName) << "\t" << temp[4] << endl;
            cout << getTrackbarPos("textureThreshold", trackbarWindowName) << "\t" << temp[5] << endl;
            cout << getTrackbarPos("uniquenessRatio", trackbarWindowName) << "\t" << temp[6] << endl;
            cout << getTrackbarPos("speckleWindowSize", trackbarWindowName) << "\t" << temp[7] << endl;
            cout << getTrackbarPos("speckleRange", trackbarWindowName) << "\t" << temp[8] << endl;
            cout << getTrackbarPos("disp12MaxDiff", trackbarWindowName) << "\t" << temp[9] << endl;
        }

        // Convert BGR to Gray_Scale
        cvtColor(imageL, imageL_grey, CV_BGR2GRAY);
        cvtColor(imageR, imageR_grey, CV_BGR2GRAY);

        Mat disp_bm = Mat(imageL.rows, imageL.cols, CV_16S);
        Mat disp8_bm = Mat(imageR.rows, imageR.cols, CV_8UC1);
        Mat disp8_bm_bgr;

#ifdef OPENCV_OLD
        bm(imageL_grey, imageR_grey, disp_bm, CV_16S);
#endif

#ifdef OPENCV_NEW
        bm->compute(imageL_grey, imageR_grey, disp_bm);
#endif

        normalize(disp_bm, disp8_bm, 0, 255, CV_MINMAX, CV_8U); // imshow("1",disp8_bm);

        //		// Change image type from 8UC1 to 32FC1
        //		cv::Mat disp32_bm = Mat( imageL.rows, imageL.cols, CV_32F );
        //		cv::Mat XYZ(disp8_bm.size(),CV_32FC3);
        //		disp8_bm.convertTo(disp32_bm, CV_32F);
        //		reprojectImageTo3D(disp8_bm, XYZ, Q, false, CV_32F);
        //		print_3D_points(disp32_bm,XYZ);

        // Image Processing
        // Invert Disparity Matrix - WARNING
        // bitwise_not(disp8_bm,disp8_bm);

        GaussianBlur(disp8_bm, res, element_blur, 0, 0); // imshow("Blur",res);
        erode(res, res, element_erode); // imshow("Erode",res);
        dilate(res, res, element_dilate); // imshow("Dilate",res);

        apply_Sobel(imageL_grey, sobel);

        //			threshold(imageL_grey,segmented,40,255,CV_THRESH_BINARY); imshow("Segmented",segmented);
        //			Mat dist;
        //			distanceTransform(segmented, dist, CV_DIST_L2, 3);
        //			normalize(dist,dist,0,1,NORM_MINMAX);	imshow("Dist",dist);
        //			threshold(dist, dist, .5, 1., CV_THRESH_BINARY); imshow("Dist2",dist);
        //			// Create the CV_8U version of the distance image
        //			// It is needed for cv::findContours()
        //			cv::Mat dist_8u;
        //			dist.convertTo(dist_8u, CV_8U);
        //
        //			// Find total markers
        //			std::vector<std::vector<cv::Point> > contours;
        //			cv::findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        //			// Total objects
        //			int ncomp = contours.size(); //cout << ncomp << endl;
        //
        //			cv::Mat markers = cv::Mat::zeros(dist.size(), CV_32SC1);
        //			for (int i = 0; i < ncomp; i++)
        //			    cv::drawContours(markers, contours, i, cv::Scalar::all(i+1), -1);
        //
        //			cv::circle(markers, cv::Point(5,5), 3, CV_RGB(255,255,255), -1);
        ////imshow("Markers",markers);
        //			//cv::watershed(segmented, markers);	imshow("segmented",segmented);

        //			// Generate random colors
        //			std::vector<cv::Vec3b> colors;
        //			for (int i = 0; i < ncomp; i++)
        //			{
        //			    int b = cv::theRNG().uniform(0, 255);
        //			    int g = cv::theRNG().uniform(0, 255);
        //			    int r = cv::theRNG().uniform(0, 255);
        //
        //			    colors.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
        //			}
        //
        //			// Create the result image
        //			cv::Mat watershed = cv::Mat::zeros(markers.size(), CV_8UC3);
        //
        //			// Fill labeled objects with random colors
        //			for (int i = 0; i < markers.rows; i++)
        //			{
        //			    for (int j = 0; j < markers.cols; j++)
        //			    {
        //			        int index = markers.at<int>(i,j);
        //			        if (index > 0 && index <= ncomp)
        //			        	watershed.at<cv::Vec3b>(i,j) = colors[index-1];
        //			        else
        //			        	watershed.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
        //			    }
        //			}
        //
        //			cv::imshow("watershed", watershed);

        // DepthMap GreyScale to RGB
        applyColorMap(disp8_bm, disp8_bm_bgr, COLORMAP_JET);
        applyColorMap(res, res_bgr, COLORMAP_JET);

        // Display
        if(show_input_image == 1) {
            imshow("Left", imageL);
            imshow("Right", imageR);
        } else {
            destroyWindow("Left");
            destroyWindow("Right");
        }

        imshow("BM Depth Image", disp8_bm);
        // imshow("BM Depth Image - BGR",disp8_bm_bgr);
        imshow("Sobel", sobel);
        bitwise_and(sobel, disp8_bm, sobel);
        imshow("Sobel AND disp8", sobel);
        imshow("Final", res_bgr);
        bitwise_not(res_bgr, res_bgr); // imshow("Final2",res_bgr);

        // apply_Tracking_Object(res_bgr,imageL);

        // Video Loop - If the last frame is reached, reset the capture and the frame_counter
        frame_counter += 1;
        if(frame_counter == capR.get(CV_CAP_PROP_FRAME_COUNT)) {
            frame_counter = 0;
            capL.set(CV_CAP_PROP_POS_FRAMES, 0);
            capR.set(CV_CAP_PROP_POS_FRAMES, 0);
        }

        key = waitKey(1);
        if(key == 'v') {
            show_stereo_parameters = !show_stereo_parameters;
        }
        if(key == 'q') {
            break;
        }
        if(key == 'z') {
            show_input_image = !show_input_image;
        }
    }
    // The camera will be closed automatically in VideoCapture Destructor
    cout << "END" << endl;
    return 0;
}
void create_windows()
{
    namedWindow("Left", 1);
    namedWindow("Right", 1);
    namedWindow("BM Depth Image", 1);
    // namedWindow("BM Depth Image - BGR",1);
    namedWindow("Sobel", 1);
    namedWindow("Sobel AND disp8", 1);
    namedWindow("Final", 1);
    // namedWindow("Final2",1);
}

void readCalibFiles(Mat& M1, Mat& D1, Mat& M2, Mat& D2, Mat& R, Mat& T)
{
    FileStorage fs("../data/calib/calib5_640_480/intrinsics.yml", FileStorage::READ);

    if(!fs.isOpened()) {
        printf("Failed to open file intrinsics.yml\n");
        return;
    }

    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    float scale = 1.f;
    M1 *= scale;
    M2 *= scale;

    fs.open("../data/calib/calib5_640_480/extrinsics.yml", FileStorage::READ);
    if(!fs.isOpened()) {
        printf("Failed to open file extrinsics.yml\n");
        return;
    }

    fs["R"] >> R;
    fs["T"] >> T;

    cout << "Intrinsics: " << endl;
    cout << "M1: " << M1 << endl;
    cout << "D1: " << D1 << endl;
    cout << "M2: " << M2 << endl;
    cout << "D2: " << D2 << endl;
    cout << "\nExtrinsics: " << endl;
    cout << "R: " << R << endl;
    cout << "T: " << T << endl;
}

void createTrackbars_sv()
{
    // create window for trackbars
    namedWindow(trackbarWindowName, WINDOW_NORMAL);
    // create memory to store trackbar name on window
    char TrackbarName[50];

    // Create memory to store Trackbar name on window
    sprintf(TrackbarName, "preFilterSize");
    sprintf(TrackbarName, "preFilterCap");
    sprintf(TrackbarName, "SADWindowSize");
    sprintf(TrackbarName, "minDisparity");
    sprintf(TrackbarName, "numberOfDisparities");
    sprintf(TrackbarName, "textureThreshold");
    sprintf(TrackbarName, "uniquenessRatio");
    sprintf(TrackbarName, "speckleWindowSize");
    sprintf(TrackbarName, "speckleRange");
    sprintf(TrackbarName, "disp12MaxDiff");

    // Create Trackbars and insert them into window
    createTrackbar("preFilterSize", trackbarWindowName, &preFilterSize, preFilterSize_MAX, on_trackbar);
    createTrackbar("preFilterCap", trackbarWindowName, &preFilterCap, preFilterCap_MAX, on_trackbar);
    createTrackbar("SADWindowSize", trackbarWindowName, &SADWindowSize, SADWindowSize_MAX, on_trackbar);
    createTrackbar("minDisparity", trackbarWindowName, &minDisparity, minDisparity_MAX, on_trackbar);
    createTrackbar(
        "numberOfDisparities", trackbarWindowName, &numberOfDisparities, numberOfDisparities_MAX, on_trackbar);
    createTrackbar("textureThreshold", trackbarWindowName, &textureThreshold, textureThreshold_MAX, on_trackbar);
    createTrackbar("uniquenessRatio", trackbarWindowName, &uniquenessRatio, uniquenessRatio_MAX, on_trackbar);
    createTrackbar("speckleWindowSize", trackbarWindowName, &speckleWindowSize, speckleWindowSize_MAX, on_trackbar);
    createTrackbar("speckleRange", trackbarWindowName, &speckleRange, speckleRange_MAX, on_trackbar);
    createTrackbar("disp12MaxDiff", trackbarWindowName, &disp12MaxDiff, disp12MaxDiff_MAX, on_trackbar);
}

void contrast_and_brightness(Mat& left, Mat& right, float alpha, float beta)
{
    // Contrast and Brightness. Do the operation: new_image(i,j) = alpha*image(i,j) + beta
    for(int y = 0; y < left.rows; y++) {
        for(int x = 0; x < left.cols; x++) {
            for(int c = 0; c < 3; c++) {
                left.at<Vec3b>(y, x)[c] = saturate_cast<uchar>(alpha * (left.at<Vec3b>(y, x)[c]) + beta);
                right.at<Vec3b>(y, x)[c] = saturate_cast<uchar>(alpha * (right.at<Vec3b>(y, x)[c]) + beta);
            }
        }
    }
}

void resize_frame(Mat* frame1, Mat* frame2)
{
#ifdef RESOLUTION_320x240
    resize(*frame1, *frame1, Size(320, 240), 0, 0, INTER_CUBIC);
    resize(*frame2, *frame2, Size(320, 240), 0, 0, INTER_CUBIC);
#endif

#ifdef RESOLUTION_640x480
    resize(*frame1, *frame1, Size(640, 480), 0, 0, INTER_CUBIC);
    resize(*frame2, *frame2, Size(640, 480), 0, 0, INTER_CUBIC);
#endif

#ifdef RESOLUTION_1280x720
    resize(*frame1, *frame1, Size(1280, 720), 0, 0, INTER_CUBIC);
    resize(*frame2, *frame2, Size(1280, 720), 0, 0, INTER_CUBIC);
#endif
}

void apply_Sobel(Mat& input, Mat& draw)
{
    Mat sobelx;
    Sobel(input, sobelx, CV_32F, 1, 0);

    double minVal, maxVal;
    minMaxLoc(sobelx, &minVal, &maxVal); // find minimum and maximum intensities
    // cout << "minVal : " << minVal << endl << "maxVal : " << maxVal << endl;

    sobelx.convertTo(draw, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
}

void morphOps(Mat& thresh)
{
    // create structuring element that will be used to "dilate" and "erode" image.
    // the element chosen here is a 3px by 3px rectangle

    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
    // dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

    erode(thresh, thresh, erodeElement);
    erode(thresh, thresh, erodeElement);

    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);
}

void trackFilteredObject(int& x, int& y, Mat threshold, Mat& cameraFeed)
{
    Mat temp;
    threshold.copyTo(temp);
    // these two vectors needed for output of findContours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    // find contours of filtered image using openCV findContours function
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    // use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;
    if(hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        // if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects < MAX_NUM_OBJECTS) {
            for(int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;

                // if the area is less than 20 px by 20px then it is probably just noise
                // if the area is the same as the 3/2 of the image size, probably just a bad filter
                // we only want the object with the largest area so we safe a reference area each
                // iteration and compare it to the area in the next iteration.
                if(area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area > refArea) {
                    x = moment.m10 / area;
                    y = moment.m01 / area;
                    objectFound = true;
                    refArea = area;
                } else
                    objectFound = false;
            }
            // let user know you found an object
            if(objectFound == true) {
                putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                // draw object location on screen
                drawObject(x, y, cameraFeed);
            }

        } else
            putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
    }
}

void drawObject(int x, int y, Mat& frame)
{
    // use some of the openCV drawing functions to draw crosshairs
    // on your tracked image!

    // UPDATE:JUNE 18TH, 2013
    // added 'if' and 'else' statements to prevent
    // memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

    circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
    if(y - 25 > 0)
        line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
    else
        line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
    if(y + 25 < FRAME_HEIGHT)
        line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
    else
        line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
    if(x - 25 > 0)
        line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
    else
        line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
    if(x + 25 < FRAME_WIDTH)
        line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
    else
        line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

    putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
}

void apply_Tracking_Object(Mat& input, Mat& camerafeed)
{
    Mat threshold;
    Mat HSV;
    int x, y;
    cvtColor(input, HSV, COLOR_BGR2HSV);
    // filter HSV image between values and store filtered image to threshold matrix
    inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
    // perform morphological operations on thresholded image to eliminate noise
    // and emphasize the filtered object(s)
    if(true)
        morphOps(threshold);
    // pass in thresholded frame to our object tracking function
    // this function will return the x and y coordinates of the
    // filtered object
    if(true)
        trackFilteredObject(x, y, threshold, camerafeed);

    erode(threshold, threshold, element_erode);
    dilate(threshold, threshold, element_erode);

    // Show frames
    imshow("HSV", input);
    imshow("Threshold ", threshold);
}
