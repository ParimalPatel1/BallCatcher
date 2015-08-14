#ifndef PROJECTIONMETHODS_H
#define PROJECTIONMETHODS_H
#include "ProjectionMethods.h"
#endif


int PreFilterSize = 0;	
int PreFilterCap = 19;	
int SADWindowSize = 33;	
int MinDisparity = 50;	
int NumberOfDisparities = 48;	
int TextureThreshold = 20;	
int UniquenessRatio = 12;	
int SpeckleWindowSize = 5;	
int SpeckleRange = 50;	
int Disp12MaxDiff = 1;	

const int PreFilterSize_MAX = 100;
const int PreFilterCap_MAX = 100;
const int SADWindowSize_MAX = 100;
const int MinDisparity_MAX = 100;
const int TextureThreshold_MAX = 100;
const int NumberOfDisparities_MAX = 50;
const int UniquenessRatio_MAX = 255;
const int SpeckleWindowSize_MAX = 100;
const int SpeckleRange_MAX = 200;
const int Disp12MaxDiff_MAX = 2;
string trackbarWindowName = "Stereo Trackbars";
Ptr<StereoBM> bm = StereoBM::create(16, 21);
Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

void on_trackbar(int a, void* b)
{
	Stereo_update(bm);
}
void Stereo_update(Ptr<StereoBM> bm)
{
	int temp;
	temp = getTrackbarPos("preFilterSize", trackbarWindowName);// *2.5 + 5;
	//if (temp % 2 == 1 && temp >= 5 && temp <= 255) {
		bm->setPreFilterSize(temp);
	//	cout << getTrackbarPos("preFilterSize", trackbarWindowName) << "\t" << temp << endl;
	//}
	temp = getTrackbarPos("preFilterCap", trackbarWindowName);//*0.625 + 1;
//	if (temp >= 1 && temp <= 63) {
		bm->setPreFilterCap(temp);
	//	cout << getTrackbarPos("preFilterCap", trackbarWindowName) << "\t" << temp << endl;
//	}
	temp = getTrackbarPos("SADWindowSize", trackbarWindowName);//*2.5 + 5;
//	if (temp % 2 == 1 && temp >= 5 && temp <= 255 && temp <= RESOLUTION.height) {
		bm->setSpeckleWindowSize(temp);
	//	cout << getTrackbarPos("SADWindowSize", trackbarWindowName) << "\t" << temp << endl;
//	}
	temp = getTrackbarPos("minDisparity", trackbarWindowName);//*2.0 - 100;
//	if (temp >= -100 && temp <= 100) {
		bm->setMinDisparity(temp);
		//cout << getTrackbarPos("minDisparity", trackbarWindowName) << "\t" << temp << endl;
	//}
	temp = getTrackbarPos("numberOfDisparities", trackbarWindowName); //* 16;
//	if (temp % 16 == 0 && temp >= 16 && temp <= 256) {
		bm->setNumDisparities(temp);
		//cout << getTrackbarPos("numberOfDisparities", trackbarWindowName) << "\t" << temp << endl;
//	}
	temp = getTrackbarPos("textureThreshold", trackbarWindowName);// * 320;
//	if (temp >= 0 && temp <= 32000) {
		bm->setTextureThreshold(temp);
		//cout << getTrackbarPos("textureThreshold", trackbarWindowName) << "\t" << temp << endl;
//	}
	temp = getTrackbarPos("uniquenessRatio", trackbarWindowName);//;*2.555;
	//if (temp >= 0 && temp <= 255) {
		bm->setUniquenessRatio(temp);
		//cout << getTrackbarPos("uniquenessRatio", trackbarWindowName) << "\t" << temp << endl;
//	}
	temp = getTrackbarPos("speckleWindowSize", trackbarWindowName)*1.0;
//	if (temp >= 0 && temp <= 100) {
		bm->setSpeckleWindowSize(temp);
	//	cout << getTrackbarPos("speckleWindowSize", trackbarWindowName) << "\t" << temp << endl;
	//}
	temp = getTrackbarPos("speckleRange", trackbarWindowName)*1.0;
	//if (temp >= 0 && temp <= 100) {
		bm->setSpeckleRange(temp);
		//cout << getTrackbarPos("speckleRange", trackbarWindowName) << "\t" << temp << endl;
	//}
	temp = getTrackbarPos("disp12MaxDiff", trackbarWindowName)*1.0;
	//if (temp >= 0 && temp <= 100) {
		bm->setDisp12MaxDiff(temp);
	//	cout << getTrackbarPos("disp12MaxDiff", trackbarWindowName) << "\t" << temp << endl;
//	}
}

void createTrackbars() { //Create window for trackbars
	char TrackbarName[50];

	// Create TrackBars Window
	namedWindow(trackbarWindowName, 0);

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
	//Create Trackbars and insert them into window
	createTrackbar("preFilterSize", trackbarWindowName, &PreFilterSize, PreFilterSize_MAX, on_trackbar);
	createTrackbar("preFilterCap", trackbarWindowName, &PreFilterCap, PreFilterCap_MAX, on_trackbar);
	createTrackbar("SADWindowSize", trackbarWindowName, &SADWindowSize, SADWindowSize_MAX, on_trackbar);
	createTrackbar("minDisparity", trackbarWindowName, &MinDisparity, MinDisparity_MAX, on_trackbar);
	createTrackbar("numberOfDisparities", trackbarWindowName, &NumberOfDisparities, NumberOfDisparities_MAX, on_trackbar);
	createTrackbar("textureThreshold", trackbarWindowName, &TextureThreshold, TextureThreshold_MAX, on_trackbar);
	createTrackbar("uniquenessRatio", trackbarWindowName, &UniquenessRatio, UniquenessRatio_MAX, on_trackbar);
	createTrackbar("speckleWindowSize", trackbarWindowName, &SpeckleWindowSize, SpeckleWindowSize_MAX, on_trackbar);
	createTrackbar("speckleRange", trackbarWindowName, &SpeckleRange, SpeckleRange_MAX, on_trackbar);
	createTrackbar("disp12MaxDiff", trackbarWindowName, &Disp12MaxDiff, Disp12MaxDiff_MAX, on_trackbar);
}
