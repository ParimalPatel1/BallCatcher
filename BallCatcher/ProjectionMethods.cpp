#ifndef PROJECTIONMETHODS_H
#define PROJECTIONMETHODS_H
#include "ProjectionMethods.h"
#endif


//int PreFilterSize = 0;	
//int PreFilterCap = 19;	
//int SADWindowSize = 3;	//33
//int MinDisparity = 100; // 130
//int NumberOfDisparities = 1; 
//int TextureThreshold = 20;	
//int UniquenessRatio = 12;	
//int SpeckleWindowSize = 5;	
//int SpeckleRange = 1;	
//int Disp12MaxDiff = 1;	

int PreFilterSize = 0; 	//*2 + 1
int PreFilterCap = 4;	
int SADWindowSize = 1;	//*2 + 1
int MinDisparity = -128; 
int NumberOfDisparities = 16; // * 16
int TextureThreshold = 20;	
int UniquenessRatio = 1;	
int SpeckleWindowSize = 100;	
int SpeckleRange = 5;	
int Disp12MaxDiff = 1;	

const int PreFilterSize_MAX = 100;
const int PreFilterCap_MAX = 100;
const int SADWindowSize_MAX = 100;
const int MinDisparity_MAX = 200;
const int TextureThreshold_MAX = 100;
const int NumberOfDisparities_MAX = 7;
const int UniquenessRatio_MAX = 50;
const int SpeckleWindowSize_MAX = 200;
const int SpeckleRange_MAX = 4;
const int Disp12MaxDiff_MAX = 10;
string trackbarWindowName = "Stereo Trackbars";
Ptr<StereoBM> bm = StereoBM::create(NumberOfDisparities, 21); // blocksize
Ptr<StereoSGBM> sgbm = StereoSGBM::create(MinDisparity, NumberOfDisparities, SADWindowSize); // blocksize

void on_trackbar(int a, void* b)
{
	Stereo_update();
}
void Stereo_update()
{
	int temp;


	temp = getTrackbarPos("preFilterCap", trackbarWindowName);
	bm->setPreFilterCap(temp);
	sgbm->setPreFilterCap(temp);

	temp = getTrackbarPos("SADWindowSize", trackbarWindowName)*2 + 1;
	bm->setSpeckleWindowSize(temp);
	sgbm->setSpeckleWindowSize(temp);

	temp = getTrackbarPos("minDisparity", trackbarWindowName);
	bm->setMinDisparity(temp);
	sgbm->setMinDisparity(temp);

	temp = getTrackbarPos("numberOfDisparities", trackbarWindowName)* 16;
	bm->setNumDisparities(temp);
	sgbm->setNumDisparities(temp);

	temp = getTrackbarPos("uniquenessRatio", trackbarWindowName);
	bm->setUniquenessRatio(temp);
	sgbm->setUniquenessRatio(temp);

	temp = getTrackbarPos("speckleWindowSize", trackbarWindowName);
	bm->setSpeckleWindowSize(temp);
	sgbm->setSpeckleWindowSize(temp);

	temp = getTrackbarPos("speckleRange", trackbarWindowName);
	sgbm->setSpeckleRange(temp);
	bm->setSpeckleRange(temp);

	temp = getTrackbarPos("disp12MaxDiff", trackbarWindowName);
	bm->setDisp12MaxDiff(temp);
	sgbm->setDisp12MaxDiff(temp);

	temp = getTrackbarPos("preFilterSize", trackbarWindowName) * 2 + 1;
	bm->setPreFilterSize(temp);
	temp = getTrackbarPos("textureThreshold", trackbarWindowName);
	bm->setTextureThreshold(temp);
		//	cout << getTrackbarPos("preFilterSize", trackbarWindowName) << "\t" << temp << endl;
		//	cout << getTrackbarPos("preFilterCap", trackbarWindowName) << "\t" << temp << endl;
		//	cout << getTrackbarPos("SADWindowSize", trackbarWindowName) << "\t" << temp << endl;
		 //cout << getTrackbarPos("minDisparity", trackbarWindowName) << "\t" << temp << endl;
		//cout << getTrackbarPos("numberOfDisparities", trackbarWindowName) << "\t" << temp << endl;
		//cout << getTrackbarPos("textureThreshold", trackbarWindowName) << "\t" << temp << endl;
		//cout << getTrackbarPos("uniquenessRatio", trackbarWindowName) << "\t" << temp << endl;
		//	cout << getTrackbarPos("speckleWindowSize", trackbarWindowName) << "\t" << temp << endl;
		//cout << getTrackbarPos("speckleRange", trackbarWindowName) << "\t" << temp << endl;
	//	cout << getTrackbarPos("disp12MaxDiff", trackbarWindowName) << "\t" << temp << endl;

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

	createTrackbar("preFilterCap", trackbarWindowName, &PreFilterCap, PreFilterCap_MAX, on_trackbar);
	createTrackbar("SADWindowSize", trackbarWindowName, &SADWindowSize, SADWindowSize_MAX, on_trackbar);
	createTrackbar("minDisparity", trackbarWindowName, &MinDisparity, MinDisparity_MAX, on_trackbar);
	createTrackbar("numberOfDisparities", trackbarWindowName, &NumberOfDisparities, NumberOfDisparities_MAX, on_trackbar);
	createTrackbar("uniquenessRatio", trackbarWindowName, &UniquenessRatio, UniquenessRatio_MAX, on_trackbar);
	createTrackbar("speckleWindowSize", trackbarWindowName, &SpeckleWindowSize, SpeckleWindowSize_MAX, on_trackbar);
	createTrackbar("speckleRange", trackbarWindowName, &SpeckleRange, SpeckleRange_MAX, on_trackbar);
	createTrackbar("disp12MaxDiff", trackbarWindowName, &Disp12MaxDiff, Disp12MaxDiff_MAX, on_trackbar);
	//	createTrackbar("preFilterSize", trackbarWindowName, &PreFilterSize, PreFilterSize_MAX, on_trackbar);
	//createTrackbar("textureThreshold", trackbarWindowName, &TextureThreshold, TextureThreshold_MAX, on_trackbar);
}
