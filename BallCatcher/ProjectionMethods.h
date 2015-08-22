
#ifndef PROJECTIONMETHODS_H_
#define PROJECTIONMETHODS_H_
#include <opencv\cv.h>
#include  <opencv2\opencv.hpp>
#include <Windows.h>

#define RESOLUTION Size(640,480)
using namespace cv;
using namespace std;
//Initial value and MAX StereoBM Parameters.
//These will be changed using trackbars

extern int PreFilterSize;	extern const int PreFilterSize_MAX;
extern int PreFilterCap;	extern const int PreFilterCap_MAX;
extern int SADWindowSize;	extern const int SADWindowSize_MAX;
extern int MinDisparity;	extern const int MinDisparity_MAX;
extern int NumberOfDisparities;	extern const int NumberOfDisparities_MAX;
extern int TextureThreshold;	extern const int TextureThreshold_MAX;
extern int UniquenessRatio;	extern const int UniquenessRatio_MAX;
extern int SpeckleWindowSize;	extern const int SpeckleWindowSize_MAX;
extern int SpeckleRange;	extern const int SpeckleRange_MAX;
extern int Disp12MaxDiff;	extern const int Disp12MaxDiff_MAX;
extern string trackbarWindowName;
extern Ptr<StereoBM> bm;
extern Ptr<StereoSGBM> sgbm;
////// STEREO
/////////// TRACKBARS START
//Setting StereoBM Parameters
extern void Stereo_update();
extern void on_trackbar(int, void*);
extern void createTrackbars();


class Stereotime
{
public:
	Stereotime()
	{
	};
	///////// START STEREO INIT
	//Ptr<StereoBM> bm = StereoBM::create(16, 9);//(0, 16, 3,0,0,2,10,10,100,16, StereoSGBM::MODE_SGBM);//(0, 16, 3);

	void Stereo_Init()
	{
		// reading intrinsic parameters
		FileStorage fs("intrinsics.yml", FileStorage::READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", "intrinsics.yml");
			return;
		}
		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;
		fs.open("extrinsics.yml", FileStorage::READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", "extrinsics.yml");
			return;
		}
		fs["R"] >> R;
		fs["T"] >> T;
		// outputs R1, R2, P1, P2
		cv::stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
		Rect DispROI = getValidDisparityROI(roi1, roi2, MinDisparity, NumberOfDisparities * 16, SADWindowSize);

		cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12); 
		cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

		
		sgbm->setPreFilterCap(PreFilterCap);
		sgbm->setBlockSize(SADWindowSize*2+1);
		int cn = map11.channels();
		sgbm->setP1(8*cn*(SADWindowSize * 2 + 1)*(SADWindowSize * 2 + 1));
		sgbm->setP2(32*cn*(SADWindowSize * 2 + 1)*(SADWindowSize * 2 + 1));
		sgbm->setMinDisparity(MinDisparity);
		sgbm->setNumDisparities(NumberOfDisparities * 16);
		sgbm->setUniquenessRatio(UniquenessRatio); // 10
		sgbm->setSpeckleWindowSize(SpeckleWindowSize); // 100
		sgbm->setSpeckleRange(SpeckleRange); // 32
		sgbm->setDisp12MaxDiff(Disp12MaxDiff); // 1
		sgbm->setMode(StereoSGBM::MODE_HH ); // StereoSGBM::MODE_SGBM

		bm->setROI1(roi1);
		bm->setROI2(roi2);
		bm->setPreFilterType(CV_STEREO_BM_XSOBEL);
		bm->setPreFilterCap(PreFilterCap);
		bm->setBlockSize(SADWindowSize * 2 + 1);
		bm->setMinDisparity(MinDisparity);
		bm->setNumDisparities(NumberOfDisparities * 16);
		bm->setTextureThreshold(TextureThreshold);
		bm->setUniquenessRatio(UniquenessRatio);
		bm->setSpeckleWindowSize(SpeckleWindowSize);
		bm->setSpeckleRange(SpeckleRange);
		bm->setDisp12MaxDiff(Disp12MaxDiff);
	}

	//////// END STEREO INIT
	//Mat LeftRemap, RightRemap;

	void remap_Grey(Mat &Gray_Left, Mat &Gray_Right)
	{
		remap(Gray_Left, Gray_Left, map11, map12, INTER_LINEAR);
		remap(Gray_Right, Gray_Right, map21, map22, INTER_LINEAR);
	}
	//Mat disp, disp8;
	// STEREO METHOD
	Mat compute_Stereo(Mat Gray_Left, Mat Gray_Right)
	{
		Mat disparity;
		//// make sure left and right are same size
		if (Gray_Left.size() != Gray_Right.size())
			return disparity;
	//	bm->compute(Gray_Left, Gray_Right, disparity);
		sgbm->compute(Gray_Left, Gray_Right, disparity);
		return disparity;

		//imshow("Disparity", disparity);
	}
	// given the normalized disparity image (should be normalized ??), now generate threshold the image so only the ball's points remain.
	// ignore any "black" points and average all the points to get the real 3D center point of the ball.
	void projectTheBall(Mat disparity)
	{
		// now project 3d points
		cv::Mat disp32_bm = Mat(disparity.size(), CV_32F);
		cv::Mat XYZ(disparity.size(), CV_32FC3);
		disparity.convertTo(disp32_bm, CV_32F);
		reprojectImageTo3D(disparity, XYZ, Q, false, CV_32F);
		//perspectiveTransform(disparity, XYZ, Q);
		// XYZ is output, where to save??
	}

	private:
		Size img_size = RESOLUTION;
		Rect roi1, roi2;
		Mat Q;

		Mat M1 = Mat(Size(3, 3), CV_64FC1);  // Camera 1 Matrix
		Mat M2 = Mat(Size(3, 3), CV_64FC1); // Camera 2 Matrix
		Mat D1 = Mat(Size(1, 12), CV_64FC1); // Camera 1 Distortion Parameters
		Mat D2 = Mat(Size(1, 12), CV_64FC1); // Camera 2 Distortion Parameters

		Mat R1, P1, R2, P2; // outputs of stereoRectify

		Mat R = Mat(Size(3, 3), CV_64FC1); // Rotation Matrix
		Mat T = Mat(Size(3, 1), CV_64FC1); // Translation Matrix

		Mat map11, map12, map21, map22;
};

#endif