
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
extern void Stereo_update(Ptr<StereoBM> bm);
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
		int SADWindowSize = 9;
		int numberOfDisparities = 16;
		if (numberOfDisparities != 1 && numberOfDisparities < 1 || numberOfDisparities % 16 != 0)
			return;
		if (SADWindowSize != 1 && SADWindowSize < 1 || SADWindowSize % 2 != 1)
			return;
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

		cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12); 
		cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

		sgbm->setPreFilterCap(63);
		int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
		sgbm->setBlockSize(sgbmWinSize);
		int cn = map11.channels();

		sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setMinDisparity(0);
		sgbm->setNumDisparities(numberOfDisparities);
		sgbm->setUniquenessRatio(10);
		sgbm->setSpeckleWindowSize(100);
		sgbm->setSpeckleRange(32);
		sgbm->setDisp12MaxDiff(1);
		sgbm->setMode(StereoSGBM::MODE_HH ); // StereoSGBM::MODE_SGBM

		//bm->setROI1(roi1);
		//bm->setROI2(roi2);
		//bm->setPreFilterCap(PreFilterCap);
		//bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
		//bm->setMinDisparity(MinDisparity);
		//bm->setNumDisparities(numberOfDisparities);
		//bm->setTextureThreshold(TextureThreshold);
		//bm->setUniquenessRatio(UniquenessRatio);
		//bm->setSpeckleWindowSize(SpeckleWindowSize);
		//bm->setSpeckleRange(SpeckleRange);
		//bm->setDisp12MaxDiff(Disp12MaxDiff);
	}

	//////// END STEREO INIT
	Mat LeftRemap, RightRemap;
	Mat disp, disp8;
	// STEREO METHOD
	void compute_Stereo(Mat Gray_Left, Mat Gray_Right, Mat &disparity)
	{
		// make sure left and right are same size
		if (Gray_Left.size() != Gray_Right.size())
			return;
		remap(Gray_Left, LeftRemap, map11, map12, INTER_LINEAR);
		remap(Gray_Right, RightRemap, map21, map22, INTER_LINEAR);
		//	imshow("RGBmapL", LeftRemap);
		//	imshow("RGBmapR", RightRemap);
		//	cvtColor(LeftRemap, Gray_Left, COLOR_BGR2GRAY);
		//	cvtColor(RightRemap, Gray_Right, COLOR_BGR2GRAY);
		//	imshow("GL", Gray_Left);
		//	imshow("GR", Gray_Right);
			// REMAP WHY YOU NO WORK 		bm->compute(Gray_Left, Gray_Right, disp); because you need to remap RGB image, not grayscale
		//	bm->compute(Gray_Left, Gray_Right, disp);
		sgbm->compute(Gray_Left, Gray_Right, disp);
		//	imshow("Disparity", disp);
		normalize(disp, disparity, 0, 255, CV_MINMAX, CV_8U); // used to be disp8
		//imshow("NormDisparity", disp8);
		// Change image type from 8UC1 to 32FC1

	}
	// given the normalized disparity image (should be normalized ??), now generate threshold the image so only the ball's points remain.
	// ignore any "black" points and average all the points to get the real 3D center point of the ball.
	void projectTheBall(Mat disparity)
	{
		// now project 3d points
		cv::Mat disp32_bm = Mat(disparity.size(), CV_32F);
		cv::Mat XYZ(disparity.size(), CV_32FC3);
		disp8.convertTo(disp32_bm, CV_32F);
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