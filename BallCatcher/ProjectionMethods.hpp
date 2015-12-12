
#ifndef PROJECTIONMETHODS_H_
#define PROJECTIONMETHODS_H_

#include  <opencv2\opencv.hpp>
#include <Windows.h>


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
extern std::string trackbarWindowName;
extern cv::Ptr<cv::StereoBM> bm;
extern cv::Ptr<cv::StereoSGBM> sgbm;
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

	void Stereo_Init(cv::Size imgsize)
	{
		img_size = imgsize;
		// reading intrinsic parameters
		cv::FileStorage fs("intrinsics.yml", cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", "intrinsics.yml");
			return;
		}
		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;
		fs.open("extrinsics.yml", cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			std::printf("Failed to open file %s\n", "extrinsics.yml");
			return;
		}
		fs["R"] >> R;
		fs["T"] >> T;
		// outputs R1, R2, P1, P2
		cv::stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
		cv::Rect DispROI = cv::getValidDisparityROI(roi1, roi2, MinDisparity, NumberOfDisparities * 16, SADWindowSize);

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
		sgbm->setMode(cv::StereoSGBM::MODE_HH ); // StereoSGBM::MODE_SGBM

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
	void remap_RGB(cv::Mat &RGB, int leftorright)
	{
		if (leftorright == 0) // left
			cv::remap(RGB, RGB, map11, map12, cv::INTER_LINEAR);
		else if (leftorright == 1) // right
			cv::remap(RGB, RGB, map21, map22, cv::INTER_LINEAR);
	}
	void remap_RGB(cv::Mat &RGBL, cv::Mat &RGBR)
	{
		cv::remap(RGBL, RGBL, map11, map12, cv::INTER_LINEAR);
		cv::remap(RGBR, RGBR, map21, map22, cv::INTER_LINEAR);
	}

	void remap_Grey(cv::Mat &Gray_Left, cv::Mat &Gray_Right)
	{
		cv::remap(Gray_Left, Gray_Left, map11, map12, cv::INTER_LINEAR);
		cv::remap(Gray_Right, Gray_Right, map21, map22, cv::INTER_LINEAR);
	}
	void remap_HSV(cv::Mat &HSV_Left, cv::Mat &HSV_Right)
	{
		cv::remap(HSV_Left, HSV_Left, map11, map12, cv::INTER_LINEAR);
		cv::remap(HSV_Right, HSV_Right, map21, map22, cv::INTER_LINEAR);
	}
	//Mat disp, disp8;
	// STEREO METHOD
	cv::Mat compute_Stereo(cv::Mat Gray_Left, cv::Mat Gray_Right)
	{
		cv::Mat disparity;
		//// make sure left and right are same size
		if (Gray_Left.size() != Gray_Right.size())
			return disparity; // how to return error when return type is mat??
	//	bm->compute(Gray_Left, Gray_Right, disparity);
		sgbm->compute(Gray_Left, Gray_Right, disparity);
		return disparity;

		//imshow("Disparity", disparity);
	}
	// given the normalized disparity image (should be normalized ??), now generate threshold the image so only the ball's points remain.
	// ignore any "black" points and average all the points to get the real 3D center point of the ball.
	std::vector<cv::Point3f> calcDisparityPoints(std::vector<cv::Point2f> inputsL, std::vector<cv::Point2f> inputsR)
	{
		int j = inputsL.size() < inputsR.size() ? inputsL.size() : inputsR.size(); // use the min num of points
		std::vector<cv::Point3f> outputpoints;
		cv::Point3f tempP;
		for (int i = 0; i < j; i++)
		{
			tempP.x = inputsL[i].x; // using left camera as base
			tempP.y = (inputsR[i].y + inputsL[i].y)/2; // y should be the same?
			tempP.z = (inputsR[i].x - inputsL[i].x); // order and use x or y?
			outputpoints.push_back(tempP);
		}
		return outputpoints;
	}
	std::vector<cv::Point3f> projectTheBall(std::vector<cv::Point3f> inputpoints)
	{
		std::vector<cv::Point3f> outputpoints;
		perspectiveTransform(inputpoints, outputpoints, Q); // now real world coords
		return outputpoints;
		// calculate trajectory and landing position // 'FLOOR' HEIGHT from CAMERA, ASSUME CAMERA is LEVEL with the 'FLOOR'


	//	cv::Mat inputmat(inputpoints); // convert vec to mat
	//	// now project 3d points
	//	cv::Mat disp32_bm = cv::Mat(inputmat.size(), CV_32F);
	//	inputmat.convertTo(disp32_bm, CV_32F);
	//	cv::Mat XYZ(inputmat.size(), CV_32FC3);
	////	reprojectImageTo3D(inputmat, XYZ, Q, false, CV_32F);

	//	cv::Vec4f threeDline;
	//	//cv::approxPolyDP(XYZ, )
	//	cv::fitLine(XYZ, threeDline, CV_DIST_L2, 0, 0.01, 0.01);
		//now calculate where the ball is going to land at y inches.

		// XYZ is output, where to save??
	}

	void calcTrajectory()
	{
	}
	private:
		cv::Size img_size;
		cv::Rect roi1, roi2;
		cv::Mat Q;

		cv::Mat M1 = cv::Mat(cv::Size(3, 3), CV_64FC1);  // Camera 1 Matrix
		cv::Mat M2 = cv::Mat(cv::Size(3, 3), CV_64FC1); // Camera 2 Matrix
		cv::Mat D1 = cv::Mat(cv::Size(1, 12), CV_64FC1); // Camera 1 Distortion Parameters
		cv::Mat D2 = cv::Mat(cv::Size(1, 12), CV_64FC1); // Camera 2 Distortion Parameters

		cv::Mat R1, P1, R2, P2; // outputs of stereoRectify

		cv::Mat R = cv::Mat(cv::Size(3, 3), CV_64FC1); // Rotation Matrix
		cv::Mat T = cv::Mat(cv::Size(3, 1), CV_64FC1); // Translation Matrix

		cv::Mat map11, map12, map21, map22;
};

#endif