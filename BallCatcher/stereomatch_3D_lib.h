/*
 * stereomatch_3D_lib.h
 *
 *  Created on: May 31, 2015
 *      Author: nicolas
 */

#include "cv.h"
#include "highgui.h"
#include "cvaux.h"
#include "stdio.h"
#include "opencv2/opencv.hpp"

#ifndef STEREOMATCH_3D_LIB_H_
#define STEREOMATCH_3D_LIB_H_

// Erode, Dilation and Blur Constants
#define EROSION_SIZE 3 // SAR
#define DILATE_SIZE 5 // SAR
#define BLUR_SIZE 3

// Main class of application
class sv
{
public:
    /**
     * Class constructor
     * @param int - defines mode of application
     * @param int - defines the number of frames taken into account in building model
     * @param string - path to the left camera stream
     * @param string - path to the right camera stream
     * @param string - path to intrinsics camera calibration parameters
     * @param string - path to extrinsics camera calibration parameters
     */
    sv(const int mode = 0);
    /**
     * runs application
     */
    void run();
    /**
     * Class destructor
     */
    ~sv(){};

private:
    void showImages();
    void handleKey(char key);

    int methodNr;
    int framesCount;
    int mode; /** 0 - normal, 1-calibration 2-disparity testing **/
};

#endif /* STEREOMATCH_3D_LIB_H_ */

#ifndef MEANSHIFTSEGMENTATION_HPP_
#define MEANSHIFTSEGMENTATION_HPP_

/**
 * Class implementing segmentation method
 */
class Segmentation
{
public:
    /**
     * Class constructor
     */
    Segmentation()
        : threshold1(200)
        , threshold2(50)
        , level(1){};

    /**
     * Returns segmented image
     * @param Mat - image to segmented
     * @return Mat - segmented image
     */
    cv::Mat segmentImage(cv::Mat);

    /**
     * Class destructor
     */
    virtual ~Segmentation(){};

    /** Segmentation parameters **/
    int threshold1;
    int threshold2;
    int level;
};

#endif /* MEANSHIFTSEGMENTATION_HPP_ */
