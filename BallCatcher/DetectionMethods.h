#pragma once
#ifndef DETECTIONMETHODS_H
#define DETECTIONMETHODS_H

#include <opencv\cv.h>
#include <opencv2\opencv.hpp>
#include <Windows.h>
#include <ctime>
#include "motiontracking.h"

using namespace cv;
using namespace std;
// prototypes
extern int searchForBalls(Mat& SearchImage, Mat& movingobjects, ballLoc& ballspotted);
#endif