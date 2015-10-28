#pragma once
#include  <opencv2\opencv.hpp>
#ifndef CIRCLEDETECT_HPP
#define CIRCLEDETECT_HPP
// input points, L output ellipse
void calcEccentricity(std::vector<cv::Point> v, cv::RotatedRect &L);


#endif