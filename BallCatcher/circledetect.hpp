#pragma once
#include <opencv2\opencv.hpp>
#ifndef CIRCLEDETECT_HPP
#define CIRCLEDETECT_HPP
// input points, L output ellipse
void findTheBall(std::vector<cv::Point> v, cv::RotatedRect& L, bool drawmode = false);

#endif