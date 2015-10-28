// Circle.h: interface for the Circle class.
// Circle class.
// Purpose : Represent the circle object
// Input : 3 different points
// Process : Calcuate the radius and center
// Output : Circle
//           
// This class originally designed for representation of discretized curvature information 
// of sequential pointlist  
// KJIST CAD/CAM     Ryu, Jae Hun ( ryu@geguri.kjist.ac.kr)
// Last update : 1999. 7. 4
#if !defined(AFX_CIRCLE_H__1EC15131_4038_11D3_8404_00C04FCC7989__INCLUDED_)
#define AFX_CIRCLE_H__1EC15131_4038_11D3_8404_00C04FCC7989__INCLUDED_

//#include "Point.h"	// Added by ClassView
//#include "stdafx.h"
//#include "MediSurf.h"
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
//#include "motiontracking.h"
#include  <opencv2\opencv.hpp>
//using namespace cv;
//using namespace std;
class Circle  
{
public:
	double GetRadius();
	cv::Point GetCenter();
	Circle(cv::Point &V1, cv::Point &V2, cv::Point &V3);	// p1, p2, p3 are co-planar
	Circle(std::vector<cv::Point> pointlist);
	Circle();
	virtual ~Circle();

private:
	double CalcCircle(cv::Point &pt1, cv::Point &pt2, cv::Point &pt3);
	double CalcCircle(std::vector<cv::Point> points);
	bool IsPerpendicular(cv::Point &pt1, cv::Point &pt2, cv::Point &pt3);
	int IsPerpendicular(std::vector<cv::Point> points);
	double m_dRadius;
	cv::Point m_Center;
	std::vector<cv::Point> points;
};

#endif // !defined(AFX_CIRCLE_H__1EC15131_4038_11D3_8404_00C04FCC7989__INCLUDED_)
