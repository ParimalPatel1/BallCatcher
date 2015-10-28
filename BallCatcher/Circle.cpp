// Circle.cpp: implementation of the Circle class.
//
//////////////////////////////////////////////////////////////////////


#include "Circle.hpp"

using namespace cv;
using namespace std;
#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Circle::Circle()
{
	this->m_dRadius=-1;		// error checking 
}

Circle::~Circle()
{

}
Circle::Circle(vector<Point> pointlist)
{
	if (pointlist.size() < 3) // need 3 or more points to create a circle for this method
	{
		this->~Circle(); // this proper way to exit constructor?? destruct externally?
		return;
	}
	points = pointlist; // does this copy or is by reference??
	if (this->IsPerpendicular(points) >= 0)
		this->CalcCircle(points);
	else
	{
		//this proper way to exit constructor ??
		this->m_dRadius = -1;
		return;
	}
}

Circle::Circle(Point &V1, Point &V2, Point &V3)
{
	this->m_dRadius=-1;		// error checking 

	Point pt1 = Point(V1.x, V1.y);
	Point pt2 = Point(V2.x, V2.y);
	Point pt3 = Point(V3.x, V3.y);
	
	if (!this->IsPerpendicular(pt1, pt2, pt3) )				this->CalcCircle(pt1, pt2, pt3);	
	else if (!this->IsPerpendicular(pt1, pt3, pt2) )		this->CalcCircle(pt1, pt3, pt2);	
	else if (!this->IsPerpendicular(pt2, pt1, pt3) )		this->CalcCircle(pt2, pt1, pt3);	
	else if (!this->IsPerpendicular(pt2, pt3, pt1) )		this->CalcCircle(pt2, pt3, pt1);	
	else if (!this->IsPerpendicular(pt3, pt2, pt1) )		this->CalcCircle(pt3, pt2, pt1);	
	else if (!this->IsPerpendicular(pt3, pt1, pt2) )		this->CalcCircle(pt3, pt1, pt2);	
	else { 
	//	TRACE("\nThe three pts are perpendicular to axis\n");
//		pt1->trace();			pt2->trace();			pt3->trace();
		
		//delete pt1;			delete pt2;				delete pt3;
		this->m_dRadius=-1;
		return ;
	}
//	delete pt1;				delete pt2;				delete pt3;

}
// returns point and the next if they are perindicular, otherwise, return -1 if no perpindicular points
int Circle::IsPerpendicular(vector<Point> points)
{
	double yDelta = 0;
	double xDelta = 0;
	int i;
	for (i = 0; i < points.size() - 1; i++)
	{

		yDelta = points[i + 1].y - points[i].y;
		xDelta = points[i + 1].x - points[i].x;
		if (fabs(yDelta) <= 0.0000001 || fabs(xDelta) <= 0.0000001) 
			{return i;}
	}
	return -1;// not 
}
bool Circle::IsPerpendicular(Point &pt1, Point &pt2, Point &pt3)
// Check the given point are perpendicular to x or y axis 
{
	double yDelta_a= pt2.y - pt1.y;
	double xDelta_a= pt2.x - pt1.x;
	double yDelta_b= pt3.y - pt2.y;
	double xDelta_b= pt3.x - pt2.x;
	

//	TRACE(" yDelta_a: %f xDelta_a: %f \n",yDelta_a,xDelta_a);
//	TRACE(" yDelta_b: %f xDelta_b: %f \n",yDelta_b,xDelta_b);

	// checking whether the line of the two pts are vertical
	if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001){
		//TRACE("The points are pependicular and parallel to x-y axis\n");
		return false;
	}

	if (fabs(yDelta_a) <= 0.0000001){
//		TRACE(" A line of two point are perpendicular to x-axis 1\n");
		return true;
	}
	else if (fabs(yDelta_b) <= 0.0000001){
//		TRACE(" A line of two point are perpendicular to x-axis 2\n");
		return true;
	}
	else if (fabs(xDelta_a)<= 0.000000001){
//		TRACE(" A line of two point are perpendicular to y-axis 1\n");
		return true;
	}
	else if (fabs(xDelta_b)<= 0.000000001){
//		TRACE(" A line of two point are perpendicular to y-axis 2\n");
		return true;
	}
	else return false ;
}

double Circle::CalcCircle(vector<Point> points)
{
	double yDelta_a = 0;
	double xDelta_a = 0;
	double yDelta_b = 0;
	double xDelta_b = 0;
	Point pt1, pt2, pt3; 
	/// for every circle, check if it has an 'orignal' center/radius, if it is orginal add it to the circle list, if not orginal, bump
	for (int i = 0; i < points.size() - 2; i++)
	{
		pt1 = points[i]; pt2 = points[i + 1]; pt3 = points[i + 2];
		yDelta_a = pt2.y - pt1.y;
		xDelta_a = pt2.x - pt1.x;
		yDelta_b = pt3.y - pt2.y;
		xDelta_b = pt3.x - pt2.x;
		// CASE 1::
		if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001) {
			this->m_Center.x = 0.5*(pt2.x + pt3.x);
			this->m_Center.y = 0.5*(pt1.y + pt2.y);
			this->m_dRadius = cv::norm(m_Center - pt1);		// calc. radius
			return this->m_dRadius;
		}

		// IsPerpendicular() assure that xDelta(s) are not zero
		// CASE 2::
		double aSlope = yDelta_a / xDelta_a; // 
		double bSlope = yDelta_b / xDelta_b;
		if (fabs(aSlope - bSlope) <= 0.000000001) {	// checking whether the given points are colinear. 	
			return -1;
		}
		// calc center
		this->m_Center.x = (aSlope*bSlope*(pt1.y - pt3.y) + bSlope*(pt1.x + pt2.x) - aSlope*(pt2.x + pt3.x)) / (2 * (bSlope - aSlope));
		this->m_Center.y = -1 * (m_Center.x - (pt1.x + pt2.x) / 2) / aSlope + (pt1.y + pt2.y) / 2;
		this->m_dRadius = cv::norm(m_Center - points[i]);	// calc. radius
	}
	return this->m_dRadius;
}
double Circle::CalcCircle(Point &pt1, Point &pt2, Point &pt3)
{
	double yDelta_a= pt2.y - pt1.y;
	double xDelta_a= pt2.x - pt1.x;
	double yDelta_b= pt3.y - pt2.y;
	double xDelta_b= pt3.x - pt2.x;
	
	if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001){
		this->m_Center.x= 0.5*(pt2.x + pt3.x);
		this->m_Center.y= 0.5*(pt1.y + pt2.y);
		this->m_dRadius= cv::norm(m_Center - pt1);		// calc. radius
		return this->m_dRadius;
	}	
	// IsPerpendicular() assure that xDelta(s) are not zero
	double aSlope=yDelta_a/xDelta_a; // 
	double bSlope=yDelta_b/xDelta_b;
	if (fabs(aSlope-bSlope) <= 0.000000001){	// checking whether the given points are colinear. 	
		//TRACE("The three pts are colinear\n");
		return -1;
	}

	// calc center
	this->m_Center.x= (aSlope*bSlope*(pt1.y - pt3.y) + bSlope*(pt1.x + pt2.x)- aSlope*(pt2.x+pt3.x) )/(2* (bSlope-aSlope) );
	this->m_Center.y = -1*(m_Center.x - (pt1.x+pt2.x)/2)/aSlope +  (pt1.y+pt2.y)/2;
	this->m_dRadius= cv::norm(m_Center - pt1);	// calc. radius
	return this->m_dRadius;
}

Point Circle::GetCenter()
{
	return this->m_Center;
}

double Circle::GetRadius()
{
	return this->m_dRadius;
}
