#include "circledetect.hpp"



float calcEccen(cv::RotatedRect rec)
{
	float ecc; // calculated eccentrinity thats returned
	float a, b; // a = length of semi major axis, b = length semi minor axis
	a = rec.size.width >= rec.size.height ? rec.size.width : rec.size.height;
	b = rec.size.width < rec.size.height ? rec.size.width : rec.size.height;
	ecc = 1.0 - ((b * b) / (a * a)); // 
	ecc = std::sqrtf(ecc);// first order for ellipse
	return ecc;
}

template <typename T>
T biggestVector(std::vector<T> const& input)
{
	int m = 0; // current maximum
	T largest;
	for (int i = 0; i < input.size(); i++)
	{
		if (input[i].size() > m)
		{
			m = input[i].size();
			largest = input[i];
		}
	}
	return largest;
}

//function - calculate largest and most circular set of points
// input v - input points
// output o - largest sequential most circular points
const int minsize = 8; // at least 5 points to fit ellipse with fitellipse
const float tolerance = 0.3; // make dynamic tolerance over static?
const float added_tolerance = 0.07;
void calcEccentricity(std::vector<cv::Point> v, cv::RotatedRect &L) // use a copy or pass by reference??
{
	std::vector<cv::RotatedRect> t; // temp 
	std::vector<cv::Point>::iterator iterb; // begin itr LEARN HOW TO USE ITERATORS FIRST
	std::vector<cv::Point>::iterator itere; // end itr
	float eto; //temp calculation for eccentricity
	float etn;
	if (v.size() < minsize - 1)
		return;
	std::vector<std::vector<cv::Point>> curves; // holds all sets of points, the one in front will be the largest, roundest set of points.
	for (int i = minsize; i < v.size() - 1; i++) // for every point in the vector, sort into 
	{
		iterb = v.begin() + i;
		itere = v.begin() + minsize + i - 1;
		std::vector<cv::Point> pvec(iterb, itere); // point vector 
		t[i] = cv::fitEllipse(pvec); // start with first minsize(5) points
		eto = calcEccen(t[i]); // get original
		if (eto >= 0 && eto < tolerance) // if not already a circle or hyperbola
		{
			do {
				i++;
				pvec.push_back(v[i]); // add another point
				t[i] = cv::fitEllipse(cv::Mat(pvec));
				etn = calcEccen(t[i]);
			} while ((etn - eto) < added_tolerance || (i < v.size() - 1)); // while new points don't offset tolerance
			pvec.pop_back(); // pop because the last point added broke the loop
			i--;
			curves.push_back(pvec); // add the finished "circle like" ellipse to the list
		}
		i += minsize;
	}
	L = cv::fitEllipse(biggestVector<std::vector<cv::Point>>(curves)); // find largest set of points, calculated rotated Rect from them, then set L to that Rect
	return;
}

