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
	T largest = input[0];
	int inputsize = input.size() - 1; 
	for (int i = 0; i < inputsize; i++) // array index is zero based
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
const int minsize = 7; // at least 5 points to fit ellipse with fitellipse
const float tolerance = 0.1; // make dynamic tolerance over static?
const float added_tolerance = 0.05;
void findTheBall(std::vector<cv::Point> v, cv::RotatedRect &L, bool drawmode) // use a copy or pass by reference??
{
	std::vector<cv::RotatedRect> t(1); // temp start with size one
	std::vector<cv::Point>::iterator iterb; // begin itr LEARN HOW TO USE ITERATORS FIRST
	std::vector<cv::Point>::iterator itere; // end itr
	float eto;// eccentricity of original ellipse
	float etn = 0;// eccentriticy of ellipse with added points
	if (v.size() < minsize - 1)
		return; // ERROR not enough points
	std::vector<std::vector<cv::Point>> curves; // holds all sets of points, the one in front will be the largest, roundest set of points.
	int vsize = v.size() - 1 - minsize;
	for (int i = 0; i < vsize; i += minsize) // for every point in the vector, sort into 
	{
		iterb = v.begin() + i;
		itere = v.begin() + minsize + i;
		std::vector<cv::Point> pvec(iterb, itere); // point vector 
		t[0] = cv::fitEllipse(pvec); // start with first minsize(5) points
		eto = calcEccen(t[0]); // get original
		if (eto >= 0 && eto < tolerance) // if not already a circle or hyperbola
		{ // satisfactory circle candidate acquired, add more points until it becomes less circle like or it becomes alot like a circle
			for (; i < vsize + minsize; i++ ) // one point at a time
			{
				pvec.push_back(v[i]); // add another point
				t[0] = cv::fitEllipse(cv::Mat(pvec));
				etn = calcEccen(t[0]);	
				if (etn < eto)
					eto = etn;
				else
					break;
				//else if ((eto - etn) > 0.2)
					//break;
			} // while new points don't offset tolerance
			pvec.pop_back(); // pop because the last point added broke the loop
			i--;
			if (t[0].size.area() > 50)
				curves.push_back(pvec); // add the finished "circle like" ellipse to the list
		}
	}
	if (curves.size() > 0)
		L = cv::fitEllipse(biggestVector<std::vector<cv::Point>>(curves)); // find largest set of points, calculated rotated Rect from them, then set L to that Rect
	if (drawmode == true)
	{
		cv::Mat ellipseimage = cv::Mat::ones(cv::Size(640, 480), CV_8UC3);
		for (int i = 0; i < curves.size(); i++)
		{
			cv::ellipse(ellipseimage, cv::fitEllipse(curves[i]), cv::Scalar(0, 0, 0), 2);
		}
		cv::imshow("ellipes", ellipseimage);
		cv::waitKey(30);
	}
	return;
}

