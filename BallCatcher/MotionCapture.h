#ifndef MotionCapture_H
#define MotionCapture_H
struct Point3D {
	double x;
	double y;
	double z;
	time_t timestamp;
	Point3D(double x_in = 0.0, double y_in = 0.0, double z_in = 0.0, double timestamp_in = 0.0) {
		x = x_in;
		y = y_in;
		z = z_in;
		timestamp = timestamp_in;
	}
};
Point3D calculateObjectPosition(std::vector<Point3D> &points, Point3D point1, Point3D point2);
#endif