#pragma once

#include <vector>

class SplineInterpolate
{
	std::vector<double> a, b, c, vxvec, vyvec;
	std::vector<double> xvec, yvec, tvec;
public:
	//spline interpolation with derivative of start and end point
	void start(double t, double x, double y, double vx, double vy);
	void end(double t, double x, double y, double vx, double vy);
	void append(double t, double x, double y);
	//get interpolated point of time t
	std::vector<std::tuple<double, double>> get(const std::vector<double> &tt) const;
};