#include "RobotCoordinator.h"

#include <cmath>

RobotCoordinator::RobotCoordinator(double _x, double _y, double _theta, double _v, double _w, double _t)
{
	x = _x, y = _y, theta = _theta, t = _t;
	v = _v, w = _w;
}

double RobotCoordinator::getv() const
{
	return v;
}

double RobotCoordinator::getw() const
{
	return w;
}

double RobotCoordinator::getTime() const
{
	return t;
}

Point RobotCoordinator::getPos() const
{
	return Point{ x, y };
}

double RobotCoordinator::getTheta() const
{
	return theta;
}

bool RobotCoordinator::stopped() const
{
	return abs(v) < 1e-6 && abs(w) < 1e-6;
}

Point RobotCoordinator::globalToRobot(const Point & p) const
{
	double dx = p.x - x;
	double dy = p.y - y;

	double rx = cos(theta) * dx - sin(theta) * dy;
	double ry = sin(theta) * dx + cos(theta) * dy;
	return Point{ rx, ry };
}

double RobotCoordinator::applyAngularVeclocity(double target_w, double maxa, double maxw, double decay, double t)
{
	if (w * decay + maxa * t < target_w) {
		target_w = w * decay + maxa * t;
	} else if (w * decay - maxa * t > target_w) {
		target_w = w * decay - maxa * t;
	}
	if (target_w > maxw) {
		target_w = maxw;
	}else if (target_w < -maxw) {
		target_w = -maxw;
	}
	return w = target_w;
}

double RobotCoordinator::applyLinearVeclocity(double target_v, double maxa, double maxv,double decay, double t)
{
	if (v * decay + maxa * t < target_v) {
		target_v = v * decay + maxa * t;
	}
	else if (v * decay - maxa * t > target_v) {
		target_v = v * decay - maxa * t;
	}
	if (target_v > maxv) {
		target_v = maxv;
	}
	else if (target_v < 0) {
		target_v = 0;
	}
	return v = target_v;
}

double RobotCoordinator::applyMovement(double tick)
{
	t += tick;
	double alpha = w * tick;
	
	double dx, dy;
	if (alpha > 1e-6) {
		double R = v / w;
		dx = (1 - cos(alpha))*R;
		dy = sin(alpha)*R;
	}else{
		dx = 0;
		dy = v * tick;
	}
	x += cos(theta)*dx + sin(theta)*dy;
	y += -sin(theta)*dx + cos(theta)*dy;

	theta += alpha;

	return v * tick;
}
