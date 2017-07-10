#pragma once
#include <cmath>

struct Point
{
	double x, y;

	friend double getDistance(const Point &q, const Point &p)
	{
		return sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y));
	}
};

class RobotCoordinator
{
	double x, y, theta;
	double v, w;
	double t;
	
public:
	RobotCoordinator(double _x = 0, double _y = 0, double _theta = 0, double _t = 0);

	double getv() const;
	double getTime() const;
	Point getPos() const;
	double getTheta() const;

	Point globalToRobot(const Point &p) const;

	double applyAngularVeclocity(double target_w, double maxa, double maxw, double decay, double t);
	double applyLinearVeclocity(double target_v, double maxa, double maxv, double decay, double t);

	double applyMovement(double tick);
};