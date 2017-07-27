#pragma once
#include <cmath>

struct Point
{
	double x, y;

	friend double getDistance(const Point &q, const Point &p)
	{
		return sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y));
	}

	//q:origin; p direction
	friend double getDirection(const Point &q, const Point &p)
	{
		const double PI = acos(-1);
		double ans =  PI/2 - atan2(p.y - q.y, p.x - q.x);
		if (ans < 0) ans += 2 * PI;
		return ans;
	}
};

class RobotCoordinator
{
protected:
	double x, y, theta;
	double v, w;
	double t;

public:
	double applyAngularVeclocity(double target_w, double maxa, double maxw, double decay, double t);
	double applyLinearVeclocity(double target_v, double maxa, double maxv, double decay, double t);

	double applyMovement(double tick);

public:
	RobotCoordinator(double _x = 0, double _y = 0, double _theta = 0, double _v = 0, double _w = 0, double _t = 0);

	double getv() const;
	double getw() const;
	double getTime() const;
	Point getPos() const;
	double getTheta() const;
	bool stopped() const;

	Point globalToRobot(const Point &p) const;

};