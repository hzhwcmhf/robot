#pragma once
#include "RobotCoordinator.h"

#include <vector>
#include <deque>
#include <tuple>

class Path
{
	struct Node
	{
		Point p;
		double t, dis;
		Node(Point _p, double _t, double _dis)
		{
			p = _p, t = _t, dis = _dis;
		}
	};

	std::deque<Node> path;
	double beforePathTime, errdis, errtime;
	std::deque<Node> record;

public:
	Path();

	void append(const Point &p, double t);
	std::tuple<Point, double, double> lookahead(RobotCoordinator &robot, double lookahead_distance, double lookahead_time);
	bool end();
	void applyMovement(RobotCoordinator &robot, double tick);

	double getErrdis() const;
	double getErrtime() const;
};