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
	//For MotorController
	//lookahead according path
	std::tuple<Point, double, double> lookahead(RobotCoordinator &robot, double lookahead_distance, double lookahead_time);
	//check path if end
	bool end();
	//move robot and record path
	void applyMovement(RobotCoordinator &robot, double tick);

	//generate time requirement of path
	void initPath(const RobotCoordinator &robot, double v, double w);

public:
	Path();

	/**
	append a path node
	input: path node coordinate, time requirement betweem this point and last point
	**/
	void append(const Point &p, double t);
	void append(const Point &p);
	
	//get error in distance
	double getErrdis() const;
	//get error in time
	double getErrtime() const;
};