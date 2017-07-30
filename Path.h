#pragma once
#include "RobotCoordinator.h"

#include <vector>
#include <deque>
#include <tuple>


class Path
{
protected:
	struct Node
	{
		Point p;
		double t, dir;
		int canStop;
		Node(Point _p, double _t, double _dir = -1, int _canStop = -1)
		{
			p = _p, t = _t;
			dir = _dir, canStop = _canStop;
		}
	};

	std::deque<Node> constraintPoints;
	std::deque<Node> path;
	double beforePathTime, errdis, errtime;
	std::deque<Node> record;

protected:
	//generate time requirement of path. This policy can be replaced.
	void initPath(const RobotCoordinator &robot, double targetV, double targetW, double targetA, double track_tick);
	//predict direction by position. This policy can be replaced.
	std::vector<double> predictDirections(Point p) const;
	//predict whether stop. This policy can be replaced.
	std::vector<int> predictStop(Point p, const std::vector<double> &dir) const;
	//predict absolute time of each point. This policy can be replaced.
	std::vector<double> predictTime(const RobotCoordinator &robot, const std::vector<double> &dir, std::vector<int> canStop, double targetV, double targetW, double targetA) const;
	//interpolate until first stop. This policy can be replaced.
	void interpolateUntilStop(const RobotCoordinator &robot, double targetV, double path_tick);

public:
	//For PathTracker
	//lookahead according path
	std::tuple<Point, double, double> lookahead(RobotCoordinator &robot, double lookahead_distance, double lookahead_time);
	//check all path is end
	bool end();
	//check this partial of path if end
	bool curveEnd();
	//move robot and record path
	void applyMovement(RobotCoordinator &robot, double tick);

	//generate path (to first stop point) from constraintPoints
	void generatePath(const RobotCoordinator &robot, double targetV, double targetW, double targetA, double track_tick, double path_tick);

	void printPathForDebug();

	Point getFirstPoint() const;
	double getFirstDir() const;

public:
	Path();

	static const double NoTimeReq/* = numeric_limits<double>::infinity()*/;
	static const double NoDirReq/* = numeric_limits<double>::infinity()*/;

	// append constraint points
	// timeReq<0 : relative time require
	// timeReq>0 : absolute time require
	// timeReq=NoTimeReq : no time require
	// dir = NoDirReq; no require
	// dir > -2*PI && dir < 2*PI clockwise
	// canStop = -1: decide by program
	// canStop = 0 : cannotStop
	// canStop = 1 : mustStop
	void append(const Point &p, double timeReq = NoTimeReq, double dir = NoDirReq, int canStop = -1);

	/**
	append a path node
	input: path node coordinate, relative time requirement
	**/
	void appendRaw(const Point &p, double t);
	
	//get error in distance
	double getErrdis() const;
	//get error in time
	double getErrtime() const;

	void printConstrainPointsForDebug() const;
};