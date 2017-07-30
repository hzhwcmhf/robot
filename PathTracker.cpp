#include "PathTracker.h"
#include "defines.h"

#include <iostream>
#include <algorithm>
#include <tuple>
#include <exception>
#include <utility>
#include <cassert>

PathTracker::PathTracker()
{
	B = DefaultParameters::motor_control_B;
	ch = DefaultParameters::motor_control_ch;
	wheel_radius = DefaultParameters::wheel_radius;
	
	diffR = ch * B / 2;

	default_lookahead_distance_ratio = DefaultParameters::default_lookahead_distance_ratio;
	min_lookahead_distance = DefaultParameters::min_lookahead_distance;
	default_lookahead_time = DefaultParameters::default_lookahead_time;
	track_tick = DefaultParameters::track_tick;
	max_angular_acceleration = DefaultParameters::max_angular_acceleration;
	max_angular_veclocity = DefaultParameters::max_angular_veclocity;
	angular_decay = DefaultParameters::angular_decay;
	max_linear_acceleration = DefaultParameters::max_linear_acceleration;
	max_linear_veclocity = DefaultParameters::max_linear_veclocity;
	linear_decay = DefaultParameters::linear_decay;

	target_v = DefaultParameters::target_v;
	target_w = DefaultParameters::target_w;
	target_a = DefaultParameters::target_a;
	path_tick = DefaultParameters::path_tick;
}

PathTracker::WheelAngularVeclocity PathTracker::convertRobotToWheel(double v, double w) const
{
	double wL = (v + diffR * w) / wheel_radius;
	double wR = (v - diffR * w) / wheel_radius;
	return WheelAngularVeclocity{wL, wR};
}

PathTracker::WheelAngularVeclocity PathTracker::convertRobotToWheel(BodyVeclocity b) const
{
	return convertRobotToWheel(b.v, b.w);
}

std::vector<PathTracker::BodyVeclocity> PathTracker::turnToPath(RobotCoordinator & robot, Path & path)
{
	const static double PI = acos(-1);

	//Get first point and firt direction from path
	//If first point is too close, turn to first direction;
	//Otherwise, turn to first point.

	Point target = path.getFirstPoint();

	std::vector<BodyVeclocity> ans;

	double straight_dis = getDistance(robot.getPos(), target);
	double angle;
	if (straight_dis < 1e-2) {
		angle = path.getFirstDir();
		if (isinf(angle)) return ans;
		angle -= robot.getTheta();
		if (angle < -PI) angle += 2 * PI;
	} else {
		Point relativePos = robot.globalToRobot(target);
		angle = getDirection(Point{ 0,0 }, relativePos);
	}
	if (angle > PI) angle -= 2 * PI;

	while (true) {
		assert(!isnan(angle) && !isinf(angle));
		double noww = robot.getw(), w;

		if (std::abs(angle) < 0.01 && std::abs(noww) < 0.01) break;

		if (noww * noww / max_angular_acceleration / 2 > std::abs(angle)) {
			w = sqrt(std::abs(angle) * 2 * max_angular_acceleration);
		} else {
			w = std::abs(noww) + max_angular_acceleration;
		}
		if (angle < 0) w = -w;

		w = robot.applyAngularVeclocity(w, max_angular_acceleration, max_angular_veclocity, angular_decay, track_tick);
		double v = robot.applyLinearVeclocity(0, max_linear_acceleration, max_linear_veclocity, linear_decay, track_tick);
		if (std::abs(v) > 1e-3) return ans;
		robot.applyMovement(track_tick);

		angle -= w * track_tick;
		ans.emplace_back(0., w);
	}

	return ans;
}

PathTracker::BodyVeclocity PathTracker::trackPathForOneStep(RobotCoordinator & robot, Path & path)
{
	if (path.curveEnd()) {
		double w, v;
		w = robot.applyAngularVeclocity(0, max_angular_acceleration, max_angular_veclocity, angular_decay, track_tick);
		v = robot.applyLinearVeclocity(0, max_linear_acceleration, max_linear_veclocity, linear_decay, track_tick);
		return BodyVeclocity{ v, w };
	}

	//pure-pursuit algorithm
	const static double PI = acos(-1);
	Point target;
	double time, distance;
	double lookahead_distance = std::max(default_lookahead_distance_ratio * robot.getv(), min_lookahead_distance);
	std::tie(target, time, distance) = path.lookahead(robot, lookahead_distance, default_lookahead_time);
	if (time < track_tick) time = track_tick;
	double straight_dis = getDistance(robot.getPos(), target);
	
	//pure-pursuit point target
	Point relativePos = robot.globalToRobot(target);
	double r, v, w, theta;
	r = (relativePos.x*relativePos.x + relativePos.y*relativePos.y) / 2 / relativePos.x;
	theta = PI - 2 * atan2(relativePos.y, relativePos.x);
	double arc_dis = theta * r;
	if (std::abs(theta) < 0.1) arc_dis = straight_dis;

	if (distance < 1e-3) {
		v = 0;
		w = 0;
		r = 0;
	}else if (distance + 1e-6 >= lookahead_distance){
		// rsin = y; r-rcos = x
		// -> r = (x2+y2)/2x
		
		if (relativePos.y < -0.1) {
			throw std::runtime_error("Path Tracing Failed! Too Curved, Not Enough Path Points or Time Limit is too Strict!");
		}
		
		v = arc_dis / time * 1.05;
		w = v / r;
		if (relativePos.y < 0) {
			r = 0, v = 0, w = 0;
		}
	} else {
		// path is end, must stop at end of the path
		
		//v = arc_dis / time * 2;  //There should be 2 to arrive on time.
		v = arc_dis / time * 1.05; //Use 1.05 instead of 2, prefer to delay rather than sudden speed change
		w = v / r;

		//time is not enough
		if (time * max_linear_acceleration < v || time * max_angular_acceleration < w) {
			v = sqrt(arc_dis * max_linear_acceleration * 2);
			w = v / r;
		}
	}

	w = robot.applyAngularVeclocity(w, max_angular_acceleration, max_angular_veclocity, angular_decay, track_tick);
	if(std::abs(w) > 0.1)
		v = w * r;
	v = robot.applyLinearVeclocity(v, max_linear_acceleration, max_linear_veclocity, linear_decay, track_tick);

	assert(!isnan(v) && !isnan(w));
	path.applyMovement(robot, track_tick);
	//std::cerr << robot.getPos().x << " " << robot.getPos().y << " " << v << std::endl;

	return BodyVeclocity{ v, w };
}

std::vector<PathTracker::BodyVeclocity> PathTracker::trackPath(RobotCoordinator &robot, Path &path, bool debug)
{
	std::vector<BodyVeclocity> controls;

	while (!path.end()) {
		//loop: make movement from a stop point to another.

		//Turn to appropriate direction
		std::vector<BodyVeclocity> turnControls = turnToPath(robot, path);
		controls.insert(controls.end(), turnControls.begin(), turnControls.end());
		
		//Generate path points from constraint points
		path.generatePath(robot, target_v, target_w, target_a, track_tick, path_tick);
		if(debug) path.printPathForDebug();
		
		//Use pure-pursuit algorithm to go along the path
		while (!path.curveEnd() || !robot.stopped()) {
			try {
				controls.push_back(trackPathForOneStep(robot, path));
				//std::cerr << controls.back().v << " " << controls.back().w << std::endl;
			}catch (const std::exception &e) {
				std::cerr << e.what() << std::endl;
				if(debug) return controls;
				throw e;
			}
		}
	}
	
	return controls;
}

