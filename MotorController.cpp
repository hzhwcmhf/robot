#include "MotorController.h"
#include "defines.h"

#include <iostream>
#include <algorithm>
#include <tuple>
#include <exception>

MotorController::MotorController()
{
	B = DefaultParameters::MotorController_B;
	ch = DefaultParameters::MotorController_ch;
	wheel_radius = DefaultParameters::wheel_radius;
	
	diffR = ch * B / 2;

	default_lookahead_ratio = DefaultParameters::default_lookahead_ratio;
	min_lookahead = DefaultParameters::min_lookahead;
	default_lookahead_time = DefaultParameters::default_lookahead_time;
	track_tick = DefaultParameters::track_tick;
	max_angular_acceleration = DefaultParameters::max_angular_acceleration;
	max_angular_veclocity = DefaultParameters::max_angular_veclocity;
	angular_decay = DefaultParameters::angular_decay;
	max_linear_acceleration = DefaultParameters::max_linear_acceleration;
	max_linear_veclocity = DefaultParameters::max_linear_veclocity;
	linear_decay = DefaultParameters::linear_decay;
}

MotorController::WheelAngularVeclocity MotorController::convertRobotToWheel(double v, double w) const
{
	double wL = (v + diffR * w) / wheel_radius;
	double wR = (v - diffR * w) / wheel_radius;
	return WheelAngularVeclocity{wL, wR};
}

MotorController::BodyVeclocity MotorController::trackPathForOneStep(RobotCoordinator & robot, Path & path)
{
	Point target;
	double time, distance;
	double lookahead_distance = std::max(lookahead_ratio * robot.getv(), min_lookahead);
	std::tie(target, time, distance) = path.lookahead(robot, lookahead_distance, default_lookahead_time);
	if (time < track_tick) time = track_tick;

	double r, v, w;
	if (distance + 1e-6 >= lookahead_distance ) {
		// do a curve in lookahead_distance, and move straightly in the left distance
		// rsin+dcos = y; r-rcos+dsin = x
		// -> r = (x2+y2-d2)/2x
		double d = distance - lookahead_distance;
		Point relativePos = robot.globalToRobot(target);
		if (relativePos.y < 0) {
			std::cerr << "Path Tracing Failed! Too Curved, Not Enough Sample or Time Limited is too Strict!" << std::endl;
			throw std::runtime_error("Path Tracing Failed! Too Curved, Not Enough Sample or Time Limited is too Strict!");
		}
		r = (relativePos.x*relativePos.x + 
				relativePos.y*relativePos.y
				- d*d) / 2 / relativePos.x;
		v = lookahead_distance / time * 1.2; // TODO: distance is approximate, try to solve theta
		w = v / r;

	} else {
		// path is end, must stop at end of the path
		Point relativePos = robot.globalToRobot(target);
		r = (relativePos.x*relativePos.x + relativePos.y*relativePos.y
				) / 2 / relativePos.x;
		v = distance / time * 2; // TODO: distance is approximate, try to solve theta
		w = v / r;
		if (time * max_linear_acceleration < v || time * max_angular_acceleration < w)
			v = sqrt(distance * max_linear_acceleration * track_tick);
		w = v / r;
	}

	w = robot.applyAngularVeclocity(w, max_angular_acceleration, max_angular_veclocity, angular_decay, track_tick);
	v = w * r;
	v = robot.applyLinearVeclocity(v, max_linear_acceleration, max_linear_veclocity, linear_decay, track_tick);

	path.applyMovement(robot, track_tick);
	std::cerr << robot.getPos().x << " " << robot.getPos().y << " " << v << std::endl;

	return BodyVeclocity{ v, w };
}

std::vector<MotorController::BodyVeclocity> MotorController::trackPath(RobotCoordinator &robot, Path &path)
{
	std::vector<BodyVeclocity> controls;
	lookahead_ratio = default_lookahead_ratio;
	while (!path.end())
	{
		controls.push_back(trackPathForOneStep(robot, path));
		//std::cerr << controls.back().v << " " << controls.back().w << std::endl;
	}
	return controls;
}

