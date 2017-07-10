#pragma once
#include "Path.h"
#include "RobotCoordinator.h"

#include <vector>

class MotorController
{
	double B; //distance between wheels
	double ch; //ICR coefficent
	double wheel_radius; 
	
	double diffR; //ICR_l + diffR = ICR_G; ICR_G + diffR = ICR_R

	double default_lookahead_ratio, min_lookahead;
	double default_lookahead_time;
	double track_tick, max_angular_acceleration, max_angular_veclocity, angular_decay;
	double max_linear_acceleration, max_linear_veclocity, linear_decay;

protected:
	double lookahead_ratio;

public:
	struct WheelAngularVeclocity
	{
		double lw, rw;
	};
	struct BodyVeclocity
	{
		double v, w;
	};

private:
	BodyVeclocity trackPathForOneStep(RobotCoordinator &robot, Path &path);

public:
	
	MotorController();
	
	/**
	input: linear veclocity(m/s), angular veclocity(rad/s, clockwise)
	output: wheel angular veclocity
	**/
	WheelAngularVeclocity convertRobotToWheel(double v, double w) const;

	std::vector<BodyVeclocity> trackPath(RobotCoordinator &robot, Path &path);
};