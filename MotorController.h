#pragma once
#include "Path.h"
#include "RobotCoordinator.h"

#include <vector>

class MotorController
{
protected:
	double B; //distance between wheels
	double ch; //ICR coefficent
	double wheel_radius; 
	
	double diffR; //ICR_l + diffR = ICR_G; ICR_G + diffR = ICR_R
	
	//pure pursuit algorithm need a lookahead point to pursuit
	//lookahead_distance indicates the minimum distance between lookahead point and robot
	//lookahead_distance =  min(default_lookahead_distance_ratio * speed, min_lookahead_distance)
	double default_lookahead_distance_ratio, min_lookahead_distance; 
	//lookahead_time indicates the minimum time between lookahead point and robot 
	double default_lookahead_time;

	//every movements of tracking path will cost track_tick(s) time
	double track_tick;
	
	//limit the robot angular speed
	//last_w * angular_decay - max_angular_acceleration <= w <= last_w * angular_decay + max_angular_acceleration
	// and:  -max_angular_veclocity <= w <= max_angular_veclocity
	double max_angular_acceleration, max_angular_veclocity, angular_decay; //(rad/s)
	//limit the robot linear speed
	//last_v * linear_decay - max_linear_acceleration <= v <= last_v * linear_decay + max_linear_acceleration
	// and:  0 <= v <= max_linear_veclocity
	double max_linear_acceleration, max_linear_veclocity, linear_decay;

	double target_v, target_w, target_a;
	double path_tick;

public:
	struct WheelAngularVeclocity
	{
		double lw, rw;
	};
	struct BodyVeclocity
	{
		double v, w;
		BodyVeclocity(double _v, double _w)
		{
			v = _v, w = _w;
		}
	};

protected:
	//Make turn to path. This policy can be replaced.
	std::vector<BodyVeclocity> turnToPath(RobotCoordinator &robot, Path &path);

private:
	BodyVeclocity trackPathForOneStep(RobotCoordinator &robot, Path &path);
public:
	
	MotorController();
	
	/**
	Convert robot body movement to wheel control signals
	input: linear veclocity(m/s), angular veclocity(rad/s, clockwise)
	output: wheel angular veclocity
	**/
	WheelAngularVeclocity convertRobotToWheel(double v, double w) const;
	WheelAngularVeclocity convertRobotToWheel(BodyVeclocity b) const;


	/**
	use robot to track path
	input: robot, path
	output: a array of body movement
	**/
	std::vector<BodyVeclocity> trackPath(RobotCoordinator &robot, Path &path, bool debug=false);
};