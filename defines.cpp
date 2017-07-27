#pragma once

#include <cmath>

namespace DefaultParameters
{
	double MotorController_B = 0.38; //distance between wheels (m)

	//Wang, Tianmiao, et al. "Analysis and experimental kinematics of a skid-steering wheeled robot based on a laser scanner sensor." Sensors 15.5 (2015): 9681-9702.
	double MotorController_ch = 1.5; //ICR coefficent for pioneer3-AT; ideal: 1

	double wheel_radius = 0.11; //(m)

	//pure pursuit algorithm need a lookahead point to pursuit
	//lookahead_distance indicates the minimum distance between lookahead point and robot
	//lookahead_distance =  min(default_lookahead_distance_ratio * speed, min_lookahead_distance)
	double default_lookahead_distance_ratio = 0.5, min_lookahead_distance = 0; //(m)
	//lookahead_time indicates the minimum time between lookahead point and robot 
	double default_lookahead_time = 0.5; //(s)

	//every movements of tracking path will cost track_tick(s) time
	double track_tick = 0.01; //(s)

	//limit the robot angular speed
	//last_w * angular_decay - max_angular_acceleration <= w <= last_w * angular_decay + max_angular_acceleration
	// and:  -max_angular_veclocity <= w <= max_angular_veclocity
	double max_angular_acceleration = 0.5, max_angular_veclocity = 1, angular_decay = pow(0.5, track_tick); //(rad/s^2, rad/s, decay coef per tick)
	//limit the robot linear speed
	//last_v * linear_decay - max_linear_acceleration <= v <= last_v * linear_decay + max_linear_acceleration
	// and:  0 <= v <= max_linear_veclocity
	double max_linear_acceleration = 0.25, max_linear_veclocity = 0.5, linear_decay = pow(0.7, track_tick);//(m/s^2, m/s, decay coef per tick)

	//target linear_veclocity, angular_veclocity, linear_acceleration for prediction
	double target_v = max_linear_veclocity * 0.4;
	double target_w = max_angular_veclocity * 0.5;
	double target_a = max_linear_acceleration * 0.2;
	double path_tick = track_tick * 5;
}