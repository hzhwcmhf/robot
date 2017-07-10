#pragma once

#include <cmath>

namespace DefaultParameters
{
	double MotorController_B = 0.38; //distance between wheels (m)

	//Wang, Tianmiao, et al. "Analysis and experimental kinematics of a skid-steering wheeled robot based on a laser scanner sensor." Sensors 15.5 (2015): 9681-9702.
	double MotorController_ch = 1.5; //ICR coefficent for pioneer3-AT; ideal: 1

	double wheel_radius = 0.11; //(m)

	double default_lookahead_ratio = 0.3;
	double min_lookahead = 0.3;
	double default_lookahead_time = 0.3;
	double track_tick = 0.01;
	double max_angular_acceleration = 0.5, max_angular_veclocity = 1, angular_decay = pow(0.5, track_tick);
	double max_linear_acceleration = 0.25, max_linear_veclocity = 0.5, linear_decay = pow(0.8, track_tick);
}