#pragma once

namespace DefaultParameters
{
	extern double MotorController_B; //distance between wheels (m)
	
	//Wang, Tianmiao, et al. "Analysis and experimental kinematics of a skid-steering wheeled robot based on a laser scanner sensor." Sensors 15.5 (2015): 9681-9702.
	extern double MotorController_ch; //ICR coefficent
	
	extern double wheel_radius; //(m)

	extern double default_lookahead_ratio;
	extern double min_lookahead;
	extern double default_lookahead_time;
	extern double track_tick;
	extern double max_angular_acceleration, max_angular_veclocity, angular_decay;
	extern double max_linear_acceleration, max_linear_veclocity, linear_decay;
	
}