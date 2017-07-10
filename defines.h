#pragma once

namespace DefaultParameters
{
	extern double MotorController_B; //distance between wheels (m)
	
	//Wang, Tianmiao, et al. "Analysis and experimental kinematics of a skid-steering wheeled robot based on a laser scanner sensor." Sensors 15.5 (2015): 9681-9702.
	extern double MotorController_ch; //ICR coefficent
	
	extern double wheel_radius; //(m)

	//pure pursuit algorithm need a lookahead point to pursuit
	//lookahead_distance indicates the minimum distance between lookahead point and robot
	//lookahead_distance =  min(default_lookahead_distance_ratio * speed, min_lookahead_distance)
	extern double default_lookahead_distance_ratio, min_lookahead_distance;
	//lookahead_time indicates the minimum time between lookahead point and robot 
	extern double default_lookahead_time;

	//every movements of tracking path will cost track_tick(s) time
	extern double track_tick;

	//limit the robot angular speed
	//last_w * angular_decay - max_angular_acceleration <= w <= last_w * angular_decay + max_angular_acceleration
	// and:  -max_angular_veclocity <= w <= max_angular_veclocity
	extern double max_angular_acceleration, max_angular_veclocity, angular_decay; //(rad/s^2, rad/s, decay coef per tick)
	//limit the robot linear speed
	//last_v * linear_decay - max_linear_acceleration <= v <= last_v * linear_decay + max_linear_acceleration
	// and:  0 <= v <= max_linear_veclocity
	extern double max_linear_acceleration, max_linear_veclocity, linear_decay; //(m/s^2, m/s, decay coef per tick)
	
}