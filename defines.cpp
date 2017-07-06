#pragma once

namespace DefaultParameters
{
	double MotorController_B = 0.38; //distance between wheels (m)

	//Wang, Tianmiao, et al. "Analysis and experimental kinematics of a skid-steering wheeled robot based on a laser scanner sensor." Sensors 15.5 (2015): 9681-9702.
	double MotorController_ch = 1.5; //ICR coefficent for pioneer3-AT; ideal: 1

	double wheel_radius = 0.11; //(m)

}