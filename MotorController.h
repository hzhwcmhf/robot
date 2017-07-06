#pragma once

class MotorController
{
	double B; //distance between wheels
	double ch; //ICR coefficent
	double wheel_radius; 
	
	double diffR; //ICR_l + diffR = ICR_G; ICR_G + diffR = ICR_R
	
public:
	struct WheelAngularVeclocity
	{
		double lw, rw;
	};
	
	MotorController();
	
	/**
	input: linear veclocity(m/s), angular veclocity(rad/s, clockwise)
	output: wheel angular veclocity
	**/
	WheelAngularVeclocity convertRobotToWheel(double v, double w);
};