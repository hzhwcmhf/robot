#include "MotorController.h"

#include "defines.h"

MotorController::MotorController()
{
	B = DefaultParameters::MotorController_B;
	ch = DefaultParameters::MotorController_ch;
	wheel_radius = DefaultParameters::wheel_radius;
	
	diffR = ch * B / 2;
}

MotorController::WheelAngularVeclocity MotorController::convertRobotToWheel(double v, double w)
{
	double wL = (v + diffR * w) / wheel_radius;
	double wR = (v - diffR * w) / wheel_radius;
	return WheelAngularVeclocity{wL, wR};
}