#pragma warning(disable:4996)
#include <iostream>
#include <cmath>
#include "MotorController.h"
#include "defines.h"

using namespace std;
const double PI = acos(-1);
int main()
{
	freopen("data.out", "w", stdout);

	cout << DefaultParameters::MotorController_B << " "
		<< DefaultParameters::MotorController_ch << " "
		<< DefaultParameters::wheel_radius << endl;


	MotorController mc;
	double v = 1;
	double w = 2*PI;
	for (int i = 0; i < 50; i++)
	{
		auto ans = mc.convertRobotToWheel(0, w);
		cout << ans.lw << " " << ans.rw << endl;
	}
	for (int i = 0; i < 500; i++)
	{
		auto ans = mc.convertRobotToWheel(v * i / 100, w);
		cout << ans.lw << " " << ans.rw << endl;
	}

	return 0;
}