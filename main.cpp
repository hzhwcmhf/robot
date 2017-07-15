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
	cout << DefaultParameters::max_linear_veclocity << " "
		<< DefaultParameters::max_angular_veclocity << endl;
	cout << DefaultParameters::track_tick << endl;

	Path path;

	path.append(Point{ 0, 0.1 });
	for (int i = 2; i < 50; i++) {
		path.append(Point{ 0, 0.1*i });
	}

	/*path.append(Point{-1 * 0.1, sin(1 * 2 * PI / 500) }, -1);
	for (int i = 2; i < 500; i++) {
		path.append(Point{-i * 0.01, sin(i * 2 * PI / 500) }, -1);
	}*/
	
	/*for (int i = 1; i < 50; i++) {
		double r = i / 20.;
		double a = i / 500. * 2 * PI;
		double y = r * cos(a);
		double x = r * sin(a);
		path.append(Point{ x, y });
	}*/

	RobotCoordinator robot;
	MotorController mc;

	auto control = mc.trackPath(robot, path);

	for (auto c : control) {
		auto ans = mc.convertRobotToWheel(c);
		cout << ans.lw << " " << ans.rw << endl;
	}

	cerr << path.getErrdis() << " " << path.getErrtime() << endl;

	
	/*double v = 1;
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
	}*/

	return 0;
}