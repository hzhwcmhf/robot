#pragma warning(disable:4996)
#include <iostream>
#include <cmath>
#include "PathTracker.h"
#include "defines.h"

using namespace std;
const double PI = acos(-1);
int main()
{
	freopen("data.out", "w", stdout);

	cout << DefaultParameters::motor_control_B << " "
		<< DefaultParameters::motor_control_ch << " "
		<< DefaultParameters::wheel_radius << endl;
	cout << DefaultParameters::max_linear_veclocity << " "
		<< DefaultParameters::max_angular_veclocity << endl;
	cout << DefaultParameters::track_tick << endl;

	Path path;

	//void append(const Point &p, double timeReq = NoTimeReq, double dir = NoDirReq, int canStop = -1);
	// timeReq<0 : relative time require
	// timeReq>0 : absolute time require
	// timeReq=Path::NoTimeReq : no time require

	// dir: reach the points in this direction. (If you want to take turn at some point, see example5)
	// dir = Path::NoDirReq; no require
	// dir > -2*PI && dir < 2*PI clockwise

	// canStop = -1: decide by program
	// canStop = 0 : cannotStop
	// canStop = 1 : mustStop

	//example1
	path.append(Point{ 0, 1 }, Path::NoTimeReq);
	path.append(Point{ 0, 2 }, -4);
	path.append(Point{ 0, 3 }, Path::NoTimeReq);
	path.append(Point{ 0, 4 }, 20);
	path.append(Point{ 1, 4 }, Path::NoTimeReq);
	path.append(Point{ 2, 4 }, -4);
	path.append(Point{ 3, 4 }, 50);
	path.append(Point{ 4, 4 }, 60);

	//example2
	/*path.append(Point{ 0, 0 }, Path::NoTimeReq, PI / 2, 0);
	path.append(Point{ 0, -3 }, Path::NoTimeReq, PI/2, 0);
	path.append(Point{ 3, -3 }, Path::NoTimeReq, 0, 0);
	path.append(Point{ 3, 0 }, Path::NoTimeReq, -PI/2, 0);
	path.append(Point{ 0, 0 }, Path::NoTimeReq, 0, 0);*/

	//example3
	/*path.append(Point{ 0, -3 }, Path::NoTimeReq, PI / 2);
	path.append(Point{ 3, -3 }, Path::NoTimeReq, 0);
	path.append(Point{ 3, 0 }, Path::NoTimeReq, 0);
	path.append(Point{ 0, 0 }, Path::NoTimeReq, 0);*/

	//example4
	/*for (int i = 0; i < 4; i++) {
		path.append(Point{ 0, 3.*i }, Path::NoTimeReq, (i&1)?PI/4:-PI/4, 0);
	}*/

	//example5
	
	/*path.append(Point{ 0, 0 }, Path::NoTimeReq, PI / 2 , 0);
	for (int i = 1; i < 4; i++) {
		path.append(Point{ 0, 3.*i }, Path::NoTimeReq, -PI / 2, 1);
		path.append(Point{ 0, 3.*i }, Path::NoTimeReq, PI / 2 , 0);
	}*/

	//example6
	/*path.append(Point{-1 * 0.01, -sin(1 * 2 * PI / 500) });
	for (int i = 2; i < 500; i++) {
		path.append(Point{-i * 0.01, -sin(i * 2 * PI / 500) });
	}*/
	
	//example7
	/*for (int i = 1; i < 50; i++) {
		double r = i / 20.;
		double a = i / 500. * 2 * PI;
		double y = r * cos(a);
		double x = r * sin(a);
		path.append(Point{ x, y });
	}*/

	RobotCoordinator robot;
	//RobotCoordinator robot(0, 0, 0.1, 0.1, 0, 5);  //change the start status of robot
	cout << robot.getPos().x << " " << robot.getPos().y << " " << robot.getTheta() << " " << robot.getTime() << endl;
	PathTracker pt;

	cout << endl;
	path.printConstrainPointsForDebug();
	cout << endl;
	auto control = pt.trackPath(robot, path, true);
	cout << endl;

	for (auto c : control) {
		auto ans = pt.convertRobotToWheel(c);
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