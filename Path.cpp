#include "Path.h"

#include <algorithm>
#include <iostream>

Path::Path()
{
	beforePathTime = 0;
	errdis = 0;
	errtime = 0;
}

void Path::append(const Point & p, double t)
{
	path.emplace_back(p, t, 0.);
}

void Path::append(const Point & p)
{
	path.emplace_back(p, -1, 0.);
}

void Path::initPath(const RobotCoordinator &robot, double v, double w)
{
	const double PI = acos(-1);
	double nowTheta = robot.getTheta();
	Point now = robot.getPos();
	for (auto &p : path) {
		double dir = getDirection(now, p.p);
		if (p.t < 0) {
			double dis = getDistance(now, p.p);
			double angle = std::abs(dir - nowTheta);
			if (2 * PI - angle < angle) angle = 2 * PI - angle;
			double t = std::max(dis / v * 1.1, angle / w); // TODO: distance is approximate
			p.t = t;
		}
		nowTheta = dir;
		now = p.p;
	}
}

std::tuple<Point, double, double> Path::lookahead(RobotCoordinator &robot, double lookahead_distance, double lookahead_time)
{
	double dis = 0, ti = beforePathTime - robot.getTime();
	Point now = robot.getPos();
	for (auto p : record) {
		Point next = p.p;
		dis += getDistance(now, next);
		now = next;
	}
	while(!path.empty()){
		Point nextp = path.front().p;
		double nextt = path.front().t;
		dis += getDistance(now, nextp);
		ti += nextt;
		if (lookahead_distance >= dis || lookahead_time > ti) {
			record.emplace_back(nextp, ti + robot.getTime(), dis);
			beforePathTime += nextt;
			path.pop_front();
			now = nextp;
		} else {
			break;
		}
	}
	if (path.empty()) {
		return std::make_tuple(record.back().p, ti, dis);
	} else {
		//linear interpolation
		Point PA = now, PB = path.front().p;
		double tA = beforePathTime - robot.getTime();
		double tB = ti;
		double disA = dis - getDistance(PA, PB);
		double disB = dis;
		double per = std::max((lookahead_time - tA) / (tB - tA), (lookahead_distance - disA) / (disB - disA));
		Point P_res = Point{ PA.x * (1 - per) + PB.x * per, PA.y * (1 - per) + PB.y * per };
		double t_res = tA * (1 - per) + tB * per;
		double dis_res = disA * (1 - per) + disB * per;
		return std::make_tuple(P_res, t_res, dis_res);
	}
	return std::tuple<Point, double, double>();
}

bool Path::end()
{
	return path.empty() && record.empty();
}

void Path::applyMovement(RobotCoordinator & robot, double tick)
{
	Point startPos = robot.getPos();
	double movedis = robot.applyMovement(tick);
	Point endPos = robot.getPos();

	while (!record.empty()) {
		double sdis = getDistance(startPos, record.front().p);
		double edis = getDistance(endPos, record.front().p);
		if (sdis < edis || edis < 1e-2) {
			errdis = std::max(errdis, getDistance(record.front().p, startPos));
			errtime = std::max(errtime, abs(robot.getTime() - record.front().t));
			//std::cerr << getDistance(record.front().p, startPos) << " ";
			//std::cerr << robot.getTime() - record.front().t << std::endl;
			record.pop_front();
		}
		else break;
	}


	/*std::vector<Node> newRecord;

	for (auto p : record) {
		Point pos = p.p;
		double dis = p.dis;
		newRecord.emplace_back(pos, p.t, dis - movedis);
		if (dis < movedis) {
			for (auto passed : newRecord) {
				errdis = std::max(errdis, getDistance(passed.p, robot.getPos()));
				errtime = std::max(errtime, abs(robot.getTime() - passed.t));
			}
			newRecord.clear();
		}
	}
	record = newRecord;*/

}



double Path::getErrdis() const
{
	return errdis;
}

double Path::getErrtime() const
{
	return errtime;
}

