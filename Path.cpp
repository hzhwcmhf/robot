#include "Path.h"
#include "SplineInterpolate.h"

#include <algorithm>
#include <iostream>
#include <climits>
#include <cassert>


const double Path::NoTimeReq = std::numeric_limits<double>::infinity();
const double Path::NoDirReq = std::numeric_limits<double>::infinity();

Path::Path()
{
	beforePathTime = 0;
	errdis = 0;
	errtime = 0;
}

void Path::append(const Point & p, double timeReq, double dir, int canStop)
{
	const static double PI = acos(-1);
	if (dir < 0) dir += 2 * PI;  //0 <= dir <= 2*PI or dir == INF
	constraintPoints.emplace_back(p, timeReq, dir, canStop);
}

void Path::appendRaw(const Point & p, double t)
{
	path.emplace_back(p, t);
}

void Path::generatePath(const RobotCoordinator & robot, double targetV, double targetW, double targetA, double track_tick, double path_tick)
{
	beforePathTime = robot.getTime();
	if (!path.empty()) {
		return;
	}

	//Generate path from constraint points, until first stop point.
	std::vector<double> dir = predictDirections(robot.getPos());
	std::vector<int> canStop = predictStop(robot.getPos(), dir);
	std::vector<double> t = predictTime(robot, dir, canStop, targetV, targetW, targetA);

	for (int i = 0; i < (int)canStop.size(); i++) {
		constraintPoints[i].canStop = canStop[i];
		constraintPoints[i].t = t[i];
		if (canStop[i]) break;
	}
	interpolateUntilStop(robot, targetV, path_tick);

	while (!constraintPoints.empty()) {
		int c = constraintPoints.front().canStop;
		constraintPoints.pop_front();
		if (c) break;
	}

	initPath(robot, targetV, targetW, targetA, track_tick);
}

void Path::printPathForDebug()
{
	for(auto x : path){
		std::cout << x.p.x << " " << x.p.y << " " << x.t << std::endl;
	}
}

Point Path::getFirstPoint() const
{
	assert(!constraintPoints.empty() || !path.empty());
	if (!path.empty()) {
		return path.front().p;
	} else {
		return constraintPoints.front().p;
	}
}

double Path::getFirstDir() const
{
	if (!path.empty()) {
		return -1;
	}else{
		return constraintPoints.front().dir;
	}
}


void Path::initPath(const RobotCoordinator &robot, double targetV, double targetW, double targetA, double track_tick)
{
	const static double PI = acos(-1);

	//estimate distance
	std::vector<double> disvec;
	double nowTheta = robot.getTheta();
	Point now = robot.getPos();
	for (auto &p : path) {
		double dir = getDirection(now, p.p);
		double dis = getDistance(now, p.p);
		double angle = std::abs(dir - nowTheta);
		if (dis < 1e-2) angle = 0;
		if (2 * PI - angle < angle) angle = 2 * PI - angle;
		double arc_dis = dis;
		if (angle > 0.1) arc_dis = dis * angle / 2 / sin(angle / 2);
		double t = std::max(arc_dis / targetV, angle / targetW);
		disvec.push_back(t * targetV);
		nowTheta = dir;
		now = p.p;
	}
	double endDis, endTime;
	for (int i = 1; i < (int)disvec.size(); i++) {
		disvec[i] += disvec[i-1];
	}
	endDis = disvec.back();
	endTime = path.back().t;
	assert(!isinf(endTime));

	//try to estimate time step by step
	int nowpos = -1;
	double nowv = robot.getv(), nowt = robot.getTime(), nowdis = 0, lastt = robot.getTime();
	while (nowpos < (int)disvec.size() - 1) {
		int nextpos;
		for (nextpos = nowpos + 1; nextpos < (int)disvec.size(); nextpos++) {
			if (!isinf(path[nextpos].t)) break;
		}
		assert(nextpos < (int)disvec.size());
		double nextPosDis = disvec[nextpos];
		double nextPosTime = path[nextpos].t;

		while (nowpos < nextpos) {

			while (nowpos < nextpos && (nowdis > disvec[nowpos + 1] - 1e-3 || nextPosTime - nowt <= 1e-3)) {
				nowpos++;
				path[nowpos].t = nowt - lastt;  //path.t will be relative time requirement
				lastt = nowt;
			}
			if (nowpos >= nextpos) break;

			assert(nextPosTime - nowt > 0);
			double v = (nextPosDis - nowdis) / (nextPosTime - nowt);
			if ((endTime - nowt - v / targetA / 2)*v < endDis - nowdis) {
				double a = 1 / targetA / 2;
				double b = -(endTime - nowt);
				double c = (endDis - nowdis);
				if (b * b - 4 * a * c <= 0) {
					targetA *= 1.1;
					continue;
				}
				v = (-b - sqrt(b*b - 4 * a*c)) / 2 / a;
			}
			//if (v * v / 2 / targetA > endDis - nowdis) v = sqrt(2 * (endDis - nowdis) / targetA) * 1.05;
			v *= 1.01;

			if (v > targetA*(endTime - nowt)) {
				targetA *= 1.1;
				continue;
			}
			
			if (v > nowv + targetA * track_tick) v = nowv + targetA * track_tick;
			if (v < nowv - targetA * track_tick) v = nowv - targetA * track_tick;
			nowv = v;
			nowdis += nowv * track_tick;
			nowt += track_tick;
		}
		
	}
}

std::vector<double> Path::predictDirections(Point p) const
{
	//predict directions by previous and next points
	std::vector<double> dir;
	for (int i = 0; i < (int)constraintPoints.size(); i++) {
		if (!isinf(constraintPoints[i].dir)) dir.push_back(constraintPoints[i].dir);
		else {
			if(i == 0) dir.push_back(getDirection(p, constraintPoints[1].p));
			else if (i + 1 == constraintPoints.size()) dir.push_back(NoDirReq);
			else dir.push_back(getDirection(constraintPoints[i - 1].p, constraintPoints[i + 1].p));
		}
		
	}
	//dir.push_back(NoDirReq);
	return dir;
}

std::vector<int> Path::predictStop(Point p, const std::vector<double> &dir) const
{
	//predict whether to stop by direction (predicted direction and direction between points)

	const static double PI = acos(-1);
	std::vector<int> canStop;
	for (int i = 0; i+1 < (int)constraintPoints.size(); i++) {
		if (constraintPoints[i].canStop >= 0) canStop.push_back(constraintPoints[i].canStop);
		else {
			double angle = getDirection(constraintPoints[i].p, constraintPoints[i + 1].p);
			double delta = std::abs(angle - dir[i]);
			if (delta > PI) delta = 2 * PI-delta;
			if (isinf(constraintPoints[i].dir) && i>0) {
				double d2 = std::abs(dir[i] - dir[i-1]);
				if (d2 > PI) d2 = 2 * PI - d2;
				delta = std::max(delta, d2);
			}
			canStop.push_back(delta >= PI/4);
		}
	}
	canStop.push_back(1);
	return canStop;
}

std::vector<double> Path::predictTime(const RobotCoordinator &robot, const std::vector<double>& dir, std::vector<int> canStop, double targetV, double targetW, double targetA) const
{
	//predict time by time requirement and distance
	//3 kinds of time requirement:
	//  1. normal/ not specified (predict by distance, limited by other explict requirement)
	//  2. absolute requirement(all time)
	//  3. relative requirement(fixed time)

	const static double PI = acos(-1);
	double lastTime = robot.getTime();
	std::vector<double> ans;
	
	for (int i = 0; i < (int)constraintPoints.size(); i++) {
		// predict time until first absolute requirement
		std::vector<double> normalTime;
		double fixedTime = 0, normalTimeSum = 0, allTime;
		bool atEnd = true;
		for (int j = i; j < (int)constraintPoints.size(); j++) {
			if (constraintPoints[j].t <= 0) {
				normalTime.push_back(0);
				fixedTime += -constraintPoints[j].t;
			}else{
				Point lastp = (j == 0 ? robot.getPos() : constraintPoints[j - 1].p);
				double dis = getDistance(lastp, constraintPoints[j].p);
				double angle;
				if (dis < 1e-2) {
					angle = 0;
				}else if (j == 0 || canStop[j-1]) {
					double sdir = getDirection(lastp, constraintPoints[j].p);
					angle = std::abs(dir[j] - sdir);
					if (angle > PI) angle = 2 * PI - angle;
				} else if (canStop[j] && !isinf(constraintPoints[j].dir)) {
					angle = 0;
				} else {
					double sdir = getDirection(lastp, constraintPoints[j].p);
					double a1 = std::abs(sdir - dir[j - 1]);
					if (2 * PI - a1 < a1) a1 = 2 * PI - a1;
					double a2 = std::abs(dir[j] - sdir);
					if (2 * PI - a2 < a2) a2 = 2 * PI - a2;
					angle = a1 + a2;
				}
				double arc_dis = dis;
				if (angle > 0.1) arc_dis = dis * angle / 2 / sin(angle / 2);
				double estimate_dis = std::max(arc_dis, angle / targetW * targetV);
				//predict distance from straight distance and direction

				double startv = targetV, endv = targetV;
				if (j == 0) startv = std::min(targetV, robot.getv());
				else if (canStop[j - 1]) startv = 0;
				if(canStop[j]) endv = 0;
				
				double t;
				double accelerate_dis = (startv + targetV) * (-startv + targetV) / targetA / 2
									+ (endv + targetV) * (-endv + targetV) / targetA / 2;
				if (accelerate_dis > estimate_dis) {
					double maxv = sqrt(targetA * estimate_dis + (startv * startv + endv * endv)/2);
					t = (maxv - startv + maxv - endv) / targetA;
				} else {
					t = (targetV * 2 - startv - endv) / targetA + (estimate_dis - accelerate_dis) / targetV;
				}
				//predict time from distance and veclocity

				normalTime.push_back(t);
				normalTimeSum += t;
				if (!isinf(constraintPoints[j].t)) {
					allTime = constraintPoints[j].t - lastTime;
					atEnd = false;
					break;
				}
			}
		}

		double c = (atEnd ? 1 : ((allTime - fixedTime) / normalTimeSum));
		if (c < 0) {
			std::cerr << "Path Tracing Failed! Time Constraints Conflict. " << std::endl;
			throw std::runtime_error("Path Tracing Failed! Time Constraints Conflict. ");
		}
		for (int j = i; j < (int)constraintPoints.size(); j++) {
			if (isinf(constraintPoints[j].t)) {
				lastTime += normalTime[j - i] * c;
				ans.push_back(lastTime);
			} else if (constraintPoints[j].t < 0) {
				lastTime += -constraintPoints[j].t;
				ans.push_back(lastTime);
			} else {
				lastTime = constraintPoints[j].t;
				ans.push_back(lastTime);
				i = j;
				break;
			}
		}
		if (atEnd) break;
	}
	return ans;
}

void Path::interpolateUntilStop(const RobotCoordinator & robot, double targetV, double path_tick)
{
	//interpolate and generate path point
	int k = 0;
	for (int i = -1; i < (int)constraintPoints.size(); ) {
		// interpolate between two direction-specified points
		SplineInterpolate si;
		double startTime;
		if (i == -1) {
			startTime = robot.getTime();
			double vx = targetV * sin(robot.getTheta());
			double vy = targetV * cos(robot.getTheta());
			si.start(robot.getTime(), robot.getPos().x, robot.getPos().y, vx, vy);
		} else {
			assert(!isinf(constraintPoints[i].dir));
			startTime = constraintPoints[i].t;
			double vx = targetV * sin(constraintPoints[i].dir);
			double vy = targetV * cos(constraintPoints[i].dir);
			si.start(constraintPoints[i].t, constraintPoints[i].p.x, constraintPoints[i].p.y, vx, vy);
		}

		int endpos;
		for (int j = i+1; j < (int)constraintPoints.size(); j++) {
			 if (!isinf(constraintPoints[j].dir)) {
				double vx = targetV * sin(constraintPoints[j].dir);
				double vy = targetV * cos(constraintPoints[j].dir);
				si.end(constraintPoints[j].t, constraintPoints[j].p.x, constraintPoints[j].p.y, vx, vy);
				endpos = j;
				break;
			}else if (constraintPoints[j].canStop == 1) { //stop and not explict direction
				si.end(constraintPoints[j].t, constraintPoints[j].p.x, constraintPoints[j].p.y, 0, 0);
				endpos = j;
				break;
			} else{
				si.append(constraintPoints[j].t, constraintPoints[j].p.x, constraintPoints[j].p.y);
			}
		}

		std::vector<double> tvec;
		for (double t = startTime + path_tick; t < constraintPoints[endpos].t; t += path_tick) {
			tvec.push_back(t);
		}
		auto xyvec = si.get(tvec);
		
		for (int j = 0; j < (int)xyvec.size(); j++) {
			while (k <= endpos && constraintPoints[k].t < tvec[j]) {
				path.emplace_back(constraintPoints[k].p, constraintPoints[k].t);
				k++;
			}
			path.emplace_back(Point{ std::get<0>(xyvec[j]), std::get<1>(xyvec[j]) }, NoTimeReq);
		}

		i = endpos;
		if (constraintPoints[i].canStop == 1) {
			for (;k <= i;k++)
				path.emplace_back(constraintPoints[k].p, constraintPoints[k].t);
			break;
		}
	}
	
}


std::tuple<Point, double, double> Path::lookahead(RobotCoordinator &robot, double lookahead_distance, double lookahead_time)
{
	//lookahead for at least lookahead_distance and lookahead_time
	//record is points before pursuit-point
	//path is points after pursuit-point

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
			record.emplace_back(nextp, ti + robot.getTime());
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
		if (per < 0) per = 0;
		Point P_res = Point{ PA.x * (1 - per) + PB.x * per, PA.y * (1 - per) + PB.y * per };
		double t_res = tA * (1 - per) + tB * per;
		double dis_res = disA * (1 - per) + disB * per;
		return std::make_tuple(P_res, t_res, dis_res);
	}
	return std::tuple<Point, double, double>();
}

bool Path::end()
{
	return path.empty() && record.empty() && constraintPoints.empty();
}

bool Path::curveEnd()
{
	return path.empty() && record.empty();
}

void Path::applyMovement(RobotCoordinator & robot, double tick)
{
	//robot apply movement. Maintian record

	Point startPos = robot.getPos();
	double movedis = robot.applyMovement(tick);
	Point endPos = robot.getPos();

	while (!record.empty()) {
		double sdis = getDistance(startPos, record.front().p);
		double edis = getDistance(endPos, record.front().p);
		if (sdis <= edis || edis < 1e-2) {
			errdis = std::max(errdis, getDistance(record.front().p, startPos));
			errtime = std::max(errtime, std::abs(robot.getTime() - record.front().t));
			//std::cerr << getDistance(record.front().p, startPos) << " ";
			//std::cerr << robot.getTime() - record.front().t << std::endl;
			record.pop_front();
		}
		else break;
	}
}



double Path::getErrdis() const
{
	return errdis;
}

double Path::getErrtime() const
{
	return errtime;
}

void Path::printConstrainPointsForDebug() const
{
	for (auto &p : constraintPoints) {
		std::cout << p.p.x << " " << p.p.y << " " << p.dir << " " << p.t << " " << p.canStop << std::endl;
	}
}

