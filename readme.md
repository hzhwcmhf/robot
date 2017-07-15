## defines

预设值（在defines.cpp中修改）

* MotorController_B 两轮间距  
* MotorController_ch ICR系数（pioneer3-AT 1.5; 理想1）  
    参照：Wang, Tianmiao, et al. "Analysis and experimental kinematics of a skid-steering wheeled robot based on a laser scanner sensor." Sensors 15.5 (2015): 9681-9702.  
* wheel_radius 车轮半径(m)

* default\_lookahead\_distance\_ratio, min\_lookahead\_distance  
	pure pursuit 算法会要求跟随路径前方的一个点。这两参数决定跟随点的距离。  
	lookahead_distance =  min(default_lookahead_distance_ratio * speed, min_lookahead_distance)
* default\_lookahead\_time  
	lookahead_time indicates the minimum time between lookahead point and robot 
* track\_tick 即路径追踪时的每一步操作时间，越小参数控制改变越频繁，pathtrack得到的控制信号越多
* max\_angular\_acceleration, max\_angular\_veclocity, angular\_decay  
	limit the robot angular speed  
	last_w * angular_decay - max_angular_acceleration <= w <= last_w * angular_decay + max_angular_acceleration  
	and:  -max_angular_veclocity <= w <= max_angular_veclocity
* max\_linear\_acceleration, max\_linear\_veclocity, linear\_decay  
	limit the robot linear speed  
	last_v * linear_decay - max_linear_acceleration <= v <= last_v * linear_decay + max_linear_acceleration  
	and:  0 <= v <= max_linear_veclocity

## MotorController

运动学控制模块

* WheelAngularVeclocity convertRobotToWheel(double v, double w)  
    参数：v 速度(m/s)  w 角速度(rad/s,顺时针)   
    返回值：结构体{左轮角速度(rad/s),右轮角速度(rad/s)}

* std::vector<BodyVeclocity> trackPath(RobotCoordinator &robot, Path &path)  
	参数：机器人,路径  
	返回值：机器人移动操作序列

## Path

* void append(const Point &p, double t)  
	参数： 下一个目标点，从上一个目标点到这个目标点所需时间。
* void append(const Point &p)  
	参数： 下一个目标点，没有时间限制。

* double getErrdis() const
* double getErrtime() const  
	获得路径规划后的距离，时间误差。

## Robot

* RobotCoordinator(double _x = 0, double _y = 0, double _theta = 0, double _v = 0, double _w = 0, double _t = 0)  
	初始化机器人位置，朝向，速度， 角速度，时间。注意机器人初始朝y轴正方向。角速度顺时针为正。
* double getv() const
* double getTime() const
* Point getPos() const
* double getTheta() const  
	获取机器人当前信息
* Point globalToRobot(const Point &p) const  
	从世界坐标系转换到机器人坐标系

## main.cpp

测试使用，可参考