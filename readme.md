## 路径追踪算法说明

路径追踪使用pure-pursuit算法  
参照：Implementation of the Pure Pursuit Path Tracking Algorithm. R. Craig Coulter, CMU, 1992. 

####算法说明：

路径规划需要一系列路径坐标点。算法会大致沿着坐标点之间的折线进行运动。坐标点需要满足的条件是，曲率不能过陡（算法无法处理下一个坐标点与当前方向大于90度的情况）。时间约束不能超过机器人速度极限。

####算法简述：

1. 机器人在每一小段时间内，重复步骤2-4
2. 从当前点，向前找一个追逐点。追逐点是满足距离当前路径距离大于lookahead_distance,时间约束大于lookahead_time的最近点。其中lookahead_distance与速度有关，速度越大，距离越大。
2. 假设机器人之后保持角速度与线速度不变，通过追逐点算出当前应有的圆弧半径。公式见上述论文。通过时间约束算出线速度，再算出角速度。
3. 尝试将目标角速度赋予机器人，但要满足角加速度约束。再按照圆弧半径算出目标线速度。尝试将目标角速度赋予机器人，但要满足线加速度约束。
4. 在当前一段时间内，应用当前角速度和线速度，计算下一时刻机器人位置。

####框架简述：

1. 路径规划所需环境参数在defines.cpp内
2. 路径规划需要Path,RobotCoordinator作为参数。Path记录路径点，RobotCoordinator记录初始机器人位置、方向、速度等。
3. 调用MotorController::trackPath进行路径追踪。
4. MotorController::trackPath首先调用Path::initPath对路径中没有设置时间约束的部分预估时间约束。再循环调用MotorController::trackPathForOneStep对每一小段时间(track_tick）计算车体角速度和线速度。
5. MotorController::trackPathForOneStep首先调用Path::lookahead计算追逐点坐标、距离、时间约束（之后有具体说明）。再根据追逐点算出圆弧半径，符合加速度约束的线速度与角速度。（计算方法如算法简述2-4。）使用Path::applyMovement计算下一时刻机器人位置和速度,更新路径点信息供下一次Path::lookahead使用。
6. 通过MotorController::trackPath得到的车体角速度和线速度，可以通过MotorController::convertRobotToWheel将机器人运动转化为轮胎的转速。

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
* track\_tick 即路径追踪时的每一步操作时间，越小参数控制改变越频繁，trackPath得到的控制信号越多
* max\_angular\_acceleration, max\_angular\_veclocity, angular\_decay  
	限制机器人角速度，满足以下条件：  
	last_w * angular_decay - max_angular_acceleration <= w <= last_w * angular_decay + max_angular_acceleration  
	and:  -max_angular_veclocity <= w <= max_angular_veclocity
* max\_linear\_acceleration, max\_linear\_veclocity, linear\_decay  
	限制机器人线速度，满足以下条件：
	last_v * linear_decay - max_linear_acceleration <= v <= last_v * linear_decay + max_linear_acceleration  
	and:  0 <= v <= max_linear_veclocity

## MotorController

运动学控制模块

接口：

* WheelAngularVeclocity convertRobotToWheel(double v, double w)  
    参数：v 速度(m/s)  w 角速度(rad/s,顺时针)   
    返回值：结构体{左轮角速度(rad/s),右轮角速度(rad/s)}

* std::vector<BodyVeclocity> trackPath(RobotCoordinator &robot, Path &path)  
	参数：机器人,路径  
	返回值：机器人移动操作序列

算法内部调用：

* BodyVeclocity trackPathForOneStep(RobotCoordinator &robot, Path &path)  
	参数：机器人，路径
	返回值：机器人一步操作
	说明：算法的主体部分，可以参见算法简述2-4。其中对路径结尾进行了特殊处理，使其尽量能够在最后停下来。

## Path

接口：

* void append(const Point &p, double t)  
	参数： 下一个目标点，从上一个目标点到这个目标点所需时间。
* void append(const Point &p)  
	参数： 下一个目标点，没有时间限制。

* double getErrdis() const
* double getErrtime() const  
	获得路径规划后的距离，时间误差。

算法内部调用:

* void Path::initPath(const RobotCoordinator &robot, double v, double w)  
	按照目标线速度和角速度，估算时间约束

* std::tuple<Point, double, double> Path::lookahead(RobotCoordinator &robot, double lookahead_distance, double lookahead_time)   
	求追踪点的算法。  
	返回值：追逐点位置，追逐点时间约束，追逐点路径距离  
	说明：除非到了路径末端，一定会满足时间约束大于lookahead_time，路径距离大于lookahead_distance。  
	
	计算方式：  
	* 路径点分为两部分，一部分是record，一部分是path。record为追逐点之前的路径点。
	* 每次进入lookahead，从机器人当前位置，沿着record连接一条折线。若path的第一个路径点加入record后，折线长仍然小于lookahead_distance或者时间约束小于lookahead_time，则从path中出队，加入record队尾。重复这个操作，直到path第一个路径点不能加入record，即此时追逐点在record队尾和path队首的折线上。
	* 对在record队尾和path队首两个点进行线性插值，求得追逐点，满足时间约束大于lookahead_time，路径距离大于lookahead_distance。  


* void Path::applyMovement(RobotCoordinator & robot, double tick)  
	对robot执行操作，并且维护record路径点。  

	说明：若record队首满足下列条件，则视为该点已经到达，弹出record队首。
	* robot执行运动后，远离了该点。（这是为了避免路径点所需角速度大于其能力范围时，绕某路径点不断旋转的问题。）
	* robot距离该点距离小于0.01 

## RobotCoordinator

接口：

* RobotCoordinator(double _x = 0, double _y = 0, double _theta = 0, double _v = 0, double _w = 0, double _t = 0)  
	初始化机器人位置，朝向，速度， 角速度，时间。注意机器人初始朝y轴正方向。角速度顺时针为正。
* double getv() const
* double getTime() const
* Point getPos() const
* double getTheta() const  
	获取机器人当前信息
* Point globalToRobot(const Point &p) const  
	从世界坐标系转换到机器人坐标系

算法内部调用：

* double applyAngularVeclocity(double target_w, double maxa, double maxw, double decay, double t)  
* double applyLinearVeclocity(double target_v, double maxa, double maxv, double decay, double t)  
	试图改变机器人角速度（线速度），但要满足加速度约束：  
	last_w * angular_decay - max_angular_acceleration <= w <= last_w * angular_decay + max_angular_acceleration  
	and:  -max_angular_veclocity <= w <= max_angular_veclocity

* double applyMovement(double tick)  
	计算下一步机器人的位置，返回运动的距离。

## main.cpp

测试使用，可参考