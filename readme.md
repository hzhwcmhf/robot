## 路径追踪算法说明

路径追踪分两部分完成：  
第一部分使用估计策略和样条插值，从限制点生成路径点。  
第二部分使用pure-pursuit算法，使小车沿路径点运动。  
第二部分参照：Implementation of the Pure Pursuit Path Tracking Algorithm. R. Craig Coulter, CMU, 1992. 

#### 算法说明：

给定一系列限制点，每个点上可以有时间限制（相对上一个限制点的时间限制；或者绝对时间限制，限制到达时间；或者没有限制），可以有方向限制（到达该点时方向；或者没有限制），可以有停止转向限制（一定停止在该点，接下来会转向再运动；一定不停止在该点；没有限制）。  
算法会在满足条件的情况下进行规划，生成路径曲线，并给出运动操作。限制点需要满足的条件是，非原地转向曲率不能过陡，时间约束不能矛盾，不能超过机器人速度极限。其他可能出现的非法情况需要按具体情况分析处理，可以通过输出观察规划中断在什么位置，再调整时间、方向、转向限制或增加更多限制点解决问题。

#### 算法简述：

	算法入口。基本分为两部分，规划和运动
	function trackPath():
		While 限制点没有全部用完：
			turnToPath(); 
				转向第一个限制点（或第一个限制点方向）
			generatePath(); 
				规划，消耗一部分限制点生成路径点（路径点只限制位置和时间）
			pure-pursuit(); 
				运动，通过pure-pursuit算法对路径点进行跟踪

	关键步骤，路径点生成，规划过程
	function generatePath():
		1. predictDirections(); 
			为没有方向的限制点估计方向（通过策略生成，策略可以更换）
		2. predictStop(); 
			为没有原地转向限制的限制点估计是否原地转向（通过估计的方向，
			同样通过策略生成）
		3. predictTime(); 
			为没有时间限制的限制点估计时间（参考估计方向和估计原地转向），
			统一所有绝对时间和相对时间限制。（策略生成）
		4. interpolateUntilStop(); 
			丢弃第1步生成的估计方向，对限制点进行三阶样条插值生成路径曲线并
			采样。插值使用参数方程，参数自变量为时间。这一步只计算到第一个
			停止点。
		5. initPath(); 
			为生成的路径样本点估计相对时间限制。（策略生成）
		6. 丢弃所有已经生成了路径的限制点。
	
	关键步骤，运动过程
	function pure-pursuit():
		While 路径点未全部满足：
			1.	从当前点，向前找一个追逐点。追逐点是满足距离当前路径距离大于
				lookahead_distance,时间约束大于lookahead_time的最近点。
				其中lookahead_distance与速度有关，速度越大，距离越大。
			2.	假设机器人之后保持角速度与线速度不变，通过追逐点算出当前应有的
				圆弧半径。通过时间约束算出线速度，再算出角速度。
			3.	尝试应用目标速度，但需要满足加速度限制。具体方法是：首先尝试将
				目标角速度赋予机器人，但要满足角加速度约束。再按照圆弧半径算出
				目标线速度。尝试将目标线速度赋予机器人，但要满足线加速度约束。
			4.	在一小段时间内，应用当前角速度和线速度，计算下一时刻机器人位置。
	
	不需规划的运动过程
	function turnToPath():
		如果第一个限制点离当前点较远，则转向该限制点。
		如果第一个限制点离当前点很近，并且有方向限制，则转向该点所限制方向。
		算法和pure-pursuit类似，但是只在原地旋转
	
	规划过程中，依靠策略进行预测，可以替换成其他策略
	function predictDirections():
		未限制方向的控制点，估计方向为该点前一点到后一点的方向。
		dir[i] = getDirection(p[i+1] - p[i-1])

	规划过程中，依靠策略进行预测，可以替换成其他策略
	function predictStop():
		对未确定转向的控制点估计是否转向。计算上一个点限制方向和这一点限制方向差，
		这一点限制方向和到下一点的方向差。若两方向差和大于一个阈值则认为是需要原
		地转向。
		条件：abs(dir[i] - dir[i-1]) + abs(getDirection(p[i+1] - p[i]) - dir[i]) > PI / 4

	规划过程中，依靠策略进行预测，可以替换成其他策略
	function predictTime():
		限制点存在三种时间限制。无限制，相对时间限制，绝对时间限制。
		将所有时间限制统一到相对时间限制。
		将任意两个绝对时间限制相减，可以得到一段路径的时间限制。
		减去这段路径间的相对时间限制，剩下的时间按照估计路程进行分配。
		举例：a(绝对0), b, c(相对5), d(绝对20),f(绝对30)
		结果：a[0], b[dis(a,b)*x],c[5],d[dis(c,d)*x], f(10)
			方括号内为相对时间。满足dis(a,b)*x+dis(c,d)*x=20-5
		距离的估计会参考点的位置，之前估计的方向，估计的停止转向限制。	
		
	规划过程中，依靠策略进行预测，可以替换成其他策略
	function interpolateUntilStop():
		将到第一个停止转向点位置的所有限制点构成的路径称为第一条路径。
		则该路径上只有起点和终点有方向限制，中间点只有位置限制。
		现将x,y坐标看为时间t的函数。已知起终点方向，即已知起终点导数。
		采用三次样条插值得到路径曲线，并按t等间隔采样。注意，由于样条
		得到的速度可能不够平滑，所以舍弃所有采样点的时间限制，只保留曲线本身。

	规划过程中，依靠策略进行预测，可以替换成其他策略
	function initPath():
		从路径点和部分路径点的时间限制，估计所有路径点时间限制。
		通过路径点的位置，估计路径点之间的距离。然后按照以下方式
		生成时间限制：
		维护小车当前速度（忽视转向）
		While 路径点没有完全通过:
			根据下一个时间限制，整条曲线的时间限制，剩余路程估计当前速度。
			尝试将调整速度，但要满足加速度限制。
			运行一小段时间，若超过某路径点，则记录该路径点所需时间。

#### 框架简述：

1. PathTracker为追踪算法，RobotCoordinator保存机器人状态，Path储存路径限制，生成路径规划，defines储存路径规划的参数。
2. 使用前需分别先初始化PathTracker，RobotCoordinator，Path。PathTracker初始化会从defines中读取默认参数；RobotCoordinator初始化需要给出初始位置，方向，速度，角速度，时间；Path初始化以后，用append加入限制点及限制条件。
3. 调用PathTracker::trackPath进行路径追踪。需要RobotCoordinator和Path作为参数。
4. trackPath的过程参考算法简述。turnToPath和pure-pursuit由PathTracker处理，generatePath由Path处理。
5. turnToPath和pure-pursuit会控制RobotCordinator，改变其状态；pure-pursuit会查询Path给出的追踪点，并在运动时维护Path的状态。
6. generatePath会分别调用predictDirections，predictStop，predictTime，interpolateUntilStop，initPath策略函数对路径进行规划。
7. 从PathTracker::trackPath得到的车体角速度和线速度，可以通过PathTracker::convertRobotToWheel将机器人运动转化为轮胎的转速。

类关系

![structure](https://github.com/hzhwcmhf/robot/raw/master/img/structure.png)

流程

![flow](https://github.com/hzhwcmhf/robot/raw/master/img/flow.png)



#### 使用示例（更多示例参考main.cpp）：

	Path path;

	path.append(Point{ 0, 1 }, Path::NoTimeReq);	//限制点(0,1) 无时间限制 无方向限制 无停止转向限制
	path.append(Point{ 0, 2 }, -4);					//限制点(0,2) 相对时限4 无方向限制 无停止转向限制
	path.append(Point{ 0, 3 }, Path::NoTimeReq);	//限制点(0,3) 无时间限制 无方向限制 无停止转向限制
	path.append(Point{ 0, 4 }, 20);					//限制点(0,4) 绝对时限20 无方向限制 无停止转向限制
	path.append(Point{ 1, 4 }, Path::NoTimeReq);	//限制点(1,4) 无时间限制 无方向限制 无停止转向限制
	path.append(Point{ 2, 4 }, -4);					//限制点(2,4) 相对时限4 无方向限制 无停止转向限制
	path.append(Point{ 3, 4 }, 50);					//限制点(3,4) 绝对时限50 无方向限制 无停止转向限制
	path.append(Point{ 4, 4 }, 60, Path::NoDirReq, 1);//限制点(4,4) 绝对时限60 无方向限制 必须停止转向

	path.append(Point{ 0, 0 }, Path::NoTimeReq, PI/2, 0);	//限制点(0,0) 无时间限制 方向向下 必须不停止转向
	path.append(Point{ 0, -3 }, Path::NoTimeReq, PI/2, 0);	//限制点(0,-3) 无时间限制 方向向下 必须不停止转向
	path.append(Point{ 3, -3 }, Path::NoTimeReq, 0, 0);		//限制点(3,-3) 无时间限制 方向向上 必须不停止转向
	path.append(Point{ 3, 0 }, Path::NoTimeReq, -PI/2, 0);	//限制点(3,0) 无时间限制 方向向左 必须不停止转向
	path.append(Point{ 0, 0 }, Path::NoTimeReq, 0, 0);		//限制点(0,0) 无时间限制 方向向上 必须不停止转向

	RobotCoordinator robot;
	MotorController mc;

	//获得车体控制操作
	auto control = mc.trackPath(robot, path);

	for (auto c : control) {
		auto ans = mc.convertRobotToWheel(c);	//转换为轮胎控制
		cout << ans.lw << " " << ans.rw << endl;
	}


#### 模拟器使用
工程根目录下有simulate.py用于模拟机器人运行。

输入格式如下（或参考main.cpp或者simulate.py的代码）
	B, ch, wheel_r
	maxv, maxw
	track_tick
	robot_x, robot_y, robot_dir, robot_time
	
	x, y, dir, t, canStop(限制点)
	...

	x, y, t(路径点)
	...

	wl, wr(操作)
	...

模拟器输出

路径  
×的点为限制点，若点上有箭头，为限制方向。  
红色的路径为规划路径，蓝色的路径为实际路径。较大的箭头为停车时方向。

![simulatorPath](https://github.com/hzhwcmhf/robot/raw/master/img/path.png)

速度  
横轴为时间，纵轴为和最大速度的比值

![simulatorV](https://github.com/hzhwcmhf/robot/raw/master/img/v.png)

角速度  
横轴为时间，纵轴为和最大角速度的比值，顺时针为正

![simulatorW](https://github.com/hzhwcmhf/robot/raw/master/img/w.png)

## defines

预设值（在defines.cpp中修改）

* MotorController_B 两轮间距  
* MotorController_ch ICR系数（pioneer3-AT 1.5; 理想1）  
    参照：Wang, Tianmiao, et al. "Analysis and experimental kinematics of a skid-steering wheeled robot based on a laser scanner sensor." Sensors 15.5 (2015): 9681-9702.  
* wheel_radius 车轮半径(m)  
* default\_lookahead\_distance\_ratio, min\_lookahead\_distance  
	pure pursuit 算法会要求追踪路径前方的一个点。这两参数决定追踪点的距离。  
	lookahead_distance =  min(default_lookahead_distance_ratio * speed, min_lookahead_distance)
* default\_lookahead\_time  
	pure pursuit 算法会要求追踪路径前方的一个点。这个参数决定追踪点提前多少时间。  
* track\_tick 即路径追踪时的每一步操作时间，越小参数控制改变越频繁，trackPath得到的控制信号越多
* max\_angular\_acceleration, max\_angular\_veclocity, angular\_decay  
	限制机器人角速度，满足以下条件：  
	last_w * angular_decay - max_angular_acceleration <= w <= last_w * angular_decay + max_angular_acceleration  
	and:  -max_angular_veclocity <= w <= max_angular_veclocity
* max\_linear\_acceleration, max\_linear\_veclocity, linear\_decay  
	限制机器人线速度，满足以下条件：
	last_v * linear_decay - max_linear_acceleration <= v <= last_v * linear_decay + max_linear_acceleration  
	and:  0 <= v <= max_linear_veclocity
* target_v, target_w, target_a  
	路径规划时(即generatePath)所设定的速度、角速度、线加速度，算法尽量在这个值附近进行规划。若太大，接近小车最大限制，可能导致追踪部分失败。
* path_tick
	路径规划时曲线插值采样的间隔。

## PathTracker

运动学控制模块

接口：

* WheelAngularVeclocity convertRobotToWheel(double v, double w)  
    参数：v 速度(m/s)  w 角速度(rad/s,顺时针)   
    返回值：结构体{左轮角速度(rad/s),右轮角速度(rad/s)}
	说明：参考了滑移转向小车模型来计算轮子角速度。
	参照：Wang, Tianmiao, et al. "Analysis and experimental kinematics of a skid-steering wheeled robot based on a laser scanner sensor." Sensors 15.5 (2015): 9681-9702.  

* std::vector<BodyVeclocity> trackPath(RobotCoordinator &robot, Path &path)  
	算法主体，路径追踪。算法内容参考算法简述。  
	参数：机器人,路径  
	返回值：机器人移动操作序列

算法内部调用：

* BodyVeclocity trackPathForOneStep(RobotCoordinator &robot, Path &path)  
	参数：机器人，路径  
	返回值：机器人一步操作  
	说明：pure-pursuit一次操作，可以参见算法简述pure-pursuit()。

* vector<BodyVeclocity> turnToPath(RobotCoordinator &robot, Path &path)
	参数：机器人，路径  
	返回值：机器人移动操作序列  
	说明：转向路径方向，可以参见算法简述turnToPath()。

## Path

接口：

* void append(const Point &p, double timeReq = NoTimeReq, double dir = NoDirReq, int canStop = -1)  
	参数：下一个目标点，时间限制，方向限制，停止转向限制。  
	* 时间限制：若timeReq<0，相对时间限制为-timeReq（上一个点到这个点的时间为-timeReq）；
	若timeReq>0，绝对时间限制为timeReq（在timeReq时刻到达这个点）；
	若timeReq=NoTimeReq，没有时间限制（依靠策略进行估计）  
	* 方向限制：若dir在-2PI到2PI之间，则限制到达该点时方向为dir；
	若dir=NoDirReq，则没有方向限制（依靠策略进行估计）  
	* 停止转向限制：若canStop=-1，没有停止转向限制（依靠策略进行估计）；
	若canStop=0，禁止停止转向；若canStop=1，一定会停止后转向下一个路径方向。
			
* void appendRaw(const Point &p, double t)  
	参数：下一个目标点，相对时间限制。  
	说明：直接加入路径点（而不是加入限制点）。路径点将先于限制点生效。

* double getErrdis() const
* double getErrtime() const  
	获得运动和路径规划的距离，时间误差。

算法内部调用:

* void printConstrainPointsForDebug()
	打印路径点及路径点时间限制

* tuple<Point, double, double> lookahead(RobotCoordinator &robot, double lookahead_distance, double lookahead_time)   
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

* bool end()
	检查限制点和路径点是否均使用完毕

* bool curveEnd()
	检查路径点是否使用完毕

* void generatePath(const RobotCoordinator &robot, double targetV, double targetW, double targetA, double track_tick, double path_tick)
	输入：机器人，目标速度，目标加速度，目标加速度，机器人运动离散化后每一步运行时间，插值后采样间隔  
	说明：该函数将消耗一部分限制点（准确的说，将消耗到第一次需要停止转向的限制点），生成符合要求的路径曲线，采样并估计相对时间间隔。参照算法简述。

* std::vector<double> predictDirections(Point p) const;
	输入：机器人当前位置  
	输出：限制点每点估计方向（若被限制，则直接为限制方向）  
	说明：参照算法简述

* std::vector<int> predictStop(Point p, const std::vector<double> &dir) const
	输入：机器人当前位置，限制点每点估计方向  
	输出：是否停止转向估计（若被限制，则直接为限制）  
	说明：参照算法简述

* std::vector<double> predictTime(const RobotCoordinator &robot, const std::vector<double> &dir, std::vector<int> canStop, double targetV, double targetW, double targetA) const
	输入：机器人，限制点每点估计方向，停止转向估计，目标速度，目标角速度，目标加速度  
	输出：限制点绝对时间估计（满足时间限制）  
	说明：参照算法简述

* void interpolateUntilStop(const RobotCoordinator &robot, double targetV, double path_tick)
	输入：机器人，目标速度，插值采样间隔  
	说明：将会把插值采样点推入path，并在限制点给出绝对时间限制。参照算法简述

* void initPath(const RobotCoordinator &robot, double targetV, double targetW, double targetA, double track_tick)
	输入：机器人，目标速度，目标角速度，目标加速度，运动离散化一步时间  
	说明：将path中没有时间限制和绝对时间限制转换为相对时间限制。参照算法简述

## RobotCoordinator

接口：

* RobotCoordinator(double _x = 0, double _y = 0, double _theta = 0, double _v = 0, double _w = 0, double _t = 0)  
	初始化机器人位置，朝向，速度， 角速度，时间。注意机器人初始朝y轴正方向。角速度顺时针为正。
* double getv() const
* double getw() const
* double getTime() const
* Point getPos() const
* double getTheta() const  
* bool stopped() const  
	获取机器人当前信息
* Point globalToRobot(const Point &p) const  
	从世界坐标系转换到机器人坐标系

算法内部调用：

* double applyAngularVeclocity(double target_w, double maxa, double maxw, double decay, double t)  
	试图改变机器人线速度，但要满足加速度约束：  
	last_v * linear_decay - max_linear_acceleration <= v <= last_v * linear_decay + max_linear_acceleration  
	and:  0 <= v <= max_linear_veclocity
* double applyLinearVeclocity(double target_v, double maxa, double maxv, double decay, double t)  
	试图改变机器人角速度，但要满足加速度约束：  
	last_w * angular_decay - max_angular_acceleration <= w <= last_w * angular_decay + max_angular_acceleration  
	and:  -max_angular_veclocity <= w <= max_angular_veclocity

* double applyMovement(double tick)  
	输入：运动时间
	计算下一步机器人的位置，返回运动的距离。

## SplineInterpolate

给定起终点导数的三次样条插值

* void start(double t, double x, double y, double vx, double vy)
* void end(double t, double x, double y, double vx, double vy)
* void append(double t, double x, double y)
	输入：绝对时间（参数方程自变量），x(t)，y(t)，(x'(t)，y'(y))  
	说明：需要先调用start，再调用多次append，最后调用end

* vector<tuple<double, double>> get(const vector<double> &tt) const;
	输入：需要插值的位置（组）  
	输出：x(t)，y(t)插值结果  
	说明：需要在end以后调用

## main.cpp

测试和模拟使用，可参考