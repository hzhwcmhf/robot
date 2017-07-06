## MotorController

运动学控制模块

* WheelAngularVeclocity convertRobotToWheel(double v, double w)  
    参数：v 速度(m/s)  w 角速度(rad/s,顺时针)   
    返回值：结构体{左轮角速度(rad/s),右轮角速度(rad/s)}

## defines

预设值（在defines.cpp中修改）

* MotorController_B 两轮间距  
* MotorController_ch ICR系数（pioneer3-AT 1.5; 理想1）  
    参照：Wang, Tianmiao, et al. "Analysis and experimental kinematics of a skid-steering wheeled robot based on a laser scanner sensor." Sensors 15.5 (2015): 9681-9702.  
* wheel_radius 车轮半径(m)

## main.cpp

测试使用，可参考