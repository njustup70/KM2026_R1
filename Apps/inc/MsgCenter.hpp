#ifndef MSGCENTER_HPP
#define MSGCENTER_HPP

#include "RobotSystem.hpp"


/*****      系统状态变量        *****/
class CenterStatusParam
{
    public:
    CenterStatusParam(){};
    bool RoboReady;            // 机器人准备就绪
};


/*****      系统控制变量        *****/
class CenterControlParam
{
    public:
    CenterControlParam(){};
    bool StopRobot;
};



/*****      用户自定义变量        *****/




/***    全局实例    ***/
extern CenterControlParam ControlParam;
extern CenterStatusParam StatusParam;
#endif