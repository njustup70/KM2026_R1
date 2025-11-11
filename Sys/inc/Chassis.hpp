#ifndef _CHASSIS_HPP_
#define _CHASSIS_HPP_

#include "stm32f4xx_hal.h"
#include "Action.hpp"
#include "std_math.hpp"
#include "motor_dji.hpp"

#define ROTATE_RADIUS 0.29185f
#define WHEEL_DIAMETER 0.154f

class MoveAct;
class ChassisClass;

/**
 * @brief 路径类，包含了一系列路径点
 */
class Path
{
    Vec2 key_points[32];            // 路径点数组
};


class MoveAct : public BaseAction
{
    private:
        // 基于内部选择的移动方式
        bool MoveAt();
        bool MoveAlong();

    public:
        Vec2 target_pos = Vec2(0, 0);   // 目标位置，单位m，场地坐标系
        float max_velo = 1.0f;          // 最大速度，单位m/s
        float max_accel = 2.0f;         // 最大加速度，单位m/s^2

        typedef enum
        {
            AtPos,          // 移动到指定位置
            AlongPath,      // 沿指定路径移动
        }MoveType;
        
        // 设置移动动作类型
        MoveType movetype = AtPos;
        // 构造函数
        MoveAct(){};
        
        // 覆写虚函数
        virtual bool OnUpdate() override;
        // 重置参数
        void Reset(Vec2 new_target);
};


/**
 * @brief 底盘类
 * @note 底盘类中大部分是实现类的接口，更多复杂的算法都不在这里
 * 这里只提供一些简单的运动控制接口
 */
class ChassisClass
{
    private:
        // 配置参数
        bool EnAccelLim;            // 是否启用加速度限制
        float Accel = 0.0f;         // 最大加速度，单位m/s^2
        bool EnVeloLim;             // 是否启用速度限制
        float VeloLim = 0.0f;       // 最大线速度，单位m/s
        float motor_spd[4];         // 四个电机的目标速度，单位m/s

        // 动作类
        MoveAct move_action;

    public:
        ChassisClass(): move_action() {};
        ~ChassisClass(){};

        typedef enum
        {
            Omni,       // 全向轮
            Meca,       // 麦克纳姆轮
            Steer,      // 舵轮
        }ChassisType;
        
        // 底盘类型，默认为全向轮
        ChassisType type = Omni;

        // 分别对应四个底盘电机
        MotorDji motors[4];

        // 属性参数
        Vec3 speed;     // 期望速度，车体右手系，x向前，y向左，z左转，单位m/s，rad/s
        float velo;     // 对应的线速度，单位m/s
        bool enabled = false;    // 底盘使能标志

        float move_precision = 0.05f; // 最小移动精度，单位m
        float rotate_precision = 0.02f; // 最小旋转精度，单位rad    （0.017453 rad / 度）

        /**         构建接口    (Build)        **/
        void Build();

        /**         维护接口    (Update)        **/
        /// @brief 底盘的周期维护函数，200Hz调用一次（被耦合在RobotSystem中）
        void Update();

        /**         直接接口    (Direct)        **/
        void Config();
        void Enable();
        void Disable(); 
        /// @brief 直接设置底盘速度（一个通用的开环行为）
        void Move(Vec3 Spd);
        void Move(Vec2 Spd);
        void Rotate(float omega);
        
        /**         抛出接口    (Launch)        **/
        /// @brief 相对当前位置移动
        BaseAction* MoveLocal(Vec2 Pos);
        /// @brief 移动到 指定位置
        BaseAction* MoveAt(Vec2 Pos);
        BaseAction* MoveAt(Vec2 Pos, float MaxVelo, float MaxAccel);
        /// @brief 沿路径移动
        BaseAction* MoveAlong(Path path_t);
};

extern ChassisClass Chassis;

#endif