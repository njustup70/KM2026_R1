#ifndef _CHASSIS_HPP_
#define _CHASSIS_HPP_

#include "std_math.hpp"
#include "motor_dji.hpp"
#include "System.hpp"


#define ROTATE_RADIUS 0.4675f  //661.10/sqrt（2）
#define WHEEL_DIAMETER 0.16386f

class MoveAct;
class ChassisType;

// /**
//  * @brief 路径类，包含了一系列路径点
//  */
// class Path
// {
//     Vec2 key_points[32];            // 路径点数组
// };

class ChasAPIHandle
{
    public:
    ChasAPIHandle(){};
    
    bool Reached = false;     // 是否到达目标位置
};


// 底盘类型
typedef enum
{
    Mecanum,    // 麦轮底盘
    Omni,       // 全向轮底盘
    Steer,   // 舵轮底盘
}ChasType;

// 每一个舵轮看作一个舵轮模块
struct SteerMods
{
    MotorDJI steer_motor;     // 舵轮电机
    MotorDJI drive_motor;     // 驱动电机
    float targ_angle;         // 目标角度，单位rad
    float targ_speed;         // 目标速度，单位m/s
    Vec2 position;            // 舵轮位置，车体右手系，x向前，y向左，单位m
};

/**
* @brief 位置环控制器
* @note  目前是打算放在app层200Hz，输出速度指令给电机速度环1000Hz
*/
struct PosController
{
    float kp = 1.3f;     // 比例系数，单位: (m/s)/m
    float kd = 0.001f;     // 微分系数（实际速度反馈阻尼），单位: (m/s)/(m/s) 无量纲

    /**
    * @param error      位置误差向量，单位 m
    * @param actual_vel 当前实际速度向量，单位 m/s
    * @return           速度指令向量，单位 m/s
    */
    Vec2 Calc(Vec2 error, Vec2 actual_vel)
    {
        return error * kp - actual_vel * kd;
    }
};

/**
* @brief Yaw控制器
*/
struct YawController
{
    float kp            = 1.3f;    // 比例系数，单位: (rad/s)/rad
    float ki            = 0.4f;    // 积分系数，单位: (rad/s)/(rad·s)
    float integral_limit = 0.6f;   // 积分限幅，单位 rad（防止 windup）

    float _integral = 0.0f;

    /**
    * @param error  yaw 角度误差，单位 rad（已归一化到 [-π, π]）
    * @param dt     时间步长，200Hz 对应 0.005f
    * @return       角速度指令，单位 rad/s
    */
    float Calc(float error, float dt)
    {
        _integral += error * dt;
        _integral = fmaxf(-integral_limit, fminf(integral_limit, _integral));
        return kp * error + ki * _integral;
    }

    void Reset() { _integral = 0.0f; }
};
/**
 * @brief 底盘PID调参结构体
 * @note  在 Keil Watch 窗口里直接修改这里的参数，
 *        然后把 apply_flag 置为 1，框架会在下一帧自动重新配置所有电机并清零该标志。
 *        修改完成后 apply_flag 会自动回到 0，无需手动清除。
 */
struct ChasPidTuner
{
    // ---- 速度环参数（全向轮模式，四轮共用） ----
    float spd_kp      = 3.6f;       // 速度环比例系数
    float spd_ki      = 2.4f;       // 速度环积分系数
    float spd_kd      = 0.0f;       // 速度环微分系数
    float spd_i_lim   = 2.0f;       // 速度环积分限幅（A）
    float spd_out_lim = 10.0f;      // 速度环电流输出限幅（A）
    float cur_lim     = 15.0f;      // 总电流限幅（A），C620最大20A

    float pos_kp = 3.0f;
    float pos_kd = 0.6f;
    float yaw_kp = 2.0f;
    float yaw_ki = 0.4f;
    float yaw_integral_limit = 0.6f;

    float deadband_start = 1.0f;  
    float deadband_end = 5.0f;    

    // ---- 前馈参数 ----
    float ff_kf = 0.75f;    // 前馈系数
    float ff_K = 0.5f;     // 被控增益
    float ff_Tc = 0.02f;   // 时间常数
 
    // ---- 触发标志位 ----
    // 在 Watch 窗口将此值置为 1，框架会在下一帧重新 Apply 参数，完成后自动清零
    uint8_t apply_flag = 0;
 
    // ---- 只读状态（供观察，请勿在Watch窗口修改） ----
    uint8_t applied_count = 0;      // 累计已应用次数，用于确认是否生效
};
/**
 * @brief 底盘类
 * @note 底盘类中大部分是实现类的接口，更多复杂的算法都不在这里
 * 这里只提供一些简单的运动控制接口
 */
class ChassisType : public Application
{
    SINGLETON(ChassisType) : Application("chassis") {};
    APPLICATION_OVERRIDE
    private:

        ChasType _chas_type = Omni;    // 默认麦轮底盘

        /*<     底盘速度与加速度限制    >*/
        /// @param 最大加速度，单位m/s^2
        float _max_accel =2.0f;       
        /// @param 最大线速度，单位m/s
        float _max_velo = 1.0f;
        /// @param 最大角速度，单位rad/s
        float _max_omega = 3.0f;
        /// @param 最大角加速度，单位rad/s^2
        float _max_beta = 4.0f;

        /*<     底盘系数设置    >*/
        /// @param FARCON模式速度削减系数，为了提高手动的控制精度（手动不求很大的速度），且保持自动挡较高的速度
        float _farcon_decspeed = 0.3f;
        float _farcon_decyawspeed = 0.3f;

        /*<     死区参数    >*/
        /// @param 线速度死区，单位m/s（小于此值设为0）
        float _speed_deadzone = 0.05f;
        /// @param 角速度死区，单位rad/s（小于此值设为0）
        float _omega_deadzone = 0.05f;

        /*<     控制相关标志位    >*/
        bool _walking = false;              // 是否正MoveAt
        bool _rotating = false;             // 是否正RotateTo
        bool _is_pos_locked = false;        // 常锁位置环
        bool _is_yaw_locked = false;        // 常锁姿态环
        
        /// @param 四个电机的目标速度，单位m/s
        float _motor_spd[4];
            
        /// @param 安全锁定计时器
        int _safe_lock_tick = 0;
        
        /// @brief 发送到电机
        inline void _SendSpdToMotor(); 

        /// @brief 底盘速度上传
        void _UploadSpeed();

        /// @brief 底盘自解算里程计更新函数
        void _UpdateChasOdom();
    public:
        /// @brief 走到某点具体实现
        bool _Walking();
        /// @brief 转到某角度具体实现
        bool _Rotating();

        void _CalculateOmniMotorSpd();

        /// @brief 初始化舵轮模块
        void _InitSteerMods();

        /// @brief 计算舵轮目标角度与速度
        void _CalculateSteerTargets(const Vec3& chassis_speed);

        /// @brief 发送舵轮控制指令
        void _SendSteerCommands();

        /// @brief 将调参结构体的参数重新应用到所有电机
        void _ApplyPidTuner();

        Vec3 targ_ges = Vec3(0, 0, 0);      // 期望姿态，车体右手系，x向前，y向左，z从上向下看逆时针，单位m，rad
        Vec3 targ_speed;                    // 期望速度，车体右手系，x向前，y向左，z左转，单位m/s，rad/s

        Vec2 _rel_target_w; // 换算到世界坐标系下的绝对目标点

        typedef struct
        {
            Vec3 pos;
            Vec3 speed;         // 自解算的速度向量
            float velocity;     // 对应的线速度，单位m/s
        }_SelfResoOdom;
    
    public:
        /// @brief 底盘控制模式
        typedef enum
        {
            API,        // 由电控API控制
            HOST,       // 由工控机控制
            FARCON,     // 由遥控器控制
            FIELD_FARCON, //场地坐标系的遥控器控制
            UPHILL,
            LOCKYAW,
            FIELD_LOCKYAW,
            OPEN,       // 开放控制（直接Move控制）
        }_ChasConMode;

    public:
        // 分别对应四个底盘电机
        MotorDJI motors[4];

        // 四个舵轮
        SteerMods steer_mods[4];
        
        // 属性参数
        float velo;     // 对应的线速度，单位m/s
        bool enabled = false;    // 底盘使能标志
        float targ_velo = 0;

        /// @param 底盘里程计
        _SelfResoOdom chas_odom;

        /// @param 控制模式
        _ChasConMode control_mode = OPEN;

        float move_precision = 0.01f;       // 最小移动精度，单位m
        float rotate_precision = 0.02f;     // 最小旋转精度，单位rad    （0.017453 rad / 度）

         /**
         * @brief Debug调参入口
         * @note  使用方法：
         *   1. 在 Watch 窗口展开 test_chas.pid_tuner
         *   2. 修改 spd_kp / spd_ki / spd_kd / spd_i_lim / spd_out_lim / cur_lim
         *   3. 将 pid_tuner.apply_flag 置为 1
         *   4. 等待一帧，applied_count 加1说明已生效，apply_flag自动回到0
         */
        ChasPidTuner pid_tuner;

    private:
        PosController  _pos_ctrl;   // XY 位置 PD 控制器
        YawController  _yaw_ctrl;   // Yaw PI 控制器
        Pids _pos_pidcontroller;


    public:
        /**         直接接口    (Direct)        **/
        void SetType(ChasType type) { _chas_type = type; };
        ChasType GetChasType() {return _chas_type;};
        void Config();
        void Enable();
        void Disable(); 
        /// @brief 直接设置底盘速度（一个通用的开环行为）
        void Move(Vec3 Spd);
        void Move(Vec2 Spd);
        void Move(Vec3 Spd, float duration);
        void Move(Vec2 Spd, float duration);
        
        void Rotate(float omega);

        void MoveAt(Vec2 Pos);
        void RotateAt(float yaw);

        /**
        * @brief 控制车体相对车体当前姿态进行X, Y方向的定量位移
        * @param rel_xy 相对车身坐标系的位移增量（x向前为正，y向左为正），单位：m
        */
        void MoveRelative(Vec2 rel_xy);

        void LockPosition();
        void UnlockPosition();

        //跟通讯相关 CommCenter
        friend class CommCenter;
};
namespace APP
{
    extern ChassisType& chassis;
}

#endif