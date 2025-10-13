#ifndef PIDS_HPP
#define PIDS_HPP
#ifdef __cplusplus
#include "stm32f4xx_hal.h"


extern "C"
{
#endif
    // 通用PID类
    class PidGeneral
    {
    private:
        // PID参数
        float Kp;      // 比例系数
        float Ki;      // 积分系数
        float Kd;      // 微分系数
        float Kf;      // 前馈系数
        float delta_t; // 时间间隔
        int reverse;   // 反向控制标志
        uint32_t dwt_dt;    // 自动DT用的DWT句柄

        // 限制参数
        float inte_lim;    // 积分限幅
        float out_lim;      // 输出限幅
        float kd_filter_rate; // Kd低通滤波系数
        
        /// @brief 举个例子吧：假设死区起始为80，结束为160，那么误差 < 80输出0，误差 > 160正常输出，80 ~ 160之间线性映射
        float deadband_start = 0; // 死区起始值
        float deadband_end = 0;   // 死区结束值

        // PID高级选项
        bool Incremental; // 是否为增量式PID
        bool Feedforward; // 是否启用前馈控制
        bool InnerAcc;    // 是否启用内部累加（仅增量式有效）
        bool AutoDt;      // 是否自动计算PID时间间隔
        bool DeadbandEnabled;    // 是否启用死区控制

        float CalcPos(float targ, float real, float output_lim = 0);               // 位置式计算
        float CalcInc(float targ, float real, float output_lim = 0);               // 增量式计算
        float CalcIncAuto(float targ, float real, float output_lim = 0);           // 带内部累加的 增量式计算
        
    public:
        // 内部状态变量
        float inte_errors;      // 积分误差累计
        float error;            // 当前误差
        float last_error;      // 上一次误差
        float prev_error;      // 上上次误差（用于增量式计算）
        
        float kd_error;        // 微分误差
        float last_kderr;       
        float prev_kderr;      
        
        float inc_output;       // 增量式PID的增量输出
        float control_value;   // 内部维护控制量累加值

        // 构造函数，带默认参数
        PidGeneral(float kp, float ki, float kd,
            float dt = 0, int reverse = 0,
            float integralLim = 0, float outputLim = 0,
            float deltaFilter = 0.5f,
            bool Incremental = false, bool Feedforward = false,
            bool InnerAcc = true);
        PidGeneral(){}; // 默认构造函数
        
        // 初始化函数，带完整参数
        void Init(float kp, float ki, float kd, float kf,
            float dt = 0, int reverse = 0,
            float integralLim = 0, float outputLim = 0,
            float deltaFilter = 0.5f,
            bool Incremental = false, bool Feedforward = false,
            bool InnerAcc = true);
        // 快速初始化函数
        void FastInit(float kp, float ki, float kd, float kf = 0,
            float outputLim = 0, int reverse = 0);
        
        // 计算PID输出（供外部调用） 
        float Calc(float targ, float real, float output_lim = 0);

        void Reset();                                       // 重置PID状态
        void ParamSet(float kp, float ki, float kd);       // 设置PID参数
        void LimitSet(float integralLim, float outputLim, float deltaFilter);          // 设置限制参数
        void RevSet(bool reverse);                       // 设置反向控制
        void DeadbandSet(float start, float end);       // 设置死区控制
    };

typedef PidGeneral Pids; // 兼容C代码中的PIDs类型定义

#ifdef __cplusplus
}
#endif
#endif