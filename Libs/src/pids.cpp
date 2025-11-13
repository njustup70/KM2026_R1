// filePath: pids.c

#include "pids.hpp"
#include "bsp_dwt.h"
#include "arm_math.h"

float limit_ab(float a, float limit)
{
    if (a > limit)
    {
        return limit;
    }
    else if (a < -limit)
    {
        return -limit;
    }
    else
    {
        return a;
    }
}


/// @brief 构造函数（完整版）
/// @param dt 若不填写，默认使用DWT库测得的时间间隔
/// @param reverse 是否反转输出
/// @param intLim 积分限幅
/// @param outLim 输出限幅
/// @param dtFilter 微分滤波器系数
/// @param Incremental 是否是增量PID
/// @param Feedforward 是否启用前馈控制 
PidGeneral::PidGeneral(float kp, float ki, float kd,float dt, int reverse,
    float intLim, float outLim, float dtFilter, bool Incremental, bool Feedforward, bool InnerAcc)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
    delta_t = dt;
    this->reverse = reverse;

    inte_errors = 0;
    last_error = 0;
    prev_error = 0;
    kd_error = 0;
    control_value = 0;

    inte_lim = intLim;
    out_lim = outLim;
    kd_filter_rate = dtFilter;
    this->Incremental = Incremental;
    this->Feedforward = Feedforward;
    this->InnerAcc = InnerAcc;
}

/**
 * @brief 完整初始化函数
 */
void PidGeneral::Init(float kp, float ki, float kd, float kf, float dt, int reverse,
    float intLim, float outLim, float dtFilter, bool Incremental, bool Feedforward, bool InnerAcc)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
    Kf = kf;
    delta_t = dt;
    this->reverse = reverse;

    inte_errors = 0;
    last_error = 0;
    prev_error = 0;
    kd_error = 0;
    control_value = 0;

    inte_lim = intLim;
    out_lim = outLim;
    kd_filter_rate = dtFilter;
    this->Incremental = Incremental;
    this->Feedforward = Feedforward;
    this->InnerAcc = InnerAcc;
    this->AutoDt = (dt == 0); // 若dt为0则启用自动计算
}

/**
 * @brief 快速初始化函数
 * @note 默认使用 内部维护增量式PID ，启用前馈控制
 */
void PidGeneral::FastInit(float kp, float ki, float kd, float kf, float outLim, int reverse)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
    Kf = kf;
    delta_t = 0;
    this->reverse = reverse;

    inte_errors = 0;
    last_error = 0;
    prev_error = 0;
    kd_error = 0;
    control_value = 0;

    inte_lim = 0;       // 由于快速初始化默认用增量式，积分限幅设为0（增量式用outlim）
    out_lim = outLim;
    kd_filter_rate = 0.5f;
    Incremental = true;
    Feedforward = true;
    InnerAcc = true;
    AutoDt = true;      // 快速初始化默认启用自动计算时间间隔
}

/**
 * @brief 重置PID控制器的状态
 * @param targ_pid PID结构体指针
 */
void PidGeneral::Reset()
{
    inte_errors = 0;
    last_error = 0;
    prev_error = 0;
    kd_error = 0;
    control_value = 0;
}

/**
 * @brief 计算PID输出（供外部调用）
 * @note 根据配置的模式，调用不同的计算函数，输出结果也不同
 */
float PidGeneral::Calc(float targ, float real, float output_lim)
{
    float result;
     
    if (Incremental)
    {
        if (InnerAcc)           result = CalcIncAuto(targ, real, output_lim);      // 增量式PID，带内部累加
        else                    result = CalcInc(targ, real, output_lim);          // 增量式PID
    }
    else                    
    {
        result = CalcPos(targ, real, output_lim);      // 位置式PID
    }

    // 死区控制
    if (DeadbandEnabled)
    {
        if (fabs(error) < deadband_start) result = 0;
        else if (fabs(error) < deadband_end)
        {
            // 线性映射
            result = (fabs(error) - deadband_start) / (deadband_end - deadband_start) * result;
        } 
    }
    
    return result;
}


/**
 * @brief 计算位置式PID的输出
 * @param error 当前误差
 * @param output_lim 输出限幅（若为0则不限制）
 * @return PID控制输出 
 */
float PidGeneral::CalcPos(float targ, float real, float output_lim)
{
    float pid_output;
    error = targ - real;

    // 自适应时间间隔
    if (AutoDt) delta_t = DWT_GetDeltaTime(&dwt_dt); // 单位为秒

    // 为Kd计算经过一阶低通滤波的误差
    kd_error = kd_error + kd_filter_rate * (error - kd_error);
    
    // 累加积分项
    inte_errors += Ki * error * delta_t;

    // 如果用户配置了积分限幅，作限幅
    if (inte_lim > 0) inte_errors = limit_ab(inte_errors, inte_lim);

    // 计算普通PID
    pid_output = Kp * error + inte_errors + Kd * ((kd_error - last_kderr) / delta_t);

    // 记录误差项，用于下次计算
    prev_error = last_error;
    last_error = error;
    last_kderr = kd_error;

    // 前馈控制
    if (Feedforward) pid_output += Kf * targ;

    // 如果用户配置了反向，作反向输出
    if (reverse) pid_output = -pid_output;
    
    // 应用输出限幅（如果配置了限幅）
    if (output_lim > 0) pid_output = limit_ab(pid_output, output_lim);

    // 应用内部输出限幅（如果配置了限幅）
    if (out_lim > 0)     pid_output = limit_ab(pid_output, out_lim);

    

    return pid_output;
}

/**
 * @brief 计算增量式PID的输出增量
 * @param targ_pid PID结构体指针
 * @param error 当前误差
 * @return 控制量的 `增量` 
 */
float PidGeneral::CalcInc(float targ, float real, float output_lim)
{
    error = targ - real;

    // 自适应时间间隔
    if (AutoDt) delta_t = DWT_GetDeltaTime(&dwt_dt); // 单位为秒

    // 增量式PID的微分项低通滤波
    kd_error = kd_error + kd_filter_rate * (error - kd_error);
    
    // 计算增量式PID
    inc_output =    Kp * (error - last_error) +
                    Ki * error * delta_t +
                    Kd * (kd_error - 2 * last_error + prev_error) / delta_t;
    
    // 记录误差项，用于下次计算
    prev_error = last_error;
    last_error = error;
    
    // 如果用户配置了反向，作反向输出
    if (reverse)            inc_output = -inc_output;
    
    // 应用输出限幅（如果配置了限幅） 
    if (output_lim > 0)     inc_output = limit_ab(inc_output, output_lim);

    // 返回增量
    return inc_output;
}

/**
 * @brief 计算增量式PID的输出（内部维护累加值和限幅），带微分滤波器
 * @param targ_pid PID结构体指针
 * @param error 当前误差
 * @return 经过累加和限幅后的控制量
 */
float PidGeneral::CalcIncAuto(float targ, float real, float output_lim)
{
    error = targ - real;
    
    // 自适应时间间隔
    if (AutoDt) delta_t = DWT_GetDeltaTime(&dwt_dt); // 单位为秒

    // 增量式PID的微分项低通滤波
    kd_error = kd_error + kd_filter_rate * (error - kd_error);
    
    // 计算控制量增量，使用误差进行微分项计算（P和I千万不能用微分滤波误差）
    inc_output =    Kp * (error - last_error) +
                    Ki * error * delta_t +
                    Kd * (kd_error - 2 * last_kderr + prev_kderr) / delta_t;
    
    // 记录误差项，用于下次计算
    prev_error = last_error;
    last_error = error;

    prev_kderr = last_kderr;
    last_kderr = kd_error;
    
    // 更新控制量
    control_value += inc_output;
    
    // 前馈控制
    if (Feedforward)        return control_value + Kf * targ;
    else                    return control_value;

    // 应用外界输出限幅（如果配置了限幅）
    if (output_lim > 0)     control_value = limit_ab(control_value, output_lim);

    // 应用内部输出限幅（如果配置了限幅）
    if (out_lim > 0)     control_value = limit_ab(control_value, out_lim);
    
    // 如果用户配置了反向，作反向输出
    if (reverse)            control_value = -control_value;
    
    
}

/**
 * @brief 修改设置 PID参数
 */
void PidGeneral::ParamSet(float kp, float ki, float kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

/**
 * @brief 修改设置 PID限幅
 */
void PidGeneral::LimitSet(float integralLim, float outputLim, float deltaFilter)
{
    inte_lim = integralLim;
    out_lim = outputLim;
    kd_filter_rate = deltaFilter;
}       

/**
 * @brief 设置反向控制
 */ 
void PidGeneral::RevSet(bool reverse)
{
    this->reverse = reverse;
}

/**
 * @brief 设置死区控制
 */
void PidGeneral::DeadbandSet(float start, float end)
{
    deadband_start = start;
    deadband_end = end;
    DeadbandEnabled = (end > start); // 若end > start（输入合法）则启用死区控制
}