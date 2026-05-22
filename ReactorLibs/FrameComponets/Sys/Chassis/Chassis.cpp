#include "Chassis.hpp"
#include "arm_math.h"
#include "Monitor.hpp"
#include "farcon.hpp"
#include "System.hpp"

extern Farcon farcon;
ChassisType& test_chas = ChassisType::GetInstance();

void ChassisType::Start()
{
    // 初始化底盘类型
    SetType(Omni);
    if(_chas_type == Omni)
    {
        // 初始化电机
        // 轮序：
        //         前
        //     1       2
        //
        //     3       4  
        for (int i = 0; i < 4; i++)
        {
            // 初始化PID
            motors[i].Init(Hardware::hcan_main, i + 1, DJI_C620);
            motors[i].ConfigPID()
                        .AsSpeedC()
                        .Spd_Coeff(0.1f, 0.07f, 0.0f)    
                        .Spd_Limit(5.0f, 10.0f)
                        .SpdLimit(7000.0f)       //469rpm（手册的额定转速应该是输出轴的） * 268/17（减速比）
                        .CurLimit(10.0f) 
                        .Apply();
            //motors[i].speed_pid.SetDeadband(1.0, 3.5);
            //ForwardLize(前馈类型, 前馈系数, 被控对象增益K, 时间常数Tc)
            //motors[i].speed_pid.ForwardLize(
            //                    PidGeneral::SpeedForward,  // 前馈类型
            //                    0.015f,                      // 前馈系数 Kf（0~1，权重）
            //                    0.75f,                      // 被控对象增益 K
            //                    0.02f                      // 时间常数 Tc (秒)
            //                    );
            //motors[i].ConfigADRC()
            //             .AsSpeedC()
            //             .ADRC_Womega(42.0f, 9.6f)
            //             .ADRC_Physic(3.0e-4f, 0.30f, 0.005f)
            //             .ADRC_Limit(15.0f)
            //             .SpdLimit(3000.0f)
            //             .ADRC_MaxPlannedVel(3000.0f)
            //             .ADRC_SOTF(0.5f)
            //             .Apply();
            motors[i].driver.SetReduRatio(MotorDJIReduConst::redu_M3508_G); 
            motors[i].driver.Enable();
        }
        test_chas.Enable();
    }
    else if(_chas_type == Steer)
    {
        _InitSteerMods();
    }
    test_chas.enabled = true;
}

void ChassisType::_InitSteerMods()
{
    // 轮序：
    //         前
    //     0       1
    //
    //     2       3    
    // 舵轮底盘参数
    float wheelbase = 0.5f;    // 前后轮距
    float trackwidth = 0.5f;   // 左右轮距

    // 初始化舵轮位置
    steer_mods[0].position = Vec2(wheelbase / 2.0f, trackwidth / 2.0f);   // 前左
    steer_mods[1].position = Vec2(wheelbase / 2.0f, -trackwidth / 2.0f);  // 前右
    steer_mods[2].position = Vec2(-wheelbase / 2.0f, trackwidth / 2.0f);   // 后左
    steer_mods[3].position = Vec2(-wheelbase / 2.0f, -trackwidth / 2.0f);  // 后右

    // 初始化电机
    for (int i = 0; i < 4; i++)
    {
        steer_mods[i].steer_motor.Init(Hardware::hcan_main, i + 1);
        steer_mods[i].drive_motor.Init(Hardware::hcan_main, i + 5);
    }
    // 初始化PID，或者ADRC
}

/**
 * @brief 将 pid_tuner 里的参数重新应用到所有电机
 * @note  由 Update() 在检测到 apply_flag==1 时调用，调用完毕后 apply_flag 自动清零
 */
void ChassisType::_ApplyPidTuner()
{
    for (int i = 0; i < 4; i++)
    {
        motors[i].ConfigPID()
                    .AsSpeedC()
                    .Spd_Coeff(pid_tuner.spd_kp, pid_tuner.spd_ki, pid_tuner.spd_kd)
                    .Spd_Limit(pid_tuner.spd_i_lim, pid_tuner.spd_out_lim)
                    .CurLimit(pid_tuner.cur_lim)
                    .Apply();
        // 注意：ConfigPID 链式调用结束时 mode 已经被设回 SpeedC，
        // 但 Apply() 不会重新 Enable driver，原有使能状态保持不变。
        motors[i].speed_pid.SetDeadband(pid_tuner.deadband_start, pid_tuner.deadband_end);
        motors[i].speed_pid.ForwardLize(
                    PidGeneral::SpeedForward,
                    pid_tuner.ff_kf,
                    pid_tuner.ff_K,
                    pid_tuner.ff_Tc
                    );
    }
 
    pid_tuner.apply_flag = 0;           // 自动清零，Watch窗口可观察到变化
    pid_tuner.applied_count++;          // 计数+1，用于确认是否生效
}

void ChassisType::Update()
{
    // 安全退debug
    if(System.out_from_debugmode)
    {
        for(int i = 0; i < 4; i++)
        {
            motors[i].Neutral();
            motors[i].driver.Disable();
        }
    }

    // ---- Debug调参检测（每帧检查，置1即触发，完成后自动归零）----
    if (pid_tuner.apply_flag == 1)
    {
        _ApplyPidTuner();
    }

    // 遥控器控制逻辑
    if(farcon.toggle[1] == 1 && farcon.toggle[2] == 0 && farcon.toggle[3] == 0)
    {
        control_mode = FARCON;
    }
    else if(farcon.toggle[1] == 0 && farcon.toggle[2] == 1)
    {
        control_mode = API; //可启用自动规划路径并自动巡航 见Logic.cpp
    }
    else
    {
        control_mode = OPEN;
    }

    if(farcon.toggle[3] == 1)
    {
        LockPosition();
    }

    if(control_mode == FARCON)
    {
        // 读取遥控器数据到底盘控制变量
        targ_speed.x = -farcon.jy_data_origin[3]*1.0f / 100.f * _max_velo;   // 前后
        targ_speed.y = -farcon.jy_data_origin[2]*1.0f / 100.f * _max_velo;   // 左右
        targ_speed.z = -farcon.jy_data_origin[0]*1.0f / 100.f * _max_omega;  // 旋转
        Move(targ_speed);
    }

    bool walking_complete = false; //这个变量只在这一帧有用
    bool rotaing_complete = false;

    // 实现闭环的地方
    if (_walking || _is_pos_locked)
    {
        walking_complete = _Walking();
    }

    // 当MoveAt完成且只是位置锁定（不是显式RotateTo）时，清除yaw锁定
    if (walking_complete && _is_yaw_locked && !_rotating)
    {
        _is_yaw_locked = false;
    }

    if (_rotating || _is_yaw_locked)
    {
        rotaing_complete = _Rotating();
    }

    // if(rotaing_complete && _is_pos_locked && !_walking)
    // {
    //     _is_pos_locked = false;   
    // } 
    
    // 将底盘的 速度targ_speed 上传到各个电机
    _UploadSpeed();

    // 更新自解算里程计
    _UpdateChasOdom();

    targ_velo = targ_speed.Length();

    // 安全锁倒计时
    _safe_lock_tick -= 5;
}



void ChassisType::_UpdateChasOdom()
{
    // （1）获得当前角度
    float theta_distan = 0;     // 单位：米
    for (int i = 0; i < 4; i++)
    {
        theta_distan += motors[i].driver.measure.total_angle;
    }
    theta_distan = theta_distan / (MotorDJIReduConst::redu_M3508 * 8192) * (PI * WHEEL_DIAMETER) / 4.0f;
    float chas_theta = theta_distan / ROTATE_RADIUS;   // 单位：弧度
    
    // （2）获取车体速度(读取而不是控制的速度，以减少误差)
    Vec3 chas_speed;        // 都是线速度
    // 旋转分量
    for (int i = 0; i < 4; i++)
    {
        chas_speed.z += motors[i].driver.measure.speed_rpm;
    }
    chas_speed.z = (chas_speed.z / 240.0f) / (MotorDJIReduConst::redu_M3508) * (PI * WHEEL_DIAMETER);

    // 获得每个电机不带旋转速度的线速度分量（用于计算x, y方向上的速度）
    float motor_spd_xy[4] = {0};
    for (int i = 0; i < 4; i++)
    {
        motor_spd_xy[i] = (motors[i].driver.measure.speed_rpm / 60.0f / motors[i].driver.redu_ratio) * (PI * WHEEL_DIAMETER) - chas_speed.z;
    }

    Vec2 chas_vxy;
    chas_vxy.x = (motor_spd_xy[1] - motor_spd_xy[2]) / 2.0f;
    chas_vxy.y = (motor_spd_xy[0] - motor_spd_xy[3]) / 2.0f;


    // （3）更新里程计，还有速率
    Vec2 delta_move = chas_vxy.Rotate(chas_theta + (PI / 4)) / 200.0f;
    chas_odom.velocity = chas_vxy.Length();

    chas_odom.speed = (delta_move * 200.0f).ToVec3();
    chas_odom.speed.z = chas_speed.z;
    
    chas_odom.pos = chas_odom.pos + delta_move.ToVec3();
    chas_odom.pos.z = chas_theta;
}

void ChassisType::_UploadSpeed()
{
    static float runtime_cnt = 0;
    // 仅当底盘使能时才工作
    if (enabled && _safe_lock_tick > 0)
    {
        runtime_cnt = 0;
        _SendSpdToMotor();
    }
    else
    {
        // 底盘未使能，分两种情况
        // (1) 底盘仍有速度
        if(chas_odom.speed.Length() > 0.15f)
        {
            if (runtime_cnt < 0.001f)   runtime_cnt = System.runtime_tick;

            // (1.1) 仍有速度，先刹车停
            if(System.runtime_tick - runtime_cnt < 1.0f)
            {
                // 否定其他接口的控制权，并进行刹车
                targ_speed = targ_speed * 0.96f;
                _SendSpdToMotor();
            }
            else    // (1.2) 1s还停不下来，强制进入空档
            {
                for (int i = 0; i < 4; i++)
                {
                    targ_speed = Vec3(0, 0, 0);
                    motors[i].Neutral();
                }
            }
        }
        else    // (2) 底盘已经停止，直接进入空档
        {
            for (int i = 0; i < 4; i++)
            {
                targ_speed = Vec3(0, 0, 0);
                motors[i].Neutral();
            }
        }

        
    }
}

inline void ChassisType::_SendSpdToMotor()
{
    // // ---- 死区控制：过滤掉微小的控制信号 ----呃死区控制有点问题
    // targ_speed.x = (fabs(targ_speed.x) < _speed_deadzone) ? 0.0f : targ_speed.x;
    // targ_speed.y = (fabs(targ_speed.y) < _speed_deadzone) ? 0.0f : targ_speed.y;
    // targ_speed.z = (fabs(targ_speed.z) < _omega_deadzone) ? 0.0f : targ_speed.z;

    if (_chas_type == Steer) 
    {
        // 舵轮
        _CalculateSteerTargets(targ_speed);
        _SendSteerCommands();
    }
    else if(_chas_type == Omni) 
    {
        // 全向轮
        _CalculateOmniMotorSpd();
    }
}

void ChassisType::_CalculateOmniMotorSpd()
{
    // 计算x, y, w合成分量
    _motor_spd[0] = (-targ_speed.x + targ_speed.y)  / (BSP_SQRT2) + targ_speed.z * ROTATE_RADIUS;
    _motor_spd[1] = (targ_speed.x + targ_speed.y) / (BSP_SQRT2) + targ_speed.z * ROTATE_RADIUS;
    _motor_spd[2] = (-targ_speed.x - targ_speed.y)  / (BSP_SQRT2) + targ_speed.z * ROTATE_RADIUS;
    _motor_spd[3] = (targ_speed.x - targ_speed.y) / (BSP_SQRT2) + targ_speed.z * ROTATE_RADIUS;

    // 发送速度指令到电机
    for (int i = 0; i < 4; i++)
    {
        // if(motors[i].mode != MotorDJIMode::SpeedC) 
        // {
        //     motors[i].Uneutral(); 
        // }
        motors[i].mode = MotorDJIMode::SpeedC;
        motors[i].SetSpd((_motor_spd[i] * 60.0f) / (PI * WHEEL_DIAMETER) * motors[i].driver.redu_ratio); // 速度转换为RPM，注意要乘以减速比
    }    
}

float my_copysign(float x, float y)
{
    return (y >= 0) ? fabs(x) : -fabs(x);
}   

void ChassisType::_CalculateSteerTargets(const Vec3& chassis_speed)
{
    float vx = chassis_speed.x;      // 底盘x方向速度（前向）
    float vy = chassis_speed.y;      // 底盘y方向速度（左向）
    float omega = chassis_speed.z;   // 底盘角速度（逆时针）
    
    for (int i = 0; i < 4; i++) 
    {
        SteerMods& module = steer_mods[i];
        
        // 计算每个舵轮的目标线速度
        // v_module = v_chassis + omega × r_module
        float v_module_x = vx - omega * module.position.y;
        float v_module_y = vy + omega * module.position.x;
        
        // 计算目标速度大小和方向
        float speed_magnitude = sqrt(v_module_x * v_module_x + v_module_y * v_module_y);
        float angle = atan2(v_module_y, v_module_x);
        
        // 优化转向角度，选择最近的转向方向
        float current_angle = module.steer_motor.driver.measure.total_angle / (module.steer_motor.driver.redu_ratio * 8192) * 2 * PI; // 这个角度要根据底盘舵向实际减速比改
        
        float angle_diff = angle - current_angle;

        // 调整角度差到[-PI, PI]范围
        while (angle_diff > PI) angle_diff -= 2 * PI;
        while (angle_diff < -PI) angle_diff += 2 * PI;
        
        // 如果角度差大于90度，反转速度方向并调整角度
        if (fabs(angle_diff) > PI / 2) 
        {
            angle_diff -= (angle_diff >= 0 ? PI : -PI);
            speed_magnitude = -speed_magnitude;
        }
        
        // 设置目标角度和速度
        module.targ_angle = current_angle + angle_diff;
        module.targ_speed = speed_magnitude;
    }
}

void ChassisType::_SendSteerCommands()
{
    for (int i = 0; i < 4; i++) 
    {
        SteerMods& module = steer_mods[i];
        
        // 发送转向指令（角度转换为电机编码器值）
        float encoder_angle = module.targ_angle / (2 * PI) * (module.steer_motor.driver.redu_ratio * 8192); // 这个编码值要根据底盘舵向实际减速比改
        module.steer_motor.SetPos(encoder_angle);
        
        // 发送驱动指令（速度转换为RPM）
        float rpm = (module.targ_speed * 60.0f) / (PI * WHEEL_DIAMETER) * module.drive_motor.driver.redu_ratio;
        module.drive_motor.SetSpd(rpm);
    }
}


void ChassisType::Enable()
{
    enabled = true;
}

void ChassisType::Disable()
{
    // 停止所有电机
    enabled = false;
}


void ChassisType::MoveAt(Vec2 Pos)
{
    targ_ges = Vec3(Pos.x, Pos.y, System.position.z);
    _walking = true;
}

static inline float NormalizeAngle(float angle)
{
    while (angle > PI) angle -= 2 * PI;
    while (angle <= -PI) angle += 2 * PI;
    return angle;
}

void ChassisType::RotateAt(float yaw)
{
    targ_ges.z = NormalizeAngle(yaw);
    _rotating = true;
}

/**
 * @brief 直接设置底盘速度（一个通用的开环行为）
 * @param Spd 期望速度：（x: 前向速度，y：左向速度，w：逆时针）（m/s，m/s，rad/s）
 * @note 轮序：     
 *                      前
 *                  0       1
 *                  
 * 
 *                  2       3
 * @warning 每次设置速度都会刷新安全锁，需要持续调用以保持底盘运动。
 * 在指令中断100ms后，底盘会自动进入空档（0电流）状态。
 * （100ms已经很长了，相当于20个指令周期都没有指令输入）
 */
void ChassisType::Move(Vec3 Spd)
{
    if (!enabled)  return;         // 开放控制未使能，直接返回

    _safe_lock_tick = 100;
    targ_speed = Spd;
}

void ChassisType::Move(Vec2 Spd)   //Spd单位m/s
{
    if (!enabled)  return;

    _safe_lock_tick = 100;

    targ_speed.x = Spd.x;
    targ_speed.y = Spd.y;
}

void ChassisType::Rotate(float omega)
{
    if (!enabled)  return;

    // 验证输入安全
    if (isnan(omega) || omega == INFINITY)
    {
        Monitor::GetInstance().LogError("Chassis: Dangerous omega!");
        return;
    }
    
    // 输入合法化
    if (omega > _max_omega)
    {
        omega = _max_omega;
    }
    else if (omega < -_max_omega)
    {
        omega = -_max_omega;
    }
    
    _safe_lock_tick = 100;
    targ_speed.z = omega;
}





/**
 * @brief 基于直接移动到位置的方式
 * @details 被 Update 调用
 * 需要用到的信息流：底盘的控制、当前的位置、底盘的速度
 * @warning 只管xy的姿态，不管yaw角
 */
bool ChassisType::_Walking()
{
    _is_yaw_locked = true;  // 启用yaw锁定

    // 计算移动向量
    Vec2 move_vec = targ_ges.ToVec2() - System.position.ToVec2();
    // 带入车体旋转，将世界坐标系转成车体坐标系
    move_vec = move_vec.Rotate(- System.position.z);

    // 检查是否到达目标位置, 如果是则返回完成
    if (move_vec.Length() < move_precision)    // 1cm范围内视为到达
    {
        Move(Vec2(0, 0));           // 停止移动
        _walking = false;
        return true;                 // 动作完成
    }

    // 计算移动速度
    float safe_velo = sqrt(2 * _max_accel * move_vec.Length()); // 意思是就算以最大加速度走，到达目标距离也能停下来，单位m/s
    float out_velo = _pos_ctrl.Calc(move_vec, chas_odom.speed.ToVec2()).Length(); // 位置控制器输出的速度，单位m/s
    //float out_velo = 3.0f * move_vec.Length(); // 简单的比例控制，距离越近速度越快，单位m/s
    // 最终的速度应该为三者中的最小值
    float final_velo = fminf(safe_velo, fminf(out_velo, _max_velo)); //单位m/s

    // 更新底盘速度（向量式更新，保证更新量不大于MaxAccel）
    Vec2 targ_speed_vec = move_vec.Norm() * final_velo;     // 计算新的目标速度，向量先归一化然后给个模值
    Vec2 curr_speed_vec = targ_speed.ToVec2();           // 当前速度，这个targ_speed是上次调用Move函数设置的目标速度
    // 计算速度差
    Vec2 delta_speed_vec = targ_speed_vec - curr_speed_vec;
    float delta_speed_len = delta_speed_vec.Length();

    // 限制加速度（向量长度自带绝对值）
    if (delta_speed_len > (_max_accel / 200.0f))   // 每次调用都是1 / 200s
    {
        delta_speed_vec = delta_speed_vec.Norm() * (_max_accel / 200.0f);
    }
    Vec2 new_speed_vec = curr_speed_vec + delta_speed_vec;

    // 调用移动接口进行移动
    Move(new_speed_vec);
		
    return false;
}

/**
 * @brief 不断尝试旋转到某个角度
 * @details 被 Update 调用
 * 需要用到的信息流：底盘的控制、当前的位置、底盘的速度
 * @warning 只管yaw角，不管xy的姿态
 */
bool ChassisType::_Rotating()
{
    //_is_pos_locked = true;  // 启用位置锁定
    // 计算旋转向量 （速度Rad / s)
    float rotate_diff = NormalizeAngle(targ_ges.z - System.position.z);

    // 检查是否到达目标位置, 如果是则返回完成
    if (fabs(rotate_diff) < 0.01f)    // 0.01rad范围内视为到达
    {
        Rotate(0);           // 停止
        _rotating = false;
        return true;                 // 动作完成
    }
    
    // 计算旋转速度 （注意绝对值）
    float safe_omega = sqrt(2 * _max_beta * fabs(rotate_diff)); 
    float out_omega = 3.0f * fabs(rotate_diff);

    // 最终的速度应该为三者中的最小值
    float targ_omega = fminf(safe_omega, fminf(out_omega, _max_omega));

    // 更新底盘速度（向量式更新，保证更新量不大于MaxAccel）
    // 计算速度差
    float delta_speed = targ_omega * (rotate_diff > 0 ? 1 : -1) - targ_speed.z;

    // 限制加速度（注意绝对值）
    if (fabs(delta_speed) > (_max_beta / 200.0f))   // 每次调用都是1 / 200s
    {
        delta_speed = (delta_speed > 0 ? 1 : -1) * (_max_beta / 200.0f);
    }
    
    float new_omega = targ_speed.z + delta_speed;

    // 调用旋转接口进行移动
    Rotate(new_omega);
		
    return false;
}

void ChassisType::LockPosition()
{
    if (!enabled) return;
    targ_ges = System.position;

    // 重置控制器，避免继承之前运动遗留的积分
    _yaw_ctrl.Reset();

    // 清除其他运动任务，进入自锁状态
    _walking        = false;
    _rotating       = false;
    _is_pos_locked  = true;
    _is_yaw_locked  = true;

    _safe_lock_tick = 100;  // 刷新安全锁
}