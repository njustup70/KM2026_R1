#include "Chassis.hpp"
#include "arm_math.h"
#include "RobotSystem.hpp"




// 唯一全局实例
ChassisClass Chassis;

void ChassisClass::Build()
{
    // 初始化电机
    for (int i = 0; i < 4; i++)
    {
        motors[i].Init(&hcan1, i + 1, MotorDJIMode::Speed_Control, true);
        motors[i].Enable();
    }
}

void ChassisClass::Enable()
{
    enabled = true;
}

void ChassisClass::Disable()
{
    // 停止所有电机
    enabled = false;
}

void ChassisClass::Update()
{
    // 仅当底盘使能时才工作
    if (enabled)
    {
        // 计算x, y, w合成分量
        motor_spd[0] = (speed.x - speed.y)  / (BSP_SQRT2) - speed.z * ROTATE_RADIUS;
        motor_spd[1] = (-speed.x - speed.y) / (BSP_SQRT2) + speed.z * ROTATE_RADIUS;
        motor_spd[2] = (speed.x + speed.y)  / (BSP_SQRT2) - speed.z * ROTATE_RADIUS;
        motor_spd[3] = (-speed.x + speed.y) / (BSP_SQRT2) + speed.z * ROTATE_RADIUS;

        // 发送速度指令到电机
        for (int i = 0; i < 4; i++)
        {
            motors[i].SetSpeed((motor_spd[i] * 60.0f) / (PI * WHEEL_DIAMETER));
        }
    }
    else
    {
        // 底盘未使能，发送 0 电流（空档）
        for (int i = 0; i < 4; i++)
        {
            motors[i].Neutral();
        }
    }
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
void ChassisClass::Move(Vec3 Spd)
{
    speed = Spd;
}

void ChassisClass::Move(Vec2 Spd)
{
    speed.x = Spd.x;
    speed.y = Spd.y;
}

void ChassisClass::Rotate(float omega)
{
    speed.z = omega;
}

BaseAction* ChassisClass::MoveAt(Vec2 Pos)
{
    // 如果当前动作正在运行，先取消
    if (move_action.GetState() == Action.RUNNING)
    {
        move_action.Cancel();
    }
    
    move_action.movetype = MoveAct::MoveType::AtPos;
    move_action.Reset(Pos);

    return &move_action;
}

BaseAction* ChassisClass::MoveAt(Vec2 Pos, float maxVelo, float maxAccel)
{
    // 如果当前动作正在运行，先取消
    if (move_action.GetState() == Action.RUNNING)
    {
        move_action.Cancel();
    }
    
    move_action.movetype = MoveAct::MoveType::AtPos;
    move_action.Reset(Pos);
    move_action.max_accel = maxAccel;
    move_action.max_velo = maxVelo;

    return &move_action;
}

BaseAction* ChassisClass::MoveAlong(Path path_t)
{
    return nullptr;
}


void MoveAct::Reset(Vec2 new_target)
{
    target_pos = new_target;            // 更新目标速度
    state = Action.READY;               // 重置状态为就绪
    timeout_ms = 0;                     // 重置超时
    dwt_tick = 0;                       // 重置启动时间
}

/**
 * @brief 移动动作的更新函数
 * @note 动作被抛出后，这玩意会以200Hz被循环调用直到动作完成
 */
bool MoveAct::OnUpdate() 
{
    // 这里实现移动到target_pos的逻辑
    if (movetype == AtPos)
    {
        return MoveAt();
    }
    else if (movetype == AlongPath)
    {
        return MoveAlong();
    }
		return true;
}



/**
 * @brief 基于直接移动到位置的方式
 * @details 被OnUpdate调用
 * 需要用到的信息流：底盘的控制、当前的位置、底盘的速度
 */
bool MoveAct::MoveAt()
{
    // 计算移动向量
    Vec2 move_vec = target_pos - System.global_position.ToVec2();

    // 检查是否到达目标位置, 如果是则返回完成
    if (move_vec.Length() < 0.05f)    // 5cm范围内视为到达
    {
        Chassis.Move(Vec2(0, 0));    // 停止移动
        return true;                 // 动作完成
    }

    // 计算移动速度
    float safe_velo = sqrt(2 * max_accel * move_vec.Length()); 
    float out_velo = 3.0f * move_vec.Length();
    // 最终的速度应该为三者中的最小值
    float final_velo = fminf(safe_velo, fminf(out_velo, max_velo));

    // 更新底盘速度（向量式更新，保证更新量不大于MaxAccel）
    Vec2 targ_speed_vec = move_vec.Norm() * final_velo;     // 计算新的目标速度
    Vec2 curr_speed_vec = Chassis.speed.ToVec2();           // 当前速度
    
    // 计算速度差
    Vec2 delta_speed_vec = targ_speed_vec - curr_speed_vec;
    float delta_speed_len = delta_speed_vec.Length();

    // 限制加速度
    if (delta_speed_len > (max_accel / 200.0f))   // 每次调用都是1 / 200s
    {
        delta_speed_vec = delta_speed_vec.Norm() * (max_accel / 200.0f);
    }
    Vec2 new_speed_vec = curr_speed_vec + delta_speed_vec;

    // 调用移动接口进行移动
    Chassis.Move(new_speed_vec);
		
    return false;
}



bool MoveAct::MoveAlong()
{
    return true;
}
