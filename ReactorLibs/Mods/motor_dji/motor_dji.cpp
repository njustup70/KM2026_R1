/**
 * @file motor_dji.cpp
 * @brief C620 / C610 电机控制类实现文件，我们战队不用GM6020（笑）
 * @author Huangney
 * @date 2025-8-21
 */
#include "motor_dji.hpp"
#include "RtosCpp.hpp"
#include "bsp_can.hpp"
#include "bsp_dwt.hpp"
#include "bsp_log.hpp"
#include "string.h"
#include "stdio.h"

float MotorDJI_SlopeLim(float& cur, float& cur_temp, float slope_value, float dt);

/**
 * @brief C620 / C610 电机额外初始化
 * @param Esc_Id 电机 ID（请查看C620 / C610说明书，注意其从 1 开始！）
 */
void MotorDJI::Init(CAN_HandleTypeDef *hcan, uint8_t motorESC_id)
{
	MotorDJI_Driver::Init(hcan, motorESC_id); 	// 初始化电机实体
}

/// @brief 设置速度
/// @param rpm 
void MotorDJI::SetSpeed(float rpm, float redu_ratio)
{
	if (mode != SpeedC) return; 	// 不是速度模式就不执行
	targ_speed = rpm;
}
/// @brief 设置位置
/// @param pos 
void MotorDJI::SetPos(float pos)
{
	if (mode != PosC) return; 	// 不是位置模式就不执行
	targ_position = pos;
}

/**
 * @brief 空档
 * @details 将电机设置为空档状态，强制要求电机不输出任何力矩
 * 类似于汽车的空挡
 * @warning 该函数会将控制模式切换为 None_Control, 使用后，需要重新设置控制模式
 */
void MotorDJI::Neutral()
{
	targ_current = 0;
	mode = NoneC;
}

int16_t MotorDJI::Control()
{
	if (enabled)
	{
		if (mode == NoneC)
		{
			targ_current = 0; 				// 目标电流清零
		}
		else
		{
			_CalcLoop();
		}
	}
	else
	{
		targ_current = 0; // 目标电流清零
	}
	
	return (int16_t)targ_current;
}

void MotorDJI::_CalcLoop()
{
	// 期望值 / 参考输入（国际单位制）
	float targ_value = 0;

	// 计算目标值的国际单位制
	if (mode == SpeedC)	targ_value = StdMath::RpmToRadS(targ_speed);
	else if (mode == PosC) targ_value = AngCodeToRad((uint16_t)targ_position);
	
	// 调用控制器，得到控制电流（单位：A）
	float cur_temp = ctrler.Calc(targ_value);

	// 转换为电流指令值（3508将 -20A ~ 20A 映射到了 -16384 ~ 16384）
	cur_temp = AmpToICode(cur_temp);

	// 限制爬坡率
	MotorDJI_SlopeLim(targ_current, cur_temp, _sloperate, 0.001f);

	// 电流限幅
	targ_current = StdMath::fclamp(targ_current, _current_limit);
}

float MotorDJI_SlopeLim(float& cur, float& cur_temp, float slope_value, float dt)
{
	// 限制爬坡率
	float delta_current = cur_temp - cur;
	float slope_rate = slope_value * dt;

	if (delta_current > slope_rate)
	{
		cur += slope_rate;
	}
	else if (delta_current < -slope_rate)
	{
		cur -= slope_rate;
	}
	else
	{
		cur = cur_temp;
	}
}

float MotorDJI::AngCodeToRad(uint16_t ang_code)
{
	return (float)ang_code / 8192.0f * (2.0f * 3.1415926f);
}

float MotorDJI::AmpToICode(uint16_t I_Ampere)
{
	return (float)I_Ampere / 16384.0f * 20.0f;
}