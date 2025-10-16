/**
 * @file motor_dji.cpp
 * @brief C620 / C610 电机控制类实现文件，我们战队不用GM6020（笑）
 * @author Huangney
 * @date 2025-8-21
 */
#include "motor_dji.hpp"
#include "bsp_can.h"

#define CAN_OFFSET (hcan == &hcan1 ? 0 : 8)
void MotorDji_RxCallback(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, CAN_HandleTypeDef *hcan);
static void MotorDji_SendCurrent(CAN_HandleTypeDef *hcan, int16_t motor_0, int16_t motor_1, int16_t motor_2, int16_t motor_3, bool more = false);


/// @brief 存储已经注册的电机实例的指针，便于回调函数查表处理
static MotorDji* MotorPointList[1 + 16] = {nullptr}; 	 // 电机ID从1开始，所以0号不使用
// 顺序记录已经注册的电机ID，便于遍历所有电机（比如3号和6号注册了，但是位置很散，查这个表有助于提升遍历效率）
uint8_t MotorIndexList[16] = {0}, MotorIndexCount = 0; 

/**
 * @brief C620 / C610 电机额外初始化
 * @param Esc_Id 电机 ID（请查看C620 / C610说明书，注意其从 1 开始！）
 */
void MotorDji::Init(CAN_HandleTypeDef *hcan, uint8_t motorESC_id, MotorDJIMode djimode, bool fastInit)
{
	
	// 首先，初始化其 速度环Pid类
	if (fastInit)	speed_pid.FastInit(3, 24.0, 0.0, 0.16, 5000, false);		// 增量速度环
	// if (fastInit)	speed_pid.Init(10.8, 0.0, 0.0, 0.05, 0.0, 0, 0.0, 0.0, 0.8, false, true, true);		// 位置速度环
	else 			speed_pid.FastInit(0.0, 0.0, 0.0, 0.0, 5000, false);
	
	// 接着是 位置环Pid类	
	// if (fastInit)	position_pid.FastInit(0.10, 1.00, 0.0, 0.0, 10000, false);		// 增量位置环
	if (fastInit)	position_pid.Init(0.108, 0.0, 0.0, 0.05, 0.0, 0, 0.0, 4000.0, 0.8, false, true, true);		// 位置位置环
	else 			position_pid.Init(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 4000.0, 0.8, false, false, false);		// 位置位置环
	
	if (fastInit)	mode = djimode;
	else			mode = djimode;
	

	/// @brief 根据电机 ID 和 CAN路线，将其存储到全局的电机实例列表中，同时顺序记录其ID
	/// 一般一根CAN总线上最多有8个电机，所以CAN1分配到1-8号电机，CAN2分配到9-16号电机
	if (motorESC_id <= 8 && motorESC_id >= 1)
	{
		MotorPointList[motorESC_id + CAN_OFFSET] = this; 					// 根据CAN总线分配到0-7或8-15
		MotorIndexList[MotorIndexCount++] = motorESC_id + CAN_OFFSET; 		// 记录电机ID
		at_can_seg = GetCanSeg(motorESC_id + CAN_OFFSET); 					// 记录电机所在的CAN段
	}

	// 初始化（即注册）该电机的CAN实例
	BspCan_InstRegist(&bspcan_inst, hcan, 0x200 + motorESC_id, 0x200 + motorESC_id, 0, 0, MotorDji_RxCallback);
}


/// @brief 更改电机控制模式
void MotorDji::SwitchMode(MotorDJIMode new_mode)
{
	// 为确保安全，切换模式时，速度清零，目标位置设置为当前位置
	targ_speed = 0;
	targ_position = measure.total_angle;

	mode = new_mode;
}

/// @brief 设置速度
/// @param rpm 
void MotorDji::SetSpeed(float rpm)
{
	if (mode != Speed_Control) return; 	// 不是速度模式就不执行
	targ_speed = rpm;
}

/// @brief 设置位置
/// @param pos 
void MotorDji::SetPos(float pos)
{
	if (mode != Pos_Control) return; 	// 不是位置模式就不执行
	targ_position = pos;
}


/// @brief 关闭电机控制
void MotorDji::Disable()
{
	enabled = false; 		// 禁用电机控制
}
/// @brief 开启电机控制
void MotorDji::Enable()
{
	enabled = true; 		// 启用电机控制
}
/// @brief 检查电机控制是否启用
bool MotorDji::IsEnabled()
{
	return enabled; 		// 返回电机控制是否启用
}


/// @brief 判断电机所在的CAN段，用于发送
/// @param motor_id 
/// @return 所在的CAN段
uint8_t MotorDji::GetCanSeg(uint8_t motor_id)
{
	if (motor_id >=1 && motor_id <= 4) return 0; // CAN1的1-4号电机
	else if (motor_id >= 5 && motor_id <= 8) return 1; // CAN1的5-8号电机
	else if (motor_id >= 9 && motor_id <= 12) return 2;
	else if (motor_id >= 13 && motor_id <= 16) return 3; // CAN2的5-8号电机
	return 0; // 默认返回0
}

/**
 * @brief 控制所有已注册电机
 * @details 该函数会遍历所有已注册的电机实例，并调用其Control方法，最后根据返回的电流值发送CAN指令
 */
void MotorDji::ControlAllMotors()
{
	// 分四段：CAN1的1-4号电机，CAN1的5-8号电机，CAN2的1-4号电机，CAN2的5-8号电机
	bool send_can_seg[4] = {false, false, false, false};
	 // 存储所有电机的目标电流 
	int16_t motor_currents[16] = {0};

	// 遍历所有注册的电机实例，调用其Control方法，并获得本次发送的电流值
	for (uint8_t i = 0; i < MotorIndexCount; i++)
	{
		uint8_t motor_id = MotorIndexList[i];
		if (MotorPointList[motor_id] != nullptr)
		{
			int16_t targ_motor_current = MotorPointList[motor_id]->Control();
			
			if (targ_motor_current != 0)		//	如果有控制需求，就激活对应的CAN段
			{
				send_can_seg[MotorPointList[motor_id]->at_can_seg] = true; // 激活对应的CAN段
				motor_currents[motor_id - 1] = targ_motor_current; // 记录电流值，注意motor_id从1开始，而这里的数组是从0开始的，所以需要减1
			}
		}
	}

	// 根据send_can_seg数组，判断是否需要发送CAN指令
	if (send_can_seg[0]) 
	{
		// 发送CAN1的1-4号电机的电流指令	
		MotorDji_SendCurrent(&hcan1, motor_currents[0], motor_currents[1], motor_currents[2], motor_currents[3]);
	}

	if (send_can_seg[1])
	{
		// 发送CAN1的5-8号电机的电流指令
		MotorDji_SendCurrent(&hcan1, motor_currents[4], motor_currents[5], motor_currents[6], motor_currents[7], true);
	}

	if (send_can_seg[2])
	{
		// 发送CAN2的1-4号电机的电流指令
		MotorDji_SendCurrent(&hcan2, motor_currents[8], motor_currents[9], motor_currents[10], motor_currents[11]);
	}

	if (send_can_seg[3])
	{
		// 发送CAN2的5-8号电机的电流指令
		MotorDji_SendCurrent(&hcan2, motor_currents[12], motor_currents[13], motor_currents[14], motor_currents[15], true);
	}
}


/**
 * @brief 控制本电机
 * @warning 不含发送指令！！
 */
int16_t MotorDji::Control()
{
	if (enabled)
	{
		if (mode == None_Control)
		{
			targ_current = 0; 			// 目标电流清零
		}
		else if (mode == Speed_Control)
		{
			MotorDji_SpeedLoop();		// 速度环控制 得到电流
		}
		else if (mode == Pos_Control)
		{
			MotorDji_PosLoop();		// 位置环控制 得到速度
			MotorDji_SpeedLoop();	// 速度环控制 得到电流
		}
	}
	else
	{
		targ_current = 0; // 目标电流清零
	}
	
	return (int16_t)targ_current;
}


/**
 * @name C620 / C610 速度环控制
 * @details 计算RPM对应的控制电流
 */
void MotorDji::MotorDji_SpeedLoop()
{
	// 获取测量结构体
	moto_measure_t *ptr = &measure;

	// 计算目标的 速度PID输出（输出为电流）
	float targ_current_temp = speed_pid.Calc(targ_speed, ptr->speed_rpm, current_limit);

	// 计算最终电流数值
	float targ_current_temp_f = (1.0f - current_LPS_rate) * targ_current + current_LPS_rate * targ_current_temp;

	// 限制爬坡率
	float delta_current = targ_current_temp_f - targ_current;
	if (delta_current > sloperate)
	{
		targ_current += sloperate;
	}
	else if (delta_current < -sloperate)
	{
		targ_current -= sloperate;
	}
	else
	{
		targ_current = targ_current_temp_f;
	}

	// 最终电流限幅
	Lim_ABS(targ_current, current_limit)
}

/**
 * @name C620 / C610 位置环控制
 * @details 计算位置对应的控制电流
 */
void MotorDji::MotorDji_PosLoop()
{
	// 获取测量结构体
	moto_measure_t *ptr = &measure;
	// 计算目标的 位置PID输出（输出为速度）	
	targ_speed = position_pid.Calc(targ_position, ptr->total_angle, speed_limit);
	
	// 最终速度限幅
	Lim_ABS(targ_speed, speed_limit)
}


/**
 * @description: 获取电机反馈信息
 * @param {moto_measure_t} *ptr电机反馈信息结构体指针
 * @param {uint8_t} *Data接收到的数据
 * @return {*}无
 */
static void MotorDji_DecodeMeasure(MotorDji* motor_p, uint8_t *Data)
{
	// 获取测量信息的指针和其他参数
	moto_measure_t *ptr = &motor_p->measure;
	float RPM_LPF_rate = motor_p->ReadRPM_LPF_rate;
	float Current_LPF_rate = motor_p->ReadCurrent_LPF_rate;

	// 更新上一次的角度
	ptr->last_angle = ptr->angle;

	// 更新电机转子位置
	ptr->angle = (uint16_t)(Data[0] << 8 | Data[1]);

	// 更新电机转速（带低通滤波）
	float speed_read = (int16_t)(Data[2] << 8 | Data[3]);
	ptr->speed_rpm = ((1.0f - RPM_LPF_rate) * ptr->speed_rpm) + (RPM_LPF_rate * speed_read);

	// 更新电机电流（带低通滤波）
	float current_read = (int16_t)(Data[4] << 8 | Data[5]) / -5;
	ptr->real_current = ((1.0f - Current_LPF_rate) * ptr->real_current) + (Current_LPF_rate * current_read);

	// 更新电机温度
	ptr->temprature = Data[6];


	// 更新圈数统计
	if (ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt--;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}


/**
 * @description: 电机上电角度=0， 之后用这个函数更新3508电机的相对开机后（为0）的相对角度。
 * @param {moto_measure_t} *ptr电机结构体指针
 * @param {uint8_t} *Data接收到的数据
 * @return {*}无
 */
static void MotorDji_DecodeInitOffset(MotorDji* motor_p, uint8_t *Data)
{
	moto_measure_t *ptr = &motor_p->measure;
	ptr->angle = (uint16_t)(Data[0] << 8 | Data[1]);
	ptr->offset_angle = ptr->angle;
}



/**
 * @description: 发送电机电流控制
 * @return {*}无
 */
static void MotorDji_SendCurrent(CAN_HandleTypeDef *hcan, int16_t motor_0, int16_t motor_1, int16_t motor_2, int16_t motor_3, bool more)
{
	uint8_t motor_current_data[8] = {0};
	motor_current_data[0] = motor_0 >> 8;
	motor_current_data[1] = motor_0;
	motor_current_data[2] = motor_1 >> 8;
	motor_current_data[3] = motor_1;
	motor_current_data[4] = motor_2 >> 8;
	motor_current_data[5] = motor_2;
	motor_current_data[6] = motor_3 >> 8;
	motor_current_data[7] = motor_3;

	// 根据是否需要发送到ID大于4的电机，选择不同的ID
	if (more)
	{
		BspCan_TxConfig txconf = BspCan_GetTxConfig(hcan, 0x1ff, BSPCAN_STD, BSPCAN_DATA, 8, 50);
		BspCan_Transmit(txconf, motor_current_data);
	}
	else
	{
		BspCan_TxConfig txconf = BspCan_GetTxConfig(hcan, 0x200, BSPCAN_STD, BSPCAN_DATA, 8, 50);
		BspCan_Transmit(txconf, motor_current_data);
	}
}


/// @brief 接收回调函数
/// @param RxHeader 
/// @param RxData 
void MotorDji_RxCallback(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, CAN_HandleTypeDef *hcan)
{
	// 注意 motor_id 和 motorESC_id 的区别
	uint8_t motor_id = ((RxHeader->StdId) & 0xff) - 0x200 + CAN_OFFSET; // 获取电机ID（带CAN偏置，以区分 CAN1和CAN2的电机ID）

	// 利用全局的电机实例列表 来获取对应的电机实例
	if (motor_id > 16 || MotorPointList[motor_id] == nullptr) return; 			// 如果电机ID不合法或未注册，直接返回
	MotorDji &motor = *MotorPointList[motor_id]; 								// 获取对应的电机实例

	// 更新电机反馈信息
	motor.measure.msg_cnt++ <= 50 ? MotorDji_DecodeInitOffset(&motor, RxData) : MotorDji_DecodeMeasure(&motor, RxData);
}
