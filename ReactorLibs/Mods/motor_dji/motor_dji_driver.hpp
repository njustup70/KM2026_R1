/**
 * @file motor_dji_driver.hpp
 * @author https://github.com/Huangney
 * @date 2026-2-14
 */
#pragma once
#include "bsp_can.hpp"

// 电机反馈信息结构体变量定义
typedef struct
{
	int16_t speed_rpm;
	int16_t real_current;
	uint8_t temprature;			
	uint16_t angle;		 		// 范围为[0, 8191]
	uint16_t last_angle;
	uint16_t offset_angle;
	int32_t round_cnt;
	int32_t total_angle;
	uint32_t msg_cnt;
} moto_measure_t;

/// @brief 电机类：C620 / C610
class MotorDJI_Driver
{
protected:
	friend void _MotorDJI_DecodeMeasure(MotorDJI_Driver* motor_p, uint8_t *Data);
    friend void _MotorDJI_RecvQualityWatch(MotorDJI_Driver* motor_p);

	// 是否启用电机控制，若为否，发送0电流指令或不发送指令
	bool _enabled = false;
	uint8_t _motor_id = 0;			// 电机ID（1 ~ 8）

	/// @brief 电机是否在线（私有）
	bool _online_priv = false;
	int _online_cnt = 0;
	
    struct
    {
        /// @brief 前10次接收数据的平均时间间隔(最小单位：0.1ms，uint16精度)，用于判断电机在线质量
        uint32_t _recv_tick = 0;
        uint32_t _recv_sum_interval = 0;
        uint16_t _recv_interval[10] = {0};
        uint8_t _recv_last_index = 0;
        bool _recv_looped = false;
        /// @brief 根据前十次间隔计算的到的频率 (单位Hz)
        float _recv_freq = 0.0f;
    }_recv_quality;

	/// @brief 获取电机所在的CAN段，用于发送
	private: uint8_t _GetCanSeg(uint8_t motor_id);

    protected: virtual int16_t Control(){};
public:
	MotorDJI_Driver(){};

	/** 	  方法		**/
	void Init(CAN_HandleTypeDef *hcan, uint8_t motorESC_id);

	void SetCurrent(float current);

	void Disable();

	void Enable();

	/// @brief 解码后回调，允许子类在数据更新后立即执行逻辑（如ADRC观测）
	virtual void PostDecodeCallback() {}

	/** 	静态方法	**/
	static void ControlAllMotors();
	
	/**----------- 	信息流变量	----------**/
	/// @brief 电机反馈信息结构体
	moto_measure_t measure;	

	/// @brief 电机是否在线（公开但只读）
	const bool& online = _online_priv;
	const bool& enabled = _enabled;

	/// @brief 电机的CAN实例
	BspCan_Instance bspcan_inst;

	/// @brief 当前电机所在的CAN段，用于发送电流 注：0-3分别对应CAN1的1-4号电机，5-8号电机，CAN2的1-4号电机，5-8号电机
	uint8_t at_can_seg = 0;
	
	/** 	控制用变量	**/
	float targ_current = 0;				// 目标电流
};

namespace MotorDJIConst
{
	const static float redu_M3508 = 19.0f;	// M3508电机的减速比
	const static float redu_M2006 = 36.0f;	// M2006电机的减速比
}

// typedef MotorDJI_Driver MotorC610;		// 本库对C620和C610的支持是通用的
// typedef MotorDJI_Driver MotorC620;		



// #ifdef __cplusplus
// }
// #endif
