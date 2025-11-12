#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "pids.hpp"

#define ABS(x) ((x > 0) ? (x) : (-x))
#define Lim_ABS(x, y) \
if (x > y) 			\
{		   			\
	x = y; 			\
}					\
else if (x < -y) 	\
{					\
	x = -y;			\
}



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

typedef enum
{
	Pos_Control,
	Speed_Control,
	None_Control
}MotorDJIMode;


/// @brief 电机类：C620 / C610
class MotorDji
{
private:
	bool enabled = false; 				// 是否启用电机控制，若为否，发送0电流指令或不发送指令
	uint16_t current_limit = 8000;		// 电流限幅
	uint16_t speed_limit = 20000;		// 速度限幅
	uint16_t sloperate = 500;			// 电流爬坡率

	/** 	  方法		**/
	void MotorDji_SpeedLoop();
	void MotorDji_PosLoop();
	uint8_t GetCanSeg(uint8_t motor_id);
	
public:
	MotorDji(){};
	~MotorDji(){};

	/** 	  方法		**/
	void Init(CAN_HandleTypeDef *hcan, uint8_t motorESC_id, MotorDJIMode djimode, bool fastInit = true);
	void SwitchMode(MotorDJIMode new_mode);
	void SetSpeed(float rpm, float redu_ratio = 19.0f);		// 3508的默认减速比（用2006的时候记得改！）
	void SetPos(float pos);
	void Neutral();
	void Disable();
	void Enable();
	bool IsEnabled();		// 返回电机控制是否启用
	int16_t Control();

	/** 	静态方法	**/
	static void ControlAllMotors();

	/** 	信息流变量	**/
	moto_measure_t measure;	// 电机反馈信息结构体
	float ReadRPM_LPF_rate = 1.0f; // 电机转速低通滤波系数
	float ReadCurrent_LPF_rate = 0.5f; // 电机转速低通滤波系数
	BspCan_Instance bspcan_inst;	// 电机的CAN实例
	float current_LPS_rate = 1.0f; 		// 电流变化低通滤波系数（一般不用）
	uint8_t at_can_seg = 0;				// 当前电机所在的CAN段，用于发送电流 注：0-3分别对应CAN1的1-4号电机，5-8号电机，CAN2的1-4号电机，5-8号电机
	

	/** 	控制用变量	**/
	
	Pids speed_pid; 				// 速度环PID
	Pids position_pid; 				// 位置环PID
	float targ_position = 0;			// 目标位置
	float targ_speed = 0;		    	// 目标速度
	float targ_current = 0;			// 目标电流

	/**		属性类变量	**/
	MotorDJIMode mode = None_Control;	// 电机当前控制模式
};


typedef MotorDji MotorC610;		// 本库对C620和C610的支持是通用的
typedef MotorDji MotorC620;		


void get_total_angle(moto_measure_t *p);



#ifdef __cplusplus
}
#endif
#endif 
