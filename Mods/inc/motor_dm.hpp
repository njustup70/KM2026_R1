#ifndef MOTOR_DM_HPP
#define MOTOR_DM_HPP

#include "stm32f4xx_hal.h"
#include "bsp_can.h"

/**
 * @brief 达妙电机类
 * 
 */
class MotorDM
{   
    private:
        // 电机控制模式
        enum ControlMode 
        {
            MODE_MIT,        // MIT模式
            MODE_POSANDVEL,  // 位置速度模式
            MODE_VEL,        // 速度模式
        };
        uint32_t Get0ffsetid() const;

    public:
        MotorDM() {};
        ~MotorDM() {};

        ControlMode mode = MODE_POSANDVEL; // 默认位置速度模式
        uint8_t motor_id = 0;              // 电机ID
        uint8_t master_id = 0;             // 主控ID
        
        float posi_max = 0;         // 位置上限，单位：弧度
        float posi_min = 0;         // 位置下限，单位：弧度

        uint8_t recv_buf[8] = {0}; 
        uint32_t error_recv = 0;      // 接收错误计数
        bool enabled = false;            // 是否使能
        BspCan_Instance motor_bspcan_inst;	// 电机的CAN实例

        float target_posi = 0;          // 位置，单位：弧度
        float target_spd = 0;           // 速度，单位：弧度/秒
        float target_torque = 0;        // 扭矩，单位：Nm

        float current_posi = 0;         // 实际位置，单位：弧度
        float current_spd = 0;          // 实际速度，单位：弧度/秒
        float current_torque = 0;       // 实际扭矩，单位：Nm

        // 初始化
        void Init(CAN_HandleTypeDef* hcan, uint8_t motor_id, uint8_t master_id);
        // 使能
        void Enable();
        // 失能
        void Disable();
        // 设置位置
        void SetPosi(float pos);

         /*    位置速度模式的电机控制方法      */
        // 设置位置和速度
        void SetPosVel(float pos, float spd);
        // 设置位置限制
        void SetPosiLim(float posi_max, float posi_min);

        // 获取当前位置和速度
        void GetPosiSpd(float &posi, float &spd);
        // 设置电机绝对0位
        void SetZeroPosi();
        // 保存电机参数（在设置完电机的零位后需要调用，进行保存）
        void SaveParam();


};


/******************    库内部的STATIC函数声明    *****************************/
static float DM_LinearRef(uint16_t code, float max);
static void DM_GetPosiSpdBuf(uint8_t buf[], float posi, float speed);
static void DM_GetPosiSpdBufLim(uint8_t buf[], float posi, float speed, float lim_H, float lim_L);
static void DM_DecodeMsg(MotorDM* motor, uint8_t buf[]);



#endif