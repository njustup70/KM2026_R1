#ifndef _ARM_HPP_
#define _ARM_HPP_

#pragma once
#include "System.hpp"
#include "stm32f4xx_hal.h"
#include "motor_dm.hpp"
#include "bsp_gpio.h"
#include "algorithm" 
//确保数据是紧凑的10字节
typedef struct __packed {
    uint8_t  sof;//帧头
    uint8_t  id;//帧id
    float    left_mm;
    float    up_mm;
} BoardData_t;

class JawType : public Application
{
    SINGLETON(JawType) : Application{"Jaw"} {};
    APPLICATION_OVERRIDE
    
    private:
 /**取矛头结构含有5个电机,0号电机（id为1）控制上层夹爪的旋转，1号电机（id为2）控制底座的旋转，2号电机（id为3）控制底座前后运动，3号电机（id为4）微调上层夹爪上下位置，4号电机（id为5）微调上层夹爪前后位置**/
    float speed[5]={500,1000,500,1500,1500}; //五个电机的速度限幅
    /**取矛头结构目标位置**/
    float jaw_rotate_target_position = 72570; //上层夹爪旋转的目标位置
    float base_rotate_target_position = 374251; //下层底座旋转的目标位置
    float base_movre_target_position = 111237; //底座前后运动的目标位置
    float jaw_updown_target_position = 0; //上层夹爪上下微调目标位置(微调)
    float jaw_forwardback_target_position = 0; //上层夹爪前后微调目标位置（微调）
    /**取矛头结构实际位置 **/
    float jaw_rotate_current_position = 0; //上层夹爪旋转的实际位置
    float base_rotate_current_position = 0; //下层底座旋转的实际位置
    float base_move_current_position = 0; //底座前后运动的实际位置  
    float jaw_updown_current_position = 0; //上层夹爪上下微调实际位置
    float jaw_forwardback_current_position = 0; //上层夹爪前后微调实际位置

    /**取矛头动作完成标志**/
    bool jaw_rotate_arrived = false;
    bool base_rotate_arrived = false;
    bool base_move_arrived = false;
    bool jaw_updown_arrived = false;
    bool jaw_forwardback_arrived = false;
    bool jaw_action = false;
    /**参数属性**/
    bool enabled = true; //取矛头结构使能标志
   
    public:
/**取矛头结构含有5个电机,0号电机（id为1）控制上层夹爪的旋转，1号电机（id为2）控制底座的旋转，2号电机（id为3）控制底座前后运动，3号电机（id为4）微调上层夹爪上下位置，4号电机（id为5）微调上层夹爪前后位置**/
    MotorDJI jaw_djmotor[5];
        /**       直接接口         **/
        void Enable();
        void Disable(); 
        /**       取矛头结构控制函数         **/
        void ControlJawRotate(float position); //控制上层夹爪旋转
        void ControlBaseRotate(float position); //控制底座旋转
        void ControlBaseMove(float position); //控制底座前后运动
        void ControlJawUpDown(float position); //控制上层夹爪上下微调
        void ControlJawForwardBack(float position); //控制上层夹爪前后微调
        /**       取矛头结构动作函数         **/
        void JawAction();
        /**       结构位置限幅设置函数         **/
        void JawForwardBackLimit(float min_position,float max_position); //设置上层夹爪前后微调限幅
        void JawUpDownLimit(float min_position,float max_position); //设置上层夹爪上下微调限幅
        void BaseMoveLimit(float min_position,float max_position); //设置底座前后运动限幅
        /**       运动速度限幅设置函数         **/
        void JawRotateSpeed(float Speed); //设置上层夹爪旋转速度限幅
        void BaseRotateSpeed(float Speed); //设置底座旋转速度限幅
        void BaseMoveSpeed(float Speed); //设置底座前后运动速度限幅
        void JawUpDownSpeed(float Speed); //设置上层夹爪上下微调速度限幅
        void JawForwardBackSpeed(float Speed); //设置上层夹爪前后微调速度限幅

        void RxDataProcess(uint8_t* rx_data); //处理接收到的数据

};




































#endif