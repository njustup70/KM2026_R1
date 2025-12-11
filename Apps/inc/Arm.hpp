#ifndef _ARM_HPP_
#define _ARM_HPP_

#pragma once
#include "RobotSystem.hpp"
#include "stm32f4xx_hal.h"
#include "motor_dm.hpp"
#include "bsp_gpio.h"
#include "algorithm" 
// 机械臂数量
#define ARM_COUNT 1

/**
 * @brief 吸块机械臂类
 * @note  作为单例，实际是包含三个（根据机械结构确定）机械臂的ArmGroup。（TODO）,but由于只有注册单例的机制，所以不打算注册3个机械臂实例了，还是先注册一个实例吧
 * 
 */
class ArmType : public Application
{
    SINGLETON(ArmType) : Application{"VacuumArm"} {};
    APPLICATION_OVERRIDE
    
    private:
    //机械臂的结构体包含base和end两个关节电机
    struct Armunit
    {
        MotorDM joint_base;  // 控制基座端
        MotorDM joint_end;   // 控制吸块末端

        /*< 电机的实际位置  >*/
        float basejoint_current_angle;  // 基座端关节实际角度，单位rad
        float endjoint_current_angle;   // 吸块端关节实际角度，单位rad
         /*< 电机的目标位置  >*/
        float basejoint_target_angle;   // 基座端关节目标角度，单位rad
        float endjoint_target_angle;    // 吸块端关节目标角度，单位rad

        bool end_arrived ;   // 吸块端关节到达目标标志位
        bool base_arrived ;  // 基座端关节到达目标标
    };
    Armunit arm[ARM_COUNT];  // 机械臂数组

        /*<     控制状态相关标志位   >*/
        bool enabled = false;    // 机械臂组使能标志(所有机械臂全使能)
        bool _is_taking_block = false;
        bool _is_holding_block = false;
        bool _is_releasing_block = false;
        
        
   
    public:
        /**       采用固定角度，采用固定角度，取块后将物块存于某个位置         **/
        void SetArmTogether_front(uint8_t motor_number);
        void SetArmTogether_middle(uint8_t motor_number);
        void SetArmTogether_rear(uint8_t motor_number);

        void SetStartPosition(uint8_t motor_number); //机械臂处于准备吸块的位置
        /**       采用灵活角度，可控制单个机械臂与某个位置        **/
        void SetSingleArmAngle(uint8_t motor_number, float basejoint_angle, float endjoint_angle);

        ///@brief 采用灵活可调角度，机械臂的解算，未来?可以和遥控器联动拨杆连续调节功能?（TODO） 
        void ReleaseBlock(uint8_t motor_number);
        void ArmAngelToMotorangel();


    private:
        BspGpio_Instance vacuum_pump_pin; 
        bool _is_vacuum_enable = false; //真空泵使能没

        /*<真空泵吸块方法>*/
    public:
        void ActivateVacuum();
    
};

///@brief 限制函数.限制某变量在最大值和最小值之间-
template<typename T>
T clamp(T val, T min_val, T max_val)
{
return (val < min_val) ? min_val : (val > max_val) ? max_val : val;
}


































#endif