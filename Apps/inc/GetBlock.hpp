#ifndef _GETBLOCK_HPP_
#define _GETBLOCK_HPP_

#include "stm32f4xx_hal.h"
#include "Action.hpp"
#include "std_math.hpp"
#include "motor_dji.hpp"


extern "C" {
    #include "bsp_gpio.h" // 确保驱动函数以 C 风格链接
}
//动作高度
// typedef enum 
// {
//     H_40,
//     H_20 ,
//     H_X20    //-20cm
// } Height;

const static float H_40[2]={3.1415926f *150/180.0f,3.5};
const static float H_20 = 3.1415926f *30/180.0f;
const static float H_X20 = -3.1415926f *30/180.0f;

//动作先后指令
typedef enum    
{
    act_UP,
    act_IN,
    act_RESET
} G_action;

//执行的动作
typedef enum 
{
    Gain_block,  //取
    Output_block //出
}TASK;
class GetBlock  :public Application
{
    SINGLETON(GetBlock) : Application("getblock") {};
    APPLICATION_OVERRIDE

    private:
    BspGpio_Instance huang1;
    MotorDM motor1;           // 杆驱动舵机
    MotorDJI motors[4];       // 传送夹爪电机
    float target_pos[3]={0.0f};      // 杆驱动舵机目标位置
    float target_spd[5]={0.0f};      // 传递速度给电机
    G_action Gstate;          // 夹爪当前动作状态
    TASK task_now;          //当前任务
    uint8_t Block_num=0;      // 物块数量
    float H[2]={0.0f};            // 目标高度
    float current_pos;      // 杆驱动舵机当前位置
    bool enabled = false;    // 抓杆使能标志
    bool Blockup = false;    // 把物块举起标志
    bool Enmotor = true;     //所有电机的开关
    
    bool GetinBlock();
    bool Outputblock();

    public:
    
    void UploadPos();
    void G_rise();
    void Block_in();
    void Block_stop();
    void G_fall();
    void G_in();
    void G_out();
    void Block_out1();
    //所有电机关闭的函数
    void Enmotors_off();    
    void Inblock(const float *h);
    void Outblock(const float *h);

};
#endif