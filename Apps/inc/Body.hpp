#ifndef _BODY_HPP_
#define _BODY_HPP_

#include "stm32f4xx_hal.h"
#include "Action.hpp"
#include "std_math.hpp"
#include "motor_dji.hpp"

#define FWHEEL_DIAMETER 0.0598f // 摩擦轮直径

class PickRod
{
private:
    bool EnVeloLim;           // 是否启用速度限制
    float VpitchLim = 400.0f; // 最大杆速度，单位m/s
    float VwheelLim = 200.0f; // 最大摩擦轮速度，单位m/s
    float motor_spd[3] = {0}; // 三个电机的目标速度，单位m/s
    float Servo_Angle = 0.0f; // 舵机的目标角度，单位度
    float Ptarget = 0.0f;     // pitch目标值
    float Wtarget = 0.0f;     // wheel目标值

public:
    MotorDji motors[3];
    PickRod() {};
    ~PickRod() {};
    void PitchMove();
    void WheelMove();
    void MoveProduce(float T_speed, int32_t T_place, uint8_t motor_id);
    // 构建初始化
    void Build();
    // 月抛函数
    void Pitchup();
    void Pitchdown();

    void AllStop();
    void Wheelup();
    void Wheeldown();
    void Wheelstop();
    void PitchStop();
    bool PickUp();
    bool PickDown();
    void PickReset();

    void Update();
    void WheelGo(float Goplace);
};
extern MotorDji motors[3];

extern PickRod pickrod;

#endif