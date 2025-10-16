#include "Robot_1_Arm.hpp"



void Robot1Arm::Init(CAN_HandleTypeDef* DMhcan, CAN_HandleTypeDef* DJIhcan, uint8_t DM_can_id, uint8_t DM_master_id, uint8_t C610_esc_id)
{
    Base_motor.Init(DMhcan, DM_can_id, DM_master_id);
    Secd_motor.Init(DJIhcan, C610_esc_id, Pos_Control);
}

void Robot1Arm::SetBaseLim(float pos_min, float pos_max)
{
    Base_posi_min = pos_min;
    Base_posi_max = pos_max;
}

void Robot1Arm::Enable()
{
    Base_motor.Enable();
    Secd_motor.Disable();
}

void Robot1Arm::SetBasePos(float pos)
{
    if(pos < Base_posi_min) pos = Base_posi_min;
    if(pos > Base_posi_max) pos = Base_posi_max;

    Base_motor.SetPosi(pos, 0.5f);
}

/**
 * @brief 设置第二关节位置
 * @param pos：单位是弧度，会被转换为2006的pulse
 */
void Robot1Arm::SetSecdPos(float pos)
{
    // 294,912 pulse 对应 2PI
    int32_t pulse = (int32_t)(pos / (2.0f * 3.1415926f) * 294912.0f);
    Secd_motor.SetPos(pulse);
}