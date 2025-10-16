#ifndef ROBOT_1_ARM_HPP
#define ROBOT_1_ARM_HPP
#include "motor_dm.hpp"
#include "motor_dji.hpp"

class Robot1Arm
{
    public:
    Robot1Arm(){};
    ~Robot1Arm(){};

    MotorDM Base_motor;
    MotorC610 Secd_motor;

    float Base_posi_min = -0.5f;
    float Base_posi_max =  0.1f;
    
    void Init(CAN_HandleTypeDef* DMhcan, CAN_HandleTypeDef* DJIhcan, uint8_t DM_can_id, uint8_t DM_master_id, uint8_t C610_esc_id);

    void SetBaseLim(float pos_min, float pos_max);

    void Enable();

    void SetBasePos(float pos);
    
    void SetSecdPos(float pos);
};


#endif

