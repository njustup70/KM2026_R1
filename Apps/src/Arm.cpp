#include "Arm.hpp"

// 两个关节的臂长（单位：米）
#define ARM_LENGTH_BASE  0.40    
#define ARM_LENGTH_END  0.20 
//电机的角度限制
#define BASE_ANGLE_MAX  2.618f   //基座端关节最大角度
#define BASE_ANGLE_MIN  -2.618f     //基座端关节最小角度
#define END_ANGLE_MAX  2.094f    //吸块端关节最大角度
#define END_ANGLE_MIN  -2.094f     //吸块端关节最小角度
// 电机减速比
#define BASE_REDUCTION_RATIO 1.0f  // base电机：自动换算为输出轴位置
#define END_REDUCTION_RATIO 25.0f  // end电机：1:25减速比，需手动换算
// 位置到达阈值（输出轴，rad）
#define POSITION_THRESHOLD 0.02f 
// 电机速度参数（输出轴，rad/s）
#define BASE_MAX_SPEED 1.0f      // base电机最大速度（输出轴）
#define END_MAX_SPEED 1.5f       // end电机最大速度（输出轴）
const float BASE_SPEED[ARM_COUNT] = {1.0F}; //3个底座电机的速度设置
const float END_SPEED[ARM_COUNT] = {1.5F};//3个末端电机的速度设置
int flag = 0;//用于错开两个电机的控制
/**
 * @brief 初始化机械臂也就是初始化电机，机器人上电就初始化机械臂到初始状态
 * 
 */
void ArmType::Start()
{
//初始化并使能机械臂上的两个电机
//目前是1个机械臂，可自行调整数量
const uint8_t BASE_MOTOR_CANIDS[ARM_COUNT] = {0x01}; //底座电机的can_id
const uint8_t BASE_MOTOR_MASTERIDS[ARM_COUNT] = {0x00}; //底座电机的master_id
const uint8_t END_MOTOR_CANIDS[ARM_COUNT] = {0x20};//末端电机的can_id
const uint8_t END_MOTOR_MASTERIDS[ARM_COUNT] = {0x40};//末端电机的master_id
    for (int i = 0; i < ARM_COUNT; i++)
    {
        //初始化base电机
        arm[i].joint_base.Init(&hcan1, BASE_MOTOR_CANIDS[i], BASE_MOTOR_MASTERIDS[i]);        //这个传电机id感觉要改不要在这里写死,而且目前还只有一个机械臂
        arm[i].joint_base.Enable();
        arm[i].joint_base.SetPosiLim(BASE_ANGLE_MAX * BASE_REDUCTION_RATIO, BASE_ANGLE_MIN * BASE_REDUCTION_RATIO);//设置base电机的输出轴的位置限制
        //初始化end电机
        arm[i].joint_end.Init(&hcan1, END_MOTOR_CANIDS[i], END_MOTOR_MASTERIDS[i]);
        arm[i].joint_end.Enable();
        arm[i].joint_end.SetPosiLim(END_ANGLE_MAX * END_REDUCTION_RATIO, END_ANGLE_MIN * END_REDUCTION_RATIO);//设置end电机的输出轴的位置限制
        //初始化目标位置和实际位置（输出轴）
        arm[i].basejoint_target_angle = 0.0f;
        arm[i].endjoint_target_angle = 0.0f;
        arm[i].basejoint_current_angle = 0.0f;
        arm[i].endjoint_current_angle = 0.0f;
        arm[i].base_arrived = false;
        arm[i].end_arrived = false;
    }
    enabled = true;
}

void ArmType::Update()
{
    // 遥控器逻辑

    //
    if(!enabled) return;
    flag++;
    for(int i=0; i < ARM_COUNT; i++)
    {
        Armunit& Arm = arm[i];
       /* -------------------- Base电机控制（自动换算为输出轴） -------------------- */
    if (flag%10<5)
    {
		 Arm.basejoint_current_angle = Arm.joint_base.current_posi / BASE_REDUCTION_RATIO; //输出轴位置
       float base_speed = BASE_SPEED[i]; //输出轴速度
       if (fabs(Arm.basejoint_current_angle - Arm.basejoint_target_angle) > POSITION_THRESHOLD)
       {
       Arm.joint_base.SetPosVel(Arm.basejoint_target_angle * BASE_REDUCTION_RATIO, base_speed);
       }
       else
       {
        Arm.joint_base.SetPosVel(Arm.basejoint_target_angle * BASE_REDUCTION_RATIO, 0.0f);
        Arm.base_arrived = true;

       }
    }

        /* -------------------- End电机控制（手动换算为输出轴） -------------------- */
        if (flag%10>=5)
        {

        Arm.endjoint_current_angle = Arm.joint_end.current_posi / END_REDUCTION_RATIO; //输出轴位置
        float end_speed = END_SPEED[i];
        if (fabs(Arm.endjoint_current_angle - Arm.endjoint_target_angle) > POSITION_THRESHOLD)
        {
        Arm.joint_end.SetPosVel(Arm.endjoint_target_angle * END_REDUCTION_RATIO, end_speed);
        }
        else
        {
            Arm.joint_end.SetPosVel(Arm.endjoint_target_angle * END_REDUCTION_RATIO, 0.0f);
            Arm.end_arrived = true;
        }
        }
    }
}


/**       设置机械臂的处于准备吸块位置         **/
void ArmType::SetStartPosition(uint8_t motor_number)
{
    if(enabled==false) return;
    else
    {
    arm[motor_number].basejoint_target_angle=-2.094f;
    arm[motor_number].endjoint_target_angle=0.523f;
     _is_taking_block = true;
     _is_holding_block = false;
     _is_releasing_block = false;
    }
}       

/**       采用固定角度，取块后将物块存于某个位置（具体位置待验证）        **/
void ArmType::SetArmTogether_front(uint8_t motor_number)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
{
    if(enabled==false) return;
    else
    {
    arm[motor_number].basejoint_target_angle=-1.047f;
    arm[motor_number].endjoint_target_angle=1.047f;
     _is_taking_block = false;
     _is_holding_block = true;
     _is_releasing_block = false;
    }
}
void ArmType::SetArmTogether_middle(uint8_t motor_number)
{
    if(enabled==false) return;
    else
    {
    arm[motor_number].basejoint_target_angle=-0.43f;
    arm[motor_number].endjoint_target_angle=0.43f;
     _is_taking_block = false;
     _is_holding_block = true;
     _is_releasing_block = false;
    }
}
void ArmType::SetArmTogether_rear(uint8_t motor_number)
{
    if(enabled==false) return;
    else
    {
    arm[motor_number].basejoint_target_angle=1.047f;
    arm[motor_number].endjoint_target_angle=-1.047f;
     _is_taking_block = false;
     _is_holding_block = true;
     _is_releasing_block = false;
    }
}
/**       采用灵活可调角度，机械臂的解算，未来?可以和遥控器联动拨杆连续调节功能?（TODO）        **/
void ArmType::SetSingleArmAngle(uint8_t motor_number, float basejoint_angle, float endjoint_angle)
{
    if(enabled==false) return;
    else
    {
    arm[motor_number].basejoint_target_angle = clamp(basejoint_angle, BASE_ANGLE_MIN, BASE_ANGLE_MAX);
    arm[motor_number].endjoint_target_angle = clamp(endjoint_angle, END_ANGLE_MIN, END_ANGLE_MAX);
     _is_taking_block = false;
     _is_holding_block = true;
     _is_releasing_block = false;
    }
}

void ArmType::ReleaseBlock(uint8_t motor_number)
{ 
    if(enabled==false) return;
    else
    {
    arm[motor_number].endjoint_target_angle=-1.57f; //放开物块的位置，也需要验证
    arm[motor_number].basejoint_target_angle=0.0f;
     _is_taking_block = false;
     _is_holding_block = false;
     _is_releasing_block = true;
     }
}
void ArmType::ActivateVacuum()
{
    _is_vacuum_enable = true;
    //逻辑待写
}
void ArmAngelToMotorangel()
{

}



