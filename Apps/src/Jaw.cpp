#include "Jaw.hpp"
#include "bsp_uart.h"
#include "motor_dji.hpp"
#include "string.h"
#include "std_math.hpp"
 float p_kp=0.5f,p_kd=0.01f,p_ki=0.0f;
float s_kp=3.0f,s_kd=0.005f,s_ki=0.25f;


void JawType::Start()
{

        for (int i = 0; i < 5; i++)
    {
       jaw_djmotor[i].Init(&hcan1, i+1,Pos_Control, false);
       jaw_djmotor[i].position_pid.Init(p_kp,p_ki,p_kd,0);
       jaw_djmotor[i].speed_pid.Init(s_kp,s_ki,s_kd,0);
       jaw_djmotor[i].speed_pid.ForwardLize(PidGeneral::SpeedForward, 1.0f, 1.0f, 1.0f);  
       jaw_djmotor[i].position_pid.SetLimit(0,speed[i],0.9f);
       jaw_djmotor[i].Enable();
    }
  
   
}

void JawType::Update()
{
          ControlJawRotate(90);
        if (jaw_rotate_arrived)
        {
            ControlBaseRotate(90);
            if (base_rotate_arrived)
            {
                ControlBaseMove(83);
                if (base_move_arrived)
                {
                    ControlJawUpDown(2);
                    if (jaw_updown_arrived)
                    {
                        ControlJawForwardBack(2);
                    }
                }
            }
        }
}


    void JawType::Enable()
{
    enabled = true; //取矛头结构使能标志
}

    void JawType::Disable()
{
    enabled = false; //取矛头结构失能标志   
}


   /**
    *@brief 控制上层夹爪旋转
    *@note 该函数用于设置上层夹爪旋转电机的目标位置，并根据当前位置与目标位置的误差判断是否到达目标位置。
    *@param position 目标位置，单位为度。
    */ 
   void JawType::ControlJawRotate(float position)
{
    if (!enabled)  return; // 如果取矛头结构未使能，直接返回
    jaw_rotate_target_position = position*819.1;
    jaw_djmotor[0].SetPos(jaw_rotate_target_position);
    jaw_rotate_current_position = jaw_djmotor[0].measure.total_angle;
    if (abs(jaw_rotate_current_position - jaw_rotate_target_position) < 5000) // 误差小于5000认为到达
    {
        jaw_rotate_arrived = true;
    }
    else
    {
        jaw_rotate_arrived = false;
    }
}
   /**
    * @brief 控制底座旋转
    * @note 该函数用于设置底座旋转电机的目标位置，并根据当前位置与目标位置的误差判断是否到达目标位置。
    * @param position 目标位置，单位为度。
    */
   void JawType::ControlBaseRotate(float position)
{
    if (!enabled)  return; // 如果取矛头结构未使能，直接返回
    base_rotate_target_position = position*4095.5;
    jaw_djmotor[1].SetPos(base_rotate_target_position);
    base_rotate_current_position = jaw_djmotor[1].measure.total_angle;
    if (abs(base_rotate_current_position - base_rotate_target_position) < 5000) // 误差小于5000认为到达
    {
        base_rotate_arrived = true;
    }
    else
    {
        base_rotate_arrived = false;
    }
}
   /**
    * @brief 控制底座前后运动
    * @note 该函数用于设置底座前后运动电机的目标位置，并根据当前位置与目标位置的误差判断是否到达目标位置。
    * @param position 目标位置，单位为mm。
    */
   void JawType::ControlBaseMove(float position)
{
    if (!enabled)  return; // 如果取矛头结构未使能，直接返回
    base_movre_target_position = position*1788.14535839;
    // BaseMoveLimit(0,85);
    jaw_djmotor[2].SetPos(base_movre_target_position);
    base_move_current_position = jaw_djmotor[2].measure.total_angle;
    if (abs(base_move_current_position - base_movre_target_position) < 5000) // 误差小于5000认为到达
    {
        base_move_arrived = true;
    }
    else
    {
        base_move_arrived = false;
    }
}
   /** 
    * @brief 控制上层夹爪上下微调
    * @note 该函数用于设置上层夹爪上下微调电机的目标位置，并根据当前位置与目标位置的误差判断是否到达目标位置。
    * @param position 目标位置，单位为mm。(正值表示下降，负值表示上升)
    */
   void JawType::ControlJawUpDown(float position)
{
    if (!enabled)  return; // 如果取矛头结构未使能，直接返回
    jaw_updown_target_position = position*-147438.0f;
    // JawUpDownLimit(0,40);
    jaw_djmotor[3].SetPos(jaw_updown_target_position);
    jaw_updown_current_position = jaw_djmotor[3].measure.total_angle;
    if (abs(jaw_updown_current_position - jaw_updown_target_position) < 5000) // 误差小于5000认为到达
    {
        jaw_updown_arrived = true;
    }
    else
    {
        jaw_updown_arrived = false;
    }
}
   /**
    * @brief 控制上层夹爪前后微调
    * @note 该函数用于设置上层夹爪前后微调电机的目标位置，并根据当前位置与目标位置的误差判断是否到达目标位置。
    * @param position 目标位置，单位为mm。(正值表示前移，负值表示后移)
    */
   void JawType::ControlJawForwardBack(float position)
{
    if (!enabled)  return; // 如果取矛头结构未使能，直接返回
    jaw_forwardback_target_position = position*(-147438.0f);
    // JawForwardBackLimit(0,40);
    jaw_djmotor[4].SetPos(jaw_forwardback_target_position);
    jaw_forwardback_current_position = jaw_djmotor[4].measure.total_angle;
    if (abs(jaw_forwardback_current_position - jaw_forwardback_target_position) < 5000) // 误差小于5000认为到达
    {
        jaw_forwardback_arrived = true;
    }
    else
    {
        jaw_forwardback_arrived = false;
    }
}
   /** 
    * @brief 设置上层夹爪前后微调运动的限幅
    * @note 该函数用于限制上层夹爪前后微调的目标位置在指定的最小值和最大值之间。
    * @param min_position 目标位置的最小值，单位为mm。
    * @param max_position 目标位置的最大值，单位为mm。
   */
   void JawType::JawForwardBackLimit(float min_position,float max_position)
{
    jaw_forwardback_target_position = Limit_ABS(jaw_forwardback_target_position, max_position*(-147438), min_position*(-147438));
}
   /** 
    * @brief 设置上层夹爪上下微调运动的限幅
    * @note 该函数用于限制上层夹爪上下微调的目标位置在指定的最小值和最大值之间。
    * @param min_position 目标位置的最小值，单位为mm。
    * @param max_position 目标位置的最大值，单位为mm。
   */
   void JawType::JawUpDownLimit(float min_position,float max_position)
{
    jaw_updown_target_position = Limit_ABS(jaw_updown_target_position, max_position*(-147438), min_position*(-147438));
}
   /** 
    * @brief 设置底座前后运动限幅
    * @note 该函数用于限制底座前后运动的目标位置在指定的最小值和最大值之间。
    * @param min_position 目标位置的最小值，单位为mm。
    * @param max_position 目标位置的最大值，单位为mm。
   */
   void JawType::BaseMoveLimit(float min_position,float max_position)
{
    base_movre_target_position = Limit_ABS(base_movre_target_position, max_position*1788.14535839, min_position*1788.14535839);
}

void JawType::JawRotateSpeed(float Speed)
{
    speed[0]=Speed;
}
void JawType::BaseRotateSpeed(float Speed)
{
    speed[1]=Speed;
}
void JawType::BaseMoveSpeed(float Speed)
{
    speed[2]=Speed;
}
void JawType::JawUpDownSpeed(float Speed)
{
    speed[3]=Speed;
}
void JawType::JawForwardBackSpeed(float Speed)
{
    speed[4]=Speed;
}