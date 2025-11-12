#include "Body.hpp"
#include "RobotSystem.hpp"
#include <math.h>

#define SlowRate   0.5f //即将到达时的速度
#define WheelSkp    10
#define WheelSki    5
#define WheelSkd     0.025
#define WheelSkf     0.16
#define WheelPkp     1.3
#define WheelPki     0.05
#define WheelPkd     0.01
#define WheelPkf      0

int Stflag=0;
int Wtflag=0;
int finishP=0;
int finishW=0;
bool PoolFlag;
MotorDji motors[3];
//唯一全局实例
PickRod pickrod;

//定义全局变量

void PickRod::Build()
{
    //初始化电机
    for(int i=1;i<3;i++)
    {
        motors[i].Init(&hcan1, i + 2, MotorDJIMode::Pos_Control, false);
        motors[i].speed_pid.Init(WheelSkp,WheelSki,WheelSkd,WheelSkf);
        motors[i].position_pid.Init(WheelPkp,WheelPki,WheelPkd,WheelPkf);
        motors[i].Enable();
        motors[i].position_pid.LimitSet(0, pickrod.VwheelLim, 0.5f);
    }
    motors[0].Init(&hcan1,  2, MotorDJIMode::Pos_Control, true);
    motors[0].position_pid.LimitSet(0, pickrod.VpitchLim, 0.5f);
    motors[0].position_pid.Init(0.108,0,0,0);
    motors[0].Enable();
}


void PickRod::PitchMove()
{
    
    if(Stflag==0)
    {
        return;
    }else if(Stflag==1)
    {
        pickrod.Ptarget = 91000.0f;
        pickrod.motor_spd[0] = 15.0f;
    }else if(Stflag==-1)
    {
        pickrod.Ptarget = -91000.0f;
        pickrod.motor_spd[0] = -15.0f;
    }
    pickrod.MoveProduce(pickrod.motor_spd[0],pickrod.Ptarget,1);
}
void PickRod::WheelMove()
{
    if(Wtflag==0)
    {
        return;
    }else if(Wtflag==1)
    {
        pickrod.Wtarget= 7800.0f;
        pickrod.motor_spd[1] = 5.0f;
        pickrod.motor_spd[2] = 5.0f;

    }else if(Wtflag==-1)
    {
        pickrod.Wtarget = -7800.0f;
        pickrod.motor_spd[1] = -5.0f;
        pickrod.motor_spd[2] = -5.0f;
    }
    pickrod.MoveProduce(pickrod.motor_spd[1],pickrod.Wtarget,2);
    pickrod.MoveProduce(pickrod.motor_spd[2],-pickrod.Wtarget,3);
    // motors[1].SetPos(target);
    // motors[1].position_pid.LimitSet(0,  fabs (target_speed)*19,0.5f);
    // motors[2].SetPos(-target);
    // motors[2].position_pid.LimitSet(0,fabs(-target_speed)*19,0.5f);

    // if(fabs((target-motors[1].measure.total_angle)/target)<0.10)
    // {
    //     if(wcnt<50)
    //     {
    //         wcnt++;
    //     }else if(wcnt>=50)
    //     {
    //         finishW=1;
    //     }
       
    // }
    // MotorDji::ControlAllMotors();
}

void PickRod::Pitchup()
{
    if(Stflag!=-1)
    {
        motors[0].measure.total_angle=0;
    }
    
    Stflag=-1;
}

void PickRod::Wheelup()
{
   if(Wtflag!=-1) 
    {
    motors[1].measure.total_angle=0;
    motors[2].measure.total_angle=0;
    }
    Wtflag=-1;
}
void PickRod::Wheeldown()
{
    if(Wtflag!=1)
    {
    motors[1].measure.total_angle=0;
    motors[2].measure.total_angle=0;
    }
   
    Wtflag=1;
}

void PickRod::Pitchdown()
{
    //先给位置变成0，每次重新初始化计数
    if(Stflag!=1)
    {
        motors[0].measure.total_angle=0;
    }
    Stflag=1;
}
//停止目前动作并且重置角度读数并且失能电机
void PickRod::Wheelstop()
{
    for(int i=1;i<3;i++)
    {
        motors[i].Disable();
        motors[i].measure.total_angle=0;
    }
    Wtflag=0;
}
//停止目前动作 并且用来在第一次的重置读数
void PickRod::PitchStop()
{
    //代码目前是按照pitch采用速度环，wheel采用位置环来写的
    motors[0].Disable();   
    motors[0].measure.total_angle=0;
    Stflag=0;
}

//停止所有的许可但是没有重置now
void PickRod::AllStop()
{
    for(int i=0;i<3;i++)
    {
        motors[i].Disable();
    }
    Wtflag=0;
    Stflag=0;
}
/**
 * @brief 杆子的移动过程函数，无论是pitch的运动还是杆子的运动都需要这个函数来控制运动过程
 */

 void PickRod::MoveProduce(float T_speed, int32_t T_place, uint8_t motor_id)
 {
    static float rate=1.0f;
    static int wcnt=0;
    static int pcnt=0;
    int nowplace;
    static float Speedlimit=0.0f;
    nowplace=motors[motor_id-1].measure.total_angle;
    float error_rate=(float)(T_place-nowplace)/T_place;
    error_rate=fabs(error_rate);

    if(error_rate>0.06f)
    {
        rate=1;
    }
    if(error_rate <0.065f)
    {
        if(rate < SlowRate)
        {
            rate=SlowRate;
        }else 
        {
            rate-=0.1f;
        }
    }
    if(error_rate<(0.02f*motor_id))
    {
        rate=0.0f;
        if(motor_id==1)
        {
          if(pcnt<50)
        {
            pcnt++;
        }else if(pcnt>=50)
        {
            finishP=1;
        }
        }else  if(motor_id==2)
        {
            if(wcnt<50)
            {
                wcnt++;
            }else if (wcnt>=50)
            {
                finishW=1;
            }
        }
        
    }
    motors[motor_id-1].SetPos(T_place);                              //设置目标点
    Speedlimit=(float)(fabs(T_speed*rate*19.0f)+110.0f);
    motors[motor_id-1].position_pid.LimitSet(0,Speedlimit, 0.5f); //限制速度
 }
 bool PickRod::PickUp()
 {
    static bool flag;
    static int pcnt =0 ;
    if(flag==0)
    {
        Stflag=0;
        Wtflag=0;
        if(!(HAL_GPIO_ReadPin(PickSensor_GPIO_Port, PickSensor_Pin))) //低电平显示检测到
        {
            if(pcnt<50)pcnt++;
        else if(pcnt>=50)flag=1;
        }    
        return false;
    }
    else if(flag==1)
    {
    pickrod.Wheelup();
    if(finishW==1)
    {
        pickrod.Pitchup();
        return false;
    }
    }
    if(finishP==1)
    {
        return true;
    }
    return false;
 }
 
 void PickRod::PickReset()
 {
    finishP=0;
    finishW=0;
    for(int i=0;i<3;i++)
    {
        motors[i].measure.total_angle=0;
    }
    Stflag=0;
    Wtflag=0;
 }

 bool PickRod::PickDown()
 {
    static bool flag =0;
    static int pcnt =0 ;
    if(flag==0)
    {
        if((HAL_GPIO_ReadPin(PickSensor_GPIO_Port, PickSensor_Pin))) //低电平显示检测到
        {
            if(pcnt<50)pcnt++;
        else if(pcnt>=50)flag=1;
        }    
        return false ;
    }
    else if(flag==1)
    { 
        pickrod.Pitchdown();
        return false;
    }
    if(finishP==1)
    {
        return true;
    }
    return false;
 }
 
 //必须在执行完pick up后才能调用
 void PickRod::WheelGo(float Goplace)
 {
    if(finishW==1)
    {
        pickrod.Wtarget+=Goplace;
    }

 }
 //更新函数
 void PickRod::Update()
 {
    pickrod.PitchMove();
    pickrod.WheelMove();
 }