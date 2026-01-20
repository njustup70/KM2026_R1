#include "GetBlock.hpp"
#include "Monitor.hpp"
#include "bsp_gpio.h"

#define Slength 362500
#define Olength 939500
#define mOlength 469750//还没有测量暂定这么多


void GetBlock::Start()
{
    BspGpio_InstRegist(&huang1, huang1_GPIO_Port, huang1_Pin); //小黄初始化
    motor1.Init(&hcan1, 0x01, 0x00);
    motor1.Enable();
    motor1.SetPosiLim(4.0f, -3.5f);
    for(int i=0;i<4;i++)     //大疆电机的初始化
    {
    if(i>=2)//这里对于3，4大疆电机的位置控制 三是收回电机,四是退块电机
    {
    motors[i].Init(&hcan1, i + 1, MotorDJIMode::Pos_Control, false);
    motors[i].position_pid.Init(0.10,0.0,0,0);
    }else //这里对于两个大疆电机的速度控制
    {
        motors[i].Init(&hcan1, i + 1, MotorDJIMode::Speed_Control, false);}
        motors[i].speed_pid.Init(8.0,0.05,0.015,0);
        motors[i].speed_pid.ForwardLize(PidGeneral::SpeedForward, 1.0f, 1.0f, 1.0f);
        motors[i].measure.total_angle=0;//目前先不记录位置所以对于totalangle不做要求和跟踪
        motors[i].Enable();
    }   
    enabled = true;
}

void GetBlock::Update() 
{
   if(!enabled) return;
   switch (task_now)
   {
   case Gain_block:
   GetinBlock();
    break;
   case Output_block:
    Outputblock();
    break;
   default:
    break;
   }
    UploadPos();
}

//@brief 上传位置和速度到电机
void GetBlock::UploadPos()
{
    //所有电机关闭的判断
    if(!Enmotor)
    {
        Enmotors_off();
        return;
    }
    //发送杆驱动舵机的位置和速度指令
    motor1.SetPosVel(target_pos[0], target_spd[0]);
    //发送1，2号电机的速度指令
   for(int i=0;i<2;i++)
   {
    motors[i].SetSpeed(target_spd[i+1],36.0f);
   }
   //发送3，4号电机的位置指令
   for (int i = 2; i < 4; i++)
   {
    motors[i].SetPos(target_pos[i-1]);
   }
     
}
//取不同高度块的逻辑函数需要在update里持续调用来判断动作执行
//目前还没有区分对于第三个物块的处理
bool GetBlock::GetinBlock()
{
    static int state =0 ;
    switch(Gstate)
    {
        //夹爪上升到指定高度，并且开启吸收物块的传送带 动作1
        case act_UP:
        target_pos[0]= H[0];
        target_spd[0] = 3.0f;
        target_spd[1] = 100.0f;
        target_spd[2]= -100.0f;
        break ;

        case act_IN:
        //不动夹爪往回收回，同时传送带将物块传送到指定位置 动作2
       target_pos[0]= H[1];
       
       target_pos[1]= -Slength;
       motors[2].position_pid.SetLimit(0,1200, 0.5f);
       if((motors[2].measure.total_angle <= -Slength + 10000)&(motor1.current_posi>=target_pos[0]-0.1))//到达指定位置停止传送带])
       {
        //传送带加速
        target_spd[1] = 200.0f;
        target_spd[2]= -200.0f;
       }else 
       {
        //传送带停止
       target_spd[1] =0.0f;
       target_spd[2] =0.0f;
       }
        break;

        case act_RESET:
        //夹爪传送带不动
        target_spd[1] =0.0f;
        target_spd[2] =0.0f;
        target_pos[1]= 0.0f;
        motors[2].position_pid.SetLimit(0,1200, 0.5f);
        if(motors[2].measure.total_angle >=-10000)//到达指定位置停止传送带])
        {
            target_pos[0]=0.0f;//回到原位置
            target_spd[0] = 3.0f;
            return true; //完成取块动作
        }
        break;

        default:
        break;
    }
    //这里是读取小黄的状态来判断动作的转换
    //目前先注释掉使用手动切换来调整
    //动作2需要检测物块到指定位置后开始收回夹爪，需要加上小黄的判断
    if(Gstate==act_UP)
    {
        state += BspGpio_GetState(&huang1)-1;//检测低电平
        if(state<=-20)
        {
            Gstate=act_IN;
            state =0;
        }
    }else if(Gstate==act_IN)
    {
        state += BspGpio_GetState(&huang1);//检测高电平
        if(state>=20)
        {
            Gstate=act_RESET;
            state =0;
        }
    }
    return false;
}

//@brief 吐出存储的物块
//
bool GetBlock::Outputblock()
{
    
    return false;
}

//@brief 夹爪上升的命令函数
void GetBlock::G_rise()
{
    target_pos[0] = H_40[0];
    target_spd[0] = 3.0f;
}

//@brief 夹爪吸收物块的函数
void GetBlock::Block_in()
{
    Blockup = true;
    target_spd[1] = 100.0f;
    target_spd[2] = -100.0f;
}

void GetBlock::Block_stop()
{
    Blockup = false;
    target_spd[1] = 0.0f;
    target_spd[2] = 0.0f;
}

void GetBlock::G_fall()
{
    target_pos[0] = 0.0f;
    target_spd[0] = 3.0f;
}

void GetBlock::G_in()
{
    target_pos[1]= -Slength;
    motors[2].position_pid.SetLimit(0,1200, 0.5f);
}
void GetBlock::G_out()
{
    target_pos[1]= 0.0f;
    motors[2].position_pid.SetLimit(0,1200, 0.5f);
}
//@brief 夹爪推出物块的函数
void GetBlock::Block_out1()
{
    target_pos[2] = -Olength;
    motors[3].position_pid.SetLimit(0,1500, 0.5f);
}

void GetBlock::Enmotors_off()
{
    for(int i=0;i<4;i++)
    {
        motors[i].Neutral();//所有电机不控制
    }
}
void GetBlock::Inblock( const float* h)
{
    H[0]=h[0];
    H[1]=h[1];
    Gstate = act_UP;
    task_now = Gain_block;
}