#include "CBoard_Connect.hpp"
#include <algorithm>
#include <cmath>
#include "farcon.hpp"
int16_t mode = 0;
extern Farcon farcon;
extern BoardComm chassis_board;
uint8_t payload[8];
uint16_t vx;
uint16_t vy;
uint16_t vz;

void R1CBoardCallback(uint8_t task_id, const uint8_t *payload, uint8_t payload_len, void *user_ctx);
void Connect_CBoard::Start()
{
  chassis_board.Init(Hardware::hcan_main, 0x220, false);
  chassis_board.RegisterTask(1, R1CBoardCallback, this);
}

void Connect_CBoard::farcon_vct()
{
  memset(payload, 0, 8);
  payload[0] = 1; // 速度模式
  payload[1]=1;
  // 放大 100 倍转为 int16
  vy = -(int16_t)(farcon.jy_data_origin[2]);
  vx = -(int16_t)(farcon.jy_data_origin[3]);
  vz = -(int16_t)(farcon.jy_data_origin[0]);
  // 大端格式：高字节 <<8 在前，低字节在后
  // 对应接收端：payload[0] <<8 | payload[1]
  payload[2] = (vx >> 8) & 0xFF; // Vx 高8位
  payload[3] = vx & 0xFF;        // Vx 低8位

  payload[4] = (vy >> 8) & 0xFF; // Vy 高8位
  payload[5] = vy & 0xFF;        // Vy 低8位

  payload[6] = (vz >> 8) & 0xFF; // Vz 高8位
  payload[7] = vz & 0xFF;        // Vz 低8位

  chassis_board.SendTask(0x210, 1, payload, 8, false);
}

void Connect_CBoard::send_KFS_all()
{
 memset(payload, 0, 8);
 payload[0] = 1;
 payload[1] = 2;//KFS
uint8_t dest[3];
     for (int i = 0; i < 3; i++)
    {
        dest[i] = 0; // 清零
        // 每个字节装4个原始数据
        for (int j = 0; j < 4; j++)
        {
            // 计算原始数据在 src 数组中的绝对索引: i*4 + j
            // 计算位移量: j*2
            dest[i] |= (farcon.KFS_values[i * 4 + j] & 0x03) << (j * 2);
        }
    }
payload[2]=dest[0];
payload[3]=dest[1];
payload[4]=dest[2];
		  chassis_board.SendTask(0x210, 1, payload, 8, false);
}

void Connect_CBoard::c_board_control()
{
   memset(payload, 0, 8);
//底盘接管模式
 payload[0] = 2;
//发送按钮
 payload[1] = 1;
 uint8_t temp_button_add_first=0;
  uint8_t temp_button_add_second=0;
      for (int i = 0; i < 8; i++)
    {
        temp_button_add_first |= (   farcon.button_first_half[i] << i);
        temp_button_add_second |= (farcon.button_second_half[i + 8] << i);
    }

    payload[2]=temp_button_add_first;
    payload[3]=temp_button_add_second;

      chassis_board.SendTask(0x210, 1, payload, 8, false);
}

void Connect_CBoard::Update()
{
//  if (farcon.toggle[1] == 0&&farcon.toggle[2] == 0)
//    farcon_vct();//速度控制模式
   if(farcon.toggle[1] == 0&&farcon.toggle[2] == 1)
  {
    send_KFS_all();
		c_board_control();
  }
//  //底盘接管模式
//  if(farcon.toggle[1] == 1&&farcon.toggle[1]==0)
//  {

//  }
}

void R1CBoardCallback(uint8_t task_id, const uint8_t *payload, uint8_t payload_len, void *user_ctx)
{
}