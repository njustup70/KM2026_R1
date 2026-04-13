#include "CBoard_Connect.hpp"
#include <algorithm>
#include <cmath>

int16_t mode = 0;

void R1CBoardCallback(uint8_t task_id, const uint8_t *payload, uint8_t payload_len, void *user_ctx);
void Connect_CBoard::Start()
{
  main_board.Init(Hardware::hcan_main, 0x220, false);
  main_board.RegisterTask(1, R1CBoardCallback, this);
}

void Connect_CBoard::Update()
{
  // 6 字节 uint8，完全匹配接收端格式
//  uint8_t payload[13];

//  // 放大 100 倍转为 int16
//  int16_t vx = (int16_t)(chassis_vx * 100.0f);
//  int16_t vy = (int16_t)(chassis_vy * 100.0f);
//  int16_t vz = (int16_t)(chassis_vz * 100.0f);

//  int16_t px = (int16_t)(chasis_posx * 100.0f);
//  int16_t py = (int16_t)(chasis_posy * 100.0f);
//  int16_t pz = (int16_t)(chasis_posz * 100.0f);

//  payload[0] = mode; // 模式

//  // 大端格式：高字节 <<8 在前，低字节在后
//  // 对应接收端：payload[1] <<8 | payload[2]
//  payload[1] = (vx >> 8) & 0xFF; // Vx 高8位
//  payload[2] = vx & 0xFF;        // Vx 低8位

//  payload[3] = (vy >> 8) & 0xFF; // Vy 高8位
//  payload[4] = vy & 0xFF;        // Vy 低8位

//  payload[5] = (vz >> 8) & 0xFF; // Vz 高8位
//  payload[6] = vz & 0xFF;        // Vz 低8位

//  payload[7] = (px >> 8) & 0xFF; // Px 高8位
//  payload[8] = px & 0xFF;        // Px 低8位

//  payload[9] = (py >> 8) & 0xFF; // Py 高8位
//  payload[10] = py & 0xFF;       // Py 低8位

//  payload[11] = (pz >> 8) & 0xFF; // Pz 高8位
//  payload[12] = pz & 0xFF;        // Pz 低8位

//  main_board.SendTask(0x210, 1, payload, 13, false);

 uint8_t payload[6];

    // 放大 100 倍转为 int16
    int16_t x = (int16_t)(chasis_posx * 100.0f);
    int16_t y = (int16_t)(chasis_posy * 100.0f);
    int16_t z = (int16_t)(chasis_posz * 100.0f);

    // 大端格式：高字节 <<8 在前，低字节在后
    // 对应接收端：payload[0] <<8 | payload[1]
    payload[0] = (x >> 8) & 0xFF;  // Vx 高8位
    payload[1] = x & 0xFF;         // Vx 低8位

    payload[2] = (y >> 8) & 0xFF;  // Vy 高8位
    payload[3] = y & 0xFF;         // Vy 低8位

    payload[4] = (z >> 8) & 0xFF;  // Vz 高8位
    payload[5] = z & 0xFF;         // Vz 低8位

    main_board.SendTask(0x210, 1, payload, 6, false);

}

void R1CBoardCallback(uint8_t task_id, const uint8_t *payload, uint8_t payload_len, void *user_ctx)
{

}