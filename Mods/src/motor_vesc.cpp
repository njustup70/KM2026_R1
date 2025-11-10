#include "motor_vesc.hpp"
#include "bsp_can.h"
#include <string.h>
#include "can.h"


#define CAN_OFFSET (hcan == &hcan1 ? 0 : 8)

uint8_t canTx_text[8];//保存发送数据
float read_current = 0;
float read_duty = 0;
int read_rpm = 0;
float test_duty_value = 0.1;

//// 全局电机实例
MotorVESC motor_vesc_1;
MotorVESC motor_vesc_2;
MotorVESC motor_vesc_3;



/// @brief 接收回调函数
/// @param RxHeader 
/// @param RxData 
 void Motor_vesc_RxCallback(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData, CAN_HandleTypeDef *hcan)
{
    MotorVescRecvData vesc_rx;
    vesc_rx.rx_header = *rxHeader;
    memcpy(vesc_rx.recv_data,rxData,8);
     motor_vesc_handle(vesc_rx);
}


/// @name motor_vesc_get_rpm
/// @brief 返回指定标签电机的真实转速
/// @param motor_id ：0-3
int motor_vesc_get_rpm(int motor_id)
{
    if (motor_id == motor_vesc_1.motor_id)
    {
        read_rpm = motor_vesc_1.motor_rpm_real;
        return read_rpm;
    }
    else if (motor_id == motor_vesc_2.motor_id)
    {
        read_rpm = motor_vesc_2.motor_rpm_real;
        return read_rpm;
    }
    else if (motor_id == motor_vesc_3.motor_id)
    {
        read_rpm = motor_vesc_3.motor_rpm_real;
        return read_rpm;
    }
    
    else return 0;
}

// 设置电机转速（外部调用,上层）
int  motor_vesc_set_rpm(int motor_id, float set_rpm)
{
    if (motor_id == motor_vesc_1.motor_id)
    {
        motor_vesc_1.motor_rpm_set = (int)set_rpm;
        return 1;
    }
    else if (motor_id == motor_vesc_2.motor_id)
    {
        motor_vesc_2.motor_rpm_set = (int)set_rpm;
        return 1;
    }
		    else if (motor_id == motor_vesc_3.motor_id)
    {
        motor_vesc_3.motor_rpm_set = (int)set_rpm;
        return 1;
    }
	return 0;
}


void MotorVESC_Init(MotorVESC *motor, CAN_HandleTypeDef *can_n, int motor_id, int motor_can_id)
{
    motor->motor_id = motor_id;
    motor->motor_can_id = motor_can_id;
    motor->targ_can_n = can_n;
    motor->motor_duty_real = 0;
    motor->motor_rpm_real = 0;
    motor->motor_duty_set = 0;
    motor->motor_rpm_set = 0;
    uint32_t motor_rx_id = ((CAN_PACKET_STATUS << 8) | motor->motor_can_id);

    uint32_t motor_tx_id = ((CAN_PACKET_SET_RPM << 8) | motor->motor_can_id); 
    BspCan_InstRegist(&motor->bspcan_inst, can_n, motor_rx_id, motor_tx_id, 1, 1, Motor_vesc_RxCallback);
}


/**
 * @name motor_vesc_handle
 * @brief 这个函数处理了收到的信息并回复电机
 * @note 
 * @details 本函数内完成了回复电机（设置电机速度等）的操作；如果有需要的操作可以在此更改
 * @warning 
 */
void motor_vesc_handle(MotorVescRecvData vesc_recvs)
{
    // 解码数据来自于CAN总线上的谁
    uint8_t can_id = vesc_recvs.rx_header.ExtId & 0xff;

    MotorVESC* targ_motor_vesc;

    // 和哪个电机匹配就和谁发
    if (can_id == motor_vesc_1.motor_can_id)
    {
        targ_motor_vesc = &motor_vesc_1;
    }
    else if (can_id == motor_vesc_2.motor_can_id)
    {
        targ_motor_vesc = &motor_vesc_2;
    }
	else if (can_id == motor_vesc_3.motor_can_id)
    {
        targ_motor_vesc = &motor_vesc_3;
    }
    else 
        return;//若无匹配，则直接返回

 targ_motor_vesc->MotorVESC_SetMotorRPM(targ_motor_vesc->motor_rpm_set);


    // 获取电调上报的消息类型
    CanPacketType vesc_status_type = (CanPacketType)(vesc_recvs.rx_header.ExtId >> 8);
    // 解码
    switch (vesc_status_type)
    {
        case CAN_PACKET_STATUS: // 第一类上报
        {
            targ_motor_vesc->motor_duty_real = (vesc_recvs.recv_data[6] * 256 + vesc_recvs.recv_data[7]) / 1000.0;

            read_current = (vesc_recvs.recv_data[4] * 256 + vesc_recvs.recv_data[5]) / 10.0;
            
            int32_t temp = (int32_t)(vesc_recvs.recv_data[0] << 24 | vesc_recvs.recv_data[1] << 16
                | vesc_recvs.recv_data[2] << 8 | vesc_recvs.recv_data[3]);
                
            if (temp < 50000 && temp > -50000)
            {
                targ_motor_vesc->motor_rpm_real = temp;
            }
            break;
            // targ_motor_vesc->motor_rpm_real = (int32_t)(vesc_recvs.recv_data[0] << 24 | vesc_recvs.recv_data[1] << 16
            //     | vesc_recvs.recv_data[2] << 8 | vesc_recvs.recv_data[3]);
        }
        default:
        break;
    }
}


// 设置电机转速
void  MotorVESC::MotorVESC_SetMotorRPM(int RPM)
{
   this->MotorVESC_SendCanTXBuffer( CAN_PACKET_SET_RPM, RPM);
}


// 设置电机占空比
void  MotorVESC::MotorVESC_SetMotorDuty( float duty)
{
  this->MotorVESC_SendCanTXBuffer(CAN_PACKET_SET_DUTY, duty);
}



// 发送CAN消息
void  MotorVESC::MotorVESC_SendCanTXBuffer(CanPacketType cmd_type, float values)
{
    static uint32_t txmailbox;            // CAN 邮箱
//    CAN_TxHeaderTypeDef TxMsg;            // TX 消息头

//    // 配置标准CAN参数
//    TxMsg.StdId = 0x00;    // 低8位为CAN_ID，高21位为指令ID
//    TxMsg.ExtId = (cmd_type << 8 | this->motor_can_id);
//    TxMsg.IDE = CAN_ID_EXT;
//    TxMsg.RTR = CAN_RTR_DATA;
//    TxMsg.DLC = 8;
int ExtId=(cmd_type << 8 | this->motor_can_id);

  BspCan_TxConfig custom_tx_conf=BspCan_GetTxConfig(this->targ_can_n,ExtId,1,0,8,50);
    uint8_t txbuf[8] = {0};
    switch (cmd_type)
    {
        case CAN_PACKET_SET_DUTY:
        {
            int32_t data;
            data = (int32_t)(values * 100000) ;
            txbuf[0] = data >> 24 ;
            txbuf[1] = data >> 16 ;
            txbuf[2] = data >> 8 ;
            txbuf[3] = data ;
            break;
        }
        case CAN_PACKET_SET_RPM:
        {
            int32_t data;
            data = (int32_t)(values) ;
            txbuf[0] = data >> 24 ;
            txbuf[1] = data >> 16 ;
            txbuf[2] = data >> 8 ;
            txbuf[3] = data ;
            break;
        }
        default:
            break;
    }
     // 等待CAN邮箱,很占时间，最好不要放在回调里
   while (HAL_CAN_GetTxMailboxesFreeLevel(this->targ_can_n) == 0);    
   BspCan_Transmit(custom_tx_conf,txbuf);
 //   HAL_CAN_AddTxMessage(this->targ_can_n,&TxMsg, txbuf, &txmailbox);
	memcpy(canTx_text,txbuf,8);
}



float MotorVESC::limit_abs(float targ_num, float limit)
{
    if (targ_num > limit)
    {
        return limit;
    }
    if (targ_num < -limit)
    {
        return -limit;
    }
    return targ_num;
}


