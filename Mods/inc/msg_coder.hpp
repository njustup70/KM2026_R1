#ifndef MSG_CODER_HPP
#define MSG_CODER_HPP

#include "stm32f4xx_hal.h"
#include "bsp_uart.h"

typedef struct
{
    uint8_t frame_head; // 帧头
    uint8_t frame_type; // 帧类型
    uint8_t data[64];   // 数据包（最大64Byte）
    int data_len;       // 数据包长度
    uint8_t frame_tail; // 帧尾
} FrameStruct;

/**
 * @brief 串口消息调制器
 * @note 只有遇到自定义通信的时候才会用到单独的调制器实例
 */
class UartMsgCoder
{
private:

public:
    UartMsgCoder() {};
    ~UartMsgCoder() {};

    FrameStruct frame; // 帧结构体

    BspUart_Instance uart_inst; // 串口实例

    /**      初始化     **/
    void Init(UART_HandleTypeDef *huart);
    void SetFrameParam(uint8_t Frame_Head, uint8_t Frame_Type, uint8_t *Data, int data_len, uint8_t Frame_Tail);
    uint8_t CalculateFrameHead(uint8_t *data, int data_len);

    /**     消息编码与解码  	**/
    int Encode(uint8_t *buf);
    int EncodeMsg(uint8_t frame_type, uint8_t *data, int data_len, uint8_t *out_buf);
    int DecodeMsg(uint8_t *buf);

    /**     发送接口     **/
    bool SendEncodedMsg(uint8_t *encoded_data, int length);
    bool SendMsg(uint8_t frame_type, uint8_t *data, int data_len);

    /**     工具方法（静态方法）  	**/
    static void FloatToBytes(float value, uint8_t *bytes);
    static float BytesToFloat(uint8_t *bytes);
    static void Uint16ToBytes(uint16_t value, uint8_t *bytes);
    static uint16_t BytesToUint16(uint8_t *bytes);
};

/**
 * @brief CAN消息调制器
 * @note 只有遇到自定义通信的时候才会用到单独的调制器实例
 */
class CanMsgCoder
{
public:
    CanMsgCoder() {};

    void Init();
};

#endif