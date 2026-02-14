#ifndef BSP_UART_H
#define BSP_UART_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_halport.hpp"

/**
 * @brief 串口工作模式定义
 * @note 对应了三种
 */
typedef enum
{
    BspUartType_Normal,
    BspUartType_IT,
    BspUartType_DMA,
}BspUart_TypeDef;



// 定义接收回调函数类型
typedef void (*BspUart_InstRxCallback)(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);

// 发送缓冲区大小
#define BSP_UART_TX_BUF_SIZE 2048

// 发送数据结构体，用于多实例共享
typedef struct
{
    uint8_t buffer[BSP_UART_TX_BUF_SIZE];          // 发送环形缓冲区
    volatile uint16_t head;                        // 缓冲区头指针（写入位置）
    volatile uint16_t tail;                        // 缓冲区尾指针（读取位置）
    volatile uint8_t is_busy;                      // 发送忙标志
    volatile uint16_t sending_len;                  // 当前正在发送的长度
}BspUart_TxData;

// 串口实例及其句柄定义
typedef struct
{
    UART_HandleTypeDef *huart;                      // 串口句柄
    BspUart_TypeDef rxtype;                         // Rx工作模式
    BspUart_TypeDef txtype;                         // Tx工作模式
    BspUart_InstRxCallback rx_callback;             // 接收回调函数

    uint8_t rx_buffer[64];                          // 接收缓冲区（最大64Byte）
    uint8_t rx_byte;                                // 单次接收的字节数，IT模式下始终为1    
    uint8_t rx_setlen;                              // 期望接收数据长度
    uint8_t rx_len;                                 // 实际接收数据长度

    // 发送缓冲区指针（支持共享）
    BspUart_TxData *tx_data;                        // 您的TX缓冲区/状态机
}BspUart_Instance;


void BspUart_InstRegist(BspUart_Instance *inst, UART_HandleTypeDef *huart, uint8_t rx_setlen,
                        BspUart_TypeDef rxtype, BspUart_TypeDef txtype, BspUart_InstRxCallback rx_callback,
                        BspUart_TxData *tx_data_storage);

void BspUart_Transmit(BspUart_Instance inst, uint8_t *data, uint8_t len);
void BspUart_Transmit_DMA(UART_HandleTypeDef* huart, uint8_t *data, uint8_t len);





#ifdef __cplusplus
}
#endif
#endif