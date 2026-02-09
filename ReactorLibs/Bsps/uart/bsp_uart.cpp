#include "bsp_uart.hpp"
#include "string.h"

#define BSPUART_MAX_CANINSTS 6          // 最多支持6个串口实例


/// @brief 收集记录所有实例，便于管理（static化避免外部调用）
static BspUart_Instance *bspuart_insts[BSPUART_MAX_CANINSTS] = {NULL};
static int bspuart_inst_count = 0; // 当前实例数量


/// @brief 注册一个串口实例
/// @param inst 实例指针
/// @param huart 所用串口
/// @param type 工作模式：普通、IT、DMA
/// @param rx_callback 响应触发的回调函数（一个Uart通道只允许一个实例存在，自然也只允许一个回调函数存在）
/// @param tx_data_storage TX缓冲区存储空间（若该串口已被其他实例注册，则可传NULL以共享缓冲区）
void BspUart_InstRegist(BspUart_Instance *inst, UART_HandleTypeDef *huart, uint8_t rx_setlen,
                        BspUart_TypeDef rxtype, BspUart_TypeDef txtype, BspUart_InstRxCallback rx_callback,
                        BspUart_TxData *tx_data_storage)
{
    // 检验参数有效性
    if (inst == NULL || huart == NULL) return; // 参数无效

    // 初始化实例基本参数
    inst->huart = huart;
    inst->rxtype = rxtype;
    inst->txtype = txtype;
    inst->rx_callback = rx_callback;
    inst->tx_data = NULL; // 先置空

    // 检查是否已有相同通道的实例，如果有，则共享其 TX Data
    for (int i = 0; i < bspuart_inst_count; i++)
    {
        if (bspuart_insts[i]->huart == huart)
        {
            // 发现已有实例注册了此huart
            if (bspuart_insts[i]->tx_data != NULL)
            {
                // 共享已有的发送缓冲区
                inst->tx_data = bspuart_insts[i]->tx_data;
            }
            break; // 只要找到一个就可以（假设它们都共享同一个）
        }
    }

    // 如果未找到共享缓冲区，且用户提供了存储空间，则初始化新缓冲区
    if (inst->tx_data == NULL && tx_data_storage != NULL)
    {
        inst->tx_data = tx_data_storage;
        
        // 初始化TX状态
        inst->tx_data->head = 0;
        inst->tx_data->tail = 0;
        inst->tx_data->is_busy = 0;
        inst->tx_data->sending_len = 0;
        memset(inst->tx_data->buffer, 0, sizeof(inst->tx_data->buffer));
    }

    // 清空接收缓冲区
    memset(inst->rx_buffer, 0, sizeof(inst->rx_buffer));
    // 期望接收数据长度，也是DMA的最大长度，DMA模式下达到本长度将触发全满中断
    inst->rx_setlen = rx_setlen;        

    // 根据工作模式，配置接收中断
    if (rxtype == BspUartType_IT)
    {
        HAL_UART_Receive_IT(huart, &inst->rx_byte, 1);
    }
    else if (rxtype == BspUartType_DMA)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, inst->rx_buffer, rx_setlen);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    // 记录实例
    if (bspuart_inst_count < BSPUART_MAX_CANINSTS)
    {
        // 允许同一通道多次注册（为了支持不同的Rx回调或共享Tx）
        bspuart_insts[bspuart_inst_count++] = inst;
    }
}


void BspUart_Transmit(BspUart_Instance inst, uint8_t *data, uint8_t len)
{
    if (inst.huart == NULL || data == NULL || len == 0) return; // 参数无效

    // 根据其txtype发送数据
    if (inst.txtype == BspUartType_Normal)
    {
        HAL_UART_Transmit(inst.huart, data, len, 100); // 普通模式，阻塞发送
    }
    else if (inst.txtype == BspUartType_IT)
    {
        HAL_UART_Transmit_IT(inst.huart, data, len);   // IT模式，非阻塞发送
    }
    else if (inst.txtype == BspUartType_DMA)
    {
        HAL_UART_Transmit_DMA(inst.huart, data, len);  // DMA模式，非阻塞发送
    }
    else
    {
        Error_Handler(); // 未知模式，进入软件错误中断
    }
}

void BspUart_Transmit_DMA(UART_HandleTypeDef* huart, uint8_t *data, uint8_t len)
{
    if (huart == NULL || data == NULL || len == 0) return; // 参数无效

    // 查找实例
    BspUart_Instance *inst = NULL;
    for (int i = 0; i < bspuart_inst_count; i++)
    {
        if (bspuart_insts[i]->huart == huart)
        {
            inst = bspuart_insts[i];
            break;
        }
    }

    // 如果未找到实例，或该实例无缓冲支持（共享的TxData为NULL），回退到直接HAL调用应
    if (inst == NULL || inst->tx_data == NULL)
    {
        HAL_UART_Transmit_DMA(huart, data, len);  // DMA模式，非阻塞发送
        return;
    }

    // 找到实例，使用缓冲发送逻辑
    // 关中断保护 FIFO 操作
    __disable_irq();

    BspUart_TxData *tx = inst->tx_data; // 便捷指针

    // 1. 将数据压入 FIFO
    for(int i=0; i<len; i++)
    {
        uint16_t next_head = (tx->head + 1) % BSP_UART_TX_BUF_SIZE;
        if(next_head != tx->tail) // 缓冲区未满
        {
            tx->buffer[tx->head] = data[i];
            tx->head = next_head;
        }
        else
        {
            // 缓冲区满，丢弃后续数据
            break; 
        }
    }

    // 2. 如果DMA空闲，启动发送
    if(tx->is_busy == 0)
    {
        if(tx->head != tx->tail)
        {
            tx->is_busy = 1;

            // 计算本次最大连续发送长度（处理环形回绕）
            uint16_t send_len = 0;
            if(tx->head > tx->tail)
            {
                send_len = tx->head - tx->tail;
            }
            else
            {
                // tail 到 缓冲区末尾
                send_len = BSP_UART_TX_BUF_SIZE - tx->tail;
            }

            tx->sending_len = send_len;
            HAL_UART_Transmit_DMA(inst->huart, &tx->buffer[tx->tail], send_len);
        }
    }

    __enable_irq();
}




/// @brief 覆写原DMA接收中断函数
/// @param huart 
/// @param Size 
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 遍历所有实例，根据huart通道，找到对应的实例
    for (int i = 0; i < bspuart_inst_count; i++)
    {
        BspUart_Instance *inst = bspuart_insts[i]; // 取出实例
        if (inst->huart == huart)
        {
            // 找到匹配的实例，调用其接收回调函数
            if (inst->rx_callback != NULL)
            {
                inst->rx_len = Size;                                // 记录实际接收长度
                inst->rx_callback(huart, inst->rx_buffer, Size);    // 调用用户定义的回调函数
            }
            // 执行后清空缓冲区
            memset(inst->rx_buffer, 0, inst->rx_setlen);
            // 重新使能DMA接收 并 关闭半满中断
            HAL_UARTEx_ReceiveToIdle_DMA(huart, inst->rx_buffer, inst->rx_setlen);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
    }
}

/// @brief 发送完成回调，用于驱动环形缓冲区继续发送
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 遍历查找实例
    for (int i = 0; i < bspuart_inst_count; i++)
    {
        BspUart_Instance *inst = bspuart_insts[i];
        if (inst->huart == huart)
        {
            // 如果没有TxData，无法进行缓冲接续，直接返回
            if (inst->tx_data == NULL) return;
            
            BspUart_TxData *tx = inst->tx_data;

            // 更新 tail，释放已发送的空间
            tx->tail = (tx->tail + tx->sending_len) % BSP_UART_TX_BUF_SIZE;
            tx->sending_len = 0;

            // 检查缓冲区是否还有数据
            if(tx->head != tx->tail)
            {
                // 计算下一段连续数据长度
                uint16_t send_len = 0;
                if(tx->head > tx->tail)
                {
                    send_len = tx->head - tx->tail;
                }
                else
                {
                    send_len = BSP_UART_TX_BUF_SIZE - tx->tail;
                }

                tx->sending_len = send_len;
                // 继续 DMA 发送
                HAL_UART_Transmit_DMA(inst->huart, &tx->buffer[tx->tail], send_len);
            }
            else
            {
                // 无数据，标记为空闲
                tx->is_busy = 0;
            }
            return; // 找到并处理后返回
        }
    }
}