#pragma once
#include "System.hpp"
#include "bsp_uart.hpp"


/**
 * @brief UART 性能与压力测试应用
 * @note 用于验证 BspUart 的 FIFO + DMA 实现是否在大规模发送下稳定
 */
class UartTest : public Application
{
    SINGLETON(UartTest) : Application("UartTest") {};

    APPLICATION_OVERRIDE

public:
    /**
     * @brief 配置测试参数
     * @param id 目标串口标识
     * @param interval_ms 发送间隔（目前受限于 App 调度频率，最高 200Hz）
     * @param packet_len 每次发送的包长度
     */
    void Config(BSP::UART::UartID id, uint32_t packet_len = 32);
};
