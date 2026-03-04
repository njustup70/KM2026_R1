#pragma once
#include "System.hpp"
#include "uart_portor.hpp"

/**
 * @brief UART 性能与压力测试应用
 * @note 用于验证 BspUart 的 FIFO 发送机制，以及 UartPortor 的分发功能。
 *       只需在 System 中注册此应用即可全自动执行测试。
 */
class UartTest : public Application
{
    SINGLETON(UartTest) : Application("UartTest") {};

    APPLICATION_OVERRIDE

private:
    UartPortor portor_;
    uint32_t tick_cnt_ = 0;       // 200Hz 帧计数，一秒=200帧
    
    // 测试状态定义
    enum class TestStage {
        PREPARE,
        BANDWIDTH_TEST, // 满功率压测
        STAGE_GAP,      // 阶段间隔（静默）
        FIFO_TEST,      // FIFO 突发压测
        DONE
    } stage_ = TestStage::PREPARE;

    // 测试包数据 (64字节)
    uint8_t test_pkg_[64];
};
