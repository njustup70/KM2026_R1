/**
 * @file uart_portor.hpp
 * @brief UART 分发代理网关 (Mod Layer)
 * @details 提供多 Mod 共享同一硬件串口的端口路由分发服务。
 *          具备最小化的信封协议 [SOF(0xA5) | PORT_ID | DATA_LEN | PAYLOAD | CHECKSUM]
 */

#pragma once

#include "bsp_uart.hpp"
#include <stdint.h>

// ---- 协议信封相关常量 ----
#define UART_PORTOR_SOF 0xA5
#define UART_PORTOR_MAX_PAYLOAD 200

// ---- 类型定义 ----
/// @brief 端口接收回调函数定义 (参数：数据指针，数据长度)
using PortRxCallback = void (*)(const uint8_t *payload, uint8_t len);

/**
 * @class UartPortor
 * @brief Uart 分发代理类
 * @note 每个需要共享串口的 Mod 在内部持有一个本类的实例，像独占串口一样使用。
 *       同一硬件串口的多个代理共用底层的实际分发槽。
 */
class UartPortor
{
public:
    UartPortor() = default;
    ~UartPortor() = default;

    /**
     * @brief 初始化代理实例（底层会自动管理共用该 huart 的所有实例）
     * @param uart_id 硬件串口标识
     * @param my_port_id 我在这个串口上的专属端口号 (0~254)
     * @param callback 当收到该端口的数据时的回调函数
     * @return 注册成功/失败
     */
    bool Init(BSP::UART::UartID uart_id, uint8_t my_port_id, PortRxCallback callback);

    /**
     * @brief 发送数据 (自动包裹信封)
     * @param payload 数据内容
     * @param len 数据长度
     */
    void Send(const uint8_t *payload, uint8_t len);

private:
   BSP::UART::UartID uart_id_ = nullptr;
   uint8_t my_port_id_ = 0;
};
