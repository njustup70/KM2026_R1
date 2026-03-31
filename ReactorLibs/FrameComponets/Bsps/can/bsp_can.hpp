#pragma once
#include <cstdint>

#define BSPCAN_MAX_CANINSTS 24

namespace BSP
{
    namespace CAN
    {
        // ---- 不透明指针 ----
        struct OpaqueCan;
        /// @brief 不透明指针 CAN_HandleTypeDef* 的框架解耦替代
        using CanID = OpaqueCan*;

        // ---- 接收相关 ----

        /// @brief 接收帧头信息（纯净类型，不含 HAL）
        struct RxHeader
        {
            uint32_t id;        // 标准或扩展 ID
            uint8_t  dlc;       // 数据长度
            bool     is_ext;    // 是否扩展帧
            bool     is_remote; // 是否远程帧
        };

        /// @brief 接收回调函数类型：来源实例 + 帧头 + 数据指针
        using RxCallback = void(*)(CanID can, const RxHeader& header, uint8_t* data);

        /// @brief CAN 接收实例（由使用者分配内存）
        struct Instance
        {
            CanID      can_id;      // 哪条 CAN 总线
            uint32_t   rx_id;       // 订阅的接收 ID
            uint8_t    rx_is_ext;   // 接收 ID 是否扩展帧
            RxCallback rx_callback; // 接收回调
        };

        using HandleTypeDef = Instance*;

        /// @brief 注册 CAN 接收实例（内部自动配置过滤器 + 启动 CAN）
        void Regist(HandleTypeDef inst, CanID can, uint32_t rx_id,
                    uint8_t rx_is_ext, RxCallback callback,
                    const char* file = __builtin_FILE(), int line = __builtin_LINE());

        // ---- 发送（独立，不绑实例） ----

        /// @brief 发送 CAN 数据帧
        /// @param can 目标 CAN 总线
        /// @param id 发送的 CAN ID
        /// @param is_ext 是否扩展帧 (0=标准帧, 1=扩展帧)
        /// @param data 数据指针（最多8字节）
        /// @param len 数据长度（1-8）
        /// @param timeout_ms 超时时间（毫秒）
        void Transmit(CanID can, uint32_t id, uint8_t is_ext,
                      const uint8_t* data, uint8_t len,
                      uint16_t timeout_ms = 10);
        
        /// @brief 获取 CAN 总线名称（用于日志等）
        const char* GetName(CanID can);
    }
}
