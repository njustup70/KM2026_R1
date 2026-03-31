#pragma once

#include <cstdint>
#include "std_math.hpp"

namespace BSP
{
namespace GPIO
{
    /**
     * @brief EXTI 处理回调类型（按引脚线分发）
     */
    using ExtiHandler = void (*)(Pin pin);

    /**
     * @brief GPIO 实例（HAL 解耦表示）
     */
    class Inst
    {
    public:
        Inst();
        explicit Inst(Pin pin);

        /**
         * @brief 读取 GPIO 电平
         * @return true 高电平，false 低电平或实例无效
         */
        bool Read() const;

        /**
         * @brief 写 GPIO 电平
         * @param high true 高电平，false 低电平
         */
        void Write(bool high) const;

        /**
         * @brief 翻转 GPIO 电平
         */
        void Toggle() const;

        /**
         * @brief 锁定 GPIO 配置
         */
        void Lock() const;

        /**
         * @brief 注册 EXTI 回调（单 pin 单 handler）
         * @param handler 回调函数
         */
        void RegisterExtiHandler(ExtiHandler handler) const;

        /**
         * @brief 当前实例是否有效
         * @return true 有效，false 无效
         */
        bool IsValid() const;

    private:
        void* port_opaque; ///< 端口不透明指针（内部映射为 GPIO_TypeDef*）
        uint16_t pin_mask; ///< 引脚位掩码（GPIO_PIN_X）
        bool valid;        ///< 实例是否有效
    };
}
}
