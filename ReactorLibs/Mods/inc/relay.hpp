#ifndef RELAY_HPP
#define RELAY_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_gpio.hpp"

#ifdef __cplusplus
}
#endif

// 继电器类
class Relay
{
public:
    Relay() {};

    enum State 
    {
        OFF = 0,
        ON = 1
    };
    enum TriggerType
    {
        HIGH_ON = 0, // 高电平触发型
        LOW_ON = 1   // 低电平触发型
    };

    /**
      * @brief 初始化一个继电器通道并绑定GPIO
      * @param GPIOx GPIO端口
      * @param Pin GPIO引脚
      * @param trig_type 触发逻辑:高电平触发或低电平触发
      */
    void Init(GPIO_TypeDef *GPIOx, uint32_t Pin, TriggerType trig_type); 
    void On();   // 打开继电器
    void Off();  // 关闭继电器

private:
    void _Set(State state); // 设置继电器状态
    BspGpio_Instance _gpio_inst; // 继电器所连接的 GPIO 实例
    TriggerType _trig_type;  // 继电器触发逻辑类型
    State _state;
    bool initialized = false;
};


#endif
