#ifndef BSP_GPIO_H
#define BSP_GPIO_H
#ifdef _cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "gpio.h"

#define BSPGPIO_MAX_INSTS 140 // 最多支持140个GPIO实例

// GPIO模式
typedef enum 
{
    BSPGPIO_MODE_Input = GPIO_MODE_INPUT,
    BSPGPIO_MODE_ANALOG = GPIO_MODE_ANALOG,

    BSPGPIO_MODE_Output_PP = GPIO_MODE_OUTPUT_PP,
    BSPGPIO_MODE_Output_OD = GPIO_MODE_OUTPUT_OD,

    BSPGPIO_MODE_AF_PP = GPIO_MODE_AF_PP,
    BSPGPIO_MODE_AF_OD = GPIO_MODE_AF_OD,

    BSPGPIO_MODE_IT_RISING = GPIO_MODE_IT_RISING,
    BSPGPIO_MODE_IT_FALLING = GPIO_MODE_IT_FALLING,
    BSPGPIO_MODE_IT_RISING_FALLING = GPIO_MODE_IT_RISING_FALLING,

    BSPGPIO_MODE_EVT_RISING = GPIO_MODE_EVT_RISING,
    BSPGPIO_MODE_EVT_FALLING = GPIO_MODE_EVT_FALLING,
    BSPGPIO_MODE_EVT_RISING_FALLING = GPIO_MODE_EVT_RISING_FALLING
} BspGpio_Mode;

// GPIO上下拉类型
typedef enum 
{
    BSPGPIO_NOPULL = GPIO_NOPULL,
    BSPGPIO_PULLUP = GPIO_PULLUP,
    BSPGPIO_PULLDOWN = GPIO_PULLDOWN

} BspGpio_Pull;

//GPIO速度
typedef enum
{
    BSPGPIO_SPEED_FREQ_LOW = GPIO_SPEED_FREQ_LOW,
    BSPGPIO_SPEED_FREQ_MEDIUM = GPIO_SPEED_FREQ_MEDIUM,
    BSPGPIO_SPEED_FREQ_HIGH = GPIO_SPEED_FREQ_HIGH,
    BSPGPIO_SPEED_FREQ_VERY_HIGH = GPIO_SPEED_FREQ_VERY_HIGH
}BspGpio_Speed;

//GPIO引脚电平
typedef enum
{
    BSPGPIO_PIN_RESET = GPIO_PIN_RESET,     //GPIO_PIN_RESET = 0
    BSPGPIO_PIN_SET = GPIO_PIN_SET
}BspGpio_PinState;

/// @brief BSPGPIO实例定义
typedef struct
{
    GPIO_TypeDef *GPIO_Port;  //GPIO初始化结构体指针
    uint32_t Pin;                //引脚号
    BspGpio_Mode Mode;           //引脚模式
    BspGpio_Pull Pull;           //上下拉
    uint32_t Speed;              //速度
    uint32_t Alternate;          //复用功能

} BspGpio_Instance;
typedef BspGpio_Instance *BspGpio_InitTypeDef;

/****** 函数 ******/
void BspGpio_InstRegist(BspGpio_Instance *inst, GPIO_TypeDef *GPIO_Init, uint32_t pin,
                        BspGpio_Mode mode, BspGpio_Pull pull, uint32_t speed, uint32_t alternate); 
void BspGpio_Init(BspGpio_Instance *inst);   

BspGpio_PinState BspGpio_ReadPin(BspGpio_Instance *inst);
void BspGpio_WritePin(BspGpio_Instance *inst, BspGpio_PinState PinState);
void BspGpio_TogglePin(BspGpio_Instance *inst);
void BspGpio_LockPin(BspGpio_Instance *inst);

typedef void (*BspGpio_ExtiHandler)(uint16_t pin);

// BSP层定义一个注册函数接口，上层应用通过这个函数将自己的逻辑函数注册进来
void BspGpio_ExtiHandlerRegist(uint16_t pin, BspGpio_ExtiHandler handler_func);
void BspGpio_EXTI_Callback(uint16_t GPIO_Pin);
static uint8_t BspGpio_GetPinIndex(uint16_t GPIO_Pin);



#ifdef _cplusplus
}
#endif
#endif
