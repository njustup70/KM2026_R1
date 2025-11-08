#include "bsp_gpio.h"
#include "main.h"
#include "gpio.h"

static BspGpio_Instance *bspgpio_insts[BSPGPIO_MAX_INSTS] = {NULL}; 
static int bspgpio_inst_count = 0; // 当前实例数量

/**
 * @brief 注册一个GPIO实例，注册的同时初始化该GPIO
 * 
 */
void BspGpio_InstRegist(BspGpio_Instance *inst, GPIO_TypeDef *GPIOx, uint32_t pin,
                        BspGpio_Mode mode, BspGpio_Pull pull, uint32_t speed, uint32_t alternate)
{
    //检验参数有效性
    if (inst == NULL || GPIOx == NULL)
    {
        return; //参数无效
    }
    //初始化实例
    inst->GPIO_Port = GPIOx;
    inst->Pin = pin;
    inst->Mode = mode;
    inst->Pull = pull;
    inst->Speed = speed;
    inst->Alternate = alternate;
}
/**
 * @brief 启用GPIO
 * 
 * @param inst 
 */
void BspGpio_Init(BspGpio_Instance *inst)
{
    GPIO_InitTypeDef BspGpio_Init = {0};
    
    // 将inst的配置数据复制到结构体中
    BspGpio_Init.Pin = inst->Pin;
    BspGpio_Init.Mode = inst->Mode;
    BspGpio_Init.Pull = inst->Pull;
    BspGpio_Init.Speed = inst->Speed;
    BspGpio_Init.Alternate = inst->Alternate;    
    
    // 调用HAL函数
    HAL_GPIO_Init(inst->GPIO_Port, &BspGpio_Init); 

    // 记录实例
    if (bspgpio_inst_count < BSPGPIO_MAX_INSTS)
    {
        bspgpio_insts[bspgpio_inst_count++] = inst;    //从0号开始
    }
}

static BspGpio_ExtiHandler exti_handlers[16] = {NULL};

// 当收到中断时，快速索引数组获取对应的处理函数
static uint8_t BspGpio_GetPinIndex(uint16_t GPIO_Pin)
{
    // 检查是否是有效的单个位掩码（例如 0x0004, 0x0020）
    if (GPIO_Pin == 0 || (GPIO_Pin & (GPIO_Pin - 1)) != 0) {
        return 16; // 无效索引
    }
    
    // 计算位的位置 (即EXTI0-15的索引)
    uint8_t index = 0;
    uint16_t temp_pin = GPIO_Pin;
    while (temp_pin > 1) // 循环直到找到设置的位
    {
        temp_pin >>= 1;
        index++;
    }
    return index;
}

void BspGpio_ExtiHandlerRegist(uint16_t pin, BspGpio_ExtiHandler handler_func)
{
    uint8_t index = BspGpio_GetPinIndex(pin);
    
    // 检查索引是否在0-15范围内
    if (index < 16)
    {
        // 将函数指针存储到静态数组中
        exti_handlers[index] = handler_func;
    }
    
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t index = BspGpio_GetPinIndex(GPIO_Pin);
    if (index < 16)
    {
        BspGpio_ExtiHandler upper_handler = exti_handlers[index];
        
        if (upper_handler != NULL)
        {
            upper_handler(GPIO_Pin);
        }
    }
}


///@brief  对HAL库的复写
BspGpio_PinState BspGpio_ReadPin(BspGpio_Instance *inst)
{
    return HAL_GPIO_ReadPin(inst->GPIO_Port, inst->Pin);
}

void BspGpio_WritePin(BspGpio_Instance *inst, BspGpio_PinState PinState)
{
    HAL_GPIO_WritePin(inst->GPIO_Port, inst->Pin, PinState);
}

void BspGpio_TogglePin(BspGpio_Instance *inst)
{
    HAL_GPIO_TogglePin(inst->GPIO_Port, inst->Pin);
}

void BspGpio_LockPin(BspGpio_Instance *inst)
{
    HAL_GPIO_LockPin(inst->GPIO_Port, inst->Pin);
}







