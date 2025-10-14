#ifndef _STATEMACHINE_HPP_
#define _STATEMACHINE_HPP_

#include "stm32f4xx_hal.h"
#include "bsp_uart.h"
#include "string.h"

class StateCore;        // 为了能引用，进行前向声明
class StateBlocks;

/**
 * @brief 状态链接，代表了状态之间的转换关系
 */
typedef struct 
{
    bool *condition;                // 条件
    StateBlocks *nextState;         // 下一个状态
}StateLink;


/**
 * @brief 状态块，代表了一个状态
 * @warning 状态块是少有的在构造函数中初始化的类，因此绝不能被定义成全局变量
 * 因为这会使得其在HAL库初始化之前就被初始化，从而引发不可预知的错误
 */
class StateBlocks
{
    public:
    StateBlocks(){};
    StateBlocks(const char *name, void (*StateAction)(StateCore *core))
    {
        strncpy(this->name, name, 15);
        this->name[15] = 0; // 确保字符串结尾
        this->StateAction = StateAction;
        linkNums = 0;
    };
    char name[16];                      // 状态名称
    uint8_t id;                         // 状态ID，由StateCore在AddState时赋值  
    void (*StateAction)(StateCore *core);          // 状态函数
    StateLink links[16];                 // 状态链接
    uint8_t linkNums;                   // 状态链接数量

    bool Complete;                      // 运行完成标志
    StateBlocks* incore;          // 对应在状态机内部的那个同位StateCore

    void AddLink(bool *condition, StateBlocks *nextState);
    uint8_t Transition();
};


/**
 * @brief 状态机核心，即状态机本体
 */
class StateCore
{
    public:
    StateCore(){};

    uint32_t dwt_tick;          // dwt计时器用句柄
    float dt;                   // 两次状态切换的时间间隔，单位秒
    StateBlocks states[24];     // 状态块数组（上限24个）
    uint8_t stateNums;          // 状态块数量
    uint8_t at_state_id;        // 当前状态ID [0, 23]

    void (*GlobalAction)(StateCore *core) = nullptr;          // 全局状态函数
    
    void CoreGraph(BspUart_Instance uart_inst);
    void AddState(StateBlocks *state);
    void Run(void);
};

#endif