#include "StateMachine.hpp"
#include "bsp_dwt.h"
#include "stdio.h"

/**
 * @brief 为状态块添加状态链接
 */
void StateBlocks::AddLink(bool *condition, StateBlocks *nextState)
{
    if (linkNums < 16)
    {
        links[linkNums].condition = condition;
        links[linkNums].nextState = nextState;
        linkNums++;
    }
}

/**
 * @brief 进行状态转移，返回下一个状态ID
 * @return 如果没有状态转移则返回当前状态ID，否则返回下一个状态ID
 */
uint8_t StateBlocks::Transition()
{
    for (uint8_t i = 0; i < linkNums; i++)
    {
        if (*(links[i].condition))
        {
            return links[i].nextState->id;
        }
    }
    return id;
}

/**
 * @brief 为状态机添加状态块
 */
void StateCore::AddState(StateBlocks state)
{
    if (stateNums < 24)
    {
        states[stateNums] = state;
        stateNums++;
    }
}


/**
 * @brief 运行状态机
 */
void StateCore::Run()
{
    // 计算时间间隔
    dt = DWT_GetDeltaTime(&dwt_tick);

    // 执行全局状态函数
    if (GlobalAction != nullptr) GlobalAction(this);
    
    // 执行当前状态函数
    states[at_state_id].StateAction(this);

    // 进行状态转移
    at_state_id = states[at_state_id].Transition();
}


/**
 * @brief 绘制状态机图
 * @details 通过遍历整个状态机，将状态机的状态和状态转换关系以图的形式发送到指定串口上 'Mermaid' 
 * @warning 该函数会阻塞程序运行！！因此禁止在线程中调用，仅做Debug用途
 */
void StateCore::CoreGraph(BspUart_Instance uart_inst)
{
    uint8_t buf[60];
    BspUart_Transmit(uart_inst, (uint8_t *)"StateGraph\n", 11);
    HAL_Delay(10);
    for (uint8_t i = 0; i < stateNums; i++)
    {
        // 发送状态转换关系（mermaid格式）
        for (uint8_t j = 0; j < states[i].linkNums; j++)
        {
            int len = snprintf((char *)buf, 48, "%s --> %s\n", states[i].name, states[i].links[j].nextState->name);
            BspUart_Transmit(uart_inst, buf, len);
            HAL_Delay(10);
        }
    }
}