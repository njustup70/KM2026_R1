#include "Monitor.hpp"
#include "stdarg.h"
#include "stdio.h"


/**
 * @brief 跟踪某个变量
 */
template <typename T>
void Monitor::Track(T& targ)
{
    using namespace std;

    // 检查追踪越界
    if (track_count >= 8)
    {
        LogError("Monitor: Track list full!\n");
        return;
    }

    // 存储其类型信息
    const type_info& targ_type = typeid(targ);
    if (targ_type == typeid(uint8_t))
    {
        track_type[track_count] = track_uint8;
    }
    else if (targ_type == typeid(int8_t))
    {
        track_type[track_count] = track_int8;
    }
    else if (targ_type == typeid(uint16_t))
    {
        track_type[track_count] = track_uint16;
    }
    else if (targ_type == typeid(int16_t))
    {
        track_type[track_count] = track_int16;
    }
    else if (targ_type == typeid(uint32_t))
    {
        track_type[track_count] = track_uint32;
    }
    else if (targ_type == typeid(int32_t))
    {
        track_type[track_count] = track_int32;
    }
    else if (targ_type == typeid(float))
    {
        track_type[track_count] = track_float;
    }
    else
    {
        LogError("Monitor: Unsupported Track type!\n");
        return;
    }

    // 存储其地址
    track_list[track_count] = (void*)&targ;

    track_count++;
}



void Monitor::LogTrack()
{

}

/**
 * @brief 发送错误日志
 * @note 默认不向遥控器发送错误日志，只向上位机发送
 */
void Monitor::LogError(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    // host_coder.SendLog(UartMsgCoder::LogError, format, args);
    va_end(args);
}


void Monitor::Init(UART_HandleTypeDef *huart_host, UART_HandleTypeDef *huart_farc, bool vofa_mode)
{
    host_coder.Init(huart_host);
    farcon_coder.Init(huart_farc);
}