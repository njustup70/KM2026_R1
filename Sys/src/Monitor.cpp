#include "Monitor.hpp"
#include "stdarg.h"
#include "stdio.h"

const static char error_head[7] = "[ERR]:";
const static char warning_head[7] = "[WRN]:";
const static char log_head[7] = "[LOG]:";

const static uint8_t LogPort = 0x01;    // 日志使用的端口号
const static uint8_t WatchPort = 0x02;  // 监视使用的端口号
const static uint8_t TrackPort = 0x03;  // 跟踪使用的端口号

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
        LogError("Monitor: Unknown Track type!\n");
        return;
    }

    // 存储其地址
    track_list[track_count] = (void*)&targ;

    track_count++;
}





static byte track_send_buf[64];
/**
 * @brief 发送监控信息
 * @note 先用着，写得草率点，后面再改
 */
void Monitor::LogTrack()
{
    memset(track_send_buf, 0, 64);

    for(int i = 0; i < track_count; i++)
    {
        switch (track_type[i])
        {
            case track_uint8:
            {
                uint8_t var = *(uint8_t*)track_list[i];
                sprintf((char*)track_send_buf, "%d", i, var);
                break;
            }
            case track_int8:
            {
                int8_t var = *(int8_t*)track_list[i];
                sprintf((char*)track_send_buf, "%d", i, var);
                break;
            }
            case track_uint16:
            {
                uint16_t var = *(uint16_t*)track_list[i];
                sprintf((char*)track_send_buf, "%d", i, var);
                break;
            }
            case track_int16:
            {
                int16_t var = *(int16_t*)track_list[i];
                sprintf((char*)track_send_buf, "%d", i, var);
                break;
            }
            case track_uint32:
            {
                uint32_t var = *(uint32_t*)track_list[i];
                sprintf((char*)track_send_buf, "%d", i, var);
                break;
            }
            case track_int32:
            {
                int32_t var = *(int32_t*)track_list[i];
                sprintf((char*)track_send_buf, "%d", i, var);
                break;
            }
            case track_float:
            {
                float var = *(float*)track_list[i];
                sprintf((char*)track_send_buf, "%.2f", i, var);
                break;
            }
        }
        // 添加逗号","
        if(i != track_count - 1)
        {
            size_t len = strlen((char*)track_send_buf);
            track_send_buf[len] = ',';
            track_send_buf[len + 1] = '\0';
        }
        else // 结束时添加换行符"\n"
        {
            size_t len = strlen((char*)track_send_buf);
            track_send_buf[len] = '\n';
            track_send_buf[len + 1] = '\0';
        }
    }

    // 发送编码后的数据
    host_coder.SendEncodedMsg(track_send_buf, strlen((char*)track_send_buf));   
}

/**
 * @brief 发送错误日志
 * @note 默认不向遥控器发送错误日志，只向上位机发送
 * @warning 发送长度不超过64字节
 */
void Monitor::LogError(const char* format, ...)
{
    // 解析可变参数列表
    va_list args;
    va_start(args, format);
    char log_buf[70];
    
    // 填充错误头
    memcpy(log_buf, error_head, 6);
    vsnprintf(log_buf + 6, 64, format, args);
    va_end(args);

    // 发送日志到上位机
    host_coder.SendMsg(0x01, (uint8_t*)log_buf, strlen(log_buf));
}

/**
 * @brief 发送警告日志
 * @note 同上
 */
void Monitor::LogWarning(const char* format, ...)
{
    // 解析可变参数列表
    va_list args;
    va_start(args, format);
    char log_buf[70];
    
    // 填充警告头
    memcpy(log_buf, warning_head, 6);
    vsnprintf(log_buf + 6, 64, format, args);
    va_end(args);

    // 发送日志到上位机
    host_coder.SendMsg(0x01, (uint8_t*)log_buf, strlen(log_buf));
}

/**
 * @brief 发送日志
 * @note 同上
 */
void Monitor::Log(const char* format, ...)
{
    // 解析可变参数列表
    va_list args;
    va_start(args, format);
    char log_buf[70];
    
    // 填充日志头
    memcpy(log_buf, log_head, 6);
    vsnprintf(log_buf + 6, 64, format, args);
    va_end(args);

    // 发送日志到上位机
    host_coder.SendMsg(0x01, (uint8_t*)log_buf, strlen(log_buf));
}


void Monitor::Init(UART_HandleTypeDef *huart_host, UART_HandleTypeDef *huart_farc, bool vofa_mode)
{
    host_coder.Init(huart_host);
    farcon_coder.Init(huart_farc);
}