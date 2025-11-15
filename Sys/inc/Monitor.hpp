#ifndef MONITOR_HPP
#define MONITOR_HPP

#include "msg_coder.hpp"
#include "typeinfo"
#include "stdarg.h"

/**
 * @brief 用于监控机器人的各项状态，还有调试、日志等功能]
 * @warning 本类也属于：单例
 */
class Monitor
{
private:
    typedef enum
    {
        track_uint8,
        track_int8,
        track_uint16,
        track_int16,
        track_uint32,
        track_int32,
        track_float,
    }Track_t;

    typedef struct
    {
        void *track_addr;       // 变量地址
        void *buf_addr;         // 将其变量值存入的缓存地址
        uint8_t bytes;          // 变量的字节数
    }MonitorLinkage;

    /// @brief 最多监测32个Bool
    static byte watch_buf[4];

    /// @brief 最多追踪32Byte的数据
    static byte track_buf[32];

    /// @brief 最多向Vofa发送64Byte数据
    static byte vofa_buf[64];

    // 变量的地址和类型存储在这里
    void* track_list[8];
    Track_t track_type[8];

    uint8_t watch_count = 0;
    uint8_t track_count = 0;

public:
    Monitor(){};
    ~Monitor(){};

    UartMsgCoder host_coder;            // 发送到上位机的编码器
    UartMsgCoder farcon_coder;          // 发送到遥控器的编码器

    /**
     * @brief 监视器的初始化函数
     * @note 上位机有两种情况，一种是视觉组的工控机，一种是调试时候的电脑
     */
    void Init(UART_HandleTypeDef *huart_host, UART_HandleTypeDef *huart_farc, bool vofa_mode = false);

    /**
     * @brief 运行方法
     * @note 该方法应被周期性调用，以处理监控任务
     * @details 内含发送日志信息、发送机器人状态码、监控维护模块等功能
     */
    void Run();

    /// @brief 发送日志
    void Log(const char* format, ...);

    /// @brief 发送警告
    void LogWarning(const char* format, ...);

    /// @brief 发送错误
    void LogError(const char* format, ...);

    /// @brief 监控某个模块的状态变化
    void Watch(bool& targ_status, bool is_neccessary = false);

    /**
     * @brief 跟踪某个变量
     * @note 调用本函数后，其将被编码并加入机器人状态码中
     * @param track_hz 跟踪频率，单位Hz，最大为200Hz
     */
    template <typename T>
    void Track(T& targ);

    /// @brief 发送监控信息
    void LogWatch();

    /// @brief 发送跟踪信息
    void LogTrack();
};



#endif