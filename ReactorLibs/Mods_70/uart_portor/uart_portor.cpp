/**
 * @file uart_portor.cpp
 */

#include "uart_portor.hpp"
#include "bsp_log.hpp"
#include <string.h>

// ================= 底层分发中心 (DispatcherHub) ================= 
// 【设计说明】
// 由于一个硬件串口 (如 huart3) 只能向 BspUart 注册一个接收回调，
// 当有多个 Mod (如电机控制、自定义通信等) 都想独占并使用该串口时会冲突。
// 为此引入了 "枢纽 (Hub)" 的概念：
// - 每个实际的硬件串口 (如 huart3) 在这里对应唯一一个 DispatcherHub。
// - 只有这个 Hub 实体去占用 BspUart 的那个唯一回调槽。
// - 多个想复用 huart3 的 Mod，其实只是在这个 Hub 下面的 Subscription(订阅表) 里挂了个号（指定自己的 port_id）。
// - 当底层真正的串口回调触发时，Hub 遍历接收到的数据，按信封里的 port_id 把数据分发给不同的 Subscription。

/// @brief 本系统支持同时被代理的硬件串口最大数量 (如只用到USART1和3，设为2即可)
static constexpr uint8_t MAX_UART_HUBS = 6;

/// @brief 每个硬件串口(Hub)下，最多允许挂载多少个订阅者(Mod)
static constexpr uint8_t MAX_PORTS_PER_HUB = 16;

/// @brief 订阅者记录
/// 记录了哪个端口号 (port_id) 绑定了哪个回调函数 (cb)
struct Subscription {
    uint8_t port_id;
    PortRxCallback cb;
};

/**
 * @brief 串口分发枢纽
 * @note 唯一对应一个 BspUart 实例
 */
struct DispatcherHub {
    BSP::UART::UartID uart_id;                  // 绑定的实际硬件标识
    BSP::UART::Handler uart_handler;            // 该串口的 BspUart 实例句柄
    Subscription subs[MAX_PORTS_PER_HUB];       // 挂载在该串口下的订阅表
    uint8_t sub_count;                          // 当前订阅者数量
};

/// @brief 全局枢纽数组。程序中每申请共享一个新的硬件串口，这里就占用一个元素
static DispatcherHub hubs[MAX_UART_HUBS];
/// @brief 当前已分配的枢纽数量
static uint8_t hub_count = 0;

static DispatcherHub* FindOrCreateHub(BSP::UART::UartID uart_id);
static void UartPortor_General_RxCallback(BSP::UART::UartID huart, uint8_t *rxData, uint8_t size);

// ================= UartPortor 代理方法 ================= 

bool UartPortor::Init(BSP::UART::UartID uart_id, uint8_t my_port_id, PortRxCallback callback)
{
    // 确认参数非空
    if (uart_id == nullptr || callback == nullptr) {
        return false;
    }

    // 获取或创建该 硬件串口 对应的 底层枢纽
    DispatcherHub *hub = FindOrCreateHub(uart_id);
    if (!hub) {
        BspLog_LogError("[UartPortor] Failed to get or create Hub for uart_id.");
        return false;
    }

    // 2. 检查端口是否已被注册
    for (uint8_t i = 0; i < hub->sub_count; ++i) {
        if (hub->subs[i].port_id == my_port_id) {
            BspLog_LogWarning("[UartPortor] Port %d is already registered on this huart.", my_port_id);
            // 这里可以选择更新回调或者报错返回，暂且允许覆盖更新
            hub->subs[i].cb = callback;
            this->uart_id_ = uart_id;
            this->my_port_id_ = my_port_id;
            return true;
        }
    }

    // 3. 注册新端口
    if (hub->sub_count >= MAX_PORTS_PER_HUB) {
        BspLog_LogError("[UartPortor] Too many ports registered on this huart.");
        return false;
    }

    hub->subs[hub->sub_count].port_id = my_port_id;
    hub->subs[hub->sub_count].cb = callback;
    hub->sub_count++;

    this->uart_id_ = uart_id;
    this->my_port_id_ = my_port_id;
    return true;
}

void UartPortor::Send(const uint8_t *payload, uint8_t len)
{
    if (!uart_id_ || !payload || len == 0 || len > UART_PORTOR_MAX_PAYLOAD) {
        return;     // 代理尚未初始化或参数非法
    }

    // 查找由于发包需要底层句柄的中心枢纽
    DispatcherHub *hub = nullptr;
    for (uint8_t i = 0; i < hub_count; ++i) {
        if (hubs[i].uart_id == uart_id_) {
            hub = &hubs[i];
            break;
        }
    }
    if (!hub || !hub->uart_handler.IsValid()) {
        return;
    }

    // 组装信封帧 [SOF | PORT_ID | DATA_LEN | PAYLOAD... | CHECKSUM]
    uint8_t frame_buf[256];
    frame_buf[0] = UART_PORTOR_SOF;
    frame_buf[1] = my_port_id_;
    frame_buf[2] = len;
    
    memcpy(&frame_buf[3], payload, len);

    // 计算校验和
    uint8_t chk = 0;
    for (int i = 0; i < 3 + len; ++i) {
        chk ^= frame_buf[i];
    }
    frame_buf[3 + len] = chk;

    // 发送
    hub->uart_handler.Transmit(frame_buf, len + 4);
}

// ================= 底层实现与解析 ================= 

static DispatcherHub* FindOrCreateHub(BSP::UART::UartID uart_id)
{
    // 先查找是否已有对应的 Hub
    for (uint8_t i = 0; i < hub_count; ++i) {
        if (hubs[i].uart_id == uart_id) {
            return &hubs[i];
        }
    }

    // 如果没有，且数量未满，则创建新的
    if (hub_count >= MAX_UART_HUBS) {
        return nullptr;
    }

    DispatcherHub *new_hub = &hubs[hub_count];
    new_hub->uart_id = uart_id;
    new_hub->sub_count = 0;
    
    // 申请 BspUart 实例并注册单一的回调（BspUart本身要求只注册一次）
    new_hub->uart_handler = BSP::UART::Apply(uart_id);
    new_hub->uart_handler.RegisterRx(BSP_UART_RX_BUF_SIZE, UartPortor_General_RxCallback);

    hub_count++;
    return new_hub;
}

static void UartPortor_General_RxCallback(BSP::UART::UartID huart, uint8_t *rxData, uint8_t size)
{
    // 1. 查找对应的 Hub
    DispatcherHub *hub = nullptr;
    for (uint8_t i = 0; i < hub_count; ++i) {
        if (hubs[i].uart_id == huart) {
            hub = &hubs[i];
            break;
        }
    }
    if (!hub) return;

    // 2. 遍历缓冲区解析帧 
    // DMA 空闲中断产生时，rxData 包含所有这段时间收到的包。
    // 我们简单遍历扫视 [SOF ... CHK] 格式即可
    uint16_t i = 0;
    while (i + 4 <= size) { // 最小有效帧长度为4字节 (SOF Port Len CHK, payload=0)
        if (rxData[i] == UART_PORTOR_SOF) {
            uint8_t port_id = rxData[i + 1];
            uint8_t len = rxData[i + 2];

            // 判断长度是否越界
            if (i + 3 + len < size) {
                // 校验 checksum
                uint8_t expect_chk = rxData[i + 3 + len];
                uint8_t calc_chk = 0;
                for (int j = 0; j < 3 + len; ++j) {
                    calc_chk ^= rxData[i + j];
                }

                if (calc_chk == expect_chk) {
                    // 校验通过，路由给对应的注册端口
                    for (uint8_t sub = 0; sub < hub->sub_count; ++sub) {
                        if (hub->subs[sub].port_id == port_id) {
                            if (hub->subs[sub].cb) {
                                hub->subs[sub].cb(&rxData[i + 3], len);
                            }
                            break; // 找到后直接结束当前包查找
                        }
                    }
                    // 本包完整解析，直接跳过此包
                    i += (4 + len);
                    continue; 
                }
            }
        }
        // 如果不是 SOF，或是 SOF 但校验不通过/长度越界，游标前进一格
        i++;
    }
}
