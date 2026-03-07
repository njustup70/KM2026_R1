#include "bsp_can.hpp"
#include "bsp_halport.hpp"
#include "bsp_dwt.hpp"
#include "bsp_log.hpp"
#include "string.h"

using namespace BSP::CAN;



// ---- 辅助函数 ----

static inline const char* GetFileName(const char* path) {
    const char* filename = path;
    for (const char* p = path; *p; ++p) {
        if (*p == '/' || *p == '\\') {
            filename = p + 1;
        }
    }
    return filename;
}

// ---- 统计变量 ----

uint32_t can_recv_times = 0;
uint32_t can_busy_times = 0;

// ---- 过滤器组资源管理 ----

static uint8_t can1_filter_idx = 0;   // CAN1过滤器组索引（0-13）
static uint8_t can2_filter_idx = 14;  // CAN2过滤器组索引（14-27）

// ---- 实例管理 ----

static HandleTypeDef bspcan_insts[BSPCAN_MAX_CANINSTS] = {nullptr};
static int bspcan_inst_count = 0;

// 记录已经激活的CAN总线
static CanID activated_cans[3] = {nullptr, nullptr, nullptr};

// ---- 过滤器配置（内部函数） ----

static int FilterAdd(HandleTypeDef inst)
{
    if (inst == nullptr || inst->can_id == nullptr)
    {
        return -1;
    }

    CAN_HandleTypeDef* hcan = reinterpret_cast<CAN_HandleTypeDef*>(inst->can_id);
    uint8_t filter_bank;

#ifdef USE_REAL_HAL
    CAN_FilterTypeDef FilterConfig = {0};

    // 分配过滤器组
    if (hcan->Instance == CAN1)
    {
        if (can1_filter_idx > 13) return -1;
        filter_bank = can1_filter_idx++;
    }
    else if (hcan->Instance == CAN2)
    {
        if (can2_filter_idx > 27) return -1;
        filter_bank = can2_filter_idx++;
    }
    else
    {
        return -1;
    }

    // 配置过滤器ID和掩码
    if (inst->rx_is_ext) // 扩展帧配置（29位ID）
    {
        FilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
        FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        FilterConfig.FilterIdHigh = (inst->rx_id >> 13) & 0xFFFF;
        FilterConfig.FilterIdLow = ((inst->rx_id << 3) | 0x04) & 0xFFFF;
    }
    else // 标准帧配置（11位ID）
    {
        FilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
        FilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
        FilterConfig.FilterIdLow = inst->rx_id << 5;
        FilterConfig.FilterIdHigh = 0x0000;
    }

    // FIFO分配
    FilterConfig.FilterFIFOAssignment = (inst->rx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    FilterConfig.SlaveStartFilterBank = 14;
    FilterConfig.FilterBank = filter_bank;
    FilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(hcan, &FilterConfig) != HAL_OK)
    {
        return -1;
    }
#else
    // Pure-Fram模式下，仅靠指针地址区分，避免引用Instance
    static CanID first_can = nullptr;
    if (first_can == nullptr) first_can = inst->can_id;

    if (inst->can_id == first_can)
    {
        if (can1_filter_idx > 13) return -1;
        filter_bank = can1_filter_idx++;
    }
    else
    {
        if (can2_filter_idx > 27) return -1;
        filter_bank = can2_filter_idx++;
    }
    
    // 假人调用消除未使用警告
    (void)hcan;
    (void)filter_bank;
#endif

    return 0;
}

// ---- CAN 总线激活（内部函数） ----

static void TryActivateCan(CanID can)
{
    // 检查是否已激活
    for (int i = 0; i < 3; i++)
    {
        if (activated_cans[i] == can) return; // 已经激活过
    }

    // 记录为已激活
    for (int i = 0; i < 3; i++)
    {
        if (activated_cans[i] == nullptr)
        {
            activated_cans[i] = can;
            break;
        }
    }

    CAN_HandleTypeDef* hcan = reinterpret_cast<CAN_HandleTypeDef*>(can);
    
    #ifdef USE_REAL_HAL
        HAL_CAN_Start(hcan);
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    #else
        (void)hcan;
    #endif
}

// ---- 公开接口：注册 ----

void BSP::CAN::Regist(HandleTypeDef inst, CanID can, uint32_t rx_id,
                       uint8_t rx_is_ext, RxCallback callback,
                       const char* file, int line)
{
    if (inst == nullptr || can == nullptr)
    {
        BspLog_LogError("[BspCAN] Regist failed: null inst or can! [%s:%d]\n", GetFileName(file), line);
        return;
    }

    // 初始化实例
    inst->can_id = can;
    inst->rx_id = rx_id;
    inst->rx_is_ext = rx_is_ext;
    inst->rx_callback = callback;

    // 添加过滤器
    if (FilterAdd(inst) != 0)
    {
        BspLog_LogError("[BspCAN] FilterAdd failed! [%s:%d]\n", GetFileName(file), line);
        return;
    }

    // 尝试激活该特定CAN外设
    TryActivateCan(can);

    // 记录实例
    if (bspcan_inst_count < BSPCAN_MAX_CANINSTS)
    {
        bspcan_insts[bspcan_inst_count++] = inst;
    }
    else
    {
        BspLog_LogError("[BspCAN] Too many instances! [%s:%d]\n", GetFileName(file), line);
    }
}

// ---- 公开接口：发送 ----

void BSP::CAN::Transmit(CanID can, uint32_t id, uint8_t is_ext,
                         const uint8_t* data, uint8_t len,
                         uint16_t timeout_ms)
{
    if (can == nullptr || data == nullptr) return;

    CAN_HandleTypeDef* hcan = reinterpret_cast<CAN_HandleTypeDef*>(can);
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = id;
    TxHeader.ExtId = id;
    TxHeader.IDE = is_ext ? CAN_ID_EXT : CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = len;

    float time_start = DWT_GetTimeline_MSec();

    while (HAL_CAN_AddTxMessage(hcan, &TxHeader, (uint8_t*)data, &TxMailbox) != HAL_OK)
    {
        if (DWT_GetTimeline_MSec() - time_start > timeout_ms)
        {
            can_busy_times++;
            return;
        }
    }
}

const char* BSP::CAN::GetName(CanID can)
{
    if (can == Hardware::hcan_main) return "hcan_main";
    if (can == Hardware::hcan_sub) return "hcan_sub";
    return "hcan_unknown";
}

// ---- 中断回调路由 ----

static void RxCallbackRouter(CAN_HandleTypeDef* hcan, uint32_t fifo)
{
    CAN_RxHeaderTypeDef halRxHeader;
    uint8_t rxData[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, fifo, &halRxHeader, rxData) != HAL_OK)
    {
        return;
    }

    can_recv_times++;

    // 构建纯净 RxHeader
    RxHeader header;
    header.is_ext = (halRxHeader.IDE == CAN_ID_EXT);
    header.id = header.is_ext ? halRxHeader.ExtId : halRxHeader.StdId;
    header.dlc = halRxHeader.DLC;
    header.is_remote = (halRxHeader.RTR == CAN_RTR_REMOTE);

    CanID canID = reinterpret_cast<CanID>(hcan);

    // 遍历所有实例，查找匹配的ID
    for (int i = 0; i < bspcan_inst_count; i++)
    {
        HandleTypeDef inst = bspcan_insts[i];
        if (inst->can_id == canID &&
            ((inst->rx_is_ext && header.is_ext && inst->rx_id == header.id) ||
             (!inst->rx_is_ext && !header.is_ext && inst->rx_id == header.id)))
        {
            if (inst->rx_callback != nullptr)
            {
                inst->rx_callback(canID, header, rxData);
            }
            return;
        }
    }
}

// ---- 覆写 HAL 中断回调 ----

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    RxCallbackRouter(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    RxCallbackRouter(hcan, CAN_RX_FIFO1);
}
