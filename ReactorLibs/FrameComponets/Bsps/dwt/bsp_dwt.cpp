/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @version V1.1.0
 * @note 本库来源于湖南大学“跃鹿战队”开源电控库，为其开源项目的一部分
 */

#include "bsp_dwt.hpp"
#include "bsp_log.hpp"

static DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;      // DWT计数器溢出的次数
static uint32_t CYCCNT_LAST;
static uint64_t CYCCNT64;

/**
 * @brief 将等待时间换算成CYCCNT tick，并限制在单圈32位差值可表达的范围内
 * @note DWT_Delay / DWT_DelayMs 内部都是通过 `(uint32_t)(now - start)` 来判断是否到时
 *       这种写法只在“目标tick数不超过 uint32_t 一整圈”时成立
 *       因此这里统一做两件事：
 *       1. 非法或非正数参数直接按0处理，避免无意义等待
 *       2. 超过单圈上限时钳制到 UINT32_MAX，避免比较目标超过可达范围后死循环
 *
 * @param wait 输入等待值，单位由 cpu_freq 决定
 * @param cpu_freq 对应单位下每秒/每毫秒的tick频率
 * @return uint32_t 可安全用于32位差值比较的等待tick数
 */
static uint32_t DWT_GetWaitTicks(double wait, uint32_t cpu_freq)
{
    // 小于等于0的等待没有实际意义，直接返回0 tick
    if (wait <= 0.0)
    {
        return 0u;
    }

    // 使用double先做乘法，避免float在大数区间过早损失精度
    double wait_ticks = wait * (double)cpu_freq;

    // 目标tick超过32位单圈差值的表达范围时，钳到最大安全值
    if (wait_ticks >= (double)UINT32_MAX)
    {
        return UINT32_MAX;
    }

    // 这里保留向下取整语义，保证实际等待不会因为进位而越过请求上界
    return (uint32_t)wait_ticks;
}

/**
 * @brief 用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_RountCount
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
 *
 * @todo 更好的方案是为dwt的时间更新单独设置一个任务?
 *       不过,使用dwt的初衷是定时不被中断/任务等因素影响,因此该实现仍然有其存在的意义
 *
 */
void DWT_CntUpdate(void)
{
    // 线程锁，防止多线程同时访问
    static volatile uint8_t bit_locker = 0;

    // 判断dwt有没有被锁定
    if (!bit_locker)
    {
        bit_locker = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT;        // 获取当前计数值

        // 当前计数值小于上一次的计数值，说明发生了溢出
        if (cnt_now < CYCCNT_LAST)
            CYCCNT_RountCount++;        // 溢出则轮次计数加1

        // 更新上一次计数值（使用已经获取的cnt_now，防止两次读取之间发生溢出）
        CYCCNT_LAST = cnt_now;
        bit_locker = 0;
    }
}


/**
 * @brief 初始化DWT,传入参数为CPU频率,单位MHz
 * @param CPU_Freq_MHz C板为168MHz,A板为180MHz
 */
void DWT_Init(uint32_t CPU_Freq_MHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_MHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;

    DWT_CntUpdate();

    BspLog_LogOK("[DWT]: Init at %d MHz\r\n", CPU_Freq_MHz);
}


/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s
 * @param cnt_last 上一次调用的时间戳（需要在外部调用点处维护）
 */
float DWT_GetDeltaTime(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CntUpdate();

    return dt;
}
/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s（精度更高的版本）
 */
double DWT_GetDeltaTime64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CntUpdate();

    return dt;
}


/**
 * @brief 更新全局系统时间
 */
void DWT_SysTimeUpdate(void)
{
    static uint64_t CNT_ms, CNT_us;

    // 检测是否发生溢出，如果是，更新轮数
    DWT_CntUpdate();
    
    // 获取当前计数值和轮次
    volatile uint32_t cnt_now = DWT->CYCCNT;
    uint32_t current_round = CYCCNT_RountCount;
    
    // 假设在DWT_CntUpdate执行完到这里获取计时器的时间里又发生了一次新的溢出
    // 此时 DWT_CntUpdate 没有被调用，所以 CYCCNT_RountCount 并未增加，而 cnt_now 会变小
    if (cnt_now < CYCCNT_LAST)
    {
        current_round++;
    }

    // 计算当前的系统时间（单位是晶振tick），一轮为 1ULL << 32 tick (由于包含 0 到 UINT32_MAX 的跳变)
    CYCCNT64 = (uint64_t)current_round * (1ULL << 32) + (uint64_t)cnt_now;

    // 经过的秒数为：系统时间（单位是tick）除以CPU频率（整除）
    SysTime.s = CYCCNT64 / CPU_FREQ_Hz;

    // 计算剩余的毫秒数（强制以64位整数类型计算乘法防溢出截断）
    CNT_ms = CYCCNT64 - ((uint64_t)SysTime.s * CPU_FREQ_Hz);
    SysTime.ms = CNT_ms / CPU_FREQ_Hz_ms;

    // 计算剩余的微秒数
    CNT_us = CNT_ms - ((uint64_t)SysTime.ms * CPU_FREQ_Hz_ms);
    SysTime.us = CNT_us / CPU_FREQ_Hz_us;
}

float DWT_GetTimeline_Sec(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timeline = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timeline;
}

float DWT_GetTimeline_MSec(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}


uint64_t DWT_GetTimeline_USec(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

void DWT_Delay(float Sec)
{
    uint32_t tickstart = DWT->CYCCNT;
    // 先把秒换算成安全的tick目标，避免在while里重复做浮点乘法和隐式类型比较
    uint32_t wait_ticks = DWT_GetWaitTicks((double)Sec, CPU_FREQ_Hz);

    // 这里依赖uint32_t减法回绕特性做忙等待，但前提是 wait_ticks 不超过单圈上限
    while ((uint32_t)(DWT->CYCCNT - tickstart) < wait_ticks)
        ;
}

void DWT_DelayMs(float MilSec)
{
    uint32_t tickstart = DWT->CYCCNT;
    // 毫秒版本与秒版本保持同一套安全策略，只是换算频率改为每毫秒tick数
    uint32_t wait_ticks = DWT_GetWaitTicks((double)MilSec, CPU_FREQ_Hz_ms);

    // 纯uint32_t比较可避免把float精度问题带进循环条件
    while ((uint32_t)(DWT->CYCCNT - tickstart) < wait_ticks)
        ;
}
