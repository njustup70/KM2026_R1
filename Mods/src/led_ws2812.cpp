#include "led_ws2812.hpp"
#include "arm_math.h"
#include "bsp_dwt.h"

#define MaxLedNums 16

static LedWs2812* targ_led;
uint32_t WS2812buf2send[MaxLedNums][24] = {0};

void LedWs2812::Init(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t LedNums)
{
    // 记录所用的PWM通道，灯数
    this->htim = htim;
    this->Channel = Channel;
    this->LedNums = LedNums;

    for(int i = 0; i < MaxLedNums ; i++)
    {
        SetColor(i, 0, 0, 0);
    }

    // 初始化PWM
    PwmMaxValue = __HAL_TIM_GET_AUTORELOAD(this->htim);
    // HIGH_WS2812 = PwmMaxValue * 0.67f;    // PWM高电平数值
    // LOW_WS2812 = PwmMaxValue * 0.33f;           // PWM低电平数值   
    HIGH_WS2812 = PwmMaxValue * 0.67f;    // PWM高电平数值
    LOW_WS2812 = PwmMaxValue * 0.33f;           // PWM低电平数值   

    // 注册
    targ_led = this;
}


/**
 * @brief 设置某个LED的颜色
 * @warning 输入的RGB为0~255范围
 */
void LedWs2812::SetColor(int8_t Led_id, uint8_t R, uint8_t G, uint8_t B)
{
    // 不允许 Led_id 大于 实际拥有的灯数
    while(Led_id > (LedNums - 1))   Led_id -= LedNums;
    while(Led_id < 0)   Led_id += LedNums;


    // 向 数组 覆写颜色
	int i = 0;
	for(i=0;i<8;i++) WS2812buf2send[Led_id][i]   = ( G & (1 << (7 -i))? (HIGH_WS2812):LOW_WS2812 ); 
	for(i=8;i<16;i++) WS2812buf2send[Led_id][i]  = ( R & (1 << (15-i))? (HIGH_WS2812):LOW_WS2812 ); 
	for(i=16;i<24;i++) WS2812buf2send[Led_id][i] = ( B & (1 << (23-i))? (HIGH_WS2812):LOW_WS2812 ); 
}

/**
 * @brief 两个颜色的流动渐变模式（改进版：每个LED独立循环并保持固定相位差）
 * @param color_0 颜色1
 * @param color_1 颜色2
 * @param resolu 渐变流动的分辨率（值越大流动越平滑）
 * @param period 流动周期，单位秒
 */
void LedWs2812::GradientFlow(Color color_0, Color color_1, float period)
{
    // 记录已运行时间
    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= period) {
        fmod(RuntimeCnt, period);
    }

    // 计算每个LED之间的相位差，确保整体效果连续
    float phaseStep = (float)period / LedNums;
    
    
    for (int i = 0; i < LedNums; i++)
    {
        // 计算当前LED的相位（带偏移）
        float phase = fmod(RuntimeCnt + (i * phaseStep), period);
        
        // 将相位转换为0-1之间的比例（0表示颜色1，1表示颜色2）
        // 使用正弦曲线让过渡更平滑，也可以改用线性过渡：float ratio = (float)phase / period;
        float ratio = 0.5f * (1.0f + sinf(2 * 3.1415 * phase / period - 1.5708));
        
        // 根据比例计算当前位置的颜色（在两种颜色间插值）
        Color color_targ = (color_0 + (color_1 - color_0) * ratio) * 0.25f;

        // 设置当前位置的颜色（亮度调整）
        SetColor(i, color_targ.r, color_targ.g, color_targ.b);
    }
}


/// @brief 直接填充灯带颜色
/// @param R 
/// @param G 
/// @param B 
void LedWs2812::Fill(uint8_t R, uint8_t G, uint8_t B)
{
    for(int i = 0; i < LedNums ; i++)
    {
        SetColor(i, R, G, B);
    }
}


/**
 * @brief 发送数据，更新LED的颜色（使用PWM + DMA）
 * @details 对于某一个LED的RGB信息，共24bit，每个bit发送：{0 ：取占空比33%，1：取占空比66%}
 * @warning PWM DMA被配置为byte了
 */
void LedWs2812::SendData()
{
    htim->Instance->CNT = htim->Instance->ARR - 1;
    HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)WS2812buf2send, LedNums * 24 + 1);
}


/**
 * @brief 覆写PWM + DMA发送完成中断
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

    if(htim == targ_led->htim)
    {
        HAL_TIM_PWM_Stop_DMA(targ_led->htim, targ_led->Channel);
    }
}