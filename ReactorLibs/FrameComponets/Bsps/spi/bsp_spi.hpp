#pragma once

#include <cstdint>
#include "std_math.hpp"

namespace BSP
{
namespace SPI
{
    /// @brief SPI DMA 事务状态（按总线维度管理）
    enum class DmaState : uint8_t
    {
        Idle = 0, ///< 总线空闲，可发起新事务
        Busy,     ///< DMA进行中，阻塞/其他DMA请求应返回
        Done,     ///< DMA完成，等待拥有者消费该状态
        Error     ///< DMA异常，等待拥有者消费该状态
    };

    // ---- 解耦句柄 ----
    struct OpaqueSpi;
    /// @brief 不透明指针 SPI_HandleTypeDef* 的框架解耦替代
    using SpiID = OpaqueSpi*;

    /**
     * @brief SPI 从设备抽象
     * @note 每个实例代表挂在某条SPI总线上的一个设备（可选CS）
     */
    class Device
    {
    public:
        Device() = default;
        Device(SpiID spi, Pin cs_pin = {'\0', 0});

        /// @brief 绑定总线与CS信息，并将CS拉高到空闲态
        void Init(SpiID spi, Pin cs_pin = {'\0', 0});

        /// @brief 阻塞发送（若总线DMA忙则直接返回）
        void Transmit(uint8_t* tx_data, uint16_t size);
        /// @brief 阻塞接收（若总线DMA忙则直接返回）
        void Receive(uint8_t* rx_data, uint16_t size);
        /// @brief 阻塞收发（若总线DMA忙则直接返回）
        void TransRecv(uint8_t* tx_data, uint8_t* rx_data, uint16_t size);

        /// @brief 发起DMA发送，成功后状态置Busy
        bool TransmitDMA(uint8_t* tx_data, uint16_t size);
        /// @brief 发起DMA接收，成功后状态置Busy
        bool ReceiveDMA(uint8_t* rx_data, uint16_t size);
        /// @brief 发起DMA收发，成功后状态置Busy
        bool TransRecvDMA(uint8_t* tx_data, uint8_t* rx_data, uint16_t size);

        /**
         * @brief 消费本设备对应DMA状态
         * @note 仅DMA拥有者可见Done/Error；消费后自动回到Idle并释放owner
         */
        DmaState ConsumeDmaState();

        /// @brief 手动拉低CS（若设备配置了CS）
        void Select();
        /// @brief 手动拉高CS（若设备配置了CS）
        void Deselect();

        SpiID spi_id;

    private:
        void* cs_port_;   ///< 内部以void*存GPIO_TypeDef*，保持对HAL类型解耦
        uint16_t cs_pin_; ///< 解码后的GPIO_PIN_X位掩码
        bool has_cs_;     ///< 是否配置了有效CS
    };
}
}
