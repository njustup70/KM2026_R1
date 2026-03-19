#include "bsp_spi.hpp"
#include "bsp_halport.hpp"

using namespace BSP::SPI;

// ---- 总线状态管理 ----

static constexpr uint8_t MAX_SPI_BUS_NUM = 6;

/**
 * @brief 每条SPI总线的运行态
 * @note
 * active_device:  当前正在占用CS的设备（用于DMA回调自动失活）
 * dma_owner_device: 当前DMA事务归属设备（用于状态消费权限）
 * dma_state:      该总线DMA状态机
 */
struct SpiBusState
{
    SpiID spi_id;
    Device* active_device;
    Device* dma_owner_device;
    DmaState dma_state;
};

static SpiBusState spi_buses[MAX_SPI_BUS_NUM];
static uint8_t spi_buses_count = 0;

// ---- 内部辅助函数 ----

/**
 * @brief 查找或创建对应SPI总线状态槽
 * @param id SPI总线句柄
 * @return 成功返回槽指针；槽已满时返回nullptr
 */
static SpiBusState* GetOrCreateBusState(SpiID id)
{
    for (uint8_t i = 0; i < spi_buses_count; i++)
    {
        if (spi_buses[i].spi_id == id)
        {
            return &spi_buses[i];
        }
    }

    if (spi_buses_count < MAX_SPI_BUS_NUM)
    {
        spi_buses[spi_buses_count].spi_id = id;
        spi_buses[spi_buses_count].active_device = nullptr;
        spi_buses[spi_buses_count].dma_owner_device = nullptr;
        spi_buses[spi_buses_count].dma_state = DmaState::Idle;
        spi_buses_count++;
        return &spi_buses[spi_buses_count - 1];
    }

    return nullptr;
}

/**
 * @brief 记录指定总线当前活跃设备
 * @param id SPI总线句柄
 * @param dev 当前占用片选的设备，传入nullptr表示清空
 * @note 该信息用于DMA回调阶段自动失活CS
 */
static void SetActiveDevice(SpiID id, Device* dev)
{
    SpiBusState* bus = GetOrCreateBusState(id);
    if (bus != nullptr)
    {
        bus->active_device = dev;
    }
}

/**
 * @brief 获取指定总线当前活跃设备
 * @param id SPI总线句柄
 * @return 返回当前活跃设备指针；未找到或未设置时返回nullptr
 */
static Device* GetActiveDevice(SpiID id)
{
    for (uint8_t i = 0; i < spi_buses_count; i++)
    {
        if (spi_buses[i].spi_id == id)
        {
            return spi_buses[i].active_device;
        }
    }
    return nullptr;
}

/**
 * @brief 更新指定总线DMA状态
 * @param id SPI总线句柄
 * @param state 目标DMA状态
 * @note 状态按总线维度维护，供接口层与回调层共享
 */
static void SetDmaState(SpiID id, DmaState state)
{
    SpiBusState* bus = GetOrCreateBusState(id);
    if (bus != nullptr)
    {
        bus->dma_state = state;
    }
}

/**
 * @brief 读取指定总线DMA状态
 * @param id SPI总线句柄
 * @return 当前DMA状态；未找到该总线时返回DmaState::Idle
 */
static DmaState GetDmaState(SpiID id)
{
    for (uint8_t i = 0; i < spi_buses_count; i++)
    {
        if (spi_buses[i].spi_id == id)
        {
            return spi_buses[i].dma_state;
        }
    }
    return DmaState::Idle;
}

/**
 * @brief 设置指定总线DMA事务拥有者
 * @param id SPI总线句柄
 * @param dev DMA事务所属设备，传入nullptr表示清空
 * @note 仅owner可消费Done/Error终态
 */
static void SetDmaOwnerDevice(SpiID id, Device* dev)
{
    SpiBusState* bus = GetOrCreateBusState(id);
    if (bus != nullptr)
    {
        bus->dma_owner_device = dev;
    }
}

/**
 * @brief 获取指定总线DMA事务拥有者
 * @param id SPI总线句柄
 * @return DMA事务拥有者；未找到或未设置时返回nullptr
 */
static Device* GetDmaOwnerDevice(SpiID id)
{
    for (uint8_t i = 0; i < spi_buses_count; i++)
    {
        if (spi_buses[i].spi_id == id)
        {
            return spi_buses[i].dma_owner_device;
        }
    }
    return nullptr;
}

// ---- 公开接口：构造/初始化 ----

/**
 * @brief 构造SPI设备对象并执行初始化
 * @param spi SPI总线句柄（不透明指针）
 * @param cs_pin 设备片选引脚配置；为空表示无CS管理
 */
Device::Device(SpiID spi, Pin cs_pin)
{
    this->Init(spi, cs_pin);
}

/**
 * @brief 初始化设备绑定关系与CS配置
 * @param spi SPI总线句柄（不透明指针）
 * @param cs_pin 设备片选引脚配置；为空表示无CS管理
 * @note 初始化结束后若存在有效CS会主动拉高，确保默认未选中
 */
void Device::Init(SpiID spi, Pin cs_pin)
{
    this->spi_id = spi;

    if (cs_pin.port == '\0' || cs_pin.port == 0)
    {
        this->has_cs_ = false;
        this->cs_port_ = nullptr;
        this->cs_pin_ = 0;
    }
    else
    {
        this->has_cs_ = true;
        this->cs_pin_ = (1 << cs_pin.number);

        switch (cs_pin.port)
        {
#ifdef GPIOA
            case 'A':
                this->cs_port_ = (void*)GPIOA;
                break;
#endif
#ifdef GPIOB
            case 'B':
                this->cs_port_ = (void*)GPIOB;
                break;
#endif
#ifdef GPIOC
            case 'C':
                this->cs_port_ = (void*)GPIOC;
                break;
#endif
#ifdef GPIOD
            case 'D':
                this->cs_port_ = (void*)GPIOD;
                break;
#endif
#ifdef GPIOE
            case 'E':
                this->cs_port_ = (void*)GPIOE;
                break;
#endif
#ifdef GPIOF
            case 'F':
                this->cs_port_ = (void*)GPIOF;
                break;
#endif
#ifdef GPIOG
            case 'G':
                this->cs_port_ = (void*)GPIOG;
                break;
#endif
#ifdef GPIOH
            case 'H':
                this->cs_port_ = (void*)GPIOH;
                break;
#endif
#ifdef GPIOI
            case 'I':
                this->cs_port_ = (void*)GPIOI;
                break;
#endif
            default:
                this->cs_port_ = nullptr;
                this->has_cs_ = false;
                break;
        }
    }

    // 初始化时拉高片选，确保设备默认处于未选中状态
    if (this->has_cs_ && this->cs_port_ != nullptr)
    {
#ifdef USE_REAL_HAL
        HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
#endif
    }
}

// ---- 公开接口：阻塞传输 ----

/**
 * @brief 以阻塞方式发送数据
 * @param tx_data 发送数据缓冲区
 * @param size 发送字节数
 * @note 当该总线DMA状态为Busy时直接返回，不发起传输
 */
void Device::Transmit(uint8_t *tx_data, uint16_t size)
{
#ifdef USE_REAL_HAL
    // DMA占用总线时，阻塞接口直接返回，避免与DMA事务冲突
    if (GetDmaState(this->spi_id) == DmaState::Busy)
        return;

    if (this->has_cs_)
        HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_RESET);

    HAL_SPI_Transmit((SPI_HandleTypeDef*)this->spi_id, tx_data, size, 100);

    if (this->has_cs_)
        HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
#endif
}

/**
 * @brief 以阻塞方式接收数据
 * @param rx_data 接收数据缓冲区
 * @param size 接收字节数
 * @note 当该总线DMA状态为Busy时直接返回，不发起传输
 */
void Device::Receive(uint8_t *rx_data, uint16_t size)
{
#ifdef USE_REAL_HAL
    if (GetDmaState(this->spi_id) == DmaState::Busy)
        return;

    if (this->has_cs_)
        HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_RESET);

    HAL_SPI_Receive((SPI_HandleTypeDef*)this->spi_id, rx_data, size, 100);

    if (this->has_cs_)
        HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
#endif
}

/**
 * @brief 以阻塞方式同时发送并接收数据
 * @param tx_data 发送数据缓冲区
 * @param rx_data 接收数据缓冲区
 * @param size 传输字节数
 * @note 当该总线DMA状态为Busy时直接返回，不发起传输
 */
void Device::TransRecv(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
#ifdef USE_REAL_HAL
    if (GetDmaState(this->spi_id) == DmaState::Busy)
        return;

    if (this->has_cs_)
        HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)this->spi_id, tx_data, rx_data, size, 100);

    if (this->has_cs_)
        HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
#endif
}

// ---- 公开接口：手动片选 ----

/**
 * @brief 手动选中设备（CS拉低）
 * @note 仅在设备配置有效CS时生效
 */
void Device::Select()
{
#ifdef USE_REAL_HAL
    if (this->has_cs_ && this->cs_port_ != nullptr)
    {
        HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_RESET);
    }
#endif
}

/**
 * @brief 手动失活设备（CS拉高）
 * @note 仅在设备配置有效CS时生效
 */
void Device::Deselect()
{
#ifdef USE_REAL_HAL
    if (this->has_cs_ && this->cs_port_ != nullptr)
    {
        HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
    }
#endif
}

// ---- 公开接口：DMA传输 ----

/**
 * @brief 发起DMA发送事务
 * @param tx_data 发送数据缓冲区
 * @param size 发送字节数
 * @return true 表示DMA启动成功并进入Busy；false 表示未启动
 * @note 失败路径会恢复CS并将状态标记为Error
 */
bool Device::TransmitDMA(uint8_t *tx_data, uint16_t size)
{
#ifdef USE_REAL_HAL
    if (GetDmaState(this->spi_id) == DmaState::Busy)
        return false;

    this->Select();
    HAL_StatusTypeDef status =
        HAL_SPI_Transmit_DMA((SPI_HandleTypeDef*)this->spi_id, tx_data, size);
    if (status == HAL_OK)
    {
        // DMA成功发起后绑定active和owner，并推进到Busy
        SetActiveDevice(this->spi_id, this);
        SetDmaOwnerDevice(this->spi_id, this);
        SetDmaState(this->spi_id, DmaState::Busy);
        return true;
    }

    this->Deselect();
    SetActiveDevice(this->spi_id, nullptr);
    SetDmaOwnerDevice(this->spi_id, nullptr);
    SetDmaState(this->spi_id, DmaState::Error);
    return false;
#endif
    return false;
}

/**
 * @brief 发起DMA接收事务
 * @param rx_data 接收数据缓冲区
 * @param size 接收字节数
 * @return true 表示DMA启动成功并进入Busy；false 表示未启动
 * @note 失败路径会恢复CS并将状态标记为Error
 */
bool Device::ReceiveDMA(uint8_t *rx_data, uint16_t size)
{
#ifdef USE_REAL_HAL
    if (GetDmaState(this->spi_id) == DmaState::Busy)
        return false;

    this->Select();
    HAL_StatusTypeDef status =
        HAL_SPI_Receive_DMA((SPI_HandleTypeDef*)this->spi_id, rx_data, size);
    if (status == HAL_OK)
    {
        SetActiveDevice(this->spi_id, this);
        SetDmaOwnerDevice(this->spi_id, this);
        SetDmaState(this->spi_id, DmaState::Busy);
        return true;
    }

    this->Deselect();
    SetActiveDevice(this->spi_id, nullptr);
    SetDmaOwnerDevice(this->spi_id, nullptr);
    SetDmaState(this->spi_id, DmaState::Error);
    return false;
#endif
    return false;
}

/**
 * @brief 发起DMA收发事务
 * @param tx_data 发送数据缓冲区
 * @param rx_data 接收数据缓冲区
 * @param size 传输字节数
 * @return true 表示DMA启动成功并进入Busy；false 表示未启动
 * @note 失败路径会恢复CS并将状态标记为Error
 */
bool Device::TransRecvDMA(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
#ifdef USE_REAL_HAL
    if (GetDmaState(this->spi_id) == DmaState::Busy)
        return false;

    this->Select();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
        (SPI_HandleTypeDef*)this->spi_id, tx_data, rx_data, size);
    if (status == HAL_OK)
    {
        SetActiveDevice(this->spi_id, this);
        SetDmaOwnerDevice(this->spi_id, this);
        SetDmaState(this->spi_id, DmaState::Busy);
        return true;
    }

    this->Deselect();
    SetActiveDevice(this->spi_id, nullptr);
    SetDmaOwnerDevice(this->spi_id, nullptr);
    SetDmaState(this->spi_id, DmaState::Error);
    return false;
#endif
    return false;
}

/**
 * @brief 读取并消费本设备DMA状态
 * @return 本设备可见的DMA状态；非owner恒为Idle
 * @note Done/Error 为一次性终态，消费后会复位为Idle并释放owner
 */
DmaState Device::ConsumeDmaState()
{
    // 确认当前想读这个的设备，是DMA的owner；否则判定为其他设备，始终视为Idle
    if (GetDmaOwnerDevice(this->spi_id) != this)
    {
        return DmaState::Idle;
    }

    // 读取状态
    DmaState state = GetDmaState(this->spi_id);
    
    // 只有owner设备能看到Done/Error状态
    // 一旦看到就消费掉，复位为Idle并释放owner
    if (state == DmaState::Done || state == DmaState::Error)
    {
        // Done/Error为一次性事件：消费后复位为Idle并释放owner
        SetDmaState(this->spi_id, DmaState::Idle);
        SetDmaOwnerDevice(this->spi_id, nullptr);
    }

    return state;
}

#ifdef USE_REAL_HAL
// ---- HAL回调桥接 ----

/**
 * @brief DMA回调公共后处理：失活当前活跃设备CS
 * @param hspi HAL层SPI句柄
 * @note 该函数只处理片选释放，不写入DMA状态
 */
static void SPI_DeselectActiveDevice(SPI_HandleTypeDef* hspi)
{
    Device* dev = GetActiveDevice((SpiID)hspi);
    if (dev != nullptr)
    {
        dev->Deselect();
    }
    SetActiveDevice((SpiID)hspi, nullptr);
}

/**
 * @brief HAL DMA发送完成回调
 * @param hspi HAL层SPI句柄
 * @note 处理顺序固定为：失活CS -> 状态置Done
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SPI_DeselectActiveDevice(hspi);
    SetDmaState((SpiID)hspi, DmaState::Done);
}

/**
 * @brief HAL DMA接收完成回调
 * @param hspi HAL层SPI句柄
 * @note 处理顺序固定为：失活CS -> 状态置Done
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SPI_DeselectActiveDevice(hspi);
    SetDmaState((SpiID)hspi, DmaState::Done);
}

/**
 * @brief HAL DMA收发完成回调
 * @param hspi HAL层SPI句柄
 * @note 处理顺序固定为：失活CS -> 状态置Done
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SPI_DeselectActiveDevice(hspi);
    SetDmaState((SpiID)hspi, DmaState::Done);
}

/**
 * @brief HAL DMA异常回调
 * @param hspi HAL层SPI句柄
 * @note 处理顺序固定为：失活CS -> 状态置Error
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    SPI_DeselectActiveDevice(hspi);
    SetDmaState((SpiID)hspi, DmaState::Error);
}
#endif
