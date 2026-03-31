# BSP UART 库使用说明

`bsp_uart` 是一个基于 STM32 HAL 库封装的串口通信库，通过 **DMA + FIFO** 实现高效发送，并利用 **IDLE (空闲中断) + DMA** 实现变长数据接收。

## 核心功能
- **异步发送**：由 DMA 自动发送，数据先写入 FIFO 以解决连续调用 DMA 时数据的截断问题，不阻塞应用层。
- **高效接收**：自动触发 IDLE 中断，支持变长数据包接收。
- **解耦设计**：使用 `UartID` (不透明指针) 替代 HAL 句柄，减少头文件依赖。

## 架构说明
- **解耦设计**：`Bsp` 层不直接面对 `HAL`，而是通过库文件 `halport` 作为统一的代理接口。
- **类型遮蔽**：上层通过 `Hardware` 命名空间访问串口实例（如 `Hardware::huart_host`），底层自动处理其与 HAL 句柄的映射，实现框架层与硬件层的深度解耦。


## 快速上手

> **注意**：通常情况下，`Bsp` 层的库应在 `Mod` 及以上的层调用。

### 获取串口句柄 (`Handler`)
在上层代码中，直接使用 `Hardware` 命名空间预定义的 `UartID` 实例来申请 `Handler`。
```cpp
#include "bsp_uart.hpp"

// 示例：获取工控机通信串口的句柄
auto uart_handler = BSP::UART::Apply(Hardware::huart_host);
```

### 发送数据 (`Transmit`)
直接调用 `Transmit` 即可。库会自动处理 FIFO 缓冲和 DMA 启动。
```cpp
uint8_t data[] = {0x01, 0x02, 0x03};
uart_handler.Transmit(data, sizeof(data));
```

### 注册并接收数据 (`RegisterRx`)
定义一个回调函数，并将其注册到串口句柄中。
```cpp
// 定义回调函数
void MyRxCallback(BSP::UART::UartID id, uint8_t *rxData, uint8_t size) {
    // 处理接收到的数据 (rxData)
}

// 注册回调（设置最大单次接收长度）
uart_handler.RegisterRx(64, MyRxCallback);
```

---

## 注意事项

- **单实例原则**：每个硬件串口只能申请一个 `Instance`。
- **回调限制**：每个实例仅允许注册 **一次** 接收回调。
- **缓冲区大小**：
  - 发送 FIFO 默认为 `2048` 字节 (`BSP_UART_TX_BUF_SIZE`)。
  - 接收单次最大包长不应超过 `64` 字节 (`BSP_UART_RX_BUF_SIZE`)。
- **常见问题**：
  - 若日志出现 `[Bsp] UART Tx FIFO Full`，说明发送频率过高或波特率过低。
  - 确保已经在 `CubeMX` 中开启了对应串口的 **DMA 接收** 和 **全局中断**。
