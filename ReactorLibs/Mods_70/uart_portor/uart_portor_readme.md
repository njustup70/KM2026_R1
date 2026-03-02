# UartPortor 使用说明

`UartPortor` 是一个层级位于 `Mod` 层的串口分发代理。它允许多个不同的 `Mod` 共享同一个硬件串口，通过轻量级的“信封协议”实现数据的端口化分发。

## 协议格式 (信封)
所有通过 `UartPortor` 发送/接收的数据都遵循以下格式：


`[SOF(0xA5) | PORT_ID | DATA_LEN | PAYLOAD | CHECKSUM]`

- **SOF**: 固定为 `0xA5`。
- **PORT_ID**: 目标端口号 (0~254)。
- **DATA_LEN**: Payload 长度。
- **CHECKSUM**: 异或校验 (从 SOF 到 Payload 结束)。

## 快速上手

### 初始化
每个需要共享串口的模块应持有一个 `UartPortor` 实例。调用 `Init` 注册串口、端口号及接收回调。

```cpp
#include "uart_portor.hpp"
UartPortor my_portor;

// 接收回调示例
void OnRxData(const uint8_t *payload, uint8_t len) 
{
    // 处理接收到的数据 payload
}

my_portor.Init(Hardware::huart_other, 0x01, OnRxData);
```

### 发送数据
调用 `Send` 方法，底层会自动完成协议信封的打包。

```cpp
uint8_t data[] = {0x01, 0x02, 0x03};
my_portor.Send(data, sizeof(data));
```

## 独占硬件串口
如果某个模块希望**独占**某个硬件串口（不与其他模块共享），则**不需要**使用本库。

您可以直接在模块内部声明并使用 `BSP::UART::Handler` (BspUart 实例) 。`UartPortor` 的底层也是基于 `BspUart` 实现的，两者的调用流程高度相似：
1. 使用 `BSP::UART::Apply(uart_id)` 获取句柄。
2. 使用 `RegisterRx()` 注册原始接收回调。
3. 使用 `Transmit()` 发送原始数据。

## 4. 注意事项
- **端口冲突**：同一硬件串口下的不同实例必须使用不同的 `PORT_ID`。
- **数据长度**：目前 `UART_PORTOR_MAX_PAYLOAD` 限制为 200 字节。
- **底层管理**：`UartPortor` 会自动管理 `BspUart` 的申请与回调注册，用户无需手动干预底层。
