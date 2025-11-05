# UART库(bsp层)
# 简介
UART BSP（板级支持包）库，它可以用来管理多个UART实例，支持不同的工作模式（普通、中断、DMA）。当我们使用zigbee无线串口、蓝牙模块等实现遥控器控制机器人与上位机（如：vofa+）通过串口调试时，需用本库进行数据的收发。

# 使用指南（必看）
## 快速开始
一般情况下，我们使用本库实现串口收发数据的逻辑顺序是这样的：
**定义实例-->实现接收回调函数-->注册实例-->使用函数发送数据**
简单的数据收发，可选择**普通串口**模式；中等数据量的数据收发，可选择**中断串口**；高速连续的发送接收数据，选择**DMA模式**。此文档以**DMA模式**为例，介绍如何快速使用本库进行串口的配置。
## 定义串口实例
**每个串口需要独立定义一个串口实例**，每个串口实例实则是一个**结构体**，包含串口句柄、工作模式、回调函数、接收缓冲区、数据长度等信息，是每个串口配置的基础,因此程序要**一开始就要定义**。
**每个uart通道只能对应一个串口实例！！！**
示例：`BspUart_Instance uart1_inst;`
无需手动引出结构体成员进行初始化，实例的注册与初始化中会自动为结构体初始化。
```c
#include "main.h" 
#include "bsp_uart.h"
...
// 定义串口实例
BspUart_Instance  uart1_inst;
...
int main(void)
{
   ...
   while(1)
   {
   ...
   }
}
```

##  实现接收回调函数
每一个串口实例都应该拥有一个对应的`接收回调函数`
`回调函数`应当具有固定的参数格式，对于本库中的串口实例，其接受的`BspUart_InstRxCallback`类型要求如下：
`void 自定义函数名(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);`
回调函数决定了当本串口接收到数据时，这些接收到的**数据**会**如何被处理**。因此，在串口实例 **注册初始化** 的时候，回调函数会被用到，即其必须 **在此之前被定义** 。
*如果你的串口实例没有接收需求，你也可以选择定义一个空内容的回调函数。*
```c
示例：
#include "main.h" 
#include "bsp_uart.h"
...
//定义串口实例
BspUart_Instance  uart1_inst;
//实现接收回调函数
void MyUartRxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
{
  if(huart==&huart1)
  {
  //在这里可以写数据处理的逻辑，如一些命令响应。
  }
}
int main(void)
{
   ...
   while(1)
   {
   ...
   }
}
```

### 参数详解
`UART_HandleTypeDef *huart`:触发接收的串口句柄
`uint8_t *rxData`：接收数据缓冲区
`uint8_t size`：实际接收的数据长度

##  串口实例的注册与初始化（关键步骤）
```c
   void BspUart_InstRegist(BspUart_Instance *inst, UART_HandleTypeDef *huart, uint8_t rx_setlen,
                        BspUart_TypeDef rxtype, BspUart_TypeDef txtype, BspUart_InstRxCallback rx_callback)
```
通过调用`BspUart_InstRegist`函数，**对串口实例`uart1_inst`结构体进行初始化**，即完成实例与串口的绑定，以及串口基本信息（如工作模式，数据接收缓冲区域等）注册，是使用库的核心步骤，只有注册成功后，串口才能正常收发数据
```c


示例：
#include "main.h" 
#include "bsp_uart.h"
...
//定义串口实例
BspUart_Instance  uart1_inst;
//实现接收回调函数
void MyUartRxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
{
  if(huart==&huart1)
  {
  //在这里可以写数据处理的逻辑，如一些命令响应。
  }
}
int main(void)
{
...
//uart实例的注册与初始化
BspUart_InstRegist(&uart1_inst,
                   &huart1,
                   128,                     //接收缓冲区的大小
                   BspUartType_DMA,         //用DMA模式接收
                   BspUartType_DMA,         //用DMA模式发送
                   MyUartRxCallback);
   ...                  
   ...                  
   while(1)
   {
   ...
   }                  
                   
                   
} 
```
### 参数详解
`BspUart_Instance *inst`：是第一步中定义的串口实例的指针
`UART_HandleTypeDef *huart`：是串口的句柄
`uint8_t rx_setlen`：是期望接收数据长度，也是DMA的最大长度，DMA模式下达到本长度将触发全满中断（DMA模式已关闭半满中断）
`BspUart_TypeDef rxtype`：接收数据的模式选择
`BspUart_TypeDef txtype`：发送数据的模式选择
`BspUart_InstRxCallback rx_callback`：指向第二步中定义的接收回调函数
# 至此，串口基本配置已完成！
##  数据发送
```c
void BspUart_Transmit(BspUart_Instance inst, uint8_t *data, uint8_t len)
```
如果需要通过**串口发送数据**时，便可调用BspUart_Transmit进行数据的发送
调用`BspUart_Transmit`进行数据的发送
```c


示例：
#include <string.h>
#include "main.h" 
#include "bsp_uart.h"

...
//定义串口实例
BspUart_Instance  uart1_inst;
//实现接收回调函数
void MyUartRxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
{
  if(huart==&huart1)
  {
  //在这里可以写数据处理的逻辑，如一些命令响应。
  }
}
int main(void)
{
   ...
   while(1)
   {
   uint8_t send_data[]="abcdefg";
   BspUart_Transmit(uart1_inst,send_data,strlen(send_data));
   }
}

```
### 参数详解
`BspUart_Instance inst`：已注册完成的`BspUart_Instance`实例
`uint8_t *data`：待发送数据的指针
`uint8_t len`：待发送数据的长度



# UART库主要特性
1. 多实例管理：本库最多支持6个串口实例同时工作
2. 多种工作模式选择：支持普通模式，中断模式和DMA模式
3. 自动缓冲区管理：内置接收缓冲区，自动处理数据接收（存在实例结构体数组中）
4. 线程安全：防止同一串口通道重复注册，保证一个物理串口只有一个实例

# 核心数据结构
    1.串口实例结构体
```c
   typedef struct
{
    UART_HandleTypeDef *huart;                      // 串口句柄
    BspUart_TypeDef rxtype;                           // Rx工作模式
    BspUart_TypeDef txtype;                           // Tx工作模式
    BspUart_InstRxCallback rx_callback;             // 接收回调函数

    uint8_t rx_buffer[64];                          // 接收缓冲区（最大64Byte）
    uint8_t rx_byte;                                // 单次接收的字节数，IT模式下始终为1    
    uint8_t rx_setlen;                             // 期望接收数据长度
    uint8_t rx_len;                                 // 实际接收数据长度
}BspUart_Instance;
```
2. 模式枚举(用来选择工作模式)
```c
typedef enum
{
    BspUartType_Normal,
    BspUartType_IT,
    BspUartType_DMA,
}BspUart_TypeDef;
```


   
   
