# Monitor 库使用文档

`Monitor` 库是 `Reactor70` 框架的核心监控组件，主要用于日志输出、变量实时追踪和模块在线状态监视。它通过 *单例模式* 提供服务，以便于在全局任何位置调用

## 快速使用

在代码中通过 `GetInstance()` 获取单例引用：

```cpp
#include "Monitor.hpp"

Monitor& monit = Monitor::GetInstance();
```

> [!NOTE]
> 系统通常在 `SystemType::Init` 中已完成初始化，默认配置为上位机通信端口 `huart_host`

## 日志功能 (Logging)
库提供了三个等级的日志接口，会自动附带系统时间戳：

*   `Log(format, ...)`：输出普通运行信息。
*   `LogWarning(format, ...)`：输出警告信息。
*   `LogError(format, ...)`：输出错误信息。

**使用示例：**
```cpp
monit.Log("System started successfully.");
monit.LogWarning("Battery voltage low: %.2f V", voltage);
monit.LogError("Motor %d offline!", motor_id);
```

## 变量追踪 (Track)

用于将变量数值发送至上位机（如 VOFA+）进行波形绘制
- 最多支持同时追踪 **8个** 变量
- 分 `普通模式`（200Hz） / `高性能模式`（1000Hz）

### 追踪
支持以下数据类型：
- `uintx_t` / `intx_t`
- `float`

调用 `Track` 以进行追踪
```cpp
float my_variable = 0.1f;
monit.Track(my_variable); // 注册追踪
```

启动后，该变量会以 **200Hz** 被发送至上位机进行实时监控

### 高性能模式
若需 1000Hz 的极快追踪速度，请调用 `Perflize()`。此时将切换为 VOFA+ 的 `JustFloat` 协议（二进制流）

```cpp
monit.Perflize(); // 切换至高性能模式
```

## 状态监视 (Watch)

用于配合系统的自检（Self-Check）功能。通过 `Watch` 一个布尔值，系统可以判断某个硬件或模块是否在线。

```cpp
// 监视电机在线状态
monit.Watch({
    .targ = &motor.is_online,      // 监视的布尔值地址
    .warning_info = "Chassis Motor", // 离线时的提示文字
    .is_neccessary = true          // 是否为必要模块（若为true，离线将触发系统级Error）
});
```

## 使用建议

**简洁至上**：日志信息尽量精炼，避免在 1000Hz 的循环中频繁调用 `Log`，以免阻塞串口。  
**追踪限制**：`Track` 注册后无法轻易移除，建议仅在程序初始化阶段注册关键调试变量。  
**缓冲区**：日志和追踪共用串口带宽，高性能模式下建议减少普通日志的输出。  
