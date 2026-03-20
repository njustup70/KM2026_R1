#pragma once
#include "bsp_halport.hpp"

__weak void MainFrameCpp(){};

/// @brief 通知 SPI 消费线程：有新的完整样本可以处理了。
/// @note 这里只暴露一个最小的唤醒接口，System 层不需要知道底层信号量细节。
void Reactor46H_NotifySpiConsume();
