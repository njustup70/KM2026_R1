#ifndef C_CPP_H
#define C_CPP_H

/**
 * @file C_Cpp.h
 * @author Huangney
 * @date 2025.9.6
 * @note 本文件用于作为C和C++代码的接口文件，所有要在C中调用的函数都放在此文件中
 */
#ifdef __cplusplus
extern "C" {
#endif

/******     RTOS函数        ******/
/// @brief 高频率的控制任务（100Hz ~ 1000Hz）
void FastControlCpp();

/// @brief 低频率的控制任务（100Hz以下）
void SlowControlCpp();

/// @brief 机器人主要进程
void RobotMainCpp();

/// @brief 机器人主任务
void RobotSystemCpp();

/******     MAIN函数        ******/
/// @brief 主函数
void MainCpp();


/******     IT函数        ******/

#ifdef __cplusplus
}
#endif

#endif