# 使用指南（必看！！！）

在RoboCon比赛中，PID控制算法主要运用与对于电机速度环模式和位置环模式的控制中，这里我分别以大疆M3508电机的速度环控制模式和位置环控制模式为例，介绍该PID通用库的使用方法。

## 1、速度环模式

电机的速度环模式一般采用增量式PID，通过速度环控制模式 输出电流值。然后将得到电流值发送给电机。

### 第一步 实例化和初始化

实例化的步骤：  
`Pids + 自定义命名`，如：
```cpp
    // 实例化速度环PID
	Pids speed_pid
```
初始化的步骤：  
调用提供的初始化函数：`void Init()`或`void FastInit()`进行初始化。具体操作为`自定义命名.FastInit(传入参数)`。具体传入参数可以在最后或者源代码中查看，这里不做过多解释。  
**注：对于增量式PID，一般采用`void FastInit()`进行初始化；对于位置式PID，一般采用`void Init()`进行初始化。**。
```cpp
    // 增量速度环（一般速度环采用void FastInit()进行初始化）
    speed_pid.FastInit(3, 24.0, 0.0, 0.16, 5000, false);
```

### 第二步 获得计算后的输出值

首先我们需要设置目标速度值和电流限幅值，在函数`float PidGeneral::Calc(float targ, float real, float output_lim)`中需要目标值和限幅值。
```cpp
    // 设置目标值
    float targ_speed = 2000.f;

    // 设置限幅值
    uint16_t current_limit = 8000;
```
接着我们需要获取当前的速度值：主要是通过读取编码器反馈的数值（根据电机/电调使用说明书进行编写），需要注意的是：当前值必须和目标值的**单位保持一致**。在下面这个示例代码中，`ptr->speed_rpm`即为当前速度值。
```cpp
    // 获取测量结构体
	moto_measure_t *ptr = &measure;

    // 获取当前速度值
    real_speed = ptr->speed_rpm;
```

然后，我们需要通过调用主计算函数（`自定义命名.Calc(传入参数)`）得到输出值（输出值为电流值），并定义一个变量用于接收计算后的输出值。

```cpp
	// 计算目标的 速度PID输出（输出为电流）
	float targ_current_temp = speed_pid.Calc(targ_speed, real_speed, current_limit);
```
### 第三步 发送得到的输出值

将第二步得到的电流值发送给电机，并且计算函数和发送函数要放在`while(1)`循环中：这是实现闭环控制的根本，通过持续感知偏差来纠正偏差，且维持稳定或改变状态都需要持续的能量或信号输入。  
注意：在控制现象不正确的时候，可以尝试在`while(1)`循环中加一段延时(一般为1--5ms)
```cpp
    while(1)
    {
        // 计算目标的 速度PID输出（输出为电流）
	    float targ_current_temp = speed_pid.Calc(targ_speed, real_speed, current_limit);
        // 应用控制量
        SetMotorCurrent(targ_current_temp);

        HAL_Delay(5);
    }
```
## 2、位置环模式

电机的位置环模式一般采用位置式PID，先通过位置环控制 得到速度，再通过速度环控制 得到电流，然后将得到电流值发送给电机。

### 第一步 实例化和初始化

实例化的步骤：  
`Pids + 自定义命名`，由于位置环模式需要位置环和速度环同时进行控制，所以我们需要实例化两个PID类，如：
```cpp
    // 实例化位置环PID
	Pids position_pid;

    // 实例化速度环PID
	Pids speed_pid
```
初始化的步骤：  
调用提供的初始化函数：`void Init()`或`void FastInit()`进行初始化。同样的，我们在这里需要同时初始化位置速度环和位置位置环。  
**注：对于增量式PID，一般采用`void FastInit()`进行初始化；对于位置式PID，一般采用`void Init()`进行初始化。**（上面提到过）。
```cpp
	// 位置速度环
    speed_pid.Init(10.8, 0.0, 0.0, 0.05, 0.0, 0, 0.0, 0.0, 0.8, false, true, true);

	// 位置位置环
    position_pid.Init(0.108, 0.0, 0.0, 0.05, 0.0, 0, 0.0, 4000.0, 0.8, false, true, true);
```

### 第二步 获得计算后的输出值

首先我们需要设置目标速度值和电流限幅值，在函数`float PidGeneral::Calc(float targ, float real, float output_lim)`中需要目标值和限幅值。
```cpp
    // 设置目标值
    float targ_position = 0;

    // 设置电流限幅值
    uint16_t current_limit = 8000;
    // 设置速度限幅值
	uint16_t speed_limit = 20000;
```
接着我们需要获取当前的速度和位置：主要是通过读取编码器反馈的数值，需要注意的是：当前值必须和目标值的**单位保持一致**。
```cpp
    // 获取测量结构体
	moto_measure_t *ptr = &measure;

    // 获取当前速度值
    real_speed = ptr->speed_rpm;
    // 获取当前位置值
    real_position = ptr->total_angle
```

然后，我们需要通过计算得出输出值，先通过位置环控制得到速度，然后通过速度环控制得到电流

```cpp
    // 计算目标的 位置PID输出（输出为速度）	
	float targ_speed = position_pid.Calc(targ_position, real_position, speed_limit);

	// 计算目标的 速度PID输出（输出为电流）
	float targ_current_temp = speed_pid.Calc(targ_speed, real_speed, current_limit);
```
### 第三步 发送得到的输出值

将第二步得到的电流值发送给电机，并且计算函数和发送函数要放在`while(1)`循环中  
注意：在控制现象不正确的时候，可以尝试在`while(1)`循环中加一段延时(一般为1--5ms)
```cpp
    while(1)
    {
        // 计算目标的 位置PID输出（输出为速度）	
    	float targ_speed = position_pid.Calc(targ_position, real_position, speed_limit);
	    // 计算目标的 速度PID输出（输出为电流）
	    float targ_current_temp = speed_pid.Calc(targ_speed, real_speed, current_limit);
        // 应用控制量
        SetMotorCurrent(targ_current_temp);

        HAL_Delay(5);
    }
```

# PID算法简介

关于PID算法的理论部分，可以参考这篇文章：[PID算法介绍](https://blog.csdn.net/m0_38106923/article/details/109545445)，所谓PID算法，是一种在工程应用领域被使用最为广泛的**反馈调节**方法，通过PID算法中比例、积分、微分三个部分的作用，达到使**系统稳定**的效果。

（1）比例调节：反应系统的基本（当前）偏差 e(t)，系数大，可以加快调节，减小误差，但过大的比例使系统稳定性下降，甚至造成系统不稳定

（2）积分调节：反应系统的累计偏差，使系统消除稳态误差，提高无差度，因为有误差，积分调节就进行，直至无误差

（3）微分调节：反映系统偏差信号的变化率 e(t)-e(t-1)，具有预见性，能预见偏差变化的趋势，产生超前的控制作用，在偏差还没有形成之前，已被微分调节作用消除，因此可以改善系统的动态性能。但是微分对噪声干扰有放大作用，加强微分对系统抗干扰不利。

**注：积分和微分都不能单独起作用，必须与比例控制配合。**

# PID控制器库简介

本库支持位置式、增量式及带内部累加的增量式三种PID模式，封装了PID算法的核心计算逻辑，提供统一的接口，可以方便地应用于像电机控制这样的闭环控制场景中。

# 代码结构

`pids.hpp`文件包含了PidGeneral类的完整定义、枚举类型和外部接口。`pids.cpp`文件实现了所有成员函数的具体逻辑。

主要文件有：  
`pids.hpp` ——— 类定义和接口声明  
`pids.cpp` ——— 成员函数的具体逻辑

# 类型定义和类结构

这里主要介绍一些关键参数的含义以及`PidGeneral`类里面的外部接口。

## 1、与PID工作模式有关的类型
这里不是通过枚举实现的，是通过类的成员变量进行配置的。

```cpp
    bool Incremental;     // 是否为增量式PID
    bool Feedforward;     // 是否启用前馈控制  
    bool InnerAcc;        // 是否启用内部累加（仅增量式有效）
    bool AutoDt;          // 是否自动计算时间间隔
    bool DeadbandEnabled; // 是否启用死区控制
```
## 2、与PID算法相关的参数
这里主要是一些常见的与PID算法相关的参数，在kp,ki,kd基础上又新加了一些前馈，限幅，死区相关的参数

```cpp
    // PID参数
    float Kp;      // 比例系数
    float Ki;      // 积分系数
    float Kd;      // 微分系数
    float Kf;      // 前馈系数
    float delta_t; // 时间间隔
    int reverse;   // 反向控制标志
    uint32_t dwt_dt;    // 自动DT用的DWT句柄

    // 限制参数
    float inte_lim;    // 积分限幅
    float out_lim;      // 输出限幅
    float kd_filter_rate; // Kd低通滤波系数
        
    // 死区参数
    float deadband_start; // 死区起始值
    float deadband_end;   // 死区结束值
```

## 3、PID的内部状态变量


```cpp
        float inte_errors;     // 积分误差累计
        float error;           // 当前误差
        float last_error;      // 上一次误差
        float prev_error;      // 上上次误差（用于增量式计算）
        
        float kd_error;        // 微分误差
        float last_kderr;      // 上一次的微分误差
        float prev_kderr;      // 上上次的微分误差
        
        float inc_output;      // 增量式PID的增量输出
        float control_value;   // 内部维护控制量累加值
```

# 外部接口

## 构造函数和初始化函数

```cpp
    // 有参构造函数
    PidGeneral(float kp, float ki, float kd,
               float dt = 0, int reverse = 0,
               float integralLim = 0, float outputLim = 0,
               float deltaFilter = 0.5f,
               bool Incremental = false, bool Feedforward = false,
               bool InnerAcc = true);

    // 默认构造函数
    PidGeneral(){};
    
    // 初始化函数，带完整参数
    void Init(float kp, float ki, float kd, float kf,
              float dt = 0, int reverse = 0,
              float integralLim = 0, float outputLim = 0,
              float deltaFilter = 0.5f,
              bool Incremental = false, bool Feedforward = false,
              bool InnerAcc = true);

    // 快速初始化函数
    void FastInit(float kp, float ki, float kd, float kf = 0,
                  float outputLim = 0, int reverse = 0);
```

对于有参构造函数和，默认构造函数，在实例化类以后系统自动调用，我们不需要调用它们，在实例化`PidGeneral`类的时候如果带了参数即为有参构造函数，如果不带参数即为默认构造函数。  
所以我们在外部实例化一个`PidGeneral`类以后，只需要调用`void Init()`或`void FastInit()`进行初始化。

**注：对于增量式PID，一般采用`void FastInit()`进行初始化；对于位置式PID，一般采用`void Init()`进行初始化。**

## 核心计算接口

```cpp
    // 主计算函数
    float Calc(float targ, float real, float output_lim = 0);

    // 重置PID状态
    void Reset();
```

`float Calc()`函数可以根据配置自动选择计算模式，`void Reset()`函数用来清除积分和历史误差。

## 参数配置接口

```cpp
    // 设置PID参数
    void ParamSet(float kp, float ki, float kd);

    // 设置限制参数
    void LimitSet(float integralLim, float outputLim, float deltaFilter);

    // 设置反向控制
    void RevSet(bool reverse);

    // 设置死区控制
    void DeadbandSet(float start, float end);
```

具体功能看注释就能明白
