#include "Test.hpp"
#include "tim.h"
#include "spi.h"
#include "main.h"
#include "std_cpp.h"
#include "odo_ops.hpp"
#include "led_ws2812.hpp"
#include "bsp_dwt.h"
#include "WS2812_yx.h"
#include "motor_dji.hpp"

Ops9 myOdo;
LedWs2812 myLedWs;

MotorDji myMotor;

Color Up70_Green(105, 209, 25);
Color Up70_Purple(70, 0, 190);
