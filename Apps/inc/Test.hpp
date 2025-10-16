#ifndef TEST_HPP
#define TEST_HPP

#include "stm32f4xx_hal.h"

void TestPart_Init();
void TestPart_Loop();
void TestPart_MainInit();

extern bool TestEnable;
#endif