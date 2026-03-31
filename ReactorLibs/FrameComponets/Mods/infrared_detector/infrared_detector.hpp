#pragma once
#include "bsp_gpio.hpp"

class InfraredDetector
{
public:
    InfraredDetector(){};

    void Init(Pin pin);
    bool Read() const;
private:
    BSP::GPIO::Inst inst;
};

