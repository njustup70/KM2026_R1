#pragma once
#include "System.hpp"
#include "motor_dm.hpp"

class BigPitch : public Application
{
    SINGLETON(BigPitch):Application("BigPitch"){};
    APPLICATION_OVERRIDE
 
public:

    MotorDM  pitchmotor;

};