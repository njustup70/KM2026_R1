#ifndef _LOGIC_HPP_
#define _LOGIC_HPP_

#pragma once
#include "System.hpp"
#include "StateCore.hpp"
#include "Chassis.hpp"

class TaskLogic: public Application
{
    SINGLETON(TaskLogic):Application("TaskLogic"){};
    APPLICATION_OVERRIDE

};

namespace APP 
{
    extern TaskLogic& logic;
};



#endif 