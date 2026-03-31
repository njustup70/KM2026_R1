/**
 * @author: Agilawood
 * @name: 继电器库 
 * @last_edit_time: 2026-1-25
 * @brief: 该库用于驱动继电器，把每个继电器通道看作一个对象
 */

#include "relay.hpp"

void Relay::Init(Pin pin, TriggerType trig_type)
{
    _gpio_inst = BSP::GPIO::Inst(pin);
    _trig_type = trig_type;
    initialized = _gpio_inst.IsValid();
    _Set(OFF);
}

void Relay::_Set(State state)
{
    if (!initialized) return;

    bool pin_state = false;
    if (_trig_type == HIGH_ON)
    {
        pin_state = (state == ON);
    }
    else 
    {
        pin_state = (state != ON);
    }
    _gpio_inst.Write(pin_state);
    _state = state;
}

void Relay::On()
{
    _Set(ON);
}

void Relay::Off()
{
    _Set(OFF);
}


