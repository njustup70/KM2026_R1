#include "infrared_detector.hpp"
#include "bsp_log.hpp"

void InfraredDetector::Init(Pin pin)
{
    inst = BSP::GPIO::Inst(pin);

    if (!inst.IsValid())
    {
        BspLog_LogError("[Infra_Detector] Init Failed: Pin %c%d Invalid!\r\n", pin.port, pin.number);
    }
}

bool InfraredDetector::Read() const
{
    return inst.Read();
}

