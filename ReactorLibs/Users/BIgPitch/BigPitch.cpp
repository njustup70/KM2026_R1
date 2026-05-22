#include "BigPitch.hpp"

void BigPitch::Start()
{
    pitchmotor.Init(Hardware::hcan_main, 3,13,POSANDVEL,DM_J4310);
    pitchmotor.SetFeedbackRange(3.14159, 10.0, 10.0);
    pitchmotor.SetAutoEnable(true);
    pitchmotor.Enable();
}
void BigPitch::Update()
{
    pitchmotor.SetPosVel(1.57f, 0.0f);
}