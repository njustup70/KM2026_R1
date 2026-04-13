#pragma once
#include "System.hpp"
#include "InterBoardComm.hpp"

class Connect_CBoard : public Application
{
  SINGLETON(Connect_CBoard) : Application("Connect_CBoard") {};
  APPLICATION_OVERRIDE

public:
  float chassis_vx;
  float chassis_vy;
  float chassis_vz;
  float chasis_posx;
  float chasis_posy;
  float chasis_posz;
};
