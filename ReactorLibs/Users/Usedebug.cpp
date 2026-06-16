#include "HostPC.hpp"
#include "Farcon.hpp"
#include "Chassis.hpp"
// #include "R1GetBlock.hpp"
#include "PathChaser.hpp"
#include "Logic.hpp"
#include "CommCenter.hpp"

HostPC& hostpc = HostPC::GetInstance();
Farcon& farcon = Farcon::GetInstance();
ChassisType& chas = ChassisType::GetInstance();
// GetBlock& getblock = GetBlock::GetInstance();
PathChaserType& path_chaser = PathChaserType::GetInstance();
TaskLogic& logic = TaskLogic::GetInstance();
CommCenter& comm = CommCenter::GetInstance();
StateCore& core = StateCore::GetInstance();
