#include "HostPC.hpp"
#include "Farcon.hpp"
#include "Chassis.hpp"
#include "R1GetBlock.hpp"
#include "PathChaser.hpp"
#include "Logic.hpp"
#include "CommCenter.hpp"

HostPC& host_pc_debug = HostPC::GetInstance();
Farcon& farcon_debug = Farcon::GetInstance();
ChassisType& chas_debug = ChassisType::GetInstance();
GetBlock& get_block_debug = GetBlock::GetInstance();
PathChaserType& path_chaser_debug = PathChaserType::GetInstance();
TaskLogic& logic_debug = TaskLogic::GetInstance();
CommCenter& comm_center_debug = CommCenter::GetInstance();
