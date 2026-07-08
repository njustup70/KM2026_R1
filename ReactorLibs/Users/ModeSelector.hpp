#define Mode_Test_Degenerate     0
#define Mode_KungFu_Master       1
#define Mode_Exploring_the_Charms 2
#define Mode_Hidden_Treasures    3 

#define Red_Halve 1
#define Blue_Halve 2

// ============请选取你的厕所(比赛模式) :)================
// 修改宏定义
// 0 = Test_Degenerate 简并模式测试用 
// 1 = KungFu_Master武林探秘
// 2 = Exploring_the_Charms崇武探幽
// 3 = Hidden_Treasures九宫藏宝

#define Current_Mode Mode_KungFu_Master
#define Halve Blue_Halve
// =====================================================

//用于代码运行时的逻辑切换
enum Match_Mode
{
  Test_Degenerate = 0,
  KungFu_Master = 1,
  Exploring_the_Charms = 2,
  Hidden_Treasures = 3,
};