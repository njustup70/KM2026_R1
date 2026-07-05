# R1需要的功能（三区）：
## 规划（全自动）的：
* 抬升到目标高度且 重试区域自动规划路径跑到九宫格前面中间  Action_PrePut -> HiddenInPath
* 从九宫格中间自动规划路径到R1地上设置的块前且取块 HiddenOutPath -> GetGroundBlock
## 手动以随机应变的：
* 任意地点可以通过按按键跑到“左中右”三个位置，AnyToMiddleGrid() -> FromMiddleToAny()
* 自动吐块或者戳块（按遥控器按键）  r1block.PutBlock();
* 手动取任意块（人遥控到块面前按一个键就可以取块）。  r1block.GetGroundBlock();
