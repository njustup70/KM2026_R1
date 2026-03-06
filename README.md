<div align="center" style="align-items: center; display: flex; justify-content: center; gap: 10px;">
    <img src="Imgs\UP70_LOGO.png" alt="Up70" width="240">
    <img src="Imgs/Up70Art_Ramp.png" alt="Up70Char" width="240" height="120" >
</div>
<div align="center">
<h1>Reactor70 电控框架</h1>
</div>

# 简介
> 项目已更名为 `Reactor70`  
> 本框架为源框架 [GeneralFramework](https://github.com/njustup70/Reactor-46H/tree/master) 的后继版本  
> 如需获取旧版本，请点击链接，切换到 原`master`分支

基于 ***DJI A Board*** 和 ***DJI C Board*** 搭建的电控框架库  
与大多数嵌入式框架一样分为 `Bsp`、`Module`、`Sys`、`App`层
框架使用 全`C++` 进行组织，并严格面向对象 

用户 ***无需更改*** App层以下的任何代码，只需要通过调用 / 编写 `Application` 即可实现逻辑

# 新特性

> ***本架构仍在持续更新中！！！请积极提交 ISSUE 和 PR***

## 接口设计
本框架 ***完全兼容*** 了旧版本的设计语言，并在此基础上进行了一些改进

原有的用户代码几乎可以 **无缝地** 衔接到新框架中

## 系统架构设计
与旧版本完全一致，仍然采用经典的四层架构设计

同时，系统运行的模块设计也完全一致

## 兼容性
### 工具链
- 新框架 ***完全兼容*** 旧版本的 使用习惯   
***同时兼容*** `Keil Assistant` 工具链 与 `OpenOCD` 工具链

### 硬件底座
- 新框架 具有极强的移植性，能够在改动硬件的前提下，完全不改动用户代码  
只需要通过 *简单的移植工作*，即可在新的硬件平台上启动。
- 新框架 **不再含有** 任何与 *硬件强绑定* 的代码，包括常见的`uproject`文件和`CubeMX.ioc`文件等
- 新框架的配置方法，请参考 ***队伍文档***


