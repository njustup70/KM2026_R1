<div align="center" style="align-items: center; display: flex; justify-content: center; gap: 10px;">
    <img src="Imgs/UP70_LOGO_Ramp_Simp.png" alt="Up70" width="240">
    <img src="Imgs/Up70Art_Ramp.png" alt="Up70Char" width="240" height="120" >
</div>

<div align="center">
<h1>南京理工大学Up70战队大疆A板电控框架</h1>
</div>

## 简介
基于 ***DJI A Board*** 搭建的电控框架库，跟大多数嵌入式框架一样被分为了 `Bsp`、`Module`、`App`、`Sys`层。其中，`App`和`Sys`处于同一级。

用户基本无需更改App层以下的任何代码，即可搭建自己的逻辑框架 

同时，用户在使用时，也 ***严禁更改非用户代码区的任何代码***

***本架构仍在持续更新中！！！请积极 <u>提 ISSUE</u> 和 <u>拉取最新代码</u>***

## Fast StartUp
### Build 构建
将需要使用的库的`Build()`接口，放入`Interface.cpp`的`Interface::Buildlize()`中
```cpp
void Interface::Buildlize()
{
    Target::Build();
}
```

### Update 维护
将需要使用的库的`Update()`接口，放入`Interface.cpp`的对应循环线程中
```cpp
void Interface::Slow()
{
    Target::Update();
}
```

### Logic 逻辑
在`AutoLogic.cpp`和`InstruLogic.cpp`中编写逻辑
```cpp
void RoboWorking()
{
    SEQLIZE
    {
        ...
    }
}
```