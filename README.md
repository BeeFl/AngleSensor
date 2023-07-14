# 电动推杆中角度传感器模块的设计与验证
作者：石磊
## 摘要
角度传感器广泛应用于电动阀门的角度控制，亦可以用于电机驱动的往复运动器
件的行程控制，本课题的目标就是针对电动推杆设计一种基于角度传感器的
角度检测系统，用于间接检测推杆的实际行程。通过减速机构将推杆行程转换为
0-360°的角度。根据需求本文首先确定了用能感应磁场角度的感应芯片MT6816
作为感应器件，用单片机作为数据处理与控制器，用脉宽调制信号（PWM 方式）
控制模拟电流信号输出的设计方案，在充分查阅资料与分析的基础上完成了电路
原理设计、PCB 版制图以及软件编程，并进行了实验验证，实验结果表明本次设
计的角度检测系统实现了0-360°的角度检测量程，分辨率为0.5°的精度，实现
了角度信号RS485 数字方式输出与4-20mA的模拟信号输出，全部指标均满足设
计要求。另外对模拟信号输出进行了线性分析与精度校准。
## 硬件原理图
![硬件原理图](https://github.com/BeeFl/AngleSensor/blob/master/hardware/%E7%A1%AC%E4%BB%B6%E5%8E%9F%E7%90%86%E5%9B%BE.png)
## 电路板实物图
![电路板实物图](https://github.com/BeeFl/AngleSensor/blob/master/hardware/%E7%94%B5%E8%B7%AF%E6%9D%BF%E5%AE%9E%E7%89%A9%E5%9B%BE.png)
## 演示视频
[演示视频](https://www.bilibili.com/video/BV1m94y1B74E)
## 角度转4-20mA电流线性度验证
![角度转4-20mA电流线性度验证](https://github.com/BeeFl/AngleSensor/blob/master/img/angle_current_linearity_test.png)
## 软件流程图
![角度转4-20mA电流线性度验证](https://github.com/BeeFl/AngleSensor/blob/master/img/%E8%BD%AF%E4%BB%B6%E6%B5%81%E7%A8%8B%E5%9B%BE.png)
