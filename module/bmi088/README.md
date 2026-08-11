# BMI088模块说明

## 代码分层

BMI088相关代码按照以下顺序使用：

```text
bsp/SPI
  -> module/bmi088
  -> algorithm/Attitude + algorithm/Math
  -> 工程内的App_IMU
  -> Main Task
```

- `bsp/SPI`负责HAL SPI、DMA回调和片选管理。
- `module/bmi088`只负责芯片初始化、寄存器配置、原始数据读取、单位换算和在线检测。
- `IMU_Bias_Calibration`负责启动零偏校准。
- `Mahony`负责使用陀螺仪和加速度数据更新四元数。
- `Rotation3D`负责固定尺寸的向量、四元数和3x3旋转矩阵运算。
- `App_IMU`负责组合以上模块，并向云台控制层提供最终数据。

## 初始化和采样

`Class_BMI088::Init()`只启动初始化状态机，后续必须周期调用`Update(Now_ms)`。

初始化过程会依次完成软复位、芯片ID检查、电源配置、量程配置和数据就绪中断配置。整个过程不创建任务，也不会使用`HAL_Delay()`阻塞Main Task。

正常运行时，ACC和GYRO数据就绪中断只记录事件。Main Task随后按照ACC到GYRO的顺序串行启动SPI DMA，避免两个器件共用SPI时发生总线冲突。

## 数据单位

- `Class_BMI088`的加速度输出单位为g。
- `Class_BMI088`的角速度输出单位为degree/s。
- `App_IMU`的加速度输出单位为m/s²。
- `App_IMU`的角速度输出单位为rad/s。
- `App_IMU`的Yaw、Pitch、Roll输出单位为degree。

## 零偏校准

默认校准窗口为800组完整ACC和GYRO数据。只有角速度模长不大于3 degree/s，并且加速度模长位于0.9g到1.1g之间的样本才参与平均。

总样本数仍用于限制校准窗口，有效样本数用于计算零偏。每次重新开始校准都会同时清零总样本数和有效样本数。如果整个窗口没有有效样本，校准进入错误状态，App不会进入Ready。

## 坐标系

云台机体系约定为X轴向前、Y轴向左、Z轴向上。传感器坐标系到机体系的安装矩阵由具体工程的`App_IMU_Config.h`设置，默认值为单位矩阵，也就是传感器XYZ与机体XYZ同向。

原代码使用的`euler_extrinsic_ZXY_body_axes_to_front_yaw_pitch_roll_deg()`仍保留在`Rotation3D`中，用于需要自定义前向轴和上方轴的场景。

## Yaw说明

BMI088没有磁力计，因此Yaw是校准完成后的启动相对积分角，不是绝对地理航向。短时间内可以用于云台位置闭环，长时间运行会受到陀螺仪零偏和温度变化影响而漂移。

## 同步到其他工程

公共驱动和算法都位于YXN_ECF。同步到另一块主控后，需要在对应工程重新编写或接入自己的App层，并在CMake中显式加入使用到的源码和头文件路径。
