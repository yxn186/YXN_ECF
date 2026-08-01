# DM4310 电机控制库使用说明

# 1. 功能概览

本库用于通过经典 CAN 控制达妙 DM-J4310 系列电机，主要提供以下功能：

- MIT 模式控制
- 位置速度模式控制
- 速度模式控制
- 力位混控模式控制
- 电机使能、失能、清除故障和设置零点
- 普通状态反馈解析
- 电机在线状态检测
- 寄存器异步读取
- 寄存器异步写入
- 参数保存到电机 Flash
- 参数操作超时检测
- bxCAN、FDCAN 等不同 CAN 驱动的统一接口接入

库通过 `Class_CAN_Interface` 发送 CAN 报文，因此电机类本身不直接依赖具体的 bxCAN 或 FDCAN 发送函数。

---

# 2. 文件与依赖

主要文件：

```text
Motor_DM4310.h
Motor_DM4310.cpp
CAN_Interface.h
```

`Motor_DM4310.cpp` 使用了：

```cpp
HAL_GetTick();
```

因此当前实现依赖 STM32 HAL，并在 `.cpp` 中包含：

```cpp
#include "main.h"
```

协议要求 `float` 为 4 字节，库中已经通过以下代码进行编译期检查：

```cpp
static_assert(
    sizeof(float) == 4U,
    "DM4310 protocol requires 32-bit float."
);
```

---

# 3. CAN 接口要求

使用本库前，需要准备一个继承自 `Class_CAN_Interface` 的 CAN 适配器对象，例如：

```cpp
Class_FDCAN_Adapter FDCAN1_Adapter;
// 或
Class_BxCAN_Adapter CAN1_Adapter;
```

电机类只通过以下统一接口发送数据：

```cpp
Enum_CAN_Transmit_Status_e Transmit(
    uint16_t id,
    const uint8_t *data,
    uint8_t length
);
```

因此初始化电机时，可以传入任意实现了该接口的 CAN 适配器。

---

# 4. 创建电机对象

```cpp
#include "Motor_DM4310.h"

Class_DM4310_Motor Motor_DM4310;
```

每个电机建议创建一个独立对象。

多个电机可以共用同一个 CAN 适配器：

```cpp
Class_DM4310_Motor Motor_1;
Class_DM4310_Motor Motor_2;
```

---

# 5. 初始化

## 5.1 初始化接口

```cpp
bool Init(
    Class_CAN_Interface *CAN_Interface,
    uint8_t Motor_ID,
    uint16_t Master_ID,
    DM_Control_Mode_e Control_Mode,
    float Position_Max,
    float Velocity_Max,
    float Torque_Max,
    uint32_t Feedback_Timeout_ms = 100U
);
```

参数说明：

| 参数 | 说明 |
|---|---|
| `CAN_Interface` | CAN 适配器对象地址 |
| `Motor_ID` | 电机基础 ID，当前库限制为 `0x00 ~ 0x0F` |
| `Master_ID` | 电机反馈报文 ID，范围 `0x000 ~ 0x7FF` |
| `Control_Mode` | 电机当前控制模式 |
| `Position_Max` | PMAX，位置映射范围 |
| `Velocity_Max` | VMAX，速度映射范围 |
| `Torque_Max` | TMAX，扭矩映射范围 |
| `Feedback_Timeout_ms` | 普通状态反馈超时时间，默认 100 ms |

## 5.2 初始化示例

```cpp
bool Init_Result = Motor_DM4310.Init(
    &FDCAN1_Adapter,
    0x01U,
    0x000U,
    DM_Control_Mode_e::Velocity,
    12.5f,
    30.0f,
    12.5f,
    100U
);
```

## 5.3 初始化注意事项

`Control_Mode` 必须和电机驱动器内部当前模式一致。

`Position_Max`、`Velocity_Max`、`Torque_Max` 必须和电机内部的 PMAX、VMAX、TMAX 一致。它们用于：

- MIT 控制命令的线性映射；
- 普通反馈中位置、速度和扭矩的反向映射。

如果 MCU 和电机使用的映射范围不一致，发送目标和解析反馈都会产生比例错误。

---

# 6. CAN 接收回调接入

所有接收到的达妙电机报文，都建议统一传入：

```cpp
bool Process_CAN_Message(
    uint16_t CAN_ID,
    const uint8_t *Data,
    uint8_t Length
);
```

该函数会自动区分：

- 普通状态反馈；
- 寄存器读取应答；
- 寄存器写入应答；
- 参数保存应答；
- 迟到或重复的参数应答。

不要只在回调中调用 `Process_CAN_Feedback()`，否则寄存器应答可能无法被正确处理。

## 6.1 通用接收示例

```cpp
void CAN_Rx_Dispatcher(
    uint16_t CAN_ID,
    const uint8_t *Data,
    uint8_t Length
)
{
    Motor_DM4310.Process_CAN_Message(
        CAN_ID,
        Data,
        Length
    );
}
```

多电机时，可以依次分发：

```cpp
void CAN_Rx_Dispatcher(
    uint16_t CAN_ID,
    const uint8_t *Data,
    uint8_t Length
)
{
    Motor_1.Process_CAN_Message(CAN_ID, Data, Length);
    Motor_2.Process_CAN_Message(CAN_ID, Data, Length);
}
```

---

# 7. 周期任务

建议在周期任务中持续更新在线状态和参数操作超时状态：

```cpp
void Motor_Task_1ms(void)
{
    Motor_DM4310.Update_Online_States();
    Motor_DM4310.Update_Parameter_States();

    // 根据当前模式发送控制命令
}
```

`Update_Online_States()` 不会发送 CAN，只检查普通反馈是否超时。

`Update_Parameter_States()` 不会主动读写寄存器，只检查当前异步参数操作是否超过 100 ms。

---

# 8. CAN 发送返回状态

控制、特殊命令和参数操作均返回：

```cpp
Enum_CAN_Transmit_Status_e
```

常见状态：

| 状态 | 说明 |
|---|---|
| `Success` | CAN 报文已成功提交发送 |
| `Busy` | CAN 接口繁忙，或已有参数操作正在等待应答 |
| `Error` | 未初始化、参数错误、模式错误或发送失败 |

注意：

```cpp
Motor_DM4310.Enable() == Success
```

只代表使能报文发送成功，不代表电机已经实际进入使能状态。真实状态应通过普通反馈中的 `States` 判断。

---

# 9. 特殊控制命令

## 9.1 使能

```cpp
Motor_DM4310.Enable();
```

## 9.2 失能

```cpp
Motor_DM4310.Disable();
```

## 9.3 清除故障

```cpp
Motor_DM4310.Clear_Error();
```

## 9.4 设置当前位置为零点

```cpp
Motor_DM4310.Set_Zero_Position();
```

设置零点会修改电机位置基准，不建议在每次上电初始化时自动执行。

特殊命令会根据初始化时配置的控制模式，自动选择对应的控制 CAN ID。

---

# 10. MIT 模式

## 10.1 模式说明

MIT 模式同时使用以下五个参数：

```text
目标位置
目标速度
位置 Kp
速度 Kd
前馈扭矩
```

可以近似理解为：

```text
输出扭矩 =
Kp × 位置误差
+ Kd × 速度误差
+ 前馈扭矩
```

MIT 模式的特点是控制自由度高，可以组合实现：

- 位置控制；
- 速度控制；
- 阻尼控制；
- 前馈扭矩控制；
- 位置、速度和扭矩的混合控制。

初始化时必须选择：

```cpp
DM_Control_Mode_e::MIT
```

### 10.2 弧度接口

```cpp
Enum_CAN_Transmit_Status_e Control_MIT_Rad(
    float Target_Position_Rad,
    float Target_Velocity,
    float Position_Kp,
    float Velocity_Kd,
    float Feedforward_Torque
);
```

### 10.3 角度接口

```cpp
Enum_CAN_Transmit_Status_e Control_MIT_Degree(
    float Target_Position_Degree,
    float Target_Velocity,
    float Position_Kp,
    float Velocity_Kd,
    float Feedforward_Torque
);
```

只有位置参数使用角度，目标速度仍然使用 `rad/s`。

## 10.4 MIT 位置控制示例

```cpp
Motor_DM4310.Control_MIT_Degree(
    30.0f,  // 目标位置：30°
    0.0f,   // 目标速度：0 rad/s
    10.0f,  // Kp
    0.5f,   // Kd
    0.0f    // 前馈扭矩
);
```

建议从较小的 `Kp`、`Kd` 开始调试。

位置控制时不要把 `Kd` 设得过小，否则容易振荡。

第一次使能前，建议先把目标位置设置为当前反馈位置：

```cpp
const DM_Motor_Feedback_t &Feedback =
    Motor_DM4310.Get_Feedback_Data();

Motor_DM4310.Control_MIT_Rad(
    Feedback.Position,
    0.0f,
    5.0f,
    0.3f,
    0.0f
);

Motor_DM4310.Enable();
```

## 10.5 MIT 速度控制示例

```cpp
Motor_DM4310.Control_MIT_Rad(
    0.0f,  // Kp为0时，位置目标不起作用
    5.0f,  // 目标速度：5 rad/s
    0.0f,  // Kp
    1.0f,  // Kd
    0.0f   // 前馈扭矩
);
```

MIT 速度控制与专用速度模式不同。MIT 速度控制主要依靠 `Kd` 产生速度误差对应的扭矩，而专用速度模式使用驱动器内部速度控制器。

## 10.6 MIT 扭矩控制示例

```cpp
Motor_DM4310.Control_MIT_Rad(
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    1.0f   // 前馈扭矩
);
```

当 `Kp = 0`、`Kd = 0` 时，输出主要由前馈扭矩决定。

扭矩控制风险较高，应从很小的前馈值开始测试，并确保电机可靠固定。

## 10.7 MIT 模式内部限幅

当前库会自动限制：

```text
位置：[-PMAX, +PMAX]
速度：[-VMAX, +VMAX]
Kp：[0, 500]
Kd：[0, 5]
前馈扭矩：[-TMAX, +TMAX]
```

---

# 11. 速度模式

## 11.1 模式说明

速度模式用于让电机持续跟踪目标速度。

初始化时选择：

```cpp
DM_Control_Mode_e::Velocity
```

控制接口：

```cpp
Enum_CAN_Transmit_Status_e Control_Velocity(
    float Target_Velocity
);
```

目标速度单位为：

```text
rad/s
```

## 11.2 调用示例

```cpp
Motor_DM4310.Control_Velocity(3.0f);
```

表示目标速度为：

```text
3 rad/s
```

反方向旋转：

```cpp
Motor_DM4310.Control_Velocity(-3.0f);
```

停止：

```cpp
Motor_DM4310.Control_Velocity(0.0f);
```

## 11.3 周期调用示例

```cpp
void Motor_Task_1ms(void)
{
    Motor_DM4310.Update_Online_States();
    Motor_DM4310.Update_Parameter_States();

    Motor_DM4310.Control_Velocity(
        Target_Velocity
    );
}
```

当前速度模式接口不会使用 `VMAX` 自动限制目标速度。应用层应根据电机能力自行限幅。

---

# 12. 位置速度模式

## 12.1 模式说明

位置速度模式用于让电机运动到指定位置，同时限制运动过程中的最大速度。

参数含义：

```text
Target_Position：最终目标位置
Velocity_Limit：运动过程中的速度上限
```

`Velocity_Limit` 不是要求电机一直保持的目标速度。

初始化时选择：

```cpp
DM_Control_Mode_e::Position_Velocity
```

## 12.2 弧度接口

```cpp
Enum_CAN_Transmit_Status_e
Control_Position_Velocity_Rad(
    float Target_Position_Rad,
    float Velocity_Limit
);
```

## 12.3 角度接口

```cpp
Enum_CAN_Transmit_Status_e
Control_Position_Velocity_Degree(
    float Target_Position_Degree,
    float Velocity_Limit
);
```

## 12.4 调用示例

运动到 `90°`，最大速度为 `2 rad/s`：

```cpp
Motor_DM4310.Control_Position_Velocity_Degree(
    90.0f,
    2.0f
);
```

弧度方式：

```cpp
Motor_DM4310.Control_Position_Velocity_Rad(
    1.5708f,
    2.0f
);
```

## 12.5 安全启动

第一次使能前，应先把目标位置设置为当前位置：

```cpp
const DM_Motor_Feedback_t &Feedback =
    Motor_DM4310.Get_Feedback_Data();

Motor_DM4310.Control_Position_Velocity_Rad(
    Feedback.Position,
    1.0f
);

Motor_DM4310.Enable();
```

如果当前实际位置和目标位置相差较大，电机可能在使能后快速运动。

当前库只检查 `Velocity_Limit` 不能为负数，不会自动限制其最大值。实际使用时应按照电机参数合理设置。

---

# 13. 力位混控模式

## 13.1 模式说明

力位混控模式可以理解为：

```text
位置控制
+ 速度限制
+ 最大电流比例限制
```

它仍然以到达目标位置为主要目标，但可以通过电流标幺值限制最大输出能力。

初始化时选择：

```cpp
DM_Control_Mode_e::Force_Position_Hybrid
```

## 13.2 弧度接口

```cpp
Enum_CAN_Transmit_Status_e
Control_Force_Position_Hybrid_Rad(
    float Target_Position_Rad,
    float Velocity_Limit,
    float Current_Limit_Per_Unit
);
```

## 13.3 角度接口

```cpp
Enum_CAN_Transmit_Status_e
Control_Force_Position_Hybrid_Degree(
    float Target_Position_Degree,
    float Velocity_Limit,
    float Current_Limit_Per_Unit
);
```

## 13.4 电流限制标幺值

```text
0.0：不允许输出电流
0.2：最大电流的 20%
0.5：最大电流的 50%
1.0：最大电流的 100%
```

它不是安培值，也不是带方向的扭矩目标。

## 13.5 调用示例

运动到 `90°`，速度限制 `2 rad/s`，电流限制为最大值的 20%：

```cpp
Motor_DM4310.Control_Force_Position_Hybrid_Degree(
    90.0f,
    2.0f,
    0.2f
);
```

## 13.6 库内限制

当前库会自动处理：

```text
Velocity_Limit < 0：返回 Error
Velocity_Limit > 100：限制为 100

Current_Limit_Per_Unit < 0：返回 Error
Current_Limit_Per_Unit > 1：限制为 1
```

第一次调试建议使用较小的速度和电流限制。

---

# 14. 普通反馈与在线状态

## 14.1 获取反馈

```cpp
const DM_Motor_Feedback_t &Feedback =
    Motor_DM4310.Get_Feedback_Data();
```

主要字段：

```cpp
Feedback.Motor_ID;
Feedback.States;

Feedback.Raw_Position;
Feedback.Raw_Velocity;
Feedback.Raw_Torque;

Feedback.Position;
Feedback.Velocity;
Feedback.Torque;

Feedback.Mos_Temperature;
Feedback.Rotor_Temperature;

Feedback.Online;
Feedback.Last_Feedback_Time;
```

其中：

```text
Position：rad
Velocity：rad/s
Torque：按照TMAX映射后的反馈值
温度：℃
```

## 14.2 在线判断

```cpp
Motor_DM4310.Update_Online_States();

bool Online =
    Motor_DM4310.Get_Online_States();
```

在线状态只由合法的普通状态反馈刷新。

参数读取、写入和保存应答不会刷新普通反馈在线状态。

## 14.3 状态判断示例

```cpp
const DM_Motor_Feedback_t &Feedback =
    Motor_DM4310.Get_Feedback_Data();

if (Feedback.Online)
{
    if (Feedback.States ==
        DM_Motor_States_e::Enabled)
    {
        // 电机已使能
    }
    else if (Feedback.States ==
             DM_Motor_States_e::Disabled)
    {
        // 电机已失能
    }
}
```

---

# 15. 寄存器读取

## 15.1 异步读取流程

寄存器读取不是阻塞操作：

```text
调用 Read_Parameter()
→ 发送读取请求
→ 等待电机 CAN 应答
→ 接收回调调用 Process_CAN_Message()
→ 从 Get_Parameter_Data() 读取结果
```

调用 `Read_Parameter()` 成功，只表示读取请求已经发送，不代表结果已经返回。

## 15.2 读取 float 寄存器

读取 PMAX：

```cpp
Enum_CAN_Transmit_Status_e Status =
    Motor_DM4310.Read_Parameter(
        DM_Register_e::PMAX
    );
```

之后在周期任务、应用状态机或调试代码中检查：

```cpp
const DM_Parameter_Data_t &Parameter =
    Motor_DM4310.Get_Parameter_Data();

if ((!Parameter.Waiting_Response) &&
    Parameter.Valid &&
    (Parameter.Register ==
     DM_Register_e::PMAX))
{
    float Position_Max =
        Parameter.Float_Value;
}
```

## 15.3 读取 uint32 寄存器

读取当前控制模式：

```cpp
Motor_DM4310.Read_Parameter(
    DM_Register_e::CTRL_MODE
);
```

检查结果：

```cpp
const DM_Parameter_Data_t &Parameter =
    Motor_DM4310.Get_Parameter_Data();

if (Parameter.Valid &&
    (Parameter.Register ==
     DM_Register_e::CTRL_MODE))
{
    uint32_t Control_Mode =
        Parameter.Uint32_Value;
}
```

## 15.4 读取超时

```cpp
if (Parameter.Timed_Out)
{
    // 超过100ms没有收到对应应答
}
```

必须周期调用：

```cpp
Motor_DM4310.Update_Parameter_States();
```

否则超时状态不会自动更新。

---

# 16. 寄存器写入

## 16.1 写入 float 寄存器

```cpp
Motor_DM4310.Write_Parameter_Float(
    DM_Register_e::PMAX,
    12.5f
);
```

## 16.2 写入 uint32 寄存器

例如修改通信超时时间：

```cpp
Motor_DM4310.Write_Parameter_Uint32(
    DM_Register_e::TIMEOUT,
    100U
);
```

例如修改控制模式：

```cpp
Motor_DM4310.Write_Parameter_Uint32(
    DM_Register_e::CTRL_MODE,
    static_cast<uint32_t>(
        DM_Control_Mode_e::Velocity
    )
);
```

## 16.3 检查写入结果

```cpp
const DM_Parameter_Write_Data_t &Write_Data =
    Motor_DM4310.Get_Parameter_Write_Data();

if ((!Write_Data.Waiting_Response) &&
    Write_Data.Valid &&
    Write_Data.Value_Matched)
{
    // 电机返回值与请求写入值一致
}
```

状态字段：

| 字段 | 说明 |
|---|---|
| `Waiting_Response` | 正在等待写入应答 |
| `Valid` | 收到了格式正确的写入应答 |
| `Value_Matched` | 应答值和请求值完全一致 |
| `Timed_Out` | 等待应答超时 |
| `Float_Value` | float 寄存器应答值 |
| `Uint32_Value` | uint32 寄存器应答值 |

## 16.4 写入后的内部同步

收到合法写入应答后，库会自动同步以下关键配置：

```text
PMAX
VMAX
TMAX
MST_ID
ESC_ID
CTRL_MODE
```

写入 `CTRL_MODE` 后，应改用对应模式的控制函数。

## 16.5 写寄存器注意事项

- 一次只能进行一个参数读取、写入或保存操作；
- 当前有操作等待应答时，新操作会返回 `Busy`；
- 库只检查寄存器类型、读写属性以及少量关键参数范围；
- 普通 float 寄存器的具体有效范围仍需按照电机手册设置；
- 不要向只读寄存器写入数据；
- 修改 `ESC_ID` 或 `MST_ID` 后，硬件 CAN 过滤器可能也需要同步修改；
- 修改 `CAN_BR` 后，MCU 的 CAN 波特率不会自动改变，通信可能立即中断；
- 修改控制模式前，建议先停止周期控制并失能电机。

---

# 17. 保存参数到 Flash

寄存器写入后通常立即生效，但掉电后是否保留取决于是否执行保存。

## 17.1 保存调用

```cpp
Motor_DM4310.Save_Parameters();
```

当前库要求：

```text
收到过有效普通反馈
并且反馈状态为 Disabled
```

否则返回 `Error`。

## 17.2 推荐流程

```cpp
Motor_DM4310.Disable();
```

等待普通反馈确认失能：

```cpp
const DM_Motor_Feedback_t &Feedback =
    Motor_DM4310.Get_Feedback_Data();

if (Feedback.Online &&
    (Feedback.States ==
     DM_Motor_States_e::Disabled))
{
    Motor_DM4310.Save_Parameters();
}
```

## 17.3 检查保存结果

```cpp
const DM_Parameter_Save_Data_t &Save_Data =
    Motor_DM4310.Get_Parameter_Save_Data();

if (Save_Data.Valid)
{
    // 收到合法保存应答
}
else if (Save_Data.Timed_Out)
{
    // 等待保存应答超时
}
```

## 17.4 Flash 注意事项

Flash 擦写寿命有限：

- 不要在初始化函数中自动保存；
- 不要在周期任务中调用；
- 不要每次写一个寄存器就自动保存；
- 建议完成一组参数修改后，人工确认无误，再保存一次。

---

# 18. 手动切换控制模式

当前库没有实现自动模式切换状态机，但可以直接写入 `CTRL_MODE`。

推荐手动流程：

```text
停止发送原模式控制帧
→ 将目标速度或输出降为0
→ 发送Disable()
→ 等待普通反馈为Disabled
→ 写CTRL_MODE
→ 等待写入应答Valid且Value_Matched
→ 根据需要Save_Parameters()
→ 开始发送新模式的安全初始目标
→ 发送Enable()
```

示例：切换到速度模式：

```cpp
Motor_DM4310.Write_Parameter_Uint32(
    DM_Register_e::CTRL_MODE,
    static_cast<uint32_t>(
        DM_Control_Mode_e::Velocity
    )
);
```

确认写入成功后：

```cpp
const DM_Parameter_Write_Data_t &Write_Data =
    Motor_DM4310.Get_Parameter_Write_Data();

if (Write_Data.Valid &&
    Write_Data.Value_Matched)
{
    Motor_DM4310.Control_Velocity(0.0f);
    Motor_DM4310.Enable();
}
```

切换到位置相关模式时，建议先读取当前精确位置，或者至少使用普通反馈当前位置作为初始目标，避免使能瞬间产生大位置误差。

---

# 19. 完整速度模式示例

```cpp
Class_DM4310_Motor Motor_DM4310;

void App_Motor_Init(void)
{
    bool Result = Motor_DM4310.Init(
        &FDCAN1_Adapter,
        0x01U,
        0x000U,
        DM_Control_Mode_e::Velocity,
        12.5f,
        30.0f,
        12.5f,
        100U
    );

    if (!Result)
    {
        return;
    }

    Motor_DM4310.Control_Velocity(0.0f);
    Motor_DM4310.Enable();
}

void App_Motor_Task_1ms(void)
{
    Motor_DM4310.Update_Online_States();
    Motor_DM4310.Update_Parameter_States();

    Motor_DM4310.Control_Velocity(
        Target_Velocity
    );
}

void App_CAN_Rx(
    uint16_t CAN_ID,
    const uint8_t *Data,
    uint8_t Length
)
{
    Motor_DM4310.Process_CAN_Message(
        CAN_ID,
        Data,
        Length
    );
}
```

---

# 20. 常见问题

## 20.1 控制函数返回 Error

常见原因：

- 电机对象未成功初始化；
- 初始化模式和调用的控制函数不匹配；
- 目标参数不合法；
- CAN ID 或数据长度异常；
- CAN 接口对象为空。

## 20.2 电机没有反馈

检查：

- `Master_ID` 是否与电机内部反馈 ID 一致；
- CAN 过滤器是否允许该 ID；
- 接收回调是否调用了 `Process_CAN_Message()`；
- CAN 波特率是否一致；
- 是否持续发送了有效控制帧；
- 电机是否供电和正确接地。

## 20.3 反馈值比例不正确

优先检查：

```text
Position_Max 是否等于电机 PMAX
Velocity_Max 是否等于电机 VMAX
Torque_Max 是否等于电机 TMAX
```

## 20.4 位置模式使能后突然运动

原因通常是：

```text
发送的目标位置和电机当前位置相差过大
```

使能前先发送当前位置作为目标，再逐渐修改目标位置。

## 20.5 参数操作一直 Busy

需要持续调用：

```cpp
Motor_DM4310.Update_Parameter_States();
```

参数操作超时后，`Waiting_Response` 才会被清除。

---

# 21. 重要安全提示

1. 第一次调试必须可靠固定电机。
2. 所有模式都应从小速度、小扭矩和低增益开始。
3. 位置模式使能前，目标位置应先等于当前位置。
4. MIT 位置控制不要一开始使用很大的 `Kp`。
5. MIT 扭矩控制应从很小的前馈扭矩开始。
6. 力位混控应从较小的电流标幺值开始。
7. `Set_Zero_Position()` 不要在每次上电时自动调用。
8. `Save_Parameters()` 不要频繁调用。
9. 写入 `CAN_BR`、`ESC_ID`、`MST_ID` 后，外部 CAN 配置可能需要同步修改。
10. `Enable()`、`Disable()` 的返回值只代表发送状态，真实状态必须看反馈。
