/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Motor_DM4310.h
  * @brief   This file contains all the function prototypes for
  *          the Motor_DM4310.c/.cpp file
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_DM4310_H__
#define __MOTOR_DM4310_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "CAN_Interface.h"
/*YOUR CODE*/

//达妙电机特殊控制命令
#define DM_Command_Clear_Error          0xFB    //清除错误
#define DM_Command_Enable               0xFC    //使能
#define DM_Command_Disable              0xFD    //失能
#define DM_Command_Set_Zero_Position    0xFE    //设置位置零点

/**
 * @brief 达妙电机模式
 * 
 */
enum class DM_Control_Mode_e : uint8_t
{
    MIT = 1,//MIT模式
    Position_Velocity = 2,//位置速度模式
    Velocity = 3,//速度模式
    Force_Position_Hybrid = 4//力位置混合模式
};

/**
 * @brief 达妙电机运行状态和故障状态
 */
enum class DM_Motor_States_e : uint8_t
{
    Disabled = 0x0,//失能
    Enabled = 0x1,//使能

    Over_Voltage = 0x8,//过压
    Under_Voltage = 0x9,//欠压
    Over_Current = 0xA,//过流
    MOS_Over_Temperature = 0xB,//MOS过温
    Motor_Over_Temperature = 0xC,//电机过温
    Communication_Lost = 0xD,//通信丢失
    Overload = 0xE//过载
};

/**
 * @brief 达妙电机反馈数据
 * 
 */
typedef struct
{
    uint8_t Motor_ID = 0;
    DM_Motor_States_e States = DM_Motor_States_e::Disabled;
   
    uint16_t Raw_Position = 0;//原始位置
    uint16_t Raw_Velocity = 0;//原始速度
    uint16_t Raw_Torque = 0;//原始力矩

    float Position = 0;//位置
    float Velocity = 0;//速度
    float Torque = 0;//力矩

    uint8_t Mos_Temperature = 0;//驱动MOS平均温度
    uint8_t Rotor_Temperature = 0;//电机内部线圈平均温度

    bool Online = false;//在线
    uint32_t Last_Feedback_Time = 0;//上次接收时间
} DM_Motor_Feedback_t;

/**
 * @brief 达妙电机配置参数
 * 
 */
typedef struct
{
    uint16_t Motor_ID = 0;
    uint16_t Master_ID = 0;

    DM_Control_Mode_e Control_Mode = DM_Control_Mode_e::MIT;

    float Position_Max = 0;
    float Velocity_Max = 0;
    float Torque_Max = 0;

    uint32_t Feedback_Timeout_ms = 100;//超时时间
} DM_Motor_Config_t;

/**
 * @brief MIT模式控制数据
 */
typedef struct
{
    /**
     * @brief 目标位置，单位rad
     */
    float Target_Position_Rad = 0.0f;

    /**
     * @brief 目标位置，单位deg
     */
    float Target_Position_Degree = 0.0f;

    /**
     * @brief 目标速度，单位rad/s
     */
    float Target_Velocity = 0.0f;

    /**
     * @brief 位置比例系数，范围0~500
     */
    float Position_Kp = 0.0f;

    /**
     * @brief 速度微分系数，范围0~5
     */
    float Velocity_Kd = 0.0f;

    /**
     * @brief 前馈扭矩
     */
    float Feedforward_Torque = 0.0f;
} DM_MIT_Mode_Data_t;

/**
 * @brief 速度模式控制数据
 */
typedef struct
{
    float Target_Velocity = 0.0f;
} DM_Velocity_Mode_Data_t;

/**
 * @brief 位置速度模式控制数据
 */
typedef struct
{
    //目标位置，单位rad
       float Target_Position_Rad = 0.0f;

    //目标位置，单位deg
    float Target_Position_Degree = 0.0f;

    //运动过程中的最大绝对速度，单位rad/s
    float Velocity_Limit = 0.0f;
} DM_Position_Velocity_Mode_Data_t;

/**
 * @brief 力位混控模式控制数据
 */
typedef struct
{
    /**
     * @brief 目标位置，单位rad
     */
    float Target_Position_Rad = 0.0f;

     /**
     * @brief 目标位置，单位deg
     */
    float Target_Position_Degree = 0.0f;

    /**
     * @brief 运动速度限制，单位rad/s，范围0~100
     */
    float Velocity_Limit = 0.0f;

    /**
     * @brief 扭矩电流限制标幺值，范围0~1
     */
    float Current_Limit_Per_Unit = 0.0f;
} DM_Force_Position_Hybrid_Mode_Data_t;

//寄存器收发相关

/**
 * @brief 达妙电机寄存器地址
 */
enum class DM_Register_e : uint8_t
{
    UV_Value   = 0x00,  //低压保护值
    KT_Value   = 0x01,  //扭矩系数
    OT_Value   = 0x02,  //过温保护值
    OC_Value   = 0x03,  //过流保护值
    ACC        = 0x04,  //加速度
    DEC        = 0x05,  //减速度
    MAX_SPD    = 0x06,  //最大速度
    MST_ID     = 0x07,  //反馈ID
    ESC_ID     = 0x08,  //接收ID
    TIMEOUT    = 0x09,  //超时报警时间
    CTRL_MODE  = 0x0A,  //控制模式
    Damp       = 0x0B,  //电机粘滞系数
    Inertia    = 0x0C,  //电机转动惯量
    HW_Ver     = 0x0D,  //保留
    SW_Ver     = 0x0E,  //软件版本号
    SN         = 0x0F,  //保留
    NPP        = 0x10,  //电机极对数
    Rs         = 0x11,  //电机相电阻
    Ls         = 0x12,  //电机相电感
    Flux       = 0x13,  //电机磁链值
    Gr         = 0x14,  //齿轮减速比
    PMAX       = 0x15,  //位置映射范围
    VMAX       = 0x16,  //速度映射范围
    TMAX       = 0x17,  //扭矩映射范围
    I_BW       = 0x18,  //电流环控制带宽
    KP_ASR     = 0x19,  //速度环Kp
    KI_ASR     = 0x1A,  //速度环Ki
    KP_APR     = 0x1B,  //位置环Kp
    KI_APR     = 0x1C,  //位置环Ki
    OV_Value   = 0x1D,  //过压保护值
    GREF       = 0x1E,  //齿轮力矩效率
    Deta       = 0x1F,  //速度环阻尼系数
    V_BW       = 0x20,  //速度环滤波带宽
    IQ_C1      = 0x21,  //电流环增强系数
    VL_C1      = 0x22,  //速度环增强系数
    CAN_BR     = 0x23,  //CAN波特率代码
    Sub_Ver    = 0x24,  //子版本号
    Boot_Ver   = 0x25,  //Boot版本号

    Direction  = 0x37,  //方向
    Motor_Offset = 0x38,//电机侧角度偏移
    Imax       = 0x3B,  //驱动板最大电流
    VBus       = 0x3C,  //电源电压
    Tpcb       = 0x3D,  //驱动板温度
    Tmtr       = 0x3E,  //电机温度
    I_U_Offset = 0x3F,  //U相电流偏置
    I_V_Offset = 0x40,  //V相电流偏置
    I_W_Offset = 0x41,  //W相电流偏置

    Motor_Position = 0x50,//电机当前位置
    Output_Position = 0x51//输出轴位置
};

/**
 * @brief 达妙寄存器数据类型
 */
enum class DM_Register_Data_Type_e : uint8_t
{
    Invalid,
    Float,
    Uint32
};

/**
 * @brief 达妙电机参数读取数据
 */
typedef struct
{
    /**
     * @brief 当前请求或最近一次成功读取的寄存器
     */
    DM_Register_e Register = DM_Register_e::UV_Value;

    /**
     * @brief 寄存器数据类型
     */
    DM_Register_Data_Type_e Data_Type = DM_Register_Data_Type_e::Invalid;

    /**
     * @brief D4~D7拼接得到的原始32位数据
     */
    uint32_t Raw_Value = 0;

    /**
     * @brief 当寄存器类型为float时使用
     */
    float Float_Value = 0.0f;

    /**
     * @brief 当寄存器类型为uint32时使用
     */
    uint32_t Uint32_Value = 0;

    /**
     * @brief 当前是否正在等待该寄存器的应答
     */
    bool Waiting_Response = false;

    /**
     * @brief 是否已经收到有效参数数据
     */
    bool Valid = false;

    /**
     * @brief 最近一次成功读取参数的时间
     */
    uint32_t Last_Update_Time = 0U;

    /**
    * @brief 请求发送时间
    */
    uint32_t Request_Time = 0U;

    /**
    * @brief 是否等待超时
    */
    bool Timed_Out = false;

} DM_Parameter_Data_t;

/**
 * @brief 达妙电机参数写入状态
 */
typedef struct
{
    /**
     * @brief 当前请求写入的寄存器
     */
    DM_Register_e Register = DM_Register_e::UV_Value;

    /**
     * @brief 寄存器数据类型
     */
    DM_Register_Data_Type_e Data_Type = DM_Register_Data_Type_e::Invalid;

    /**
     * @brief 请求写入的32位原始数据
     */
    uint32_t Request_Raw_Value = 0U;

    /**
     * @brief 电机应答返回的32位原始数据
     */
    uint32_t Response_Raw_Value = 0U;

    /**
     * @brief 当寄存器类型为float时，保存返回值
     */
    float Float_Value = 0.0f;

    /**
     * @brief 当寄存器类型为uint32时，保存返回值
     */
    uint32_t Uint32_Value = 0U;

    /**
     * @brief 当前是否正在等待写入应答
     */
    bool Waiting_Response = false;

    /**
     * @brief 是否收到合法写入应答
     */
    bool Valid = false;

    /**
     * @brief 返回值是否与请求写入值一致
     */
    bool Value_Matched = false;

    /**
     * @brief 最近一次写入应答时间
     */
    uint32_t Last_Update_Time = 0U;

    /**
    * @brief 请求发送时间
    */
    uint32_t Request_Time = 0U;

    /**
    * @brief 是否等待超时
    */
    bool Timed_Out = false;
    
} DM_Parameter_Write_Data_t;

/**
 * @brief 达妙电机参数存储状态
 */
typedef struct
{
    /**
     * @brief 是否正在等待电机返回存储应答
     */
    bool Waiting_Response = false;

    /**
     * @brief 是否收到合法的存储应答
     */
    bool Valid = false;

    /**
     * @brief 本次操作是否等待超时
     */
    bool Timed_Out = false;

    /**
     * @brief 发送存储请求的时间
     */
    uint32_t Request_Time = 0;

    /**
     * @brief 最近一次成功存储的时间
     */
    uint32_t Last_Update_Time = 0;
} DM_Parameter_Save_Data_t;

class Class_DM4310_Motor
{
    public:
    
    /**
    * @brief 初始化
    * 
    * @param CAN_Interface CAN接口
    * @param Motor_ID 电机ID
    * @param Master_ID CANID
    * @param Control_Mode 控制模式
    * @param Position_Max 位置MAX
    * @param velocity_Max 速度MAX
    * @param Torque_Max 力矩MAX
    * @param Feedback_Timeout_ms 超时时间，单位毫秒
    * @return true 初始化成功
    * @return false 初始化失败
    */
    bool Init(Class_CAN_Interface *CAN_Interface,
                              uint8_t Motor_ID,uint16_t Master_ID,DM_Control_Mode_e Control_Mode,
                              float Position_Max,float Velocity_Max,float Torque_Max,
                              uint32_t Feedback_Timeout_ms = 100U);

    /**
    * @brief 请求读取一个电机寄存器
    *
    * @param Register 需要读取的寄存器
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    *
    * @note 该函数只发送读取请求，参数值通过后续CAN应答获得
    */
    Enum_CAN_Transmit_Status_e Read_Parameter(DM_Register_e Register);

    /**
    * @brief 统一处理达妙电机接收到的CAN报文
    *
    * @param CAN_ID 接收帧ID
    * @param Data 接收数据
    * @param Length 数据长度
    *
    * @return true 报文属于本电机并成功处理
    * @return false 报文不属于本电机或格式错误
    */
    bool Process_CAN_Message(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length);

    /**
    * @brief 获取最近一次参数读取数据
    */
    const DM_Parameter_Data_t &Get_Parameter_Data(void) const;

    /**
    * @brief 写入float类型寄存器
    *
    * @param Register 寄存器地址
    * @param Value 需要写入的float值
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    *
    * @note 写入立即生效，但掉电后丢失，需要另行保存参数
    */
    Enum_CAN_Transmit_Status_e Write_Parameter_Float(DM_Register_e Register,float Value);

    /**
    * @brief 写入uint32类型寄存器
    *
    * @param Register 寄存器地址
    * @param Value 需要写入的uint32值
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    *
    * @note 写入立即生效，但掉电后丢失，需要另行保存参数
    */
    Enum_CAN_Transmit_Status_e Write_Parameter_Uint32(DM_Register_e Register,uint32_t Value);

    /**
    * @brief 获取最近一次参数写入状态
    */
    const DM_Parameter_Write_Data_t &Get_Parameter_Write_Data(void) const;

    /**
    * @brief 将当前全部参数存储到电机内部Flash
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    *
    * @warning 只有电机处于失能状态时才允许执行
    * @warning Flash擦写寿命有限，不应频繁调用
    */
    Enum_CAN_Transmit_Status_e Save_Parameters(void);

    /**
    * @brief 更新参数操作的超时状态
    *
    * @note 建议放在周期任务中调用
    */
    void Update_Parameter_States(void);

    /**
    * @brief 获取参数存储状态
    */
    const DM_Parameter_Save_Data_t &Get_Parameter_Save_Data(void) const;

    /**
    * @brief 处理达妙电机普通状态反馈帧
    *
    * @param CAN_ID 接收到的CAN标准帧ID
    * @param Data 接收到的数据
    * @param Length 数据长度
    *
    * @return true 该帧属于本电机且解析成功
    * @return false 该帧不属于本电机或数据非法
    */
    bool Process_CAN_Feedback(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length);

    /**
    * @brief 获取电机反馈数据
    *
    * @return const DM_Motor_Feedback_t& 只读反馈数据引用
    */
    const DM_Motor_Feedback_t &Get_Feedback_Data(void) const;

    /**
    * @brief 使能电机
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    */
    Enum_CAN_Transmit_Status_e Enable(void);

    /**
    * @brief 失能电机
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    */
    Enum_CAN_Transmit_Status_e Disable(void);

    /**
    * @brief 将电机当前位置设置为零点
    *
    * @warning 该命令会修改电机的位置零点，不应在每次初始化时自动调用
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    */
    Enum_CAN_Transmit_Status_e Set_Zero_Position(void);

    /**
    * @brief 清除电机故障
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    */
    Enum_CAN_Transmit_Status_e Clear_Error(void);

    /**
    * @brief 使用MIT模式控制电机
    *
    * @param Target_Position_Rad 目标位置，单位rad
    * @param Target_Velocity 目标速度，单位rad/s
    * @param Position_Kp 位置比例系数，范围0~500
    * @param Velocity_Kd 速度微分系数，范围0~5
    * @param Feedforward_Torque 前馈扭矩
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    *
    * @note 电机的软件配置模式必须为MIT模式
    */
    Enum_CAN_Transmit_Status_e Control_MIT_Rad(float Target_Position_Rad,float Target_Velocity,float Position_Kp,float Velocity_Kd,float Feedforward_Torque);

    /**
    * @brief 使用MIT模式控制电机
    *
    * @param Target_Position_Degree 目标位置，单位deg
    * @param Target_Velocity 目标速度，单位rad/s
    * @param Position_Kp 位置比例系数，范围0~500
    * @param Velocity_Kd 速度微分系数，范围0~5
    * @param Feedforward_Torque 前馈扭矩
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    *
    * @note 电机的软件配置模式必须为MIT模式
    */
    Enum_CAN_Transmit_Status_e Control_MIT_Degree(float Target_Position_Degree,float Target_Velocity,float Position_Kp,float Velocity_Kd,float Feedforward_Torque);

    /**
    * @brief 使用速度模式控制电机
    *
    * @param Target_Velocity 目标速度，单位rad/s
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    *
    * @note 电机的软件配置模式必须为速度模式
    */
    Enum_CAN_Transmit_Status_e Control_Velocity(float Target_Velocity);

    /**
    * @brief 使用位置速度模式控制电机
    *
    * @param Target_Position 目标位置，单位rad
    * @param Velocity_Limit 运动过程中的最大绝对速度，单位rad/s
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    *
    * @note 电机的软件配置模式必须为位置速度模式
    */
    Enum_CAN_Transmit_Status_e Control_Position_Velocity_Rad(float Target_Position_Rad,float Velocity_Limit);

    /**
    * @brief 使用位置速度模式控制电机
    *
    * @param Target_Position_Degree 目标位置，单位deg
    * @param Velocity_Limit 运动过程中的最大绝对速度，单位rad/s
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    *
    * @note 电机的软件配置模式必须为位置速度模式
    */
    Enum_CAN_Transmit_Status_e Control_Position_Velocity_Degree(float Target_Position_Degree,float Velocity_Limit);

    /**
    * @brief 使用力位混控模式控制电机
    *
    * @param Target_Position 目标位置，单位rad
    * @param Velocity_Limit 速度限制，单位rad/s，范围0~100
    * @param Current_Limit_Per_Unit 扭矩电流限制标幺值，范围0~1
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    *
    * @note 电机的软件配置模式必须为力位混控模式
    */
    Enum_CAN_Transmit_Status_e Control_Force_Position_Hybrid_Rad(float Target_Position_Rad,float Velocity_Limit,float Current_Limit_Per_Unit);

    /**
    * @brief 使用力位混控模式控制电机
    *
    * @param Target_Position_Degree 目标位置，单位deg
    * @param Velocity_Limit 速度限制，单位rad/s
    * @param Current_Limit_Per_Unit 扭矩电流限制标幺值
    *
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    *
    * @note 电机的软件配置模式必须为力位混控模式
    */
    Enum_CAN_Transmit_Status_e Control_Force_Position_Hybrid_Degree(float Target_Position_Degree,float Velocity_Limit,float Current_Limit_Per_Unit);

    /**
    * @brief 更新电机在线状态
    *
    * 根据最后一次合法反馈的时间判断电机是否掉线。
    * 建议在周期任务中调用。
    */
    void Update_Online_States(void);

    /**
    * @brief 获取电机当前在线状态
    *
    * @return true 电机在线
    * @return false 电机离线
    */
    bool Get_Online_States(void) const;

    //----------私有成员函数和变量----------
    private:

    /**
     * @brief 获取寄存器对应的数据类型
     * 
     * @param Register 寄存器地址
     * @return DM_Register_Data_Type_e 寄存器对应数据类型
     */
    static DM_Register_Data_Type_e Get_Register_Data_Type(DM_Register_e Register);

    /**
    * @brief 处理参数读取应答帧
    */
    bool Process_Parameter_Read_Response(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length);
    
    /**
    * @brief 判断寄存器是否允许写入
    * 
    * @param Register 寄存器地址
    * @return true 寄存器允许写入（读写）
    * @return false 寄存器不允许写入（只读）
    */
    static bool Is_Register_Writable(DM_Register_e Register);

    /**
    * @brief 发送原始32位参数写入请求
    * 
    * @param Register 寄存器地址
    * @param Data_Type 寄存器数据类型
    * @param Raw_Value 原始32位值
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    */
    Enum_CAN_Transmit_Status_e Write_Parameter_Raw(DM_Register_e Register,DM_Register_Data_Type_e Data_Type,uint32_t Raw_Value);

    /**
    * @brief 处理参数写入应答
    * 
    * @param CAN_ID CAN总线ID
    * @param Data 数据指针
    * @param Length 数据长度
    * @return true 处理成功
    * @return false 处理失败
    */
    bool Process_Parameter_Write_Response(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length);

    /**
    * @brief 判断当前是否存在尚未完成的参数操作
    */
    bool Is_Parameter_Operation_Busy(void) const;

    /**
    * @brief 处理参数存储应答
    */
    bool Process_Parameter_Save_Response(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length);

    /**
    * @brief 参数应答等待超时时间
    *
    * 手册说明Flash写入最长约30ms，
    * 因此MCU侧预留100ms。
    */
    static constexpr uint32_t Parameter_Response_Timeout_ms = 100U;

    /**
    * @brief 将uint32_t转换为4字节小端数据
    *
    * @param Value 需要转换的uint32_t数据
    * @param Data 保存结果的4字节数组
    */
    static void Uint32_To_Little_Endian_Bytes(uint32_t Value,uint8_t *Data);

    /**
    * @brief 将4字节小端数据转换为uint32_t
    * 
    * @param Data 指向4字节小端数据的指针
    * @return uint32_t 转换后的无符号整数
    */
    static uint32_t Little_Endian_Bytes_To_Uint32(const uint8_t *Data);

    /**
     * @brief 获取当前控制模式对应的CAN发送ID
     *
     * @return uint16_t 控制报文CAN标准帧ID
     */
    uint16_t Get_Control_CAN_ID(void) const;

    /**
     * @brief 通过统一CAN接口发送数据
     *
     * @param CAN_ID CAN标准帧ID
     * @param Data 数据地址
     * @param Length 数据长度，范围0~8
     * @return Enum_CAN_Transmit_Status_e CAN发送状态
     */
    Enum_CAN_Transmit_Status_e Transmit_Data(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length);

    /**
    * @brief 发送达妙电机特殊控制命令
    *
    * @param Command 特殊控制命令
    * @return Enum_CAN_Transmit_Status_e CAN发送状态
    */
    Enum_CAN_Transmit_Status_e Send_Special_Command(uint8_t Command);

    //----------工具函数----------

    /**
    * @brief 将浮点数线性映射为无符号定点整数
    *
    * @param Value 需要转换的浮点值
    * @param Minimum 映射范围最小值
    * @param Maximum 映射范围最大值
    * @param Bit_Count 目标整数位数
    *
    * @return uint16_t 映射后的整数
    */
    static uint16_t Float_To_Uint(float Value,float Minimum,float Maximum,uint8_t Bit_Count);

    /**
    * @brief 将float数据转换为小端字节序
    *
    * @param Value 需要转换的float数据
    * @param Data 保存结果的4字节数组
    */
    static void Float_To_Little_Endian_Bytes(float Value,uint8_t *Data);

    /**
    * @brief 将无符号定点原始值线性还原为浮点数
    *
    * @param Raw_Value 原始整数
    * @param Minimum 浮点最小值
    * @param Maximum 浮点最大值
    * @param Bit_Count 原始数据位数
    *
    * @return float 还原后的浮点数
    */
    static float Uint_To_Float(uint32_t Raw_Value,float Minimum,float Maximum,uint8_t Bit_Count);

    /**
    * @brief 将uint16_t转换为小端字节序
    *
    * @param Value 需要转换的数据
    * @param Data 保存结果的2字节数组
    */
    static void Uint16_To_Little_Endian_Bytes(uint16_t Value,uint8_t *Data);

    //类、结构体、枚举等

    /**
     * @brief CAN接口
     * 
     */
    Class_CAN_Interface *CAN_Interface_Instance = nullptr;

    /**
    * @brief 参数读取数据
    */
    DM_Parameter_Data_t Parameter_Data;

    /**
    * @brief 参数写入状态
    */
    DM_Parameter_Write_Data_t Parameter_Write_Data;

    /**
    * @brief 参数存储状态
    */
    DM_Parameter_Save_Data_t Parameter_Save_Data;

    /**
     * @brief 电机反馈数据
     * 
     */
    DM_Motor_Feedback_t Feedback_Data;

    /**
     * @brief 电机配置数据
     * 
     */
    DM_Motor_Config_t Config_Data;

    /**
     * @brief 电机初始化标志
     * 
     */
    bool Init_Flag = false;

    /**
    * @brief MIT模式最近一次实际发送的控制数据
    */
    DM_MIT_Mode_Data_t MIT_Mode_Data;

    /**
    * @brief 速度模式最近一次控制数据
    */
    DM_Velocity_Mode_Data_t Velocity_Mode_Data;

    /**
    * @brief 位置速度模式最近一次控制数据
    */
    DM_Position_Velocity_Mode_Data_t Position_Velocity_Mode_Data;

    /**
    * @brief 力位混控模式最近一次实际发送的数据
    */
    DM_Force_Position_Hybrid_Mode_Data_t Force_Position_Hybrid_Mode_Data;
};





#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DM4310_H__ */
