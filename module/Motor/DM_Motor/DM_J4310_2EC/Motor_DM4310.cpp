/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Motor_DM4310.cpp
  * @brief   达妙4310电机
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "Motor_DM4310.h"
#include "CAN_Interface.h"
#include "string.h"
#include "main.h"

//STM32 中 float 确实是 4 字节 如果换到不符合协议的平台，会直接编译报错，而不是发生数组越界或发送错误数据。
static_assert(sizeof(float) == 4U,"DM4310 protocol requires 32-bit float.");

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
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool Class_DM4310_Motor::Init(Class_CAN_Interface *CAN_Interface,
                              uint8_t Motor_ID,uint16_t Master_ID,DM_Control_Mode_e Control_Mode,
                              float Position_Max,float Velocity_Max,float Torque_Max,
                              uint32_t Feedback_Timeout_ms)
{
    Init_Flag = false;

    //防空
    if (CAN_Interface == nullptr)
    {
        return Init_Flag;
    }

    if (Feedback_Timeout_ms == 0)
    {
        return Init_Flag;
    }

    //达妙反馈D[0]的低4位区分电机ID
    //0x0F = 00001111
    if(Motor_ID > 0x0F)
    {
        return Init_Flag;
    }

    //CAN标准帧ID最大为11位
    if(Master_ID > 0x7FF)
    {
        return Init_Flag;
    }

    //检查控制模式是否合法
    switch (Control_Mode)
    {
        case DM_Control_Mode_e::MIT:
        case DM_Control_Mode_e::Position_Velocity:
        case DM_Control_Mode_e::Velocity:
        case DM_Control_Mode_e::Force_Position_Hybrid:
        {
            break;
        }

        default:
        {
            return Init_Flag;
        }
    }

     //必须为正数
    if ((Position_Max <= 0.0f) || (Velocity_Max <= 0.0f) || (Torque_Max <= 0.0f))
    {
        return Init_Flag;
    }

    //初始化内部数据
    Parameter_Data = {};
    Parameter_Write_Data = {};
    Parameter_Save_Data = {};

    Feedback_Data = {};
    Config_Data = {};

    MIT_Mode_Data = {};
    Velocity_Mode_Data = {};
    Position_Velocity_Mode_Data = {};
    Force_Position_Hybrid_Mode_Data = {};

    //保存信息
    this->CAN_Interface_Instance = CAN_Interface;

    Config_Data.Motor_ID = Motor_ID;
    Config_Data.Master_ID = Master_ID;
    Config_Data.Control_Mode = Control_Mode;
    Config_Data.Position_Max = Position_Max;
    Config_Data.Velocity_Max = Velocity_Max;
    Config_Data.Torque_Max = Torque_Max;
    Config_Data.Feedback_Timeout_ms = Feedback_Timeout_ms;

    Feedback_Data.Motor_ID = Motor_ID;
    Feedback_Data.Online = false;
    Feedback_Data.Last_Feedback_Time = 0; 

    Init_Flag = true;

    return Init_Flag;
}

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
bool Class_DM4310_Motor::Process_CAN_Message(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length)
{
    //储存参数应答优先级最高，先判断是否是参数存储应答
    if (Process_Parameter_Save_Response(CAN_ID,Data,Length))
    {
        return true;
    }

    //优先判断当前等待的参数读取应答 传入读取寄存器响应回调
    if (Process_Parameter_Read_Response(CAN_ID,Data,Length))
    {
        return true;
    }
    
    //判断当前等待的参数写入应答 传入写入寄存器响应回调
    if (Process_Parameter_Write_Response(CAN_ID,Data,Length))
    {
        return true;
    }

    //丢弃迟到、重复或当前不再等待的参数应答，防止被当作普通反馈解析。
    if ((Data != nullptr) && (Length == 8U) && (CAN_ID == Config_Data.Master_ID))
    {
        const uint16_t Response_Motor_ID = static_cast<uint16_t>(Data[0]) | (static_cast<uint16_t>(Data[1]) << 8U);
        const DM_Register_e Response_Register = static_cast<DM_Register_e>(Data[3]);
        const bool Is_Known_Register = (Get_Register_Data_Type(Response_Register) != DM_Register_Data_Type_e::Invalid);
        const bool Is_Parameter_Command = (Data[2] == 0x33) || (Data[2] == 0x55);

        if ((Response_Motor_ID == Config_Data.Motor_ID) && Is_Known_Register && Is_Parameter_Command)
        {
            //该帧属于本电机参数协议 但当前不是正在等待的有效应答
            return true;
        }
    }

    //不是参数应答，再按照普通状态反馈处理 传入反馈回调
    if (Process_CAN_Feedback(CAN_ID,Data,Length))
    {
        return true;
    }

    return false;
}

/**
 * @brief 获取当前控制模式对应的CAN发送ID
 *
 * @return uint16_t 控制报文CAN标准帧ID
 */
uint16_t Class_DM4310_Motor::Get_Control_CAN_ID(void) const
{
    switch (Config_Data.Control_Mode)
    {
        //MIT模式
        case DM_Control_Mode_e::MIT:
        {
            return Config_Data.Motor_ID;
        }

        //位置速度模式
        case DM_Control_Mode_e::Position_Velocity:
        {
            return 0x100 + Config_Data.Motor_ID;
        }

        //速度模式
        case DM_Control_Mode_e::Velocity:
        {
            return 0x200 + Config_Data.Motor_ID;
        }

        //力位混合模式
        case DM_Control_Mode_e::Force_Position_Hybrid:
        {
            return 0x300 + Config_Data.Motor_ID;
        }

        //无效
        default:
        {
            return 0xFFFF;
        }
    }
}

///----------CAN发送相关----------
/**
 * @brief 通过统一CAN接口发送数据
 *
 * @param CAN_ID CAN标准帧ID
 * @param Data 数据地址
 * @param Length 数据长度
 * @return Enum_CAN_Transmit_Status_e CAN发送状态
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Transmit_Data(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length)
{
    //防止未完成初始化
    if (!Init_Flag)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //防止CAN接口指针是空的
    if (CAN_Interface_Instance == nullptr)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //检测是否标准帧
    if (CAN_ID > 0x7FFU)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //检测数据长度是否合法
    if (Length > 8U)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //检测数据指针是否合法
    if ((Length > 0U) && (Data == nullptr))
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //发送
    return CAN_Interface_Instance->Transmit(CAN_ID,Data,Length);
}

///----------电机特殊控制相关----------
/**
 * @brief 发送达妙电机特殊控制命令
 *
 * @param Command 特殊控制命令
 * @return Enum_CAN_Transmit_Status_e CAN发送状态
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Send_Special_Command(uint8_t Command)
{ 
    //四个特殊命令格式特殊 D[0]~D[6]为0xFF D[7]为对应的特殊命令
    uint8_t Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,static_cast<uint8_t>(Command)};

    //属于控制帧 需要获取当前控制模式对应的CAN发送ID
    const uint16_t Control_CAN_ID = Get_Control_CAN_ID();

    return Transmit_Data(Control_CAN_ID,Data,8);
}

/**
 * @brief 使能电机
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Enable(void)
{
    return Send_Special_Command(DM_Command_Enable);
}

/**
 * @brief 失能电机
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Disable(void)
{
    return Send_Special_Command(DM_Command_Disable);
}

/**
 * @brief 将电机当前位置设置为零点
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Set_Zero_Position(void)
{
    return Send_Special_Command(DM_Command_Set_Zero_Position);
}

/**
 * @brief 清除电机故障
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Clear_Error(void)
{
    return Send_Special_Command(DM_Command_Clear_Error);
}

///----------反馈相关----------
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
bool Class_DM4310_Motor::Process_CAN_Feedback(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length)
{

    if (!Init_Flag)
    {
        return false;
    }

    if (Data == nullptr)
    {
        return false;
    }

    if (Length != 8U)
    {
        return false;
    }

    if (CAN_ID != Config_Data.Master_ID)
    {
        return false;
    }

    //D[0]低4位为电机ID，高4位为状态码。
    const uint8_t Feedback_Motor_ID = Data[0] & 0x0F;

    const uint8_t States_Code = (Data[0] >> 4U) & 0x0FU;

    //检查是否对应的ID
    if (Feedback_Motor_ID != Config_Data.Motor_ID)
    {
        return false;
    }

    //16位位置：D[1]为高8位，D[2]为低8位
    const uint16_t Position_Raw =(static_cast<uint16_t>(Data[1]) << 8) |static_cast<uint16_t>(Data[2]);

    //12位速度：D[3]为高8位，D[4]高4位为低4位
    const uint16_t Velocity_Raw =(static_cast<uint16_t>(Data[3]) << 4) |(static_cast<uint16_t>(Data[4]) >> 4);

    //12位扭矩：D[4]低4位为高4位，D[5]为低8位
    const uint16_t Torque_Raw =(static_cast<uint16_t>(Data[4] & 0x0FU) << 8) |static_cast<uint16_t>(Data[5]);

    //更新反馈数据
    Feedback_Data.Motor_ID = Feedback_Motor_ID;
    Feedback_Data.States =static_cast<DM_Motor_States_e>(States_Code);

    Feedback_Data.Raw_Position = Position_Raw;
    Feedback_Data.Raw_Velocity = Velocity_Raw;
    Feedback_Data.Raw_Torque = Torque_Raw;

    //按PMAX、VMAX、TMAX进行对称线性映射。
    Feedback_Data.Position = Uint_To_Float(Position_Raw,-Config_Data.Position_Max,Config_Data.Position_Max,16);

    Feedback_Data.Velocity = Uint_To_Float(Velocity_Raw,-Config_Data.Velocity_Max,Config_Data.Velocity_Max,12);

    Feedback_Data.Torque = Uint_To_Float(Torque_Raw,-Config_Data.Torque_Max,Config_Data.Torque_Max,12);

    //Mos and Rotor温度 单位摄氏度
    Feedback_Data.Mos_Temperature = Data[6];
    Feedback_Data.Rotor_Temperature = Data[7];

    //电机在线标志和上次接收时间
    Feedback_Data.Online = true;
    Feedback_Data.Last_Feedback_Time = HAL_GetTick();

    return true;
}

/**
 * @brief 获取电机反馈数据
 */
const DM_Motor_Feedback_t &Class_DM4310_Motor::Get_Feedback_Data(void) const
{
    return Feedback_Data;
}

//----------寄存器相关----------
/**
 * @brief 获取寄存器对应的数据类型
 * 
 * @param Register 寄存器地址
 * @return DM_Register_Data_Type_e 寄存器对应数据类型
 */
DM_Register_Data_Type_e Class_DM4310_Motor::Get_Register_Data_Type(DM_Register_e Register)
{
    switch (Register)
    {
        //uint32_t类型寄存器
        case DM_Register_e::MST_ID:
        case DM_Register_e::ESC_ID:
        case DM_Register_e::TIMEOUT:
        case DM_Register_e::CTRL_MODE:
        case DM_Register_e::HW_Ver:
        case DM_Register_e::SW_Ver:
        case DM_Register_e::SN:
        case DM_Register_e::NPP:
        case DM_Register_e::CAN_BR:
        case DM_Register_e::Sub_Ver:
        case DM_Register_e::Boot_Ver:
        {
            return DM_Register_Data_Type_e::Uint32;
        }

        //float类型寄存器
        case DM_Register_e::UV_Value:
        case DM_Register_e::KT_Value:
        case DM_Register_e::OT_Value:
        case DM_Register_e::OC_Value:
        case DM_Register_e::ACC:
        case DM_Register_e::DEC:
        case DM_Register_e::MAX_SPD:
        case DM_Register_e::Damp:
        case DM_Register_e::Inertia:
        case DM_Register_e::Rs:
        case DM_Register_e::Ls:
        case DM_Register_e::Flux:
        case DM_Register_e::Gr:
        case DM_Register_e::PMAX:
        case DM_Register_e::VMAX:
        case DM_Register_e::TMAX:
        case DM_Register_e::I_BW:
        case DM_Register_e::KP_ASR:
        case DM_Register_e::KI_ASR:
        case DM_Register_e::KP_APR:
        case DM_Register_e::KI_APR:
        case DM_Register_e::OV_Value:
        case DM_Register_e::GREF:
        case DM_Register_e::Deta:
        case DM_Register_e::V_BW:
        case DM_Register_e::IQ_C1:
        case DM_Register_e::VL_C1:
        case DM_Register_e::Direction:
        case DM_Register_e::Motor_Offset:
        case DM_Register_e::Imax:
        case DM_Register_e::VBus:
        case DM_Register_e::Tpcb:
        case DM_Register_e::Tmtr:
        case DM_Register_e::I_U_Offset:
        case DM_Register_e::I_V_Offset:
        case DM_Register_e::I_W_Offset:
        case DM_Register_e::Motor_Position:
        case DM_Register_e::Output_Position:
        {
            return DM_Register_Data_Type_e::Float;
        }

        default:
        {
            return DM_Register_Data_Type_e::Invalid;
        }
    }
}

/**
 * @brief 将4字节小端数据转换为uint32_t
 * 
 * @param Data 指向4字节小端数据的指针
 * @return uint32_t 转换后的无符号整数
 */
uint32_t Class_DM4310_Motor::Little_Endian_Bytes_To_Uint32(const uint8_t *Data)
{
    if (Data == nullptr) return 0;

    return static_cast<uint32_t>(Data[0]) | (static_cast<uint32_t>(Data[1]) << 8) | (static_cast<uint32_t>(Data[2]) << 16) | (static_cast<uint32_t>(Data[3]) << 24);
}

/**
 * @brief 请求读取一个电机寄存器
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Read_Parameter(DM_Register_e Register)
{
    //防止未完成初始化
    if (!Init_Flag)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //防止当前存在尚未完成的参数操作
    if (Is_Parameter_Operation_Busy())
    {
        return Enum_CAN_Transmit_Status_e::Busy;
    }

    //获取寄存器对应的数据类型
    const DM_Register_Data_Type_e Data_Type = Get_Register_Data_Type(Register);

    //防止寄存器无效
    if (Data_Type == DM_Register_Data_Type_e::Invalid)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //获取电机CAN ID
    const uint16_t Motor_CAN_ID = Config_Data.Motor_ID;

    //创建发送数据数组
    uint8_t Data[4] = {0};

    //D0：基础CAN ID低8位 D1：基础CAN ID高8位 D2：读取命令0x33 D3：寄存器地址RID
    Data[0] = static_cast<uint8_t>(Motor_CAN_ID & 0x00FF);

    Data[1] = static_cast<uint8_t>((Motor_CAN_ID >> 8U) & 0x00FF);

    Data[2] = 0x33;

    Data[3] = static_cast<uint8_t>(Register);

    //保存传参 记录当前等待的寄存器
    //新的请求会覆盖旧的等待信息！
    Parameter_Data.Register = Register;
    Parameter_Data.Data_Type = Data_Type;
    Parameter_Data.Raw_Value = 0;
    Parameter_Data.Float_Value = 0.0f;
    Parameter_Data.Uint32_Value = 0;
    Parameter_Data.Valid = false;
    Parameter_Data.Waiting_Response = true;
    Parameter_Data.Timed_Out = false;
    Parameter_Data.Request_Time = HAL_GetTick();

    //发送读取请求
    const Enum_CAN_Transmit_Status_e Transmit_Status =Transmit_Data(0x7FFU,Data,4U);

    //申请发送失败则取消等待状态
    if (Transmit_Status != Enum_CAN_Transmit_Status_e::Success)
    {
        Parameter_Data.Waiting_Response = false;
    }

    return Transmit_Status;
}

/**
 * @brief 处理参数读取应答帧
 * 
 * @param CAN_ID 发送该帧的CAN ID
 * @param Data 指向数据的指针
 * @param Length 数据长度
 * @return true 成功处理应答帧
 * @return false 处理应答帧失败
 */
bool Class_DM4310_Motor::Process_Parameter_Read_Response(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length)
{
    //防止未完成初始化
    if (!Init_Flag) return false;

    //防空
    if (Data == nullptr) return false;

    //防止数据长度不合法
    if (Length != 8) return false;

    //防止CAN ID不合法
    if (CAN_ID != Config_Data.Master_ID) return false;

    //当前没有等待读取结果 则不把该帧当作本次读取应答（可能是其他电机的应答）
    if (!Parameter_Data.Waiting_Response) return false;
    
    //D0、D1为目标电机基础CAN ID。
    const uint16_t Response_Motor_CAN_ID = static_cast<uint16_t>(Data[0]) | (static_cast<uint16_t>(Data[1]) << 8);

    //如果D0、D1不等于本电机的基础CAN ID，则说明该帧不是本电机的应答帧。
    if (Response_Motor_CAN_ID != Config_Data.Motor_ID) return false;

    //D2必须是读取命令0x33。
    if (Data[2] != 0x33) return false;

    //D3必须是当前正在等待的寄存器
    const DM_Register_e Response_Register = static_cast<DM_Register_e>(Data[3]);
    if (Response_Register != Parameter_Data.Register) return false;

    
    //D4~D7为小端32位数据。
    const uint32_t Raw_Value = Little_Endian_Bytes_To_Uint32(&Data[4]);

    //保存读取结果
    Parameter_Data.Raw_Value = Raw_Value;

    //根据寄存器类型保存对应的值
    if (Parameter_Data.Data_Type == DM_Register_Data_Type_e::Float)
    {   
        //复制float位模式，不能进行数值强制转换。
        memcpy(&Parameter_Data.Float_Value,&Raw_Value,sizeof(float));
        Parameter_Data.Uint32_Value = 0;
    }
    else if (Parameter_Data.Data_Type == DM_Register_Data_Type_e::Uint32)
    {
        Parameter_Data.Uint32_Value = Raw_Value;
        Parameter_Data.Float_Value = 0.0f;
    }
    else
    {
        return false;
    }

    //标记为已经收到有效参数数据
    Parameter_Data.Valid = true;
    Parameter_Data.Waiting_Response = false;
    Parameter_Data.Last_Update_Time = HAL_GetTick();
    Parameter_Data.Timed_Out = false;

    return true;
}

/**
 * @brief 获取最近一次参数读取数据
 */
const DM_Parameter_Data_t &Class_DM4310_Motor::Get_Parameter_Data(void) const
{
    return Parameter_Data;
}

/**
 * @brief 判断寄存器是否允许写入
 * 
 * @param Register 寄存器地址
 * @return true 寄存器允许写入（读写）
 * @return false 寄存器不允许写入（只读）
 */
bool Class_DM4310_Motor::Is_Register_Writable(DM_Register_e Register)
{
    //确认寄存器受支持
    if (Get_Register_Data_Type(Register) == DM_Register_Data_Type_e::Invalid)
    {
        return false;
    }

    //将寄存器地址转换为uint8_t类型，方便后续比较
    const uint8_t Register_Address = static_cast<uint8_t>(Register);

    //0x00～0x0A：RW
    if (Register_Address <= 0x0AU)
    {
        return true;
    }

    //0x15～0x23：RW
    if ((Register_Address >= 0x15U) && (Register_Address <= 0x23U))
    {
        return true;
    }

    //0x0B～0x14：RO and 后续寄存器：RO
    return false;
}

/**
 * @brief 将uint32_t转换为4字节小端数据
 *
 * @param Value 需要转换的uint32_t数据
 * @param Data 保存结果的4字节数组
 */
void Class_DM4310_Motor::Uint32_To_Little_Endian_Bytes(uint32_t Value,uint8_t *Data)
{
    if (Data == nullptr)
    {
        return;
    }

    Data[0] = static_cast<uint8_t>(Value & 0x000000FF);
    Data[1] = static_cast<uint8_t>((Value >> 8U) & 0x000000FF);
    Data[2] = static_cast<uint8_t>((Value >> 16U) & 0x000000FF);
    Data[3] = static_cast<uint8_t>((Value >> 24U) & 0x000000FF);
}

/**
 * @brief 发送原始32位参数写入请求
 * 
 * @param Register 寄存器地址
 * @param Data_Type 寄存器数据类型
 * @param Raw_Value 原始32位值
 * @return Enum_CAN_Transmit_Status_e CAN发送状态
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Write_Parameter_Raw(DM_Register_e Register,DM_Register_Data_Type_e Data_Type,uint32_t Raw_Value)
{
    //防止未完成初始化
    if (!Init_Flag) return Enum_CAN_Transmit_Status_e::Error;

    //同一时间只允许等待一个参数操作
    if (Is_Parameter_Operation_Busy()) return Enum_CAN_Transmit_Status_e::Busy;

    //检查寄存器是否允许写入
    if (!Is_Register_Writable(Register)) return Enum_CAN_Transmit_Status_e::Error;

    //检查寄存器数据类型是否合法
    if (Data_Type == DM_Register_Data_Type_e::Invalid) return Enum_CAN_Transmit_Status_e::Error;

    //创建发送数据数组
    uint8_t Data[8] = {0U};

    Data[0] = static_cast<uint8_t>(Config_Data.Motor_ID & 0x00FF);
    Data[1] = static_cast<uint8_t>((Config_Data.Motor_ID >> 8) & 0x00FF);
    Data[2] = 0x55;
    Data[3] = static_cast<uint8_t>(Register);

    Uint32_To_Little_Endian_Bytes(Raw_Value,&Data[4]);

    //保存写入请求信息
    Parameter_Write_Data.Register = Register;
    Parameter_Write_Data.Data_Type = Data_Type;
    Parameter_Write_Data.Request_Raw_Value = Raw_Value;
    Parameter_Write_Data.Response_Raw_Value = 0;
    Parameter_Write_Data.Float_Value = 0.0f;
    Parameter_Write_Data.Uint32_Value = 0;
    Parameter_Write_Data.Waiting_Response = true;
    Parameter_Write_Data.Valid = false;
    Parameter_Write_Data.Value_Matched = false;
    Parameter_Write_Data.Timed_Out = false;
    Parameter_Write_Data.Request_Time = HAL_GetTick();

    //发送写入请求
    const Enum_CAN_Transmit_Status_e States = Transmit_Data(0x7FF,Data,8);

    //发送失败则取消等待状态
    if (States != Enum_CAN_Transmit_Status_e::Success)
    {
        Parameter_Write_Data.Waiting_Response = false;
    }

    return States;
}

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
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Write_Parameter_Float(DM_Register_e Register,float Value)
{
    //检查该寄存器本身是否为float类型。
    if (Get_Register_Data_Type(Register) != DM_Register_Data_Type_e::Float) return Enum_CAN_Transmit_Status_e::Error;

    uint32_t Raw_Value = 0;
    
    //复制float位模式，不是进行数值强制转换。
    memcpy(&Raw_Value,&Value,sizeof(float));

    //发送
    return Write_Parameter_Raw(Register,DM_Register_Data_Type_e::Float,Raw_Value);
}

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
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Write_Parameter_Uint32(DM_Register_e Register,uint32_t Value)
{
    //检查该寄存器本身是否为uint32类型。
    if (Get_Register_Data_Type(Register) != DM_Register_Data_Type_e::Uint32) return Enum_CAN_Transmit_Status_e::Error;
    
    
    //对几个影响通信和控制的关键寄存器进行基本检查。
    if ((Register == DM_Register_e::MST_ID) && (Value > 0x7FF))
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    
    //当前库利用反馈D0低4位区分电机 因此这里继续限制ESC_ID不超过0x0F。
    if ((Register == DM_Register_e::ESC_ID) && (Value > 0x0F))
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //控制模式寄存器只能写入1~4
    if ((Register == DM_Register_e::CTRL_MODE) && ((Value < 1) || (Value > 4)))
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //CAN波特率寄存器只能写入0~9
    if ((Register == DM_Register_e::CAN_BR) && (Value > 9))
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //发送
    return Write_Parameter_Raw(Register,DM_Register_Data_Type_e::Uint32,Value);
}

/**
 * @brief 处理参数写入应答
 * 
 * @param CAN_ID CAN总线ID
 * @param Data 数据指针
 * @param Length 数据长度
 * @return true 处理成功
 * @return false 处理失败
 */
bool Class_DM4310_Motor::Process_Parameter_Write_Response(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length)
{
    if (!Init_Flag) return false;
    if (Data == nullptr) return false;
    if (Length != 8) return false;

    if (!Parameter_Write_Data.Waiting_Response) return false;

    //应答ID等于当前Master ID(一般) 写入MST_ID时，手册没有明确说明确认帧使用旧ID还是立即使用新ID，因此这里同时接受两者。
    bool CAN_ID_Matched = (CAN_ID == Config_Data.Master_ID);

    if ((Parameter_Write_Data.Register == DM_Register_e::MST_ID) && (Parameter_Write_Data.Data_Type == DM_Register_Data_Type_e::Uint32))
    {
        const uint16_t New_Master_ID = static_cast<uint16_t>(Parameter_Write_Data.Request_Raw_Value);

        if (CAN_ID == New_Master_ID)
        {
            CAN_ID_Matched = true;
        }
    }

    //如果CAN ID不匹配，则说明该帧不是本电机的应答帧。
    if (!CAN_ID_Matched)
    {
        return false;
    }
    
    //D0、D1应为本次请求使用的基础Motor ID。
    const uint16_t Response_Motor_ID =static_cast<uint16_t>(Data[0]) | (static_cast<uint16_t>(Data[1]) << 8);

    bool Motor_ID_Matched = (Response_Motor_ID == Config_Data.Motor_ID);

    
    //修改ESC_ID时，保险起见同时接受新ID。
    if (Parameter_Write_Data.Register == DM_Register_e::ESC_ID)
    {
        const uint16_t New_Motor_ID = static_cast<uint16_t>(Parameter_Write_Data.Request_Raw_Value);

        if (Response_Motor_ID == New_Motor_ID)
        {
            Motor_ID_Matched = true;
        }
    }

    //如果电机ID不匹配，则说明该帧不是本电机的应答帧。
    if (!Motor_ID_Matched)
    {
        return false;
    }

    //D2必须是写入命令0x55。
    if (Data[2] != 0x55U)
    {
        return false;
    }

    //D3必须是当前正在等待的寄存器
    const DM_Register_e Response_Register = static_cast<DM_Register_e>(Data[3]);

    //如果寄存器不匹配，则说明该帧不是本电机的应答帧。
    if (Response_Register != Parameter_Write_Data.Register)
    {
        return false;
    }

    //D4~D7为小端32位数据
    const uint32_t Response_Raw_Value = Little_Endian_Bytes_To_Uint32(&Data[4]);

    //保存应答值
    Parameter_Write_Data.Response_Raw_Value = Response_Raw_Value;
    Parameter_Write_Data.Value_Matched = (Response_Raw_Value == Parameter_Write_Data.Request_Raw_Value);

    //根据寄存器类型保存对应的值
    if (Parameter_Write_Data.Data_Type == DM_Register_Data_Type_e::Float)
    {
        memcpy(&Parameter_Write_Data.Float_Value,&Response_Raw_Value,sizeof(float));

        Parameter_Write_Data.Uint32_Value = 0U;
    }
    else if (Parameter_Write_Data.Data_Type == DM_Register_Data_Type_e::Uint32)
    {
        Parameter_Write_Data.Uint32_Value = Response_Raw_Value;
        Parameter_Write_Data.Float_Value = 0.0f;
    }
    else
    {
        return false;
    }

    //标记为已经收到有效参数数据
    Parameter_Write_Data.Waiting_Response = false;
    Parameter_Write_Data.Valid = true;
    Parameter_Write_Data.Last_Update_Time = HAL_GetTick();
    Parameter_Write_Data.Timed_Out = false;

    //根据电机返回的实际值，同步类内部关键配置。
    switch (Parameter_Write_Data.Register)
    {
        case DM_Register_e::PMAX:
        {
            if (Parameter_Write_Data.Float_Value > 0.0f)
            {
                Config_Data.Position_Max = Parameter_Write_Data.Float_Value;
            }
            break;
        }
        case DM_Register_e::VMAX:
        {
            if (Parameter_Write_Data.Float_Value > 0.0f)
            {
                Config_Data.Velocity_Max = Parameter_Write_Data.Float_Value;
            }
            break;
        }
        case DM_Register_e::TMAX:
        {
            if (Parameter_Write_Data.Float_Value > 0.0f)
            {
                Config_Data.Torque_Max = Parameter_Write_Data.Float_Value;
            }
            break;
        }
        case DM_Register_e::MST_ID:
        {
            if (Parameter_Write_Data.Uint32_Value <= 0x7FF)
            {
                Config_Data.Master_ID = static_cast<uint16_t>(Parameter_Write_Data.Uint32_Value);
            }
            break;
        }
        case DM_Register_e::ESC_ID:
        {
            if (Parameter_Write_Data.Uint32_Value <= 0x0F)
            {
                Config_Data.Motor_ID = static_cast<uint16_t>(Parameter_Write_Data.Uint32_Value);

                Feedback_Data.Motor_ID =static_cast<uint8_t>(Config_Data.Motor_ID);
            }
            break;
        }
        case DM_Register_e::CTRL_MODE:
        {
            const uint32_t Mode = Parameter_Write_Data.Uint32_Value;

            if ((Mode >= 1) && (Mode <= 4))
            {
                Config_Data.Control_Mode = static_cast<DM_Control_Mode_e>(Mode);

                //手册说明模式切换时控制命令会归零，
                //软件侧保存值也同步清零。
                MIT_Mode_Data = {};
                Velocity_Mode_Data = {};
                Position_Velocity_Mode_Data = {};
                Force_Position_Hybrid_Mode_Data = {};
            }
            break;
        }
        default:
        {
            break;
        }
    }

    return true;
}

/**
 * @brief 获取最近一次参数写入状态
 */
const DM_Parameter_Write_Data_t &Class_DM4310_Motor::Get_Parameter_Write_Data(void) const
{
    return Parameter_Write_Data;
}

/**
 * @brief 判断当前是否存在尚未完成的参数操作
 */
bool Class_DM4310_Motor::Is_Parameter_Operation_Busy(void) const
{
    return Parameter_Data.Waiting_Response || Parameter_Write_Data.Waiting_Response || Parameter_Save_Data.Waiting_Response;
}

/**
 * @brief 将当前全部参数存储到电机内部Flash
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Save_Parameters(void)
{
    //防止未完成初始化
    if (!Init_Flag)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //防止当前存在尚未完成的参数操作
    if (Is_Parameter_Operation_Busy())
    {
        return Enum_CAN_Transmit_Status_e::Busy;
    }
    
    //手册规定存储参数仅在失能模式下有效 要求电机在线！是为了避免对象初始化后的默认Disabled状态被误认为真实的电机状态。
    if ((!Feedback_Data.Online) || (Feedback_Data.States != DM_Motor_States_e::Disabled))
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    uint8_t Data[4] = {0U};
    
    //参数协议使用基础Motor ID。
    Data[0] = static_cast<uint8_t>(Config_Data.Motor_ID & 0x00FF);
    Data[1] = static_cast<uint8_t>((Config_Data.Motor_ID >> 8) & 0x00FF);

    //存储参数固定命令。
    Data[2] = 0xAA;
    Data[3] = 0x01;

    //先设置等待状态，再发送请求。
    Parameter_Save_Data.Waiting_Response = true;
    Parameter_Save_Data.Valid = false;
    Parameter_Save_Data.Timed_Out = false;
    Parameter_Save_Data.Request_Time = HAL_GetTick();

    const Enum_CAN_Transmit_Status_e Status =Transmit_Data(0x7FF,Data,4);

    
    //CAN发送失败时，不继续等待应答。
    if (Status != Enum_CAN_Transmit_Status_e::Success)
    {
        Parameter_Save_Data.Waiting_Response = false;
    }

    return Status;
}

/**
 * @brief 处理参数存储应答
 * 
 * @param CAN_ID CAN总线ID
 * @param Data 数据指针
 * @param Length 数据长度
 * @return true 处理成功
 * @return false 处理失败
 */
bool Class_DM4310_Motor::Process_Parameter_Save_Response(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length)
{
    if (!Init_Flag) return false;
    if (Data == nullptr) return false;
    //存储应答长度固定为4字节。
    if (Length != 4U) return false;

    //当前没有等待存储应答 则该帧不属于本次存储操作。
    if (!Parameter_Save_Data.Waiting_Response) return false;

    //存储应答必须来自当前Master ID
    if (CAN_ID != Config_Data.Master_ID) return false;

    //D0和D1为基础Motor ID。
    const uint16_t Response_Motor_ID = static_cast<uint16_t>(Data[0]) | (static_cast<uint16_t>(Data[1]) << 8);
    if (Response_Motor_ID != Config_Data.Motor_ID) return false;

    //D2和D3必须是固定的0xAA、0x01。
    if ((Data[2] != 0xAAU) || (Data[3] != 0x01U)) return false;

    const uint32_t Current_Time = HAL_GetTick();
    Parameter_Save_Data.Waiting_Response = false;
    Parameter_Save_Data.Valid = true;
    Parameter_Save_Data.Timed_Out = false;
    Parameter_Save_Data.Last_Update_Time = Current_Time;

    return true;
}

/**
 * @brief 更新参数操作的超时状态
 */
void Class_DM4310_Motor::Update_Parameter_States(void)
{
    if (!Init_Flag) return;

    const uint32_t Current_Time = HAL_GetTick();

    //参数读取超时
    if (Parameter_Data.Waiting_Response)
    {
        const uint32_t Elapsed_Time = Current_Time - Parameter_Data.Request_Time;

        if (Elapsed_Time > Parameter_Response_Timeout_ms)
        {
            Parameter_Data.Waiting_Response = false;
            Parameter_Data.Valid = false;
            Parameter_Data.Timed_Out = true;
        }
    }

    //参数写入超时
    if (Parameter_Write_Data.Waiting_Response)
    {
        const uint32_t Elapsed_Time = Current_Time - Parameter_Write_Data.Request_Time;

        if (Elapsed_Time > Parameter_Response_Timeout_ms)
        {
            Parameter_Write_Data.Waiting_Response =false;
            Parameter_Write_Data.Valid = false;
            Parameter_Write_Data.Timed_Out = true;
            Parameter_Write_Data.Value_Matched = false;
        }
    }

    //参数存储超时
    if (Parameter_Save_Data.Waiting_Response)
    {
        const uint32_t Elapsed_Time = Current_Time - Parameter_Save_Data.Request_Time;

        if (Elapsed_Time > Parameter_Response_Timeout_ms)
        {
            Parameter_Save_Data.Waiting_Response = false;
            Parameter_Save_Data.Valid = false;
            Parameter_Save_Data.Timed_Out = true;
        }
    }
}

/**
 * @brief 获取参数存储状态
 */
const DM_Parameter_Save_Data_t &Class_DM4310_Motor::Get_Parameter_Save_Data(void) const
{
    return Parameter_Save_Data;
}

//----------模式控制相关----------
/**
 * @brief 将浮点数线性映射为无符号定点整数
 */
uint16_t Class_DM4310_Motor::Float_To_Uint(float Value,float Minimum,float Maximum,uint8_t Bit_Count)
{
    //当前函数只需要处理12位和16位数据。
    if ((Bit_Count == 0) || (Bit_Count > 16) || (Maximum <= Minimum))
    {
        return 0U;
    }

    //将输入值限制在协议映射范围中 防止转换后超过目标位数
    if (Value < Minimum)
    {
        Value = Minimum;
    }
    else if (Value > Maximum)
    {
        Value = Maximum;
    }

    const uint32_t Raw_Maximum = (1UL << Bit_Count) - 1UL;

    const float Scaled_Value = (Value - Minimum) * static_cast<float>(Raw_Maximum) / (Maximum - Minimum);

    //四舍五入的转uint16_t
    return static_cast<uint16_t>(Scaled_Value + 0.5f);
}

/**
 * @brief 将float数据转换为小端字节序
 *
 * @param Value 需要转换的float数据
 * @param Data 保存结果的4字节数组
 */
void Class_DM4310_Motor::Float_To_Little_Endian_Bytes(float Value,uint8_t *Data)
{
    if (Data == nullptr) return;

    uint32_t Raw_Data = 0;

    //将float的32位二进制内容复制到uint32_t中
    //使用复制位模式，而不是进行数值类型转换
    memcpy(&Raw_Data,&Value,sizeof(float));

    //按照低字节在前的顺序放入CAN数据。 
    Data[0] = static_cast<uint8_t>(Raw_Data & 0xFF);
    Data[1] = static_cast<uint8_t>((Raw_Data >> 8U) & 0xFF);
    Data[2] = static_cast<uint8_t>((Raw_Data >> 16U) & 0xFF);
    Data[3] = static_cast<uint8_t>((Raw_Data >> 24U) & 0xFF);
}

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
float Class_DM4310_Motor::Uint_To_Float(uint32_t Raw_Value,float Minimum,float Maximum,uint8_t Bit_Count)
{
    if ((Bit_Count == 0U) || (Bit_Count > 31U) || (Maximum <= Minimum)) return 0.0f;

    //计算对应位数能表示的最大整数
    const uint32_t Raw_Maximum = (1UL << Bit_Count) - 1UL;

    if (Raw_Value > Raw_Maximum)
    {
        Raw_Value = Raw_Maximum;
    }

    //线性映射
    return Minimum +static_cast<float>(Raw_Value) *(Maximum - Minimum) / static_cast<float>(Raw_Maximum);
}

/**
 * @brief 将uint16_t转换为小端字节序
 */
void Class_DM4310_Motor::Uint16_To_Little_Endian_Bytes(uint16_t Value,uint8_t *Data)
{
    if (Data == nullptr)
    {
        return;
    }

    Data[0] = static_cast<uint8_t>(Value & 0x00FF);

    Data[1] = static_cast<uint8_t>((Value >> 8U) & 0x00FF);
}

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
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Control_MIT_Rad(float Target_Position_Rad,float Target_Velocity,float Position_Kp,float Velocity_Kd,float Feedforward_Torque)
{
    //模式校验
    if (Config_Data.Control_Mode != DM_Control_Mode_e::MIT)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //AI加的感觉有点意思？
    /*
    * 防止NaN进入数据映射。
    *
    * NaN是唯一一个与自身比较也不相等的浮点数，
    * 因此可以使用 Value != Value 判断。
    */
    if ((Target_Position_Rad != Target_Position_Rad) || (Target_Velocity != Target_Velocity) || (Position_Kp != Position_Kp) || (Velocity_Kd != Velocity_Kd) || (Feedforward_Torque != Feedforward_Torque))
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //将位置Position限制在[-PMAX, +PMAX]
    if (Target_Position_Rad > Config_Data.Position_Max)
    {
        Target_Position_Rad = Config_Data.Position_Max;
    }
    else if (Target_Position_Rad < -Config_Data.Position_Max)
    {
        Target_Position_Rad = -Config_Data.Position_Max;
    }


    //将速度Velocity限制在[-VMAX, +VMAX]

    if (Target_Velocity > Config_Data.Velocity_Max)
    {
        Target_Velocity = Config_Data.Velocity_Max;
    }
    else if (Target_Velocity < -Config_Data.Velocity_Max)
    {
        Target_Velocity = -Config_Data.Velocity_Max;
    }

    //Kp协议范围为[0, 500]
    if (Position_Kp > 500.0f)
    {
        Position_Kp = 500.0f;
    }
    else if (Position_Kp < 0.0f)
    {
        Position_Kp = 0.0f;
    }

    //Kd协议范围为[0, 5]
    if (Velocity_Kd > 5.0f)
    {
        Velocity_Kd = 5.0f;
    }
    else if (Velocity_Kd < 0.0f)
    {
        Velocity_Kd = 0.0f;
    }

    //将前馈扭矩限制在[-TMAX, +TMAX]
    if (Feedforward_Torque > Config_Data.Torque_Max)
    {
        Feedforward_Torque = Config_Data.Torque_Max;
    }
    else if (Feedforward_Torque < -Config_Data.Torque_Max)
    {
        Feedforward_Torque = -Config_Data.Torque_Max;
    }

    //保存传参
    MIT_Mode_Data.Target_Position_Rad = Target_Position_Rad;
    MIT_Mode_Data.Target_Position_Degree = Target_Position_Rad * 57.29577951308232f;
    MIT_Mode_Data.Target_Velocity = Target_Velocity;
    MIT_Mode_Data.Position_Kp = Position_Kp;
    MIT_Mode_Data.Velocity_Kd = Velocity_Kd;
    MIT_Mode_Data.Feedforward_Torque = Feedforward_Torque;


    //将浮点数映射为MIT模式要求的类型
    const uint16_t Target_Position_After = Float_To_Uint(Target_Position_Rad,-Config_Data.Position_Max,Config_Data.Position_Max,16);
    const uint16_t Target_Velocity_After = Float_To_Uint(Target_Velocity,-Config_Data.Velocity_Max,Config_Data.Velocity_Max,12);
    const uint16_t Position_Kp_After = Float_To_Uint(Position_Kp,0.0f,500.0f,12);
    const uint16_t Velocity_Kd_After = Float_To_Uint(Velocity_Kd,0.0f,5.0f,12);
    const uint16_t Feedforward_Torque_After = Float_To_Uint(Feedforward_Torque,-Config_Data.Torque_Max,Config_Data.Torque_Max,12);

    //创建发送数据数组
    uint8_t Data[8] = {0U};

    //D[0]：Position位置高8位 D[1]：Position位置低8位
    Data[0] = static_cast<uint8_t>((Target_Position_After >> 8) & 0xFF);
    Data[1] = static_cast<uint8_t>(Target_Position_After & 0xFF);

    //D[2]：Velocity速度高8位，即Velocity[11:4]
    Data[2] = static_cast<uint8_t>((Target_Velocity_After >> 4) & 0xFF);

    //D[3]高4位：Velocity[3:0] D[3]低4位：Kp[11:8]
    Data[3] = static_cast<uint8_t>(((Target_Velocity_After & 0x000F) << 4) | ((Position_Kp_After >> 8) & 0x000F));

    //D[4]：Kp低8位
    Data[4] = static_cast<uint8_t>(Position_Kp_After & 0x00FF);

    //D[5]：Kd高8位，即Kd[11:4]
    Data[5] = static_cast<uint8_t>((Velocity_Kd_After >> 4U) & 0x00FF);

    //D[6]高4位：Kd[3:0] D[6]低4位：Torque[11:8]
    Data[6] = static_cast<uint8_t>(((Velocity_Kd_After & 0x000F) << 4) | ((Feedforward_Torque_After >> 8) & 0x000F));

    //D[7]：Torque低8位
    Data[7] = static_cast<uint8_t>(Feedforward_Torque_After & 0x00FF);

    //发送
    return Transmit_Data(Get_Control_CAN_ID(),Data,8U);
}

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
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Control_MIT_Degree(float Target_Position_Degree,float Target_Velocity,float Position_Kp,float Velocity_Kd,float Feedforward_Torque)
{
    const float Target_Position_Rad = Target_Position_Degree * 0.017453292519943295f;

    return Control_MIT_Rad(Target_Position_Rad,Target_Velocity,Position_Kp,Velocity_Kd,Feedforward_Torque);
}

/**
 * @brief 使用速度模式控制电机
 *
 * @param Target_Velocity 目标速度，单位rad/s
 *
 * @return Enum_CAN_Transmit_Status_e CAN发送状态
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Control_Velocity(float Target_Velocity)
{
    //校验模式
    if (Config_Data.Control_Mode != DM_Control_Mode_e::Velocity)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //创建发送数据数组
    uint8_t Data[4] = {0U};

    //保存目标速度
    Velocity_Mode_Data.Target_Velocity = Target_Velocity;

    //转换小端排序float
    Float_To_Little_Endian_Bytes(Target_Velocity,Data);

    //发送
    return Transmit_Data(Get_Control_CAN_ID(),Data,4U);
}

/**
 * @brief 使用位置速度模式控制电机
 *
 * @param Target_Position_Rad 目标位置，单位rad
 * @param Velocity_Limit 运动过程中的最大绝对速度，单位rad/s
 *
 * @return Enum_CAN_Transmit_Status_e CAN发送状态
 */
Enum_CAN_Transmit_Status_e
Class_DM4310_Motor::Control_Position_Velocity_Rad(float Target_Position_Rad,float Velocity_Limit)
{
    //校验模式
    if (Config_Data.Control_Mode != DM_Control_Mode_e::Position_Velocity)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //防止是负数
    if (Velocity_Limit < 0.0f)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //保存传参
    Position_Velocity_Mode_Data.Target_Position_Rad = Target_Position_Rad;
    Position_Velocity_Mode_Data.Target_Position_Degree = Target_Position_Rad * 57.29577951308232f;
    Position_Velocity_Mode_Data.Velocity_Limit = Velocity_Limit;

    //创建发送数组
    uint8_t Data[8] = {0U};

    //D[0]~D[3]：目标位置float，小端序。
    Float_To_Little_Endian_Bytes(Target_Position_Rad,&Data[0]);

    
    //D[4]~D[7]：最大速度float，小端序。
    Float_To_Little_Endian_Bytes(Velocity_Limit,&Data[4]);

    //发送
    return Transmit_Data(Get_Control_CAN_ID(),Data,8U);
}


/**
 * @brief 使用位置速度模式控制电机
 *
 * @param Target_Position_Degree 目标位置，单位deg
 * @param Velocity_Limit 运动过程中的最大绝对速度，单位rad/s
 *
 * @return Enum_CAN_Transmit_Status_e CAN发送状态
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Control_Position_Velocity_Degree(float Target_Position_Degree,float Velocity_Limit)
{
    const float Target_Position_Rad = Target_Position_Degree * 0.017453292519943295f;

    return Control_Position_Velocity_Rad(Target_Position_Rad,Velocity_Limit);
}

/**
 * @brief 使用力位混控模式控制电机
 *
 * @param Target_Position_Rad 目标位置，单位rad
 * @param Velocity_Limit 速度限制，单位rad/s
 * @param Current_Limit_Per_Unit 扭矩电流限制标幺值
 *
 * @return Enum_CAN_Transmit_Status_e CAN发送状态
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Control_Force_Position_Hybrid_Rad(float Target_Position_Rad,float Velocity_Limit,float Current_Limit_Per_Unit)
{
    //模式校验
    if (Config_Data.Control_Mode != DM_Control_Mode_e::Force_Position_Hybrid)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //检查速度限制
    if (Velocity_Limit < 0.0f)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //电流限制是无符号标幺值，
    //协议有效范围为0~1。
    if (Current_Limit_Per_Unit < 0.0f)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    //手册规定超过上限时限制到协议最大值。
    if (Velocity_Limit > 100.0f)
    {
        Velocity_Limit = 100.0f;
    }

    if (Current_Limit_Per_Unit > 1.0f)
    {
        Current_Limit_Per_Unit = 1.0f;
    }

    //保存传参
    Force_Position_Hybrid_Mode_Data.Target_Position_Rad = Target_Position_Rad;
    Force_Position_Hybrid_Mode_Data.Target_Position_Degree = Target_Position_Rad * 57.29577951308232f;
    Force_Position_Hybrid_Mode_Data.Velocity_Limit = Velocity_Limit;
    Force_Position_Hybrid_Mode_Data.Current_Limit_Per_Unit = Current_Limit_Per_Unit;

    //速度限制放大一百倍（加0.5f是为了将正数四舍五入到最近的整数，而不是直接截断小数部分）
    const uint16_t Velocity_Limit_After = static_cast<uint16_t>(Velocity_Limit * 100.0f + 0.5f);

    //电流标幺值放大10000倍
    const uint16_t Current_Limit_Per_Unit_After =static_cast<uint16_t>(Current_Limit_Per_Unit * 10000.0f + 0.5f);

    //创建发送数据数组
    uint8_t Data[8] = {0U};

    //位置目标：D[0]~D[3]：目标位置float，小端。
    Float_To_Little_Endian_Bytes(Target_Position_Rad,&Data[0]);

    
    //速度限制：D[4]~D[5]：速度限制×100，uint16_t，小端。
    Uint16_To_Little_Endian_Bytes(Velocity_Limit_After,&Data[4]);

    
    //D[6]~D[7]：电流限制×10000，uint16_t，小端。
    Uint16_To_Little_Endian_Bytes(Current_Limit_Per_Unit_After,&Data[6]);

    //发送
    return Transmit_Data(Get_Control_CAN_ID(),Data,8);
}

/**
 * @brief 使用力位混控模式控制电机
 *
 * @param Target_Position_Degree 目标位置，单位deg
 * @param Velocity_Limit 速度限制，单位rad/s
 * @param Current_Limit_Per_Unit 扭矩电流限制标幺值
 *
 * @return Enum_CAN_Transmit_Status_e CAN发送状态
 */
Enum_CAN_Transmit_Status_e Class_DM4310_Motor::Control_Force_Position_Hybrid_Degree(float Target_Position_Degree,float Velocity_Limit,float Current_Limit_Per_Unit)
{
    const float Target_Position_Rad = Target_Position_Degree * 0.017453292519943295f;

    return Control_Force_Position_Hybrid_Rad(Target_Position_Rad,Velocity_Limit,Current_Limit_Per_Unit);
}

/**
 * @brief 更新电机在线状态
 */
void Class_DM4310_Motor::Update_Online_States(void)
{
    //初始化结束前保持False
    if (!Init_Flag)
    {
        Feedback_Data.Online = false;
        return;
    }

    //如果从未收到过合法反馈，目前已经是离线状态，
    //不需要继续计算超时时间。
    if (!Feedback_Data.Online)
    {
        return;
    }

    const uint32_t Current_Time = HAL_GetTick();

    const uint32_t Time_Since_Last_Feedback = Current_Time - Feedback_Data.Last_Feedback_Time;

    //超时判断
    if (Time_Since_Last_Feedback > Config_Data.Feedback_Timeout_ms)
    {
        Feedback_Data.Online = false;
    }
}

/**
 * @brief 获取电机当前在线状态
 */
bool Class_DM4310_Motor::Get_Online_States(void) const
{
    return Feedback_Data.Online;
}