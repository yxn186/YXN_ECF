/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bmi088.cpp
  * @brief   BMI088传感器驱动
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bmi088.h"
#include <string.h>
/*YOUR CODE*/

Class_BMI088 *Class_BMI088::BMI088_Instance = nullptr;

/**
 * @brief 初始化BMI088驱动并准备非阻塞初始化状态机
 *
 * @param New_Config SPI、GPIO、量程和反馈超时等初始化参数
 * @return bool true表示参数有效并已经开始初始化
 */
bool Class_BMI088::Init(const BMI088_Config_t &New_Config)
{
    //SPI和两个片选缺少任意一个都无法区分ACC与GYRO器件
    if ((New_Config.SPI_Handler == nullptr) ||
        (New_Config.ACC_CS_GPIOx == nullptr) ||
        (New_Config.GYRO_CS_GPIOx == nullptr))
    {
        return false;
    }
    if ((New_Config.ACC_Range_G <= 0.0f) ||
        (New_Config.GYRO_Range_DPS <= 0.0f) ||
        (New_Config.Feedback_Timeout_ms == 0))
    {
        return false;
    }

    Config = New_Config;
    States = BMI088_States_e::Initializing;
    Init_State = Init_State_e::Start_ACC_Reset;
    Transfer_Type = Transfer_Type_e::None;

    Init_Finished = false;
    Online = false;
    New_Sample = false;
    ACC_Sample_Valid = false;
    Transfer_Finished = false;
    Transfer_Error = false;
    ACC_Data_Ready = false;
    GYRO_Data_Ready = false;
    Raw_Sample_Ready = false;

    ACC_Chip_ID = 0;
    GYRO_Chip_ID = 0;
    Config_Register_Index = 0;
    State_Start_Time = HAL_GetTick();
    Init_Finished_Time = 0;
    Last_Sample_Time = 0;

    //初始化开始前主动释放两个片选，避免上电期间误选中传感器
    GPIO_PinState ACC_Inactive_Level = GPIO_PIN_SET;
    GPIO_PinState GYRO_Inactive_Level = GPIO_PIN_SET;
    if (Config.ACC_CS_Active_Level == GPIO_PIN_SET)
    {
        ACC_Inactive_Level = GPIO_PIN_RESET;
    }
    if (Config.GYRO_CS_Active_Level == GPIO_PIN_SET)
    {
        GYRO_Inactive_Level = GPIO_PIN_RESET;
    }

    HAL_GPIO_WritePin(Config.ACC_CS_GPIOx,Config.ACC_CS_Pin,ACC_Inactive_Level);
    HAL_GPIO_WritePin(Config.GYRO_CS_GPIOx,Config.GYRO_CS_Pin,GYRO_Inactive_Level);

    //SPI2只挂载这一颗BMI088，通过静态入口把HAL回调转回当前对象
    BMI088_Instance = this;
    SPI_Init(Config.SPI_Handler,SPI_Complete_Callback);
    SPI_Set_Error_Callback(Config.SPI_Handler,SPI_Error_Callback);

    return true;
}

/**
 * @brief 推进BMI088非阻塞初始化、采样和在线检测
 *
 * @param Now_ms 当前系统时间，单位ms
 */
void Class_BMI088::Update(uint32_t Now_ms)
{
    if (States == BMI088_States_e::Uninitialized)
    {
        return;
    }

    if (Transfer_Error)
    {
        Transfer_Error = false;
        Set_Error_State();
    }

    if (States == BMI088_States_e::Initializing)
    {
        Init_Process(Now_ms);
        return;
    }

    if (States == BMI088_States_e::Error)
    {
        return;
    }

    Data_Process(Now_ms);
}

/**
 * @brief 记录BMI088的ACC或GYRO数据就绪中断
 *
 * @param GPIO_Pin 产生外部中断的GPIO引脚
 */
void Class_BMI088::Process_GPIO_EXTI(uint16_t GPIO_Pin)
{
    //中断中只记录事件，SPI DMA统一由Main Task串行启动
    if (GPIO_Pin == Config.ACC_INT_Pin)
    {
        ACC_Data_Ready = true;
    }
    else if (GPIO_Pin == Config.GYRO_INT_Pin)
    {
        GYRO_Data_Ready = true;
    }
}

/**
 * @brief 获取是否产生了一组新的ACC与GYRO完整数据
 *
 * @return bool true表示上层还没有处理本次完整数据
 */
bool Class_BMI088::Get_New_Sample_Flag()
{
    return New_Sample;
}

/**
 * @brief 清除完整数据更新标志
 */
void Class_BMI088::Clear_New_Sample_Flag()
{
    New_Sample = false;
}

/**
 * @brief 按当前步骤推进BMI088软复位、ID检查和寄存器配置
 *
 * @param Now_ms 当前系统时间，单位ms
 */
void Class_BMI088::Init_Process(uint32_t Now_ms)
{
    //每次Update只推进当前一步，等待时间用时间戳完成，不阻塞其他应用
    switch (Init_State)
    {
        case Init_State_e::Start_ACC_Reset:
        {
            //先软复位ACC，复位完成后必须等待芯片重新启动
            if (Start_ACC_Write(BMI088_ACC_SOFTRESET,BMI088_ACC_SOFTRESET_VALUE))
            {
                Init_State = Init_State_e::Wait_ACC_Reset;
            }
            break;
        }

        case Init_State_e::Wait_ACC_Reset:
        {
            if (Take_Transfer_Finished())
            {
                State_Start_Time = Now_ms;
                Init_State = Init_State_e::Wait_ACC_Reset_Delay;
            }
            break;
        }

        case Init_State_e::Wait_ACC_Reset_Delay:
        {
            if (Now_ms - State_Start_Time >= 10)
            {
                Init_State = Init_State_e::Start_GYRO_Reset;
            }
            break;
        }

        case Init_State_e::Start_GYRO_Reset:
        {
            //ACC等待结束后再复位GYRO，避免两个片选操作同时占用SPI
            if (Start_GYRO_Write(BMI088_GYRO_SOFTRESET,BMI088_GYRO_SOFTRESET_VALUE))
            {
                Init_State = Init_State_e::Wait_GYRO_Reset;
            }
            break;
        }

        case Init_State_e::Wait_GYRO_Reset:
        {
            if (Take_Transfer_Finished())
            {
                State_Start_Time = Now_ms;
                Init_State = Init_State_e::Wait_GYRO_Reset_Delay;
            }
            break;
        }

        case Init_State_e::Wait_GYRO_Reset_Delay:
        {
            if (Now_ms - State_Start_Time >= 40)
            {
                Init_State = Init_State_e::Start_ACC_Dummy_Read;
            }
            break;
        }

        case Init_State_e::Start_ACC_Dummy_Read:
        {
            //BMI088 ACC使用SPI读取时需要先做一次Dummy Read唤醒接口
            if (Start_ACC_Read(BMI088_ACC_CHIP_ID,1,Transfer_Type_e::Read_ACC_ID))
            {
                Init_State = Init_State_e::Wait_ACC_Dummy_Read;
            }
            break;
        }

        case Init_State_e::Wait_ACC_Dummy_Read:
        {
            if (Take_Transfer_Finished())
            {
                Init_State = Init_State_e::Start_ACC_ID_Read;
            }
            break;
        }

        case Init_State_e::Start_ACC_ID_Read:
        {
            if (Start_ACC_Read(BMI088_ACC_CHIP_ID,1,Transfer_Type_e::Read_ACC_ID))
            {
                Init_State = Init_State_e::Wait_ACC_ID_Read;
            }
            break;
        }

        case Init_State_e::Wait_ACC_ID_Read:
        {
            if (Take_Transfer_Finished())
            {
                Init_State = Init_State_e::Start_GYRO_ID_Read;
            }
            break;
        }

        case Init_State_e::Start_GYRO_ID_Read:
        {
            if (Start_GYRO_Read(BMI088_GYRO_CHIP_ID,1,Transfer_Type_e::Read_GYRO_ID))
            {
                Init_State = Init_State_e::Wait_GYRO_ID_Read;
            }
            break;
        }

        case Init_State_e::Wait_GYRO_ID_Read:
        {
            if (Take_Transfer_Finished())
            {
                Init_State = Init_State_e::Check_Chip_ID;
            }
            break;
        }

        case Init_State_e::Check_Chip_ID:
        {
            //ACC应返回0x1E，GYRO应返回0x0F，错误ID不继续写配置
            if ((ACC_Chip_ID != BMI088_ACC_CHIP_ID_VALUE) ||
                (GYRO_Chip_ID != BMI088_GYRO_CHIP_ID_VALUE))
            {
                Set_Error_State();
                break;
            }

            Config_Register_Index = 0;
            Init_State = Init_State_e::Start_Config_Register;
            break;
        }

        case Init_State_e::Start_Config_Register:
        {
            //十二项配置按ACC电源、量程、GYRO参数和数据就绪中断的顺序写入
            if (Config_Register_Index >= 12)
            {
                Init_Finished = true;
                Online = false;
                States = BMI088_States_e::Ready;
                Init_Finished_Time = Now_ms;
                ACC_Data_Ready = false;
                GYRO_Data_Ready = false;
                Init_State = Init_State_e::Finished;
                break;
            }

            if (Start_Config_Register())
            {
                Init_State = Init_State_e::Wait_Config_Register;
            }
            break;
        }

        case Init_State_e::Wait_Config_Register:
        {
            if (Take_Transfer_Finished())
            {
                State_Start_Time = Now_ms;
                if (Config_Register_Delay_ms > 0)
                {
                    Init_State = Init_State_e::Wait_Config_Delay;
                }
                else
                {
                    Config_Register_Index++;
                    Init_State = Init_State_e::Start_Config_Register;
                }
            }
            break;
        }

        case Init_State_e::Wait_Config_Delay:
        {
            if (Now_ms - State_Start_Time >= Config_Register_Delay_ms)
            {
                Config_Register_Index++;
                Init_State = Init_State_e::Start_Config_Register;
            }
            break;
        }

        case Init_State_e::Finished:
        case Init_State_e::Error:
        default:
            break;
    }
}

/**
 * @brief 处理BMI088数据就绪请求、单位换算和在线检测
 *
 * @param Now_ms 当前系统时间，单位ms
 */
void Class_BMI088::Data_Process(uint32_t Now_ms)
{
    //只有ACC和GYRO都读取完成后才向上层发布一组新数据
    if (Raw_Sample_Ready)
    {
        Raw_Sample_Ready = false;
        Update_Scaled_Data();
        New_Sample = true;
        Online = true;
        States = BMI088_States_e::Ready;
        Last_Sample_Time = Now_ms;
    }

    //共享SPI总线固定按照ACC再GYRO的顺序读取，避免两次DMA重叠
    if (Transfer_Type == Transfer_Type_e::None)
    {
        if (!ACC_Sample_Valid && ACC_Data_Ready)
        {
            ACC_Data_Ready = false;
            Start_ACC_Read(BMI088_ACCEL_XOUT_L,6,Transfer_Type_e::Read_ACC_Data);
        }
        else if (ACC_Sample_Valid && GYRO_Data_Ready)
        {
            GYRO_Data_Ready = false;
            Start_GYRO_Read(BMI088_GYRO_X_L,6,Transfer_Type_e::Read_GYRO_Data);
        }
    }

    uint32_t Online_Reference_Time = Last_Sample_Time;
    if (Online_Reference_Time == 0)
    {
        Online_Reference_Time = Init_Finished_Time;
    }

    //初始化完成后尚无数据时，从初始化完成时间开始计算第一次超时
    if (Now_ms - Online_Reference_Time > Config.Feedback_Timeout_ms)
    {
        Online = false;
        States = BMI088_States_e::Offline;
    }
}

/**
 * @brief 启动一次加速度计寄存器DMA读取
 *
 * @param Register 起始寄存器地址
 * @param Length 需要读取的真实数据长度
 * @param New_Transfer_Type 本次传输用途
 * @return bool true表示DMA成功启动
 */
bool Class_BMI088::Start_ACC_Read(uint8_t Register,uint8_t Length,Transfer_Type_e New_Transfer_Type)
{
    if ((Transfer_Type != Transfer_Type_e::None) || (Length == 0) || (Length > (uint8_t)(sizeof(Tx_Buffer) - 2)))
    {
        return false;
    }

    //ACC在地址字节后会返回一个额外Dummy Byte，所以多提供一个时钟字节
    Tx_Buffer[0] = Register | BMI088_READ;
    memset(&Tx_Buffer[1],BMI088_DUMMY,Length + 1);
    memset(Rx_Buffer,0,Length + 2);

    Transfer_Type = New_Transfer_Type;
    Transfer_Finished = false;
    uint8_t SPI_States = SPI_Transmit_Receive_Data(Config.SPI_Handler,
                                                Config.ACC_CS_GPIOx,
                                                Config.ACC_CS_Pin,
                                                Config.ACC_CS_Active_Level,
                                                Tx_Buffer,
                                                Rx_Buffer,
                                                1,
                                                Length + 1);
    if (SPI_States != HAL_OK)
    {
        Transfer_Type = Transfer_Type_e::None;
        return false;
    }

    return true;
}

/**
 * @brief 启动一次陀螺仪寄存器DMA读取
 *
 * @param Register 起始寄存器地址
 * @param Length 需要读取的真实数据长度
 * @param New_Transfer_Type 本次传输用途
 * @return bool true表示DMA成功启动
 */
bool Class_BMI088::Start_GYRO_Read(uint8_t Register,uint8_t Length,Transfer_Type_e New_Transfer_Type)
{
    if ((Transfer_Type != Transfer_Type_e::None) || (Length == 0) || (Length > (uint8_t)(sizeof(Tx_Buffer) - 1)))
    {
        return false;
    }

    //GYRO没有ACC额外的Dummy Byte，地址后直接读取真实寄存器数据
    Tx_Buffer[0] = Register | BMI088_READ;
    memset(&Tx_Buffer[1],BMI088_DUMMY,Length);
    memset(Rx_Buffer,0,Length + 1);

    Transfer_Type = New_Transfer_Type;
    Transfer_Finished = false;
    uint8_t SPI_States = SPI_Transmit_Receive_Data(Config.SPI_Handler,
                                                Config.GYRO_CS_GPIOx,
                                                Config.GYRO_CS_Pin,
                                                Config.GYRO_CS_Active_Level,
                                                Tx_Buffer,
                                                Rx_Buffer,
                                                1,
                                                Length);
    if (SPI_States != HAL_OK)
    {
        Transfer_Type = Transfer_Type_e::None;
        return false;
    }

    return true;
}

/**
 * @brief 启动一次加速度计单寄存器DMA写入
 *
 * @param Register 目标寄存器地址
 * @param Data 需要写入的一个字节
 * @return bool true表示DMA成功启动
 */
bool Class_BMI088::Start_ACC_Write(uint8_t Register,uint8_t Data)
{
    if (Transfer_Type != Transfer_Type_e::None)
    {
        return false;
    }

    Tx_Buffer[0] = Register & BMI088_WRITE;
    Tx_Buffer[1] = Data;
    Transfer_Type = Transfer_Type_e::Write_Register;
    Transfer_Finished = false;

    uint8_t SPI_States = SPI_Transmit_Data(Config.SPI_Handler,
                                        Config.ACC_CS_GPIOx,
                                        Config.ACC_CS_Pin,
                                        Config.ACC_CS_Active_Level,
                                        Tx_Buffer,
                                        2);
    if (SPI_States != HAL_OK)
    {
        Transfer_Type = Transfer_Type_e::None;
        return false;
    }

    return true;
}

/**
 * @brief 启动一次陀螺仪单寄存器DMA写入
 *
 * @param Register 目标寄存器地址
 * @param Data 需要写入的一个字节
 * @return bool true表示DMA成功启动
 */
bool Class_BMI088::Start_GYRO_Write(uint8_t Register,uint8_t Data)
{
    if (Transfer_Type != Transfer_Type_e::None)
    {
        return false;
    }

    Tx_Buffer[0] = Register & BMI088_WRITE;
    Tx_Buffer[1] = Data;
    Transfer_Type = Transfer_Type_e::Write_Register;
    Transfer_Finished = false;

    uint8_t SPI_States = SPI_Transmit_Data(Config.SPI_Handler,
                                        Config.GYRO_CS_GPIOx,
                                        Config.GYRO_CS_Pin,
                                        Config.GYRO_CS_Active_Level,
                                        Tx_Buffer,
                                        2);
    if (SPI_States != HAL_OK)
    {
        Transfer_Type = Transfer_Type_e::None;
        return false;
    }

    return true;
}

/**
 * @brief 按配置索引启动下一项BMI088寄存器写入
 *
 * @return bool true表示对应寄存器写入已经启动
 */
bool Class_BMI088::Start_Config_Register()
{
    Config_Register_Delay_ms = 0;

    switch (Config_Register_Index)
    {
        //1.把ACC从Suspend模式拉到Normal模式并使能采样
        case 0:
            Config_Register_Delay_ms = 2;
            return Start_ACC_Write(BMI088_ACC_PWR_CONF,BMI088_ACC_PWR_ACTIVE_MODE);
        case 1:
            Config_Register_Delay_ms = 5;
            return Start_ACC_Write(BMI088_ACC_PWR_CTRL,BMI088_ACC_ENABLE_ACC_ON);

        //2.设置ACC采样率、滤波模式和量程
        case 2:
            return Start_ACC_Write(BMI088_ACC_CONF,Config.ACC_Config_Value);
        case 3:
            return Start_ACC_Write(BMI088_ACC_RANGE,Config.ACC_Range_Value);

        //3.设置GYRO正常模式、量程和带宽
        case 4:
            return Start_GYRO_Write(BMI088_GYRO_LPM1,BMI088_GYRO_NORMAL_MODE);
        case 5:
            return Start_GYRO_Write(BMI088_GYRO_RANGE,Config.GYRO_Range_Value);
        case 6:
            return Start_GYRO_Write(BMI088_GYRO_BANDWIDTH,Config.GYRO_Bandwidth_Value);

        //4.ACC数据就绪使用INT1推挽高电平输出
        case 7:
            return Start_ACC_Write(BMI088_INT1_IO_CTRL,BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_HIGH);
        case 8:
            return Start_ACC_Write(BMI088_INT_MAP_DATA,BMI088_ACC_INT1_DRDY_INTERRUPT);

        //5.GYRO数据就绪使用INT3推挽高电平输出
        case 9:
            return Start_GYRO_Write(BMI088_GYRO_CTRL,BMI088_DRDY_ON);
        case 10:
            return Start_GYRO_Write(BMI088_GYRO_INT3_INT4_IO_CONF,BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_HIGH);
        case 11:
            return Start_GYRO_Write(BMI088_GYRO_INT3_INT4_IO_MAP,BMI088_GYRO_DRDY_IO_INT3);
        default:
            return false;
    }
}

/**
 * @brief 读取并清除一次SPI传输完成标志
 *
 * @return bool true表示刚刚完成了一次SPI传输
 */
bool Class_BMI088::Take_Transfer_Finished()
{
    if (!Transfer_Finished)
    {
        return false;
    }

    Transfer_Finished = false;
    return true;
}

/**
 * @brief 根据配置量程把六轴原始值换算为g和degree/s
 */
void Class_BMI088::Update_Scaled_Data()
{
    float ACC_Scale = Config.ACC_Range_G / 32768.0f;
    float GYRO_Scale = Config.GYRO_Range_DPS / 32768.0f;

    ACC_X_G = ACC_Raw_X * ACC_Scale;
    ACC_Y_G = ACC_Raw_Y * ACC_Scale;
    ACC_Z_G = ACC_Raw_Z * ACC_Scale;

    GYRO_X_DPS = GYRO_Raw_X * GYRO_Scale;
    GYRO_Y_DPS = GYRO_Raw_Y * GYRO_Scale;
    GYRO_Z_DPS = GYRO_Raw_Z * GYRO_Scale;
}

/**
 * @brief 停止BMI088驱动流程并进入错误状态
 */
void Class_BMI088::Set_Error_State()
{
    States = BMI088_States_e::Error;
    Init_State = Init_State_e::Error;
    Transfer_Type = Transfer_Type_e::None;
    Init_Finished = false;
    Online = false;
}

/**
 * @brief 将SPI BSP完成回调转发到当前BMI088对象
 *
 * @param Tx_Buffer 本次通信使用的发送缓冲区
 * @param Rx_Buffer 本次通信使用的接收缓冲区
 * @param Tx_Length 协议发送段长度
 * @param Rx_Length 协议接收段长度
 */
void Class_BMI088::SPI_Complete_Callback(uint8_t *Tx_Buffer,uint8_t *Rx_Buffer,uint16_t Tx_Length,uint16_t Rx_Length)
{
    if (BMI088_Instance != nullptr)
    {
        BMI088_Instance->Process_SPI_Complete(Tx_Buffer,Rx_Buffer,Tx_Length,Rx_Length);
    }
}

/**
 * @brief 将SPI BSP错误回调转发到当前BMI088对象
 */
void Class_BMI088::SPI_Error_Callback()
{
    if (BMI088_Instance != nullptr)
    {
        BMI088_Instance->Process_SPI_Error();
    }
}

/**
 * @brief 解析一次SPI DMA完成后的芯片ID或六轴寄存器数据
 *
 * @param Tx_Buffer 本次通信使用的发送缓冲区
 * @param Rx_Buffer 本次通信使用的接收缓冲区
 * @param Tx_Length 协议发送段长度
 * @param Rx_Length 协议接收段长度
 */
void Class_BMI088::Process_SPI_Complete(uint8_t *Tx_Buffer,uint8_t *Rx_Buffer,uint16_t Tx_Length,uint16_t Rx_Length)
{
    //先释放当前传输类型，回调结束后Main Task才能启动下一次DMA
    Transfer_Type_e Finished_Transfer = Transfer_Type;
    Transfer_Type = Transfer_Type_e::None;

    if (Finished_Transfer == Transfer_Type_e::Write_Register)
    {
        Transfer_Finished = true;
        return;
    }

    if ((Tx_Buffer == nullptr) || (Rx_Buffer == nullptr) || (Tx_Length != 1))
    {
        Transfer_Error = true;
        return;
    }

    if (Finished_Transfer == Transfer_Type_e::Read_ACC_ID)
    {
        if (Rx_Length != 2)
        {
            Transfer_Error = true;
            return;
        }
        ACC_Chip_ID = Rx_Buffer[2];
        Transfer_Finished = true;
    }
    else if (Finished_Transfer == Transfer_Type_e::Read_GYRO_ID)
    {
        if (Rx_Length != 1)
        {
            Transfer_Error = true;
            return;
        }
        GYRO_Chip_ID = Rx_Buffer[1];
        Transfer_Finished = true;
    }
    else if (Finished_Transfer == Transfer_Type_e::Read_ACC_Data)
    {
        if (Rx_Length != 7)
        {
            Transfer_Error = true;
            return;
        }

        //BMI088六轴寄存器均为低字节在前、高字节在后
        ACC_Raw_X = (int16_t)((Rx_Buffer[3] << 8) | Rx_Buffer[2]);
        ACC_Raw_Y = (int16_t)((Rx_Buffer[5] << 8) | Rx_Buffer[4]);
        ACC_Raw_Z = (int16_t)((Rx_Buffer[7] << 8) | Rx_Buffer[6]);
        ACC_Sample_Valid = true;
    }
    else if (Finished_Transfer == Transfer_Type_e::Read_GYRO_Data)
    {
        if (Rx_Length != 6)
        {
            Transfer_Error = true;
            return;
        }

        GYRO_Raw_X = (int16_t)((Rx_Buffer[2] << 8) | Rx_Buffer[1]);
        GYRO_Raw_Y = (int16_t)((Rx_Buffer[4] << 8) | Rx_Buffer[3]);
        GYRO_Raw_Z = (int16_t)((Rx_Buffer[6] << 8) | Rx_Buffer[5]);

        //GYRO完成且前面已有ACC数据时，才发布一组时间相邻的完整样本
        if (ACC_Sample_Valid)
        {
            ACC_Sample_Valid = false;
            Raw_Sample_Ready = true;
        }
    }
}

/**
 * @brief 记录SPI DMA错误并等待Update统一处理
 */
void Class_BMI088::Process_SPI_Error()
{
    Transfer_Type = Transfer_Type_e::None;
    Transfer_Error = true;
}
