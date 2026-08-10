/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    BoardCAN.cpp
  * @brief   舵步板间通讯库
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "BoardCAN.h"

//====================工具函数====================

//板间数据统一使用大端格式：高字节在前 低字节在后

/**
 * @brief 打包int16_t到CAN数据帧
 *
 * @param Data 指向CAN数据帧的指针
 * @param Index 数据在帧中的起始索引
 * @param Value 要打包的int16_t值
 */
static void BoardCAN_Pack_i16(uint8_t *Data,uint8_t Index,int16_t Value)
{
    Data[Index] = static_cast<uint8_t>(static_cast<uint16_t>(Value) >> 8);
    Data[Index + 1] = static_cast<uint8_t>(Value);
}

/**
 * @brief 解包CAN数据帧中的int16_t值
 *
 * @param Data 指向CAN数据帧的指针
 * @param Index 数据在帧中的起始索引
 * @return int16_t 解包后的int16_t值
 */
static int16_t BoardCAN_Unpack_i16(const uint8_t *Data,uint8_t Index)
{
    return static_cast<int16_t>((static_cast<uint16_t>(Data[Index]) << 8) | static_cast<uint16_t>(Data[Index + 1]));
}

//初始化

/**
 * @brief 初始化舵轮步兵板间CAN模块
 *
 * @param CAN_Interface CAN发送接口
 * @param Board_Roll 当前主控角色
 */
void Class_SteeringWheel_Infantry_BoardCAN::Init(Class_CAN_Interface *CAN_Interface,SteeringWheel_Infantry_BoardCAN_Role_e Board_Roll)
{
    //保存CAN接口和当前主控角色
    this->CAN_Interface = CAN_Interface;
    Board = Board_Roll;

    //清除上一次运行保存的遥控数据和收发序号
    Remote_Data = {};
    Tx_Sequence = 0;
}

//发送函数

/**
 * @brief 发送遥控器四通道摇杆数据
 *
 * @param Right_X 右摇杆X轴数据
 * @param Right_Y 右摇杆Y轴数据
 * @param Left_X 左摇杆X轴数据
 * @param Left_Y 左摇杆Y轴数据
 * @return Enum_CAN_Transmit_Status_e CAN发送状态
 */
Enum_CAN_Transmit_Status_e Class_SteeringWheel_Infantry_BoardCAN::Send_Remote_Joystick_Data(int16_t Right_X,int16_t Right_Y,int16_t Left_X,int16_t Left_Y)
{
    if (CAN_Interface == nullptr)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    uint8_t Data[8] = {0};

    BoardCAN_Pack_i16(Data,0,Right_X);
    BoardCAN_Pack_i16(Data,2,Right_Y);
    BoardCAN_Pack_i16(Data,4,Left_X);
    BoardCAN_Pack_i16(Data,6,Left_Y);

    return CAN_Interface->Transmit(BoardCAN_Remote_Joystick_Data_ID,Data,8);
}

/**
 * @brief 发送遥控器三档开关、拨轮和在线状态
 *
 * @param Dial_Wheel 拨轮数据
 * @param Left_ThreeKey 左侧三档开关状态
 * @param Right_ThreeKey 右侧三档开关状态
 * @param source_online DR16数据源是否在线
 * @return Enum_CAN_Transmit_Status_e CAN发送状态
 */
Enum_CAN_Transmit_Status_e Class_SteeringWheel_Infantry_BoardCAN::Send_Remote_ThreeKey_And_Dial_Wheel_Data(int16_t Dial_Wheel,BoardCAN_Remote_ThreeKey_e Left_ThreeKey,BoardCAN_Remote_ThreeKey_e Right_ThreeKey,bool source_online)
{
    if (CAN_Interface == nullptr)
    {
        return Enum_CAN_Transmit_Status_e::Error;
    }

    uint8_t Data[8] = {0};

    BoardCAN_Pack_i16(Data,0,Dial_Wheel);

    // Data[2]低2位为左开关，高2位为右开关
    Data[2] = static_cast<uint8_t>((static_cast<uint8_t>(Left_ThreeKey) & 0x03) | ((static_cast<uint8_t>(Right_ThreeKey) & 0x03) << 2));
    Data[3] = source_online ? 1 : 0;
    Data[4] = Tx_Sequence++;
    
    //Data[5]~Data[7] 预留

    return CAN_Interface->Transmit(BoardCAN_Remote_ThreeKey_And_Dial_Wheel_Data_ID,Data,8);
}

//接收

/**
 * @brief 解析板间CAN遥控器数据帧
 *
 * @param CAN_ID CAN标准帧ID
 * @param Data CAN数据区
 * @param Length CAN数据长度
 * @param Now_ms 当前系统时间，单位ms
 * @return bool true表示该帧已被本模块处理
 */
bool Class_SteeringWheel_Infantry_BoardCAN::Process_CAN_Message(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length,uint32_t Now_ms)
{
    if ((Data == nullptr) || (Length != 8))
    {
        return false;
    }

    switch (CAN_ID)
    {
        case BoardCAN_Remote_Joystick_Data_ID:
        {
            Remote_Data.Right_X = BoardCAN_Unpack_i16(Data,0);
            Remote_Data.Right_Y = BoardCAN_Unpack_i16(Data,2);
            Remote_Data.Left_X = BoardCAN_Unpack_i16(Data,4);
            Remote_Data.Left_Y = BoardCAN_Unpack_i16(Data,6);

            Remote_Data.Joystick_Data_Received = true;
            Remote_Data.Joystick_Data_Last_Rx_Time = Now_ms;
            return true;
        }

        case BoardCAN_Remote_ThreeKey_And_Dial_Wheel_Data_ID:
        {
            Remote_Data.Dial_Wheel = BoardCAN_Unpack_i16(Data,0);
            Remote_Data.Left_ThreeKey = static_cast<BoardCAN_Remote_ThreeKey_e>(Data[2] & 0x03U);
            Remote_Data.Right_ThreeKey = static_cast<BoardCAN_Remote_ThreeKey_e>((Data[2] >> 2) & 0x03U);
            Remote_Data.DR16_Online = (Data[3] & 0x01U) != 0U;
            Remote_Data.Rx_Sequence = Data[4];

            Remote_Data.ThreeKey_And_Dial_Wheel_Data_Received = true;
            Remote_Data.ThreeKey_And_Dial_Wheel_Data_Last_Rx_Time = Now_ms;
            return true;
        }

        default:
            return false;
    }
}

//检查Online

/**
 * @brief 判断板间共享的遥控器数据是否在线
 *
 * @param Now_ms 当前系统时间，单位ms
 * @param Timeout_ms 接收超时时间，单位ms
 * @return bool true表示数据源在线且两类遥控帧均未超时
 */
bool Class_SteeringWheel_Infantry_BoardCAN::Is_Remote_Online(uint32_t Now_ms,uint32_t Timeout_ms)
{
    if ((!Remote_Data.DR16_Online) || (!Remote_Data.Joystick_Data_Received) || (!Remote_Data.ThreeKey_And_Dial_Wheel_Data_Received))
    {
        return false;
    }

    if ((Now_ms - Remote_Data.Joystick_Data_Last_Rx_Time > Timeout_ms) || (Now_ms - Remote_Data.ThreeKey_And_Dial_Wheel_Data_Last_Rx_Time > Timeout_ms))
    {
        return false;
    }

    return true;
}
