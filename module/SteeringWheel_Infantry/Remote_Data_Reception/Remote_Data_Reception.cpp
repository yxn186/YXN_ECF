/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Remote_Data_Reception.cpp
  * @brief   舵轮步兵遥控数据接收层库
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "Remote_Data_Reception.h"

//define Gimbal是0 Chassis是1
#define SteeringWheel_Infantry_Board_Gimbal 0
#define SteeringWheel_Infantry_Board_Chassis 1

//define DR16安装位置
#ifndef SteeringWheel_Infantry_DR16_Location
#define SteeringWheel_Infantry_DR16_Location    SteeringWheel_Infantry_Board_Gimbal
#endif

//板间CAN的摇杆定点数缩放倍率、发送周期和接收超时时间
#define SteeringWheel_Infantry_Remote_CAN_Scale             1000.0f
#define SteeringWheel_Infantry_Remote_CAN_Transmit_Period_ms 10U
#define SteeringWheel_Infantry_Remote_CAN_Timeout_ms         100U

//====================工具函数====================

/**
 * @brief 将归一化浮点遥控数据转换为板间CAN定点数
 *
 * @param Value 归一化浮点数据
 * @return int16_t 板间CAN定点数
 */
static int16_t Remote_Data_Reception_Float_To_CAN(float Value)
{
    if (Value > 1.0f)
    {
        Value = 1.0f;
    }
    else if (Value < -1.0f)
    {
        Value = -1.0f;
    }

    return static_cast<int16_t>(Value * SteeringWheel_Infantry_Remote_CAN_Scale);
}

/**
 * @brief 将板间CAN定点数还原为归一化浮点遥控数据
 *
 * @param Value 板间CAN定点数
 * @return float 归一化浮点数据
 */
static float Remote_Data_Reception_CAN_To_Float(int16_t Value)
{
    return static_cast<float>(Value) / SteeringWheel_Infantry_Remote_CAN_Scale;
}

/**
 * @brief 将DR16开关状态转换为板间通用三档开关稳态
 *
 * @param ThreeKey DR16三档开关状态
 * @return BoardCAN_Remote_ThreeKey_e 三档开关稳态
 */
static BoardCAN_Remote_ThreeKey_e Remote_Data_Reception_Convert_ThreeKey(Enum_DR16_Switch_Status ThreeKey)
{
    switch (ThreeKey)
    {
        case DR16_Switch_Status_UP:
        case DR16_Switch_Status_TRIG_MIDDLE_UP:
            return BoardCAN_Remote_ThreeKey_e::Up;

        case DR16_Switch_Status_DOWN:
        case DR16_Switch_Status_TRIG_MIDDLE_DOWN:
            return BoardCAN_Remote_ThreeKey_e::Down;

        case DR16_Switch_Status_MIDDLE:
        case DR16_Switch_Status_TRIG_UP_MIDDLE:
        case DR16_Switch_Status_TRIG_DOWN_MIDDLE:
            return BoardCAN_Remote_ThreeKey_e::Middle;

        default:
            return BoardCAN_Remote_ThreeKey_e::Error;
    }
}

//初始化

/**
 * @brief 初始化遥控数据接收库
 *
 * @param DR16 DR16对象，当前板未安装DR16时允许传入nullptr
 * @param BoardCAN 板间CAN对象
 * @param Board_Roll 当前主控角色
 */
void Class_SteeringWheel_Infantry_Remote_Data_Reception::Init(Class_DR16 *DR16,Class_SteeringWheel_Infantry_BoardCAN *BoardCAN,SteeringWheel_Infantry_BoardCAN_Role_e Board_Roll)
{
    //保存DR16、板间CAN对象和当前主控角色
    this->DR16 = DR16;
    this->BoardCAN = BoardCAN;
    this->BoardCAN_This_Board_Roll = Board_Roll;

    BoardCAN_Last_Transmit_Time = 0;
    Reset_Remote_Data();
}

/**
 * @brief 判断DR16是否安装在当前主控
 *
 * @return bool true表示当前主控直接连接DR16
 */
bool Class_SteeringWheel_Infantry_Remote_Data_Reception::Is_DR16_Local_Board()
{
    return static_cast<uint8_t>(BoardCAN_This_Board_Roll) == SteeringWheel_Infantry_DR16_Location;
}

//更新

/**
 * @brief 更新遥控数据并按需发送板间CAN数据
 *
 * @param Now_ms 当前系统时间，单位ms
 */
void Class_SteeringWheel_Infantry_Remote_Data_Reception::Update(uint32_t Now_ms)
{
    if (Is_DR16_Local_Board())
    {
        //DR16安装在本板 使用本地DR16数据源
        Update_From_DR16(Now_ms);
    }
    else
    {
        //DR16安装在另一块主控 使用板间CAN数据源
        Update_From_BoardCAN(Now_ms);
    }
}

/**
 * @brief 从本板DR16更新遥控数据
 *
 * @param Now_ms 当前系统时间，单位ms
 */
void Class_SteeringWheel_Infantry_Remote_Data_Reception::Update_From_DR16(uint32_t Now_ms)
{
    if ((DR16 == nullptr) || (DR16->Get_Status() != DR16_Status_ENABLE))
    {
        Reset_Remote_Data();
        Send_DR16_Data_To_BoardCAN(Now_ms);
        return;
    }

    Remote_Online = true;
    Right_X = DR16->Get_Right_X();
    Right_Y = DR16->Get_Right_Y();
    Left_X = DR16->Get_Left_X();
    Left_Y = DR16->Get_Left_Y();
    Dial_Wheel = DR16->Get_Yaw();
    Left_ThreeKey = Remote_Data_Reception_Convert_ThreeKey(DR16->Get_Left_Switch());
    Right_ThreeKey = Remote_Data_Reception_Convert_ThreeKey(DR16->Get_Right_Switch());

    //将本板DR16数据发送到另一块主控
    Send_DR16_Data_To_BoardCAN(Now_ms);
}

/**
 * @brief 从板间CAN更新遥控数据
 *
 * @param Now_ms 当前系统时间，单位ms
 */
void Class_SteeringWheel_Infantry_Remote_Data_Reception::Update_From_BoardCAN(uint32_t Now_ms)
{
    if ((BoardCAN == nullptr) || (!BoardCAN->Is_Remote_Online(Now_ms,SteeringWheel_Infantry_Remote_CAN_Timeout_ms)))
    {
        Reset_Remote_Data();
        return;
    }

    Remote_Online = true;
    Right_X = Remote_Data_Reception_CAN_To_Float(BoardCAN->Get_Right_X());
    Right_Y = Remote_Data_Reception_CAN_To_Float(BoardCAN->Get_Right_Y());
    Left_X = Remote_Data_Reception_CAN_To_Float(BoardCAN->Get_Left_X());
    Left_Y = Remote_Data_Reception_CAN_To_Float(BoardCAN->Get_Left_Y());
    Dial_Wheel = Remote_Data_Reception_CAN_To_Float(BoardCAN->Get_Dial_Wheel());
    Left_ThreeKey = BoardCAN->Get_Left_ThreeKey();
    Right_ThreeKey = BoardCAN->Get_Right_ThreeKey();
}

/**
 * @brief 按固定周期将本板DR16数据发送到另一块主控
 *
 * @param Now_ms 当前系统时间，单位ms
 */
void Class_SteeringWheel_Infantry_Remote_Data_Reception::Send_DR16_Data_To_BoardCAN(uint32_t Now_ms)
{
    if ((BoardCAN == nullptr) || (Now_ms - BoardCAN_Last_Transmit_Time < SteeringWheel_Infantry_Remote_CAN_Transmit_Period_ms))
    {
        return;
    }

    BoardCAN_Last_Transmit_Time = Now_ms;

    BoardCAN->Send_Remote_Joystick_Data(Remote_Data_Reception_Float_To_CAN(Right_X),
                                        Remote_Data_Reception_Float_To_CAN(Right_Y),
                                        Remote_Data_Reception_Float_To_CAN(Left_X),
                                        Remote_Data_Reception_Float_To_CAN(Left_Y));

    BoardCAN->Send_Remote_ThreeKey_And_Dial_Wheel_Data(Remote_Data_Reception_Float_To_CAN(Dial_Wheel),Left_ThreeKey,Right_ThreeKey,Remote_Online);
}

/**
 * @brief 清除遥控数据并进入离线状态
 */
void Class_SteeringWheel_Infantry_Remote_Data_Reception::Reset_Remote_Data()
{
    Remote_Online = false;

    Right_X = 0.0f;
    Right_Y = 0.0f;
    Left_X = 0.0f;
    Left_Y = 0.0f;
    Dial_Wheel = 0.0f;

    Left_ThreeKey = BoardCAN_Remote_ThreeKey_e::Error;
    Right_ThreeKey = BoardCAN_Remote_ThreeKey_e::Error;
}
