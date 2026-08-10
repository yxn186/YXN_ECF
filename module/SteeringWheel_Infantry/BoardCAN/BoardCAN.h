/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    BoardCAN.h
  * @brief   This file contains all the function prototypes for
  *          the BoardCAN.c/.cpp file
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARDCAN_H__
#define __BOARDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "CAN_Interface.h"

/*YOUR CODE*/

#define BoardCAN_Remote_Joystick_Data_ID                0x300               //遥控器摇杆数据ID
#define BoardCAN_Remote_ThreeKey_And_Dial_Wheel_Data_ID 0x301               //遥控器三档开关和拨轮数据ID

/**
 * @brief 舵轮步兵板间CAN角色枚举
 * 
 */
enum class SteeringWheel_Infantry_BoardCAN_Role_e : uint8_t
{
    Gimbal = 0,
    Chassis
};

/**
 * @brief 舵轮步兵板间CAN遥控器三档开关枚举
 * 
 */
enum class BoardCAN_Remote_ThreeKey_e : uint8_t
{
    Error = 0,
    Middle,
    Down ,
    Up
};

/**
 * @brief 板间CAN共享的遥控器数据
 *
 */
typedef struct
{
    int16_t Right_X = 0;
    int16_t Right_Y = 0;
    int16_t Left_X = 0;
    int16_t Left_Y = 0;

    int16_t Dial_Wheel = 0;

    BoardCAN_Remote_ThreeKey_e Left_ThreeKey = BoardCAN_Remote_ThreeKey_e::Error;
    BoardCAN_Remote_ThreeKey_e Right_ThreeKey = BoardCAN_Remote_ThreeKey_e::Error;

    bool DR16_Online = false;
    bool Joystick_Data_Received = false;
    bool ThreeKey_And_Dial_Wheel_Data_Received = false;

    uint8_t Rx_Sequence = 0;
    uint32_t Joystick_Data_Last_Rx_Time = 0;
    uint32_t ThreeKey_And_Dial_Wheel_Data_Last_Rx_Time = 0;
}BoardCAN_Remote_Data_t;

class Class_SteeringWheel_Infantry_BoardCAN
{
public:
    /**
     * @brief 初始化舵轮步兵板间CAN模块
     *
     * @param CAN_Interface CAN发送接口
     * @param Board_Roll 当前主控角色
     */
    void Init(Class_CAN_Interface *CAN_Interface,SteeringWheel_Infantry_BoardCAN_Role_e Board_Roll);

    /**
     * @brief 发送遥控器四通道摇杆数据
     *
     * @param Right_X 右摇杆X轴数据
     * @param Right_Y 右摇杆Y轴数据
     * @param Left_X 左摇杆X轴数据
     * @param Left_Y 左摇杆Y轴数据
     * @return Enum_CAN_Transmit_Status_e CAN发送状态
     */
    Enum_CAN_Transmit_Status_e Send_Remote_Joystick_Data(int16_t Right_X,int16_t Right_Y,int16_t Left_X,int16_t Left_Y);

    /**
     * @brief 发送遥控器三档开关、拨轮和在线状态
     *
     * @param Dial_Wheel 拨轮数据
     * @param Left_ThreeKey 左侧三档开关状态
     * @param Right_ThreeKey 右侧三档开关状态
     * @param source_online DR16数据源是否在线
     * @return Enum_CAN_Transmit_Status_e CAN发送状态
     */
    Enum_CAN_Transmit_Status_e Send_Remote_ThreeKey_And_Dial_Wheel_Data(int16_t Dial_Wheel,BoardCAN_Remote_ThreeKey_e Left_ThreeKey,BoardCAN_Remote_ThreeKey_e Right_ThreeKey,bool source_online);

    /**
     * @brief 解析板间CAN遥控器数据帧
     *
     * @param CAN_ID CAN标准帧ID
     * @param Data CAN数据区
     * @param Length CAN数据长度
     * @param Now_ms 当前系统时间，单位ms
     * @return bool true表示该帧已被本模块处理
     */
    bool Process_CAN_Message(uint16_t CAN_ID,const uint8_t *Data,uint8_t Length,uint32_t Now_ms);

    /**
     * @brief 判断板间共享的遥控器数据是否在线
     *
     * @param Now_ms 当前系统时间，单位ms
     * @param Timeout_ms 接收超时时间，单位ms
     * @return bool true表示数据源在线且两类遥控帧均未超时
     */
    bool Is_Remote_Online(uint32_t Now_ms,uint32_t Timeout_ms);

    inline int16_t Get_Right_X()
    {
        return (Remote_Data.Right_X);
    }

    inline int16_t Get_Right_Y()
    {
        return (Remote_Data.Right_Y);
    }

    inline int16_t Get_Left_X()
    {
        return (Remote_Data.Left_X);
    }

    inline int16_t Get_Left_Y()
    {
        return (Remote_Data.Left_Y);
    }

    inline int16_t Get_Dial_Wheel()
    {
        return (Remote_Data.Dial_Wheel);
    }

    inline BoardCAN_Remote_ThreeKey_e Get_Left_ThreeKey()
    {
        return (Remote_Data.Left_ThreeKey);
    }

    inline BoardCAN_Remote_ThreeKey_e Get_Right_ThreeKey()
    {
        return (Remote_Data.Right_ThreeKey);
    }

private:
    Class_CAN_Interface *CAN_Interface = nullptr;

    SteeringWheel_Infantry_BoardCAN_Role_e Board = SteeringWheel_Infantry_BoardCAN_Role_e::Gimbal;

    BoardCAN_Remote_Data_t Remote_Data{};

    uint8_t Tx_Sequence = 0;
};

#ifdef __cplusplus
}
#endif

#endif /* __BOARDCAN_H__ */
