/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Remote_Data_Reception.h
  * @brief   This file contains all the function prototypes for
  *          the Remote_Data_Reception.c/.cpp file
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REMOTE_DATA_RECEPTION_H__
#define __REMOTE_DATA_RECEPTION_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "DR16.h"
#include "BoardCAN.h"

/*YOUR CODE*/

/**
 * @brief 舵轮步兵遥控数据接收类
 *
 * 根据Init传入的当前主控角色和DR16安装位置，自动选择本地DR16或板间CAN作为数据源。
 */
class Class_SteeringWheel_Infantry_Remote_Data_Reception
{
public:
    /**
     * @brief 初始化遥控数据接收库
     *
     * @param DR16 DR16对象，当前板未安装DR16时允许传入nullptr
     * @param BoardCAN 板间CAN对象
     * @param Board_Roll 当前主控角色
     */
    void Init(Class_DR16 *DR16,Class_SteeringWheel_Infantry_BoardCAN *BoardCAN,SteeringWheel_Infantry_BoardCAN_Role_e Board_Roll);

    /**
     * @brief 判断DR16是否安装在当前主控
     *
     * @return bool true表示当前主控直接连接DR16
     */
    bool Is_DR16_Local_Board();

    /**
     * @brief 更新遥控数据并按需发送板间CAN数据
     *
     * @param Now_ms 当前系统时间，单位ms
     */
    void Update(uint32_t Now_ms);

    /**
     * @brief 获取遥控数据在线状态
     *
     * @return bool true表示当前遥控数据源在线
     */
    inline bool Get_Online_State()
    {
        return (Remote_Online);
    }

    /**
     * @brief 获取遥控器右侧X轴摇杆数据
     *
     * @return float 归一化到-1~1的右侧X轴摇杆数据
     */
    inline float Get_Right_X()
    {
        return (Right_X);
    }

    /**
     * @brief 获取遥控器右侧Y轴摇杆数据
     *
     * @return float 归一化到-1~1的右侧Y轴摇杆数据
     */
    inline float Get_Right_Y()
    {
        return (Right_Y);
    }

    /**
     * @brief 获取遥控器左侧X轴摇杆数据
     *
     * @return float 归一化到-1~1的左侧X轴摇杆数据
     */
    inline float Get_Left_X()
    {
        return (Left_X);
    }

    /**
     * @brief 获取遥控器左侧Y轴摇杆数据
     *
     * @return float 归一化到-1~1的左侧Y轴摇杆数据
     */
    inline float Get_Left_Y()
    {
        return (Left_Y);
    }

    /**
     * @brief 获取遥控器拨轮数据
     *
     * @return float 归一化到-1~1的拨轮数据
     */
    inline float Get_Dial_Wheel()
    {
        return (Dial_Wheel);
    }

    /**
     * @brief 获取遥控器左侧三档开关状态
     *
     * @return BoardCAN_Remote_ThreeKey_e 左侧三档开关稳态
     */
    inline BoardCAN_Remote_ThreeKey_e Get_Left_ThreeKey()
    {
        return (Left_ThreeKey);
    }

    /**
     * @brief 获取遥控器右侧三档开关状态
     *
     * @return BoardCAN_Remote_ThreeKey_e 右侧三档开关稳态
     */
    inline BoardCAN_Remote_ThreeKey_e Get_Right_ThreeKey()
    {
        return (Right_ThreeKey);
    }

private:
    Class_DR16 *DR16 = nullptr;

    Class_SteeringWheel_Infantry_BoardCAN *BoardCAN = nullptr;

    SteeringWheel_Infantry_BoardCAN_Role_e BoardCAN_This_Board_Roll = SteeringWheel_Infantry_BoardCAN_Role_e::Gimbal;

    bool Remote_Online = false;

    float Right_X = 0.0f;
    float Right_Y = 0.0f;
    float Left_X = 0.0f;
    float Left_Y = 0.0f;
    float Dial_Wheel = 0.0f;

    BoardCAN_Remote_ThreeKey_e Left_ThreeKey = BoardCAN_Remote_ThreeKey_e::Error;
    BoardCAN_Remote_ThreeKey_e Right_ThreeKey = BoardCAN_Remote_ThreeKey_e::Error;

    uint32_t BoardCAN_Last_Transmit_Time = 0;

    void Update_From_DR16(uint32_t Now_ms);

    void Update_From_BoardCAN(uint32_t Now_ms);

    void Send_DR16_Data_To_BoardCAN(uint32_t Now_ms);

    void Reset_Remote_Data();
};






#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_DATA_RECEPTION_H__ */
