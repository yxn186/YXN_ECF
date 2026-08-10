/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    DJI_Motor.h
  * @brief   This file contains all the function prototypes for
  *          the DJI_Motor.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "CAN_Interface.h"
/*YOUR CODE*/

/**
 * @brief 大疆电机反馈超时时间，单位ms
 *
 * 可通过编译选项覆盖该宏，以适配不同的反馈周期。
 */
#ifndef DJI_MOTOR_FEEDBACK_TIMEOUT_MS
#define DJI_MOTOR_FEEDBACK_TIMEOUT_MS 100U
#endif

class Class_DJI_Motor;

/**
 * @brief 大疆系电机型号枚举
 * 
 */
typedef enum
{
    DJI_Motor_6020,
    DJI_Motor_3508
}DJI_Motor_Type_Typedef;

/**
 * @brief 大疆电机类 组
 * 
 */
class Class_DJI_Motor_Group
{
public:
    void Init(Class_CAN_Interface *CAN_Interface, DJI_Motor_Type_Typedef type);

    void Register_Motor(Class_DJI_Motor *motor);

    /**
    * @brief 大疆电机上传数据给电机进行控制
    * 
    */
    void Push_Data(void);


    /**
     * @brief 处理接收到的CAN数据进程函数
     * 
     * @param id CAN标准帧ID
     * @param data 数据地址
     * @param length 数据长度
     */
    void Process_CAN_Feedback(uint16_t id,const uint8_t *data,uint8_t length);

protected:
    
    /**
    * @brief 大疆电机获取电机对应CAN-ID的起始ID
    * 
    * @return uint16_t 0x201--3508 0x205--6020
    */
    uint16_t Get_Rx_Start_ID(void) const;
    
    /**
    * @brief 大疆电机获取低位ID发送数据CAN标识符
    * 
    * @return uint16_t 0x200--3508 0x1FF--6020
    */
    uint16_t Get_Tx_Low_ID(void) const;

    /**
    * @brief 大疆电机获取高位ID发送数据CAN标识符
    * 
    * @return uint16_t 0x200--3508 0x1FF--6020
    */
    uint16_t Get_Tx_High_ID(void) const;

private:
    Class_CAN_Interface *CAN_Interface = nullptr;
    DJI_Motor_Type_Typedef Type = DJI_Motor_3508;

    Class_DJI_Motor *Motor_List[8] = {nullptr};

    uint8_t TxData_Low[8] = {0};
    uint8_t TxData_High[8] = {0};

};

/**
 * @brief 大疆电机类
 * 
 */
class Class_DJI_Motor
{
public:
    /**
     * @brief 大疆电机初始化函数
     *
     * @param type 电机型号 DJI_Motor_6020 / DJI_Motor_3508
     * @param id 电机ID（3508: 1-8，6020: 1-7）
     * @param group 电机组
     */
    void Init(DJI_Motor_Type_Typedef type, uint8_t id, Class_DJI_Motor_Group *group);

    /**
     * @brief 更新大疆电机在线状态
     *
     * 根据是否收到过合法反馈以及距离最近反馈的时间判断电机是否在线。
     */
    void Update_Online_State(void);

    /**
     * @brief 获取大疆电机当前在线状态
     *
     * @return true 已收到合法反馈，且反馈未超时
     * @return false 尚未收到合法反馈，或反馈已经超时
     */
    bool Get_Online_State(void) const;

    void Set_Out(int16_t out);

    int16_t Get_Out() const
    {
        return Out;
    }

    uint16_t Get_RawAngle() const
    {
        return RawAngle;
    }

    float Get_Angle() const
    {
        return Angle;
    }

    float Get_Continuous_Angle() const
    {
        return Continuous_Angle;
    }

    float Get_AngleSpeed(void);

    int16_t Get_Torque_Current() const
    {
        return Torque_Current;
    }

    uint8_t Get_Temperature() const
    {
        return Temperature;
    }

protected:
    void FeedBack_Data(const uint8_t *data);

    /**
    * @brief 大疆系电机限幅函数
    * 
    * @param out 
    * @return int16_t 
    */
    int16_t Limit_Out(int16_t out) const;

private:
    DJI_Motor_Type_Typedef Type = DJI_Motor_3508;
    uint8_t ID = 0;
    Class_DJI_Motor_Group *Group = nullptr;

    uint16_t RawAngle = 0;
    float Angle = 0;
    float Last_Angle = 0;
    float Continuous_Angle = 0;
    int16_t Speed_Rpm = 0;
    int16_t Torque_Current = 0;
    uint8_t Temperature = 0;

    bool Online = false;                 // 当前反馈是否在线
    uint32_t Last_Feedback_Time = 0;     // 最近一次合法反馈时间，单位ms
    bool Feedback_Initialized = false;   // 是否至少收到过一次合法反馈

    int16_t Out = 0;

    friend class Class_DJI_Motor_Group;
};


#ifdef __cplusplus
}
#endif

#endif /* __DJI_MOTOR_H__ */
