/* USER CODE BEGIN Header */
/**
  ******************************************************************************
    * @file    Common_Motor.h
  * @brief   This file contains all the function prototypes for
    *          the Common_Motor.c/.cpp file
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMON_MOTOR_H__
#define __COMMON_MOTOR_H__

#include "stm32f405xx.h"
#include <cstdint>
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

//前置库
#include "bsp_pwm.h"
#include "bsp_encoder.h"
/*YOUR CODE*/

typedef enum
{
    COMMON_MOTOR_NORMAL = 0,
    COMMON_MOTOR_DIR_PROTECT
} Common_Motor_State_e;

class Class_Common_Motor
{
    public:

        /**
        * @brief 初始化电机
        * 
        * @param Enable_GPIO_Port 使能引脚的GPIO端口
        * @param Enable_Pin 使能引脚
        * @param INA_htim INA通道的定时器句柄
        * @param INA_Channel INA通道
        * @param INB_htim INB通道的定时器句柄
        * @param INB_Channel INB通道
        * @param Motor_Wheel_Radius 轮半径
        * @param Motor_Direction 电机方向 1或-1 便于人为调整 默认是1
        * @param Encoder_htim 编码器定时器句柄
        * @param Encoder_Update_Period_ms 编码器更新周期，单位 ms
        * @param Encoder_Number 编码器分辨率
        * @param Encoder_Multiple 编码器倍增
        * @param Gear_Ratio 电机齿轮比
        * @param Encoder_Direction 编码器方向 1或-1 便于人为调整 默认是1
        */
        void Init(GPIO_TypeDef *Enable_GPIO_Port, uint16_t Enable_Pin, 
                  TIM_HandleTypeDef *INA_htim, uint32_t INA_Channel, 
                  TIM_HandleTypeDef *INB_htim, uint32_t INB_Channel,
                  float Motor_Wheel_Radius, int8_t Motor_Direction,
                  TIM_HandleTypeDef *Encoder_htim,
                  uint16_t Encoder_Update_Period_ms, int32_t Encoder_Number,int16_t Encoder_Multiple,
                  float Gear_Ratio, int8_t Encoder_Direction);

        /**
        * @brief 设置电机目标占空比
        * 
        * @param Duty 目标占空比，范围[-0.98, 0.98]
        */          
        void Set_Target_Duty(float Duty);

        /**
        * @brief 设置强制无力模式
        *
        * @param Enable true：强制无力滑行
        *               false：恢复正常控制
        */
        void Set_NoForce_State(bool Enable);

        /**
        * @brief 更新电机状态 1ms调用周期
        * 
        */
        void Update(void);

        /**
         * @brief 获取电机实际速度，单位为m/s
         * 
         * @return float 电机实际速度
         */
        float Get_Motor_Speed(void)
        {
            return Motor_Speed;
        }

        /**
         * @brief 获取电机输出轴相对角度，范围为 [-180, 180)
         */
        float Get_Motor_Output_Angle_Deg(void) const
        {
            return Motor_Output_Angle_Deg;
        }

        /**
         * @brief 获取电机输出轴连续角度，不进行360度限制
         */
        float Get_Motor_Output_Continuous_Angle_Deg(void) const
        {
            return Motor_Output_Continuous_Angle_Deg;
        }

        /**
         * @brief 将当前位置设置为电机输出轴相对角度零点
         * @note 在下一次编码器更新时生效
         */
        void Reset_Motor_Output_Angle(void);

        Class_PWM INA;
        Class_PWM INB;
        Class_Encoder Motor_Encoder;

    private:
    GPIO_TypeDef *Enable_GPIO_Port;
    uint16_t Enable_Pin;

    float Motor_Wheel_Radius = 0.0f;
    float Motor_Speed = 0.0f;

    uint16_t Encoder_Update_Period_ms = 1;
    uint16_t Encoder_Update_Divider = 0;

    int32_t Encoder_Counts_Per_Output_Revolution = 1;
    int64_t Motor_Output_Angle_Zero_Count = 0;
    float Motor_Output_Angle_Deg = 0.0f;
    float Motor_Output_Continuous_Angle_Deg = 0.0f;
    bool Motor_Output_Angle_Reset_Pending = false;

    void Update_Motor_Output_Angle(void);

    int8_t Motor_Direction = 1;

    bool NoForce_State = false;

    /**
     * @brief 初始化电机使能引脚
     * 
     * @param GPIO_Port 使能引脚的GPIO端口
     * @param GPIO_Pin 使能引脚
     */
    void Enable_Init(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin)
    {
        Enable_GPIO_Port = GPIO_Port;
        Enable_Pin = GPIO_Pin;
        HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);
    }

    bool Enable_State = false;

    /**
     * @brief 设置电机使能状态
     * 
     * @param State 使能状态，true为使能，false为禁用
     */
    void Set_Enable(bool State)
    {
        if (Enable_State == State) return;
        Enable_State = State;
        HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, State ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    Common_Motor_State_e State = COMMON_MOTOR_NORMAL;
    
    float Target_Duty = 0.0f;

    int8_t Last_Direction = 0;

    uint32_t Protect_Start_Time = 0;

    
};






#ifdef __cplusplus
}
#endif

#endif /* __COMMON_MOTOR_H__ */
