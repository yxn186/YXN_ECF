/* USER CODE BEGIN Header */
/**
  ******************************************************************************
    * @file    Common_Motor.cpp
  * @brief   电机库
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "Common_Motor.h"

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
 * @param Gear_Ratio 减速比
 * @param Encoder_Direction 编码器方向 1或-1 便于人为调整 默认是1
 */
void Class_Common_Motor::Init(GPIO_TypeDef *Enable_GPIO_Port, uint16_t Enable_Pin,
                       TIM_HandleTypeDef *INA_htim, uint32_t INA_Channel, 
                       TIM_HandleTypeDef *INB_htim, uint32_t INB_Channel,
                       float Motor_Wheel_Radius, int8_t Motor_Direction,
                       TIM_HandleTypeDef *Encoder_htim,
                       uint16_t Encoder_Update_Period_ms, int32_t Encoder_Number,int16_t Encoder_Multiple,
                       float Gear_Ratio, int8_t Encoder_Direction)
{
    //设置引脚和定时器参数
    Enable_Init(Enable_GPIO_Port, Enable_Pin);
    INA.Init(INA_htim, INA_Channel,0.98);
    INB.Init(INB_htim, INB_Channel,0.98);

    this->Motor_Direction = Motor_Direction;
    this->Motor_Wheel_Radius = Motor_Wheel_Radius;

    if (Encoder_Update_Period_ms == 0)
    {
        Encoder_Update_Period_ms = 1;
    }
    this->Encoder_Update_Period_ms = Encoder_Update_Period_ms;
    Encoder_Update_Divider = 0;

    Motor_Encoder.Init(Encoder_htim,
                       (float)Encoder_Update_Period_ms / 1000.0f,
                       Encoder_Number,Encoder_Multiple,Gear_Ratio,Encoder_Direction);

    float Counts_Per_Output_Revolution =
        (float)Encoder_Number * (float)Encoder_Multiple * Gear_Ratio;

    if (Counts_Per_Output_Revolution < 1.0f)
    {
        Counts_Per_Output_Revolution = 1.0f;
    }

    Encoder_Counts_Per_Output_Revolution =
        (int32_t)(Counts_Per_Output_Revolution + 0.5f);
    Motor_Output_Angle_Zero_Count = Motor_Encoder.Get_Directed_TotalCount();
    Motor_Output_Angle_Deg = 0.0f;
    Motor_Output_Continuous_Angle_Deg = 0.0f;
    Motor_Output_Angle_Reset_Pending = false;
}

/**
 * @brief 将当前位置设置为电机输出轴相对角度零点
 */
void Class_Common_Motor::Reset_Motor_Output_Angle(void)
{
    Motor_Output_Angle_Deg = 0.0f;
    Motor_Output_Continuous_Angle_Deg = 0.0f;
    Motor_Output_Angle_Reset_Pending = true;
}

/**
 * @brief 根据编码器累计计数更新输出轴相对角度
 */
void Class_Common_Motor::Update_Motor_Output_Angle(void)
{
    int64_t Total_Count = Motor_Encoder.Get_Directed_TotalCount();

    if (Motor_Output_Angle_Reset_Pending)
    {
        Motor_Output_Angle_Zero_Count = Total_Count;
        Motor_Output_Angle_Deg = 0.0f;
        Motor_Output_Continuous_Angle_Deg = 0.0f;
        Motor_Output_Angle_Reset_Pending = false;
        return;
    }

    int64_t Relative_Count = Total_Count - Motor_Output_Angle_Zero_Count;

    Motor_Output_Continuous_Angle_Deg =
        (float)Relative_Count * 360.0f /
        (float)Encoder_Counts_Per_Output_Revolution;

    int64_t Count_In_One_Revolution =
        Relative_Count % (int64_t)Encoder_Counts_Per_Output_Revolution;

    if (Count_In_One_Revolution < 0)
    {
        Count_In_One_Revolution += Encoder_Counts_Per_Output_Revolution;
    }

    Motor_Output_Angle_Deg =
        (float)Count_In_One_Revolution * 360.0f /
        (float)Encoder_Counts_Per_Output_Revolution;

    if (Motor_Output_Angle_Deg >= 180.0f)
    {
        Motor_Output_Angle_Deg -= 360.0f;
    }
}

/**
 * @brief 设置电机目标占空比
 * 
 * @param Duty 目标占空比，范围[-0.98, 0.98]
 */
void Class_Common_Motor::Set_Target_Duty(float Duty)
{
    if (Duty > 0.99f) Duty = 0.99f;
    if (Duty < -0.99f) Duty = -0.99f;

    if ((Duty > -0.01f) && (Duty < 0.01f)) Duty = 0.0f;
    

    Target_Duty = Duty * Motor_Direction;
}

/**
 * @brief 设置强制无力模式
 *
 * @param Enable true：强制无力滑行
 *               false：恢复正常控制
 */
void Class_Common_Motor::Set_NoForce_State(bool Enable)
{
    NoForce_State = Enable;

    if (NoForce_State)
    {
        //先关闭驱动器，立即进入无力状态
        Set_Enable(false);

        // 同时把PWM输出清零
        INA.SetDuty(0.0f);
        INB.SetDuty(0.0f);

        Target_Duty = 0.0f;

        //取消可能正在进行的换向保护状态
        State = COMMON_MOTOR_NORMAL;
        Last_Direction = 0;
    }
}

/**
 * @brief 更新电机状态 1ms调用周期
 * 
 */
void Class_Common_Motor::Update(void)
{
    Encoder_Update_Divider++;
    if (Encoder_Update_Divider >= Encoder_Update_Period_ms)
    {
        Encoder_Update_Divider = 0;
        Motor_Encoder.Update();
        Update_Motor_Output_Angle();
    }

    //计算实际速度
    Motor_Speed = Motor_Encoder.Get_Output_W() * Motor_Wheel_Radius;

    // 强制无力模式优先级最高
    if (NoForce_State)
    {
        Set_Enable(false);
        INA.SetDuty(0.0f);
        INB.SetDuty(0.0f);

        State = COMMON_MOTOR_NORMAL;
        return;
    }

    int8_t Target_Direction = 0;

    if (Target_Duty > 0.0f)
    {
        Target_Direction = 1;
    }
    else if (Target_Duty < 0.0f)
    {
        Target_Direction = -1;
    }
    else
    {
        Target_Direction = 0;
    }

    switch (State)
    {
        case COMMON_MOTOR_NORMAL:
        {
            if ((Last_Direction != 0) && (Target_Direction != 0) && (Last_Direction != Target_Direction))
            {
                //检测到正反转切换
                INA.SetDuty(0.0f);
                INB.SetDuty(0.0f);
                Set_Enable(false);

                Protect_Start_Time = HAL_GetTick();

                State = COMMON_MOTOR_DIR_PROTECT;//进入换向保护状态
                return;
            }

            // 没有换向，正常输出
            if (Target_Duty > 0.0f)
            {
                Set_Enable(true);
                INA.SetDuty(Target_Duty);
                INB.SetDuty(0.0f);
                Last_Direction = 1;
            }
            else if (Target_Duty < 0.0f)
            {
                Set_Enable(true);
                INA.SetDuty(0.0f);
                INB.SetDuty(-Target_Duty);
                Last_Direction = -1;
            }
            else//Target_Duty=0 刹车
            {
                //用力刹车
                INA.SetDuty(0.0f);
                INB.SetDuty(0.0f);

                //用力刹车 true
                //此处使用情况为机械臂 用false
                Set_Enable(false);
            }

            break;
        }

        case COMMON_MOTOR_DIR_PROTECT:
        {
            // 等待保护时间
            if (HAL_GetTick() - Protect_Start_Time >= 2)
            {
                Last_Direction = 0;
                State = COMMON_MOTOR_NORMAL;
            }

            break;
        }
    }
}
