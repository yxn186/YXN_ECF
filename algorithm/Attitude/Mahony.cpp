/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Mahony.cpp
  * @brief   Mahony IMU姿态解算
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */

#include "Mahony.h"
#include <math.h>

/**
 * @brief 初始化Mahony参数并把姿态重置为单位四元数
 *
 * @param New_Kp 加速度修正姿态的比例系数
 * @param New_Ki 加速度修正姿态的积分系数
 * @param New_ACC_Norm_Min_G 允许使用加速度修正的最小模长，单位g
 * @param New_ACC_Norm_Max_G 允许使用加速度修正的最大模长，单位g
 */
void Class_Mahony::Init(float New_Kp,float New_Ki,float New_ACC_Norm_Min_G,float New_ACC_Norm_Max_G)
{
    Kp = New_Kp;
    Ki = New_Ki;
    ACC_Norm_Min_G = New_ACC_Norm_Min_G;
    ACC_Norm_Max_G = New_ACC_Norm_Max_G;
    Reset();
}

/**
 * @brief 清除积分项并把当前姿态恢复为单位四元数
 */
void Class_Mahony::Reset()
{
    Quaternion_W = 1.0f;
    Quaternion_X = 0.0f;
    Quaternion_Y = 0.0f;
    Quaternion_Z = 0.0f;

    Integral_X = 0.0f;
    Integral_Y = 0.0f;
    Integral_Z = 0.0f;
}

/**
 * @brief 使用一组角速度和加速度数据更新姿态四元数
 *
 * @param GYRO_X_Rad_s X轴角速度，单位rad/s
 * @param GYRO_Y_Rad_s Y轴角速度，单位rad/s
 * @param GYRO_Z_Rad_s Z轴角速度，单位rad/s
 * @param ACC_X_G X轴加速度，单位g
 * @param ACC_Y_G Y轴加速度，单位g
 * @param ACC_Z_G Z轴加速度，单位g
 * @param Dt_s 本次姿态积分间隔，单位s
 */
void Class_Mahony::Update(float GYRO_X_Rad_s,float GYRO_Y_Rad_s,float GYRO_Z_Rad_s,
                          float ACC_X_G,float ACC_Y_G,float ACC_Z_G,float Dt_s)
{
    if ((Dt_s <= 0.0f) || (Dt_s > 0.1f))
    {
        return;
    }

    //云台明显加速时不使用加速度修正，避免把平移加速度当成重力方向
    float ACC_Norm_G = sqrtf(ACC_X_G * ACC_X_G + ACC_Y_G * ACC_Y_G + ACC_Z_G * ACC_Z_G);
    if ((ACC_Norm_G > ACC_Norm_Min_G) && (ACC_Norm_G < ACC_Norm_Max_G))
    {
        ACC_X_G /= ACC_Norm_G;
        ACC_Y_G /= ACC_Norm_G;
        ACC_Z_G /= ACC_Norm_G;

        float Half_Gravity_X = Quaternion_X * Quaternion_Z - Quaternion_W * Quaternion_Y;
        float Half_Gravity_Y = Quaternion_W * Quaternion_X + Quaternion_Y * Quaternion_Z;
        float Half_Gravity_Z = Quaternion_W * Quaternion_W - 0.5f + Quaternion_Z * Quaternion_Z;

        float Half_Error_X = ACC_Y_G * Half_Gravity_Z - ACC_Z_G * Half_Gravity_Y;
        float Half_Error_Y = ACC_Z_G * Half_Gravity_X - ACC_X_G * Half_Gravity_Z;
        float Half_Error_Z = ACC_X_G * Half_Gravity_Y - ACC_Y_G * Half_Gravity_X;

        //积分项只在Ki启用时保留，关闭Ki后不留下旧的累计值
        if (Ki > 0.0f)
        {
            Integral_X += Ki * Half_Error_X * Dt_s;
            Integral_Y += Ki * Half_Error_Y * Dt_s;
            Integral_Z += Ki * Half_Error_Z * Dt_s;

            GYRO_X_Rad_s += Integral_X;
            GYRO_Y_Rad_s += Integral_Y;
            GYRO_Z_Rad_s += Integral_Z;
        }
        else
        {
            Integral_X = 0.0f;
            Integral_Y = 0.0f;
            Integral_Z = 0.0f;
        }

        GYRO_X_Rad_s += Kp * Half_Error_X;
        GYRO_Y_Rad_s += Kp * Half_Error_Y;
        GYRO_Z_Rad_s += Kp * Half_Error_Z;
    }

    //使用实际dt积分四元数，不把采样周期写死在算法内部
    float Half_Dt = 0.5f * Dt_s;
    GYRO_X_Rad_s *= Half_Dt;
    GYRO_Y_Rad_s *= Half_Dt;
    GYRO_Z_Rad_s *= Half_Dt;

    float Old_W = Quaternion_W;
    float Old_X = Quaternion_X;
    float Old_Y = Quaternion_Y;

    Quaternion_W += -Old_X * GYRO_X_Rad_s - Old_Y * GYRO_Y_Rad_s - Quaternion_Z * GYRO_Z_Rad_s;
    Quaternion_X +=  Old_W * GYRO_X_Rad_s + Old_Y * GYRO_Z_Rad_s - Quaternion_Z * GYRO_Y_Rad_s;
    Quaternion_Y +=  Old_W * GYRO_Y_Rad_s - Old_X * GYRO_Z_Rad_s + Quaternion_Z * GYRO_X_Rad_s;
    Quaternion_Z +=  Old_W * GYRO_Z_Rad_s + Old_X * GYRO_Y_Rad_s - Old_Y * GYRO_X_Rad_s;

    //每次更新后归一化，防止长时间积分让四元数模长逐渐偏离1
    float Quaternion_Norm = sqrtf(Quaternion_W * Quaternion_W +
                                  Quaternion_X * Quaternion_X +
                                  Quaternion_Y * Quaternion_Y +
                                  Quaternion_Z * Quaternion_Z);
    if (Quaternion_Norm <= 0.000001f)
    {
        Reset();
        return;
    }

    float Quaternion_Norm_Inv = 1.0f / Quaternion_Norm;
    Quaternion_W *= Quaternion_Norm_Inv;
    Quaternion_X *= Quaternion_Norm_Inv;
    Quaternion_Y *= Quaternion_Norm_Inv;
    Quaternion_Z *= Quaternion_Norm_Inv;
}
