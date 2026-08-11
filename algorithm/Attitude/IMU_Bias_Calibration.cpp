/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    IMU_Bias_Calibration.cpp
  * @brief   IMU零偏校准
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */

#include "IMU_Bias_Calibration.h"
#include <math.h>

/**
 * @brief 开始一次新的IMU零偏校准
 *
 * @param New_Target_Samples 本次校准检查的总样本数
 * @param New_GYRO_Norm_Max_DPS 有效样本允许的最大角速度模长，单位degree/s
 * @param New_ACC_Norm_Min_G 有效样本允许的最小加速度模长，单位g
 * @param New_ACC_Norm_Max_G 有效样本允许的最大加速度模长，单位g
 */
void Class_IMU_Bias_Calibration::Start(uint32_t New_Target_Samples,float New_GYRO_Norm_Max_DPS,float New_ACC_Norm_Min_G,float New_ACC_Norm_Max_G)
{
    //重新校准时总样本数和有效样本数都必须从零开始
    Target_Samples = New_Target_Samples;
    Current_Samples = 0;
    Effective_Samples = 0;

    GYRO_Norm_Max_DPS = New_GYRO_Norm_Max_DPS;
    ACC_Norm_Min_G = New_ACC_Norm_Min_G;
    ACC_Norm_Max_G = New_ACC_Norm_Max_G;

    GYRO_Sum_X_DPS = 0.0f;
    GYRO_Sum_Y_DPS = 0.0f;
    GYRO_Sum_Z_DPS = 0.0f;
    ACC_Sum_X_G = 0.0f;
    ACC_Sum_Y_G = 0.0f;
    ACC_Sum_Z_G = 0.0f;

    GYRO_Bias_X_DPS = 0.0f;
    GYRO_Bias_Y_DPS = 0.0f;
    GYRO_Bias_Z_DPS = 0.0f;
    ACC_Bias_X_G = 0.0f;
    ACC_Bias_Y_G = 0.0f;
    ACC_Bias_Z_G = 0.0f;

    if ((Target_Samples == 0) ||
        (GYRO_Norm_Max_DPS <= 0.0f) ||
        (ACC_Norm_Min_G <= 0.0f) ||
        (ACC_Norm_Max_G <= ACC_Norm_Min_G))
    {
        States = IMU_Bias_Calibration_States_e::No_Valid_Sample;
        return;
    }

    States = IMU_Bias_Calibration_States_e::Calibrating;
}

/**
 * @brief 输入一组IMU样本并推进零偏校准
 *
 * @param GYRO_X_DPS X轴角速度，单位degree/s
 * @param GYRO_Y_DPS Y轴角速度，单位degree/s
 * @param GYRO_Z_DPS Z轴角速度，单位degree/s
 * @param ACC_X_G X轴加速度，单位g
 * @param ACC_Y_G Y轴加速度，单位g
 * @param ACC_Z_G Z轴加速度，单位g
 * @return IMU_Bias_Calibration_States_e 输入本次样本后的校准状态
 */
IMU_Bias_Calibration_States_e Class_IMU_Bias_Calibration::Push_Sample(float GYRO_X_DPS,float GYRO_Y_DPS,float GYRO_Z_DPS,
                                                                       float ACC_X_G,float ACC_Y_G,float ACC_Z_G)
{
    if (States != IMU_Bias_Calibration_States_e::Calibrating)
    {
        return States;
    }

    //只有角速度较小且重力模长正常的样本才参与平均
    float GYRO_Norm_DPS = sqrtf(GYRO_X_DPS * GYRO_X_DPS +
                                GYRO_Y_DPS * GYRO_Y_DPS +
                                GYRO_Z_DPS * GYRO_Z_DPS);
    float ACC_Norm_G = sqrtf(ACC_X_G * ACC_X_G +
                             ACC_Y_G * ACC_Y_G +
                             ACC_Z_G * ACC_Z_G);

    if ((GYRO_Norm_DPS <= GYRO_Norm_Max_DPS) &&
        (ACC_Norm_G > ACC_Norm_Min_G) &&
        (ACC_Norm_G < ACC_Norm_Max_G))
    {
        GYRO_Sum_X_DPS += GYRO_X_DPS;
        GYRO_Sum_Y_DPS += GYRO_Y_DPS;
        GYRO_Sum_Z_DPS += GYRO_Z_DPS;

        ACC_Sum_X_G += ACC_X_G;
        ACC_Sum_Y_G += ACC_Y_G;
        ACC_Sum_Z_G += ACC_Z_G;
        Effective_Samples++;
    }

    Current_Samples++;
    if (Current_Samples < Target_Samples)
    {
        return States;
    }

    //总窗口结束后没有静止样本，不能用零值假装校准成功
    if (Effective_Samples == 0)
    {
        States = IMU_Bias_Calibration_States_e::No_Valid_Sample;
        return States;
    }

    //陀螺仪静止均值可以直接作为三轴零偏
    float Sample_Inv = 1.0f / Effective_Samples;
    GYRO_Bias_X_DPS = GYRO_Sum_X_DPS * Sample_Inv;
    GYRO_Bias_Y_DPS = GYRO_Sum_Y_DPS * Sample_Inv;
    GYRO_Bias_Z_DPS = GYRO_Sum_Z_DPS * Sample_Inv;

    float ACC_Average_X_G = ACC_Sum_X_G * Sample_Inv;
    float ACC_Average_Y_G = ACC_Sum_Y_G * Sample_Inv;
    float ACC_Average_Z_G = ACC_Sum_Z_G * Sample_Inv;
    float ACC_Average_Norm_G = sqrtf(ACC_Average_X_G * ACC_Average_X_G +
                                     ACC_Average_Y_G * ACC_Average_Y_G +
                                     ACC_Average_Z_G * ACC_Average_Z_G);

    //保留平均重力方向，只去掉加速度计模长方向上的固定偏差
    if (ACC_Average_Norm_G > 0.000001f)
    {
        float Gravity_X = ACC_Average_X_G / ACC_Average_Norm_G;
        float Gravity_Y = ACC_Average_Y_G / ACC_Average_Norm_G;
        float Gravity_Z = ACC_Average_Z_G / ACC_Average_Norm_G;

        ACC_Bias_X_G = ACC_Average_X_G - Gravity_X;
        ACC_Bias_Y_G = ACC_Average_Y_G - Gravity_Y;
        ACC_Bias_Z_G = ACC_Average_Z_G - Gravity_Z;
    }

    States = IMU_Bias_Calibration_States_e::Finished;
    return States;
}

/**
 * @brief 获取本次校准总采样窗口的完成进度
 *
 * @return float 校准进度，范围为0.0到1.0
 */
float Class_IMU_Bias_Calibration::Get_Progress()
{
    if (Target_Samples == 0)
    {
        return 0.0f;
    }

    float Progress = (float)Current_Samples / Target_Samples;
    if (Progress > 1.0f)
    {
        Progress = 1.0f;
    }

    return Progress;
}
