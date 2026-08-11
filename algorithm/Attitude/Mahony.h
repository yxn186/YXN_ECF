/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Mahony.h
  * @brief   Mahony IMU姿态解算
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAHONY_H__
#define __MAHONY_H__

#include "main.h"

/**
 * @brief 可重复创建的Mahony IMU姿态解算类
 */
class Class_Mahony
{
public:
    /**
     * @brief 初始化Mahony参数并重置四元数
     *
     * @param Kp 加速度修正姿态的比例系数
     * @param Ki 加速度修正姿态的积分系数
     * @param ACC_Norm_Min_G 允许使用加速度修正的最小模长，单位g
     * @param ACC_Norm_Max_G 允许使用加速度修正的最大模长，单位g
     */
    void Init(float Kp,float Ki,float ACC_Norm_Min_G,float ACC_Norm_Max_G);

    /**
     * @brief 重置四元数和积分状态
     */
    void Reset();

    /**
     * @brief 使用一组陀螺仪和加速度数据更新姿态
     *
     * @param GYRO_X_Rad_s X轴角速度，单位rad/s
     * @param GYRO_Y_Rad_s Y轴角速度，单位rad/s
     * @param GYRO_Z_Rad_s Z轴角速度，单位rad/s
     * @param ACC_X_G X轴加速度，单位g
     * @param ACC_Y_G Y轴加速度，单位g
     * @param ACC_Z_G Z轴加速度，单位g
     * @param Dt_s 本次积分时间，单位s
     */
    void Update(float GYRO_X_Rad_s,float GYRO_Y_Rad_s,float GYRO_Z_Rad_s,
                float ACC_X_G,float ACC_Y_G,float ACC_Z_G,float Dt_s);

    /**
     * @brief 获取当前姿态四元数的W分量
     *
     * @return float 四元数W分量
     */
    inline float Get_Quaternion_W() { return Quaternion_W; }

    /**
     * @brief 获取当前姿态四元数的X分量
     *
     * @return float 四元数X分量
     */
    inline float Get_Quaternion_X() { return Quaternion_X; }

    /**
     * @brief 获取当前姿态四元数的Y分量
     *
     * @return float 四元数Y分量
     */
    inline float Get_Quaternion_Y() { return Quaternion_Y; }

    /**
     * @brief 获取当前姿态四元数的Z分量
     *
     * @return float 四元数Z分量
     */
    inline float Get_Quaternion_Z() { return Quaternion_Z; }

private:
    float Kp = 1.0f;
    float Ki = 0.0f;
    float ACC_Norm_Min_G = 0.9f;
    float ACC_Norm_Max_G = 1.1f;

    float Quaternion_W = 1.0f;
    float Quaternion_X = 0.0f;
    float Quaternion_Y = 0.0f;
    float Quaternion_Z = 0.0f;

    float Integral_X = 0.0f;
    float Integral_Y = 0.0f;
    float Integral_Z = 0.0f;
};

#endif /* __MAHONY_H__ */
