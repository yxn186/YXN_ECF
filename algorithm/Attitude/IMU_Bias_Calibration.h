/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    IMU_Bias_Calibration.h
  * @brief   IMU零偏校准
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __IMU_BIAS_CALIBRATION_H__
#define __IMU_BIAS_CALIBRATION_H__

#include "main.h"

/**
 * @brief IMU零偏校准结果
 */
enum class IMU_Bias_Calibration_States_e : uint8_t
{
    Uninitialized = 0,
    Calibrating,
    Finished,
    No_Valid_Sample
};

/**
 * @brief IMU零偏校准类
 *
 * 保留原BMI088校准逻辑：在固定总采样窗口内筛选静止样本，再使用有效样本求平均值。
 */
class Class_IMU_Bias_Calibration
{
public:
    /**
     * @brief 开始一次零偏校准
     *
     * @param Target_Samples 总采样窗口
     * @param GYRO_Norm_Max_DPS 允许参与校准的最大角速度模长
     * @param ACC_Norm_Min_G 允许参与校准的最小加速度模长
     * @param ACC_Norm_Max_G 允许参与校准的最大加速度模长
     */
    void Start(uint32_t Target_Samples,float GYRO_Norm_Max_DPS,float ACC_Norm_Min_G,float ACC_Norm_Max_G);

    /**
     * @brief 输入一组IMU样本
     *
     * @param GYRO_X_DPS X轴角速度，单位degree/s
     * @param GYRO_Y_DPS Y轴角速度，单位degree/s
     * @param GYRO_Z_DPS Z轴角速度，单位degree/s
     * @param ACC_X_G X轴加速度，单位g
     * @param ACC_Y_G Y轴加速度，单位g
     * @param ACC_Z_G Z轴加速度，单位g
     * @return IMU_Bias_Calibration_States_e 输入本次样本后的校准状态
     */
    IMU_Bias_Calibration_States_e Push_Sample(float GYRO_X_DPS,float GYRO_Y_DPS,float GYRO_Z_DPS,
                                               float ACC_X_G,float ACC_Y_G,float ACC_Z_G);

    /**
     * @brief 获取当前校准状态
     *
     * @return IMU_Bias_Calibration_States_e 当前校准状态
     */
    inline IMU_Bias_Calibration_States_e Get_States() { return States; }

    /**
     * @brief 获取本次校准需要检查的总样本数
     *
     * @return uint32_t 总样本数
     */
    inline uint32_t Get_Target_Samples() { return Target_Samples; }

    /**
     * @brief 获取本次校准已经检查的样本数
     *
     * @return uint32_t 已检查样本数
     */
    inline uint32_t Get_Current_Samples() { return Current_Samples; }

    /**
     * @brief 获取满足静止条件并参与平均的样本数
     *
     * @return uint32_t 有效样本数
     */
    inline uint32_t Get_Effective_Samples() { return Effective_Samples; }

    /**
     * @brief 获取本次校准的总窗口进度
     *
     * @return float 校准进度，范围为0.0到1.0
     */
    float Get_Progress();

    /**
     * @brief 获取X轴陀螺仪零偏
     *
     * @return float X轴零偏，单位degree/s
     */
    inline float Get_GYRO_Bias_X_DPS() { return GYRO_Bias_X_DPS; }

    /**
     * @brief 获取Y轴陀螺仪零偏
     *
     * @return float Y轴零偏，单位degree/s
     */
    inline float Get_GYRO_Bias_Y_DPS() { return GYRO_Bias_Y_DPS; }

    /**
     * @brief 获取Z轴陀螺仪零偏
     *
     * @return float Z轴零偏，单位degree/s
     */
    inline float Get_GYRO_Bias_Z_DPS() { return GYRO_Bias_Z_DPS; }

    /**
     * @brief 获取X轴加速度计零偏
     *
     * @return float X轴零偏，单位g
     */
    inline float Get_ACC_Bias_X_G() { return ACC_Bias_X_G; }

    /**
     * @brief 获取Y轴加速度计零偏
     *
     * @return float Y轴零偏，单位g
     */
    inline float Get_ACC_Bias_Y_G() { return ACC_Bias_Y_G; }

    /**
     * @brief 获取Z轴加速度计零偏
     *
     * @return float Z轴零偏，单位g
     */
    inline float Get_ACC_Bias_Z_G() { return ACC_Bias_Z_G; }

private:
    IMU_Bias_Calibration_States_e States = IMU_Bias_Calibration_States_e::Uninitialized;

    uint32_t Target_Samples = 0;
    uint32_t Current_Samples = 0;
    uint32_t Effective_Samples = 0;

    float GYRO_Norm_Max_DPS = 3.0f;
    float ACC_Norm_Min_G = 0.9f;
    float ACC_Norm_Max_G = 1.1f;

    float GYRO_Sum_X_DPS = 0.0f;
    float GYRO_Sum_Y_DPS = 0.0f;
    float GYRO_Sum_Z_DPS = 0.0f;
    float ACC_Sum_X_G = 0.0f;
    float ACC_Sum_Y_G = 0.0f;
    float ACC_Sum_Z_G = 0.0f;

    float GYRO_Bias_X_DPS = 0.0f;
    float GYRO_Bias_Y_DPS = 0.0f;
    float GYRO_Bias_Z_DPS = 0.0f;
    float ACC_Bias_X_G = 0.0f;
    float ACC_Bias_Y_G = 0.0f;
    float ACC_Bias_Z_G = 0.0f;
};

#endif /* __IMU_BIAS_CALIBRATION_H__ */
