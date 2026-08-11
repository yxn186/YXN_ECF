/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Rotation3D.h
  * @brief   三维旋转数学工具
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ROTATION3D_H__
#define __ROTATION3D_H__

#include "main.h"

/**
 * @brief 三维浮点向量
 */
typedef struct
{
    float X = 0.0f;
    float Y = 0.0f;
    float Z = 0.0f;
} Vector3f_t;

/**
 * @brief WXYZ顺序的浮点四元数
 */
typedef struct
{
    float W = 1.0f;
    float X = 0.0f;
    float Y = 0.0f;
    float Z = 0.0f;
} Quaternionf_t;

/**
 * @brief 3x3浮点旋转矩阵
 */
typedef struct
{
    float Data[3][3] =
    {
        {1.0f,0.0f,0.0f},
        {0.0f,1.0f,0.0f},
        {0.0f,0.0f,1.0f}
    };
} Matrix3f_t;

/**
 * @brief 将3x3矩阵设置为单位矩阵
 *
 * @param Matrix 需要写入的矩阵
 */
void Rotation3D_Set_Identity(Matrix3f_t *Matrix);

/**
 * @brief 计算两个3x3矩阵的乘积
 *
 * @param A 左侧矩阵
 * @param B 右侧矩阵
 * @param Result 用于保存A乘B的矩阵
 */
void Rotation3D_Matrix_Multiply(const Matrix3f_t *A,const Matrix3f_t *B,Matrix3f_t *Result);

/**
 * @brief 计算3x3矩阵的转置
 *
 * @param Source 原矩阵
 * @param Result 用于保存转置结果的矩阵
 */
void Rotation3D_Matrix_Transpose(const Matrix3f_t *Source,Matrix3f_t *Result);

/**
 * @brief 使用3x3矩阵旋转一个三维向量
 *
 * @param Matrix 旋转矩阵
 * @param Vector 需要旋转的向量
 * @return Vector3f_t 旋转后的向量，参数无效时返回零向量
 */
Vector3f_t Rotation3D_Matrix_Multiply_Vector(const Matrix3f_t *Matrix,const Vector3f_t *Vector);

/**
 * @brief 将四元数转换为3x3旋转矩阵
 *
 * @param Quaternion 输入四元数
 * @param Matrix 输出旋转矩阵
 */
void Rotation3D_Quaternion_To_Matrix(const Quaternionf_t *Quaternion,Matrix3f_t *Matrix);

/**
 * @brief 将3x3旋转矩阵转换为四元数
 *
 * @param Matrix 输入旋转矩阵
 * @param Quaternion 输出四元数
 */
void Rotation3D_Matrix_To_Quaternion(const Matrix3f_t *Matrix,Quaternionf_t *Quaternion);

/**
 * @brief 计算两个四元数的乘积
 *
 * @param A 左侧四元数
 * @param B 右侧四元数
 * @param Result 用于保存A乘B的四元数
 */
void Rotation3D_Quaternion_Multiply(const Quaternionf_t *A,const Quaternionf_t *B,Quaternionf_t *Result);

/**
 * @brief 将四元数转换为机体系Yaw、Pitch和Roll
 *
 * @param Quaternion 输入四元数
 * @param Yaw 输出Yaw角，单位degree，允许传入nullptr
 * @param Pitch 输出Pitch角，单位degree，允许传入nullptr
 * @param Roll 输出Roll角，单位degree，允许传入nullptr
 */
void Rotation3D_Quaternion_To_Yaw_Pitch_Roll_Degree(const Quaternionf_t *Quaternion,float *Yaw,float *Pitch,float *Roll);

/**
 * @brief 将角度限制到(-180,180]范围
 *
 * @param Angle 输入角度，单位degree
 * @return float 包角后的角度，单位degree
 */
float Rotation3D_Wrap_Angle_Degree(float Angle);

/**
 * @brief 保留原BMI088代码使用的ZXY外旋方向修正函数
 *
 * @param ex_z_deg 绕固定Z轴的外旋角，单位degree
 * @param ex_x_deg 绕固定X轴的外旋角，单位degree
 * @param ex_y_deg 绕固定Y轴的外旋角，单位degree
 * @param f_body_x 机体前向量的X分量
 * @param f_body_y 机体前向量的Y分量
 * @param f_body_z 机体前向量的Z分量
 * @param u_body_x 机体上向量的X分量
 * @param u_body_y 机体上向量的Y分量
 * @param u_body_z 机体上向量的Z分量
 * @param yaw_deg 输出Yaw角，单位degree，允许传入nullptr
 * @param pitch_deg 输出Pitch角，单位degree，允许传入nullptr
 * @param roll_deg 输出Roll角，单位degree，允许传入nullptr
 */
void euler_extrinsic_ZXY_body_axes_to_front_yaw_pitch_roll_deg(float ex_z_deg,
                                                               float ex_x_deg,
                                                               float ex_y_deg,
                                                               float f_body_x,
                                                               float f_body_y,
                                                               float f_body_z,
                                                               float u_body_x,
                                                               float u_body_y,
                                                               float u_body_z,
                                                               float *yaw_deg,
                                                               float *pitch_deg,
                                                               float *roll_deg);

#endif /* __ROTATION3D_H__ */
