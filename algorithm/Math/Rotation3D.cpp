/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Rotation3D.cpp
  * @brief   三维旋转数学工具
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */

#include "Rotation3D.h"
#include <math.h>

#define Rotation3D_Degree_To_Radian 0.017453292519943295f
#define Rotation3D_Radian_To_Degree 57.29577951308232f

/**
 * @brief 计算两个三维向量的点积
 *
 * @param A 第一个向量
 * @param B 第二个向量
 * @return float 两个向量的点积
 */
static float Rotation3D_Vector_Dot(const Vector3f_t &A,const Vector3f_t &B)
{
    return A.X * B.X + A.Y * B.Y + A.Z * B.Z;
}

/**
 * @brief 计算两个三维向量的叉积
 *
 * @param A 第一个向量
 * @param B 第二个向量
 * @return Vector3f_t A叉乘B的结果
 */
static Vector3f_t Rotation3D_Vector_Cross(const Vector3f_t &A,const Vector3f_t &B)
{
    Vector3f_t Result;
    Result.X = A.Y * B.Z - A.Z * B.Y;
    Result.Y = A.Z * B.X - A.X * B.Z;
    Result.Z = A.X * B.Y - A.Y * B.X;
    return Result;
}

/**
 * @brief 计算三维向量的模长
 *
 * @param Vector 输入向量
 * @return float 向量模长
 */
static float Rotation3D_Vector_Norm(const Vector3f_t &Vector)
{
    return sqrtf(Vector.X * Vector.X + Vector.Y * Vector.Y + Vector.Z * Vector.Z);
}

/**
 * @brief 将三维向量归一化
 *
 * @param Vector 输入向量
 * @return Vector3f_t 归一化后的向量，零向量保持不变
 */
static Vector3f_t Rotation3D_Vector_Normalize(Vector3f_t Vector)
{
    float Norm = Rotation3D_Vector_Norm(Vector);
    if (Norm > 0.000001f)
    {
        Vector.X /= Norm;
        Vector.Y /= Norm;
        Vector.Z /= Norm;
    }
    return Vector;
}

/**
 * @brief 根据世界系前向量和上向量计算Yaw、Pitch与Roll
 *
 * @param Forward_World 机体前方向在世界系中的向量
 * @param Up_World 机体上方向在世界系中的向量
 * @param Yaw 输出Yaw角，单位degree，允许传入nullptr
 * @param Pitch 输出Pitch角，单位degree，允许传入nullptr
 * @param Roll 输出Roll角，单位degree，允许传入nullptr
 */
static void Rotation3D_Forward_Up_To_Yaw_Pitch_Roll_Degree(Vector3f_t Forward_World,
                                                            Vector3f_t Up_World,
                                                            float *Yaw,
                                                            float *Pitch,
                                                            float *Roll)
{
    Forward_World = Rotation3D_Vector_Normalize(Forward_World);
    Up_World = Rotation3D_Vector_Normalize(Up_World);

    float Yaw_Rad = atan2f(Forward_World.Y,Forward_World.X);
    float Pitch_Rad = atan2f(-Forward_World.Z,
                             sqrtf(Forward_World.X * Forward_World.X + Forward_World.Y * Forward_World.Y));

    //把世界系Z轴投影到垂直于前向量的平面，作为零Roll参考
    Vector3f_t Reference_Up = {0.0f,0.0f,1.0f};
    float Reference_Dot_Forward = Rotation3D_Vector_Dot(Reference_Up,Forward_World);
    Vector3f_t Projected_Reference_Up =
    {
        Reference_Up.X - Reference_Dot_Forward * Forward_World.X,
        Reference_Up.Y - Reference_Dot_Forward * Forward_World.Y,
        Reference_Up.Z - Reference_Dot_Forward * Forward_World.Z
    };

    //前向量接近竖直时Z轴无法提供参考，改用世界系Y轴
    if (Rotation3D_Vector_Norm(Projected_Reference_Up) < 0.000001f)
    {
        Reference_Up = {0.0f,1.0f,0.0f};
        Reference_Dot_Forward = Rotation3D_Vector_Dot(Reference_Up,Forward_World);
        Projected_Reference_Up =
        {
            Reference_Up.X - Reference_Dot_Forward * Forward_World.X,
            Reference_Up.Y - Reference_Dot_Forward * Forward_World.Y,
            Reference_Up.Z - Reference_Dot_Forward * Forward_World.Z
        };
    }
    Projected_Reference_Up = Rotation3D_Vector_Normalize(Projected_Reference_Up);

    float Up_Dot_Forward = Rotation3D_Vector_Dot(Up_World,Forward_World);
    Vector3f_t Projected_Up =
    {
        Up_World.X - Up_Dot_Forward * Forward_World.X,
        Up_World.Y - Up_Dot_Forward * Forward_World.Y,
        Up_World.Z - Up_Dot_Forward * Forward_World.Z
    };
    Projected_Up = Rotation3D_Vector_Normalize(Projected_Up);

    float Roll_Rad = 0.0f;
    if (Rotation3D_Vector_Norm(Projected_Up) > 0.000001f)
    {
        Vector3f_t Cross_Value = Rotation3D_Vector_Cross(Projected_Reference_Up,Projected_Up);
        float Sin_Roll = Rotation3D_Vector_Dot(Forward_World,Cross_Value);
        float Cos_Roll = Rotation3D_Vector_Dot(Projected_Reference_Up,Projected_Up);
        Roll_Rad = atan2f(Sin_Roll,Cos_Roll);
    }

    if (Yaw != nullptr) *Yaw = Yaw_Rad * Rotation3D_Radian_To_Degree;
    if (Pitch != nullptr) *Pitch = Pitch_Rad * Rotation3D_Radian_To_Degree;
    if (Roll != nullptr) *Roll = Roll_Rad * Rotation3D_Radian_To_Degree;
}

/**
 * @brief 将3x3矩阵设置为单位矩阵
 *
 * @param Matrix 需要写入的矩阵
 */
void Rotation3D_Set_Identity(Matrix3f_t *Matrix)
{
    if (Matrix == nullptr)
    {
        return;
    }

    for (uint8_t Row = 0;Row < 3;Row++)
    {
        for (uint8_t Column = 0;Column < 3;Column++)
        {
            Matrix->Data[Row][Column] = (Row == Column) ? 1.0f : 0.0f;
        }
    }
}

/**
 * @brief 计算两个3x3矩阵的乘积
 *
 * @param A 左侧矩阵
 * @param B 右侧矩阵
 * @param Result 用于保存A乘B的矩阵
 */
void Rotation3D_Matrix_Multiply(const Matrix3f_t *A,const Matrix3f_t *B,Matrix3f_t *Result)
{
    if ((A == nullptr) || (B == nullptr) || (Result == nullptr))
    {
        return;
    }

    Matrix3f_t Temp;
    for (uint8_t Row = 0;Row < 3;Row++)
    {
        for (uint8_t Column = 0;Column < 3;Column++)
        {
            Temp.Data[Row][Column] = A->Data[Row][0] * B->Data[0][Column] +
                                     A->Data[Row][1] * B->Data[1][Column] +
                                     A->Data[Row][2] * B->Data[2][Column];
        }
    }
    *Result = Temp;
}

/**
 * @brief 计算3x3矩阵的转置
 *
 * @param Source 原矩阵
 * @param Result 用于保存转置结果的矩阵
 */
void Rotation3D_Matrix_Transpose(const Matrix3f_t *Source,Matrix3f_t *Result)
{
    if ((Source == nullptr) || (Result == nullptr))
    {
        return;
    }

    Matrix3f_t Temp;
    for (uint8_t Row = 0;Row < 3;Row++)
    {
        for (uint8_t Column = 0;Column < 3;Column++)
        {
            Temp.Data[Row][Column] = Source->Data[Column][Row];
        }
    }
    *Result = Temp;
}

/**
 * @brief 使用3x3矩阵旋转一个三维向量
 *
 * @param Matrix 旋转矩阵
 * @param Vector 需要旋转的向量
 * @return Vector3f_t 旋转后的向量，参数无效时返回零向量
 */
Vector3f_t Rotation3D_Matrix_Multiply_Vector(const Matrix3f_t *Matrix,const Vector3f_t *Vector)
{
    Vector3f_t Result;
    if ((Matrix == nullptr) || (Vector == nullptr))
    {
        return Result;
    }

    Result.X = Matrix->Data[0][0] * Vector->X + Matrix->Data[0][1] * Vector->Y + Matrix->Data[0][2] * Vector->Z;
    Result.Y = Matrix->Data[1][0] * Vector->X + Matrix->Data[1][1] * Vector->Y + Matrix->Data[1][2] * Vector->Z;
    Result.Z = Matrix->Data[2][0] * Vector->X + Matrix->Data[2][1] * Vector->Y + Matrix->Data[2][2] * Vector->Z;
    return Result;
}

/**
 * @brief 将四元数转换为3x3旋转矩阵
 *
 * @param Quaternion 输入四元数
 * @param Matrix 输出旋转矩阵
 */
void Rotation3D_Quaternion_To_Matrix(const Quaternionf_t *Quaternion,Matrix3f_t *Matrix)
{
    if ((Quaternion == nullptr) || (Matrix == nullptr))
    {
        return;
    }

    float Norm = sqrtf(Quaternion->W * Quaternion->W +
                       Quaternion->X * Quaternion->X +
                       Quaternion->Y * Quaternion->Y +
                       Quaternion->Z * Quaternion->Z);
    if (Norm <= 0.000001f)
    {
        Rotation3D_Set_Identity(Matrix);
        return;
    }

    //先归一化四元数，避免输入模长误差破坏旋转矩阵正交性
    float W = Quaternion->W / Norm;
    float X = Quaternion->X / Norm;
    float Y = Quaternion->Y / Norm;
    float Z = Quaternion->Z / Norm;

    Matrix->Data[0][0] = 1.0f - 2.0f * (Y * Y + Z * Z);
    Matrix->Data[0][1] = 2.0f * (X * Y - W * Z);
    Matrix->Data[0][2] = 2.0f * (X * Z + W * Y);
    Matrix->Data[1][0] = 2.0f * (X * Y + W * Z);
    Matrix->Data[1][1] = 1.0f - 2.0f * (X * X + Z * Z);
    Matrix->Data[1][2] = 2.0f * (Y * Z - W * X);
    Matrix->Data[2][0] = 2.0f * (X * Z - W * Y);
    Matrix->Data[2][1] = 2.0f * (Y * Z + W * X);
    Matrix->Data[2][2] = 1.0f - 2.0f * (X * X + Y * Y);
}

/**
 * @brief 将3x3旋转矩阵转换为四元数
 *
 * @param Matrix 输入旋转矩阵
 * @param Quaternion 输出四元数
 */
void Rotation3D_Matrix_To_Quaternion(const Matrix3f_t *Matrix,Quaternionf_t *Quaternion)
{
    if ((Matrix == nullptr) || (Quaternion == nullptr))
    {
        return;
    }

    float Trace = Matrix->Data[0][0] + Matrix->Data[1][1] + Matrix->Data[2][2];
    if (Trace > 0.0f)
    {
        float S = sqrtf(Trace + 1.0f) * 2.0f;
        Quaternion->W = 0.25f * S;
        Quaternion->X = (Matrix->Data[2][1] - Matrix->Data[1][2]) / S;
        Quaternion->Y = (Matrix->Data[0][2] - Matrix->Data[2][0]) / S;
        Quaternion->Z = (Matrix->Data[1][0] - Matrix->Data[0][1]) / S;
    }
    else if ((Matrix->Data[0][0] > Matrix->Data[1][1]) && (Matrix->Data[0][0] > Matrix->Data[2][2]))
    {
        float S = sqrtf(1.0f + Matrix->Data[0][0] - Matrix->Data[1][1] - Matrix->Data[2][2]) * 2.0f;
        Quaternion->W = (Matrix->Data[2][1] - Matrix->Data[1][2]) / S;
        Quaternion->X = 0.25f * S;
        Quaternion->Y = (Matrix->Data[0][1] + Matrix->Data[1][0]) / S;
        Quaternion->Z = (Matrix->Data[0][2] + Matrix->Data[2][0]) / S;
    }
    else if (Matrix->Data[1][1] > Matrix->Data[2][2])
    {
        float S = sqrtf(1.0f + Matrix->Data[1][1] - Matrix->Data[0][0] - Matrix->Data[2][2]) * 2.0f;
        Quaternion->W = (Matrix->Data[0][2] - Matrix->Data[2][0]) / S;
        Quaternion->X = (Matrix->Data[0][1] + Matrix->Data[1][0]) / S;
        Quaternion->Y = 0.25f * S;
        Quaternion->Z = (Matrix->Data[1][2] + Matrix->Data[2][1]) / S;
    }
    else
    {
        float S = sqrtf(1.0f + Matrix->Data[2][2] - Matrix->Data[0][0] - Matrix->Data[1][1]) * 2.0f;
        Quaternion->W = (Matrix->Data[1][0] - Matrix->Data[0][1]) / S;
        Quaternion->X = (Matrix->Data[0][2] + Matrix->Data[2][0]) / S;
        Quaternion->Y = (Matrix->Data[1][2] + Matrix->Data[2][1]) / S;
        Quaternion->Z = 0.25f * S;
    }
}

/**
 * @brief 计算两个四元数的乘积
 *
 * @param A 左侧四元数
 * @param B 右侧四元数
 * @param Result 用于保存A乘B的四元数
 */
void Rotation3D_Quaternion_Multiply(const Quaternionf_t *A,const Quaternionf_t *B,Quaternionf_t *Result)
{
    if ((A == nullptr) || (B == nullptr) || (Result == nullptr))
    {
        return;
    }

    Quaternionf_t Temp;
    Temp.W = A->W * B->W - A->X * B->X - A->Y * B->Y - A->Z * B->Z;
    Temp.X = A->W * B->X + A->X * B->W + A->Y * B->Z - A->Z * B->Y;
    Temp.Y = A->W * B->Y - A->X * B->Z + A->Y * B->W + A->Z * B->X;
    Temp.Z = A->W * B->Z + A->X * B->Y - A->Y * B->X + A->Z * B->W;
    *Result = Temp;
}

/**
 * @brief 将四元数转换为机体系Yaw、Pitch和Roll
 *
 * @param Quaternion 输入四元数
 * @param Yaw 输出Yaw角，单位degree，允许传入nullptr
 * @param Pitch 输出Pitch角，单位degree，允许传入nullptr
 * @param Roll 输出Roll角，单位degree，允许传入nullptr
 */
void Rotation3D_Quaternion_To_Yaw_Pitch_Roll_Degree(const Quaternionf_t *Quaternion,float *Yaw,float *Pitch,float *Roll)
{
    Matrix3f_t Matrix;
    Rotation3D_Quaternion_To_Matrix(Quaternion,&Matrix);

    Vector3f_t Forward_Body = {1.0f,0.0f,0.0f};
    Vector3f_t Up_Body = {0.0f,0.0f,1.0f};
    Vector3f_t Forward_World = Rotation3D_Matrix_Multiply_Vector(&Matrix,&Forward_Body);
    Vector3f_t Up_World = Rotation3D_Matrix_Multiply_Vector(&Matrix,&Up_Body);
    Rotation3D_Forward_Up_To_Yaw_Pitch_Roll_Degree(Forward_World,Up_World,Yaw,Pitch,Roll);
}

/**
 * @brief 将角度限制到(-180,180]范围
 *
 * @param Angle 输入角度，单位degree
 * @return float 包角后的角度，单位degree
 */
float Rotation3D_Wrap_Angle_Degree(float Angle)
{
    while (Angle > 180.0f)
    {
        Angle -= 360.0f;
    }
    while (Angle <= -180.0f)
    {
        Angle += 360.0f;
    }
    return Angle;
}

/**
 * @brief 根据ZXY外旋角和自定义机体前、上方向计算最终姿态角
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
                                                               float *roll_deg)
{
    float Z = ex_z_deg * Rotation3D_Degree_To_Radian;
    float X = ex_x_deg * Rotation3D_Degree_To_Radian;
    float Y = ex_y_deg * Rotation3D_Degree_To_Radian;

    float Sin_X = sinf(X);
    float Cos_X = cosf(X);
    float Sin_Y = sinf(Y);
    float Cos_Y = cosf(Y);
    float Sin_Z = sinf(Z);
    float Cos_Z = cosf(Z);

    //按照固定轴Z、X、Y的顺序建立一次完整旋转矩阵
    Matrix3f_t Matrix;
    Matrix.Data[0][0] = Cos_Y * Cos_Z + Sin_Y * Sin_X * Sin_Z;
    Matrix.Data[0][1] = Sin_Y * Sin_X * Cos_Z - Cos_Y * Sin_Z;
    Matrix.Data[0][2] = Sin_Y * Cos_X;
    Matrix.Data[1][0] = Cos_X * Sin_Z;
    Matrix.Data[1][1] = Cos_X * Cos_Z;
    Matrix.Data[1][2] = -Sin_X;
    Matrix.Data[2][0] = Sin_X * Sin_Z * Cos_Y - Sin_Y * Cos_Z;
    Matrix.Data[2][1] = Sin_X * Cos_Y * Cos_Z + Sin_Y * Sin_Z;
    Matrix.Data[2][2] = Cos_X * Cos_Y;

    Vector3f_t Forward_Body = {f_body_x,f_body_y,f_body_z};
    Vector3f_t Up_Body = {u_body_x,u_body_y,u_body_z};
    Vector3f_t Forward_World = Rotation3D_Matrix_Multiply_Vector(&Matrix,&Forward_Body);
    Vector3f_t Up_World = Rotation3D_Matrix_Multiply_Vector(&Matrix,&Up_Body);

    Rotation3D_Forward_Up_To_Yaw_Pitch_Roll_Degree(Forward_World,Up_World,yaw_deg,pitch_deg,roll_deg);
}
