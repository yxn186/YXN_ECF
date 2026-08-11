/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bmi088.h
  * @brief   BMI088传感器驱动
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BMI088_H__
#define __BMI088_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bmi088reg.h"
#include "bsp_spi.h"
/*YOUR CODE*/

/**
 * @brief BMI088驱动运行状态
 */
enum class BMI088_States_e : uint8_t
{
    Uninitialized = 0,
    Initializing,
    Ready,
    Offline,
    Error
};

/**
 * @brief BMI088初始化参数
 */
typedef struct
{
    SPI_HandleTypeDef *SPI_Handler = nullptr;                    //BMI088使用的SPI句柄

    GPIO_TypeDef *ACC_CS_GPIOx = nullptr;                        //加速度计片选端口
    uint16_t ACC_CS_Pin = 0;                                    //加速度计片选引脚
    GPIO_PinState ACC_CS_Active_Level = GPIO_PIN_RESET;          //加速度计片选有效电平

    GPIO_TypeDef *GYRO_CS_GPIOx = nullptr;                       //陀螺仪片选端口
    uint16_t GYRO_CS_Pin = 0;                                   //陀螺仪片选引脚
    GPIO_PinState GYRO_CS_Active_Level = GPIO_PIN_RESET;         //陀螺仪片选有效电平

    uint16_t ACC_INT_Pin = 0;                                   //加速度数据就绪中断引脚
    uint16_t GYRO_INT_Pin = 0;                                  //陀螺仪数据就绪中断引脚

    uint8_t ACC_Config_Value = BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set; //加速度计采样配置寄存器值
    uint8_t ACC_Range_Value = BMI088_ACC_RANGE_6G;               //加速度计量程寄存器值
    float ACC_Range_G = 6.0f;                                   //加速度计换算量程，单位g

    uint8_t GYRO_Bandwidth_Value = BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set; //陀螺仪带宽和采样率寄存器值
    uint8_t GYRO_Range_Value = BMI088_GYRO_2000;                 //陀螺仪量程寄存器值
    float GYRO_Range_DPS = 2000.0f;                             //陀螺仪换算量程，单位degree/s

    uint32_t Feedback_Timeout_ms = 100;                          //完整数据反馈超时时间，单位ms
} BMI088_Config_t;

/**
 * @brief BMI088传感器类
 *
 * 本类只负责传感器初始化、SPI采样和单位换算，不负责姿态解算。
 */
class Class_BMI088
{
public:
    /**
     * @brief 初始化BMI088驱动
     *
     * @param Config SPI、GPIO、量程和超时等初始化参数
     * @return bool true表示参数有效并开始初始化
     */
    bool Init(const BMI088_Config_t &Config);

    /**
     * @brief 推进BMI088初始化、采样和在线检测
     *
     * @param Now_ms 当前系统时间，单位ms
     */
    void Update(uint32_t Now_ms);

    /**
     * @brief 处理BMI088数据就绪GPIO中断
     *
     * @param GPIO_Pin 触发中断的GPIO引脚
     */
    void Process_GPIO_EXTI(uint16_t GPIO_Pin);

    /**
     * @brief 获取是否产生了一组新的ACC与GYRO完整数据
     *
     * @return bool true表示上层还没有处理本次完整数据
     */
    bool Get_New_Sample_Flag();

    /**
     * @brief 清除完整数据更新标志
     */
    void Clear_New_Sample_Flag();

    /**
     * @brief 获取BMI088驱动当前运行状态
     *
     * @return BMI088_States_e 当前运行状态
     */
    inline BMI088_States_e Get_States() { return States; }

    /**
     * @brief 获取BMI088初始化是否已经完成
     *
     * @return bool true表示芯片ID和全部配置寄存器已经处理完成
     */
    inline bool Get_Init_Finished() { return Init_Finished; }

    /**
     * @brief 获取BMI088当前在线状态
     *
     * @return bool true表示完整数据没有超过反馈超时时间
     */
    inline bool Get_Online_State() { return Online; }

    /**
     * @brief 获取最近一组完整数据的接收时间
     *
     * @return uint32_t 最近接收时间，单位ms
     */
    inline uint32_t Get_Last_Sample_Time() { return Last_Sample_Time; }

    /**
     * @brief 获取加速度计X轴原始值
     *
     * @return int16_t X轴寄存器原始值
     */
    inline int16_t Get_ACC_Raw_X() { return ACC_Raw_X; }

    /**
     * @brief 获取加速度计Y轴原始值
     *
     * @return int16_t Y轴寄存器原始值
     */
    inline int16_t Get_ACC_Raw_Y() { return ACC_Raw_Y; }

    /**
     * @brief 获取加速度计Z轴原始值
     *
     * @return int16_t Z轴寄存器原始值
     */
    inline int16_t Get_ACC_Raw_Z() { return ACC_Raw_Z; }

    /**
     * @brief 获取陀螺仪X轴原始值
     *
     * @return int16_t X轴寄存器原始值
     */
    inline int16_t Get_GYRO_Raw_X() { return GYRO_Raw_X; }

    /**
     * @brief 获取陀螺仪Y轴原始值
     *
     * @return int16_t Y轴寄存器原始值
     */
    inline int16_t Get_GYRO_Raw_Y() { return GYRO_Raw_Y; }

    /**
     * @brief 获取陀螺仪Z轴原始值
     *
     * @return int16_t Z轴寄存器原始值
     */
    inline int16_t Get_GYRO_Raw_Z() { return GYRO_Raw_Z; }

    /**
     * @brief 获取加速度计X轴换算值
     *
     * @return float X轴加速度，单位g
     */
    inline float Get_ACC_X_G() { return ACC_X_G; }

    /**
     * @brief 获取加速度计Y轴换算值
     *
     * @return float Y轴加速度，单位g
     */
    inline float Get_ACC_Y_G() { return ACC_Y_G; }

    /**
     * @brief 获取加速度计Z轴换算值
     *
     * @return float Z轴加速度，单位g
     */
    inline float Get_ACC_Z_G() { return ACC_Z_G; }

    /**
     * @brief 获取陀螺仪X轴换算值
     *
     * @return float X轴角速度，单位degree/s
     */
    inline float Get_GYRO_X_DPS() { return GYRO_X_DPS; }

    /**
     * @brief 获取陀螺仪Y轴换算值
     *
     * @return float Y轴角速度，单位degree/s
     */
    inline float Get_GYRO_Y_DPS() { return GYRO_Y_DPS; }

    /**
     * @brief 获取陀螺仪Z轴换算值
     *
     * @return float Z轴角速度，单位degree/s
     */
    inline float Get_GYRO_Z_DPS() { return GYRO_Z_DPS; }

private:
    /**
     * @brief BMI088非阻塞初始化步骤
     */
    enum class Init_State_e : uint8_t
    {
        Start_ACC_Reset = 0,
        Wait_ACC_Reset,
        Wait_ACC_Reset_Delay,
        Start_GYRO_Reset,
        Wait_GYRO_Reset,
        Wait_GYRO_Reset_Delay,
        Start_ACC_Dummy_Read,
        Wait_ACC_Dummy_Read,
        Start_ACC_ID_Read,
        Wait_ACC_ID_Read,
        Start_GYRO_ID_Read,
        Wait_GYRO_ID_Read,
        Check_Chip_ID,
        Start_Config_Register,
        Wait_Config_Register,
        Wait_Config_Delay,
        Finished,
        Error
    };

    /**
     * @brief 当前SPI DMA传输内容
     */
    enum class Transfer_Type_e : uint8_t
    {
        None = 0,
        Write_Register,
        Read_ACC_ID,
        Read_GYRO_ID,
        Read_ACC_Data,
        Read_GYRO_Data
    };

    BMI088_Config_t Config;                                      //初始化时保存的硬件和采样配置
    BMI088_States_e States = BMI088_States_e::Uninitialized;     //驱动对上层公开的运行状态
    Init_State_e Init_State = Init_State_e::Start_ACC_Reset;     //当前非阻塞初始化步骤
    volatile Transfer_Type_e Transfer_Type = Transfer_Type_e::None; //当前DMA传输内容

    bool Init_Finished = false;                                  //全部配置寄存器是否已经写完
    bool Online = false;                                         //完整数据是否在超时时间内更新
    bool New_Sample = false;                                     //上层是否还有新数据未处理
    bool ACC_Sample_Valid = false;                               //是否已经读取了本组ACC数据

    volatile bool Transfer_Finished = false;                     //SPI完成回调写入，Task读取
    volatile bool Transfer_Error = false;                        //SPI错误回调写入，Task读取
    volatile bool ACC_Data_Ready = false;                        //ACC外部中断留下的采样请求
    volatile bool GYRO_Data_Ready = false;                       //GYRO外部中断留下的采样请求
    volatile bool Raw_Sample_Ready = false;                      //六轴原始值已经组合完成

    uint8_t Tx_Buffer[16] = {0};                                 //寄存器读写发送缓冲区
    uint8_t Rx_Buffer[16] = {0};                                 //寄存器读取接收缓冲区

    uint8_t ACC_Chip_ID = 0;
    uint8_t GYRO_Chip_ID = 0;
    uint8_t Config_Register_Index = 0;
    uint32_t Config_Register_Delay_ms = 0;

    uint32_t State_Start_Time = 0;                               //当前等待步骤开始时间
    uint32_t Init_Finished_Time = 0;                             //驱动初始化完成时间
    uint32_t Last_Sample_Time = 0;                               //最近完整六轴数据时间

    int16_t ACC_Raw_X = 0;
    int16_t ACC_Raw_Y = 0;
    int16_t ACC_Raw_Z = 0;
    int16_t GYRO_Raw_X = 0;
    int16_t GYRO_Raw_Y = 0;
    int16_t GYRO_Raw_Z = 0;

    float ACC_X_G = 0.0f;
    float ACC_Y_G = 0.0f;
    float ACC_Z_G = 0.0f;
    float GYRO_X_DPS = 0.0f;
    float GYRO_Y_DPS = 0.0f;
    float GYRO_Z_DPS = 0.0f;

    static Class_BMI088 *BMI088_Instance;

    /**
     * @brief 将SPI BSP完成回调转发到当前BMI088对象
     *
     * @param Tx_Buffer 本次通信使用的发送缓冲区
     * @param Rx_Buffer 本次通信使用的接收缓冲区
     * @param Tx_Length 协议发送段长度
     * @param Rx_Length 协议接收段长度
     */
    static void SPI_Complete_Callback(uint8_t *Tx_Buffer,uint8_t *Rx_Buffer,uint16_t Tx_Length,uint16_t Rx_Length);

    /**
     * @brief 将SPI BSP错误回调转发到当前BMI088对象
     */
    static void SPI_Error_Callback();

    /**
     * @brief 按当前初始化步骤推进软复位、ID检查和寄存器配置
     *
     * @param Now_ms 当前系统时间，单位ms
     */
    void Init_Process(uint32_t Now_ms);

    /**
     * @brief 处理数据就绪请求、单位换算和在线检测
     *
     * @param Now_ms 当前系统时间，单位ms
     */
    void Data_Process(uint32_t Now_ms);

    /**
     * @brief 处理一次SPI DMA完成后的寄存器数据
     *
     * @param Tx_Buffer 本次通信使用的发送缓冲区
     * @param Rx_Buffer 本次通信使用的接收缓冲区
     * @param Tx_Length 协议发送段长度
     * @param Rx_Length 协议接收段长度
     */
    void Process_SPI_Complete(uint8_t *Tx_Buffer,uint8_t *Rx_Buffer,uint16_t Tx_Length,uint16_t Rx_Length);

    /**
     * @brief 记录SPI DMA错误，交给Update统一进入错误状态
     */
    void Process_SPI_Error();

    /**
     * @brief 启动一次加速度计寄存器DMA读取
     *
     * @param Register 起始寄存器地址
     * @param Length 需要读取的真实数据长度
     * @param New_Transfer_Type 本次传输用途
     * @return bool true表示DMA成功启动
     */
    bool Start_ACC_Read(uint8_t Register,uint8_t Length,Transfer_Type_e New_Transfer_Type);

    /**
     * @brief 启动一次陀螺仪寄存器DMA读取
     *
     * @param Register 起始寄存器地址
     * @param Length 需要读取的真实数据长度
     * @param New_Transfer_Type 本次传输用途
     * @return bool true表示DMA成功启动
     */
    bool Start_GYRO_Read(uint8_t Register,uint8_t Length,Transfer_Type_e New_Transfer_Type);

    /**
     * @brief 启动一次加速度计单寄存器DMA写入
     *
     * @param Register 目标寄存器地址
     * @param Data 需要写入的一个字节
     * @return bool true表示DMA成功启动
     */
    bool Start_ACC_Write(uint8_t Register,uint8_t Data);

    /**
     * @brief 启动一次陀螺仪单寄存器DMA写入
     *
     * @param Register 目标寄存器地址
     * @param Data 需要写入的一个字节
     * @return bool true表示DMA成功启动
     */
    bool Start_GYRO_Write(uint8_t Register,uint8_t Data);

    /**
     * @brief 按配置索引启动下一项BMI088寄存器写入
     *
     * @return bool true表示对应寄存器写入已经启动
     */
    bool Start_Config_Register();

    /**
     * @brief 读取并清除一次SPI传输完成标志
     *
     * @return bool true表示刚刚完成了一次SPI传输
     */
    bool Take_Transfer_Finished();

    /**
     * @brief 根据配置量程把六轴原始值换算为g和degree/s
     */
    void Update_Scaled_Data();

    /**
     * @brief 停止驱动流程并进入不可继续运行的错误状态
     */
    void Set_Error_State();
};

#endif /* __BMI088_H__ */
