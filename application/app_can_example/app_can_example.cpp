/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_can_example.cpp
  * @brief   app_can_example
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "app_can_example.h"

extern Class_DM4310_Motor DM4310_Motor;

//-----------------------bxCAN案例----------------------------

/**
 * @brief CAN1 FIFO0接收数据统一分发函数
 *
 * @param Rx_Buffer CAN接收缓冲区
 */
void CAN1_Rx_Dispatcher(CAN_Rx_Buffer_t *Rx_Buffer)
{
    if (Rx_Buffer == nullptr)
    {
        return;
    }

    /*
     * bxCAN的DLC直接表示有效字节数量，范围0~8。
     */
    const uint16_t CAN_ID =
        static_cast<uint16_t>(Rx_Buffer->Header.StdId);

    const uint8_t Data_Length =
        static_cast<uint8_t>(Rx_Buffer->Header.DLC);

    /*
     * 交给DM4310电机对象判断并解析。
     */
    DM4310_Motor.Process_CAN_Feedback(
        CAN_ID,
        Rx_Buffer->Data,
        Data_Length
    );

    /*
     * 后续有其他CAN模块时，也在这里分发。
     */
    // DJI_Motor_Group.Process_Rx_Data(
    //     CAN_ID,
    //     Rx_Buffer->Data,
    //     Data_Length
    // );
}

Class_BxCAN_Adapter BxCAN1_Adapter;
Class_DM4310_Motor DM4310_Motor;

void CAN_Init(void)
{

    //DM电机初始化示例

    //1----初始化适配层----
    BxCAN1_Adapter.Init(&hcan1);

    //2----初始化电机----
    DM4310_Motor.Init(
        &BxCAN1_Adapter,
        0x01,                            // Motor ID
        0x000,                           // Master ID
        DM_Control_Mode_e::Velocity,
        12.5f,                           // 临时PMAX
        30.0f,                           // 临时VMAX
        12.5f                            // 临时TMAX
    );

    //3----bxcan初始化流程----
    //----注册统一回调函数----
    CAN_Register_RxCallBack_FIFO0_Function(
        CAN1_Rx_Dispatcher
    );

    //----bxcan滤波器配置----
    /*
     * 暂时配置FIFO0全通滤波器。
     *
     * 过滤器编号：
     * CAN1通常使用0~13；
     * CAN2通常使用14~27。
     */
    CAN_Filter_Mask_Config(
        &hcan1,
        CAN_FILTER(0) |
        CAN_FIFO_0 |
        CAN_STDID |
        CAN_DATA_TYPE,
        0x000,
        0x000
    );

    //初始化bxcan
    CAN_Init(&hcan1);
}

//----------------------FDCAN案例----------------------------
#include "Motor_DM4310.h"
#include "bsp_fdcan.h"

extern Class_DM4310_Motor DM4310_Motor;

/**
 * @brief FDCAN1接收数据统一分发函数
 *
 * @param Header FDCAN接收报文头
 * @param Buffer 接收数据
 */
void FDCAN1_Rx_Dispatcher(FDCAN_RxHeaderTypeDef &Header,uint8_t *Buffer)
{
    if (Buffer == nullptr)
    {
        return;
    }

    /*
     * 当前库只处理标准数据帧。
     */
    if (Header.IdType != FDCAN_STANDARD_ID)
    {
        return;
    }

    if (Header.RxFrameType != FDCAN_DATA_FRAME)
    {
        return;
    }

    //保存CANID 数据长度
    const uint16_t CAN_ID =static_cast<uint16_t>(Header.Identifier);

    const uint8_t Data_Length = FDCAN_Convert_DLC_To_Length(Header.DataLength);

    //放入DM电机接收回调（分发进）
    DM4310_Motor.Process_CAN_Feedback(CAN_ID,Buffer,Data_Length);

    /*
     * 后续其他模块也统一放到这里分发。
     */
}


//FDCAN初始化
Class_FDCAN_Adapter FDCAN1_Adapter;
Class_DM4310_Motor DM4310_Motor;

void CAN_Init(void)
{
    /*
     * 发送适配器绑定FDCAN句柄。
     */
    FDCAN1_Adapter.Init(&hfdcan1);

    /*
     * 初始化DM4310对象。
     */
    DM4310_Motor.Init(&FDCAN1_Adapter,0x01,0x000,DM_Control_Mode_e::Velocity,12.5f,30.0f,12.5f);

    /*
     * 底层初始化CAN，同时注册接收分发函数。
     */
    CAN_Init(&hfdcan1,FDCAN1_Rx_Dispatcher);
}