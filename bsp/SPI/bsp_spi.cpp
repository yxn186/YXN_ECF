/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_spi.cpp
  * @brief   SPI BSP
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_spi.h"
/*YOUR CODE*/

Struct_SPI_Manage_Object SPI1_Manage_Object = {};
Struct_SPI_Manage_Object SPI2_Manage_Object = {};
Struct_SPI_Manage_Object SPI3_Manage_Object = {};

/**
 * @brief 根据HAL句柄获取SPI管理对象
 *
 * @param hspi SPI句柄
 * @return Struct_SPI_Manage_Object* 对应管理对象，句柄无效时返回nullptr
 */
static Struct_SPI_Manage_Object *SPI_Get_Manage_Object(SPI_HandleTypeDef *hspi)
{
    if (hspi == nullptr)
    {
        return nullptr;
    }

    if (hspi->Instance == SPI1)
    {
        return &SPI1_Manage_Object;
    }
    if (hspi->Instance == SPI2)
    {
        return &SPI2_Manage_Object;
    }
    if (hspi->Instance == SPI3)
    {
        return &SPI3_Manage_Object;
    }

    return nullptr;
}

/**
 * @brief 释放当前SPI通信使用的片选
 *
 * @param Manage_Object 当前SPI管理对象
 */
static void SPI_Release_Chip_Select(Struct_SPI_Manage_Object *Manage_Object)
{
    if ((Manage_Object == nullptr) || (Manage_Object->Activate_GPIOx == nullptr))
    {
        return;
    }

    GPIO_PinState Inactive_Level = GPIO_PIN_SET;
    if (Manage_Object->Activate_Level == GPIO_PIN_SET)
    {
        Inactive_Level = GPIO_PIN_RESET;
    }

    HAL_GPIO_WritePin(Manage_Object->Activate_GPIOx,Manage_Object->Activate_GPIO_Pin,Inactive_Level);
}

/**
 * @brief 清除一次DMA通信的临时信息
 *
 * @param Manage_Object 当前SPI管理对象
 */
static void SPI_Clear_Transfer(Struct_SPI_Manage_Object *Manage_Object)
{
    if (Manage_Object == nullptr)
    {
        return;
    }

    Manage_Object->Activate_GPIOx = nullptr;
    Manage_Object->Activate_GPIO_Pin = 0;
    Manage_Object->Tx_Buffer = nullptr;
    Manage_Object->Rx_Buffer = nullptr;
    Manage_Object->Tx_Buffer_Length = 0;
    Manage_Object->Rx_Buffer_Length = 0;
    Manage_Object->Busy = false;
}

/**
 * @brief 初始化SPI BSP并保存通信完成回调
 *
 * @param hspi SPI句柄
 * @param Callback_Function 通信完成回调函数
 */
void SPI_Init(SPI_HandleTypeDef *hspi,SPI_Callback Callback_Function)
{
    Struct_SPI_Manage_Object *Manage_Object = SPI_Get_Manage_Object(hspi);
    if (Manage_Object == nullptr)
    {
        return;
    }

    Manage_Object->SPI_Handler = hspi;
    Manage_Object->Callback_Function = Callback_Function;
    SPI_Clear_Transfer(Manage_Object);
}

/**
 * @brief 设置SPI DMA错误回调函数
 *
 * @param hspi SPI句柄
 * @param Error_Callback_Function 通信错误回调函数
 */
void SPI_Set_Error_Callback(SPI_HandleTypeDef *hspi,SPI_Error_Callback Error_Callback_Function)
{
    Struct_SPI_Manage_Object *Manage_Object = SPI_Get_Manage_Object(hspi);
    if (Manage_Object == nullptr)
    {
        return;
    }

    Manage_Object->Error_Callback_Function = Error_Callback_Function;
}

/**
 * @brief 获取SPI是否正在执行DMA通信
 *
 * @param hspi SPI句柄
 * @return bool true表示SPI BSP仍占用本次DMA通信
 */
bool SPI_Get_Busy_State(SPI_HandleTypeDef *hspi)
{
    Struct_SPI_Manage_Object *Manage_Object = SPI_Get_Manage_Object(hspi);
    if (Manage_Object == nullptr)
    {
        return false;
    }

    return Manage_Object->Busy;
}

/**
 * @brief 拉低片选并使用DMA发送SPI数据
 *
 * @param hspi SPI句柄
 * @param GPIOx 片选GPIO端口，不使用软件片选时可传入nullptr
 * @param GPIO_Pin 片选GPIO引脚
 * @param Activate_Level 片选有效电平
 * @param Tx_Buffer 发送缓冲区
 * @param Tx_Length 发送数据长度
 * @return uint8_t HAL_OK表示DMA成功启动，其他值表示本次启动失败
 */
uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin,GPIO_PinState Activate_Level,uint8_t *Tx_Buffer,uint16_t Tx_Length)
{
    Struct_SPI_Manage_Object *Manage_Object = SPI_Get_Manage_Object(hspi);
    if ((Manage_Object == nullptr) || (Tx_Buffer == nullptr) || (Tx_Length == 0))
    {
        return HAL_ERROR;
    }
    if (Manage_Object->Busy || (hspi->State != HAL_SPI_STATE_READY))
    {
        return HAL_BUSY;
    }

    //DMA期间保存本次通信上下文，完成回调和错误回调都需要用它释放片选
    Manage_Object->Activate_GPIOx = GPIOx;
    Manage_Object->Activate_GPIO_Pin = GPIO_Pin;
    Manage_Object->Activate_Level = Activate_Level;
    Manage_Object->Tx_Buffer = Tx_Buffer;
    Manage_Object->Rx_Buffer = nullptr;
    Manage_Object->Tx_Buffer_Length = Tx_Length;
    Manage_Object->Rx_Buffer_Length = 0;
    Manage_Object->Busy = true;

    if (GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(GPIOx,GPIO_Pin,Activate_Level);
    }

    HAL_StatusTypeDef HAL_States = HAL_SPI_Transmit_DMA(hspi,Tx_Buffer,Tx_Length);
    if (HAL_States != HAL_OK)
    {
        SPI_Release_Chip_Select(Manage_Object);
        SPI_Clear_Transfer(Manage_Object);
    }

    return HAL_States;
}

/**
 * @brief 拉低片选并阻塞发送SPI数据
 *
 * @param hspi SPI句柄
 * @param GPIOx 片选GPIO端口，不使用软件片选时可传入nullptr
 * @param GPIO_Pin 片选GPIO引脚
 * @param Activate_Level 片选有效电平
 * @param Tx_Buffer 发送缓冲区
 * @param Tx_Length 发送数据长度
 * @param Timeout 阻塞等待超时时间，单位ms
 * @return uint8_t HAL_SPI_Transmit返回的HAL状态
 */
uint8_t SPI_Transmit_Data_Blocking(SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin,GPIO_PinState Activate_Level,uint8_t *Tx_Buffer,uint16_t Tx_Length,uint32_t Timeout)
{
    Struct_SPI_Manage_Object *Manage_Object = SPI_Get_Manage_Object(hspi);
    if ((Manage_Object == nullptr) || (Tx_Buffer == nullptr) || (Tx_Length == 0))
    {
        return HAL_ERROR;
    }
    if (Manage_Object->Busy || (hspi->State != HAL_SPI_STATE_READY))
    {
        return HAL_BUSY;
    }

    if (GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(GPIOx,GPIO_Pin,Activate_Level);
    }

    HAL_StatusTypeDef HAL_States = HAL_SPI_Transmit(hspi,Tx_Buffer,Tx_Length,Timeout);

    if (GPIOx != nullptr)
    {
        GPIO_PinState Inactive_Level = GPIO_PIN_SET;
        if (Activate_Level == GPIO_PIN_SET)
        {
            Inactive_Level = GPIO_PIN_RESET;
        }
        HAL_GPIO_WritePin(GPIOx,GPIO_Pin,Inactive_Level);
    }

    return HAL_States;
}

/**
 * @brief 拉低片选并使用DMA完成一次SPI全双工通信
 *
 * @param hspi SPI句柄
 * @param GPIOx 片选GPIO端口，不使用软件片选时可传入nullptr
 * @param GPIO_Pin 片选GPIO引脚
 * @param Activate_Level 片选有效电平
 * @param Tx_Buffer 发送缓冲区，长度必须能容纳Tx_Length与Rx_Length之和
 * @param Rx_Buffer 接收缓冲区，长度必须能容纳Tx_Length与Rx_Length之和
 * @param Tx_Length 协议中的发送段长度
 * @param Rx_Length 协议中的接收段长度
 * @return uint8_t HAL_OK表示DMA成功启动，其他值表示本次启动失败
 */
uint8_t SPI_Transmit_Receive_Data(SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin,GPIO_PinState Activate_Level,uint8_t *Tx_Buffer,uint8_t *Rx_Buffer,uint16_t Tx_Length,uint16_t Rx_Length)
{
    Struct_SPI_Manage_Object *Manage_Object = SPI_Get_Manage_Object(hspi);
    if ((Manage_Object == nullptr) || (Tx_Buffer == nullptr) || (Rx_Buffer == nullptr))
    {
        return HAL_ERROR;
    }
    if ((Tx_Length == 0) || (Rx_Length == 0))
    {
        return HAL_ERROR;
    }
    if (Manage_Object->Busy || (hspi->State != HAL_SPI_STATE_READY))
    {
        return HAL_BUSY;
    }

    //Tx_Length和Rx_Length是上层协议分段，HAL实际收发两段长度之和
    Manage_Object->Activate_GPIOx = GPIOx;
    Manage_Object->Activate_GPIO_Pin = GPIO_Pin;
    Manage_Object->Activate_Level = Activate_Level;
    Manage_Object->Tx_Buffer = Tx_Buffer;
    Manage_Object->Rx_Buffer = Rx_Buffer;
    Manage_Object->Tx_Buffer_Length = Tx_Length;
    Manage_Object->Rx_Buffer_Length = Rx_Length;
    Manage_Object->Busy = true;

    if (GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(GPIOx,GPIO_Pin,Activate_Level);
    }

    uint16_t Total_Length = Tx_Length + Rx_Length;
    HAL_StatusTypeDef HAL_States = HAL_SPI_TransmitReceive_DMA(hspi,Tx_Buffer,Rx_Buffer,Total_Length);
    if (HAL_States != HAL_OK)
    {
        SPI_Release_Chip_Select(Manage_Object);
        SPI_Clear_Transfer(Manage_Object);
    }

    return HAL_States;
}

/**
 * @brief 统一处理SPI DMA完成事件
 *
 * @param hspi 产生完成事件的SPI句柄
 */
static void SPI_DMA_Complete(SPI_HandleTypeDef *hspi)
{
    Struct_SPI_Manage_Object *Manage_Object = SPI_Get_Manage_Object(hspi);
    if (Manage_Object == nullptr)
    {
        return;
    }

    SPI_Callback Callback_Function = Manage_Object->Callback_Function;
    uint8_t *Tx_Buffer = Manage_Object->Tx_Buffer;
    uint8_t *Rx_Buffer = Manage_Object->Rx_Buffer;
    uint16_t Tx_Length = Manage_Object->Tx_Buffer_Length;
    uint16_t Rx_Length = Manage_Object->Rx_Buffer_Length;

    //先释放片选并清除Busy，再允许上层回调启动下一次DMA
    SPI_Release_Chip_Select(Manage_Object);
    SPI_Clear_Transfer(Manage_Object);

    if (Callback_Function != nullptr)
    {
        Callback_Function(Tx_Buffer,Rx_Buffer,Tx_Length,Rx_Length);
    }
}

/**
 * @brief 处理HAL SPI只发送DMA完成回调
 *
 * @param hspi 产生完成事件的SPI句柄
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_DMA_Complete(hspi);
}

/**
 * @brief 处理HAL SPI全双工DMA完成回调
 *
 * @param hspi 产生完成事件的SPI句柄
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_DMA_Complete(hspi);
}

/**
 * @brief 处理HAL SPI错误并保证片选得到释放
 *
 * @param hspi 产生错误事件的SPI句柄
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    Struct_SPI_Manage_Object *Manage_Object = SPI_Get_Manage_Object(hspi);
    if (Manage_Object == nullptr)
    {
        return;
    }

    SPI_Error_Callback Error_Callback_Function = Manage_Object->Error_Callback_Function;

    //错误路径同样必须释放片选，否则传感器会一直占用总线
    SPI_Release_Chip_Select(Manage_Object);
    SPI_Clear_Transfer(Manage_Object);

    if (Error_Callback_Function != nullptr)
    {
        Error_Callback_Function();
    }
}
