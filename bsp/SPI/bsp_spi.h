/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_spi.h
  * @brief   SPI BSP
  * @author  yxn
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/*YOUR CODE*/

/**
 * @brief SPI通信完成回调函数
 *
 * @param Tx_Buffer 本次通信使用的发送缓冲区
 * @param Rx_Buffer 本次通信使用的接收缓冲区，只发送时为nullptr
 * @param Tx_Length 调用接口时记录的发送长度
 * @param Rx_Length 调用接口时记录的接收长度
 */
typedef void (*SPI_Callback)(uint8_t *Tx_Buffer,uint8_t *Rx_Buffer,uint16_t Tx_Length,uint16_t Rx_Length);

/**
 * @brief SPI通信错误回调函数
 */
typedef void (*SPI_Error_Callback)(void);

/**
 * @brief SPI通信管理对象
 */
struct Struct_SPI_Manage_Object
{
    SPI_HandleTypeDef *SPI_Handler;
    SPI_Callback Callback_Function;
    SPI_Error_Callback Error_Callback_Function;

    GPIO_TypeDef *Activate_GPIOx;
    uint16_t Activate_GPIO_Pin;
    GPIO_PinState Activate_Level;

    uint8_t *Tx_Buffer;
    uint8_t *Rx_Buffer;
    uint16_t Tx_Buffer_Length;
    uint16_t Rx_Buffer_Length;

    bool Busy;
};

/**
 * @brief 初始化SPI BSP
 *
 * @param hspi SPI句柄
 * @param Callback_Function 通信完成回调函数
 */
void SPI_Init(SPI_HandleTypeDef *hspi,SPI_Callback Callback_Function);

/**
 * @brief 设置SPI通信错误回调函数
 *
 * @param hspi SPI句柄
 * @param Error_Callback_Function 通信错误回调函数
 */
void SPI_Set_Error_Callback(SPI_HandleTypeDef *hspi,SPI_Error_Callback Error_Callback_Function);

/**
 * @brief 获取SPI是否正在执行DMA通信
 *
 * @param hspi SPI句柄
 * @return bool true表示SPI正在通信
 */
bool SPI_Get_Busy_State(SPI_HandleTypeDef *hspi);

/**
 * @brief 使用DMA发送SPI数据
 *
 * @param hspi SPI句柄
 * @param GPIOx 片选GPIO端口，不使用软件片选时可传入nullptr
 * @param GPIO_Pin 片选GPIO引脚
 * @param Activate_Level 片选有效电平
 * @param Tx_Buffer 发送缓冲区
 * @param Tx_Length 发送数据长度
 * @return uint8_t HAL_OK表示DMA成功启动，其他值表示本次启动失败
 */
uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin,GPIO_PinState Activate_Level,uint8_t *Tx_Buffer,uint16_t Tx_Length);

/**
 * @brief 阻塞发送SPI数据
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
uint8_t SPI_Transmit_Data_Blocking(SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin,GPIO_PinState Activate_Level,uint8_t *Tx_Buffer,uint16_t Tx_Length,uint32_t Timeout);

/**
 * @brief 使用DMA同时收发SPI数据
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
 * @note 实际收发长度为Tx_Length与Rx_Length之和，两个长度会原样传给完成回调。
 */
uint8_t SPI_Transmit_Receive_Data(SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin,GPIO_PinState Activate_Level,uint8_t *Tx_Buffer,uint8_t *Rx_Buffer,uint16_t Tx_Length,uint16_t Rx_Length);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SPI_H__ */
