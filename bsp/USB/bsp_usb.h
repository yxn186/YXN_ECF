/**
 * @file bsp_usb.h
 * @brief USB CDC双缓冲收发接口
 */

#ifndef DRV_USB_H
#define DRV_USB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "usbd_cdc_if.h"
#include <string.h>
#include "stdbool.h"
/* Exported macros -----------------------------------------------------------*/

//USB CDC单次接收缓冲区长度
#define USB_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/

/**
 * @brief USB接收回调函数类型
 */
typedef void (*USB_Callback)(uint8_t *Buffer, uint16_t Length);

/**
 * @brief USB通信处理结构体
 */
struct Struct_USB_Manage_Object
{
    USB_Callback Callback_Function;

    //双缓冲接收区
    uint8_t Rx_Buffer_0[USB_BUFFER_SIZE];
    uint8_t Rx_Buffer_1[USB_BUFFER_SIZE];
    uint8_t *Rx_Buffer_Active;
    uint8_t *Rx_Buffer_Ready;

    //最近一次接收时间戳
    uint64_t Rx_Time_Stamp;
};

/* Exported variables --------------------------------------------------------*/

extern bool Global_Init_Finished;

extern struct Struct_USB_Manage_Object USB0_Manage_Object;

/* Exported function declarations --------------------------------------------*/
/**
 * @brief 默认USB接收回调
 *
 * @param Buffer USB接收缓冲区
 * @param Length 本次接收数据长度
 */
void USB_Serial_Callback(uint8_t *Buffer,uint16_t Length);

/**
 * @brief 初始化USB接收缓冲区和数据回调
 *
 * @param Callback_Function USB数据处理回调函数
 */
void USB_Init(USB_Callback Callback_Function);

/**
 * @brief 通过USB CDC发送数据
 *
 * @param Data 发送数据缓冲区
 * @param Length 发送数据长度
 * @return uint8_t USB设备栈返回状态
 */
uint8_t USB_Transmit_Data(uint8_t *Data,uint16_t Length);

/**
 * @brief 通过USB CDC发送格式化文本
 *
 * @param Format printf格式字符串
 * @param ... 格式化参数
 */
void USB_Printf(const char *Format,...);

/**
 * @brief 完成本次USB接收并重新挂载下一块缓冲区
 *
 * @param Size 本次接收数据长度
 */
void USB_ReceiveCallback(uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif

