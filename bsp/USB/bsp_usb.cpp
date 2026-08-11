/**
 * @file bsp_usb.cpp
 * @brief USB CDC双缓冲收发接口
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_usb.h"
#include <stdarg.h>
#include <stdio.h>

Struct_USB_Manage_Object USB0_Manage_Object = {nullptr};

//USB HS设备句柄和初始接收缓冲区由CubeMX生成
extern USBD_HandleTypeDef hUsbDeviceHS;
extern uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/**
 * @brief USB虚拟串口回调函数
 *
 * @param Buffer 接收缓冲区
 * @param Length 接收数据长度
 */
__weak void USB_Serial_Callback(uint8_t *Buffer, uint16_t Length)
{
    USB_Transmit_Data(Buffer,Length);
}

/**
 * @brief 初始化USB接收缓冲区和数据回调
 *
 * @param Callback_Function USB数据处理回调函数
 */
void USB_Init(USB_Callback Callback_Function)
{
    USB0_Manage_Object.Callback_Function = Callback_Function;
    USB0_Manage_Object.Rx_Buffer_Active = UserRxBufferHS;
    USB0_Manage_Object.Rx_Buffer_Ready = nullptr;
}

/**
 * @brief 通过USB CDC发送数据
 *
 * @param Data 发送数据缓冲区
 * @param Length 发送数据长度
 * @return uint8_t USB设备栈返回状态
 */
uint8_t USB_Transmit_Data(uint8_t *Data,uint16_t Length)
{
    if ((Data == nullptr) || (Length == 0))
    {
        return USBD_FAIL;
    }

    return CDC_Transmit_HS(Data,Length);
}

/**
 * @brief 完成本次USB接收并重新挂载下一块缓冲区
 *
 * @param Size 接收数据长度
 */
void USB_ReceiveCallback(uint16_t Size)
{
    if (USB0_Manage_Object.Rx_Buffer_Active == nullptr)
    {
        USB0_Manage_Object.Rx_Buffer_Active = UserRxBufferHS;
    }

    //初始化完成前只重新挂载接收端点
    if (!Global_Init_Finished)
    {
        USBD_CDC_SetRxBuffer(&hUsbDeviceHS,USB0_Manage_Object.Rx_Buffer_Active);
        USBD_CDC_ReceivePacket(&hUsbDeviceHS);
        return;
    }

    //切换接收缓冲区后立即重新挂载OUT端点
    USB0_Manage_Object.Rx_Buffer_Ready = USB0_Manage_Object.Rx_Buffer_Active;
    if (USB0_Manage_Object.Rx_Buffer_Active == USB0_Manage_Object.Rx_Buffer_0)
    {
        USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_1;
    }
    else
    {
        USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_0;
    }

    USBD_CDC_SetRxBuffer(&hUsbDeviceHS,USB0_Manage_Object.Rx_Buffer_Active);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);

    if (Size > USB_BUFFER_SIZE)
    {
        Size = USB_BUFFER_SIZE;
    }

    if (USB0_Manage_Object.Callback_Function != nullptr)
    {
        USB0_Manage_Object.Callback_Function(USB0_Manage_Object.Rx_Buffer_Ready,Size);
    }
}

/**
 * @brief 通过USB CDC发送格式化文本
 *
 * @param Format printf格式字符串
 * @param ... 格式化参数
 */
void USB_Printf(const char *Format,...)
{
    char printf_temp_buffer[256];

    va_list argument_list;
    va_start(argument_list,Format);
    int length = vsnprintf(printf_temp_buffer,sizeof(printf_temp_buffer),Format,argument_list);
    va_end(argument_list);

    if (length <= 0)
    {
        return;
    }

    if (length >= (int)sizeof(printf_temp_buffer))
    {
        length = (int)sizeof(printf_temp_buffer) - 1;
    }

    (void)USB_Transmit_Data((uint8_t *)printf_temp_buffer, (uint16_t)length);
}

