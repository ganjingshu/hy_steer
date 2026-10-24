/**
 * @file bsp_usb.c
 * @author your name (you@domain.com)
 * @brief usb是单例bsp,因此不保存实例
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bsp_usb.h"
#include "bsp_log.h"

static uint8_t *bsp_usb_rx_buffer; // 接收到的数据会被放在这里,buffer size为2048

uint8_t *USBInit(USB_Init_Config_s usb_conf)
{
    // 注册接收/发送完成回调,并把接收缓冲区指针交给上层
    bsp_usb_rx_buffer = CDCGetRxBuffer();
    CDC_RegisterRxCallback(usb_conf.rx_cbk);
    CDC_RegisterTxCallback(usb_conf.tx_cbk);
    LOGINFO("USB init success");
    return bsp_usb_rx_buffer;
}

void USBTransmit(uint8_t *buffer, uint16_t len)
{
    CDC_Transmit_HS(buffer, len); // 发送
}
