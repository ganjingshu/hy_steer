/**
 * @file bsp_usb.h
 * @author your name (you@domain.com)
 * @brief 提供对usb vcp(virtual com port)的操作接口,hid和msf考虑后续添加
 * @attention 这一版在usbd_cdc_if.c中添加了接收/发送完成回调(CubeMX生成后会被覆盖),后续需要由模板重新生成
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include "usbd_cdc_if.h"

typedef struct
{
    USBCallback tx_cbk;
    USBCallback rx_cbk;
} USB_Init_Config_s;

/* @note 虚拟串口的波特率/校验位/数据位等动态可变,取决于上位机的设定 */
/* 使用时不需要关心这些设置(作为从机) */

uint8_t *USBInit(USB_Init_Config_s usb_conf); // bsp初始化时调用,注册回调并返回接收缓冲区指针

void USBTransmit(uint8_t *buffer, uint16_t len); // 通过usb发送数据
