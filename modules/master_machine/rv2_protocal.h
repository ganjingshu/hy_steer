/**
 * @file rv2_protocal.h
 * @brief 同济 RV2 视觉通信协议帧收发
 * @note  帧结构直接使用框架 master_process.h 中的 Vision_Send_s / Vision_Recv_s,
 *        不再单独定义线格式结构体,避免布局重复定义
 */

#ifndef RV2_PROTOCAL_H
#define RV2_PROTOCAL_H

#include "master_process.h"

#define RV2_PROTOCAL_HEADER 0x5053u // 帧头 "SP"

void RV2_GimbalToSendData(Vision_Send_s *send, uint8_t *tx_buf, uint16_t *tx_buf_len);
void RV2_GimbalToRecvData(Vision_Recv_s *receive, uint8_t *rx_buf, uint16_t rx_buf_len);

#endif // RV2_PROTOCAL_H
