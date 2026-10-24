/**
 * @file rv2_protocal.c
 * @brief 同济 RV2 视觉通信协议帧收发,适配本框架接口
 * @note  使用框架 modules/algorithm 中的 crc_16() (CRC-16/IBM, poly 0xA001),
 *        帧结构直接复用 master_process.h 的 Vision_Send_s / Vision_Recv_s
 */

#include "rv2_protocal.h"
#include "crc16.h"
#include "bsp_log.h"
#include <string.h>

/**
 * @brief  将框架的 Vision_Send_s 打包为 RV2 帧并计算 CRC16
 * @param  send       框架视觉发送数据
 * @param  tx_buf     输出帧缓冲
 * @param  tx_buf_len 输出帧长度
 * @note   帧格式: header(0x5053 "SP") + mode + 数据 + crc16(小端,不含crc字段本身)
 */
void RV2_GimbalToSendData(Vision_Send_s *send, uint8_t *tx_buf, uint16_t *tx_buf_len)
{
    // 静态缓冲保证函数退出后帧数据不被释放
    static Vision_Send_s send_frame = {
        .header = RV2_PROTOCAL_HEADER,
        .mode   = 1,
    };

    // 姿态/IMU数据
    send_frame.q[0]          = send->q[0];
    send_frame.q[1]          = send->q[1];
    send_frame.q[2]          = send->q[2];
    send_frame.q[3]          = send->q[3];
    send_frame.yaw           = send->yaw;
    send_frame.yaw_vel       = send->yaw_vel;
    send_frame.pitch         = send->pitch;
    send_frame.pitch_vel     = send->pitch_vel;
    send_frame.bullet_speed  = send->bullet_speed;
    // send_frame.bullet_count  = send->bullet_count; // 暂未使用

    // 整帧CRC16校验(不含crc字段本身)
    send_frame.crc_16 = crc_16((uint8_t *)&send_frame, sizeof(send_frame) - 2);

    memcpy(tx_buf, &send_frame, sizeof(send_frame));
    *tx_buf_len = sizeof(send_frame);
}

/**
 * @brief  解析视觉下发的 RV2 帧,直接写入框架的 Vision_Recv_s
 * @param  receive    框架视觉接收数据
 * @param  rx_buf     接收到的原始帧
 * @param  rx_buf_len 帧长度
 * @note   帧头 0x53 0x50 ("SP"),crc16 在帧尾以大端存放
 */
void RV2_GimbalToRecvData(Vision_Recv_s *receive, uint8_t *rx_buf, uint16_t rx_buf_len)
{
    // 长度保护,防止短帧越界
    if (rx_buf_len < sizeof(Vision_Recv_s))
        return;

    // 包头校验
    if (rx_buf[0] != 0x53 || rx_buf[1] != 0x50)
    {
        receive->mode = NO_TARGET;
        LOGERROR("RV2 Receive Header error");
        return;
    }

    // CRC16整帧校验(不含crc字段本身),接收帧的crc为大端存放
    uint16_t checksum = crc_16(rx_buf, rx_buf_len - 2);
    if (rx_buf[rx_buf_len - 1] != ((checksum & 0xFF00) >> 8) ||
        rx_buf[rx_buf_len - 2] != (checksum & 0x00FF))
    {
        LOGERROR("RV2 Receive checksum error");
        return;
    }

    // 校验通过,直接按框架结构解析
    memcpy(receive, rx_buf, sizeof(Vision_Recv_s));
}
