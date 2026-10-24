/**
 * @file    master_process.c
 * @brief   视觉通信模块 - 同济 RV2 协议
 * @version 2.0
 * @note    同济协议下视觉端已完成弹道解算，MCU端职责：
 *          1. 接收 pitch/yaw（角度值）→ 低通滤波 → 输出给云台PID
 *          2. 回传 IMU 姿态 + 弹速/子弹计数给视觉
 *          3. 离线检测与通信恢复
 */

#include "master_process.h"
#include "rv2_protocal.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "bsp_dwt.h"

#define PI2 (2.0f * 3.1415926535f)

/* ======================== 静态全局变量 ======================== */

static Vision_Recv_s  recv_data;
static Vision_Send_s  send_data;
static DaemonInstance *vision_daemon_instance;

//用于低通滤波的时间参数
#define PITCH_LPF_RC  0.022f   // pitch 滤波时间常数
#define YAW_LPF_RC    0.03f    // yaw  滤波时间常数

/* ==================== 公共设置函数 ==================== */

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    UNUSED(enemy_color);
    UNUSED(work_mode);
    UNUSED(bullet_speed);
}

void VisionSetAltitude(float yaw, float pitch)
{
    send_data.yaw   = yaw;
    send_data.pitch = pitch;
}

void VisionSetImuData(float w, float x, float y, float z,
                       float yaw_vel, float pitch_vel)
{
    send_data.q[0]      = w;
    send_data.q[1]      = x;
    send_data.q[2]      = y;
    send_data.q[3]      = z;
    send_data.yaw_vel   = yaw_vel;
    send_data.pitch_vel = pitch_vel;
}

void VisionBulletData(float bullet_speed, uint16_t bullet_count)
{
    send_data.bullet_speed = bullet_speed;
    send_data.bullet_count = bullet_count;
}

#ifdef VISION_USE_UART
static USARTInstance *vision_usart_instance;
#endif
/* ==================== 离线处理 ==================== */

static void VisionOfflineCallback(void *id)
{
    UNUSED(id);
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif
    LOGWARNING("[vision] offline, restarting...");
}

/* ==================== 低通滤波 ==================== */

/**
 * @brief  对视觉下发的 pitch/yaw 做一阶低通滤波
 * @note   同济协议下视觉已算好角度(°)，此处仅做平滑 + 丢失保持 + yaw跨圈补偿
 *         目标丢失时保持上一次滤波值，防止云台抖回零位
 */
void VisionTrajectory()
{
    static uint32_t dt_cnt = 0;
    static float    last_pitch = 0.0f, last_yaw = 0.0f;
    static float    yaw_circle = 0.0f;

    float   dt      = DWT_GetDeltaT(&dt_cnt);
    uint8_t is_lost = (recv_data.mode != TRACKING);

    yaw_circle = (int)(send_data.yaw / PI2);    // 云台累计圈数

    if (!is_lost) {
        // Pitch 一阶低通
        recv_data.pitch =
            recv_data.pitch * dt / (PITCH_LPF_RC + dt) +
            last_pitch * PITCH_LPF_RC / (PITCH_LPF_RC + dt);
        last_pitch = recv_data.pitch;

        // Yaw 一阶低通 + 跨圈补偿
        float raw_yaw = recv_data.yaw + yaw_circle * 360.0f;
        recv_data.yaw =
            raw_yaw * dt / (YAW_LPF_RC + dt) +
            last_yaw * YAW_LPF_RC / (YAW_LPF_RC + dt);
        last_yaw = recv_data.yaw;
    }
    // 丢失时 recv_data.pitch/yaw 保持上一次滤波值不变
}

/* ================================================================
 *                     UART 传输实现
 * ================================================================ */
#ifdef VISION_USE_UART
#include "bsp_usart.h"

static void DecodeVision()
{
    DaemonReload(vision_daemon_instance);

    RV2_GimbalToRecvData(&recv_data,
                         vision_usart_instance->recv_buff,
                         VISION_RECV_SIZE);
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf = {
        .module_callback = DecodeVision,
        .recv_buff_size  = VISION_RECV_SIZE,
        .usart_handle    = _handle,
    };
    vision_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback,
        .owner_id     = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint8_t  send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    
    RV2_GimbalToSendData(&send_data, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA);
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

/* ================================================================
 *                     USB VCP 传输实现
 * ================================================================ */
#ifdef VISION_USE_VCP
#include "bsp_usb.h"
static uint8_t *vis_recv_buff;
static void DecodeVision(uint16_t recv_len)
{
    DaemonReload(vision_daemon_instance);

    RV2_GimbalToRecvData(&recv_data, vis_recv_buff, recv_len);
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle);// 仅为了消除警告
    USB_Init_Config_s conf = { .rx_cbk = DecodeVision };
    vis_recv_buff = USBInit(conf);
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback,// 离线时调用的回调函数,会重启串口接收
        .owner_id     = NULL,
        .reload_count = 5, //50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
    VisionTrajectory();
}

void VisionSend()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint8_t  send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;

    RV2_GimbalToSendData(&send_data, send_buff, &tx_len);
    USBTransmit(send_buff, tx_len);
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_VCP