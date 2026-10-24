#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include <stdint.h>
#include "bsp_usart.h"

#define VISION_RECV_SIZE 48u // 当前为固定值,48字节
#define VISION_SEND_SIZE 48u

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{	//默认状态
	NO_TARGET = 0,
	TRACKING = 1,

	//调试状态
	// NO_TARGET = 1,
	// TRACKING = 0,

	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
    uint16_t header ;//SP
    uint8_t mode; // 0:不控制 (N0_TARGET),1:控制云台但不开火(TRACKING),2:控制云台且开火(READY_TO_FIRE)
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc; 
    uint16_t crc16;
} Vision_Recv_s;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
    uint16_t header ;//SP
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;// 子弹累计发送次数
    uint16_t crc_16;
} Vision_Send_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);
void VisionSend();
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);
void VisionSetAltitude(float yaw, float pitch);
void VisionSetImuData(float w, float x, float y, float z, float yaw_vel, float pitch_vel);
void VisionBulletData(float bullet_speed, uint16_t bullet_count);
void VisionTrajectory();

#endif // !MASTER_PROCESS_H