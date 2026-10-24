#ifndef LZMOTOR_H
#define LZMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

/**
 * @file lzmotor.h
 * @brief 灵足时代RS00电机驱动,私有协议(29位扩展帧,CAN2.0 1Mbps),协议说明见《RS00使用说明书》第4章
 *
 * 电机上电默认处于运控模式:发送使能帧(通信类型3)后,以500Hz发送运控控制帧(通信类型1)即可驱动。
 * 运控模式内部控制律为 t_ref = Kd*(v_set-v_actual) + Kp*(p_set-p_actual) + t_ff,
 * 本驱动固定发送Kp=Kd=0、位置/速度期望=0,即等效纯力矩控制(t_ff),位置/速度/力矩串级PID由板载完成,
 * 与DM4310驱动结构一致。
 * 扩展帧ID格式:bit28-24=通信类型,bit23-8=数据区2,bit7-0=目标ID;控制帧的力矩位于ID的data区。
 *
 * 使用示例(电机CAN_ID=3,主机CAN_ID=4,挂在CAN1上):
 *     Motor_Init_Config_s lz_config = {
 *         .can_init_config = { .can_handle = &hcan1 }, // tx/rx id由驱动内部按私有协议设置
 *         .controller_param_init_config = { 角度/速度/力矩PID配置,同DM4310 },
 *         .controller_setting_init_config = { 闭环类型与反馈来源,同DM4310 },
 *     };
 *     LZMotorInstance *motor = LZMotorInit(&lz_config, 3, 4); // motor_id, master_id
 * 注意:若电机当前处于CANopen/MIT协议,需先通过上位机(protocol_1标志位)或通信类型25切换协议,重新上电生效。
 */

#define LZ_MOTOR_CNT 4

// 私有协议数据范围,见说明书4.2.1/4.2.3
#define LZ_P_MIN (-12.57f) // 位置,rad,周期性循环±4π
#define LZ_P_MAX (12.57f)
#define LZ_V_MIN (-33.0f) // 速度,rad/s
#define LZ_V_MAX (33.0f)
#define LZ_T_MIN (-14.0f) // 力矩,Nm
#define LZ_T_MAX (14.0f)

// 私有协议扩展帧ID组装:bit28-24=通信类型(mode),bit23-8=数据区2(data),bit7-0=目标ID(id)
#define LZ_MAKE_ID(mode, data, id) (((uint32_t)(mode) << 24) | ((uint32_t)(data) << 8) | (id))

// 通信类型(说明书4.1)
typedef enum
{
    LZ_GET_ID = 0,      // 获取设备ID
    LZ_CONTROL = 1,     // 运控模式控制指令
    LZ_FEEDBACK = 2,    // 电机反馈
    LZ_ENABLE = 3,      // 使能运行
    LZ_STOP = 4,        // 停止运行(数据区Byte0=1清故障)
    LZ_SET_ZERO = 6,    // 设置机械零位
    LZ_SET_CAN_ID = 7,  // 设置CAN_ID
    LZ_READ_PARAM = 17, // 单个参数读取
    LZ_WRITE_PARAM = 18 // 单个参数写入(掉电丢失,需搭配类型22保存)
} LZ_Mode_e;

// 反馈帧ID bit23-22的模式状态
typedef enum
{
    LZ_STATE_RESET = 0,
    LZ_STATE_CALI = 1,
    LZ_STATE_MOTOR = 2,
} LZ_State_e;

typedef struct
{
    float position;           // 电机位置反馈,rad,-12.57~12.57(±4π,周期性循环)
    float velocity;           // 电机速度反馈,rad/s
    float torque;             // 电机力矩反馈,Nm
    float temperature;        // 电机温度,℃
    float last_position;      // 上一次位置,用于跨周期圈数判断
    int32_t total_round;      // 位置跨周期累计的圈数(每圈1440°)
    float angle_single_round; // 单圈角度,-720°~720°
    float total_angle;        // 多圈累计角度,单位°
    float speed_dps;          // 电机速度反馈,°/s
    uint8_t mode_state;       // 反馈帧ID bit23-22:0=Reset,1=Cali,2=Motor
    uint16_t fault;           // 反馈帧ID bit21-16故障位:bit16欠压,bit17驱动,bit18过温,bit19磁编码,bit20堵转,bit21未标定
} LZ_Motor_Measure_s;

typedef struct
{
    LZ_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance *motor_daemon;
    uint16_t last_fault; // 上一次故障状态,用于故障出现时单次打印
    uint8_t motor_id;    // 电机CAN_ID
    uint8_t master_id;   // 主机CAN_ID
} LZMotorInstance;

/**
 * @brief 初始化灵足RS00电机,注册CAN实例并发送使能帧
 * @param config 电机控制配置,can_init_config中只需填写can_handle(扩展帧ID等由驱动内部按私有协议设置)
 * @param motor_id 电机CAN_ID(控制帧ID的bit7-0)
 * @param master_id 主机CAN_ID(使能帧ID的data区,反馈帧ID的bit7-0)
 */
LZMotorInstance *LZMotorInit(Motor_Init_Config_s *config, uint8_t motor_id, uint8_t master_id);

void LZMotorSetRef(LZMotorInstance *motor, float ref);

void LZMotorOuterLoop(LZMotorInstance *motor, Closeloop_Type_e closeloop_type);

void LZMotorEnable(LZMotorInstance *motor);

void LZMotorStop(LZMotorInstance *motor);

/* 修改电机的实际闭环反馈来源,与DJIMotorChangeFeed行为一致 */
void LZMotorChangeFeed(LZMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

void LZMotorControlInit();

#endif // !LZMOTOR_H
