#include "gimbal.h"
#include "dmmotor.h"
#include "lzmotor.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "referee_task.h"
#include "referee_UI.h"

static attitude_t *gimbal_IMU_data; // 云台IMU数据
static DMMotorInstance *pitch_motor;
static LZMotorInstance *roll_motor;

// roll调试变量:目标转速(dps),纯速度环直接给定,volatile可在调试器里实时修改(正式使用前删除)
static volatile float roll_vel_debug = 0;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU

void GimbalInit()
{   
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x05, // 主机发送控制帧的ID,即DM4310的CAN_ID,需与达妙上位机中电机配置一致
            .rx_id = 0x05, // 电机回传反馈帧的ID,即DM4310的master_id,需与达妙上位机中电机配置一致
        },
        .controller_param_init_config = {
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[0],
            .mit_kp = 50,  // 电机内部位置环Kp,上位机调参(范围0-500)
            .mit_kd = 1,   // 电机内部速度环Kd,上位机调参(范围0-5)
        },
        .controller_setting_init_config = {
            .mit_pos_vel_mode = 1, // 使用电机内部速度位置模式,跳过本工程PID
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // 方向待装机后确认
        },
        .motor_type = DM};

    // ROLL
    Motor_Init_Config_s roll_config = {
        .can_init_config = {
            .can_handle = &hcan2, // RS00挂在CAN2上,扩展帧tx/rx ID由驱动内部按私有协议设置
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.0, // 待整定,单位dps/°,1°误差输出30dps
                .Ki = 0.0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve =  PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 100,
                .Output_LPF_RC = 0,
                .MaxOut = 800, // 角度环输出限幅,单位dps
            },
            .speed_PID = {
                .Kp = 0.08, // 台架验证值,单位Nm/(dps)
                .Ki = 0.04,
                .Kd = 0,
                .Improve = PID_Integral_Limit | PID_OutputFilter,
                .Output_LPF_RC = 0.005,
                .IntegralLimit = 10,
                .MaxOut = 10, // 速度环输出限幅,单位Nm(RS00力矩范围±14Nm)
            },
        },
        .controller_setting_init_config = {
            .speed_feedback_source = MOTOR_FEED, // 电机速度反馈,单位dps
            .outer_loop_type = ANGLE_LOOP,       // 速度环为最外层:pid_ref直接作为目标转速
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 方向待装机后确认
        },
        .motor_type = LZ};

    // YAW电机(GM6020)已移入底盘应用chassis.c,此处只保留pitch和roll
    pitch_motor = DMMotorInit(&pitch_config);
    roll_motor = LZMotorInit(&roll_config, 3, 4); // RS00电机CAN_ID=3,主机CAN_ID=4

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // yaw电机(GM6020)已移入底盘应用chassis.c由底盘控制,这里只处理pitch
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DMMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DMMotorEnable(pitch_motor);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch); // pitch会在robot_cmd中处理好多圈和单圈
        break;
    // 云台自由模式,使用编码器反馈
    case GIMBAL_FREE_MODE: // 后续删除
        DMMotorEnable(pitch_motor);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP, MOTOR_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }

    // 台架调试:roll不依赖遥控器与云台模式,直接按调试变量给定转速(纯速度环);正式使用前恢复模式分支控制
    // LZMotorEnable(roll_motor);
    // LZMotorSetRef(roll_motor, roll_vel_debug);

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu(yaw电机单圈角度已随电机移入底盘应用,经chassis_feed回传)
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}