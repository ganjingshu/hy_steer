// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include <math.h>  // 用于fabsf函数

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data_r;  // 视觉发送数据

static PIDInstance *pid_pitch_vision,*pid_yaw_vision;
static Publisher_t *vision_recv_data_pub;

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息



static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
 float yaw_data;  //调速度环用的变量
 float pitch_data;  //调速度环用的变量
extern float vision_yaw_angle;
extern float vision_pitch_angle;

extern uint8_t loader_is_reversing;


static volatile float pitch_vel_data; //调试变量(volatile防止-Og优化掉,可在调试器里实时改值)
static volatile float yaw_vel_data;
static volatile float roll_vel_data;  //调试变量(roll轴目标转速,单位dps,纯速度环)

// 添加裁判系统UI初始化函数声明
extern void MyUIInit();

void RobotCMDInit()
{

    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个（遥控器数据接收）
    //vision_recv_data = VisionInit(&huart1); // 视觉通信串口
    vision_recv_data = VisionInit(NULL);    // 虚拟串口不需要UART句柄

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    vision_recv_data_pub = PubRegister("vision_recv_data", sizeof(Vision_Recv_s));

    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.yaw = 0;
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
    // 将offset_angle绕圈到[-180,180),保证跟随PID始终朝偏差小的方向转(走短程)
    if (chassis_cmd_send.offset_angle < -180.0f)
        chassis_cmd_send.offset_angle += 360.0f;
    else if (chassis_cmd_send.offset_angle > 180.0f)
        chassis_cmd_send.offset_angle -= 360.0f;
}

float chassis_speed_rocker=0;//十级10000
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{




  // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘跟随云台
    {
        // chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        // gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;//底盘云台分离
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;


    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],底盘和云台分离,底盘使用IMU数据
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

        // chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;//底盘云台分离
        // gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    }

    // 云台参数,确定云台控制数据
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中],视觉模式，目前给发射做调试使用
    {
        // gimbal_cmd_send.yaw += 0.002f * (float)rc_data[TEMP].rc.rocker_l_+pid_yaw_vision->Output;
        // gimbal_cmd_send.pitch -= 0.002f * (float)rc_data[TEMP].rc.rocker_l1-pid_pitch_vision->Output;
        // gimbal_cmd_send.yaw -= vision_recv_data->yaw*RAD_2_DEGREE+0.001f * (float)rc_data[TEMP].rc.rocker_l_;
        // gimbal_cmd_send.pitch +=vision_recv_data->pitch*RD_2_DEGREE;
        //自瞄控制变量
        gimbal_cmd_send.yaw=(vision_recv_data->yaw*RAD_2_DEGREE-5)*0.6+gimbal_cmd_send.yaw_vel*20+0.001f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch=(vision_recv_data->pitch*RAD_2_DEGREE)*0.5+gimbal_cmd_send.pitch_vel*0.1+0.0015f * (float)rc_data[TEMP].rc.rocker_l1;
        //vision_l_yaw_tar = yaw_l_motor->measure.total_angle + gimbal_cmd_recv.yaw*RAD_2_DEGREE*0.3 + gimbal_cmd_recv.yaw_vel*15*(-1);
    }
    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || vision_recv_data->mode == NO_TARGET)
    { // 按照摇杆的输出大小进行角度增量,增益系数需调整
        gimbal_cmd_send.yaw += -0.003f * (float)rc_data[TEMP].rc.rocker_l_;//左水平拨杆,控制yaw角度
        gimbal_cmd_send.pitch += -0.0015f * (float)rc_data[TEMP].rc.rocker_l1;//左竖直拨杆,控制pitch角度

    }
     //云台软件限位（pitch）
    if(gimbal_cmd_send.pitch>=PITCH_MAX_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    }
    else if(gimbal_cmd_send.pitch<=PITCH_MIN_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    }

    // 射击控制（调试与检录用）
    if(switch_is_mid(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right))//左开关中，右开关下（摩擦轮关，拨弹关）
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode=LOAD_STOP;
    }
    else if(switch_is_mid(rc_data[TEMP].rc.switch_left) && switch_is_mid(rc_data[TEMP].rc.switch_right))//左开关中，右开关中（摩擦轮开，拨弹关）
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.load_mode=LOAD_STOP;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
    else if(switch_is_mid(rc_data[TEMP].rc.switch_left) && switch_is_up(rc_data[TEMP].rc.switch_right))//左开关中，右开关上（摩擦轮开，拨弹开）
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.load_mode=LOAD_1_BULLET;
        //shoot_cmd_send.load_mode=LOAD_stovepipe;//卡弹反拨

        // yaw_data=fabsf(gimbal_cmd_send.yaw - gimbal_fetch_data.gimbal_imu_data.Yaw);
        // pitch_data=fabsf(gimbal_cmd_send.pitch - gimbal_fetch_data.gimbal_imu_data.Pitch);
        // if( yaw_data < 0.8 &&  pitch_data < 0.8)    //当pitch和yaw角度都在0.8以下时,才开启发射
        // {
        //   if(loader_is_reversing==1 )
        //   {
        //   shoot_cmd_send.load_mode=LOAD_stovepipe;
        //   }
        //   else
        //   {
        //   shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
        //   }
        //}
        // else
        // {
        //     shoot_cmd_send.load_mode=LOAD_STOP;
        // }
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }


    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    chassis_cmd_send.vx = 15.0f * (float)(rc_data[TEMP].rc.rocker_r_-12); // _水平方向（-11是为了补偿遥控器波杆偏移）  //-11   //*-20
    chassis_cmd_send.vy = 15.0f * (float)rc_data[TEMP].rc.rocker_r1; // 竖直方向

    shoot_cmd_send.shoot_rate = 8;  //每秒多少发
}




/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    shoot_cmd_send.shoot_mode = SHOOT_ON;

    chassis_cmd_send.vy = (-rc_data[TEMP].key[KEY_PRESS].w  + rc_data[TEMP].key[KEY_PRESS].s ) * chassis_cmd_send.chassis_speed_buff; // 系数待测
    chassis_cmd_send.vx = (-rc_data[TEMP].key[KEY_PRESS].a  + rc_data[TEMP].key[KEY_PRESS].d ) * chassis_cmd_send.chassis_speed_buff; // 系数待测

    //自瞄模式（右键按下，左键松开）
    // if(rc_data[TEMP].mouse.press_r % 2 == 1 && rc_data[TEMP].mouse.press_l % 2==0 )
    // {
    //    shoot_cmd_send.friction_mode = FRICTION_ON;
    //    //自瞄控制变量
    //    gimbal_cmd_send.yaw=(vision_recv_data->yaw*RAD_2_DEGREE ) * 0.65;   //右减左加
    //    gimbal_cmd_send.pitch=(vision_recv_data->pitch*RAD_2_DEGREE ) * 0.5  ;//上减下加  //8

        // gimbal_cmd_send.yaw = 0; // 系数待测
        // gimbal_cmd_send.pitch =0;


        // yaw_data=fabsf(gimbal_cmd_send.yaw - gimbal_fetch_data.gimbal_imu_data.Yaw);
        // pitch_data=fabsf(gimbal_cmd_send.pitch - gimbal_fetch_data.gimbal_imu_data.Pitch);
        // if( yaw_data < 0.8 &&  pitch_data < 0.8)    //当pitch和yaw角度都在0.8以下时,才开启发射
        // {

        // if (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 2 == 1)
        //  {
        //     if(loader_is_reversing==1 )
        //   {
        //   shoot_cmd_send.load_mode=LOAD_stovepipe;
        //   }
        //   else
        //   {
        //   shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
        //   }
        //  }
        // }
        // else
        // {
        //     shoot_cmd_send.load_mode=LOAD_STOP;
        // }


    // }
    // else
    // {
        gimbal_cmd_send.yaw += -(float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
        gimbal_cmd_send.pitch += -(float)rc_data[TEMP].mouse.y / 660 * 10;
    //}


   //云台软件限位（pitch）
    if(gimbal_cmd_send.pitch>=PITCH_MAX_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    }
    else if(gimbal_cmd_send.pitch<=PITCH_MIN_ANGLE)
    {
        gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    }


    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 2) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.shoot_rate = 10;//15
        break;
    default:
        shoot_cmd_send.shoot_rate = 14;
        break;
    }

    switch (rc_data[TEMP].mouse.press_l % 2) // 鼠标左键设置发射模式（左键按下开火，左键松开停火）
    {
    case 0:
        if(rc_data[TEMP].mouse.press_r % 2 == 0)
        {
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
        break;

    default:
        if(rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2 == 1 )
        {
          if(loader_is_reversing==1 )
          {
          shoot_cmd_send.load_mode=LOAD_stovepipe;
          }
          else
          {
          shoot_cmd_send.load_mode=LOAD_BURSTFIRE;
          }
        }
        break;
    }

    //目前是使用长按shift小陀螺
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键切换底盘模式（小陀螺，底盘跟随云台）
    // {
    // case 0:
    //     chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    //     break;
    // default:
    //     chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    //     break;
    // }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 2) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 3000;//2000
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 5000;//4000
        break;
    }

    if (rc_data[TEMP].key[KEY_PRESS].ctrl == 1) // ctrl键按下时重新初始化UI
    {
       MyUIInit(); // 重新初始化UI位置和任务
    }

    if(rc_data[TEMP].key[KEY_PRESS].shift == 1)
    {
    chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    }
    else
    {
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }

    // switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    // {
    // case 1:

    //     break;
    // default:

    //     break;
    // }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制   2
         // MouseKeySet();
         RemoteControlSet();
    else if(switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[中],视觉模式（此时还是遥控器控制）
        RemoteControlSet();
     else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制    1
        RemoteControlSet();
        //  MouseKeySet();
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // roll轴转速为调试变量,直接给定,不参与遥控器/键鼠逻辑
    gimbal_cmd_send.roll_vel = roll_vel_data;

    //自瞄
    //VisionRadaControlSet();


    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(vision_recv_data_pub, (void *)vision_recv_data);
}
