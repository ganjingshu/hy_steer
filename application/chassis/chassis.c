/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "power_control.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "dji_motor.h"
#include <stdbool.h>

#include "shoot.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_OMNI_WHEEL (RADIUS_OMNI_WHEEL * 2 * PI)   // 全向轮周长
#define PERIMETER_STEER_WHEEL (RADIUS_STEER_WHEEL * 2 * PI)  // 舵轮周长
/* 舵轮驱动速度换算系数:舵轮半径(60mm)小于全向轮(70mm),相同转速下舵轮线速度低,
   会被全向轮拖着走,转速乘 r_omni/r_steer 使轮缘线速度对齐 */
#define STEER_DRIVE_SPEED_RATIO (RADIUS_OMNI_WHEEL * 1.0f / RADIUS_STEER_WHEEL)
/* 小陀螺起转放闸角:刚进入自旋时,前/后舵轮实际几何角都转到该角度以上,四个3508才给速度 */
#define SPIN_GYRO_READY_ANGLE 85.0f

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static PIDInstance buffer_PID;             // 用于底盘的缓冲能量PID
static PIDInstance chassis_follow_PID;     // 用于底盘跟随云台的PID控制
static PID_Init_Config_s chassis_follow_pid_conf; // 底盘跟随云台PID配置
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI


// 添加发射模块订阅者
static Subscriber_t *shoot_sub; // 发射模块订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 发射控制命令

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_left, *motor_right, *motor_front, *motor_back, *steer_front, *steer_back; // left right forward back,前后为舵向轮组,左右为全向轮

//调试变量
static float chassis_lb_v=0; // 底盘前轮(舵向驱动)电机速度
static float chassis_rb_v=0; // 底盘后轮(舵向驱动)电机速度
static float chassis_lf_v=0; // 底盘左全向轮电机速度
static float chassis_rf_v=0; // 底盘右全向轮电机速度
static volatile float angle_target_front=0;    // 前舵向目标单圈角(零位+DIR×几何目标角,度)调试用
static volatile float angle_target_back=0;     // 后舵向目标单圈角(零位+DIR×几何目标角,度)调试用
static volatile float steer_front_angle_ref=0; // 前舵向角度环设定值(目标多圈角,度)调试用
static volatile float steer_back_angle_ref=0;  // 后舵向角度环设定值(目标多圈角,度)调试用
float chassis_lb_speed=400;

static float chassis_pid_output[4];
static float chassis_pid_totaloutput;
static DJIMotorInstance *chassis_motor_instance[4]; 

static float chassis_power_limit,chassis_input_power,chassis_power_buffer;//裁判系统获取的功率限制值、当前功率值、当前缓冲能量值
static float chassis_power_max;//计算使用的最大功率值
static float chassis_power_offset = -5; // 功率冗余，可修改
static float output_zoom_coeff = 0;     // 输出缩放系数

//三个系数
float toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
float k1 = 1.26e-07;                      // k1，9.50000043e-08
float k2 = 1.95000013e-07;                // k2
float constant_coefficient = 3.5f;

//标示是否处在缓冲能量低状态
bool isLowBuffer = false;

// 辅助函数定义
static float float_Square(float x) { return x * x; }
static float abs_limit(float x, float limit) { return (x > limit) ? limit : ((x < -limit) ? -limit : x); }
/* 角度折返到 [-180,180),用于舵向单圈空间的最短路径计算 */
static float wrap_180(float x)
{
    while (x > 180.0f)
        x -= 360.0f;
    while (x < -180.0f)
        x += 360.0f;
    return x;
}
 
#define CHASSIS_POWER_COFFICIENT (1 - (float)(120 - 45) / (float)(135 - 50)) // 这个量出现是因为我们的电机阻力较大，导致理论值和实际值相差较大，用于补偿



/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;                      // 将云台系的速度投影到底盘
static float vt_front, vt_back, vt_left, vt_right, angle_front, angle_back;           // 底盘速度解算后的临时输出,待进行限幅
static volatile bool spin_gyro_wait = false; // 小陀螺起转等待:切进小陀螺且舵轮未到85°时扣住四个3508,放闸后不再扣
static volatile bool prev_rotate_mode = false; // 上一周期是否处于小陀螺模式,模式切换沿触发等待

/* ------------------------------ 舵向零位与方向 ------------------------------ */
/*
 * 齐奇(RM2026开源)式零位标定:固定机械偏移常量,一次标定写入,机械不变则不重标,
 * 零位与上电姿态无关,开电即走.
 * 标定流程:轮组摆正(直尺贴底盘侧板对齐) → 调试器 watch 窗口读
 * steer_*->measure.angle_single_round(或临时加RTT打印) → 读数填入下方宏.
 * 机械装配或编码器更换后必须重新标定.
 * 舵向方向:模块反装时几何角与电机转向相反,把对应 DIR 宏置 -1.0f(装反就改这里).
 */
#define STEER_FRONT_ZERO_OFFSET 25.53f 
#define STEER_BACK_ZERO_OFFSET 143.17f  
#define STEER_FRONT_DIR 1.0f         // 舵向几何角与电机转向同向:+1,反向:-1
#define STEER_BACK_DIR 1.0f      // 沿用原 steer_rf 模块安装方向,待装机确认
static float steer_front_zero_offset, steer_back_zero_offset;

void ChassisInit()
{

    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 2, // 17
                .Ki = 1,   // 3.5
                .Kd = 0,   // 0
                .IntegralLimit = 5000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 16384, // 取消功率限制:速度环输出直接就是电流值,放开到M3508满量程(原10000≈61%)
                .Output_LPF_RC = 0.3,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 设置为开环，电机设定值由下面的功率控制设定，不走普通的pid
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    // 前后左右布局的ID:前=1 后=2 左=3 右=4,与电机拨码一致(拨码改了这里也要改)
    // 直走(vx>0)时两个全向轮(左lf/右rf)速度反馈为正,反转标志由该约定推出
    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL; // 左,全向轮:vt_left=-vx,反转后直走为正
    motor_left = DJIMotorInit(&chassis_motor_config);


    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE; // 右,全向轮:vt_right=-vx,反转后直走为正
    motor_right = DJIMotorInit(&chassis_motor_config);

        Motor_Init_Config_s chassis_steermotor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 1.2, // 17
                .Ki = 0.5,   // 3.5
                .Kd = 0,   // 0
                .IntegralLimit = 5000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 16384, // 取消功率限制:速度环输出直接就是电流值,放开到M3508满量程(原10000≈61%)
                .Output_LPF_RC = 0.3,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 设置为开环，电机设定值由下面的功率控制设定，不走普通的pid
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };

    // 前/后舵向驱动轮共用一套PID,只改tx_id和反转标志(PIDInit在初始化时拷贝参数,复用结构体安全)
    chassis_steermotor_config.can_init_config.tx_id = 1;
    chassis_steermotor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL; // 前,直走为正
    motor_front = DJIMotorInit(&chassis_steermotor_config);

    chassis_steermotor_config.can_init_config.tx_id = 2;
    chassis_steermotor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL; // 后,直走为正
    motor_back = DJIMotorInit(&chassis_steermotor_config);

    // 功率限制用的电机指针表(顺序:前后左右),之前只声明未赋值,会空指针
    chassis_motor_instance[0] = motor_front;
    chassis_motor_instance[1] = motor_back;
    chassis_motor_instance[2] = motor_left;
    chassis_motor_instance[3] = motor_right;



    Motor_Init_Config_s chassis_motor_config_steer = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 30, // 17
                .Ki = 50,   // 3.5
                .Kd = 0,   // 0
                .IntegralLimit = 5000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 10000,
                .Output_LPF_RC = 0.3,
            },
            .angle_PID = {
                .Kp = 35,//20, // 8
                .Ki = 60,
                .Kd = 0,
                .DeadBand = 0.1, //死区
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,
                .MaxOut = 300,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            // .power_limit_flag = POWER_LIMIT_ON // 新版电机库无此字段,功率限制由 LimitChassisOutput() 实现
        },
        .motor_type = GM6020,
    };



    // 前后舵向GM6020:方向统一在应用层用 STEER_*_DIR 处理,
    // 角度环的 REVERSE 会镜像设定值,与"以当前角为基准生成设定值"的舵向控制不兼容
    chassis_motor_config_steer.can_init_config.tx_id = 1; // 前舵向
    chassis_motor_config_steer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    steer_front = DJIMotorInit(&chassis_motor_config_steer);

    chassis_motor_config_steer.can_init_config.tx_id = 2; // 后舵向
    chassis_motor_config_steer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    steer_back = DJIMotorInit(&chassis_motor_config_steer);
    


    // referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

/* Buffer环暂未测试，逻辑是计算期望buffer与实际buffer的差值，转换为冗余的功率，todo：输入给功率控制部分，待完善 */
    PID_Init_Config_s Buffer_pid_conf = {
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,
        .IntegralLimit = 1000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 1000,
    };
    PIDInit(&buffer_PID, &Buffer_pid_conf); // 缓冲能量PID初始化

    PID_Init_Config_s chassis_follow_pid_conf = {
        .Kp = 0.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .DeadBand = 1.0f,
        .IntegralLimit = 5000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 10000,
    };
    
    PIDInit(&chassis_follow_PID, &chassis_follow_pid_conf); // 底盘跟随云台PID初始化
    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息（但目前是单板方案）
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    // 添加发射模块订阅者初始化
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
#endif // ONE_BOARD
}

/* 前后左右布局:轮组位于四边中点(x=前后,前正;y=左右,右正),wz项力臂只取单边坐标 */
#define FRONT_CENTER ((HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)   // 前(舵向)
#define BACK_CENTER ((HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)    // 后(舵向)
#define LEFT_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X) * DEGREE_2_RAD)   // 左(全向)
#define RIGHT_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X) * DEGREE_2_RAD)  // 右(全向)

/* 舵向轮组在底盘系下的位置(与上方*_CENTER宏同源,单位mm):x=前后(前正),y=左右(右正) */
#define STEER_FRONT_X (HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y)
#define STEER_FRONT_Y 0
#define STEER_BACK_X (-(HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y))
#define STEER_BACK_Y 0

/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_FREE_MODE)
    { // 自由模式:舵向锁回正(前进方向),驱动轮直给vx,左右全向轮保持解算
        angle_front = 0.0f;
        angle_back = 0.0f;
        vt_front = chassis_vx;
        vt_back = chassis_vx;
        vt_left = -chassis_vx - chassis_cmd_recv.wz * LEFT_CENTER;
        vt_right = -chassis_vx + chassis_cmd_recv.wz * RIGHT_CENTER;
        spin_gyro_wait = false;
        prev_rotate_mode = false;
        return;
    }

    // 左/右全向轮:轴线沿左右方向,只出力于vx与wz;横向vy由滚轮自由滑动,轮体不转
    vt_left = -chassis_vx - chassis_cmd_recv.wz * LEFT_CENTER;   // 左,全向
    vt_right = -chassis_vx + chassis_cmd_recv.wz * RIGHT_CENTER;  // 右,全向
    // 前/后舵向驱动:速度取轮位合速度的模(纯vy时前后同向同速),就近原则翻转在SteerModuleSetRef内处理
    vt_front = sqrtf(chassis_vx * chassis_vx +
                  (chassis_vy + chassis_cmd_recv.wz * FRONT_CENTER) * (chassis_vy + chassis_cmd_recv.wz * FRONT_CENTER)); // 前,舵向驱动
    vt_back = sqrtf(chassis_vx * chassis_vx +
                  (chassis_vy - chassis_cmd_recv.wz * BACK_CENTER) * (chassis_vy - chassis_cmd_recv.wz * BACK_CENTER));   // 后,舵向驱动

    // 舵向目标角 = 轮子处实际速度方向(平移 + 自转切向分量ω×r),纯平移(wz=0)时退化为 atan2(vy,vx)
    // 距离乘 DEGREE_2_RAD,与上方 vt_* 中 wz*CENTER 项保持同一缩放;wz 逆时针为正
    angle_front = atan2(chassis_vy + chassis_cmd_recv.wz * STEER_FRONT_X * DEGREE_2_RAD,
                        chassis_vx - chassis_cmd_recv.wz * STEER_FRONT_Y * DEGREE_2_RAD) * 57.2958f;
    angle_back = atan2(chassis_vy + chassis_cmd_recv.wz * STEER_BACK_X * DEGREE_2_RAD,
                       chassis_vx - chassis_cmd_recv.wz * STEER_BACK_Y * DEGREE_2_RAD) * 57.2958f;

    // 小陀螺放闸:切进CHASSIS_ROTATE(模式切换沿)时,若前/后舵轮未转到85°以上,扣住四个3508,
    // 等两个舵轮实际都到85°(切线方向±90°附近)再给速度,避免起转瞬间轮子横着被拖;
    // 放闸后本次自旋期间不再扣(舵轮在90°附近抖动/推杆不会反复切速度),退出小陀螺模式后重新武装
    bool rotate_mode = (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE);
    if (rotate_mode)
    {
        float geo_front = wrap_180(STEER_FRONT_DIR * (steer_front->measure.angle_single_round - STEER_FRONT_ZERO_OFFSET));
        float geo_back  = wrap_180(STEER_BACK_DIR  * (steer_back->measure.angle_single_round  - STEER_BACK_ZERO_OFFSET));
        if (!prev_rotate_mode && !spin_gyro_wait &&
            (fabsf(geo_front) < SPIN_GYRO_READY_ANGLE || fabsf(geo_back) < SPIN_GYRO_READY_ANGLE))
            spin_gyro_wait = true; // 刚切进小陀螺且舵轮未到位:开始等待
        if (spin_gyro_wait)
        {
            // 放闸:舵轮到位;或目标角偏离切线(推杆平移时目标角<85°,舵轮到不了85°,放弃等待防死锁)
            if ((fabsf(geo_front) >= SPIN_GYRO_READY_ANGLE && fabsf(geo_back) >= SPIN_GYRO_READY_ANGLE) ||
                fabsf(angle_front) < SPIN_GYRO_READY_ANGLE || fabsf(angle_back) < SPIN_GYRO_READY_ANGLE)
                spin_gyro_wait = false; // 放闸
            else
                vt_left = vt_right = vt_front = vt_back = 0.0f; // 等舵轮到位,四个3508都不动
        }
    }
    else
    {
        spin_gyro_wait = false;
    }
    prev_rotate_mode = rotate_mode;
}


/* 台架调试:未接裁判系统(referee_data为NULL)时的手动功率上限(W);
   比赛接上裁判系统后走裁判系统下发值,此宏不再生效 */
#define CHASSIS_POWER_LIMIT_MANUAL 50.0f

/**
 * @brief 对功率进行缩放,参考西交利物浦的方案:估算四个驱动轮总功率,
 *        超限时按比例压缩各轮功率并反解电流上限写入speed_PID.MaxOut(逐轮限流)
 *
 */
static void LimitChassisOutput()
{
    chassis_pid_totaloutput = 0;
    // 估算四个驱动轮(M3508,不含舵向GM6020)的当前功率并累加,功率模型
    // P = k1·I² + k2·n² + toque_coefficient·n·I + constant(I=电流指令,n=转速rpm),
    // 与下方反解电流的一元二次方程系数(a=k1, b=toque_coefficient·n, c=k2·n²-P+constant)保持一致。
    // 两个取值要点:
    // 1) n用rpm(speed_aps/6):系数按原框架rpm标定,直接代speed_aps会让k2·n²放大36倍;
    // 2) I用PID钳位前的需求(Pout+Iout+Dout)而不是钳位后的Output:若用Output,限流会压低估算功率,
    //    误判未超限后恢复满电流又再超限,形成bang-bang抖动(表现为功率超限后电机疯转/抽搐)
    for (int i = 0; i < 4; i++)
    {
        PIDInstance *pid = &chassis_motor_instance[i]->motor_controller.speed_PID;
        float current_demand = pid->Pout + pid->Iout + pid->Dout;
        float speed_rpm = chassis_motor_instance[i]->measure.speed_aps / 6.0f; // 度每秒→rpm
        chassis_pid_output[i] = k1 * float_Square(current_demand) + k2 * float_Square(speed_rpm) +
                                toque_coefficient * speed_rpm * current_demand + constant_coefficient;
        if (chassis_pid_output[i] < 0)
            continue; // 负功率=电机发电/制动,不计入消耗
        chassis_pid_totaloutput += chassis_pid_output[i];
    }
    if (referee_data)
    { // 已接裁判系统:功率上限取裁判系统下发值,并按缓冲能量做迟滞调节
        chassis_power_limit = referee_data->GameRobotState.chassis_power_limit; // 从裁判系统获取的能量限制
        chassis_input_power = referee_data->PowerHeatData.chassis_power;
        chassis_power_buffer = referee_data->PowerHeatData.chassis_power_buffer;

        // 根据缓冲能量和当前功率限制，计算最大功率值
        chassis_power_offset = -1 * CHASSIS_POWER_COFFICIENT * (chassis_power_limit)-0;
        chassis_power_max = chassis_power_limit + chassis_power_offset;

        if (isLowBuffer)
        {
            chassis_power_max = chassis_power_max - 25;
            if (chassis_power_buffer >= 55.0f)
            {
                isLowBuffer = false;
            }
        }
        else
        {
            // 缓冲能量判断，如果缓冲能量少，则马上减小功率，减少量待测
            if (chassis_power_buffer < 10.0f)
            {
                isLowBuffer = true;
                chassis_power_max = chassis_power_max - 35;
            }
        }
    }
    else
    { // 未接裁判系统(台架):用手动功率上限直接限功率
        chassis_power_limit = CHASSIS_POWER_LIMIT_MANUAL;
        chassis_power_max = CHASSIS_POWER_LIMIT_MANUAL;
    }

  

    if (chassis_pid_totaloutput > chassis_power_max) // 超出功率
    {
        output_zoom_coeff = chassis_power_max / chassis_pid_totaloutput;
        for (int i = 0; i < 4; i++)
        {
            chassis_pid_output[i] *= output_zoom_coeff;
            PIDInstance *pid = &chassis_motor_instance[i]->motor_controller.speed_PID;
            if (chassis_pid_output[i] < 0)
            { // 发电/制动轮不消耗功率,不限流,恢复满量程
                pid->MaxOut = 16384.0f;
                continue;
            }

            //公式法解力矩功率与实际电机功率的一元二次方程 k1·I²+b·I+c=0,转速用rpm(与累加侧一致)
            float a = k1;
            float speed_rpm = chassis_motor_instance[i]->measure.speed_aps / 6.0f;
            float b = toque_coefficient * speed_rpm;
            float c = k2 * float_Square(speed_rpm) - chassis_pid_output[i] + constant_coefficient;
            // 只取正根:c<0(预算高于空转底耗)时两根一正一负,正根即允许电流;c>=0说明该轮光维持
            // 当前转速就已超预算,允许电流置0。注意绝不能取负根的绝对值——那会在最该限流时
            // 反而给出几千的电流上限(功率越超、电机越疯转的元凶)
            float temp = 0.0f;
            if (c < 0.0f)
            {
                temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
                if (temp < 0.0f)
                    temp = 0.0f;
            }
            // 旧版 DJIMotorSetOutputLimit() 在新版库中不存在,直接限幅速度环输出
            pid->MaxOut = abs_limit(temp, 13000);
            // 抗积分饱和:限流期间把积分钳到当前上限内,否则恢复满电流瞬间Iout释放造成冲击
            pid->Iout = abs_limit(pid->Iout, pid->MaxOut);
        }
    }
    else
    {
        // 未超功率:恢复满量程电流上限(16384)。注意不能写成 MaxOut=|Output|,
        // 否则上限会被每周期锁死在当前输出,电机之后再也无法加大电流
        for (int i = 0; i < 4; i++)
            chassis_motor_instance[i]->motor_controller.speed_PID.MaxOut = 16384.0f;
    }
}


/**
 * @brief 舵向轮组控制器,移植齐奇 RM2026 开源方案:
 *        1. 零位常量 + 几何角,在单圈空间把误差折返到 [-180,180)
 *        2. 就近原则:误差超过 90° 时目标等效翻转 180°,驱动轮速度取反,舵向最多只转 90°
 *        3. cos³ 轮速补偿:舵向未对准时压低驱动速度,减小侧向拖拽
 *        4. 以当前多圈角为基准叠加误差作为设定值,角度环误差即最短路径角,
 *           同时规避多圈累计问题(设定值每周期相对当前角重新生成)
 *
 * @param steer        舵向电机
 * @param drive        与该舵向轮组配对的驱动电机
 * @param zero_offset  舵向零位(单圈编码器角,0~360)
 * @param dir          安装方向修正,几何角与电机转向同向为+1,反向为-1
 * @param target_geo   几何目标角(度)
 * @param drive_speed  驱动轮目标速度(舵向解算值)
 */
static void SteerModuleSetRef(DJIMotorInstance *steer, DJIMotorInstance *drive,
                              float zero_offset, float dir, float target_geo, float drive_speed)
{
    if (drive_speed == 0.0f && !spin_gyro_wait)
    { // 松摇杆回正:无指令时目标几何角置0,就近原则自动走最短路径回前进方向;
      // 小陀螺等待舵轮到位期间(spin_gyro_wait)保持切线目标角,不回收,否则永远转不到85°
        target_geo = 0.0f;
    }

    // 当前几何角 = 安装方向修正 × (单圈编码器角 - 零位常量),折返到[-180,180)
    float cur_geo = wrap_180(dir * (steer->measure.angle_single_round - zero_offset));
    float err = wrap_180(target_geo - cur_geo);
    bool flip = false;
    if (err > 90.0f) // 就近原则:等效翻转180°后只需反向转不超过90°
    {
        err -= 180.0f;
        flip = true;
    }
    else if (err < -90.0f)
    {
        err += 180.0f;
        flip = true;
    }

    // 设定值 = 当前多圈角 + 方向修正后的最短路径角,角度环误差 = dir * err
    DJIMotorSetRef(steer, steer->measure.total_angle + dir * err);

    // 就近原则:翻转时驱动轮速度取反;cos³ 补偿:误差趋近90°时速度趋近0
    float cos_err = arm_cos_f32(err * DEGREE_2_RAD);
    cos_err = cos_err * cos_err * cos_err;
    if (flip)
        drive_speed = -drive_speed;
    DJIMotorSetRef(drive, drive_speed * cos_err);
}

/**
 * @brief 设定底盘电机参考值:固定方向轮直接给速度,舵向轮组走就近原则+cos³补偿
 *
 */
static void ChassisSetRef()
{
    DJIMotorSetRef(motor_left, vt_left);
    DJIMotorSetRef(motor_right, vt_right);
    // 舵轮驱动速度按半径比换算(舵轮60mm < 全向轮70mm)。注意必须乘在传给SteerModuleSetRef的
    // drive_speed上:drive电机的ref最终由它内部DJIMotorSetRef(drive, ...)设置,外层直接SetRef会被覆盖
    SteerModuleSetRef(steer_front, motor_front, steer_front_zero_offset, STEER_FRONT_DIR, angle_front, vt_front * STEER_DRIVE_SPEED_RATIO);
    SteerModuleSetRef(steer_back, motor_back, steer_back_zero_offset, STEER_BACK_DIR, angle_back, vt_back * STEER_DRIVE_SPEED_RATIO);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */


/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD


    // 获取发射模块数据并更新UI摩擦轮状态
    // SubGetMessage(shoot_sub, &shoot_cmd_recv);
    // ui_data.friction_mode = shoot_cmd_recv.friction_mode;

    //SetPowerLimit(referee_data->GameRobotState.chassis_power_limit);//设置功率限制（裁判系统）
    SetPowerLimit(99999);//设置功率限制（自己实际设定）—— 新版库无此函数,功率限制由 LimitChassisOutput() 实现
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止（电流为0）
        DJIMotorStop(motor_left);
        DJIMotorStop(motor_right);
        DJIMotorStop(motor_front);
        DJIMotorStop(motor_back);
        DJIMotorStop(steer_front);
        DJIMotorStop(steer_back);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_left);
        DJIMotorEnable(motor_right);
        DJIMotorEnable(motor_front);
        DJIMotorEnable(motor_back);
        DJIMotorEnable(steer_front);
        DJIMotorEnable(steer_back);
    }


    // 根据控制模式设定旋转速度
    static chassis_mode_e prev_chassis_mode = CHASSIS_ZERO_FORCE;

    // 退出跟随模式时清零积分,避免下次进入跟随模式时积分残留造成冲击
    if (prev_chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW &&
        chassis_cmd_recv.chassis_mode != CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        chassis_follow_PID.Iout = 0;
        chassis_follow_PID.ITerm = 0;
        chassis_follow_PID.Last_ITerm = 0;
    }
    prev_chassis_mode = chassis_cmd_recv.chassis_mode;

    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,以offset_angle为误差的PID输出为转速
        chassis_cmd_recv.wz = PIDCalculate(&chassis_follow_PID, chassis_cmd_recv.offset_angle, 0.0f);
        break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 1500;
        break;
    default: // FREE_MODE 摇杆直通,DEBUG_MODE 用 debug_data,wz 由 cmd 层直接给出
        break;
    }

    // 台架临时:云台未对齐/未接时offset_angle把vx/vy混叠,强制为0;装机比赛前删掉本行恢复云台系旋转
    chassis_cmd_recv.offset_angle = 0;

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
     cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
     sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 舵向零位:固定机械偏移常量(见文件顶部宏),与上电姿态无关
    steer_front_zero_offset = STEER_FRONT_ZERO_OFFSET;
    steer_back_zero_offset = STEER_BACK_ZERO_OFFSET;
    angle_target_front = steer_front_zero_offset + STEER_FRONT_DIR * angle_front; // 前舵向目标单圈角 = 零位 + DIR×几何目标角
    angle_target_back  = steer_back_zero_offset  + STEER_BACK_DIR  * angle_back;  // 后舵向目标单圈角(注意 DIR=-1,与几何角反号)

    // 设定底盘闭环参考值
    ChassisSetRef();
    steer_front_angle_ref = steer_front->motor_controller.pid_ref; // 前舵向角度环设定值(目标多圈角,度)
    steer_back_angle_ref  = steer_back->motor_controller.pid_ref;  // 后舵向角度环设定值(目标多圈角,度)

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();


    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    chassis_lb_v = motor_front->measure.speed_aps/6.0; // 前舵向驱动
    chassis_rb_v = motor_back->measure.speed_aps/6.0; // 后舵向驱动
    chassis_lf_v = motor_left->measure.speed_aps/6.0; // 左全向轮
    chassis_rf_v = motor_right->measure.speed_aps/6.0; // 右全向轮

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}