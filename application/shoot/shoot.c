#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "can_comm.h"

#include <math.h> // 用于fabsf函数

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r;
#ifndef GIMBAL_BOARD // 双板时拨盘在底盘板注册(见chassis.c),云台板不注册,经CANComm转发控制
static DJIMotorInstance *loader; // 拨盘电机
#endif
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

uint8_t loader_is_reversing = 0; // 卡弹反转标志,由发射模块在检测到卡弹后置1,cmd层据此选择LOAD_stovepipe;当前未接入卡弹检测

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

/* 拨盘卡弹检测与反拨:电流超阈值持续0.1s才触发,反拨一个弹位后恢复 */
#define LOADER_JAM_CURRENT 7000     // 拨盘2006电流阈值(原始值),超过判为卡弹
#define LOADER_JAM_DWELL_MS 100     // 电流超阈值需持续该时长才判卡弹(ms),滤掉瞬时尖峰
#define LOADER_REVERSE_ARRIVE 5.0f // 反拨到位判定:与目标角误差(°)
#define LOADER_REVERSE_TIMEOUT 1200 // 反拨超时(ms);角度环上限500°/s反完一个弹位约3.5s,须留够余量
static uint8_t loader_reversing = 0;     // 反拨状态机标志
static uint8_t loader_jam_armed = 1;     // 触发使能:电流回落到阈值以下才重新武装
static uint32_t loader_reverse_start = 0; // 反拨开始时间戳
static uint32_t loader_overcurrent_start = 0; // 超阈开始时刻(0=未超阈)
static float loader_reverse_target = 0;   // 反拨目标角(触发时锁存,回退一个弹位)

#ifdef GIMBAL_BOARD
static CANCommInstance *loader_comm;           // 拨盘CANComm:云台板发控制量(0x313),收底盘板回传的角度/电流(0x314)
static Loader_Ctrl_Forward_s loader_ctrl_send; // 每周期发送给底盘板的拨盘控制量
static Loader_Feed_Forward_s loader_feed_recv; // 底盘板回传的拨盘角度/电流
#endif

/* 统一单板/双板的拨盘操作接口:单板直接操作电机;双板(GIMBAL_BOARD)更新CANComm发送结构,ShootTask末尾统一发送 */
#ifdef GIMBAL_BOARD
static void LoaderStop(void)                     { loader_ctrl_send.enable = 0; }
static void LoaderEnable(void)                   { loader_ctrl_send.enable = 1; }
static void LoaderSetLoop(Closeloop_Type_e loop) { loader_ctrl_send.outer_loop = (uint8_t)loop; }
static void LoaderSetRef(float ref)              { loader_ctrl_send.ref = ref; }
static float LoaderAngle(void)                   { return loader_feed_recv.total_angle; }
static float LoaderCurrent(void)                 { return loader_feed_recv.real_current; }
#else
static void LoaderStop(void)                     { DJIMotorStop(loader); }
static void LoaderEnable(void)                   { DJIMotorEnable(loader); }
static void LoaderSetLoop(Closeloop_Type_e loop) { DJIMotorOuterLoop(loader, loop); }
static void LoaderSetRef(float ref)              { DJIMotorSetRef(loader, ref); }
static float LoaderAngle(void)                   { return loader->measure.total_angle; }
static float LoaderCurrent(void)                 { return loader->measure.real_current; }
#endif

/**
 * @brief 拨盘卡弹反拨:监测拨盘2006电流,超阈值持续0.1s才触发反拨一个弹位,
 *        到位或超时后恢复load_mode设定;反拨期间置loader_is_reversing供cmd层查询
 * @note  在ShootTask中每周期调用,反拨动作覆盖本周期load_mode对拨盘的设定
 */
static void LoaderJamReverse()
{
    // 急停/关发射时不参与,并复位反拨状态
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        loader_reversing = 0;
        loader_is_reversing = 0;
        loader_overcurrent_start = 0;
        return;
    }

    if (loader_reversing)
    {
        // 反拨中:角度环继续回退到锁存的目标角,到位或超时结束
        LoaderSetLoop(ANGLE_LOOP);
        LoaderSetRef(loader_reverse_target);
        if (fabsf(LoaderAngle() - loader_reverse_target) < LOADER_REVERSE_ARRIVE ||
            DWT_GetTimeline_ms() - loader_reverse_start > LOADER_REVERSE_TIMEOUT)
        {
            loader_reversing = 0;      // 反拨结束,下一周期恢复load_mode设定
            loader_is_reversing = 0;
        }
        return;
    }

    // 未在反拨:电流回落到阈值以下重新武装并清零超阈计时,防反拨-恢复-再反拨的抽搐
    if (LoaderCurrent() <= LOADER_JAM_CURRENT)
    {
        loader_jam_armed = 1;
        loader_overcurrent_start = 0;
    }

    // 已武装且电流超阈值:持续0.1s才判为卡弹,触发反拨并锁存回退一个弹位的目标角
    if (loader_jam_armed && LoaderCurrent() > LOADER_JAM_CURRENT)
    {
        if (loader_overcurrent_start == 0)
            loader_overcurrent_start = DWT_GetTimeline_ms(); // 记录超阈开始时刻
        if (DWT_GetTimeline_ms() - loader_overcurrent_start >= LOADER_JAM_DWELL_MS)
        {
            loader_jam_armed = 0; // 一次卡弹只反拨一次
            loader_reversing = 1;
            loader_reverse_start = DWT_GetTimeline_ms();
            loader_reverse_target = LoaderAngle() - ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER;
            loader_is_reversing = 1;
            LoaderSetLoop(ANGLE_LOOP);
            LoaderSetRef(loader_reverse_target);
        }
    }
}

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508
    };
    
    friction_config.can_init_config.tx_id = 4; // 左摩擦轮,根据实际情况修改txid
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 5; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

#ifndef GIMBAL_BOARD // 双板时拨盘在底盘板注册(见chassis.c),云台板经CANComm转发,不注册
    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 8,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 35, // 10
                .Ki = 10,
                .Kd = 0,
                .MaxOut = 7000, // 角度环输出上限=反拨/单发速度上限(°/s):2000时一个弹位约0.9s,嫌慢可再调大
                .Improve = PID_Integral_Limit | PID_DerivativeFilter,
                .IntegralLimit = 20000,
            },
            .speed_PID = {
                .Kp = 5, // 10
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 20000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 7500,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP, // 必须含ANGLE_LOOP:否则位置环不计算,角度目标直接当速度参考,反拨/单发会全速疯转
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);
#endif // !GIMBAL_BOARD

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s loader_comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x313, // 发拨盘控制量给底盘板
            .rx_id = 0x314, // 收底盘板回传的拨盘角度/电流
        },
        .recv_data_len = sizeof(Loader_Feed_Forward_s),
        .send_data_len = sizeof(Loader_Ctrl_Forward_s),
    };
    loader_comm = CANCommInit(&loader_comm_conf);
#endif // GIMBAL_BOARD
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

#ifdef GIMBAL_BOARD
    if (loader_comm->update_flag) // 更新底盘板回传的拨盘角度/电流
        loader_feed_recv = *(Loader_Feed_Forward_s *)CANCommGet(loader_comm);
#endif

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        LoaderStop();
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        LoaderEnable();
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        LoaderSetLoop(SPEED_LOOP); // 切换到速度环
        LoaderSetRef(0);           // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
        LoaderSetLoop(ANGLE_LOOP);                                                          // 切换到角度环
        LoaderSetRef(LoaderAngle() + ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER);     // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        dead_time = 150;                                                                    // 完成1发弹丸发射的时间
        break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        LoaderSetLoop(ANGLE_LOOP);                                                              // 切换到角度环
        LoaderSetRef(LoaderAngle() + 3 * ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER);     // 增加3发
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 300;                                                                        // 完成3发弹丸发射的时间
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        LoaderSetLoop(SPEED_LOOP);
        LoaderSetRef(shoot_cmd_recv.shoot_rate * ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)[°/s]
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
    // todo 后面再加
        LoaderSetLoop(ANGLE_LOOP);
        LoaderSetRef(LoaderAngle() - ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER);
        // ...
        break;
    // 卡弹反拨,行为与LOAD_REVERSE一致,由cmd层根据loader_is_reversing标志触发
    case LOAD_stovepipe:
        LoaderSetLoop(ANGLE_LOOP);
        LoaderSetRef(LoaderAngle() - ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER);
        // ...
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 卡弹检测:拨盘电流超阈值自动反拨一个弹位(见LoaderJamReverse),反拨动作覆盖本周期load_mode设定
    LoaderJamReverse();

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        // TODO 数据待测量
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            DJIMotorSetRef(friction_l, 37000);
            DJIMotorSetRef(friction_r, 37000);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // TODO 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    shoot_feedback_data.loader_angle = LoaderAngle();
    shoot_feedback_data.loader_current = LoaderCurrent();
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);

#ifdef GIMBAL_BOARD
    CANCommSend(loader_comm, (uint8_t *)&loader_ctrl_send); // 每周期发送一次拨盘控制量
#endif
}