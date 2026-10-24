#include "lzmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static LZMotorInstance *lz_motor_instance[LZ_MOTOR_CNT];
static osThreadId lz_task_handle[LZ_MOTOR_CNT];

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 发送非控制帧(使能/停止等),帧数据区全0,data区2填入ID的bit23-8
 * @param mode 通信类型,见LZ_Mode_e
 * @param data ID数据区2,使能/停止帧填主机CAN_ID
 */
static void LZMotorSetMode(LZ_Mode_e mode, uint16_t data, LZMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0, 8);
    motor->motor_can_instace->txconf.Identifier = LZ_MAKE_ID(mode, data, motor->motor_id);
    CANTransmit(motor->motor_can_instace, 1);
}

static void LZMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    LZMotorInstance *motor = (LZMotorInstance *)motor_can->id;
    LZ_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[0] << 8) | rxbuff[1]); // 反馈帧数据均为高字节在前
    measure->position = uint_to_float(tmp, LZ_P_MIN, LZ_P_MAX, 16);
    tmp = (uint16_t)((rxbuff[2] << 8) | rxbuff[3]);
    measure->velocity = uint_to_float(tmp, LZ_V_MIN, LZ_V_MAX, 16);
    tmp = (uint16_t)((rxbuff[4] << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, LZ_T_MIN, LZ_T_MAX, 16);
    measure->temperature = (float)(((uint16_t)((rxbuff[6] << 8) | rxbuff[7])) * 0.1f);

    // 反馈帧ID中携带模式状态(bit23-22)和故障位(bit21-16),故障出现时打印一次
    measure->mode_state = (uint8_t)((motor_can->rx_identifier >> 22) & 0x3);
    measure->fault = (uint16_t)((motor_can->rx_identifier >> 16) & 0x3F);
    if (measure->fault != motor->last_fault)
    {
        motor->last_fault = measure->fault;
        if (measure->fault)
            LOGWARNING("[lzmotor] Motor fault! can id [%d], fault bits [0x%02x]", motor->motor_id, measure->fault);
    }

    // 位置反馈为周期性循环±4π(1440°),与DM4310同理处理跨周期的圈数累计,
    // 防止位置环在边界处出现角度跳变(判断阈值取半个周期720°)
    measure->angle_single_round = measure->position * RAD_2_DEGREE; // -720°~720°
    if (measure->position - measure->last_position > 4.0f * PI)
        measure->total_round--;
    else if (measure->position - measure->last_position < -4.0f * PI)
        measure->total_round++;
    measure->total_angle = measure->total_round * 1440.0f + measure->angle_single_round;
    measure->speed_dps = measure->velocity * RAD_2_DEGREE; // 转换成°/s,与角度环输出单位保持一致
}

static void LZMotorLostCallback(void *motor_ptr)
{
    LZMotorInstance *motor = (LZMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instace->can_handle == &hcan1 ? 1 : 2;
    LOGWARNING("[lzmotor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_id);
}

LZMotorInstance *LZMotorInit(Motor_Init_Config_s *config, uint8_t motor_id, uint8_t master_id)
{
    LZMotorInstance *motor = (LZMotorInstance *)malloc(sizeof(LZMotorInstance));
    memset(motor, 0, sizeof(LZMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    motor->motor_id = motor_id;
    motor->master_id = master_id;

    // 按私有协议填写CAN注册配置(扩展帧):
    // 反馈帧ID=类型2|电机CAN_ID(bit15-8)|主机CAN_ID(bit7-0),bit23-16为模式状态和故障位,用掩码屏蔽后匹配
    config->can_init_config.tx_id = motor_id; // 实际发送ID按帧类型动态计算,此处仅用于注册去重
    config->can_init_config.rx_id = LZ_MAKE_ID(LZ_FEEDBACK, motor_id, master_id);
    config->can_init_config.id_type = 1; // 扩展帧
    config->can_init_config.rx_id_mask = 0x1F00FFFF;
    config->can_init_config.can_module_callback = LZMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = LZMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    // 电机上电默认处于运控模式,发送使能运行帧后即可接收运控控制帧
    LZMotorSetMode(LZ_ENABLE, master_id, motor);
    DWT_Delay(0.1);

    lz_motor_instance[idx++] = motor;
    return motor;
}

void LZMotorSetRef(LZMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void LZMotorEnable(LZMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void LZMotorStop(LZMotorInstance *motor) // 不使用停止帧是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void LZMotorOuterLoop(LZMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

/* 修改电机的实际闭环反馈来源,与DJIMotorChangeFeed行为一致 */
void LZMotorChangeFeed(LZMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOGERROR("[lzmotor] loop type error, check memory access and func param");
}

// 位置/速度/力矩串级PID,与DJI/DM电机控制结构保持一致:
// 角度环输出°/s,速度环输出Nm,最终以运控模式(Kp=Kd=0,位置/速度期望=0)下发纯力矩t_ff
// 若close_loop_type未启用任何闭环,则pid_ref直接作为力矩指令
void LZMotorTask(void const *argument)
{
    float pid_ref, pid_measure, set;
    LZMotorInstance *motor = (LZMotorInstance *)argument;
    LZ_Motor_Measure_s *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    uint16_t torque_uint;
    while (1)
    {
        pid_ref = motor->pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 计算位置环,只有启用位置环且外层闭环为位置时才会计算
        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时才会计算
        if ((setting->close_loop_type & SPEED_LOOP) && (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor->speed_feedforward_ptr;

            if (setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_speed_feedback_ptr;
            else
                pid_measure = measure->speed_dps; // MOTOR_FEED
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor->speed_PID, pid_measure, pid_ref);
        }

        // 计算力矩环,反馈只有电机自身传感器(换算后的力矩),目前只要启用了力矩环就计算
        if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor->current_feedforward_ptr;
        if (setting->close_loop_type & CURRENT_LOOP)
            pid_ref = PIDCalculate(&motor->current_PID, measure->torque, pid_ref);

        if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 最终输出限幅后作为力矩指令下发
        set = pid_ref;
        LIMIT_MIN_MAX(set, LZ_T_MIN, LZ_T_MAX);
        if (motor->stop_flag == MOTOR_STOP)
            set = 0;

        // 运控模式控制帧:力矩位于ID的data区(bit23-8),
        // 数据区为位置/速度期望和Kp/Kd(高字节在前),全部置0时电机输出纯力矩t_ff
        torque_uint = float_to_uint(set, LZ_T_MIN, LZ_T_MAX, 16);
        motor->motor_can_instace->txconf.Identifier = LZ_MAKE_ID(LZ_CONTROL, torque_uint, motor->motor_id);
        memset(motor->motor_can_instace->tx_buff, 0, 8);

        CANTransmit(motor->motor_can_instace, 1);

        osDelay(2);
    }
}

void LZMotorControlInit()
{
    char lz_task_name[5] = "lz";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char lz_id_buff[2] = {0};
        __itoa(i, lz_id_buff, 10);
        strcat(lz_task_name, lz_id_buff);
        osThreadDef(lz_task_name, LZMotorTask, osPriorityNormal, 0, 256); // 串级PID计算需要较大的栈空间
        lz_task_handle[i] = osThreadCreate(osThread(lz_task_name), lz_motor_instance[i]);
    }
}
