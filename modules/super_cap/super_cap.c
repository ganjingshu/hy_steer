/*
 * @Descripttion: 超级电容通信协议实现
 *               通信流程: 上电->电容发0x301(0xFF)->主控发0x302->循环交互0x303/0x304
 * @Protocol:
 *   0x301 就绪帧   电容->主控  1B  [ready]
 *   0x302 初始化帧 主控->电容  8B  [Vmax,Vmin,C,N,ESR,0,0,0]
 *   0x303 控制帧   主控->电容  8B  [Pchassis,Exceed,Mode,0,0,0,0,0]
 *   0x304 反馈帧   电容->主控  8B  [V_cap×100,I_cap×100,P_chassis×100,P_max×2,Mode]
 *   0x001 安全帧   电容->主控  8B  [cur,volt,temp,dp,can,dma,overall,0]
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:29:49
 */
#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "daemon.h"

/* 0x302初始化帧发送实例（收发就绪帧和初始化帧） */
static SuperCapInstance *super_cap_instance = NULL;
/* 0x303控制帧发送实例（收发控制帧、反馈帧） */
static SuperCapInstance *super_cap_instance1 = NULL;
/* 0x001安全帧接收实例（仅接收，不发送） */
static SuperCapInstance *super_cap_instance2 = NULL;
/* 安全帧数据（0x001） */
static SafetyFrame_s safety_frame = {
    .overall_level = SAFETY_NORMAL,  /* 默认正常，收到0x001后更新 */
};
/* 超电在线状态标志，用于检测离线→上线的状态转换以触发重新初始化 */
static uint8_t cap_online = 0;
/* 离线检测守护实例 */
static DaemonInstance *super_cap_daemon = NULL;

/**
 * @brief CAN接收回调函数，解析超级电容发来的协议帧
 * @param _instance CAN实例指针
 */
static void SuperCapRxCallback(CANInstance *_instance)
{
    uint8_t *rxbuff = _instance->rx_buff;

    /* 0x301 就绪帧：电容上电/重启后发送0xFF，始终回应初始化帧以支持重复上线 */
    if (_instance->rx_id == 0x301)
    {
        super_cap_instance->cap_msg.ready = rxbuff[0];
        if (rxbuff[0] == 0xFF)
        {
            capinit(24, 5, 50, 11, 17);
            cap_online = 1;
        }
    }
    /* 0x304 反馈帧：电容每500ms发送一次状态 */
    else if (_instance->rx_id == 0x304)
    {
        SuperCap_Msg_s *msg = &super_cap_instance1->cap_msg;

        /* Byte0-1: V_cap 电容电压 uint16大端 ×100 (V) */
        uint16_t voltage_val = (uint16_t)(rxbuff[0] << 8 | rxbuff[1]);
        msg->cap_voltage = (float)voltage_val / 100.0f;

        /* Byte2-3: I_cap 电容电流 int16大端 ×100 (A) 负值=放电 */
        int16_t current_val = (int16_t)(rxbuff[2] << 8 | rxbuff[3]);
        msg->cap_current = (float)current_val / 100.0f;

        /* Byte4-5: P_chassis 底盘功率 uint16大端 ×100 (W) */
        uint16_t power_val = (uint16_t)(rxbuff[4] << 8 | rxbuff[5]);
        msg->chassis_power = (float)power_val / 100.0f;

        /* Byte6: P_max 可用功率上限 uint8 ×2 (W) 精度2W 范围0~510W */
        msg->max_power = (float)(rxbuff[6] * 2);

        /* Byte7: Mode 当前工作模式 0=Silent 1=Work 2=Charge */
        msg->mode = (WorkMode)rxbuff[7];

        if (super_cap_daemon != NULL)
            DaemonReload(super_cap_daemon);
    }
    /* 0x001 安全帧：安全状态变化时发送 */
    else if (_instance->rx_id == 0x001)
    {
        safety_frame.current_protect     = (SafetyLevel)rxbuff[0]; /* Byte0 电流保护 */
        safety_frame.voltage_protect     = (SafetyLevel)rxbuff[1]; /* Byte1 电压保护 */
        safety_frame.temp_protect        = (SafetyLevel)rxbuff[2]; /* Byte2 温度保护 */
        safety_frame.power_diff_protect  = (SafetyLevel)rxbuff[3]; /* Byte3 功率差保护 */
        safety_frame.can_error           = (SafetyLevel)rxbuff[4]; /* Byte4 CAN通信错误 */
        safety_frame.dma_error           = (SafetyLevel)rxbuff[5]; /* Byte5 DMA错误 */
        safety_frame.overall_level       = (SafetyLevel)rxbuff[6]; /* Byte6 整体安全等级 */
    }
}

/**
 * @brief 初始化超级电容模块
 * @note  tx_id=0x302时创建初始化帧发送实例（收到0x301后回调发送一次初始化帧）
 *        tx_id=0x303时创建控制帧发送实例
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    if(supercap_config->can_config.tx_id==0x302){
    super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance, 0, sizeof(SuperCapInstance));

    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);

    return super_cap_instance;
}
    if(supercap_config->can_config.tx_id==0x303){
    super_cap_instance1 = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance1, 0, sizeof(SuperCapInstance));

    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance1->can_ins = CANRegister(&supercap_config->can_config);

    Daemon_Init_Config_s daemon_conf = {
        .owner_id = super_cap_instance1,
        .reload_count = supercap_config->offline_timeout > 0 ? supercap_config->offline_timeout : 150,
        .init_count = supercap_config->offline_timeout > 0 ? supercap_config->offline_timeout : 150,
    };
    super_cap_daemon = DaemonRegister(&daemon_conf);
    super_cap_instance1->daemon = super_cap_daemon;

    /* 低压自动充电状态初始化(只有控制帧实例需要) */
    super_cap_instance1->auto_charge_voltage = supercap_config->auto_charge_voltage;
    super_cap_instance1->auto_charge = 0;
    super_cap_instance1->auto_charge_armed = 1;
    super_cap_instance1->last_requested_mode = MODE_SILENT;

    return super_cap_instance1;
}
    if(supercap_config->can_config.tx_id==0x001){
    super_cap_instance2 = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance2, 0, sizeof(SuperCapInstance));

    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance2->can_ins = CANRegister(&supercap_config->can_config);
    return super_cap_instance2;
}
    return NULL;
}

/**
 * @brief 发送初始化帧 0x302
 * @note  协议格式: [Vmax, Vmin, C_single, N_series, ESR, 0, 0, 0]
 *        示例: [24, 6, 50, 11, 17, 0, 0, 0] = 11个50F串联 ESR=0.17Ω 24V电容组
 */
void capinit(uint8_t v_max, uint8_t v_min, uint8_t c_single, uint8_t n_series, uint8_t esr)
{
    if (super_cap_instance == NULL || super_cap_instance->can_ins == NULL)
        return;

    memset(super_cap_instance->can_ins->tx_buff, 0, 8);
    super_cap_instance->can_ins->tx_buff[0] = v_max;      /* Byte0: 电容最大电压 (V) */
    super_cap_instance->can_ins->tx_buff[1] = v_min;      /* Byte1: 电容最小电压 (V) */
    super_cap_instance->can_ins->tx_buff[2] = c_single;   /* Byte2: 单个电容值 (F) */
    super_cap_instance->can_ins->tx_buff[3] = n_series;   /* Byte3: 串联数量 */
    super_cap_instance->can_ins->tx_buff[4] = esr;        /* Byte4: ESR ×0.01 (Ω) */
    /* Byte5-7: 保留 填0 */
    CANTransmit(super_cap_instance->can_ins, 1);
}

/**
 * @brief 发送控制帧 0x303
 * @note  协议格式: [Pchassis, Exceed, Mode, PowerLimit, 0x00, 0x00, 0x00, 0x00]
 *        需持续以≥10Hz频率发送
 */
void CapController_Send(uint8_t p_chassis, ExceedMode exceed_en, WorkMode work_mode)
{
    if (super_cap_instance1 == NULL || super_cap_instance1->can_ins == NULL)
        return;

    memset(super_cap_instance1->can_ins->tx_buff, 0, 8);
    super_cap_instance1->can_ins->tx_buff[0] = p_chassis;   /* Byte0: 底盘功率限制 0~210W */
    super_cap_instance1->can_ins->tx_buff[1] = exceed_en;   /* Byte1: 超限模式 0=10.5A 1=14A */
    super_cap_instance1->can_ins->tx_buff[2] = work_mode;   /* Byte2: 工作模式 */
  
    /* Byte4-7: 保留 填0x00 */
    CANTransmit(super_cap_instance1->can_ins, 1);
}

/**
 * @brief 通用CAN发送接口
 */
void SuperCapSend(SuperCapInstance *instance, uint8_t *data)
{
    memcpy(instance->can_ins->tx_buff, data, 8);
    CANTransmit(instance->can_ins,1);
}

/**
 * @brief 获取超级电容状态信息
 */
SuperCap_Msg_s SuperCapGet(SuperCapInstance *instance)
{
    return instance->cap_msg;
}

/**
 * @brief 获取安全帧信息
 * @note  使用示例:
 *        SafetyFrame_s sf = GetSafetyFrame();
 *        if (sf.overall_level == SAFETY_FAULT) { 系统强制静默; }
 */
SafetyFrame_s GetSafetyFrame(void)
{
    return safety_frame;
}

/**
 * @brief 超级电容统一控制逻辑
 * @note  安全故障时无条件强制静默，禁止电容输出功率
 *        正常时按遥控器指令选择工作模式
 */
void SuperCapControl(SuperCapInstance *cap_ctrl, uint8_t chassis_power_limit, WorkMode requested_mode, uint8_t chassis_output_enable)
{
    SafetyFrame_s sf = GetSafetyFrame();
    uint8_t p_chassis;

    /* 参数校验 */
    if (cap_ctrl == NULL)
        return;

    /* ===== 低压自动充电(宿主只传"用户想要的模式",这里可能把它改写成 MODE_CHARGE) ===== */

    /* 请求模式发生变化 → 视为一次手动操作,立即解除自动充电锁存 */
    if (requested_mode != cap_ctrl->last_requested_mode)
    {
        cap_ctrl->auto_charge = 0;
        cap_ctrl->auto_charge_armed = 0;   /* 需等电压回升过阈值才重新武装 */
    }
    cap_ctrl->last_requested_mode = requested_mode;

    /* 离线时电压是陈旧值,不可信,所以只在在线时判断 */
    if (cap_ctrl->auto_charge_voltage > 0.0f && SuperCapIsOnline())
    {
        SuperCap_Msg_s cap_msg = SuperCapGet(cap_ctrl);

        if (cap_msg.cap_voltage >= cap_ctrl->auto_charge_voltage)
            cap_ctrl->auto_charge_armed = 1;   /* 电压恢复 → 重新武装 */
        else if (cap_ctrl->auto_charge_armed)
            cap_ctrl->auto_charge = 1;         /* 跌破阈值且已武装 → 接管 */
    }

    if (cap_ctrl->auto_charge)
        requested_mode = MODE_CHARGE;

    /* ===== 在线状态机:离线 -> 在线时重新握手 =====
       0x301 只在电容板(重新)上电时发一次,以下两种情况都会错过它:
         1. CAN 瞬时掉线后恢复(板子没重启,不会再发 0x301)
         2. 电容板比主控先上电(0x301 发过了,主控还没准备好)
       只靠 0x301 置 ready 会让超电永久失效,只能给电容板断电重启。这里用 daemon(0x304)
       的在线边沿补上这个缺口:只要重新听到 0x304,就认为板子活着,重发一次 0x302
       并重新置位 ready。 */
    uint8_t online = (super_cap_daemon != NULL) && DaemonIsOnline(super_cap_daemon);

    if (!online)
    {
        if (cap_online)                        /* 在线 -> 离线 */
        {
            cap_online = 0;
            super_cap_instance->cap_msg.ready = CAP_NOT_READY;
            CapController_Send(0, EXCEED_DISABLE, MODE_SILENT);
        }
        return;
    }

    if (!cap_online)                           /* 离线 -> 在线:重新握手 */
    {
        cap_online = 1;
        super_cap_instance->cap_msg.ready = CAP_READY;
        capinit(24, 5, 50, 11, 17);            /* TODO: 参数应提到配置结构体里 */
        return;                                /* 本周期先不发控制帧,下一周期开始正常控制 */
    }

    /* 板子明确报未就绪时仍然要等(0x301 可能上报 0x00) */
    if (super_cap_instance->cap_msg.ready != CAP_READY)
        return;

    /* 裁判系统禁用底盘输出（机器人死亡或超功率限制），强制静默 */
    if (!chassis_output_enable)
    {
        CapController_Send(0, EXCEED_DISABLE, MODE_SILENT);
        return;
    }

    /* 防止 uint8_t 减法下溢 */
    p_chassis = (chassis_power_limit > 10) ? (uint8_t)(chassis_power_limit ) : 0;

    if (sf.overall_level == SAFETY_FAULT)
    {
        /* 安全故障：无条件强制静默，禁止电容输出功率 */
        CapController_Send(p_chassis, EXCEED_DISABLE, MODE_SILENT);
        return;
    }

    /* 正常：按遥控器指令选择工作模式 */
    switch (requested_mode)
    {
        case MODE_WORK:
            /* 工作模式：电容对外输出功率 */
            CapController_Send(p_chassis, EXCEED_DISABLE, MODE_WORK);
            break;
        case MODE_SILENT:
            /* 静默模式：仅更新参数，电容不输出 */
            CapController_Send(p_chassis, EXCEED_DISABLE, MODE_SILENT);
            break;
        case MODE_CHARGE:
            /* 充电模式：电容吸收功率进行充电 */
            CapController_Send(p_chassis, EXCEED_DISABLE, MODE_CHARGE);
            break;
        default:
            /* 未知模式：安全起见强制静默 */
            CapController_Send(p_chassis, EXCEED_DISABLE, MODE_SILENT);
            break;
    }

    /* 模式不匹配超时检测：电容反馈与指令持续不一致超过阈值则强制静默 */
    /* 0x304反馈帧每500ms更新一次，15周期≈1.5s，足够电容完成模式切换 */
    // if (super_cap_instance1->cap_msg.mode != requested_mode)
    // {
    //     mode_mismatch_cnt++;
    //     if (mode_mismatch_cnt > 15)
    //     {
    //         CapController_Send(p_chassis, EXCEED_DISABLE, MODE_SILENT);
    //     }
    // }
    // else
    // {
    //     mode_mismatch_cnt = 0;
    // }
}

/**
 * @brief 查询超级电容是否在线
 * @note  基于0x304反馈帧的超时判断，超过offline_timeout个周期未收到则判定离线
 * @return uint8_t 1=在线，0=离线
 */
uint8_t SuperCapIsOnline(void)
{
    if (super_cap_daemon == NULL)
        return 0;
    return DaemonIsOnline(super_cap_daemon);
}