/*
 * @Descripttion: 超级电容通信协议模块
 *               CAN总线 1Mbps，主控与超级电容交互功率控制和状态反馈
 * @Protocol:
 *   0x301 就绪帧   电容->主控  1B  0xFF=就绪
 *   0x302 初始化帧 主控->电容  8B  [Vmax,Vmin,C,N,ESR,0,0,0]
 *   0x303 控制帧   主控->电容  8B  [Pchassis,Exceed,Mode,0,0,0,0,0]
 *   0x304 反馈帧   电容->主控  8B  [V_H,V_L,I_H,I_L,P_H,P_L,P_max,Mode]
 *   0x001 安全帧   电容->主控  8B  [cur,volt,temp,dp,can,dma,overall,0]
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:25:46
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"
#include "daemon.h"

/* 超级电容就绪状态 */
typedef enum
{
    CAP_NOT_READY = 0x00,  /* 未就绪 */
    CAP_READY     = 0xFF,  /* 就绪（0x301就绪帧） */
} CapReady;

/* 超级电容硬件类型 */
typedef enum
{
    CAP_TYPE_1 = 0x00,
    CAP_TYPE_2 = 0x01,
    CAP_TYPE_3 = 0x02,
} CapType;

/* 工作模式（0x303控制帧Byte2 / 0x304反馈帧Byte7） */
typedef enum
{
    MODE_SILENT = 0x00,  /* 静默模式：仅更新功率参数，不输出 */
    MODE_WORK   = 0x01,  /* 工作模式：正常功率输出 */
    MODE_CHARGE = 0x02,  /* 充电模式：对电容充电 */
} WorkMode;

/* 超限模式（0x303控制帧Byte1） */
typedef enum
{
    EXCEED_DISABLE = 0x00,  /* 关闭超限：电流限制10.5A */
    EXCEED_ENABLE  = 0x01,  /* 开启超限：电流限制14A */
} ExceedMode;

/* 安全保护等级（0x001安全帧） */
typedef enum
{
    SAFETY_NORMAL  = 0,  /* 正常 */
    SAFETY_WARNING = 1,  /* 警告 */
    SAFETY_FAULT   = 2,  /* 故障（强制静默模式） */
} SafetyLevel;

#pragma pack(1)

/* 超级电容状态信息（0x304反馈帧解析结果） */
typedef struct
{
    CapReady ready;           /* 就绪状态（来自0x301） */
    float    cap_voltage;     /* 电容电压 (V)        uint16大端×100 */
    float    cap_current;     /* 电容电流 (A) 负=放电 int16大端×100 */
    float    chassis_power;   /* 底盘功率 (W)        uint16大端×100 */
    float    max_power;       /* 可用功率上限 (W)    uint8×2 */
    WorkMode mode;            /* 当前工作模式 */
} SuperCap_Msg_s;

/* 安全帧信息（0x001安全帧解析结果） */
typedef struct
{
    SafetyLevel current_protect;     /* Byte0 电流保护: >10.5A警告 >14A故障 */
    SafetyLevel voltage_protect;     /* Byte1 电压保护: 越限 */
    SafetyLevel temp_protect;        /* Byte2 温度保护: >70°C警告 >80°C故障 */
    SafetyLevel power_diff_protect;  /* Byte3 功率差保护: >25W异常 */
    SafetyLevel can_error;           /* Byte4 CAN通信错误: BusOff或掉线 */
    SafetyLevel dma_error;           /* Byte5 DMA错误: 累计3次 */
    SafetyLevel overall_level;       /* Byte6 整体等级: 上述最严重等级 */
} SafetyFrame_s;

#pragma pack()

/* 超级电容CAN实例 */
typedef struct
{
    CANInstance *can_ins;      /* CAN实例指针 */
    SuperCap_Msg_s cap_msg;    /* 超级电容状态信息 */
    DaemonInstance *daemon;    /* 离线检测守护实例 */

    /* 低压自动充电状态(由模块内部维护,宿主不用碰) */
    float    auto_charge_voltage;   /* 触发阈值(V);<=0 表示关闭该功能 */
    uint8_t  auto_charge;           /* 自动充电锁存:1=已接管当前模式 */
    uint8_t  auto_charge_armed;     /* 是否允许触发:手动解除后需电压回升才重新武装 */
    WorkMode last_requested_mode;   /* 上一周期的请求模式,用于检测手动切换 */
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
    uint16_t offline_timeout;       /* 离线超时计数，0=使用默认值。0x304每500ms一帧，建议≥3帧超时 */
    float    auto_charge_voltage;   /* 低压自动充电阈值(V)，<=0 关闭该功能(默认)；例:6.0f */
} SuperCap_Init_Config_s;

/**
 * @brief 初始化超级电容模块
 * @note  根据tx_id区分：0x302=初始化帧发送实例，0x303=控制帧发送实例
 *        初始化帧实例会连续发送3次初始化帧（间隔0.3ms）
 * @param supercap_config 初始化配置（含CAN句柄、收发ID）
 * @return SuperCapInstance* 实例指针
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);

/**
 * @brief 发送初始化帧 0x302
 * @note  协议格式: [Vmax, Vmin, C_single, N_series, ESR, 0, 0, 0]
 * @param v_max     电容最大电压 (V)
 * @param v_min     电容最小电压 (V)
 * @param c_single  单个电容值 (F)
 * @param n_series  串联数量
 * @param esr       等效串联电阻 ×0.01 (Ω)
 */
void capinit(uint8_t v_max, uint8_t v_min, uint8_t c_single, uint8_t n_series, uint8_t esr);

/**
 * @brief 发送控制帧 0x303
 * @note  协议格式: [Pchassis, Exceed, Mode, 0x00, 0x00, 0x00, 0x00, 0x00]
 *        需持续以≥10Hz频率发送
 * @param p_chassis   底盘功率限制 (W)，范围0~210
 * @param exceed_en   超限模式
 * @param work_mode   工作模式
*/
void CapController_Send(uint8_t p_chassis, ExceedMode exceed_en, WorkMode work_mode);

/**
 * @brief 通用CAN发送接口
 * @param instance 超级电容实例
 * @param data     8字节发送数据
 */
void SuperCapSend(SuperCapInstance *instance, uint8_t *data);

/**
 * @brief 获取超级电容状态信息
 * @param instance 超级电容实例
 * @return SuperCap_Msg_s 状态信息（含电压、电流、功率、模式）
 */
SuperCap_Msg_s SuperCapGet(SuperCapInstance *instance);

/**
 * @brief 获取安全帧信息
 * @return SafetyFrame_s 安全状态（6项检测+整体等级）
 */
SafetyFrame_s GetSafetyFrame(void);

/**
 * @brief 超级电容统一控制逻辑
 * @note  安全故障时无条件强制静默，禁止电容输出功率
 *        正常时按遥控器指令选择工作模式（MODE_WORK / MODE_SILENT / MODE_CHARGE）
 *        需以≥10Hz频率周期性调用
 *
 *        低压自动充电(constructor 里配了 auto_charge_voltage > 0 时生效):
 *          电容电压跌破阈值 → 自动把模式改写成 MODE_CHARGE 并锁存，不会自动跳出；
 *          任何一次"requested_mode 发生变化"都视为手动操作 → 立即解除锁存,按传入模式执行。
 *          解除后会解除武装,要等电压回升到阈值以上才重新允许自动充电,避免反复横跳。
 *          电容离线时电压不可信,不参与判断。
 *
 * @param cap_ctrl               控制帧发送实例（tx_id=0x303初始化返回的实例）
 * @param chassis_power_limit    裁判系统下发的底盘功率限制值 (W)，范围0~210
 * @param requested_mode         遥控器指令中的目标工作模式(模块可能因低压自动充电改写它)
 * @param chassis_output_enable  底盘输出使能，0=裁判系统禁用底盘(死亡/超功)，强制静默
 */
void SuperCapControl(SuperCapInstance *cap_ctrl, uint8_t chassis_power_limit, WorkMode requested_mode, uint8_t chassis_output_enable);

/**
 * @brief 查询超级电容是否在线
 * @note  基于0x304反馈帧的超时判断，离线时电容已断开或CAN总线异常
 * @return uint8_t 1=在线，0=离线
 */
uint8_t SuperCapIsOnline(void);

#endif // !SUPER_CAP_H