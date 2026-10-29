# super_cap —— 超级电容通信模块

## 1. 模块职责

通过 **CAN 1Mbps** 与超级电容板通信:

- **下行**:下发底盘功率限制、工作模式
- **上行**:接收电容电压/电流/底盘功率/可用功率上限/工作模式,以及 6 项安全保护等级

本模块**只负责协议解析和状态机**,不负责功率分配。功率怎么用、限多少,由上层
(`power_control` / `chassis`)决定。

**依赖**:`bsp_can.h`(CAN 收发)、`daemon.h`(离线检测)。不依赖遥控器、裁判系统、功率控制。

---

## 2. 通信协议

### 2.1 报文总表

| CAN ID | 方向 | 发送时机 | 长度 | 内容 |
|---|---|---|---|---|
| `0x301` | 电容 → 主控 | 电容上电/重启时发一次 | 1B | 就绪标志 |
| `0x302` | 主控 → 电容 | 收到 0x301 后发一次(重连时也会重发) | 8B | 电容组参数 |
| `0x303` | 主控 → 电容 | **持续,≥10Hz** | 8B | 功率限制 + 超限模式 + 工作模式 |
| `0x304` | 电容 → 主控 | **每 500ms 一帧** | 8B | 电容状态反馈 |
| `0x001` | 电容 → 主控 | 安全状态变化时 | 8B | 6 项保护等级 |

### 2.2 字段编码

#### 0x301 就绪帧(电容 → 主控)

| 字节 | 字段 | 说明 |
|---|---|---|
| 0 | ready | `0xFF` = 就绪;其他 = 未就绪 |

#### 0x302 初始化帧(主控 → 电容)

| 字节 | 字段 | 单位/缩放 |
|---|---|---|
| 0 | `Vmax` | 电容最大电压 (V) |
| 1 | `Vmin` | 电容最小电压 (V) |
| 2 | `C_single` | 单体电容值 (F) |
| 3 | `N_series` | 串联数量 |
| 4 | `ESR` | 等效串联电阻 ×0.01 (Ω) |
| 5~7 | 保留 | 填 0 |

本工程当前使用:`[24, 5, 50, 11, 17]` → 24V 上限、5V 下限、11 串、ESR 0.17Ω。

#### 0x303 控制帧(主控 → 电容)

| 字节 | 字段 | 说明 |
|---|---|---|
| 0 | `Pchassis` | 底盘功率限制 (W),范围 **0~210** |
| 1 | `Exceed` | `0`=电流限制 10.5A,`1`=14A |
| 2 | `Mode` | `0`=静默,`1`=工作,`2`=充电 |
| 3~7 | 保留 | 填 0 |

#### 0x304 反馈帧(电容 → 主控)

| 字节 | 字段 | 编码 | 说明 |
|---|---|---|---|
| 0~1 | `V_cap` | `uint16` 大端 ×100 | 电容电压 (V) |
| 2~3 | `I_cap` | `int16` 大端 ×100 | 电容电流 (A),**负值 = 放电** |
| 4~5 | `P_chassis` | `uint16` 大端 ×100 | 底盘功率 (W) |
| 6 | `P_max` | `uint8` ×2 | 可用功率上限 (W),精度 2W,范围 0~510 |
| 7 | `Mode` | `0/1/2` | 电容**当前实际**工作模式 |

#### 0x001 安全帧(电容 → 主控)

| 字节 | 字段 | 说明 |
|---|---|---|
| 0 | `current_protect` | 电流保护:>10.5A 警告,>14A 故障 |
| 1 | `voltage_protect` | 电压保护 |
| 2 | `temp_protect` | 温度保护:>70°C 警告,>80°C 故障 |
| 3 | `power_diff_protect` | 功率差保护:>25W 异常 |
| 4 | `can_error` | CAN 通信错误(BusOff 或掉线) |
| 5 | `dma_error` | DMA 错误(累计 3 次) |
| 6 | `overall_level` | 以上各项中最严重的等级 |
| 7 | 保留 | — |

等级取值:`0`=正常,`1`=警告,`2`=故障。

### 2.3 通信时序

```
电容上电
   │  0x301(0xFF)
   ▼
主控 ── 0x302(电容参数)──▶ 电容
   │
   │  循环:主控 0x303(≥10Hz) ──▶ 电容
   │        电容 0x304(每500ms) ──▶ 主控
   │
   └─ 异常时:电容 0x001(安全等级) ──▶ 主控
```

---

## 3. 快速上手

### 3.1 初始化

**必须注册三个实例,且顺序为 `0x302 → 0x303 → 0x001`。**

```c
SuperCap_Init_Config_s cfg = {
    .can_config = {
        .can_handle = &hcan1,
        .tx_id      = 0x302,   /* 初始化帧 */
        .rx_id      = 0x301,   /* 就绪帧   */
    },
    .offline_timeout     = 150,   /* DaemonTask 周期数,100Hz 下 = 1.5s */
    .auto_charge_voltage = 6.0f,  /* 低压自动充电阈值,填 0 关闭 */
};

SuperCapInit(&cfg);                     /* ① 0x302 实例(模块内部持有) */

cfg.can_config.tx_id = 0x303;           /* 控制帧 */
cfg.can_config.rx_id = 0x304;           /* 反馈帧 */
SuperCapInstance *cap = SuperCapInit(&cfg);   /* ② 0x303 实例,后续都用这个指针 */

cfg.can_config.tx_id = 0x001;           /* 安全帧(只收) */
cfg.can_config.rx_id = 0x001;
SuperCapInit(&cfg);                     /* ③ 0x001 实例 */
```

> **顺序很重要**:`0x301` 回调内部会解引用 `0x302` 实例,如果先注册其他实例再收到
> `0x301`,会访问空指针。目前代码未做防护,务必按此顺序初始化。

### 3.2 周期调用

```c
/* ≥10Hz,建议 100~200Hz。一般放在底盘/功率控制任务里 */
SuperCapControl(cap, chassis_power_limit, requested_mode, chassis_output_enable);
```

| 参数 | 说明 |
|---|---|
| `cap` | `tx_id = 0x303` 初始化返回的实例 |
| `chassis_power_limit` | 裁判系统下发的底盘功率限制 (W),范围 0~210 |
| `requested_mode` | 期望的工作模式,**模块可能因低压自动充电改写它** |
| `chassis_output_enable` | `0` = 裁判禁用底盘(死亡/超功),强制静默 |

### 3.3 读取反馈

```c
if (SuperCapIsOnline())
{
    SuperCap_Msg_s m = SuperCapGet(cap);

    /* m.cap_voltage  电容电压 (V) */
    /* m.cap_current  电容电流 (A),负=放电 */
    /* m.chassis_power底盘功率 (W) */
    /* m.max_power    可用功率上限 (W) */
    /* m.mode         电容实际工作模式 */
}
```

> 注意:`SuperCapGet()` 返回的是**结构体副本**,每次调用都会拷贝。

---

## 4. 内部机制

### 4.1 在线状态机(断线重连)

离线检测基于 `0x304` 帧的 daemon:`offline_timeout` 个 `DaemonTask` 周期内没收到就算离线
(默认 150,100Hz 下 = 1.5s = 连丢 3 帧)。

```
        ┌──────────── 正常控制 ────────────┐
        │                                  │
   离线→在线                          在线→离线
  (重发0x302,置ready)              (清ready,发SILENT)
        │                                  │
        └──────────── 静默等待 ─────────────┘
```

**为什么要处理"离线 → 在线"边沿**:`0x301` 只在电容板(重新)上电时发一次。以下两种情况
会让 `ready` 永远回不到 `0xFF`,导致超电永久失效:

1. **CAN 瞬时掉线后恢复** —— 板子没重启,不会再发 `0x301`
2. **电容板先于主控上电** —— `0x301` 发过了,主控还没准备好接收

因此模块在重新听到 `0x304` 时,会主动重发一次 `0x302` 并把 `ready` 置回,完成重新握手。

### 4.2 低压自动充电

配置 `auto_charge_voltage > 0` 时启用(`<= 0` 则完全关闭):

| 条件 | 行为 |
|---|---|
| 电压跌破阈值 | 锁存,把模式改写成 `MODE_CHARGE` |
| `requested_mode` **发生变化** | 视为一次手动操作,立即解除锁存 |
| 手动解除后 | "解除武装",等电压**回升到阈值以上**才重新允许自动充电 |

- 锁存**不会自动跳出**,必须有手动操作
- 判断电压时要求电容在线,离线时电压是陈旧值,不采信
- 目的:电容没电时不再让它对外输出,同时避免刚切回工作就被抽干、两种模式反复横跳

### 4.3 `SuperCapControl()` 的检查顺序

顺序是有意义的,不要随意调整:

```
1. cap_ctrl == NULL                    → 直接返回
2. 低压自动充电状态更新                 → 可能改写 requested_mode
3. daemon 离线?                        → (离线边沿时发一次 SILENT) 返回
4. cap_online == 0?                    → 重新握手(发 0x302)返回
5. ready != CAP_READY?                 → 返回(板子自称未就绪)
6. !chassis_output_enable?             → 发 SILENT 返回
7. 安全等级 == SAFETY_FAULT?           → 发 SILENT 返回
8. 按 requested_mode 发 0x303
```

### 4.4 两个容易混淆的变量

| 变量 | 含义 | 谁维护 |
|---|---|---|
| `SuperCapIsOnline()` | **能不能听到它**(0x304 的 daemon) | 模块内部 |
| `cap_msg.ready` | **它有没有宣布就绪**(0x301) | 模块内部的在线状态机 |

---

## 5. 公开 API

| 函数 | 说明 |
|---|---|
| `SuperCapInit(&cfg)` | 按 `tx_id` 创建 0x302 / 0x303 / 0x001 实例 |
| `SuperCapControl(cap, limit, mode, enable)` | **主入口**:周期调用,内部完成模式决策和 0x303 下发 |
| `SuperCapGet(cap)` | 取反馈结构体副本 |
| `SuperCapIsOnline()` | 是否在线(基于 0x304 daemon) |
| `GetSafetyFrame()` | 取 0x001 安全帧 |
| `capinit(...)` | 手动发 0x302(一般不需要直接调用) |
| `CapController_Send(...)` | 手动发 0x303(绕过所有状态机,慎用) |
| `SuperCapSend(...)` | 通用 CAN 帧发送(**当前工程未使用**) |

### 枚举

```c
WorkMode     : MODE_SILENT=0, MODE_WORK=1, MODE_CHARGE=2
ExceedMode   : EXCEED_DISABLE=0, EXCEED_ENABLE=1
SafetyLevel  : SAFETY_NORMAL=0, SAFETY_WARNING=1, SAFETY_FAULT=2
CapReady     : CAP_NOT_READY=0x00, CAP_READY=0xFF
```

---

## 6. 配置项

| 字段 | 说明 | 推荐值 |
|---|---|---|
| `can_config.can_handle` | CAN 句柄 | — |
| `can_config.tx_id` | `0x302` / `0x303` / `0x001` | 决定创建哪个实例 |
| `can_config.rx_id` | `0x301` / `0x304` / `0x001` | 配对 |
| `offline_timeout` | 离线超时(**)DaemonTask 周期数**,非毫秒**) | 150 @100Hz = 1.5s |
| `auto_charge_voltage` | 低压自动充电阈值 (V);`<= 0` 关闭 | 6.0 |

---

## 7. 已知限制与注意事项

1. **单例限制**:`super_cap.c` 内部用文件级 `static` 保存 `0x302`/`0x303`/`0x001` 实例和
   daemon,**一个工程只能有一组超级电容**。多电容场景需要改成"配置随实例持有"的结构。

2. **电容参数硬编码**:`capinit(24, 5, 50, 11, 17)` 目前写死在两处(0x301 回调 + 重连逻辑),
   不在 `SuperCap_Init_Config_s` 里。换成不同参数的电容组时必须改模块源码,且两处都要改。
   **这是移植时最容易踩的坑。**

3. **`p_chassis` 无上限钳位**:`SuperCapControl()` 只做了 `>10` 的下限判断,没有 210W 的
   上限检查。超过 210 会发出协议外的值。

4. **0x301 回调未判空**:`super_cap_instance` 在回调中被无条件解引用,依赖初始化顺序保证。

5. **daemon 无离线回调**:掉线时只有 daemon 计数归零,没有日志/报警。

6. **`init_count` 被覆盖(上层问题)**:本工程 `daemon.c` 里
   `instance->temp_count = config->reload_count;` 覆盖了 `init_count` 的赋值,导致所有
   daemon **上电即视为在线 `reload_count` 个周期**。对超电的影响:启动时会主动握手
   (有利于上电顺序),但若电容不存在,会有约 1.5s 的"误以为在线"窗口。

7. **`CapType` 枚举未使用**、**`SuperCapSend()` 未被调用** —— 当前是死代码。

8. **`ready` 主要作为"板子自称未就绪"的判据**:重连逻辑会主动置 `CAP_READY`,所以它不再
   严格等价于"收到过 0x301"。

---

## 8. 调试方法

### 8.1 关键变量(调试器 watch)

```
SuperCapIsOnline()          // 1=能收到0x304
cap_msg.mode                // 电容实际工作模式:0静默/1工作/2充电
cap_msg.cap_voltage         // 电容电压(V)
cap_msg.max_power           // 可用功率上限(W)
cap_msg.chassis_power       // 电容测到的底盘功率(W)
GetSafetyFrame().overall_level
```

> 本模块没有专用的调试接口,`SuperCapIsOnline()` / `GetSafetyFrame()` 是函数调用,
> 在大多数调试器里可以直接求值;如果不行,就在调用处存一个静态变量。

### 8.2 验证清单

| # | 操作 | 期望 |
|---|---|---|
| 1 | 正常运行 | `SuperCapIsOnline()==1`,`cap_msg.mode` 跟随下发的模式 |
| 2 | 拔 CAN 1.5s 再插回 | 先离线,恢复后能重新控制,**并能在 CAN 分析仪上看到一帧 0x302** |
| 3 | 电容先上电,主控后上电 | 开机后能正常控制 |
| 4 | 电压跌破 `auto_charge_voltage` | `cap_msg.mode == 2`(充电) |
| 5 | 拨动遥控器切换模式 | 立即退出充电,按新选择执行 |
| 6 | 电容触发安全故障 | `SuperCapControl()` 发 `MODE_SILENT` |

### 8.3 CAN 分析仪观察点

- `0x302`:每次重连后应出现一次
- `0x303`:稳定运行时持续出现(≥10Hz)
- `0x304`:电容板每 500ms 一帧

---

## 9. 移植提示

1. 拷贝 `super_cap.c` / `super_cap.h` 即可,`super_cap.md`(本文档)仅供参考
2. 删除 `super_cap.c` 中未使用的 `#include "bsp_dwt.h"`
3. 确认新工程的 `bsp_can` 提供 `CANInstance.tx_buff/rx_buff(8字节)`、`rx_id`、`tx_id`、
   `can_module_callback`,回调签名 `void (*)(CANInstance *)`
4. 确认 `DaemonTask()` 的调用频率,按 `1.5s / 周期` 换算 `offline_timeout`
5. 按 `0x302 → 0x303 → 0x001` 顺序初始化
6. CAN ID `0x301/0x302/0x303/0x304/0x001` 不能与其他设备冲突,波特率 **1Mbps**
7. 上层只需三件事:初始化、周期调用 `SuperCapControl()`、按需 `SuperCapGet()`。
   **低压自动充电和断线重连都在模块内部完成,不需要上层复现。**
