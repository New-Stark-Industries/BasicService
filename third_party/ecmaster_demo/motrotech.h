/*-----------------------------------------------------------------------------
 * motrotech.h
 *
 * 作用（非常重要，先读这里）：
 * - 这是一个“示例级”的 CiA402(DS402) 伺服驱动控制模块，配合 `EcMasterDemoDc` 使用。
 * - 该模块做了两件事：
 *   1) 在 `MT_Setup()` 里根据主站已加载的 ENI/配置，从 EC‑Master 获取每个 PDO 变量的位偏移，
 *      然后把 `My_Motor[]` 里的一堆指针（如 0x6040/0x6041/0x607A...）指向 ProcessImage
 *      中对应的内存地址（这就是“PDO 映射”）。
 *   2) 在 `MT_Workpd()` 每个总线周期里：
 *      - 先跑一个简化的 CiA402 状态机（读 StatusWord 0x6041，写 ControlWord 0x6040）
 *      - 再根据当前状态生成 TargetPosition/TargetVelocity 等目标（这里用一个简单的往复速度曲线）。
 *
 * 使用位置：
 * - `EcDemoApp.cpp` 的 `myAppInit/myAppPrepare/myAppSetup/myAppWorkpd` 里分别调用：
 *   `MT_Init()` / `MT_Prepare()` / `MT_Setup()` / `MT_Workpd()`。
 *
 * 重要假设/限制：
 * - 该示例假设“每个轴的对象索引”按固定偏移排布：Axis0 用 0x6040、Axis1 用 0x6040+0x800 ...，
 *   即 `OBJOFFSET = 0x800`。这不是 EtherCAT 通用规则，而是你们从站/ENI 的约定写法；
 *   如果你的从站对象不按这个规律，必须改 `MT_Setup()` 的匹配规则。
 * - 这里用到的对象（0x6040/0x6041/0x607A/...）必须被映射进 PDO，否则指针会保持为 EC_NULL。
 *
 * 说明：本文件仅为示例代码，不保证覆盖所有驱动/所有状态转换；工程化使用前需要结合实际伺服手册完善。
 *---------------------------------------------------------------------------*/
#ifndef __MOTROTECH_H__
#define __MOTROTECH_H__     1

/*-INCLUDES------------------------------------------------------------------*/
#include "EcNotification.h"
#include "EcDemoParms.h"
#include "EcSlaveInfo.h"

/* [2026-01-23] 驱动器控制模式（写入 0x6060）*/
typedef enum _DriveMode
{
    DRIVE_MODE_PT  = 4,   /* Profile Torque - 扭矩/阻抗控制 */
    DRIVE_MODE_CSP = 8,   /* Cyclic Sync Position - 位置控制 */
    DRIVE_MODE_CST = 10,  /* Cyclic Sync Torque - 纯力矩控制 */
} DriveMode;

/* [2026-01-23] 运动功能（应用层逻辑）*/
typedef enum _MotionFunc
{
    MOTION_IDLE      = 0,  /* 空闲/保持 */
    MOTION_HOME      = 1,  /* 回零 */
    MOTION_AGING     = 2,  /* 老化测试 */
    MOTION_CONTROL   = 3,  /* 正常控制（set 命令）*/
    MOTION_SHUTDOWN  = 4,  /* 停机/释放 */
} MotionFunc;

/* [2026-01-23] 重构：分离驱动模式和运动功能 */
typedef struct _MotorCmd_
{
    EC_T_BYTE  drive_mode;   /* 驱动器模式: DRIVE_MODE_PT/CSP/CST */
    EC_T_BYTE  motion_func;  /* 运动功能: MOTION_IDLE/HOME/AGING/CONTROL/SHUTDOWN */
    EC_T_SBYTE direction;    /* 方向: 0=正向, -1=反向（用于PT模式扭矩反转）*/
    EC_T_REAL  q;            /* 目标位置 (rad) -> 0x607A */
    EC_T_REAL  dq;           /* 目标速度 (rad/s) -> 0x60B1 或限速 */
    EC_T_REAL  tau;          /* 前馈力矩/目标力矩 (N.m) -> 0x6071/0x60B2 */
    EC_T_REAL  kp;           /* 刚度系数 -> 0x3500 (SDO) */
    EC_T_REAL  kd;           /* 阻尼系数 -> 0x3501 (SDO) */
    EC_T_REAL  tau_limit;    /* 力矩保护限制 (N.m) */
    EC_T_REAL  range;        /* 老化范围 (rad) */
} MotorCmd_;

/* [2026-01-19] 目的：按照最新《电机协议字段支持性详细分析表》更新接收结构体 */
typedef struct _MotorState_
{
    EC_T_BYTE  mode;              /* 电机当前模式 0x6061 */
    EC_T_REAL  q_fb;              /* 关节反馈位置 (rad) -> 0x6064 */
    EC_T_REAL  dq_fb;             /* 关节反馈速度 (rad/s) -> 0x606C */
    EC_T_REAL  ddq_fb;            /* 关节反馈加速度 (需差分计算) */
    EC_T_REAL  tau_fb;            /* 关节反馈力矩 (N.m) -> 0x6077 */
    EC_T_WORD  temperature[2];    /* [0]:MCU温度 0x3008, [1]:电机温度 0x3009 */
    EC_T_REAL  vol;               /* 母线电压 (V) -> 0x300B */
    EC_T_DWORD sensor[2];         /* 预留传感器数据 */
    EC_T_DWORD motorstate;        /* 电机状态 (0x6041 状态字或错误码) */
} MotorState_;
/*-DEFINES-------------------------------------------------------------------*/
#define MOTROTECH_VERS_MAJ             0   /* major version */             
#define MOTROTECH_VERS_MIN             0   /* minor version */             
#define MOTROTECH_VERS_SERVICEPACK     6   /* service pack */           
#define MOTROTECH_VERS_BUILD           0   /* build number */   

#define MAX_SLAVE_NUM             8
#define MAX_AXIS_NUM              8

/* 一个 slave 上可能有多个轴（多轴伺服/多通道），该示例用 “对象索引 + 轴号 * OBJOFFSET” 来区分各轴对象 */
#define OBJOFFSET                 0x800

/* 故障复位相关：示例里通过计数方式在 reset 与 disable voltage 之间切换，避免一直刷 reset */
#define COUNTLIMIT                10       /* Reset Fault cycle count limit */

/* DS402 对象索引（注意：这些只是“对象号”，是否在 PDO 里取决于从站 PDO 映射/ENI） */

#define DRV_OBJ_ERROR_CODE                  0x603F
#define DRV_OBJ_CONTROL_WORD                0x6040
#define DRV_OBJ_STATUS_WORD                 0x6041
#define DRV_OBJ_MODES_OF_OPERATION          0x6060
#define DRV_OBJ_MODES_OF_OPERATION_DISPLAY  0x6061
#define DRV_OBJ_POSITION_ACTUAL_VALUE       0x6064
#define DRV_OBJ_POSITION_WINDOW             0x6067
#define DRV_OBJ_POSITION_WINDOW_TIME        0x6068
#define DRV_OBJ_VELOCITY_ACTUAL_VALUE       0x606C
#define DRV_OBJ_TARGET_TORQUE               0x6071
#define DRV_OBJ_TORQUE_ACTUAL_VALUE         0x6077
#define DRV_OBJ_TARGET_POSITION             0x607A
#define DRV_OBJ_POSITION_RANGE_LIMIT        0x607B
#define DRV_IDN_POSITION_RANGE_LIMIT_MIN    1
#define DRV_IDN_POSITION_RANGE_LIMIT_MAX    2
#define DRV_OBJ_SOFTWARE_POSITION_LIMIT     0x607D
#define DRV_IDN_SOFTWARE_POSITION_LIMIT_MIN 1
#define DRV_IDN_SOFTWARE_POSITION_LIMIT_MAX 2
#define DRV_OBJ_PROFILE_VELOCITY            0x6081
#define DRV_OBJ_PROFILE_ACC                 0x6083
#define DRV_OBJ_PROFILE_DEC                 0x6084
#define DRV_OBJ_MOTION_PROFILE_TYPE         0x6086
#define DRV_OBJ_POS_ENCODER_RESOLUTION      0x608F
#define DRV_OBJ_POS_FACTOR                  0x6093
#define DRV_OBJ_HOMING_METHOD               0x6098
#define DRV_OBJ_HOMING_SPEED                0x6099
#define DRV_IDN_HOMING_SEARCH_SPEED_SWITCH  1
#define DRV_IDN_HOMING_SEARCH_SPEED_ZERO    2
#define DRV_OBJ_HOMING_ACCELERATION         0x609A
#define DRV_OBJ_HOMING_OFFSET               0x607C
#define DRV_OBJ_PROFILE_JERK_USE            0x60A3
#define DRV_OBJ_PROFILE_JERK                0x60A4
#define DRV_OBJ_VELOCITY_OFFSET             0x60B1
#define DRV_OBJ_TORQUE_OFFSET               0x60B2
#define DRV_OBJ_POS_OPTION_MODE             0x60F2
#define DRV_OBJ_FOLLOWING_ERROR             0x60F4
#define DRV_OBJ_DIGITAL_INPUTS              0x60FD
#define DRV_OBJ_DIGITAL_OUTPUTS             0x60FE
#define DRV_OBJ_TARGET_VELOCITY             0x60FF

/* 你文档里提到的一些“可选反馈对象”（是否能进 PDO 取决于 ENI/从站支持） */
#define DRV_OBJ_MCU_TEMPERATURE             0x3008
#define DRV_OBJ_MOTOR_TEMPERATURE           0x3009
#define DRV_OBJ_IGBT_TEMPERATURE            0x300F
#define DRV_OBJ_DC_LINK_VOLTAGE             0x300B
#define DRV_OBJ_GEAR_RATIO                  0x3D06  /* PrD.06 减速比 索引 0x3D06*/


#define DRV_OBJ_DIGITAL_INPUT               0x6000
#define DRV_OBJ_DIGITAL_INPUT_SUBINDEX_1    0x1
#define DRV_OBJ_DIGITAL_INPUT_SUBINDEX_2    0x2

#define DRV_OBJ_DIGITAL_OUTPUT              0x7010
#define DRV_OBJ_DIGITAL_OUTPUT_SUBINDEX_1   0x1
#define DRV_OBJ_DIGITAL_OUTPUT_SUBINDEX_2   0x2

/* DS402 object 0x6040: Control word */
#define DRV_CRTL_SWITCH_ON          0x0001          /* Bit 0: */
#define DRV_CRTL_ENABLE_VOLTAGE     0x0002          /* Bit 1: */
#define DRV_CRTL_QUICK_STOP         0x0004          /* Bit 2: */
#define DRV_CRTL_ENABLE_OP          0x0008          /* Bit 3: */
#define DRV_CTRL_INTER_POS_ENA      0x0010          /* Bit 4: Interpolated position mode: enable interpolation */
#define DRV_CRTL_FAULT_RESET        0x0080          /* Bit 7: */
#define DRV_CRTL_HALT               0x0100          /* Bit 8: */
#define DRV_CRTL_OP_MODE_SPEC       0x0200          /* Bit 9: */
#define DRV_CRTL_RES_10             0x0400          /* Bit 10: */
#define DRV_CRTL_MANU_SPEC          0xF800          /* Bit 11-15: */
/* DS402 drive/device control commands */
#define DRV_CTRL_CMD_MASK               0x008F          /* Control commands Mask */
#define DRV_CTRL_CMD_SHUTDOWN           0x0006          /* Shutdown (Transition 2, 6, 8) */
#define DRV_CTRL_CMD_SWITCHON           0x0007          /* Switch On (Transition 3) */
#define DRV_CTRL_CMD_DIS_VOLTAGE        0x0000          /* Disable Voltage (Transition 7, 9, 10, 12) */
#define DRV_CTRL_CMD_DIS_VOLTAGE_MASK   0x0082          /* Disable Voltage Mask */
#define DRV_CTRL_CMD_QUICK_STOP         0x0002          /* Quick Stop (Transition 7, 10, 11) */
#define DRV_CTRL_CMD_QUICK_STOP_MASK    0x0086          /* Disable Voltage Mask */
#define DRV_CTRL_CMD_DIS_OPERATION      0x0007          /* Disable Operation (Transition 5) */
#define DRV_CTRL_CMD_ENA_OPERATION      0x000F          /* Enable Operation (Transition 4) */
#define DRV_CTRL_CMD_RESET_MALFCT       DRV_CRTL_FAULT_RESET          /* Reset Malfunction (0->1 edge ) (Transition 15) */


/* DS402 object 0x6041: Status word */
#define DRV_STAT_RDY_SWITCH_ON          0x0001          /* Bit 0: Ready to switch on */
#define DRV_STAT_SWITCHED_ON            0x0002          /* Bit 1: Switched On */
#define DRV_STAT_OP_ENABLED             0x0004          /* Bit 2: Operation enabled */
#define DRV_STAT_FAULT                  0x0008          /* Bit 3: Fault */
#define DRV_STAT_VOLTAGE_ENABLED        0x0010          /* Bit 4: Optional bit: Voltage enabled */
#define DRV_STAT_QUICK_STOP             0x0020          /* Bit 5: Optional bit: Quick stop      */
#define DRV_STAT_SWITCH_ON_DIS          0x0040          /* Bit 6: Switch on disabled */
#define DRV_STAT_STATUS_TOGGLE          0x0400          /* Bit 10: Optional bit: Status toggle (csp, csv mode) */
#define DRV_STAT_VELOCITY_ZERO          0x0400          /* Bit 10: Optional bit: Velocity 0 (ip mode) */
#define DRV_STAT_OP_MODE_CSP            0x1000          /* Bit 12: Optional bit: CSP drive follows the command value */
#define DRV_STAT_FOLLOW_ERR             0x2000          /* Bit 13: Optional bit: Following error (csp, csv mode) */
#define DRV_STAT_RUNNING                0x4000          /* Bit 14: Running */
#define DRV_STAT_IDLE                   0x8000          /* Bit 15: Idle */

#define STATUSWORD_STATE_MASK                                0x006F /**< \brief status mask*/
#define STATUSWORD_STATE_MASK_EN                             0x004F /**< \brief status mask*/
#define STATUSWORD_STATE_NOTREADYTOSWITCHON                  0x0000 /**< \brief Not ready to switch on*/
#define STATUSWORD_STATE_SWITCHEDONDISABLED                  0x0040 /**< \brief Switched on but disabled*/
#define STATUSWORD_STATE_READYTOSWITCHON                     0x0021 /**< \brief Ready to switch on*/
#define STATUSWORD_STATE_SWITCHEDON                          0x0023 /**< \brief Switched on*/
#define STATUSWORD_STATE_OPERATIONENABLED                    0x0027 /**< \brief Operation enabled*/
#define STATUSWORD_STATE_QUICKSTOPACTIVE                     0x0007 /**< \brief Quickstop active*/
#define STATUSWORD_STATE_QUICKSTOPACTIVE_EN                  0x0005 /**< \brief Quickstop active*/
#define STATUSWORD_STATE_FAULT                               0x0008 /**< \brief Fault state*/

/* DS402 device control（状态机）状态枚举
 * - `wActState`：从 StatusWord 解析出来的当前状态（简化版）
 * - `wReqState`：应用希望达到的目标状态（由上层命令映射而来）
 */
enum MC_T_CIA402_STATE
{
	DRV_DEV_STATE_NOT_READY = 0, /* Not ready to switch on : Status Word x0xx 0000 */
	DRV_DEV_STATE_SWITCHON_DIS = 1, /* Switch on disabled     : Status Word x1xx 0000 */
	DRV_DEV_STATE_READY_TO_SWITCHON = 2, /* Ready to switch on     : Status Word x01x 0001 */
	DRV_DEV_STATE_SWITCHED_ON = 3, /* Switched on            : Status Word x011 0011 */
	DRV_DEV_STATE_OP_ENABLED = 4,
	/* Operation enabled      : Status Word x011 0111 */
	DRV_DEV_STATE_QUICK_STOP = 5, /* Quick stop active      : Status Word 0000 0111 */
	DRV_DEV_STATE_MALFCT_REACTION = 6, /* Malfunction/Fault reaction active Status Word (xxxx 1111) oder (xx10 1111) */
	DRV_DEV_STATE_MALFUNCTION = 7           /* Malfunction/Fault                 */
};

/* 上层“命令”枚举：由 `MT_SetSwitch()` 写入 S_ProcessState[]，再在 `Process_Commands()` 里转换为 wReqState */
/* 上层“命令”枚举（Application Command）
 *
 * 作用与数据流：
 * - 上层通过 `MT_SetSwitch(eStateCmd)` 下发命令，函数会把命令写入每个轴的 `S_ProcessState[]`。
 * - 每个周期 `MT_Workpd()` 会调用 `Process_Commands()`：
 *   - 读取 `S_ProcessState[mIndex]`
 *   - 计算该轴的目标状态 `wReqState`（CiA402 目标态）
 *   - 再根据当前状态 `wActState`（由 0x6041 StatusWord 解析）写 0x6040 ControlWord 推动状态机
 *
 * 注意：
 * - 这是 demo 级接口：只有 START/SHUTDOWN 在代码里有明确的目标态映射；
 *   其它命令更多是“占位”，后续工程化需结合从站手册完善（例如 quick stop、halt 等）。
 */
enum eStateCmd
{
	/* 不下发任何新命令（保持默认行为）。
	 * demo 中：会落到 Process_Commands() 的 default 分支，目标态通常被设为 SWITCHED_ON。
	 */
	COMMAND_NONE      = 0,

	/* 关闭/回退到更安全的状态（类似 CiA402 Shutdown 流程）。
	 * demo 中：Process_Commands() 会将 wReqState 设为 READY_TO_SWITCHON，
	 *          并通过写 0x6040（Shutdown=0x0006）推动驱动退出 OP_ENABLED。
	 */
	COMMAND_SHUTDOWN  = 1,

	/* 启动/使能（让轴进入可运动状态）。
	 * demo 中：Process_Commands() 会将 wReqState 设为 OP_ENABLED，
	 *          并通过 0x6040 的 Shutdown(0x0006)->SwitchOn(0x0007)->EnableOp(0x000F)
	 *          最短路径推进到 Operation Enabled；到达后 MT_Workpd() 才会写入目标位置/速度让电机动起来。
	 */
	COMMAND_START     = 2,

	/* 故障复位（Fault Reset）。
	 * demo 中：并未做完整的“复位条件检查/复位后再使能”流程，只在检测到 Fault 时用计数方式脉冲写复位位（0x0080）。
	 * 工程化时应按从站手册：故障原因->复位->重新上电/使能 的完整状态路径实现。
	 */
	COMMAND_RESET     = 3,

	/* Halt（暂停/保持，通常用于 CSP/CST 的 halt 位或专用对象）。
	 * demo 中：未实现完整 halt 逻辑，当前更多是占位。
	 */
	COMMAND_HALT      = 4,

	/* Pause（暂停，通常是应用层语义：停止更新目标/保持当前目标等）。
	 * demo 中：未实现，当前更多是占位。
	 */
	COMMAND_PAUSE     = 5,

	/* QuickStop（快速停止，CiA402 定义的快速停机流程）。
	 * demo 中：未实现完整 quick stop（应结合 0x605A Quick stop option code 等），当前更多是占位。
	 */
	COMMAND_QUICKSTOP = 6,

	/* Stop（停止）。
	 * demo 中：MT_SetSwitch(COMMAND_STOP) 会在写入命令后调用 CheckMotorStateStop() 做简单等待，
	 *          但并未实现严格的“速度降到 0/状态退回 switched on/ready”闭环。
	 */
	COMMAND_STOP      = 7
};
/* DS402 Modes of Operation（0x6060），这里只列出常见模式 */
enum MC_T_CIA402_OPMODE
{
	DRV_MODE_OP_PROF_POS = 1,
	/* profile position mode */
	DRV_MODE_OP_VELOCITY = 2, /* velocity mode (frequency converter) */
	DRV_MODE_OP_PROF_VEL = 3, /* profile velocity mode */
	DRV_MODE_OP_PROF_TOR = 4, /* profile torque mode */

	DRV_MODE_OP_HOMING = 6,
	/* homing mode */
	DRV_MODE_OP_INTER_POS = 7,
	/* interpolated position mode */
	DRV_MODE_OP_CSP = 8, /* cyclic synchronous position mode */
	DRV_MODE_OP_CSV = 9, /* cyclic synchronous velocity mode */
	DRV_MODE_OP_CST = 10          /* cyclic synchronous torque   mode */
};

/* DS402 modes of operation 0x6060 */
enum MC_T_MOVING_STAT
{
	MOVE_STAT_POS_ACC = 1,
	MOVE_STAT_POS_CON = 2,
	MOVE_STAT_POS_DEC = 3,
	MOVE_STAT_NEG_ACC = 4,
	MOVE_STAT_NEG_CON = 5,
	MOVE_STAT_NEG_DEC = 6
};

/* 单轴运行时上下文（重要）：
 * - 里面的 `pwControlWord/pwStatusWord/pnTargetPosition/...` 都是指针，指向 ProcessImage 的某块内存。
 * - 这些指针由 `MT_Setup()` 在启动后初始化；如果某对象未映射到 PDO，则对应指针为 EC_NULL。
 * - `fCurPos/fCurVel` 是示例算法的内部单位（与 INC_PERMM、编码器单位换算相关）。
 */
typedef struct _Motor_Type
{
	EC_T_WORD   wStationAddress;

	/*-PDO_OUTPUT（主站写给从站的输出区）-----------------------------------------*/
	EC_T_WORD*  pwControlWord;        /* 0x6040: ControlWord */
	EC_T_INT*   pnTargetPosition;     /* 0x607A: TargetPosition (通常是 int32) */
	EC_T_INT*   pnTargetVelocity;     /* 0x60FF: TargetVelocity */
	EC_T_WORD*  pwTargetTorque;       /* 0x6071: TargetTorque */
	EC_T_INT*   pnVelocityOffset;     /* 0x60B1: Velocity Offset */
	EC_T_SWORD* pwTorqueOffset;       /* 0x60B2: Torque Offset */
	EC_T_BYTE*  pbyModeOfOperation;   /* 0x6060: Mode of Operation */
	EC_T_WORD*  pwOutput_1;           /* 0x7010/1: 数字输出1（示例） */
	EC_T_WORD*  pwOutput_2;           /* 0x7010/2: 数字输出2（示例） */

	/*-PDO_INPUT（从站上报给主站的输入区）------------------------------------------*/
	EC_T_WORD*  pwErrorCode;          /* 0x603F: Error Code（故障码） */
	EC_T_WORD*  pwStatusWord;         /* 0x6041: StatusWord（状态字，用于状态机判定） */
	EC_T_INT*   pnActPosition;        /* 0x6064: Position Actual Value */
	EC_T_INT*   pnActVelocity;        /* 0x606C: Velocity Actual Value */
	EC_T_WORD*  pwActTorque;          /* 0x6077: Torque Actual Value */
	EC_T_DWORD* pdwActFollowErr;      /* 0x60F4: Following Error */
	EC_T_WORD*  pwInput_1;            /* 0x6000/1: 数字输入1（示例） */
	EC_T_WORD*  pwInput_2;            /* 0x6000/2: 数字输入2（示例） */

	/*-可选扩展输入（需要 ENI 映射）----------------------------------------------*/
	EC_T_SWORD* psTempMcu;            /* 0x3008: MCU 温度（示例，类型/单位取决于从站） */
	EC_T_SWORD* psTempMotor;          /* 0x3009: 电机温度（示例） */
	EC_T_SWORD* psTempIgbt;           /* 0x300F: IGBT 温度（示例） */
	EC_T_WORD*  pwDcLinkVoltage;      /* 0x300B: 母线电压（示例，很多驱动是 uint16/0.1V） */

	/*-单位换算（rad <-> PUU）---------------------------------------------------*/
	/* [2026-01-13] 作用：把上层 rad/rad/s 与驱动对象的 PUU(count)/PUU/s 对齐 */
	EC_T_LREAL  fCntPerRad;           /* PUU(count)/rad：q(rad) * fCntPerRad -> 0x607A int32 */
	EC_T_LREAL  fRadPerCnt;           /* rad/PUU(count)：q_cnt * fRadPerCnt -> q_fb(rad) */
	EC_T_LREAL  fGearRatio;           /* [2026-01-21] 从 SDO 0x3D06 读取的减速比 */

	MC_T_CIA402_STATE   wReqState;
	MC_T_CIA402_STATE   wActState;
	MC_T_CIA402_OPMODE  eModesOfOperation;
	EC_T_LREAL          fCurPos;      /* 示例内部当前位置（会换算到 TargetPosition） */
	EC_T_LREAL          fZeroPos;     /* 预留：示例里未使用/未赋值 */
	EC_T_LREAL          fCurVel;      /* 示例内部速度（会换算到 TargetVelocity） */
	MC_T_MOVING_STAT    eMovingStat;
	EC_T_DWORD          dwConRunCnt;
	EC_T_DWORD          dwResetCount; /* 故障复位节拍计数（示例用） */
    EC_T_LREAL          fLimitMin;    /* [2026-01-20] 软件左限位 */
    EC_T_LREAL          fLimitMax;    /* [2026-01-20] 软件右限位 */
    EC_T_BOOL           bLimitValid;  /* 限位是否已示教有效 */
    /*-扭矩控制 (PT模式)--------------------------------------------------------*/
    EC_T_LREAL  fRatedTorque;         /* 额定扭矩 (N·m)，从 0x6076 读取 */
} My_Motor_Type;

/* 一个 slave（按固定站地址）对应多少轴（wAxisCnt） */
typedef struct _SLAVE_MOTOR_TYPE
{
	EC_T_WORD           wStationAddress;
	EC_T_WORD           wAxisCnt;
}SLAVE_MOTOR_TYPE;

extern My_Motor_Type       My_Motor[MAX_AXIS_NUM];
extern SLAVE_MOTOR_TYPE    My_Slave[MAX_SLAVE_NUM];

/* 下面这组函数与 EcMasterDemoDc 的 myApp* 回调一一对应：
 * - Init：初始化本模块的静态变量/数组
 * - Prepare：根据上层填写的 `My_Slave[]`，生成 `My_Motor[]` 的站地址列表（并检查从站是否 present）
 * - Setup：在 master 已配置网络后，查出每个 PDO 变量的偏移并建立指针映射
 * - Workpd：每个周期运行（写 0x6060/0x6040/0x607A/0x60FF 等）
 */
EC_T_DWORD MT_Init(T_EC_DEMO_APP_CONTEXT* pAppContext);
EC_T_DWORD MT_Prepare(T_EC_DEMO_APP_CONTEXT* pAppContext);
EC_T_DWORD MT_Setup(T_EC_DEMO_APP_CONTEXT*   pAppContext);
EC_T_VOID  MT_Workpd(T_EC_DEMO_APP_CONTEXT* pAppContext);
EC_T_DWORD MT_SetAxisOpMod(EC_T_WORD wAxis, MC_T_CIA402_OPMODE eMode);
EC_T_VOID  MT_SetSwitch(eStateCmd command);

/* [2026-01-23] 全局驱动模式管理 */
EC_T_VOID   MT_SetGlobalDriveMode(DriveMode mode);
DriveMode   MT_GetGlobalDriveMode(EC_T_VOID);
const char* MT_GetDriveModeName(DriveMode mode);

/* 上层接口：每轴写命令/读状态（demo 级，无锁；如需强一致性请自行加锁/双缓冲） */
EC_T_VOID  MT_SetMotorCmd(EC_T_WORD wAxis, const MotorCmd_* pCmd);
EC_T_BOOL  MT_GetMotorState(EC_T_WORD wAxis, MotorState_* pStateOut);
EC_T_VOID  MT_TeachLimit(EC_T_WORD wAxis, EC_T_BOOL bIsMax);

/* 设置每轴单位换算：encoder_cpr(计数/转) 与 gear_ratio(减速比，电机转/输出转)
 * 换算关系：
 *   cnt_per_rad = encoder_cpr * gear_ratio / (2*pi)
 *   q_cnt  = q_rad  * cnt_per_rad
 *   dq_cnt = dq_rad * cnt_per_rad
 */
EC_T_BOOL  MT_SetAxisUnitScale(EC_T_WORD wAxis, EC_T_LREAL encoder_cpr, EC_T_LREAL gear_ratio);

/* [2026-01-22] 设置驱动器内置软限位
 * 通过 SDO 写入 0x607D:1 (最小限位) 和 0x607D:2 (最大限位)
 * 参数单位：弧度 (rad)，函数内部自动转换为编码器计数 (PUU)
 */
EC_T_BOOL  MT_SetDriveSoftLimits(EC_T_WORD wAxis, EC_T_LREAL fMinLimitRad, EC_T_LREAL fMaxLimitRad);

#endif /* INC_MOTROTECH */
/*-END OF SOURCE FILE--------------------------------------------------------*/