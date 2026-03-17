/*-----------------------------------------------------------------------------
 * EcDemoApp.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Holger Oelhaf
 * Description              Application specific settings for EC-Master demo
 *---------------------------------------------------------------------------*/

/* =============================================================================
 * 文件解读：
 * 这是 `EcMasterDemoDc` demo 的头文件，主要作用是：
 * - 聚合 demo 需要的头文件（EC‑Master API + demo 公共模块）
 * - 定义 demo 名称、RAS 服务器开关、DC/DCM 相关参数（超时等）
 * - 声明 demo 对外入口函数：`EcDemoApp()` 以及帮助信息打印函数
 *
 * 你在二次开发时常改的地方：
 * - 若要调整 DC/DCM 初始化容忍度/超时：修改本文件里的 `ETHERCAT_DC_*` / `ETHERCAT_DCM_TIMEOUT`
 * - 若要启用/禁用 RAS：关注 `INCLUDE_RAS_SERVER` 的条件编译逻辑
 * ============================================================================= */

#ifndef INC_ECDEMOAPP_H
#define INC_ECDEMOAPP_H 1

/*-LOGGING-------------------------------------------------------------------*/
/* pEcLogParms 是本 demo 常用的日志参数指针宏。
 * 说明：很多 EcLogMsg(...) 的调用会使用 pEcLogParms / pEcLogContext（在 EcLogging.h 体系里定义）。
 */
#ifndef pEcLogParms
#define pEcLogParms (&(pAppContext->LogParms))
#endif

#define INCLUDE_EC_MASTER

/*-INCLUDES------------------------------------------------------------------*/
#include "EcMaster.h"
#include "EcDemoPlatform.h"
#include "EcDemoParms.h"
#include "EcLogging.h"
#include "EcNotification.h"
#include "EcSdoServices.h"
#include "EcSelectLinkLayer.h"
#include "EcSlaveInfo.h"
#include "EcDemoTimingTaskPlatform.h"

/*-DEFINES-------------------------------------------------------------------*/
/* demo 名称：会用于启动日志、帮助信息等显示 */
#define EC_DEMO_APP_NAME (EC_T_CHAR*)"EcMasterDemoDc"

/* the RAS server is necessary to support the EC-Engineer or other remote applications */
/* RAS（Remote API Server）说明：
 * - 让 EC‑Engineer 或其它远程工具通过网络连接到运行中的主站
 * - 下面这段逻辑表示：如果系统支持 socket（EC_SOCKET_SUPPORTED），默认启用 RAS
 */
#if (!defined INCLUDE_RAS_SERVER) && (defined EC_SOCKET_SUPPORTED)
#define INCLUDE_RAS_SERVER
#endif

#if (defined INCLUDE_RAS_SERVER)
#include "EcRasServer.h"
/* RAS 相关默认参数（见 EcDemoApp.cpp 中 emRasSrvStart 配置） */
#define ECMASTERRAS_MAX_WATCHDOG_TIMEOUT    10000
#define ECMASTERRAS_CYCLE_TIME              2
#endif

/******************************/
/* EC-Master DC configuration */
/******************************/
/* DC/DCM 参数说明（会在 EcDemoApp.cpp 中用于 ecatDcConfigure / 状态切换超时）：
 * - ETHERCAT_DC_TIMEOUT:      DC 初始化允许的最长时间（毫秒）
 * - ETHERCAT_DC_*BURST*:      静态漂移补偿相关（burst cycles / bulk）
 * - ETHERCAT_DC_DEV_LIMIT:    DC 偏差限制（broadcast read 能容忍的最高位）
 * - ETHERCAT_DC_SETTLE_TIME:  DC 初始化后的稳定等待时间（毫秒）
 * - ETHERCAT_DCM_TIMEOUT:     DCM 相关 timeout（例如 SAFEOP 切换时额外等待）
 */
#define ETHERCAT_DC_TIMEOUT             12000   /* DC initialization timeout in ms */
#define ETHERCAT_DC_ARMW_BURSTCYCLES    10000   /* DC burst cycles (static drift compensation) */
#define ETHERCAT_DC_ARMW_BURSTSPP       12      /* DC burst bulk (static drift compensation) */
#define ETHERCAT_DC_DEV_LIMIT           13      /* DC deviation limit (highest bit tolerate by the broadcast read) */
#define ETHERCAT_DC_SETTLE_TIME         1500    /* DC settle time in ms */
#define ETHERCAT_DCM_TIMEOUT            30000   /* DC initialization timeout in ms */

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
/* 帮助信息输出（命令行 -h/-help 时会用到） */
EC_T_VOID  ShowSyntaxAppUsage(T_EC_DEMO_APP_CONTEXT* pAppContext);
EC_T_VOID  ShowSyntaxApp(T_EC_DEMO_APP_CONTEXT* pAppContext);
EC_T_VOID  ShowSyntaxLinkLayer(EC_T_VOID);

/* demo 主入口（由 EcDemoMain.cpp 调用）
 * - 完成主站初始化、加载 ENI、配置 DC/DCM、启动周期任务与主循环
 */
EC_T_DWORD EcDemoApp(T_EC_DEMO_APP_CONTEXT* pAppContext);

/* 从外部注入电机完整配置（在调用 EcDemoApp 之前调用）
 * slave_addrs[i] = ENI 中第 i 个电机的 PhysAddr（站地址）
 * dds_indices[i] = 该电机对应全身第几号电机（DDS 数组索引，0-indexed）
 * count          = 电机数量
 * calib_file     = cfg.json 的绝对路径，NULL 则使用默认路径
 */
void EcDemo_SetMotorConfig(const EC_T_WORD* slave_addrs, const int* dds_indices, int count,
                           const char* calib_file = EC_NULL);
void EcDemo_SetMotorMap(const int* map, int count); /* 兼容旧接口，不推荐使用 */

/* 性能统计输出宏（依赖 `EcLogging.h` 中的 CAtEmLogging 实现） */
#define PRINT_PERF_MEAS() ((EC_NULL != pEcLogContext)?((CAtEmLogging*)pEcLogContext)->PrintPerfMeas(pAppContext->dwInstanceId, 0, pEcLogContext) : 0)
#define PRINT_HISTOGRAM() ((EC_NULL != pEcLogContext)?((CAtEmLogging*)pEcLogContext)->PrintHistogramAsCsv(pAppContext->dwInstanceId, pAppContext->pvPerfMeas) : 0)

#endif /* INC_ECDEMOAPP_H */

/*-END OF SOURCE FILE--------------------------------------------------------*/
