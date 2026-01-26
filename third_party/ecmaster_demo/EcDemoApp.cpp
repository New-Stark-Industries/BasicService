/*-----------------------------------------------------------------------------
 * EcDemoApp.cpp
 * Copyright                acontis technologies GmbH, Weingarten, Germany
 * Response                 Holger Oelhaf
 * Description              EC-Master demo application
 *---------------------------------------------------------------------------*/

/* =============================================================================
 * 文件解读（建议先读完本段再看代码）：
 *
 * 这是 `EcMasterDemoDc` 的“核心业务文件”。它把 EtherCAT 主站的典型生命周期串起来：
 *
 * - `EcDemoMain.cpp` 负责：解析命令行、初始化 OS/日志/定时任务，然后调用 `EcDemoApp()`。
 * - 本文件 `EcDemoApp()` 负责：
 *   1) 初始化主站（ecatInitMaster / license / perf meas / optional RAS/pcap）
 *   2) 配置网络（ecatConfigureNetwork，通常加载 ENI）并注册通知回调（ecatRegisterClient）
 *   3) 配置 DC/DCM（ecatDcConfigure / ecatDcmConfigure）
 *   4) 把主站从 INIT -> PREOP -> SAFEOP -> OP（并在合适阶段调用应用回调）
 *   5) 进入 demo 主循环：诊断、打印 DCM 状态、处理通知、直到退出
 *   6) 退出清理：停止轴/线程、回到 INIT、注销 client、deinit master
 *
 * - `EcMasterJobTask()` 是“周期任务线程”：
 *   它在每个周期被 scheduler 唤醒（pvJobTaskEvent），按固定顺序调用 `ecatExecJob()`：
 *   StartTask -> ProcessAllRxFrames(读输入) -> myAppWorkpd(应用写输出) -> SendAllCycFrames(发输出)
 *   -> MasterTimer -> SendAcycFrames -> StopTask
 *
 * - “应用层逻辑”通过 myApp* 函数组装：
 *   - `myAppInit()`      ：初始化应用变量/模块（此工程里调用 Motrotech 的 `MT_Init()`）
 *   - `myAppPrepare()`   ：准备 slave/轴等（此工程里写死站地址并调用 `MT_Prepare()`）
 *   - `myAppSetup()`     ：在 PREOP 做 SDO/OD/PDO 映射等（此工程里调用 `MT_Setup()`）
 *   - `myAppWorkpd()`    ：每周期处理过程数据（此工程里调用 `MT_Workpd()`）
 *
 * 常见改动入口（你二次开发通常改这些）：
 * - 从站站地址/轴数：`myAppPrepare()`
 * - PDO 指针映射、CiA402/伺服控制：`motrotech.cpp` 的 `MT_Setup()/MT_Workpd()`
 * - DC/DCM 同步方式与参数：`EcDemoApp()` 中 “configure DC/DCM” 段落
 * ============================================================================= */

/*-INCLUDES------------------------------------------------------------------*/
#include "EcDemoApp.h"
#include "motrotech.h"
//2026-1-13 添加再开一个线程用来运行时输入指令
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <ctime>
#include <sstream>

/*-DEFINES-------------------------------------------------------------------*/
#define CALIB_FILE_NAME "robot_calib.json"
#define TOTAL_JOINTS 7

/*-CALIBRATION DATA----------------------------------------------------------*/
// 全局标定数据结构（所有参数从 JSON 文件读取）
struct CalibData {
    bool loaded;                        // 是否已加载标定数据
    bool limits_written_to_drive;       // 软限位是否已写入驱动器
    int total_joints;                   // 关节总数
    float calib_positions[TOTAL_JOINTS]; // 标定时的编码器位置（负极限）
    int directions[TOTAL_JOINTS];       // 方向 (0=正向, -1=负向)
    float range_min[TOTAL_JOINTS];      // 运动范围最小值
    float range_max[TOTAL_JOINTS];      // 运动范围最大值
    float soft_limit_min[TOTAL_JOINTS];  // 软件限位最小值（计算得出）
    float soft_limit_max[TOTAL_JOINTS];  // 软件限位最大值（计算得出）
    char joint_names[TOTAL_JOINTS][32];  // 关节名称
    char communication[TOTAL_JOINTS][32]; // 通讯方式
};

static CalibData g_CalibData = {false, false};

// 将软限位写入驱动器的辅助函数
static bool WriteSoftLimitsToDrive()
{
    if (!g_CalibData.loaded) {
        printf("[限位] 错误: 标定数据未加载\n");
        return false;
    }
    
    printf("[限位] 正在将软限位写入驱动器...\n");
    bool all_success = true;
    
    for (int i = 0; i < g_CalibData.total_joints; i++) {
        EC_T_BOOL result = MT_SetDriveSoftLimits(
            (EC_T_WORD)i,
            (EC_T_LREAL)g_CalibData.soft_limit_min[i],
            (EC_T_LREAL)g_CalibData.soft_limit_max[i]
        );
        
        if (result) {
            printf("  %s: [%.4f, %.4f] rad - OK\n", 
                   g_CalibData.joint_names[i],
                   g_CalibData.soft_limit_min[i],
                   g_CalibData.soft_limit_max[i]);
        } else {
            printf("  %s: [%.4f, %.4f] rad - FAILED\n", 
                   g_CalibData.joint_names[i],
                   g_CalibData.soft_limit_min[i],
                   g_CalibData.soft_limit_max[i]);
            all_success = false;
        }
    }
    
    if (all_success) {
        g_CalibData.limits_written_to_drive = true;
        printf("[限位] 所有软限位已成功写入驱动器\n");
    } else {
        printf("[限位] 部分软限位写入失败\n");
    }
    
    return all_success;
}

// 辅助函数：从 JSON 字符串中提取数值
static float ParseJsonFloat(const std::string& content, size_t start_pos, const char* key)
{
    std::string search_key = std::string("\"") + key + "\":";
    size_t pos = content.find(search_key, start_pos);
    if (pos == std::string::npos) return 0.0f;
    pos += search_key.length();
    // 跳过空格
    while (pos < content.length() && (content[pos] == ' ' || content[pos] == '\t')) pos++;
    size_t end = content.find_first_of(",}\n", pos);
    return (float)atof(content.substr(pos, end - pos).c_str());
}

// 辅助函数：从 JSON 字符串中提取整数
static int ParseJsonInt(const std::string& content, size_t start_pos, const char* key)
{
    std::string search_key = std::string("\"") + key + "\":";
    size_t pos = content.find(search_key, start_pos);
    if (pos == std::string::npos) return 0;
    pos += search_key.length();
    while (pos < content.length() && (content[pos] == ' ' || content[pos] == '\t')) pos++;
    size_t end = content.find_first_of(",}\n", pos);
    return atoi(content.substr(pos, end - pos).c_str());
}

// 辅助函数：从 JSON 字符串中提取字符串值
static void ParseJsonString(const std::string& content, size_t start_pos, const char* key, char* out, size_t out_size)
{
    std::string search_key = std::string("\"") + key + "\":";
    size_t pos = content.find(search_key, start_pos);
    if (pos == std::string::npos) { out[0] = '\0'; return; }
    pos += search_key.length();
    // 跳过空格和引号
    while (pos < content.length() && (content[pos] == ' ' || content[pos] == '\t' || content[pos] == '"')) pos++;
    size_t end = content.find('"', pos);
    if (end == std::string::npos) { out[0] = '\0'; return; }
    std::string value = content.substr(pos, end - pos);
    strncpy(out, value.c_str(), out_size - 1);
    out[out_size - 1] = '\0';
}

// JSON 解析函数：从文件读取所有标定参数
static bool LoadCalibrationFile(const char* filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        printf("[标定] 警告: 无法打开标定文件 %s\n", filename);
        return false;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();
    file.close();
    
    // 读取关节总数
    g_CalibData.total_joints = ParseJsonInt(content, 0, "total_joints");
    if (g_CalibData.total_joints <= 0 || g_CalibData.total_joints > TOTAL_JOINTS) {
        printf("[标定] 错误: 无效的关节总数 %d\n", g_CalibData.total_joints);
        return false;
    }
    
    printf("[标定] 关节总数: %d\n", g_CalibData.total_joints);
    
    // 解析每个关节的数据
    for (int i = 0; i < g_CalibData.total_joints; i++) {
        // 查找 "index": i+1 对应的关节块
        char search_pattern[64];
        snprintf(search_pattern, sizeof(search_pattern), "\"index\": %d", i + 1);
        
        size_t idx_pos = content.find(search_pattern);
        if (idx_pos == std::string::npos) {
            printf("[标定] 错误: 找不到关节 %d 的数据\n", i + 1);
            return false;
        }
        
        // 读取各字段
        ParseJsonString(content, idx_pos, "name", g_CalibData.joint_names[i], 32);
        ParseJsonString(content, idx_pos, "communication", g_CalibData.communication[i], 32);
        g_CalibData.calib_positions[i] = ParseJsonFloat(content, idx_pos, "position");
        g_CalibData.directions[i] = ParseJsonInt(content, idx_pos, "direction");
        g_CalibData.range_min[i] = ParseJsonFloat(content, idx_pos, "range_min");
        g_CalibData.range_max[i] = ParseJsonFloat(content, idx_pos, "range_max");
        
        // 计算软件限位
        float total_range = g_CalibData.range_max[i] - g_CalibData.range_min[i];
        if (g_CalibData.directions[i] == 0) {
            // 正向关节：标定位置是最小值
            g_CalibData.soft_limit_min[i] = g_CalibData.calib_positions[i];
            g_CalibData.soft_limit_max[i] = g_CalibData.calib_positions[i] + total_range;
        } else {
            // 负向关节：标定位置是最大值
            g_CalibData.soft_limit_max[i] = g_CalibData.calib_positions[i];
            g_CalibData.soft_limit_min[i] = g_CalibData.calib_positions[i] - total_range;
        }
    }
    
    g_CalibData.loaded = true;
    
    printf("[标定] 已加载标定文件: %s\n", filename);
    printf("[标定] 关节参数:\n");
    for (int i = 0; i < g_CalibData.total_joints; i++) {
        printf("  %s: pos=%.4f, dir=%d, range=[%.2f, %.2f], limit=[%.4f, %.4f]\n", 
               g_CalibData.joint_names[i],
               g_CalibData.calib_positions[i],
               g_CalibData.directions[i],
               g_CalibData.range_min[i],
               g_CalibData.range_max[i],
               g_CalibData.soft_limit_min[i], 
               g_CalibData.soft_limit_max[i]);
    }
    
    return true;
}

// 检查目标位置是否在软件限位范围内
static bool CheckSoftLimit(int axis, float target_pos, float* clamped_pos = nullptr)
{
    if (!g_CalibData.loaded) {
        return true;  // 未加载标定数据时不检查
    }
    
    if (axis < 0 || axis >= TOTAL_JOINTS) {
        return false;
    }
    
    float min_limit = g_CalibData.soft_limit_min[axis];
    float max_limit = g_CalibData.soft_limit_max[axis];
    
    if (target_pos < min_limit || target_pos > max_limit) {
        printf("[限位] 错误: 轴 %d 目标位置 %.4f 超出限位范围 [%.4f, %.4f]\n",
               axis + 1, target_pos, min_limit, max_limit);
        return false;
    }
    
    return true;
}
/* 下面 3 个宏用于应用层性能统计（PerfMeas）：给不同“工作段”一个编号 */
#define PERF_myAppWorkpd       0
#define PERF_DCM_Logfile       1
#define MAX_JOB_NUM            2

#define MBX_TIMEOUT 5000

#define DCM_ENABLE_LOGFILE

/*-LOCAL VARIABLES-----------------------------------------------------------*/
/* 应用级性能统计条目名称（配合 ecatPerfMeasAppCreate/Start/End 使用） */
static EC_T_PERF_MEAS_INFO_PARMS S_aPerfMeasInfos[MAX_JOB_NUM] =
{
    {"myAppWorkPd                    ", 0},
    {"Write DCM logfile              ", 0}
};

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
//声明输入线程
static void* CmdThread(void*);
static EC_T_VOID  EcMasterJobTask(EC_T_VOID* pvAppContext);
static EC_T_DWORD EcMasterNotifyCallback(EC_T_DWORD dwCode, EC_T_NOTIFYPARMS* pParms);
#if (defined INCLUDE_RAS_SERVER)
static EC_T_DWORD RasNotifyCallback(EC_T_DWORD dwCode, EC_T_NOTIFYPARMS* pParms);
#endif

/*-MYAPP---------------------------------------------------------------------*/
/* 这是本 demo 的“应用私有上下文”，目前只用于 flash 输出演示（可选）。
 * - bFlash 开启时，在 `myAppPrepare()/myAppWorkpd()` 里会周期性改写某段 PdOut
 * - 用途：让你看到 PDO 输出确实在变化（也可用于控制灯等）
 */
typedef struct _T_MY_APP_DESC
{
    EC_T_DWORD dwFlashPdBitSize; /* Size of process data memory */
    EC_T_DWORD dwFlashPdBitOffs; /* Process data offset of data */
    EC_T_DWORD dwFlashTimer;
    EC_T_DWORD dwFlashInterval;
    EC_T_BYTE  byFlashVal;          /* flash pattern */
    EC_T_BYTE* pbyFlashBuf;         /* flash buffer */
    EC_T_DWORD dwFlashBufSize;      /* flash buffer size */
} T_MY_APP_DESC;
static EC_T_DWORD myAppInit(T_EC_DEMO_APP_CONTEXT* pAppContext);
static EC_T_DWORD myAppPrepare(T_EC_DEMO_APP_CONTEXT* pAppContext);
static EC_T_DWORD myAppSetup(T_EC_DEMO_APP_CONTEXT* pAppContext);
static EC_T_DWORD myAppWorkpd(T_EC_DEMO_APP_CONTEXT* pAppContext);
static EC_T_DWORD myAppDiagnosis(T_EC_DEMO_APP_CONTEXT* pAppContext);
static EC_T_DWORD myAppNotify(EC_T_DWORD dwCode, EC_T_NOTIFYPARMS* pParms);

/*-FUNCTION DEFINITIONS------------------------------------------------------*/

/********************************************************************************/
/** \brief EC-Master demo application.
*
* This is an EC-Master demo application.
*
* \return  Status value.
*/
EC_T_DWORD EcDemoApp(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    EC_T_DWORD             dwRetVal          = EC_E_NOERROR;
    EC_T_DWORD             dwRes             = EC_E_NOERROR;

    T_EC_DEMO_APP_PARMS*   pAppParms         = &pAppContext->AppParms;
    EC_T_VOID*             pvJobTaskHandle   = EC_NULL;

    EC_T_REGISTERRESULTS   RegisterClientResults;
    OsMemset(&RegisterClientResults, 0, sizeof(EC_T_REGISTERRESULTS));

    CEcTimer               oAppDuration;

    EC_T_BOOL              bFirstDcmStatus   = EC_TRUE;
    CEcTimer               oDcmStatusTimer;

#if (defined INCLUDE_RAS_SERVER)
    EC_T_VOID*             pvRasServerHandle = EC_NULL;
#endif

#if (defined INCLUDE_PCAP_RECORDER)
    CPcapRecorder*         pPcapRecorder     = EC_NULL;
#endif

    /* 1) 检查 LinkLayer 参数是否齐全（命令行里必须选择链路层） */
    if (EC_NULL == pAppParms->apLinkParms[0])
    {
        dwRetVal = EC_E_INVALIDPARM;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Missing link layer parameter\n"));
        goto Exit;
    }

    /* 2) 本 demo 仅支持 polling 链路层模式（不支持 interrupt） */
    if (pAppParms->apLinkParms[0]->eLinkMode != EcLinkMode_POLLING)
    {
        dwRetVal = EC_E_INVALIDPARM;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Link layer in 'interrupt' mode is not supported by %s. Please select 'polling' mode.\n", EC_DEMO_APP_NAME));
        goto Exit;
    }

    /* 3) 创建通知处理器：
     * - EC‑Master 的通知（链路变化、状态变化、错误等）都会进入回调
     * - 该类负责把通知“排队”，并在主循环里统一处理（ProcessNotificationJobs）
     */
    pAppContext->pNotificationHandler = EC_NEW(CEmNotification(pAppContext));
    if (EC_NULL == pAppContext->pNotificationHandler)
    {
        dwRetVal = EC_E_NOMEMORY;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot create notification handler\n"));
        goto Exit;
    }

    /* 4) 分配应用私有数据（本 demo 用于 flash 输出演示） */
    pAppContext->pMyAppDesc = (T_MY_APP_DESC*)OsMalloc(sizeof(T_MY_APP_DESC));
    if (EC_NULL == pAppContext->pMyAppDesc)
    {
        dwRetVal = EC_E_NOMEMORY;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot create myApp descriptor\n"));
        goto Exit;
    }
    OsMemset(pAppContext->pMyAppDesc, 0, sizeof(T_MY_APP_DESC));

    /* 5) 应用层初始化回调（此工程内会初始化 Motrotech 模块） */
    dwRes = myAppInit(pAppContext);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppInit failed: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        goto Exit;
    }

#if (defined INCLUDE_RAS_SERVER)
    /* start RAS server if enabled */
    if (pAppParms->bStartRasServer)
    {
        ECMASTERRAS_T_SRVPARMS oRemoteApiConfig;

        OsMemset(&oRemoteApiConfig, 0, sizeof(ECMASTERRAS_T_SRVPARMS));
        oRemoteApiConfig.dwSignature        = ECMASTERRASSERVER_SIGNATURE;
        oRemoteApiConfig.dwSize             = sizeof(ECMASTERRAS_T_SRVPARMS);
        oRemoteApiConfig.oAddr.dwAddr       = 0;                            /* INADDR_ANY */
        oRemoteApiConfig.wPort              = pAppParms->wRasServerPort;
        oRemoteApiConfig.dwCycleTime        = ECMASTERRAS_CYCLE_TIME;
        oRemoteApiConfig.dwCommunicationTimeout = ECMASTERRAS_MAX_WATCHDOG_TIMEOUT;
        oRemoteApiConfig.oAcceptorThreadCpuAffinityMask = pAppParms->CpuSet;
        oRemoteApiConfig.dwAcceptorThreadPrio           = MAIN_THREAD_PRIO;
        oRemoteApiConfig.dwAcceptorThreadStackSize      = JOBS_THREAD_STACKSIZE;
        oRemoteApiConfig.oClientWorkerThreadCpuAffinityMask = pAppParms->CpuSet;
        oRemoteApiConfig.dwClientWorkerThreadPrio           = MAIN_THREAD_PRIO;
        oRemoteApiConfig.dwClientWorkerThreadStackSize      = JOBS_THREAD_STACKSIZE;
        oRemoteApiConfig.pfnRasNotify    = RasNotifyCallback;                       /* RAS notification callback function */
        oRemoteApiConfig.pvRasNotifyCtxt = pAppContext->pNotificationHandler;       /* RAS notification callback function context */
        oRemoteApiConfig.dwMaxQueuedNotificationCnt = 100;                          /* pre-allocation */
        oRemoteApiConfig.dwMaxParallelMbxTferCnt    = 50;                           /* pre-allocation */
        oRemoteApiConfig.dwCycErrInterval           = 500;                          /* span between to consecutive cyclic notifications of same type */

        if (pAppParms->nVerbose >= 1)
        {
            OsMemcpy(&oRemoteApiConfig.LogParms, &pAppContext->LogParms, sizeof(EC_T_LOG_PARMS));
            oRemoteApiConfig.LogParms.dwLogLevel = EC_LOG_LEVEL_ERROR;
        }
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Start Remote API Server now\n"));
        dwRes = emRasSrvStart(&oRemoteApiConfig, &pvRasServerHandle);
        if (EC_E_NOERROR != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot spawn Remote API Server\n"));
        }
    }
#endif

    /* 6) 初始化 EtherCAT 主站（最核心一步）
     * - 绑定 OS 参数、LinkLayer、周期时间、最大从站数、异步帧额度、日志级别等
     * - 成功后，主站栈已就绪，但还未加载 ENI/未进入 OP
     */
    {
        EC_T_INIT_MASTER_PARMS oInitParms;

        OsMemset(&oInitParms, 0, sizeof(EC_T_INIT_MASTER_PARMS));
        oInitParms.dwSignature                   = ATECAT_SIGNATURE;
        oInitParms.dwSize                        = sizeof(EC_T_INIT_MASTER_PARMS);
        oInitParms.pOsParms                      = &pAppParms->Os;
        oInitParms.pLinkParms                    = pAppParms->apLinkParms[0];
        oInitParms.pLinkParmsRed                 = pAppParms->apLinkParms[1];
        oInitParms.dwBusCycleTimeUsec            = pAppParms->dwBusCycleTimeUsec;
        oInitParms.dwMaxBusSlaves                = pAppParms->dwMaxBusSlaves;
        oInitParms.dwMaxAcycFramesQueued         = MASTER_CFG_MAX_ACYC_FRAMES_QUEUED;
        if (oInitParms.dwBusCycleTimeUsec >= 1000)
        {
            oInitParms.dwMaxAcycBytesPerCycle    = MASTER_CFG_MAX_ACYC_BYTES_PER_CYC;
            oInitParms.dwMaxAcycFramesPerCycle   = 4;
        }
        else
        {
            oInitParms.dwMaxAcycBytesPerCycle    = 1500;
            oInitParms.dwMaxAcycFramesPerCycle   = 1;
            oInitParms.dwMaxAcycCmdsPerCycle     = 20;
            oInitParms.bNoConsecutiveAcycFrames  = EC_TRUE;
        }
        oInitParms.dwEcatCmdMaxRetries           = MASTER_CFG_MAX_ACYC_CMD_RETRIES;

        OsMemcpy(&oInitParms.LogParms, &pAppContext->LogParms, sizeof(EC_T_LOG_PARMS));
        oInitParms.LogParms.dwLogLevel = pAppParms->dwMasterLogLevel;

        if (pAppParms->dwPerfMeasLevel > 0)
        {
            oInitParms.PerfMeasInternalParms.bEnabled = EC_TRUE;

            if (pAppParms->dwPerfMeasLevel > 1)
            {
                oInitParms.PerfMeasInternalParms.HistogramParms.dwBinCount = 202;
            }
        }
        else
        {
            oInitParms.PerfMeasInternalParms.bEnabled = EC_FALSE;
        }

        dwRes = ecatInitMaster(&oInitParms);
        if (dwRes != EC_E_NOERROR)
        {
            dwRetVal = dwRes;
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot initialize EtherCAT-Master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
            goto Exit;
        }
        /* 7) 许可：评估版/授权版会在这里设置 license key（为空则跳过） */
        if (0 != OsStrlen(pAppParms->szLicenseKey))
        {
            dwRes = ecatSetLicenseKey(pAppParms->szLicenseKey);
            if (dwRes != EC_E_NOERROR)
            {
                dwRetVal = dwRes;
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot set license key: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
                goto Exit;
            }
        }
    }

    /* 8) 初始化应用级性能统计（可选，通过命令行 -perf 打开） */
    if (pAppParms->dwPerfMeasLevel > 0)
    {
        EC_T_PERF_MEAS_APP_PARMS oPerfMeasAppParms;
        OsMemset(&oPerfMeasAppParms, 0, sizeof(EC_T_PERF_MEAS_APP_PARMS));
        oPerfMeasAppParms.dwNumMeas = MAX_JOB_NUM;
        oPerfMeasAppParms.aPerfMeasInfos = S_aPerfMeasInfos;
        if (pAppParms->dwPerfMeasLevel > 1)
        {
            oPerfMeasAppParms.HistogramParms.dwBinCount = 202;
        }

        dwRes = ecatPerfMeasAppCreate(&oPerfMeasAppParms, &pAppContext->pvPerfMeas);
        if (dwRes != EC_E_NOERROR)
        {
            dwRetVal = dwRes;
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot initialize app performance measurement: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
            goto Exit;
        }
        pAppContext->dwPerfMeasLevel = pAppParms->dwPerfMeasLevel;
    }

    /* 9) 打印当前用于 EtherCAT 的网卡 MAC（辅助确认你选对了网卡/链路层） */
    {
        ETHERNET_ADDRESS oSrcMacAddress;
        OsMemset(&oSrcMacAddress, 0, sizeof(ETHERNET_ADDRESS));

        dwRes = ecatGetSrcMacAddress(&oSrcMacAddress);
        if (dwRes != EC_E_NOERROR)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot get MAC address: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        }
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "EtherCAT network adapter MAC: %02X-%02X-%02X-%02X-%02X-%02X\n",
            oSrcMacAddress.b[0], oSrcMacAddress.b[1], oSrcMacAddress.b[2], oSrcMacAddress.b[3], oSrcMacAddress.b[4], oSrcMacAddress.b[5]));
    }

    /* 10) EtherCAT 报文抓包（可选，PCAP recorder）。用于问题定位/抓包分析 */
#if (defined INCLUDE_PCAP_RECORDER)
    if (pAppParms->bPcapRecorder)
    {
        pPcapRecorder = EC_NEW(CPcapRecorder());
        if (EC_NULL == pPcapRecorder)
        {
            dwRetVal = EC_E_NOMEMORY;
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: %d: Creating PcapRecorder failed: %s (0x%lx)\n", pAppContext->dwInstanceId, ecatGetText(dwRetVal), dwRetVal));
            goto Exit;
        }
        dwRes = pPcapRecorder->InitInstance(pAppContext->dwInstanceId, pAppParms->dwPcapRecorderBufferFrameCnt, pAppParms->szPcapRecorderFileprefix);
        if (dwRes != EC_E_NOERROR)
        {
            dwRetVal = dwRes;
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: %d: Initialize PcapRecorder failed: %s (0x%lx)\n", pAppContext->dwInstanceId, ecatGetText(dwRes), dwRes));
            goto Exit;
        }
    }
#endif /* INCLUDE_PCAP_RECORDER */

    /* 11) 创建 JobTask 线程：真正的“每周期收发帧/处理 PDO”都在这个线程中完成
     * - JobTask 由 scheduler 唤醒（pvJobTaskEvent），因此需要正确的 TimingTask/调度器配置
     */
    {
        CEcTimer oTimeout(2000);

        pAppContext->bJobTaskRunning  = EC_FALSE;
        pAppContext->bJobTaskShutdown = EC_FALSE;
        pvJobTaskHandle = OsCreateThread((EC_T_CHAR*)"EcMasterJobTask", EcMasterJobTask, pAppParms->CpuSet,
            pAppParms->dwJobsThreadPrio, pAppParms->dwJobsThreadStackSize, (EC_T_VOID*)pAppContext);

        /* wait until thread is running */
        while (!oTimeout.IsElapsed() && !pAppContext->bJobTaskRunning)
        {
            OsSleep(10);
        }
        if (!pAppContext->bJobTaskRunning)
        {
            dwRetVal = EC_E_TIMEOUT;
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Timeout starting JobTask\n"));
            goto Exit;
        }
    }

    /* 12) 设置 OEM key（如有） */
    if (0 != pAppParms->qwOemKey)
    {
        dwRes = ecatSetOemKey(pAppParms->qwOemKey);
        if (dwRes != EC_E_NOERROR)
        {
            dwRetVal = dwRes;
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot set OEM key at master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
            goto Exit;
        }
    }
    if (pAppParms->eJunctionRedMode != eJunctionRedundancyMode_Disabled)
    {
        dwRes = ecatIoCtl(EC_IOCTL_SB_SET_JUNCTION_REDUNDANCY_MODE, &pAppParms->eJunctionRedMode, sizeof(EC_T_JUNCTION_REDUNDANCY_MODE), EC_NULL, 0, EC_NULL);
        if (dwRes != EC_E_NOERROR)
        {
            dwRetVal = dwRes;
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure junction redundancy mode: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
            goto Exit;
        }
    }
    /* 13) 配置网络（通常就是加载 ENI）
     * - eCnfType/pbyCnfData/dwCnfDataLen 由命令行 -f 指定 ENI 后解析得到
     * - 如果不提供 ENI，这里也会生成一个“临时 ENI”（功能有限）
     */
    dwRes = ecatConfigureNetwork(pAppParms->eCnfType, pAppParms->pbyCnfData, pAppParms->dwCnfDataLen);
    if (dwRes != EC_E_NOERROR)
    {
        dwRetVal = dwRes;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure EtherCAT-Master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        goto Exit;
    }

    /* 14) 注册通知回调：主站会把事件通过 EcMasterNotifyCallback 通知到应用 */
    dwRes = ecatRegisterClient(EcMasterNotifyCallback, pAppContext, &RegisterClientResults);
    if (dwRes != EC_E_NOERROR)
    {
        dwRetVal = dwRes;
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot register client: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        goto Exit;
    }
    pAppContext->pNotificationHandler->SetClientID(RegisterClientResults.dwClntId);

    /* 15) 配置 DC/DCM
     * - DC（Distributed Clocks）负责从站时钟同步
     * - DCM（Drift Compensation Mechanism）负责更高层的同步/偏移控制（不同模式）
     * - 通常只有在加载 ENI（pbyCnfData != NULL）时才做这些配置
     */
    if (EC_NULL != pAppParms->pbyCnfData)
    {
        /* 15.1) 配置 DC：超时、漂移补偿 burst、偏差限制、settle time 等 */
        {
            EC_T_DC_CONFIGURE oDcConfigure;

            OsMemset(&oDcConfigure, 0, sizeof(EC_T_DC_CONFIGURE));
            oDcConfigure.dwTimeout          = ETHERCAT_DC_TIMEOUT;
            oDcConfigure.dwDevLimit         = ETHERCAT_DC_DEV_LIMIT;
            oDcConfigure.dwSettleTime       = ETHERCAT_DC_SETTLE_TIME;
            oDcConfigure.dwTotalBurstLength = ETHERCAT_DC_ARMW_BURSTCYCLES;
            if (pAppParms->dwBusCycleTimeUsec < 1000)
            {
                /* reduce frames sent within each cycle for cycle time below 1 ms */
                oDcConfigure.dwBurstBulk = ETHERCAT_DC_ARMW_BURSTSPP / 2;
            }
            else
            {
                oDcConfigure.dwBurstBulk = ETHERCAT_DC_ARMW_BURSTSPP;
            }
            /* for short cycle times the cyclic distribution is sufficient. disable acyclic distribution to reduce CPU load. */
            if ((pAppParms->dwBusCycleTimeUsec < 2000) && (eDcmMode_Dcx != pAppParms->eDcmMode))
            {
                oDcConfigure.bAcycDistributionDisabled = EC_TRUE;
            }

            dwRes = ecatDcConfigure(&oDcConfigure);
            if (dwRes != EC_E_NOERROR )
            {
                dwRetVal = dwRes;
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure DC! (Result = 0x%x)\n", dwRes));
                goto Exit;
            }
        }
        /* 15.2) 配置 DCM：根据命令行选择模式（BusShift/MasterShift/MasterRefClock/.../DCX） */
        if (pAppParms->bDcmLogEnabled && !pAppParms->bDcmConfigure)
        {
            EC_T_BOOL bBusShiftConfiguredByEni = EC_FALSE;
            dwRes = ecatDcmGetBusShiftConfigured(&bBusShiftConfiguredByEni);
            if (dwRes != EC_E_NOERROR)
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot check if BusShift is configured  (Result = 0x%x)\n", dwRes));
            }
            if (bBusShiftConfiguredByEni)
            {
                pAppParms->bDcmConfigure = EC_TRUE;
                pAppParms->eDcmMode = eDcmMode_BusShift;
            }
        }
        if (pAppParms->bDcmConfigure)
        {
            /* 这里根据总线周期时间计算一些“经验参数”：
             * - nCtlSetValNsec：帧发送时间与 DC 基准的相对位置（示例默认 66%）
             * - dwInSyncLimitNsec：判定 InSync 的门限（示例默认 25%）
             * 实际工程可以按抖动、网络拓扑、主机性能等进一步调优。
             */
            EC_T_DWORD dwCycleTimeNsec   = pAppParms->dwBusCycleTimeUsec * 1000; /* cycle time in nsec */
            EC_T_INT   nCtlSetValNsec    = dwCycleTimeNsec * 2 / 3 /* 66% */;    /* distance between cyclic frame send time and DC base on bus */
            EC_T_DWORD dwInSyncLimitNsec = dwCycleTimeNsec / 4 /* 25% */;        /* limit for DCM InSync monitoring */

            EC_T_DCM_CONFIG oDcmConfig;
            OsMemset(&oDcmConfig, 0, sizeof(EC_T_DCM_CONFIG));

            switch (pAppParms->eDcmMode)
            {
            case eDcmMode_Off:
                oDcmConfig.eMode = eDcmMode_Off;
                break;
            case eDcmMode_BusShift:
                oDcmConfig.eMode = eDcmMode_BusShift;
                oDcmConfig.u.BusShift.nCtlSetVal    = nCtlSetValNsec;
                oDcmConfig.u.BusShift.dwInSyncLimit = dwInSyncLimitNsec;
                oDcmConfig.u.BusShift.bLogEnabled = pAppParms->bDcmLogEnabled;
                oDcmConfig.u.BusShift.pGetTimeElapsedSinceCycleStartContext = pAppContext->pTimingTaskContext;
                if (pAppParms->bDcmSyncToCycleStart)
                {
                    oDcmConfig.u.BusShift.pfnGetTimeElapsedSinceCycleStart = CDemoTimingTaskPlatform::GetTimeElapsedSinceCycleStart;
                }
                if (pAppParms->bDcmControlLoopDisabled)
                {
                    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM control loop disabled\n"));
                    oDcmConfig.u.BusShift.bCtlOff = EC_TRUE;
                }
                break;
            case eDcmMode_MasterShift:
                oDcmConfig.eMode = eDcmMode_MasterShift;
                oDcmConfig.u.MasterShift.nCtlSetVal    = nCtlSetValNsec;
                oDcmConfig.u.MasterShift.dwInSyncLimit = dwInSyncLimitNsec;
                oDcmConfig.u.MasterShift.bLogEnabled = pAppParms->bDcmLogEnabled;
                oDcmConfig.u.MasterShift.pGetTimeElapsedSinceCycleStartContext = pAppContext->pTimingTaskContext;
                if (pAppParms->bDcmSyncToCycleStart)
                {
                    oDcmConfig.u.MasterShift.pfnGetTimeElapsedSinceCycleStart = CDemoTimingTaskPlatform::GetTimeElapsedSinceCycleStart;
                }
                oDcmConfig.u.MasterShift.pAdjustCycleTimeContext = pAppContext->pTimingTaskContext;
                oDcmConfig.u.MasterShift.pfnAdjustCycleTime = CDemoTimingTaskPlatform::AdjustCycleTime;
                if (pAppParms->bDcmControlLoopDisabled)
                {
                    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM control loop disabled\n"));
                    oDcmConfig.u.MasterShift.bCtlOff = EC_TRUE;
                }
                break;
            case eDcmMode_MasterRefClock:
                oDcmConfig.eMode = eDcmMode_MasterRefClock;
                oDcmConfig.u.MasterRefClock.nCtlSetVal  = nCtlSetValNsec;
                oDcmConfig.u.MasterRefClock.dwInSyncLimit = dwInSyncLimitNsec;
                oDcmConfig.u.MasterRefClock.bLogEnabled = pAppParms->bDcmLogEnabled;
                oDcmConfig.u.MasterRefClock.pGetHostTimeContext = pAppContext->pTimingTaskContext;
                oDcmConfig.u.MasterRefClock.pfnGetHostTime = CDemoTimingTaskPlatform::GetHostTime;
                break;
            case eDcmMode_LinkLayerRefClock:
                oDcmConfig.eMode = eDcmMode_LinkLayerRefClock;
                oDcmConfig.u.LinkLayerRefClock.nCtlSetVal = nCtlSetValNsec;
                oDcmConfig.u.LinkLayerRefClock.dwInSyncLimit = dwInSyncLimitNsec;
                oDcmConfig.u.LinkLayerRefClock.bLogEnabled = pAppParms->bDcmLogEnabled;
                break;
            case eDcmMode_Dcx:
                oDcmConfig.eMode = eDcmMode_Dcx;
                /* DCX MasterShift */
                oDcmConfig.u.Dcx.MasterShift.nCtlSetVal = nCtlSetValNsec;
                oDcmConfig.u.Dcx.MasterShift.dwInSyncLimit = dwInSyncLimitNsec;
                oDcmConfig.u.Dcx.MasterShift.bLogEnabled = pAppParms->bDcmLogEnabled;
                oDcmConfig.u.Dcx.MasterShift.pGetTimeElapsedSinceCycleStartContext = pAppContext->pTimingTaskContext;
                if (pAppParms->bDcmSyncToCycleStart)
                {
                    oDcmConfig.u.Dcx.MasterShift.pfnGetTimeElapsedSinceCycleStart = CDemoTimingTaskPlatform::GetTimeElapsedSinceCycleStart;
                }
                oDcmConfig.u.Dcx.MasterShift.pAdjustCycleTimeContext = pAppContext->pTimingTaskContext;
                oDcmConfig.u.Dcx.MasterShift.pfnAdjustCycleTime = CDemoTimingTaskPlatform::AdjustCycleTime;
                /* DCX BusShift */
                oDcmConfig.u.Dcx.nCtlSetVal = nCtlSetValNsec;
                oDcmConfig.u.Dcx.dwInSyncLimit = dwInSyncLimitNsec;
                oDcmConfig.u.Dcx.bLogEnabled = pAppParms->bDcmLogEnabled;
                oDcmConfig.u.Dcx.dwExtClockTimeout = 1000;
                oDcmConfig.u.Dcx.wExtClockFixedAddr = 0; /* 0 only when clock adjustment in external mode configured by EcEngineer */
                if (pAppParms->bDcmControlLoopDisabled)
                {
                    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM control loop disabled\n"));

                    oDcmConfig.u.Dcx.MasterShift.bCtlOff = EC_TRUE;
                    oDcmConfig.u.Dcx.bCtlOff = EC_TRUE;
                }
                break;
            default:
                dwRetVal = EC_E_NOTSUPPORTED;
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "DCM mode is not supported!\n"));
                goto Exit;

            }
            dwRes = ecatDcmConfigure(&oDcmConfig, 0);
            switch (dwRes)
            {
            case EC_E_NOERROR:
                break;
            case EC_E_FEATURE_DISABLED:
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure DCM mode!\n"));
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Start with -dcmmode off to run the DC demo without DCM, or prepare the ENI file to support the requested DCM mode\n"));
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "In ET9000 for example, select under ""Advanced settings\\Distributed clocks"" ""DC in use"" and ""Slave Mode""\n"));
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "to support BusShift and MasterRefClock modes.\n"));
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Please refer to the class A manual for further information\n"));
                dwRetVal = dwRes;
                goto Exit;
            default:
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot configure DCM mode! %s (Result = 0x%x)\n", ecatGetText(dwRes), dwRes));
                dwRetVal = dwRes;
                goto Exit;
            }
        }
    }
#if (defined INCLUDE_SLAVE_STATISTICS)
    /* enable and reset statistics */
    {
        EC_T_DWORD dwPeriodMs = 1000;

        dwRes = ecatIoCtl(EC_IOCTL_SET_SLVSTAT_PERIOD, (EC_T_BYTE*)&dwPeriodMs, sizeof(EC_T_DWORD), EC_NULL, 0, EC_NULL);
        if (dwRes != EC_E_NOERROR)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot set slave statistics period: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        }
        dwRes = ecatClearSlaveStatistics(INVALID_SLAVE_ID);
        if (dwRes != EC_E_NOERROR)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot reset slave statistics: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        }
    }
#endif

    /* print found slaves */
    /* 16) （可选）扫描总线并打印从站信息：用于确认拓扑/从站识别是否正常 */
    if (pAppParms->dwAppLogLevel >= EC_LOG_LEVEL_VERBOSE)
    {
        dwRes = ecatScanBus(ETHERCAT_SCANBUS_TIMEOUT);
        pAppContext->pNotificationHandler->ProcessNotificationJobs();
        switch (dwRes)
        {
        case EC_E_NOERROR:
        case EC_E_BUSCONFIG_MISMATCH:
        case EC_E_LINE_CROSSED:
            PrintSlaveInfos(pAppContext);
            break;
        default:
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot scan bus: %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
            break;
        }
        if (dwRes != EC_E_NOERROR)
        {
            dwRetVal = dwRes;
            goto Exit;
        }
    }

    /* print process variable name and offset for all variables of all slaves */
    if (pAppContext->AppParms.bPrintVars)
    {
        PrintAllSlavesProcVarInfos(pAppContext);
    }

    /* 17) 状态机推进：INIT -> PREOP -> SAFEOP -> OP
     * - INIT：基础初始化态
     * - PREOP：邮箱通信可用，适合做 SDO/OD 读写、PDO 映射准备
     * - SAFEOP：过程数据已建立但输出一般不生效（安全态）
     * - OP：过程数据收发正常，开始真正控制设备
     */
    /* 17.1) set master to INIT（重新回到 INIT，确保状态干净） */
    dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_INIT);
    pAppContext->pNotificationHandler->ProcessNotificationJobs();
    if (dwRes != EC_E_NOERROR)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot start set master state to INIT: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        dwRetVal = dwRes;
        goto Exit;
    }

    /* 17.2) 应用准备阶段：例如设置站地址、轴数、检查从站存在等 */
    dwRes = myAppPrepare(pAppContext);
    if (EC_E_NOERROR != dwRes)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare failed: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        dwRetVal = dwRes;
        goto Exit;
    }

    /* 17.3) PREOP：此后可以进行 SDO/OD/映射等设置 */
    dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_PREOP);
    pAppContext->pNotificationHandler->ProcessNotificationJobs();
    if (dwRes != EC_E_NOERROR)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot start set master state to PREOP: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        dwRetVal = dwRes;
        goto Exit;
    }

    /* 只有加载 ENI 时才做 Setup/SAFEOP/OP（无 ENI 时，很多配置/同步能力会缺失） */
    if (EC_NULL != pAppParms->pbyCnfData)
    {
        /* 17.4) Setup：应用在 PREOP 做从站参数配置（本工程里做 PDO 指针映射 + 可选读对象字典） */
        dwRes = myAppSetup(pAppContext);
        if (EC_E_NOERROR != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "myAppSetup failed: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
            dwRetVal = dwRes;
            goto Exit;
        }

        /* 17.5) SAFEOP：此处常见失败原因是 DCM 未能 InSync（所以 timeout 更长） */
        dwRes = ecatSetMasterState(ETHERCAT_DCM_TIMEOUT + ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_SAFEOP);
        pAppContext->pNotificationHandler->ProcessNotificationJobs();
        if (dwRes != EC_E_NOERROR)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot start set master state to SAFEOP: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));

            /* most of the time SAFEOP is not reachable due to DCM */
            if ((eDcmMode_Off != pAppParms->eDcmMode) && (eDcmMode_LinkLayerRefClock != pAppParms->eDcmMode))
            {
            EC_T_DWORD dwStatus = 0;
            EC_T_INT   nDiffCur = 0, nDiffAvg = 0, nDiffMax = 0;

                dwRes = ecatDcmGetStatus(&dwStatus, &nDiffCur, &nDiffAvg, &nDiffMax);
                if (dwRes == EC_E_NOERROR)
                {
                    if (dwStatus != EC_E_NOERROR)
                    {
                        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "DCM Status: %s (0x%08X)\n", ecatGetText(dwStatus), dwStatus));
                    }
                }
                else
                {
                    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot get DCM status! %s (0x%08X)\n", ecatGetText(dwRes), dwRes));
                }
            }
            dwRetVal = dwRes;
            goto Exit;
        }

        /* 17.6) OP：进入正式运行态（PDO 每周期收发） */
        dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_OP);
        pAppContext->pNotificationHandler->ProcessNotificationJobs();
        if (dwRes != EC_E_NOERROR)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot start set master state to OP: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
            dwRetVal = dwRes;
            goto Exit;
        }
    }
    else
    {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "No ENI file provided. EC-Master started with generated ENI file.\n"));
    }

    if (pAppContext->dwPerfMeasLevel > 0)
    {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "\nJob times during startup <INIT> to <%s>:\n", ecatStateToStr(ecatGetMasterState())));
        PRINT_PERF_MEAS();
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "\n"));
        /* clear job times of startup phase */
        ecatPerfMeasAppReset(pAppContext->pvPerfMeas, EC_PERF_MEAS_ALL);
        ecatPerfMeasReset(EC_PERF_MEAS_ALL);
    }

    /* 18) demo 主循环：
     * - 诊断（myAppDiagnosis）
     * - 周期性打印 DCM/DCX 状态（如果启用）
     * - 处理通知队列（ProcessNotificationJobs）
     * - 直到用户终止/超时
     */
    if (pAppParms->dwDemoDuration != 0)
    {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%s will stop in %ds...\n", EC_DEMO_APP_NAME, pAppParms->dwDemoDuration / 1000));
        oAppDuration.Start(pAppParms->dwDemoDuration);
    }
    bRun = EC_TRUE;
    {
        CEcTimer oPerfMeasPrintTimer;

        if (pAppParms->bPerfMeasShowCyclic)
        {
            oPerfMeasPrintTimer.Start(10000);
        }

        /* [2026-01-20] 安全修改：开机默认不自动下发 START 命令，保持在未使能状态 */
        // MT_SetSwitch(COMMAND_START); 

        while (bRun)
        {
            if (oPerfMeasPrintTimer.IsElapsed())
            {
                PRINT_PERF_MEAS();
                oPerfMeasPrintTimer.Restart();
            }

            /* check if demo shall terminate */
            bRun = !(OsTerminateAppRequest() || oAppDuration.IsElapsed());

            /* 轻量级诊断钩子（默认空实现，可放报警/状态打印等） */
            myAppDiagnosis(pAppContext);

            if (EC_NULL != pAppParms->pbyCnfData)
            {
                if ((eDcmMode_Off != pAppParms->eDcmMode) && (eDcmMode_LinkLayerRefClock != pAppParms->eDcmMode))
                {
                    EC_T_DWORD dwStatus = 0;
                    EC_T_BOOL  bWriteDiffLog = EC_FALSE;
                    EC_T_INT   nDiffCur = 0, nDiffAvg = 0, nDiffMax = 0;

                    if (!oDcmStatusTimer.IsStarted() || oDcmStatusTimer.IsElapsed())
                    {
                        bWriteDiffLog = EC_TRUE;
                        oDcmStatusTimer.Start(5000);
                    }

                    dwRes = ecatDcmGetStatus(&dwStatus, &nDiffCur, &nDiffAvg, &nDiffMax);
                    if (dwRes == EC_E_NOERROR)
                    {
                        if (bFirstDcmStatus)
                        {
                            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM during startup (<INIT> to <%s>)\n", ecatStateToStr(ecatGetMasterState())));
                        }
                        if ((dwStatus != EC_E_NOTREADY) && (dwStatus != EC_E_BUSY) && (dwStatus != EC_E_NOERROR))
                        {
                            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM Status: %s (0x%08X)\n", ecatGetText(dwStatus), dwStatus));
                        }
                        if (bWriteDiffLog && pAppParms->bDcmLogEnabled)
                        {
                            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCM Diff (cur/avg/max) [nsec]: %7d/ %7d/ %7d\n", nDiffCur, nDiffAvg, nDiffMax));
                        }
                    }
                    else
                    {
                        if ((eEcatState_OP == ecatGetMasterState()) || (eEcatState_SAFEOP == ecatGetMasterState()))
                        {
                            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot get DCM status! %s (0x%08X)\n", ecatGetText(dwRes), dwRes));
                        }
                    }
                    if (eDcmMode_Dcx == pAppParms->eDcmMode && EC_E_NOERROR == dwRes)
                    {
                    EC_T_INT64 nTimeStampDiff = 0;
                        dwRes = ecatDcxGetStatus(&dwStatus, &nDiffCur, &nDiffAvg, &nDiffMax, &nTimeStampDiff);
                        if (EC_E_NOERROR == dwRes)
                        {
                            if (bFirstDcmStatus)
                            {
                                EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCX during startup (<INIT> to <%s>)\n", ecatStateToStr(ecatGetMasterState())));
                            }
                            if ((dwStatus != EC_E_NOTREADY) && (dwStatus != EC_E_BUSY) && (dwStatus != EC_E_NOERROR))
                            {
                                EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCX Status: %s (0x%08X)\n", ecatGetText(dwStatus), dwStatus));
                            }
                            if (bWriteDiffLog && pAppParms->bDcmLogEnabled)
                            {
                                EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DCX Diff(ns): Cur=%7d, Avg=%7d, Max=%7d, TimeStamp=%7d\n", nDiffCur, nDiffAvg, nDiffMax, nTimeStampDiff));
                            }
                        }
                        else
                        {
                            if ((eEcatState_OP == ecatGetMasterState()) || (eEcatState_SAFEOP == ecatGetMasterState()))
                            {
                                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot get DCX status! %s (0x%08X)\n", ecatGetText(dwRes), dwRes));
                            }
                        }
                    }
                    if (bFirstDcmStatus && (EC_E_NOERROR == dwRes))
                    {
                        bFirstDcmStatus = EC_FALSE;
                        ecatDcmResetStatus();
                    }
                }
            }
            /* 把异步通知（状态变化、错误、Ras 等）统一在此处理，避免在回调里做重活 */
            pAppContext->pNotificationHandler->ProcessNotificationJobs();

            OsSleep(5);
        }
    }

    if (pAppParms->dwAppLogLevel != EC_LOG_LEVEL_SILENT)
    {
        EC_T_DWORD dwCurrentUsage = 0;
        EC_T_DWORD dwMaxUsage = 0;
        dwRes = ecatGetMemoryUsage(&dwCurrentUsage, &dwMaxUsage);
        if (EC_E_NOERROR != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot read memory usage of master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
            goto Exit;
        }
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Memory Usage Master     (cur/max) [bytes]: %u/%u\n", dwCurrentUsage, dwMaxUsage));

#if (defined INCLUDE_RAS_SERVER)
        if (EC_NULL != pvRasServerHandle)
        {
            dwRes = emRasGetMemoryUsage(pvRasServerHandle, &dwCurrentUsage, &dwMaxUsage);
            if (EC_E_NOERROR != dwRes)
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot read memory usage of RAS: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
                goto Exit;
            }
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Memory Usage RAS Server (cur/max) [bytes]: %u/%u\n", dwCurrentUsage, dwMaxUsage));
        }
#endif
    }

Exit:
    /* 19) 退出/清理：先让轴进入 shutdown（demo 的 Motrotech 行为），再停主站/线程 */
    MT_SetSwitch(COMMAND_SHUTDOWN);
    
    /* set master state to INIT */
    if (eEcatState_UNKNOWN != ecatGetMasterState())
    {
        if (pAppParms->dwPerfMeasLevel > 0)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "\nJob times before shutdown\n"));
            PRINT_PERF_MEAS();
        }
        if (pAppParms->dwPerfMeasLevel > 1)
        {
            PRINT_HISTOGRAM();
        }

        dwRes = ecatSetMasterState(ETHERCAT_STATE_CHANGE_TIMEOUT, eEcatState_INIT);
        pAppContext->pNotificationHandler->ProcessNotificationJobs();
        if (EC_E_NOERROR != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot stop EtherCAT-Master: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
        }
    }

#if (defined INCLUDE_PCAP_RECORDER)
    SafeDelete(pPcapRecorder);
#endif /* INCLUDE_PCAP_RECORDER */

    /* unregister client */
    if (EC_NULL != pAppContext->pNotificationHandler)
    {
        EC_T_DWORD dwClientId = pAppContext->pNotificationHandler->GetClientID();
        if (INVALID_CLIENT_ID != dwClientId)
        {
            dwRes = ecatUnregisterClient(dwClientId);
            if (EC_E_NOERROR != dwRes)
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Cannot unregister client: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
            }
            pAppContext->pNotificationHandler->SetClientID(INVALID_CLIENT_ID);
        }
    }

#if (defined INCLUDE_RAS_SERVER)
    /* stop RAS server */
    if (EC_NULL != pvRasServerHandle)
    {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Stop Remote Api Server\n"));
        dwRes = emRasSrvStop(pvRasServerHandle, 2000);
        if (EC_E_NOERROR != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Remote API Server shutdown failed\n"));
        }
    }
#endif

    /* 20) 停止 JobTask：设置 shutdown 标志，等待线程退出，再释放句柄 */
    {
        CEcTimer oTimeout(2000);
        pAppContext->bJobTaskShutdown = EC_TRUE;
        while (pAppContext->bJobTaskRunning && !oTimeout.IsElapsed())
        {
            OsSleep(10);
        }
        if (EC_NULL != pvJobTaskHandle)
        {
            OsDeleteThreadHandle(pvJobTaskHandle);
            pvJobTaskHandle = EC_NULL;
        }
    }

    /* 21) 反初始化主站（释放内部资源） */
    dwRes = ecatDeinitMaster();
    if (EC_E_NOERROR != dwRes)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: Cannot de-initialize EtherCAT-Master: %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
    }

    SafeDelete(pAppContext->pNotificationHandler);
    if (EC_NULL != pAppContext->pMyAppDesc)
    {
        SafeOsFree(pAppContext->pMyAppDesc->pbyFlashBuf);
        SafeOsFree(pAppContext->pMyAppDesc);
    }

    return dwRetVal;
}

/********************************************************************************/
/** \brief  Trigger jobs to drive master, and update process data.
*
* \return N/A
*/
static EC_T_VOID EcMasterJobTask(EC_T_VOID* pvAppContext)
{
    EC_T_DWORD dwRes = EC_E_ERROR;
    EC_T_INT   nOverloadCounter = 0;               /* counter to check if cycle time is to short */
    T_EC_DEMO_APP_CONTEXT* pAppContext = (T_EC_DEMO_APP_CONTEXT*)pvAppContext;
    T_EC_DEMO_APP_PARMS*   pAppParms   = &pAppContext->AppParms;

    EC_T_USER_JOB_PARMS oJobParms;
    OsMemset(&oJobParms, 0, sizeof(EC_T_USER_JOB_PARMS));

    /* 周期任务主循环：由 scheduler 通过 pvJobTaskEvent 触发（一般一周期触发一次） */
    pAppContext->bJobTaskRunning = EC_TRUE;
    do
    {
        /* 等待下一周期（来自定时/调度任务的 event） */
        OsDbgAssert(pAppContext->pvJobTaskEvent != 0);
        dwRes = OsWaitForEvent(pAppContext->pvJobTaskEvent, 3000);
        if (EC_E_NOERROR != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: OsWaitForEvent(): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
            OsSleep(500);
        }

        /* 下面是“每周期固定顺序”的主站工作流（高频路径）：
         * 1) StartTask：PerfMeas 辅助（增强测量）
         * 2) ProcessAllRxFrames：处理收到的所有 cyclic 帧（更新输入过程数据）
         * 3) myAppWorkpd：应用根据输入计算输出（写输出过程数据）
         * 4) SendAllCycFrames：发送本周期的 cyclic 帧（把输出发到从站）
         * 5) MasterTimer：主站内部定时维护（无总线流量）
         * 6) SendAcycFrames：发送排队的 mailbox/异步帧（SDO 等）
         * 7) StopTask：PerfMeas 辅助（增强测量）
         */
        /* start Task (required for enhanced performance measurement) */
        dwRes = ecatExecJob(eUsrJob_StartTask, EC_NULL);
        if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: ecatExecJob(eUsrJob_StartTask): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
        }

        /* 处理所有收到的帧（读入最新输入过程数据） */
        dwRes = ecatExecJob(eUsrJob_ProcessAllRxFrames, &oJobParms);
        if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: ecatExecJob(eUsrJob_ProcessAllRxFrames): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
        }

        if (EC_E_NOERROR == dwRes)
        {
            if (!oJobParms.bAllCycFramesProcessed)
            {
                /* 连续 frame loss 说明系统过载/周期太短/抖动太大（demo 用计数器做简单告警节流） */
                nOverloadCounter += 10;
                if (nOverloadCounter >= 50)
                {
                    if ((pAppContext->dwPerfMeasLevel > 0) && (nOverloadCounter < 60))
                    {
                        PRINT_PERF_MEAS();
                    }
                    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Error: System overload: Cycle time too short or huge jitter!\n"));
                }
                else
                {
                    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "eUsrJob_ProcessAllRxFrames - not all previously sent frames are received/processed (frame loss)!\n"));
                }
            }
            else
            {
                /* everything o.k.? If yes, decrement overload counter */
                if (nOverloadCounter > 0)    nOverloadCounter--;
            }
        }

        /* DCM 日志：如果启用，把 DCM 内部日志写到 logging 中（便于分析同步质量） */
#ifdef DCM_ENABLE_LOGFILE
        if (pAppParms->bDcmLogEnabled)
        {
            EC_T_CHAR* pszLog = EC_NULL;

            if (pAppContext->dwPerfMeasLevel > 0)
            {
                ecatPerfMeasAppStart(pAppContext->pvPerfMeas, PERF_DCM_Logfile);
            }
            ecatDcmGetLog(&pszLog);
            if ((EC_NULL != pszLog))
            {
                ((CAtEmLogging*)pEcLogContext)->LogDcm(pszLog);
            }
            if (pAppContext->dwPerfMeasLevel > 0)
            {
                ecatPerfMeasAppEnd(pAppContext->pvPerfMeas, PERF_DCM_Logfile);
            }
        }
#endif

        if (pAppContext->dwPerfMeasLevel > 0)
        {
            ecatPerfMeasAppStart(pAppContext->pvPerfMeas, PERF_myAppWorkpd);
        }
        {   /* 只有 SAFEOP/OP 才调用 myAppWorkpd（因为 PDO 才有意义） */
            EC_T_STATE eMasterState = ecatGetMasterState();

            if ((eEcatState_SAFEOP == eMasterState) || (eEcatState_OP == eMasterState))
            {
                myAppWorkpd(pAppContext);
            }
        }
        if (pAppContext->dwPerfMeasLevel > 0)
        {
            ecatPerfMeasAppEnd(pAppContext->pvPerfMeas, PERF_myAppWorkpd);
        }

        /* 发送本周期所有 cyclic 帧（把 PdOut 写到从站） */
        dwRes = ecatExecJob(eUsrJob_SendAllCycFrames, &oJobParms);
        if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ecatExecJob( eUsrJob_SendAllCycFrames,    EC_NULL ): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
        }

        /* remove this code when using licensed version */
        if (EC_E_EVAL_EXPIRED == dwRes)
        {
            bRun = EC_FALSE; /* set shutdown flag */
        }

        /* 主站内部维护（无总线流量） */
        dwRes = ecatExecJob(eUsrJob_MasterTimer, EC_NULL);
        if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ecatExecJob(eUsrJob_MasterTimer, EC_NULL): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
        }

        /* 发送排队的异步帧（mailbox/SDO 等），该路径一般是低频/按需 */
        dwRes = ecatExecJob(eUsrJob_SendAcycFrames, EC_NULL);
        if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ecatExecJob(eUsrJob_SendAcycFrames, EC_NULL): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
        }

        /* stop Task (required for enhanced performance measurement) */
        dwRes = ecatExecJob(eUsrJob_StopTask, EC_NULL);
        if (EC_E_NOERROR != dwRes && EC_E_INVALIDSTATE != dwRes && EC_E_LINK_DISCONNECTED != dwRes)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: ecatExecJob(eUsrJob_StopTask): %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
        }
#if !(defined NO_OS)
    } while (!pAppContext->bJobTaskShutdown);

    pAppContext->bJobTaskRunning = EC_FALSE;
#else
    /* in case of NO_OS the job task function is called cyclically within the timer ISR */
    } while (EC_FALSE);
    pAppContext->bJobTaskRunning = !pAppContext->bJobTaskShutdown;
#endif

    return;
}

/********************************************************************************/
/** \brief  Handler for master notifications
*
* \return  Status value.
*/
static EC_T_DWORD EcMasterNotifyCallback(
    EC_T_DWORD         dwCode,  /**< [in]   Notification code */
    EC_T_NOTIFYPARMS*  pParms   /**< [in]   Notification parameters */
)
{
    EC_T_DWORD dwRetVal = EC_E_NOERROR;
    CEmNotification* pNotificationHandler = EC_NULL;

    if ((EC_NULL == pParms) || (EC_NULL == pParms->pCallerData))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    pNotificationHandler = ((T_EC_DEMO_APP_CONTEXT*)pParms->pCallerData)->pNotificationHandler;

    /* 通知分两类：
     * - EC_NOTIFY_APP 范围：留给应用自定义（走 myAppNotify）
     * - 其他：交给默认通知处理器（CEmNotification::ecatNotify）
     */
    if ((dwCode >= EC_NOTIFY_APP) && (dwCode <= EC_NOTIFY_APP + EC_NOTIFY_APP_MAX_CODE))
    {
        /* notification for application */
        dwRetVal = myAppNotify(dwCode - EC_NOTIFY_APP, pParms);
    }
    else
    {
        /* default notification handler */
        dwRetVal = pNotificationHandler->ecatNotify(dwCode, pParms);
    }

Exit:
    return dwRetVal;
}

/********************************************************************************/
/** \brief  RAS notification handler
 *
 * \return EC_E_NOERROR or error code
 */
#ifdef INCLUDE_RAS_SERVER
static EC_T_DWORD RasNotifyCallback(
    EC_T_DWORD         dwCode,
    EC_T_NOTIFYPARMS*  pParms
)
{
    EC_T_DWORD dwRetVal = EC_E_NOERROR;
    CEmNotification* pNotificationHandler = EC_NULL;

    if ((EC_NULL == pParms) || (EC_NULL == pParms->pCallerData))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    pNotificationHandler = (CEmNotification*)pParms->pCallerData;
    dwRetVal = pNotificationHandler->emRasNotify(dwCode, pParms);

Exit:
    return dwRetVal;
}
#endif

/*-MYAPP---------------------------------------------------------------------*/

/***************************************************************************************************/
/**
\brief  Initialize Application

\return EC_E_NOERROR on success, error code otherwise.
*/
static EC_T_DWORD myAppInit(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    MT_Init(pAppContext);
    pthread_t tid;
    pthread_create(&tid, nullptr, CmdThread, nullptr);
    pthread_detach(tid);
    EC_UNREFPARM(pAppContext);

    return EC_E_NOERROR;
}

/***************************************************************************************************/
/**
\brief  Initialize Slave Instance.

Find slave parameters.
\return EC_E_NOERROR on success, error code otherwise.
*/
static EC_T_DWORD myAppPrepare(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    EC_T_DWORD          dwRes      = EC_E_NOERROR;
    T_MY_APP_DESC*      pMyAppDesc = pAppContext->pMyAppDesc;
    EC_T_CFG_SLAVE_INFO oCfgSlaveInfo;
    OsMemset(&oCfgSlaveInfo, 0, sizeof(EC_T_CFG_SLAVE_INFO));

    if (EC_NULL != pAppContext->AppParms.pbyCnfData)
    {
        /* [修改] 增加到 7 个电机，站号从 1001 到 1007 */
        for (int i = 0; i < 7; i++) {
            My_Slave[i].wStationAddress = (EC_T_WORD)(1001 + i);
            My_Slave[i].wAxisCnt = 1;
        }

        MT_Prepare(pAppContext);
    }

    if (pAppContext->AppParms.bFlash)
    {
        EC_T_WORD wFlashSlaveAddr = pAppContext->AppParms.wFlashSlaveAddr;

        /* check if slave address is provided */
        if (wFlashSlaveAddr != INVALID_FIXED_ADDR)
        {
            /* get slave's process data offset and some other infos */
            dwRes = ecatGetCfgSlaveInfo(EC_TRUE, wFlashSlaveAddr, &oCfgSlaveInfo);
            if (dwRes != EC_E_NOERROR)
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare: ecatGetCfgSlaveInfo() returns with error=0x%x, slave address=%d\n", dwRes, wFlashSlaveAddr));
                goto Exit;
            }

            if (oCfgSlaveInfo.dwPdSizeOut != 0)
            {
                pMyAppDesc->dwFlashPdBitSize = oCfgSlaveInfo.dwPdSizeOut;
                pMyAppDesc->dwFlashPdBitOffs = oCfgSlaveInfo.dwPdOffsOut;
            }
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare: Slave address=%d has no outputs, therefore flashing not possible\n", wFlashSlaveAddr));
            }
        }
        else
        {
            /* get complete process data output size */
            EC_T_MEMREQ_DESC oPdMemorySize;
            OsMemset(&oPdMemorySize, 0, sizeof(EC_T_MEMREQ_DESC));

            dwRes = ecatIoCtl(EC_IOCTL_GET_PDMEMORYSIZE, EC_NULL, 0, &oPdMemorySize, sizeof(EC_T_MEMREQ_DESC), EC_NULL);
            if (dwRes != EC_E_NOERROR)
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare: ecatIoControl(EC_IOCTL_GET_PDMEMORYSIZE) returns with error=0x%x\n", dwRes));
                goto Exit;
            }
            pMyAppDesc->dwFlashPdBitSize = oPdMemorySize.dwPDOutSize * 8;
        }
        if (pMyAppDesc->dwFlashPdBitSize > 0)
        {
            pMyAppDesc->dwFlashInterval = 20000; /* flash every 20 msec */
            pMyAppDesc->dwFlashBufSize = BIT2BYTE(pMyAppDesc->dwFlashPdBitSize);
            pMyAppDesc->pbyFlashBuf = (EC_T_BYTE*)OsMalloc(pMyAppDesc->dwFlashBufSize);
            if (EC_NULL == pMyAppDesc->pbyFlashBuf)
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppPrepare: no memory left \n"));
                goto Exit;
            }
            OsMemset(pMyAppDesc->pbyFlashBuf, 0 , pMyAppDesc->dwFlashBufSize);
        }
    }

Exit:
    return EC_E_NOERROR;
}

/***************************************************************************************************/
/**
\brief  Setup slave parameters (normally done in PREOP state)

  - SDO up- and Downloads
  - Read Object Dictionary

\return EC_E_NOERROR on success, error code otherwise.
*/
static EC_T_DWORD myAppSetup(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    EC_T_DWORD dwRetVal = EC_E_NOERROR;
    EC_T_DWORD dwRes    = EC_E_NOERROR;

    MT_Setup(pAppContext);

    /* read CoE object dictionary from device */
    if (pAppContext->AppParms.bReadOD)
    {
        EC_T_BOOL bStopReading = EC_FALSE;
        dwRes = CoeReadObjectDictionary(pAppContext, &bStopReading, emGetSlaveId(pAppContext->dwInstanceId, pAppContext->AppParms.wReadODSlaveAddr), EC_TRUE, MBX_TIMEOUT);
        if (dwRes != EC_E_NOERROR)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: myAppSetup: CoeReadObjectDictionary %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
            dwRetVal = dwRes;
            goto Exit;
        }
    }

Exit:
    return dwRetVal;
}

/***************************************************************************************************/
/**
\brief  demo application working process data function.

  This function is called in every cycle after the the master stack is started.

*/
static EC_T_DWORD myAppWorkpd(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    T_MY_APP_DESC* pMyAppDesc = pAppContext->pMyAppDesc;
    EC_T_BYTE*     pbyPdOut   = ecatGetProcessImageOutputPtr();

    MT_Workpd(pAppContext);

    /* demo code flashing */
    if (pMyAppDesc->dwFlashPdBitSize != 0)
    {
        pMyAppDesc->dwFlashTimer += pAppContext->AppParms.dwBusCycleTimeUsec;
        if (pMyAppDesc->dwFlashTimer >= pMyAppDesc->dwFlashInterval)
        {
            pMyAppDesc->dwFlashTimer = 0;

            /* flash with pattern */
            pMyAppDesc->byFlashVal++;
            OsMemset(pMyAppDesc->pbyFlashBuf, pMyAppDesc->byFlashVal, pMyAppDesc->dwFlashBufSize);

            /* update PdOut */
            EC_COPYBITS(pbyPdOut, pMyAppDesc->dwFlashPdBitOffs, pMyAppDesc->pbyFlashBuf, 0, pMyAppDesc->dwFlashPdBitSize);
        }
    }
    return EC_E_NOERROR;
}

/***************************************************************************************************/
/**
\brief  demo application doing some diagnostic tasks

  This function is called in sometimes from the main demo task
*/
static EC_T_DWORD myAppDiagnosis(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    EC_UNREFPARM(pAppContext);
    return EC_E_NOERROR;
}

/********************************************************************************/
/** \brief  Handler for application notifications, see emNotifyApp()
 *
 * \return EC_E_NOERROR on success, error code otherwise.
 */
static EC_T_DWORD myAppNotify(
    EC_T_DWORD        dwCode, /* [in] Application notification code */
    EC_T_NOTIFYPARMS* pParms  /* [in] Notification parameters */
)
{
    EC_T_DWORD dwRetVal = EC_E_INVALIDPARM;
    T_EC_DEMO_APP_CONTEXT* pAppContext = (T_EC_DEMO_APP_CONTEXT*)pParms->pCallerData;

    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "myAppNotify: Unhandled notification code %d received\n", dwCode));

    return dwRetVal;
}

EC_T_VOID ShowSyntaxAppUsage(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    const EC_T_CHAR* szAppUsage = "<LinkLayer> [-f ENI-FileName] [-t time] [-b cycle time] [-a affinity] [-v lvl] [-perf [level]] [-log prefix [msg cnt]] [-lic key] [-oem key] [-maxbusslaves cnt] [-flash address] [-readod address] [-printvars]"
#if (defined INCLUDE_RAS_SERVER)
        " [-sp [port]]"
#endif
        " [-dcmmode mode [synctocyclestart]] [-ctloff]"
#if (defined INCLUDE_PCAP_RECORDER)
        " [-rec [prefix [frame cnt]]]"
#endif
        " [-junctionred]\n"
        ;
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "%s V%s for %s %s\n", EC_DEMO_APP_NAME, EC_FILEVERSIONSTR, ATECAT_PLATFORMSTR, EC_COPYRIGHT));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Syntax:\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "%s %s", EC_DEMO_APP_NAME, szAppUsage));
}

EC_T_VOID ShowSyntaxApp(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -maxbusslaves              Max number of slaves\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     cnt                      Default = %d\n", MASTER_CFG_ECAT_MAX_BUS_SLAVES));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -flash                     Flash outputs\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     address                  0 = all, >0 = slave station address\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -readod                    Read CoE object dictionary from device\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     address                  0 = MASTER_SLAVE_ID, >0 = slave station address\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -printvars                 Print process variable name and offset for all variables of all slaves\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -dcmmode                   Set DCM mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     off                      Off (default)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     busshift                 BusShift mode (default if configured in ENI)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     mastershift              MasterShift mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     masterrefclock           MasterRefClock mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     linklayerrefclock        LinkLayerRefClock mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     dcx                      External synchronization mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     [synctocyclestart        Sync to cycle start: 0 = disabled (default), 1 = enabled]\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -dcmlog                    Enable DCM logging (default: disabled)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -ctloff                    Disable DCM control loop for diagnosis (default: enabled)\n"));
#if (defined INCLUDE_PCAP_RECORDER)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -rec                       Record network traffic to pcap file\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [prefix                   Pcap file name prefix\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [frame cnt]               Frame count for log buffer allocation (default = %d, with %d bytes per message)]\n", PCAP_RECORDER_BUF_FRAME_CNT, ETHERNET_MAX_FRAMEBUF_LEN));
#endif
}
//2026-1-13 输入线程
static void* CmdThread(void*)
{
    char line[256];

    /* [2026-01-22] 启动时自动加载标定文件 */
    printf("\n[系统] 正在加载标定数据...\n");
    if (!LoadCalibrationFile(CALIB_FILE_NAME)) {
        printf("[系统] 提示: 未找到标定文件，请先运行 calib 命令进行标定\n");
        printf("[系统] 标定完成后，home 命令才能正常使用\n\n");
    }

    /* [2026-01-14] 目的：启动后先选择运行模式（0自动demo / 1手动命令） */
    printf("[CMD] 请选择模式: 0=自动demo  1=手动命令\n");
    printf("[CMD] 输入 0 或 1 后回车: ");
    fflush(stdout);

    /* [2026-01-23] 默认使用 CSP 模式 */
    MT_SetGlobalDriveMode(DRIVE_MODE_CSP);
    printf("[CMD] 默认全局模式: CSP (位置控制)\n");
    fflush(stdout);
    
    printf("\n[CMD] === 紧急命令 ===\n");
    printf("  estop / e                                   紧急停止（所有轴立即释放）\n");
    printf("\n[CMD] === 控制命令 ===\n");
    printf("  mode pt|csp|cst                             选择全局控制模式\n");
    printf("  set <axis> ...                              根据当前模式下发控制\n");
    printf("    PT模式:  set <axis> <tau> <kp> <q> <kd> <dq>\n");
    printf("    CSP模式: set <axis> <q> <dq>\n");
    printf("    CST模式: set <axis> <tau>\n");
    printf("  enable <axis>                               使能电机\n");
    printf("  stop <axis>                                 停机/释放\n");
    printf("\n[CMD] === 运动功能 ===\n");
    printf("  show                                        显示所有轴位置\n");
    printf("  get <axis>                                  获取单轴详细状态\n");
    printf("  home                                        回零\n");
    printf("  aging <axis> <speed>                        老化测试\n");
    printf("  calib                                       标定\n");
    printf("  torque <axis> <tau> <kp> <q> <kd> <dq>       阻抗控制/PT模式\n");
    fflush(stdout);

    while (fgets(line, sizeof(line), stdin) != nullptr) {
        // 去掉换行
        line[strcspn(line, "\r\n")] = 0;

        /* [2026-01-26] 帮助命令 */
        if (strcmp(line, "help") == 0 || strcmp(line, "h") == 0 || strcmp(line, "?") == 0) {
            printf("\n");
            printf("╔══════════════════════════════════════════════════════════════════╗\n");
            printf("║                        命令帮助                                  ║\n");
            printf("╠══════════════════════════════════════════════════════════════════╣\n");
            printf("║ 【紧急命令】                                                     ║\n");
            printf("║   estop / e              紧急停止（所有轴立即释放）              ║\n");
            printf("╠══════════════════════════════════════════════════════════════════╣\n");
            printf("║ 【模式选择】 (先选模式，再发控制命令)                            ║\n");
            printf("║   mode pt                切换到 PT 模式（阻抗控制）              ║\n");
            printf("║   mode csp               切换到 CSP 模式（位置控制）[默认]       ║\n");
            printf("║   mode cst               切换到 CST 模式（纯力矩控制）           ║\n");
            printf("╠══════════════════════════════════════════════════════════════════╣\n");
            printf("║ 【控制命令】                                                     ║\n");
            printf("║   enable <axis>          使能电机并保持当前位置                  ║\n");
            printf("║   stop <axis>            停机/释放单轴                           ║\n");
            printf("║   set <axis> ...         根据当前模式下发控制：                  ║\n");
            printf("║     PT模式:  set <axis> <tau> <kp> <q> <kd> <dq>                 ║\n");
            printf("║              tau=前馈扭矩, kp=刚度, q=目标位置                   ║\n");
            printf("║              kd=阻尼, dq=目标速度                                ║\n");
            printf("║     CSP模式: set <axis> <q> <dq>                                 ║\n");
            printf("║              q=目标位置(rad), dq=速度(rad/s)                     ║\n");
            printf("║     CST模式: set <axis> <tau>                                    ║\n");
            printf("║              tau=目标力矩(N.m)                                   ║\n");
            printf("╠══════════════════════════════════════════════════════════════════╣\n");
            printf("║ 【运动功能】                                                     ║\n");
            printf("║   home                   回零（顺序：7→1）                       ║\n");
            printf("║   aging <axis> <speed>   老化测试                                ║\n");
            printf("║   calib                  标定（记录负极限位置）                  ║\n");
            printf("╠══════════════════════════════════════════════════════════════════╣\n");
            printf("║ 【状态查询】                                                     ║\n");
            printf("║   show                   显示所有轴位置                          ║\n");
            printf("║   get <axis>             获取单轴详细状态                        ║\n");
            printf("╠══════════════════════════════════════════════════════════════════╣\n");
            printf("║ 【参数说明】                                                     ║\n");
            printf("║   <axis>  轴号 1-7                                               ║\n");
            printf("║   <q>     位置，单位 rad                                         ║\n");
            printf("║   <dq>    速度，单位 rad/s                                       ║\n");
            printf("║   <tau>   力矩，单位 N.m                                         ║\n");
            printf("║   <kp>    刚度系数                                               ║\n");
            printf("║   <kd>    阻尼系数                                               ║\n");
            printf("╚══════════════════════════════════════════════════════════════════╝\n");
            printf("\n");
            fflush(stdout);
            continue;
        }

        /* [2026-01-23] 紧急停止：所有轴立即停止 */
        if (strcmp(line, "estop") == 0 || strcmp(line, "e") == 0) {
            printf("\n[ESTOP] !!! 紧急停止 - 所有轴停机 !!!\n");
            for (int i = 0; i < 7; i++) {
                MotorCmd_ cmd{};
                cmd.motion_func = MOTION_SHUTDOWN;
                MT_SetMotorCmd((EC_T_WORD)i, &cmd);
            }
            printf("[ESTOP] 所有轴已释放\n\n");
            fflush(stdout);
            continue;
        }

        /* [2026-01-23] 全局驱动模式选择 */
        if (strncmp(line, "mode ", 5) == 0)
        {
            char mode_str[16] = {0};
            sscanf(line + 5, "%s", mode_str);
            
            if (strcmp(mode_str, "pt") == 0) {
                MT_SetGlobalDriveMode(DRIVE_MODE_PT);
                printf("[CMD] OK: 全局模式 = PT (阻抗控制)\n");
                printf("     set 用法: set <axis> <tau> <kp> <q> <kd> <dq>\n");
            } else if (strcmp(mode_str, "csp") == 0) {
                MT_SetGlobalDriveMode(DRIVE_MODE_CSP);
                printf("[CMD] OK: 全局模式 = CSP (位置控制)\n");
                printf("     set 用法: set <axis> <q> <dq>\n");
            } else if (strcmp(mode_str, "cst") == 0) {
                MT_SetGlobalDriveMode(DRIVE_MODE_CST);
                printf("[CMD] OK: 全局模式 = CST (力矩控制)\n");
                printf("     set 用法: set <axis> <tau>\n");
            } else {
                printf("[CMD] 用法: mode pt|csp|cst\n");
            }
            fflush(stdout);
            continue;
        }

        /* [2026-01-20] 一键查看所有轴位置 (显示为 1-7 号轴) */
        if (strcmp(line, "show") == 0) {
            printf("\n[CMD] --- Current 7-Axis Positions (rad) ---\n");
            for (int i = 0; i < 7; i++) {
                MotorState_ st;
                if (MT_GetMotorState((EC_T_WORD)i, &st)) {
                    printf("  Axis %d: %8.4f\n", i + 1, st.q_fb);
                }
            }
            printf("--------------------------------------------\n");
            fflush(stdout);
            continue;
        }

        /* [2026-01-20] 示教限位 (轴号由 1-7 转为 0-6) */
        if (strncmp(line, "teach_min ", 10) == 0) {
            int axis = 0;
            if (sscanf(line + 10, "%d", &axis) == 1) {
                MT_TeachLimit((EC_T_WORD)(axis - 1), EC_FALSE);
                printf("[CMD] OK: Recorded MIN limit for Axis %d\n", axis);
            }
            continue;
        }
        if (strncmp(line, "teach_max ", 10) == 0) {
            int axis = 0;
            if (sscanf(line + 10, "%d", &axis) == 1) {
                MT_TeachLimit((EC_T_WORD)(axis - 1), EC_TRUE);
                printf("[CMD] OK: Recorded MAX limit for Axis %d\n", axis);
            }
            continue;
        }

        /* [2026-01-20] 快捷使能命令 (轴号 1-7) */
        if (strncmp(line, "enable ", 7) == 0) {
            // ... 保持原有逻辑 ...
            int axis = 0;
            if (sscanf(line + 7, "%d", &axis) == 1) {
                MotorState_ st;
                MotorCmd_ cmd{};
                if (MT_GetMotorState((EC_T_WORD)(axis - 1), &st)) {
                    cmd.q = st.q_fb;
                } else {
                    cmd.q = 0;
                }
                cmd.kp = 32.0f;
                cmd.kd = 30.0f;
                cmd.drive_mode = MT_GetGlobalDriveMode();
                cmd.motion_func = MOTION_CONTROL;
                MT_SetMotorCmd((EC_T_WORD)(axis - 1), &cmd);
                printf("[CMD] OK: enabling axis %d at pos %.3f with default KP/KD\n", axis, cmd.q);
            }
            continue;
        }

        /* [2026-01-20] 老化测试专属命令：aging <轴号> <速度> */
        if (strncmp(line, "aging ", 6) == 0) {
            int axis = 0;
            float speed = 0;
            if (sscanf(line + 6, "%d %f", &axis, &speed) == 2) {
                // [2026-01-22] 检查是否已加载标定数据（aging 需要软件限位保护）
                if (!g_CalibData.loaded) {
                    printf("[CMD] 错误: 未加载标定数据，aging 命令需要软件限位保护\n");
                    printf("[CMD] 请先运行 calib 命令进行标定\n");
                    fflush(stdout);
                    continue;
                }
                
                MotorCmd_ cmd{};
                cmd.drive_mode = MT_GetGlobalDriveMode();
                cmd.motion_func = MOTION_AGING;
                cmd.dq   = speed; // 设置运行速度
                cmd.kp   = 32.0f; // 阻抗参数
                cmd.kd   = 30.0f;
                MT_SetMotorCmd((EC_T_WORD)(axis - 1), &cmd);
                printf("[CMD] OK: Start aging for Axis %d at speed %.3f rad/s\n", axis, speed);
                printf("[CMD] 软件限位: [%.4f, %.4f] rad\n", 
                       g_CalibData.soft_limit_min[axis - 1], 
                       g_CalibData.soft_limit_max[axis - 1]);
            } else {
                printf("[CMD] 用法: aging <1-7> <速度>\n");
            }
            continue;
        }
        /* [2026-01-22] 扭矩控制命令：torque <axis> <tau> <kp> <q_des> <kd> <dq_des> */
        if (strncmp(line, "torque ", 7) == 0) {
            int axis = 0;
            float tau = 0, kp = 0, q_des = 0, kd = 0, dq_des = 0;
            if (sscanf(line + 7, "%d %f %f %f %f %f", &axis, &tau, &kp, &q_des, &kd, &dq_des) == 6) {
                // 软件限位检查 (检查目标位置)
                if (!CheckSoftLimit(axis - 1, q_des)) {
                    printf("[CMD] 拒绝: 目标位置超出软件限位范围\n");
                    fflush(stdout);
                    continue;
                }
                
                MotorCmd_ cmd{};
                cmd.drive_mode = DRIVE_MODE_PT;
                cmd.motion_func = MOTION_CONTROL;
                cmd.tau = tau;    // 前馈扭矩 (N·m)
                cmd.kp = kp;      // 刚度系数
                cmd.q = q_des;    // 目标位置 (rad)
                cmd.kd = kd;      // 阻尼系数
                cmd.dq = dq_des;  // 目标速度 (rad/s)
                
                MT_SetMotorCmd((EC_T_WORD)(axis - 1), &cmd);
                printf("[CMD] OK: Torque control axis=%d tau=%.3f kp=%.1f q=%.3f kd=%.1f dq=%.3f\n",
                       axis, tau, kp, q_des, kd, dq_des);
            } else {
                printf("[CMD] 用法: torque <1-7> <tau> <kp> <q_des> <kd> <dq_des>\n");
                printf("      tau: 前馈扭矩(N·m), kp: 刚度, q_des: 目标位置(rad)\n");
                printf("      kd: 阻尼, dq_des: 目标速度(rad/s)\n");
            }
            fflush(stdout);
            continue;
        }
        /* [2026-01-22] 标定命令：读取 7 个关节位置并生成 JSON 文件 */
        if (strcmp(line, "calib") == 0) {
            printf("\n[标定] ========================================\n");
            printf("[标定] 请将 7 个关节手动反转到【负向极限】位置\n");
            printf("[标定] 完成后输入 ok 确认: ");
            fflush(stdout);
            
            char confirm[64];
            if (fgets(confirm, sizeof(confirm), stdin) != nullptr) {
                confirm[strcspn(confirm, "\r\n")] = 0;
                if (strcmp(confirm, "ok") == 0) {
                    printf("[标定] 正在读取关节位置...\n");
                    
                    // 7 个关节的固定参数 (URDF 命名)
                    const char* joint_names[7] = {"arm_j1_l", "arm_j2_l", "arm_j3_l", "arm_j4_l", "arm_j5_l", "arm_j6_l", "arm_j7_l"};
                    const int directions[7] = {0, 0, 0, -1, -1, -1, -1};  // 0=正向, -1=负向
                    const float range_min[7] = {-3.53f, -1.78f, -0.82f, -2.39f, -1.61f, -0.35f, -1.61f};
                    const float range_max[7] = { 1.43f,  0.91f,  0.82f,  0.30f,  1.61f,  0.35f,  1.61f};
                    const int total_joints = 7;
                    
                    // 读取实际位置
                    float positions[7];
                    for (int i = 0; i < total_joints; i++) {
                        MotorState_ st;
                        if (MT_GetMotorState((EC_T_WORD)i, &st)) {
                            positions[i] = st.q_fb;
                            printf("[标定] %s: %.4f rad\n", joint_names[i], positions[i]);
                        } else {
                            positions[i] = 0.0f;
                            printf("[标定] %s: 读取失败，使用 0.0\n", joint_names[i]);
                        }
                    }
                    
                    // 生成 JSON 文件
                    std::ofstream json("robot_calib.json");
                    if (json.is_open()) {
                        // 获取当前时间
                        time_t now = time(nullptr);
                        char timebuf[64];
                        strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", localtime(&now));
                        
                        json << "{\n";
                        json << "  \"robot_calibration\": {\n";
                        json << "    \"total_joints\": " << total_joints << ",\n";
                        json << "    \"timestamp\": \"" << timebuf << "\",\n";
                        json << "    \"joints\": [\n";
                        
                        for (int i = 0; i < total_joints; i++) {
                            json << "      {\n";
                            json << "        \"index\": " << (i + 1) << ",\n";
                            json << "        \"name\": \"" << joint_names[i] << "\",\n";
                            json << "        \"communication\": \"EtherCAT\",\n";
                            json << "        \"position\": " << positions[i] << ",\n";
                            json << "        \"direction\": " << directions[i] << ",\n";
                            json << "        \"range_min\": " << range_min[i] << ",\n";
                            json << "        \"range_max\": " << range_max[i] << "\n";
                            json << "      }" << (i < total_joints - 1 ? "," : "") << "\n";
                        }
                        
                        json << "    ]\n";
                        json << "  }\n";
                        json << "}\n";
                        json.close();
                        printf("[标定] 已生成配置文件: %s\n", CALIB_FILE_NAME);
                        
                        // [2026-01-22] 生成后自动加载标定数据
                        printf("[标定] 正在加载标定数据...\n");
                        if (LoadCalibrationFile(CALIB_FILE_NAME)) {
                            printf("[标定] 标定数据已加载\n");
                            
                            // [2026-01-22] 将软限位写入驱动器
                            WriteSoftLimitsToDrive();
                            
                            printf("[标定] 现在可以使用 home 命令回零\n");
                        }
                    } else {
                        printf("[标定] 错误: 无法创建文件 %s\n", CALIB_FILE_NAME);
                    }
                } else {
                    printf("[标定] 已取消\n");
                }
            }
            printf("[标定] ========================================\n\n");
            fflush(stdout);
            continue;
        }

        /* [2026-01-22] 回零命令：根据标定文件将所有关节移动到零点位置 */
        if (strcmp(line, "home") == 0) {
            printf("\n[回零] ========================================\n");
            
            // 检查是否已加载标定数据
            if (!g_CalibData.loaded) {
                printf("[回零] 错误: 未加载标定数据！\n");
                printf("[回零] 请先运行 calib 命令进行标定，或确保 %s 文件存在\n", CALIB_FILE_NAME);
                printf("[回零] ========================================\n\n");
                fflush(stdout);
                continue;
            }
            
            // [2026-01-22] 检查软限位是否已写入驱动器，如果没有则写入
            if (!g_CalibData.limits_written_to_drive) {
                printf("[回零] 软限位尚未写入驱动器，正在写入...\n");
                WriteSoftLimitsToDrive();
            }
            
            // 使用已加载的标定数据计算零点位置
            float zero_positions[TOTAL_JOINTS];
            printf("[回零] 使用标定数据计算零点位置...\n");
            for (int i = 0; i < g_CalibData.total_joints; i++) {
                float offset = -g_CalibData.range_min[i];  // |range_min|
                if (g_CalibData.directions[i] == 0) {
                    // 正向关节：零点 = 标定位置 + |range_min|
                    zero_positions[i] = g_CalibData.calib_positions[i] + offset;
                } else {
                    // 负向关节：零点 = 标定位置 - |range_min|
                    zero_positions[i] = g_CalibData.calib_positions[i] - offset;
                }
                printf("  %s: 标定=%.4f, 零点=%.4f (dir=%d)\n", 
                       g_CalibData.joint_names[i], g_CalibData.calib_positions[i], zero_positions[i], g_CalibData.directions[i]);
            }
            
            printf("\n[回零] 开始回零，顺序：%d→...→1\n", g_CalibData.total_joints);
            printf("[回零] 速度: 0.5 rad/s\n");
            printf("[回零] 按回车开始，或输入 cancel 取消: ");
            fflush(stdout);
            
            char confirm[64];
            if (fgets(confirm, sizeof(confirm), stdin) != nullptr) {
                confirm[strcspn(confirm, "\r\n")] = 0;
                if (strcmp(confirm, "cancel") == 0) {
                    printf("[回零] 已取消\n");
                } else {
                    // [2026-01-26] 先把所有轴设为 SHUTDOWN，确保其他轴不会乱动
                    printf("[回零] 正在停止所有轴...\n");
                    for (int j = 0; j < g_CalibData.total_joints; j++) {
                        MotorCmd_ shutdown_cmd{};
                        shutdown_cmd.motion_func = MOTION_SHUTDOWN;
                        MT_SetMotorCmd((EC_T_WORD)j, &shutdown_cmd);
                    }
                    OsSleep(300);  // 等待所有轴停止
                    
                    // 从最后一个轴到第 1 个轴逐个回零
                    for (int i = g_CalibData.total_joints - 1; i >= 0; i--) {
                        int axis_num = i + 1;  // 显示用的轴号 1-7
                        printf("\n[回零] >>> 轴 %d (%s) 开始回零...\n", axis_num, g_CalibData.joint_names[i]);
                        
                        // 0. 先释放电机，重置位置同步标志
                        MotorCmd_ cmd{};
                        cmd.motion_func = MOTION_SHUTDOWN;
                        MT_SetMotorCmd((EC_T_WORD)i, &cmd);
                        OsSleep(200);  // 等待释放完成
                        
                        // 1. 再使能电机（会自动同步当前位置）
                        MotorState_ st;
                        MT_GetMotorState((EC_T_WORD)i, &st);
                        cmd.q = st.q_fb;  // 先同步到当前位置
                        cmd.kp = 32.0f;
                        cmd.kd = 30.0f;
                        cmd.direction = (EC_T_SBYTE)g_CalibData.directions[i];  // 设置方向（用于PT模式扭矩反转）
                        cmd.drive_mode = MT_GetGlobalDriveMode();  // 跟随全局模式
                        cmd.motion_func = MOTION_CONTROL;
                        MT_SetMotorCmd((EC_T_WORD)i, &cmd);
                        OsSleep(500);  // 等待使能稳定
                        
                        // 2. 设置目标位置为零点
                        cmd.q = zero_positions[i];
                        cmd.dq = 0.5f;  // 固定速度 0.5 rad/s
                        MT_SetMotorCmd((EC_T_WORD)i, &cmd);
                        
                        // 3. 等待到达目标位置
                        float target = zero_positions[i];
                        int timeout = 30000;  // 最长等待 30 秒
                        int elapsed = 0;
                        float last_pos = st.q_fb;
                        int stuck_count = 0;  // 卡住计数器
                        
                        while (elapsed < timeout) {
                            OsSleep(100);
                            elapsed += 100;
                            MT_GetMotorState((EC_T_WORD)i, &st);
                            float error = st.q_fb - target;
                            if (error < 0) error = -error;  // 取绝对值
                            
                            // 每秒打印一次进度
                            if (elapsed % 1000 == 0) {
                                printf("  [轴%d] 当前: %.4f, 目标: %.4f, 误差: %.4f\n", 
                                       axis_num, st.q_fb, target, error);
                            }
                            
                            // 误差小于 0.01 rad 认为到达（约 0.6°）
                            if (error < 0.02f) {
                                printf("  [轴%d] ✓ 已到达零点位置: %.4f rad (误差: %.4f)\n", axis_num, st.q_fb, error);
                                break;
                            }
                            
                            // 检测是否卡住（位置 3 秒没变化）
                            float pos_change = st.q_fb - last_pos;
                            if (pos_change < 0) pos_change = -pos_change;
                            if (pos_change < 0.001f) {
                                stuck_count++;
                                if (stuck_count >= 30) {  // 3 秒没动
                                    printf("  [轴%d] ⚠ 电机卡住，当前: %.4f, 目标: %.4f, 误差: %.4f\n", 
                                           axis_num, st.q_fb, target, error);
                                    break;
                                }
                            } else {
                                stuck_count = 0;
                                last_pos = st.q_fb;
                            }
                        }
                        
                        if (elapsed >= timeout) {
                            printf("  [轴%d] ✗ 超时，当前位置: %.4f\n", axis_num, st.q_fb);
                        }
                    }
                    
                    // [2026-01-26] 回零完成后，把所有轴的 dq 设为 0，防止 PT 模式下持续产生扭矩
                    for (int i = 0; i < g_CalibData.total_joints; i++) {
                        MotorCmd_ cmd{};
                        MotorState_ st;
                        MT_GetMotorState((EC_T_WORD)i, &st);
                        cmd.q = zero_positions[i];  // 保持在零点
                        cmd.dq = 0.0f;              // 速度设为 0！
                        cmd.kp = 32.0f;
                        cmd.kd = 30.0f;
                        cmd.tau = 0.0f;
                        cmd.direction = (EC_T_SBYTE)g_CalibData.directions[i];
                        cmd.drive_mode = MT_GetGlobalDriveMode();
                        cmd.motion_func = MOTION_CONTROL;
                        MT_SetMotorCmd((EC_T_WORD)i, &cmd);
                    }
                    
                    printf("\n[回零] ========================================\n");
                    printf("[回零] 回零完成！所有轴保持在零点位置。\n");
                }
            }
            printf("[回零] ========================================\n\n");
            fflush(stdout);
            continue;
        }

        if (strncmp(line, "disable ", 8) == 0) {
            int axis = 0;
            if (sscanf(line + 8, "%d", &axis) == 1) {
                MotorCmd_ cmd{};
                cmd.motion_func = MOTION_SHUTDOWN;
                MT_SetMotorCmd((EC_T_WORD)(axis - 1), &cmd);
                printf("[CMD] OK: disabling axis %d\n", axis);
            }
            continue;
        }

        /* [2026-01-23] 重构：set 命令根据全局模式自动解析 */
        if (strncmp(line, "set ", 4) == 0) {
            int axis = 0;
            sscanf(line + 4, "%d", &axis);
            
            DriveMode mode = MT_GetGlobalDriveMode();
            MotorCmd_ cmd{};
            cmd.drive_mode = mode;
            cmd.motion_func = MOTION_CONTROL;
            
            if (mode == DRIVE_MODE_PT) {
                // PT模式: set <axis> <tau> <kp> <q> <kd> <dq>
                float tau, kp, q, kd, dq;
                if (sscanf(line + 4, "%*d %f %f %f %f %f", &tau, &kp, &q, &kd, &dq) == 5) {
                    if (!CheckSoftLimit(axis - 1, q)) {
                        printf("[CMD] 拒绝: 目标位置超出软件限位范围\n");
                        fflush(stdout);
                        continue;
                    }
                    cmd.tau = tau;
                    cmd.kp = kp;
                    cmd.q = q;
                    cmd.kd = kd;
                    cmd.dq = dq;
                    MT_SetMotorCmd((EC_T_WORD)(axis - 1), &cmd);
                    printf("[CMD] OK: axis=%d [PT] tau=%.2f kp=%.1f q=%.2f kd=%.1f dq=%.2f\n",
                           axis, tau, kp, q, kd, dq);
                } else {
                    printf("[CMD] PT模式用法: set <axis> <tau> <kp> <q> <kd> <dq>\n");
                }
            } else if (mode == DRIVE_MODE_CSP) {
                // CSP模式: set <axis> <q> <dq>
                float q, dq;
                if (sscanf(line + 4, "%*d %f %f", &q, &dq) == 2) {
                    if (!CheckSoftLimit(axis - 1, q)) {
                        printf("[CMD] 拒绝: 目标位置超出软件限位范围\n");
                        fflush(stdout);
                        continue;
                    }
                    cmd.q = q;
                    cmd.dq = dq;
                    cmd.kp = 32.0f;  // 默认阻抗参数
                    cmd.kd = 30.0f;
                    MT_SetMotorCmd((EC_T_WORD)(axis - 1), &cmd);
                    printf("[CMD] OK: axis=%d [CSP] q=%.3f dq=%.2f\n", axis, q, dq);
                } else {
                    printf("[CMD] CSP模式用法: set <axis> <q> <dq>\n");
                }
            } else if (mode == DRIVE_MODE_CST) {
                // CST模式: set <axis> <tau>
                float tau;
                if (sscanf(line + 4, "%*d %f", &tau) == 1) {
                    cmd.tau = tau;
                    MT_SetMotorCmd((EC_T_WORD)(axis - 1), &cmd);
                    printf("[CMD] OK: axis=%d [CST] tau=%.3f\n", axis, tau);
                } else {
                    printf("[CMD] CST模式用法: set <axis> <tau>\n");
                }
            }
            fflush(stdout);
            continue;
        }

        /* [2026-01-19] 目的：适配新协议字段的 get 命令 (轴号由 1-7 转为 0-6) */
        if (strncmp(line, "get ", 4) == 0)
        {
            int axis = 0;
            if (sscanf(line + 4, "%d", &axis) == 1)
            {
                MotorState_ st;
                OsMemset(&st, 0, sizeof(st));
                if (MT_GetMotorState((EC_T_WORD)(axis - 1), &st))
                {
                    printf("[CMD] --- Axis %d Feedback ---\n", axis);
                    printf("  Mode: 0x%02X | Status: 0x%08X\n", st.mode, st.motorstate);
                    printf("  Pos: %.6f rad | Vel: %.6f rad/s | Tau: %.3f N.m\n", 
                           st.q_fb, st.dq_fb, st.tau_fb);
                    printf("  Vol: %.1f V | Temp: MCU %.1f degC, Motor %.1f degC\n", 
                           st.vol, (float)st.temperature[0]*0.1f, (float)st.temperature[1]*0.1f);
                }
                else
                {
                    printf("[CMD] FAIL: get axis %d (有效范围 1-7)\n", axis);
                }
            }
            else
            {
                printf("[CMD] 用法: get <1-7>\n");
            }
            fflush(stdout);
            continue;
        }

        if (strncmp(line, "stop ", 5) == 0) {
            int axis = 0;
            if (sscanf(line + 5, "%d", &axis) == 1) {
                MotorCmd_ cmd{};
                cmd.motion_func = MOTION_SHUTDOWN;
                MT_SetMotorCmd((EC_T_WORD)(axis - 1), &cmd);
                printf("[CMD] OK: stop axis=%d\n", axis);
            } else {
                printf("[CMD] 用法: stop <1-7>\n");
            }
            fflush(stdout);
            continue;
        }

        if (strncmp(line, "scale ", 6) == 0) {
            int axis = 0;
            double cpr = 0, ratio = 0;
            if (sscanf(line + 6, "%d %lf %lf", &axis, &cpr, &ratio) == 3) {
                if (MT_SetAxisUnitScale((EC_T_WORD)(axis - 1), (EC_T_LREAL)cpr, (EC_T_LREAL)ratio)) {
                    printf("[CMD] OK: scale axis=%d cpr=%.0f ratio=%.6f\n", axis, cpr, ratio);
                } else {
                    printf("[CMD] FAIL: scale 参数不合法\n");
                }
            } else {
                printf("[CMD] 用法: scale <1-7> <encoder_cpr> <gear_ratio>\n");
            }
            fflush(stdout);
            continue;
        }

        printf("[CMD] 未识别命令: %s\n", line);
        fflush(stdout);
    }
    return nullptr;
}

/*-END OF SOURCE FILE--------------------------------------------------------*/
