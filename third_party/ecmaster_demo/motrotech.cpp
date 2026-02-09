/*-------------------------------motrotech----------------------------------
 * motrotech.cpp
 *
 * 这是一个“示例级”的伺服轴控制文件（配合 `motrotech.h`）。
 *
 * =============================================================================
 * 【变更记录 / Change log】
 * - 2026-01-13：为适配“手动控制接口（MotorCmd_/MotorState_）”做改造：
 *   1) 增加 rad/rad/s 与驱动 PUU(count) 的换算接口 `MT_SetAxisUnitScale()`
 *   2) `MT_Workpd()` 在 OP_ENABLED 时支持按 MotorCmd_.q/dq 直接写 0x607A/0x60FF
 *   3) `MT_Workpd()` 读取 0x6064/0x606C 并换算为 rad/rad/s 回填到 MotorState_
 *   4) 可选读取温度/电压对象（0x3008/0x3009/0x300F/0x300B），前提是 ENI 已映射到 TxPDO
 * =============================================================================
 *
 * =============================================================================
 * 【这份 demo 的“数据流”一图流（读代码时先记住这条链）】
 *
 *   启动阶段：
 *     1) EcDemoApp.cpp: myAppPrepare()   -> 填 My_Slave[] (站地址/轴数)
 *     2) EcDemoApp.cpp: myAppPrepare()   -> MT_Prepare()  检查从站 present + 生成 My_Motor[]
 *     3) EcDemoApp.cpp: myAppSetup()     -> MT_Setup()    根据 ENI/PDO 映射建立 PDO 指针
 *
 *   周期阶段（每个 bus cycle）：
 *     EcMasterJobTask()：
 *       - ProcessAllRxFrames  -> 更新 PdIn（从站->主站）
 *       - myAppWorkpd()       -> MT_Workpd():
 *                                a) Process_Commands(): CiA402 状态机（读 6041 写 6040）
 *                                b) 写 6060/607A/60FF 等目标（demo 是往复运动）
 *       - SendAllCycFrames    -> 发送 PdOut（主站->从站）
 *
 * 【最重要的隐含前提】
 *   - 你的 ENI 必须把相关对象映射进 PDO，否则 MT_Setup() 找不到变量 -> 指针为 EC_NULL -> 写不进去。
 *   - 本 demo 假设多轴对象索引按 “base + axis*0x800(OBJOFFSET)” 排布（见 motrotech.h）。
 * =============================================================================
 *
 * 运行流程（从上到下建议按这个顺序看）：
 * - `MT_Init()`：清空全局数组，设置默认状态/默认工作模式（默认 CSP）。
 * - `MT_Prepare()`：根据 `My_Slave[]`（上层在 `EcDemoApp.cpp` 的
 *`myAppPrepare()` 里填）
 *   - 检查从站是否 present
 *   - 生成 `My_Motor[]` 的站地址列表，并统计 MotorCount / SlaveCount
 * - `MT_Setup()`：关键！把每个轴用到的对象（0x6040/0x6041/0x607A/0x6064...）
 *   映射到主站 ProcessImage 的地址（即把 `My_Motor[].pwControlWord` 等指针指向
 *PDO 内存）。
 * - `MT_Workpd()`：每周期调用
 *   - 先 `Process_Commands()` 跑一遍简化版 CiA402 状态机（根据状态字写控制字）
 *   - 再根据状态生成目标位置/速度等（这里用一个简单的往复速度曲线演示）
 *
 * 注意：这是
 *demo，很多安全/边界/异常处理没有做全（例如限位、跟随误差处理、伺服参数配置等）。
 *---------------------------------------------------------------------------*/

/*-INCLUDES------------------------------------------------------------------*/

#include "motrotech.h"
#include "EcDemoApp.h"

/* motrotech.cpp 以 g++ 编译（见 Makefile），所以这里补上标准整型定义给 int64_t 使用 */
#include <stdint.h>

/* [2026-01-13] 常量：避免依赖 M_PI（不同编译选项下可能未定义） */
#define MT_PI 3.1415926535897932384626433832795

/* [2026-01-13] 工具函数：double -> int32 饱和（避免 rad->count 换算后溢出） */
static EC_T_INT MtSatToInt32(EC_T_LREAL x)
{
  if (x > 2147483647.0)  return (EC_T_INT)2147483647;
  if (x < -2147483648.0) return (EC_T_INT)(-2147483647 - 1);
  return (EC_T_INT)x;
}

/*-DEFINES-------------------------------------------------------------------*/
/* 位置/速度换算系数（示例用）
 * - 代码里把内部 `fCurPos` 乘以 INC_PERMM 后写入 TargetPosition（int32）
 * - 具体含义依赖你们伺服的单位（counts/mm、pulse/mm 等）和 ENI/PDO 定义
 *
 * 如果你要接你们的上层接口（rad / rad/s / Nm），这里建议改成“明确的单位换算”：
 *   target_pos[count] = q[rad] * gear_ratio * encoder_cpr / (2*pi)
 *   target_vel[count/s] = dq[rad/s] * gear_ratio * encoder_cpr / (2*pi)
 */
#define INC_PERMM 10

/* 往复运动的速度上限（示例内部单位，最终会换算写入 TargetVelocity） */
#define MAX_VEL 20

/* 加速度/减速度（示例内部单位） */
#define ACC_DEC 10

/* 匀速保持时间（秒），用于在正向/反向匀速阶段停留一段时间 */
#define CONRUNSEC 20
/*-Local Functions-----------------------------------------------------------*/
/* 简化版 CiA402 状态机与命令处理：读 0x6041，写 0x6040
 *
 * 你后续如果要把驱动模型换成“你们自定义的 MotorCmd_/MotorState_”：
 * - 这块很可能要“替换/裁剪”（不再依赖 0x6040/0x6041 的 CiA402）
 * - 或者把你们的 mode 映射到 CiA402 的控制字/模式字（取决于从站协议）
 */
static EC_T_DWORD Process_Commands(T_EC_DEMO_APP_CONTEXT *pAppContext);

/* 在 stop/shutdown 时等待轴退出 OP_ENABLED（demo
 * 级，逻辑比较粗糙，仅用于“等一等”） */
static EC_T_VOID CheckMotorStateStop(EC_T_VOID);

/*-GLOBAL VARIABLES-----------------------------------------------------------*/
/* `My_Motor[]`：每个轴的运行时上下文（包含一堆 PDO 指针）
 * `My_Slave[]`：上层配置的 slave 列表（站地址 + 轴数）
 *
 * 注意：My_Motor[] 里“指针成员”的生命周期：
 *   - 在 MT_Init()/MT_Prepare() 之后仍然可能为 EC_NULL
 *   - 只有 MT_Setup() 成功根据 ENI 找到对应 PDO entry，才会指向 PdIn/PdOut 内存
 */
My_Motor_Type My_Motor[MAX_AXIS_NUM];
SLAVE_MOTOR_TYPE My_Slave[MAX_SLAVE_NUM];

/*-LOCAL VARIABLES-----------------------------------------------------------*/
/* 轴总数（所有 slave 的轴数累加） */
static EC_T_INT MotorCount = 0;
/* slave 数量（本 demo 的 “slave” 计数，通常等于参与控制的从站个数） */
static EC_T_INT SlaveCount = 0;
/* 每个轴的“命令输入”（上层通过 MT_SetSwitch 写入） */
static volatile eStateCmd S_ProcessState[MAX_AXIS_NUM];

/* 你们的“手动控制接口”：上层写 MotorCmd_，周期里写到 PDO；周期里把 PDO 反馈填到 MotorState_ */
/* 注意：这里不要用 volatile struct，否则 C++ 里结构体拷贝/赋值会被限定符卡住。
 * demo 级场景：上层线程写 cmd、周期线程读 cmd；接受“偶尔读到中间态”的风险。
 * 若你要强一致性：请加锁或做双缓冲（两份 cmd + 版本号）。
 */
static MotorCmd_           S_MotorCmd[MAX_AXIS_NUM];
static EC_T_BOOL           S_MotorCmdValid[MAX_AXIS_NUM];
static MotorState_         S_MotorState[MAX_AXIS_NUM];
/* 总线周期时间（秒），在 MT_Setup() 里由 dwBusCycleTimeUsec 计算出来 */
static EC_T_LREAL fTimeSec = 0.0000;

/* [2026-01-23] 全局驱动模式（所有轴共用，默认 CSP） */
static DriveMode S_GlobalDriveMode = DRIVE_MODE_CSP;

/*-FUNCTION DEFINITIONS------------------------------------------------------*/

/******************************************************************************
 * MT_Init
 * 初始化本模块的全局变量/数组。
 * - 清空 `My_Motor[]` / `My_Slave[]`
 * - 给每个轴设置默认状态与默认工作模式（默认 CSP）
 *
 * 被谁调用：`EcDemoApp.cpp -> myAppInit()`
 * 返回：EC_E_NOERROR（demo 中永远成功）
 ******************************************************************************/
EC_T_DWORD MT_Init(T_EC_DEMO_APP_CONTEXT *pAppContext) {
  EcLogMsg(EC_LOG_LEVEL_INFO,
           (pEcLogContext, EC_LOG_LEVEL_INFO,
            "\n Motrotech: "
            "___________________MT_Init_______________________________"));

  /* 1) 清空“运行时上下文数组”
   * - My_Motor[]：每轴的 PDO 指针 + 内部状态
   * - My_Slave[]：从站站地址/轴数（上层会在 myAppPrepare() 里填）
   */
  OsMemset(My_Motor, 0, MAX_AXIS_NUM * sizeof(My_Motor_Type));
  OsMemset(My_Slave, 0, MAX_SLAVE_NUM * sizeof(SLAVE_MOTOR_TYPE));
  OsMemset((EC_T_VOID*)S_MotorCmd, 0, MAX_AXIS_NUM * sizeof(MotorCmd_));
  OsMemset((EC_T_VOID*)S_MotorCmdValid, 0, MAX_AXIS_NUM * sizeof(EC_T_BOOL));
  OsMemset((EC_T_VOID*)S_MotorState, 0, MAX_AXIS_NUM * sizeof(MotorState_));

  /* 2) 给每个轴设置初值（“默认模式/默认状态”）
   * 注意：这些不是从站真实状态；真实状态必须在 MT_Setup() 映射到 StatusWord 后，
   * 由 Process_Commands() 在周期里读取并解析出来。
   */
  for (EC_T_DWORD dwIndex = 0; dwIndex < MAX_AXIS_NUM; dwIndex++) {
    /* 初始状态：既未 ready，也未 enabled。真正状态要等 PDO 输入（StatusWord）到来后解析。 */
    My_Motor[dwIndex].wReqState = DRV_DEV_STATE_NOT_READY;
    My_Motor[dwIndex].wActState = DRV_DEV_STATE_NOT_READY;
    /* 默认工作模式：CSP（循环同步位置）。如果你的驱动不是 CiA402，应该把这一套换成你们自定义 mode。 */
    My_Motor[dwIndex].eModesOfOperation = DRV_MODE_OP_CSP;

    /* [2026-01-13] 默认单位换算：1 rad = 1 count（即“不换算”）。
     * 工程使用时请调用 MT_SetAxisUnitScale() 配置 encoder_cpr 与 gear_ratio。
     */
    My_Motor[dwIndex].fCntPerRad = 1.0;
    My_Motor[dwIndex].fRadPerCnt = 1.0;

    /* [2026-01-19] 目的：初始化内存命令结构体的 kp/kd 默认值（匹配 PDF 手册定义）
     * 这样做可以防止程序启动后第一次执行 set 命令时，如果参数没变也会触发一次 SDO 写入。
     */
    S_MotorCmd[dwIndex].kp = 32.0f;
    S_MotorCmd[dwIndex].kd = 30.0f;
    S_MotorCmd[dwIndex].motion_func = MOTION_IDLE; // [2026-01-23] 安全：初始设为 IDLE 模式
  }
  /* [2026-01-21] 删除硬编码，改为在 MT_Prepare() 从 SDO 读取减速比 */

  return EC_E_NOERROR;
}

EC_T_BOOL MT_SetAxisUnitScale(EC_T_WORD wAxis, EC_T_LREAL encoder_cpr, EC_T_LREAL gear_ratio)
{
  /* [2026-01-13] 作用：配置“rad <-> PUU(count)”换算系数
   *
   * - encoder_cpr：编码器分辨率（count / motor_rev）
   * - gear_ratio ：减速比（motor_rev / output_rev）
   *
   * 换算：
   *   cnt_per_rad = encoder_cpr * gear_ratio / (2*pi)
   *   q_cnt  = q_rad  * cnt_per_rad
   *   dq_cnt = dq_rad * cnt_per_rad
   */
  if (wAxis >= MAX_AXIS_NUM) {
    return EC_FALSE;
  }
  if ((encoder_cpr <= 0.0) || (gear_ratio <= 0.0)) {
    return EC_FALSE;
  }
  const EC_T_LREAL cntPerRad = (encoder_cpr * gear_ratio) / (2.0 * (EC_T_LREAL)MT_PI);
  if (cntPerRad <= 0.0) {
    return EC_FALSE;
  }
  My_Motor[wAxis].fCntPerRad = cntPerRad;
  My_Motor[wAxis].fRadPerCnt = 1.0 / cntPerRad;
  return EC_TRUE;
}

/******************************************************************************
 * MT_Prepare
 * “准备阶段”：根据上层已填好的
 *`My_Slave[]`，生成每个轴的站地址列表，并检查从站是否存在。
 *
 * 关键点：
 * - `My_Slave[]` 通常在 `EcDemoApp.cpp -> myAppPrepare()` 里填写（示例里写死
 *1001/1002）。
 * - 这里用 `ecatIsSlavePresent()` 检查从站是否 present。
 * - 然后把 `My_Motor[].wStationAddress` 填好，并累计 `MotorCount` /
 *`SlaveCount`。
 *
 * 注意：代码里 dwSlaveNum=ecatGetNumConfiguredSlaves()，但访问的是
 *My_Slave[dwSlaveIdx]； 所以要求 `My_Slave[]` 至少填到 dwSlaveNum
 *项，否则可能越界（demo 假设配置正确）。
 ******************************************************************************/
EC_T_DWORD MT_Prepare(T_EC_DEMO_APP_CONTEXT *pAppContext) {

  EcLogMsg(EC_LOG_LEVEL_INFO,
           (pEcLogContext, EC_LOG_LEVEL_INFO,
            "\n Motrotech: "
            "___________________MT_Prepare_______________________________"));

  /* MT_Prepare() 的目标是：把“从站配置(My_Slave[])”摊平成“每轴列表(My_Motor[])”。
   * 之后 MT_Setup()/MT_Workpd() 都是按轴循环（for i in MotorCount）工作。
   */
  EC_T_DWORD dwRetVal;
  EC_T_DWORD dwSlaveNum = ecatGetNumConfiguredSlaves();
  /* 这里“按配置从站数”循环，但使用的是 My_Slave[dwSlaveIdx]。
   * 所以前提是：My_Slave[] 至少填了 dwSlaveNum 项（demo 假设你们填对了）。
   */
  for (EC_T_DWORD dwSlaveIdx = 0; dwSlaveIdx < dwSlaveNum; dwSlaveIdx++) {
    EC_T_BOOL bPresent = EC_FALSE;
    /* ecatIsSlavePresent 的输入是 slaveId（由 station address 转换得到），用于判断从站是否在线/可访问 */
    EC_T_DWORD dwRes = ecatIsSlavePresent(
        ecatGetSlaveId(My_Slave[dwSlaveIdx].wStationAddress), &bPresent);
    if ((EC_E_NOERROR != dwRes) || (EC_TRUE != bPresent)) {
      EcLogMsg(EC_LOG_LEVEL_ERROR,
               (pEcLogContext, EC_LOG_LEVEL_ERROR,
                "ERROR: Slave_%d is not present (Result = %s 0x%x)",
                My_Slave[dwSlaveIdx].wStationAddress, ecatGetText(dwRes),
                dwRes));
      dwRetVal = dwRes;
      break;
    }

    /* 生成 My_Motor[]：把“每个轴”绑定到其所属 slave 的 station address */
    for (EC_T_WORD i = 0; i < My_Slave[dwSlaveIdx].wAxisCnt; i++) {
      My_Motor[MotorCount + i].wStationAddress =
          My_Slave[dwSlaveIdx].wStationAddress;
    }
    MotorCount += My_Slave[dwSlaveIdx].wAxisCnt;
    SlaveCount++;
  }

  /* 注意：这里即使发现从站不 present，当前 demo 仍然返回 EC_E_NOERROR（见 return）。
   * 如果你想把它工程化：建议 return dwRetVal，并在上层处理失败（退出/重试/降级）。
   */
  EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
                               "Motrotech: (%d) Axises have find", MotorCount));
  /* [2026-01-21] 注意：SDO 读取移到 MT_Setup()，因为 MT_Prepare() 在 INIT 状态调用，SDO 不可用 */
  return EC_E_NOERROR;
}

/******************************************************************************
 * MT_Setup
 * “映射阶段”：建立 PDO 指针映射（把对象索引 -> ProcessImage 地址）。
 *
 * 典型做法：
 * - `ecatGetProcessImageInputPtr/OutputPtr()` 拿到过程数据输入/输出缓冲区基地址
 * - `ecatGetCfgSlaveInfo()` 得到该 slave 的 PDO 变量数量
 * - `ecatGetSlaveOutpVarInfoEx()` / `ecatGetSlaveInpVarInfoEx()`
 *得到每个变量的：
 *   - 对象索引（wIndex）、子索引（wSubIndex）
 *   - 在过程映像中的位偏移（nBitOffs）
 * - 根据索引匹配我们关心的对象，然后把 `My_Motor[]` 里对应指针指向
 *`pbyPDOut/pbyPDIn + nBitOffs/8`
 *
 * 关键假设：
 * - 多轴对象按 `objectIndex + axis * OBJOFFSET` 排布（例如 axis1 的 ControlWord
 *是 0x6040+0x800）。 如果从站不按这个规则映射，你需要改下面 `if (wIndex == base
 *+ dwAxis*OBJOFFSET)` 的匹配方式。
 *
 * 被谁调用：`EcDemoApp.cpp -> myAppSetup()`
 ******************************************************************************/
EC_T_DWORD MT_Setup(T_EC_DEMO_APP_CONTEXT *pAppContext) {
  EC_T_DWORD dwRetVal = EC_E_NOERROR;
  EC_T_WORD wReadEntries = 0;
  EC_T_WORD wWriteEntries = 0;

  /* 【特别说明：为什么这里全是“指针”】
   *
   * EC‑Master 把所有 PDO 数据放在两块连续内存中：
   * - PdOut：主站写 → 从站读（输出过程数据）
   * - PdIn ：从站写 → 主站读（输入过程数据）
   *
   * 你在 ENI 里把某个对象（如 0x6040）映射到 PDO 后，EC‑Master 就知道：
   * - 这个对象在 PdOut/PdIn 里的“位偏移 nBitOffs”
   *
   * 本函数做的事：把
   *   (对象索引 wIndex + 子索引 wSubIndex)  →  (PdOut/PdIn 的内存地址)
   * 映射成指针存在 My_Motor[] 里。
   *
   * 之后每周期控制时，只要对这些指针写值，主站在 SendAllCycFrames
   * 时就会把值发给从站。
   *
   * 也因此：如果 ENI 没映射某对象，指针会保持为 EC_NULL，后面写不进去 →
   * 电机不会动。
   */
  EC_T_WORD mySlaveOutVarInfoNum = 0;
  EC_T_WORD mySlaveInVarInfoNum = 0;
  EC_T_DWORD MyAxis_num_tmp = 0;
  EC_T_DWORD MyAxis_num_cnt = 0;
  EC_T_PROCESS_VAR_INFO_EX *pProcessVarInfoOut = EC_NULL;
  EC_T_PROCESS_VAR_INFO_EX *pProcessVarInfoIn = EC_NULL;
  EC_T_CFG_SLAVE_INFO s_SlaveInfo;
  /* PdIn/PdOut 是 EC‑Master 的“过程映像（ProcessImage）”基地址：
   * - PdOut：主站写（RxPDO），在 SendAllCycFrames 时发送给从站
   * - PdIn ：从站写（TxPDO），在 ProcessAllRxFrames 后可读到
   */
  EC_T_BYTE *pbyPDIn = ecatGetProcessImageInputPtr();
  EC_T_BYTE *pbyPDOut = ecatGetProcessImageOutputPtr();

  EcLogMsg(EC_LOG_LEVEL_INFO,
           (pEcLogContext, EC_LOG_LEVEL_INFO,
            "\n Motrotech: ___________________MT_Setup______________________"));

  /* PdOut/PdIn 基地址必须有效，否则无法建立指针映射 */
  if ((pbyPDOut != EC_NULL) && (pbyPDIn != EC_NULL)) {
    for (EC_T_DWORD dwSlaveIdx = 0; dwSlaveIdx < SlaveCount; dwSlaveIdx++) {
      /* 防御性：多 slave 情况下，每个 slave 都会重新 OsMalloc 一次；
       * 必须在进入下一轮前释放上一次分配，避免泄漏。
       */
      if (pProcessVarInfoOut != EC_NULL) {
        OsFree(pProcessVarInfoOut);
        pProcessVarInfoOut = EC_NULL;
      }
      if (pProcessVarInfoIn != EC_NULL) {
        OsFree(pProcessVarInfoIn);
        pProcessVarInfoIn = EC_NULL;
      }

      OsMemset(&s_SlaveInfo, 0, sizeof(EC_T_CFG_SLAVE_INFO));
      /* ecatGetCfgSlaveInfo 会返回该从站在 ENI 中配置的 PDO/变量数量等信息 */
      if (ecatGetCfgSlaveInfo(EC_TRUE, My_Slave[dwSlaveIdx].wStationAddress,
                              &s_SlaveInfo) != EC_E_NOERROR) {
        EcLogMsg(EC_LOG_LEVEL_ERROR,
                 (pEcLogContext, EC_LOG_LEVEL_ERROR,
                  "ERROR: ecatGetCfgSlaveInfo() returns with error."));
        continue;
      }

      /********************** PDO_OUT（主站输出 -> 从站输入）
       * ***********************
       *
       * 这里枚举“这个 slave 的所有输出 PDO 变量”（也就是主站要写的那部分）。
       * 对每个变量，我们用它的 wIndex/wSubIndex 来判断是不是我们关心的对象，
       * 如果是，就用 nBitOffs 把它定位到 PdOut 里的地址，然后保存到
       * My_Motor[].xxx 指针。
       *
       * 注意：nBitOffs 是“位偏移”，所以要 /8 变成字节偏移。
       */
      mySlaveOutVarInfoNum = s_SlaveInfo.wNumProcessVarsOutp;
      EcLogMsg(EC_LOG_LEVEL_INFO,
               (pEcLogContext, EC_LOG_LEVEL_INFO,
                "Motrotech: OutVarInfoNum = %d", mySlaveOutVarInfoNum));

      pProcessVarInfoOut = (EC_T_PROCESS_VAR_INFO_EX *)OsMalloc(
          sizeof(EC_T_PROCESS_VAR_INFO_EX) * mySlaveOutVarInfoNum);
      if (pProcessVarInfoOut == EC_NULL) {
        dwRetVal = EC_E_NOMEMORY;
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
                                     "Motrotech: Malloc memory fail"));
      } else {
        dwRetVal = ecatGetSlaveOutpVarInfoEx(
            EC_TRUE, My_Slave[dwSlaveIdx].wStationAddress, mySlaveOutVarInfoNum,
            pProcessVarInfoOut, &wReadEntries);
        if (EC_E_NOERROR != dwRetVal) {
          EcLogMsg(EC_LOG_LEVEL_ERROR,
                   (pEcLogContext, EC_LOG_LEVEL_ERROR,
                    "ERROR: emGetSlaveInpVarInfoEx() (Result = %s 0x%x)",
                    ecatGetText(dwRetVal), dwRetVal));
        }
      }

      /* 遍历本 slave 的所有输出变量，逐个判断是否是我们关心的对象。
       *
       * 你后续要实现你们的 MotorCmd_（mode/q/dq/tau/kp/kd...）：
       *   - 就是在这里新增 “if (wIndex/wSubIndex 匹配你们对象)” 然后把指针指向 PdOut。
       *   - 写入时建议用 EC_SETBITS 写 float32，避免对齐/aliasing 问题。
       */
      for (EC_T_DWORD i = 0; i < mySlaveOutVarInfoNum; i++) {
        for (EC_T_DWORD dwAxis = 0; dwAxis < My_Slave[dwSlaveIdx].wAxisCnt;
             dwAxis++) {
          /* MyAxis_num_cnt：累计偏移（前面 slave 的轴数总和）
           * MyAxis_num_tmp：当前 slave 的第 dwAxis 个轴在全局 My_Motor[]
           * 中的下标
           */
          MyAxis_num_tmp = MyAxis_num_cnt + dwAxis;

          /* 【对象索引如何区分多轴？】
           * 本 demo 假设：同一 slave 的多轴对象按固定规律排列：
           *   axis0: baseIndex
           *   axis1: baseIndex + 0x800
           *   axis2: baseIndex + 2*0x800
           * 也就是：wIndex == baseIndex + dwAxis * OBJOFFSET
           * 如果你实际从站的对象不是这个规律，这里必须改。
           */
          if (pProcessVarInfoOut[i].wIndex ==
              DRV_OBJ_CONTROL_WORD + dwAxis * OBJOFFSET) // 0x6040 - ControlWord
          {
            /* 0x6040 ControlWord（主站写 → 驱动读）
             * - 这是 CiA402 状态机的“命令口”，驱动靠它完成
             * Shutdown/SwitchOn/EnableOp/FaultReset 等转换。
             * - 指针指向 PdOut 中该变量所在地址。
             */
            My_Motor[MyAxis_num_tmp].pwControlWord =
                (EC_T_WORD *)&(pbyPDOut[pProcessVarInfoOut[i].nBitOffs / 8]);
            /* 备注：这里用“指针指向 ProcessImage”，后续周期直接 EC_SETWORD 写入即可 */
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pwControlWord = 0x%08X",
                      MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pwControlWord));
          } else if (pProcessVarInfoOut[i].wIndex ==
                     DRV_OBJ_TARGET_POSITION +
                         dwAxis * OBJOFFSET) // 0x607A - TargetPosition
          {
            /* 0x607A TargetPosition（主站写 → 驱动读）
             * - 在 CSP 等模式下，驱动会跟随该目标位置。
             * - 只有驱动已进入 Operation Enabled
             * 后，该目标才会被执行（否则可能忽略或被限制）。
             */
            My_Motor[MyAxis_num_tmp].pnTargetPosition =
                (EC_T_INT *)&(pbyPDOut[pProcessVarInfoOut[i].nBitOffs / 8]);
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pnTargetPosition = 0x%08X",
                      MyAxis_num_tmp,
                      My_Motor[MyAxis_num_tmp].pnTargetPosition));
          } else if (pProcessVarInfoOut[i].wIndex ==
                     DRV_OBJ_TARGET_VELOCITY +
                         dwAxis * OBJOFFSET) // 0x60FF - TargetVelocity
          {
            /* 0x60FF TargetVelocity（主站写 → 驱动读）
             * - 在 CSV 等模式下它是主要目标；在某些驱动的 CSP
             * 实现里也可能作为辅助（取决于厂家）。
             */
            My_Motor[MyAxis_num_tmp].pnTargetVelocity =
                (EC_T_INT *)&(pbyPDOut[pProcessVarInfoOut[i].nBitOffs / 8]);
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pnTargetVelocity = 0x%08X",
                      MyAxis_num_tmp,
                      My_Motor[MyAxis_num_tmp].pnTargetVelocity));
          } else if (pProcessVarInfoOut[i].wIndex ==
                     DRV_OBJ_TARGET_TORQUE +
                         dwAxis * OBJOFFSET) // 0x6071 - TargetTorque
          {
            My_Motor[MyAxis_num_tmp].pwTargetTorque =
                (EC_T_WORD *)&(pbyPDOut[pProcessVarInfoOut[i].nBitOffs / 8]);
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pwTargetTorque = 0x%08X",
                      MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pwTargetTorque));
          } else if (pProcessVarInfoOut[i].wIndex ==
                     DRV_OBJ_VELOCITY_OFFSET +
                         dwAxis * OBJOFFSET) // 0x60B1 - Velocity Offset
          {
            My_Motor[MyAxis_num_tmp].pnVelocityOffset =
                (EC_T_INT *)&(pbyPDOut[pProcessVarInfoOut[i].nBitOffs / 8]);
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pnVelocityOffset = 0x%08X",
                      MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pnVelocityOffset));
          } else if (pProcessVarInfoOut[i].wIndex ==
                     DRV_OBJ_TORQUE_OFFSET +
                         dwAxis * OBJOFFSET) // 0x60B2 - Torque Offset
          {
            My_Motor[MyAxis_num_tmp].pwTorqueOffset =
                (EC_T_SWORD *)&(pbyPDOut[pProcessVarInfoOut[i].nBitOffs / 8]);
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pwTorqueOffset = 0x%08X",
                      MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pwTorqueOffset));
          } else if (pProcessVarInfoOut[i].wIndex ==
                     DRV_OBJ_MODES_OF_OPERATION +
                         dwAxis * OBJOFFSET) // 0x6060 - Mode of Operation
          {
            /* 0x6060 Modes of operation（主站写 → 驱动读）
             * - 选择驱动的工作模式（CSP/CSV/CST/PP 等）。
             * - 这个对象如果被映射到 PDO，则可以每周期写；否则要用 SDO 下载。
             */
            My_Motor[MyAxis_num_tmp].pbyModeOfOperation =
                (EC_T_BYTE *)&(pbyPDOut[pProcessVarInfoOut[i].nBitOffs / 8]);
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pbyModeOfOperation = 0x%08X",
                      MyAxis_num_tmp,
                      My_Motor[MyAxis_num_tmp].pbyModeOfOperation));
          } else if (pProcessVarInfoOut[i].wIndex ==
                     DRV_OBJ_DIGITAL_OUTPUT +
                         dwAxis * OBJOFFSET) // 0x7010 - Output 1
          {
            if (pProcessVarInfoOut[i].wSubIndex ==
                DRV_OBJ_DIGITAL_OUTPUT_SUBINDEX_1) // 0x7010/1 - Output subindex
                                                   // 1
            {
              My_Motor[MyAxis_num_tmp].pwOutput_1 =
                  (EC_T_WORD *)&(pbyPDOut[pProcessVarInfoOut[i].nBitOffs / 8]);
              EcLogMsg(EC_LOG_LEVEL_INFO,
                       (pEcLogContext, EC_LOG_LEVEL_INFO,
                        "Motrotech: MyAxis[%d].pwOutput_1 = 0x%08X",
                        MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pwOutput_1));
            } else if (pProcessVarInfoOut[i].wSubIndex ==
                       DRV_OBJ_DIGITAL_OUTPUT_SUBINDEX_2) // 0x7010/2 - Output
                                                          // subindex 2
            {
              My_Motor[MyAxis_num_tmp].pwOutput_2 =
                  (EC_T_WORD *)&(pbyPDOut[pProcessVarInfoOut[i].nBitOffs / 8]);
              EcLogMsg(EC_LOG_LEVEL_INFO,
                       (pEcLogContext, EC_LOG_LEVEL_INFO,
                        "Motrotech: MyAxis[%d].pwOutput_2 = 0x%08X",
                        MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pwOutput_2));
            }
          } else {
            // EcLogMsg(EC_LOG_LEVEL_INFO,(pEcLogContext,
            // EC_LOG_LEVEL_INFO,"Motrotech: My_Slave[%d] Output undefine the
            // index %d / usbindex %d", i, (pProcessVarInfoOut[i].wIndex,
            // pProcessVarInfoOut[i].wSubIndex));
          }
        }
      }

      /********************** PDO_IN（从站输出 -> 主站输入）
       * ***********************
       *
       * 这里枚举“这个 slave 的所有输入 PDO
       * 变量”（也就是从站上报给主站的那部分）。 最关键的是：
       * - 0x6041 StatusWord：驱动的 CiA402 状态（决定下一步该写哪个
       * ControlWord）
       * - 0x6064 ActualPosition：用于对齐/监控（demo 在未使能时用它避免跳变）
       */
      mySlaveInVarInfoNum = s_SlaveInfo.wNumProcessVarsInp;
      EcLogMsg(EC_LOG_LEVEL_INFO,
               (pEcLogContext, EC_LOG_LEVEL_INFO,
                "Motrotech: InVarInfoNum = %d", mySlaveInVarInfoNum));

      pProcessVarInfoIn = (EC_T_PROCESS_VAR_INFO_EX *)OsMalloc(
          sizeof(EC_T_PROCESS_VAR_INFO_EX) * mySlaveInVarInfoNum);
      if (pProcessVarInfoIn == EC_NULL) {
        dwRetVal = EC_E_NOMEMORY;
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
                                     "Motrotech: Malloc memory fail"));
      } else {
        dwRetVal = ecatGetSlaveInpVarInfoEx(
            EC_TRUE, My_Slave[dwSlaveIdx].wStationAddress, mySlaveInVarInfoNum,
            pProcessVarInfoIn, &wWriteEntries);
        if (EC_E_NOERROR != dwRetVal) {
          EcLogMsg(EC_LOG_LEVEL_ERROR,
                   (pEcLogContext, EC_LOG_LEVEL_ERROR,
                    "ERROR: emGetSlaveInpVarInfoEx() (Result = %s 0x%x)",
                    ecatGetText(dwRetVal), dwRetVal));
        }
      }

      /* 遍历本 slave 的所有输入变量，逐个判断是否是我们关心的对象。
       *
       * 你后续要实现你们的 MotorState_（q_fb/dq_fb/ddq_fb/tau_fb/温度/电压/错误码...）：
       *   - 就是在这里新增 “if (wIndex/wSubIndex 匹配你们对象)” 然后把指针指向 PdIn。
       *   - 注意温度/错误码这些常常是 int16/uint32，不一定是 float。
       */
      for (int i = 0; i < mySlaveInVarInfoNum; i++) {
        for (EC_T_DWORD dwAxis = 0; dwAxis < My_Slave[dwSlaveIdx].wAxisCnt;
             dwAxis++) {
          MyAxis_num_tmp = MyAxis_num_cnt + dwAxis;
          if (pProcessVarInfoIn[i].wIndex ==
              DRV_OBJ_ERROR_CODE + dwAxis * OBJOFFSET) // 0x603F - Error Code
          {
            My_Motor[MyAxis_num_tmp].pwErrorCode =
                (EC_T_WORD *)&(pbyPDIn[pProcessVarInfoIn[i].nBitOffs / 8]);
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pwErrorCode = 0x%08X",
                      MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pwErrorCode));
          } else if (pProcessVarInfoIn[i].wIndex ==
                     DRV_OBJ_STATUS_WORD +
                         dwAxis * OBJOFFSET) // 0x6041 - StatusWord
          {
            /* 0x6041 StatusWord（从站写 → 主站读）
             * - 这是 CiA402 状态机“当前态”的依据。
             * - `Process_Commands()` 每周期都会读它，然后决定写哪个 0x6040
             * 控制字。
             */
            My_Motor[MyAxis_num_tmp].pwStatusWord =
                (EC_T_WORD *)&(pbyPDIn[pProcessVarInfoIn[i].nBitOffs / 8]);
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pwStatusWord = 0x%08X",
                      MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pwStatusWord));
          } else if (pProcessVarInfoIn[i].wIndex ==
                     DRV_OBJ_POSITION_ACTUAL_VALUE +
                         dwAxis * OBJOFFSET) // 0x6064 - ActualPosition
          {
            /* 0x6064 ActualPosition（从站写 → 主站读）
             * - demo 在未使能时，把 TargetPosition 设为
             * ActualPosition（避免使能瞬间跳变/追赶）。
             */
            My_Motor[MyAxis_num_tmp].pnActPosition =
                (EC_T_INT *)&(pbyPDIn[pProcessVarInfoIn[i].nBitOffs / 8]);
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pnActPosition = 0x%08X",
                      MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pnActPosition));
          } else if (pProcessVarInfoIn[i].wIndex ==
                     DRV_OBJ_VELOCITY_ACTUAL_VALUE +
                         dwAxis * OBJOFFSET) // 0x606C - ActualVelocity
          {
            My_Motor[MyAxis_num_tmp].pnActVelocity =
                (EC_T_INT *)&(pbyPDIn[pProcessVarInfoIn[i].nBitOffs / 8]);
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pnActVelocity = 0x%08X",
                      MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pnActVelocity));
          } else if (pProcessVarInfoIn[i].wIndex ==
                     DRV_OBJ_TORQUE_ACTUAL_VALUE +
                         dwAxis * OBJOFFSET) // 0x6077 - ActTorque
          {
            My_Motor[MyAxis_num_tmp].pwActTorque =
                (EC_T_WORD *)&(pbyPDIn[pProcessVarInfoIn[i].nBitOffs / 8]);
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pwActTorque = 0x%08X",
                      MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pwActTorque));
          } else if (pProcessVarInfoIn[i].wIndex ==
                     DRV_OBJ_FOLLOWING_ERROR +
                         dwAxis * OBJOFFSET) // 0x60F4 - ActualFollowErr
          {
            My_Motor[MyAxis_num_tmp].pdwActFollowErr =
                (EC_T_DWORD *)&(pbyPDIn[pProcessVarInfoIn[i].nBitOffs / 8]);
            EcLogMsg(EC_LOG_LEVEL_INFO,
                     (pEcLogContext, EC_LOG_LEVEL_INFO,
                      "Motrotech: MyAxis[%d].pdwActFollowErr = 0x%08X",
                      MyAxis_num_tmp,
                      My_Motor[MyAxis_num_tmp].pdwActFollowErr));
          } else if (pProcessVarInfoIn[i].wIndex ==
                     DRV_OBJ_DIGITAL_INPUT +
                         dwAxis * OBJOFFSET) // 0x6000 - Input 1
          {
            if (pProcessVarInfoIn[i].wSubIndex ==
                DRV_OBJ_DIGITAL_INPUT_SUBINDEX_1) // 0x6000/1 - Input subindex 1
            {
              My_Motor[MyAxis_num_tmp].pwInput_1 =
                  (EC_T_WORD *)&(pbyPDIn[pProcessVarInfoIn[i].nBitOffs / 8]);
              EcLogMsg(EC_LOG_LEVEL_INFO,
                       (pEcLogContext, EC_LOG_LEVEL_INFO,
                        "Motrotech: MyAxis[%d].pwInput_1 = 0x%08X",
                        MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pwInput_1));
            } else if (pProcessVarInfoIn[i].wSubIndex ==
                       DRV_OBJ_DIGITAL_INPUT_SUBINDEX_2) // 0x6000/2 - Input
                                                         // subindex 2
            {
              My_Motor[MyAxis_num_tmp].pwInput_2 =
                  (EC_T_WORD *)&(pbyPDIn[pProcessVarInfoIn[i].nBitOffs / 8]);
              EcLogMsg(EC_LOG_LEVEL_INFO,
                       (pEcLogContext, EC_LOG_LEVEL_INFO,
                        "Motrotech: MyAxis[%d].pwInput_2 = 0x%08X",
                        MyAxis_num_tmp, My_Motor[MyAxis_num_tmp].pwInput_2));
            }
          } else if (pProcessVarInfoIn[i].wIndex ==
                     DRV_OBJ_MCU_TEMPERATURE + dwAxis * OBJOFFSET) // 0x3008
          {
            My_Motor[MyAxis_num_tmp].psTempMcu =
                (EC_T_SWORD *)&(pbyPDIn[pProcessVarInfoIn[i].nBitOffs / 8]);
          } else if (pProcessVarInfoIn[i].wIndex ==
                     DRV_OBJ_MOTOR_TEMPERATURE + dwAxis * OBJOFFSET) // 0x3009
          {
            My_Motor[MyAxis_num_tmp].psTempMotor =
                (EC_T_SWORD *)&(pbyPDIn[pProcessVarInfoIn[i].nBitOffs / 8]);
          } else if (pProcessVarInfoIn[i].wIndex ==
                     DRV_OBJ_IGBT_TEMPERATURE + dwAxis * OBJOFFSET) // 0x300F
          {
            My_Motor[MyAxis_num_tmp].psTempIgbt =
                (EC_T_SWORD *)&(pbyPDIn[pProcessVarInfoIn[i].nBitOffs / 8]);
          } else if (pProcessVarInfoIn[i].wIndex ==
                     DRV_OBJ_DC_LINK_VOLTAGE + dwAxis * OBJOFFSET) // 0x300B
          {
            My_Motor[MyAxis_num_tmp].pwDcLinkVoltage =
                (EC_T_WORD *)&(pbyPDIn[pProcessVarInfoIn[i].nBitOffs / 8]);
          } else {
            // EcLogMsg(EC_LOG_LEVEL_INFO,(pEcLogContext,
            // EC_LOG_LEVEL_INFO,"Motrotech: My_Slave[%d] Input undefine the
            // index %d / usbindex %d", i, (pProcessVarInfoIn[i].wIndex,
            // pProcessVarInfoIn[i].wSubIndex));
          }
        }
      }
      /* 处理完一个 slave，就把全局轴下标偏移推进（下一个 slave
       * 的轴会接着往后排） */
      MyAxis_num_cnt += My_Slave[dwSlaveIdx].wAxisCnt;
    }
  }

  if (pProcessVarInfoOut != EC_NULL) {
    OsFree(pProcessVarInfoOut);
    pProcessVarInfoOut = EC_NULL;
  }

  if (pProcessVarInfoIn != EC_NULL) {
    OsFree(pProcessVarInfoIn);
    pProcessVarInfoIn = EC_NULL;
  }

  /* 把周期时间从 usec 换算成秒，后面速度/位置积分会用到
   * 举例：dwBusCycleTimeUsec=1000 → fTimeSec=0.001s
   */
  fTimeSec = (EC_T_LREAL)pAppContext->AppParms.dwBusCycleTimeUsec / 1000000;

  /* [2026-01-21] 从 SDO 0x3D06 读取每轴减速比（必须在 PREOP 之后才能 SDO 通信）
   * 编码器固定 17 位 (131072)
   */
  const EC_T_LREAL fEncoderCpr = 131072.0;  /* 17 位编码器，固定值 */
  
  for (EC_T_INT i = 0; i < MotorCount; i++)
  {
      EC_T_DWORD dwSlaveId = ecatGetSlaveId(My_Motor[i].wStationAddress);
      EC_T_DWORD dwBytesRead = 0;
      
      /* 读取减速比 0x3D06 (PrD.06) */
      EC_T_DWORD dwGearRatioRaw = 0;
      EC_T_DWORD dwRes = ecatCoeSdoUpload(dwSlaveId, DRV_OBJ_GEAR_RATIO, 0, 
          (EC_T_BYTE*)&dwGearRatioRaw, sizeof(dwGearRatioRaw), &dwBytesRead, 3000, 0);
      
      EC_T_LREAL fGearRatio = 81.0;  /* 默认值（读取失败时使用） */
      if (dwRes == EC_E_NOERROR && dwBytesRead > 0) {
          fGearRatio = (EC_T_LREAL)dwGearRatioRaw;
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, 
              "Axis[%d] GearRatio from SDO 0x3D06 = %d\n", i, (EC_T_INT)dwGearRatioRaw));
      } else {
          EcLogMsg(EC_LOG_LEVEL_WARNING, (pEcLogContext, EC_LOG_LEVEL_WARNING, 
              "Axis[%d] Read GearRatio(0x3D06) failed (0x%x), using default 81\n", i, dwRes));
      }
      My_Motor[i].fGearRatio = fGearRatio;
      /* 计算单位换算系数 */
      MT_SetAxisUnitScale((EC_T_WORD)i, fEncoderCpr, fGearRatio);

      // [2026-01-22] 读取额定扭矩 0x6076
      EC_T_DWORD dwRatedTorqueRaw = 0;
      dwRes = ecatCoeSdoUpload(dwSlaveId, 0x6076, 0,
          (EC_T_BYTE*)&dwRatedTorqueRaw, sizeof(dwRatedTorqueRaw), &dwBytesRead, 3000, 0);
      if (dwRes == EC_E_NOERROR && dwRatedTorqueRaw > 0) {
          My_Motor[i].fRatedTorque = (EC_T_LREAL)dwRatedTorqueRaw * 0.001;  // 0.001 N·m 单位
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
              "Axis[%d] Rated Torque = %.3f N·m\n", i, My_Motor[i].fRatedTorque));
      } else {
          My_Motor[i].fRatedTorque = 1.0;  // 默认 1 N·m，避免除零
          EcLogMsg(EC_LOG_LEVEL_WARNING, (pEcLogContext, EC_LOG_LEVEL_WARNING,
              "Axis[%d] Failed to read rated torque, using default 1.0 N·m\n", i));
      }
  }

  return EC_E_NOERROR;
}

/******************************************************************************
 * MT_Workpd
 * 每个总线周期调用一次（核心循环）。
 *
 * 做两件事：
 * 1) `Process_Commands()`：先根据 StatusWord 更新状态，并写 ControlWord
 *推动状态机到目标态 2) 若轴已到
 *OP_ENABLED：生成目标位置/速度（示例为正反往复、梯形速度曲线）并写入 PDO
 *
 * 注意：
 * - lPosTmp 用 int64 临时保存，但最终写入的是
 *int32（EC_T_INT），存在溢出边界问题；demo 里用极值判定切换方向。
 * - `IncFactor`
 *是一个经验系数，用来把速度积分成位置；实际工程要用真实单位/插补周期。
 ******************************************************************************/
/* [2026-01-20] 记录示教限位 */
EC_T_VOID MT_TeachLimit(EC_T_WORD wAxis, EC_T_BOOL bIsMax) {
    if (wAxis >= MAX_AXIS_NUM) return;
    if (bIsMax) {
        My_Motor[wAxis].fLimitMax = S_MotorState[wAxis].q_fb;
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Axis %d: TEACH MAX = %d (x1000)\n", wAxis, (EC_T_INT)(My_Motor[wAxis].fLimitMax*1000)));
    } else {
        My_Motor[wAxis].fLimitMin = S_MotorState[wAxis].q_fb;
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Axis %d: TEACH MIN = %d (x1000)\n", wAxis, (EC_T_INT)(My_Motor[wAxis].fLimitMin*1000)));
    }
    // 当两个都设置过且 Min < Max 时，激活保护
    if (My_Motor[wAxis].fLimitMin < My_Motor[wAxis].fLimitMax) {
        My_Motor[wAxis].bLimitValid = EC_TRUE;
    }
}

EC_T_VOID MT_Workpd(T_EC_DEMO_APP_CONTEXT *pAppContext) {
  /* IncFactor：demo 用于“速度积分成位置”的经验系数（并非严格物理单位）
   * - 这里把总线周期（usec）带进去，得到一个随周期变化的比例因子
   * - 后面用：fCurPos += fCurVel * IncFactor  来生成“目标轨迹位置”（最终写到 0x607A）
   */
  EC_T_LREAL IncFactor = (EC_T_LREAL)0.0010922 *
                         (EC_T_LREAL)pAppContext->AppParms.dwBusCycleTimeUsec;
  /* lPosTmp：写入 TargetPosition(0x607A) 前的临时整型目标（int64 防溢出）
   * - 最终仍会强转成 32bit 写入（EC_T_INT），所以超界时 demo 用极值切换方向
   */
  int64_t lPosTmp;

  /* MT_Workpd() = “周期控制主循环（按轴）”
   * 你可以把它理解为：每 1ms（或你设置的 cycle time）运行一次的控制器回调。
   *
   * 在你们的目标架构里，通常会把这段替换为：
   *   - 从上层读取 MotorCmd_[i]
   *   - 写 RxPDO（mode/q/dq/tau/kp/kd...）
   *   - 读 TxPDO，更新 MotorState_[i]
   */

  /* 【每周期的第一步：根据 MotorCmd_.mode 更新“状态机命令”】
   *
   * demo 约定：
   * - MotorCmd_.mode == 0  -> 请求 SHUTDOWN（退出使能）
   * - MotorCmd_.mode != 0  -> 请求 START（进入 OP_ENABLED）
   * 如果你想完全自己控制 CiA402（例如只用 MT_SetSwitch），可以不调用 MT_SetMotorCmd()。
   */
  /* [2026-01-14] 目的：按运行模式决定 CiA402 状态机命令来源
   * - AUTO：自动进入 OP_ENABLED（保持原demo“自动跑”）
   * - MANUAL：按 MotorCmd_.mode 决定 start/shutdown
   */
  for (EC_T_INT i = 0; i < MotorCount; i++) {
    /* [2026-01-23] 根据 motion_func 决定状态机命令 */
    if (S_MotorCmdValid[i]) {
      S_ProcessState[i] = (S_MotorCmd[i].motion_func == MOTION_SHUTDOWN) ? COMMAND_SHUTDOWN : COMMAND_START;
    }
    /* 没有效cmd：不改S_ProcessState，维持上一次状态 */
  }

  /* 【每周期的第二步：跑 CiA402 状态机】
   * - 读 0x6041，更新 wActState
   * - 根据命令决定 wReqState “期望状态 / 目标状态（requested state）”，表示当前这一轴希望驱动最终到达的 CiA402 状态
   * - 写 0x6040 推进状态
   *
   * 只有当 wActState == OP_ENABLED 时，后面的 TargetPosition/Velocity
   * 才会真正驱动电机运动。
   */
  Process_Commands(pAppContext);

  for (EC_T_INT i = 0; i < MotorCount; i++) {
    /* pDemoAxis：第 i 个轴的运行时上下文（包含 PDO 指针、状态机状态、轨迹变量等） */
    My_Motor_Type *pDemoAxis = &My_Motor[i];
    if (EC_NULL == pDemoAxis)
      continue;

    /* ====== 先读反馈：填充 MotorState_（即使未使能也可读） ====== */
    MotorState_ st;
    OsMemset(&st, 0, sizeof(st));
    if (pDemoAxis->pwStatusWord) {
      st.motorstate = (EC_T_DWORD)EC_GETWORD(pDemoAxis->pwStatusWord);
    }
    if (pDemoAxis->pbyModeOfOperation) {
      // 0x6061 运行模式显示 (虽然指针名为 pbyModeOfOperation 但在映射时我们通常也映射了 0x6061)
      // 注意：这里假设 pbyModeOfOperation 指向的是 0x6061，如果 ENI 只映射了 0x6060 则读不到真实模式
      st.mode = *pDemoAxis->pbyModeOfOperation; 
    }
    if (pDemoAxis->pnActPosition) {
      /* [2026-01-19] 作用：把 0x6064(ActualPosition, int32 PUU) 换算成 rad 回填 */
      st.q_fb = (EC_T_REAL)((EC_T_LREAL)(*pDemoAxis->pnActPosition) * pDemoAxis->fRadPerCnt);
    }
    if (pDemoAxis->pnActVelocity) {
      /* [2026-01-19] 作用：把 0x606C(ActualVelocity, int32 PUU/s) 换算成 rad/s 回填 */
      st.dq_fb = (EC_T_REAL)((EC_T_LREAL)(*pDemoAxis->pnActVelocity) * pDemoAxis->fRadPerCnt);
    }
    st.ddq_fb = 0.0f; /* 文档：不支持，需由 dq_fb 差分计算 */
    
    /* [2026-01-19] 目的：0x6077 实际扭矩 -> N.m (按 0.1% 换算) */
    if (pDemoAxis->pwActTorque) {
      st.tau_fb = (EC_T_REAL)(EC_T_SWORD)EC_GETWORD(pDemoAxis->pwActTorque) * 0.001f; 
    }
    
    /* 温度/电压：只有 ENI 映射了相应对象，指针才会非空 */
    st.temperature[0] = (pDemoAxis->psTempMcu) ? (*pDemoAxis->psTempMcu) : 0;
    st.temperature[1] = (pDemoAxis->psTempMotor) ? (*pDemoAxis->psTempMotor) : 0;
    if (pDemoAxis->pwDcLinkVoltage) {
      st.vol = (EC_T_REAL)(*pDemoAxis->pwDcLinkVoltage) * 0.1f; // 0.1V 转换
    }
    st.sensor[0] = 0;
    st.sensor[1] = 0;
    S_MotorState[i] = st;

    /* Mode of Operation（0x6060）
     * - demo 默认在 MT_Init 中设为 CSP（8）
     * - 如果 0x6060 被映射到 PDO，就在这里每周期刷新一次（很多驱动允许）
     */
    if (EC_NULL != pDemoAxis->pbyModeOfOperation) {
      /* 写 0x6060 Modes of operation（PDO 映射存在时才可写）
       * - demo 默认写 CSP(8)
       * - 如果你们用自定义模式/自定义对象，这里通常会改成写你们的 mode
       */
      EC_SETBITS(pDemoAxis->pbyModeOfOperation,
                 (EC_T_BYTE *)&pDemoAxis->eModesOfOperation, 0, 8);
    }
    /* 只有在 OP_ENABLED（已使能）时才生成运动命令，否则进行“对齐初始化”：
     * - 先让 TargetPosition = ActualPosition
     * - 避免从站刚使能时出现“目标突变”，导致猛冲/报错
     */
    /* 如果上层提供了 MotorCmd_，则优先使用“手动目标”；否则继续使用 demo 自带往复轨迹 */
    const EC_T_BOOL bHaveCmd = (i < MAX_AXIS_NUM) ? S_MotorCmdValid[i] : EC_FALSE;
    MotorCmd_ cmd;
    OsMemset(&cmd, 0, sizeof(cmd));
    if (bHaveCmd) {
      cmd = S_MotorCmd[i];
    }
    /* [2026-01-19] 优化：手动模式下增加平滑移动逻辑，防止突跳并实现到达即停 */
    static EC_T_BOOL bFirstEnable[MAX_AXIS_NUM] = { EC_FALSE };
    if (pDemoAxis->wActState != DRV_DEV_STATE_OP_ENABLED) {
        bFirstEnable[i] = EC_FALSE; // 未使能时，重置同步标记
    }

    /* [2026-01-23] 重构：基于 motion_func 和 drive_mode 的控制逻辑 */
    static EC_T_INT nDirection[MAX_AXIS_NUM] = { 1 }; // 老化用：1=正向, -1=反向
    
    /* ===== MOTION_SHUTDOWN: 停机/释放 ===== */
    if (bHaveCmd && (cmd.motion_func == MOTION_SHUTDOWN)) {
      // motion_func == SHUTDOWN 会触发状态机退出 OP_ENABLED
      // 不做任何控制输出
    }
    /* ===== MOTION_AGING: 老化测试 ===== */
    else if (bHaveCmd && (cmd.motion_func == MOTION_AGING) && (pDemoAxis->wActState == DRV_DEV_STATE_OP_ENABLED)) {
      if (!bFirstEnable[i]) {
          pDemoAxis->fCurPos = S_MotorState[i].q_fb; 
          bFirstEnable[i] = EC_TRUE;
      }

      // 获取限位范围
      EC_T_LREAL fMinLimit, fMaxLimit;
      if (pDemoAxis->bLimitValid) {
          fMinLimit = pDemoAxis->fLimitMin;
          fMaxLimit = pDemoAxis->fLimitMax;
      } else {
          EC_T_LREAL fRange = (cmd.range > 0) ? (EC_T_LREAL)cmd.range : 5.0;
          fMinLimit = -fRange;
          fMaxLimit = fRange;
      }

      EC_T_LREAL fMaxVel = (cmd.dq > 0) ? (EC_T_LREAL)cmd.dq : 1.0; 
      EC_T_LREAL fMaxStep = fMaxVel * fTimeSec;

      // 自动切换方向
      if (pDemoAxis->fCurPos >= fMaxLimit) {
          nDirection[i] = -1;
      } else if (pDemoAxis->fCurPos <= fMinLimit) {
          nDirection[i] = 1;
      }

      pDemoAxis->fCurPos += (nDirection[i] * fMaxStep);

      // 根据 drive_mode 写入 PDO
      if (cmd.drive_mode == DRIVE_MODE_PT) {
        // PT 模式老化：用阻抗控制
        MotorState_ *pState = &S_MotorState[i];
        EC_T_LREAL tau_cmd = cmd.kp * (pDemoAxis->fCurPos - pState->q_fb) + cmd.kd * (0 - pState->dq_fb);
        EC_T_LREAL fRatedTorque = pDemoAxis->fRatedTorque > 0 ? pDemoAxis->fRatedTorque : 1.0;
        EC_T_LREAL tau_permille = tau_cmd / fRatedTorque * 1000.0;
        if (tau_permille > 3000.0) tau_permille = 3000.0;
        if (tau_permille < -3000.0) tau_permille = -3000.0;
        if (pDemoAxis->pwTargetTorque != EC_NULL) {
            EC_SETWORD(pDemoAxis->pwTargetTorque, (EC_T_WORD)((EC_T_SWORD)tau_permille));
        }
        if (pDemoAxis->pbyModeOfOperation != EC_NULL) {
            *pDemoAxis->pbyModeOfOperation = DRIVE_MODE_PT;
        }
      } else {
        // CSP/CST 模式老化：用位置控制
        if (pDemoAxis->pnTargetPosition != EC_NULL) {
          const EC_T_LREAL q_cnt = pDemoAxis->fCurPos * pDemoAxis->fCntPerRad;
          EC_SETDWORD(pDemoAxis->pnTargetPosition, MtSatToInt32(q_cnt));
        }
        if (pDemoAxis->pbyModeOfOperation != EC_NULL) {
            *pDemoAxis->pbyModeOfOperation = DRIVE_MODE_CSP;
        }
      }
    }
    /* ===== MOTION_CONTROL: 正常控制（PT/CSP/CST）===== */
    else if (bHaveCmd && (cmd.motion_func == MOTION_CONTROL) && (pDemoAxis->wActState == DRV_DEV_STATE_OP_ENABLED)) {
      // 同步初始位置
      if (!bFirstEnable[i]) {
          pDemoAxis->fCurPos = S_MotorState[i].q_fb; 
          bFirstEnable[i] = EC_TRUE;
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Axis %d: Position Sync to %d (x1000)\n", i, (EC_T_INT)(pDemoAxis->fCurPos*1000)));
      }

      /* --- 根据 drive_mode 执行不同控制 --- */
      if (cmd.drive_mode == DRIVE_MODE_PT) {
        // PT 模式：阻抗控制 τ = τ_ff + kp*(q_des-q_fb) + kd*(dq_des-dq_fb)
        MotorState_ *pState = &S_MotorState[i];
        EC_T_LREAL q_fb = pState->q_fb;
        EC_T_LREAL dq_fb = pState->dq_fb;
        EC_T_LREAL tau_cmd = cmd.tau + cmd.kp * (cmd.q - q_fb) + cmd.kd * (cmd.dq - dq_fb);
        
        
        EC_T_LREAL fRatedTorque = pDemoAxis->fRatedTorque > 0 ? pDemoAxis->fRatedTorque : 1.0;
        EC_T_LREAL tau_permille = tau_cmd / fRatedTorque * 1000.0;
        if (tau_permille > 3000.0) tau_permille = 3000.0;
        if (tau_permille < -3000.0) tau_permille = -3000.0;
        
        if (pDemoAxis->pwTargetTorque != EC_NULL) {
            EC_SETWORD(pDemoAxis->pwTargetTorque, (EC_T_WORD)((EC_T_SWORD)tau_permille));
        }
        if (pDemoAxis->pbyModeOfOperation != EC_NULL) {
            *pDemoAxis->pbyModeOfOperation = DRIVE_MODE_PT;
        }
      }
      else if (cmd.drive_mode == DRIVE_MODE_CSP) {
        // CSP 模式：平滑位置控制
        EC_T_LREAL fTargetQ = (EC_T_LREAL)cmd.q;
        EC_T_LREAL fMaxVel = (cmd.dq > 0) ? (EC_T_LREAL)cmd.dq : 1.0; 
        EC_T_LREAL fMaxStep = fMaxVel * fTimeSec;

        EC_T_LREAL fDiff = fTargetQ - pDemoAxis->fCurPos;
        if (fDiff > fMaxStep) {
            pDemoAxis->fCurPos += fMaxStep;
        } else if (fDiff < -fMaxStep) {
            pDemoAxis->fCurPos -= fMaxStep;
        } else {
            pDemoAxis->fCurPos = fTargetQ;
        }

        if (pDemoAxis->pnTargetPosition != EC_NULL) {
          const EC_T_LREAL q_cnt = pDemoAxis->fCurPos * pDemoAxis->fCntPerRad;
          EC_SETDWORD(pDemoAxis->pnTargetPosition, MtSatToInt32(q_cnt));
        }
        if (pDemoAxis->pnVelocityOffset != EC_NULL) {
          const EC_T_LREAL dq_cnt = (EC_T_LREAL)cmd.dq * pDemoAxis->fCntPerRad;
          EC_SETDWORD(pDemoAxis->pnVelocityOffset, MtSatToInt32(dq_cnt));
        }
        if (pDemoAxis->pwTorqueOffset != EC_NULL) {
          const EC_T_SWORD tau_raw = (EC_T_SWORD)(cmd.tau * 1000.0f);
          EC_SETWORD(pDemoAxis->pwTorqueOffset, tau_raw);
        }
        if (pDemoAxis->pbyModeOfOperation != EC_NULL) {
            *pDemoAxis->pbyModeOfOperation = DRIVE_MODE_CSP;
        }
      }
      else if (cmd.drive_mode == DRIVE_MODE_CST) {
        // CST 模式：纯力矩控制
        EC_T_LREAL fRatedTorque = pDemoAxis->fRatedTorque > 0 ? pDemoAxis->fRatedTorque : 1.0;
        EC_T_LREAL tau_permille = cmd.tau / fRatedTorque * 1000.0;
        if (tau_permille > 3000.0) tau_permille = 3000.0;
        if (tau_permille < -3000.0) tau_permille = -3000.0;
        
        if (pDemoAxis->pwTargetTorque != EC_NULL) {
            EC_SETWORD(pDemoAxis->pwTargetTorque, (EC_T_WORD)((EC_T_SWORD)tau_permille));
        }
        if (pDemoAxis->pbyModeOfOperation != EC_NULL) {
            *pDemoAxis->pbyModeOfOperation = DRIVE_MODE_CST;
        }
      }
    }
    /* ===== 已使能但无有效命令或 IDLE：保持当前位置 ===== */
    else if (pDemoAxis->wActState == DRV_DEV_STATE_OP_ENABLED) {
      if (pDemoAxis->pnActPosition != EC_NULL && pDemoAxis->pnTargetPosition != EC_NULL) {
          EC_SETDWORD(pDemoAxis->pnTargetPosition, *pDemoAxis->pnActPosition);
      }
      if (pDemoAxis->pnTargetVelocity != EC_NULL) {
        EC_SETDWORD(pDemoAxis->pnTargetVelocity, 0);
      }
      pDemoAxis->fCurVel = 0;
    }
    /* ===== [已删除] 旧自动模式代码开始 ===== */
    else if (0) { // 占位，防止编译错误
      int64_t lPosTmp = 0;
      switch (pDemoAxis->eMovingStat) {
      case MOVE_STAT_POS_ACC:
        /* 正向加速阶段：速度逐步增加到 +MAX_VEL */
        pDemoAxis->fCurVel += fTimeSec * ACC_DEC;
        if (pDemoAxis->fCurVel >= MAX_VEL) {
          pDemoAxis->eMovingStat = MOVE_STAT_POS_CON;
        }
        break;
      case MOVE_STAT_POS_CON:
        /* 正向匀速阶段：保持 +MAX_VEL 一段时间（用 dwConRunCnt 计数） */
        pDemoAxis->fCurVel = MAX_VEL;
        if (pDemoAxis->dwConRunCnt++ > CONRUNSEC / fTimeSec) {
          pDemoAxis->dwConRunCnt = 0;
          pDemoAxis->eMovingStat = MOVE_STAT_POS_DEC;
        }
        break;
      case MOVE_STAT_POS_DEC:
        /* 正向减速阶段：速度逐步减到 0，随后切到反向加速 */
        pDemoAxis->fCurVel -= fTimeSec * ACC_DEC;
        if (pDemoAxis->fCurVel <= 0) {
          pDemoAxis->eMovingStat = MOVE_STAT_NEG_ACC;
        }
        break;
      case MOVE_STAT_NEG_ACC:
        /* 反向加速阶段：速度逐步降低到 -MAX_VEL */
        pDemoAxis->fCurVel -= fTimeSec * ACC_DEC;
        if (pDemoAxis->fCurVel <= -MAX_VEL) {
          pDemoAxis->eMovingStat = MOVE_STAT_NEG_CON;
        }
        break;
      case MOVE_STAT_NEG_CON:
        /* 反向匀速阶段：保持 -MAX_VEL 一段时间 */
        pDemoAxis->fCurVel = -MAX_VEL;
        if (pDemoAxis->dwConRunCnt++ > CONRUNSEC / fTimeSec) {
          pDemoAxis->dwConRunCnt = 0;
          pDemoAxis->eMovingStat = MOVE_STAT_NEG_DEC;
        }
        break;
      case MOVE_STAT_NEG_DEC:
        /* 反向减速阶段：速度逐步回到 0，随后切回正向加速 */
        pDemoAxis->fCurVel += fTimeSec * ACC_DEC;
        if (pDemoAxis->fCurVel >= 0) {
          pDemoAxis->eMovingStat = MOVE_STAT_POS_ACC;
        }
        break;
      default:
        /* 异常/未初始化：回到一个确定的初始阶段 */
        pDemoAxis->fCurVel = 0;
        pDemoAxis->dwConRunCnt = 0;
        pDemoAxis->eMovingStat = MOVE_STAT_POS_ACC;
        break;
      }

      /* 位置积分：pos += vel * dt
       * 注意：这里 dt 没有直接用 fTimeSec，而是用一个经验系数 IncFactor。
       * 真实工程里建议明确单位：pos[count] += vel[count/s] * dt[s]。
       */
      pDemoAxis->fCurPos += pDemoAxis->fCurVel * IncFactor;

      /* 写入 PDO：TargetPosition / TargetVelocity /
       * TargetTorque（如果对应对象已映射） 写入之后并不会立刻到达从站：
       * - 真正发送发生在 EcMasterJobTask 的 eUsrJob_SendAllCycFrames
       */
      if (pDemoAxis->pnTargetPosition != EC_NULL) {
        /* 写 0x607A TargetPosition（int32）
         * - 注意：这里写的是“目标”，不是“实际位置”
         * - EC_SETDWORD：SDK 封装的 32bit 写入（包含必要的对齐/字节序处理）
         */
        EC_SETDWORD(pDemoAxis->pnTargetPosition, (EC_T_INT)lPosTmp);
      }
      if (pDemoAxis->pnTargetVelocity != EC_NULL) {
        /* 写 0x60FF TargetVelocity（int32，单位取决于从站定义） */
        EC_SETDWORD(pDemoAxis->pnTargetVelocity,
                    (EC_T_INT)(pDemoAxis->fCurVel * INC_PERMM));
      }
      /* demo 原本会写死 0x6071 TargetTorque=200；为了避免误触发扭矩指令，这里不再周期写入。 */
    }
    /* ===== 未使能状态：对齐内部位置到实际位置 ===== */
    else {
      /* 未使能时：把内部状态对齐到实际位置，避免一使能就跳变 */
      // ... 保持你原来的 else 内容 ...
      /* 未使能时：把内部状态对齐到实际位置，避免一使能就跳变 */
      /* 注意：这里默认 pnActPosition 已映射；若未映射会解引用空指针（工程化要加保护）
       * 实际工程建议：
       *   if (pnActPosition && pnTargetPosition) { ... } else { 报错/降级 }
       */
      pDemoAxis->fCurPos = (EC_T_LREAL)(*pDemoAxis->pnActPosition) / INC_PERMM;
      pDemoAxis->fCurVel = 0;
      if (pDemoAxis->pnTargetPosition != EC_NULL) {
        /* 未使能时把目标位置“贴住”实际位置，避免使能瞬间产生大跟随误差 */
        EC_SETDWORD(pDemoAxis->pnTargetPosition, *pDemoAxis->pnActPosition);
      }
    }


  } /* loop through axis list */
}

/* [2026-01-23] 设置全局驱动模式 */
EC_T_VOID MT_SetGlobalDriveMode(DriveMode mode)
{
  S_GlobalDriveMode = mode;
}

/* [2026-01-23] 获取全局驱动模式 */
DriveMode MT_GetGlobalDriveMode(EC_T_VOID)
{
  return S_GlobalDriveMode;
}

/* [2026-01-23] 获取驱动模式名称 */
const char* MT_GetDriveModeName(DriveMode mode)
{
  switch (mode) {
    case DRIVE_MODE_PT:  return "PT";
    case DRIVE_MODE_CSP: return "CSP";
    case DRIVE_MODE_CST: return "CST";
    default:             return "UNKNOWN";
  }
}

/* 上层写入每轴 MotorCmd_（demo 级：无锁，直接覆盖） */
EC_T_VOID MT_SetMotorCmd(EC_T_WORD wAxis, const MotorCmd_* pCmd)
{
  if ((pCmd == EC_NULL) || (wAxis >= MAX_AXIS_NUM)) {
    return;
  }

  /* [2026-01-19] 目的：处理 kp (0x3500) 和 kd (0x3501) 的 SDO 异步下发
   * 说明：由于这两个字段是配置类参数，且不支持 PDO 映射，因此需要通过 CoE SDO 下载。
   * 此函数由交互线程（CmdThread）调用，SDO 写入是阻塞的，但不会影响主控制周期的实时性。
   */
  EC_T_DWORD dwSlaveId = ecatGetSlaveId(My_Motor[wAxis].wStationAddress);

  // 1. 处理 KP 变更
  if (pCmd->kp != S_MotorCmd[wAxis].kp) {
      EC_T_REAL fVal = pCmd->kp;
      // 下载到索引 0x3500, 子索引 0
      EC_T_DWORD dwRes = ecatCoeSdoDownload(dwSlaveId, 0x3500, 0, (EC_T_BYTE*)&fVal, sizeof(EC_T_REAL), 1000, 0);
      if (dwRes != EC_E_NOERROR) {
          EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, 
              "SDO Write KP(0x3500) failed for axis %d: %s(0x%x)\n", wAxis, ecatGetText(dwRes), dwRes));
      } else {
          /* [2026-01-19] 修正：由于 EcLogMsg 可能不支持浮点格式化，改为打印 10 倍整数 */
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "SDO Write KP=%d (x10) to axis %d OK\n", (EC_T_INT)(fVal*10), wAxis));
      }
  }

  // 2. 处理 KD 变更
  if (pCmd->kd != S_MotorCmd[wAxis].kd) {
      EC_T_REAL fVal = pCmd->kd;
      // 下载到索引 0x3501, 子索引 0
      EC_T_DWORD dwRes = ecatCoeSdoDownload(dwSlaveId, 0x3501, 0, (EC_T_BYTE*)&fVal, sizeof(EC_T_REAL), 1000, 0);
      if (dwRes != EC_E_NOERROR) {
          EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, 
              "SDO Write KD(0x3501) failed for axis %d: %s(0x%x)\n", wAxis, ecatGetText(dwRes), dwRes));
      } else {
          /* [2026-01-19] 修正：由于 EcLogMsg 可能不支持浮点格式化，改为打印 10 倍整数 */
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "SDO Write KD=%d (x10) to axis %d OK\n", (EC_T_INT)(fVal*10), wAxis));
      }
  }

  S_MotorCmd[wAxis] = *pCmd;
  S_MotorCmdValid[wAxis] = EC_TRUE;
}

/* 上层读取每轴 MotorState_（demo 级：无锁，读取到的是“最近一次周期刷新”的快照） */
EC_T_BOOL MT_GetMotorState(EC_T_WORD wAxis, MotorState_* pStateOut)
{
  if ((pStateOut == EC_NULL) || (wAxis >= MAX_AXIS_NUM)) {
    return EC_FALSE;
  }
  *pStateOut = S_MotorState[wAxis];
  return EC_TRUE;
}

/* 设置指定轴的 Operation Mode（0x6060）。
 * 注意：若 0x6060 没映射到 PDO，需要用 CoE SDO 下载（此处仅留 TODO）。
 */
EC_T_DWORD MT_MT_SetAxisOpMod(EC_T_WORD wAxis, MC_T_CIA402_OPMODE eMode) {
  EC_T_DWORD dwRes = EC_E_NOERROR;
  My_Motor_Type *pDemoAxis = &My_Motor[wAxis];
  if (EC_NULL == pDemoAxis) {
    dwRes = EC_E_NOTFOUND;
    return dwRes;
  }
  pDemoAxis->eModesOfOperation = eMode;
  if (EC_NULL != pDemoAxis->pbyModeOfOperation) {
    EC_SETBITS(pDemoAxis->pbyModeOfOperation,
               (EC_T_BYTE *)&pDemoAxis->eModesOfOperation, 0, 8);
  } else {
    // Todo: ecatCoeDownload --> 0x6060
  }
  return dwRes;
}

/******************************************************************************
 * MT_SetSwitch
 * 上层“广播式”下发命令：把 command 写入每个轴的 `S_ProcessState[]`。
 * - 在 `MT_Workpd()` 每周期调用的 `Process_Commands()` 会把该命令转换成目标状态
 *wReqState。
 * - stop/shutdown 时会额外调用 `CheckMotorStateStop()` 等待轴退出
 *OP_ENABLED（demo 级）。
 ******************************************************************************/
EC_T_VOID MT_SetSwitch(eStateCmd command) {
  for (EC_T_INT mIndex = 0; mIndex < MotorCount; mIndex++) {
    if (mIndex < MotorCount) {
      switch (command) {
      case COMMAND_SHUTDOWN:
        S_ProcessState[mIndex] = COMMAND_SHUTDOWN;
        break;
      case COMMAND_START:
        S_ProcessState[mIndex] = COMMAND_START;
        break;
      case COMMAND_RESET:
        S_ProcessState[mIndex] = COMMAND_RESET;
        break;
      case COMMAND_HALT:
        S_ProcessState[mIndex] = COMMAND_HALT;
        break;
      case COMMAND_PAUSE:
        S_ProcessState[mIndex] = COMMAND_PAUSE;
        break;
      case COMMAND_QUICKSTOP:
        S_ProcessState[mIndex] = COMMAND_QUICKSTOP;
        break;
      case COMMAND_STOP:
        S_ProcessState[mIndex] = COMMAND_STOP;
        break;
      case COMMAND_NONE:
        break;
      }
    }
  }
  if (COMMAND_STOP == command || COMMAND_SHUTDOWN == command) {
    CheckMotorStateStop();
  }
}

/******************************************************************************
 * Process_Commands（local）
 * 简化版 CiA402 状态机：
 * - 前提：master 必须处于 OP 状态（否则不做任何控制）
 * - 对每个轴：
 *   1) 根据 `S_ProcessState[mIndex]` 计算期望状态 `wReqState`
 *   2) 读 StatusWord（0x6041），解析出当前状态 `wActState`
 *   3) 如果未达到目标态，写
 *ControlWord（0x6040）触发状态转换（shutdown/switchon/enable op/fault
 *reset...）
 *
 * 说明：这不是完整的 CiA402 状态机实现，只覆盖 demo 需要的最小路径。
 ******************************************************************************/
static EC_T_DWORD Process_Commands(T_EC_DEMO_APP_CONTEXT *pAppContext) {
  EC_T_WORD wControl;
  EC_T_DWORD S_dwStatus;
  /* 只有主站在 OP 状态才做驱动控制，否则避免写 PDO 造成异常 */
  if (eEcatState_OP != ecatGetMasterState())
    return EC_E_NOERROR;

  for (EC_T_INT mIndex = 0; mIndex < MotorCount; mIndex++) {
    /* pDemoAxis：第 mIndex 个轴（你可以理解为“轴编号”） */
    My_Motor_Type *pDemoAxis = &My_Motor[mIndex];
    /* 如果 StatusWord 没映射到 PDO，这个轴就无法跑状态机（也就无法使能运动） */
    if (pDemoAxis->pwStatusWord != EC_NULL) {
      /* 1) 上层命令（COMMAND_START/SHUTDOWN/STOP...）→ 目标状态 wReqState
       * - START    → 希望到 OP_ENABLED（可执行目标）
       * - SHUTDOWN → 希望到 READY_TO_SWITCHON（退回安全态）
       * - 其他     → 希望保持 SWITCHED_ON（示例默认）
       */
      switch (S_ProcessState[mIndex]) {
      case COMMAND_SHUTDOWN:
        /* 退回 READY_TO_SWITCHON：本 demo 用 shutdown 控制字推进到更安全的状态 */
        pDemoAxis->wReqState = DRV_DEV_STATE_READY_TO_SWITCHON;
        break;
      case COMMAND_START:
        /* 目标：OP_ENABLED（可执行 TargetPosition/Velocity/Torque 等） */
        pDemoAxis->wReqState = DRV_DEV_STATE_OP_ENABLED;
        break;
      case COMMAND_RESET:
      case COMMAND_HALT:
      case COMMAND_PAUSE:
      case COMMAND_QUICKSTOP:
      case COMMAND_STOP:
      case COMMAND_NONE:
      default:
        pDemoAxis->wReqState = DRV_DEV_STATE_SWITCHED_ON;
        break;
      }

      /* 2) 读取状态字（0x6041），解析当前状态 wActState
       * 这里的 STATUSWORD_STATE_* 掩码/常量来自 motrotech.h：
       * - 它们对应 CiA402 的标准状态编码（简化版）。
       */
      S_dwStatus = EC_GETWORD(pDemoAxis->pwStatusWord);
      /* ====== 下面这段是在“检测 Fault（故障态）” ======
       *
       * - S_dwStatus 来自 0x6041 StatusWord（从站->主站）
       * - STATUSWORD_STATE_FAULT 在 motrotech.h 里定义为 0x0008，对应 CiA402 的 bit3 Fault
       * - 一旦检测到 Fault，就把本地状态机强制切到 DRV_DEV_STATE_MALFUNCTION，
       *   这样后面的分支会写 0x6040 的 Fault Reset(0x0080) 尝试复位
       */
      if ((S_dwStatus & STATUSWORD_STATE_FAULT) &&
          /* 避免每周期重复“进入故障态”的处理与刷屏日志 */
          (pDemoAxis->wActState != DRV_DEV_STATE_MALFUNCTION)) {
        /* 打印：第 mIndex 轴进入 Fault（同时打印当前 StatusWord 便于排查） */
        EcLogMsg(EC_LOG_LEVEL_INFO,
                 (pEcLogContext, EC_LOG_LEVEL_INFO,
                  "Axis[%d] To Fault Reaction 0x%04x\n", mIndex, S_dwStatus));
        /* 更新本地“当前状态”：进入故障态（后续会走 fault reset 分支） */
        pDemoAxis->wActState = DRV_DEV_STATE_MALFUNCTION;
      } else {
        /* ====== 非 Fault 场景：解析 0x6041 StatusWord -> 本地 wActState（当前状态） ======
         *
         * 这里做的事：把从站上报的 StatusWord（16bit）翻译成“状态机状态枚举”：
         *   - DRV_DEV_STATE_NOT_READY / SWITCHON_DIS / READY_TO_SWITCHON / SWITCHED_ON / OP_ENABLED / QUICK_STOP ...
         *
         * 为什么要用 “掩码 + 固定值”：
         * - CiA402 的状态不是单独看某一位，而是看一组关键位组合（bit0/1/2/3/5/6 等）。
         * - 所以通常做法是：先用 mask 把无关位清掉，再与标准状态码比较。
         *
         * 本 demo 用了两种 mask：
         * - STATUSWORD_STATE_MASK_EN（0x004F）：用于解析 NotReady / SwitchOnDisabled（不关心 bit5 QuickStop）
         * - STATUSWORD_STATE_MASK（0x006F）：用于解析 Ready/SwitchedOn/OpEnabled/QuickStopActive（包含 bit5/bit6）
         */
        if (((S_dwStatus & STATUSWORD_STATE_MASK_EN) ==
             STATUSWORD_STATE_NOTREADYTOSWITCHON) &&
            (pDemoAxis->wActState != DRV_DEV_STATE_NOT_READY)) {
          /* 状态：Not ready to switch on（未就绪） */
          EcLogMsg(EC_LOG_LEVEL_INFO,
                   (pEcLogContext, EC_LOG_LEVEL_INFO,
                    "Axis[%d] To Not ready to switch on 0x%04x\n", mIndex,
                    S_dwStatus));
          pDemoAxis->wActState = DRV_DEV_STATE_NOT_READY;
        } else if (((S_dwStatus & STATUSWORD_STATE_MASK_EN) ==
                    STATUSWORD_STATE_SWITCHEDONDISABLED) &&
                   (pDemoAxis->wActState != DRV_DEV_STATE_SWITCHON_DIS)) {
          /* 状态：Switch on disabled（禁止上电/未使能上电） */
          EcLogMsg(EC_LOG_LEVEL_INFO,
                   (pEcLogContext, EC_LOG_LEVEL_INFO,
                    "Axis[%d] To Switch on disabled 0x%04x\n", mIndex,
                    S_dwStatus));
          pDemoAxis->wActState = DRV_DEV_STATE_SWITCHON_DIS;
        } else if (((S_dwStatus & STATUSWORD_STATE_MASK) ==
                    STATUSWORD_STATE_READYTOSWITCHON) &&
                   (pDemoAxis->wActState != DRV_DEV_STATE_READY_TO_SWITCHON)) {
          /* 状态：Ready to switch on（准备好上电） */
          EcLogMsg(EC_LOG_LEVEL_INFO,
                   (pEcLogContext, EC_LOG_LEVEL_INFO,
                    "Axis[%d] To Ready to switch on 0x%04x\n", mIndex,
                    S_dwStatus));
          pDemoAxis->wActState = DRV_DEV_STATE_READY_TO_SWITCHON;
        } else if (((S_dwStatus & STATUSWORD_STATE_MASK) ==
                    STATUSWORD_STATE_SWITCHEDON) &&
                   (pDemoAxis->wActState != DRV_DEV_STATE_SWITCHED_ON)) {
          /* 状态：Switched on（已上电/已接通，但未使能运行） */
          EcLogMsg(EC_LOG_LEVEL_INFO,
                   (pEcLogContext, EC_LOG_LEVEL_INFO,
                    "Axis[%d] To Switched on 0x%04x\n", mIndex, S_dwStatus));
          pDemoAxis->wActState = DRV_DEV_STATE_SWITCHED_ON;
        } else if (((S_dwStatus & STATUSWORD_STATE_MASK) ==
                    STATUSWORD_STATE_QUICKSTOPACTIVE_EN) &&
                   (pDemoAxis->wActState != DRV_DEV_STATE_QUICK_STOP)) {
          /* 状态：Quick stop active（快速停止激活） */
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
                                       "Axis[%d] To Quick stop active 0x%04x\n",
                                       mIndex, S_dwStatus));
          pDemoAxis->wActState = DRV_DEV_STATE_QUICK_STOP;
        } else if (((S_dwStatus & STATUSWORD_STATE_MASK) ==
                    STATUSWORD_STATE_OPERATIONENABLED) &&
                   (pDemoAxis->wActState != DRV_DEV_STATE_OP_ENABLED)) {
          /* 状态：Operation enabled（已使能运行）
           * - 只有到这个状态，后续 MT_Workpd() 写入的 TargetPosition/TargetVelocity 才会被执行
           */
          EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
                                       "Axis[%d] To Operation enabled 0x%04x\n",
                                       mIndex, S_dwStatus));
          pDemoAxis->wActState = DRV_DEV_STATE_OP_ENABLED;
        }

        /* 如果“当前状态 == 期望状态”，说明状态机已到位，不需要再写新的控制字 */
        if (pDemoAxis->wActState == pDemoAxis->wReqState)
          continue;
      }
      /* 3) 根据当前状态决定写什么控制字（0x6040），推动状态转换
       *
       * 典型最短路径（从 SwitchOnDisabled 到 OperationEnabled）：
       *   Shutdown(0x0006) → SwitchOn(0x0007) → EnableOp(0x000F)
       *
       * 这就是“为什么电机会动”的第一步：只有驱动进了 Operation enabled，
       * 它才会执行你写入的 TargetPosition/TargetVelocity。
       */
      switch (pDemoAxis->wActState) {
      case DRV_DEV_STATE_NOT_READY:
      case DRV_DEV_STATE_SWITCHON_DIS:
        /* Shutdown: 0x0006（最短路径第一步） */
        wControl = DRV_CTRL_CMD_SHUTDOWN;
        break;
      case DRV_DEV_STATE_READY_TO_SWITCHON:
        /* SwitchOn: 0x0007（最短路径第二步） */
        wControl = DRV_CTRL_CMD_SWITCHON;
        break;
      case DRV_DEV_STATE_SWITCHED_ON:
        /* EnableOperation: 0x000F（最短路径第三步） */
        wControl = DRV_CTRL_CMD_ENA_OPERATION;
        break;
      case DRV_DEV_STATE_OP_ENABLED:
        /* 已经在 OP_ENABLED：
         * - 如果上层要求退回 READY_TO_SWITCHON：写 Shutdown
         * - 如果上层仍要求 OP_ENABLED：继续保持 EnableOperation
         */
        if (DRV_DEV_STATE_READY_TO_SWITCHON == pDemoAxis->wReqState) {
          wControl = DRV_CTRL_CMD_SHUTDOWN;
        } else if (DRV_DEV_STATE_OP_ENABLED == pDemoAxis->wReqState) {
          wControl = DRV_CTRL_CMD_ENA_OPERATION;
        }
        break;
      case DRV_DEV_STATE_QUICK_STOP:
        /* Quick stop active：demo 选择写 Shutdown 退回（工程化时要结合 quick stop option code） */
        wControl = DRV_CTRL_CMD_SHUTDOWN;
        break;
      case DRV_DEV_STATE_MALFCT_REACTION:
      case DRV_DEV_STATE_MALFUNCTION:
        /* 故障态：示例做“复位脉冲”，并在一定计数后发 disable voltage */
        wControl = DRV_CTRL_CMD_RESET_MALFCT;
        if (pDemoAxis->dwResetCount++ > 20) {
          if (pDemoAxis->dwResetCount++ > 22) {
            pDemoAxis->dwResetCount = 0;
          }
          wControl = DRV_CTRL_CMD_DIS_VOLTAGE;
        }
        break;
      default:
        wControl = DRV_CTRL_CMD_SHUTDOWN;
        break;
      }
    }

    /* 4) 写控制字（0x6040）到 PdOut：下一次 SendAllCycFrames 时就会送到从站 */
    if (pDemoAxis->pwControlWord != EC_NULL) {
      /* 真正“落地”的动作：把 wControl 写进 0x6040 ControlWord */
      EC_SETWORD(pDemoAxis->pwControlWord, wControl);
    }
  }
  return EC_E_NOERROR;
}

/* stop/shutdown 时等待轴不再处于 OP_ENABLED。
 * 注意：这是 demo 级实现：
 * - 判断条件写法比较绕（等价于 wActState == OP_ENABLED）
 * - 也没有更新 wActState 的实时机制（依赖外部周期刷新）
 * 工程化使用需要更严谨的停止逻辑（例如依据速度为 0、状态机到 switched on/ready
 * 等）。
 */
static EC_T_VOID CheckMotorStateStop(EC_T_VOID) {
  CEcTimer oTimeout;
  /* 最多等 2 秒，避免死等 */
  oTimeout.Start(2000);
  EC_T_INT MovingStop;

  do {
    MovingStop = 0;
    for (EC_T_INT mIndex = 0; mIndex < MotorCount; mIndex++) {
      My_Motor_Type *pDemoAxis = &My_Motor[mIndex];
      /* 这里的判断写法比较绕：
       *   !(OP_ENABLED != wActState)  等价于  (wActState == OP_ENABLED)
       * demo 的意思是：统计“还有多少轴仍在 OP_ENABLED”，如果还有就继续等
       */
      if (!(DRV_DEV_STATE_OP_ENABLED != pDemoAxis->wActState) &&
          (EC_NULL != pDemoAxis)) {
        MovingStop += 1;
      }
    }
    if (MovingStop > 0)
      /* 让出 CPU，等待下一轮状态刷新 */
      OsSleep(1);
  } while ((MovingStop != 0) && !oTimeout.IsElapsed());
}
/*-----------------------------------------------------------------------------
 * [2026-01-22] MT_SetDriveSoftLimits
 *   设置驱动器内置软限位，通过 SDO 写入 0x607D:1 (最小限位) 和 0x607D:2 (最大限位)
 *   参数单位：弧度 (rad)，函数内部自动转换为编码器计数 (PUU)
 *---------------------------------------------------------------------------*/
#define DRV_OBJ_SOFTWARE_POSITION_LIMIT 0x607D
#define SDO_TIMEOUT 5000

EC_T_BOOL MT_SetDriveSoftLimits(EC_T_WORD wAxis, EC_T_LREAL fMinLimitRad, EC_T_LREAL fMaxLimitRad)
{
    if (wAxis >= MotorCount) {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR,
            "MT_SetDriveSoftLimits: Invalid axis %d (max=%d)\n", wAxis, MotorCount - 1));
        return EC_FALSE;
    }
    
    My_Motor_Type* pDemoAxis = &My_Motor[wAxis];
    if (pDemoAxis->wStationAddress == 0) {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR,
            "MT_SetDriveSoftLimits: Axis %d has no valid station address\n", wAxis));
        return EC_FALSE;
    }
    
    // 获取单位换算系数
    EC_T_LREAL fCntPerRad = pDemoAxis->fCntPerRad;
    if (fCntPerRad <= 0.0) {
        // 使用默认值：17-bit 编码器 (131072) * 减速比 81 / (2*PI)
        fCntPerRad = 131072.0 * 81.0 / (2.0 * MT_PI);
        EcLogMsg(EC_LOG_LEVEL_WARNING, (pEcLogContext, EC_LOG_LEVEL_WARNING,
            "MT_SetDriveSoftLimits: Axis %d using default fCntPerRad=%.2f\n", wAxis, fCntPerRad));
    }
    
    // 将弧度转换为编码器计数 (PUU)
    EC_T_INT nMinLimitPuu = MtSatToInt32(fMinLimitRad * fCntPerRad);
    EC_T_INT nMaxLimitPuu = MtSatToInt32(fMaxLimitRad * fCntPerRad);
    
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
        "Axis[%d] Setting soft limits: min=%.4f rad (%d PUU), max=%.4f rad (%d PUU)\n",
        wAxis, fMinLimitRad, nMinLimitPuu, fMaxLimitRad, nMaxLimitPuu));
    
    EC_T_DWORD dwRes;
    EC_T_DWORD dwDataLen = sizeof(EC_T_INT);
    
    // 获取 slave ID（通过站地址查找）
    EC_T_DWORD dwSlaveId = ecatGetSlaveId(pDemoAxis->wStationAddress);
    if (dwSlaveId == INVALID_SLAVE_ID) {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR,
            "MT_SetDriveSoftLimits: Axis %d - cannot find slave with address %d\n", 
            wAxis, pDemoAxis->wStationAddress));
        return EC_FALSE;
    }
    
    // 写入最小限位 (0x607D:1)
    dwRes = ecatCoeSdoDownload(
        dwSlaveId,
        DRV_OBJ_SOFTWARE_POSITION_LIMIT,
        1,  // subindex 1 = min position limit
        (EC_T_BYTE*)&nMinLimitPuu,
        dwDataLen,
        SDO_TIMEOUT,
        0
    );
    if (dwRes != EC_E_NOERROR) {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR,
            "Axis[%d] Failed to set min soft limit (0x607D:1): 0x%08X\n", wAxis, dwRes));
        return EC_FALSE;
    }
    
    // 写入最大限位 (0x607D:2)
    dwRes = ecatCoeSdoDownload(
        dwSlaveId,
        DRV_OBJ_SOFTWARE_POSITION_LIMIT,
        2,  // subindex 2 = max position limit
        (EC_T_BYTE*)&nMaxLimitPuu,
        dwDataLen,
        SDO_TIMEOUT,
        0
    );
    if (dwRes != EC_E_NOERROR) {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR,
            "Axis[%d] Failed to set max soft limit (0x607D:2): 0x%08X\n", wAxis, dwRes));
        return EC_FALSE;
    }
    
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
        "Axis[%d] Soft limits set successfully\n", wAxis));
    
    return EC_TRUE;
}

/* ============================================================================
 * [2026-01-27] MT_SetHomingMethod35 - 将当前位置设置为零点
 * 
 * 功能：通过 SDO 写入 Homing Method = 35，将驱动器当前位置设为零点
 * 
 * CiA 402 Homing 状态机流程：
 *   1. 切换到 Homing 模式 (0x6060 = 6)
 *   2. 设置 Homing Method = 35
 *   3. 状态机切换：Shutdown -> Switch On -> Enable Operation
 *   4. 发送 Homing Start (ControlWord bit 4 = 1)
 *   5. 等待 Homing Attained (StatusWord bit 12 = 1) 或超时
 *   6. 切换回 CSP 模式
 * 
 * 参数：
 *   wAxis - 轴号 (0-13)
 * 
 * 返回：
 *   EC_TRUE  - 设置成功
 *   EC_FALSE - 设置失败
 * ============================================================================ */
EC_T_BOOL MT_SetHomingMethod35(EC_T_WORD wAxis)
{
    if (wAxis >= MotorCount) {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR,
            "MT_SetHomingMethod35: Invalid axis %d (max=%d)\n", wAxis, MotorCount - 1));
        return EC_FALSE;
    }
    
    My_Motor_Type* pDemoAxis = &My_Motor[wAxis];
    EC_T_DWORD dwSlaveId = ecatGetSlaveId(pDemoAxis->wStationAddress);
    if (dwSlaveId == INVALID_SLAVE_ID) {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR,
            "MT_SetHomingMethod35: Axis %d - cannot find slave\n", wAxis));
        return EC_FALSE;
    }
    
    if (pDemoAxis->pwControlWord == EC_NULL || pDemoAxis->pwStatusWord == EC_NULL) {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR,
            "MT_SetHomingMethod35: Axis %d - PDO not mapped\n", wAxis));
        return EC_FALSE;
    }
    
    EC_T_DWORD dwRes;
    EC_T_WORD wStatusWord;
    
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
        "Axis[%d] Starting Homing Method 35 (keep enabled)...\n", wAxis));
    
    /* === 保持使能状态下执行 Homing === */
    
    /* 1. 设置 0x6098 = 35 (Homing Method = 当前位置为原点) */
    EC_T_SBYTE byMethod = 35;
    dwRes = ecatCoeSdoDownload(dwSlaveId, 0x6098, 0,
        (EC_T_BYTE*)&byMethod, sizeof(byMethod), SDO_TIMEOUT, 0);
    if (dwRes != EC_E_NOERROR) {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR,
            "Axis[%d] Failed to set 0x6098=35: 0x%08X\n", wAxis, dwRes));
        return EC_FALSE;
    }
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
        "Axis[%d] Step 1: 0x6098 = 35 (Homing Method)\n", wAxis));
    
    /* 2. 【关键修复】切换 eModesOfOperation 为 Homing，让 PDO 周期写入也发 Homing 模式
     *    之前的 bug：只通过 SDO 写 0x6060=6，但 MT_Workpd() 每周期通过 PDO 写 0x6060=CSP(8)，
     *    导致 SDO 设置被立即覆盖，驱动器始终没有真正进入 Homing 模式。
     */
    MC_T_CIA402_OPMODE eOldMode = pDemoAxis->eModesOfOperation;
    pDemoAxis->eModesOfOperation = DRV_MODE_OP_HOMING;  /* PDO 周期写入也切换到 Homing */
    OsSleep(100);  /* 等几个周期让 PDO 生效 */
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
        "Axis[%d] Step 2: eModesOfOperation = 6 (Homing Mode via PDO)\n", wAxis));
    
    /* 3. 确保处于 Operation Enabled 状态 */
    EC_SETWORD(pDemoAxis->pwControlWord, 0x000F);
    OsSleep(100);
    
    /* 4. 启动 Homing (ControlWord bit 4 = 1) */
    EC_SETWORD(pDemoAxis->pwControlWord, 0x001F);
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
        "Axis[%d] Step 4: Homing started (ControlWord=0x001F)\n", wAxis));
    
    /* 5. 等待 Homing 完成（两阶段检测，避免 CSP 模式下 bit12 残留导致假阳性）
     *
     * 问题背景：
     *   - 在 CSP 模式下，StatusWord bit12 = 1 表示 "drive follows command"
     *   - 切换到 Homing 模式后，bit12 含义变为 "Homing Attained"
     *   - 如果不等 bit12 先清零，会误判为 Homing 已完成（假阳性）
     *
     * 方案：
     *   阶段1：等待 bit12 变为 0（确认驱动器已切换到 Homing 模式，旧 bit12 已清除）
     *   阶段2：等待 bit12 变为 1（真正的 Homing Attained）
     */
    int timeout_ms = 5000;
    int elapsed = 0;
    EC_T_BOOL bHomingOk = EC_FALSE;
    
    /* 阶段1：等待 bit12 清零（最多等 1 秒，某些驱动可能直接跳过） */
    EC_T_BOOL bBit12Cleared = EC_FALSE;
    while (elapsed < 1000) {
        OsSleep(20);
        elapsed += 20;
        wStatusWord = EC_GETWORD(pDemoAxis->pwStatusWord);
        if (!(wStatusWord & 0x1000)) {
            bBit12Cleared = EC_TRUE;
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
                "Axis[%d] Homing bit12 cleared, StatusWord=0x%04X (%dms)\n", wAxis, wStatusWord, elapsed));
            break;
        }
        /* Homing error (bit 13 = 1) */
        if (wStatusWord & 0x2000) {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR,
                "Axis[%d] Homing error during phase1! StatusWord=0x%04X\n", wAxis, wStatusWord));
            break;
        }
    }
    if (!bBit12Cleared) {
        EcLogMsg(EC_LOG_LEVEL_WARNING, (pEcLogContext, EC_LOG_LEVEL_WARNING,
            "Axis[%d] bit12 did not clear in 1s (StatusWord=0x%04X), continue waiting for homing...\n", wAxis, wStatusWord));
    }
    
    /* 阶段2：等待 bit12 置位（Homing Attained） */
    while (elapsed < timeout_ms) {
        OsSleep(50);
        elapsed += 50;
        
        wStatusWord = EC_GETWORD(pDemoAxis->pwStatusWord);
        
        if (wStatusWord & 0x1000) {
            bHomingOk = EC_TRUE;
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
                "Axis[%d] Homing attained! StatusWord=0x%04X (elapsed=%dms)\n", wAxis, wStatusWord, elapsed));
            break;
        }
        
        /* Homing error (bit 13 = 1) */
        if (wStatusWord & 0x2000) {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR,
                "Axis[%d] Homing error! StatusWord=0x%04X\n", wAxis, wStatusWord));
            break;
        }
    }
    
    if (!bHomingOk && elapsed >= timeout_ms) {
        wStatusWord = EC_GETWORD(pDemoAxis->pwStatusWord);
        EcLogMsg(EC_LOG_LEVEL_WARNING, (pEcLogContext, EC_LOG_LEVEL_WARNING,
            "Axis[%d] Homing timeout, StatusWord=0x%04X\n", wAxis, wStatusWord));
    }
    
    /* 6. 恢复 CSP 模式（通过 eModesOfOperation，让 PDO 周期写入也切回 CSP）*/
    pDemoAxis->eModesOfOperation = eOldMode;
    OsSleep(100);  /* 等几个周期让 PDO 生效 */
    
    /* 7. 重新使能 */
    EC_SETWORD(pDemoAxis->pwControlWord, 0x0006);
    OsSleep(50);
    EC_SETWORD(pDemoAxis->pwControlWord, 0x0007);
    OsSleep(50);
    EC_SETWORD(pDemoAxis->pwControlWord, 0x000F);
    OsSleep(100);
    
    if (bHomingOk) {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
            "Axis[%d] Homing Method 35 completed - position should now be zero\n", wAxis));
    }
    
    return bHomingOk;
}


/* ============================================================================
 * [2026-01-28] DDS 工作模式相关函数实现
 * ============================================================================ */

/* 全局运行模式（调试模式 or 工作模式）*/
static RunMode S_RunMode = RUN_MODE_DEBUG;  /* 默认调试模式 */

/* [DDS] 设置运行模式 */
EC_T_VOID MT_SetRunMode(RunMode mode)
{
    S_RunMode = mode;
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO,
        "[DDS] Run mode set to: %s\n", 
        (mode == RUN_MODE_DEBUG) ? "DEBUG" : "WORK"));
}

/* [DDS] 获取运行模式 */
RunMode MT_GetRunMode(EC_T_VOID)
{
    return S_RunMode;
}

/* [DDS] CRC32 校验函数
 * 和 stark_sdk_cpp 中的 Crc32Core 保持一致
 */
uint32_t MT_Crc32(uint32_t* ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }
            if (data & xbit) {
                CRC32 ^= dwPolynomial;
            }
            xbit >>= 1;
        }
    }
    return CRC32;
}

/* [DDS] 单个电机命令转换：DDS_MotorCmd -> MotorCmd_
 * 
 * 转换说明：
 *   - DDS 的 mode 字段：0=关闭, 1=使能
 *   - 内部的 motion_func：根据 mode 设置为 SHUTDOWN 或 CONTROL
 *   - 内部的 drive_mode：使用全局驱动模式（调试模式下可设置，默认CSP）
 */
EC_T_VOID MT_ConvertDDSCmdToMotorCmd(const DDS_MotorCmd* pDDSCmd, MotorCmd_* pMotorCmd, DriveMode driveMode)
{
    if (pDDSCmd == EC_NULL || pMotorCmd == EC_NULL) {
        return;
    }
    
    /* 清空目标结构体 */
    OsMemset(pMotorCmd, 0, sizeof(MotorCmd_));
    
    /* 设置驱动模式（使用传入的模式，通常是全局模式）*/
    pMotorCmd->drive_mode = (EC_T_BYTE)driveMode;
    
    /* 根据 DDS mode 字段设置 motion_func
     * DDS mode: 0=关闭, 1=使能
     * 内部 motion_func: MOTION_SHUTDOWN / MOTION_CONTROL
     */
    if (pDDSCmd->mode == 0) {
        pMotorCmd->motion_func = MOTION_SHUTDOWN;
    } else {
        pMotorCmd->motion_func = MOTION_CONTROL;
    }
    
    /* 复制控制参数 */
    pMotorCmd->q   = (EC_T_REAL)pDDSCmd->q;    /* 目标位置 */
    pMotorCmd->dq  = (EC_T_REAL)pDDSCmd->dq;   /* 目标速度 */
    pMotorCmd->tau = (EC_T_REAL)pDDSCmd->tau;  /* 前馈力矩 */
    pMotorCmd->kp  = (EC_T_REAL)pDDSCmd->kp;   /* 刚度 */
    pMotorCmd->kd  = (EC_T_REAL)pDDSCmd->kd;   /* 阻尼 */
    
    /* 方向默认为正向（DDS 没有 direction 字段）*/
    pMotorCmd->direction = 0;
}

/* [DDS] 单个电机状态转换：MotorState_ -> DDS_MotorState */
EC_T_VOID MT_ConvertMotorStateToDDS(const MotorState_* pMotorState, DDS_MotorState* pDDSState)
{
    if (pMotorState == EC_NULL || pDDSState == EC_NULL) {
        return;
    }
    
    /* 清空目标结构体 */
    OsMemset(pDDSState, 0, sizeof(DDS_MotorState));
    
    /* 复制状态数据 */
    pDDSState->mode       = pMotorState->mode;
    pDDSState->q          = pMotorState->q_fb;       /* 实际位置 */
    pDDSState->dq         = pMotorState->dq_fb;      /* 实际速度 */
    pDDSState->ddq        = pMotorState->ddq_fb;     /* 实际加速度 */
    pDDSState->tau_est    = pMotorState->tau_fb;     /* 估计力矩 */
    pDDSState->vol        = pMotorState->vol;        /* 母线电压 */
    pDDSState->motorstate = pMotorState->motorstate; /* 状态字 */
    
    /* 温度 */
    pDDSState->temperature[0] = (int16_t)pMotorState->temperature[0];
    pDDSState->temperature[1] = (int16_t)pMotorState->temperature[1];
    
    /* 传感器数据 */
    pDDSState->sensor[0] = pMotorState->sensor[0];
    pDDSState->sensor[1] = pMotorState->sensor[1];
}

/* [DDS] 处理 DDS 命令
 * 
 * 功能：
 *   1. 校验 CRC
 *   2. 提取 motor_cmd[14]~[20]（第15-21个电机）
 *   3. 转换并写入内部 MotorCmd_[0]~[6]
 * 
 * 索引映射：
 *   DDS 索引 14 -> 内部索引 0 -> 轴1
 *   DDS 索引 15 -> 内部索引 1 -> 轴2
 *   ...
 *   DDS 索引 20 -> 内部索引 6 -> 轴7
 */
EC_T_BOOL MT_ProcessDDSCommand(const DDS_LowCmd* pDDSCmd)
{
    if (pDDSCmd == EC_NULL) {
        return EC_FALSE;
    }
    
    /* 1. CRC 校验
     * 计算除 crc 字段外的所有数据的 CRC32
     * 数据长度 = (结构体大小 / 4) - 1（减去 crc 字段）
     */
    uint32_t calcCrc = MT_Crc32((uint32_t*)pDDSCmd, 
                                 (sizeof(DDS_LowCmd) / sizeof(uint32_t)) - 1);
    if (calcCrc != pDDSCmd->crc) {
        EcLogMsg(EC_LOG_LEVEL_WARNING, (pEcLogContext, EC_LOG_LEVEL_WARNING,
            "[DDS] CRC check failed! Expected: 0x%08X, Got: 0x%08X\n",
            calcCrc, pDDSCmd->crc));
        return EC_FALSE;
    }
    
    /* 2. 获取当前全局驱动模式（调试模式下可设置，默认CSP）*/
    DriveMode driveMode = MT_GetGlobalDriveMode();
    
    /* 3. 遍历我们使用的电机，进行索引映射和命令转换
     * DDS 索引: DDS_MOTOR_OFFSET + i = 14 + i
     * 内部索引: i = 0 ~ 6
     */
    for (int i = 0; i < DDS_MOTOR_USED; i++) {
        int ddsIndex = DDS_MOTOR_OFFSET + i;  /* DDS 数组索引: 14, 15, ..., 20 */
        
        /* 转换 DDS 命令到内部格式 */
        MotorCmd_ motorCmd;
        MT_ConvertDDSCmdToMotorCmd(&pDDSCmd->motor_cmd[ddsIndex], &motorCmd, driveMode);
        
        /* 写入内部命令缓冲区（内部索引: 0, 1, ..., 6）*/
        MT_SetMotorCmd((EC_T_WORD)i, &motorCmd);
    }
    
    return EC_TRUE;
}

/* [DDS] 获取 DDS 状态
 * 
 * 功能：
 *   1. 从内部 MotorState_[0]~[6] 读取状态
 *   2. 填充到 DDS_LowState 的 motor_state[14]~[20]
 *   3. 计算并填充 CRC
 */
EC_T_VOID MT_GetDDSState(DDS_LowState* pDDSState)
{
    if (pDDSState == EC_NULL) {
        return;
    }
    
    /* 清空结构体 */
    OsMemset(pDDSState, 0, sizeof(DDS_LowState));
    
    /* 填充版本信息（可根据需要修改）*/
    pDDSState->version[0] = 1;
    pDDSState->version[1] = 0;
    
    /* 填充模式信息 */
    pDDSState->mode_pr = 0;
    pDDSState->mode_machine = (uint8_t)MT_GetRunMode();
    
    /* TODO: 填充 tick（时间戳）*/
    static uint32_t s_tick = 0;
    pDDSState->tick = s_tick++;
    
    /* 遍历我们使用的电机，进行状态转换
     * 内部索引: i = 0 ~ 6
     * DDS 索引: DDS_MOTOR_OFFSET + i = 14 + i
     */
    for (int i = 0; i < DDS_MOTOR_USED; i++) {
        int ddsIndex = DDS_MOTOR_OFFSET + i;  /* DDS 数组索引: 14, 15, ..., 20 */
        
        /* 获取内部电机状态 */
        MotorState_ motorState;
        MT_GetMotorState((EC_T_WORD)i, &motorState);
        
        /* 转换到 DDS 格式 */
        MT_ConvertMotorStateToDDS(&motorState, &pDDSState->motor_state[ddsIndex]);
    }
    
    /* 计算并填充 CRC */
    pDDSState->crc = MT_Crc32((uint32_t*)pDDSState, 
                               (sizeof(DDS_LowState) / sizeof(uint32_t)) - 1);
}

/*--------------------------------------------------------------------------------END
 * OF SOURCE
 * FILE----------------------------------------------------------------------*/
