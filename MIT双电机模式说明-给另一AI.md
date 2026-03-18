# MIT 模式控制说明（供上层/另一 AI 写程序用）

## 0. 使用范围与其它模式一致

**MIT 模式与 PT/CSP/CST 一样**：每个关节对应**一个电机**（一轴一机），无对装。电机数量、DDS 索引与内部轴号的对应关系由 **motor_map**（cfg.json 或 busi.yaml）决定，与其它驱动模式相同。

**ENI 文件**：`conf/eni.xml`（工程根目录下的 conf/eni.xml）。  
**PDO 映射**：支持 MIT 的驱动器使用 **RxPDO 0x1601（Sm=2）** 与 **TxPDO 0x1A01（Sm=3）**，且需包含 0x4xxx 对象（见下节表格）。

**PDO 对象字典**（与 conf/eni.xml 一致，按映射顺序）：

| 方向 | 索引 | 子索引 | 名称 | 数据类型 | 位长 | 单位/说明 |
|------|------|--------|------|----------|------|-----------|
| RxPDO | 0x6040 | 0 | Controlword | UINT | 16 | 状态机控制字 |
| RxPDO | 0x6060 | 0 | Mode of Operation | SINT | 8 | MIT 模式 = -6 |
| RxPDO | 0x4000 | 0 | Mit TargetPos | REAL | 32 | rad，期望位置 |
| RxPDO | 0x4001 | 0 | Mit TargetVel | REAL | 32 | rad/s，期望速度 |
| RxPDO | 0x4002 | 0 | Mit TargeTor | REAL | 32 | N·m，前馈力矩 |
| RxPDO | 0x4003 | 0 | Mit Kp | REAL | 32 | 刚度 |
| RxPDO | 0x4004 | 0 | Mit Kd | REAL | 32 | 阻尼 |
| TxPDO | 0x6041 | 0 | Statusword | UINT | 16 | 状态字，0x1237=Operation enabled |
| TxPDO | 0x6061 | 0 | Mode of Operation Display | SINT | 8 | 当前模式，-6=MIT |
| TxPDO | 0x4007 | 0 | Mit ActualPos | REAL | 32 | rad，位置反馈 |
| TxPDO | 0x4008 | 0 | Mit ActualVel | REAL | 32 | rad/s，速度反馈 |
| TxPDO | 0x4009 | 0 | Mit ActualTor | REAL | 32 | N·m，力矩反馈 |

主站每周期写 RxPDO（0x4000~0x4004 + 0x6060），读 TxPDO（0x4007~0x4009）得到 q_fb、dq_fb、tau_fb。

---

## 1. 硬件与拓扑

- **电机数量与拓扑**：由 ENI 与 motor_map 决定，**一轴一机**，与 PT/CSP/CST 一致。
- **轴号约定**：内部轴号 `wAxis = 0, 1, 2, ...` 对应轴 1、2、3、...；DDS 索引由 `MotorToDdsIndex(wAxis)` / motor_map 确定。

---

## 2. MIT 模式公式（在驱动器内计算）

驱动器按下列公式计算输出力矩（主站只下发右侧各项，不参与计算）：

```
输出力矩 = 输入力矩 + kp×(期望位置 - 实际位置) + kd×(期望速度 - 实际速度)
         = tau   + kp×(q_des - q_fb)         + kd×(dq_des - dq_fb)
```

- **tau**：前馈力矩 (N·m)
- **q_des**：期望位置 (rad)
- **dq_des**：期望速度 (rad/s)
- **kp**：刚度 (N·m/rad)
- **kd**：阻尼 (N·m·s/rad)
- **q_fb / dq_fb**：驱动器内部反馈（编码器位置/速度），主站可通过 TxPDO 读回。

主站需**每周期**稳定下发 tau、q、dq、kp、kd 和 0x6060=-6（MIT 模式）；无需“数值变化”驱动器才会更新。

---

## 3. PDO 映射（CoE 对象索引）

- **RxPDO（主站→驱动器）**
  - 0x6040：Controlword（状态机）
  - 0x6060：Mode of Operation，**MIT 模式 = -6**（必须为有符号 -6，不能写成 250）
  - 0x4000：Mit TargetPos (float, rad)
  - 0x4001：Mit TargetVel (float, rad/s)
  - 0x4002：Mit TargeTor (float, N·m)
  - 0x4003：Mit Kp (float)
  - 0x4004：Mit Kd (float)

- **TxPDO（驱动器→主站）**
  - 0x6041：Statusword
  - 0x6061：Mode of Operation Display（读回 -6 表示当前为 MIT）
  - 0x4007：Mit ActualPos (float, rad)
  - 0x4008：Mit ActualVel (float, rad/s)
  - 0x4009：Mit ActualTor (float, N·m)

ENI 中支持 MIT 的驱动器使用上述 RxPDO/TxPDO，且需包含上述 0x4xxx 对象。

---

## 4. 上层 API（与本工程一致）

### 4.1 轴数与模式

- `MT_GetMotorCount()`：返回当前使用的电机数量（由 motor_map 决定，与其它模式一致）。
- `MT_SetGlobalDriveMode(DRIVE_MODE_MIT)`：全局切到 MIT 模式（0x6060=-6）。在发 MIT 指令前需先设为 MIT。
- `MT_GetGlobalDriveMode()`：当前全局模式。

### 4.2 下发 MIT 指令

- **函数**：`void MT_SetMotorCmd(EC_T_WORD wAxis, const MotorCmd_* pCmd);`
- **轴号**：`wAxis = 0, 1, ...` 对应轴 1、2、...，与 PT/CSP/CST 一致。
- **结构体**（MIT 时只关心以下字段，单位与上节一致）：

```c
typedef struct _MotorCmd_ {
    EC_T_SBYTE drive_mode;   // 必须 = DRIVE_MODE_MIT (-6)，类型必须为有符号
    EC_T_BYTE  motion_func;  // 正常控制填 MOTION_CONTROL (3)，停机填 MOTION_SHUTDOWN (4)
    EC_T_REAL  q;            // 期望位置 (rad)  -> 0x4000
    EC_T_REAL  dq;           // 期望速度 (rad/s)-> 0x4001
    EC_T_REAL  tau;          // 前馈力矩 (N·m)  -> 0x4002
    EC_T_REAL  kp;           // 刚度             -> 0x4003
    EC_T_REAL  kd;           // 阻尼             -> 0x4004
    // 其余字段 MIT 模式下可置 0
} MotorCmd_;
```

- **重要**：
  - `drive_mode` 必须为 **EC_T_SBYTE**，且赋值为 **DRIVE_MODE_MIT (-6)**；若用无符号类型，-6 会变成 250，MIT 分支不生效。
  - MIT 模式下 **kp/kd 由 PDO 每周期下发**，必须使用本次 `pCmd` 里的 kp/kd，不可沿用旧缓存。

### 4.3 读反馈

- **函数**：`EC_T_BOOL MT_GetMotorState(EC_T_WORD wAxis, MotorState_* pStateOut);`
- **常用字段**（MIT 时由 0x4007/0x4008/0x4009 填）：

```c
typedef struct _MotorState_ {
    EC_T_BYTE  mode;         // 0x6061，MIT 时应为 -6（若用 byte 读可能显示 250，需按有符号解释）
    EC_T_REAL  q_fb;         // 位置反馈 (rad)
    EC_T_REAL  dq_fb;        // 速度反馈 (rad/s)
    EC_T_REAL  tau_fb;       // 力矩反馈 (N·m)
    EC_T_DWORD motorstate;   // 0x6041 Statusword，0x1237 表示 Operation enabled
} MotorState_;
```

---

## 5. 控制流程（建议）

1. 启动后 ENI 加载、总线 OP，`MT_Prepare`/`MT_Setup` 已完成（本工程在 `myAppPrepare` 中配置从站并调用）。
2. 调用 `MT_SetGlobalDriveMode(DRIVE_MODE_MIT)` 切到 MIT。
3. 每周期（与 EtherCAT 周期一致，如 1 ms）：根据规划或 DDS 填好 `MotorCmd_`（drive_mode=MIT, motion_func=MOTION_CONTROL, q/dq/tau/kp/kd），对每个轴调用 `MT_SetMotorCmd(wAxis, &cmd)`，与 PT/CSP/CST 的用法一致。
4. 需要读反馈时：`MT_GetMotorState(wAxis, &state)`，使用 q_fb、dq_fb、tau_fb 等。
5. 停机：将 `motion_func = MOTION_SHUTDOWN` 后对对应轴调用 `MT_SetMotorCmd`，或使用本工程的 estop/stop 流程。

---

## 6. 易错点与约束

- **drive_mode 类型**：必须为 **EC_T_SBYTE**，且与 **DRIVE_MODE_MIT (-6)** 比较时直接用 `== DRIVE_MODE_MIT`，否则 MIT 分支进不去。
- **MIT 下 kp/kd**：必须用本次命令的 kp/kd 全量写入，不可在 MIT 分支里“保留旧值”。
- **0x6060**：每周期写 -6（MIT）；0x6061 读回 -6（或无符号 250）表示当前确为 MIT。
- **单位**：位置/速度均为 **rad / rad/s**，力矩为 **N·m**。
- **轴号**：`wAxis` 从 0 起，与 motor_map、ENI 从站顺序一致，与其它模式相同。

---

## 7. 参考文件（本工程）

- **PDO 对象字典**：见第 0 节表格（与 conf/eni.xml 一致）；代码中对象索引与类型见 `motrotech.h` 宏定义及注释。
- 从站与轴配置：`EcDemoApp.cpp` 中 `myAppPrepare()` 的 `My_Slave[]`。
- MIT 下发与 PDO 写入：`motrotech.cpp` 中 `MT_Workpd()` 的 `DRIVE_MODE_MIT` 分支（写 0x4000~0x4004 与 0x6060）。
- 命令入口：`MT_SetMotorCmd()` 中 `drive_mode == DRIVE_MODE_MIT` 分支（全量写入，不保留 kp/kd）。
- 数据结构与常量：`motrotech.h`（MotorCmd_、MotorState_、DRIVE_MODE_MIT、0x4xxx 宏、MT_GetMotorCount/MT_GetMotorState/MT_SetMotorCmd/MT_SetGlobalDriveMode）。
- ENI：`conf/eni.xml`（RxPDO 0x1601、TxPDO 0x1A01 含 0x4000~0x4004、0x4007~0x4009）。
