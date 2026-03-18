# motor_map 与 MIT 模式说明（供另一程序/上层实现用）

本文档说明 **motor_map** 的作用与配置方式，以及 **MIT 模式** 的 PDO 映射与 API，便于另一程序接入：实现 motor_map 的读取/配置、以及 MIT 控制的数据结构与周期下发逻辑。不涉及本工程的 show/set 等具体交互命令。

---

## 一、motor_map 的作用与修改

### 1.1 作用

**motor_map** 定义两件事：

1. **有多少个 EtherCAT 电机参与控制**（电机数量）。
2. **每个电机对应全身的哪一号关节**（内部轴号 0,1,2,… 与 DDS 索引的对应关系）。

因此：

- **MotorCount**（参与控制的轴数）由 motor_map 决定。
- **内部轴号 wAxis**（0, 1, 2, …）与 **DDS 索引**（LowCmd/LowState 的 motor_cmd/motor_state 数组下标）的对应关系也由 motor_map 决定：内部轴 `i` 对应 DDS 索引 `dds_indices[i]`，同时该轴在 ENI 中的从站站地址为 `slave_addrs[i]`。

下游逻辑（如 DDS 命令处理）会：按内部轴号 0～MotorCount-1 循环，用 `MotorToDdsIndex(i)` 得到 DDS 索引，从 LowCmd 取该索引的命令下发到轴 `i`；读回时用 `MT_GetMotorState(i)` 填到 LowState 的对应索引。

### 1.2 配置来源（二选一）

- **来自 cfg.json**（推荐）：业务配置里 `motor_map_from_cfg: true` 时，从 **cfg.json** 的 `motor_map` 字段读取；与标定文件同一文件，便于一致。
- **来自 busi.yaml**：`motor_map_from_cfg: false` 时，从 **busi.yaml** 的 `ethercat_demo.motor_map` 读取。

另一程序只需实现其中一种（或两种都支持），并保证解析结果一致（见下）。

### 1.3 两种格式与解析结果

**格式 A：cfg.json 中 28 个 bool（当前推荐）**

- 位置：`cfg.json` 根节点下的 `"motor_map"`。
- 类型：长度为 28 的 bool 数组，下标即 **DDS 索引**（0～27）。
- 含义：`true` 表示该 DDS 索引对应一个 EtherCAT 电机；`false` 表示不对应。
- 站地址约定：按 DDS 索引顺序，第 1 个 `true` 对应站地址 1002，第 2 个 1003，依次递增。

解析逻辑（伪代码）：

```text
dds_indices = []
slave_addrs = []
next_addr = 1002
for i in 0..27:
  if motor_map[i] == true:
    dds_indices.append(i)
    slave_addrs.append(next_addr)
    next_addr += 1
```

得到：电机数量 `count = len(dds_indices)`，内部轴 `j` 对应 DDS 索引 `dds_indices[j]`、站地址 `slave_addrs[j]`。

**格式 B：busi.yaml / 列表格式**

- 位置：`busi.yaml` 中 `ethercat_demo.motor_map`，或 cfg.json 中 `motor_map` 为对象数组时。
- 每项：`slave_addr`（ENI 中该电机的站地址）、`dds_index`（DDS 索引，0～27）。
- 数组顺序即内部轴顺序：第 0 项对应内部轴 0，第 1 项对应内部轴 1，……

解析逻辑：遍历数组，收集 `slave_addr` 与 `dds_index`，得到 `slave_addrs[]` 和 `dds_indices[]`，长度即为电机数量。

### 1.4 下游使用（另一程序需实现的映射）

解析得到 `(slave_addrs, dds_indices, count)` 后，另一程序需要：

1. **从站/轴配置**：只对“参与控制”的电机建表，例如 `My_Slave[i].wStationAddress = slave_addrs[i]`，`My_Slave[i].wAxisCnt = 1`，共 `count` 个从站；ENI 中其他从站可不参与此表或按现有逻辑处理。
2. **轴↔DDS 映射**：内部轴号 `wAxis = i` 对应 DDS 索引 `dds_indices[i]`。下发时：从 LowCmd 的 `motor_cmd[dds_indices[i]]` 取命令发给轴 `i`；上报时：把轴 `i` 的状态写入 LowState 的 `motor_state[dds_indices[i]]`。
3. **MotorCount**：等于 `count`；周期内循环 `for i in 0..MotorCount-1` 做上述收发。

本工程中对应接口：`EcDemo_SetMotorConfig(slave_addrs, dds_indices, count, calib_path)` 会设置 `g_MotorCount`、`g_MotorSlaveAddr`、`g_MotorDdsIndex`，并调用 `MT_SetMotorMap(dds_indices, count)`；`myAppPrepare()` 中根据 `g_MotorCount` 填 `My_Slave[]`；`MT_Prepare()` 再根据 ENI 与 My_Slave 计算实际在线的 MotorCount。

### 1.5 修改 motor_map 的注意点

- 增加电机：在 cfg.json 的 28 个 bool 中多设几个 `true`，或在 busi 列表里增加 `{ slave_addr, dds_index }`；ENI 中必须有对应站地址的从站，且 PDO 一致。
- 减少电机：把对应位置改为 `false` 或从列表删除；保证 ENI 中仍存在的从站与 motor_map 一致，避免轴号错位。
- 修改后需重启或重新加载配置，使 MotorCount 与映射关系更新。

---

## 二、MIT 模式（与 PT/CSP/CST 并列）

MIT 模式与 PT/CSP/CST 一样：**一轴一机**，轴数与 DDS 映射由 motor_map 决定。以下仅描述 MIT 特有的 PDO、公式与数据结构，便于另一程序实现 MIT 映射与周期收发。

### 2.1 模式与 0x6060

- **0x6060（Mode of Operation）**：MIT 模式 = **-6**（有符号，不能写成无符号 250）。
- 主站需**每周期**写 0x6060 = -6，驱动器读回 0x6061 = -6 表示当前处于 MIT。

### 2.2 MIT 公式（在驱动器内计算）

主站只下发右侧变量，不参与计算：

```text
输出力矩 = tau + kp×(q_des - q_fb) + kd×(dq_des - dq_fb)
```

- tau：前馈力矩 (N·m)；q_des / dq_des：期望位置(rad)、速度(rad/s)；kp / kd：刚度、阻尼；q_fb / dq_fb：驱动器反馈，主站通过 TxPDO 读回。

### 2.3 PDO 映射（CoE 对象索引）

**RxPDO（主站→驱动器）**

| 索引   | 说明           | 类型  | 单位/说明        |
|--------|----------------|-------|------------------|
| 0x6040 | Controlword    | UINT16| 状态机           |
| 0x6060 | Mode of Operation | SINT8 | MIT = -6     |
| 0x4000 | Mit TargetPos  | REAL  | rad              |
| 0x4001 | Mit TargetVel  | REAL  | rad/s            |
| 0x4002 | Mit TargeTor   | REAL  | N·m              |
| 0x4003 | Mit Kp         | REAL  | 刚度             |
| 0x4004 | Mit Kd         | REAL  | 阻尼             |

**TxPDO（驱动器→主站）**

| 索引   | 说明           | 类型  | 单位/说明        |
|--------|----------------|-------|------------------|
| 0x6041 | Statusword     | UINT16| 0x1237 = Operation enabled |
| 0x6061 | Mode of Operation Display | SINT8 | -6 = MIT |
| 0x4007 | Mit ActualPos  | REAL  | rad               |
| 0x4008 | Mit ActualVel  | REAL  | rad/s             |
| 0x4009 | Mit ActualTor  | REAL  | N·m               |
| 0x4020 | （可选）外置力矩 | REAL  | N·m               |

ENI 中支持 MIT 的驱动器通常使用 RxPDO 0x1601、TxPDO 0x1A01，并包含上述 0x4xxx 对象；0x4020 若映射则读回为外置力矩，可填入状态结构体单独字段（如 tau_ext）。

### 2.4 数据结构（另一程序需对齐）

**下发（每周期写入 PDO）**

- `drive_mode`：**EC_T_SBYTE**，MIT 时为 **-6**（不可用无符号，否则 -6 变 250）。
- `motion_func`：正常控制填 **MOTION_CONTROL(3)**，停机填 **MOTION_SHUTDOWN(4)**。
- `q, dq, tau, kp, kd`：对应 0x4000～0x4004，单位 rad、rad/s、N·m。

**读回（从 PDO 填状态）**

- `mode`：0x6061，MIT 时为 -6（若用 byte 读可能为 250，需按有符号解释）。
- `q_fb, dq_fb, tau_fb`：0x4007～0x4009。
- 若有 0x4020：可填 `tau_ext`（外置力矩）。

### 2.5 周期逻辑（另一程序需实现）

1. 全局模式设为 MIT（等价于每周期写 0x6060 = -6）。
2. 每 EtherCAT 周期：
   - 对每个轴 `i`（0～MotorCount-1）：
     - 用 motor_map 得到 DDS 索引 `dds = dds_indices[i]`。
     - 从规划/ LowCmd 取该关节的 `MotorCmd_`（drive_mode=-6, motion_func=3, q/dq/tau/kp/kd）。
     - 将 q/dq/tau/kp/kd 写入该轴 RxPDO（0x4000～0x4004），0x6060 写 -6。
   - 从 TxPDO 读 0x4007～0x4009（及可选 0x4020），填该轴 `MotorState_`，再按 dds 写回 LowState。
3. kp/kd 必须每周期用本次命令全量写入，不可沿用旧值。

### 2.6 易错点

- **drive_mode** 必须为有符号类型且赋值为 -6，比较时用 `(EC_T_SBYTE)mode == (EC_T_SBYTE)DRIVE_MODE_MIT`。
- **0x6060** 每周期写 -6；无命令周期也要写 -6，否则驱动器可能退回其它模式。
- **单位**：位置/速度 rad、rad/s，力矩 N·m。

---

## 三、本工程参考位置（便于对照）

- motor_map 解析：`BasicService.cpp` 中根据 `motor_map_from_cfg` 读 cfg.json 或 busi.yaml，得到 `slave_addrs`、`dds_indices`，调用 `EcDemo_SetMotorConfig`。
- 从站/轴配置：`EcDemoApp.cpp` 中 `EcDemo_SetMotorConfig`、`myAppPrepare()` 的 `My_Slave[]`。
- 轴↔DDS 映射：`motrotech.cpp` 中 `MT_SetMotorMap`、`MotorToDdsIndex`、`MT_ProcessDDSCommand` / `MT_GetDDSState`。
- MIT PDO 写入/读回：`motrotech.cpp` 中 `MT_Workpd()` 的 `DRIVE_MODE_MIT` 分支，以及状态读取处对 0x4007～0x4009、0x4020 的填充。
- 常量与结构体：`motrotech.h`（DRIVE_MODE_MIT=-6、0x4xxx 宏、MotorCmd_、MotorState_）。

另一程序只需实现：**motor_map 的解析与配置**（得到 count、slave_addrs、dds_indices 并用于从站表与轴↔DDS 映射），以及 **MIT 的 PDO 映射与每周期下发/读回**，无需实现 show/set 等具体交互命令。
