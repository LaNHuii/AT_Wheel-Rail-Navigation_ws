# 轮轨导航上位机—STM32 CAN-FD 接口说明

版本：V1.2

日期：2026-10-17

适用范围：人物跟随行为树、进退轨局部状态机、自由路面/轨内控制与 STM32 底盘之间的接口。

> 本版按当前上位机代码整理。电控组实现时以本文的字节布局、
> `can_protocol.hpp/.cpp` 和 `test_can_protocol.cpp` 三者一致为验收标准。

## 1. 系统边界

```text
人物跟随任务 / 行为树
        │ ExecuteRailTransition.action（ROS 2）
        ▼
进轨、退轨、夹轨局部状态机
        │ Twist / Control / SteeringTask（ROS 2）
        ▼
wr_chassis_gateway
        │ 专用 CAN-FD（SocketCAN can0）
        ▼
STM32：仲裁、舵轮运动学、电机闭环、实时安全
        │
        ├── FDCAN1：驱动电机（现有）
        ├── FDCAN2：转向电机（现有）
        └── 底盘主 IMU
```

- 行为树只编排任务，不直接发 CAN，也不直接控制单个电机。
- 局部状态机负责对准、接近、前轮探测、前轮释放、后轮进轨、四轮终验和退轨。
- 上位机根据四轮转矩/角度反馈判断是否接触轨道；STM32 执行有边界的探测、保持、释放和实时硬保护。
- 夹轨接触不是 STM32 预先给出的“已上轨”真值。上位机从原始模块反馈作出判断后，对已接触轮发送 `HOLD`。
- 主 IMU 改为底盘 IMU。MID-360S 只提供点云，其内置 IMU 不作为主定位 IMU。

## 2. 物理接口

| 项目 | 要求 |
| --- | --- |
| 工控机—STM32 | 独立 CAN-FD 通道，工控机使用 USB-CAN-FD 或 PCIe-CAN-FD，Linux 暴露为 `can0` |
| CAN 标识符 | 11 bit 标准帧 |
| 建议波特率 | 仲裁段 1 Mbit/s，数据段 4 Mbit/s；最终以线束实测确认 |
| 线束 | 屏蔽双绞线 CAN_H/CAN_L，共地；总线两端各 120 Ω，不能形成星形长支线 |
| 负载 | CAN-FD，单帧最多 64 byte，不做应用层分片 |
| 电机总线 | 现有 FDCAN1/FDCAN2 保持不变，上位机链路不得直接混入电机总线 |
| 主 IMU | 接入 STM32，内部物理口由电控选 SPI/UART/CAN；STM32统一按本协议向上位机发送 |

若现有控制板两个 FDCAN 均已占用于电机，电控组必须增加第三路独立 CAN-FD 控制器/外部桥接器，或调整控制板硬件。不能在未做带宽、ID 和故障隔离设计的情况下共用电机总线。

说明：“通信全部采用 CAN”指上位机与 STM32 的底盘控制和状态链路。MID-360S 点云仍使用设备原生以太网，否则雷达无法按厂家链路工作。

## 3. CAN-FD 线协议

所有多字节字段采用小端序，浮点为 IEEE-754 `float32`。不得将 C 结构体用 `memcpy` 直接放入 CAN 帧；必须逐字段编码，以避免对齐和编译器差异。CAN-FD 已有链路 CRC，V1 不重复增加应用层 CRC。

所有帧前 8 byte 为公共头：

| 偏移 | 类型 | 字段 |
| --- | --- | --- |
| 0 | `uint8` | 协议版本，当前为 1 |
| 1 | `uint8` | 消息类型 |
| 2 | `uint16` | 标志位，V1 置 0 |
| 4 | `uint32` | 帧序号，发送方每发一帧递增 |

### 3.1 CAN ID 与周期

| CAN ID | 方向 | 内容 | 周期/触发 |
| --- | --- | --- | --- |
| `0x100` | 上位机→STM32 | 底盘速度 `vx/vy/wz`、有效期、会话号 | 50 Hz |
| `0x101` | 上位机→STM32 | 通用转向任务：探测/取消/释放/保持、模块掩码、角度/速度/超时 | 事件触发，需确认 |
| `0x102` | 上位机→STM32 | 使能、急停请求、清故障、控制源、底盘模式 | 事件触发，需立即确认；当前不周期保活 |
| `0x103` | 上位机→STM32 | 时间同步请求 | 1 Hz |
| `0x180` | STM32→上位机 | 整车状态、实测速度、故障、电池、最后命令序号 | 100 Hz |
| `0x181` | STM32→上位机 | FL 模块原始状态 | 100 Hz |
| `0x182` | STM32→上位机 | FR 模块原始状态 | 100 Hz |
| `0x183` | STM32→上位机 | RR 模块原始状态 | 100 Hz |
| `0x184` | STM32→上位机 | RL 模块原始状态 | 100 Hz |
| `0x185` | STM32→上位机 | 主 IMU 原始角速度、线加速度、温度、状态 | 推荐 200 Hz，最低 100 Hz |
| `0x186` | STM32→上位机 | 主 IMU 姿态四元数（若 IMU/STM32 有可信融合） | 与原始 IMU 同步；无则不发 |
| `0x187` | STM32→上位机 | 转向任务执行状态与各模块掩码 | 任务期间 50 Hz，结束至少发 3 次 |
| `0x188` | STM32→上位机 | 命令确认/拒绝及原因 | 收到 `0x101/0x102` 后立即发送 |
| `0x189` | STM32→上位机 | 故障事件 | 状态变化立即发送 |
| `0x18A` | STM32→上位机 | 心跳 | 10 Hz |
| `0x18B` | STM32→上位机 | 时间同步响应 | 每个请求响应 |

当前上位机已编解码 `0x100~0x103`、`0x180~0x188` 和 `0x18B`。
`0x189` 故障事件和 `0x18A` 心跳仅保留 ID，当前适配器尚未解码，
不作为首次联调的必须项。故障和在线状态首先通过 `0x180~0x188` 闭环。

### 3.2 模块顺序和掩码

固定顺序为 `FL, FR, RR, RL`：

| 模块 | 掩码 |
| --- | --- |
| FL | `0x01` |
| FR | `0x02` |
| RR | `0x04` |
| RL | `0x08` |
| 前轮组 | `0x03` |
| 四轮 | `0x0F` |

转向任务动作：`IDLE=0, PROBE=1, CANCEL=2, RELEASE=3, HOLD=4`。`HOLD` 必须只停止掩码选中的模块继续加角，并保持其当前角度；不能等待其它轮一起接触。

### 3.3 坐标、单位和编码约定

- 底盘坐标按 ROS 右手系：`+x` 车头向前，`+y` 车体向左，`+z` 向上。
- `vx/vy` 单位为 m/s，`wz` 单位为 rad/s，`wz>0` 表示从上方看逆时针旋转。
- 转向角和转向角速度分别为 rad、rad/s；转矩为 N·m，电流为 A，温度为 °C。
- 驱动位置为 rad，驱动速度为 rad/s。轮式/轨式有效半径由 STM32 按当前模式处理。
- 所有整数和 IEEE-754 `float32` 都按小端序逐字段编码；保留字节发送时置 0。

### 3.4 上位机→STM32 精确帧布局

#### `0x100` 底盘速度，长度 28 byte，消息类型 `0x10`

| 偏移 | 长度 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 8 | 公共头 | `version/type/flags/frame_sequence` | `type=0x10` |
| 8 | 4 | `float32` | `vx` | m/s，轨内允许受限负值 |
| 12 | 4 | `float32` | `vy` | m/s，轨内必须为 0 |
| 16 | 4 | `float32` | `wz` | rad/s，轨内必须为 0 |
| 20 | 2 | `uint16` | `valid_for_ms` | 当前为 200 ms |
| 22 | 2 | 保留 | - | 置 0 |
| 24 | 4 | `uint32` | `session_id` | 上位机会话号 |

#### `0x101` 通用转向/夹轨任务，长度 32 byte，消息类型 `0x11`

| 偏移 | 长度 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 8 | 公共头 | - | `type=0x11` |
| 8 | 4 | `uint32` | `task_sequence` | 任务幂等序号 |
| 12 | 1 | `uint8` | `action` | `0~4` |
| 13 | 1 | `uint8` | `module_mask` | `FL/FR/RR/RL=1/2/4/8` |
| 14 | 2 | 保留 | - | 置 0 |
| 16 | 4 | `float32` | `max_angle` | 最大探测角，rad |
| 20 | 4 | `float32` | `angular_speed` | 探测角速度，rad/s |
| 24 | 4 | `float32` | `timeout` | 任务超时，s |
| 28 | 4 | `uint32` | `session_id` | 上位机会话号 |

#### `0x102` 控制权与底盘模式，长度 16 byte，消息类型 `0x12`

| 偏移 | 长度 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 8 | 公共头 | - | `type=0x12` |
| 8 | 1 | `uint8` | `requested_control_source` | `NONE=0, MANUAL=1, AUTONOMOUS=2` |
| 9 | 1 | `uint8` | `requested_mode` | `FREE=0, ENTRY=1, ON_RAIL=2, EXIT=3` |
| 10 | 1 | 位标志 | `enable/quick_stop/clear_fault` | bit0/bit1/bit2 |
| 11 | 1 | 保留 | - | 置 0 |
| 12 | 4 | `uint32` | `session_id` | 上位机会话号 |

`0x102` 当前由上位机事件触发，不是周期保活帧。STM32 不得因为长时间未收到
`0x102` 而自动撤销已确认的控制源；运动看门狗以 50 Hz 的 `0x100` 为准。
ROS 2 `ChassisControlCommand.valid_for_ms` 当前不编码进 `0x102`，不得把它当作帧内字段。

#### `0x103` 时间同步请求，长度 16 byte，消息类型 `0x13`

| 偏移 | 长度 | 类型 | 字段 |
| --- | --- | --- | --- |
| 0 | 8 | 公共头 | `type=0x13` |
| 8 | 8 | `uint64` | `host_send_time_ns` |

### 3.5 STM32→上位机精确帧布局

#### `0x180` 整车状态，长度 52 byte，消息类型 `0x20`

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 8 | 公共头 |
| 8 | 8 | `sample_time_us` |
| 16 | 4 | `boot_id` |
| 20 | 4 | `last_command_sequence` |
| 24 | 1 | `active_control_source` |
| 25 | 1 | `active_mode` |
| 26 | 1 | bit0 `enabled`，bit1 `physical_estop`，bit2 `remote_online`，bit3 `command_timed_out`，bit4 `motor_power_ready` |
| 27 | 1 | 保留 |
| 28/32/36 | 4×3 | 实测 `vx/vy/wz` |
| 40 | 4 | `chassis_fault_bits` |
| 44/48 | 4×2 | `battery_voltage/battery_current` |

#### `0x181~0x184` 四模块状态，每帧 64 byte，消息类型 `0x21`

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 8 | 公共头 |
| 8 | 8 | `sample_time_us` |
| 16 | 1 | `module_id`：`FL=0, FR=1, RR=2, RL=3`，必须与 CAN ID 匹配 |
| 17 | 1 | bit0 转向在线，bit1 驱动在线，bit2 转向零点有效，bit3 转矩有效，bit4 驱动位置有效 |
| 18/20 | 2×2 | `steering_fault_bits/drive_fault_bits` |
| 22 | 2 | 保留 |
| 24/28 | 4×2 | 转向角 rad / 转向角速度 rad/s |
| 32/36 | 4×2 | 转向电流 A / 转向转矩 N·m |
| 40/44 | 4×2 | 驱动位置 rad / 驱动速度 rad/s |
| 48/52 | 4×2 | 驱动电流 A / 驱动转矩 N·m |
| 56/60 | 4×2 | 转向/驱动温度 °C |

#### `0x185` 主 IMU 原始数据，长度 48 byte，消息类型 `0x22`

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 8 | 公共头 |
| 8 | 8 | `sample_time_us` |
| 16/20/24 | 4×3 | `gyro_x/y/z`，rad/s |
| 28/32/36 | 4×3 | `accel_x/y/z`，m/s²，包含重力 |
| 40 | 4 | 温度 °C |
| 44 | 4 | bit0 陀螺有效，bit1 加速度有效，bit2 姿态有效，bit3 标定完成，bit8~31 故障位 |

#### `0x186` 可选 IMU 姿态，长度 36 byte，消息类型 `0x23`

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 8 | 公共头 |
| 8 | 8 | `sample_time_us` |
| 16/20/24/28 | 4×4 | 四元数 `x/y/z/w` |
| 32 | 4 | `status_bits`，bit2=1 才使用姿态 |

#### `0x187` 转向任务状态，长度 20 byte，消息类型 `0x24`

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 8 | 公共头 |
| 8 | 4 | `task_sequence` |
| 12/13 | 1×2 | `action/state` |
| 14/15/16/17 | 1×4 | `active/holding/completed/safety_stopped` 模块掩码 |
| 18 | 2 | `result_code` |

`state`：`IDLE=0, EXECUTING=1, HOLDING=2, COMPLETED=3, CANCELLED=4, TIMEOUT=5, SAFETY_STOPPED=6, FAULT=7`。

#### `0x188` 命令确认，长度 17 byte，消息类型 `0x25`

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 8 | 公共头 |
| 8 | 4 | `acknowledged_sequence` |
| 12 | 1 | 被确认的消息类型：`0x11` 或 `0x12` |
| 13 | 1 | `accepted`：0/1 |
| 14 | 2 | `result_code` |
| 16 | 1 | `active_module_mask` |

`result_code` 统一约定：`0=成功, 1=版本/类型/长度错误, 2=参数非法,
3=模式不允许, 4=控制源未授权, 5=忙, 6=过期或序号错误, 7=超时,
8=安全保护触发, 9=硬件故障`。其余保留。

#### `0x18B` 时间同步响应，长度 32 byte，消息类型 `0x28`

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 8 | 公共头 |
| 8 | 8 | 原样返回 `host_send_time_ns` |
| 16 | 8 | `mcu_receive_time_us` |
| 24 | 8 | `mcu_send_time_us` |

## 4. 主 IMU 数据要求

STM32 必须发送传感器实际采样时刻，而不是 CAN 发送时刻：

| 字段 | 类型/单位 | 必须性 |
| --- | --- | --- |
| `sample_time_us` | `uint64`，STM32 单调时钟 µs | 必须 |
| `angular_velocity[3]` | `float32`，rad/s，右手系 | 必须 |
| `linear_acceleration[3]` | `float32`，m/s²，包含重力 | 必须 |
| `temperature` | `float32`，°C | 建议 |
| `orientation_xyzw[4]` | `float32` | 可选，只有可信时发送 |
| `status_bits` | `uint32` | 必须 |

`status_bits` 最低定义：bit0 陀螺仪有效，bit1 加速度计有效，bit2 姿态有效，bit3 标定完成；其余位用于过量程、通信错误、自检错误等。IMU 坐标系外参和协方差在上位机 YAML 中标定，STM32 不应伪造协方差。上位机用 `0x103/0x18B` 对齐 STM32 单调时钟，并在 `boot_id` 改变时立即使旧时间偏移失效。

## 5. SDK 语义接口

电控组无需为行为树的每个节点各写一个硬件接口。只实现以下通用入口即可：

```c
bool wr_can_sdk_init(const WrCanSdkConfig *config);
void wr_can_sdk_on_rx_frame(const WrCanFrame *frame, uint64_t receive_time_us);
void wr_can_sdk_tick(uint64_t now_us);
bool wr_can_sdk_pop_tx_frame(WrCanFrame *frame);

void wr_chassis_apply_velocity(const WrChassisCommand *command);
void wr_chassis_apply_control(const WrControlCommand *command);
void wr_chassis_apply_steering_task(const WrSteeringTaskCommand *command);
void wr_chassis_force_safe_stop(void);
bool wr_chassis_read_main_imu(WrMainImuSample *sample);
```

对应文件：`src/wr_chassis_gateway/include/wr_chassis_gateway/stm32_can_sdk_contract.h`。该文件是接口契约，不是可直接烧录的 STM32 驱动；电控组需结合 HAL FDCAN、RTOS/裸机调度和现有 chassis_service 实现。

### 上位机 ROS 2 映射

| ROS 2 接口 | CAN/用途 |
| --- | --- |
| `/cmd_vel_chassis` | 编码为 `0x100` |
| `chassis_control_command` | 编码为 `0x102` |
| `rail_clamp_request` | 进离轨状态机向轨道控制器发布，不直接跨 CAN |
| `rail_clamp_command` | 轨道控制器经安全处理后发布，编码为 `0x101` |
| `chassis_state` | 解码 `0x180 + 0x181~0x184` |
| `imu/data_raw`、`imu/status` | 解码 `0x185/0x186` |
| `steering_task_state` | 解码 `0x187` |
| `chassis_command_ack` | 解码 `0x188` |
| `ExecuteRailTransition.action` | 行为树到局部状态机，不直接跨 CAN |
| `FollowPerson.action` | 行为树到人物跟随局部状态机，不直接跨 CAN |
| `SetMotionSource.srv` | 上位机内部选择唯一速度源，不直接跨 CAN |

注意：`0x181~0x184` 的原始模块状态由上位机轨道控制器生成
`rail_clamp_feedback`，不要求 STM32 额外伪造“已上轨”布尔真值。

## 6. 行为树与局部状态机约定

行为树通过 `FollowPerson.action` 运行长期跟随任务，并通过 `SetMotionSource` 在人物跟随、自由路面绕障和进退轨控制之间移交唯一运动权。通过 `ExecuteRailTransition.action` 发起 `ENTER` 或 `EXIT_FORWARD`；`EXIT_REVERSE` 仅作为兼容接口保留，当前无后向传感器，不允许自动调用。局部状态机反馈 `ALIGN → APPROACH → FRONT_PROBE → FRONT_RELEASE → REAR_ADVANCE → FINAL_PROBE → CONFIRMED` 等阶段。取消 action 时，局部状态机必须先发 `CANCEL`/零速，再向行为树报告取消完成。

固定前端进轨流程：

1. MID-360S 点云和前相机估计入口位姿，上位机完成对准。
2. 前轮进入候选轨位后，对 `0x03` 发缓慢 `PROBE`。
3. 上位机从 `0x181/0x182` 的角度、速度、转矩判断各轮接触；某轮确认后立即对该轮发 `HOLD`。
4. 前轮均确认后释放/回零，再低速推进后轮上轨。
5. 对 `0x0F` 终验；四轮独立确认后切换 `ON_RAIL`。
6. 任一阶段超时、反馈失联、硬过矩、过温或电机故障：STM32先安全停，上位机状态机失败退出，行为树决定重试/跳过。

## 7. 模式与安全要求

- `FREE_SURFACE` 使用轮胎有效半径；`ON_RAIL` 使用内侧圆柱有效半径。
- `ON_RAIL` 时 STM32 必须拒绝或清零 `vy`、`wz`；`RAIL_ENTRY` 允许受限对准和前进。
- `ON_RAIL` 的 `vx` 为有符号值；首版允许上位机发送受限低速负 `vx` 维持人物间距，
  但不允许负 `vx` 自动执行退轨。负速度上限仍由上位机和 STM32 双重限制。
- 上电默认不使能；遥控/自主控制源必须互斥，遥控优先级和接管逻辑由电控固化。
- 控制优先级为：物理急停/硬保护 > 遥控器 > 上位机自主控制。遥控接管时
  STM32 必须把 `active_control_source` 切换为 `MANUAL` 并拒绝自主速度。
  遥控释放后先保持零速，只有重新收到自主控制申请和新的有效速度命令才恢复运动。
  上位机内部还要求新的 `/autonomy_enable=true`；该信号不新增 CAN 帧，复用
  `0x102` 的自主控制申请及 `0x180.active_control_source` 确认完成双侧重授权。
- `0x100` 超过该帧 `valid_for_ms` 未更新（当前 200 ms），STM32 必须将速度清零并置 `command_timed_out=1`。
- 物理急停必须直连下位机安全链路；CAN `quick_stop` 只是软件快速停车，不是安全认证急停。
- 电流、转矩、温度、堵转和转向机械限位的硬保护全部在 STM32/驱动器闭环执行，不得依赖 ROS 及时响应。
- 帧序号用于丢帧/延迟检查；任务序号用于一次夹轨任务的幂等性，两者不能混用。
- `session_id` 或 STM32 `boot_id` 变化后，双方必须丢弃旧命令、旧任务和旧时间同步结果。
- 当前上位机对 `chassis_fault_bits` 采用“任一位非零即停车”策略。
  细分故障位可按现有电机驱动器定义，但必须在联调表中固定后才能变更。

## 8. 电控组交付清单

1. 一路与电机总线隔离的上位机 CAN-FD 物理通道及接线定义。
2. V1 编解码、帧序号、任务序号、会话号、确认/拒绝和时间同步。
3. 50 Hz 速度接收与 200 ms 看门狗；模式和控制源仲裁。
4. 四模块各自的转角、转速、转矩/电流、在线、零点、温度和故障反馈。
5. `PROBE/CANCEL/RELEASE/HOLD` 通用转向任务执行器及硬电流/转矩保护。
6. 底盘主 IMU 100~200 Hz 原始数据和真实采样时间戳。
7. 整车状态、物理急停、使能、电池、故障和命令确认。
8. 与上位机联合冻结故障位、结果码、转矩换算、机械零点和标定阈值。

尚未具备实车数据时，转矩接触阈值、硬过矩阈值、温度阈值和有效轮径不得写死为“最终值”；应放在电控参数区，并在低功率、架空和实轨试验中逐级标定。

## 9. 对接时序

### 9.1 上电和自动控制

1. STM32 上电后生成新 `boot_id`，默认零速度，控制源为 `NONE`。
2. STM32 周期发布 `0x180~0x185`；上位机通过 `0x103/0x18B` 建立时间映射。
3. 上位机发 `0x102`，请求 `AUTONOMOUS` 和目标底盘模式。
4. STM32 立即回 `0x188`，并在 `0x180` 中报告实际控制源/模式。
5. 只有 `0x180.active_control_source=AUTONOMOUS` 且无故障时，STM32 才执行 `0x100`。

### 9.2 遥控接管和释放

1. 检测到有效遥控操作时，STM32 直接使遥控速度生效，并报告 `MANUAL`。
2. `MANUAL` 期间忽略自主 `0x100`，但仍可接收和记录 CAN 帧。
3. 遥控释放后进入 `NONE` 并保持零速度，不得自动继续上次自主速度。
4. 重新收到有效 `0x102 AUTONOMOUS` 并在 `0x180` 确认后，才允许新 `0x100` 生效。

### 9.3 夹轨任务

1. 收到 `0x101 PROBE` 后检查控制源、模式、掩码、角度、速度和超时，回 `0x188`。
2. 接受后每个选中模块独立低速加角，持续发 `0x181~0x184` 和 `0x187`。
3. 任一模块达到 STM32 硬电流/转矩上限时立即停该模块，不等待上位机。
4. 收到该模块 `HOLD` 后保持当前角度；`RELEASE` 安全回零；`CANCEL` 立即停止任务。
5. 超时、过流、过转矩、过温、编码器/零点异常或电机离线时，进入 `SAFETY_STOPPED/FAULT`。

## 10. 联调验收顺序

1. **不上电机**：用 `candump/cansend` 验证 ID、长度、小端序、帧序号和 ACK。
2. **架空低功率**：验证四模块编号、转向/驱动正方向、零点、电流/转矩/温度单位。
3. **遥控优先级**：自主运动中接管，确认上位机指令失效；释放后确认不自动恢复。
4. **通信看门狗**：中断 `0x100` 超过 200 ms，确认底盘零速并报超时。
5. **主 IMU**：检查静止重力方向、转动角速度符号、采样时间戳、`boot_id` 重启处理。
6. **单轮 PROBE**：限低电流，验证接触即停、`HOLD/RELEASE/CANCEL` 和硬保护。
7. **前轮与四轮夹轨**：验证 `0x03` 和 `0x0F` 掩码、独立停轮、任务序号和终态重发。
8. **整车低速**：分别验证自由路面、进轨、轨内正/负 `vx` 和向前离轨。

首次联调完成条件：上位机可稳定收到 `chassis_state`、`swerve_state`、
`imu/data_raw`、`steering_task_state` 和 `chassis_command_ack`；遥控接管、指令超时和硬保护均能在 ROS 中看到可追溯状态。
