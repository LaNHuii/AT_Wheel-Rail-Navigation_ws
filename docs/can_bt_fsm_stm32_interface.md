# 轮轨导航上位机—STM32 CAN-FD 接口说明

版本：V1.1

日期：2026-09-24

适用范围：人物跟随行为树、进退轨局部状态机、自由路面/轨内控制与 STM32 底盘之间的接口。

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
- 上位机根据四轮转矩/角度反馈判断是否接触轨道；STM32只执行有边界的探测、保持、释放并实施硬保护。
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
| `0x102` | 上位机→STM32 | 使能、急停请求、清故障、控制源、底盘模式 | 事件触发，需确认；建议 10 Hz 保活 |
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

完整字节布局以代码中的 `can_protocol.hpp/.cpp` 和单元测试为唯一实现基准；STM32 侧可直接依据 `stm32_can_sdk_contract.h` 实现同构编解码。

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
| `rail_clamp_command` | 编码为 `0x101` |
| `chassis_state` | 解码 `0x180 + 0x181~0x184` |
| `imu/data_raw`、`imu/status` | 解码 `0x185/0x186` |
| `steering_task_state` | 解码 `0x187` |
| `chassis_command_ack` | 解码 `0x188` |
| `ExecuteRailTransition.action` | 行为树到局部状态机，不直接跨 CAN |
| `FollowPerson.action` | 行为树到人物跟随局部状态机，不直接跨 CAN |
| `SetMotionSource.srv` | 上位机内部选择唯一速度源，不直接跨 CAN |

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
- 上电默认不使能；遥控/自主控制源必须互斥，遥控优先级和接管逻辑由电控固化。
- `0x100` 超过 200 ms 未更新，STM32必须将速度清零并锁定为通信超时状态。
- 物理急停必须直连下位机安全链路；CAN `quick_stop` 只是软件快速停车，不是安全认证急停。
- 电流、转矩、温度、堵转和转向机械限位的硬保护全部在 STM32/驱动器闭环执行，不得依赖 ROS 及时响应。
- 帧序号用于丢帧/延迟检查；任务序号用于一次夹轨任务的幂等性，两者不能混用。
- `session_id` 或 STM32 `boot_id` 变化后，双方必须丢弃旧命令、旧任务和旧时间同步结果。

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
