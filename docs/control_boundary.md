# 上位机、仿真与下位机控制边界

## 稳定逻辑接口

所有导航控制源先输出 `geometry_msgs/msg/Twist`：

- `linear.x`：车体前向速度 `vx`，m/s；
- `linear.y`：车体左向速度 `vy`，m/s；
- `angular.z`：车体逆时针角速度 `wz`，rad/s。

各模块分别发布到 `/cmd_vel_follow`、`/cmd_vel_navigation`、`/cmd_vel_transition` 或 `/cmd_vel_manual`。行为树通过 `SetMotionSource` 选择唯一来源，`cmd_vel_arbiter` 输出 `/cmd_vel_selected`；随后 `cmd_vel_guard` 完成有限值检查、同比例限速和 200 ms 超时清零，输出 `/cmd_vel_chassis`。

## 后端分支

```text
人物跟随 / Nav2 / 进退轨 / 遥控
          │
  cmd_vel_arbiter（唯一运动权）
          │ /cmd_vel_selected
          │
  cmd_vel_guard（上位机安全层）
          │
  /cmd_vel_chassis: vx, vy, wz
          │
     ┌────┴────┐
     │         │
 Gazebo后端   实车后端
     │         │
 舵轮运动学   通信封装
     │         │
 8个仿真关节  SocketCAN（专用CAN-FD）
               │
          下位机舵轮运动学
               │
        FDCAN1/FDCAN2电机总线
```

## 电控仓库只读核对结果

当前 STM32 工程已经实现：

- 四舵轮逆运动学与正运动学；
- 最近等效舵角选择；
- 驱动限速和每 10 ms 斜坡；
- 四个驱动电机和四个转向电机控制；
- 遥控、反馈超时、发送失败和锁存故障停车。

当前物理接口为：

- FDCAN1 500 kbit/s：四个驱动电机；
- FDCAN2 1 Mbit/s：四个 RS06 转向电机；
- UART5：FS-iA10B 遥控器输入；
- USART1：日志输出。

控制优先级固定为：物理急停/下位机硬保护高于遥控器，遥控器高于上位机自主控制。
STM32 是控制源仲裁的最终执行者。其状态切换为 `MANUAL` 或 `ESTOP` 时，
上位机立即撤销当前自主运动源并清零自主速度。遥控释放后不得恢复旧命令；
必须由 STM32 重新确认 `AUTONOMOUS`，再由上位机重新选择运动源并发送新命令。
上位机行为树还必须收到新的 `/autonomy_enable=true` 才会重新启动人物跟随，
从而避免遥控释放后续跑接管前的 Action 或速度。

电控仓库目前没有工控机命令入口，也没有定义工控机串口、工控机 CAN 或以太网协议。因此导航仓库不假装任何协议已经可用。

## 后端规则

### Gazebo

启用 `wr_swerve_controller`，因为仿真必须得到四个舵角和四个轮速。CAD 关节方向差异由 `wr_gazebo/gazebo_command_adapter.py` 处理。

### 实车

不启动 `wr_swerve_controller`，只把 `/cmd_vel_chassis` 交给实车通信适配器。STM32负责舵轮运动学、电机方向、转向零点和电机闭环。

`real_chassis_adapter_node` 支持 `dry_run` 和 Linux SocketCAN 两种模式。实车配置通过专用 CAN-FD 下发速度、控制模式和夹轨任务，并接收整车、四模块、主 IMU、任务结果和确认帧。默认配置仍为 `dry_run`，必须完成电控联调和安全阈值标定后才能切换实车。

## 夹轨语义接口

夹轨不复用普通速度命令，而通过可替换的语义接口交给底盘后端：

- `wr_interfaces/msg/RailClampCommand`：以一个 `module_mask` 通用字段选择前轮组或四轮，复用开始探测、取消、释放、最大角度、速度和超时；
- `wr_interfaces/msg/RailClampFeedback`：回传当前操作掩码、四轮独立接触锁存、角度/速度/转矩和失败原因；
- `wr_interfaces/srv/SetRailMode`：只有四轮均确认接触后才允许进入轨内模式。

进轨顺序固定为：前端对准 → 前轮组探测确认 → 前轮组释放回零 → 后轮组前进上轨 → 四轮终验。前轮组成功只发布 `FRONT_VERIFIED`，不能提前进入轨内模式；只有四轮终验成功才进入 `ON_RAIL`。

仿真后端用显式虚拟接触事件验证状态机，因为 Gazebo 的位置控制器不会提供可信的转向堵转力矩。实车后端必须关闭 `simulate_contact`，使用电控回传的四轮独立转矩或等效电流；接触阈值、硬过转矩阈值和电机驱动器保护值均须由电控标定。单轮接触后，上位机保持该轮当时角度，不再继续增加夹角。

## 协议选型边界

- 工控机—STM32：固定为专用 CAN-FD，Linux 侧使用 SocketCAN `can0`。不得与现有电机 CAN 总线静默混接。
- STM32—电机：继续使用现有 FDCAN1/FDCAN2，不由导航上位机接管。
- MID-360—工控机：点云继续使用雷达原生以太网链路；内置 IMU 不再作为主 IMU。
- 底盘主 IMU—STM32—工控机：主 IMU 接入 STM32，由 STM32 采样、打下位机时间戳并通过专用 CAN-FD 发送。
- 前置 RGB-D 相机—工控机：根据最终型号选 USB 或以太网，与电机 CAN 总线无关。

完整线协议、周期和 SDK 契约见 `docs/can_bt_fsm_stm32_interface.md`。

## 双层安全

- 上位机：唯一速度源仲裁、有限值检查、速度限制、200 ms无命令清零；CAN 适配器另有独立 250 ms 上游看门狗。
- 下位机：通信超时、运动学检查、电机反馈检查、故障锁存和执行器安全停止。

夹轨时，上位机负责 0～10° 的缓慢探测、单轮接触锁存、总超时和模式编排；下位机负责电流/转矩硬限制、堵转保护、温度保护和失联急停。上位机给出的 10° 是搜索上限，不是必须到达的目标角。

上位机安全层不能替代下位机的实时安全保护。

轨内 `vx` 为有符号速度：允许受限的低速负值用于跟随人员短距离后退；STM32在
`ON_RAIL` 模式仍必须强制 `vy=0`、`wz=0`。当前没有后向感知，因此自动退轨和
持续长距离倒退不在首版验收范围内。
