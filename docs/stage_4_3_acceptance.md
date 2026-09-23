# 阶段 4.3 验收记录：四舵轮与 ros2_control

日期：2026-09-20

## 范围

本阶段只验证自由路面四舵轮的运动学、8 个关节接口、反馈与超时停车。未接入 Gazebo 接触物理、轨道、传感器或 Nav2。

控制模块顺序固定为：`front_left, front_right, rear_right, rear_left`（FL、FR、RR、RL），与电控参考仓库保持一致。

## 可替换边界

- 上层输入：`geometry_msgs/msg/Twist`，话题 `cmd_vel`。
- 转向输出：4 路位置命令，话题 `steering_position_controller/commands`。
- 驱动输出：4 路角速度命令，话题 `wheel_velocity_controller/commands`。
- 统一反馈：`wr_interfaces/msg/SwerveState`，话题 `swerve_state`。
- 当前硬件插件：`mock_components/GenericSystem`。
- 后续替换：Gazebo 使用 `GazeboSystem`；实车使用独立硬件适配器。运动学节点及上层话题不随底层替换。

## 安全行为

- 转向机械范围按已确认的 `[-100 deg, +100 deg]` 实现。
- 等价舵角由轮速正反转解决，优先选择相对当前舵角转动量较小的解。
- 任一车轮超过角速度上限时，四轮按同一比例缩放，保持合成运动方向。
- `cmd_vel` 超过 0.25 s 未更新时，四轮速度强制归零，舵角保持当前位置。

## 验收结果

1. 三个 ros2_control 控制器均成功加载并处于 `active`：关节状态、转向位置、车轮速度。
2. 4 个转向位置接口和 4 个驱动速度接口均为 `available + claimed`。
3. 运动学单元测试通过：前进、倒车反转、横移等价解、原地旋转、等比例限速、非法参数拒绝。
4. 输入 `(vx, vy, wz) = (0.20, 0.10, 0.15)` 后，四个模块得到不同且有效的舵角/轮速，`state_valid=true`。
5. 停止输入超过 0.25 s 后，四个驱动反馈速度均为 0，`command_timed_out=true`。
6. 启动与 Ctrl-C 停止过程无节点崩溃。

## 非阻塞待标定项

- 轮胎有效半径当前暂定 0.0935 m；电控参考代码中的 0.0215 m 与网格尺寸含义不一致，实车前必须联合标定。
- 转向零点、方向符号、驱动减速比与方向符号尚未由实车数据确认。
- 模拟硬件不代表接触动力学；地面和轨道物理稳定性属于阶段 4.4。

结论：阶段 4.3 通过，允许进入阶段 4.4。
