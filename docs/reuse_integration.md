# 三项参考能力整合说明

本次整合不把外部导航栈整体复制进主工作空间，避免重复控制器、重复行为树和底盘接口冲突。

## rosbag_sensor_trimmer

- 以 `tools/optional_tools.repos` 固定到上游提交 `ebce35c40f620dd46bc00c61132246aadbca0a3a`。
- 源码导入到被 Git 忽略的 `external/`，保持上游代码不修改。
- 使用 `tools/build_sensor_tools.sh` 独立构建，Qt、Livox和rosbag依赖不会影响主工作空间。
- `tools/record_real_vehicle.sh`记录本项目标准话题，`tools/trim_sensor_bag.sh`截取实验片段，`tools/validate_sensor_bag.sh`执行只读验证。
- 可选工具额外依赖Livox ROS 2驱动、Qt5及rosbag2插件；缺失时构建脚本会明确停止，不会改动系统环境。
- 上游许可证为 Apache-2.0。

## agt_traversability_lab

该仓库当前公开代码主要提供实验结构和点云可视化，且未提供许可证文件，因此没有复制其源码。本项目按照相同的模块边界独立实现：

- `wr_lidar_safety`继续负责硬停车走廊和轨内/自由路面差异；
- 新增 `/local_obstacle_grid`，发布机器人附近的滚动障碍证据；
- 新增 `/local_traversability`，发布前、左、右候选走廊的通行裕度；
- 局部栅格不替代全局地图，也不直接取得底盘控制权。

## agt_navigation_runtime

该仓库当前克隆入口需要GitHub鉴权，因此没有复制其源码。本项目依据公开的运行时分层独立补齐：

- `wr_runtime_monitor`汇总MID-360S、底盘主IMU、STM32状态、局部安全、定位和视觉新鲜度；
- `/system_health`给出故障位和 `autonomy_ready`；
- 真车CAN配置要求健康状态有效，否则自动速度源归零；
- STM32遥控仍保持最高优先级，不受上位机健康门控影响；
- `wr_bringup/real_vehicle.launch.py`统一启动真车数据链和控制链，启动时不自动使能自主运动。

## 真车联调入口

```bash
source /opt/ros/humble/setup.bash
source /tmp/at_wheel_rail_navigation/install/setup.bash

# 完整人物跟随
ros2 launch wr_bringup real_vehicle.launch.py

# 尚未安装相机时，只进行雷达、IMU、里程计和轨道感知联调
ros2 launch wr_bringup real_vehicle.launch.py require_vision:=false
```

所有自动运动仍需操作员显式发送 `/autonomy_enable=true`，并且STM32必须报告自主控制权已经生效。
