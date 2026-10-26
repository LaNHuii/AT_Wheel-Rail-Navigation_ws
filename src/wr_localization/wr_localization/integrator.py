"""将底盘系速度积分为平面轮速里程位姿。

max_dt 用于拒绝过长的数据间隔，参数在 wheel_odometry_node.py 声明并可由
launch 或 YAML 覆盖。STM32 boot_id 变化时不对断电时间段积分。
"""

from dataclasses import dataclass
from math import cos, pi, sin


@dataclass
class PlanarPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class PlanarTwistIntegrator:
    """使用中点航向角对平面速度进行数值积分。"""
    def __init__(self, max_dt: float = 0.25) -> None:
        self.max_dt = max_dt
        self.pose = PlanarPose()
        self.last_stamp: float | None = None
        self.boot_id: int | None = None

    def update(self, stamp: float, boot_id: int, vx: float, vy: float, wz: float) -> PlanarPose:
        # 下位机重启后只更新时间基准，不将重启间隔误当成运动。
        if self.boot_id is None or boot_id != self.boot_id:
            self.boot_id = boot_id
            self.last_stamp = stamp
            return self.pose
        if self.last_stamp is None:
            self.last_stamp = stamp
            return self.pose
        dt = stamp - self.last_stamp
        self.last_stamp = stamp
        if dt <= 0.0 or dt > self.max_dt:
            return self.pose
        # 中点积分比直接使用帧起点航向角的转弯误差更小。
        midpoint_yaw = self.pose.yaw + 0.5 * wz * dt
        self.pose.x += (vx * cos(midpoint_yaw) - vy * sin(midpoint_yaw)) * dt
        self.pose.y += (vx * sin(midpoint_yaw) + vy * cos(midpoint_yaw)) * dt
        self.pose.yaw = (self.pose.yaw + wz * dt + pi) % (2.0 * pi) - pi
        return self.pose
