"""局部障碍物与绕行走廊的纯几何判断。

输入必须是 base_link 坐标系下的点云。几何范围、点数阈值和建议侧向速度
通过 config/lidar_safety.yaml 调节，不在本文件内硬编码实车标定值。
"""

from dataclasses import dataclass
import math

import numpy as np


FREE_SURFACE = 0
ON_RAIL = 2


@dataclass
class SafetyConfig:
    min_obstacle_z: float = -0.22
    max_obstacle_z: float = 1.50
    robot_front_x: float = 0.45
    self_min_x: float = -0.46
    self_max_x: float = 0.46
    self_half_width: float = 0.40
    self_min_z: float = -0.24
    self_max_z: float = 0.55
    free_corridor_half_width: float = 0.43
    rail_corridor_half_width: float = 0.24
    hard_stop_distance: float = 0.18
    free_block_distance: float = 0.65
    rail_block_distance: float = 1.00
    rotation_radius: float = 0.62
    target_exclusion_radius: float = 0.42
    bypass_offset: float = 0.62
    bypass_corridor_half_width: float = 0.22
    bypass_check_distance: float = 1.10
    minimum_points: int = 4


@dataclass
class SafetyResult:
    emergency_stop: bool
    forward_path_blocked: bool
    rotation_safe: bool
    nearest_obstacle_distance: float
    bypass_direction: int = 0


def evaluate_safety(points, mode, config=None, target_xy=None):
    """评估 base_link 坐标系下的 XYZ 点，不依赖特定雷达驱动。"""
    cfg = config or SafetyConfig()
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.size == 0:
        return SafetyResult(False, False, True, math.inf)
    xyz = xyz.reshape((-1, 3))
    finite = np.isfinite(xyz).all(axis=1)
    height = (xyz[:, 2] >= cfg.min_obstacle_z) & (xyz[:, 2] <= cfg.max_obstacle_z)
    inside_self = (
        (xyz[:, 0] >= cfg.self_min_x) & (xyz[:, 0] <= cfg.self_max_x)
        & (np.abs(xyz[:, 1]) <= cfg.self_half_width)
        & (xyz[:, 2] >= cfg.self_min_z) & (xyz[:, 2] <= cfg.self_max_z))
    xyz = xyz[finite & height & ~inside_self]
    if xyz.size == 0:
        return SafetyResult(False, False, True, math.inf)

    half_width = (
        cfg.rail_corridor_half_width if mode == ON_RAIL
        else cfg.free_corridor_half_width)
    clearance = xyz[:, 0] - cfg.robot_front_x
    forward = (clearance >= 0.0) & (np.abs(xyz[:, 1]) <= half_width)
    nearest = float(np.min(clearance[forward])) if np.any(forward) else math.inf
    emergency = int(np.count_nonzero(
        forward & (clearance <= cfg.hard_stop_distance))) >= cfg.minimum_points

    soft_points = np.ones(xyz.shape[0], dtype=bool)
    if target_xy is not None and all(math.isfinite(value) for value in target_xy):
        target_distance = np.hypot(xyz[:, 0] - target_xy[0], xyz[:, 1] - target_xy[1])
        soft_points &= target_distance > cfg.target_exclusion_radius
    block_distance = (
        cfg.rail_block_distance if mode == ON_RAIL else cfg.free_block_distance)
    blocked = int(np.count_nonzero(
        forward & soft_points & (clearance <= block_distance))) >= cfg.minimum_points

    swept_radius = np.hypot(xyz[:, 0], xyz[:, 1])
    rotation_unsafe = int(np.count_nonzero(
        swept_radius <= cfg.rotation_radius)) >= cfg.minimum_points
    bypass_direction = 0
    if mode == FREE_SURFACE and blocked and not emergency:
        look_ahead = (
            (clearance >= 0.0) & (clearance <= cfg.bypass_check_distance) & soft_points)
        left_count = int(np.count_nonzero(
            look_ahead &
            (np.abs(xyz[:, 1] - cfg.bypass_offset) <= cfg.bypass_corridor_half_width)))
        right_count = int(np.count_nonzero(
            look_ahead &
            (np.abs(xyz[:, 1] + cfg.bypass_offset) <= cfg.bypass_corridor_half_width)))
        if min(left_count, right_count) < cfg.minimum_points:
            bypass_direction = 1 if left_count <= right_count else -1
    return SafetyResult(emergency, blocked, not rotation_unsafe, nearest, bypass_direction)
