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
    minimum_points: int = 4


@dataclass
class SafetyResult:
    emergency_stop: bool
    forward_path_blocked: bool
    rotation_safe: bool
    nearest_obstacle_distance: float


def evaluate_safety(points, mode, config=None, target_xy=None):
    """Evaluate base_link XYZ points without assuming a particular LiDAR driver."""
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
    return SafetyResult(emergency, blocked, not rotation_unsafe, nearest)
