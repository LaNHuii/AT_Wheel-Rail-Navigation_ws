"""局部障碍栅格与通行裕度计算。

该模块只使用已经转换到 ``base_link`` 的障碍点，不承担全局建图。
栅格范围、分辨率和走廊宽度在 ``config/lidar_safety.yaml`` 中调节。
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class TraversabilityConfig:
    grid_min_x: float = -0.60
    grid_max_x: float = 2.00
    grid_half_width: float = 1.30
    grid_resolution: float = 0.10
    occupied_points_per_cell: int = 2
    evaluation_min_x: float = 0.45
    evaluation_max_x: float = 1.55
    evaluation_corridor_half_width: float = 0.28
    evaluation_lateral_offset: float = 0.62


@dataclass
class TraversabilityResult:
    occupancy: np.ndarray
    forward_clearance: float
    left_clearance: float
    right_clearance: float
    preferred_direction: int
    obstacle_point_count: int


def _clearance_score(occupancy, x_centres, y_centres, x_min, x_max, y_centre, half_width):
    """以走廊内未占用栅格比例表示局部通行裕度，范围为 0～1。"""
    region = (
        (x_centres[:, None] >= x_min)
        & (x_centres[:, None] <= x_max)
        & (np.abs(y_centres[None, :] - y_centre) <= half_width)
    )
    total = int(np.count_nonzero(region))
    if total == 0:
        return 0.0
    occupied = int(np.count_nonzero(occupancy & region))
    return float(max(0.0, min(1.0, 1.0 - occupied / total)))


def build_local_traversability(points, config=None):
    """把局部障碍点投影到二维栅格，并比较前、左、右三条候选走廊。"""
    cfg = config or TraversabilityConfig()
    if (
            cfg.grid_resolution <= 0.0
            or cfg.grid_max_x <= cfg.grid_min_x
            or cfg.grid_half_width <= 0.0
            or cfg.occupied_points_per_cell <= 0):
        raise ValueError('局部栅格参数无效')

    width = int(np.ceil((2.0 * cfg.grid_half_width) / cfg.grid_resolution))
    height = int(np.ceil((cfg.grid_max_x - cfg.grid_min_x) / cfg.grid_resolution))
    counts = np.zeros((height, width), dtype=np.uint16)
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.size:
        xyz = xyz.reshape((-1, 3))
        inside = (
            np.isfinite(xyz).all(axis=1)
            & (xyz[:, 0] >= cfg.grid_min_x)
            & (xyz[:, 0] < cfg.grid_max_x)
            & (xyz[:, 1] >= -cfg.grid_half_width)
            & (xyz[:, 1] < cfg.grid_half_width)
        )
        selected = xyz[inside]
        if selected.size:
            rows = ((selected[:, 0] - cfg.grid_min_x) / cfg.grid_resolution).astype(int)
            cols = ((selected[:, 1] + cfg.grid_half_width) / cfg.grid_resolution).astype(int)
            np.add.at(counts, (rows, cols), 1)
    occupancy = counts >= cfg.occupied_points_per_cell
    x_centres = cfg.grid_min_x + (np.arange(height) + 0.5) * cfg.grid_resolution
    y_centres = -cfg.grid_half_width + (np.arange(width) + 0.5) * cfg.grid_resolution
    corridor_args = (
        cfg.evaluation_min_x,
        cfg.evaluation_max_x,
        cfg.evaluation_corridor_half_width,
    )
    forward = _clearance_score(
        occupancy, x_centres, y_centres, corridor_args[0], corridor_args[1], 0.0,
        corridor_args[2])
    left = _clearance_score(
        occupancy, x_centres, y_centres, corridor_args[0], corridor_args[1],
        cfg.evaluation_lateral_offset, corridor_args[2])
    right = _clearance_score(
        occupancy, x_centres, y_centres, corridor_args[0], corridor_args[1],
        -cfg.evaluation_lateral_offset, corridor_args[2])
    preferred = 0
    if max(left, right) > forward:
        preferred = 1 if left >= right else -1
    return TraversabilityResult(
        occupancy=occupancy,
        forward_clearance=forward,
        left_clearance=left,
        right_clearance=right,
        preferred_direction=preferred,
        obstacle_point_count=int(np.sum(counts, dtype=np.uint64)),
    )
