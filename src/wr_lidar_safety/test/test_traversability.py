"""局部通行性栅格的纯算法测试。"""

import numpy as np

from wr_lidar_safety.traversability import TraversabilityConfig, build_local_traversability


def test_empty_grid_reports_clear_corridors():
    result = build_local_traversability(np.empty((0, 3)))
    assert result.forward_clearance == 1.0
    assert result.left_clearance == 1.0
    assert result.right_clearance == 1.0
    assert result.preferred_direction == 0


def test_forward_obstacle_prefers_clearer_side():
    config = TraversabilityConfig(occupied_points_per_cell=1)
    forward_obstacle = np.array([
        [0.8, -0.1, 0.0],
        [0.9, 0.0, 0.0],
        [1.0, 0.1, 0.0],
        [0.9, -0.6, 0.0],
    ])
    result = build_local_traversability(forward_obstacle, config)
    assert result.forward_clearance < result.left_clearance
    assert result.left_clearance > result.right_clearance
    assert result.preferred_direction == 1


def test_invalid_grid_configuration_is_rejected():
    try:
        build_local_traversability([], TraversabilityConfig(grid_resolution=0.0))
    except ValueError:
        return
    raise AssertionError('无效分辨率应当被拒绝')
