"""验证自由路面与轨道模式下的障碍停车、目标排除和侧向绕行几何。"""

import numpy as np

from wr_lidar_safety.safety_geometry import FREE_SURFACE, ON_RAIL, evaluate_safety


def cluster(x, y, z=0.0, count=6):
    return np.array([[x + 0.001 * index, y, z] for index in range(count)])


def test_close_points_trigger_emergency_stop():
    result = evaluate_safety(cluster(0.55, 0.0), FREE_SURFACE)
    assert result.emergency_stop
    assert result.forward_path_blocked


def test_selected_person_is_removed_only_from_soft_blocking_zone():
    points = cluster(0.80, 0.0)
    result = evaluate_safety(points, FREE_SURFACE, target_xy=(0.80, 0.0))
    assert not result.forward_path_blocked
    assert not result.emergency_stop


def test_hard_stop_still_sees_selected_person():
    points = cluster(0.55, 0.0)
    result = evaluate_safety(points, FREE_SURFACE, target_xy=(0.55, 0.0))
    assert result.emergency_stop


def test_rail_corridor_ignores_rails_but_stops_for_centre_obstacle():
    rails = np.vstack((cluster(0.8, 0.30), cluster(0.8, -0.30)))
    assert not evaluate_safety(rails, ON_RAIL).forward_path_blocked
    obstacle = np.vstack((rails, cluster(0.8, 0.0)))
    assert evaluate_safety(obstacle, ON_RAIL).forward_path_blocked


def test_rotation_requires_clear_full_swept_radius():
    assert not evaluate_safety(cluster(0.0, 0.58), FREE_SURFACE).rotation_safe
    assert evaluate_safety(cluster(0.0, 0.80), FREE_SURFACE).rotation_safe


def test_free_surface_block_selects_clear_lateral_corridor():
    centre = cluster(0.85, 0.0)
    blocked_right = cluster(0.90, -0.62)
    result = evaluate_safety(np.vstack((centre, blocked_right)), FREE_SURFACE)
    assert result.forward_path_blocked
    assert result.bypass_direction == 1


def test_rail_obstacle_never_requests_lateral_bypass():
    result = evaluate_safety(cluster(0.85, 0.0), ON_RAIL)
    assert result.forward_path_blocked
    assert result.bypass_direction == 0
