"""验证双平行轨道检测、入口位姿、尺寸约束与通行性判断。"""

import math

import numpy as np

from wr_rail_perception.rail_geometry import detect_rail_geometry


def scene(two_rails=True, centre_y=0.0, heading=0.0):
    rng = np.random.default_rng(4)
    ground_x = rng.uniform(0.15, 4.0, 3500)
    ground_y = rng.uniform(-1.2, 1.2, 3500)
    ground_z = rng.normal(-0.253, 0.002, 3500)
    points = [np.column_stack((ground_x, ground_y, ground_z))]
    direction = np.array([math.cos(heading), math.sin(heading)])
    normal = np.array([-direction[1], direction[0]])
    for offset in (-0.30, 0.30) if two_rails else (-0.30,):
        along = np.linspace(0.35, 3.6, 180)
        xy = along[:, None] * direction + (centre_y + offset) * normal
        xy += rng.normal(0.0, 0.004, xy.shape)
        z = rng.normal(-0.203, 0.003, along.shape[0])
        points.append(np.column_stack((xy, z)))
    return np.vstack(points)


def test_detects_parallel_rails_with_expected_spacing():
    result = detect_rail_geometry(scene())
    assert result.detected
    assert abs(result.spacing - 0.60) < 0.03
    assert abs(result.entry_y) < 0.03
    assert abs(result.heading) < math.radians(2.0)
    assert result.visible_length > 2.5
    assert result.confidence > 0.75
    assert not result.blocked


def test_estimates_offset_and_heading():
    result = detect_rail_geometry(scene(centre_y=0.12, heading=math.radians(4.0)))
    assert result.detected
    assert abs(result.entry_y - 0.12) < 0.06
    assert abs(result.heading - math.radians(4.0)) < math.radians(2.0)


def test_rejects_single_line():
    result = detect_rail_geometry(scene(two_rails=False))
    assert not result.detected


def test_marks_tall_objects_inside_track_as_blocked():
    points = scene()
    obstacle = np.column_stack((
        np.full(20, 1.5), np.linspace(-0.1, 0.1, 20), np.full(20, 0.1)))
    result = detect_rail_geometry(np.vstack((points, obstacle)))
    assert result.detected and result.blocked
