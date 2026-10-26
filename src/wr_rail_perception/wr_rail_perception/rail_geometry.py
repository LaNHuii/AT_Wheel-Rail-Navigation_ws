"""基于双平行直线几何先验的轨道入口检测。

输入是 base_link 坐标系下的 XYZ 点。流程为 ROI 裁剪、地面高度估计、
RANSAC 直线提取、双轨间距约束和通行走廊判断。
调车参数位于 config/rail_perception.yaml；实车应重点标定 ROI、高度带、
line_distance_threshold、expected_spacing 和 minimum_line_points。
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class RailGeometryConfig:
    roi_min_x: float = 0.15
    roi_max_x: float = 4.0
    roi_half_width: float = 1.2
    roi_min_z: float = -0.45
    roi_max_z: float = 0.10
    ground_bin_size: float = 0.01
    rail_min_height: float = 0.015
    rail_max_height: float = 0.085
    line_distance_threshold: float = 0.025
    minimum_line_points: int = 35
    ransac_iterations: int = 120
    maximum_heading: float = math.radians(35.0)
    maximum_parallel_error: float = math.radians(7.0)
    expected_spacing: float = 0.60
    spacing_tolerance: float = 0.08
    minimum_visible_length: float = 0.80
    obstacle_min_height: float = 0.10
    obstacle_corridor_margin: float = 0.10
    obstacle_lookahead: float = 2.0
    minimum_obstacle_points: int = 5


@dataclass
class LineModel:
    point: np.ndarray
    direction: np.ndarray
    inlier_count: int
    length: float
    residual: float
    first_point: np.ndarray


@dataclass
class RailGeometryResult:
    detected: bool
    entry_x: float = 0.0
    entry_y: float = 0.0
    heading: float = 0.0
    spacing: float = 0.0
    visible_length: float = 0.0
    confidence: float = 0.0
    blocked: bool = False
    ground_z: float = 0.0
    reason: str = ''


def _estimate_ground_z(z_values, bin_size):
    """用低高度主峰估计局部地面高度。"""
    if z_values.size < 10:
        return None
    lower = float(np.percentile(z_values, 2.0))
    upper = float(np.percentile(z_values, 80.0))
    if upper <= lower:
        return float(np.median(z_values))
    bins = max(2, int(math.ceil((upper - lower) / bin_size)))
    histogram, edges = np.histogram(z_values, bins=bins, range=(lower, upper))
    index = int(np.argmax(histogram))
    return float(0.5 * (edges[index] + edges[index + 1]))


def _refine_line(points):
    centre = np.mean(points, axis=0)
    covariance = np.cov((points - centre).T)
    values, vectors = np.linalg.eigh(covariance)
    direction = vectors[:, int(np.argmax(values))]
    if direction[0] < 0.0:
        direction = -direction
    projection = (points - centre) @ direction
    first = centre + float(np.min(projection)) * direction
    residual = np.abs(
        (points[:, 0] - centre[0]) * direction[1]
        - (points[:, 1] - centre[1]) * direction[0])
    return LineModel(
        point=centre,
        direction=direction,
        inlier_count=int(points.shape[0]),
        length=float(np.ptp(projection)),
        residual=float(np.mean(residual)),
        first_point=first,
    )


def _extract_lines(points, config):
    """使用固定随机种子的 RANSAC 逐条提取候选轨道线。"""
    remaining = np.asarray(points, dtype=np.float64)
    lines = []
    rng = np.random.default_rng(7)
    for _ in range(6):
        if remaining.shape[0] < config.minimum_line_points:
            break
        best_mask = None
        best_score = (-1, -1.0)
        for _ in range(config.ransac_iterations):
            indices = rng.choice(remaining.shape[0], size=2, replace=False)
            delta = remaining[indices[1]] - remaining[indices[0]]
            norm = float(np.linalg.norm(delta))
            if norm < 0.10:
                continue
            direction = delta / norm
            if direction[0] < 0.0:
                direction = -direction
            if abs(math.atan2(direction[1], direction[0])) > config.maximum_heading:
                continue
            offset = remaining - remaining[indices[0]]
            distance = np.abs(offset[:, 0] * direction[1] - offset[:, 1] * direction[0])
            mask = distance <= config.line_distance_threshold
            count = int(np.count_nonzero(mask))
            if count < config.minimum_line_points:
                continue
            projected = offset[mask] @ direction
            length = float(np.ptp(projected))
            score = (count, length)
            if score > best_score:
                best_score = score
                best_mask = mask
        if best_mask is None:
            break
        model = _refine_line(remaining[best_mask])
        if model.length >= config.minimum_visible_length:
            lines.append(model)
        remaining = remaining[~best_mask]
    return lines


def detect_rail_geometry(points, config=None):
    """检测双轨入口位姿、几何置信度与轨内障碍状态。"""
    cfg = config or RailGeometryConfig()
    # ROI 以车体前方为 x 正方向。修改雷达安装角后，应先校验 TF，
    # 再在 rail_perception.yaml 调节 roi_min/max_x、roi_half_width 和 z 范围。
    xyz = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    finite = np.isfinite(xyz).all(axis=1)
    roi = (
        (xyz[:, 0] >= cfg.roi_min_x) & (xyz[:, 0] <= cfg.roi_max_x)
        & (np.abs(xyz[:, 1]) <= cfg.roi_half_width)
        & (xyz[:, 2] >= cfg.roi_min_z) & (xyz[:, 2] <= cfg.roi_max_z)
    )
    xyz = xyz[finite & roi]
    if xyz.shape[0] < 2 * cfg.minimum_line_points:
        return RailGeometryResult(False, reason='insufficient ROI points')
    ground_z = _estimate_ground_z(xyz[:, 2], cfg.ground_bin_size)
    if ground_z is None:
        return RailGeometryResult(False, reason='ground height unavailable')
    height = xyz[:, 2] - ground_z
    candidates = xyz[
        (height >= cfg.rail_min_height) & (height <= cfg.rail_max_height), :2]
    if candidates.shape[0] < 2 * cfg.minimum_line_points:
        return RailGeometryResult(False, ground_z=ground_z, reason='insufficient rail-height points')
    # RANSAC 只生成候选线，后续还必须通过平行度、间距和可见长度约束。
    lines = _extract_lines(candidates, cfg)
    best = None
    for left_index in range(len(lines)):
        for right_index in range(left_index + 1, len(lines)):
            first = lines[left_index]
            second = lines[right_index]
            dot = float(np.clip(np.dot(first.direction, second.direction), -1.0, 1.0))
            parallel_error = math.acos(abs(dot))
            if parallel_error > cfg.maximum_parallel_error:
                continue
            second_direction = second.direction if dot >= 0.0 else -second.direction
            direction = first.direction + second_direction
            direction /= np.linalg.norm(direction)
            normal = np.array([-direction[1], direction[0]])
            spacing = abs(float(np.dot(second.point - first.point, normal)))
            spacing_error = abs(spacing - cfg.expected_spacing)
            if spacing_error > cfg.spacing_tolerance:
                continue
            visible = min(first.length, second.length)
            residual = first.residual + second.residual
            score = (
                visible
                - 2.0 * spacing_error
                - 2.0 * parallel_error
                - residual
            )
            if best is None or score > best[0]:
                best = (score, first, second, direction, spacing, visible, parallel_error)
    if best is None:
        return RailGeometryResult(False, ground_z=ground_z, reason='no valid parallel rail pair')
    _, first, second, direction, spacing, visible, parallel_error = best
    entry = 0.5 * (first.first_point + second.first_point)
    heading = math.atan2(float(direction[1]), float(direction[0]))
    spacing_score = 1.0 - abs(spacing - cfg.expected_spacing) / cfg.spacing_tolerance
    parallel_score = 1.0 - parallel_error / cfg.maximum_parallel_error
    length_score = min(1.0, visible / 2.0)
    confidence = float(np.clip(
        0.45 * spacing_score + 0.30 * parallel_score + 0.25 * length_score,
        0.0, 1.0))
    along = (xyz[:, :2] - entry) @ direction
    across = np.abs((xyz[:, :2] - entry) @ np.array([-direction[1], direction[0]]))
    obstacle_height = xyz[:, 2] - ground_z
    # 通行性判断使用双轨中心走廊。obstacle_lookahead、高度和点数阈值
    # 在 rail_perception.yaml 调节；阈值偏小更保守，但更容易误停。
    obstacle = (
        (along >= 0.0) & (along <= cfg.obstacle_lookahead)
        & (across <= 0.5 * spacing + cfg.obstacle_corridor_margin)
        & (obstacle_height >= cfg.obstacle_min_height)
    )
    blocked = int(np.count_nonzero(obstacle)) >= cfg.minimum_obstacle_points
    return RailGeometryResult(
        True,
        entry_x=float(entry[0]),
        entry_y=float(entry[1]),
        heading=heading,
        spacing=spacing,
        visible_length=visible,
        confidence=confidence,
        blocked=blocked,
        ground_z=ground_z,
        reason='parallel rail pair detected',
    )
