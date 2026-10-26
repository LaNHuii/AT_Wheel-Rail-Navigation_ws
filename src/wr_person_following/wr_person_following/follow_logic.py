"""人物跟随局部状态机。

该文件只根据已融合的人员目标、底盘模式和安全状态生成底盘速度，
不负责视觉识别、雷达障碍物提取或电机闭环。
调车参数集中在 config/person_following.yaml，单位与影响见该配置文件。
"""

from dataclasses import dataclass
import math


FREE_SURFACE = 0
RAIL_ENTRY = 1
ON_RAIL = 2
RAIL_EXIT = 3

TARGET_NONE = 0
TARGET_TRACKED = 1
TARGET_TEMPORARILY_LOST = 2
TARGET_IDENTITY_UNCERTAIN = 3
TARGET_SENSOR_FAULT = 4

ACTIVITY_UNKNOWN = 0
ACTIVITY_MOVING = 1
ACTIVITY_STATIONARY = 2
ACTIVITY_DEPOSITING = 3

IDLE = 0
FOLLOWING = 1
TARGET_STATIONARY = 2
LOST_WAIT = 3
SEARCHING = 4
WAITING_FOR_TARGET = 5
OBSTACLE_HOLD = 6
TRANSITION_HOLD = 7
SENSOR_HOLD = 8
HARDWARE_HOLD = 9


@dataclass
class FollowConfig:
    desired_gap: float = 0.30
    gap_deadband: float = 0.05
    lost_wait_timeout: float = 30.0
    search_radius: float = 3.0
    search_angular_speed: float = 0.25
    max_free_vx: float = 0.20
    max_free_vy: float = 0.08
    max_rail_vx: float = 0.12
    max_rail_reverse_vx: float = 0.05
    max_wz: float = 0.40
    distance_gain: float = 0.8
    lateral_gain: float = 0.6
    heading_gain: float = 1.2
    max_heading_for_translation: float = math.radians(25.0)
    allow_target_rebind: bool = True
    requested_session_id: int = 0


@dataclass
class FollowInputs:
    now: float
    chassis_mode: int = FREE_SURFACE
    chassis_ok: bool = False
    safety_fresh: bool = False
    emergency_stop: bool = False
    forward_path_blocked: bool = False
    rotation_safe: bool = False
    bypass_active: bool = False
    suggested_lateral_velocity: float = 0.0
    target_state: int = TARGET_NONE
    target_session_id: int = 0
    binding_id: int = 0
    tracker_id: int = -1
    activity: int = ACTIVITY_UNKNOWN
    position_x: float = 0.0
    lateral_offset: float = 0.0
    nearest_body_distance: float = 0.0
    longitudinal_distance: float = 0.0
    original_target: bool = True


@dataclass
class FollowOutput:
    state: int
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    measured_gap: float = 0.0
    target_valid: bool = False
    reason: str = ''


class PersonFollowLogic:
    """确定性局部状态机；感知、路径规划和电机闭环均在外部完成。"""

    def __init__(self, config=None):
        self.config = config or FollowConfig()
        self.active = False
        self.bound_binding_id = 0
        self.lost_since = None
        self.search_started_at = None
        self.loss_observed = False

    def start(self, config=None):
        if config is not None:
            self.config = config
        self.active = True
        self.bound_binding_id = 0
        self.lost_since = None
        self.search_started_at = None
        self.loss_observed = False

    def stop(self):
        self.active = False
        self.lost_since = None
        self.search_started_at = None
        self.loss_observed = False

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def _target_is_acceptable(self, data):
        if data.target_state != TARGET_TRACKED:
            return False
        if self.config.requested_session_id and (
                data.target_session_id != self.config.requested_session_id):
            return False
        if data.binding_id == 0:
            return False
        if self.bound_binding_id and data.binding_id != self.bound_binding_id:
            if not self.config.allow_target_rebind:
                return False
            if not self.loss_observed:
                # 当前绑定仍有效时，不允许因为旁人更近而切换目标。
                return False
        return True

    def _follow(self, data):
        # 跟随距离基准：轨内使用轨道纵向间距，自由路面使用车头到人体最近表面距离。
        # 可调：desired_front_gap、gap_deadband、distance_gain，
        # 位置：config/person_following.yaml。
        on_rail = data.chassis_mode == ON_RAIL
        gap = data.longitudinal_distance if on_rail else data.nearest_body_distance
        if not math.isfinite(gap) or gap <= 0.0:
            return FollowOutput(SENSOR_HOLD, measured_gap=gap, reason='invalid target range')
        if data.activity in (ACTIVITY_STATIONARY, ACTIVITY_DEPOSITING):
            return FollowOutput(
                TARGET_STATIONARY, measured_gap=gap, target_valid=True,
                reason='target stationary or depositing')
        error = gap - self.config.desired_gap
        vx = 0.0
        if error > self.config.gap_deadband:
            limit = self.config.max_rail_vx if on_rail else self.config.max_free_vx
            vx = self._clamp(self.config.distance_gain * error, 0.0, limit)
        elif on_rail and error < -self.config.gap_deadband:
            vx = self._clamp(
                self.config.distance_gain * error,
                -self.config.max_rail_reverse_vx,
                0.0,
            )
        if on_rail:
            # 轨内只输出纵向速度；max_rail_reverse_vx 限制跟人倒车速度。
            return FollowOutput(
                FOLLOWING, vx=vx, measured_gap=gap, target_valid=True,
                reason='rail bidirectional longitudinal following')
        # 自由路面可横移和转向，但人向车辆后退时 vx 保持为零。
        # 可调：max_free_vy、max_wz、lateral_gain、heading_gain、
        # max_heading_for_translation，位置：config/person_following.yaml。
        heading = math.atan2(data.lateral_offset, max(data.position_x, 0.05))
        vy = self._clamp(
            self.config.lateral_gain * data.lateral_offset,
            -self.config.max_free_vy,
            self.config.max_free_vy,
        )
        wz = self._clamp(
            self.config.heading_gain * heading, -self.config.max_wz, self.config.max_wz)
        if abs(heading) > self.config.max_heading_for_translation:
            vx = 0.0
        return FollowOutput(
            FOLLOWING, vx=vx, vy=vy, wz=wz, measured_gap=gap, target_valid=True,
            reason='free-surface following')

    def update(self, data):
        # 安全优先级高于跟随：底盘、感知或障碍物条件不满足时只输出停车状态。
        if not self.active:
            return FollowOutput(IDLE, reason='follow action inactive')
        if not data.chassis_ok:
            return FollowOutput(HARDWARE_HOLD, reason='chassis state stale or unsafe')
        if not data.safety_fresh or data.target_state == TARGET_SENSOR_FAULT:
            if data.target_state == TARGET_SENSOR_FAULT:
                self.loss_observed = True
            return FollowOutput(SENSOR_HOLD, reason='perception safety evidence unavailable')
        if data.chassis_mode in (RAIL_ENTRY, RAIL_EXIT):
            return FollowOutput(TRANSITION_HOLD, reason='rail transition owns chassis motion')

        if self._target_is_acceptable(data):
            self.bound_binding_id = data.binding_id
            self.lost_since = None
            self.search_started_at = None
            self.loss_observed = False
            if data.emergency_stop:
                return FollowOutput(
                    OBSTACLE_HOLD, measured_gap=data.nearest_body_distance,
                    target_valid=True, reason='hard obstacle stop')
            if data.chassis_mode == ON_RAIL and data.forward_path_blocked:
                return FollowOutput(
                    OBSTACLE_HOLD, measured_gap=data.longitudinal_distance,
                    target_valid=True, reason='rail obstacle hold')
            if data.forward_path_blocked:
                # 第一版只允许自由路面使用雷达给出的有界侧向绕行速度；
                # 轨内遇障始终停车。绕行速度上限受 max_free_vy 约束。
                if data.bypass_active and abs(data.suggested_lateral_velocity) > 1.0e-6:
                    return FollowOutput(
                        FOLLOWING,
                        vx=min(0.04, self.config.max_free_vx),
                        vy=self._clamp(
                            data.suggested_lateral_velocity,
                            -self.config.max_free_vy,
                            self.config.max_free_vy),
                        measured_gap=data.nearest_body_distance,
                        target_valid=True,
                        reason='bounded free-surface lateral bypass')
                return FollowOutput(
                    OBSTACLE_HOLD, measured_gap=data.nearest_body_distance,
                    target_valid=True, reason='no safe local bypass corridor')
            return self._follow(data)

        if data.emergency_stop or data.forward_path_blocked:
            return FollowOutput(OBSTACLE_HOLD, reason='obstacle blocks motion')

        if data.target_state != TARGET_TRACKED:
            self.loss_observed = True
        if self.lost_since is None:
            self.lost_since = data.now
        return FollowOutput(
            WAITING_FOR_TARGET,
            reason='target unavailable; first release holds position without active search')
