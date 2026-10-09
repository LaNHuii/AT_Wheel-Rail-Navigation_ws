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
    max_rail_vx: float = 0.12
    max_wz: float = 0.40
    distance_gain: float = 0.8
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
    """Deterministic local FSM; perception, path planning and motor loops stay external."""

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
                # Never jump to a closer bystander while the current binding is healthy.
                return False
        if self.lost_since is not None and (
                not math.isfinite(data.nearest_body_distance)
                or data.nearest_body_distance > self.config.search_radius):
            return False
        return True

    def _follow(self, data):
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
        if on_rail:
            return FollowOutput(
                FOLLOWING, vx=vx, measured_gap=gap, target_valid=True,
                reason='rail longitudinal following')
        heading = math.atan2(data.lateral_offset, max(data.position_x, 0.05))
        wz = self._clamp(
            self.config.heading_gain * heading, -self.config.max_wz, self.config.max_wz)
        if abs(heading) > self.config.max_heading_for_translation:
            vx = 0.0
        return FollowOutput(
            FOLLOWING, vx=vx, wz=wz, measured_gap=gap, target_valid=True,
            reason='free-surface following')

    def update(self, data):
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
            if data.emergency_stop or data.forward_path_blocked or data.bypass_active:
                return FollowOutput(
                    OBSTACLE_HOLD, measured_gap=data.nearest_body_distance,
                    target_valid=True, reason='obstacle layer owns stop or bypass')
            return self._follow(data)

        if data.emergency_stop or data.forward_path_blocked:
            return FollowOutput(OBSTACLE_HOLD, reason='obstacle blocks motion')

        if data.target_state != TARGET_TRACKED:
            self.loss_observed = True
        if self.lost_since is None:
            self.lost_since = data.now
        if data.chassis_mode == ON_RAIL:
            return FollowOutput(
                WAITING_FOR_TARGET,
                reason='target lost on rail; no rotation, lateral motion, or retreat')

        if data.now - self.lost_since < self.config.lost_wait_timeout:
            return FollowOutput(LOST_WAIT, reason='waiting before free-surface search')
        if not data.rotation_safe:
            return FollowOutput(
                WAITING_FOR_TARGET, reason='full-footprint rotation is not verified safe')
        if self.search_started_at is None:
            self.search_started_at = data.now
        search_duration = 2.0 * math.pi / self.config.search_angular_speed
        if data.now - self.search_started_at < search_duration:
            return FollowOutput(
                SEARCHING, wz=self.config.search_angular_speed,
                reason='one bounded free-surface search turn')
        return FollowOutput(
            WAITING_FOR_TARGET, reason='search turn complete; waiting for a valid binding')
