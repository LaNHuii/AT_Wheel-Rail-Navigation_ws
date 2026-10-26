"""自然进轨与离轨的顺序状态机。

进轨顺序固定为：对准、接近、前轮夹轨验证、释放、后轮上轨、四轮终验。
该流程在执行时独占底盘运动权，不接受跟人追赶速度。
对准容差、速度和行程位于 config/rail_controller.yaml；夹轨转矩阈值待实车标定。
"""

from dataclasses import dataclass


IDLE = 'idle'
ALIGN = 'align'
APPROACH = 'approach'
FRONT_PROBE = 'front_probe'
FRONT_RELEASE = 'front_release'
REAR_ADVANCE = 'rear_advance'
FINAL_PROBE = 'final_probe'
CONFIRMED = 'confirmed'
EXIT_RELEASE = 'exit_release'
EXIT_FORWARD = 'exit_forward'
COMPLETE = 'complete'
FAILED = 'failed'

CLAMP_IDLE = 0
CLAMP_PROBING = 1
CLAMP_CONFIRMED = 3
CLAMP_FAILURE_STATES = (4, 5, 6, 7)
CLAMP_NONE = 0
CLAMP_START_PROBE = 1
CLAMP_CANCEL = 2
CLAMP_RELEASE = 3
FRONT_PAIR_MASK = 0x03
ALL_MODULES_MASK = 0x0F


@dataclass
class RuntimeTransitionConfig:
    alignment_samples: int = 10
    lateral_tolerance: float = 0.015
    heading_tolerance: float = 0.035
    front_probe_entry_x: float = 0.20
    align_lateral_gain: float = 1.5
    align_heading_gain: float = 1.8
    max_align_vy: float = 0.05
    max_align_wz: float = 0.15
    approach_speed: float = 0.06
    rear_advance_speed: float = 0.06
    rear_advance_distance: float = 0.72
    exit_speed: float = 0.06
    exit_distance: float = 0.85


@dataclass
class RuntimeTransitionInput:
    estimate_valid: bool = False
    entry_x: float = 0.0
    lateral_error: float = 0.0
    heading_error: float = 0.0
    traversable: bool = False
    safety_blocked: bool = False
    clamp_state: int = CLAMP_IDLE
    clamp_mask: int = 0
    contacts: tuple = (False, False, False, False)
    travelled: float = 0.0


@dataclass
class RuntimeTransitionOutput:
    phase: str
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    clamp_action: int = CLAMP_NONE
    module_mask: int = 0
    done: bool = False
    failed: bool = False
    reason: str = ''


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


class RuntimeRailTransition:
    """按严格顺序推进进轨或离轨，任一夹轨故障都进入失败状态。"""
    def __init__(self, operation, config=None):
        self.config = config or RuntimeTransitionConfig()
        self.operation = operation
        self.phase = ALIGN if operation == 'enter' else EXIT_RELEASE
        self._aligned_count = 0
        self._command_sent = False

    def _change(self, phase):
        self.phase = phase
        self._command_sent = False

    @staticmethod
    def _confirmed(data, expected_mask):
        return (
            data.clamp_state == CLAMP_CONFIRMED
            and data.clamp_mask == expected_mask
            and all(data.contacts[index] for index in range(4)
                    if expected_mask & (1 << index))
        )

    def update(self, data):
        if data.clamp_state in CLAMP_FAILURE_STATES:
            self._change(FAILED)
            return RuntimeTransitionOutput(self.phase, failed=True, reason='clamp task failed')
        if data.safety_blocked:
            return RuntimeTransitionOutput(self.phase, reason='obstacle hold')
        if self.operation == 'exit':
            return self._update_exit(data)
        return self._update_entry(data)

    def _update_entry(self, data):
        # 对准必须连续多帧满足容差，避免单帧点云抖动触发接近。
        # 可调：alignment_samples、lateral_tolerance、heading_tolerance、
        # align_lateral_gain、align_heading_gain，位置：config/rail_controller.yaml。
        if self.phase in (ALIGN, APPROACH) and not data.estimate_valid:
            return RuntimeTransitionOutput(self.phase, reason='waiting for rail estimate')
        if self.phase in (ALIGN, APPROACH) and not data.traversable:
            return RuntimeTransitionOutput(self.phase, reason='rail entrance is not passable')
        if self.phase == ALIGN:
            aligned = (
                abs(data.lateral_error) <= self.config.lateral_tolerance
                and abs(data.heading_error) <= self.config.heading_tolerance)
            self._aligned_count = self._aligned_count + 1 if aligned else 0
            if self._aligned_count >= self.config.alignment_samples:
                self._change(APPROACH)
            return RuntimeTransitionOutput(
                self.phase,
                vy=_clamp(-self.config.align_lateral_gain * data.lateral_error,
                          -self.config.max_align_vy, self.config.max_align_vy),
                wz=_clamp(-self.config.align_heading_gain * data.heading_error,
                          -self.config.max_align_wz, self.config.max_align_wz))
        if self.phase == APPROACH:
            if data.entry_x <= self.config.front_probe_entry_x:
                self._change(FRONT_PROBE)
                return RuntimeTransitionOutput(self.phase)
            return RuntimeTransitionOutput(self.phase, vx=self.config.approach_speed)
        if self.phase == FRONT_PROBE:
            # 只探测前两个模块；每个模块转矩接触后应由夹轨闭环停止继续加角。
            if self._confirmed(data, FRONT_PAIR_MASK):
                self._change(FRONT_RELEASE)
                return RuntimeTransitionOutput(self.phase)
            action = CLAMP_NONE if self._command_sent else CLAMP_START_PROBE
            self._command_sent = True
            return RuntimeTransitionOutput(
                self.phase, clamp_action=action, module_mask=FRONT_PAIR_MASK)
        if self.phase == FRONT_RELEASE:
            if self._command_sent and data.clamp_state == CLAMP_IDLE:
                self._change(REAR_ADVANCE)
                return RuntimeTransitionOutput(self.phase)
            action = CLAMP_NONE if self._command_sent else CLAMP_RELEASE
            self._command_sent = True
            return RuntimeTransitionOutput(
                self.phase, clamp_action=action, module_mask=FRONT_PAIR_MASK)
        if self.phase == REAR_ADVANCE:
            # 前轮验证并释放后才允许后轮上轨。rear_advance_speed/distance
            # 在轨道控制 YAML 中调节，仿真专用短行程在 rail_controller_sim.yaml。
            if data.travelled >= self.config.rear_advance_distance:
                self._change(FINAL_PROBE)
                return RuntimeTransitionOutput(self.phase)
            return RuntimeTransitionOutput(self.phase, vx=self.config.rear_advance_speed)
        if self.phase == FINAL_PROBE:
            # 四轮终验必须同时具备完整接触位图和夹轨成功状态。
            if self._confirmed(data, ALL_MODULES_MASK):
                self._change(CONFIRMED)
                return RuntimeTransitionOutput(self.phase, done=True)
            action = CLAMP_NONE if self._command_sent else CLAMP_START_PROBE
            self._command_sent = True
            return RuntimeTransitionOutput(
                self.phase, clamp_action=action, module_mask=ALL_MODULES_MASK)
        return RuntimeTransitionOutput(
            self.phase, done=self.phase == CONFIRMED, failed=self.phase == FAILED)

    def _update_exit(self, data):
        if self.phase == EXIT_RELEASE:
            if self._command_sent and data.clamp_state == CLAMP_IDLE:
                self._change(EXIT_FORWARD)
                return RuntimeTransitionOutput(self.phase)
            action = CLAMP_NONE if self._command_sent else CLAMP_RELEASE
            self._command_sent = True
            return RuntimeTransitionOutput(
                self.phase, clamp_action=action, module_mask=ALL_MODULES_MASK)
        if self.phase == EXIT_FORWARD:
            if data.travelled >= self.config.exit_distance:
                self._change(COMPLETE)
                return RuntimeTransitionOutput(self.phase, done=True)
            return RuntimeTransitionOutput(self.phase, vx=self.config.exit_speed)
        return RuntimeTransitionOutput(
            self.phase, done=self.phase == COMPLETE, failed=self.phase == FAILED)
