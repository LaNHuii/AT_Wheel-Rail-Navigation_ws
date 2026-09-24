from dataclasses import dataclass


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


@dataclass(frozen=True)
class TransitionCommand:
    vx: float
    vy: float
    wz: float
    phase: str
    done: bool = False


class EntryTransitionPolicy:
    """Phased entry using entrance-relative longitudinal/lateral/heading errors."""

    def __init__(self, alignment_samples=15):
        self.phase = 'align'
        self.alignment_samples = alignment_samples
        self._aligned_count = 0

    def update(self, longitudinal, lateral_error, heading_error, all_on_rail):
        if self.phase == 'align':
            aligned = abs(lateral_error) < 0.004 and abs(heading_error) < 0.010
            self._aligned_count = self._aligned_count + 1 if aligned else 0
            if self._aligned_count >= self.alignment_samples:
                self.phase = 'approach'
            return TransitionCommand(
                0.0,
                clamp(-2.5 * lateral_error, -0.06, 0.06),
                clamp(-3.0 * heading_error, -0.20, 0.20),
                self.phase,
            )

        if self.phase == 'approach':
            if longitudinal >= -0.50:
                self.phase = 'cross_u_end'
            return TransitionCommand(
                0.080,
                clamp(-1.2 * lateral_error, -0.012, 0.012),
                clamp(-1.5 * heading_error, -0.05, 0.05),
                self.phase,
            )

        done = all_on_rail and longitudinal >= 0.65
        return TransitionCommand(0.0 if done else 0.150, 0.0, 0.0, self.phase, done)


class ExitTransitionPolicy:
    """Reverse exit with symmetric crossing of the transverse U-end pipe."""

    def __init__(self):
        self.phase = 'rail_reverse'

    def update(self, longitudinal, lateral_error, heading_error, all_on_ground):
        if self.phase == 'rail_reverse':
            if longitudinal <= 0.48:
                self.phase = 'cross_u_end'
            return TransitionCommand(-0.080, 0.0, 0.0, self.phase)

        if self.phase == 'cross_u_end':
            if longitudinal <= -0.50:
                self.phase = 'ground_departure'
            return TransitionCommand(-0.150, 0.0, 0.0, self.phase)

        done = all_on_ground and longitudinal <= -0.65
        return TransitionCommand(
            0.0 if done else -0.060,
            0.0 if done else clamp(-2.5 * lateral_error, -0.08, 0.08),
            0.0 if done else clamp(-3.0 * heading_error, -0.25, 0.25),
            self.phase,
            done,
        )


CLAMP_IDLE = 0
CLAMP_PROBING = 1
CLAMP_PARTIAL_CONTACT = 2
CLAMP_PROBE_CONFIRMED = 3
CLAMP_FAILURE_STATES = (4, 5, 6, 7)

CLAMP_NONE = 0
CLAMP_START_PROBE = 1
CLAMP_CANCEL = 2
CLAMP_RELEASE = 3

FRONT_PAIR_MASK = 0x03
ALL_MODULES_MASK = 0x0F


@dataclass(frozen=True)
class StagedTransitionCommand:
    vx: float
    vy: float
    wz: float
    phase: str
    clamp_action: int = CLAMP_NONE
    module_mask: int = 0
    done: bool = False
    failed: bool = False


class FixedFrontStagedEntryPolicy:
    """Forward-only entry with front-pair proof before final four-module proof."""

    def __init__(
            self, alignment_samples=15, front_stop_longitudinal=0.15,
            final_stop_longitudinal=0.85):
        self.phase = 'align'
        self.alignment_samples = alignment_samples
        self.front_stop_longitudinal = front_stop_longitudinal
        self.final_stop_longitudinal = final_stop_longitudinal
        self._aligned_count = 0
        self._command_sent = False

    def _change_phase(self, phase):
        self.phase = phase
        self._command_sent = False

    @staticmethod
    def _confirmed(state, mask, contacts, expected_mask):
        return (
            state == CLAMP_PROBE_CONFIRMED
            and mask == expected_mask
            and all(contacts[index] for index in range(4) if expected_mask & (1 << index))
        )

    def update(
            self, longitudinal, lateral_error, heading_error,
            clamp_state=CLAMP_IDLE, feedback_mask=0,
            contacts=(False, False, False, False)):
        if clamp_state in CLAMP_FAILURE_STATES:
            return StagedTransitionCommand(
                0.0, 0.0, 0.0, self.phase, failed=True)

        if self.phase == 'align':
            aligned = abs(lateral_error) < 0.004 and abs(heading_error) < 0.010
            self._aligned_count = self._aligned_count + 1 if aligned else 0
            if self._aligned_count >= self.alignment_samples:
                self._change_phase('approach')
            return StagedTransitionCommand(
                0.0,
                clamp(-2.5 * lateral_error, -0.06, 0.06),
                clamp(-3.0 * heading_error, -0.20, 0.20),
                self.phase,
            )

        if self.phase == 'approach':
            if longitudinal >= self.front_stop_longitudinal:
                self._change_phase('front_probe')
                return StagedTransitionCommand(0.0, 0.0, 0.0, self.phase)
            return StagedTransitionCommand(
                0.080,
                clamp(-1.2 * lateral_error, -0.012, 0.012),
                clamp(-1.5 * heading_error, -0.05, 0.05),
                self.phase,
            )

        if self.phase == 'front_probe':
            if self._confirmed(clamp_state, feedback_mask, contacts, FRONT_PAIR_MASK):
                self._change_phase('front_release')
                return StagedTransitionCommand(0.0, 0.0, 0.0, self.phase)
            action = CLAMP_NONE if self._command_sent else CLAMP_START_PROBE
            self._command_sent = True
            return StagedTransitionCommand(
                0.0, 0.0, 0.0, self.phase, action, FRONT_PAIR_MASK)

        if self.phase == 'front_release':
            if self._command_sent and clamp_state == CLAMP_IDLE:
                self._change_phase('rear_advance')
                return StagedTransitionCommand(0.0, 0.0, 0.0, self.phase)
            action = CLAMP_NONE if self._command_sent else CLAMP_RELEASE
            self._command_sent = True
            return StagedTransitionCommand(
                0.0, 0.0, 0.0, self.phase, action, FRONT_PAIR_MASK)

        if self.phase == 'rear_advance':
            if longitudinal >= self.final_stop_longitudinal:
                self._change_phase('final_probe')
                return StagedTransitionCommand(0.0, 0.0, 0.0, self.phase)
            return StagedTransitionCommand(0.150, 0.0, 0.0, self.phase)

        if self.phase == 'final_probe':
            if self._confirmed(clamp_state, feedback_mask, contacts, ALL_MODULES_MASK):
                self._change_phase('confirmed')
                return StagedTransitionCommand(
                    0.0, 0.0, 0.0, self.phase, done=True)
            action = CLAMP_NONE if self._command_sent else CLAMP_START_PROBE
            self._command_sent = True
            return StagedTransitionCommand(
                0.0, 0.0, 0.0, self.phase, action, ALL_MODULES_MASK)

        return StagedTransitionCommand(
            0.0, 0.0, 0.0, self.phase, done=self.phase == 'confirmed')


class TurnaroundPolicy:
    """In-place 180-degree turn used before every fixed-front rail entry."""

    def __init__(self, tolerance=0.02, confirmation_samples=10):
        self.tolerance = tolerance
        self.confirmation_samples = confirmation_samples
        self._aligned_count = 0

    def update(self, yaw_error):
        aligned = abs(yaw_error) <= self.tolerance
        self._aligned_count = self._aligned_count + 1 if aligned else 0
        done = self._aligned_count >= self.confirmation_samples
        wz = 0.0 if done else clamp(1.5 * yaw_error, -0.45, 0.45)
        return TransitionCommand(0.0, 0.0, wz, 'turnaround', done)


class ForwardExitPolicy:
    """Normal forward exit; reverse ExitTransitionPolicy remains for blocked rows."""

    def __init__(self):
        self.phase = 'release'
        self._release_sent = False

    def update(self, longitudinal, all_on_ground, clamp_state):
        if self.phase == 'release':
            if self._release_sent and clamp_state == CLAMP_IDLE:
                self.phase = 'rail_forward'
                return StagedTransitionCommand(0.0, 0.0, 0.0, self.phase)
            action = CLAMP_NONE if self._release_sent else CLAMP_RELEASE
            self._release_sent = True
            return StagedTransitionCommand(
                0.0, 0.0, 0.0, self.phase, action, ALL_MODULES_MASK)
        if self.phase == 'rail_forward':
            if longitudinal >= -0.48:
                self.phase = 'cross_u_end'
            return StagedTransitionCommand(0.080, 0.0, 0.0, self.phase)
        done = all_on_ground and longitudinal >= 0.65
        return StagedTransitionCommand(
            0.0 if done else 0.150, 0.0, 0.0, self.phase, done=done)
