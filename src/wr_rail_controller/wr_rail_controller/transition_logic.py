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
