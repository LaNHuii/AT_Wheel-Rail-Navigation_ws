import math


class ClampDetector:
    """Per-module contact detector with persistent, latched confirmation."""

    def __init__(
        self,
        module_count=4,
        effort_threshold=1.0,
        velocity_threshold=0.03,
        minimum_probe_angle=math.radians(0.5),
        confirmation_samples=5,
    ):
        if module_count <= 0 or confirmation_samples <= 0:
            raise ValueError('module_count and confirmation_samples must be positive')
        if effort_threshold <= 0.0 or velocity_threshold < 0.0 or minimum_probe_angle < 0.0:
            raise ValueError('clamp thresholds are invalid')
        self.module_count = module_count
        self.effort_threshold = effort_threshold
        self.velocity_threshold = velocity_threshold
        self.minimum_probe_angle = minimum_probe_angle
        self.confirmation_samples = confirmation_samples
        self.reset()

    def reset(self):
        self._counts = [0] * self.module_count
        self._contacted = [False] * self.module_count

    def update(self, angles, velocities, efforts):
        if not (len(angles) == len(velocities) == len(efforts) == self.module_count):
            raise ValueError('angles, velocities, and efforts must match module_count')
        for index, (angle, velocity, effort) in enumerate(zip(angles, velocities, efforts)):
            if self._contacted[index]:
                continue
            blocked = (
                abs(angle) >= self.minimum_probe_angle
                and abs(velocity) <= self.velocity_threshold
                and abs(effort) >= self.effort_threshold
            )
            self._counts[index] = self._counts[index] + 1 if blocked else 0
            if self._counts[index] >= self.confirmation_samples:
                self._contacted[index] = True
        return list(self._contacted)


class OvertorqueDetector:
    """Short persistence filter for the independent hard-fault threshold."""

    def __init__(self, module_count=4, effort_limit=3.0, confirmation_samples=2):
        if module_count <= 0 or confirmation_samples <= 0 or effort_limit <= 0.0:
            raise ValueError('overtorque detector parameters must be positive')
        self.module_count = module_count
        self.effort_limit = effort_limit
        self.confirmation_samples = confirmation_samples
        self.reset()

    def reset(self):
        self._counts = [0] * self.module_count

    def update(self, efforts):
        if len(efforts) != self.module_count:
            raise ValueError('efforts must match module_count')
        faults = []
        for index, effort in enumerate(efforts):
            excessive = abs(effort) >= self.effort_limit
            self._counts[index] = self._counts[index] + 1 if excessive else 0
            faults.append(self._counts[index] >= self.confirmation_samples)
        return faults
