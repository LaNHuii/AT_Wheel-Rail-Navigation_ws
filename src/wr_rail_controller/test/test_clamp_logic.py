import math

import pytest

from wr_rail_controller.clamp_logic import ClampDetector, OvertorqueDetector


def test_requires_sustained_effort_and_low_velocity():
    detector = ClampDetector(confirmation_samples=3)
    angles = [math.radians(4.0)] * 4
    assert detector.update(angles, [0.2] * 4, [3.0] * 4) == [False] * 4
    assert detector.update(angles, [0.0] * 4, [3.0] * 4) == [False] * 4
    assert detector.update(angles, [0.0] * 4, [3.0] * 4) == [False] * 4
    assert detector.update(angles, [0.0] * 4, [3.0] * 4) == [True] * 4


def test_rejects_effort_before_minimum_probe_angle():
    detector = ClampDetector(
        minimum_probe_angle=math.radians(2.0), confirmation_samples=1)
    angles = [math.radians(1.0)] * 4
    assert detector.update(angles, [0.0] * 4, [10.0] * 4) == [False] * 4


def test_tracks_each_module_independently():
    detector = ClampDetector(confirmation_samples=1)
    angles = [math.radians(4.0)] * 4
    assert detector.update(angles, [0.0] * 4, [2.0, 0.0, 2.0, 0.0]) == [True, False, True, False]
    # A confirmed module remains stopped even if holding torque later falls.
    assert detector.update(angles, [0.0] * 4, [0.0, 0.0, 0.0, 0.0]) == [True, False, True, False]


def test_overtorque_is_independent_and_persistent():
    detector = OvertorqueDetector(effort_limit=5.0, confirmation_samples=2)
    assert detector.update([6.0, 0.0, 0.0, 0.0]) == [False] * 4
    assert detector.update([6.0, 0.0, -7.0, 0.0]) == [True, False, False, False]
    assert detector.update([0.0, 0.0, -7.0, 0.0]) == [False, False, True, False]


def test_validates_lengths():
    detector = ClampDetector()
    with pytest.raises(ValueError):
        detector.update([0.0], [0.0], [0.0])
    with pytest.raises(ValueError):
        OvertorqueDetector().update([0.0])
