"""验证舍轮运动学、转向角等价化与回零夹轨阶段逻辑。"""

from wr_rail_controller.swerve_math import calculate_swerve_commands
from wr_rail_controller.transition_logic import (
    ALL_MODULES_MASK,
    CLAMP_IDLE,
    CLAMP_PROBE_CONFIRMED,
    CLAMP_RELEASE,
    CLAMP_START_PROBE,
    FRONT_PAIR_MASK,
    EntryTransitionPolicy,
    ExitTransitionPolicy,
    FixedFrontStagedEntryPolicy,
    ForwardExitPolicy,
    TurnaroundPolicy,
)


def test_entry_policy_aligns_approaches_and_crosses_without_lateral_commands():
    policy = EntryTransitionPolicy(alignment_samples=2)
    first = policy.update(-0.8, 0.01, 0.02, False)
    assert first.phase == 'align' and first.vx == 0.0
    policy.update(-0.8, 0.0, 0.0, False)
    aligned = policy.update(-0.8, 0.0, 0.0, False)
    assert aligned.phase == 'approach'
    approach = policy.update(-0.6, 0.002, 0.003, False)
    assert approach.vx == 0.08
    crossing = policy.update(-0.49, 0.0, 0.0, False)
    assert crossing.phase == 'cross_u_end'
    crossing = policy.update(0.2, 0.05, 0.10, False)
    assert (crossing.vx, crossing.vy, crossing.wz) == (0.15, 0.0, 0.0)
    done = policy.update(0.7, 0.0, 0.0, True)
    assert done.done and done.vx == 0.0


def test_fixed_front_entry_requires_front_then_four_module_proof():
    policy = FixedFrontStagedEntryPolicy(
        alignment_samples=1, front_stop_longitudinal=0.1,
        final_stop_longitudinal=0.8)
    assert policy.update(0.0, 0.0, 0.0).phase == 'approach'
    assert policy.update(0.2, 0.0, 0.0).phase == 'front_probe'
    front_probe = policy.update(0.2, 0.0, 0.0)
    assert (front_probe.clamp_action, front_probe.module_mask) == (
        CLAMP_START_PROBE, FRONT_PAIR_MASK)
    front_ok = policy.update(
        0.2, 0.0, 0.0, CLAMP_PROBE_CONFIRMED, FRONT_PAIR_MASK,
        (True, True, False, False))
    assert front_ok.phase == 'front_release' and not front_ok.done
    release = policy.update(
        0.2, 0.0, 0.0, CLAMP_PROBE_CONFIRMED, FRONT_PAIR_MASK,
        (True, True, False, False))
    assert (release.clamp_action, release.module_mask) == (
        CLAMP_RELEASE, FRONT_PAIR_MASK)
    assert policy.update(
        0.2, 0.0, 0.0, CLAMP_IDLE).phase == 'rear_advance'
    assert policy.update(0.9, 0.0, 0.0).phase == 'final_probe'
    final_probe = policy.update(0.9, 0.0, 0.0)
    assert (final_probe.clamp_action, final_probe.module_mask) == (
        CLAMP_START_PROBE, ALL_MODULES_MASK)
    done = policy.update(
        0.9, 0.0, 0.0, CLAMP_PROBE_CONFIRMED, ALL_MODULES_MASK,
        (True, True, True, True))
    assert done.done and done.phase == 'confirmed'


def test_entry_swerve_math_executes_lateral_alignment_commands():
    steering, wheels = calculate_swerve_commands(
        [0.355, 0.355, -0.355, -0.355],
        [0.335, -0.335, -0.335, 0.335],
        0.0935, 2.88, 8.0, 0.0, -0.05, 0.0, [0.0] * 4)
    assert all(abs(angle + 1.57079632679) < 1.0e-6 for angle in steering)
    assert all(speed > 0.0 for speed in wheels)


def test_turnaround_and_forward_exit_policies():
    turn = TurnaroundPolicy(tolerance=0.02, confirmation_samples=2)
    assert turn.update(1.0).wz > 0.0
    assert not turn.update(0.01).done
    assert turn.update(0.0).done
    exit_policy = ForwardExitPolicy()
    release = exit_policy.update(-0.7, False, CLAMP_PROBE_CONFIRMED)
    assert release.clamp_action == CLAMP_RELEASE
    assert exit_policy.update(-0.7, False, CLAMP_IDLE).phase == 'rail_forward'


def test_exit_policy_uses_symmetric_crossing_then_ground_alignment():
    policy = ExitTransitionPolicy()
    assert policy.update(0.7, 0.0, 0.0, False).vx == -0.08
    crossing = policy.update(0.47, 0.02, 0.03, False)
    assert crossing.phase == 'cross_u_end'
    crossing = policy.update(0.0, 0.02, 0.03, False)
    assert (crossing.vx, crossing.vy, crossing.wz) == (-0.15, 0.0, 0.0)
    departure = policy.update(-0.51, 0.02, 0.03, False)
    assert departure.phase == 'ground_departure'
    done = policy.update(-0.7, 0.0, 0.0, True)
    assert done.done and done.vx == 0.0
