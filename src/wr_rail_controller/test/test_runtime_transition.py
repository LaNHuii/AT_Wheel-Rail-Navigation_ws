"""验证对准、前轮验证、后轮上轨、四轮终验和向前离轨的顺序。"""

from wr_rail_controller.runtime_transition import (
    ALL_MODULES_MASK,
    CLAMP_CONFIRMED,
    CLAMP_IDLE,
    CLAMP_RELEASE,
    CLAMP_START_PROBE,
    FRONT_PAIR_MASK,
    RuntimeRailTransition,
    RuntimeTransitionConfig,
    RuntimeTransitionInput,
)


def clear_entry(**values):
    data = RuntimeTransitionInput(
        estimate_valid=True, entry_x=1.0, lateral_error=0.0,
        heading_error=0.0, traversable=True)
    for key, value in values.items():
        setattr(data, key, value)
    return data


def test_runtime_entry_uses_perception_then_two_stage_clamp():
    machine = RuntimeRailTransition(
        'enter', RuntimeTransitionConfig(alignment_samples=1))
    assert machine.update(clear_entry()).phase == 'approach'
    assert machine.update(clear_entry(entry_x=0.5)).vx > 0.0
    assert machine.update(clear_entry(entry_x=0.1)).phase == 'front_probe'
    front = machine.update(clear_entry(entry_x=0.1))
    assert (front.clamp_action, front.module_mask) == (
        CLAMP_START_PROBE, FRONT_PAIR_MASK)
    assert machine.update(clear_entry(
        clamp_state=CLAMP_CONFIRMED, clamp_mask=FRONT_PAIR_MASK,
        contacts=(True, True, False, False))).phase == 'front_release'
    release = machine.update(clear_entry(
        clamp_state=CLAMP_CONFIRMED, clamp_mask=FRONT_PAIR_MASK,
        contacts=(True, True, False, False)))
    assert (release.clamp_action, release.module_mask) == (
        CLAMP_RELEASE, FRONT_PAIR_MASK)
    assert machine.update(clear_entry(clamp_state=CLAMP_IDLE)).phase == 'rear_advance'
    assert machine.update(clear_entry(travelled=0.5)).vx > 0.0
    assert machine.update(clear_entry(travelled=0.8)).phase == 'final_probe'
    final = machine.update(clear_entry(travelled=0.8))
    assert (final.clamp_action, final.module_mask) == (
        CLAMP_START_PROBE, ALL_MODULES_MASK)
    done = machine.update(clear_entry(
        clamp_state=CLAMP_CONFIRMED, clamp_mask=ALL_MODULES_MASK,
        contacts=(True, True, True, True), travelled=0.8))
    assert done.done


def test_runtime_entry_holds_for_obstacle_or_missing_estimate():
    machine = RuntimeRailTransition('enter')
    assert machine.update(RuntimeTransitionInput()).vx == 0.0
    assert machine.update(clear_entry(safety_blocked=True)).vx == 0.0


def test_runtime_forward_exit_releases_then_drives_configured_distance():
    machine = RuntimeRailTransition('exit')
    release = machine.update(RuntimeTransitionInput())
    assert release.clamp_action == CLAMP_RELEASE
    assert machine.update(RuntimeTransitionInput(clamp_state=CLAMP_IDLE)).phase == 'exit_forward'
    assert machine.update(RuntimeTransitionInput(travelled=0.4)).vx > 0.0
    assert machine.update(RuntimeTransitionInput(travelled=0.9)).done
