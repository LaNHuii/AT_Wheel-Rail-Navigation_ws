from wr_rail_controller.transition_logic import EntryTransitionPolicy, ExitTransitionPolicy


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
