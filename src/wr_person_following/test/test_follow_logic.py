"""验证自由路面/轨内跟随、目标绑定、障碍停车和超时安全行为。"""

from wr_person_following.follow_logic import (
    ACTIVITY_DEPOSITING,
    FOLLOWING,
    FREE_SURFACE,
    HARDWARE_HOLD,
    OBSTACLE_HOLD,
    ON_RAIL,
    SENSOR_HOLD,
    TARGET_SENSOR_FAULT,
    TARGET_STATIONARY,
    TARGET_TRACKED,
    WAITING_FOR_TARGET,
    FollowInputs,
    PersonFollowLogic,
)


def ready(now=0.0, mode=FREE_SURFACE):
    return FollowInputs(
        now=now, chassis_mode=mode, chassis_ok=True, safety_fresh=True,
        rotation_safe=True, target_state=TARGET_TRACKED, target_session_id=7,
        binding_id=11, tracker_id=3, position_x=1.0, lateral_offset=0.0,
        nearest_body_distance=1.0, longitudinal_distance=1.0)


def started():
    logic = PersonFollowLogic()
    logic.start()
    return logic


def test_free_surface_following_rotates_toward_person_and_moves_forward():
    logic = started()
    data = ready()
    data.lateral_offset = 0.2
    output = logic.update(data)
    assert output.state == FOLLOWING
    assert output.vx > 0.0 and output.wz > 0.0 and output.vy > 0.0


def test_rail_following_is_longitudinal_only():
    logic = started()
    data = ready(mode=ON_RAIL)
    data.lateral_offset = 0.4
    output = logic.update(data)
    assert output.state == FOLLOWING
    assert output.vx > 0.0 and output.vy == 0.0 and output.wz == 0.0


def test_rail_following_reverses_slowly_when_person_moves_too_close():
    logic = started()
    data = ready(mode=ON_RAIL)
    data.longitudinal_distance = 0.10
    output = logic.update(data)
    assert -logic.config.max_rail_reverse_vx <= output.vx < 0.0
    assert output.vy == 0.0 and output.wz == 0.0


def test_stationary_or_depositing_target_forces_stop():
    logic = started()
    data = ready()
    data.activity = ACTIVITY_DEPOSITING
    output = logic.update(data)
    assert output.state == TARGET_STATIONARY
    assert (output.vx, output.vy, output.wz) == (0.0, 0.0, 0.0)


def test_rail_target_loss_waits_without_search_or_retreat():
    logic = started()
    logic.update(ready(now=0.0, mode=ON_RAIL))
    lost = ready(now=40.0, mode=ON_RAIL)
    lost.target_state = 0
    output = logic.update(lost)
    assert output.state == WAITING_FOR_TARGET
    assert (output.vx, output.vy, output.wz) == (0.0, 0.0, 0.0)


def test_target_loss_holds_position_without_active_search():
    logic = started()
    logic.update(ready(now=0.0))
    lost = ready(now=1.0)
    lost.target_state = 0
    output = logic.update(lost)
    assert output.state == WAITING_FOR_TARGET
    assert (output.vx, output.vy, output.wz) == (0.0, 0.0, 0.0)


def test_new_binding_cannot_steal_a_healthy_task():
    logic = started()
    assert logic.update(ready()).state == FOLLOWING
    replacement = ready(now=1.0)
    replacement.binding_id = 12
    assert logic.update(replacement).state == WAITING_FOR_TARGET
    # 仅 tracker/binding 变化不能抢走正在健康执行的任务；
    # 视觉必须先明确报告丢失或身份不确定区间。
    replacement.now = 2.0
    assert logic.update(replacement).state == WAITING_FOR_TARGET
    assert logic.bound_binding_id == 11


def test_obstacle_and_stale_evidence_force_zero_command():
    logic = started()
    obstacle = ready()
    obstacle.forward_path_blocked = True
    assert logic.update(obstacle).state == OBSTACLE_HOLD
    stale = ready()
    stale.safety_fresh = False
    assert logic.update(stale).state == SENSOR_HOLD
    stale.safety_fresh = True
    stale.chassis_ok = False
    assert logic.update(stale).state == HARDWARE_HOLD
    stale.chassis_ok = True
    stale.target_state = TARGET_SENSOR_FAULT
    assert logic.update(stale).state == SENSOR_HOLD


def test_free_surface_uses_lidar_lateral_bypass_only_when_available():
    logic = started()
    data = ready()
    data.forward_path_blocked = True
    data.bypass_active = True
    data.suggested_lateral_velocity = 0.06
    output = logic.update(data)
    assert output.state == FOLLOWING
    assert output.vx > 0.0
    assert output.vy == 0.06


def test_too_close_never_commands_blind_reverse():
    logic = started()
    data = ready()
    data.nearest_body_distance = 0.15
    output = logic.update(data)
    assert output.vx == 0.0
