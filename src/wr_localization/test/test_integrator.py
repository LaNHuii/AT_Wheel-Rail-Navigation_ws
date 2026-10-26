"""验证平面里程积分、异常时间间隔拒绝与 STM32 重启处理。"""

from math import isclose, pi

from wr_localization.integrator import PlanarTwistIntegrator


def test_integrates_forward_velocity():
    integrator = PlanarTwistIntegrator()
    integrator.update(1.0, 7, 1.0, 0.0, 0.0)
    pose = integrator.update(1.1, 7, 1.0, 0.0, 0.0)
    assert isclose(pose.x, 0.1)
    assert isclose(pose.y, 0.0)


def test_integrates_body_lateral_velocity_after_rotation():
    integrator = PlanarTwistIntegrator()
    integrator.pose.yaw = pi / 2.0
    integrator.update(1.0, 7, 0.0, 1.0, 0.0)
    pose = integrator.update(1.1, 7, 0.0, 1.0, 0.0)
    assert pose.x < -0.099
    assert abs(pose.y) < 1.0e-9


def test_rejects_large_gap_and_resets_time_on_mcu_reboot():
    integrator = PlanarTwistIntegrator(max_dt=0.2)
    integrator.update(1.0, 1, 1.0, 0.0, 0.0)
    assert integrator.update(2.0, 1, 1.0, 0.0, 0.0).x == 0.0
    assert integrator.update(2.1, 2, 1.0, 0.0, 0.0).x == 0.0
    assert isclose(integrator.update(2.2, 2, 1.0, 0.0, 0.0).x, 0.1)
