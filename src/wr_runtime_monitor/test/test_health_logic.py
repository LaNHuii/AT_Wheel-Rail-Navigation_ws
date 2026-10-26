"""运行健康状态纯逻辑测试。"""

from wr_runtime_monitor.health_logic import (
    IMU_STALE,
    VISION_STALE,
    HealthConfig,
    HealthInputs,
    evaluate_health,
)


def test_all_required_inputs_ready():
    result = evaluate_health(HealthInputs(
        now=10.0,
        lidar_at=9.9,
        imu_at=9.95,
        chassis_at=9.9,
        safety_at=9.9,
        localization_at=0.0,
        vision_at=9.9,
        chassis_safe=True,
        safety_stop=False,
    ))
    assert result.autonomy_ready
    assert result.fault_mask == 0


def test_stale_required_inputs_block_autonomy():
    result = evaluate_health(HealthInputs(
        now=10.0,
        lidar_at=9.9,
        imu_at=0.0,
        chassis_at=9.9,
        safety_at=9.9,
        vision_at=0.0,
        chassis_safe=True,
        safety_stop=False,
    ))
    assert not result.autonomy_ready
    assert result.fault_mask & IMU_STALE
    assert result.fault_mask & VISION_STALE


def test_radar_only_profile_does_not_require_vision():
    config = HealthConfig(require_vision=False)
    result = evaluate_health(HealthInputs(
        now=10.0,
        lidar_at=9.9,
        imu_at=9.95,
        chassis_at=9.9,
        safety_at=9.9,
        vision_at=0.0,
        chassis_safe=True,
        safety_stop=False,
    ), config)
    assert result.autonomy_ready
