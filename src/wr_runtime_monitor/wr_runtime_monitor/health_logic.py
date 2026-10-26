"""传感器新鲜度和自主运行条件的纯逻辑判断。"""

from dataclasses import dataclass


LIDAR_STALE = 1
IMU_STALE = 2
CHASSIS_STALE = 4
SAFETY_STALE = 8
LOCALIZATION_STALE = 16
VISION_STALE = 32
CHASSIS_UNSAFE = 64
SAFETY_STOP = 128


@dataclass
class HealthConfig:
    lidar_timeout: float = 0.35
    imu_timeout: float = 0.15
    chassis_timeout: float = 0.25
    safety_timeout: float = 0.35
    localization_timeout: float = 0.50
    vision_timeout: float = 0.35
    require_lidar: bool = True
    require_imu: bool = True
    require_chassis: bool = True
    require_safety: bool = True
    require_localization: bool = False
    require_vision: bool = True


@dataclass
class HealthInputs:
    now: float
    lidar_at: float = float('-inf')
    imu_at: float = float('-inf')
    chassis_at: float = float('-inf')
    safety_at: float = float('-inf')
    localization_at: float = float('-inf')
    vision_at: float = float('-inf')
    chassis_safe: bool = False
    safety_stop: bool = True


@dataclass
class HealthResult:
    lidar_fresh: bool
    imu_fresh: bool
    chassis_fresh: bool
    safety_fresh: bool
    localization_fresh: bool
    vision_fresh: bool
    autonomy_ready: bool
    fault_mask: int
    reason: str


def evaluate_health(inputs, config=None):
    """根据接收时刻计算健康状态；缺失的必需数据按故障处理。"""
    cfg = config or HealthConfig()
    fresh = {
        'lidar': inputs.now - inputs.lidar_at <= cfg.lidar_timeout,
        'imu': inputs.now - inputs.imu_at <= cfg.imu_timeout,
        'chassis': inputs.now - inputs.chassis_at <= cfg.chassis_timeout,
        'safety': inputs.now - inputs.safety_at <= cfg.safety_timeout,
        'localization': inputs.now - inputs.localization_at <= cfg.localization_timeout,
        'vision': inputs.now - inputs.vision_at <= cfg.vision_timeout,
    }
    requirements = {
        'lidar': cfg.require_lidar,
        'imu': cfg.require_imu,
        'chassis': cfg.require_chassis,
        'safety': cfg.require_safety,
        'localization': cfg.require_localization,
        'vision': cfg.require_vision,
    }
    stale_bits = {
        'lidar': LIDAR_STALE,
        'imu': IMU_STALE,
        'chassis': CHASSIS_STALE,
        'safety': SAFETY_STALE,
        'localization': LOCALIZATION_STALE,
        'vision': VISION_STALE,
    }
    fault_mask = 0
    reasons = []
    for name, required in requirements.items():
        if required and not fresh[name]:
            fault_mask |= stale_bits[name]
            reasons.append(name + ' stale')
    if cfg.require_chassis and fresh['chassis'] and not inputs.chassis_safe:
        fault_mask |= CHASSIS_UNSAFE
        reasons.append('chassis unsafe')
    if cfg.require_safety and fresh['safety'] and inputs.safety_stop:
        fault_mask |= SAFETY_STOP
        reasons.append('local safety stop')
    ready = fault_mask == 0
    return HealthResult(
        lidar_fresh=fresh['lidar'],
        imu_fresh=fresh['imu'],
        chassis_fresh=fresh['chassis'],
        safety_fresh=fresh['safety'],
        localization_fresh=fresh['localization'],
        vision_fresh=fresh['vision'],
        autonomy_ready=ready,
        fault_mask=fault_mask,
        reason='ready' if ready else ', '.join(reasons),
    )
