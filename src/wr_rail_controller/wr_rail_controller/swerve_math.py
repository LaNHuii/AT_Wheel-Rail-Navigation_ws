import math


def calculate_swerve_commands(
        module_x, module_y, wheel_radius, steering_limit,
        max_wheel_angular_velocity, vx, vy, wz, current_angles):
    """Match the Gazebo C++ swerve solver while the rail supervisor owns joints."""
    if not (len(module_x) == len(module_y) == len(current_angles) == 4):
        raise ValueError('swerve geometry and steering state must contain four modules')
    angles = []
    wheel_speeds = []
    for x, y, current in zip(module_x, module_y, current_angles):
        module_vx = vx - wz * y
        module_vy = vy + wz * x
        speed = math.hypot(module_vx, module_vy)
        if speed < 1.0e-8:
            angles.append(max(-steering_limit, min(current, steering_limit)))
            wheel_speeds.append(0.0)
            continue
        base_angle = math.atan2(module_vy, module_vx)
        candidates = []
        for half_turns in range(-2, 3):
            candidate = base_angle + half_turns * math.pi
            if -steering_limit <= candidate <= steering_limit:
                reverse = abs(half_turns) % 2 == 1
                angular_speed = (-1.0 if reverse else 1.0) * speed / wheel_radius
                candidates.append((abs(candidate - current), candidate, angular_speed))
        if not candidates:
            raise ValueError('no steering solution exists within the configured limit')
        _, angle, angular_speed = min(candidates)
        angles.append(angle)
        wheel_speeds.append(angular_speed)

    peak = max(abs(value) for value in wheel_speeds)
    if peak > max_wheel_angular_velocity:
        scale = max_wheel_angular_velocity / peak
        wheel_speeds = [value * scale for value in wheel_speeds]
    return angles, wheel_speeds
