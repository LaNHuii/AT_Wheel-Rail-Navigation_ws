#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class GazeboCommandAdapter(Node):
    """Applies simulator-only joint signs without changing navigation interfaces."""

    def __init__(self):
        super().__init__('gazebo_command_adapter')
        self.drive_signs = self.declare_parameter(
            'drive_signs', [-1.0, 1.0, -1.0, -1.0]
        ).value
        self.steering_signs = self.declare_parameter(
            'steering_signs', [1.0, 1.0, 1.0, 1.0]
        ).value
        self.steering_offsets = self.declare_parameter(
            'steering_offsets', [0.0, 0.0, 0.0, 0.0]
        ).value
        if not all(len(values) == 4 for values in (
            self.drive_signs, self.steering_signs, self.steering_offsets
        )):
            raise ValueError('all Gazebo swerve adapter arrays must contain four entries')

        self.steering_publisher = self.create_publisher(
            Float64MultiArray, '/steering_position_controller/commands', 10
        )
        self.wheel_publisher = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10
        )
        self.create_subscription(
            Float64MultiArray, '/gazebo_raw/steering_commands', self.on_steering, 10
        )
        self.create_subscription(
            Float64MultiArray, '/gazebo_raw/wheel_commands', self.on_wheel, 10
        )

    def on_steering(self, message):
        if len(message.data) != 4:
            self.get_logger().error('rejected steering command not containing four values')
            return
        output = Float64MultiArray()
        output.data = [
            sign * value + offset
            for value, sign, offset in zip(
                message.data, self.steering_signs, self.steering_offsets
            )
        ]
        self.steering_publisher.publish(output)

    def on_wheel(self, message):
        if len(message.data) != 4:
            self.get_logger().error('rejected wheel command not containing four values')
            return
        output = Float64MultiArray()
        output.data = [
            sign * value for value, sign in zip(message.data, self.drive_signs)
        ]
        self.wheel_publisher.publish(output)


def main():
    rclpy.init()
    node = GazeboCommandAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
