#!/usr/bin/env python3
"""验收夹轨释放、向前离轨和自由路面模式恢复。"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

from verify_rail_entry_closed_loop import WHEEL_LINKS
from verify_rail_entry_state_machine import StateMachineEntryVerifier
from wr_rail_controller.transition_logic import ExitTransitionPolicy


ENTRY_X = -1.0


class RailExitVerifier(StateMachineEntryVerifier):
    def exit_with_guarded_alignment(self, timeout):
        end_time = time.monotonic() + timeout
        policy = ExitTransitionPolicy()
        grounded_samples = 0
        peak_abs_y = 0.0
        peak_abs_yaw = 0.0
        while time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.02)
            state = self.snapshot()
            abs_y = abs(state['y'])
            abs_yaw = abs(state['yaw'])
            peak_abs_y = max(peak_abs_y, abs_y)
            peak_abs_yaw = max(peak_abs_yaw, abs_yaw)
            all_on_ground = max(state['clearances'].values()) < 0.001
            result = policy.update(
                state['x'] - ENTRY_X, state['y'], state['yaw'], all_on_ground)
            command = Twist()
            command.linear.x = result.vx
            command.linear.y = result.vy
            command.angular.z = result.wz
            self.publisher.publish(command)

            if result.done:
                grounded_samples += 1
                if grounded_samples >= 10:
                    return True, result.phase, peak_abs_y, peak_abs_yaw
            else:
                grounded_samples = 0
        return False, policy.phase, peak_abs_y, peak_abs_yaw


def main():
    rclpy.init()
    node = RailExitVerifier()
    failures = []
    try:
        if not node.wait_until_ready(20.0):
            print('FAIL: Gazebo model/link states were not available within 20 seconds.')
            return 1

        node.zero_for(2.0)
        start = node.snapshot()
        exited, final_phase, peak_abs_y, peak_abs_yaw = node.exit_with_guarded_alignment(45.0)
        exit_end = node.snapshot()
        node.zero_for(1.0)
        stopped_first = node.snapshot()
        node.zero_for(2.0)
        stopped_final = node.snapshot()

        reverse_travel = start['x'] - exit_end['x']
        stop_drift = math.hypot(
            stopped_final['x'] - stopped_first['x'],
            stopped_final['y'] - stopped_first['y'],
        )

        print('exit policy: rail reverse -> symmetric U-end crossing -> ground departure')
        print(f'final phase: {final_phase}')
        print(f'start pose: x={start["x"]:.4f} m, y={start["y"]:.4f} m, '
              f'yaw={start["yaw"]:.4f} rad')
        print(f'reverse travel: {reverse_travel:.4f} m, peak |y|={peak_abs_y:.4f} m, '
              f'peak |yaw|={peak_abs_yaw:.4f} rad')
        for name in WHEEL_LINKS:
            print(f'{name} final tire clearance: '
                  f'{stopped_final["clearances"][name] * 1000.0:.2f} mm')
        print(f'stop drift over 2 s: {stop_drift * 1000.0:.2f} mm')

        if min(start['clearances'].values()) < 0.002:
            failures.append('exit test did not start with all four tires clear of the floor')
        if not exited:
            failures.append('all four tires did not return to the concrete within 45 seconds')
        if reverse_travel < 1.0:
            failures.append('chassis did not reverse far enough through the exit')
        if peak_abs_y > 0.05:
            failures.append('exit exceeded 50 mm lateral error')
        if peak_abs_yaw > 0.12:
            failures.append('exit exceeded 0.12 rad yaw error')
        still_lifted = [
            name for name, value in stopped_final['clearances'].items()
            if value > 0.001
        ]
        if still_lifted:
            failures.append('tire(s) remained lifted after exit: ' + ', '.join(still_lifted))
        if stop_drift > 0.030:
            failures.append('chassis did not remain stopped after exit')

        if failures:
            for failure in failures:
                print(f'FAIL: {failure}')
            return 1

        print('RAIL_EXIT_VERIFICATION_PASSED')
        return 0
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
