#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

from verify_rail_entry_closed_loop import (
    ClosedLoopEntryVerifier,
    WHEEL_LINKS,
)
from wr_rail_controller.transition_logic import EntryTransitionPolicy


ENTRY_X = -1.0


class StateMachineEntryVerifier(ClosedLoopEntryVerifier):
    def __init__(self):
        super().__init__()
        self._node_name = 'verify_rail_entry_state_machine'

    def enter_with_guarded_alignment(self, timeout):
        end_time = time.monotonic() + timeout
        policy = EntryTransitionPolicy()
        lifted_samples = 0
        peak_abs_y = 0.0
        peak_abs_yaw = 0.0
        while time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.02)
            state = self.snapshot()
            abs_y = abs(state['y'])
            abs_yaw = abs(state['yaw'])
            peak_abs_y = max(peak_abs_y, abs_y)
            peak_abs_yaw = max(peak_abs_yaw, abs_yaw)
            all_on_rail = min(state['clearances'].values()) > 0.002
            result = policy.update(
                state['x'] - ENTRY_X, state['y'], state['yaw'], all_on_rail)
            command = Twist()
            command.linear.x = result.vx
            command.linear.y = result.vy
            command.angular.z = result.wz
            self.publisher.publish(command)

            if result.done:
                lifted_samples += 1
                if lifted_samples >= 10:
                    return True, result.phase, peak_abs_y, peak_abs_yaw
            else:
                lifted_samples = 0
        return False, policy.phase, peak_abs_y, peak_abs_yaw


def main():
    rclpy.init()
    node = StateMachineEntryVerifier()
    failures = []
    try:
        if not node.wait_until_ready(20.0):
            print('FAIL: Gazebo model/link states were not available within 20 seconds.')
            return 1

        node.zero_for(2.0)
        start = node.snapshot()
        entered, final_phase, peak_abs_y, peak_abs_yaw = node.enter_with_guarded_alignment(45.0)
        entry_end = node.snapshot()
        node.zero_for(1.0)
        stopped_first = node.snapshot()
        node.zero_for(2.0)
        stopped_final = node.snapshot()

        travel = entry_end['x'] - start['x']
        stop_drift = math.hypot(
            stopped_final['x'] - stopped_first['x'],
            stopped_final['y'] - stopped_first['y'],
        )

        print('alignment source: Gazebo ground truth (test-only perception substitute)')
        print('entry policy: align -> controlled approach -> symmetric U-end crossing')
        print(f'final phase: {final_phase}')
        print(f'start pose: x={start["x"]:.4f} m, y={start["y"]:.4f} m, '
              f'yaw={start["yaw"]:.4f} rad')
        print(f'entry travel: dx={travel:.4f} m, peak |y|={peak_abs_y:.4f} m, '
              f'peak |yaw|={peak_abs_yaw:.4f} rad')
        for name in WHEEL_LINKS:
            print(f'{name} final tire clearance: '
                  f'{stopped_final["clearances"][name] * 1000.0:.2f} mm')
        print(f'stop drift over 2 s: {stop_drift * 1000.0:.2f} mm')

        if max(start['clearances'].values()) > 0.001:
            failures.append('test did not start with all four tires on the concrete')
        if not entered:
            failures.append('all four tires did not lift within the 45 second entry window')
        if travel < 1.0:
            failures.append('chassis did not advance far enough through the entry')
        if peak_abs_y > 0.05:
            failures.append('state-machine entry exceeded 50 mm lateral error')
        if peak_abs_yaw > 0.12:
            failures.append('state-machine entry exceeded 0.12 rad yaw error')
        not_lifted = [
            name for name, value in stopped_final['clearances'].items()
            if value < 0.002
        ]
        if not_lifted:
            failures.append('tire(s) were not supported clear of the floor: ' + ', '.join(not_lifted))
        if stop_drift > 0.030:
            failures.append('chassis did not remain stopped after entry')

        if failures:
            for failure in failures:
                print(f'FAIL: {failure}')
            return 1

        print('RAIL_ENTRY_STATE_MACHINE_VERIFICATION_PASSED')
        return 0
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
