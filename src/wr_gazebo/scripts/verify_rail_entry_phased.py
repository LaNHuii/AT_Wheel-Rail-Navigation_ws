#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

from verify_rail_entry_closed_loop import WHEEL_LINKS, clamp
from verify_rail_entry_state_machine import StateMachineEntryVerifier


class PhasedEntryVerifier(StateMachineEntryVerifier):
    def enter_with_phased_control(self, timeout):
        end_time = time.monotonic() + timeout
        phase = 'align'
        alignment_holds = 0
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
            command = Twist()

            if phase == 'align':
                command.linear.y = clamp(-2.5 * state['y'], -0.08, 0.08)
                command.angular.z = clamp(-3.0 * state['yaw'], -0.25, 0.25)
                precisely_aligned = abs_y < 0.0025 and abs_yaw < 0.006
                alignment_holds = alignment_holds + 1 if precisely_aligned else 0
                if alignment_holds >= 25:
                    phase = 'approach'

            elif phase == 'approach':
                if min(state['clearances']['front_left_wheel_link'], state['clearances']['front_right_wheel_link']) > 0.0015 and abs_y < 0.012 and abs_yaw < 0.025:
                    phase = 'rail_transition'
                elif abs_y > 0.012 or abs_yaw > 0.025:
                    phase = 'align'
                    alignment_holds = 0
                else:
                    command.linear.x = 0.050
                    command.linear.y = clamp(-1.0 * state['y'], -0.015, 0.015)
                    command.angular.z = clamp(-1.5 * state['yaw'], -0.06, 0.06)

            else:
                command.linear.x = 0.040

            self.publisher.publish(command)

            if state['x'] > -0.30 and min(state['clearances'].values()) > 0.002:
                lifted_samples += 1
                if lifted_samples >= 10:
                    return True, phase, peak_abs_y, peak_abs_yaw
            else:
                lifted_samples = 0
        return False, phase, peak_abs_y, peak_abs_yaw


def main():
    rclpy.init()
    node = PhasedEntryVerifier()
    failures = []
    try:
        if not node.wait_until_ready(20.0):
            print('FAIL: Gazebo model/link states were not available within 20 seconds.')
            return 1

        node.zero_for(2.0)
        start = node.snapshot()
        entered, final_phase, peak_abs_y, peak_abs_yaw = node.enter_with_phased_control(60.0)
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
        print('entry policy: align -> approach -> vx-only rail transition')
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
            failures.append('all four tires did not lift within the 60 second entry window')
        if travel < 1.0:
            failures.append('chassis did not advance far enough through the entry')
        if peak_abs_y > 0.05:
            failures.append('phased entry exceeded 50 mm lateral error')
        if peak_abs_yaw > 0.12:
            failures.append('phased entry exceeded 0.12 rad yaw error')
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

        print('RAIL_ENTRY_PHASED_VERIFICATION_PASSED')
        return 0
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
