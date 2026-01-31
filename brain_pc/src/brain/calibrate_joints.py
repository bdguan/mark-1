#!/usr/bin/env python3
"""
Joint calibration tool.
Commands known angles to each joint and compares what the robot reports back.
Computes accurate radians_to_degrees and degrees_to_radians conversion factors.

Usage:
  1. Launch robot (real.launch.py) + interface (firmware_launch.py)
  2. Run this script
  3. It will move one joint at a time to known positions
  4. Outputs corrected calibration values for arm_constants.yaml
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import yaml
import os


class JointCalibrator(Node):
    def __init__(self):
        super().__init__('joint_calibrator')

        self.latest_feedback = None

        # Subscribe to robot feedback (actual angles from robot in degrees)
        self.feedback_sub = self.create_subscription(
            Float32MultiArray,
            '/query_response_arm',
            self.feedback_callback,
            10
        )

        # Publish arm commands (in degrees, after calibration)
        self.command_pub = self.create_publisher(
            Float32MultiArray,
            '/arm_hardware_command',
            10
        )

        self.get_logger().info('Joint calibrator ready')
        self.get_logger().info('Make sure robot launch + interface are running')

    def feedback_callback(self, msg):
        if len(msg.data) >= 6:
            self.latest_feedback = list(msg.data[:6])

    def wait_for_feedback(self, timeout=5.0):
        """Wait until we get a feedback message"""
        start = time.time()
        self.latest_feedback = None
        while self.latest_feedback is None and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_feedback

    def get_current_angles(self, samples=5):
        """Get current robot angles, averaged over multiple samples"""
        readings = []
        for _ in range(samples):
            fb = self.wait_for_feedback(timeout=2.0)
            if fb is not None:
                readings.append(fb)
            time.sleep(0.2)

        if not readings:
            return None

        avg = np.mean(readings, axis=0)
        return avg.tolist()

    def run_calibration(self):
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('JOINT CALIBRATION TOOL')
        self.get_logger().info('='*60)
        self.get_logger().info('')
        self.get_logger().info('This tool reads the current robot position and compares')
        self.get_logger().info('what MoveIt thinks vs what the robot reports.')
        self.get_logger().info('')

        # First, get current robot angles
        self.get_logger().info('Reading current robot angles...')
        current = self.get_current_angles(samples=10)

        if current is None:
            self.get_logger().error('No feedback from robot! Is the interface running?')
            return

        self.get_logger().info(f'Robot reports (degrees): '
                               f'[{current[0]:.2f}, {current[1]:.2f}, {current[2]:.2f}, '
                               f'{current[3]:.2f}, {current[4]:.2f}, {current[5]:.2f}]')

        # Load current calibration
        yaml_path = os.path.join(
            os.path.dirname(__file__), '..',
            'mycobot_320pi_controller', 'calibrations', 'arm_constants.yaml'
        )
        yaml_path = os.path.abspath(yaml_path)

        if os.path.exists(yaml_path):
            with open(yaml_path) as f:
                cal = yaml.safe_load(f)
            self.get_logger().info(f'Loaded calibration from: {yaml_path}')
        else:
            self.get_logger().warn(f'Calibration file not found at: {yaml_path}')
            cal = {}

        current_r2d = [
            cal.get('arm_joint_1_radians_to_degrees', 57.2958),
            cal.get('arm_joint_2_radians_to_degrees', 57.2958),
            cal.get('arm_joint_3_radians_to_degrees', 57.2958),
            cal.get('arm_joint_4_radians_to_degrees', 57.2958),
            cal.get('arm_joint_5_radians_to_degrees', 57.2958),
            cal.get('arm_joint_6_radians_to_degrees', 57.2958),
        ]

        current_d2r = [
            cal.get('arm_joint_1_degrees_to_radians', 0.01745),
            cal.get('arm_joint_2_degrees_to_radians', 0.01745),
            cal.get('arm_joint_3_degrees_to_radians', 0.01745),
            cal.get('arm_joint_4_degrees_to_radians', 0.01745),
            cal.get('arm_joint_5_degrees_to_radians', 0.01745),
            cal.get('arm_joint_6_degrees_to_radians', 0.01745),
        ]

        self.get_logger().info('')
        self.get_logger().info('Current calibration (rad_to_deg):')
        for i in range(6):
            self.get_logger().info(f'  Joint {i+1}: {current_r2d[i]:.4f} (inverse: {current_d2r[i]:.8f})')

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('INTERACTIVE CALIBRATION')
        self.get_logger().info('='*60)
        self.get_logger().info('')
        self.get_logger().info('For each joint, move it to a known angle using RViz,')
        self.get_logger().info('then enter what MoveIt shows (in degrees).')
        self.get_logger().info('The robot will report what it actually reads.')
        self.get_logger().info('Do this at 2-3 different angles per joint for accuracy.')
        self.get_logger().info('')

        new_r2d = list(current_r2d)
        new_d2r = list(current_d2r)

        for joint_idx in range(6):
            self.get_logger().info(f'--- Joint {joint_idx + 1} ---')
            self.get_logger().info(f'Current factor: {current_r2d[joint_idx]:.4f}')

            measurements = []

            while True:
                response = input(f'  J{joint_idx+1}: Move to a position, then enter MoveIt angle in degrees (or "done"): ').strip()

                if response.lower() == 'done':
                    break

                try:
                    moveit_deg = float(response)
                except ValueError:
                    print('  Invalid number, try again')
                    continue

                # Read what robot reports
                robot_angles = self.get_current_angles(samples=10)
                if robot_angles is None:
                    print('  No feedback from robot!')
                    continue

                robot_deg = robot_angles[joint_idx]
                moveit_rad = np.radians(moveit_deg)

                self.get_logger().info(
                    f'  MoveIt: {moveit_deg:.2f}° ({moveit_rad:.4f} rad) | '
                    f'Robot reports: {robot_deg:.2f}°'
                )

                if abs(moveit_rad) > 0.01:  # Avoid division by near-zero
                    measured_factor = robot_deg / moveit_rad
                    measurements.append((moveit_rad, robot_deg, measured_factor))
                    self.get_logger().info(f'  Measured factor: {measured_factor:.4f}')
                else:
                    self.get_logger().info(f'  Angle too small for reliable factor calculation')

            if measurements:
                # Compute best-fit factor using least squares
                rads = np.array([m[0] for m in measurements])
                degs = np.array([m[1] for m in measurements])

                # Linear fit: degrees = factor * radians
                factor = np.sum(rads * degs) / np.sum(rads * rads)
                inverse = 1.0 / factor if abs(factor) > 1e-6 else 0.01745

                new_r2d[joint_idx] = factor
                new_d2r[joint_idx] = inverse

                self.get_logger().info(
                    f'  Joint {joint_idx+1} new factor: {factor:.4f} '
                    f'(was {current_r2d[joint_idx]:.4f}, inverse: {inverse:.10f})'
                )
            else:
                self.get_logger().info(f'  No measurements for Joint {joint_idx+1}, keeping current value')

        # Output results
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('NEW CALIBRATION VALUES')
        self.get_logger().info('='*60)

        yaml_output = ''
        for i in range(6):
            yaml_output += f'arm_joint_{i+1}_radians_to_degrees: {new_r2d[i]:.4f}\n'
        yaml_output += '\n'
        for i in range(6):
            yaml_output += f'arm_joint_{i+1}_degrees_to_radians: {new_d2r[i]:.10f}\n'
        yaml_output += '\n'
        yaml_output += f'gripper_radians_to_degrees: {cal.get("gripper_radians_to_degrees", 1.0)}\n'
        yaml_output += f'gripper_degrees_to_radians: {cal.get("gripper_degrees_to_radians", 1.0)}\n'

        self.get_logger().info(f'\n{yaml_output}')

        # Ask to save
        save = input('Save to arm_constants.yaml? (y/n): ').strip().lower()
        if save == 'y':
            with open(yaml_path, 'w') as f:
                f.write(yaml_output)
            self.get_logger().info(f'Saved to {yaml_path}')
            self.get_logger().info('Rebuild and restart to apply:')
            self.get_logger().info('  colcon build --packages-select mycobot_320pi_controller')
            self.get_logger().info('  source install/setup.bash')
        else:
            self.get_logger().info('Not saved. Copy the values above manually if needed.')


def main(args=None):
    rclpy.init(args=args)
    node = JointCalibrator()
    try:
        node.run_calibration()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
