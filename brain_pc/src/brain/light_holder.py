#!/usr/bin/env python3
"""
Light holder script: Move to pickup position, grab object, then move to hold position.
Uses FollowJointTrajectory for both arm and gripper.

Usage:
  1. Launch real robot stack
  2. python3 brain/light_holder.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time

# All angles in degrees
PICKUP_POSE = [29, -5, 59, -52, -140, -45]
HOLD_POSE = [-2, -2, 106, -48, -101, -30]
HOME_POSE = [0, 0, 0, 0, 0, 0]

ARM_JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
GRIPPER_JOINT_NAMES = ['left_jaw_joint_1']

GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = -1.11


class LightHolder(Node):
    def __init__(self):
        super().__init__('light_holder')

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )

        self.current_arm_pose = list(HOME_POSE)

    def move_to(self, angles_deg, duration_sec=3.0, num_points=50):
        """Send a smooth multi-point trajectory with trapezoidal velocity profile."""
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Arm controller not available!')
            return False

        start_rad = [math.radians(a) for a in self.current_arm_pose]
        goal_rad = [math.radians(a) for a in angles_deg]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINT_NAMES

        for i in range(num_points):
            t_frac = (i + 1) / num_points
            # Smooth S-curve: 3t^2 - 2t^3 (zero velocity at start and end)
            s = 3.0 * t_frac**2 - 2.0 * t_frac**3
            # Velocity: 6t - 6t^2 (normalized)
            ds = 6.0 * t_frac - 6.0 * t_frac**2

            point = JointTrajectoryPoint()
            point.positions = [
                start_rad[j] + s * (goal_rad[j] - start_rad[j])
                for j in range(6)
            ]
            point.velocities = [
                (ds / duration_sec) * (goal_rad[j] - start_rad[j])
                for j in range(6)
            ]

            t = t_frac * duration_sec
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)

            goal.trajectory.points.append(point)

        # Final point: explicit zero velocity
        goal.trajectory.points[-1].velocities = [0.0] * 6

        self.get_logger().info(f'Moving to {angles_deg} ({num_points} waypoints)')
        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.current_arm_pose = list(angles_deg)
        self.get_logger().info('Move complete.')
        return True

    def set_gripper(self, position, duration_sec=1.0):
        """Send gripper position (0.0 = open, -1.11 = closed)."""
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper controller not available!')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = GRIPPER_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        goal.trajectory.points = [point]

        self.get_logger().info(f'Gripper -> {position}')
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Gripper done.')
        return True

    def gripper_open(self, settle_time=1.0):
        self.get_logger().info('Opening gripper')
        self.set_gripper(GRIPPER_OPEN)
        time.sleep(settle_time)

    def gripper_close(self, settle_time=1.0):
        self.get_logger().info('Closing gripper')
        self.set_gripper(GRIPPER_CLOSED)
        time.sleep(settle_time)

    def run(self):
        self.get_logger().info('=== LIGHT HOLDER ===')

        # Open gripper first
        self.get_logger().info('--- Opening gripper ---')
        self.gripper_open()

        # Move to pickup position
        self.get_logger().info('--- Moving to pickup position ---')
        self.move_to(PICKUP_POSE, duration_sec=3)

        # Wait 2 seconds
        self.get_logger().info('--- Waiting 2 seconds ---')
        time.sleep(2.0)

        # Close gripper
        self.gripper_close()

        # Move to hold position
        self.get_logger().info('--- Moving to hold position ---')
        self.move_to(HOLD_POSE, duration_sec=3)

        self.get_logger().info('=== DONE ===')


def main():
    rclpy.init()
    node = LightHolder()
    try:
        node.run()
    except KeyboardInterrupt:
        print('\nCancelled')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
