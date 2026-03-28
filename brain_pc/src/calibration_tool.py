#!/usr/bin/env python3
"""
Joint Calibration Tool
Send exact joint angles (in degrees) to the robot arm.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import sys


class CalibrationTool(Node):
    def __init__(self):
        super().__init__('calibration_tool')
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    def send_joint_angles(self, angles_deg, duration_sec=2.0):
        """Send joint angles in degrees"""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            print("ERROR: Arm controller not available!")
            return False

        # Convert degrees to radians
        angles_rad = [math.radians(a) for a in angles_deg]

        # Build trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = angles_rad
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        goal.trajectory.points = [point]

        print(f"\nSending command:")
        for i, name in enumerate(self.joint_names):
            print(f"  {name}: {angles_deg[i]:>7.2f}° ({angles_rad[i]:>7.4f} rad)")

        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            print("ERROR: Goal rejected!")
            return False

        print("\nMoving...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        print("Done!")
        return True


def print_help():
    print("""
Joint Calibration Tool
======================

Usage:
  python3 calibration_tool.py <j1> <j2> <j3> <j4> <j5> <j6>   - Set all joints (degrees)
  python3 calibration_tool.py j2 90                          - Set single joint (degrees)
  python3 calibration_tool.py home                           - All joints to 0
  python3 calibration_tool.py j2 90 --time 5                 - Move over 5 seconds

Examples:
  python3 calibration_tool.py 0 90 0 0 0 0      # Joint 2 to 90°
  python3 calibration_tool.py j2 45             # Only joint 2 to 45°
  python3 calibration_tool.py j2 -90            # Joint 2 to -90°
  python3 calibration_tool.py home              # All to zero
""")


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ['-h', '--help', 'help']:
        print_help()
        return

    rclpy.init()
    tool = CalibrationTool()

    angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    duration = 2.0

    # Parse --time argument if present
    args = sys.argv[1:]
    if '--time' in args:
        idx = args.index('--time')
        duration = float(args[idx + 1])
        args = args[:idx] + args[idx+2:]

    # Parse joint commands
    if args[0] == 'home':
        angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    elif args[0].startswith('j') and len(args[0]) == 2:
        # Single joint command: j2 90
        joint_num = int(args[0][1])
        if 1 <= joint_num <= 6:
            angles[joint_num - 1] = float(args[1])
        else:
            print(f"Invalid joint number: {joint_num}")
            return
    elif len(args) >= 6:
        # All joints specified
        angles = [float(a) for a in args[:6]]
    else:
        print("Invalid arguments. Use --help for usage.")
        return

    try:
        tool.send_joint_angles(angles, duration)
    except KeyboardInterrupt:
        print("\nCancelled")
    finally:
        tool.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
