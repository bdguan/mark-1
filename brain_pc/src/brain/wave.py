#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint

# CONFIGURATION
GROUP_NAME = "arm"
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

class WaveBot(Node):
    def __init__(self):
        super().__init__('wave_bot')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def move_joints(self, target_joints):
        """Sends a joint goal to MoveIt"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ MoveIt Action Server not found. Is demo.launch.py running?')
            return False

        # 1. Create the Goal Message
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = GROUP_NAME
        goal_msg.request.allowed_planning_time = 2.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # 2. Add Joint Constraints
        goal_msg.request.goal_constraints = [Constraints()]
        for i, angle in enumerate(target_joints):
            jc = JointConstraint()
            jc.joint_name = JOINT_NAMES[i]
            jc.position = float(angle)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_msg.request.goal_constraints[0].joint_constraints.append(jc)

        # 3. Send and Wait
        self.get_logger().info(f'📤 Sending Goal: {target_joints}...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal Rejected!')
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        if result.error_code.val == 1: # 1 = SUCCESS
            self.get_logger().info('✅ Movement Complete!')
            return True
        else:
            self.get_logger().error(f'❌ Movement Failed with Error Code: {result.error_code.val}')
            return False

def main(args=None):
    rclpy.init(args=args)
    bot = WaveBot()

    # --- THE WAVE SEQUENCE ---
    
    # 1. Go Home (All Zeros)
    # (Angle order: J1, J2, J3, J4, J5, J6) in RADIANS
    bot.move_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # 2. Wave Left (J1 = 0.5 rad)
    # Note: 0.5 rad is approx 30 degrees
    bot.move_joints([0.5, 0.0, 0.0, 0.0, 0.0, 0.0])

    # 3. Wave Right (J1 = -0.5 rad)
    bot.move_joints([-0.5, 0.0, 0.0, 0.0, 0.0, 0.0])

    # 4. Back Home
    bot.move_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()