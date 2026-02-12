#!/usr/bin/env python3
"""
Adds a table collision plane to the MoveIt2 planning scene.

The table surface sits 3cm below the robot base (base_plate).
Publishes once and exits — the planning scene persists in move_group.

Usage:
    python3 add_table_collision.py
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import time


def main():
    rclpy.init()
    node = Node("add_table_collision")

    pub = node.create_publisher(CollisionObject, "/collision_object", 10)

    # Wait for move_group to be subscribed
    node.get_logger().info("Waiting for move_group to subscribe to /collision_object...")
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.5)

    # Build the table collision object
    table = CollisionObject()
    table.header = Header()
    table.header.frame_id = "base_plate"
    table.header.stamp = node.get_clock().now().to_msg()
    table.id = "table"
    table.operation = CollisionObject.ADD

    # Table dimensions: 1m x 1m x 2cm thick slab
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [1.0, 1.0, 0.02]  # x, y, z

    # Position: top surface at z = -0.03 (3cm below base_plate)
    # Box center = -0.03 - half_thickness = -0.03 - 0.01 = -0.04
    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = -0.04
    pose.orientation.w = 1.0

    table.primitives.append(box)
    table.primitive_poses.append(pose)

    # Publish a few times to ensure delivery
    for i in range(5):
        table.header.stamp = node.get_clock().now().to_msg()
        pub.publish(table)
        node.get_logger().info(f"Published table collision object ({i+1}/5)")
        time.sleep(0.2)

    node.get_logger().info(
        "Table added to planning scene: 1m x 1m x 2cm, "
        "top surface at z = -0.03m (3cm below base_plate)"
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
