#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Float32MultiArray
import yaml
import os


# ----------------------------------------------------------
# Find workspace root
# ----------------------------------------------------------
def find_workspace_root(start_path):

    path = start_path

    while True:

        if os.path.exists(os.path.join(path, "src")):
            return path

        parent = os.path.dirname(path)

        if parent == path:
            return None

        path = parent


# ----------------------------------------------------------
# Node
# ----------------------------------------------------------
class ConstantsPublisher(Node):

    def __init__(self):
        super().__init__('arm_yaml_constants_publisher')

        self.constants_pub = self.create_publisher(
            Float32MultiArray,
            '/arm_calibrations',
            10
        )

        self.create_timer(1.0, self.publish_constants)

        self.yaml_path = self.locate_yaml()

        if self.yaml_path:
            self.get_logger().info(
                f"Using calibration file: {self.yaml_path}"
            )
        else:
            self.get_logger().error(
                "ERROR: arm_constants.yaml NOT FOUND in SRC!"
            )

        self.clock = Clock()



    # ------------------------------------------------------
    # Locate YAML
    # ------------------------------------------------------
    def locate_yaml(self):

        this_file = os.path.dirname(os.path.abspath(__file__))
        root = find_workspace_root(this_file)

        if root is None:
            return None

        yaml_path = os.path.join(
            root,
            "src",
            "mycobot_320pi_controller",
            "calibrations",
            "arm_constants.yaml"
        )

        return yaml_path if os.path.exists(yaml_path) else None



    # ------------------------------------------------------
    # Load Constants
    # ------------------------------------------------------
    def load_constants(self):

        if self.yaml_path is None:
            return None

        try:
            data = yaml.safe_load(open(self.yaml_path))

            return [

                data.get('arm_joint_1_radians_to_degrees', 0.0),
                data.get('arm_joint_2_radians_to_degrees', 0.0),
                data.get('arm_joint_3_radians_to_degrees', 0.0),
                data.get('arm_joint_4_radians_to_degrees', 0.0),
                data.get('arm_joint_5_radians_to_degrees', 0.0),
                data.get('arm_joint_6_radians_to_degrees', 0.0),

                data.get('arm_joint_1_degrees_to_radians', 0.0),
                data.get('arm_joint_2_degrees_to_radians', 0.0),
                data.get('arm_joint_3_degrees_to_radians', 0.0),
                data.get('arm_joint_4_degrees_to_radians', 0.0),
                data.get('arm_joint_5_degrees_to_radians', 0.0),
                data.get('arm_joint_6_degrees_to_radians', 0.0),

                data.get('gripper_radians_to_degrees', 0.0),
                data.get('gripper_degrees_to_radians', 0.0),
            ]

        except Exception as e:
            self.get_logger().warn(f"Failed to load YAML: {e}")
            return None



    # ------------------------------------------------------
    # Publish
    # ------------------------------------------------------
    def publish_constants(self):

        constants = self.load_constants()
        if constants is None:
            self.get_logger().warn("No valid constants.")
            return

        msg = Float32MultiArray()
        msg.data = constants
        self.constants_pub.publish(msg)



# ----------------------------------------------------------
# Main with clean shutdown
# ----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ConstantsPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        # rclpy.shutdown()



if __name__ == "__main__":
    main()
