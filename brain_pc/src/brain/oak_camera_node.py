#!/usr/bin/env python3
"""
ROS 2 node for OAK-D-SR camera using depthai v3 API.
Publishes mono camera feeds as ROS Image topics.

NOTE: Default resolution is 640x400 (native 16:10 aspect ratio of the
1280x800 sensor). This ensures getCameraIntrinsics() values are correct.
Using 640x480 causes depthai to crop the image to change aspect ratio,
but getCameraIntrinsics() assumes scaling — giving wrong focal length.
"""

import depthai as dai
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class OakCameraNode(Node):
    def __init__(self):
        super().__init__('oak_camera_node')

        self.declare_parameter('fps', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 400)

        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value

        self.bridge = CvBridge()

        # Publishers
        self.left_pub = self.create_publisher(Image, '/oak/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/oak/right/image_raw', 10)

        self.get_logger().info('Starting OAK-D-SR camera...')
        self.run_camera()

    def run_camera(self):
        with dai.Pipeline(dai.Device()) as pipeline:
            device = pipeline.getDefaultDevice()
            connected = device.getConnectedCameraFeatures()
            self.get_logger().info(f'Connected cameras: {[c.socket.name for c in connected]}')

            res = (self.width, self.height)

            # CAM_B (left stereo)
            camB = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            outB = camB.requestOutput(res)
            qB = outB.createOutputQueue(maxSize=4, blocking=False)

            # CAM_C (right stereo)
            camC = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            outC = camC.requestOutput(res)
            qC = outC.createOutputQueue(maxSize=4, blocking=False)

            pipeline.start()
            self.get_logger().info(f'OAK-D-SR streaming at {self.width}x{self.height}')

            while rclpy.ok():
                # Left camera
                left_data = qB.tryGet()
                if left_data is not None:
                    left_frame = left_data.getCvFrame()
                    if len(left_frame.shape) == 2:
                        left_msg = self.bridge.cv2_to_imgmsg(left_frame, encoding='mono8')
                    else:
                        left_msg = self.bridge.cv2_to_imgmsg(left_frame, encoding='bgr8')
                    left_msg.header.stamp = self.get_clock().now().to_msg()
                    left_msg.header.frame_id = 'oak_left_camera_optical_frame'
                    self.left_pub.publish(left_msg)

                # Right camera
                right_data = qC.tryGet()
                if right_data is not None:
                    right_frame = right_data.getCvFrame()
                    if len(right_frame.shape) == 2:
                        right_msg = self.bridge.cv2_to_imgmsg(right_frame, encoding='mono8')
                    else:
                        right_msg = self.bridge.cv2_to_imgmsg(right_frame, encoding='bgr8')
                    right_msg.header.stamp = self.get_clock().now().to_msg()
                    right_msg.header.frame_id = 'oak_right_camera_optical_frame'
                    self.right_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OakCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
