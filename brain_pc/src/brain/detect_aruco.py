#!/usr/bin/env python3
"""Detect any ArUco marker and identify its dictionary and ID."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


DICTIONARIES = {
    '4X4_50': cv2.aruco.DICT_4X4_50,
    '4X4_100': cv2.aruco.DICT_4X4_100,
    '4X4_250': cv2.aruco.DICT_4X4_250,
    '4X4_1000': cv2.aruco.DICT_4X4_1000,
    '5X5_50': cv2.aruco.DICT_5X5_50,
    '5X5_100': cv2.aruco.DICT_5X5_100,
    '5X5_250': cv2.aruco.DICT_5X5_250,
    '5X5_1000': cv2.aruco.DICT_5X5_1000,
    '6X6_50': cv2.aruco.DICT_6X6_50,
    '6X6_100': cv2.aruco.DICT_6X6_100,
    '6X6_250': cv2.aruco.DICT_6X6_250,
    '6X6_1000': cv2.aruco.DICT_6X6_1000,
    '7X7_50': cv2.aruco.DICT_7X7_50,
    '7X7_100': cv2.aruco.DICT_7X7_100,
    '7X7_250': cv2.aruco.DICT_7X7_250,
    '7X7_1000': cv2.aruco.DICT_7X7_1000,
    'ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.latest_image = None

        self.image_sub = self.create_subscription(
            Image, '/oak/left/image_raw', self.image_callback, 10
        )
        self.get_logger().info('Waiting for camera image... Hold marker in view.')

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg)

    def run(self):
        cv2.namedWindow('ArUco Detect', cv2.WINDOW_NORMAL)
        found_any = False

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.latest_image is None:
                continue

            image = self.latest_image.copy()
            display = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if len(image.shape) == 2 else image.copy()

            for name, dict_id in DICTIONARIES.items():
                dictionary = cv2.aruco.Dictionary_get(dict_id)
                params = cv2.aruco.DetectorParameters_create()
                corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=params)

                if ids is not None:
                    found_any = True
                    cv2.aruco.drawDetectedMarkers(display, corners, ids)
                    for marker_id in ids.flatten():
                        msg = f'FOUND: Dictionary={name}, ID={marker_id}'
                        self.get_logger().info(msg)
                        cv2.putText(display, msg, (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if not found_any:
                cv2.putText(display, 'No markers detected - hold marker in view',
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow('ArUco Detect', display)
            found_any = False

            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
