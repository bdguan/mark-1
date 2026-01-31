#!/usr/bin/env python3
"""
Hand-eye calibration (eye-to-hand) using ArUco marker on gripper.

Usage:
  1. Launch robot + OAK camera node
  2. ArUco marker should be on top of gripper facing up
  3. Run this script
  4. Use RViz to move the arm to different poses
  5. Press SPACE to capture at each pose (aim for 15+)
  6. Press Q to finish and compute calibration
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf2_ros
import time
import json
import os

# --- ARUCO CONFIG ---
ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_ID = 1
MARKER_SIZE = 0.0380  # 38.0mm in meters


class HandEyeCalibrator(Node):
    def __init__(self):
        super().__init__('hand_eye_calibrator')

        self.bridge = CvBridge()
        self.latest_image = None

        # TF listener to get end-effector pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to OAK-D left camera
        self.image_sub = self.create_subscription(
            Image, '/oak/left/image_raw', self.image_callback, 10
        )

        self.get_logger().info(f'Hand-eye calibrator ready')
        self.get_logger().info(f'ArUco: 4X4_50, ID={MARKER_ID}, size={MARKER_SIZE*1000:.1f}mm')
        self.get_logger().info(f'')
        self.get_logger().info(f'Instructions:')
        self.get_logger().info(f'  1. ArUco marker on top of gripper facing camera')
        self.get_logger().info(f'  2. Move arm to a pose using RViz')
        self.get_logger().info(f'  3. Press SPACE in the OpenCV window to capture')
        self.get_logger().info(f'  4. Repeat for 15+ poses (vary position AND orientation)')
        self.get_logger().info(f'  5. Press Q to compute calibration')
        self.get_logger().info(f'')
        self.get_logger().info(f'Tips:')
        self.get_logger().info(f'  - Move to all corners of the workspace')
        self.get_logger().info(f'  - Vary the wrist tilt at each position')
        self.get_logger().info(f'  - Keep the marker fully visible in frame')
        self.get_logger().info(f'  - Wait for arm to stop moving before capturing')

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg)

    def get_end_effector_pose(self):
        """Get current end-effector pose relative to base"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_plate', 'link_6', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return transform
        except Exception as e:
            self.get_logger().warn(f'Could not get TF: {e}')
            return None

    def transform_to_matrix(self, transform):
        """Convert ROS transform to 4x4 homogeneous matrix"""
        t = transform.transform.translation
        r = transform.transform.rotation
        qx, qy, qz, qw = r.x, r.y, r.z, r.w

        rot = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])

        mat = np.eye(4)
        mat[:3, :3] = rot
        mat[:3, 3] = [t.x, t.y, t.z]
        return mat

    def rotation_matrix_to_quaternion(self, R):
        """Convert 3x3 rotation matrix to quaternion (x, y, z, w)"""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        return qx, qy, qz, qw

    def run_calibration(self):
        """Main calibration loop"""
        self.get_logger().info('Waiting for camera image...')
        while self.latest_image is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        image = self.latest_image
        h, w = image.shape[:2]

        # OAK-D-SR CAM_B intrinsics for 640x400 (native 16:10 aspect ratio).
        # NOTE: getCameraIntrinsics(640, 480) returns wrong values because
        # requestOutput((640, 480)) crops to change aspect ratio from 16:10 to
        # 4:3, but the API assumes simple scaling. Use 640x400 to avoid this.
        camera_matrix = np.array([
            [399.19, 0.00, 309.10],
            [0.00, 399.16, 201.20],
            [0.00, 0.00, 1.00]
        ], dtype=np.float64)
        dist_coeffs = np.zeros(5)

        self.get_logger().info(f'Image size: {w}x{h}, focal length: {camera_matrix[0,0]:.1f}px')
        if w != 640 or h != 400:
            self.get_logger().warn(f'Expected 640x400 but got {w}x{h} — intrinsics may be wrong!')
        self.get_logger().info(f'>>> Move the arm and press SPACE to capture <<<')

        # ArUco setup
        dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT)
        params = cv2.aruco.DetectorParameters_create()

        robot_poses = []
        marker_rotations = []
        marker_translations = []

        cv2.namedWindow('Calibration', cv2.WINDOW_NORMAL)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.latest_image is None:
                continue

            image = self.latest_image.copy()
            display = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if len(image.shape) == 2 else image.copy()

            # Detect ArUco marker
            corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=params)

            marker_found = False
            marker_corners = None

            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id == MARKER_ID:
                        marker_found = True
                        marker_corners = corners[i]
                        break
                cv2.aruco.drawDetectedMarkers(display, corners, ids)

            n = len(robot_poses)
            if marker_found:
                status = f'MARKER DETECTED | Captures: {n} | SPACE=capture Q=finish'
                color = (0, 255, 0)
            else:
                status = f'No marker | Captures: {n} | SPACE=capture Q=finish'
                color = (0, 0, 255)

            cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.imshow('Calibration', display)

            key = cv2.waitKey(30) & 0xFF

            if key == ord(' '):
                if not marker_found:
                    self.get_logger().warn('Cannot capture - marker not detected!')
                    continue

                # Get robot pose
                ee_transform = self.get_end_effector_pose()
                if ee_transform is None:
                    self.get_logger().warn('Cannot capture - no TF available!')
                    continue

                # --- Diagnostics: marker apparent size and tilt ---
                pts = marker_corners[0]
                edge_lengths = [np.linalg.norm(pts[(i+1)%4] - pts[i]) for i in range(4)]
                avg_edge_px = np.mean(edge_lengths)
                edge_ratio = max(edge_lengths) / min(edge_lengths)
                # Marker center in image
                cx_marker = np.mean(pts[:, 0])
                cy_marker = np.mean(pts[:, 1])
                # Distance from principal point (image center)
                dist_from_center = np.sqrt((cx_marker - camera_matrix[0,2])**2 + (cy_marker - camera_matrix[1,2])**2)

                # Estimate marker pose in camera frame
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    marker_corners, MARKER_SIZE, camera_matrix, dist_coeffs
                )
                rvec = rvecs[0].flatten()
                tvec = tvecs[0].flatten()
                rot_mat, _ = cv2.Rodrigues(rvec)

                # Tilt angle
                marker_normal = rot_mat[:, 2]
                tilt_angle = np.degrees(np.arccos(np.clip(abs(marker_normal[2]), -1, 1)))

                # Simple pinhole estimate for comparison
                fx = camera_matrix[0, 0]
                pinhole_dist = (MARKER_SIZE * fx) / avg_edge_px

                opencv_dist = np.linalg.norm(tvec)

                self.get_logger().info(
                    f'  edges(px): [{", ".join(f"{e:.1f}" for e in edge_lengths)}] '
                    f'avg={avg_edge_px:.1f} ratio={edge_ratio:.2f}'
                )
                self.get_logger().info(
                    f'  tilt={tilt_angle:.1f}° | center=({cx_marker:.0f},{cy_marker:.0f}) '
                    f'dist_from_center={dist_from_center:.0f}px'
                )
                self.get_logger().info(
                    f'  pinhole={pinhole_dist:.3f}m | solvePnP={opencv_dist:.3f}m | '
                    f'diff={((opencv_dist/pinhole_dist)-1)*100:+.1f}%'
                )
                self.get_logger().info(
                    f'  tvec: x={tvec[0]:.4f} y={tvec[1]:.4f} z={tvec[2]:.4f}'
                )

                # Store
                ee_matrix = self.transform_to_matrix(ee_transform)
                robot_poses.append(ee_matrix)
                marker_rotations.append(rot_mat)
                marker_translations.append(tvec.reshape(3, 1))

                n = len(robot_poses)
                self.get_logger().info(
                    f'Capture #{n}: '
                    f'EE=({ee_matrix[0,3]:.3f}, {ee_matrix[1,3]:.3f}, {ee_matrix[2,3]:.3f}) '
                    f'Marker dist={opencv_dist:.3f}m'
                )
                if n < 15:
                    self.get_logger().info(f'  Need {15 - n} more. Move to a different pose.')
                else:
                    self.get_logger().info(f'  {n} captures. You can keep going or press Q to finish.')

            elif key == ord('q'):
                break

        cv2.destroyAllWindows()

        # --- SOLVE ---
        if len(robot_poses) < 4:
            self.get_logger().error(f'Need at least 4 captures, got {len(robot_poses)}. Aborting.')
            return

        self.get_logger().info(f'')
        self.get_logger().info(f'Computing hand-eye calibration with {len(robot_poses)} captures...')

        R_gripper2base = [p[:3, :3] for p in robot_poses]
        t_gripper2base = [p[:3, 3].reshape(3, 1) for p in robot_poses]
        R_target2cam = marker_rotations
        t_target2cam = marker_translations

        methods = {
            'TSAI': cv2.CALIB_HAND_EYE_TSAI,
            'PARK': cv2.CALIB_HAND_EYE_PARK,
            'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
            'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
            'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
        }

        results = {}
        for name, method in methods.items():
            try:
                R_cam2base, t_cam2base = cv2.calibrateHandEye(
                    R_gripper2base, t_gripper2base,
                    R_target2cam, t_target2cam,
                    method=method
                )
                results[name] = (R_cam2base, t_cam2base)
                self.get_logger().info(
                    f'  {name}: t=({t_cam2base[0,0]:.4f}, {t_cam2base[1,0]:.4f}, {t_cam2base[2,0]:.4f})'
                )
            except Exception as e:
                self.get_logger().warn(f'  {name} failed: {e}')

        if not results:
            self.get_logger().error('All methods failed!')
            return

        best = 'PARK' if 'PARK' in results else list(results.keys())[0]
        R_cam2base, t_cam2base = results[best]
        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(R_cam2base)

        self.get_logger().info(f'')
        self.get_logger().info(f'Best result ({best}):')
        self.get_logger().info(f'  Translation: x={t_cam2base[0,0]:.4f}, y={t_cam2base[1,0]:.4f}, z={t_cam2base[2,0]:.4f}')
        self.get_logger().info(f'  Quaternion:  qx={qx:.6f}, qy={qy:.6f}, qz={qz:.6f}, qw={qw:.6f}')

        cmd = (
            f'ros2 run tf2_ros static_transform_publisher '
            f'--x {t_cam2base[0,0]:.6f} --y {t_cam2base[1,0]:.6f} --z {t_cam2base[2,0]:.6f} '
            f'--qx {qx:.6f} --qy {qy:.6f} --qz {qz:.6f} --qw {qw:.6f} '
            f'--frame-id base_plate --child-frame-id oak_camera_link'
        )

        self.get_logger().info(f'')
        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'COPY THIS COMMAND:')
        self.get_logger().info(cmd)
        self.get_logger().info(f'{"="*60}')

        # Save to file
        save_path = os.path.join(os.path.dirname(__file__), 'oak_calibration.json')
        cal_data = {
            'method': best,
            'translation': {'x': float(t_cam2base[0,0]), 'y': float(t_cam2base[1,0]), 'z': float(t_cam2base[2,0])},
            'quaternion': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)},
            'num_captures': len(robot_poses),
            'static_tf_command': cmd,
            'all_methods': {
                name: {'translation': r[1].flatten().tolist()}
                for name, r in results.items()
            }
        }
        with open(save_path, 'w') as f:
            json.dump(cal_data, f, indent=2)
        self.get_logger().info(f'Saved to: {save_path}')


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrator()
    try:
        node.run_calibration()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
