#!/usr/bin/env python3
"""Read real camera intrinsics from OAK-D-SR calibration data."""

import depthai as dai
import json

with dai.Pipeline(dai.Device()) as pipeline:
    device = pipeline.getDefaultDevice()
    calib = device.readCalibration()

    for socket in [dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C]:
        name = socket.name
        intrinsics = calib.getCameraIntrinsics(socket, 640, 480)
        distortion = calib.getDistortionCoefficients(socket)
        fov = calib.getFov(socket)

        print(f"\n=== {name} (640x480) ===")
        print(f"FOV: {fov:.1f} degrees")
        print(f"Intrinsics matrix:")
        for row in intrinsics:
            print(f"  [{row[0]:.2f}, {row[1]:.2f}, {row[2]:.2f}]")
        print(f"Distortion: {[f'{d:.6f}' for d in distortion[:5]]}")
