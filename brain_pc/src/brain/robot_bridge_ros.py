#!/usr/bin/env python3
"""
ROS2 WebSocket Bridge for MoveIt2 control.
Connects to a cloud WebSocket server as a robot client, receives movement
commands, plans+executes via MoveIt2 (collision avoidance), and streams
joint states back at ~10Hz.

Supported command types:
  - command:      Discrete WASD/UJ/open/close movements
  - pose_target:  Absolute Cartesian pose (x,y,z,qx,qy,qz,qw)
  - joint_target: Direct joint angle targets (joint_1..joint_6)

Usage:
    export WS_URL=ws://your-server:8080
    export ROBOT_TUNNEL_SECRET=your-secret
    python3 robot_bridge_ros.py
"""
import os
import json
import math
import time
import asyncio
import threading
from typing import Optional, Dict, Any, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tf2_ros
from tf2_ros import TransformException


# =============================================================================
# CONFIGURATION (from URDF/SRDF exploration)
# =============================================================================

GROUP_NAME = "arm"
BASE_LINK = "base_plate"
END_EFFECTOR_LINK = "link_6"
ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

GRIPPER_JOINT_NAME = "left_jaw_joint_1"
GRIPPER_OPEN_POS = 0.0
GRIPPER_CLOSED_POS = -1.11
GRIPPER_THRESHOLD = -0.3  # Below this = closed

# Movement step size in meters
STEP_SIZE_M = 0.020  # 20mm

# Joint limits (radians) — used for validation before sending to MoveIt
JOINT_LIMITS = {
    "joint_1": (-2.93, 2.93),
    "joint_2": (-2.35, 2.35),
    "joint_3": (-2.53, 2.53),
    "joint_4": (-2.53, 2.53),
    "joint_5": (-2.93, 2.93),
    "joint_6": (-3.14, 3.14),
}

# WebSocket configuration
DEFAULT_WS_URL = "ws://localhost:8080"
DEFAULT_ROBOT_SECRET = "dev-robot-secret"

# Reconnect backoff schedule (seconds)
RECONNECT_DELAYS = [1, 2, 4, 8, 15, 30]

# Joint state streaming rate
STREAM_RATE_HZ = 10.0


class RobotBridgeRos(Node):
    """ROS2 node bridging WebSocket commands to MoveIt2."""

    def __init__(self):
        super().__init__("robot_bridge_ros")

        # State tracking
        self._busy = False
        self._busy_lock = threading.Lock()
        self._last_command_success = True
        self._last_error = ""  # type: str
        self._gripper_state = "open"  # "open" or "closed"

        # Joint state cache
        self._joint_positions = {}  # type: Dict[str, float]
        self._last_streamed_joints = {}  # type: Dict[str, float]
        self._last_streamed_gripper = ""
        self._last_streamed_busy = None  # type: Optional[bool]

        # TF2 buffer for end-effector pose lookup
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Callback group for concurrent callbacks
        self._cb_group = ReentrantCallbackGroup()

        # Action clients
        self._move_group_client = ActionClient(
            self, MoveGroup, "move_action", callback_group=self._cb_group
        )
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            callback_group=self._cb_group,
        )

        # Joint state subscriber
        self._joint_sub = self.create_subscription(
            JointState,
            "joint_states",
            self._joint_state_callback,
            10,
            callback_group=self._cb_group,
        )

        # WebSocket connection
        self._ws = None
        self._ws_connected = False
        self._ws_loop = None  # type: Optional[asyncio.AbstractEventLoop]
        self._ws_send_queue = asyncio.Queue()  # type: asyncio.Queue

        # Executor thread for blocking MoveIt calls
        self._executor_thread = None  # type: Optional[threading.Thread]

        self.get_logger().info("RobotBridgeRos initialized")

    # =========================================================================
    # Joint State Handling
    # =========================================================================

    def _joint_state_callback(self, msg: JointState):
        """Cache joint positions and update gripper state."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_positions[name] = msg.position[i]

        # Update gripper state from joint position
        if GRIPPER_JOINT_NAME in self._joint_positions:
            pos = self._joint_positions[GRIPPER_JOINT_NAME]
            self._gripper_state = "closed" if pos < GRIPPER_THRESHOLD else "open"

    def _get_current_ee_pose(self) -> Optional[Pose]:
        """Get current end-effector pose via TF2."""
        try:
            transform = self._tf_buffer.lookup_transform(
                BASE_LINK, END_EFFECTOR_LINK, rclpy.time.Time()
            )
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
        except TransformException as ex:
            self.get_logger().warn(f"Could not get EE pose: {ex}")
            return None

    # =========================================================================
    # MoveIt2 Execution (blocking via Event, run in thread)
    # =========================================================================

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        """
        Wait for a future to complete using threading.Event.
        This avoids spin_until_future_complete which conflicts with main executor.
        """
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        return event.wait(timeout=timeout_sec)

    def _execute_pose_target(self, target_pose: Pose) -> bool:
        """Plan and execute to a pose target via MoveGroup action. Blocking."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available")
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = GROUP_NAME
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        goal_msg.request.num_planning_attempts = 3

        # Create pose constraint
        constraints = Constraints()

        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = BASE_LINK
        pos_constraint.link_name = END_EFFECTOR_LINK
        pos_constraint.weight = 1.0

        # Bounding region (small sphere around target)
        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.001]  # 1mm tolerance sphere
        bounding_volume.primitives.append(primitive)
        primitive_pose = Pose()
        primitive_pose.position = target_pose.position
        primitive_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(primitive_pose)
        pos_constraint.constraint_region = bounding_volume
        constraints.position_constraints.append(pos_constraint)

        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = BASE_LINK
        orient_constraint.link_name = END_EFFECTOR_LINK
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)

        goal_msg.request.goal_constraints = [constraints]

        self.get_logger().info(
            f"Planning to pose: ({target_pose.position.x:.3f}, "
            f"{target_pose.position.y:.3f}, {target_pose.position.z:.3f})"
        )

        # Send goal and wait via Event (not spin_until_future_complete)
        send_future = self._move_group_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_future, timeout_sec=10.0):
            self.get_logger().error("Timeout sending goal")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=60.0):
            self.get_logger().error("Timeout waiting for execution")
            return False

        result = result_future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Movement complete")
            return True
        else:
            self.get_logger().error(
                f"Movement failed: error code {result.error_code.val}"
            )
            return False

    def _execute_joint_target(self, joints_dict: Dict[str, float]) -> bool:
        """
        Plan and execute to a joint angle target via MoveGroup action. Blocking.

        joints_dict: { "joint_1": 0.0, "joint_2": 0.35, ... }
        Only the joints present in the dict will be constrained; missing joints
        keep their current values.
        """
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available")
            return False

        # Validate and clamp joint values
        target_joints = {}
        for name in ARM_JOINT_NAMES:
            if name in joints_dict:
                val = float(joints_dict[name])
                lo, hi = JOINT_LIMITS.get(name, (-3.14, 3.14))
                val = max(lo, min(hi, val))
                target_joints[name] = val
            elif name in self._joint_positions:
                # Use current position for unspecified joints
                target_joints[name] = self._joint_positions[name]

        if not target_joints:
            self.get_logger().warn("No valid joints in joint_target")
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = GROUP_NAME
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        goal_msg.request.num_planning_attempts = 3

        # Create joint constraints
        constraints = Constraints()
        for name, value in target_joints.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01  # ~0.6 degrees
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints = [constraints]

        joint_str = ", ".join(
            f"{n}={v:.3f}" for n, v in sorted(target_joints.items())
        )
        self.get_logger().info(f"Planning to joints: {joint_str}")

        # Send goal and wait
        send_future = self._move_group_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_future, timeout_sec=10.0):
            self.get_logger().error("Timeout sending joint goal")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Joint goal rejected by MoveGroup")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=60.0):
            self.get_logger().error("Timeout waiting for joint execution")
            return False

        result = result_future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Joint target reached")
            return True
        else:
            self.get_logger().error(
                f"Joint target failed: error code {result.error_code.val}"
            )
            return False

    def _execute_gripper(self, open_gripper: bool) -> bool:
        """Open or close gripper. Blocking."""
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper controller not available")
            return False

        position = GRIPPER_OPEN_POS if open_gripper else GRIPPER_CLOSED_POS
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT_NAME]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=1, nanosec=0)
        goal.trajectory.points = [point]

        self.get_logger().info(
            f"Gripper -> {'open' if open_gripper else 'closed'}"
        )

        send_future = self._gripper_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, timeout_sec=5.0):
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        return self._wait_for_future(result_future, timeout_sec=10.0)

    # =========================================================================
    # Command Processing
    # =========================================================================

    def _process_discrete_command(self, command: str) -> bool:
        """Process WASD/UJ/open/close commands. Returns success."""
        current_pose = self._get_current_ee_pose()
        if current_pose is None:
            self._last_error = "Could not get current end-effector pose"
            return False

        # Gripper commands
        if command == "open":
            return self._execute_gripper(open_gripper=True)
        elif command == "close":
            return self._execute_gripper(open_gripper=False)

        # Movement commands (base frame)
        # W/S = +/- Y, A/D = -/+ X, U/J = +/- Z
        dx, dy, dz = 0.0, 0.0, 0.0
        if command == "w":
            dy = STEP_SIZE_M
        elif command == "s":
            dy = -STEP_SIZE_M
        elif command == "a":
            dx = -STEP_SIZE_M
        elif command == "d":
            dx = STEP_SIZE_M
        elif command == "u":
            dz = STEP_SIZE_M
        elif command == "j":
            dz = -STEP_SIZE_M
        else:
            self._last_error = f"Unknown command: {command}"
            return False

        # Create target pose with offset
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x + dx
        target_pose.position.y = current_pose.position.y + dy
        target_pose.position.z = current_pose.position.z + dz
        target_pose.orientation = current_pose.orientation

        return self._execute_pose_target(target_pose)

    def _process_pose_target(self, pose_data: Dict[str, float]) -> bool:
        """Process absolute pose target. Returns success."""
        target_pose = Pose()
        target_pose.position.x = pose_data.get("x", 0.0)
        target_pose.position.y = pose_data.get("y", 0.0)
        target_pose.position.z = pose_data.get("z", 0.0)
        target_pose.orientation.x = pose_data.get("qx", 0.0)
        target_pose.orientation.y = pose_data.get("qy", 0.0)
        target_pose.orientation.z = pose_data.get("qz", 0.0)
        target_pose.orientation.w = pose_data.get("qw", 1.0)

        return self._execute_pose_target(target_pose)

    def _process_joint_target(self, joints_data: Dict[str, float]) -> bool:
        """Process direct joint angle target. Returns success."""
        return self._execute_joint_target(joints_data)

    def _run_command_in_thread(self, command_type: str, data: Any):
        """Run a command in a background thread to not block WS loop."""

        def execute():
            with self._busy_lock:
                self._busy = True

            try:
                if command_type == "command":
                    success = self._process_discrete_command(data)
                elif command_type == "pose_target":
                    success = self._process_pose_target(data)
                elif command_type == "joint_target":
                    success = self._process_joint_target(data)
                else:
                    success = False
                    self._last_error = f"Unknown command type: {command_type}"

                self._last_command_success = success
                if not success and not self._last_error:
                    self._last_error = "Command execution failed"

            except Exception as e:
                self._last_command_success = False
                self._last_error = str(e)
                self.get_logger().error(f"Command error: {e}")

            finally:
                with self._busy_lock:
                    self._busy = False
                # Send state update after command completes
                self._queue_state_update()

        thread = threading.Thread(target=execute, daemon=True)
        thread.start()

    def _queue_state_update(self):
        """Queue a state message to be sent via WebSocket."""
        state_msg = {
            "type": "state",
            "busy": self._busy,
            "lastCommandSuccess": self._last_command_success,
            "gripperState": self._gripper_state,
        }
        if not self._last_command_success and self._last_error:
            state_msg["error"] = self._last_error

        if self._ws_loop is not None:
            asyncio.run_coroutine_threadsafe(
                self._ws_send_queue.put(state_msg), self._ws_loop
            )

    # =========================================================================
    # WebSocket Connection
    # =========================================================================

    async def _ws_sender(self):
        """Coroutine to send queued messages."""
        while True:
            msg = await self._ws_send_queue.get()
            if self._ws is not None and self._ws_connected:
                try:
                    await self._ws.send(json.dumps(msg))
                except Exception as e:
                    self.get_logger().warn(f"WS send error: {e}")

    async def _ws_receiver(self):
        """Coroutine to receive and process messages."""
        try:
            async for message in self._ws:
                try:
                    data = json.loads(message)
                    msg_type = data.get("type", "")

                    if msg_type == "ping":
                        await self._ws_send_queue.put({"type": "pong"})

                    elif msg_type == "command":
                        with self._busy_lock:
                            if self._busy:
                                await self._ws_send_queue.put(
                                    {
                                        "type": "state",
                                        "busy": True,
                                        "error": "still executing",
                                    }
                                )
                                continue
                        cmd = data.get("command", "")
                        self._last_error = ""
                        self._run_command_in_thread("command", cmd)

                    elif msg_type == "pose_target":
                        with self._busy_lock:
                            if self._busy:
                                await self._ws_send_queue.put(
                                    {
                                        "type": "state",
                                        "busy": True,
                                        "error": "still executing",
                                    }
                                )
                                continue
                        pose = data.get("pose", {})
                        self._last_error = ""
                        self._run_command_in_thread("pose_target", pose)

                    elif msg_type == "joint_target":
                        with self._busy_lock:
                            if self._busy:
                                await self._ws_send_queue.put(
                                    {
                                        "type": "state",
                                        "busy": True,
                                        "error": "still executing",
                                    }
                                )
                                continue
                        joints = data.get("joints", {})
                        self._last_error = ""
                        self._run_command_in_thread("joint_target", joints)

                    else:
                        self.get_logger().debug(
                            f"Unknown message type: {msg_type}"
                        )

                except json.JSONDecodeError:
                    self.get_logger().warn(f"Invalid JSON: {message}")

        except Exception as e:
            self.get_logger().warn(f"WS receiver error: {e}")

    async def _stream_joint_states(self):
        """Stream joint states at ~10Hz, only when values change."""
        interval = 1.0 / STREAM_RATE_HZ
        while True:
            await asyncio.sleep(interval)
            if not self._ws_connected or not self._joint_positions:
                continue

            # Check if anything changed
            joints_changed = False
            for name in ARM_JOINT_NAMES:
                if name in self._joint_positions:
                    current = round(self._joint_positions[name], 4)
                    last = self._last_streamed_joints.get(name)
                    if last is None or abs(current - last) > 0.0001:
                        joints_changed = True
                        break

            gripper_changed = self._gripper_state != self._last_streamed_gripper
            busy_changed = self._busy != self._last_streamed_busy

            if not (joints_changed or gripper_changed or busy_changed):
                continue

            # Build and send message
            joints_dict = {}
            for name in ARM_JOINT_NAMES:
                if name in self._joint_positions:
                    joints_dict[name] = round(self._joint_positions[name], 4)

            msg = {
                "type": "joint_states",
                "joints": joints_dict,
                "gripperState": self._gripper_state,
                "busy": self._busy,
                "timestamp": time.time(),
            }
            await self._ws_send_queue.put(msg)

            # Update last sent values
            self._last_streamed_joints = dict(joints_dict)
            self._last_streamed_gripper = self._gripper_state
            self._last_streamed_busy = self._busy

    async def _connect_ws(self):
        """Connect to WebSocket server with auto-reconnect."""
        import websockets
        from urllib.parse import quote

        ws_url = os.environ.get("WS_URL", DEFAULT_WS_URL)
        robot_secret = os.environ.get("ROBOT_TUNNEL_SECRET", DEFAULT_ROBOT_SECRET)
        full_url = f"{ws_url}?robot_secret={quote(robot_secret, safe='')}"

        self.get_logger().info(f"WS_URL env: '{ws_url}'")
        self.get_logger().info(f"ROBOT_TUNNEL_SECRET env: '{robot_secret}'")
        self.get_logger().info(f"Full connect URL: '{full_url}'")

        reconnect_idx = 0

        while True:
            try:
                self.get_logger().info(f"Connecting to {ws_url}...")
                async with websockets.connect(full_url) as ws:
                    self._ws = ws
                    self._ws_connected = True
                    reconnect_idx = 0
                    self.get_logger().info("WebSocket connected")

                    # Run receiver and streamer concurrently.
                    # When either exits (e.g. connection drop), cancel the other
                    # so reconnect logic triggers.
                    receiver = asyncio.create_task(self._ws_receiver())
                    streamer = asyncio.create_task(self._stream_joint_states())
                    done, pending = await asyncio.wait(
                        [receiver, streamer],
                        return_when=asyncio.FIRST_COMPLETED,
                    )
                    for task in pending:
                        task.cancel()
                    # Re-raise any exception from the completed task
                    for task in done:
                        if task.exception():
                            raise task.exception()

            except Exception as e:
                self._ws_connected = False
                self._ws = None
                delay = RECONNECT_DELAYS[
                    min(reconnect_idx, len(RECONNECT_DELAYS) - 1)
                ]
                self.get_logger().warn(
                    f"WS connection failed: {e}. Retry in {delay}s"
                )
                await asyncio.sleep(delay)
                reconnect_idx += 1

    async def _ws_main(self):
        """Main async entry point for WebSocket thread."""
        self._ws_loop = asyncio.get_event_loop()
        # Start sender task
        asyncio.create_task(self._ws_sender())
        # Connect (blocks with reconnect loop)
        await self._connect_ws()

    def start_ws_thread(self):
        """Start WebSocket bridge in a daemon thread."""

        def run_async():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._ws_main())

        thread = threading.Thread(target=run_async, daemon=True)
        thread.start()
        self.get_logger().info("WebSocket thread started")


def main(args=None):
    rclpy.init(args=args)
    node = RobotBridgeRos()

    # Start WebSocket bridge on daemon thread
    node.start_ws_thread()

    # Use MultiThreadedExecutor to allow spin_until_future_complete
    # from background threads while main spin is running
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # Run ROS2 spin on main thread
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()