#!/usr/bin/env python3
"""
ROS 2 - ROBOT BRIDGE with Thread-Safe ZMQ Communication
Minimal bridge connecting hardware interface to robot via ZMQ.
"""

import zmq
import time
import threading
from typing import Optional

# ROS 2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32


class RobotBridge(Node):
    """Thread-safe ROS 2 node bridging hardware interface to robot server"""

    def __init__(self):
        super().__init__('robot_bridge')

        # Load parameters from YAML file
        self.declare_parameter('robot_ip', '192.168.1.218')
        self.declare_parameter('robot_port', '5555')
        self.declare_parameter('query_rate', 10.0)
        self.declare_parameter('timeout', 2.0)
        self.declare_parameter('gripper_close_threshold', 0.5)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('log_interval', 1.0)
        
        # Get parameter values
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.robot_port = self.get_parameter('robot_port').get_parameter_value().string_value
        self.query_rate = self.get_parameter('query_rate').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.gripper_threshold = self.get_parameter('gripper_close_threshold').get_parameter_value().double_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.log_interval = self.get_parameter('log_interval').get_parameter_value().double_value
        
        # Calculate sleep time from query rate
        self.query_sleep_time = 1.0 / self.query_rate if self.query_rate > 0 else 0.1
        
        # Thread synchronization
        self.socket_lock = threading.Lock()
        self.reconnect_lock = threading.Lock()
        
        # Connection state
        self.zmq_context = None
        self.zmq_socket = None
        self.robot_connected = False
        self.last_connection_time = 0
        self.reconnect_in_progress = False

        # Track last commanded gripper state for feedback
        self.last_gripper_command = 0.0  # Start at open position
        
        # Setup ROS connections
        self.setup_ros_connections()
        
        # Initial connection attempt
        self.connect_to_robot()
        
        # Start feedback thread
        self.running = True
        self.feedback_thread = threading.Thread(target=self.feedback_loop, daemon=True)
        self.feedback_thread.start()
        
        self.get_logger().info("🤖 Thread-Safe Robot Bridge initialized")
        self.get_logger().info(f"  Robot: {self.robot_ip}:{self.robot_port}")
        self.get_logger().info(f"  Query rate: {self.query_rate} Hz")
    
    def setup_ros_connections(self):
        """Setup ROS 2 publishers and subscribers"""
        # Publishers (to hardware interface)
        self.arm_feedback_pub = self.create_publisher(
            Float32MultiArray, 
            '/query_response_arm', 
            10
        )
        
        self.gripper_feedback_pub = self.create_publisher(
            Float32, 
            '/query_response_gripper', 
            10
        )
        
        # Subscribers (from hardware interface)
        self.arm_command_sub = self.create_subscription(
            Float32MultiArray,
            '/arm_hardware_command',
            self.arm_command_callback,
            10
        )
        
        self.gripper_command_sub = self.create_subscription(
            Float32,
            '/gripper_hardware_command',
            self.gripper_command_callback,
            10
        )
    
    def connect_to_robot(self) -> bool:
        """Connect to robot via ZMQ - Thread-safe"""
        with self.reconnect_lock:
            if self.reconnect_in_progress:
                return False
                
            self.reconnect_in_progress = True
            
        try:
            self.get_logger().info(f"🔌 Connecting to robot at {self.robot_ip}:{self.robot_port}...")
            
            # Clean up any existing connection
            self._cleanup_zmq()
            
            # Create new context and socket
            self.zmq_context = zmq.Context()
            self.zmq_socket = self.zmq_context.socket(zmq.REQ)
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, int(self.timeout * 1000))
            self.zmq_socket.setsockopt(zmq.LINGER, 0)
            self.zmq_socket.connect(f"tcp://{self.robot_ip}:{self.robot_port}")
            
            # Test connection with GET_ANGLES
            with self.socket_lock:
                try:
                    self.zmq_socket.send_string("GET_ANGLES")
                    reply = self.zmq_socket.recv_string()
                    
                    if reply and "ANGLES:" in reply:
                        self.robot_connected = True
                        self.last_connection_time = time.time()
                        
                        angles_str = reply.split(":")[1]
                        angles = [float(x) for x in angles_str.split(",")]
                        if len(angles) >= 6:
                            self.get_logger().info(f"✅ Connected to robot at {self.robot_ip}:{self.robot_port}")
                            self.get_logger().info(f"📊 Initial angles: {angles[0]:.1f}, {angles[1]:.1f}, {angles[2]:.1f}, {angles[3]:.1f}, {angles[4]:.1f}, {angles[5]:.1f}°")
                        
                        with self.reconnect_lock:
                            self.reconnect_in_progress = False
                        return True
                    else:
                        self.get_logger().warn("⚠️ Connected but robot didn't respond properly")
                        
                except Exception as e:
                    self.get_logger().error(f"❌ Connection test failed: {e}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to robot: {e}")
        
        # Connection failed
        self._cleanup_zmq()
        with self.reconnect_lock:
            self.reconnect_in_progress = False
        return False
    
    def _cleanup_zmq(self):
        """Safely cleanup ZMQ resources"""
        try:
            if self.zmq_socket:
                with self.socket_lock:
                    self.zmq_socket.close(linger=0)
                    self.zmq_socket = None
        except Exception as e:
            if self.debug_mode:
                self.get_logger().debug(f"Cleanup error: {e}")
        
        # Don't terminate context here - leave it for next connection attempt
        self.robot_connected = False
    
    def send_zmq_command(self, command: str) -> Optional[str]:
        """Thread-safe ZMQ command sending with auto-reconnect"""
        # Check if we need to reconnect
        if not self.robot_connected:
            if self.debug_mode:
                self.get_logger().debug("Not connected, attempting reconnect...")
            if not self.connect_to_robot():
                return None
        
        with self.socket_lock:
            if not self.zmq_socket:
                return None
            
            try:
                self.zmq_socket.send_string(command)
                reply = self.zmq_socket.recv_string()
                
                # Update last successful communication time
                self.last_connection_time = time.time()
                return reply
                
            except zmq.Again:
                if self.debug_mode:
                    self.get_logger().warn(f"⚠️ Timeout waiting for reply to: {command}")
                self._cleanup_zmq()
                return None
            except zmq.ZMQError as e:
                self.get_logger().error(f"❌ ZMQ error: {e}")
                self._cleanup_zmq()
                return None
            except Exception as e:
                self.get_logger().error(f"❌ Unexpected error: {e}")
                self._cleanup_zmq()
                return None
    
    def arm_command_callback(self, msg: Float32MultiArray):
        """Handle arm commands from hardware interface"""
        if not self.running:
            return

        if len(msg.data) >= 6:
            try:
                # The hardware interface sends angles in degrees
                angles = [f"{angle:.1f}" for angle in msg.data[:6]]
                cmd_str = "MOVE_ANGLES:" + ",".join(angles)
                
                if self.debug_mode:
                    self.get_logger().info(f"🤖 Sending: {cmd_str}")
                
                reply = self.send_zmq_command(cmd_str)
                
                if reply and self.debug_mode:
                    self.get_logger().info(f"🤖 Robot: {reply}")
                    
            except Exception as e:
                self.get_logger().error(f"❌ Error sending arm command: {e}")
    
    def gripper_command_callback(self, msg: Float32):
        """Handle gripper commands from hardware interface"""
        if not self.running:
            return

        try:
            # Store the raw commanded value for feedback
            self.last_gripper_command = msg.data

            # Use absolute value since gripper joint has negative range (0 to -1.11)
            gripper_value = abs(msg.data)

            # Determine open/close based on threshold
            if gripper_value > self.gripper_threshold:
                command = "GRIPPER_CLOSE"
            else:
                command = "GRIPPER_OPEN"

            if self.debug_mode:
                self.get_logger().info(f"🖐️ Sending: {command} (value: {gripper_value:.2f})")

            reply = self.send_zmq_command(command)

            if reply and self.debug_mode:
                self.get_logger().info(f"🤖 Robot: {reply}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error sending gripper command: {e}")
    
    def feedback_loop(self):
        """Continuously query robot state and publish feedback - Thread-safe"""
        last_log_time = time.time()
        consecutive_errors = 0
        max_consecutive_errors = 3
        
        while self.running and rclpy.ok():
            try:
                # Query robot for current state
                reply = self.send_zmq_command("GET_ANGLES")
                
                if reply and "ANGLES:" in reply:
                    # Reset error counter on success
                    consecutive_errors = 0
                    
                    # Parse angles from robot response
                    angles_str = reply.split(":")[1]
                    angles = [float(x) for x in angles_str.split(",")]
                    
                    if len(angles) >= 6:
                        # Publish arm feedback
                        arm_msg = Float32MultiArray()
                        arm_msg.data = [float(angle) for angle in angles[:6]]
                        self.arm_feedback_pub.publish(arm_msg)
                        
                        # Log at specified interval
                        current_time = time.time()
                        if current_time - last_log_time > self.log_interval:
                            if self.debug_mode:
                                self.get_logger().info(
                                    f"📊 Robot state: "
                                    f"[{angles[0]:6.1f}, {angles[1]:6.1f}, {angles[2]:6.1f}, "
                                    f"{angles[3]:6.1f}, {angles[4]:6.1f}, {angles[5]:6.1f}]°"
                                )
                            last_log_time = current_time
                    else:
                        consecutive_errors += 1
                        if consecutive_errors >= max_consecutive_errors:
                            self.get_logger().warn(f"⚠️ {consecutive_errors} consecutive errors, forcing reconnection")
                            self._cleanup_zmq()
                            time.sleep(1.0)
                else:
                    consecutive_errors += 1
                    if consecutive_errors >= max_consecutive_errors:
                        self.get_logger().warn(f"⚠️ {consecutive_errors} consecutive errors, forcing reconnection")
                        self._cleanup_zmq()
                        time.sleep(1.0)
                
                # Publish gripper feedback (last commanded value)
                gripper_msg = Float32()
                gripper_msg.data = self.last_gripper_command
                self.gripper_feedback_pub.publish(gripper_msg)
                
                # Sleep between queries
                time.sleep(self.query_sleep_time)
                
            except Exception as e:
                self.get_logger().error(f"❌ Error in feedback loop: {e}")
                time.sleep(1.0)
                consecutive_errors += 1
    
    def health_check(self):
        """Get current connection health status"""
        return {
            "connected": self.robot_connected,
            "last_connection": self.last_connection_time,
            "reconnect_in_progress": self.reconnect_in_progress,
            "uptime": time.time() - self.last_connection_time if self.last_connection_time > 0 else 0
        }
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info("🛑 Starting shutdown sequence...")
        self.running = False
        
        # Wait for feedback thread to finish
        if self.feedback_thread.is_alive():
            self.get_logger().info("Waiting for feedback thread to stop...")
            self.feedback_thread.join(timeout=2.0)
        
        # Cleanup ZMQ
        self._cleanup_zmq()
        if self.zmq_context:
            try:
                self.zmq_context.term()
            except:
                pass
        
        self.get_logger().info("👋 Robot Bridge shutdown complete")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*50)
    print("🤖 ROS 2 - THREAD-SAFE ROBOT BRIDGE")
    print("="*50)
    print("Minimal bridge with thread-safe ZMQ communication")
    print("="*50)
    
    bridge = RobotBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("\n🛑 Bridge stopped by user")
    except Exception as e:
        print(f"\n❌ Bridge error: {e}")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()