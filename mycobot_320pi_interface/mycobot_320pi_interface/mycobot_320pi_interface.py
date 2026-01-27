#!/usr/bin/env python3
"""
ROS 2 - ROBOT BRIDGE with YAML configuration
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
    """Minimal ROS 2 node bridging hardware interface to robot server"""
    
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
        
        # Setup ZMQ connection
        self.zmq_context = None
        self.zmq_socket = None
        self.robot_connected = False
        
        # Setup ROS connections
        self.setup_ros_connections()
        
        # Connect to robot
        self.connect_to_robot()
        
        # Start feedback thread
        self.feedback_thread = threading.Thread(target=self.feedback_loop, daemon=True)
        self.feedback_thread.start()
        
        self.get_logger().info("🤖 Robot Bridge initialized")
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
    
    def connect_to_robot(self):
        """Connect to robot via ZMQ"""
        try:
            self.zmq_context = zmq.Context()
            self.zmq_socket = self.zmq_context.socket(zmq.REQ)
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, int(self.timeout * 1000))
            self.zmq_socket.connect(f"tcp://{self.robot_ip}:{self.robot_port}")
            self.robot_connected = True
            
            # Test connection with GET_ANGLES
            reply = self.send_zmq_command("GET_ANGLES")
            if reply and "ANGLES:" in reply:
                self.get_logger().info(f"✅ Connected to robot at {self.robot_ip}:{self.robot_port}")
                # Log initial angles
                angles_str = reply.split(":")[1]
                angles = [float(x) for x in angles_str.split(",")]
                if len(angles) >= 6:
                    self.get_logger().info(f"📊 Initial angles: {angles[0]:.1f}, {angles[1]:.1f}, {angles[2]:.1f}, {angles[3]:.1f}, {angles[4]:.1f}, {angles[5]:.1f}°")
            else:
                self.get_logger().warn("⚠️ Connected but robot didn't respond properly")
                self.robot_connected = False
                
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to robot: {e}")
            self.robot_connected = False
    
    def send_zmq_command(self, command: str) -> Optional[str]:
        """Send ZMQ command to robot and get response"""
        if not self.robot_connected or not self.zmq_socket:
            return None
        
        try:
            self.zmq_socket.send_string(command)
            reply = self.zmq_socket.recv_string()
            return reply
        except zmq.Again:
            if self.debug_mode:
                self.get_logger().warn(f"⚠️ Timeout waiting for reply to: {command}")
            self.robot_connected = False
            return None
        except Exception as e:
            self.get_logger().error(f"❌ ZMQ error: {e}")
            self.robot_connected = False
            return None
    
    def arm_command_callback(self, msg: Float32MultiArray):
        """Handle arm commands from hardware interface"""
        if not self.robot_connected:
            if self.debug_mode:
                self.get_logger().warn("⚠️ Robot not connected, ignoring arm command")
            return
        
        if len(msg.data) >= 6:
            try:
                # The hardware interface sends angles in degrees
                # Format directly for robot: MOVE_ANGLES:angle1,angle2,...,angle6
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
        if not self.robot_connected:
            if self.debug_mode:
                self.get_logger().warn("⚠️ Robot not connected, ignoring gripper command")
            return
        
        try:
            gripper_value = msg.data
            
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
        """Continuously query robot state and publish feedback"""
        last_log_time = time.time()
        
        while rclpy.ok():
            try:
                # Query robot for current state
                if self.robot_connected:
                    reply = self.send_zmq_command("GET_ANGLES")
                    
                    if reply and "ANGLES:" in reply:
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
                    
                    # Publish gripper feedback (dummy value - update if robot provides gripper feedback)
                    gripper_msg = Float32()
                    gripper_msg.data = 0.0  # Default open state
                    self.gripper_feedback_pub.publish(gripper_msg)
                    
                else:
                    # Try to reconnect
                    self.get_logger().info("🔄 Reconnecting to robot...")
                    self.connect_to_robot()
                    time.sleep(2.0)  # Wait before retry
                
                time.sleep(self.query_sleep_time)
                
            except Exception as e:
                self.get_logger().error(f"❌ Error in feedback loop: {e}")
                time.sleep(1.0)
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.zmq_socket:
            self.zmq_socket.close()
        if self.zmq_context:
            self.zmq_context.term()
        self.get_logger().info("👋 Robot Bridge shutdown complete")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*50)
    print("🤖 ROS 2 - MYCOBOT 320PI BRIDGE")
    print("="*50)
    print("Minimal bridge connecting hardware interface to robot")
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


if __name__ == "__main__":
    main()