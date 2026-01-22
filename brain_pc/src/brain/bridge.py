import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import zmq
import math
import time

# --- CONFIGURATION ---
ROBOT_IP = "192.168.1.218"
PORT = "5555"

# JOINT NAMES (Must match MoveIt)
JOINT_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# 🛠️ CALIBRATION ZONE (Your Final Values) 🛠️
JOINT_CORRECTIONS = [
    {'invert': False, 'offset': 0.0},
    {'invert': True,  'offset': 0.0},
    {'invert': True,  'offset': 0.0},
    {'invert': True,  'offset': 0.0},
    {'invert': False, 'offset': 0.0},
    {'invert': True,  'offset': -135.55} # <--- Your calibration applied here
]

class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')
        
        # 1. Setup ZMQ (Client)
        print(f"🔌 Connecting to Golden Server at {ROBOT_IP}...")
        self.my_context = zmq.Context()
        self.socket = self.my_context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{ROBOT_IP}:{PORT}")
        print("✅ Connected!")

        # 2. Setup ROS Listener
        self.subscription = self.create_subscription(
            JointState, 
            'joint_states', 
            self.listener_callback, 
            10
        )
        
        self.last_sent_time = 0
        self.last_gripper_state = "UNKNOWN"

    def listener_callback(self, msg):
        try:
            # --- PART A: ARM ANGLES (Rad -> Deg) ---
            current_angles = []
            found_all = True
            for name in JOINT_ORDER:
                if name in msg.name:
                    idx = msg.name.index(name)
                    # Convert Rad to Deg
                    deg = math.degrees(msg.position[idx])
                    current_angles.append(deg)
                else:
                    found_all = False
            
            # Send Arm Data (Max 20Hz to prevent flooding)
            if found_all and len(current_angles) == 6:
                # Apply Offsets
                final_angles = []
                for i, ang in enumerate(current_angles):
                    corr = JOINT_CORRECTIONS[i]
                    val = -ang if corr['invert'] else ang
                    val += corr['offset']
                    final_angles.append(val)

                if time.time() - self.last_sent_time > 0.05:
                    cmd_str = ",".join(["%.2f" % a for a in final_angles])
                    self.socket.send_string(f"MOVE_ANGLES:{cmd_str}")
                    self.socket.recv_string() # Ack
                    self.last_sent_time = time.time()

            # --- PART B: GRIPPER LOGIC ---
            # We watch 'left_jaw_joint_1' (Sim Gripper Joint)
            if "left_jaw_joint_1" in msg.name:
                idx = msg.name.index("left_jaw_joint_1")
                # Sim Value: 0.0 (Open) to 0.8 (Closed)
                # We use a threshold of 0.1 to detect "Intent to Close"
                sim_val = abs(msg.position[idx])
                
                new_state = "CLOSE" if sim_val > 0.1 else "OPEN"
                
                if new_state != self.last_gripper_state:
                    print(f"🖐 Gripper Transition: {new_state}")
                    
                    # Send appropriate command to Golden Server
                    if new_state == "CLOSE":
                        self.socket.send_string("GRIPPER_CLOSE")
                    else:
                        self.socket.send_string("GRIPPER_OPEN")
                        
                    print(f"🤖 Robot Replied: {self.socket.recv_string()}")
                    self.last_gripper_state = new_state

        except Exception as e:
            print(f"Bridge Error: {e}")

def main():
    rclpy.init()
    node = RobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()