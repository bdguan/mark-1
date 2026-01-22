import zmq
import time

# --- CONFIGURATION ---
ROBOT_IP = "192.168.1.218"  # The IP we found earlier
PORT = "5555"

def main():
    print(f"🔌 Connecting to robot at {ROBOT_IP}:{PORT}...")
    
    # Setup ZMQ Context
    context = zmq.Context()
    socket = context.socket(zmq.REQ) # REQ because we send a request and wait for a reply
    socket.connect(f"tcp://{ROBOT_IP}:{PORT}")
    
    print("✅ Connected! Starting Gripper Loop (Ctrl+C to stop)...")
    
    try:
        while True:
            # 1. CLOSE GRIPPER
            print("   👉 Sending: GRIPPER_CLOSE")
            socket.send_string("GRIPPER_CLOSE")
            reply = socket.recv_string() # We MUST wait for reply to keep sync
            print(f"      🤖 Robot said: {reply}")
            
            time.sleep(2.0) # Wait for physical movement

            # 2. OPEN GRIPPER
            print("   👉 Sending: GRIPPER_OPEN")
            socket.send_string("GRIPPER_OPEN")
            reply = socket.recv_string()
            print(f"      🤖 Robot said: {reply}")
            
            time.sleep(2.0)

    except KeyboardInterrupt:
        print("\n🛑 Loop stopped by user.")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    main()