import zmq
import sys
import tty
import termios

# --- SETUP ---
ROBOT_IP = "192.168.1.218"
PORT = "5555"

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect(f"tcp://{ROBOT_IP}:{PORT}")

def get_char():
    """Reads a single keypress without waiting for Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

print("🔌 Connecting to robot...")
socket.send_string("GET_ANGLES")
resp = socket.recv_string()

# Parse current angles
if "ANGLES" not in resp:
    print("❌ Could not read angles. Is server.py running?")
    sys.exit()

data = resp.split(":")[1]
current_angles = [float(x) for x in data.split(",")]
j6_val = current_angles[5]

print("\n" + "="*40)
print("🎯 J6 PRECISION CALIBRATOR")
print("="*40)
print("Instructions:")
print("  [W]  Rotate Left  (+1.0°)")
print("  [S]  Rotate Right (-1.0°)")
print("  [E]  Fine Left    (+0.1°)")
print("  [D]  Fine Right   (-0.1°)")
print("  [Q]  Quit & Save")
print("-" * 40)

while True:
    print(f"\rCurrent J6 Angle: {j6_val:.2f}°   <-- MAKE THIS LEVEL", end="")
    
    key = get_char().lower()
    
    if key == 'q':
        print(f"\n\n✅ FINAL OFFSET: {j6_val:.2f}")
        print("Copy this value into your bridge.py!")
        break
    elif key == 'w': j6_val += 1.0
    elif key == 's': j6_val -= 1.0
    elif key == 'e': j6_val += 0.1
    elif key == 'd': j6_val -= 0.1
    
    # Send new command
    current_angles[5] = j6_val
    cmd_str = ",".join(["%.2f" % a for a in current_angles])
    socket.send_string(f"MOVE_ANGLES:{cmd_str}")
    socket.recv_string() # Wait for Ack