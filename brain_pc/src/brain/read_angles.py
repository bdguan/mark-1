import zmq

# Connect to the Robot Server
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.1.218:5555")

print("🔌 Connecting to robot...")
# Send the "GET_ANGLES" command (built into your server.py)
socket.send_string("GET_ANGLES")
response = socket.recv_string()

print("\n" + "="*30)
print(f"🎯 CURRENT REAL ANGLES:\n{response}")
print("="*30 + "\n")

# Parse specifically for J6 if needed
if "ANGLES" in response:
    data = response.split(":")[1]
    angles = [float(x) for x in data.split(",")]
    print(f"👉 YOUR J6 OFFSET IS: {angles[5]}")