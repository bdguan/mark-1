MYCOBOT 320 PI ros2 humble

source install/setup.bash
ros2 launch mycobot_320pi_bringup mycobot_320pi_real.launch.py


>>> connect real robot via socket tcp:
source install/setup.bash
ros2 launch mycobot_320pi_interface mycobot_320pi_firmware_launch.py


pip install pyzmq