cd ~/swarm_ws
source install/setup.bash
ros2 launch swarm_control swarm_launch.launch.py

cd ~/swarm_ws
source install/setup.bash
ros2 run swarm_control swarm_brain

source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/model/mother/cmd_vel