#Setup 
Follow install instructions for ROS2 Jazzy
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

apt-get install ros-jazzy-hardware-interface


sudo apt-get install ros-jazzy-twist-mux-msgs
sudo apt-get install ros-jazzy-key-teleop
sudo apt install libserial-dev
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house
