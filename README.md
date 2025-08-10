#Setup 
Follow install instructions for ROS2 Jazzy
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

apt-get install ros-jazzy-hardware-interface


sudo apt-get install ros-jazzy-twist-mux-msgs
sudo apt-get install ros-jazzy-key-teleop
sudo apt install libserial-dev
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house

# Launch Instructions Section 7: Navigation
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house

# Launch smoother server
ros2 run nav2_smoother smoother_server --ros-args --params-file /home/robertcowher/rosprojects/bumperbot_ws/src/bumperbot_navigation/config/smoother_server.yaml
ros2 lifecycle set /smoother_server 1
ros2 lifecycle set /smoother_server 3

# Launch Planner Server
ros2 run nav2_planner planner_server --ros-args --params-file /home/robertcowher/rosprojects/bumperbot_ws/src/bumperbot_navigation/config/planner_server.yaml
ros2 lifecycle set /planner_server 1
ros2 lifecycle set /planner_server 3

# Launch Controller Server
ros2 run nav2_controller controller_server --ros-args --params-file /home/robertcowher/rosprojects/bumperbot_ws/src/bumperbot_navigation/config/controller_server.yaml
ros2 lifecycle set /controller_server 1
ros2 lifecycle set /controller_server 3 

# Send Goal 
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "goal:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: 'map'
  pose:
    position:
      x: 4.0
      y: -2.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
start:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
planner_id: 'GridBased'
use_start: false".

