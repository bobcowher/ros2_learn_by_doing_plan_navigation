import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware_v2"),
            "launch",
            "robot_controller.launch.py"
        ),
    )
    
    laser_driver = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_lidar"),
            "launch",
            "rplidar.launch.py"
        ),
    )

    imu_driver_node = Node(
        package="bumperbot_firmware",
        executable="mpu6050_driver.py"
    )

    return LaunchDescription([
        hardware_interface,
        laser_driver,
        imu_driver_node,
    ])
