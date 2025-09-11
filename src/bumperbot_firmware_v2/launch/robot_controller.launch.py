import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("bumperbot_description"),
                    "urdf",
                    "bumperbot.urdf.xacro",
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    robot_driver_node = Node(
        package='bumperbot_firmware_v2',
        executable='robot_driver_node',
        name='robot_driver_node',
        remappings=[('cmd_vel', 'bumperbot_controller/cmd_vel_unstamped')]
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            robot_driver_node,
        ]
    )
