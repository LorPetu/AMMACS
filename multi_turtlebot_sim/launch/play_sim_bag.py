import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    path_to_bag_arg =  DeclareLaunchArgument('bag_file', description='Path to the ROS 2 bag file to play')
    path_to_bag = LaunchConfiguration('bag_file')

    bag_play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', path_to_bag],
            output='screen'
        )
    
    ld.add_action(path_to_bag_arg)
    ld.add_action(bag_play)

    return ld
