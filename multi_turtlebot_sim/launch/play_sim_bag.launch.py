import subprocess
import launch
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, ExecuteProcess, TimerAction,OpaqueFunction,GroupAction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch.event_handlers import OnProcessExit, OnShutdown
import os
import re
from launch.substitutions import PythonExpression


def get_param(package_name, param_file):
    return os.path.join(get_package_share_directory(package_name), 'config', param_file)

def select_bag_file(context, *args, **kwargs):
    """Function to prompt the user to select a bag file and return actions dynamically."""
    default_bag_dir = "/home/lor/rosbags"
    namespace = str(context.launch_configurations['name_space'])
    speed = str(context.launch_configurations['speed'])

    command = f'zenity --file-selection --title="Select ROS 2 Bag File" --file-filter="*.db3" --filename="{default_bag_dir}/"'
    process = subprocess.run(command, shell=True, stdout=subprocess.PIPE, text=True)
    bag_file = process.stdout.strip()

    # Extract bag name without '_0' and file extension
    bag_name = re.sub(r'(_0)?\.db3$', '', bag_file.split('/')[-1])

    if not bag_file:
        raise RuntimeError("No bag file selected. Exiting launch.")

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file, '-r', speed],
            output='screen'
        ),
        Node(
            package='performance', 
            executable='anal',      
            name='performance_node', 
            output='screen',
            parameters=[{'namespace': namespace},
                        {'bag_name':bag_name},
                        {'use_sim_time': True}] 
        )
        
    ]

def launch_setup(context, *args, **kwargs):
    """Function to dynamically launch ground nodes based on num_robots argument."""
    num_robots_value = int(context.launch_configurations['num_robots'])
    actions = []

    for idx in range(num_robots_value):
        robot_prefix = f'robot{idx}'

        actions.append(
            GroupAction(
                actions=[
                    LogInfo(msg=f"Group action for robot: {robot_prefix} out of {num_robots_value}"),
                    ExecuteProcess( 
                    cmd=[[
                        # executable
                        'ros2 run tf2_ros static_transform_publisher ',
                        # parameters
                        '0 0 0 0 0 0 world ',
                        PythonExpression(["'/", robot_prefix, "/odom'"])
                        ]],
                        shell=True
                    ),
                    Node(
                        package='gbeam2_ground',
                        name='poly_draw',
                        executable='poly_drawer',
                        namespace=robot_prefix,
                        parameters=[LaunchConfiguration('ground_param')]
                    ),
                    Node(
                        package='gbeam2_ground',
                        name='graph_draw',
                        executable='graph_drawer',
                        namespace=robot_prefix,
                        parameters=[LaunchConfiguration('ground_param'), {"N_robot": num_robots_value}]
                    ),
                    Node(
                        package='gbeam2_ground',
                        name='comm_drawer',
                        executable='communication_drawer',
                        parameters=[LaunchConfiguration('ground_param')]
                    )
                ]
            )
        )
    return actions

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    num_robots_arg = DeclareLaunchArgument('num_robots', default_value='2', description='Number of robots to simulate')
    name_space_arg = DeclareLaunchArgument('name_space',default_value='robot0', description='Namespace of the robot to be analyzed')
    speed_arg = DeclareLaunchArgument('speed', default_value='4.0',description='Record bag play at speed')

    ground_param_arg = DeclareLaunchArgument(
        'ground_param',
        default_value=[get_param('gbeam2_ground', 'ground_param.yaml')],
        description='Path to config file for joystick translator'
    )

    # Foxglove bridge
    foxglove_bridge = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])

    watch_dog = Node(
            package='performance',
            executable='bag_watchdog',
            name='bag_watchdog',
            parameters=[{'use_sim_time': False}]        
    )

    # Event handler to shut down the launch process when both nodes have exited
    shutdown_event_handler = RegisterEventHandler(
        OnProcessExit(
                target_action=watch_dog,
                on_exit=[LogInfo(msg="Both analyzer and watchdog have stopped. Shutting down launch..."), launch.actions.Shutdown()]
            )
    )

    # Add actions
    ld.add_action(num_robots_arg)
    ld.add_action(name_space_arg)
    ld.add_action(speed_arg)
    ld.add_action(ground_param_arg)
    ld.add_action(foxglove_bridge)
    ld.add_action(OpaqueFunction(function=select_bag_file))  # Select bag file dynamically
    ld.add_action(OpaqueFunction(function=launch_setup))  # Launch ground nodes dynamically
    ld.add_action(watch_dog)
    ld.add_action(shutdown_event_handler)


    return ld
