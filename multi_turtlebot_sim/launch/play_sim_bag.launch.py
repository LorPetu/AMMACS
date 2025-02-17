import subprocess
import launch
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, ExecuteProcess, TimerAction,OpaqueFunction,GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
import os
from launch.substitutions import PythonExpression


def get_param(package_name, param_file):
    return os.path.join(get_package_share_directory(package_name), 'config', param_file)

def select_bag_file(context, *args, **kwargs):
    """Function to prompt the user to select a bag file and return actions dynamically."""
    default_bag_dir = "/home/lor/rosbags"

    command = f'zenity --file-selection --title="Select ROS 2 Bag File" --file-filter="*.db3" --filename="{default_bag_dir}/"'
    process = subprocess.run(command, shell=True, stdout=subprocess.PIPE, text=True)
    bag_file = process.stdout.strip()

    if not bag_file:
        raise RuntimeError("No bag file selected. Exiting launch.")

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file, '-r 2.0'],
            output='screen'
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
    ground_param_arg = DeclareLaunchArgument(
        'ground_param',
        default_value=[get_param('gbeam2_ground', 'ground_param.yaml')],
        description='Path to config file for joystick translator'
    )

    # Foxglove bridge
    foxglove_bridge = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])

    # Add actions
    ld.add_action(num_robots_arg)
    ld.add_action(ground_param_arg)
    ld.add_action(foxglove_bridge)
    ld.add_action(OpaqueFunction(function=select_bag_file))  # Select bag file dynamically
    ld.add_action(OpaqueFunction(function=launch_setup))  # Launch ground nodes dynamically

    return ld


# def generate_launch_description():
#     ld = LaunchDescription()

#     num_robots_arg  = DeclareLaunchArgument('num_robots', default_value='2', description='Number of robots to simulate')

#     # Retrieve launch configurations
#     num_robots = LaunchConfiguration('num_robots')
#     # # Open Rqt GUI (Optional)
#     # open_gui = ExecuteProcess(
#     #         cmd=['ros2', 'run', 'rqt_gui', 'rqt_gui'],
#     #         output='screen'
#     #     )
#     # Set default parent folder for bag selection
#     default_bag_dir = "/home/lor/rosbags"  # Change this to your desired directory

#     # Open a file selection dialog using Zenity, starting from the default folder
#     command = f'zenity --file-selection --title="Select ROS 2 Bag File" --file-filter="*.db3" --filename="{default_bag_dir}/"'
#     process = subprocess.run(command, shell=True, stdout=subprocess.PIPE, text=True)
#     bag_file = process.stdout.strip()  # Get the selected file path

#     if not bag_file:
#         raise RuntimeError("No bag file selected. Exiting launch.")
    
#     select_bag = ExecuteProcess(
#             cmd=['ros2', 'bag', 'play', bag_file],
#             output='screen'
#         )
    
#         # Foxglove bridge
#     foxglove_bridge = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])

#     # Declare ground param argument
#     ground_param_arg = DeclareLaunchArgument(
#         'ground_param',
#         default_value=[get_param('gbeam2_ground', 'ground_param.yaml')],
#         description='Path to config file for joystick translator'
#     )

#     # Function to launch ground nodes dynamically
#     def launch_setup(context, *args, **kwargs):
#         num_robots_value = int(context.launch_configurations['num_robots'])

#         actions = []
#         for idx in range(num_robots_value):
#             robot_prefix = f'robot{idx}'

#             actions.append(
#                 GroupAction(
#                     actions=[
#                         Node(
#                             package='gbeam2_ground',
#                             name='poly_draw',
#                             executable='poly_drawer',
#                             namespace=robot_prefix,
#                             parameters=[LaunchConfiguration('ground_param')]
#                         ),
#                         Node(
#                             package='gbeam2_ground',
#                             name='graph_draw',
#                             executable='graph_drawer',
#                             namespace=robot_prefix,
#                             parameters=[LaunchConfiguration('ground_param'), {"N_robot": num_robots_value}]
#                         ),
#                         Node(
#                             package='gbeam2_ground',
#                             name='comm_drawer',
#                             executable='communication_drawer',
#                             parameters=[LaunchConfiguration('ground_param')]
#                         )
#                     ]
#                 )
#             )

#         return actions

#     # Add actions to launch description
#     ld.add_action(num_robots_arg)
#     ld.add_action(ground_param_arg)
#     ld.add_action(select_bag)
#     ld.add_action(select_bag)
#     ld.add_action(foxglove_bridge)
#     ld.add_action(OpaqueFunction(function=launch_setup))


#     return ld
