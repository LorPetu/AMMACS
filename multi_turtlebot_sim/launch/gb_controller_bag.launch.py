import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch.actions import LogInfo, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessIO
from launch.substitutions import PythonExpression

## Launch file description
# This launch file starts the visualization part of gbeam2_ground within rviz2
# Then starts in order: polytope_generation_node --> graph_update_node --> set mapping status true --> exploration_node
# It could happens that the printout of the node does not appear in the order depicted above, this is not a problem


def generate_launch_description():
    robot_prefix_arg = DeclareLaunchArgument('robot_prefix', default_value='')
    robot_prefix = LaunchConfiguration('robot_prefix')
    
    config = os.path.join(
    get_package_share_directory('gbeam2_controller'),
    'config',
    'global_param.yaml'
    )

    world_static_tf = ExecuteProcess( 
        cmd=[[
            # executable
            'ros2 run tf2_ros static_transform_publisher ',
            # parameters
            '0 0 0 0 0 0 world ',
            PythonExpression(["'/", LaunchConfiguration('robot_prefix'), "/odom'"])
        ]],
        shell=True
    )

    play_bag = ExecuteProcess( 
        cmd=[[
            # executable
            'ros2 bag play /home/lor/GBEAM2_MultiUAV/subset',
        ]],
        shell=True
    )

    graph_update=Node(
        package = 'gbeam2_controller',
        name = 'graph_update',                  
        executable = 'graph_update_node',
        parameters = [config],
        namespace=robot_prefix,
        # depends_on = ['poly_gen']
    )


    ld = LaunchDescription(
        [
        world_static_tf,
        graph_update,
        play_bag,

        ]
    )

    config_ground = os.path.join(
        get_package_share_directory('gbeam2_ground'),
        'config',
        'ground_param.yaml'
        )
    # rviz_config_file = os.path.join(
    # get_package_share_directory('gbeam2_simulator'),
    # 'rviz',
    # 'turtlebot3_gazebo_poly&graph_draw.rviz"'
    # )

    ld.add_action(
        Node(
        package = 'gbeam2_ground',
        name = 'poly_draw',                  
        executable = 'poly_drawer',
        namespace=robot_prefix,
        parameters = [config_ground]
    ))

    ld.add_action(
        Node(
        package = 'gbeam2_ground',
        name = 'graph_draw',                  
        executable = 'graph_drawer',
        namespace=robot_prefix,
        parameters = [config_ground]
    ))



    ld.add_action(robot_prefix_arg)

    return ld