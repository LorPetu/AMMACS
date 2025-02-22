import os
import ament_index_python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, GroupAction, ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart

import xacro

def get_param(package_name, param_file):
    return os.path.join(get_package_share_directory(package_name), 'config',param_file)


def generate_launch_description():
    # Launch configuration variables specific to simulation
    N_robot_arg = DeclareLaunchArgument('N_robot', default_value='2')
    N_robot = LaunchConfiguration('N_robot')

    robot_prefix_arg = DeclareLaunchArgument('robot_prefix', default_value='')
    robot_prefix = LaunchConfiguration('robot_prefix')

    lidar_height_arg = DeclareLaunchArgument('lidar_height', default_value='0.122')
    lidar_height = LaunchConfiguration('lidar_height')

    coll_bitmap_arg = DeclareLaunchArgument('coll_bitmap', default_value='0x01')
    coll_bitmap = LaunchConfiguration('coll_bitmap')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='')
    use_sim_time = LaunchConfiguration('use_sim_time')

    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    x_pose = LaunchConfiguration('x_pose')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    y_pose = LaunchConfiguration('y_pose')

    limit_xi_arg = DeclareLaunchArgument('limit_xi', default_value='-2.5')          
    limit_xs_arg = DeclareLaunchArgument('limit_xs', default_value= '2.5')              
    limit_yi_arg = DeclareLaunchArgument('limit_yi', default_value='-2.5')             
    limit_ys_arg = DeclareLaunchArgument('limit_ys', default_value='2.5')             

    limit_xi = LaunchConfiguration('limit_xi')          
    limit_xs = LaunchConfiguration('limit_xs')              
    limit_yi = LaunchConfiguration('limit_yi')             
    limit_ys = LaunchConfiguration('limit_ys')               




    # Obtain urdf from xacro files.
    multi_turtlebot_sim_pkg_dir = get_package_share_directory('multi_turtlebot_sim')
    xacro_file_path = os.path.join(multi_turtlebot_sim_pkg_dir, 'urdf', 'turtlebot3_waffle.urdf.xacro')
    robot_desc = Command(['xacro ', xacro_file_path, ' frame_prefix:=', robot_prefix, ' topic_prefix:=', robot_prefix, ' lidar_height_arg:=',lidar_height, ' collision_bitmap:=',coll_bitmap])

    # Robot state publisher
    # This node will take the urdf description and:
    # - publish the transforms using the prefix set by the frame_prefix parameter.
    # - publish the robot description under the set namespace.
    # - subscribe to joint states under the set namespace.
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=robot_prefix,
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix':
                    PythonExpression(["'", LaunchConfiguration('robot_prefix'), "/'"])
            }],
        )
    
    # Create a tf from world to robot_prefix/odom
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

    # Spawn robot
    start_gazebo_ros_spawner_cmd =  TimerAction(
        period=2.0,
        actions=[
            Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', PathJoinSubstitution([robot_prefix, 'waffle']),
            '-topic', PathJoinSubstitution(['/',robot_prefix, 'robot_description']),
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

        ]
    )
    



    # launch gbeam 2 os.path.join(get_package_share_directory('gbeam2_simulator'),'launch/ground_nodes.launch.py')
    # Param config files
    communication_param_arg = DeclareLaunchArgument(
                'communication_param',
                default_value=[
                    get_param('gbeam2_communication','communication_param.yaml')
                ],
                description='Path to config file for joystick translator')
    

    cooperative_launch = GroupAction(
        actions=[
            communication_param_arg,

            SetRemap(src='/scan',dst= PythonExpression(["'/", LaunchConfiguration('robot_prefix'), "/scan'"])),
            
            # #status_node 
            Node(
                package='gbeam2_communication',
                name='statusTX_node',
                executable='status_transmitter',
                namespace=robot_prefix,
                parameters=[
                    LaunchConfiguration('communication_param'),
                    {"N_robot": N_robot}  
                ]
                # parameters=[os.path.join(get_package_share_directory('gbeam2_communication'),'config','communication_param.yaml'),
                # {"N_robot": N_robot}]
            ),            
            #ddrive
            Node(
                package='gbeam2_simulator',
                name="pos_contr",
                executable='ddrive_position',
                namespace=robot_prefix,
                parameters=[os.path.join(
            get_package_share_directory('gbeam2_simulator'),
            'config',
            'ddrive_param.yaml'
            )]
    )])

    # Gbeam controller launch 

    gbeam2_parameter = [LaunchConfiguration('global_param'), {"N_robot": N_robot}]

    if N_robot != '1':
        gbeam2_parameter.extend([{"limit_xi" : limit_xi} ,{"limit_xs":limit_xs} ,{"limit_yi":limit_yi} ,{"limit_ys":limit_ys} ])


    gbeam2_launch =   GroupAction(
        actions=[
            DeclareLaunchArgument(
                'global_param',
                default_value=[
                    get_param('old_gbeam2_controller','global_param.yaml')
                ],
                description='Path to config file for joystick translator'),

            # poly_gen
            Node(
                package = 'old_gbeam2_controller',
                name = 'poly_gen',               
                executable = 'polytope_generation_node',
                parameters = gbeam2_parameter,
                namespace= robot_prefix
            ),

            # graph_update
            Node(
                package = 'old_gbeam2_controller',
                name = 'graph_update',                  
                executable = 'graph_update_node',
                parameters = gbeam2_parameter,
                namespace=robot_prefix,
                # depends_on = ['poly_gen']
            ),
            # graph_expl 
            Node(
                package = 'old_gbeam2_controller',
                name = 'graph_expl',                 
                executable = 'exploration_node',
                parameters = gbeam2_parameter, 
                namespace=robot_prefix,
                # depends_on = ['graph_update']
            )            

        ]
    )

    # Ground nodes for visualization
    ground_param_arg = DeclareLaunchArgument(
            'ground_param',
            default_value=[
                get_param('gbeam2_ground','ground_param.yaml')
            ],
            description='Path to config file for joystick translator')
    
    ground_action= GroupAction(
          actions=[
                ground_param_arg,
                Node(
                    package = 'gbeam2_ground',
                    name = 'poly_draw',                  
                    executable = 'poly_drawer',
                    namespace=robot_prefix,
                    parameters = [LaunchConfiguration('ground_param')]
                ),
                Node(
                    package = 'gbeam2_ground',
                    name = 'graph_draw',                  
                    executable = 'graph_drawer',
                    namespace=robot_prefix,
                    parameters = [LaunchConfiguration('ground_param'),{"N_robot": N_robot}]
                ),
                Node(
                    package = 'gbeam2_ground',
                    name = 'comm_drawer',                  
                    executable = 'communication_drawer',
                    parameters = [LaunchConfiguration('ground_param')]
                )

          ]
    )



            

            
    ld = LaunchDescription()

    # Declare the launch options
    # - Arguments
    ld.add_action(N_robot_arg)
    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(lidar_height_arg)
    ld.add_action(robot_prefix_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(coll_bitmap_arg)
    ld.add_action(limit_xi_arg)
    ld.add_action(limit_xs_arg)
    ld.add_action(limit_yi_arg)
    ld.add_action(limit_ys_arg)

    # - Spawner and world interactions
    ld.add_action(robot_state_publisher)
    ld.add_action(world_static_tf)
    ld.add_action(RegisterEventHandler(OnProcessStart(
             target_action=robot_state_publisher, on_start=[start_gazebo_ros_spawner_cmd])))
    
    # - Groups of Nodes 
    ld.add_action(cooperative_launch)
    ld.add_action(gbeam2_launch)
    ld.add_action(ground_action)

    return ld
