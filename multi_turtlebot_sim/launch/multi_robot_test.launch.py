import launch
import ast 
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, ExecuteProcess, TimerAction,OpaqueFunction,GroupAction
from launch.substitutions import LaunchConfiguration
import os
#/home/lor/GBEAM2_MultiUAV/src/multi_turtlebot_sim/launch/spawn_cooperative_agent.launch.py
def launch_spawn_gbeam2(namespace, lidar, x_pose, y_pose,num_robot,bitmap):
    launch =  IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('multi_turtlebot_sim'), 'launch', 'spawn_cooperative_agent.launch.py')
                    ),
                    launch_arguments={
                        'N_robot' : num_robot,
                        'robot_prefix': namespace,
                        'lidar_height': str(lidar),  # Ensure parameters are passed as strings
                        'coll_bitmap': str(bitmap),
                        'x_pose': str(x_pose),
                        'y_pose': str(y_pose),
                    }.items()
                )
    return launch

def recordBagfor(N_robot, sim_name):
    """Records a ROS 2 bag for a given number of robots and a simulation name."""
    
    # Get the current timestamp
    timestamp = datetime.now().strftime("%m-%d_%H-%M-%S")
    bag_dir = os.path.join('rosbags', f"{timestamp}_{sim_name}")

    # Topics to record
    target_topics = [
        'scan', 'odom','gbeam/reachability_graph', 'gbeam/merged_graph', 'gbeam/target_pos_ref',
        'gbeam/gbeam_pos_ref', 'coop/Globalclusters', 'timers', 'external_nodes', 'Task',
        'gbeam/free_polytope'
    ]

    topic_to_record = []

    for i in range(N_robot):
        namespace = f'robot{i}'
        for topic in target_topics:
            topic_to_record.append(f'{namespace}/{topic}')

    # Global topic 
    topic_to_record.append('/status')
    topic_to_record.append('/tf')
    topic_to_record.append('/clock')

    return ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--output', bag_dir,'-a'], #+ topic_to_record,
        output='screen'
    )


def generate_launch_description():
    """Generate the launch description dynamically based on the number of robots."""
    ld = LaunchDescription()

    # Declare launch arguments
    num_robots_arg  = DeclareLaunchArgument('num_robots', default_value='2', description='Number of robots to simulate')
    positions_arg   = DeclareLaunchArgument('robot_positions', default_value="[(1.5, -1.8), (1.65, -0.18)]", description='Positions of robots')
    sim_name_arg    = DeclareLaunchArgument('sim_name', default_value='my_simulation', description='Name of the simulation')

    # Retrieve launch configurations
    num_robots = LaunchConfiguration('num_robots')
    positions  = LaunchConfiguration('robot_positions')
    sim_name   = LaunchConfiguration('sim_name')
    # # Open Rqt GUI (Optional)
    # open_gui = ExecuteProcess(
    #         cmd=['ros2', 'run', 'rqt_gui', 'rqt_gui'],
    #         output='screen'
    #     )

    
    standalone_world = ExecuteProcess(cmd=["ros2", "launch", "multi_turtlebot_sim", "standalone_world.launch.py"])
    
    
    foxglove_bridge  = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])
    setup_world = GroupAction(
                actions=[
                    standalone_world,
                    foxglove_bridge
                ]
            )

    offset=1.0

    def launch_setup(context, *args, **kwargs):
        """Function to dynamically process launch arguments at runtime."""
        
        parsed_sim_name = sim_name.perform(context)
        num_robots_value = int(context.launch_configurations['num_robots'])  # Ensure it's an integer

        actions = []

        # Parse positions at real time
        try:
            parsed_positions = ast.literal_eval(context.launch_configurations['robot_positions'])  # Parse at runtime
        except (ValueError, SyntaxError):
            raise RuntimeError("Invalid format for robot_positions. Expected a list of tuples.")

        

        # Log simulation name
        actions.append(LogInfo(msg=f"Simulation name: {context.launch_configurations['sim_name']}"))

        # Create TimerActions dynamically
        for idx, (x_pose, y_pose) in enumerate(parsed_positions[:num_robots_value]):  # Ensure we respect num_robots
            namespace = f'robot{idx}'
            lidar_height = 0.17 + (idx * 0.2)  # Example lidar height variation
            bitmap= f'0x{idx}0'

            actions.append(TimerAction(
                period=offset + idx*10.0,  # Delay to avoid simultaneous launches
                actions=[
                    launch_spawn_gbeam2(namespace, lidar_height, x_pose, y_pose, num_robots,bitmap),
                ]
            ))

        # Record the overall simulation
        actions.append(recordBagfor(num_robots_value,parsed_sim_name))

        return actions
    


    # Add launch arguments
    ld.add_action(num_robots_arg)
    ld.add_action(positions_arg)
    ld.add_action(sim_name_arg)
    # ld.add_action(open_gui)
    ld.add_action(setup_world)

    # Evaluate launch configurations using OpaqueFunction
    ld.add_action(OpaqueFunction(function=launch_setup))

    

    return ld
