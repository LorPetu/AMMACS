import launch
import ast 
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, ExecuteProcess, TimerAction,OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os
#/home/lor/GBEAM2_MultiUAV/src/multi_turtlebot_sim/launch/spawn_cooperative_agent.launch.py
def launch_spawn_gbeam2(namespace, lidar, x_pose, y_pose,num_robot):
    launch =  IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('multi_turtlebot_sim'), 'launch', 'spawn_cooperative_agent.launch.py')
                    ),
                    launch_arguments={
                        'N_robot' : num_robot,
                        'robot_prefix': namespace,
                        'lidar_height': str(lidar),  # Ensure parameters are passed as strings
                        'x_pose': str(x_pose),
                        'y_pose': str(y_pose),
                    }.items()
                )
    return launch

def recordBagfor(namespace,sim_name):
    # Get the current date and time in a format suitable for filenames
    timestamp = datetime.now().strftime("%m-%d_%H-%M-%S")

    # Define the bag directory format: "sim_name_YYYY-MM-DD_HH-MM-SS"
    bag_dir = ['rosbags/',timestamp, '_', sim_name]

    return ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--output', ''.join(bag_dir), namespace + '/gbeam/merged_graph', '/status'],
        output='screen'
    )


def generate_launch_description():
    """Generate the launch description dynamically based on the number of robots."""
    ld = LaunchDescription()

    # Declare launch arguments
    num_robots_arg  = DeclareLaunchArgument('num_robots', default_value='2', description='Number of robots to simulate')
    positions_arg   = DeclareLaunchArgument('robot_positions', default_value="[(1.65, -1.8), (0.0, 0.0)]", description='Positions of robots')
    sim_name_arg    = DeclareLaunchArgument('sim_name', default_value='my_simulation', description='Name of the simulation')

    # Retrieve launch configurations
    num_robots = LaunchConfiguration('num_robots')
    positions  = LaunchConfiguration('robot_positions')
    sim_name   = LaunchConfiguration('sim_name')
    # Open Rqt GUI (Optional)
    open_gui = ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_gui', 'rqt_gui'],
            output='screen'
        )
    
    standalone_world = ExecuteProcess(cmd=["ros2", "launch", "multi_turtlebot_sim", "standalone_world.launch.py"])
    
    
    foxglove_bridge  = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])
    setup_world = TimerAction(
                period=0.0,  # Delay to avoid simultaneous launches
                actions=[
                    standalone_world,
                    foxglove_bridge,
                ]
            )

    offset=1.0

    def launch_setup(context, *args, **kwargs):
        """Function to dynamically process launch arguments at runtime."""
        parsed_positions = ast.literal_eval(context.launch_configurations['robot_positions'])  # Parse at runtime
        parsed_sim_name = sim_name.perform(context)
        num_robots_value = int(context.launch_configurations['num_robots'])  # Ensure it's an integer

        actions = []

        

        # Log simulation name
        actions.append(LogInfo(msg=f"Simulation name: {context.launch_configurations['sim_name']}"))

        # Create TimerActions dynamically
        for idx, (x_pose, y_pose) in enumerate(parsed_positions[:num_robots_value]):  # Ensure we respect num_robots
            namespace = f'robot{idx}'
            lidar_height = 0.1 + (idx * 0.2)  # Example lidar height variation

            actions.append(TimerAction(
                period=offset + idx + 1.0,  # Delay to avoid simultaneous launches
                actions=[
                    launch_spawn_gbeam2(namespace, lidar_height, x_pose, y_pose, num_robots),
                    recordBagfor(namespace,parsed_sim_name)
                ]
            ))

        return actions

    # Add launch arguments
    ld.add_action(num_robots_arg)
    ld.add_action(positions_arg)
    ld.add_action(sim_name_arg)
    ld.add_action(open_gui)
    ld.add_action(setup_world)

    # Evaluate launch configurations using OpaqueFunction
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
