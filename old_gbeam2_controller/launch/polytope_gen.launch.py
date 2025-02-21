# # Example polytope_generation_launch.py
# import launch
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
#     return launch.LaunchDescription([
#         DeclareLaunchArgument(
#             'global_param',
#             default_value='/home/pica/code/GBEAM2_MultiUAV/src/old_gbeam2_controller/config/global_param.yaml', # Adjust the path accordingly
#             description='/home/pica/code/GBEAM2_MultiUAV/src/old_gbeam2_controller/config/global_param.yaml'
#         ),
#         Node(
#             package='old_gbeam2_controller', # Replace with your package name
#             executable='polytope_generation_node', # Replace with your node executable name
#             name='poly_gen', # Name of the node
#             output='screen', # Output to screen
#             parameters=[LaunchConfiguration('global_param')]
#         )
#     ])

#### PARTE CAMBIATA #####
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('old_gbeam2_controller'),
        'config',
        'global_param.yaml'
        )
        
    node=Node(
        package = 'old_gbeam2_controller',
        name = 'poly_gen',                  ##qua il node name è poly_gen, il nome che viene dato alla classe nel polytope_generation_node
        executable = 'polytope_generation_node',
        parameters = [config]
    )

    ld.add_action(node)
    return ld