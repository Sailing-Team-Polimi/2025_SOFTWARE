import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[os.path.join(get_package_share_directory("state_estimation"), 'config', 'ekf.yaml')],
        remappings=[('/myodometry/filtered', '/odometry/filtered')],
    )
    ld.add_action(ekf_node)
    
    gps_node = Node(
        package= 'state_estimation',
        executable = 'gps_to_local',
        name = 'gps_to_local',
        remappings=[('/gps_data', '/fix')],
    )
    
    ld.add_action(gps_node)
    
    visualization_node = Node(
        package = 'state_estimation',
        executable = 'local_to_gps',
        name = 'local_to_gps',
        remappings=[('/gps_data', '/fix')],
    )
    
    ld.add_action(visualization_node)
    
    return ld  