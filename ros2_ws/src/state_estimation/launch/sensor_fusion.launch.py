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
    )
    
    ld.add_action(gps_node)
    
    visualization_node = Node(
        package = 'state_estimation',
        executable = 'local_to_gps',
        name = 'local_to_gps',
    )
    
    ld.add_action(visualization_node)
    
    
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        parameters=[os.path.join(get_package_share_directory("state_estimation"), 'config', 'ekf_local.yaml')],
        remappings=[('/odometry/filtered', '/odometry/local')],
    )
    ld.add_action(ekf_local_node)
    
    pot_to_wand_node = Node(
        package = 'state_estimation',
        executable = 'pot_to_wand_node',
        name = 'pot_to_wand_node',
    )
    ld.add_action(pot_to_wand_node)
    
    height_node = Node(
        package = 'state_estimation',
        executable = 'ride_height_node',
        name = 'height_node',
        parameters = [{
            'wand_topic':'/wand_angle_deg',
            'L_wand': 0.7,
            'L_rod': 1.3,
            'variance_z' : 0.0004
        }]
    )
    
    ld.add_action(height_node)
    


    
    return ld