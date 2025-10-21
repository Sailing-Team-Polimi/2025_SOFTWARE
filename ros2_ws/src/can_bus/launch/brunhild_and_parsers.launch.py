import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    ld = LaunchDescription()

    params_file = os.path.join( # Path to mpu parameters
        get_package_share_directory('can_bus'),
        'config', 
        'BrunhildAndParsers.yaml'
    )

    # ----------------   BRUNILDE NODE ------------------ #

    brunilde_node = Node(
        package='can_bus',
        executable='brunilde_simple_node',
        name='brunilde_node',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
        output='screen',
        remappings=[
            ('/fromBrunilde', '/fromBrunilde'),
            ('/toBrunilde', '/toBrunilde'),
        ]
    )

    ld.add_action(brunilde_node)

    # --------------- POT PARSER -------------------- #

    pot_parser = Node(
        package='can_bus',
        executable='general_parser_node',
        name='wand_parser',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/raw_data', '/fromBrunilde'),
            ('/parsed_data', '/wand_angle'),
        ]
    )

    ld.add_action(pot_parser)   

    # ---------------- IMU PARSER ----------------- #

    imu_parser = Node(
        package='can_bus',
        executable='imu_parser_node',
        name='imu_parser',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/serial_data', '/fromBrunilde'),
            ('/imu_data', '/imu_data'),
        ]
    )

    ld.add_action(imu_parser) 

    # ------------------------------------------ #

    # Run the nodes
    return ld
