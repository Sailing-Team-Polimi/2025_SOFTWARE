import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():
    
    ld = LaunchDescription()

    params_file = os.path.join( # Path to mpu parameters
        get_package_share_directory('gui'),
        'config', 
        'FoxGlove.yaml'
    )

    # ---------------- URDF Publisher ------------------ #

    # --------------- STATE PUBLISHER -------------------- #

    boat_state = Node(
        package='gui',
        executable='boat_state_pub',
        name='state_pub',
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/wand_angle', '/wand_angle'),
        ]
    )

    ld.add_action(boat_state)

    # ---------------- BASE LINK ----------------- #


    # --------------- FOXGLOVE BRIDGE ----------------- #
    
    foxglove_bridge = Node(
        package = 'foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_handler',
        arguments=['--ros-args', '--log-level', 'INFO'],
        output='screen',
    )

    ld.add_action(foxglove_bridge)

    # Run the nodes
    return ld