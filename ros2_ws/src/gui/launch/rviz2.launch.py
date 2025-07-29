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

    # Use xacro to process the file
    xacro_file = os.path.join(
        get_package_share_directory('gui'),
        'description',
        'teti/teti.urdf.xacro'
    )

    # ---------------- URDF Publisher ------------------ #

    boad_description = xacro.process_file(xacro_file).toxml()

    boat_urdf_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='gate_manager',
        parameters=[{'robot_description': boad_description}],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/serial_data', '/serial_data'),
            ('/servo_angle', '/servo_angle'),
        ]
    )

    ld.add_action(boat_urdf_pub)

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
    # if state estimation is not available

    base_link_broadcaster = Node(
        package='gui',
        executable='base_link_bc',
        name='base_link',
        remappings=[
            ('/est_z', '/est/z'),
            ('/est_xy','/est/xy'),
            ('/est_rpy', '/est/rpy')
        ],
    )

    ld.add_action(base_link_broadcaster)


    # Run the nodes
    return ld